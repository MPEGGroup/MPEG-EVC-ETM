/* The copyright in this software is being made available under the BSD
*  License, included below. This software may be subject to other third party
*  and contributor rights, including patent rights, and no such rights are
*  granted under this license.
*  
*  Copyright (c) 2019-2020, ISO/IEC
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*   * Neither the name of the ISO/IEC nor the names of its contributors may
*     be used to endorse or promote products derived from this software without
*     specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
*  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
*  THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "evce_def.h"
#include "evce_mode.h"
#include "evc_ipred.h"
#include "evc_tbl.h"
#include "evc_df.h"
#include <math.h>
#include "evc_util.h"
#include "evce_ibc_hash_wrapper.h"

#if FAST_RECURSE_OPT
typedef int(*LOSSY_ES_FUNC)(EVCE_CU_DATA *, int, double, int, int, int, int, int, int);
#endif

static s32 entropy_bits[1024];

void evce_sbac_bit_reset(EVCE_SBAC * sbac)
{
    sbac->code           &= 0x7FFFF;
    sbac->code_bits       = 11;
    sbac->pending_byte    = 0;
    sbac->is_pending_byte = 0;
    sbac->stacked_ff      = 0;
    sbac->stacked_zero    = 0;
    sbac->bitcounter      = 0;
#if CABAC_ZERO_WORD // init Bin counter
    sbac->bin_counter = 0;
#endif 
}

u32 evce_get_bit_number(EVCE_SBAC *sbac)
{
    return sbac->bitcounter + 8 * (sbac->stacked_zero + sbac->stacked_ff) + 8 * (sbac->is_pending_byte ? 1 : 0) + 8 - sbac->code_bits + 3;
}

void evce_rdo_bit_cnt_mvp(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][MV_D], int pidx, int mvp_idx)
{
    int refi0, refi1;

    if(pidx != PRED_DIR)
    {
        refi0 = refi[REFP_0];
        refi1 = refi[REFP_1];
        if(IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
        {
            evce_eco_mvp_idx(&core->bs_temp, mvp_idx);
            evce_eco_mvd(&core->bs_temp, mvd[REFP_0]);
        }
        if(slice_type == SLICE_B && REFI_IS_VALID(refi1))
        {
            evce_eco_mvp_idx(&core->bs_temp, mvp_idx);
            evce_eco_mvd(&core->bs_temp, mvd[REFP_1]);
        }
    }
}

void evce_rdo_bit_cnt_affine_mvp(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][VER_NUM][MV_D], int pidx, int mvp_idx, int vertex_num)
{
    int refi0, refi1;
    int vertex;
    int b_zero = 1;

    if(pidx != PRED_DIR)
    {
        refi0 = refi[REFP_0];
        refi1 = refi[REFP_1];
        if(IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
        {
            evce_eco_affine_mvp_idx( &core->bs_temp, mvp_idx );
            b_zero = 1;
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                if(mvd[REFP_0][vertex][MV_X] != 0 || mvd[REFP_0][vertex][MV_Y] != 0)
                {
                    b_zero = 0;
                    break;
                }
            }
            evce_eco_affine_mvd_flag(&core->bs_temp, b_zero, REFP_0);
            if(b_zero == 0)
                for(vertex = 0; vertex < vertex_num; vertex++)
                    evce_eco_mvd(&core->bs_temp, mvd[REFP_0][vertex]);
        }
        if(slice_type == SLICE_B && REFI_IS_VALID(refi1))
        {
            evce_eco_affine_mvp_idx( &core->bs_temp, mvp_idx );
            b_zero = 1;
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                if(mvd[REFP_1][vertex][MV_X] != 0 || mvd[REFP_1][vertex][MV_Y] != 0)
                {
                    b_zero = 0;
                    break;
                }
            }
            evce_eco_affine_mvd_flag(&core->bs_temp, b_zero, REFP_1);
            if(b_zero == 0)
                for(vertex = 0; vertex < vertex_num; vertex++)
                    evce_eco_mvd(&core->bs_temp, mvd[REFP_1][vertex]);
        }
    }
}

void evce_rdo_bit_cnt_cu_intra_luma(EVCE_CTX *ctx, EVCE_CORE *core, s32 slice_type, s32 cup, s16 coef[N_C][MAX_CU_DIM])
{
    EVCE_SBAC *sbac = &core->s_temp_run;
    int log2_cuw = core->log2_cuw;
    int log2_cuh = core->log2_cuh;
    int* nnz = core->nnz;

    if(slice_type != SLICE_I && (ctx->sps.tool_admvp == 0 || !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2))
#if M50761_CHROMA_NOT_SPLIT
        && evce_check_all_preds(ctx, core)
#endif
        )
    {
        evce_sbac_encode_bin(0, sbac, core->s_temp_run.ctx.skip_flag + core->ctx_flags[CNID_SKIP_FLAG], &core->bs_temp); /* skip_flag */
        evce_eco_pred_mode(&core->bs_temp, MODE_INTRA, core->ctx_flags[CNID_PRED_MODE]);
    }

    if (((slice_type == SLICE_I)
#if M50761_CHROMA_NOT_SPLIT
        || evce_check_only_intra(ctx, core)
#endif
        ) 
#if M50761_CHROMA_NOT_SPLIT
        && evce_check_luma(ctx, core)
#endif
        && ctx->param.use_ibc_flag && core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
    {
        evce_eco_ibc_flag(&core->bs_temp, 0,core->ctx_flags[CNID_IBC_FLAG]);
    }

    if (ctx->sps.tool_eipd)
    {
        evce_eco_intra_dir(&core->bs_temp, core->ipm[0], core->mpm, core->mpm_ext, core->pims);
    }
    else 
    {
        evce_eco_intra_dir_b(&core->bs_temp, core->ipm[0], core->mpm_b_list, core->mpm_ext, core->pims);
    }
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->cu_qp_delta_code = core->dqp_temp_run.cu_qp_delta_code;
        core->cu_qp_delta_is_coded = core->dqp_temp_run.cu_qp_delta_is_coded;
        ctx->tile[core->tile_idx].qp_prev_eco = core->dqp_temp_run.prev_QP;
    }
#endif
    evce_eco_coef(&core->bs_temp, coef, log2_cuw, log2_cuh, MODE_INTRA, core->nnz_sub, 0, RUN_L, ctx->sps.tool_ats, core->ats_intra_cu, core->ats_mode, 0, ctx
#if DQP
                  , core, -1, core->qp
#endif
#if M50761_CHROMA_NOT_SPLIT
                  , core->tree_cons
#endif
    );
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->dqp_temp_run.cu_qp_delta_code = core->cu_qp_delta_code;
        core->dqp_temp_run.cu_qp_delta_is_coded = core->cu_qp_delta_is_coded;
        core->dqp_temp_run.prev_QP = ctx->tile[core->tile_idx].qp_prev_eco;
        core->dqp_temp_run.curr_QP = core->qp;
    }
#endif
}

void evce_rdo_bit_cnt_cu_intra_chroma(EVCE_CTX *ctx, EVCE_CORE *core, s32 slice_type, s32 cup, s16 coef[N_C][MAX_CU_DIM])
{
    EVCE_SBAC *sbac = &core->s_temp_run;
    int log2_cuw = core->log2_cuw;
    int log2_cuh = core->log2_cuh;
    int *nnz = core->nnz;

    if (ctx->sps.tool_eipd)
    {
        evce_eco_intra_dir_c(&core->bs_temp, core->ipm[1], core->ipm[0]);
    }

    evce_eco_coef(&core->bs_temp, coef, log2_cuw, log2_cuh, MODE_INTRA, core->nnz_sub, 0, RUN_CB | RUN_CR, ctx->sps.tool_ats, 0, 0, 0, ctx
#if DQP
                  , core, -1, 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                  , core->tree_cons
#endif
    );
}

void evce_rdo_bit_cnt_cu_intra(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s16 coef[N_C][MAX_CU_DIM])
{
    EVCE_SBAC *sbac = &core->s_temp_run;
    int log2_cuw = core->log2_cuw;
    int log2_cuh = core->log2_cuh;
    int* nnz = core->nnz;
#if ATS_INTER_DEBUG
    assert(core->ats_inter_info == 0);
#endif

    if(slice_type != SLICE_I && (ctx->sps.tool_admvp == 0 || !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2))
#if M50761_CHROMA_NOT_SPLIT
        && evce_check_all_preds(ctx, core)
#endif
        )
    {
        evce_sbac_encode_bin(0, sbac, core->s_temp_run.ctx.skip_flag + core->ctx_flags[CNID_SKIP_FLAG], &core->bs_temp); /* skip_flag */
        evce_eco_pred_mode(&core->bs_temp, MODE_INTRA, core->ctx_flags[CNID_PRED_MODE]);
    }

    if ( ((slice_type == SLICE_I)
#if M50761_CHROMA_NOT_SPLIT
        || evce_check_only_intra(ctx, core) 
#endif
        ) 
#if M50761_CHROMA_NOT_SPLIT
        && evce_check_luma(ctx, core)
#endif
        && ctx->param.use_ibc_flag && core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
    {
        evce_eco_ibc_flag(&core->bs_temp, 0, core->ctx_flags[CNID_IBC_FLAG]);
    }

    if (ctx->sps.tool_eipd)
    {
#if M50761_CHROMA_NOT_SPLIT
        if (evce_check_luma(ctx, core))
        {
#endif
        evce_eco_intra_dir(&core->bs_temp, core->ipm[0], core->mpm, core->mpm_ext, core->pims);
#if M50761_CHROMA_NOT_SPLIT
        }
        else
        {
            evc_assert(nnz[Y_C] == 0);
        }
        if (evce_check_chroma(ctx, core))
        {
#endif
        evce_eco_intra_dir_c(&core->bs_temp, core->ipm[1], core->ipm[0]);
#if M50761_CHROMA_NOT_SPLIT
        }
        else
        {
            evc_assert(nnz[U_C] == 0);
            evc_assert(nnz[V_C] == 0);
        }
#endif
    }
    else
    {
#if FIX_EIPD_OFF & M50761_CHROMA_NOT_SPLIT
        if (evce_check_luma(ctx, core))
        {
#endif
            evce_eco_intra_dir_b(&core->bs_temp, core->ipm[0], core->mpm_b_list, core->mpm_ext, core->pims);
#if FIX_EIPD_OFF & M50761_CHROMA_NOT_SPLIT
        }
#endif
    }
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->cu_qp_delta_code = core->dqp_temp_run.cu_qp_delta_code;
        core->cu_qp_delta_is_coded = core->dqp_temp_run.cu_qp_delta_is_coded;
        ctx->tile[core->tile_idx].qp_prev_eco = core->dqp_temp_run.prev_QP;
    }
#endif
    evce_eco_coef(&core->bs_temp, coef, log2_cuw, log2_cuh, MODE_INTRA, core->nnz_sub, 0, RUN_L | RUN_CB | RUN_CR, ctx->sps.tool_ats, core->ats_intra_cu, core->ats_mode, 0, ctx
#if DQP
                  , core, ctx->pps.cu_qp_delta_enabled_flag ? 1 : 0, core->qp
#endif
#if M50761_CHROMA_NOT_SPLIT
        , core->tree_cons
#endif
    );
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->dqp_temp_run.cu_qp_delta_code = core->cu_qp_delta_code;
        core->dqp_temp_run.cu_qp_delta_is_coded = core->cu_qp_delta_is_coded;
        core->dqp_temp_run.prev_QP = ctx->tile[core->tile_idx].qp_prev_eco;
        core->dqp_temp_run.curr_QP = core->qp;
    }
#endif
}

void evce_rdo_bit_cnt_cu_inter_comp(EVCE_CORE * core, s16 coef[N_C][MAX_CU_DIM], int ch_type, int pidx, EVCE_CTX * ctx
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int* nnz = core->nnz;
    EVCE_SBAC* sbac = &core->s_temp_run;
    int log2_cuw = core->log2_cuw;
    int log2_cuh = core->log2_cuh;
    int b_no_cbf = 0;

    if(ch_type == Y_C)
    {
        evce_eco_coef(&core->bs_temp, coef, log2_cuw, log2_cuh, MODE_INTER, core->nnz_sub, b_no_cbf, RUN_L, ctx->sps.tool_ats, 0, 0, core->ats_inter_info, ctx
#if DQP
            , core, 0, core->qp
#endif

#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }
       
    if(ch_type == U_C)
    {
        evce_eco_coef(&core->bs_temp, coef, log2_cuw, log2_cuh, MODE_INTER, core->nnz_sub, b_no_cbf, RUN_CB, ctx->sps.tool_ats, 0, 0, core->ats_inter_info, ctx
#if DQP
            , core, -1, 0
#endif
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }

    if(ch_type == V_C)
    {
        evce_eco_coef(&core->bs_temp, coef, log2_cuw, log2_cuh, MODE_INTER, core->nnz_sub, b_no_cbf, RUN_CR, ctx->sps.tool_ats, 0, 0, core->ats_inter_info, ctx
#if DQP
            , core, -1, 0
#endif
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }
}

void evce_rdo_bit_cnt_cu_ibc(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s16 mvd[MV_D],
  s16 coef[N_C][MAX_CU_DIM], u8 mvp_idx, u8 ibc_flag)
{
    int b_no_cbf = 0;

    if (slice_type != SLICE_I
#if M50761_CHROMA_NOT_SPLIT
        && evce_check_all_preds(ctx, core)
#endif
        )
    {
        evce_sbac_encode_bin(0, &core->s_temp_run, core->s_temp_run.ctx.skip_flag + core->ctx_flags[CNID_SKIP_FLAG], &core->bs_temp); /* skip_flag */
        evce_eco_pred_mode(&core->bs_temp, MODE_INTER, core->ctx_flags[CNID_PRED_MODE]);
    }

    if ((!(core->skip_flag == 1 && slice_type == SLICE_I)
#if M50761_CHROMA_NOT_SPLIT
        || evce_check_only_intra(ctx, core)
#endif
        )
#if M50761_CHROMA_NOT_SPLIT
        && evce_check_luma(ctx, core)
#endif
        )
    {
        evce_eco_ibc(&core->bs_temp, ibc_flag, core->ctx_flags[CNID_IBC_FLAG]);
    }

    evce_eco_mvd(&core->bs_temp, mvd);
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->cu_qp_delta_code = core->dqp_temp_run.cu_qp_delta_code;
        core->cu_qp_delta_is_coded = core->dqp_temp_run.cu_qp_delta_is_coded;
        ctx->tile[core->tile_idx].qp_prev_eco = core->dqp_temp_run.prev_QP;
    }
#endif
    evce_eco_coef(&core->bs_temp, coef, core->log2_cuw, core->log2_cuh, MODE_IBC, core->nnz_sub, b_no_cbf, RUN_L | RUN_CB | RUN_CR, ctx->sps.tool_ats, 0, 0, 0, ctx
#if DQP
        , core, ctx->pps.cu_qp_delta_enabled_flag ? 1 : 0, core->qp
#endif
#if M50761_CHROMA_NOT_SPLIT
        , core->tree_cons
#endif
    );
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->dqp_temp_run.cu_qp_delta_code = core->cu_qp_delta_code;
        core->dqp_temp_run.cu_qp_delta_is_coded = core->cu_qp_delta_is_coded;
        core->dqp_temp_run.prev_QP = ctx->tile[core->tile_idx].qp_prev_eco;
                core->dqp_temp_run.curr_QP = core->qp;
    }
#endif
}

void evce_rdo_bit_cnt_cu_inter(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][MV_D], s16 coef[N_C][MAX_CU_DIM], int pidx, u8 * mvp_idx, u8 mvr_idx, u8 bi_idx, s16 affine_mvd[REFP_NUM][VER_NUM][MV_D])
{
    int refi0, refi1;
    int vertex = 0;
    int vertex_num = core->affine_flag + 1;

    EVCE_PINTER *pi = &ctx->pinter;

    int b_no_cbf = 0;
    b_no_cbf |= pidx == AFF_DIR;
    b_no_cbf |= pidx == PRED_DIR_MMVD;
    b_no_cbf |= pidx == PRED_DIR;

    if(ctx->sps.tool_admvp == 0)
    {
        b_no_cbf = 0;
    }

    if(slice_type != SLICE_I)
    {
        if (ctx->sps.tool_admvp && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2)
        {
            evc_assert(0);
        }

        evce_sbac_encode_bin(0, &core->s_temp_run, core->s_temp_run.ctx.skip_flag + core->ctx_flags[CNID_SKIP_FLAG], &core->bs_temp); /* skip_flag */

#if M50761_CHROMA_NOT_SPLIT
        if (evce_check_all_preds(ctx, core))
#endif
        {
            evce_eco_pred_mode(&core->bs_temp, MODE_INTER, core->ctx_flags[CNID_PRED_MODE]);
        }
        if (
#if M50761_CHROMA_NOT_SPLIT
            !evce_check_only_inter(ctx, core) && evce_check_luma(ctx, core) &&
#endif
             ctx->param.use_ibc_flag && core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
        {
            evce_eco_ibc_flag(&core->bs_temp, 0, core->ctx_flags[CNID_IBC_FLAG]);
        }

        if(ctx->sps.tool_amvr)
        {
            evce_eco_mvr_idx(&core->bs_temp, mvr_idx);
        }

        
        int dir_flag = (pidx == PRED_DIR);
        dir_flag |= (pidx == PRED_DIR_MMVD);
        dir_flag |= (pidx == AFF_DIR);

        if(ctx->sps.tool_admvp == 0)
        {
            evce_eco_direct_mode_flag(&core->bs_temp, dir_flag);
        }
        else
        {
            if(mvr_idx == 0)
            {
                evce_eco_merge_mode_flag(&core->bs_temp, dir_flag);
            }
        }

        if(ctx->sps.tool_mmvd)
        {
            if(dir_flag)
            {
                evce_eco_mmvd_flag(&core->bs_temp, pidx == PRED_DIR_MMVD);
            }

            if((pidx == PRED_DIR_MMVD))
            {
                evce_eco_mmvd_info(&core->bs_temp, pi->mmvd_idx[pidx], ctx->sh.mmvd_group_enable_flag && !((1 << core->log2_cuw)*(1 << core->log2_cuh) <= NUM_SAMPLES_BLOCK));
            }
        }

        // affine direct in rdo
        if(core->cuw >= 8 && core->cuh >= 8 && ctx->sps.tool_affine && ((pidx == PRED_DIR) || (pidx == AFF_DIR)))
        {
            evce_sbac_encode_bin(core->affine_flag != 0, &core->s_temp_run, core->s_temp_run.ctx.affine_flag + core->ctx_flags[CNID_AFFN_FLAG], &core->bs_temp); /* direct affine_flag */

            if(core->affine_flag)
                evce_eco_affine_mrg_idx(&core->bs_temp, mvp_idx[REFP_0]);
        }

        if (ctx->sps.tool_admvp == 1 && pidx == PRED_DIR && !core->affine_flag && mvr_idx == 0)
        {
            evce_eco_merge_idx(&core->bs_temp, mvp_idx[0]);
        }
        

        if((((pidx % ORG_PRED_NUM) != PRED_DIR) && ((pidx % ORG_PRED_NUM) != PRED_DIR_MMVD)) || ((pidx >= AFF_L0) && (pidx <= AFF_6_BI) && (pidx != AFF_DIR)) )
        {
            evce_eco_inter_pred_idc(&core->bs_temp, refi, slice_type, 1 << core->log2_cuw, 1 << core->log2_cuh, ctx->sps.tool_admvp);

            // affine inter in rdo
            if (core->cuw >= 16 && core->cuh >= 16 && ctx->sps.tool_affine && mvr_idx == 0)
            {
                evce_sbac_encode_bin(core->affine_flag != 0, &core->s_temp_run, core->s_temp_run.ctx.affine_flag + core->ctx_flags[CNID_AFFN_FLAG], &core->bs_temp); /* inter affine_flag */
            }

            if(core->affine_flag)
            {
                evce_sbac_encode_bin(core->affine_flag - 1, &core->s_temp_run, core->s_temp_run.ctx.affine_mode, &core->bs_temp); /* inter affine_mode */
            }

            if(!core->affine_flag)
            {
                if(ctx->sps.tool_admvp == 1 && REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
                {
                    evce_eco_bi_idx(&core->bs_temp, bi_idx - 1);
                }
            }
            refi0 = refi[REFP_0];
            refi1 = refi[REFP_1];
            if(IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
            {
                if(ctx->sps.tool_admvp == 0)
                {
                    evce_eco_refi(&core->bs_temp, ctx->rpm.num_refp[REFP_0], refi0);
                    evce_eco_mvp_idx(&core->bs_temp, mvp_idx[REFP_0]);
                    evce_eco_mvd(&core->bs_temp, mvd[REFP_0]);
                }
                else
                {
                    if(bi_idx != BI_FL0 && bi_idx != BI_FL1)
                    {
                        evce_eco_refi(&core->bs_temp, ctx->rpm.num_refp[REFP_0], refi0);
                    }

                    if(core->affine_flag)
                    {
                        int b_zero = 1;

                        evce_eco_affine_mvp_idx(&core->bs_temp, mvp_idx[REFP_0]);

                        for(vertex = 0; vertex < vertex_num; vertex++)
                        {
                            int mvd_x = affine_mvd[REFP_0][vertex][MV_X];
                            int mvd_y = affine_mvd[REFP_0][vertex][MV_Y];
                            if(mvd_x != 0 || mvd_y != 0)
                            {
                                b_zero = 0;
                                break;
                            }
                        }
                        evce_eco_affine_mvd_flag(&core->bs_temp, b_zero, REFP_0);

                        if(b_zero == 0)
                        {
                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                evce_eco_mvd(&core->bs_temp, affine_mvd[REFP_0][vertex]);
                            }
                        }
                    }
                    else
                    {
                        if(bi_idx != BI_FL0)
                        {
                            evce_eco_mvd(&core->bs_temp, mvd[REFP_0]);
                        }
                    }
                }
            }

            if(slice_type == SLICE_B && REFI_IS_VALID(refi1))
            {
                if(ctx->sps.tool_admvp == 0)
                {
                    evce_eco_refi(&core->bs_temp, ctx->rpm.num_refp[REFP_1], refi1);
                    evce_eco_mvp_idx(&core->bs_temp, mvp_idx[REFP_1]);
                    evce_eco_mvd(&core->bs_temp, mvd[REFP_1]);
                }
                else
                {
                    if(bi_idx != BI_FL0 && bi_idx != BI_FL1)
                    {
                        evce_eco_refi(&core->bs_temp, ctx->rpm.num_refp[REFP_1], refi1);
                    }

                    if(core->affine_flag)
                    {
                        int b_zero = 1;

                        evce_eco_affine_mvp_idx(&core->bs_temp, mvp_idx[REFP_1]);

                        for(vertex = 0; vertex < vertex_num; vertex++)
                        {
                            int mvd_x = affine_mvd[REFP_1][vertex][MV_X];
                            int mvd_y = affine_mvd[REFP_1][vertex][MV_Y];
                            if(mvd_x != 0 || mvd_y != 0)
                            {
                                b_zero = 0;
                                break;
                            }
                        }
                        evce_eco_affine_mvd_flag(&core->bs_temp, b_zero, REFP_1);

                        if(b_zero == 0)
                        {
                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                evce_eco_mvd(&core->bs_temp, affine_mvd[REFP_1][vertex]);
                            }
                        }
                    }
                    else
                    {
                        if(bi_idx != BI_FL1)
                        {
                            evce_eco_mvd(&core->bs_temp, mvd[REFP_1]);
                        }
                    }
                }
            }
        }
    }
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->cu_qp_delta_code = core->dqp_temp_run.cu_qp_delta_code;
        core->cu_qp_delta_is_coded = core->dqp_temp_run.cu_qp_delta_is_coded;
        ctx->tile[core->tile_idx].qp_prev_eco = core->dqp_temp_run.prev_QP;
    }
#endif
    evce_eco_coef(&core->bs_temp, coef, core->log2_cuw, core->log2_cuh, MODE_INTER, core->nnz_sub, b_no_cbf, RUN_L | RUN_CB | RUN_CR, ctx->sps.tool_ats, 0, 0, core->ats_inter_info, ctx
#if DQP
        , core, ctx->pps.cu_qp_delta_enabled_flag ? 1 : 0, core->qp
#endif
#if M50761_CHROMA_NOT_SPLIT
        , core->tree_cons
#endif
    );
#if DQP_RDO
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        core->dqp_temp_run.cu_qp_delta_code = core->cu_qp_delta_code;
        core->dqp_temp_run.cu_qp_delta_is_coded = core->cu_qp_delta_is_coded;
        core->dqp_temp_run.prev_QP = ctx->tile[core->tile_idx].qp_prev_eco;
        core->dqp_temp_run.curr_QP = core->qp;
    }
#endif
}

void evce_rdo_bit_cnt_cu_skip(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, int mvp_idx0, int mvp_idx1, int c_num , int tool_mmvd)
{
    if(slice_type != SLICE_I)
    {
        evce_sbac_encode_bin(1, &core->s_temp_run, core->s_temp_run.ctx.skip_flag + core->ctx_flags[CNID_SKIP_FLAG], &core->bs_temp); /* skip_flag */

        if(tool_mmvd)
        {
            evce_eco_mmvd_flag(&core->bs_temp, core->mmvd_flag);
        }

        if(core->mmvd_flag)
        {
            evce_eco_mmvd_info(&core->bs_temp, c_num, ctx->sh.mmvd_group_enable_flag && !((1 << core->log2_cuw)*(1 << core->log2_cuh) <= NUM_SAMPLES_BLOCK));
        }
        else
        {
            // affine skip mode in rdo
            if(core->cuw >= 8 && core->cuh >= 8 && ctx->sps.tool_affine)
            {
                evce_sbac_encode_bin(core->affine_flag != 0, &core->s_temp_run, core->s_temp_run.ctx.affine_flag + core->ctx_flags[CNID_AFFN_FLAG], &core->bs_temp); /* skip affine_flag */
            }
            if(core->affine_flag)
            {
                evce_eco_affine_mrg_idx(&core->bs_temp, mvp_idx0);
                return;
            }

            if(!ctx->sps.tool_admvp)
            {
                evce_eco_mvp_idx(&core->bs_temp, mvp_idx0);

                if(slice_type == SLICE_B)
                {
                    evce_eco_mvp_idx(&core->bs_temp, mvp_idx1);
                }
            }
            else
            {
                evce_eco_merge_idx(&core->bs_temp, mvp_idx0);
            }
        }
    }
}

void evce_init_bits_est()
{
    int i = 0;
    double p;

    for(i = 0; i < 1024; i++)
    {
        p = (512 * (i + 0.5)) / 1024;
        entropy_bits[i] = (s32)(-32768 * (log(p) / log(2.0) - 9));
    }
}

static s32 biari_no_bits(int symbol, SBAC_CTX_MODEL* cm)
{
    u16 mps, state;

    mps = (*cm) & 1;
    state = (*cm) >> 1;
    state = ((u16)(symbol != 0) != mps) ? state : (512 - state);

    return entropy_bits[state << 1];
}

static void evce_rdoq_bit_est(EVCE_SBAC * sbac, EVCE_CORE * core)
{
    int bin, ctx;

    for(bin = 0; bin < 2; bin++)
    {
        core->rdoq_est_cbf_luma[bin] = biari_no_bits(bin, sbac->ctx.cbf_luma);
        core->rdoq_est_cbf_cb[bin] = biari_no_bits(bin, sbac->ctx.cbf_cb);
        core->rdoq_est_cbf_cr[bin] = biari_no_bits(bin, sbac->ctx.cbf_cr);
        core->rdoq_est_cbf_all[bin] = biari_no_bits(bin, sbac->ctx.cbf_all);
    }

    for (ctx = 0; ctx < NUM_CTX_SIG_COEFF_FLAG; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            core->rdoq_est_sig_coeff[ctx][bin] = biari_no_bits(bin, sbac->ctx.sig_coeff_flag + ctx);
        }
    }

    for (ctx = 0; ctx < NUM_CTX_GTX; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            core->rdoq_est_gtx[ctx][bin] = biari_no_bits(bin, sbac->ctx.coeff_abs_level_greaterAB_flag + ctx);
        }
    }

    for (ctx = 0; ctx < NUM_CTX_LAST_SIG_COEFF; ctx++)
    {
        for (bin = 0; bin < 2; bin++)
        {
            core->rdoq_est_last_sig_coeff_x[ctx][bin] = biari_no_bits(bin, sbac->ctx.last_sig_coeff_x_prefix + ctx);
            core->rdoq_est_last_sig_coeff_y[ctx][bin] = biari_no_bits(bin, sbac->ctx.last_sig_coeff_y_prefix + ctx);
        }
    }

    for(ctx = 0; ctx < NUM_CTX_CC_RUN; ctx++)
    {
        for(bin = 0; bin < 2; bin++)
        {
            core->rdoq_est_run[ctx][bin] = biari_no_bits(bin, sbac->ctx.run + ctx);
        }
    }

    for(ctx = 0; ctx < NUM_CTX_CC_LEVEL; ctx++)
    {
        for(bin = 0; bin < 2; bin++)
        {
            core->rdoq_est_level[ctx][bin] = biari_no_bits(bin, sbac->ctx.level + ctx);
        }
    }

    for(ctx = 0; ctx < NUM_CTX_CC_LAST; ctx++)
    {
        for(bin = 0; bin < 2; bin++)
        {
            core->rdoq_est_last[ctx][bin] = biari_no_bits(bin, sbac->ctx.last + ctx);
        }
    }
}

static int init_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh, int qp_y, int qp_u, int qp_v)
{
    int i, j;
    int cuw_scu, cuh_scu;

    cuw_scu = 1 << (log2_cuw - MIN_CU_LOG2);
    cuh_scu = 1 << (log2_cuh - MIN_CU_LOG2);

    for(i = 0; i < NUM_CU_DEPTH; i++)
    {
        for(j = 0; j < NUM_BLOCK_SHAPE; j++)
        {
            evc_mset(cu_data->split_mode[i][j], 0, cuw_scu * cuh_scu * sizeof(s8));
            evc_mset(cu_data->suco_flag[i][j], 0, cuw_scu * cuh_scu * sizeof(s8));
        }
    }

    evc_mset(cu_data->qp_y, qp_y, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->qp_u, qp_u, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->qp_v, qp_v, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->mpm[0], 0, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->mpm[1], 0, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->ipm[0], 0, cuw_scu * cuh_scu * sizeof(s8));
    evc_mset(cu_data->ipm[1], 0, cuw_scu * cuh_scu * sizeof(s8));
    for(i = 0; i < 8; i++)
    {
        evc_mset(cu_data->mpm_ext[i], 0, cuw_scu * cuh_scu * sizeof(u8));
    }
#if DMVR_FLAG
        evc_mset(cu_data->dmvr_flag, 0, cuw_scu * cuh_scu * sizeof(s8));
#endif
    evc_mset(cu_data->ats_intra_cu, 0, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->ats_mode_h, 0, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->ats_mode_v, 0, cuw_scu * cuh_scu * sizeof(u8));
    evc_mset(cu_data->ats_inter_info, 0, cuw_scu * cuh_scu * sizeof(s8));

#if TRACE_ENC_CU_DATA
    evc_mset(cu_data->trace_idx, 0, cuw_scu * cuh_scu * sizeof(cu_data->trace_idx[0]));
#endif
#if TRACE_ENC_HISTORIC
    for (i = 0; i < cuw_scu * cuh_scu; ++i)
    {
        cu_data->history_buf->currCnt = 0;
        cu_data->history_buf->m_maxCnt = ALLOWED_CHECKED_NUM;
#if TRACE_ENC_CU_DATA
        evc_mset(cu_data->history_buf->history_cu_table, 0x00, ALLOWED_CHECKED_NUM * sizeof(cu_data->history_buf->history_cu_table[0]));
#endif
        evc_mset(&cu_data->history_buf->history_mv_table[0], 0x00, ALLOWED_CHECKED_NUM * sizeof(cu_data->history_buf->history_mv_table[0]) * REFP_NUM * MV_D);
        evc_mset(&cu_data->history_buf->history_refi_table[0], 0x00, ALLOWED_CHECKED_NUM * sizeof(cu_data->history_buf->history_refi_table[0]) * REFP_NUM);
    }
#endif

    return EVC_OK;
}

static int copy_cu_data(EVCE_CU_DATA *dst, EVCE_CU_DATA *src, int x, int y, int log2_cuw, int log2_cuh, int log2_cus, int cud
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int i, j, k;
    int cuw, cuh, cus;
    int cuw_scu, cuh_scu, cus_scu;
    int cx, cy;
    int size, idx_dst, idx_src;

    cx = x >> MIN_CU_LOG2;    //x = position in LCU, cx = 4x4 CU horizontal index
    cy = y >> MIN_CU_LOG2;    //y = position in LCU, cy = 4x4 CU vertical index

    cuw = 1 << log2_cuw;    //current CU width
    cuh = 1 << log2_cuh;    //current CU height
    cus = 1 << log2_cus;    //current CU buffer stride (= current CU width)
    cuw_scu = 1 << (log2_cuw - MIN_CU_LOG2);    //4x4 CU number in width
    cuh_scu = 1 << (log2_cuh - MIN_CU_LOG2);    //4x4 CU number in height
    cus_scu = 1 << (log2_cus - MIN_CU_LOG2);    //4x4 CU number in stride

    // only copy src's first row of 4x4 CUs to dis's all 4x4 CUs

#if M50761_CHROMA_NOT_SPLIT
    if (evc_check_luma(tree_cons))
    {
#endif
    for(j = 0; j < cuh_scu; j++)
    {
        idx_dst = (cy + j) * cus_scu + cx;
        idx_src = j * cuw_scu;

        size = cuw_scu * sizeof(s8);
        for(k = cud; k < NUM_CU_DEPTH; k++)
        {
            for(i = 0; i < NUM_BLOCK_SHAPE; i++)
            {
                evc_mcpy(dst->split_mode[k][i] + idx_dst, src->split_mode[k][i] + idx_src, size);
                evc_mcpy(dst->suco_flag[k][i] + idx_dst, src->suco_flag[k][i] + idx_src, size);
            }
        }

        evc_mcpy(dst->ats_intra_cu + idx_dst, src->ats_intra_cu + idx_src, size);
        evc_mcpy(dst->ats_mode_h + idx_dst, src->ats_mode_h + idx_src, size);
        evc_mcpy(dst->ats_mode_v + idx_dst, src->ats_mode_v + idx_src, size);
        evc_mcpy(dst->ats_inter_info + idx_dst, src->ats_inter_info + idx_src, size);
        evc_mcpy(dst->qp_y + idx_dst, src->qp_y + idx_src, size);
#if !M50761_CHROMA_NOT_SPLIT
        evc_mcpy(dst->qp_u + idx_dst, src->qp_u + idx_src, size);
        evc_mcpy(dst->qp_v + idx_dst, src->qp_v + idx_src, size);
#endif
        evc_mcpy(dst->pred_mode + idx_dst, src->pred_mode + idx_src, size);
        evc_mcpy(dst->mpm[0] + idx_dst, src->mpm[0] + idx_src, size);
        evc_mcpy(dst->mpm[1] + idx_dst, src->mpm[1] + idx_src, size);
        evc_mcpy(dst->ipm[0] + idx_dst, src->ipm[0] + idx_src, size);
#if !M50761_CHROMA_NOT_SPLIT
        evc_mcpy(dst->ipm[1] + idx_dst, src->ipm[1] + idx_src, size);
#endif
        for(i = 0; i < 8; i++)
        {
            evc_mcpy(dst->mpm_ext[i] + idx_dst, src->mpm_ext[i] + idx_src, size);
        }
        evc_mcpy(dst->skip_flag + idx_dst, src->skip_flag + idx_src, size);

        evc_mcpy(dst->ibc_flag + idx_dst, src->ibc_flag + idx_src, size);

#if DMVR_FLAG
        evc_mcpy(dst->dmvr_flag + idx_dst, src->dmvr_flag + idx_src, size);
#endif
        evc_mcpy(dst->mmvd_flag + idx_dst, src->mmvd_flag + idx_src, size);
        evc_mcpy(dst->affine_flag + idx_dst, src->affine_flag + idx_src, size);
        evc_mcpy(dst->depth + idx_dst, src->depth + idx_src, size);
        size = cuw_scu * sizeof(u32);
        evc_mcpy(dst->map_scu + idx_dst, src->map_scu + idx_src, size);
        evc_mcpy(dst->map_affine + idx_dst, src->map_affine + idx_src, size);
        evc_mcpy(dst->map_cu_mode + idx_dst, src->map_cu_mode + idx_src, size);
        size = cuw_scu * sizeof(u8) * REFP_NUM;
        evc_mcpy(*(dst->refi + idx_dst), *(src->refi + idx_src), size);
        evc_mcpy(*(dst->mvp_idx + idx_dst), *(src->mvp_idx + idx_src), size);

        size = cuw_scu * sizeof(u8);
        evc_mcpy(dst->mvr_idx + idx_dst, src->mvr_idx + idx_src, size);

        size = cuw_scu * sizeof(u8);
        evc_mcpy(dst->bi_idx + idx_dst, src->bi_idx + idx_src, size);

        size = cuw_scu * sizeof(s16);
        evc_mcpy(dst->mmvd_idx + idx_dst, src->mmvd_idx + idx_src, size);

        size = cuw_scu * sizeof(s16) * REFP_NUM * MV_D;
        evc_mcpy(dst->mv + idx_dst, src->mv + idx_src, size);
#if DMVR_LAG
        evc_mcpy(dst->unrefined_mv + idx_dst, src->unrefined_mv + idx_src, size);
#endif
        evc_mcpy(dst->mvd + idx_dst, src->mvd + idx_src, size);

        size = cuw_scu * sizeof(int);
#if M50761_CHROMA_NOT_SPLIT
        k = Y_C;
#else
        for(k = 0; k < N_C; k++)
#endif
        {
            evc_mcpy(dst->nnz[k] + idx_dst, src->nnz[k] + idx_src, size);
#if !M50761_CHROMA_NOT_SPLIT
        }
        for (k = 0; k < N_C; k++)
        {
#endif
            for (i = 0; i < MAX_SUB_TB_NUM; i++)
            {
                evc_mcpy(dst->nnz_sub[k][i] + idx_dst, src->nnz_sub[k][i] + idx_src, size);
            }
        }

#if TRACE_ENC_CU_DATA
        size = cuw_scu * sizeof(dst->trace_idx[0]);
        evc_mcpy(dst->trace_idx + idx_dst, src->trace_idx + idx_src, size);
        //for (i = 0; i < cuw_scu; ++i)
        //{
        //    evc_assert(dst->trace_idx[idx_dst + i] != 0);
        //}
#endif
#if TRACE_ENC_HISTORIC
        size = cuw_scu * sizeof(dst->history_buf[0]);
        evc_mcpy(dst->history_buf + idx_dst, src->history_buf + idx_src, size);
#endif

    }

    for(j = 0; j < cuh; j++)
    {
        idx_dst = (y + j) * cus + x;
        idx_src = j * cuw;

        size = cuw * sizeof(s16);
        evc_mcpy(dst->coef[Y_C] + idx_dst, src->coef[Y_C] + idx_src, size);
        size = cuw * sizeof(pel);
        evc_mcpy(dst->reco[Y_C] + idx_dst, src->reco[Y_C] + idx_src, size);
    }

#if M50761_CHROMA_NOT_SPLIT
    }
    if (evc_check_chroma(tree_cons))
    {
#endif
    for(j = 0; j < cuh >> 1; j++)
    {
        idx_dst = ((y >> 1) + j) * (cus >> 1) + (x >> 1);
        idx_src = j * (cuw >> 1);

        size = (cuw >> 1) * sizeof(s16);
        evc_mcpy(dst->coef[U_C] + idx_dst, src->coef[U_C] + idx_src, size);
        evc_mcpy(dst->coef[V_C] + idx_dst, src->coef[V_C] + idx_src, size);
        size = (cuw >> 1) * sizeof(pel);
        evc_mcpy(dst->reco[U_C] + idx_dst, src->reco[U_C] + idx_src, size);
        evc_mcpy(dst->reco[V_C] + idx_dst, src->reco[V_C] + idx_src, size);
    }
#if M50761_CHROMA_NOT_SPLIT
    for (j = 0; j < cuh_scu; j++)
    {
        idx_dst = (cy + j) * cus_scu + cx;
        idx_src = j * cuw_scu;

        size = cuw_scu * sizeof(s8);
        evc_mcpy(dst->qp_u + idx_dst, src->qp_u + idx_src, size);
        evc_mcpy(dst->qp_v + idx_dst, src->qp_v + idx_src, size);
        evc_mcpy(dst->ipm[1] + idx_dst, src->ipm[1] + idx_src, size);
        evc_mcpy(dst->pred_mode_chroma + idx_dst, src->pred_mode_chroma + idx_src, size);

        size = cuw_scu * sizeof(int);
        for (k = U_C; k < N_C; k++)
        {
            evc_mcpy(dst->nnz[k] + idx_dst, src->nnz[k] + idx_src, size);

            for (i = 0; i < MAX_SUB_TB_NUM; i++)
            {
                evc_mcpy(dst->nnz_sub[k][i] + idx_dst, src->nnz_sub[k][i] + idx_src, size);
            }
        }
    }
    }
#endif

    return EVC_OK;
}

int evce_hmvp_init(EVC_HISTORY_BUFFER *history_buffer)
{
    evc_mset(history_buffer->history_mv_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * MV_D * sizeof(s16));
#if TRACE_ENC_CU_DATA
    evc_mset(history_buffer->history_cu_table, 0, sizeof(history_buffer->history_cu_table[0])* ALLOWED_CHECKED_NUM);
#endif

    for (int i = 0; i < ALLOWED_CHECKED_NUM; i++)
    {
        history_buffer->history_refi_table[i][REFP_0] = REFI_INVALID;
        history_buffer->history_refi_table[i][REFP_1] = REFI_INVALID;
    }

    history_buffer->currCnt = 0;
    history_buffer->m_maxCnt = ALLOWED_CHECKED_NUM;

    return EVC_OK;
}

static int init_history_buffer(EVC_HISTORY_BUFFER *history_buffer)
{
    evc_mset(history_buffer->history_mv_table,   0, ALLOWED_CHECKED_NUM * REFP_NUM * MV_D * sizeof(s16));
#if TRACE_ENC_CU_DATA
    evc_mset(history_buffer->history_cu_table, 0, sizeof(history_buffer->history_cu_table[0])* ALLOWED_CHECKED_NUM);
#endif


    //evc_mset(history_buffer->history_refi_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * sizeof(s8));
    for (int i = 0; i < ALLOWED_CHECKED_NUM; i++)
    {
        history_buffer->history_refi_table[i][REFP_0] = REFI_INVALID;
        history_buffer->history_refi_table[i][REFP_1] = REFI_INVALID;
    }

    history_buffer->currCnt = 0;
    history_buffer->m_maxCnt = ALLOWED_CHECKED_NUM;

    return EVC_OK;
}

static int copy_history_buffer(EVC_HISTORY_BUFFER *dst, EVC_HISTORY_BUFFER *src)
{
    memcpy(dst->history_mv_table,   src->history_mv_table,   sizeof(s16)* ALLOWED_CHECKED_NUM * REFP_NUM * MV_D);
    memcpy(dst->history_refi_table, src->history_refi_table, sizeof(s8)* ALLOWED_CHECKED_NUM * REFP_NUM);
#if TRACE_ENC_CU_DATA
    memcpy(dst->history_cu_table, src->history_cu_table, sizeof(src->history_cu_table[0])* ALLOWED_CHECKED_NUM);
#endif

    dst->currCnt = src->currCnt;
    dst->m_maxCnt = src->m_maxCnt;

    return EVC_OK;
}

static int get_cu_pred_data(EVCE_CU_DATA *src, int x, int y, int log2_cuw, int log2_cuh, int log2_cus, int cud, EVCE_MODE *mi
#if AFFINE_UPDATE 
                            , EVCE_CTX *ctx, EVCE_CORE *core
#endif
)
{
    int cuw, cuh, cus;
    int cuw_scu, cuh_scu, cus_scu;
    int cx, cy;
    int idx_src;

    cx = x >> MIN_CU_LOG2;    //x = position in LCU, cx = 4x4 CU horizontal index
    cy = y >> MIN_CU_LOG2;    //y = position in LCU, cy = 4x4 CU vertical index

    cuw = 1 << log2_cuw;    //current CU width
    cuh = 1 << log2_cuh;    //current CU height
    cus = 1 << log2_cus;    //current CU buffer stride (= current CU width)
    cuw_scu = 1 << (log2_cuw - MIN_CU_LOG2);    //4x4 CU number in width
    cuh_scu = 1 << (log2_cuh - MIN_CU_LOG2);    //4x4 CU number in height
    cus_scu = 1 << (log2_cus - MIN_CU_LOG2);    //4x4 CU number in stride

    // only copy src's first row of 4x4 CUs to dis's all 4x4 CUs

    idx_src = cy * cus_scu + cx;

    mi->cu_mode = src->pred_mode[idx_src];
    mi->affine_flag = src->affine_flag[idx_src];
    mi->mv[REFP_0][MV_X] = src->mv[idx_src][REFP_0][MV_X];
    mi->mv[REFP_0][MV_Y] = src->mv[idx_src][REFP_0][MV_Y];
    mi->mv[REFP_1][MV_X] = src->mv[idx_src][REFP_1][MV_X];
    mi->mv[REFP_1][MV_Y] = src->mv[idx_src][REFP_1][MV_Y];

    mi->refi[REFP_0] = src->refi[idx_src][REFP_0];
    mi->refi[REFP_1] = src->refi[idx_src][REFP_1];

#if TRACE_ENC_CU_DATA
    mi->trace_cu_idx = src->trace_idx[idx_src];
#endif
#if TRACE_ENC_HISTORIC
    evc_mcpy(&mi->history_buf, src->history_buf + idx_src, sizeof(mi->history_buf));
#endif

#if TRACE_ENC_CU_DATA_CHECK
    evc_assert(mi->trace_cu_idx != 0);
#endif
    return EVC_OK;
}

#if DQP
void get_min_max_qp(EVCE_CTX * ctx, EVCE_CORE *core, s8 * min_qp, s8 * max_qp, int * is_dqp_set, SPLIT_MODE split_mode, int cuw, int cuh, u8 qp, int x0, int y0)
{
    *is_dqp_set = 0;
    if (!ctx->pps.cu_qp_delta_enabled_flag)
    {
        *min_qp = ctx->tile[core->tile_idx].qp;                           // Clip?
        *max_qp = ctx->tile[core->tile_idx].qp;
    }
    else
    {
        if (!(ctx->sps.dquant_flag))
        {
            if (split_mode != NO_SPLIT)
            {
                *min_qp = qp;                           // Clip?
                *max_qp = qp;
            }
            else
            {
                *min_qp = ctx->tile[core->tile_idx].qp;
                *max_qp = ctx->tile[core->tile_idx].qp + ctx->sh.dqp;
            }
        }
        else
        {
            *min_qp = qp;                           // Clip?
            *max_qp = qp;
            if (split_mode == NO_SPLIT && (CONV_LOG2(cuw) + CONV_LOG2(cuh) >= ctx->pps.cu_qp_delta_area) && core->cu_qp_delta_code_mode != 2)
            {
                core->cu_qp_delta_code_mode = 1;
                *min_qp = ctx->tile[core->tile_idx].qp;
                *max_qp = ctx->tile[core->tile_idx].qp + ctx->sh.dqp;

                if (CONV_LOG2(cuw) == 7 || CONV_LOG2(cuh) == 7)
                {
                    *is_dqp_set = 1;
                    core->cu_qp_delta_code_mode = 2;
                }
                else
                {
                    *is_dqp_set = 0;
                }
            }
            else if ((((CONV_LOG2(cuw) + CONV_LOG2(cuh) == ctx->pps.cu_qp_delta_area + 1) && (split_mode == SPLIT_TRI_VER || split_mode == SPLIT_TRI_HOR)) ||
                (CONV_LOG2(cuh) + CONV_LOG2(cuw) == ctx->pps.cu_qp_delta_area && core->cu_qp_delta_code_mode != 2)))
            {
                core->cu_qp_delta_code_mode = 2;
                *is_dqp_set = 1;
                *min_qp = ctx->tile[core->tile_idx].qp;
                *max_qp = ctx->tile[core->tile_idx].qp + ctx->sh.dqp;
            }
        }
    }
}
#endif
static int mode_cu_init(EVCE_CTX * ctx, EVCE_CORE * core, int x, int y, int log2_cuw, int log2_cuh, int cud)
{
#if TRACE_ENC_CU_DATA
    static u64  trace_idx = 1;
    core->trace_idx = trace_idx++;
#endif
    core->cuw = 1 << log2_cuw;
    core->cuh = 1 << log2_cuh;
    core->log2_cuw = log2_cuw;
    core->log2_cuh = log2_cuh;
    core->x_scu = PEL2SCU(x);
    core->y_scu = PEL2SCU(y);
    core->scup = ((u32)core->y_scu * ctx->w_scu) + core->x_scu;
    core->avail_cu = 0;
    core->avail_lr = LR_10;

    core->nnz[Y_C] = core->nnz[U_C] = core->nnz[V_C] = 0;
    evc_mset(core->nnz_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);
    core->cud = cud;
    core->cu_mode = MODE_INTRA;
    core->affine_flag = 0;
#if DMVR_FLAG
    core->dmvr_flag = 0;
#endif

    core->ibc_flag = 0;

    core->ats_inter_info = 0;
#if DQP
    /* Getting the appropriate QP based on dqp table*/
    int qp_i_cb, qp_i_cr;

    core->qp_y = GET_LUMA_QP(core->qp);

    qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_u_offset);
    qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_v_offset);

    core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * (BIT_DEPTH - 8);
    core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * (BIT_DEPTH - 8);

    ctx->pinter.qp_y = core->qp_y;
    ctx->pinter.qp_u = core->qp_u;
    ctx->pinter.qp_v = core->qp_v;

    ctx->pibc.qp_y = core->qp_y;
    ctx->pibc.qp_u = core->qp_u;
    ctx->pibc.qp_v = core->qp_v;

#endif
    evce_rdoq_bit_est(&core->s_curr_best[log2_cuw - 2][log2_cuh - 2],  core);

    return EVC_OK;
}

static void mode_cpy_rec_to_ref(EVCE_CORE *core, int x, int y, int w, int h, EVC_PIC *pic
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    EVCE_CU_DATA *cu_data;
    pel           *src, *dst;
    int            j, s_pic, off, size;
    int            log2_w, log2_h;
    int            stride;

    log2_w = CONV_LOG2(w);
    log2_h = CONV_LOG2(h);

    cu_data = &core->cu_data_best[log2_w - 2][log2_h - 2];

    s_pic = pic->s_l;

    stride = w;

    if(x + w > pic->w_l)
    {
        w = pic->w_l - x;
    }

    if(y + h > pic->h_l)
    {
        h = pic->h_l - y;
    }

#if M50761_CHROMA_NOT_SPLIT
    if (evc_check_luma(tree_cons))
    {
#endif
    /* luma */
    src = cu_data->reco[Y_C];
    dst = pic->y + x + y * s_pic;
    size = sizeof(pel) * w;

    for(j = 0; j < h; j++)
    {
        evc_mcpy(dst, src, size);
        src += stride;
        dst += s_pic;
    }
#if M50761_CHROMA_NOT_SPLIT
    }
    if (evc_check_chroma(tree_cons))
    {
#endif    
    /* chroma */
    s_pic = pic->s_c;
    off = (x >> 1) + (y >> 1) * s_pic;
    size = (sizeof(pel) * w) >> 1;

    src = cu_data->reco[U_C];
    dst = pic->u + off;
    for(j = 0; j < (h >> 1); j++)
    {
        evc_mcpy(dst, src, size);
        src += (stride >> 1);
        dst += s_pic;
    }

    src = cu_data->reco[V_C];
    dst = pic->v + off;
    for(j = 0; j < (h >> 1); j++)
    {
        evc_mcpy(dst, src, size);
        src += (stride >> 1);
        dst += s_pic;
    }
#if M50761_CHROMA_NOT_SPLIT
    }
#endif
}

void evce_set_affine_mvf(EVCE_CTX *ctx, EVCE_CORE *core, EVCE_MODE *mi)
{
    EVCE_CU_DATA *cu_data;
    int   log2_cuw, log2_cuh;
    int   w_cu;
    int   h_cu;
    int   i;
    int   lidx;
    int   idx;
    int   vertex_num = core->affine_flag + 1;
    int   aff_scup[VER_NUM];

    log2_cuw = CONV_LOG2(core->cuw);
    log2_cuh = CONV_LOG2(core->cuh);
    cu_data = &core->cu_data_temp[log2_cuw - 2][log2_cuh - 2];

    w_cu = core->cuw >> MIN_CU_LOG2;
    h_cu = core->cuh >> MIN_CU_LOG2;

    aff_scup[0] = 0;
    aff_scup[1] = (w_cu - 1);
    aff_scup[2] = (h_cu - 1) * w_cu;
    aff_scup[3] = (w_cu - 1) + (h_cu - 1) * w_cu;

    // derive sub-block size
    int sub_w = 4, sub_h = 4;
    derive_affine_subblock_size_bi( mi->affine_mv, mi->refi, core->cuw, core->cuh, &sub_w, &sub_h, vertex_num, NULL);

    int   sub_w_in_scu = PEL2SCU( sub_w );
    int   sub_h_in_scu = PEL2SCU( sub_h );
    int   half_w = sub_w >> 1;
    int   half_h = sub_h >> 1;

    for(lidx = 0; lidx < REFP_NUM; lidx++)
    {
        if(mi->refi[lidx] >= 0)
        {
            s16( *ac_mv )[MV_D] = mi->affine_mv[lidx];
            int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
            int mv_scale_hor = ac_mv[0][MV_X] << 7;
            int mv_scale_ver = ac_mv[0][MV_Y] << 7;
            int mv_scale_tmp_hor, mv_scale_tmp_ver;

            // convert to 2^(storeBit + iBit) precision
            dmv_hor_x = (ac_mv[1][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuw);      // deltaMvHor
            dmv_hor_y = (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuw);
            if ( vertex_num == 3 )
            {
                dmv_ver_x = (ac_mv[2][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuh);  // deltaMvVer
                dmv_ver_y = (ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuh);
            }
            else
            {
                dmv_ver_x = -dmv_hor_y;                                                 // deltaMvVer
                dmv_ver_y = dmv_hor_x;
            }

            idx = 0;
            for ( int h = 0; h < h_cu; h += sub_h_in_scu )
            {
                for ( int w = 0; w < w_cu; w += sub_w_in_scu )
                {
                    if ( w == 0 && h == 0 )
                    {
                        mv_scale_tmp_hor = ac_mv[0][MV_X];
                        mv_scale_tmp_ver = ac_mv[0][MV_Y];
                    }
                    else if ( w + sub_w_in_scu == w_cu && h == 0 )
                    {
                        mv_scale_tmp_hor = ac_mv[1][MV_X];
                        mv_scale_tmp_ver = ac_mv[1][MV_Y];
                    }
                    else if ( w == 0 && h + sub_h_in_scu == h_cu && vertex_num == 3 )
                    {
                        mv_scale_tmp_hor = ac_mv[2][MV_X];
                        mv_scale_tmp_ver = ac_mv[2][MV_Y];
                    }
                    else
                    {
                        int pos_x = (w << MIN_CU_LOG2) + half_w;
                        int pos_y = (h << MIN_CU_LOG2) + half_h;

                        mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
                        mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

                        // 1/16 precision, 18 bits, same as MC
                        evc_mv_rounding_s32( mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0 );
#if AFFINE_CLIPPING_BF
                        mv_scale_tmp_hor = EVC_CLIP3( -(1 << 17), (1 << 17) - 1, mv_scale_tmp_hor );
                        mv_scale_tmp_ver = EVC_CLIP3( -(1 << 17), (1 << 17) - 1, mv_scale_tmp_ver );
#else
                        mv_scale_tmp_hor = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor );
                        mv_scale_tmp_ver = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver );
#endif

                        // 1/4 precision, 16 bits for storage
                        mv_scale_tmp_hor >>= 2;
                        mv_scale_tmp_ver >>= 2;
                    }
                    
                    // save MV for each 4x4 block
                    for ( int y = h; y < h + sub_h_in_scu; y++ )
                    {
                        for ( int x = w; x < w + sub_w_in_scu; x++ )
                        {
                            idx = x + y * w_cu;
                            cu_data->mv[idx][lidx][MV_X] = (s16)mv_scale_tmp_hor;
                            cu_data->mv[idx][lidx][MV_Y] = (s16)mv_scale_tmp_ver;
                        }
                    }
                }
            }
            // save mvd for encoding, and reset vertex mv
            for(i = 0; i < vertex_num; i++)
            {
                cu_data->mvd[aff_scup[i]][lidx][MV_X] = mi->affine_mvd[lidx][i][MV_X];
                cu_data->mvd[aff_scup[i]][lidx][MV_Y] = mi->affine_mvd[lidx][i][MV_Y];
            }
        }
    }
}

static void copy_to_cu_data(EVCE_CTX *ctx, EVCE_CORE *core, EVCE_MODE *mi, s16 coef_src[N_C][MAX_CU_DIM])
{
    EVCE_CU_DATA *cu_data;
    int i, j, idx, size;
    int log2_cuw, log2_cuh;

    log2_cuw = CONV_LOG2(core->cuw);
    log2_cuh = CONV_LOG2(core->cuh);

    cu_data = &core->cu_data_temp[log2_cuw - 2][log2_cuh - 2];

#if M50761_CHROMA_NOT_SPLIT
    if (evce_check_luma(ctx, core))
    {
#endif
    /* copy coef */
    size = core->cuw * core->cuh * sizeof(s16);
    evc_mcpy(cu_data->coef[Y_C], coef_src[Y_C], size);
#if !M50761_CHROMA_NOT_SPLIT
    size = (core->cuw * core->cuh * sizeof(s16)) >> 2;
    evc_mcpy(cu_data->coef[U_C], coef_src[U_C], size);
    evc_mcpy(cu_data->coef[V_C], coef_src[V_C], size);
#endif

    /* copy reco */
    size = core->cuw * core->cuh * sizeof(pel);
    evc_mcpy(cu_data->reco[Y_C], mi->rec[Y_C], size);
#if !M50761_CHROMA_NOT_SPLIT
    size = (core->cuw * core->cuh * sizeof(pel)) >> 2;
    evc_mcpy(cu_data->reco[U_C], mi->rec[U_C], size);
    evc_mcpy(cu_data->reco[V_C], mi->rec[V_C], size);
#endif

#if TRACE_ENC_CU_DATA_CHECK
    evc_assert(core->trace_idx == mi->trace_cu_idx);
    evc_assert(core->trace_idx != 0);
#endif

    /* copy mode info */
    idx = 0;
    for(j = 0; j < core->cuh >> MIN_CU_LOG2; j++)
    {
        for(i = 0; i < core->cuw >> MIN_CU_LOG2; i++)
        {
            cu_data->pred_mode[idx + i] = core->cu_mode;
            cu_data->skip_flag[idx + i] = ((core->cu_mode == MODE_SKIP) || (core->cu_mode == MODE_SKIP_MMVD)) ? 1 : 0;
            cu_data->mmvd_flag[idx + i] = core->cu_mode == MODE_SKIP_MMVD ? 1 : 0;
            cu_data->nnz[Y_C][idx + i] = core->nnz[Y_C];
#if M50761_CHROMA_NOT_SPLIT
            for (int sb = 0; sb < MAX_SUB_TB_NUM; sb++)
            {
               cu_data->nnz_sub[Y_C][sb][idx + i] = core->nnz_sub[Y_C][sb];
            }
#if DQP
            cu_data->qp_y[idx + i] = core->qp_y;
            MCU_RESET_QP(cu_data->map_scu[idx + i]);
            if (ctx->pps.cu_qp_delta_enabled_flag)
            {
                MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->cu_mode == MODE_INTRA, ctx->slice_num, core->qp);
            }
            else
            {
                MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->cu_mode == MODE_INTRA, ctx->slice_num, ctx->tile[core->tile_idx].qp);
            }
#else
            cu_data->qp_y[idx + i] = core->qp_y;
            MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->cu_mode == MODE_INTRA, ctx->slice_num, ctx->tile[core->tile_idx].qp);
#endif
#else
            cu_data->nnz[U_C][idx + i] = core->nnz[U_C];
            cu_data->nnz[V_C][idx + i] = core->nnz[V_C];
            for(int c = 0; c < N_C; c++)
            {
                for(int sb = 0; sb < MAX_SUB_TB_NUM; sb++)
                {
                    cu_data->nnz_sub[c][sb][idx + i] = core->nnz_sub[c][sb];
                }
            }
#if DQP
            if(ctx->pps.cu_qp_delta_enabled_flag)
            {
                cu_data->qp_y[idx + i] = core->qp_y;
                cu_data->qp_u[idx + i] = core->qp_u;
                cu_data->qp_v[idx + i] = core->qp_v;
                MCU_RESET_QP(cu_data->map_scu[idx + i]);
                MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->cu_mode == MODE_INTRA, ctx->slice_num, core->qp);
            }
            else
            {
                cu_data->qp_y[idx + i] = core->qp_y;
                cu_data->qp_u[idx + i] = core->qp_u;
                cu_data->qp_v[idx + i] = core->qp_v;
                MCU_RESET_QP(cu_data->map_scu[idx + i]);
                MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->cu_mode == MODE_INTRA, ctx->slice_num, ctx->tile[core->tile_idx].qp);
            }
#else
            cu_data->qp_u[idx + i] = core->qp_u;
            cu_data->qp_v[idx + i] = core->qp_v;
            cu_data->qp_y[idx + i] = core->qp_y;
            MCU_SET_IF_COD_SN_QP(cu_data->map_scu[idx + i], core->cu_mode == MODE_INTRA, ctx->slice_num, ctx->tile[core->tile_idx].qp);
#endif
#endif
            if(cu_data->skip_flag[idx + i])
            {
                MCU_SET_SF(cu_data->map_scu[idx + i]);
            }
            else
            {
                MCU_CLR_SF(cu_data->map_scu[idx + i]);
            }
#if DMVR_FLAG
            MCU_CLR_DMVRF(cu_data->map_scu[idx + i]);
            if(core->cu_mode == MODE_SKIP || core->cu_mode == MODE_DIR)
            {
                cu_data->dmvr_flag[idx + i] = core->dmvr_flag;
                if(cu_data->dmvr_flag[idx + i])
                {
                    MCU_SET_DMVRF(cu_data->map_scu[idx + i]);
                }
            }
#endif
            cu_data->depth[idx + i] = core->cud;
            cu_data->ats_intra_cu[idx + i] = core->ats_intra_cu;
            cu_data->ats_mode_h[idx + i] = core->ats_mode >> 1;
            cu_data->ats_mode_v[idx + i] = core->ats_mode & 1;
            cu_data->ats_inter_info[idx + i] = core->ats_inter_info;
            cu_data->affine_flag[idx + i] = core->affine_flag;
            if(core->affine_flag)
            {
                MCU_SET_AFF(cu_data->map_scu[idx + i], core->affine_flag);

                MCU_SET_AFF_LOGW(cu_data->map_affine[idx + i], log2_cuw);
                MCU_SET_AFF_LOGH(cu_data->map_affine[idx + i], log2_cuh);
                MCU_SET_AFF_XOFF(cu_data->map_affine[idx + i], i);
                MCU_SET_AFF_YOFF(cu_data->map_affine[idx + i], j);
            }
            else
            {
                MCU_CLR_AFF(cu_data->map_scu[idx + i]);
            }

            if (ctx->param.use_ibc_flag)
            {
              cu_data->ibc_flag[idx + i] = core->ibc_flag;
              if (core->ibc_flag)
              {
                MCU_SET_IBC(cu_data->map_scu[idx + i]);
              }
              else
              {
                MCU_CLR_IBC(cu_data->map_scu[idx + i]);
              }
            }

            MCU_SET_LOGW(cu_data->map_cu_mode[idx + i], log2_cuw);
            MCU_SET_LOGH(cu_data->map_cu_mode[idx + i], log2_cuh);

            if(core->cu_mode == MODE_SKIP_MMVD)
            {
                MCU_SET_MMVDS(cu_data->map_cu_mode[idx + i]);
            }
            else
            {
                MCU_CLR_MMVDS(cu_data->map_cu_mode[idx + i]);
            }

            if(core->cu_mode == MODE_INTRA)
            {
                cu_data->ipm[0][idx + i] = core->ipm[0];
#if !M50761_CHROMA_NOT_SPLIT
                cu_data->ipm[1][idx + i] = core->ipm[1];
#endif
                cu_data->mv[idx + i][REFP_0][MV_X] = 0;
                cu_data->mv[idx + i][REFP_0][MV_Y] = 0;
                cu_data->mv[idx + i][REFP_1][MV_X] = 0;
                cu_data->mv[idx + i][REFP_1][MV_Y] = 0;
                cu_data->refi[idx + i][REFP_0] = -1;
                cu_data->refi[idx + i][REFP_1] = -1;
            }
            else if (core->cu_mode == MODE_IBC)
            {
              cu_data->refi[idx + i][REFP_0] = -1;
              cu_data->refi[idx + i][REFP_1] = -1;
              cu_data->mvp_idx[idx + i][REFP_0] = mi->mvp_idx[REFP_0];
              cu_data->mvp_idx[idx + i][REFP_1] = 0;
              cu_data->mv[idx + i][REFP_0][MV_X] = mi->mv[REFP_0][MV_X];
              cu_data->mv[idx + i][REFP_0][MV_Y] = mi->mv[REFP_0][MV_Y];
              cu_data->mv[idx + i][REFP_1][MV_X] = 0;
              cu_data->mv[idx + i][REFP_1][MV_Y] = 0;

              cu_data->mvd[idx + i][REFP_0][MV_X] = mi->mvd[REFP_0][MV_X];
              cu_data->mvd[idx + i][REFP_0][MV_Y] = mi->mvd[REFP_0][MV_Y];
            }
            else
            {
                cu_data->refi[idx + i][REFP_0] = mi->refi[REFP_0];
                cu_data->refi[idx + i][REFP_1] = mi->refi[REFP_1];
                cu_data->mvp_idx[idx + i][REFP_0] = mi->mvp_idx[REFP_0];
                cu_data->mvp_idx[idx + i][REFP_1] = mi->mvp_idx[REFP_1];
                cu_data->mvr_idx[idx + i] = mi->mvr_idx;
                cu_data->bi_idx[idx + i] = mi->bi_idx;
                cu_data->mmvd_idx[idx + i] = mi->mmvd_idx;
#if DMVR_LAG
                if(cu_data->dmvr_flag[idx + i])
                {
                    cu_data->mv[idx + i][REFP_0][MV_X] = mi->dmvr_mv[idx + i][REFP_0][MV_X];
                    cu_data->mv[idx + i][REFP_0][MV_Y] = mi->dmvr_mv[idx + i][REFP_0][MV_Y];
                    cu_data->mv[idx + i][REFP_1][MV_X] = mi->dmvr_mv[idx + i][REFP_1][MV_X];
                    cu_data->mv[idx + i][REFP_1][MV_Y] = mi->dmvr_mv[idx + i][REFP_1][MV_Y];

                    cu_data->unrefined_mv[idx + i][REFP_0][MV_X] = mi->mv[REFP_0][MV_X];
                    cu_data->unrefined_mv[idx + i][REFP_0][MV_Y] = mi->mv[REFP_0][MV_Y];
                    cu_data->unrefined_mv[idx + i][REFP_1][MV_X] = mi->mv[REFP_1][MV_X];
                    cu_data->unrefined_mv[idx + i][REFP_1][MV_Y] = mi->mv[REFP_1][MV_Y];
                }
                else
#endif
                {
                    cu_data->mv[idx + i][REFP_0][MV_X] = mi->mv[REFP_0][MV_X];
                    cu_data->mv[idx + i][REFP_0][MV_Y] = mi->mv[REFP_0][MV_Y];
                    cu_data->mv[idx + i][REFP_1][MV_X] = mi->mv[REFP_1][MV_X];
                    cu_data->mv[idx + i][REFP_1][MV_Y] = mi->mv[REFP_1][MV_Y];
                }

                cu_data->mvd[idx + i][REFP_0][MV_X] = mi->mvd[REFP_0][MV_X];
                cu_data->mvd[idx + i][REFP_0][MV_Y] = mi->mvd[REFP_0][MV_Y];
                cu_data->mvd[idx + i][REFP_1][MV_X] = mi->mvd[REFP_1][MV_X];
                cu_data->mvd[idx + i][REFP_1][MV_Y] = mi->mvd[REFP_1][MV_Y];
            }
#if TRACE_ENC_CU_DATA
            cu_data->trace_idx[idx + i] = core->trace_idx;
#endif
#if TRACE_ENC_HISTORIC
            evc_mcpy(cu_data->history_buf + idx + i, &core->history_buffer, sizeof(core->history_buffer));
#endif
        }

        idx += core->cuw >> MIN_CU_LOG2;
    }

    if(core->affine_flag)
    {
        evce_set_affine_mvf(ctx, core, mi);
    }
#if TRACE_ENC_CU_DATA_CHECK
    int w = PEL2SCU(core->cuw);
    int h = PEL2SCU(core->cuh);
    idx = 0;
    for (j = 0; j < h; ++j, idx += w)
    {
        for (i = 0; i < w; ++i)
        {
            evc_assert(cu_data->trace_idx[idx + i] == core->trace_idx);
        }
    }
#endif
#if M50761_CHROMA_NOT_SPLIT
    }
    if (evce_check_chroma(ctx, core))
    {
        /* copy coef */
        size = (core->cuw * core->cuh * sizeof(s16)) >> 2;
        evc_mcpy(cu_data->coef[U_C], coef_src[U_C], size);
        evc_mcpy(cu_data->coef[V_C], coef_src[V_C], size);

        /* copy reco */
        size = (core->cuw * core->cuh * sizeof(pel)) >> 2;
        evc_mcpy(cu_data->reco[U_C], mi->rec[U_C], size);
        evc_mcpy(cu_data->reco[V_C], mi->rec[V_C], size);

        /* copy mode info */
        idx = 0;
        for (j = 0; j < core->cuh >> MIN_CU_LOG2; j++)
        {
            for (i = 0; i < core->cuw >> MIN_CU_LOG2; i++)
            {
                cu_data->pred_mode_chroma[idx + i] = core->cu_mode;
                cu_data->nnz[U_C][idx + i] = core->nnz[U_C];
                cu_data->nnz[V_C][idx + i] = core->nnz[V_C];
                for (int c = U_C; c < N_C; c++)
                {
                    for (int sb = 0; sb < MAX_SUB_TB_NUM; sb++)
                    {
                        cu_data->nnz_sub[c][sb][idx + i] = core->nnz_sub[c][sb];
                    }
                }

                cu_data->qp_u[idx + i] = core->qp_u;
                cu_data->qp_v[idx + i] = core->qp_v;

                if (core->cu_mode == MODE_INTRA)
                {
                    cu_data->ipm[1][idx + i] = core->ipm[1];
                }
            }
            idx += core->cuw >> MIN_CU_LOG2;
        }
    }
#endif
}
#if !CODE_CLEAN
static void update_history_buffer(EVC_HISTORY_BUFFER *history_buffer, EVCE_MODE *mi, int slice_type)
{
    int i;
    if(history_buffer->currCnt == history_buffer->m_maxCnt)
    {
        for(i = 1; i < history_buffer->currCnt; i++)
        {
            evc_mcpy(history_buffer->history_mv_table[i - 1], history_buffer->history_mv_table[i], REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(history_buffer->history_refi_table[i - 1], history_buffer->history_refi_table[i], REFP_NUM * sizeof(s8));
        }

        evc_mcpy(history_buffer->history_mv_table[history_buffer->currCnt - 1], mi->mv, REFP_NUM * MV_D * sizeof(s16));
        evc_mcpy(history_buffer->history_refi_table[history_buffer->currCnt - 1], mi->refi, REFP_NUM * sizeof(s8));
    }
    else
    {
        evc_mcpy(history_buffer->history_mv_table[history_buffer->currCnt], mi->mv, REFP_NUM * MV_D * sizeof(s16));
        evc_mcpy(history_buffer->history_refi_table[history_buffer->currCnt], mi->refi, REFP_NUM * sizeof(s8));

        history_buffer->currCnt++;
    }
}
#endif
#if AFFINE_UPDATE
static void update_history_buffer_affine(EVC_HISTORY_BUFFER *history_buffer, EVCE_MODE *mi, int slice_type, EVCE_CORE *core)
{
    int i;
    if(history_buffer->currCnt == history_buffer->m_maxCnt)
    {
        for(i = 1; i < history_buffer->currCnt; i++)
        {
            evc_mcpy(history_buffer->history_mv_table[i - 1], history_buffer->history_mv_table[i], REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(history_buffer->history_refi_table[i - 1], history_buffer->history_refi_table[i], REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            history_buffer->history_cu_table[i - 1] = history_buffer->history_cu_table[i];
#endif
        }
        if(mi->affine_flag)
        {
            mi->mv_sp[REFP_0][MV_X] = 0;
            mi->mv_sp[REFP_0][MV_Y] = 0;
            mi->refi_sp[REFP_0] = REFI_INVALID;
            mi->mv_sp[REFP_1][MV_X] = 0;
            mi->mv_sp[REFP_1][MV_Y] = 0;
            mi->refi_sp[REFP_1] = REFI_INVALID;
            for (int lidx = 0; lidx < REFP_NUM; lidx++)
            {
                if (mi->refi[lidx] >= 0)
                {
                    s16(*ac_mv)[MV_D] = mi->affine_mv[lidx];
                    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
                    int mv_scale_hor = ac_mv[0][MV_X] << 7;
                    int mv_scale_ver = ac_mv[0][MV_Y] << 7;
                    int mv_y_hor = mv_scale_hor;
                    int mv_y_ver = mv_scale_ver;
                    int mv_scale_tmp_hor, mv_scale_tmp_ver;


                    dmv_hor_x = (ac_mv[1][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuw);
                    dmv_hor_y = (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuw);

                    if (core->affine_flag == 2)
                    {
                        dmv_ver_x = (ac_mv[2][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuh);
                        dmv_ver_y = (ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuh);
                    }
                    else
                    {
                        dmv_ver_x = -dmv_hor_y;
                        dmv_ver_y = dmv_hor_x;
                    }
                    int pos_x = 1 << (core->log2_cuw - 1);
                    int pos_y = 1 << (core->log2_cuh - 1);

                    mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
                    mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

#if HMVP_ON_AFFINE_UPDATE_BF
                    evc_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);
                    mv_scale_tmp_hor = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, mv_scale_tmp_ver);
#else
                    evc_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#if AFFINE_CLIPPING_BF
                    mv_scale_tmp_hor = EVC_CLIP3(-(1 << 17), (1 << 17) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(1 << 17), (1 << 17) - 1, mv_scale_tmp_ver);
#else
                    mv_scale_tmp_hor = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver);
#endif
                    mv_scale_tmp_hor >>= 2;
                    mv_scale_tmp_ver >>= 2;
#endif

                    mi->mv_sp[lidx][MV_X] = mv_scale_tmp_hor;
                    mi->mv_sp[lidx][MV_Y] = mv_scale_tmp_ver;
                    mi->refi_sp[lidx] = mi->refi[lidx];

                }
            }
            // some spatial neighbor may be unavailable
            if((slice_type == SLICE_P && REFI_IS_VALID(mi->refi_sp[REFP_0])) ||
                (slice_type == SLICE_B && (REFI_IS_VALID(mi->refi_sp[REFP_0]) || REFI_IS_VALID(mi->refi_sp[REFP_1]))))
            {
                evc_mcpy(history_buffer->history_mv_table[history_buffer->currCnt - 1], mi->mv_sp, REFP_NUM * MV_D * sizeof(s16));
                evc_mcpy(history_buffer->history_refi_table[history_buffer->currCnt - 1], mi->refi_sp, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
                history_buffer->history_cu_table[history_buffer->currCnt - 1] = mi->trace_cu_idx;
#endif
            }
        }
        else
        {
            evc_mcpy(history_buffer->history_mv_table[history_buffer->currCnt - 1], mi->mv, REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(history_buffer->history_refi_table[history_buffer->currCnt - 1], mi->refi, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            history_buffer->history_cu_table[history_buffer->currCnt - 1] = mi->trace_cu_idx;
#endif
        }
    }
    else
    {
        if(mi->affine_flag)
        {
            mi->mv_sp[REFP_0][MV_X] = 0;
            mi->mv_sp[REFP_0][MV_Y] = 0;
            mi->refi_sp[REFP_0] = REFI_INVALID;
            mi->mv_sp[REFP_1][MV_X] = 0;
            mi->mv_sp[REFP_1][MV_Y] = 0;
            mi->refi_sp[REFP_1] = REFI_INVALID;
            for (int lidx = 0; lidx < REFP_NUM; lidx++)
            {
                if (mi->refi[lidx] >= 0)
                {
                    s16(*ac_mv)[MV_D] = mi->affine_mv[lidx];
                    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
                    int mv_scale_hor = ac_mv[0][MV_X] << 7;
                    int mv_scale_ver = ac_mv[0][MV_Y] << 7;
                    int mv_y_hor = mv_scale_hor;
                    int mv_y_ver = mv_scale_ver;
                    int mv_scale_tmp_hor, mv_scale_tmp_ver;

                    dmv_hor_x = (ac_mv[1][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuw);
                    dmv_hor_y = (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuw);

                    if (core->affine_flag == 2)
                    {
                        dmv_ver_x = (ac_mv[2][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuh);
                        dmv_ver_y = (ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuh);
                    }
                    else
                    {
                        dmv_ver_x = -dmv_hor_y;
                        dmv_ver_y = dmv_hor_x;
                    }
                    int pos_x = 1 << (core->log2_cuw - 1);
                    int pos_y = 1 << (core->log2_cuh - 1);

                    mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
                    mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

#if HMVP_ON_AFFINE_UPDATE_BF
                    evc_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);
                    mv_scale_tmp_hor = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, mv_scale_tmp_ver);
#else
                    evc_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#if AFFINE_CLIPPING_BF
                    mv_scale_tmp_hor = EVC_CLIP3(-(1 << 17), (1 << 17) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(1 << 17), (1 << 17) - 1, mv_scale_tmp_ver);
#else
                    mv_scale_tmp_hor = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver);
#endif
                    mv_scale_tmp_hor >>= 2;
                    mv_scale_tmp_ver >>= 2;
#endif
                    mi->mv_sp[lidx][MV_X] = mv_scale_tmp_hor;
                    mi->mv_sp[lidx][MV_Y] = mv_scale_tmp_ver;
                    mi->refi_sp[lidx] = mi->refi[lidx];
                }
            }
            // some spatial neighbor may be unavailable
            if((slice_type == SLICE_P && REFI_IS_VALID(mi->refi_sp[REFP_0])) ||
                (slice_type == SLICE_B && (REFI_IS_VALID(mi->refi_sp[REFP_0]) || REFI_IS_VALID(mi->refi_sp[REFP_1]))))
            {
                evc_mcpy(history_buffer->history_mv_table[history_buffer->currCnt], mi->mv_sp, REFP_NUM * MV_D * sizeof(s16));
                evc_mcpy(history_buffer->history_refi_table[history_buffer->currCnt], mi->refi_sp, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
                history_buffer->history_cu_table[history_buffer->currCnt] = mi->trace_cu_idx;
#endif
            }
        }
        else
        {
            evc_mcpy(history_buffer->history_mv_table[history_buffer->currCnt], mi->mv, REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(history_buffer->history_refi_table[history_buffer->currCnt], mi->refi, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            history_buffer->history_cu_table[history_buffer->currCnt] = mi->trace_cu_idx;
#endif
        }

        history_buffer->currCnt++;
    }
}
#endif

static void update_map_scu(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int src_cuw, int src_cuh)
{
    u32  *map_scu = 0, *src_map_scu = 0;
    s8   *map_ipm = 0, *src_map_ipm = 0;
    s16(*map_mv)[REFP_NUM][MV_D] = 0, (*src_map_mv)[REFP_NUM][MV_D] = 0;
#if DMVR_LAG
    s16(*map_unrefined_mv)[REFP_NUM][MV_D] = 0, (*src_map_unrefined_mv)[REFP_NUM][MV_D] = 0;
#endif
    s8 (*map_refi)[REFP_NUM] = 0;
    s8 **src_map_refi = NULL;
    s8   *map_depth = 0, *src_depth = 0;
    int   size_depth;
    int   w, h, i, size, size_ipm, size_mv, size_refi;
    int   log2_src_cuw, log2_src_cuh;
    int   scu_x, scu_y;
    u32  *map_affine = 0, *src_map_affine = 0;
    u32  *map_cu_mode = 0, *src_map_cu_mode = 0;
    u8   *map_ats_inter = 0, *src_map_ats_inter = 0;

    scu_x = x >> MIN_CU_LOG2;
    scu_y = y >> MIN_CU_LOG2;
    log2_src_cuw = CONV_LOG2(src_cuw);
    log2_src_cuh = CONV_LOG2(src_cuh);

    map_scu = ctx->map_scu + scu_y * ctx->w_scu + scu_x;
    src_map_scu = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_scu;

    map_ipm = ctx->map_ipm + scu_y * ctx->w_scu + scu_x;
    src_map_ipm = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].ipm[0];

    map_mv = ctx->map_mv + scu_y * ctx->w_scu + scu_x;
    src_map_mv = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].mv;

    map_refi = ctx->map_refi + scu_y * ctx->w_scu + scu_x;
    src_map_refi = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].refi;
    map_depth = ctx->map_depth + scu_y * ctx->w_scu + scu_x;
    src_depth = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].depth;
#if DMVR_LAG
        map_unrefined_mv = ctx->map_unrefined_mv + scu_y * ctx->w_scu + scu_x;
        src_map_unrefined_mv = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].unrefined_mv;
#endif
    map_affine = ctx->map_affine + scu_y * ctx->w_scu + scu_x;
    src_map_affine = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_affine;
    map_ats_inter = ctx->map_ats_inter + scu_y * ctx->w_scu + scu_x;
    src_map_ats_inter = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].ats_inter_info;
    map_cu_mode = ctx->map_cu_mode + scu_y * ctx->w_scu + scu_x;
    src_map_cu_mode = core->cu_data_best[log2_src_cuw - 2][log2_src_cuh - 2].map_cu_mode;

    if(x + src_cuw > ctx->w)
    {
        w = (ctx->w - x) >> MIN_CU_LOG2;
    }
    else
    {
        w = (src_cuw >> MIN_CU_LOG2);
    }

    if(y + src_cuh > ctx->h)
    {
        h = (ctx->h - y) >> MIN_CU_LOG2;
    }
    else
    {
        h = (src_cuh >> MIN_CU_LOG2);
    }

    size = sizeof(u32) * w;
    size_ipm = sizeof(u8) * w;
    size_mv = sizeof(s16) * w * REFP_NUM * MV_D;
    size_refi = sizeof(s8) * w * REFP_NUM;
    size_depth = sizeof(s8) * w;

    for(i = 0; i < h; i++)
    {
        evc_mcpy(map_scu, src_map_scu, size);
        evc_mcpy(map_ipm, src_map_ipm, size_ipm);
        evc_mcpy(map_mv, src_map_mv, size_mv);
        evc_mcpy(map_refi, *(src_map_refi), size_refi);
#if DMVR_LAG
        evc_mcpy(map_unrefined_mv, src_map_unrefined_mv, size_mv);
#endif

        evc_mcpy(map_depth, src_depth, size_depth);

        map_depth += ctx->w_scu;
        src_depth += (src_cuw >> MIN_CU_LOG2);

        map_scu += ctx->w_scu;
        src_map_scu += (src_cuw >> MIN_CU_LOG2);

        map_ipm += ctx->w_scu;
        src_map_ipm += (src_cuw >> MIN_CU_LOG2);

        map_mv += ctx->w_scu;
        src_map_mv += (src_cuw >> MIN_CU_LOG2);
        
#if DMVR_LAG
        map_unrefined_mv += ctx->w_scu;
        src_map_unrefined_mv += (src_cuw >> MIN_CU_LOG2);
#endif
        map_refi += ctx->w_scu;
        src_map_refi += (src_cuw >> MIN_CU_LOG2);

        evc_mcpy(map_affine, src_map_affine, size);
        map_affine += ctx->w_scu;
        src_map_affine += (src_cuw >> MIN_CU_LOG2);
        evc_mcpy(map_ats_inter, src_map_ats_inter, size_ipm);
        map_ats_inter += ctx->w_scu;
        src_map_ats_inter += (src_cuw >> MIN_CU_LOG2);

        evc_mcpy(map_cu_mode, src_map_cu_mode, size);
        map_cu_mode += ctx->w_scu;
        src_map_cu_mode += (src_cuw >> MIN_CU_LOG2);
    }
}

static void clear_map_scu(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int cuw, int cuh)
{
    u32 *map_scu;
    int w, h, i, size;
    u32 *map_cu_mode = ctx->map_cu_mode + (y >> MIN_CU_LOG2) * ctx->w_scu + (x >> MIN_CU_LOG2);
    u32 *map_affine;
    map_affine = ctx->map_affine + (y >> MIN_CU_LOG2) * ctx->w_scu + (x >> MIN_CU_LOG2);
    u8 *map_ats_inter;
    map_ats_inter = ctx->map_ats_inter + (y >> MIN_CU_LOG2) * ctx->w_scu + (x >> MIN_CU_LOG2);
    map_scu = ctx->map_scu + (y >> MIN_CU_LOG2) * ctx->w_scu + (x >> MIN_CU_LOG2);

    if(x + cuw > ctx->w)
    {
        cuw = ctx->w - x;
    }

    if(y + cuh > ctx->h)
    {
        cuh = ctx->h - y;
    }

    w = (cuw >> MIN_CU_LOG2);
    h = (cuh >> MIN_CU_LOG2);

    size = sizeof(u32) * w;

    for(i = 0; i < h; i++)
    {
        evc_mset(map_scu, 0, size);
        map_scu += ctx->w_scu;
        evc_mset(map_affine, 0, size);
        map_affine += ctx->w_scu;
        evc_mset(map_ats_inter, 0, sizeof(u8) * w);
        map_ats_inter += ctx->w_scu;
        evc_mset(map_cu_mode, 0, size);
        map_cu_mode += ctx->w_scu;
    }
}

void evce_init_bef_data(EVCE_CORE* core, EVCE_CTX* ctx)
{
    int stride = 1 << ctx->log2_culine;
    int max_size = stride * stride; //size of a CTU
    int x_qt = stride >> 1;
    int y_qt = (stride >> 1) * stride;
    int boundary_CTU = 0;
    int ctu_size = 1 << (ctx->log2_max_cuwh);
    int x0 = core->x_pel;
    int y0 = core->y_pel;

    if((x0 / ctu_size + 1) * ctu_size > ctx->w || (y0 / ctu_size + 1) * ctu_size > ctx->h)
    {
        boundary_CTU = 1;
    }

    for(int m1 = 0; m1 < MAX_CU_DEPTH; m1++)
    {
        for(int m2 = 0; m2 < MAX_CU_DEPTH; m2++)
        {
            if(m1 >= ctx->log2_culine - 1 && m2 >= ctx->log2_culine - 1) //forced qt
            {
                evc_mset(&core->bef_data[m1][m2][0][0], 0, sizeof(EVCE_BEF_DATA) * MAX_BEF_DATA_NUM);
                evc_mset(&core->bef_data[m1][m2][x_qt][0], 0, sizeof(EVCE_BEF_DATA) * MAX_BEF_DATA_NUM);
                evc_mset(&core->bef_data[m1][m2][y_qt][0], 0, sizeof(EVCE_BEF_DATA) * MAX_BEF_DATA_NUM);
                evc_mset(&core->bef_data[m1][m2][x_qt + y_qt][0], 0, sizeof(EVCE_BEF_DATA) * MAX_BEF_DATA_NUM);
            }
            else
            {
                if(ALLOW_SPLIT_RATIO(max(m1, m2) + 2, abs(m1 - m2)) || boundary_CTU)
                {
                    evc_mset(&core->bef_data[m1][m2][0][0], 0, sizeof(EVCE_BEF_DATA) * MAX_BEF_DATA_NUM * max_size);
                }
            }
        }
    }
}

u16 evc_get_lr(u16 avail)
{
    u16 avail_lr = avail;
#if ENC_SUCO_FAST_CONFIG == 1
    avail_lr = 0;
#elif ENC_SUCO_FAST_CONFIG == 2
    avail_lr = (avail == LR_10 || avail == LR_00) ? 0 : 1;
#else // ENC_SUCO_FAST_CONFIG == 4
    avail_lr = avail;
#endif
    return avail_lr;
}

static double mode_coding_unit(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, int cud, EVCE_MODE *mi)
{
    s16(*coef)[MAX_CU_DIM] = core->ctmp;
    pel    *rec[N_C];
    double  cost_best, cost;
    int     i, s_rec[N_C];
#if M50761_CHROMA_NOT_SPLIT
    int start_comp = evce_check_luma(ctx, core) ? Y_C : U_C;
    int end_comp = evce_check_chroma(ctx, core) ? N_C : U_C;
#endif
#if AQS
    u16 avail_cu_rec;
#endif
    evc_assert(abs(log2_cuw - log2_cuh) <= 2);
    mode_cu_init(ctx, core, x, y, log2_cuw, log2_cuh, cud);
#if M50761_CHROMA_NOT_SPLIT
#if FIX_BTT_OFF
    if (ctx->sps.sps_btt_flag && log2_cuw == 2 && log2_cuh == 2 && ctx->sps.tool_admvp)
#else
    if (log2_cuw == 2 && log2_cuh == 2 && ctx->sps.tool_admvp)
#endif
    {
        // Check only in main profile
        evc_assert(!evce_check_all(ctx, core));
        evc_assert(evce_check_only_intra(ctx, core));
    }

    if (((log2_cuw + log2_cuh) == 5 && ctx->sps.tool_admvp) )
    {
        evc_assert(!evce_check_all_preds(ctx, core));

        if (evce_check_only_intra(ctx, core))
        {
            evc_assert(!evce_check_all(ctx, core));
        }
    }
#if !FIX_ADMPV_OFF
    if (!ctx->sps.tool_admvp)
    {
        evc_assert(evce_check_all(ctx, core));
        evc_assert(evce_check_all_preds(ctx, core));
    }
#endif
#endif
    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, (1 << log2_cuw), (1 << log2_cuh), ctx->w_scu, ctx->h_scu, ctx->map_scu, ctx->map_tidx);
    evc_get_ctx_some_flags(core->x_scu, core->y_scu, 1 << log2_cuw, 1 << log2_cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, core->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
                         , ctx->param.use_ibc_flag, ctx->sps.ibc_log_max_size, ctx->map_tidx);      

    /* inter *************************************************************/
    cost_best = MAX_COST;
    core->cost_best = MAX_COST;

    if(ctx->slice_type != SLICE_I && (ctx->sps.tool_admvp == 0 || !(log2_cuw <= MIN_CU_LOG2 && log2_cuh <= MIN_CU_LOG2))
#if M50761_CHROMA_NOT_SPLIT
        && (!evce_check_only_intra(ctx, core))
#endif
        )
    {
        core->avail_cu = evc_get_avail_inter(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu, ctx->map_tidx);
        cost = ctx->fn_pinter_analyze_cu(ctx, core, x, y, log2_cuw, log2_cuh, mi, coef, rec, s_rec);

        if(cost < cost_best)
        {
            cost_best = cost;
#if TRACE_ENC_CU_DATA
            mi->trace_cu_idx = core->trace_idx;
#endif
#if TRACE_ENC_HISTORIC
            evc_mcpy(&mi->history_buf, &core->history_buffer, sizeof(core->history_buffer));
#endif
#if TRACE_ENC_CU_DATA_CHECK
            evc_assert(core->trace_idx != 0);
#endif

#if M50761_CHROMA_NOT_SPLIT
            for (i = start_comp; i < end_comp; i++)
#else
            for(i = 0; i < N_C; i++)
#endif
            {
                mi->rec[i] = rec[i];
                mi->s_rec[i] = s_rec[i];
            }
#if DQP_RDO 
            if (ctx->pps.cu_qp_delta_enabled_flag)
            {
                evce_set_qp(ctx, core, core->dqp_next_best[log2_cuw - 2][log2_cuh - 2].prev_QP);
            }
#endif
            copy_to_cu_data(ctx, core, mi, coef);
        }
    }

    {
      if (ctx->param.use_ibc_flag == 1 && (core->nnz[Y_C] != 0 || core->nnz[U_C] != 0 || core->nnz[V_C] != 0 || cost_best == MAX_COST)
#if M50761_CHROMA_NOT_SPLIT
          && (!evce_check_only_inter(ctx, core)) && evce_check_luma(ctx, core)
#endif
          )
      {
        if (log2_cuw <= ctx->sps.ibc_log_max_size && log2_cuh <= ctx->sps.ibc_log_max_size)
        {
          core->avail_cu = evc_get_avail_ibc(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu, ctx->map_tidx);
          cost = ctx->fn_pibc_analyze_cu(ctx, core, x, y, log2_cuw, log2_cuh, mi, coef, rec, s_rec);

          if (cost < cost_best)
          {
            cost_best = cost;
            core->cu_mode = MODE_IBC;
            core->ibc_flag = 1;

            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
#if DQP_RDO
            DQP_STORE(core->dqp_next_best[log2_cuw - 2][log2_cuh - 2], core->dqp_temp_best);
#endif

            mi->pred_y_best = ctx->pibc.pred[0][Y_C];

            /* save all cu inforamtion ********************/
            mi->mvp_idx[0] = ctx->pibc.mvp_idx;
            {
              mi->mv[0][MV_X] = ctx->pibc.mv[0][MV_X];
              mi->mv[0][MV_Y] = ctx->pibc.mv[0][MV_Y];
            }

            mi->mvd[0][MV_X] = ctx->pibc.mvd[MV_X];
            mi->mvd[0][MV_Y] = ctx->pibc.mvd[MV_Y];

#if M50761_CHROMA_NOT_SPLIT
            for (i = start_comp; i < end_comp; i++)
#else
            for (i = 0; i < N_C; i++)
#endif
            {
              mi->rec[i] = rec[i];
              mi->s_rec[i] = s_rec[i];
            }

            core->skip_flag = 0;
            core->affine_flag = 0;
#if MMVD
            core->new_skip_flag = 0;
#endif
#if DMVR_FLAG
            core->dmvr_flag = 0;
#endif
            copy_to_cu_data(ctx, core, mi, coef);
          }
        }
      }
    }

    /* intra *************************************************************/
    if( (ctx->slice_type == SLICE_I || core->nnz[Y_C] != 0 || core->nnz[U_C] != 0 || core->nnz[V_C] != 0 || cost_best == MAX_COST)
#if M50761_CHROMA_NOT_SPLIT
        && (!evce_check_only_inter(ctx, core))
#endif
        )
    {
        core->cost_best = cost_best;
        core->dist_cu_best = EVC_INT32_MAX;

        if(core->cu_mode != MODE_IBC && core->cost_best != MAX_COST)
        {

            EVCE_PINTRA *pi = &ctx->pintra;

            core->inter_satd = evce_satd_16b(log2_cuw, log2_cuh, pi->o[Y_C] + (y * pi->s_o[Y_C]) + x, mi->pred_y_best, pi->s_o[Y_C], 1 << log2_cuw);
        }
        else
        {
            core->inter_satd = EVC_UINT32_MAX;
        }
#if DQP_RDO
        if (ctx->pps.cu_qp_delta_enabled_flag)
        {
            evce_set_qp(ctx, core, core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].curr_QP);
        }
#endif
        core->avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, log2_cuw, log2_cuh, ctx->map_scu, ctx->map_tidx);
        cost = ctx->fn_pintra_analyze_cu(ctx, core, x, y, log2_cuw, log2_cuh, mi, coef, rec, s_rec);

        if(cost < cost_best)
        {
            cost_best = cost;
#if TRACE_ENC_CU_DATA
            mi->trace_cu_idx = core->trace_idx;
#endif
#if TRACE_ENC_HISTORIC
            evc_mcpy(&mi->history_buf, &core->history_buffer, sizeof(core->history_buffer));
#endif
#if TRACE_ENC_CU_DATA_CHECK
            evc_assert(core->trace_idx != 0);
#endif
            core->cu_mode = MODE_INTRA;
            core->ibc_flag = 0;
            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
#if DQP_RDO
            DQP_STORE(core->dqp_next_best[log2_cuw - 2][log2_cuh - 2], core->dqp_temp_best);
#endif
            core->dist_cu_best = core->dist_cu;

#if M50761_CHROMA_NOT_SPLIT
            for (i = start_comp; i < end_comp; i++)
#else
            for(i = 0; i < N_C; i++)
#endif
            {
                mi->rec[i] = rec[i];
                mi->s_rec[i] = s_rec[i];
            }

            core->affine_flag = 0;
            copy_to_cu_data(ctx, core, mi, coef);
        }
    }

    return cost_best;
}

static u16 evc_get_avail_block(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int log2_cuw, int log2_cuh, u32 *map_scu, u8* map_tidx)
{
    u16 avail = 0;
    int log2_scuw, log2_scuh, scuw, scuh;

    log2_scuw = log2_cuw - MIN_CU_LOG2;
    log2_scuh = log2_cuh - MIN_CU_LOG2;
    scuw = 1 << log2_scuw;
    scuh = 1 << log2_scuh;

    if(x_scu > 0 && MCU_GET_COD(map_scu[scup - 1]) && (map_tidx[scup] == map_tidx[scup - 1]))
    {
        SET_AVAIL(avail, AVAIL_LE);
        if(y_scu + scuh < h_scu && MCU_GET_COD(map_scu[scup + (scuh * w_scu) - 1]) && (map_tidx[scup] == map_tidx[scup + (scuh * w_scu) - 1]))
        {
            SET_AVAIL(avail, AVAIL_LO_LE);
        }
    }

    if(y_scu > 0)
    {
        if (map_tidx[scup] == map_tidx[scup - w_scu])
        {
            SET_AVAIL(avail, AVAIL_UP);
        }
        if (map_tidx[scup] == map_tidx[scup - w_scu + scuw - 1])
        {
            SET_AVAIL(avail, AVAIL_RI_UP);
        }

        if(x_scu > 0 && MCU_GET_COD(map_scu[scup - w_scu - 1]) && (map_tidx[scup] == map_tidx[scup - w_scu - 1]) && (map_tidx[scup] == map_tidx[scup-1]))
        {
            SET_AVAIL(avail, AVAIL_UP_LE);
        }
        if(x_scu + scuw < w_scu && MCU_GET_COD(map_scu[scup - w_scu + scuw]) && (map_tidx[scup] == map_tidx[scup - w_scu + scuw]))
        {
            SET_AVAIL(avail, AVAIL_UP_RI);
        }
    }

    if(x_scu + scuw < w_scu && MCU_GET_COD(map_scu[scup + scuw]) && (map_tidx[scup] == map_tidx[scup + scuw]))
    {
        SET_AVAIL(avail, AVAIL_RI);

        if(y_scu + scuh < h_scu && MCU_GET_COD(map_scu[scup + (w_scu * scuh) + scuw]) && (map_tidx[scup] == map_tidx[scup + (w_scu * scuh) + scuw]))
        {
            SET_AVAIL(avail, AVAIL_LO_RI);
        }
    }

    return avail;
}

#if FAST_RECURSE_OPT
static int check_nev_block(EVCE_CTX *ctx, int x0, int y0, int log2_cuw, int log2_cuh, int *do_curr, int *do_split, int cud, int *nbr_map_skip_flag, EVCE_CORE * core)
#else
static int check_nev_block(EVCE_CTX *ctx, int x0, int y0, int log2_cuw, int log2_cuh, int *do_curr, int *do_split, int cud, EVCE_CORE * core)
#endif
{
    int avail_cu;
    int size_cnt[MAX_CU_DEPTH];
    int pos;
    int log2_scuw, log2_scuh, scuw, scuh;
    int tmp;
    int min_depth, max_depth;
    int cup;
    int x_scu, y_scu;
    int w, h;
#if FAST_RECURSE_OPT
    int nbr_map_skipcnt = 0;
    int nbr_map_cnt = 0;
#endif
#if M50761_CHROMA_NOT_SPLIT
    evc_assert(evce_check_luma(ctx, core));
#endif

    x_scu = (x0 >> MIN_CU_LOG2);
    y_scu = (y0 >> MIN_CU_LOG2);

    cup = y_scu * ctx->w_scu + x_scu;

    log2_scuw = log2_cuw - MIN_CU_LOG2;
    log2_scuh = log2_cuh - MIN_CU_LOG2;
    scuw = 1 << log2_scuw;
    scuh = 1 << log2_scuh;

    evc_mset(size_cnt, 0, sizeof(int) * MAX_CU_DEPTH);

    *do_curr = 1;
    *do_split = 1;
    avail_cu = evc_get_avail_block(x_scu, y_scu, ctx->w_scu, ctx->h_scu, cup, log2_cuw, log2_cuh, ctx->map_scu, ctx->map_tidx);

    min_depth = MAX_CU_DEPTH;
    max_depth = 0;

    if(IS_AVAIL(avail_cu, AVAIL_UP))
    {
        for(w = 0; w < scuw; w++)
        {
            pos = cup - ctx->w_scu + w;

            tmp = ctx->map_depth[pos];
            min_depth = tmp < min_depth ? tmp : min_depth;
            max_depth = tmp > max_depth ? tmp : max_depth;

#if FAST_RECURSE_OPT
            nbr_map_skipcnt += (1 == (MCU_GET_SF(ctx->map_scu[pos]) || MCU_GET_MMVDS(ctx->map_cu_mode[pos])));
            nbr_map_cnt++;
#endif
        }
    }

    if(IS_AVAIL(avail_cu, AVAIL_UP_RI))
    {
        pos = cup - ctx->w_scu + scuw;

        tmp = ctx->map_depth[pos];
        min_depth = tmp < min_depth ? tmp : min_depth;
        max_depth = tmp > max_depth ? tmp : max_depth;
    }

    if(IS_AVAIL(avail_cu, AVAIL_LE))
    {
        for(h = 0; h < scuh; h++)
        {
            pos = cup - 1 + (h * ctx->w_scu);

            tmp = ctx->map_depth[pos];
            min_depth = tmp < min_depth ? tmp : min_depth;
            max_depth = tmp > max_depth ? tmp : max_depth;

#if FAST_RECURSE_OPT
            nbr_map_skipcnt += (1 == (MCU_GET_SF(ctx->map_scu[pos])|| MCU_GET_MMVDS(ctx->map_cu_mode[pos])));
            nbr_map_cnt++;
#endif
        }
    }

    if(IS_AVAIL(avail_cu, AVAIL_LO_LE))
    {
        pos = cup + (ctx->w_scu * scuh) - 1;

        tmp = ctx->map_depth[pos];
        min_depth = tmp < min_depth ? tmp : min_depth;
        max_depth = tmp > max_depth ? tmp : max_depth;
    }

    if(IS_AVAIL(avail_cu, AVAIL_UP_LE))
    {
        pos = cup - ctx->w_scu - 1;

        tmp = ctx->map_depth[pos];
        min_depth = tmp < min_depth ? tmp : min_depth;
        max_depth = tmp > max_depth ? tmp : max_depth;
    }

    if(IS_AVAIL(avail_cu, AVAIL_RI))
    {
        for(h = 0; h < scuh; h++)
        {
            pos = cup + scuw + (h * ctx->w_scu);

            tmp = ctx->map_depth[pos];
            min_depth = tmp < min_depth ? tmp : min_depth;
            max_depth = tmp > max_depth ? tmp : max_depth;

#if FAST_RECURSE_OPT
            nbr_map_skipcnt += (1 == (MCU_GET_SF(ctx->map_scu[pos]) || MCU_GET_MMVDS(ctx->map_cu_mode[pos])));
            nbr_map_cnt++;
#endif
        }
    }

    if(IS_AVAIL(avail_cu, AVAIL_LO_RI))
    {
        pos = cup + (ctx->w_scu * scuh) + scuw;

        tmp = ctx->map_depth[pos];
        min_depth = tmp < min_depth ? tmp : min_depth;
        max_depth = tmp > max_depth ? tmp : max_depth;
    }

    if(avail_cu)
    {
        if(cud < min_depth - 1)
        {
            if(log2_cuw > MIN_CU_LOG2 && log2_cuh > MIN_CU_LOG2)
                *do_curr = 0;
            else
                *do_curr = 1;
        }

        if(cud > max_depth + 1)
        {
            *do_split = (*do_curr) ? 0 : 1;
        }
    }
#if FAST_RECURSE_OPT
    else
    {
        max_depth = MAX_CU_DEPTH;
        min_depth = 0;
    }

    *nbr_map_skip_flag = 0;
    if((ctx->slice_type != SLICE_I) && (nbr_map_skipcnt > (nbr_map_cnt / 2)))
    {
        *nbr_map_skip_flag = 1;
    }

    return (max_depth);
#else
    return 0;
#endif
}

static void check_run_split(EVCE_CORE *core, int log2_cuw, int log2_cuh, int cup, int next_split, int do_curr, int do_split, u16 bef_data_idx, int* split_allow, int boundary
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int i;
    double min_cost = MAX_COST;
    int run_list[MAX_SPLIT_NUM]; //a smaller set of allowed split modes based on a save & load technique

    if(!next_split)
    {
        split_allow[0] = 1;

        for(i = 1; i < MAX_SPLIT_NUM; i++)
        {
            split_allow[i] = 0;
        }

        return;
    }
    if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit)
    {
        if((core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].nosplit < 1
            && core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split >= 1))
        {
            run_list[0] = 0;

            for(i = 1; i < MAX_SPLIT_NUM; i++)
            {
                if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[i] < min_cost && split_allow[i])
                {
                    min_cost = core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[i];
                }
            }

            if(min_cost == MAX_COST)
            {
                run_list[0] = 1;
                for(i = 1; i < MAX_SPLIT_NUM; i++)
                {
                    if((core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split >> i) & 0x01)
                    {
                        run_list[i] = 1;
                    }
                    else
                    {
                        run_list[i] = 0;
                    }
                }
            }
            else
            {
                for(i = 1; i < MAX_SPLIT_NUM; i++)
                {
#if RDO_DBK //harmonize with cu split fast algorithm
                    double th = (min_cost < 0) ? 0.99 : 1.02;
                    if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[i] <= th * min_cost)
#else
                    if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[i] <= (1.01) * min_cost)
#endif
                    {
                        run_list[i] = 1;
                    }
                    else if((core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split >> i) & 0x01)
                    {
                        run_list[i] = 1;
                    }
                    else
                    {
                        run_list[i] = 0;
                    }
                }
            }
        }
        else if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].nosplit == 0
                && core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split == 0)
        {
            run_list[0] = 1;
            for(i = 1; i < MAX_SPLIT_NUM; i++)
            {
                if((core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split >> i) & 0x01)
                {
                    run_list[i] = 1;
            }
                else
                {
                    run_list[i] = 0;
                }
            }
        }
        else
        {
            run_list[0] = 1;

            for(i = 1; i < MAX_SPLIT_NUM; i++)
            {
                run_list[i] = 0;
            }
        }
    }
    else
    {
        for(i = 0; i < MAX_SPLIT_NUM; i++)
        {
            run_list[i] = 1;
            core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[i] = MAX_COST;
        }

        run_list[0] &= do_curr;

        for(i = 1; i < MAX_SPLIT_NUM; i++)
        {
            run_list[i] &= do_split;
        }
    }

    //modified split_allow by the save & load decision
    int num_run = 0;
    split_allow[0] = run_list[0];
    for(i = 1; i < MAX_SPLIT_NUM; i++)
    {
        split_allow[i] = run_list[i] && split_allow[i];
        num_run += split_allow[i];
    }

    //if all further splitting modes are not tried, at least we need try NO_SPLIT
    if(num_run == 0)
        split_allow[0] = 1;
}

#if RDO_DBK
void calc_delta_dist_filter_boundary(EVCE_CTX* ctx, EVC_PIC *pic_rec, EVC_PIC *pic_org, int cuw, int cuh,
                                     pel(*src)[MAX_CU_DIM], int s_src, int x, int y, u16 avail_lr, u8 intra_flag,
                                     u8 cbf_l, s8 *refi, s16(*mv)[MV_D], u8 is_mv_from_mvf, u8 ats_inter_info, EVCE_CORE * core)
{
    int i, j;
    int log2_cuw = CONV_LOG2(cuw);
    int log2_cuh = CONV_LOG2(cuh);
    int x_offset = 8; //for preparing deblocking filter taps
    int y_offset = 8;
    int x_tm = 4; //for calculating template dist
    int y_tm = 4; //must be the same as x_tm
    int log2_x_tm = CONV_LOG2(x_tm);
    int log2_y_tm = CONV_LOG2(y_tm);
    EVC_PIC *pic_dbk = ctx->pic_dbk;
    int s_l_dbk = pic_dbk->s_l;
    int s_c_dbk = pic_dbk->s_c;
    int s_l_org = pic_org->s_l;
    int s_c_org = pic_org->s_c;
    pel* dst_y = pic_dbk->y + y * s_l_dbk + x;
    pel* dst_u = pic_dbk->u + (y >> 1) * s_c_dbk + (x >> 1);
    pel* dst_v = pic_dbk->v + (y >> 1) * s_c_dbk + (x >> 1);
    pel* org_y = pic_org->y + y * s_l_org + x;
    pel* org_u = pic_org->u + (y >> 1) * s_c_org + (x >> 1);
    pel* org_v = pic_org->v + (y >> 1) * s_c_org + (x >> 1);
    int x_scu = x >> MIN_CU_LOG2;
    int y_scu = y >> MIN_CU_LOG2;
    int t = x_scu + y_scu * ctx->w_scu;
    //cu info to save
    u8 intra_flag_save, cbf_l_save;
    u8 do_filter = 0;
    u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
    u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
    int y_begin = ((ctx->tile[core->tile_num].ctba_rs_first) / ctx->w_lcu) << ctx->log2_max_cuwh;
    int y_begin_uv = (((ctx->tile[core->tile_num].ctba_rs_first) / ctx->w_lcu) << ctx->log2_max_cuwh)>>1;

    if(ctx->sh.deblocking_filter_on)
    {
        do_filter = 1;
    }

    if(do_filter == 0)
    {
        core->delta_dist[Y_C] = core->delta_dist[U_C] = core->delta_dist[V_C] = 0;
        return; //if no filter is applied, just return delta_dist as 0
    }

    //reset
    for (i = 0; i < N_C; i++)
    {
        core->dist_filter[i] = core->dist_nofilt[i] = 0;
    }


    /********************** prepare pred/rec pixels (not filtered) ****************************/

    //fill src to dst
    for(i = 0; i < cuh; i++)
        evc_mcpy(dst_y + i*s_l_dbk, src[Y_C] + i*s_src, cuw * sizeof(pel));

    //fill top
    if (y != y_begin)
    {
        for(i = 0; i < y_offset; i++)
            evc_mcpy(dst_y + (-y_offset + i)*s_l_dbk, pic_rec->y + (y - y_offset + i)*s_l_dbk + x, cuw * sizeof(pel));
    }

    //fill left
    if(avail_lr == LR_10 || avail_lr == LR_11)
    {
        for(i = 0; i < cuh; i++)
            evc_mcpy(dst_y + i*s_l_dbk - x_offset, pic_rec->y + (y + i)*s_l_dbk + (x - x_offset), x_offset * sizeof(pel));
    }

    //fill right
    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        for(i = 0; i < cuh; i++)
            evc_mcpy(dst_y + i*s_l_dbk + cuw, pic_rec->y + (y + i)*s_l_dbk + (x + cuw), x_offset * sizeof(pel));
    }

    //modify parameters from y to uv
    cuw >>= 1;  cuh >>= 1;  x_offset >>= 1;  y_offset >>= 1;  s_src >>= 1;  x >>= 1;  y >>= 1;  x_tm >>= 1;  y_tm >>= 1;
    log2_cuw -= 1;  log2_cuh -= 1;  log2_x_tm -= 1;  log2_y_tm -= 1;

    //fill src to dst
    for(i = 0; i < cuh; i++)
    {
        evc_mcpy(dst_u + i * s_c_dbk, src[U_C] + i * s_src, cuw * sizeof(pel));
        evc_mcpy(dst_v + i * s_c_dbk, src[V_C] + i * s_src, cuw * sizeof(pel));
    }

    //fill top
    if (y != y_begin_uv)
    {
        for(i = 0; i < y_offset; i++)
        {
            evc_mcpy(dst_u + (-y_offset + i)*s_c_dbk, pic_rec->u + (y - y_offset + i)*s_c_dbk + x, cuw * sizeof(pel));
            evc_mcpy(dst_v + (-y_offset + i)*s_c_dbk, pic_rec->v + (y - y_offset + i)*s_c_dbk + x, cuw * sizeof(pel));
        }
    }

    //fill left
    if (avail_lr == LR_10 || avail_lr == LR_11)
    {
        for(i = 0; i < cuh; i++)
        {
            evc_mcpy(dst_u + i * s_c_dbk - x_offset, pic_rec->u + (y + i)*s_c_dbk + (x - x_offset), x_offset * sizeof(pel));
            evc_mcpy(dst_v + i * s_c_dbk - x_offset, pic_rec->v + (y + i)*s_c_dbk + (x - x_offset), x_offset * sizeof(pel));
        }
    }

    //fill right
    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        for(i = 0; i < cuh; i++)
        {
            evc_mcpy(dst_u + i * s_c_dbk + cuw, pic_rec->u + (y + i)*s_c_dbk + (x + cuw), x_offset * sizeof(pel));
            evc_mcpy(dst_v + i * s_c_dbk + cuw, pic_rec->v + (y + i)*s_c_dbk + (x + cuw), x_offset * sizeof(pel));
        }
    }

    //recover
    cuw <<= 1;  cuh <<= 1;  x_offset <<= 1;  y_offset <<= 1;  s_src <<= 1;  x <<= 1;  y <<= 1;  x_tm <<= 1;  y_tm <<= 1;
    log2_cuw += 1;  log2_cuh += 1;  log2_x_tm += 1;  log2_y_tm += 1;

    //add distortion of current
    core->dist_nofilt[Y_C] += evce_ssd_16b(log2_cuw, log2_cuh, dst_y, org_y, s_l_dbk, s_l_org);

    //add distortion of top
    if (y != y_begin)

    {
        core->dist_nofilt[Y_C] += evce_ssd_16b(log2_cuw, log2_y_tm, dst_y - y_tm * s_l_dbk, org_y - y_tm * s_l_org, s_l_dbk, s_l_org);
    }
    if(avail_lr == LR_10 || avail_lr == LR_11)
    {
        core->dist_nofilt[Y_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_y - x_tm, org_y - x_tm, s_l_dbk, s_l_org);
    }
    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        core->dist_nofilt[Y_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_y + cuw, org_y + cuw, s_l_dbk, s_l_org);
    }
    cuw >>= 1;  cuh >>= 1;  x_offset >>= 1;  y_offset >>= 1;  s_src >>= 1;  x >>= 1;  y >>= 1;  x_tm >>= 1;  y_tm >>= 1;
    log2_cuw -= 1;  log2_cuh -= 1;  log2_x_tm -= 1;  log2_y_tm -= 1;
    core->dist_nofilt[U_C] += evce_ssd_16b(log2_cuw, log2_cuh, dst_u, org_u, s_c_dbk, s_c_org);
    core->dist_nofilt[V_C] += evce_ssd_16b(log2_cuw, log2_cuh, dst_v, org_v, s_c_dbk, s_c_org);

    if (y != y_begin_uv)
    {
        core->dist_nofilt[U_C] += evce_ssd_16b(log2_cuw, log2_y_tm, dst_u - y_tm*s_c_dbk, org_u - y_tm*s_c_org, s_c_dbk, s_c_org);
        core->dist_nofilt[V_C] += evce_ssd_16b(log2_cuw, log2_y_tm, dst_v - y_tm*s_c_dbk, org_v - y_tm*s_c_org, s_c_dbk, s_c_org);
    }
    if(avail_lr == LR_10 || avail_lr == LR_11)
    {
        core->dist_nofilt[U_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_u - x_tm, org_u - x_tm, s_c_dbk, s_c_org);
        core->dist_nofilt[V_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_v - x_tm, org_v - x_tm, s_c_dbk, s_c_org);
    }
    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        core->dist_nofilt[U_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_u + cuw, org_u + cuw, s_c_dbk, s_c_org);
        core->dist_nofilt[V_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_v + cuw, org_v + cuw, s_c_dbk, s_c_org);
    }

    //recover
    cuw <<= 1;  cuh <<= 1;  x_offset <<= 1;  y_offset <<= 1;  s_src <<= 1;  x <<= 1;  y <<= 1;  x_tm <<= 1;  y_tm <<= 1;
    log2_cuw += 1;  log2_cuh += 1;  log2_x_tm += 1;  log2_y_tm += 1;

    /********************************* filter the pred/rec **************************************/
    if(do_filter)
    {
        pic_dbk->pic_deblock_alpha_offset = ctx->deblock_alpha_offset;
        pic_dbk->pic_deblock_beta_offset = ctx->deblock_beta_offset;
        int w_scu = cuw >> MIN_CU_LOG2;
        int h_scu = cuh >> MIN_CU_LOG2;
        int ind, k;
        //save current best cu info
        intra_flag_save       = MCU_GET_IF(ctx->map_scu[t]);
        cbf_l_save            = MCU_GET_CBFL(ctx->map_scu[t]);        
        //set map info of current cu to current mode
        for(j = 0; j < h_scu; j++)
        {
            ind = (y_scu + j) * ctx->w_scu + x_scu;
            for(i = 0; i < w_scu; i++)
            {
                k = ind + i;
#if M50761_CHROMA_NOT_SPLIT
                if (evce_check_luma(ctx, core))
                {
#endif
                if(intra_flag)
                    MCU_SET_IF(ctx->map_scu[k]);
                else
                    MCU_CLR_IF(ctx->map_scu[k]);
                if(cbf_l)
                    MCU_SET_CBFL(ctx->map_scu[k]);
                else
                    MCU_CLR_CBFL(ctx->map_scu[k]);
#if M50761_CHROMA_NOT_SPLIT
                }
#endif
                if(refi != NULL && !is_mv_from_mvf)
                {
                    ctx->map_refi[k][REFP_0] = refi[REFP_0];
                    ctx->map_refi[k][REFP_1] = refi[REFP_1];
                    ctx->map_mv[k][REFP_0][MV_X] = mv[REFP_0][MV_X];
                    ctx->map_mv[k][REFP_0][MV_Y] = mv[REFP_0][MV_Y];
                    ctx->map_mv[k][REFP_1][MV_X] = mv[REFP_1][MV_X];
                    ctx->map_mv[k][REFP_1][MV_Y] = mv[REFP_1][MV_Y];

                    ctx->map_unrefined_mv[k][REFP_0][MV_X] = mv[REFP_0][MV_X];
                    ctx->map_unrefined_mv[k][REFP_0][MV_Y] = mv[REFP_0][MV_Y];
                    ctx->map_unrefined_mv[k][REFP_1][MV_X] = mv[REFP_1][MV_X];
                    ctx->map_unrefined_mv[k][REFP_1][MV_Y] = mv[REFP_1][MV_Y];
                }
#if DQP
                if(ctx->pps.cu_qp_delta_enabled_flag)
                {
                    MCU_RESET_QP(ctx->map_scu[k]);
                    MCU_SET_QP(ctx->map_scu[k], ctx->core->qp);
                }
                else
                {
                    MCU_SET_QP(ctx->map_scu[k], ctx->tile[core->tile_idx].qp); 
                }
#else
                MCU_SET_QP(ctx->map_scu[k], ctx->tile[core->tile_idx].qp); //TODO: this is wrong when using cu delta qp
#endif
                //clear coded (necessary)
                MCU_CLR_COD(ctx->map_scu[k]);
            }
        }

        if (ats_inter_info && cbf_l)
        {
            set_cu_cbf_flags(1, ats_inter_info, log2_cuw, log2_cuh, ctx->map_scu + t, ctx->w_scu);
        }

        //first, horizontal filtering
        // As of now filtering across tile boundaries is disabled
        evc_deblock_cu_hor(pic_dbk, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
            , core->tree_cons
#endif
            , ctx->map_tidx, 0
#if ADDB_FLAG_FIX
            , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
            , ctx->map_ats_inter
#endif
        );

        //clean coded flag in between two directional filtering (not necessary here)
        for(j = 0; j < h_scu; j++)
        {
            ind = (y_scu + j) * ctx->w_scu + x_scu;
            for(i = 0; i < w_scu; i++)
            {
                k = ind + i;
                MCU_CLR_COD(ctx->map_scu[k]);
            }
        }

        //then, vertical filtering
        evc_deblock_cu_ver(pic_dbk, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                           , ctx->map_cu_mode
#endif
                           , ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
            , core->tree_cons
#endif
            , ctx->map_tidx, 0
#if ADDB_FLAG_FIX
            , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
            , ctx->map_ats_inter
#endif
        );

        //recover best cu info
        for(j = 0; j < h_scu; j++)
        {
            ind = (y_scu + j) * ctx->w_scu + x_scu;
            for(i = 0; i < w_scu; i++)
            {
                k = ind + i;
#if M50761_CHROMA_NOT_SPLIT
                if (evce_check_luma(ctx, core))
                {
#endif
                    if (intra_flag_save)
                    {
                        MCU_SET_IF(ctx->map_scu[k]);
                    }
                    else
                    {
                        MCU_CLR_IF(ctx->map_scu[k]);
                    }

                    if (cbf_l_save)
                    {
                        MCU_SET_CBFL(ctx->map_scu[k]);
                    }
                    else
                    {
                        MCU_CLR_CBFL(ctx->map_scu[k]);
                    }
#if M50761_CHROMA_NOT_SPLIT
                }
#endif

                MCU_CLR_COD(ctx->map_scu[k]);
            }
        }
    }
    /*********************** calc dist of filtered pixels *******************************/
    //add current
    core->dist_filter[Y_C] += evce_ssd_16b(log2_cuw, log2_cuh, dst_y, org_y, s_l_dbk, s_l_org);
    //add  top
    if (y != y_begin)
    {
        core->dist_filter[Y_C] += evce_ssd_16b(log2_cuw, log2_y_tm, dst_y - y_tm * s_l_dbk, org_y - y_tm * s_l_org, s_l_dbk, s_l_org);
    }
    //add left
    if(avail_lr == LR_10 || avail_lr == LR_11)
    {
        core->dist_filter[Y_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_y - x_tm, org_y - x_tm, s_l_dbk, s_l_org);
    }
    //add right
    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        core->dist_filter[Y_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_y + cuw, org_y + cuw, s_l_dbk, s_l_org);
    }
    //modify parameters from y to uv
    cuw >>= 1;  cuh >>= 1;  x_offset >>= 1;  y_offset >>= 1;  s_src >>= 1;  x >>= 1;  y >>= 1;  x_tm >>= 1;  y_tm >>= 1;
    log2_cuw -= 1;  log2_cuh -= 1;  log2_x_tm -= 1;  log2_y_tm -= 1;
    //add current
    core->dist_filter[U_C] += evce_ssd_16b(log2_cuw, log2_cuh, dst_u, org_u, s_c_dbk, s_c_org);
    core->dist_filter[V_C] += evce_ssd_16b(log2_cuw, log2_cuh, dst_v, org_v, s_c_dbk, s_c_org);
    //add top
    if (y != y_begin_uv)
    {
        core->dist_filter[U_C] += evce_ssd_16b(log2_cuw, log2_y_tm, dst_u - y_tm * s_c_dbk, org_u - y_tm * s_c_org, s_c_dbk, s_c_org);
        core->dist_filter[V_C] += evce_ssd_16b(log2_cuw, log2_y_tm, dst_v - y_tm * s_c_dbk, org_v - y_tm * s_c_org, s_c_dbk, s_c_org);
    }
    //add left
    if(avail_lr == LR_10 || avail_lr == LR_11)
    {
        core->dist_filter[U_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_u - x_tm, org_u - x_tm, s_c_dbk, s_c_org);
        core->dist_filter[V_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_v - x_tm, org_v - x_tm, s_c_dbk, s_c_org);
    }
    //add right
    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        core->dist_filter[U_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_u + cuw, org_u + cuw, s_c_dbk, s_c_org);
        core->dist_filter[V_C] += evce_ssd_16b(log2_x_tm, log2_cuh, dst_v + cuw, org_v + cuw, s_c_dbk, s_c_org);
    }
    //recover
    cuw <<= 1;  cuh <<= 1;  x_offset <<= 1;  y_offset <<= 1;  s_src <<= 1;  x <<= 1;  y <<= 1;  x_tm <<= 1;  y_tm <<= 1;
    log2_cuw += 1;  log2_cuh += 1;  log2_x_tm += 1;  log2_y_tm += 1;

    /******************************* derive delta dist ********************************/
    core->delta_dist[Y_C] = core->dist_filter[Y_C] - core->dist_nofilt[Y_C];
    core->delta_dist[U_C] = core->dist_filter[U_C] - core->dist_nofilt[U_C];
    core->delta_dist[V_C] = core->dist_filter[V_C] - core->dist_nofilt[V_C];
}
#endif

#if FAST_RECURSE_OPT
void get_cud_min_max_avg(EVCE_CU_DATA *best_cu_data, int cuw, int cuh, int sub_cuw, int sub_cuh, int cux_offset, int cuy_offset, int *cud_min, int* cud_max, int *cud_avg)
{
    int i, j, idx, depth;
    int x_offset_scu = cux_offset >> MIN_CU_LOG2;
    int y_offset_scu = cuy_offset >> MIN_CU_LOG2;
    int min_depth = MAX_CU_DEPTH;
    int max_depth = 0;
    int sum = 0;

    evc_assert(cuw >= (1 << MIN_CU_LOG2));
    evc_assert(cuh >= (1 << MIN_CU_LOG2));
    evc_assert(sub_cuw >= (1 << MIN_CU_LOG2));
    evc_assert(sub_cuh >= (1 << MIN_CU_LOG2));
    evc_assert(sub_cuw <= cuw);
    evc_assert(sub_cuh <= cuh);
    evc_assert((cux_offset + sub_cuw) <= cuw);
    evc_assert((cuy_offset + sub_cuh) <= cuh);

    for(j = 0; j < (sub_cuh >> MIN_CU_LOG2); j++)
    {
        for(i = 0; i < (sub_cuw >> MIN_CU_LOG2); i++)
        {
            idx = (x_offset_scu + i) + ((y_offset_scu + j) * (cuw >> MIN_CU_LOG2));

            depth = best_cu_data->depth[idx];

            if(depth < min_depth)
            {
                min_depth = depth;
            }

            if(depth > max_depth)
            {
                max_depth = depth;
            }

            sum += depth;
        }
    }

    *cud_min = min_depth;
    *cud_max = max_depth;
    *cud_avg = sum / ((sub_cuw >> MIN_CU_LOG2) * (sub_cuh >> MIN_CU_LOG2));
}
#endif

#if FAST_RECURSE_OPT
static int lossycheck_biver(EVCE_CU_DATA *cu_data, int eval_parent_node_first, double cost_best, int log2_cuw, int log2_cuh, int cuw, int cuh, int cud, int nev_max_depth)
{
    int ans = 0;
    if(!eval_parent_node_first)
    {
        if(cost_best != MAX_COST)
        {
            int cud_min, cud_max, cud_avg;

            get_cud_min_max_avg(cu_data, cuw, cuh, cuw, cuh, 0, 0, &cud_min, &cud_max, &cud_avg);

            if(((cud_min > (cud + 6))) ||
               ((cud_min > (cud + 3)) && (cud_max > cud_min)) ||
               ((cud_max == (cud + 3)) && ((cud + 3) < nev_max_depth)))
            {
                ans = 1;
            }
        }
    }
    return ans;
}

static int lossycheck_bihor(EVCE_CU_DATA *cu_data, int eval_parent_node_first, double cost_best, int log2_cuw, int log2_cuh, int cuw, int cuh, int cud, int nev_max_depth)
{
    int ans = 0;
    int cud_min, cud_max, cud_avg;

    if(!eval_parent_node_first)
    {
        if(cost_best != MAX_COST)
        {
            get_cud_min_max_avg(cu_data, cuw, cuh, cuw, cuh, 0, 0, &cud_min, &cud_max, &cud_avg);

            if(((cud_min > (cud + 6))) ||
               ((cud_min > (cud + 3)) && (cud_max > cud_min)) ||
               ((cud_max == (cud + 3)) && ((cud + 3) < nev_max_depth)))
            {
                ans = 1;
            }
        }
    }
    else if(cuw == cuh)
    {
        if(cost_best != MAX_COST) //TODO: Check once more
        {
            get_cud_min_max_avg(cu_data, cuw, cuh, cuw, cuh, 0, 0, &cud_min, &cud_max, &cud_avg);

            if((cud_min > (cud + 2)) /*&& (cud_max > cud_min) */)
            {
                ans = 1;
            }
        }
    }
    return ans;
}

static int lossycheck_ttver(EVCE_CU_DATA *cu_data, int eval_parent_node_first, double cost_best, int log2_cuw, int log2_cuh, int cuw, int cuh, int cud, int nev_max_depth)
{
    int ans = 0;
    int cud_min, cud_max, cud_avg;

    if(cost_best != MAX_COST)
    {
        get_cud_min_max_avg(cu_data, cuw, cuh, (cuw >> 1), cuh, (cuw >> 2), 0, &cud_min, &cud_max, &cud_avg);

        if((cud_min > (cud + 3)) /*&& (cud_max > cud_min)*/)
        {
            ans = 1;
        }
    }
    return ans;
}

static int lossycheck_tthor(EVCE_CU_DATA *cu_data, int eval_parent_node_first, double cost_best, int log2_cuw, int log2_cuh, int cuw, int cuh, int cud, int nev_max_depth)
{
    int ans = 0;
    int cud_min, cud_max, cud_avg;

    if(cost_best != MAX_COST)
    {
        get_cud_min_max_avg(cu_data, cuw, cuh, cuw, (cuh >> 1), 0, (cuh >> 2), &cud_min, &cud_max, &cud_avg);

        if((cud_min > (cud + 3)) /*&& (cud_max > cud_min)*/)
        {
            ans = 1;
        }
    }
    return ans;
}
#endif

static double mode_coding_tree(EVCE_CTX *ctx, EVCE_CORE *core, int x0, int y0, int cup, int log2_cuw, int log2_cuh, int cud, EVCE_MODE *mi, int next_split
                               , int parent_suco, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth
#if DQP
    , u8 qp
#endif
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    // x0 = CU's left up corner horizontal index in entrie frame
    // y0 = CU's left up corner vertical index in entire frame
    // cuw = CU width, log2_cuw = CU width in log2
    // cuh = CU height, log2_chu = CU height in log2
    // ctx->w = frame width, ctx->h = frame height
    int cuw = 1 << log2_cuw;
    int cuh = 1 << log2_cuh;
    s8 best_split_mode = NO_SPLIT;
    int bit_cnt;
    double cost_best = MAX_COST;
    double cost_temp = MAX_COST;
    EVCE_SBAC s_temp_depth = {0};
    int boundary = !(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);
    int split_allow[SPLIT_QUAD + 1]; //allowed split by normative and non-normative selection
    s8 best_suco_flag = 0;
    u16 avail_lr = evc_check_nev_avail(PEL2SCU(x0), PEL2SCU(y0), cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, ctx->map_tidx);
    SPLIT_MODE split_mode = NO_SPLIT;
    int do_split, do_curr;
    double best_split_cost = MAX_COST;
    double best_curr_cost = MAX_COST;
    int split_mode_child[4] = { NO_SPLIT, NO_SPLIT, NO_SPLIT, NO_SPLIT };
    int curr_split_allow[SPLIT_QUAD + 1]; //allowed split by normative selection, used in entropy coding
    u8  remaining_split = 0;
    int num_split_tried = 0;
    int num_split_to_try = 0;
    int bef_data_idx = 0;
#if ET_BY_RDC_CHILD_SPLIT
    double split_cost[6] = { MAX_COST, MAX_COST, MAX_COST, MAX_COST, MAX_COST, MAX_COST };
    int split_mode_child_rdo[6][4];
#endif
#if FAST_RECURSE_OPT
    int nev_max_depth = 0;
    int eval_parent_node_first = 1;
    int nbr_map_skip_flag = 0;
    int cud_min = cud;
    int cud_max = cud;
    int cud_avg = cud;
#endif
#if DQP
    EVCE_DQP dqp_temp_depth = { 0 };
    u8 best_dqp = qp;
    s8 min_qp, max_qp;
    double cost_temp_dqp;
    double cost_best_dqp = MAX_COST;
    int dqp_coded = 0;
    int loop_counter;
    int dqp_loop;
    int cu_mode_dqp = 0;
    int dist_cu_best_dqp = 0;
    int ibc_flag_dqp = 0;
#endif
#if M50761_CHROMA_NOT_SPLIT
    core->tree_cons = tree_cons;    
#if M52165
    if (
#if FIX_BTT_OFF
        ctx->sps.sps_btt_flag &&
#endif
        log2_cuw == 2 && log2_cuh == 2 && (evce_check_luma(ctx, core) || evce_check_all(ctx, core)) && ctx->sps.tool_admvp
#else
    if (log2_cuw == 2 && log2_cuh == 2 && (evce_check_luma(ctx, core) || evce_check_all(ctx)) && ctx->sps.tool_admvp
#endif
        )
    {
        // Check only for main profile
        evc_assert(evce_check_only_intra(ctx, core));
    }
#endif

    // stroe the previous stored history MV list to m_pSplitTempMotLUTs, backup
    EVC_HISTORY_BUFFER OrigMotLUT, TempSubMotLUT;
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
    if (ctx->sps.tool_hmvp)
#else
    if (ctx->sps.tool_admvp)
#endif
    {
#endif
        copy_history_buffer(&OrigMotLUT, &core->m_pTempMotLUTs[log2_cuw - 2][log2_cuh - 2]);
#if HISTORY_UNDER_ADMVP_FIX
    }
#endif
    core->avail_lr = avail_lr;
    bef_data_idx = evc_get_lr(core->avail_lr);
    core->bef_data_idx = bef_data_idx;
#if DQP_RDO 
    if (ctx->pps.cu_qp_delta_enabled_flag)
    {
        bef_data_idx = (!!(qp - ctx->tile[core->tile_idx].qp) << 2) | bef_data_idx;
        core->bef_data_idx = bef_data_idx;
    }
#endif
    SBAC_LOAD(core->s_curr_before_split[log2_cuw - 2][log2_cuh - 2], core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

    //decide allowed split modes for the current node
    //based on CU size located at boundary
    if (cuw > ctx->min_cuwh || cuh > ctx->min_cuwh)
    {
        /***************************** Step 1: decide normatively allowed split modes ********************************/
        int boundary_b = boundary && (y0 + cuh > ctx->h) && !(x0 + cuw > ctx->w);
        int boundary_r = boundary && (x0 + cuw > ctx->w) && !(y0 + cuh > ctx->h);
        evc_check_split_mode(split_allow, log2_cuw, log2_cuh, boundary, boundary_b, boundary_r, ctx->log2_max_cuwh
                             , parent_split, same_layer_split, node_idx, parent_split_allow, qt_depth, btt_depth
                             , x0, y0, ctx->w, ctx->h
                             , &remaining_split, ctx->sps.sps_btt_flag
#if M50761_CHROMA_NOT_SPLIT
        , core->tree_cons.mode_cons
#endif
        );
        //save normatively allowed split modes, as it will be used in in child nodes for entropy coding of split mode
        memcpy(curr_split_allow, split_allow, sizeof(int)*MAX_SPLIT_NUM);
        for(int i = 1; i < MAX_SPLIT_NUM; i++)
            num_split_to_try += split_allow[i];

        /***************************** Step 2: reduce split modes by fast algorithm ********************************/
        do_split = do_curr = 1;
        if(!boundary)
        {
#if FAST_RECURSE_OPT
            nev_max_depth = check_nev_block(ctx, x0, y0, log2_cuw, log2_cuh, &do_curr, &do_split, cud, &nbr_map_skip_flag, core);
#else
            check_nev_block(ctx, x0, y0, log2_cuw, log2_cuh, &do_curr, &do_split, cud, core);
#endif
            do_split = do_curr = 1;
        }

        check_run_split(core, log2_cuw, log2_cuh, cup, next_split, do_curr, do_split, bef_data_idx, split_allow, boundary
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }
    else
    {
        split_allow[0] = 1;
        for(int i = 1; i < MAX_SPLIT_NUM; i++)
        {
            split_allow[i] = 0;
        }
    }

#if FAST_RECURSE_OPT && !FAST_RECURSE_OPT_FIX
    evc_assert(nev_max_depth >= 0);

    if((cud < (nev_max_depth - 0)) && split_allow[0] && !boundary && !nbr_map_skip_flag && (1 == abs(log2_cuw - log2_cuh)))
    {
        int i;
        int eval_split = 0;
        u8 is_TT_side_node = (node_idx != 1) && (parent_split == SPLIT_TRI_HOR || parent_split == SPLIT_TRI_VER);

        for(i = 1; i < MAX_SPLIT_NUM; i++)
        {
            eval_split |= split_allow[i];
        }

        if(eval_split && !is_TT_side_node)
        {
            eval_parent_node_first = (cuw == cuh);
        }
    }
#endif

#if FAST_RECURSE_OPT && !FAST_RECURSE_OPT_FIX
    if(!boundary  && eval_parent_node_first)
#else
    if(!boundary)
#endif
    {
        cost_temp = 0.0;

        init_cu_data(&core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], log2_cuw, log2_cuh, ctx->qp, ctx->qp, ctx->qp);

        // copy previous stored history MV list to current cu
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
        if (ctx->sps.tool_hmvp)
#else
        if (ctx->sps.tool_admvp)
#endif
        {
#endif
            copy_history_buffer(&core->history_buffer, &OrigMotLUT);
#if HISTORY_UNDER_ADMVP_FIX
        }
#endif
#if DQP
        ctx->sh.qp_prev_mode = core->dqp_data[log2_cuw - 2][log2_cuh - 2].prev_QP;
        best_dqp = ctx->sh.qp_prev_mode;
#endif
        split_mode = NO_SPLIT;
        if(split_allow[split_mode])
        {
            if ((cuw > ctx->min_cuwh || cuh > ctx->min_cuwh)
#if M50761_CHROMA_NOT_SPLIT
                && evce_check_luma(ctx, core)
#endif
                )
            {
                /* consider CU split mode */
                SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
                evce_sbac_bit_reset(&core->s_temp_run);
                evc_set_split_mode(NO_SPLIT, cud, 0, cuw, cuh, cuw, core->cu_data_temp[log2_cuw - 2][log2_cuh - 2].split_mode);
                evce_eco_split_mode(&core->bs_temp, ctx, core, cud, 0, cuw, cuh, cuw, parent_split, same_layer_split, node_idx, parent_split_allow, curr_split_allow, qt_depth, btt_depth, x0, y0);

                bit_cnt = evce_get_bit_number(&core->s_temp_run);
                cost_temp += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                SBAC_STORE(core->s_curr_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_run);
            }
            core->cup = cup;
#if DQP
            int is_dqp_set = 0;
            get_min_max_qp(ctx, core, &min_qp, &max_qp, &is_dqp_set, split_mode, cuw, cuh, qp, x0, y0);
            for (int dqp = min_qp; dqp <= max_qp; dqp++)
            {
                core->qp = GET_QP((s8)qp, dqp - (s8)qp);
                core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].curr_QP = core->qp;
                if (core->cu_qp_delta_code_mode != 2 || is_dqp_set)
                {
                    core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].cu_qp_delta_code = 1 + is_dqp_set;
                    core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].cu_qp_delta_is_coded = 0;
                }
                cost_temp_dqp = cost_temp;
                init_cu_data(&core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], log2_cuw, log2_cuh, ctx->qp, ctx->qp, ctx->qp);

                // copy previous stored history MV list to current cu
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                if (ctx->sps.tool_hmvp)
#else
                if (ctx->sps.tool_admvp)
#endif
                {
#endif
                    copy_history_buffer(&core->history_buffer, &OrigMotLUT);
#if HISTORY_UNDER_ADMVP_FIX
                }
#endif
                clear_map_scu(ctx, core, x0, y0, cuw, cuh);
                cost_temp_dqp += mode_coding_unit(ctx, core, x0, y0, log2_cuw, log2_cuh, cud, mi);

                if (cost_best > cost_temp_dqp)
                {
                    ibc_flag_dqp = core->ibc_flag;
                    cu_mode_dqp = core->cu_mode;
                    dist_cu_best_dqp = core->dist_cu_best;
                    /* backup the current best data */
                    copy_cu_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], &core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud
#if M50761_CHROMA_NOT_SPLIT
                        , core->tree_cons
#endif
                    );
                    cost_best = cost_temp_dqp;
                    best_split_mode = NO_SPLIT;
                    SBAC_STORE(s_temp_depth, core->s_next_best[log2_cuw - 2][log2_cuh - 2]);
#if DQP_RDO
                    DQP_STORE(dqp_temp_depth, core->dqp_next_best[log2_cuw - 2][log2_cuh - 2]);
#endif
                    mode_cpy_rec_to_ref(core, x0, y0, cuw, cuh, PIC_MODE(ctx)
#if M50761_CHROMA_NOT_SPLIT
                        , core->tree_cons
#endif
                    );
#if M50761_CHROMA_NOT_SPLIT
                    if (evce_check_luma(ctx, core))
                    {
#endif 
                    // update history MV list
                    // in mode_coding_unit, ctx->fn_pinter_analyze_cu will store the best MV in mi
                    // if the cost_temp has been update above, the best MV is in mi

                    get_cu_pred_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud, mi
#if AFFINE_UPDATE 
                        , ctx, core
#endif
                    );

#if AFFINE_UPDATE 
                    if (mi->cu_mode != MODE_INTRA && mi->cu_mode != MODE_IBC
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                        && ctx->sps.tool_hmvp
#else
                        && ctx->sps.tool_admvp
#endif
#endif
                        )
                    {
                        update_history_buffer_affine(&core->history_buffer, mi, ctx->slice_type, core);
                    }

#endif       
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                    if (ctx->sps.tool_hmvp)
#else
                    if (ctx->sps.tool_admvp)
#endif
                    {
#endif
                        copy_history_buffer(&core->m_pBestMotLUTs[log2_cuw - 2][log2_cuh - 2], &core->history_buffer);
#if HISTORY_UNDER_ADMVP_FIX
                    }
#endif
#if M50761_CHROMA_NOT_SPLIT
                }
#endif
                }
            }
            if (is_dqp_set && core->cu_qp_delta_code_mode == 2)
            {
                core->cu_qp_delta_code_mode = 0;
            }
            cost_temp = cost_best;
            core->ibc_flag = ibc_flag_dqp;
            core->cu_mode = cu_mode_dqp;
            core->dist_cu_best = dist_cu_best_dqp;
#else            
            clear_map_scu(ctx, core, x0, y0, cuw, cuh);
            cost_temp += mode_coding_unit(ctx, core, x0, y0, log2_cuw, log2_cuh, cud, mi);
#endif
#if TRACE_COSTS
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("Block [");
            EVC_TRACE_INT(x0);
            EVC_TRACE_STR(", ");
            EVC_TRACE_INT(y0);
            EVC_TRACE_STR("]x(");
            EVC_TRACE_INT(cuw);
            EVC_TRACE_STR("x");
            EVC_TRACE_INT(cuh);
            EVC_TRACE_STR(") split_type ");
            EVC_TRACE_INT(NO_SPLIT);
            EVC_TRACE_STR(" cost is ");
            EVC_TRACE_DOUBLE(cost_temp);
            EVC_TRACE_STR("\n");
#endif
        }
        else
        {
            cost_temp = MAX_COST;
        }

        if(!core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit)
        {
            core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[split_mode] = cost_temp;
            best_curr_cost = cost_temp;
        }

#if ET_BY_RDC_CHILD_SPLIT
        split_cost[split_mode] = cost_temp;
#endif
#if !DQP
        if(cost_best > cost_temp)
        {
            /* backup the current best data */
            copy_cu_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], &core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud
#if M50761_CHROMA_NOT_SPLIT
                , ctx->tree_cons
#endif
            );
            cost_best = cost_temp;
            best_split_mode = NO_SPLIT;
            SBAC_STORE(s_temp_depth, core->s_next_best[log2_cuw - 2][log2_cuh - 2]);
            mode_cpy_rec_to_ref(core, x0, y0, cuw, cuh, PIC_MODE(ctx)
#if M50761_CHROMA_NOT_SPLIT
                , ctx->tree_cons
#endif    
            );

#if M50761_CHROMA_NOT_SPLIT
            if (evce_check_luma(ctx, core))
            {
#endif 
            // update history MV list
            // in mode_coding_unit, ctx->fn_pinter_analyze_cu will store the best MV in mi
            // if the cost_temp has been update above, the best MV is in mi

            get_cu_pred_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud, mi
#if AFFINE_UPDATE 
                , ctx, core
#endif
            );

#if AFFINE_UPDATE 
            if (mi->cu_mode != MODE_INTRA && mi->cu_mode != MODE_IBC
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                && ctx->sps.tool_hmvp
#else
                && ctx->sps.tool_admvp
#endif
#endif
                )
            {
                update_history_buffer_affine(&core->history_buffer, mi, ctx->slice_type, core);
            }

#endif 
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
            if (ctx->sps.tool_hmvp)
#else
            if (ctx->sps.tool_admvp)
#endif
            {
#endif
                copy_history_buffer(&core->m_pBestMotLUTs[log2_cuw - 2][log2_cuh - 2], &core->history_buffer);
#if HISTORY_UNDER_ADMVP_FIX
            }
#endif
#if M50761_CHROMA_NOT_SPLIT
            }
#endif 
        }
#endif
        if(split_allow[split_mode] != 0)
        {
            core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].visit = 1;
        }
    }

#if ENC_ECU_ADAPTIVE
    if(cost_best != MAX_COST && cud >= (ctx->poc.poc_val % 2 ? (ctx->sps.sps_btt_flag ? ENC_ECU_DEPTH - 2 : ENC_ECU_DEPTH_B - 2) : (ctx->sps.sps_btt_flag ? ENC_ECU_DEPTH : ENC_ECU_DEPTH_B))
#else
    if(cost_best != MAX_COST && cud >= ENC_ECU_DEPTH
#endif
       && (core->cu_mode == MODE_SKIP || core->cu_mode == MODE_SKIP_MMVD)
       )
    {
        next_split = 0;
    }

    if(cost_best != MAX_COST && ctx->sh.slice_type == SLICE_I && core->ibc_flag != 1)
    {
        int dist_cu = core->dist_cu_best;
        int dist_cu_th = 1 << (log2_cuw + log2_cuh + 7);

        if(dist_cu < dist_cu_th)
        {
            u8 bits_inc_by_split = 0;
            bits_inc_by_split += (log2_cuw + log2_cuh >= 6) ? 2 : 0; //two split flags
            bits_inc_by_split += 8; //one more (intra dir + cbf + edi_flag + mtr info) + 1-bit penalty, approximately 8 bits

            if(dist_cu < ctx->lambda[0] * bits_inc_by_split)
                next_split = 0;
        }
    }

    if((cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE) && next_split)
    {
        SPLIT_MODE split_mode_order[MAX_SPLIT_NUM];
        int split_mode_num = 0;
#if M50761_CHROMA_NOT_SPLIT
        core->tree_cons = tree_cons;
#endif
        evc_split_get_split_rdo_order(cuw, cuh, split_mode_order);

#if FAST_RECURSE_OPT
        static LOSSY_ES_FUNC lossy_es[MAX_SPLIT_NUM] = {NULL, lossycheck_biver, lossycheck_bihor, lossycheck_ttver, lossycheck_tthor, NULL};
#endif

        for(split_mode_num = 1; split_mode_num < MAX_SPLIT_NUM; ++split_mode_num)
        {
            split_mode = split_mode_order[split_mode_num];
            int is_mode_TT = evc_split_is_TT(split_mode);
            int TT_not_skiped = is_mode_TT ? (best_split_mode != NO_SPLIT || cost_best == MAX_COST) : 1;

            if(split_allow[split_mode] && TT_not_skiped)
            {
                int suco_flag = 0;
                SPLIT_DIR split_dir = evc_split_get_direction(split_mode);
                int is_mode_BT = evc_split_is_BT(split_mode);
                u8  allow_suco = ctx->sps.sps_suco_flag ? evc_check_suco_cond(cuw, cuh, split_mode, boundary, ctx->log2_max_cuwh, ctx->sps.log2_diff_ctu_size_max_suco_cb_size, ctx->sps.log2_diff_max_suco_min_suco_cb_size) : 0;
                int num_suco = (split_dir == SPLIT_VER) ? 2 : 1;
                EVC_SPLIT_STRUCT split_struct;
                double cost_suco[2] = {MAX_COST, MAX_COST};
                int prev_suco_num = is_mode_TT ? 1 : (is_mode_BT ? 0 : 2);
                int prev_suco = core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[prev_suco_num];
#if FAST_RECURSE_OPT
                if(lossy_es[split_mode] && lossy_es[split_mode](&(core->cu_data_best[log2_cuw - 2][log2_cuh - 2])
                    , eval_parent_node_first, cost_best, log2_cuw, log2_cuh, cuw, cuh, cud, nev_max_depth))
                {
                    split_allow[split_mode] = 0;
                }
#endif

                if(split_allow[split_mode])
                {
                    num_split_tried++;
                }

                if(is_mode_TT)
                {
                    if(prev_suco == 0 && core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[0] > 0)
                    {
                        prev_suco = core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[0];
                    }
                }
                else
                {
                    if(!is_mode_BT)
                    {
                        // QT case
                        if(prev_suco == 0 && (core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[0] > 0 || core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[1] > 0))
                        {
                            prev_suco = core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[0] > 0 ? core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[0] : core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[1];
                        }
                    }
                }
                evc_split_get_part_structure( split_mode, x0, y0, cuw, cuh, cup, cud, ctx->log2_culine, &split_struct );

                if(split_allow[split_mode])
                {
#if M50761_CHROMA_NOT_SPLIT
                    split_struct.tree_cons = tree_cons;

                    BOOL mode_cons_changed = FALSE;
                    BOOL mode_cons_signal = FALSE;

                    if ( ctx->sps.tool_admvp && ctx->sps.sps_btt_flag ) // TODO: Tim, is special check needed here? create the specific variable for local dual tree ON/OFF
                    {
                        split_struct.tree_cons.changed = tree_cons.mode_cons == eAll && !evc_is_chroma_split_allowed( cuw, cuh, split_mode );
                        mode_cons_changed = evc_signal_mode_cons( &core->tree_cons, &split_struct.tree_cons ); 
                        mode_cons_signal = mode_cons_changed && ( ctx->sh.slice_type != SLICE_I ) && ( evc_get_mode_cons_by_split( split_mode, cuw, cuh ) == eAll );
                    }

                    for (int mode_num = 0; mode_num < (mode_cons_signal ? 2 : 1); ++mode_num)
                    {
                        if (mode_cons_changed)
                        {
                            evc_set_tree_mode(&split_struct.tree_cons, mode_num == 0 ? eOnlyIntra : eOnlyInter);
                        }
#if DQP
                        cost_suco[0] = MAX_COST;
                        cost_suco[1] = MAX_COST;
#endif
#endif
                    for(suco_flag = 0; suco_flag < num_suco; ++suco_flag)
                    {
                        int suco_order[SPLIT_MAX_PART_COUNT];
                        evc_split_get_suco_order(suco_flag, split_mode, suco_order);
                        int prev_log2_sub_cuw = split_struct.log_cuw[suco_order[0]];
                        int prev_log2_sub_cuh = split_struct.log_cuh[suco_order[0]];
#if DQP
                        int is_dqp_set = 0;
#endif
                        if(num_suco == 2)
                        {
                            if(prev_suco > 0 && suco_flag != (prev_suco - 1) && allow_suco)
                            {
                                continue;
                            }

                            if(!allow_suco && suco_flag != parent_suco)
                            {
                                continue;
                            }
                        }

                        init_cu_data(&core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], log2_cuw, log2_cuh, ctx->qp, ctx->qp, ctx->qp);
                        clear_map_scu(ctx, core, x0, y0, cuw, cuh);

                        int part_num = 0;

                        cost_temp = 0.0;

                        if(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h)
                        {
                            /* consider CU split flag */
                            SBAC_LOAD(core->s_temp_run, core->s_curr_before_split[log2_cuw - 2][log2_cuh - 2]);
                            evce_sbac_bit_reset(&core->s_temp_run);
                            evc_set_split_mode(split_mode, cud, 0, cuw, cuh, cuw, core->cu_data_temp[log2_cuw - 2][log2_cuh - 2].split_mode);
                            evce_eco_split_mode(&core->bs_temp, ctx, core, cud, 0, cuw, cuh, cuw, parent_split, same_layer_split, node_idx, parent_split_allow, curr_split_allow, qt_depth, btt_depth, x0, y0);

                            if(num_suco == 2)
                            {
                                evc_set_suco_flag(suco_flag, cud, 0, cuw, cuh, cuw, core->cu_data_temp[log2_cuw - 2][log2_cuh - 2].suco_flag);
                                evce_eco_suco_flag(&core->bs_temp, ctx, core, cud, 0, cuw, cuh, cuw, split_mode, boundary, ctx->log2_max_cuwh);
                            }
                            else
                            {
                                evc_set_suco_flag(suco_flag, cud, 0, cuw, cuh, cuw, core->cu_data_temp[log2_cuw - 2][log2_cuh - 2].suco_flag);
                            }
#if M50761_CHROMA_NOT_SPLIT
                            if ( ctx->sps.tool_admvp && ctx->sps.sps_btt_flag && mode_cons_signal )       // TODO: Tim, is special check needed here? create the specific variable for local dual tree ON/OFF
                            {
                                    evce_eco_mode_constr(&core->bs_temp, split_struct.tree_cons.mode_cons, core->ctx_flags[CNID_MODE_CONS]);
                            }
#endif
                            bit_cnt = evce_get_bit_number(&core->s_temp_run);
                            cost_temp += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                            SBAC_STORE(core->s_curr_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_run);
                        }
#if DQP
                        get_min_max_qp(ctx, core, &min_qp, &max_qp, &is_dqp_set, split_mode, cuw, cuh, qp, x0, y0);

                        loop_counter = 0;
                        if (is_dqp_set)
                        {
                            loop_counter = EVC_ABS(max_qp - min_qp);
                        }
                        cost_best_dqp = MAX_COST;
                        for (dqp_loop = 0; dqp_loop <= loop_counter; dqp_loop++)
                        {
                            int dqp = min_qp + dqp_loop;
                            core->qp = GET_QP((s8)qp, dqp - (s8)qp);
                            if (is_dqp_set)
                            {
                                core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].cu_qp_delta_code = 2;
                                core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].cu_qp_delta_is_coded = 0;
                                core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2].curr_QP = core->qp;
                            }

                            cost_temp_dqp = cost_temp;
                            init_cu_data(&core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], log2_cuw, log2_cuh, ctx->qp, ctx->qp, ctx->qp);
                            clear_map_scu(ctx, core, x0, y0, cuw, cuh);
#endif
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                        if (ctx->sps.tool_hmvp)
#else
                        if (ctx->sps.tool_admvp)
#endif
                        {
#endif
                            copy_history_buffer(&TempSubMotLUT, &OrigMotLUT);
#if HISTORY_UNDER_ADMVP_FIX
                        }
#endif
#if TRACE_ENC_CU_DATA_CHECK
                        static int counter_in[MAX_CU_LOG2 - MIN_CU_LOG2][MAX_CU_LOG2 - MIN_CU_LOG2] = { 0, };
                        counter_in[log2_cuw - MIN_CU_LOG2][log2_cuh - MIN_CU_LOG2]++;
#endif

                        for(part_num = 0; part_num < split_struct.part_count; ++part_num)
                        {
                            int cur_part_num = suco_order[part_num];
                            int log2_sub_cuw = split_struct.log_cuw[cur_part_num];
                            int log2_sub_cuh = split_struct.log_cuh[cur_part_num];
                            int x_pos = split_struct.x_pos[cur_part_num];
                            int y_pos = split_struct.y_pos[cur_part_num];
                            int cur_cuw = split_struct.width[cur_part_num];
                            int cur_cuh = split_struct.height[cur_part_num];
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                            if (ctx->sps.tool_hmvp)
#else
                            if (ctx->sps.tool_admvp)
#endif
                            {
#endif
                                copy_history_buffer(&core->m_pTempMotLUTs[log2_sub_cuw - 2][log2_sub_cuh - 2], &TempSubMotLUT);
#if HISTORY_UNDER_ADMVP_FIX
                            }
#endif
                            if((x_pos < ctx->w) && (y_pos < ctx->h))
                            {
                                if (part_num == 0)
                                {
                                    SBAC_LOAD(core->s_curr_best[log2_sub_cuw - 2][log2_sub_cuh - 2], core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
#if DQP
                                    DQP_STORE(core->dqp_curr_best[log2_sub_cuw - 2][log2_sub_cuh - 2], core->dqp_curr_best[log2_cuw - 2][log2_cuh - 2]);
#endif
                                }
                                else
                                {
                                    SBAC_LOAD(core->s_curr_best[log2_sub_cuw - 2][log2_sub_cuh - 2], core->s_next_best[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2]);
#if DQP
                                    DQP_STORE(core->dqp_curr_best[log2_sub_cuw - 2][log2_sub_cuh - 2], core->dqp_next_best[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2]);
#endif
                                }
#if DQP
                                cost_temp_dqp += mode_coding_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], log2_sub_cuw, log2_sub_cuh, split_struct.cud[cur_part_num], mi, 1
                                                              , (num_suco == 2) ? suco_flag : parent_suco, split_mode, split_mode_child
                                                              , part_num, curr_split_allow, INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, boundary)
#if DQP
                                                              , core->qp
#endif
#if M50761_CHROMA_NOT_SPLIT
                                                              , split_struct.tree_cons
#endif
                                                             );
                                core->qp = GET_QP((s8)qp, dqp - (s8)qp);
#else
                                cost_temp += mode_coding_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], log2_sub_cuw, log2_sub_cuh, split_struct.cud[cur_part_num], mi, 1
                                                              , (num_suco == 2) ? suco_flag : parent_suco, split_mode, split_mode_child
                                                              , part_num, curr_split_allow, INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, boundary)
#if M50761_CHROMA_NOT_SPLIT
                                                              , split_struct.tree_cons
#endif
                                                             );
#endif
                                copy_cu_data(&core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], &core->cu_data_best[log2_sub_cuw - 2][log2_sub_cuh - 2], x_pos - split_struct.x_pos[0], y_pos - split_struct.y_pos[0], log2_sub_cuw, log2_sub_cuh, log2_cuw, cud
#if M50761_CHROMA_NOT_SPLIT
                                    , split_struct.tree_cons
#endif
                                );
                                update_map_scu(ctx, core, x_pos, y_pos, cur_cuw, cur_cuh);
                                prev_log2_sub_cuw = log2_sub_cuw;
                                prev_log2_sub_cuh = log2_sub_cuh;
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                                if (ctx->sps.tool_hmvp)
#else
                                if (ctx->sps.tool_admvp)
#endif
                                {
#endif
                                    copy_history_buffer(&TempSubMotLUT, &core->m_pBestMotLUTs[log2_sub_cuw - 2][log2_sub_cuh - 2]);
#if HISTORY_UNDER_ADMVP_FIX
                                }
#endif
                            }
#if M50761_CHROMA_NOT_SPLIT                            
                            core->tree_cons = tree_cons;
#endif
                        }
#if M50761_CHROMA_NOT_SPLIT
                        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
                        {
                            evc_assert(evc_check_only_intra(split_struct.tree_cons));

                            core->tree_cons = split_struct.tree_cons;
                            core->tree_cons.tree_type = TREE_C;
                          
                            EVC_TRACE_COUNTER;
                            EVC_TRACE_STR("Cost luma: ");
                            EVC_TRACE_DOUBLE(cost_temp);
                            EVC_TRACE_STR("\n");
                            double cost_node = mode_coding_unit(ctx, core, x0, y0, log2_cuw, log2_cuh, cud, mi)
                                ;
#if DQP
                            cost_temp_dqp += cost_node;
#else
                            cost_temp += cost_node;
#endif
                            EVC_TRACE_STR("Cost chroma: ");
                            EVC_TRACE_DOUBLE(cost_node);
                            EVC_TRACE_STR("\n");
                            update_map_scu(ctx, core, x0, y0, cuw, cuh);
                            core->tree_cons = tree_cons;
                        }
#endif
#if TRACE_COSTS
                        EVC_TRACE_COUNTER;
                        EVC_TRACE_STR("Block [");
                        EVC_TRACE_INT(x0);
                        EVC_TRACE_STR(", ");
                        EVC_TRACE_INT(y0);
                        EVC_TRACE_STR("]x(");
                        EVC_TRACE_INT(cuw);
                        EVC_TRACE_STR("x");
                        EVC_TRACE_INT(cuh);
                        EVC_TRACE_STR(") split_type ");
                        EVC_TRACE_INT(split_mode);
                        EVC_TRACE_STR(" cost is ");
                        EVC_TRACE_DOUBLE(cost_temp);
                        EVC_TRACE_STR("\n");
#endif
#if TRACE_ENC_CU_DATA_CHECK
                        static int counter_out = 0;
                        counter_out++;
                        {
                            EVCE_CU_DATA *cu_data = &(core->cu_data_temp[log2_cuw - 2][log2_cuh - 2]);
                            int cuw = 1 << (log2_cuw - MIN_CU_LOG2);
                            int cuh = 1 << (log2_cuh - MIN_CU_LOG2);
                            int cus = cuw;
                            int idx = 0;
                            for (int j = 0; j < cuh; ++j)
                            {
                                int y_pos = y0 + (j << MIN_CU_LOG2);
                                for (int i = 0; i < cuw; ++i)
                                {
                                    int x_pos = x0 + (i << MIN_CU_LOG2);
                                    if ((x_pos < ctx->w) && (y_pos < ctx->h))
                                        evc_assert(cu_data->trace_idx[idx + i] != 0);
                                }
                                idx += cus;
                            }
                        }
#endif
#if DQP
                        if (cost_suco[suco_flag] > cost_temp_dqp)
                        {
                            cost_suco[suco_flag] = cost_temp_dqp;
                        }
                        if (cost_best_dqp > cost_temp_dqp)
                        {
                            cost_best_dqp = cost_temp_dqp;
                        }
#else
                        cost_suco[suco_flag] = cost_temp;
#endif

#if DQP
                        if (cost_best - 0.0001 > cost_temp_dqp)
#else
                        if(cost_best - 0.0001 > cost_temp)
#endif
                        {
                            /* backup the current best data */
                            copy_cu_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], &core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud
#if M50761_CHROMA_NOT_SPLIT
                                       , core->tree_cons
#endif
                            );
#if DQP
                            cost_best = cost_temp_dqp;
                            best_dqp = core->dqp_data[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2].prev_QP;
#if DQP_RDO
                            DQP_STORE(dqp_temp_depth, core->dqp_next_best[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2]);
#endif
#else
                            cost_best = cost_temp;
#endif
                            SBAC_STORE(s_temp_depth, core->s_next_best[prev_log2_sub_cuw - 2][prev_log2_sub_cuh - 2]);
                            best_split_mode = split_mode;
                            best_suco_flag = suco_flag;
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                            if (ctx->sps.tool_hmvp)
#else
                            if (ctx->sps.tool_admvp)
#endif
                            {
#endif
                                copy_history_buffer(&core->m_pBestMotLUTs[log2_cuw - 2][log2_cuh - 2], &TempSubMotLUT);
#if HISTORY_UNDER_ADMVP_FIX
                            }
#endif
                        }
#if DQP
                        }
                        cost_temp = cost_best_dqp;

                        if (is_dqp_set)
                        {
                            core->cu_qp_delta_code_mode = 0;
                        }
#endif
                    }
#if M50761_CHROMA_NOT_SPLIT
                    }
#endif
                }

                if(num_suco == 2)
                {
                    cost_temp = cost_suco[0] < cost_suco[1] ? cost_suco[0] : cost_suco[1];
                }

                if(split_mode != NO_SPLIT && cost_temp < best_split_cost)
                    best_split_cost = cost_temp;

#if ET_BY_RDC_CHILD_SPLIT
                split_cost[split_mode] = cost_temp;
                memcpy(split_mode_child_rdo[split_mode], split_mode_child, sizeof(int) * 4);
#endif
                if(!core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit)
                {
                    cost_temp = cost_suco[0] < cost_suco[1] ? cost_suco[0] : cost_suco[1];
                    core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[split_mode] = cost_temp;
                }
                else if((core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split >> split_mode) & 0x01)
                {
                    cost_temp = cost_suco[0] < cost_suco[1] ? cost_suco[0] : cost_suco[1];
                    core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[split_mode] = cost_temp;
                    core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split &= ~(1 << split_mode);
                }
                
                if(num_suco == 2 && core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[prev_suco_num] == 0 && allow_suco)
                {
                    core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].suco[prev_suco_num] = cost_suco[0] < cost_suco[1] ? 1 : 2;
                }
            }

            if(!core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit && num_split_tried > 0)
            {
                if((best_curr_cost * (1.10)) < best_split_cost)
                {
                    break;
                }
            }

#if ET_BY_RDC_CHILD_SPLIT
            int rdc_bits_th = 5;
            if(cuw < cuh)
            {
                if(split_cost[NO_SPLIT] != MAX_COST && split_cost[SPLIT_BI_HOR] != MAX_COST)
                {
                    if(split_cost[SPLIT_BI_HOR] < split_cost[NO_SPLIT] + ctx->lambda[0] * rdc_bits_th && split_cost[SPLIT_BI_HOR] > split_cost[NO_SPLIT]
                       && split_mode_child_rdo[SPLIT_BI_HOR][0] == NO_SPLIT && split_mode_child_rdo[SPLIT_BI_HOR][1] == NO_SPLIT)
                    {
                        break;
                    }
                }
            }
            else
            {
                if(split_cost[NO_SPLIT] != MAX_COST && split_cost[SPLIT_BI_VER] != MAX_COST)
                {
                    if(split_cost[SPLIT_BI_VER] < split_cost[NO_SPLIT] + ctx->lambda[0] * rdc_bits_th && split_cost[SPLIT_BI_VER] > split_cost[NO_SPLIT]
                       && split_mode_child_rdo[SPLIT_BI_VER][0] == NO_SPLIT && split_mode_child_rdo[SPLIT_BI_VER][1] == NO_SPLIT)
                    {
                        break;
                    }
                }
            }
#endif
        }
    }

    // restore the original history MV in m_pSplitTempMotLUTs to m_pTempMotLUTs
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
    if (ctx->sps.tool_hmvp)
#else
    if (ctx->sps.tool_admvp)
#endif
    {
#endif
        copy_history_buffer(&core->m_pTempMotLUTs[log2_cuw - 2][log2_cuh - 2], &OrigMotLUT);
        copy_history_buffer(&core->history_buffer, &OrigMotLUT);
#if HISTORY_UNDER_ADMVP_FIX
    }
#endif
#if FAST_RECURSE_OPT && !FAST_RECURSE_OPT_FIX
    /* Evaluate the parent mode after recursion */
    if(!boundary  && !eval_parent_node_first)
    {
        cost_temp = 0.0;
        split_mode = NO_SPLIT; /* Go back to no split mode */

        init_cu_data(&core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], log2_cuw, log2_cuh, ctx->qp, ctx->qp, ctx->qp);
        if(!eval_parent_node_first)
        {
            if(cost_best != MAX_COST)
            {
                get_cud_min_max_avg(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], cuw, cuh, cuw, cuh, 0, 0, &cud_min, &cud_max, &cud_avg);

                if((cud_min > (cud + 1)) || (cud_max > (cud + 1)))
                {
                    split_allow[split_mode] = 0;
                }
            }
        }

        if(split_allow[split_mode])
        {
            if(cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE)
            {
                /* consider CU split mode */
                SBAC_LOAD(core->s_temp_run, core->s_curr_before_split[log2_cuw - 2][log2_cuh - 2]);
                evce_sbac_bit_reset(&core->s_temp_run);
                evc_set_split_mode(NO_SPLIT, cud, 0, cuw, cuh, cuw, core->cu_data_temp[log2_cuw - 2][log2_cuh - 2].split_mode);
                evce_eco_split_mode(&core->bs_temp, ctx, core, cud, 0, cuw, cuh, cuw, parent_split, same_layer_split, node_idx, parent_split_allow, curr_split_allow, qt_depth, btt_depth, x0, y0);
                bit_cnt = evce_get_bit_number(&core->s_temp_run);
                cost_temp += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                SBAC_STORE(core->s_curr_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_run);
            }

            core->cup = cup;

            clear_map_scu(ctx, core, x0, y0, cuw, cuh);
            cost_temp += mode_coding_unit(ctx, core, x0, y0, log2_cuw, log2_cuh, cud, mi);
        }
        else
        {
            cost_temp = MAX_COST;
        }

        if(!core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit)
        {
            core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_cost[split_mode] = cost_temp;
            best_curr_cost = cost_temp;
        }

        if(cost_best > cost_temp)
        {
            /* backup the current best data */
            copy_cu_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], &core->cu_data_temp[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud
#if M50761_CHROMA_NOT_SPLIT
                , ctx->tree_cons
#endif
            );
            cost_best = cost_temp;
            best_split_mode = NO_SPLIT;
            SBAC_STORE(s_temp_depth, core->s_next_best[log2_cuw - 2][log2_cuh - 2]);
            mode_cpy_rec_to_ref(core, x0, y0, cuw, cuh, PIC_MODE(ctx)
#if M50761_CHROMA_NOT_SPLIT
                , ctx->tree_cons
#endif
            );

            // update history MV list
            // in mode_coding_unit, ctx->fn_pinter_analyze_cu will store the best MV in mi
            // if the cost_temp has been update above, the best MV is in mi

            get_cu_pred_data(&core->cu_data_best[log2_cuw - 2][log2_cuh - 2], 0, 0, log2_cuw, log2_cuh, log2_cuw, cud, mi
            );

#if AFFINE_UPDATE 
            if (mi->cu_mode != MODE_INTRA && !mi->affine_flag && mi->cu_mode != MODE_IBC
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
                && ctx->sps.tool_hmvp
#else
                && ctx->sps.tool_admvp
#endif
#endif
                )
            {
                update_history_buffer_affine(&core->history_buffer, mi, ctx->slice_type);
            }

#endif
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
            if (ctx->sps.tool_hmvp)
#else
            if (ctx->sps.tool_admvp)
#endif
            {
#endif
                copy_history_buffer(&core->m_pBestMotLUTs[log2_cuw - 2][log2_cuh - 2], &core->history_buffer);
#if HISTORY_UNDER_ADMVP_FIX
            }
#endif
        }

        if(split_allow[split_mode] != 0)
        {
            core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].visit = 1;
        }
    }
#endif

    mode_cpy_rec_to_ref(core, x0, y0, cuw, cuh, PIC_MODE(ctx)
#if M50761_CHROMA_NOT_SPLIT
        , core->tree_cons
#endif
    );

    /* restore best data */
    evc_set_split_mode(best_split_mode, cud, 0, cuw, cuh, cuw, core->cu_data_best[log2_cuw - 2][log2_cuh - 2].split_mode);
    evc_set_suco_flag(best_suco_flag, cud, 0, cuw, cuh, cuw, core->cu_data_best[log2_cuw - 2][log2_cuh - 2].suco_flag);

    SBAC_LOAD(core->s_next_best[log2_cuw - 2][log2_cuh - 2], s_temp_depth);
#if DQP_RDO
    DQP_LOAD(core->dqp_next_best[log2_cuw - 2][log2_cuh - 2], dqp_temp_depth);
#endif
    same_layer_split[node_idx] = best_split_mode;

    if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit != 1)
    {
        core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split = remaining_split;
    }

    if(num_split_to_try > 0)
    {
        if(best_split_mode == NO_SPLIT)
        {
            if(core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].remaining_split == 0)
            {
                core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].nosplit += 1;
            }
        }
        else
        {
            core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split += 1;
        }

        core->bef_data[log2_cuw - 2][log2_cuh - 2][cup][bef_data_idx].split_visit = 1;
    }

    evc_assert(cost_best != MAX_COST);
#if TRACE_ENC_CU_DATA_CHECK
    int i, j, w, h, w_scu;
    w = PEL2SCU(core->cuw);
    h = PEL2SCU(core->cuh);
    w_scu = 1 << (log2_cuw - MIN_CU_LOG2);
    for (j = 0; j < h; ++j)
    {
        int y_pos = core->y_pel + (j << MIN_CU_LOG2);
        for (i = 0; i < w; ++i)
        {
            int x_pos = core->x_pel + (i << MIN_CU_LOG2);
            if (x_pos < ctx->w && y_pos < ctx->h)
                evc_assert(core->cu_data_best[log2_cuw - 2][log2_cuh - 2].trace_idx[i + j * w_scu] != 0);
        }
    }
#endif
#if M50761_CHROMA_NOT_SPLIT    
    core->tree_cons = tree_cons;
#endif
    return (cost_best > MAX_COST) ? MAX_COST : cost_best;
}

static int mode_init_frame(EVCE_CTX *ctx)
{
    EVCE_MODE *mi;
    int ret;

    mi = &ctx->mode;

    /* set default values to mode information */
    mi->log2_culine = ctx->log2_max_cuwh - MIN_CU_LOG2;

    /* initialize pintra */
    if(ctx->fn_pintra_init_frame)
    {
        ret = ctx->fn_pintra_init_frame(ctx);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    /* initialize pinter */
    if(ctx->fn_pinter_init_frame)
    {
        ret = ctx->fn_pinter_init_frame(ctx);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    if (ctx->param.use_ibc_flag)
    {
      /* initialize pibc */
      if (ctx->fn_pibc_init_frame)
      {
        ret = ctx->fn_pibc_init_frame(ctx);
        evc_assert_rv(ret == EVC_OK, ret);
      }
      if (ctx->param.ibc_hash_search_flag)
        rebuild_hashmap(ctx->ibc_hash_handle, PIC_ORIG(ctx));
    }

    return EVC_OK;
}

static int mode_init_lcu(EVCE_CTX *ctx, EVCE_CORE *core)
{
    int ret;
    int num_size_idx = MAX_TR_LOG2 - MIN_CU_LOG2 + 1;

    evc_mset(ctx->ats_inter_num_pred, 0, sizeof(u8) * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2));


    /* initialize pintra */
    if(ctx->fn_pintra_init_lcu)
    {
        ret = ctx->fn_pintra_init_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    /* initialize pinter */
    if(ctx->fn_pinter_init_lcu)
    {
        ret = ctx->fn_pinter_init_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    if (ctx->param.use_ibc_flag)
    {
      /* initialize pibc */
      if (ctx->fn_pibc_init_lcu)
      {
        ret = ctx->fn_pibc_init_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
      }
    }

    return EVC_OK;
}

static int mode_analyze_frame(EVCE_CTX *ctx)
{
    return EVC_OK;
}

static void update_to_ctx_map(EVCE_CTX *ctx, EVCE_CORE *core)
{
    EVCE_CU_DATA *cu_data;
    int  cuw, cuh, i, j, w, h;
    int  x, y;
    int  core_idx, ctx_idx;
    s8(*map_refi)[REFP_NUM];
    s16(*map_mv)[REFP_NUM][MV_D];
#if DMVR_LAG
    s16(*map_unrefined_mv)[REFP_NUM][MV_D];
#endif
    u8* map_ats_intra_cu;
    u8* map_ats_mode_h;
    u8* map_ats_mode_v;
    u8   *map_ats_inter;
    s8   *map_ipm;

    cu_data = &core->cu_data_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2];
    cuw = ctx->max_cuwh;
    cuh = ctx->max_cuwh;
    x = core->x_pel;
    y = core->y_pel;

    if(x + cuw > ctx->w)
    {
        cuw = ctx->w - x;
    }

    if(y + cuh > ctx->h)
    {
        cuh = ctx->h - y;
    }

    w = cuw >> MIN_CU_LOG2;
    h = cuh >> MIN_CU_LOG2;

    /* copy mode info */
    core_idx = 0;
    ctx_idx = (y >> MIN_CU_LOG2) * ctx->w_scu + (x >> MIN_CU_LOG2);

    map_ipm = ctx->map_ipm;
    map_refi = ctx->map_refi;
    map_mv = ctx->map_mv;
#if DMVR_LAG
    map_unrefined_mv = ctx->map_unrefined_mv;
#endif
    map_ats_intra_cu = ctx->map_ats_intra_cu;
    map_ats_mode_h = ctx->map_ats_mode_h;
    map_ats_mode_v = ctx->map_ats_mode_v;
    map_ats_inter = ctx->map_ats_inter;

    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            map_ats_intra_cu[ctx_idx + j] = cu_data->ats_intra_cu[core_idx + j];
            map_ats_mode_h[ctx_idx + j] = cu_data->ats_mode_h[core_idx + j];
            map_ats_mode_v[ctx_idx + j] = cu_data->ats_mode_v[core_idx + j];
            map_ats_inter[ctx_idx + j] = cu_data->ats_inter_info[core_idx + j];

            if (core->cu_mode == MODE_IBC)
            {
                map_ats_inter[ctx_idx + j] = 0;
            }

            if(cu_data->pred_mode[core_idx + j] == MODE_INTRA)
            {
                map_ipm[ctx_idx + j] = cu_data->ipm[0][core_idx + j];
                map_mv[ctx_idx + j][REFP_0][MV_X] = 0;
                map_mv[ctx_idx + j][REFP_0][MV_Y] = 0;
                map_mv[ctx_idx + j][REFP_1][MV_X] = 0;
                map_mv[ctx_idx + j][REFP_1][MV_Y] = 0;
            }
            else
            {
                map_refi[ctx_idx + j][REFP_0] = cu_data->refi[core_idx + j][REFP_0];
                map_refi[ctx_idx + j][REFP_1] = cu_data->refi[core_idx + j][REFP_1];
                map_mv[ctx_idx + j][REFP_0][MV_X] = cu_data->mv[core_idx + j][REFP_0][MV_X];
                map_mv[ctx_idx + j][REFP_0][MV_Y] = cu_data->mv[core_idx + j][REFP_0][MV_Y];
                map_mv[ctx_idx + j][REFP_1][MV_X] = cu_data->mv[core_idx + j][REFP_1][MV_X];
                map_mv[ctx_idx + j][REFP_1][MV_Y] = cu_data->mv[core_idx + j][REFP_1][MV_Y];

#if DMVR_LAG //SEMIH: copy the map_mv to map_unrefined_mv in the else condition. --> use this in DBF.
                if (cu_data->dmvr_flag[core_idx + j])
                {
                  map_unrefined_mv[ctx_idx + j][REFP_0][MV_X] = cu_data->unrefined_mv[core_idx + j][REFP_0][MV_X];
                  map_unrefined_mv[ctx_idx + j][REFP_0][MV_Y] = cu_data->unrefined_mv[core_idx + j][REFP_0][MV_Y];
                  map_unrefined_mv[ctx_idx + j][REFP_1][MV_X] = cu_data->unrefined_mv[core_idx + j][REFP_1][MV_X];
                  map_unrefined_mv[ctx_idx + j][REFP_1][MV_Y] = cu_data->unrefined_mv[core_idx + j][REFP_1][MV_Y];
                }
                else
                {
                  map_unrefined_mv[ctx_idx + j][REFP_0][MV_X] = cu_data->mv[core_idx + j][REFP_0][MV_X];
                  map_unrefined_mv[ctx_idx + j][REFP_0][MV_Y] = cu_data->mv[core_idx + j][REFP_0][MV_Y];
                  map_unrefined_mv[ctx_idx + j][REFP_1][MV_X] = cu_data->mv[core_idx + j][REFP_1][MV_X];
                  map_unrefined_mv[ctx_idx + j][REFP_1][MV_Y] = cu_data->mv[core_idx + j][REFP_1][MV_Y];
                }
#endif
            }
        }
        ctx_idx += ctx->w_scu;
        core_idx += (ctx->max_cuwh >> MIN_CU_LOG2);
    }

    update_map_scu(ctx, core, core->x_pel, core->y_pel, ctx->max_cuwh, ctx->max_cuwh);
}

static int mode_analyze_lcu(EVCE_CTX *ctx, EVCE_CORE *core)
{
    EVCE_MODE *mi;
    u32 *map_scu;
    int i, j, w, h;
    int * split_mode_child = core->split_mode_child;
    int * parent_split_allow = core->parent_split_allow;
    for (int i = 0; i < 6; i++)
    {
        parent_split_allow[i] = 0;
        if(i==5)
            parent_split_allow[i] = 1;
    }

    mi = &ctx->mode;

    /* initialize cu data */
    init_cu_data(&core->cu_data_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], ctx->log2_max_cuwh, ctx->log2_max_cuwh, ctx->qp, ctx->qp, ctx->qp);
    init_cu_data(&core->cu_data_temp[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], ctx->log2_max_cuwh, ctx->log2_max_cuwh, ctx->qp, ctx->qp, ctx->qp);
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
    if (ctx->sps.tool_hmvp)
#else
    if (ctx->sps.tool_admvp)
#endif
    {
#endif
    evce_hmvp_init(&core->m_pTempMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2]);
    evce_hmvp_init(&core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2]);
    
    copy_history_buffer(&core->m_pTempMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], &core->history_buffer);
    copy_history_buffer(&core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], &core->history_buffer);
#if HISTORY_UNDER_ADMVP_FIX
    }
#endif
    evc_mset(mi->mvp_idx, 0, sizeof(u8) * REFP_NUM);
    evc_mset(mi->mvd, 0, sizeof(s16) * REFP_NUM * MV_D);

    /* decide mode */
    mode_coding_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->log2_max_cuwh, ctx->log2_max_cuwh, 0, mi, 1
                     , 0, NO_SPLIT, split_mode_child, 0, parent_split_allow, 0, 0
#if DQP
                     , ctx->tile[core->tile_idx].qp
#endif
#if M50761_CHROMA_NOT_SPLIT
                     , evc_get_default_tree_cons()
#endif
    );

#if TRACE_ENC_CU_DATA_CHECK
    h = w = 1 << (ctx->log2_max_cuwh - MIN_CU_LOG2);
    for(j = 0; j < h; ++j)
    {
        int y_pos = core->y_pel + (j << MIN_CU_LOG2);
        for(i = 0; i < w; ++i)
        {
            int x_pos = core->x_pel + (i << MIN_CU_LOG2);
            if(x_pos < ctx->w && y_pos < ctx->h)
                evc_assert(core->cu_data_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].trace_idx[i + h * j] != 0);
        }
    }
#endif

    update_to_ctx_map(ctx, core);
    copy_cu_data(&ctx->map_cu_data[core->lcu_num], &core->cu_data_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2],
                 0, 0, ctx->log2_max_cuwh, ctx->log2_max_cuwh, ctx->log2_max_cuwh, 0
#if M50761_CHROMA_NOT_SPLIT
        , evc_get_default_tree_cons()
#endif
    );
#if TRACE_ENC_CU_DATA_CHECK
    h = w = 1 << (ctx->log2_max_cuwh - MIN_CU_LOG2);
    for(j = 0; j < h; ++j)
    {
        int y_pos = core->y_pel + (j << MIN_CU_LOG2);
        for(i = 0; i < w; ++i)
        {
            int x_pos = core->x_pel + (i << MIN_CU_LOG2);
            if(x_pos < ctx->w && y_pos < ctx->h)
                evc_assert(core->cu_data_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].trace_idx[i + h * j] != 0);
        }
    }
    for(j = 0; j < h; ++j)
    {
        int y_pos = core->y_pel + (j << MIN_CU_LOG2);
        for(i = 0; i < w; ++i)
        {
            int x_pos = core->x_pel + (i << MIN_CU_LOG2);
            if(x_pos < ctx->w && y_pos < ctx->h)
                evc_assert(ctx->map_cu_data[core->lcu_num].trace_idx[i + h * j] != 0);
        }
    }
#endif
#if HISTORY_UNDER_ADMVP_FIX
#if M53737
    if (ctx->sps.tool_hmvp)
#else
    if (ctx->sps.tool_admvp)
#endif
    {
#endif
        copy_history_buffer(&core->history_buffer, &core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2]);
#if HISTORY_UNDER_ADMVP_FIX
    }
#endif
    /* Reset all coded flag for the current lcu */
    core->x_scu = PEL2SCU(core->x_pel);
    core->y_scu = PEL2SCU(core->y_pel);
    map_scu = ctx->map_scu + ((u32)core->y_scu * ctx->w_scu) + core->x_scu;
    w = EVC_MIN(1 << (ctx->log2_max_cuwh - MIN_CU_LOG2), ctx->w_scu - core->x_scu);
    h = EVC_MIN(1 << (ctx->log2_max_cuwh - MIN_CU_LOG2), ctx->h_scu - core->y_scu);

    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            MCU_CLR_COD(map_scu[j]);
        }
        map_scu += ctx->w_scu;
    }

    return EVC_OK;
}

static int mode_set_complexity(EVCE_CTX *ctx, int complexity)
{
    EVCE_MODE  *mi;

    mi = &ctx->mode;
    evc_assert_rv(mi != NULL, EVC_ERR_UNEXPECTED);

    return EVC_OK;
}

int evce_mode_create(EVCE_CTX *ctx, int complexity)
{
    EVCE_MODE *mi;

    mi = &ctx->mode;

    /* create mode information structure */
    evc_assert_rv(mi, EVC_ERR_OUT_OF_MEMORY);
    evc_mset(mi, 0, sizeof(EVCE_MODE));

    /* set function addresses */
    ctx->fn_mode_init_frame = mode_init_frame;
    ctx->fn_mode_init_lcu = mode_init_lcu;

    ctx->fn_mode_analyze_frame = mode_analyze_frame;
    ctx->fn_mode_analyze_lcu = mode_analyze_lcu;

    ctx->fn_mode_set_complexity = mode_set_complexity;

    return ctx->fn_mode_set_complexity(ctx, complexity);
}
