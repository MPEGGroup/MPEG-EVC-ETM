/* The copyright in this software is being made available under the BSD
*  License, included below. This software may be subject to other third party
*  and contributor rights, including patent rights, and no such rights are
*  granted under this license.
*  
*  Copyright (c) 2019, ISO/IEC
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

#include <math.h>
#include "evce_def.h"
#include "evce_eco.h"
#include "evc_df.h"
#include "evce_mode.h"
#include "evc_util.h"
#if DQP && RANDOM_DQP_GENERATION
#include <stdlib.h>
#endif
#if ALF
#include "enc_alf_wrapper.h"
#endif
#if APS_ALF_SEQ_FIX
int last_intra_poc = INT_MAX;
BOOL aps_counter_reset = FALSE;
#endif
#if IBC
#include "evce_ibc_hash_wrapper.h"
#endif

#if GRAB_STAT
#include "evc_debug.h"
#endif
/* Convert EVCE into EVCE_CTX */
#define EVCE_ID_TO_CTX_R(id, ctx) \
    evc_assert_r((id)); \
    (ctx) = (EVCE_CTX *)id; \
    evc_assert_r((ctx)->magic == EVCE_MAGIC_CODE);

/* Convert EVCE into EVCE_CTX with return value if assert on */
#define EVCE_ID_TO_CTX_RV(id, ctx, ret) \
    evc_assert_rv((id), (ret)); \
    (ctx) = (EVCE_CTX *)id; \
    evc_assert_rv((ctx)->magic == EVCE_MAGIC_CODE, (ret));

static const s8 tbl_poc_gop_offset[5][15] =
{
    { -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { -4,   -6,   -7,   -5,   -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { -8,   -12, -14,  -15,  -13,  -10,  -11,   -9,   -4,   -6,   -7,   -5,   -2,   -3,   -1}   /* gop_size = 16 */
};

static const s8 tbl_slice_depth_P_orig[GOP_P] = { FRM_DEPTH_3,  FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_1 };

static const s8 tbl_slice_depth_P[5][16] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, FRM_DEPTH_1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 4 */
    { FRM_DEPTH_3, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_1, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 8 */
    { FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_2, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_1,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 16 */
    { FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_2, \
      FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_1 }
};

static const s8 tbl_slice_depth[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 4 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 8 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 16 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_5, \
      FRM_DEPTH_5,  FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5 }
};

static const s8 tbl_slice_depth_orig[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 4 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 8 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 16 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, \
      FRM_DEPTH_3,  FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5 }
};

static EVCE_CTX * ctx_alloc(void)
{
    EVCE_CTX * ctx;

    ctx = (EVCE_CTX*)evc_malloc_fast(sizeof(EVCE_CTX));
    evc_assert_rv(ctx, NULL);
    evc_mset_x64a(ctx, 0, sizeof(EVCE_CTX));
    return ctx;
}

static void ctx_free(EVCE_CTX * ctx)
{
    evc_mfree_fast(ctx);
}

static EVCE_CORE * core_alloc(void)
{
    EVCE_CORE * core;
    int i, j;

    core = (EVCE_CORE *)evc_malloc_fast(sizeof(EVCE_CORE));

    evc_assert_rv(core, NULL);
    evc_mset_x64a(core, 0, sizeof(EVCE_CORE));

    for(i = 0; i < MAX_CU_DEPTH; i++)
    {
        for(j = 0; j < MAX_CU_DEPTH; j++)
        {
            evce_create_cu_data(&core->cu_data_best[i][j], i, j);
            evce_create_cu_data(&core->cu_data_temp[i][j], i, j);
        }
    }

    return core;
}

static void core_free(EVCE_CORE * core)
{
    int i, j;

    for(i = 0; i < MAX_CU_DEPTH; i++)
    {
        for(j = 0; j < MAX_CU_DEPTH; j++)
        {
            evce_delete_cu_data(&core->cu_data_best[i][j], i, j);
            evce_delete_cu_data(&core->cu_data_temp[i][j], i, j);
        }
    }

    evc_mfree_fast(core);
}

static int set_init_param(EVCE_CDSC * cdsc, EVCE_PARAM * param)
{
    /* check input parameters */
    evc_assert_rv(cdsc->w > 0 && cdsc->h > 0, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv((cdsc->w & (MIN_CU_SIZE-1)) == 0,EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv((cdsc->h & (MIN_CU_SIZE-1)) == 0,EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(cdsc->qp >= MIN_QUANT && cdsc->qp <= MAX_QUANT, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(cdsc->iperiod >= 0 ,EVC_ERR_INVALID_ARGUMENT);

    if(cdsc->disable_hgop == 0)
    {
        evc_assert_rv(cdsc->max_b_frames == 0 || cdsc->max_b_frames == 1 || \
                       cdsc->max_b_frames == 3 || cdsc->max_b_frames == 7 || \
                       cdsc->max_b_frames == 15, EVC_ERR_INVALID_ARGUMENT);

        if(cdsc->max_b_frames != 0)
        {
            if(cdsc->iperiod % (cdsc->max_b_frames + 1) != 0)
            {
                evc_assert_rv(0, EVC_ERR_INVALID_ARGUMENT);
            }
        }
    }

    if (cdsc->ref_pic_gap_length != 0)
    {
        evc_assert_rv(cdsc->max_b_frames == 0, EVC_ERR_INVALID_ARGUMENT);
    }


    if (cdsc->max_b_frames == 0)
    {
        if (cdsc->ref_pic_gap_length == 0)
        {
            cdsc->ref_pic_gap_length = 1;
        }
        evc_assert_rv(cdsc->ref_pic_gap_length == 1 || cdsc->ref_pic_gap_length == 2 || \
                      cdsc->ref_pic_gap_length == 4 || cdsc->ref_pic_gap_length == 8 || \
                      cdsc->ref_pic_gap_length == 16, EVC_ERR_INVALID_ARGUMENT);
    }

    /* set default encoding parameter */
    param->w              = cdsc->w;
    param->h              = cdsc->h;
    param->bit_depth      = cdsc->out_bit_depth;
    param->qp             = cdsc->qp;
    param->fps            = cdsc->fps;
    param->i_period       = cdsc->iperiod;
    param->f_ifrm         = 0;
    param->use_deblock    = 1;
#if M49023_DBF_IMPROVE
    param->deblock_alpha_offset = cdsc->deblock_aplha_offset;
    param->deblock_beta_offset = cdsc->deblock_beta_offset;
#endif
    param->qp_max         = MAX_QUANT;
    param->qp_min         = MIN_QUANT;
    param->use_pic_sign   = 0;
    param->max_b_frames   = cdsc->max_b_frames;
    param->ref_pic_gap_length = cdsc->ref_pic_gap_length;
    param->gop_size       = param->max_b_frames +1;
    param->use_closed_gop = (cdsc->closed_gop)? 1: 0;
#if IBC
    param->use_ibc_flag = (cdsc->ibc_flag) ? 1 : 0;
    param->ibc_search_range_x = cdsc->ibc_search_range_x;
    param->ibc_search_range_y = cdsc->ibc_search_range_y;
    param->ibc_hash_search_flag = cdsc->ibc_hash_search_flag;
    param->ibc_hash_search_max_cand = cdsc->ibc_hash_search_max_cand;
    param->ibc_hash_search_range_4smallblk = cdsc->ibc_hash_search_range_4smallblk;
    param->ibc_fast_method = cdsc->ibc_fast_method;
#endif
    param->use_hgop       = (cdsc->disable_hgop)? 0: 1;
#if USE_SLICE_DQP
    param->qp_incread_frame = cdsc->add_qp_frame;
#endif
#if DQP_CFG
    param->use_dqp = cdsc->use_dqp;
    param->cu_qp_delta_area = cdsc->cu_qp_delta_area;
#endif

    if(cdsc->tool_iqt == 0)
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_base;
    }
    else
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_main;
    }

    return EVC_OK;
}

static int set_enc_param(EVCE_CTX * ctx, EVCE_PARAM * param)
{
    int ret = EVC_OK;
    ctx->qp = (u8)param->qp;
    return ret;
}

static void set_nalu(EVCE_CTX * ctx, EVC_NALU * nalu, int nalu_type)
{
    nalu->nal_unit_size = 0;
    nalu->forbidden_zero_bit = 0;
    nalu->nal_unit_type_plus1 = nalu_type + 1;
    nalu->nuh_temporal_id = 0;
    nalu->nuh_temporal_id = 0;
    nalu->nuh_reserved_zero_5bits = 0;
    nalu->nuh_extension_flag = 0;
}

static void set_sps(EVCE_CTX * ctx, EVC_SPS * sps)
{
    sps->profile_idc = ctx->cdsc.profile;
    sps->level_idc = ctx->cdsc.level;
    sps->pic_width_in_luma_samples = ctx->param.w;
    sps->pic_height_in_luma_samples = ctx->param.h;
#if IBC
    sps->ibc_flag = (ctx->param.use_ibc_flag) ? 1 : 0;
    sps->ibc_log_max_size = IBC_MAX_CU_LOG2;
#endif
    if(ctx->param.max_b_frames > 0)
    {
        sps->max_num_ref_pics = MAX_NUM_ACTIVE_REF_FRAME_B;
    }
    else
    {
        sps->max_num_ref_pics = MAX_NUM_ACTIVE_REF_FRAME_LDB;
    }

    if(sps->profile_idc == PROFILE_MAIN)
    {
        sps->sps_btt_flag = ctx->cdsc.btt;
        sps->sps_suco_flag = ctx->cdsc.suco;
    }
    else
    {
        sps->sps_btt_flag = 0;
        sps->sps_suco_flag = 0;
    }

    if(sps->profile_idc == PROFILE_MAIN)
    {
        sps->log2_diff_ctu_max_11_cb_size = ctx->log2_max_cuwh - ctx->cdsc.framework_cu11_max;
        sps->log2_diff_max_11_min_11_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_cu11_min;
        sps->log2_diff_max_11_max_12_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_cu12_max;
        sps->log2_diff_min_11_min_12_cb_size_minus1 = ctx->cdsc.framework_cu12_min - ctx->cdsc.framework_cu11_min - 1;
        sps->log2_diff_max_12_max_14_cb_size_minus1 = ctx->cdsc.framework_cu12_max - ctx->cdsc.framework_cu14_max - 1;
        sps->log2_diff_min_12_min_14_cb_size_minus1 = ctx->cdsc.framework_cu14_min - ctx->cdsc.framework_cu12_min - 1;
        sps->log2_diff_max_11_max_tt_cb_size_minus1 = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_tris_max - 1;
        sps->log2_diff_min_11_min_tt_cb_size_minus2 = ctx->cdsc.framework_tris_min - ctx->cdsc.framework_cu11_min - 2;
        sps->log2_diff_ctu_size_max_suco_cb_size = ctx->log2_max_cuwh - ctx->cdsc.framework_suco_max;
        sps->log2_diff_max_suco_min_suco_cb_size = ctx->cdsc.framework_suco_max - ctx->cdsc.framework_suco_min;
    }

    sps->tool_amvr = ctx->cdsc.tool_amvr;
    sps->tool_mmvd = ctx->cdsc.tool_mmvd;
#if AFFINE
    sps->tool_affine = ctx->cdsc.tool_affine;
#endif
#if DMVR
    sps->tool_dmvr = ctx->cdsc.tool_dmvr;
#endif
#if ALF
    sps->tool_alf = ctx->cdsc.tool_alf;
#endif
#if HTDF
    sps->tool_htdf = ctx->cdsc.tool_htdf;
#endif
#if ADMVP
    sps->tool_admvp = ctx->cdsc.tool_admvp;
#endif
    sps->tool_amis = ctx->cdsc.tool_amis;
    sps->tool_eipd = ctx->cdsc.tool_eipd;
    sps->tool_iqt = ctx->cdsc.tool_iqt;
#if ADCC
    sps->tool_adcc = ctx->cdsc.tool_adcc;
#endif
    sps->tool_cm_init = ctx->cdsc.tool_cm_init;
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
    sps->tool_ats = ctx->cdsc.tool_ats;
#endif

    if(sps->profile_idc == PROFILE_MAIN)
    {
        sps->log2_ctu_size_minus2 = ctx->log2_max_cuwh - 2;
        sps->tool_rpl = 1;
        sps->tool_pocs = 1;
    }
    else
    {
        sps->tool_rpl = 0;
        sps->tool_pocs = 0;
        sps->log2_sub_gop_length = (int)(log2(ctx->param.gop_size) + .5);
        ctx->ref_pic_gap_length = ctx->param.ref_pic_gap_length;
        sps->log2_ref_pic_gap_length = (int)(log2(ctx->param.ref_pic_gap_length) + .5);
    }

    sps->long_term_ref_pics_flag = 0;

    if (!sps->tool_rpl)
    {
        sps->rpls_l0_num = 0;
        sps->rpls_l1_num = 0;
        sps->rpl_candidates_present_flag = 0;
        sps->rpl1_same_as_rpl0_flag = 0;
    }
    else
    {
        sps->rpls_l0_num = ctx->cdsc.rpls_l0_cfg_num;
        sps->rpls_l1_num = ctx->cdsc.rpls_l1_cfg_num;
        sps->rpl_candidates_present_flag = 1;
        sps->rpl1_same_as_rpl0_flag = 0;
    }

    memcpy(sps->rpls_l0, ctx->cdsc.rpls_l0, ctx->cdsc.rpls_l0_cfg_num * sizeof(sps->rpls_l0[0]));
    memcpy(sps->rpls_l1, ctx->cdsc.rpls_l1, ctx->cdsc.rpls_l1_cfg_num * sizeof(sps->rpls_l1[0]));

#if DQP
    sps->dquant_flag = ctx->cdsc.profile == 0 ? 0 : 1;                 /*Baseline : Active SPSs shall have sps_dquant_flag equal to 0 only*/
#endif
}

static void set_pps(EVCE_CTX * ctx, EVC_PPS * pps)
{
    pps->single_tile_in_pic_flag = 1;
    pps->constrained_intra_pred_flag = ctx->cdsc.constrained_intra_pred;
#if DQP
    pps->cu_qp_delta_enabled_flag = ctx->cdsc.use_dqp;
    pps->cu_qp_delta_area         = ctx->cdsc.cu_qp_delta_area;
#endif
}

#if ALF_PARAMETER_APS
static void set_aps(EVCE_CTX * ctx, EVC_APS * aps)
{
}
#endif

typedef struct _QP_ADAPT_PARAM
{
    int qp_offset_layer;
    double qp_offset_model_offset;
    double qp_offset_model_scale;
} QP_ADAPT_PARAM;

QP_ADAPT_PARAM qp_adapt_param_ra[8] = 
{
    {-3,  0.0000, 0.0000},
    { 1,  0.0000, 0.0000},
    { 1, -4.8848, 0.2061},
    { 4, -5.7476, 0.2286},
    { 5, -5.9000, 0.2333},
    { 6, -7.1444, 0.3000},
    { 7, -7.1444, 0.3000},
    { 8, -7.1444, 0.3000},
};

QP_ADAPT_PARAM qp_adapt_param_ld[8] =
{
    {-1,  0.0000, 0.0000},
    { 1,  0.0000, 0.0000},
    { 4, -6.5000, 0.2590},
    { 5, -6.5000, 0.2590},
    { 6, -6.5000, 0.2590},
    { 7, -6.5000, 0.2590},
    { 8, -6.5000, 0.2590},
    { 9, -6.5000, 0.2590},
};

  //Implementation for selecting and assigning RPL0 & RPL1 candidates in the SPS to SH
static void select_assign_rpl_for_sh(EVCE_CTX *ctx, EVC_SH *sh)
{
    //TBD: when NALU types are implemented; if the current picture is an IDR, simply return without doing the rest of the codes for this function

    /* introduce this variable for LD reason. The predefined RPL in the cfg file is made assuming GOP size is 4 for LD configuration*/
    int gopSize = (ctx->param.gop_size == 1) ? 4 : ctx->param.gop_size;

    //Assume it the pic is in the normal GOP first. Normal GOP here means it is not the first (few) GOP in the beginning of the bitstream
    sh->rpl_l0_idx = sh->rpl_l1_idx = -1;
    sh->ref_pic_list_sps_flag[0] = sh->ref_pic_list_sps_flag[1] = 0;

    int availableRPLs = (ctx->cdsc.rpls_l0_cfg_num < gopSize) ? ctx->cdsc.rpls_l0_cfg_num : gopSize;
    for (int i = 0; i < availableRPLs; i++)
    {
        int pocIdx = (sh->poc % gopSize == 0) ? gopSize : sh->poc % gopSize;
        if (pocIdx == ctx->cdsc.rpls_l0[i].poc)
        {
            sh->rpl_l0_idx = i;
            sh->rpl_l1_idx = sh->rpl_l0_idx;
            break;
        }
    }

    //For special case when the pic is in the first (few) GOP in the beginning of the bitstream.
    if(ctx->param.gop_size == 1)                          //For low delay configuration
    {
        if (sh->poc <= (ctx->cdsc.rpls_l0_cfg_num - gopSize))
        {
            sh->rpl_l0_idx = sh->poc + gopSize - 1;
            sh->rpl_l1_idx = sh->rpl_l0_idx;
        }
    }
    else                                                 //For random access configuration
    {
        //for (int i = ctx->param.gop_size; i < ctx->cdsc.rpls_l0_cfg_num; i++)
        for (int i = gopSize; i < ctx->cdsc.rpls_l0_cfg_num; i++)
        {
            int pocIdx = ctx->param.i_period == 0 ? sh->poc : (sh->poc % ctx->param.i_period == 0) ? ctx->param.i_period : sh->poc % ctx->param.i_period;
            if (pocIdx == ctx->cdsc.rpls_l0[i].poc)
            {
                sh->rpl_l0_idx = i;
                sh->rpl_l1_idx = i;
                break;
            }
        }
    }
    if (ctx->slice_type != SLICE_I)
    {
        ctx->slice_type = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].pic_type == 'P' ? SLICE_P : SLICE_B;
    }
    //Copy RPL0 from the candidate in SPS to this SH
    sh->rpl_l0.poc = sh->poc;
    sh->rpl_l0.tid = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].tid;
    sh->rpl_l0.ref_pic_num = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].ref_pic_num;
    sh->rpl_l0.ref_pic_active_num = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].ref_pic_active_num;
    for (int i = 0; i < sh->rpl_l0.ref_pic_num; i++)
        sh->rpl_l0.ref_pics[i] = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].ref_pics[i];

    //Copy RPL0 from the candidate in SPS to this SH
    sh->rpl_l1.poc = sh->poc;
    sh->rpl_l1.tid = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].tid;
    sh->rpl_l1.ref_pic_num = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].ref_pic_num;
    sh->rpl_l1.ref_pic_active_num = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].ref_pic_active_num;
    for (int i = 0; i < sh->rpl_l1.ref_pic_num; i++)
        sh->rpl_l1.ref_pics[i] = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].ref_pics[i];

    if (sh->rpl_l0_idx != -1)
    {
        sh->ref_pic_list_sps_flag[0] = 1;
    }

    if (sh->rpl_l1_idx != -1)
    {
        sh->ref_pic_list_sps_flag[1] = 1;
    }
}

//Return value 0 means all ref pic listed in the given rpl are available in the DPB
//Return value 1 means there is at least one ref pic listed in the given rpl not available in the DPB
static int check_refpic_available(int currentPOC, EVC_PM *pm, EVC_RPL *rpl)
{
    for (int i = 0; i < rpl->ref_pic_num; i++)
    {
        int isExistInDPB = 0;
        for (int j = 0; !isExistInDPB && j < MAX_PB_SIZE; j++)
        {
            if (pm->pic[j] && pm->pic[j]->is_ref && pm->pic[j]->ptr == (currentPOC - rpl->ref_pics[i]))
                isExistInDPB = 1;
        }
        if (!isExistInDPB) //Found one ref pic missing return 1
            return 1;
    }
    return 0; 
}

//Return value 0 means no explicit RPL is created. The given input parameters rpl0 and rpl1 are not modified
//Return value 1 means the given input parameters rpl0 and rpl1 are modified
static int create_explicit_rpl(EVC_PM *pm, EVC_SH *sh)

{
    int currentPOC = sh->poc;
    EVC_RPL *rpl0 = &sh->rpl_l0;
    EVC_RPL *rpl1 = &sh->rpl_l1;
    if (!check_refpic_available(currentPOC, pm, rpl0) && !check_refpic_available(currentPOC, pm, rpl1))
    {
        return 0;
    }
    
    EVC_PIC * pic = NULL;

    int isRPLChanged = 0;
    //Remove ref pic in RPL0 that is not available in the DPB
    for (int ii = 0; ii < rpl0->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl0->ref_pics[ii]))
                isAvailable = 1;
            pic = NULL;
        }
        if (!isAvailable)
        {
            for (int jj = ii; jj < rpl0->ref_pic_num - 1; jj++)
                rpl0->ref_pics[jj] = rpl0->ref_pics[jj + 1];
            ii--;
            rpl0->ref_pic_num--;
            isRPLChanged = 1;
        }
    }
    if (isRPLChanged)
        sh->rpl_l0_idx = -1;

    //Remove ref pic in RPL1 that is not available in the DPB
    isRPLChanged = 0;
    for (int ii = 0; ii < rpl1->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl1->ref_pics[ii]))
                isAvailable = 1;
            pic = NULL;
        }
        if (!isAvailable)
        {
            for (int jj = ii; jj < rpl1->ref_pic_num - 1; jj++)
                rpl1->ref_pics[jj] = rpl1->ref_pics[jj + 1];
            ii--;
            rpl1->ref_pic_num--;
            isRPLChanged = 1;
        }
    }
    if (isRPLChanged)
        sh->rpl_l1_idx = -1;

    /*if number of ref pic in RPL0 is less than its number of active ref pic, try to copy from RPL1*/
    if (rpl0->ref_pic_num < rpl0->ref_pic_active_num)
    {
        for (int ii = rpl0->ref_pic_num; ii < rpl0->ref_pic_active_num; ii++)
        {
            //First we need to find ref pic in RPL1 that is not already in RPL0
            int isAlreadyIncluded = 1;
            int idx = -1;
            int status = 0;
            do {
                status = 0;
                idx++;
                for (int mm = 0; mm < rpl0->ref_pic_num && idx < rpl1->ref_pic_num; mm++)
                {
                    if (rpl1->ref_pics[idx] == rpl0->ref_pics[mm])
                        status = 1;
                }
                if (!status) isAlreadyIncluded = 0;
            } while (isAlreadyIncluded && idx < rpl1->ref_pic_num);

            if (idx < rpl1->ref_pic_num)
            {
                rpl0->ref_pics[ii] = rpl1->ref_pics[idx];
                rpl0->ref_pic_num++;
            }
        }
        if (rpl0->ref_pic_num < rpl0->ref_pic_active_num) rpl0->ref_pic_active_num = rpl0->ref_pic_num;
    }

    /*same logic as above, just apply to RPL1*/
    if (rpl1->ref_pic_num < rpl1->ref_pic_active_num)
    {
        for (int ii = rpl1->ref_pic_num; ii < rpl1->ref_pic_active_num; ii++)
        {
            int isAlreadyIncluded = 1;
            int idx = -1;
            int status = 0;
            do {
                status = 0;
                idx++;
                for (int mm = 0; mm < rpl1->ref_pic_num && idx < rpl0->ref_pic_num; mm++)
                {
                    if (rpl0->ref_pics[idx] == rpl1->ref_pics[mm])
                        status = 1;
                }
                if (!status) isAlreadyIncluded = 0;
            } while (isAlreadyIncluded && idx < rpl0->ref_pic_num);

            if (idx < rpl0->ref_pic_num)
            {
                rpl1->ref_pics[ii] = rpl0->ref_pics[idx];
                rpl1->ref_pic_num++;
            }
        }
        if (rpl1->ref_pic_num < rpl1->ref_pic_active_num) rpl1->ref_pic_active_num = rpl1->ref_pic_num;
    }
    return 1;
}

static void set_sh(EVCE_CTX *ctx, EVC_SH *sh)
{
    double qp;
    int qp_l_i;
    int qp_c_i;
    QP_ADAPT_PARAM *qp_adapt_param = ctx->param.max_b_frames == 0 ? qp_adapt_param_ld : qp_adapt_param_ra;

    if (ctx->sps.tool_rpl)
    {
        sh->poc = ctx->ptr;
        select_assign_rpl_for_sh(ctx, sh);
        sh->num_ref_idx_active_override_flag = 1;
    }

    sh->dtr = ctx->dtr & DTR_BIT_MSK;
    sh->slice_type = ctx->slice_type;
    sh->deblocking_filter_on = (ctx->param.use_deblock) ? 1 : 0;
#if M49023_DBF_IMPROVE
    sh->sh_deblock_alpha_offset = ctx->param.deblock_alpha_offset;
    sh->sh_deblock_beta_offset = ctx->param.deblock_beta_offset;
#endif
    sh->dptr = ctx->ptr - ctx->dtr;
    sh->layer_id = ctx->layer_id;
    sh->single_tile_in_slice_flag = 1;
#if M49023_ADMVP_IMPROVE
    sh->collocated_from_list_idx = (sh->slice_type == SLICE_P) ? REFP_0 : REFP_1;  // Specifies source (List ID) of the collocated picture, equialent of the collocated_from_l0_flag
    sh->collocated_from_ref_idx = 0;        // Specifies source (RefID_ of the collocated picture, equialent of the collocated_ref_idx
    sh->collocated_mvp_source_list_idx = REFP_0;  // Specifies source (List ID) in collocated pic that provides MV information (Applicability is function of NoBackwardPredFlag)
#endif 

    /* set lambda */
#if USE_SLICE_DQP
    qp = EVC_CLIP3(0, MAX_QUANT, (ctx->param.qp_incread_frame != 0 && (int)(ctx->ptr) >= ctx->param.qp_incread_frame) ? ctx->qp + 1.0 : ctx->qp);
#else
    qp = ctx->qp;
#endif

    if(ctx->param.use_hgop)
    {
        double dqp_offset;
        int qp_offset;

        if (ctx->sps.tool_rpl)
        {
            qp += qp_adapt_param[ctx->layer_id].qp_offset_layer;
            dqp_offset = qp * qp_adapt_param[ctx->layer_id].qp_offset_model_scale + qp_adapt_param[ctx->layer_id].qp_offset_model_offset + 0.5;
        }
        else
        {
            qp += qp_adapt_param[ctx->slice_depth].qp_offset_layer;
            dqp_offset = qp * qp_adapt_param[ctx->slice_depth].qp_offset_model_scale + qp_adapt_param[ctx->slice_depth].qp_offset_model_offset + 0.5;
        }

        qp_offset = (int)floor(EVC_CLIP3(0.0, 3.0, dqp_offset));
        qp += qp_offset;
    }

    sh->qp = (u8)EVC_CLIP3(-(6 * (BIT_DEPTH - 8)), MAX_QUANT, qp);
    sh->qp_u = (u8)(sh->qp + ctx->cdsc.cb_qp_offset); 
    sh->qp_v = (u8)(sh->qp + ctx->cdsc.cr_qp_offset);
#if M49023_DBF_IMPROVE
    sh->sh_deblock_alpha_offset = ctx->cdsc.deblock_aplha_offset;
    sh->sh_deblock_beta_offset = ctx->cdsc.deblock_beta_offset;
#endif

    qp_l_i = sh->qp;
    ctx->lambda[0] = 0.57 * pow(2.0, (qp_l_i - 12.0) / 3.0);
    qp_c_i = evc_tbl_qp_chroma_ajudst[sh->qp_u];
    ctx->dist_chroma_weight[0] = pow(2.0, (qp_l_i - qp_c_i) / 3.0);
    qp_c_i = evc_tbl_qp_chroma_ajudst[sh->qp_v];
    ctx->dist_chroma_weight[1] = pow(2.0, (qp_l_i - qp_c_i) / 3.0);
    ctx->lambda[1] = ctx->lambda[0] / ctx->dist_chroma_weight[0];
    ctx->lambda[2] = ctx->lambda[0] / ctx->dist_chroma_weight[1];
    ctx->sqrt_lambda[0] = sqrt(ctx->lambda[0]);
    ctx->sqrt_lambda[1] = sqrt(ctx->lambda[1]);
    ctx->sqrt_lambda[2] = sqrt(ctx->lambda[2]);
}
static int evce_eco_tree(EVCE_CTX * ctx, EVCE_CORE * core, int x0, int y0, int cup, int cuw, int cuh, int cud, int next_split, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth
#if DQP
    , int cu_qp_delta_code
#endif
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int ret;
    EVC_BSW * bs;
    s8  split_mode;
    s8  suco_flag = 0;
    int bound;
    int split_mode_child[4] = {NO_SPLIT, NO_SPLIT, NO_SPLIT, NO_SPLIT};
    int split_allow[6];
#if M50761_CHROMA_NOT_SPLIT
    ctx->tree_cons = tree_cons;
#endif

    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[core->lcu_num].split_mode);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[core->lcu_num].suco_flag);

    same_layer_split[node_idx] = split_mode;

    bs = &ctx->bs;

#if DQP
    if(ctx->sps.sps_btt_flag)
    {
        if(ctx->pps.cu_qp_delta_enabled_flag && ctx->sps.dquant_flag)
        {
            if (split_mode == NO_SPLIT && (CONV_LOG2(cuw) + CONV_LOG2(cuh) >= ctx->pps.cu_qp_delta_area) && cu_qp_delta_code != 2)
            {
                if (CONV_LOG2(cuw) == 7 || CONV_LOG2(cuh) == 7)
                {
                    cu_qp_delta_code = 2;
                }
                else
                {
                    cu_qp_delta_code = 1;
                }
                core->cu_qp_delta_is_coded = 0;
            }
            else if ((((CONV_LOG2(cuw) + CONV_LOG2(cuh) == ctx->pps.cu_qp_delta_area + 1) && (split_mode == SPLIT_TRI_VER || split_mode == SPLIT_TRI_HOR)) ||
                (CONV_LOG2(cuh) + CONV_LOG2(cuw) == ctx->pps.cu_qp_delta_area && cu_qp_delta_code != 2)))
            {
                cu_qp_delta_code = 2;
                core->cu_qp_delta_is_coded = 0;
            }
        }
    }
#endif

    if(split_mode != NO_SPLIT)
    {
        if(!ctx->sps.sps_btt_flag || ((x0 + cuw <= ctx->w) && (y0 + cuh <= ctx->h)))
        {
            evce_eco_split_mode(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh
                                , parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
        }

        bound = !((x0 + cuw <= ctx->w) && (y0 + cuh <= ctx->h));
        evce_eco_suco_flag(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh, split_mode, bound, ctx->log2_max_cuwh);
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x0, y0, cuw, cuh, cup, cud, ctx->log2_culine, &split_struct
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
        evc_split_get_suco_order(suco_flag, split_mode, suco_order);
#if M50761_CHROMA_NOT_SPLIT
        BOOL mode_cons_changed = evc_signal_mode_cons(&ctx->tree_cons, &split_struct.tree_cons)
#if CHROMA_NOT_SPLIT_EXCLUDE_IBC
            && !ctx->sps.ibc_flag
#endif
            ;
        BOOL mode_cons_signal = mode_cons_changed && (ctx->sh.slice_type != SLICE_I) && (evc_get_mode_cons_by_split(split_mode, cuw, cuh) == eAll);
        if (mode_cons_changed)
        {
            MODE_CONS mode = evce_derive_mode_cons(ctx, core->lcu_num, cup);
            evc_set_tree_mode(&split_struct.tree_cons, mode);
        }

        if (split_mode != SPLIT_QUAD )       // Only for main profile
        {
            if (mode_cons_signal)
            {
                evc_get_ctx_some_flags(PEL2SCU(x0), PEL2SCU(y0), cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
#if IBC
                    , ctx->param.use_ibc_flag, ctx->sps.ibc_log_max_size
#endif
                );

                evce_eco_mode_constr(bs, split_struct.tree_cons.mode_cons, ctx->ctx_flags[CNID_MODE_CONS]);
            }
        }
        else
        {
            split_struct.tree_cons = evc_get_default_tree_cons();
        }
#endif


        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                ret = evce_eco_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], sub_cuw, sub_cuh, split_struct.cud[cur_part_num], 1, split_mode, split_mode_child, part_num, split_allow, INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, bound)
#if DQP
                    , cu_qp_delta_code
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , split_struct.tree_cons
#endif
                );
                evc_assert_g(EVC_SUCCEEDED(ret), ERR);
            }
#if M50761_CHROMA_NOT_SPLIT
            ctx->tree_cons = tree_cons;
#endif
        }
#if M50761_CHROMA_NOT_SPLIT
        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
        {
            evc_assert(x0 + cuw <= PIC_ORIG(ctx)->w_l && y0 + cuh <= PIC_ORIG(ctx)->h_l);
            TREE_CONS local_tree_cons = split_struct.tree_cons;
            local_tree_cons.tree_type = TREE_C;
            ret = evce_eco_unit(ctx, core, x0, y0, cup, cuw, cuh, local_tree_cons);
            ctx->tree_cons = tree_cons;
        }
#endif
    }
    else
    {
        evc_assert(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);

        if((cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE) && next_split
#if M50761_CHROMA_NOT_SPLIT
            && evce_check_luma(ctx)
#endif
            )
        {
            evce_eco_split_mode(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh
                                , parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
        }

#if DQP
        core->cu_qp_delta_code = cu_qp_delta_code;
#endif
        ret = evce_eco_unit(ctx, core, x0, y0, cup, cuw, cuh
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
        evc_assert_g(EVC_SUCCEEDED(ret), ERR);
    }

    return EVC_OK;
ERR:
    return ret;
}

int evce_ready(EVCE_CTX * ctx)
{
    EVCE_CORE * core = NULL;
    int          w, h, ret, i;
    s64          size;
#if ATS_INTER_PROCESS
    ctx->map_ats_inter = NULL;
    ctx->ats_inter_info_pred = NULL;
    ctx->ats_inter_pred_dist = NULL;
    ctx->ats_inter_num_pred = NULL;
#endif

    evc_assert(ctx);

    core = core_alloc();
    evc_assert_gv(core != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

    /* set various value */
    ctx->core = core;
    w = ctx->w = ctx->param.w;
    h = ctx->h = ctx->param.h;
    ctx->f = w *h;

    evce_init_bits_est();

    if (ctx->cdsc.btt)
    {
        ctx->max_cuwh = 1 << ctx->cdsc.framework_ctu_size;
        if (w < ctx->max_cuwh * 2 && h < ctx->max_cuwh * 2)
        {
            ctx->max_cuwh = ctx->max_cuwh >> 1;
        }
        else
        {
            ctx->max_cuwh = ctx->max_cuwh;
        }
    }
    else
    {
        ctx->max_cuwh = 64; 
    }

    ctx->log2_max_cuwh = CONV_LOG2(ctx->max_cuwh);
    ctx->max_cud = ctx->log2_max_cuwh - MIN_CU_LOG2;
    ctx->w_lcu = (w + ctx->max_cuwh - 1) >> ctx->log2_max_cuwh;
    ctx->h_lcu = (h + ctx->max_cuwh - 1) >> ctx->log2_max_cuwh;
    ctx->f_lcu = ctx->w_lcu * ctx->h_lcu;
    ctx->w_scu = (w + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->h_scu = (h + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->f_scu = ctx->w_scu * ctx->h_scu;
    ctx->log2_culine = ctx->log2_max_cuwh - MIN_CU_LOG2;
    ctx->log2_cudim = ctx->log2_culine << 1;

    ctx->cdsc.framework_cu11_max = min(ctx->log2_max_cuwh, ctx->cdsc.framework_cu11_max);
    ctx->cdsc.framework_cu12_max = min(ctx->cdsc.framework_cu11_max, ctx->cdsc.framework_cu12_max);
    ctx->cdsc.framework_cu14_max = min(ctx->cdsc.framework_cu12_max, ctx->cdsc.framework_cu14_max);
    ctx->cdsc.framework_suco_max = min(ctx->log2_max_cuwh, ctx->cdsc.framework_suco_max);
#if ALF
    ctx->enc_alf = new_enc_ALF();
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);
    call_create_enc_ALF(p, ctx->w, ctx->h, ctx->max_cuwh, ctx->max_cuwh, 5);
#endif
#if IBC
    if (ctx->param.use_ibc_flag)
    {
      ctx->ibc_hash_handle = create_enc_IBC(ctx->w, ctx->h);
    }
#endif
    /*  allocate CU data map*/
    if(ctx->map_cu_data == NULL)
    {
        size = sizeof(EVCE_CU_DATA) * ctx->f_lcu;
        ctx->map_cu_data = (EVCE_CU_DATA*)evc_malloc_fast(size);
        evc_assert_gv(ctx->map_cu_data, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_cu_data, 0, size);

        for(i = 0; i < (int)ctx->f_lcu; i++)
        {
            evce_create_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
        }
    }

    /* allocate maps */
    if(ctx->map_scu == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_scu = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_scu, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_scu, 0, size);
    }

#if DQP
    if (ctx->map_input_dqp == NULL)
    {
        size = sizeof(s8) * ctx->f_scu;
        ctx->map_input_dqp = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_input_dqp, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_input_dqp, 0, size);
    }
#endif

    if(ctx->map_ipm == NULL)
    {
        size = sizeof(s8) * ctx->f_scu;
        ctx->map_ipm = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ipm, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ipm, -1, size);
    }
#if !M50761_REMOVE_BLOCK_SIZE_MAP
    size = sizeof(s16) * ctx->f_scu * 2;
    ctx->map_block_size = evc_malloc_fast(size);
    evc_assert_gv(ctx->map_block_size, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset(ctx->map_block_size, -1, size);
#endif
    size = sizeof(s8) * ctx->f_scu;
    ctx->map_depth = evc_malloc_fast(size);
    evc_assert_gv(ctx->map_depth, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset(ctx->map_depth, -1, size);

#if AFFINE
    if (ctx->map_affine == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_affine = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_affine, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_affine, 0, size);
    }
#endif

    if(ctx->map_cu_mode == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_cu_mode = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_cu_mode, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_cu_mode, 0, size);
    }

#if ATS_INTRA_PROCESS
    if (ctx->map_ats_intra_cu == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_intra_cu = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_intra_cu, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_intra_cu, 0, size);
    }
    if (ctx->map_ats_tu_h == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_tu_h = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_tu_h, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_tu_h, 0, size);
    }
    if (ctx->map_ats_tu_v == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_tu_v = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_tu_v, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_tu_v, 0, size);
    }
#endif
#if ATS_INTER_PROCESS
    if (ctx->map_ats_inter == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_inter = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_inter, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_inter, -1, size);
    }
    if (ctx->ats_inter_info_pred == NULL)
    {
        int num_route = ATS_INTER_SL_NUM;
        int num_size_idx = MAX_TR_LOG2 - MIN_CU_LOG2 + 1;
        size = sizeof(u32) * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2) * num_route; //only correct when the largest cu is <=128
        ctx->ats_inter_pred_dist = evc_malloc_fast(size);
        evc_assert_gv(ctx->ats_inter_pred_dist, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

        size = sizeof(u8)  * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2) * num_route;
        ctx->ats_inter_info_pred = evc_malloc_fast(size);
        evc_assert_gv(ctx->ats_inter_info_pred, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

        size = sizeof(u8)  * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2);
        ctx->ats_inter_num_pred = evc_malloc_fast(size);
        evc_assert_gv(ctx->ats_inter_num_pred, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    }
#endif
#if DQP
    if (ctx->map_dqp_used == NULL)
    {
        /* max cu size - dqp_unit_size - scu_unit_size*/
        size = sizeof(s8) * ctx->f_scu;
        ctx->map_dqp_used = (u8 *)evc_malloc(size);
        evc_assert_gv(ctx->map_dqp_used, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_dqp_used, DQP_UNUSED, size);
    }
#endif

    /* initialize reference picture manager */
    ctx->pa.fn_alloc = evce_pic_alloc;
    ctx->pa.fn_free  = evce_pic_free;
    ctx->pa.w        = ctx->w;
    ctx->pa.h        = ctx->h;
    ctx->pa.pad_l    = PIC_PAD_SIZE_L;
    ctx->pa.pad_c    = PIC_PAD_SIZE_C;
    ctx->pic_cnt     = 0;
    ctx->pic_icnt    = -1;
    ctx->dtr         = 0;
    ctx->ptr         = 0;

    ret = evc_picman_init(&ctx->rpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, &ctx->pa);
    evc_assert_g(EVC_SUCCEEDED(ret), ERR);

    ctx->pico_max_cnt = 1 + (ctx->param.max_b_frames << 1) ;
    ctx->frm_rnum = ctx->param.max_b_frames;
    ctx->qp = ctx->param.qp;
#if M49023_DBF_IMPROVE
    ctx->deblock_alpha_offset = ctx->param.deblock_alpha_offset;
    ctx->deblock_beta_offset = ctx->param.deblock_beta_offset;
#endif
    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        ctx->pico_buf[i] = (EVCE_PICO*)evc_malloc(sizeof(EVCE_PICO));
        evc_assert_gv(ctx->pico_buf[i], ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->pico_buf[i], 0, sizeof(EVCE_PICO));
    }

    return EVC_OK;
ERR:
    for (i = 0; i < (int)ctx->f_lcu; i++)
    {
        evce_delete_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
    }
    evc_mfree_fast(ctx->map_cu_data);
    evc_mfree_fast(ctx->map_ipm);
#if !M50761_REMOVE_BLOCK_SIZE_MAP
    evc_mfree_fast(ctx->map_block_size);
#endif
    evc_mfree_fast(ctx->map_depth);
#if AFFINE
    evc_mfree_fast(ctx->map_affine);
#endif
#if ATS_INTRA_PROCESS
    evc_mfree_fast(ctx->map_ats_intra_cu);
    evc_mfree_fast(ctx->map_ats_tu_h);
    evc_mfree_fast(ctx->map_ats_tu_v);
#endif
#if ATS_INTER_PROCESS
    evc_mfree_fast(ctx->map_ats_inter);
    evc_mfree_fast(ctx->ats_inter_pred_dist);
    evc_mfree_fast(ctx->ats_inter_info_pred);
    evc_mfree_fast(ctx->ats_inter_num_pred);
#endif
    evc_mfree_fast(ctx->map_cu_mode);
#if DQP
    evc_mfree_fast(ctx->map_dqp_used);
    evc_mfree_fast(ctx->map_input_dqp);
#endif
    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        evc_mfree_fast(ctx->pico_buf[i]);
    }

    if(core)
    {
        core_free(core);
    }
    return ret;
}

void evce_flush(EVCE_CTX * ctx)
{
    int i;
    evc_assert(ctx);

    evc_mfree_fast(ctx->map_scu);
    for(i = 0; i < (int)ctx->f_lcu; i++)
    {
        evce_delete_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
    }
    evc_mfree_fast(ctx->map_cu_data);
    evc_mfree_fast(ctx->map_ipm);
#if !M50761_REMOVE_BLOCK_SIZE_MAP
    evc_mfree_fast(ctx->map_block_size);
#endif
    evc_mfree_fast(ctx->map_depth);
#if AFFINE
    evc_mfree_fast(ctx->map_affine);
#endif
#if ATS_INTRA_PROCESS
    evc_mfree_fast(ctx->map_ats_intra_cu);
    evc_mfree_fast(ctx->map_ats_tu_h);
    evc_mfree_fast(ctx->map_ats_tu_v);
#endif
#if ATS_INTER_PROCESS
    evc_mfree_fast(ctx->map_ats_inter);
    evc_mfree_fast(ctx->ats_inter_pred_dist);
    evc_mfree_fast(ctx->ats_inter_info_pred);
    evc_mfree_fast(ctx->ats_inter_num_pred);
#endif
    evc_mfree_fast(ctx->map_cu_mode);
#if DQP
    evc_mfree_fast(ctx->map_input_dqp);
    evc_mfree_fast(ctx->map_dqp_used);
#endif
#if RDO_DBK
    evc_picbuf_free(ctx->pic_dbk);
#endif
    evc_picman_deinit(&ctx->rpm);
    core_free(ctx->core);
    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        evc_mfree_fast(ctx->pico_buf[i]);
    }
    for(i = 0; i < EVCE_MAX_INBUF_CNT; i++)
    {
        if(ctx->inbuf[i]) ctx->inbuf[i]->release(ctx->inbuf[i]);
    }
}

static void deblock_tree(EVCE_CTX * ctx, EVC_PIC * pic, int x, int y, int cuw, int cuh, int cud, int cup, int is_hor
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    s8  split_mode;
    int lcu_num;
    s8  suco_flag = 0;
#if M50761_CHROMA_NOT_SPLIT
    ctx->tree_cons = tree_cons;
#endif

#if M49023_DBF_IMPROVE
    pic->pic_deblock_alpha_offset = ctx->sh.sh_deblock_alpha_offset;
    pic->pic_deblock_beta_offset = ctx->sh.sh_deblock_beta_offset;
#endif
    lcu_num = (x >> ctx->log2_max_cuwh) + (y >> ctx->log2_max_cuwh) * ctx->w_lcu;
    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[lcu_num].split_mode);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[lcu_num].suco_flag);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x, y, cuw, cuh, cup, cud, ctx->log2_culine, &split_struct
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
        evc_split_get_suco_order(suco_flag, split_mode, suco_order);
#if M50761_CHROMA_NOT_SPLIT
        BOOL mode_cons_changed = evc_signal_mode_cons(&ctx->tree_cons, &split_struct.tree_cons)
#if CHROMA_NOT_SPLIT_EXCLUDE_IBC
            && !ctx->sps.ibc_flag
#endif
            ;
        if (split_mode != SPLIT_QUAD )       // Only for main profile
        {
            if (mode_cons_changed)
            {
                MODE_CONS mode = evce_derive_mode_cons(ctx, lcu_num, cup);
                evc_set_tree_mode(&split_struct.tree_cons, mode);
            }
        }
        else
        {
            // In base profile we have small chroma blocks
            split_struct.tree_cons = evc_get_default_tree_cons();
            mode_cons_changed = FALSE;
        }
#endif
        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                deblock_tree(ctx, pic, x_pos, y_pos, sub_cuw, sub_cuh, split_struct.cud[cur_part_num], split_struct.cup[cur_part_num], is_hor
#if M50761_CHROMA_NOT_SPLIT
                    , split_struct.tree_cons
#endif
                );
            }
#if M50761_CHROMA_NOT_SPLIT
            ctx->tree_cons = tree_cons;
#endif
        }
#if M50761_CHROMA_NOT_SPLIT
        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
        {
            ctx->tree_cons = split_struct.tree_cons;
            ctx->tree_cons.tree_type = TREE_C;
            split_mode = NO_SPLIT;
        }
#endif
    }
#if M50761_CHROMA_NOT_SPLIT
    if (split_mode == NO_SPLIT)
#else
    else
#endif
    {
#if ATS_INTER_PROCESS // deblock
        int t = (x >> MIN_CU_LOG2) + (y >> MIN_CU_LOG2) * ctx->w_scu;
        u8 ats_inter_info = ctx->map_ats_inter[t];
        u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
        u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
#endif
        if(is_hor)
        {
            if (cuh > MAX_TR_SIZE)
            {
              
                evc_deblock_cu_hor(pic, x, y              , cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, 
#if M50761_DMVR_SIMP_DEBLOCK
                  ctx->map_unrefined_mv
#else
                  ctx->map_mv
#endif
                  , ctx->w_scu, ctx->log2_max_cuwh, ctx->refp
#if M49023_DBF_IMPROVE
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , ctx->tree_cons
#endif
                );
                evc_deblock_cu_hor(pic, x, y + MAX_TR_SIZE, cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, 
#if M50761_DMVR_SIMP_DEBLOCK
                  ctx->map_unrefined_mv
#else
                  ctx->map_mv
#endif
                  , ctx->w_scu, ctx->log2_max_cuwh, ctx->refp
#if M49023_DBF_IMPROVE
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , ctx->tree_cons
#endif
                );
            }
            else
            {
                evc_deblock_cu_hor(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, 
#if M50761_DMVR_SIMP_DEBLOCK
                  ctx->map_unrefined_mv
#else
                  ctx->map_mv
#endif
                  , ctx->w_scu, ctx->log2_max_cuwh, ctx->refp
#if M49023_DBF_IMPROVE
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , ctx->tree_cons
#endif
                );
            }
        }
        else
        {
            if (cuw > MAX_TR_SIZE)
            {
                evc_deblock_cu_ver(pic, x              , y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, 
#if M50761_DMVR_SIMP_DEBLOCK
                  ctx->map_unrefined_mv
#else
                  ctx->map_mv
#endif
                  , ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                    , ctx->map_cu_mode
#endif
                    , ctx->refp
#if M49023_DBF_IMPROVE
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , ctx->tree_cons
#endif
                );
                evc_deblock_cu_ver(pic, x + MAX_TR_SIZE, y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi,
#if M50761_DMVR_SIMP_DEBLOCK
                  ctx->map_unrefined_mv
#else
                  ctx->map_mv
#endif
                  , ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                    , ctx->map_cu_mode
#endif                            
                    , ctx->refp
#if M49023_DBF_IMPROVE
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , ctx->tree_cons
#endif
                );
            }
            else
            {
                evc_deblock_cu_ver(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, 
#if M50761_DMVR_SIMP_DEBLOCK
                  ctx->map_unrefined_mv
#else
                  ctx->map_mv
#endif
                  , ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                                   , ctx->map_cu_mode
#endif
                                   , ctx->refp
#if M49023_DBF_IMPROVE
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , ctx->tree_cons
#endif
                );
            }
        }
    }
#if M50761_CHROMA_NOT_SPLIT
    ctx->tree_cons = tree_cons;
#endif
}

int evce_deblock_h263(EVCE_CTX * ctx, EVC_PIC * pic)
{
    int i, j;
    u32 k;

    for(k = 0; k < ctx->f_scu; k++)
    {
        MCU_CLR_COD(ctx->map_scu[k]);
#if M50761_DMVR_SIMP_DEBLOCK
        if (!MCU_GET_DMVRF(ctx->map_scu[k])) {
          ctx->map_unrefined_mv[k][REFP_0][MV_X] = ctx->map_mv[k][REFP_0][MV_X];
          ctx->map_unrefined_mv[k][REFP_0][MV_Y] = ctx->map_mv[k][REFP_0][MV_Y];
          ctx->map_unrefined_mv[k][REFP_1][MV_X] = ctx->map_mv[k][REFP_1][MV_X];
          ctx->map_unrefined_mv[k][REFP_1][MV_Y] = ctx->map_mv[k][REFP_1][MV_Y];
        }
#endif
    }

    /* horizontal filtering */
    for(j = 0; j < ctx->h_lcu; j++)
    {
        for(i = 0; i < ctx->w_lcu; i++)
        {
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
            );
        }
    }

    for(k = 0; k < ctx->f_scu; k++)
    {
        MCU_CLR_COD(ctx->map_scu[k]);
    }

    /* vertical filtering */
    for(j = 0; j < ctx->h_lcu; j++)
    {
        for(i = 0; i < ctx->w_lcu; i++)
        {
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
            );
        }
    }

    return EVC_OK;
}
#if ALF
#if ALF_PARAMETER_APS
int evce_alf_aps(EVCE_CTX * ctx, EVC_PIC * pic, EVC_SH* sh, EVC_APS* aps)
{
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);

    double lambdas[3];
    for (int i = 0; i < 3; i++)
        lambdas[i] = (ctx->lambda[i]) * ALF_LAMBDA_SCALE; //this is for appr match of different lambda sets


    set_resetALFBufferFlag(p, sh->slice_type == SLICE_I ? 1 : 0);
    alf_aps_enc_opt_process(p, lambdas, ctx, pic, &(sh->alf_sh_param));

    aps->alf_aps_param = sh->alf_sh_param;
    if (sh->alf_sh_param.resetALFBufferFlag) // reset aps index counter (buffer) if ALF flag reset is present
    {
        ctx->aps_counter = -1;
    }
    sh->alf_on = sh->alf_sh_param.enabledFlag[0];
#if APS_ALF_CTU_FLAG
    if (sh->alf_on == 0)
    {
        sh->alf_sh_param.isCtbAlfOn = 0;
    }
#endif
    if (sh->alf_on)
    {
        if (aps->alf_aps_param.temporalAlfFlag)
        {
            aps->aps_id = sh->alf_sh_param.prevIdx;
#if M50662_LUMA_CHROMA_SEPARATE_APS
            sh->aps_id_y = sh->alf_sh_param.prevIdxComp[0];
            sh->aps_id_ch = sh->alf_sh_param.prevIdxComp[1];
#endif
            sh->aps_signaled = aps->aps_id;
        }
        else
        {
            aps->aps_id = alf_aps_get_current_alf_idx();
#if M50662_LUMA_CHROMA_SEPARATE_APS
            sh->aps_id_y = aps->aps_id;
            sh->aps_id_ch = aps->aps_id;
#endif
            sh->aps_signaled = aps->aps_id;
        }
    }
    return EVC_OK;
}
#else
int evce_alf(EVCE_CTX * ctx, EVC_PIC * pic, EVC_SH* sh)
{
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);

    double lambdas[3];
    for(int i = 0; i < 3; i++)
        lambdas[i] = (ctx->lambda[i]) * ALF_LAMBDA_SCALE; //this is for appr match of different lambda sets

    set_resetALFBufferFlag(p, sh->slice_type == SLICE_I ? 1 : 0);
    call_enc_ALFProcess(p, lambdas, ctx, pic, &(sh->alf_sh_param) );
    return EVC_OK;
}
#endif
#endif

int evce_picbuf_get_inbuf(EVCE_CTX * ctx, EVC_IMGB ** imgb)
{
    int i, opt, align[EVC_IMGB_MAX_PLANE], pad[EVC_IMGB_MAX_PLANE];

    for(i = 0; i < EVCE_MAX_INBUF_CNT; i++)
    {
        if(ctx->inbuf[i] == NULL)
        {
            opt = EVC_IMGB_OPT_NONE;

            /* set align value*/
            align[0] = MIN_CU_SIZE;
            align[1] = MIN_CU_SIZE >> 1;
            align[2] = MIN_CU_SIZE >> 1;

            /* no padding */
            pad[0] = 0;
            pad[1] = 0;
            pad[2] = 0;

            *imgb = evc_imgb_create(ctx->param.w, ctx->param.h, EVC_COLORSPACE_YUV420_10LE, opt, pad, align);
            evc_assert_rv(*imgb != NULL, EVC_ERR_OUT_OF_MEMORY);

            ctx->inbuf[i] = *imgb;

            (*imgb)->addref(*imgb);
            return EVC_OK;
        }
        else if(ctx->inbuf[i]->getref(ctx->inbuf[i]) == 1)
        {
            *imgb = ctx->inbuf[i];

            (*imgb)->addref(*imgb);
            return EVC_OK;
        }
    }

    return EVC_ERR_UNEXPECTED;
}


#if ALF_PARAMETER_APS
int evce_aps_header(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat, EVC_APS * aps)
{
    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* encode sequence parameter set */
    /* skip first four byte to write the bitstream size */
    evce_bsw_skip_slice_size(bs);

    /* Encode APS nalu header */
    EVC_NALU aps_nalu;
    set_nalu(ctx, &aps_nalu, EVC_APS_NUT);

    /* Write ALF-APS */
    set_aps(ctx, aps); // TBD: empty function call
    evc_assert_rv(evce_eco_aps(bs, aps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_APS_NUT;

    return EVC_OK;
}
#endif
int evce_enc_header(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;
    EVC_NALU  nalu;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* encode sequence parameter set */
    /* skip first four byte to write the bitstream size */
    evce_bsw_skip_slice_size(bs);

    /* nalu header */
    set_nalu(ctx, &nalu, EVC_SPS_NUT);
    evce_eco_nalu(bs, nalu);

    /* sequence parameter set*/
    set_sps(ctx, sps);
    evc_assert_rv(evce_eco_sps(bs, sps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* picture parameter set*/
    set_pps(ctx, pps);
    evc_assert_rv(evce_eco_pps(bs, sps, pps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_SPS_NUT;

    return EVC_OK;
}

static const s8 poc_offset_from_doc_offset[5][16] =
{
    { 0,  -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { 0,  -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { 0,  -4,   -6,   -2,   -7,   -5,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    { 0,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { 0,  -8,   -12,   -4,  -14,  -10,  -6,   -2,  -15,  -13,  -11,   -9,   -7,   -5,   -3,   -1}   /* gop_size = 16 */
};

int poc_derivation(EVCE_CTX * ctx, int layer_id)
{
    int sub_gop_length = (int)pow(2.0, ctx->sps.log2_sub_gop_length);
    int expected_temporal_id = 0;
    int doc_offset, poc_offset;
    if (layer_id == 0)
    {
        ctx->poc = ctx->prev_pic_order_cnt_val + sub_gop_length;
        ctx->prev_doc_offset = 0;
        ctx->prev_pic_order_cnt_val = ctx->poc;
        return EVC_OK;
    }
    doc_offset = (ctx->prev_doc_offset + 1) % sub_gop_length;
    if (doc_offset == 0)
    {
        ctx->prev_pic_order_cnt_val += sub_gop_length;
    } else
    {
        expected_temporal_id = 1 + (int)log2(doc_offset);
    }
    while (layer_id != expected_temporal_id)
    {
        doc_offset = (doc_offset + 1) % sub_gop_length;
        if (doc_offset == 0)
        {
            expected_temporal_id = 0;
        } else
        {
            expected_temporal_id = 1 + (int)log2(doc_offset);
        }
    }
    //poc_offset = (int)(sub_gop_length * ((2.0 * doc_offset + 1) / (int)pow(2.0, layer_id) - 2));
    poc_offset = poc_offset_from_doc_offset[sub_gop_length >> 2][doc_offset];
    ctx->poc = ctx->prev_pic_order_cnt_val + poc_offset;
    ctx->prev_doc_offset = doc_offset;

    return EVC_OK;
}

static void decide_normal_gop(EVCE_CTX * ctx, u32 pic_imcnt)
{
    int i_period, gop_size, pos;
    u32        pic_icnt_b;

    i_period = ctx->param.i_period;
    gop_size = ctx->param.gop_size;

    if(i_period == 0 && pic_imcnt == 0)
    {
        ctx->slice_type = SLICE_I;
        ctx->slice_depth = FRM_DEPTH_0;
        ctx->poc = pic_imcnt;
        ctx->prev_doc_offset = 0;
        ctx->prev_pic_order_cnt_val = ctx->poc;
        ctx->slice_ref_flag = 1;
    }
    else if((i_period != 0) && pic_imcnt % i_period == 0)
    {
        ctx->slice_type = SLICE_I;
        ctx->slice_depth = FRM_DEPTH_0;
        ctx->poc = pic_imcnt;
        ctx->prev_doc_offset = 0;
        ctx->prev_pic_order_cnt_val = ctx->poc;
        ctx->slice_ref_flag = 1;
    }
    else if(pic_imcnt % gop_size == 0)
    {
        ctx->slice_type = SLICE_B;
        ctx->slice_ref_flag = 1;
        ctx->slice_depth = FRM_DEPTH_1;
        ctx->poc = pic_imcnt;
        ctx->prev_doc_offset = 0;
        ctx->prev_pic_order_cnt_val = ctx->poc;
        ctx->slice_ref_flag = 1;
    }
    else
    {
        ctx->slice_type = SLICE_B;
        if(ctx->param.use_hgop)
        {
            pos = (pic_imcnt % gop_size) - 1;

            if (ctx->sps.tool_pocs)
            {
                ctx->slice_depth = tbl_slice_depth_orig[gop_size >> 2][pos];
                ctx->poc = ((pic_imcnt / gop_size) * gop_size) +
                    tbl_poc_gop_offset[gop_size >> 2][pos];
            }
            else
            {
                ctx->slice_depth = tbl_slice_depth[gop_size >> 2][pos];
                int layer_id = ctx->slice_depth - (ctx->slice_depth > 0);
                poc_derivation(ctx, layer_id);
            }
            if (!ctx->sps.tool_pocs && gop_size >= 2)
            {
                ctx->slice_ref_flag = (ctx->slice_depth == tbl_slice_depth[gop_size >> 2][gop_size - 2] ? 0 : 1);
            }
            else
            {
                ctx->slice_ref_flag = 1;
            }
        }
        else
        {
            pos = (pic_imcnt % gop_size) - 1;
            ctx->slice_depth = FRM_DEPTH_2;
            ctx->poc = ((pic_imcnt / gop_size) * gop_size) - gop_size + pos + 1;
            ctx->slice_ref_flag = 0;
        }
        /* find current encoding picture's(B picture) pic_icnt */
        pic_icnt_b = ctx->poc;

        /* find pico again here */
        ctx->pico_idx = (u8)(pic_icnt_b % ctx->pico_max_cnt);
        ctx->pico = ctx->pico_buf[ctx->pico_idx];

        PIC_ORIG(ctx) = &ctx->pico->pic;
    }
}

/* slice_type / slice_depth / poc / PIC_ORIG setting */
static void decide_slice_type(EVCE_CTX * ctx)
{
    u32 pic_imcnt, pic_icnt;
    int i_period, gop_size;
    int force_cnt = 0;

    i_period = ctx->param.i_period;
    gop_size = ctx->param.gop_size;
    pic_icnt = (ctx->pic_cnt + ctx->param.max_b_frames);
    pic_imcnt = pic_icnt;
    ctx->pico_idx = pic_icnt % ctx->pico_max_cnt;
    ctx->pico = ctx->pico_buf[ctx->pico_idx];
    PIC_ORIG(ctx) = &ctx->pico->pic;

    if(gop_size == 1) /* IPPP... */
    {
        pic_imcnt = (i_period > 0) ? pic_icnt % i_period : pic_icnt;
        if(pic_imcnt == 0)
        {
            ctx->slice_type = SLICE_I;
            ctx->slice_depth = FRM_DEPTH_0;
            ctx->poc = 0;
            ctx->slice_ref_flag = 1;
        }
        else
        {
            ctx->slice_type = SLICE_B;
            
            if(ctx->param.use_hgop)
            {
                if (ctx->sps.tool_rpl)
                {
                    ctx->slice_depth = tbl_slice_depth_P_orig[(pic_imcnt - 1) % GOP_P];
                }
                else
                {
                    ctx->slice_depth = tbl_slice_depth_P[ctx->param.ref_pic_gap_length >> 2][(pic_imcnt - 1) % ctx->param.ref_pic_gap_length];
                }
            }
            else
            {
                ctx->slice_depth = FRM_DEPTH_1;
            }
            ctx->poc = (i_period > 0) ? ctx->pic_cnt % i_period : ctx->pic_cnt;
            ctx->slice_ref_flag = 1;
        }
    }
    else /* include B Picture (gop_size = 2 or 4 or 8 or 16) */
    {
        if(pic_icnt == gop_size - 1) /* special case when sequence start */
        {
            ctx->slice_type = SLICE_I;
            ctx->slice_depth = FRM_DEPTH_0;
            ctx->poc = 0;
            ctx->prev_doc_offset = 0;
            ctx->prev_pic_order_cnt_val = ctx->poc;
            ctx->slice_ref_flag = 1;

            /* flush the first IDR picture */
            PIC_ORIG(ctx) = &ctx->pico_buf[0]->pic;
            ctx->pico = ctx->pico_buf[0];
        }
        else if(ctx->force_slice)
        {
            for(force_cnt = ctx->force_ignored_cnt; force_cnt < gop_size; force_cnt++)
            {
                pic_icnt = (ctx->pic_cnt + ctx->param.max_b_frames + force_cnt);
                pic_imcnt = pic_icnt;

                decide_normal_gop(ctx, pic_imcnt);

                if(ctx->poc <= (int)ctx->pic_ticnt)
                {
                    break;
                }
            }
            ctx->force_ignored_cnt = force_cnt;
        }
        else /* normal GOP case */
        {
            decide_normal_gop(ctx, pic_imcnt);
        }
    }
    if (ctx->param.use_hgop && (gop_size > 1 || ctx->sps.tool_rpl))
    {
        if (ctx->sps.tool_rpl)
        {
            ctx->layer_id = ctx->slice_depth;
        } else
        {
            ctx->layer_id = ctx->slice_depth - (ctx->slice_depth > 0);
        }
    }
    else
    {
        ctx->layer_id = 0;
    }
    ctx->ptr = ctx->poc;
    ctx->dtr = ctx->ptr;
}

int evce_enc_pic_prepare(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_PARAM   * param;
    int             ret;
    int             size;

    evc_assert_rv(PIC_ORIG(ctx) != NULL, EVC_ERR_UNEXPECTED);

    param = &ctx->param;
    ret = set_enc_param(ctx, param);
    evc_assert_rv(ret == EVC_OK, ret);

    PIC_CURR(ctx) = evc_picman_get_empty_pic(&ctx->rpm, &ret);
    evc_assert_rv(PIC_CURR(ctx) != NULL, ret);
    ctx->map_refi = PIC_CURR(ctx)->map_refi;
    ctx->map_mv = PIC_CURR(ctx)->map_mv;
#if DMVR_LAG
    ctx->map_unrefined_mv = PIC_CURR(ctx)->map_unrefined_mv;
#endif
    PIC_MODE(ctx) = PIC_CURR(ctx);
#if RDO_DBK
    if(ctx->pic_dbk == NULL)
    {
        ctx->pic_dbk = evce_pic_alloc(&ctx->rpm.pa, &ret);
        evc_assert_rv(ctx->pic_dbk != NULL, ret);
    }
#endif

    decide_slice_type(ctx);

    ctx->lcu_cnt = ctx->f_lcu;
    ctx->slice_num = 0;

    if(ctx->slice_type == SLICE_I) ctx->last_intra_ptr = ctx->ptr;

    size = sizeof(s8) * ctx->f_scu * REFP_NUM;
    evc_mset_x64a(ctx->map_refi, -1, size);

    size = sizeof(s16) * ctx->f_scu * REFP_NUM * MV_D;
    evc_mset_x64a(ctx->map_mv, 0, size);
#if DMVR_LAG
    size = sizeof(s16) * ctx->f_scu * REFP_NUM * MV_D;
    evc_mset_x64a(ctx->map_unrefined_mv, 0, size);
#endif
    /* initialize bitstream container */
    evc_bsw_init(&ctx->bs, bitb->addr, bitb->bsize, NULL);

    /* clear map */
    evc_mset_x64a(ctx->map_scu, 0, sizeof(u32) * ctx->f_scu);

#if AFFINE
    evc_mset_x64a(ctx->map_affine, 0, sizeof(u32) * ctx->f_scu);
#endif
#if ATS_INTER_PROCESS
    evc_mset_x64a(ctx->map_ats_inter, 0, sizeof(u8) * ctx->f_scu);
#endif
    evc_mset_x64a(ctx->map_cu_mode, 0, sizeof(u32) * ctx->f_scu);

    return EVC_OK;
}

int evce_enc_pic_finish(EVCE_CTX *ctx, EVC_BITB *bitb, EVCE_STAT *stat)
{
    EVC_IMGB *imgb_o, *imgb_c;
    int        ret;
    int        i, j;

    evc_mset(stat, 0, sizeof(EVCE_STAT));

    /* expand current encoding picture, if needs */
    ctx->fn_picbuf_expand(ctx, PIC_CURR(ctx));

    /* picture buffer management */
    ret = evc_picman_put_pic(&ctx->rpm, PIC_CURR(ctx), ctx->nalu.nal_unit_type_plus1 - 1 == EVC_IDR_NUT,
                              ctx->ptr, ctx->dtr, ctx->layer_id, 0, ctx->refp,
                              ctx->slice_ref_flag, ctx->sps.tool_rpl, ctx->ref_pic_gap_length);

    evc_assert_rv(ret == EVC_OK, ret);

    imgb_o = PIC_ORIG(ctx)->imgb;
    evc_assert(imgb_o != NULL);

    imgb_c = PIC_CURR(ctx)->imgb;
    evc_assert(imgb_c != NULL);

    /* set stat */
    stat->write = EVC_BSW_GET_WRITE_BYTE(&ctx->bs);
    stat->nalu_type = ctx->slice_type == SLICE_I ? EVC_IDR_NUT : EVC_NONIDR_NUT;
    stat->stype = ctx->slice_type;
    stat->fnum = ctx->pic_cnt;
    stat->qp = ctx->sh.qp;
    stat->poc = ctx->ptr;
    stat->tid = ctx->sh.layer_id;

    for(i = 0; i < 2; i++)
    {
        stat->refpic_num[i] = ctx->rpm.num_refp[i];
        for (j = 0; j < stat->refpic_num[i]; j++)
        {
            stat->refpic[i][j] = ctx->refp[j][i].ptr;
        }
    }

    ctx->pic_cnt++; /* increase picture count */
    ctx->param.f_ifrm = 0; /* clear force-IDR flag */
    ctx->pico->is_used = 0;

    imgb_c->ts[0] = bitb->ts[0] = imgb_o->ts[0];
    imgb_c->ts[1] = bitb->ts[1] = imgb_o->ts[1];
    imgb_c->ts[2] = bitb->ts[2] = imgb_o->ts[2];
    imgb_c->ts[3] = bitb->ts[3] = imgb_o->ts[3];

    if(imgb_o) imgb_o->release(imgb_o);

    return EVC_OK;
}

int evce_enc_pic(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CORE * core;
    EVC_BSW   * bs;
    EVC_SH    * sh;
#if ALF_PARAMETER_APS
    EVC_APS   * aps;
#endif
    int          ret;
    u32          i;
    int split_mode_child[4];
    int split_allow[6] = { 0, 0, 0, 0, 0, 1 };

    bs = &ctx->bs;
    core = ctx->core;
    sh = &ctx->sh;
#if ALF_PARAMETER_APS
    aps = &ctx->aps;
#if APS_ALF_SEQ_FIX
    aps_counter_reset = FALSE;
    if ((int)ctx->ptr > last_intra_poc)
    {
        last_intra_poc = INT_MAX;
        aps_counter_reset = TRUE;
    }
    if (ctx->slice_type == SLICE_I)
        last_intra_poc = ctx->ptr;

    if (aps_counter_reset)
        ctx->aps_counter = 0;
#endif
    if (ctx->slice_type == SLICE_I)
    {
        ctx->aps_counter = -1;
        aps->aps_id = -1;
        ctx->sh.aps_signaled = -1; // reset stored aps id in tile group header
        ctx->aps_temp = 0;
    }
#endif
    if (!ctx->sps.tool_rpl)
    {
        /* initialize reference pictures */
        ret = evc_picman_refp_init(&ctx->rpm, ctx->sps.max_num_ref_pics, ctx->slice_type, ctx->ptr, ctx->layer_id, ctx->last_intra_ptr, ctx->refp);
    }
    else
    {
        /* Set slice header */
        //This needs to be done before reference picture marking and reference picture list construction are invoked
        set_sh(ctx, sh);
#if GRAB_STAT
        evc_stat_set_poc(ctx->sh.poc);
#endif

        if (sh->slice_type != SLICE_I && sh->poc != 0) //TBD: change this condition to say that if this slice is not a slice in IDR picture
        {
            ret = create_explicit_rpl(&ctx->rpm, sh);
            if (ret == 1)
            {
                if (sh->rpl_l0_idx == -1)
                    sh->ref_pic_list_sps_flag[0] = 0;
                if (sh->rpl_l1_idx == -1)
                    sh->ref_pic_list_sps_flag[1] = 0;
            }
        }

        /* reference picture marking */
        ret = evc_picman_refpic_marking(&ctx->rpm, sh);
        evc_assert_rv(ret == EVC_OK, ret);

        /* reference picture lists construction */
        ret = evc_picman_refp_rpl_based_init(&ctx->rpm, sh, ctx->refp);
#if M49023_ADMVP_IMPROVE
        if (sh->slice_type != SLICE_I)
        {
            int dptr0 = (int)(ctx->ptr) - (int)(ctx->refp[0][REFP_0].ptr);
            int dptr1 = (int)(ctx->ptr) - (int)(ctx->refp[0][REFP_1].ptr);
            sh->temporal_mvp_asigned_flag = !(((dptr0 > 0) && (dptr1 > 0)) || ((dptr0 < 0) && (dptr1 < 0)));
            //            printf("tmvp: %d %d %d %d\n", ctx->ptr, ctx->refp[0][REFP_0].ptr, ctx->refp[0][REFP_1].ptr, sh->temporal_mvp_asigned_flag);
        }
#endif
    }
    evc_assert_rv(ret == EVC_OK, ret);

    /* initialize mode decision for frame encoding */
    ret = ctx->fn_mode_init_frame(ctx);
    evc_assert_rv(ret == EVC_OK, ret);

    ctx->fn_mode_analyze_frame(ctx);

    /* slice layer encoding loop */
    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->f_lcu;

    /* Set nalu header */
    set_nalu(ctx, &ctx->nalu, (ctx->slice_type == SLICE_I && ctx->param.use_closed_gop) ? EVC_IDR_NUT: EVC_NONIDR_NUT);

    if (!ctx->sps.tool_rpl)
    {
        /* Set slice header */
        set_sh(ctx, sh);
    }

    core->qp_y = ctx->sh.qp + 6 * (BIT_DEPTH - 8);
    core->qp_u = evc_tbl_qp_chroma_ajudst[sh->qp_u] + 6 * (BIT_DEPTH - 8);
    core->qp_v = evc_tbl_qp_chroma_ajudst[sh->qp_v] + 6 * (BIT_DEPTH - 8);
#if M50662_HISTORY_CTU_ROW_RESET
    ret = evce_hmvp_init(&(core->history_buffer));
    evc_assert_rv(ret == EVC_OK, ret);
#else
#if ADMVP
    evc_mset(core->history_buffer.history_mv_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * MV_D * sizeof(s16));
    
    //evc_mset(core->history_buffer.history_refi_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * sizeof(s8));
    for (int i = 0; i < ALLOWED_CHECKED_NUM; i++)
    {
        core->history_buffer.history_refi_table[i][REFP_0] = REFI_INVALID;
        core->history_buffer.history_refi_table[i][REFP_1] = REFI_INVALID;
    }

    core->history_buffer.currCnt = 0;
    core->history_buffer.m_maxCnt = ALLOWED_CHECKED_NUM;
#endif
#endif
    /* initialize entropy coder */
    evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
    evce_sbac_reset(&core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);

    core->bs_temp.pdata[1] = &core->s_temp_run;
#if DQP
#if RANDOM_DQP_GENERATION
    srand(0);
#endif
    /* generate random input dqp map */
    /* A table can be created here based on image characterstic or cost calculations*/
    for(i = 0; i < ctx->f_scu; i++)
    {
        ctx->map_input_dqp[i] = (rand() % 25) - 12;
    }
#endif

    /* LCU encoding */
#if TRACE_RDO_EXCLUDE_I
    if(ctx->slice_type != SLICE_I)
    {
#endif
        EVC_TRACE_SET(0);
#if TRACE_RDO_EXCLUDE_I
    }
#endif
    if (ctx->sps.tool_mmvd && (ctx->slice_type == SLICE_B))
    {
        sh->mmvd_group_enable_flag = !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr);
    }
#if M50632_IMPROVEMENT_MMVD
    else if (ctx->sps.tool_mmvd && (ctx->slice_type == SLICE_P))
    {
        sh->mmvd_group_enable_flag = 0;
    }
#endif
    else
    {
        sh->mmvd_group_enable_flag = 0;
    }
#if DQP
    ctx->sh.qp_prev = ctx->sh.qp;
#endif
    while(1)
    {
        /* initialize structures *****************************************/
        ret = ctx->fn_mode_init_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
#if M50662_HISTORY_CTU_ROW_RESET
        if (core->x_pel == 0)
        {
            ret = evce_hmvp_init(&(core->history_buffer));
            evc_assert_rv(ret == EVC_OK, ret);
        }
#endif
        /* mode decision *************************************************/
        SBAC_LOAD(core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], *GET_SBAC_ENC(bs));
        core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].is_bitcount = 1;
        evce_init_bef_data(core, ctx);
#if GRAB_STAT
        evc_stat_set_enc_state(TRUE);
#endif
        ret = ctx->fn_mode_analyze_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
#if IBC
        if (ctx->param.use_ibc_flag && (ctx->param.ibc_fast_method & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE) && ctx->param.ibc_hash_search_flag)
        {
          reset_ibc_search_range(ctx, core->x_pel, core->y_pel, ctx->max_cuwh, ctx->max_cuwh);
        }
#endif
        /* entropy coding ************************************************/

        ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1
                             , NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
            , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
            , evc_get_default_tree_cons()
#endif
        );
#if GRAB_STAT
        evc_stat_set_enc_state(FALSE);
        evc_stat_write_lcu(core->x_pel, core->y_pel, ctx->w, ctx->h, ctx->max_cuwh, ctx->log2_culine, ctx, core, ctx->map_cu_data[core->lcu_num].split_mode, ctx->map_cu_data[core->lcu_num].suco_flag);
#endif
        evc_assert_rv(ret == EVC_OK, ret);

        /* prepare next step *********************************************/
        core->x_lcu++;
        if(core->x_lcu >= ctx->w_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }
        core->x_pel = core->x_lcu << ctx->log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->log2_max_cuwh;
        core->lcu_num++;
        ctx->lcu_cnt--;

#if HISTORY_LCU_COPY_BUG_FIX
        evc_mcpy(&core->history_buffer, &core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], sizeof(core->history_buffer));
#endif

        /* end_of_picture_flag */
        if(ctx->lcu_cnt > 0)
        {
            evce_eco_slice_end_flag(bs, 0);
        }
        else
        {
            evce_eco_slice_end_flag(bs, 1);
            evce_sbac_finish(bs);
            break;
        }
    }

    /* deblocking filter */
    if(ctx->param.use_deblock)
    {
        ret = ctx->fn_deblock(ctx, PIC_MODE(ctx));
        evc_assert_rv(ret == EVC_OK, ret);
    }

#if ALF
    /* adaptive loop filter */
    sh->alf_on = ctx->sps.tool_alf;
    if(sh->alf_on)
    {
#if ALF_PARAMETER_APS
        ret = ctx->fn_alf(ctx, PIC_MODE(ctx), sh, aps);
#else
        ret = ctx->fn_alf(ctx, PIC_MODE(ctx), sh);
#endif
        evc_assert_rv(ret == EVC_OK, ret);
    }
#endif

    /* Bit-stream re-writing (START) */
    evc_bsw_init(&ctx->bs, (u8*)bitb->addr, bitb->bsize, NULL);
#if TRACE_START_POC
    if (fp_trace_started == 1)
    {
        EVC_TRACE_SET(1);
    }
    else
    {
        EVC_TRACE_SET(0);
    }
#else
#if TRACE_RDO_EXCLUDE_I
    if(ctx->slice_type != SLICE_I)
    {
#endif
        EVC_TRACE_SET(1);
#if TRACE_RDO_EXCLUDE_I
    }
#endif
#endif

    EVC_NALU aps_nalu;
    set_nalu(ctx, &aps_nalu, EVC_APS_NUT);
    int aps_nalu_size = 0;

#if ALF_PARAMETER_APS
    /* Encode ALF in APS */
    if ((ctx->sps.tool_alf) && (ctx->sh.alf_on)) // User defined params
    {
        if ((aps->alf_aps_param.enabledFlag[0]) && (aps->alf_aps_param.temporalAlfFlag == 0))    // Encoder defined parameters (RDO): ALF is selected, and new ALF was derived for TG
        {
            /* Encode APS nalu header */
            ret = evce_eco_nalu(bs, aps_nalu);
            evc_assert_rv(ret == EVC_OK, ret);

            /* Write ALF-APS */
            set_aps(ctx, aps); // TBD: empty function call
            evc_assert_rv(evce_eco_aps(bs, aps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

            evc_bsw_deinit(bs);
            aps_nalu.nal_unit_size = evce_bsw_write_nalu_size(bs);
            aps_nalu_size = aps_nalu.nal_unit_size + 4;
        }
    }
#endif

    int* size_field = (int*)(*(&bs->cur));

    /* Encode nalu header */
    ret = evce_eco_nalu(bs, ctx->nalu);
    evc_assert_rv(ret == EVC_OK, ret);

    /* Encode slice header */
#if ALF
    sh->num_ctb = ctx->f_lcu;
#endif
    ret = evce_eco_sh(bs, &ctx->sps, &ctx->pps, sh);
    evc_assert_rv(ret == EVC_OK, ret);

    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->f_lcu;
    for(i = 0; i < ctx->f_scu; i++)
    {
        MCU_CLR_COD(ctx->map_scu[i]);
    }

    evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
#if DQP
    ctx->sh.qp_prev = ctx->sh.qp;
#endif
#if DQP
    {
        int size;
        //reset dqp used map data
        size = sizeof(s8) * ctx->f_scu;
        evc_mset_x64a(ctx->map_dqp_used, DQP_UNUSED, size);
    }
#endif

#if GRAB_STAT
    evc_stat_set_enc_state(FALSE);
#endif
    /* Encode slice data */
    while(1)
    {
#if APS_ALF_CTU_FLAG
        evc_AlfSliceParam* alfSliceParam = &(ctx->sh.alf_sh_param);
        if ((alfSliceParam->isCtbAlfOn) && (sh->alf_on))
        {
            EVCE_SBAC *sbac;
            sbac = GET_SBAC_ENC(bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("Usage of ALF: ");
#if ALF_CTU_MAP_DYNAMIC
            evce_sbac_encode_bin((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)), sbac, sbac->ctx.ctb_alf_flag, bs);
            EVC_TRACE_INT((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)));
#else
            evce_sbac_encode_bin((int)(alfSliceParam->alfCtuEnableFlag[0][core->lcu_num]), sbac, sbac->ctx.ctb_alf_flag, bs);
            EVC_TRACE_INT((int)(alfSliceParam->alfCtuEnableFlag[0][core->lcu_num]));
#endif
            EVC_TRACE_STR("\n");
        }
#endif
        ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1, NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
            , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
            , evc_get_default_tree_cons()
#endif
        );
        evc_assert_rv(ret == EVC_OK, ret);

        core->x_lcu++;
        if(core->x_lcu >= ctx->w_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }
        core->x_pel = core->x_lcu << ctx->log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->log2_max_cuwh;
        core->lcu_num++;
        ctx->lcu_cnt--;
        /* end_of_picture_flag */
        if(ctx->lcu_cnt > 0)
        {
            evce_eco_slice_end_flag(bs, 0);
        }
        else
        {
            evce_eco_slice_end_flag(bs, 1);
            evce_sbac_finish(bs);
            break;
        }
    }

#if TRACE_DBF
    /* deblocking filter */
    if (ctx->param.use_deblock)
    {
        ret = ctx->fn_deblock(ctx, PIC_CURR_BDBF(ctx));
        evc_assert_rv(ret == EVC_OK, ret);
    }
#endif
    /* Bit-stream re-writing (END) */
    evc_bsw_deinit(bs);
    *size_field = EVC_BSW_GET_WRITE_BYTE(bs) - 4 - aps_nalu_size;

    return EVC_OK;
}

int evce_enc(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    int            ret;
    int            gop_size, pic_cnt;

    pic_cnt = ctx->pic_icnt - ctx->frm_rnum;
    gop_size = ctx->param.gop_size;

    ctx->force_slice = ((ctx->pic_ticnt % gop_size >= ctx->pic_ticnt - pic_cnt + 1) && FORCE_OUT(ctx)) ? 1 : 0;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* initialize variables for a picture encoding */
    ret = ctx->fn_enc_pic_prepare(ctx, bitb, stat);
    evc_assert_rv(ret == EVC_OK, ret);

    /* encode one picture */
    ret = ctx->fn_enc_pic(ctx, bitb, stat);
    evc_assert_rv(ret == EVC_OK, ret);

    /* finishing of encoding a picture */
    ctx->fn_enc_pic_finish(ctx, bitb, stat);
    evc_assert_rv(ret == EVC_OK, ret);

    return EVC_OK;
}

int evce_push_frm(EVCE_CTX * ctx, EVC_IMGB * imgb)
{
    EVC_PIC    * pic;

    ctx->pic_icnt++;

    ctx->pico_idx = ctx->pic_icnt % ctx->pico_max_cnt;
    ctx->pico = ctx->pico_buf[ctx->pico_idx];
    ctx->pico->pic_icnt = ctx->pic_icnt;
    ctx->pico->is_used = 1;
    pic = &ctx->pico->pic;
    PIC_ORIG(ctx) = pic;

    /* set pushed image to current input (original) image */
    evc_mset(pic, 0, sizeof(EVC_PIC));

    pic->buf_y = imgb->baddr[0];
    pic->buf_u = imgb->baddr[1];
    pic->buf_v = imgb->baddr[2];

    pic->y = imgb->a[0];
    pic->u = imgb->a[1];
    pic->v = imgb->a[2];

    pic->w_l = imgb->w[0];
    pic->h_l = imgb->h[0];
    pic->w_c = imgb->w[1];
    pic->h_c = imgb->h[1];

    pic->s_l = STRIDE_IMGB2PIC(imgb->s[0]);
    pic->s_c = STRIDE_IMGB2PIC(imgb->s[1]);

    pic->imgb = imgb;

    imgb->addref(imgb);

    return EVC_OK;
}

int evce_platform_init(EVCE_CTX * ctx)
{
    int ret = EVC_ERR_UNKNOWN;

    /* create mode decision */
    ret = evce_mode_create(ctx, 0);
    evc_assert_rv(EVC_OK == ret, ret);

    /* create intra prediction analyzer */
    ret = evce_pintra_create(ctx, 0);
    evc_assert_rv(EVC_OK == ret, ret);

    /* create inter prediction analyzer */
    ret = evce_pinter_create(ctx, 0);
    evc_assert_rv(EVC_OK == ret, ret);
#if IBC
    if (ctx->param.use_ibc_flag)
    {
      /* create ibc prediction analyzer */
      ret = evce_pibc_create(ctx, 0);
      evc_assert_rv(EVC_OK == ret, ret);
    }
#endif
#if RDO_DBK
    ctx->pic_dbk = NULL;
#endif

#if ALF
#if ALF_PARAMETER_APS
    ctx->fn_alf = evce_alf_aps;
#else
    ctx->fn_alf = evce_alf;
#endif
#endif
    ctx->fn_ready = evce_ready;
    ctx->fn_flush = evce_flush;
    ctx->fn_enc = evce_enc;
    ctx->fn_enc_header = evce_enc_header;
    ctx->fn_enc_pic = evce_enc_pic;
    ctx->fn_enc_pic_prepare = evce_enc_pic_prepare;
    ctx->fn_enc_pic_finish = evce_enc_pic_finish;
    ctx->fn_push = evce_push_frm;
    ctx->fn_deblock = evce_deblock_h263;
    ctx->fn_picbuf_expand = evce_picbuf_expand;
    ctx->fn_get_inbuf = evce_picbuf_get_inbuf;
    ctx->pf = NULL;

    return EVC_OK;
}

void evce_platform_deinit(EVCE_CTX * ctx)
{
    evc_assert(ctx->pf == NULL);

    ctx->fn_ready = NULL;
    ctx->fn_flush = NULL;
    ctx->fn_enc = NULL;
    ctx->fn_enc_pic = NULL;
    ctx->fn_enc_pic_prepare = NULL;
    ctx->fn_enc_pic_finish = NULL;
    ctx->fn_push = NULL;
    ctx->fn_deblock = NULL;
#if ALF
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);
    call_destroy_enc_ALF(p);
    delete_enc_ALF(ctx->enc_alf);
    ctx->fn_alf = NULL;
#endif
#if IBC
    if (ctx->param.use_ibc_flag)
    {
      destroy_enc_IBC(ctx->ibc_hash_handle);
      ctx->ibc_hash_handle = NULL;
    }
#endif
    ctx->fn_picbuf_expand = NULL;
    ctx->fn_get_inbuf = NULL;
}

EVCE evce_create(EVCE_CDSC * cdsc, int * err)
{
    EVCE_CTX  * ctx;
    int          ret;

#if ENC_DEC_TRACE
    fp_trace = fopen("enc_trace.txt", "w+");
#endif
#if GRAB_STAT
    evc_stat_init("enc_stat.vtmbmsstats", esu_only_enc, 0, -1, ence_stat_cu);
    enc_stat_header(cdsc->w, cdsc->h);
#endif
    ctx = NULL;

    /* memory allocation for ctx and core structure */
    ctx = ctx_alloc();
    evc_assert_gv(ctx != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mcpy(&ctx->cdsc, cdsc, sizeof(EVCE_CDSC));

    /* set default value for encoding parameter */
    ret = set_init_param(cdsc, &ctx->param);
    evc_assert_g(ret == EVC_OK, ERR);

    ret = evce_platform_init(ctx);
    evc_assert_g(ret == EVC_OK, ERR);

    ret = evc_scan_tbl_init();
    evc_assert_g(ret == EVC_OK, ERR);

    evce_init_err_scale();
    evce_split_tbl_init(ctx);

    if(ctx->fn_ready != NULL)
    {
        ret = ctx->fn_ready(ctx);
        evc_assert_g(ret == EVC_OK, ERR);
    }

    /* set default value for ctx */
    ctx->magic = EVCE_MAGIC_CODE;
    ctx->id = (EVCE)ctx;
#if ALF_PARAMETER_APS
    ctx->sh.aps_signaled = -1;
#endif

#if ATS_INTRA_PROCESS
    evc_init_multi_tbl();
    evc_init_multi_inv_tbl();
#endif

    return (ctx->id);
ERR:
    if(ctx)
    {
        evce_platform_deinit(ctx);
        ctx_free(ctx);
    }
    if(err) *err = ret;
    return NULL;
}

void evce_delete(EVCE id)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_R(id, ctx);
#if ENC_DEC_TRACE
    fclose(fp_trace);
#endif
#if GRAB_STAT
    evc_stat_finish();
#endif

    if(ctx->fn_flush != NULL)
    {
        ctx->fn_flush(ctx);
    }
    evce_platform_deinit(ctx);

    ctx_free(ctx);

    evc_scan_tbl_delete();
}

int evce_encode_sps(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc_header, EVC_ERR_UNEXPECTED);

    /* update BSB */
    bitb->err = 0;

    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_NALU  nalu;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* nalu header */
    set_nalu(ctx, &nalu, EVC_SPS_NUT);
    evce_eco_nalu(bs, nalu);

    /* sequence parameter set*/
    set_sps(ctx, sps);
    evc_assert_rv(evce_eco_sps(bs, sps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_SPS_NUT;

    return EVC_OK;
}

int evce_encode_pps(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc_header, EVC_ERR_UNEXPECTED);

    /* update BSB */
    bitb->err = 0;

    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;
    EVC_NALU  nalu;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* nalu header */
    set_nalu(ctx, &nalu, EVC_PPS_NUT);
    evce_eco_nalu(bs, nalu);

    /* sequence parameter set*/
    set_pps(ctx, pps);
    evc_assert_rv(evce_eco_pps(bs, sps, pps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_PPS_NUT;

    return EVC_OK;
}

/*int evce_encode_header(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc_header, EVC_ERR_UNEXPECTED);

    bitb->err = 0;

    return ctx->fn_enc_header(ctx, bitb, stat);
}*/

static int check_frame_delay(EVCE_CTX * ctx)
{
    if(ctx->pic_icnt < ctx->frm_rnum)
    {
        return EVC_OK_OUT_NOT_AVAILABLE;
    }
    return EVC_OK;
}

static int check_more_frames(EVCE_CTX * ctx)
{
    EVCE_PICO  * pico;
    int           i;

    if(FORCE_OUT(ctx))
    {
        /* pseudo evce_push() for bumping process ****************/
        ctx->pic_icnt++;
        /**********************************************************/

        for(i=0; i<ctx->pico_max_cnt; i++)
        {
            pico = ctx->pico_buf[i];
            if(pico != NULL)
            {
                if(pico->is_used == 1)
                {
                    return EVC_OK;
                }
            }
        }

        return EVC_OK_NO_MORE_FRM;
    }

    return EVC_OK;
}

int evce_encode(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc, EVC_ERR_UNEXPECTED);

    /* bumping - check whether input pictures are remaining or not in pico_buf[] */
    if(EVC_OK_NO_MORE_FRM == check_more_frames(ctx))
    {
        return EVC_OK_NO_MORE_FRM;
    }

    /* store input picture and return if needed */
    if(EVC_OK_OUT_NOT_AVAILABLE == check_frame_delay(ctx))
    {
        return EVC_OK_OUT_NOT_AVAILABLE;
    }

    /* update BSB */
    bitb->err = 0;

    return ctx->fn_enc(ctx, bitb, stat);
}

int evce_push(EVCE id, EVC_IMGB * img)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_push, EVC_ERR_UNEXPECTED);

    return ctx->fn_push(ctx, img);
}

int evce_config(EVCE id, int cfg, void * buf, int * size)
{
    EVCE_CTX      * ctx;
    int               t0;
    EVC_IMGB      * imgb;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);

    switch(cfg)
    {
        /* set config **********************************************************/
        case EVCE_CFG_SET_FORCE_OUT:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.force_output = (t0) ? 1 : 0;
            /* store total input picture count at this time */
            ctx->pic_ticnt = ctx->pic_icnt;
            break;

        case EVCE_CFG_SET_FINTRA:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.f_ifrm = t0;
            break;
        case EVCE_CFG_SET_QP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 >= MIN_QUANT && t0 <= MAX_QUANT, \
                           EVC_ERR_INVALID_ARGUMENT);
            ctx->param.qp = t0;
            break;
        case EVCE_CFG_SET_FPS:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 > 0, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.fps = t0;
            break;
        case EVCE_CFG_SET_I_PERIOD:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 >= 0, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.i_period = t0;
            break;
        case EVCE_CFG_SET_QP_MIN:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 >= MIN_QUANT, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.qp_min = t0;
            break;
        case EVCE_CFG_SET_QP_MAX:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 <= MAX_QUANT, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.qp_max = t0;
            break;
        case EVCE_CFG_SET_USE_DEBLOCK:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.use_deblock = t0;
            break;
#if M49023_DBF_IMPROVE
        case EVCE_CFG_SET_DEBLOCK_A_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.deblock_alpha_offset = t0;
            break;
        case EVCE_CFG_SET_DEBLOCK_B_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.deblock_beta_offset = t0;
            break;
#endif
        case EVCE_CFG_SET_USE_PIC_SIGNATURE:
            ctx->param.use_pic_sign = (*((int *)buf)) ? 1 : 0;
            break;

            /* get config *******************************************************/
        case EVCE_CFG_GET_QP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.qp;
            break;
        case EVCE_CFG_GET_WIDTH:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.w;
            break;
        case EVCE_CFG_GET_HEIGHT:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.h;
            break;
        case EVCE_CFG_GET_FPS:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.fps;
            break;
        case EVCE_CFG_GET_I_PERIOD:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.i_period;
            break;
        case EVCE_CFG_GET_RECON:
            evc_assert_rv(*size == sizeof(EVC_IMGB**), EVC_ERR_INVALID_ARGUMENT);
            imgb = PIC_CURR(ctx)->imgb;
            *((EVC_IMGB **)buf) = imgb;
            imgb->addref(imgb);
            break;
        case EVCE_CFG_GET_USE_DEBLOCK:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.use_deblock;
            break;
        case EVCE_CFG_GET_CLOSED_GOP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.use_closed_gop;
            break;
        case EVCE_CFG_GET_HIERARCHICAL_GOP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.use_hgop;
            break;
#if M49023_DBF_IMPROVE
        case EVCE_CFG_GET_DEBLOCK_A_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.deblock_alpha_offset;
            break;
        case EVCE_CFG_GET_DEBLOCK_B_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.deblock_beta_offset;
            break;
#endif
        default:
            evc_trace("unknown config value (%d)\n", cfg);
            evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
    }

    return EVC_OK;
}

int evce_get_inbuf(EVCE id, EVC_IMGB ** img)
{
    EVCE_CTX *ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_get_inbuf, EVC_ERR_UNEXPECTED);
    evc_assert_rv(img != NULL, EVC_ERR_INVALID_ARGUMENT);

    return ctx->fn_get_inbuf(ctx, img);
}

void evce_malloc_1d(void** dst, int size)
{
    if(*dst == NULL)
    {
        *dst = evc_malloc_fast(size);
        evc_mset(*dst, 0, size);
    }
}

void evce_malloc_2d(s8*** dst, int size_1d, int size_2d, int type_size)
{
    int i;

    if(*dst == NULL)
    {
        *dst = evc_malloc_fast(size_1d * sizeof(s8*));
        evc_mset(*dst, 0, size_1d * sizeof(s8*));


        (*dst)[0] = evc_malloc_fast(size_1d * size_2d * type_size);
        evc_mset((*dst)[0], 0, size_1d * size_2d * type_size);

        for(i = 1; i < size_1d; i++)
        {
            (*dst)[i] = (*dst)[i - 1] + size_2d * type_size;
        }
    }
}

int evce_create_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh)
{
    int i, j;
    int cuw_scu, cuh_scu;
    int size_8b, size_16b, size_32b, cu_cnt, pixel_cnt;

    cuw_scu = 1 << log2_cuw;
    cuh_scu = 1 << log2_cuh;

    size_8b = cuw_scu * cuh_scu * sizeof(s8);
    size_16b = cuw_scu * cuh_scu * sizeof(s16);
    size_32b = cuw_scu * cuh_scu * sizeof(s32);
    cu_cnt = cuw_scu * cuh_scu;
    pixel_cnt = cu_cnt << 4;

    evce_malloc_1d((void**)&cu_data->qp_y, size_8b);
    evce_malloc_1d((void**)&cu_data->qp_u, size_8b);
    evce_malloc_1d((void**)&cu_data->qp_v, size_8b);
    evce_malloc_1d((void**)&cu_data->pred_mode, size_8b);
#if M50761_CHROMA_NOT_SPLIT
    evce_malloc_1d((void**)&cu_data->pred_mode_chroma, size_8b);
#endif
    evce_malloc_2d((s8***)&cu_data->mpm, 2, cu_cnt, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->ipm, 2, cu_cnt, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->mpm_ext, 8, cu_cnt, sizeof(u8));
    evce_malloc_1d((void**)&cu_data->skip_flag, size_8b);
#if IBC
    evce_malloc_1d((void**)&cu_data->ibc_flag, size_8b);
#endif
#if DMVR_FLAG
    evce_malloc_1d((void**)&cu_data->dmvr_flag, size_8b);
#endif
    evce_malloc_2d((s8***)&cu_data->refi, cu_cnt, REFP_NUM, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->mvp_idx, cu_cnt, REFP_NUM, sizeof(u8));
    evce_malloc_1d((void**)&cu_data->mvr_idx, size_8b);
    evce_malloc_1d((void**)&cu_data->bi_idx, size_8b);
    evce_malloc_1d((void**)&cu_data->mmvd_idx, size_16b);
    evce_malloc_1d((void**)&cu_data->mmvd_flag, size_8b);

#if ATS_INTRA_PROCESS
    evce_malloc_1d((void**)& cu_data->ats_intra_cu, size_8b);
    evce_malloc_1d((void**)& cu_data->ats_tu_h, size_8b);
    evce_malloc_1d((void**)& cu_data->ats_tu_v, size_8b);
#endif
#if ATS_INTER_PROCESS
    evce_malloc_1d((void**)&cu_data->ats_inter_info, size_8b);
#endif

    for(i = 0; i < N_C; i++)
    {
        evce_malloc_1d((void**)&cu_data->nnz[i], size_32b);
    }
    for (i = 0; i < N_C; i++)
    {
        for (j = 0; j < 4; j++)
        {
            evce_malloc_1d((void**)&cu_data->nnz_sub[i][j], size_32b);
        }
    }
    evce_malloc_1d((void**)&cu_data->map_scu, size_32b);
#if AFFINE
    evce_malloc_1d((void**)&cu_data->affine_flag, size_8b);
    evce_malloc_1d((void**)&cu_data->map_affine, size_32b);
#endif    
    evce_malloc_1d((void**)&cu_data->map_cu_mode, size_32b);
#if !M50761_REMOVE_BLOCK_SIZE_MAP
    evce_malloc_2d((s8***)&cu_data->block_size, cu_cnt, 2, sizeof(s16));
#endif
    evce_malloc_1d((void**)&cu_data->depth, size_8b);

    for(i = 0; i < N_C; i++)
    {
        evce_malloc_1d((void**)&cu_data->coef[i], (pixel_cnt >> (!!(i)* 2)) * sizeof(s16));
        evce_malloc_1d((void**)&cu_data->reco[i], (pixel_cnt >> (!!(i)* 2)) * sizeof(pel));
    }

    return EVC_OK;
}

void evce_free_1d(void* dst)
{
    if(dst != NULL)
    {
        evc_mfree_fast(dst);
    }
}

void evce_free_2d(void** dst)
{
    if (dst)
    {
        if (dst[0])
        {
            evc_mfree_fast(dst[0]);
        }
        evc_mfree_fast(dst);
    }
}

int evce_delete_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh)
{
    int i, j;

    evce_free_1d((void*)cu_data->qp_y);
    evce_free_1d((void*)cu_data->qp_u);
    evce_free_1d((void*)cu_data->qp_v);
    evce_free_1d((void*)cu_data->pred_mode);
#if M50761_CHROMA_NOT_SPLIT
    evce_free_1d((void*)cu_data->pred_mode_chroma);
#endif
    evce_free_2d((void**)cu_data->mpm);
    evce_free_2d((void**)cu_data->ipm);
    evce_free_2d((void**)cu_data->mpm_ext);
    evce_free_1d((void*)cu_data->skip_flag);
#if IBC
    evce_free_1d((void*)cu_data->ibc_flag);
#endif
#if DMVR_FLAG
    evce_free_1d((void*)cu_data->dmvr_flag);
#endif
    evce_free_2d((void**)cu_data->refi);
    evce_free_2d((void**)cu_data->mvp_idx);
    evce_free_1d(cu_data->mvr_idx);
    evce_free_1d(cu_data->bi_idx);
    evce_free_1d((void*)cu_data->mmvd_idx);
    evce_free_1d((void*)cu_data->mmvd_flag);

    for (i = 0; i < N_C; i++)
    {
        evce_free_1d((void*)cu_data->nnz[i]);
    }
    for (i = 0; i < N_C; i++)
    {
        for (j = 0; j < 4; j++)
        {
            evce_free_1d((void*)cu_data->nnz_sub[i][j]);
        }
    }
    evce_free_1d((void*)cu_data->map_scu);
#if AFFINE
    evce_free_1d((void*)cu_data->affine_flag);
    evce_free_1d((void*)cu_data->map_affine);
#endif   
#if ATS_INTRA_PROCESS
    evce_free_1d((void*)cu_data->ats_intra_cu);
    evce_free_1d((void*)cu_data->ats_tu_h);
    evce_free_1d((void*)cu_data->ats_tu_v);
#endif
#if ATS_INTER_PROCESS
    evce_free_1d((void*)cu_data->ats_inter_info);
#endif
    evce_free_1d((void*)cu_data->map_cu_mode);
#if !M50761_REMOVE_BLOCK_SIZE_MAP
    evce_free_2d((void**)cu_data->block_size);
#endif
    evce_free_1d((void*)cu_data->depth);
    for (i = 0; i < N_C; i++)
    {
        evce_free_1d((void*)cu_data->coef[i]);
        evce_free_1d((void*)cu_data->reco[i]);
    }

    return EVC_OK;
}

#if ALF
void codeAlfCtuEnableFlag(EVC_BSW *bs, EVCE_CTX * ctx, int refId, int compIdx)
{
// TO DO
}
#endif
