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
#include "evcd_def.h"
#include "evcd_eco.h"
#include "evc_df.h"
#if GRAB_STAT
#include "evc_debug.h"
#endif

#if ALF
#include "wrapper.h"
#endif

/* convert EVCD into EVCD_CTX */
#define EVCD_ID_TO_CTX_R(id, ctx) \
    evc_assert_r((id)); \
    (ctx) = (EVCD_CTX *)id; \
    evc_assert_r((ctx)->magic == EVCD_MAGIC_CODE);

/* convert EVCD into EVCD_CTX with return value if assert on */
#define EVCD_ID_TO_CTX_RV(id, ctx, ret) \
    evc_assert_rv((id), (ret)); \
    (ctx) = (EVCD_CTX *)id; \
    evc_assert_rv((ctx)->magic == EVCD_MAGIC_CODE, (ret));

static EVCD_CTX * ctx_alloc(void)
{
    EVCD_CTX * ctx;

    ctx = (EVCD_CTX*)evc_malloc_fast(sizeof(EVCD_CTX));

    evc_assert_rv(ctx != NULL, NULL);
    evc_mset_x64a(ctx, 0, sizeof(EVCD_CTX));

    /* set default value */
    ctx->dtr           = 0;
    ctx->pic_cnt       = 0;

    return ctx;
}

static void ctx_free(EVCD_CTX * ctx)
{
    evc_mfree_fast(ctx);
}

static EVCD_CORE * core_alloc(void)
{
    EVCD_CORE * core;

    core = (EVCD_CORE*)evc_malloc_fast(sizeof(EVCD_CORE));

    evc_assert_rv(core, NULL);
    evc_mset_x64a(core, 0, sizeof(EVCD_CORE));

    return core;
}

static void core_free(EVCD_CORE * core)
{
    evc_mfree_fast(core);
}

static void sequence_deinit(EVCD_CTX * ctx)
{
    evc_mfree(ctx->map_scu);
    evc_mfree(ctx->map_split);
    evc_mfree(ctx->map_ipm);
    evc_mfree(ctx->map_suco);
#if AFFINE
    evc_mfree(ctx->map_affine);
#endif
    evc_mfree(ctx->map_cu_mode);
#if ATS_INTER_PROCESS
    evc_mfree(ctx->map_ats_inter);
#endif
#if DQP
    evc_mfree(ctx->map_dqp_used);
#endif
    evc_picman_deinit(&ctx->dpm);
}

static int sequence_init(EVCD_CTX * ctx, EVC_SPS * sps)
{
    int size;
    int ret;

    if(sps->pic_width_in_luma_samples != ctx->w || sps->pic_height_in_luma_samples != ctx->h)
    {
        /* resolution was changed */
        sequence_deinit(ctx);

        ctx->w = sps->pic_width_in_luma_samples;
        ctx->h = sps->pic_height_in_luma_samples;

        if (ctx->sps.sps_btt_flag)
        {
            ctx->max_cuwh = 1 << (sps->log2_ctu_size_minus2 + 2);
        }
        else
        {
            ctx->max_cuwh = 1 << 6;
        }

        ctx->log2_max_cuwh = CONV_LOG2(ctx->max_cuwh);
    }

    size = ctx->max_cuwh;
    ctx->w_lcu = (ctx->w + (size - 1)) / size;
    ctx->h_lcu = (ctx->h + (size - 1)) / size;
    ctx->f_lcu = ctx->w_lcu * ctx->h_lcu;
    ctx->w_scu = (ctx->w + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->h_scu = (ctx->h + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->f_scu = ctx->w_scu * ctx->h_scu;

#if ALF
    ctx->alf              = new_ALF();
    AdaptiveLoopFilter* p = (AdaptiveLoopFilter*)(ctx->alf);
    call_create_ALF(p, ctx->w, ctx->h, ctx->max_cuwh, ctx->max_cuwh, 5);
#endif

    /* alloc SCU map */
    if(ctx->map_scu == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_scu = (u32 *)evc_malloc(size);
        evc_assert_gv(ctx->map_scu, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_scu, 0, size);
    }

#if AFFINE
    /* alloc affine SCU map */
    if (ctx->map_affine == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_affine = (u32 *)evc_malloc(size);
        evc_assert_gv(ctx->map_affine, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_affine, 0, size);
    }
#endif
    /* alloc cu mode SCU map */
    if(ctx->map_cu_mode == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_cu_mode = (u32 *)evc_malloc(size);
        evc_assert_gv(ctx->map_cu_mode, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_cu_mode, 0, size);
    }
#if ATS_INTER_PROCESS
    if (ctx->map_ats_inter == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_inter = (u8 *)evc_malloc(size);
        evc_assert_gv(ctx->map_ats_inter, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_ats_inter, 0, size);
    }
#endif

    /* alloc map for CU split flag */
    if(ctx->map_split == NULL)
    {
        size = sizeof(s8) * ctx->f_lcu * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU;
        ctx->map_split = evc_malloc(size);
        evc_assert_gv(ctx->map_split, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_split, 0, size);
    }

    /* alloc map for LCU suco flag */
    if(ctx->map_suco == NULL)
    {
        size = sizeof(s8) * ctx->f_lcu * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU;
        ctx->map_suco = evc_malloc(size);
        evc_assert_gv(ctx->map_suco, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_suco, 0, size);
    }

    /* alloc map for intra prediction mode */
    if(ctx->map_ipm == NULL)
    {
        size = sizeof(s8) * ctx->f_scu;
        ctx->map_ipm = (s8 *)evc_malloc(size);
        evc_assert_gv(ctx->map_ipm, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_ipm, -1, size);
    }

#if DQP
    if (ctx->map_dqp_used == NULL)
    {
        size = sizeof(s8) * ctx->f_scu;
        ctx->map_dqp_used = (u8 *)evc_malloc(size);
        evc_assert_gv(ctx->map_dqp_used, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_dqp_used, DQP_UNUSED, size);
    }
#endif

    /* initialize reference picture manager */
    ctx->pa.fn_alloc = evcd_picbuf_alloc;
    ctx->pa.fn_free = evcd_picbuf_free;
    ctx->pa.w = ctx->w;
    ctx->pa.h = ctx->h;
    ctx->pa.pad_l = PIC_PAD_SIZE_L;
    ctx->pa.pad_c = PIC_PAD_SIZE_C;
    ctx->ref_pic_gap_length = (int)pow(2.0, sps->log2_ref_pic_gap_length);

    ret = evc_picman_init(&ctx->dpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, &ctx->pa);
    evc_assert_g(EVC_SUCCEEDED(ret), ERR);

    evcd_split_tbl_init(ctx);

    if(ctx->sps.tool_iqt == 0)
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_base;
    }
    else
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_main;
    }

    return EVC_OK;
ERR:
    sequence_deinit(ctx);

    return ret;
}

static void slice_deinit(EVCD_CTX * ctx)
{
}

static int slice_init(EVCD_CTX * ctx, EVCD_CORE * core, EVC_SH * sh)
{
    core->lcu_num = 0;
    core->x_lcu = 0;
    core->y_lcu = 0;
    core->x_pel = 0;
    core->y_pel = 0;
    core->qp_y = sh->qp + 6 * (BIT_DEPTH - 8);
    core->qp_u = evc_tbl_qp_chroma_ajudst[sh->qp_u] + 6 * (BIT_DEPTH - 8);
    core->qp_v = evc_tbl_qp_chroma_ajudst[sh->qp_v] + 6 * (BIT_DEPTH - 8);

    ctx->dtr_prev_low = sh->dtr;
    ctx->dtr = sh->dtr;
    ctx->ptr = sh->dptr + sh->dtr; /* PTR */

    /* clear maps */
    evc_mset_x64a(ctx->map_scu, 0, sizeof(u32) * ctx->f_scu);
#if AFFINE
    evc_mset_x64a(ctx->map_affine, 0, sizeof(u32) * ctx->f_scu);
#endif
#if ATS_INTER_PROCESS
    evc_mset_x64a(ctx->map_ats_inter, 0, sizeof(u8) * ctx->f_scu);
#endif
    evc_mset_x64a(ctx->map_cu_mode, 0, sizeof(u32) * ctx->f_scu);
    if(ctx->sh.slice_type == SLICE_I)
    {
        ctx->last_intra_ptr = ctx->ptr;
    }
#if M50662_HISTORY_CTU_ROW_RESET
    evcd_hmvp_init(core);
#else
#if ADMVP
    evc_mset(core->history_buffer.history_mv_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * MV_D * sizeof(s16));
#if TRACE_ENC_CU_DATA
    evc_mset(core->history_buffer.history_cu_table, 0, ALLOWED_CHECKED_NUM * sizeof(core->history_buffer.history_cu_table[0]));
#endif
    
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
    return EVC_OK;
}
#if M50662_HISTORY_CTU_ROW_RESET
static int evcd_hmvp_init(EVCD_CORE * core)
{
    evc_mset(core->history_buffer.history_mv_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * MV_D * sizeof(s16));

    for (int i = 0; i < ALLOWED_CHECKED_NUM; i++)
    {
        core->history_buffer.history_refi_table[i][REFP_0] = REFI_INVALID;
        core->history_buffer.history_refi_table[i][REFP_1] = REFI_INVALID;
    }

    core->history_buffer.currCnt = 0;
    core->history_buffer.m_maxCnt = ALLOWED_CHECKED_NUM;
    return core->history_buffer.currCnt;
}
#endif

int is_ref_pic(EVCD_CTX * ctx, EVC_SH * sh)
{
    return (sh->layer_id == 0 || sh->layer_id < ctx->sps.log2_sub_gop_length);
}

static const s8 poc_offset_from_doc_offset[5][16] =
{
    { 0,  -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { 0,  -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { 0,  -4,   -6,   -2,   -7,   -5,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    { 0,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { 0,  -8,   -12,   -4,  -14,  -10,  -6,   -2,  -15,  -13,  -11,   -9,   -7,   -5,   -3,   -1}   /* gop_size = 16 */
};

int poc_derivation(EVCD_CTX * ctx, EVC_SH * sh)
{
    int sub_gop_length = (int)pow(2.0, ctx->sps.log2_sub_gop_length);
    int expected_temporal_id = 0;
    int doc_offset, poc_offset;
    if (sh->layer_id == 0)
    {
        sh->poc = ctx->prev_pic_order_cnt_val + sub_gop_length;
        ctx->prev_doc_offset = 0;
        ctx->prev_pic_order_cnt_val = sh->poc;
        return EVC_OK;
    }
    doc_offset = (ctx->prev_doc_offset + 1) % sub_gop_length;
    if (doc_offset == 0)
    {
        ctx->prev_pic_order_cnt_val += sub_gop_length;
    }
    else
    {
        expected_temporal_id = 1 + (int)log2(doc_offset);
    }
    while (sh->layer_id != expected_temporal_id)
    {
        doc_offset = (doc_offset + 1) % sub_gop_length;
        if (doc_offset == 0)
        {
            expected_temporal_id = 0;
        }
        else
        {
            expected_temporal_id = 1 + (int)log2(doc_offset);
        }
    }
    //poc_offset = (int)(sub_gop_length * ((2.0 * doc_offset + 1) / (int)pow(2.0, sh->layer_id) - 2));
    poc_offset = poc_offset_from_doc_offset[sub_gop_length >> 2][doc_offset];
    sh->poc = ctx->prev_pic_order_cnt_val + poc_offset;
    ctx->prev_doc_offset = doc_offset;

    return EVC_OK;
}

static void make_stat(EVCD_CTX * ctx, int btype, EVCD_STAT * stat)
{
    int i, j;
    stat->nalu_type = btype;
    stat->stype = 0;
    stat->fnum = -1;
    if(ctx)
    {
        stat->read += EVC_BSR_GET_READ_BYTE(&ctx->bs);
        if(btype < EVC_SPS_NUT)
        {
            stat->fnum = ctx->pic_cnt;
            stat->stype = ctx->sh.slice_type;

            /* increase decoded picture count */
            ctx->pic_cnt++;

            stat->poc = ctx->ptr;

            stat->tid = ctx->sh.layer_id;

            for(i = 0; i < 2; i++)
            {
                stat->refpic_num[i] = ctx->dpm.num_refp[i];
                for(j = 0; j < stat->refpic_num[i]; j++)
                {
                    stat->refpic[i][j] = ctx->refp[j][i].ptr;
                }
            }
        }
    }
}

static void evcd_itdq(EVCD_CTX * ctx, EVCD_CORE * core)
{
    evc_sub_block_itdq(core->coef, core->log2_cuw, core->log2_cuh, core->qp_y, core->qp_u, core->qp_v, core->is_coef, core->is_coef_sub, ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
#if IBC
                       , core->pred_mode == MODE_IBC ? 0 : core->ats_intra_cu
                       , core->pred_mode == MODE_IBC ? 0 : ((core->ats_intra_tu_h << 1) | core->ats_intra_tu_v)
#else
                       , core->ats_intra_cu, ((core->ats_intra_tu_h << 1) | core->ats_intra_tu_v)
#endif
#endif
#if ATS_INTER_PROCESS
#if IBC
                         , core->pred_mode == MODE_IBC ? 0 : core->ats_inter_info
#else
                       , core->ats_inter_info
#endif
#endif
    );
}

#if M48879_IMPROVEMENT_INTRA
static void get_nbr_yuv(int x, int y, int cuw, int cuh, EVCD_CTX * ctx, EVCD_CORE * core)
#else
static void get_nbr_yuv(int x, int y, int cuw, int cuh, u16 avail_cu, EVC_PIC *pic_rec, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 *map_scu, int w_scu, int h_scu)
#endif
{
#if M48879_IMPROVEMENT_INTRA
    int  s_rec;
    pel *rec;
    int constrained_intra_flag = core->pred_mode == MODE_INTRA && ctx->pps.constrained_intra_pred_flag;
#if M50761_CHROMA_NOT_SPLIT
    if (evcd_check_luma(ctx))
    {
#endif
    /* Y */
    s_rec = ctx->pic->s_l;
    rec = ctx->pic->y + (y * s_rec) + x;
    if (ctx->sps.tool_eipd)
    {
        evc_get_nbr(x, y, cuw, cuh, rec, s_rec, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, Y_C, constrained_intra_flag);
    }
    else
    {
        evc_get_nbr_b(x, y, cuw, cuh, rec, s_rec, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, Y_C, constrained_intra_flag);
    }
#if M50761_CHROMA_NOT_SPLIT
    }
    if (evcd_check_chroma(ctx))
    {
#endif
    cuw >>= 1;
    cuh >>= 1;
    x >>= 1;
    y >>= 1;
    s_rec = ctx->pic->s_c;

    /* U */
    rec = ctx->pic->u + (y * s_rec) + x;
    if (ctx->sps.tool_eipd)
    {
        evc_get_nbr(x, y, cuw, cuh, rec, s_rec, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, U_C, constrained_intra_flag);
    }
    else
    {
        evc_get_nbr_b(x, y, cuw, cuh, rec, s_rec, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, U_C, constrained_intra_flag);
    }

    /* V */
    rec = ctx->pic->v + (y * s_rec) + x;
    if (ctx->sps.tool_eipd)
    {
        evc_get_nbr(x, y, cuw, cuh, rec, s_rec, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, V_C, constrained_intra_flag);
    }
    else
    {
        evc_get_nbr_b(x, y, cuw, cuh, rec, s_rec, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, V_C, constrained_intra_flag);
    }
#if M50761_CHROMA_NOT_SPLIT
    }
#endif
#else
    int  s_rec;
    pel *rec;

    /* Y */
    s_rec = pic_rec->s_l;
    rec = pic_rec->y + (y * s_rec) + x;
    evc_get_nbr(x, y, cuw, cuh, rec, s_rec, avail_cu, nb, scup, map_scu, w_scu, h_scu, Y_C);

    cuw >>= 1;
    cuh >>= 1;
    x >>= 1;
    y >>= 1;
    s_rec = pic_rec->s_c;

    /* U */
    rec = pic_rec->u + (y * s_rec) + x;
    evc_get_nbr(x, y, cuw, cuh, rec, s_rec, avail_cu, nb, scup, map_scu, w_scu, h_scu, U_C);

    /* V */
    rec = pic_rec->v + (y * s_rec) + x;
    evc_get_nbr(x, y, cuw, cuh, rec, s_rec, avail_cu, nb, scup, map_scu, w_scu, h_scu, V_C);
#endif
}

#if ADMVP
static void update_history_buffer_parse(EVCD_CORE *core, int slice_type)
{
    int i;
    if(core->history_buffer.currCnt == core->history_buffer.m_maxCnt)
    {
        for(i = 1; i < core->history_buffer.currCnt; i++)
        {
            evc_mcpy(core->history_buffer.history_mv_table[i - 1], core->history_buffer.history_mv_table[i], REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(core->history_buffer.history_refi_table[i - 1], core->history_buffer.history_refi_table[i], REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            core->history_buffer.history_cu_table[i - 1] = core->history_buffer.history_cu_table[i];
#endif
        }

        evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt - 1], core->mv, REFP_NUM * MV_D * sizeof(s16));
        evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt - 1], core->refi, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
        core->history_buffer.history_cu_table[core->history_buffer.currCnt - 1] = core->trace_idx;
#endif
    }
    else
    {
        evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt], core->mv, REFP_NUM * MV_D * sizeof(s16));
        evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt], core->refi, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
        core->history_buffer.history_cu_table[core->history_buffer.currCnt] = core->trace_idx;
#endif

        core->history_buffer.currCnt++;
    }
}

#if AFFINE_UPDATE && AFFINE
static void update_history_buffer_parse_affine(EVCD_CORE *core, int slice_type)
{
    int i;
    if(core->history_buffer.currCnt == core->history_buffer.m_maxCnt)
    {
        for(i = 1; i < core->history_buffer.currCnt; i++)
        {
            evc_mcpy(core->history_buffer.history_mv_table[i - 1], core->history_buffer.history_mv_table[i], REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(core->history_buffer.history_refi_table[i - 1], core->history_buffer.history_refi_table[i], REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            core->history_buffer.history_cu_table[i - 1] = core->history_buffer.history_cu_table[i];
#endif
        }
#if !M49023_ADMVP_IMPROVE || M50662_AFFINE_MV_HISTORY_TABLE
        if(core->affine_flag)
        {
            core->mv_sp[REFP_0][MV_X] = 0;
            core->mv_sp[REFP_0][MV_Y] = 0;
            core->refi_sp[REFP_0] = REFI_INVALID;
            core->mv_sp[REFP_1][MV_X] = 0;
            core->mv_sp[REFP_1][MV_Y] = 0;
            core->refi_sp[REFP_1] = REFI_INVALID;
            for (int lidx = 0; lidx < REFP_NUM; lidx++)
            {
                if (core->refi[lidx] >= 0)
                {
                    s16(*ac_mv)[MV_D] = core->affine_mv[lidx];
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


                    evc_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
                    mv_scale_tmp_hor = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver);


                    mv_scale_tmp_hor >>= 2;
                    mv_scale_tmp_ver >>= 2;

                    core->mv_sp[lidx][MV_X] = mv_scale_tmp_hor;
                    core->mv_sp[lidx][MV_Y] = mv_scale_tmp_ver;
                    core->refi_sp[lidx] = core->refi[lidx];

                }
            }
            // some spatial neighbor may be unavailable
            if((slice_type == SLICE_P && REFI_IS_VALID(core->refi_sp[REFP_0])) ||
                (slice_type == SLICE_B && (REFI_IS_VALID(core->refi_sp[REFP_0]) || REFI_IS_VALID(core->refi_sp[REFP_1]))))
            {
                evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt - 1], core->mv_sp, REFP_NUM * MV_D * sizeof(s16));
                evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt - 1], core->refi_sp, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
                core->history_buffer.history_cu_table[core->history_buffer.currCnt - 1] = core->trace_idx;
#endif
            }
        }
        else
#endif
        {
            evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt - 1], core->mv, REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt - 1], core->refi, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            core->history_buffer.history_cu_table[core->history_buffer.currCnt - 1] = core->trace_idx;
#endif
        }
    }
    else
    {
#if !M49023_ADMVP_IMPROVE || M50662_AFFINE_MV_HISTORY_TABLE
        if(core->affine_flag)
        {
            core->mv_sp[REFP_0][MV_X] = 0;
            core->mv_sp[REFP_0][MV_Y] = 0;
            core->refi_sp[REFP_0] = REFI_INVALID;
            core->mv_sp[REFP_1][MV_X] = 0;
            core->mv_sp[REFP_1][MV_Y] = 0;
            core->refi_sp[REFP_1] = REFI_INVALID;
            for (int lidx = 0; lidx < REFP_NUM; lidx++)
            {
                if (core->refi[lidx] >= 0)
                {
                    s16(*ac_mv)[MV_D] = core->affine_mv[lidx];
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


                    evc_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
                    mv_scale_tmp_hor = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor);
                    mv_scale_tmp_ver = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver);

                    mv_scale_tmp_hor >>= 2;
                    mv_scale_tmp_ver >>= 2;

                    core->mv_sp[lidx][MV_X] = mv_scale_tmp_hor;
                    core->mv_sp[lidx][MV_Y] = mv_scale_tmp_ver;
                    core->refi_sp[lidx] = core->refi[lidx];

                }
            }
            if((slice_type == SLICE_P && REFI_IS_VALID(core->refi_sp[REFP_0])) ||
                (slice_type == SLICE_B && (REFI_IS_VALID(core->refi_sp[REFP_0]) || REFI_IS_VALID(core->refi_sp[REFP_1]))))
            {
                evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt], core->mv_sp, REFP_NUM * MV_D * sizeof(s16));
                evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt], core->refi_sp, REFP_NUM * sizeof(s8));
            }
        }
        else
#endif
        {
            evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt], core->mv, REFP_NUM * MV_D * sizeof(s16));
            evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt], core->refi, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
            core->history_buffer.history_cu_table[core->history_buffer.currCnt] = core->trace_idx;
#endif
        }

        core->history_buffer.currCnt++;
    }
}
#endif
#endif

void evcd_get_direct_motion(EVCD_CTX * ctx, EVCD_CORE * core)
{
    s8            srefi[REFP_NUM][MAX_NUM_MVP];
    s16           smvp[REFP_NUM][MAX_NUM_MVP][MV_D];
    u32           cuw, cuh;

    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);
#if ADMVP
#if M49023_ADMVP_IMPROVE
    if (ctx->sps.tool_admvp == 0)
    {
        evc_get_motion_skip_baseline(ctx->sh.slice_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, srefi, smvp, core->avail_cu
        );
    }
    else
    {
#endif
#endif
#if M49023_ADMVP_IMPROVE
        evc_get_motion_merge_main(ctx->ptr, ctx->sh.slice_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_lr
#if DMVR_LAG
            , ctx->map_unrefined_mv
#endif
#if ADMVP
            , core->history_buffer
#endif
#if IBC
            , core->ibc_flag
#endif
#if M49023_ADMVP_IMPROVE
            , (EVC_REFP(*)[2])ctx->refp[0]
            , &ctx->sh
#endif
#if M50761_TMVP_8X8_GRID
            , ctx->log2_max_cuwh
#endif
        );
    }
#endif

    core->refi[REFP_0] = srefi[REFP_0][core->mvp_idx[REFP_0]];
    core->refi[REFP_1] = srefi[REFP_1][core->mvp_idx[REFP_1]];

    core->mv[REFP_0][MV_X] = smvp[REFP_0][core->mvp_idx[REFP_0]][MV_X];
    core->mv[REFP_0][MV_Y] = smvp[REFP_0][core->mvp_idx[REFP_0]][MV_Y];

    if (ctx->sh.slice_type == SLICE_P)
    {
        core->refi[REFP_1] = REFI_INVALID;
        core->mv[REFP_1][MV_X] = 0;
        core->mv[REFP_1][MV_Y] = 0;
    }
    else
    {
        core->mv[REFP_1][MV_X] = smvp[REFP_1][core->mvp_idx[REFP_1]][MV_X];
        core->mv[REFP_1][MV_Y] = smvp[REFP_1][core->mvp_idx[REFP_1]][MV_Y];
    }
}

void evcd_get_skip_motion(EVCD_CTX * ctx, EVCD_CORE * core)
{
    int REF_SET[3][MAX_NUM_ACTIVE_REF_FRAME] = { {0,0,}, };
    int cuw, cuh, inter_dir = 0;
    s8            srefi[REFP_NUM][MAX_NUM_MVP];
    s16           smvp[REFP_NUM][MAX_NUM_MVP][MV_D];

    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);

    if (ctx->sps.tool_mmvd && core->mmvd_flag)
    {
        evcd_get_mmvd_motion(ctx, core);
    }
    else
    {
        if (ctx->sps.tool_amis == 0)
        {
            evc_get_motion(core->scup, REFP_0, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, core->avail_cu, srefi[REFP_0], smvp[REFP_0]);

            core->refi[REFP_0] = srefi[REFP_0][core->mvp_idx[REFP_0]];
   
            core->mv[REFP_0][MV_X] = smvp[REFP_0][core->mvp_idx[REFP_0]][MV_X];
            core->mv[REFP_0][MV_Y] = smvp[REFP_0][core->mvp_idx[REFP_0]][MV_Y];

            if (ctx->sh.slice_type == SLICE_P)
            {
                core->refi[REFP_1] = REFI_INVALID;
                core->mv[REFP_1][MV_X] = 0;
                core->mv[REFP_1][MV_Y] = 0;
            }
            else
            {
                evc_get_motion(core->scup, REFP_1, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, core->avail_cu, srefi[REFP_1], smvp[REFP_1]);

                core->refi[REFP_1] = srefi[REFP_1][core->mvp_idx[REFP_1]];
                core->mv[REFP_1][MV_X] = smvp[REFP_1][core->mvp_idx[REFP_1]][MV_X];
                core->mv[REFP_1][MV_Y] = smvp[REFP_1][core->mvp_idx[REFP_1]][MV_Y];
            }
        }
        else
        {
            evcd_get_direct_motion(ctx, core);
        }
    }
}

void evcd_get_inter_motion(EVCD_CTX * ctx, EVCD_CORE * core)
{
    int cuw, cuh;
    s16           mvp[MAX_NUM_MVP][MV_D];
    s8            refi[MAX_NUM_MVP];

    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);

    int inter_dir_idx;
    for (inter_dir_idx = 0; inter_dir_idx < 2; inter_dir_idx++)
    {
        /* 0: forward, 1: backward */
        if (((core->inter_dir + 1) >> inter_dir_idx) & 1)
        {
            if (ctx->sps.tool_amis == 0)
            {
                evc_get_motion(core->scup, inter_dir_idx, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, core->avail_cu, refi, mvp);
                core->mv[inter_dir_idx][MV_X] = mvp[core->mvp_idx[inter_dir_idx]][MV_X] + core->mvd[inter_dir_idx][MV_X];
                core->mv[inter_dir_idx][MV_Y] = mvp[core->mvp_idx[inter_dir_idx]][MV_Y] + core->mvd[inter_dir_idx][MV_Y];
            }
            else
            {
                if (core->bi_idx == BI_FL0 || core->bi_idx == BI_FL1)
                {
                    core->refi[inter_dir_idx] = evc_get_first_refi(core->scup, inter_dir_idx, ctx->map_refi, ctx->map_mv, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, core->mvr_idx, core->avail_lr
#if DMVR_LAG
                        , ctx->map_unrefined_mv
#endif
#if ADMVP
                        , core->history_buffer
                        , ctx->sps.tool_admvp
#endif
                    );
                }   

                evc_get_motion_from_mvr(core->mvr_idx, ctx->ptr, core->scup, inter_dir_idx, core->refi[inter_dir_idx], ctx->dpm.num_refp[inter_dir_idx], ctx->map_mv, ctx->map_refi, ctx->refp, \
                    cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mvp, refi, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                    , ctx->map_unrefined_mv
#endif
#if ADMVP
                    , core->history_buffer
                    , ctx->sps.tool_admvp
#endif
                );
                core->mvp_idx[inter_dir_idx] = 0;

                if (core->bi_idx == BI_FL0 + inter_dir_idx)
                {
                    core->mvd[inter_dir_idx][MV_X] = core->mvd[inter_dir_idx][MV_Y] = 0;
                }


                core->mv[inter_dir_idx][MV_X] = mvp[core->mvp_idx[inter_dir_idx]][MV_X] + (core->mvd[inter_dir_idx][MV_X] << core->mvr_idx);
                core->mv[inter_dir_idx][MV_Y] = mvp[core->mvp_idx[inter_dir_idx]][MV_Y] + (core->mvd[inter_dir_idx][MV_Y] << core->mvr_idx);
            }
        }
        else
        {
            core->refi[inter_dir_idx] = REFI_INVALID;
            core->mv[inter_dir_idx][MV_X] = 0;
            core->mv[inter_dir_idx][MV_Y] = 0;
        }
    }
}

#if AFFINE
void evcd_get_affine_motion(EVCD_CTX * ctx, EVCD_CORE * core)
{
#if M49023_ADMVP_IMPROVE
    int          cuw, cuh;
#else
    int          cuw, cuh, k;
#endif
    s16          affine_mvp[MAX_NUM_MVP][VER_NUM][MV_D];
    s8           refi[MAX_NUM_MVP];
    
    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);

    if (core->pred_mode == MODE_SKIP || core->pred_mode == MODE_DIR) // affine merge motion vector
    {
        s16 aff_mrg_mvp[AFF_MAX_CAND][REFP_NUM][VER_NUM][MV_D];
        s8  aff_refi[AFF_MAX_CAND][REFP_NUM];
        int vertex_num[AFF_MAX_CAND];
        int vertex, lidx;
        int mrg_idx = core->mvp_idx[0];

        evc_get_affine_merge_candidate(ctx->ptr, ctx->sh.slice_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, aff_refi, aff_mrg_mvp, vertex_num, ctx->map_scu, ctx->map_affine
            , ctx->log2_max_cuwh
#if DMVR_LAG
            , ctx->map_unrefined_mv
#endif
            , core->avail_lr
#if M50761_TMVP_ALIGN_SPEC || M50662_AFFINE_IBC_TMVP_SUCO_FIX
            , &ctx->sh
#endif
        );

        core->affine_flag = vertex_num[core->mvp_idx[0]] - 1;

        for (lidx = 0; lidx < REFP_NUM; lidx++)
        {
            if (REFI_IS_VALID(aff_refi[mrg_idx][lidx]))
            {
                core->refi[lidx] = aff_refi[mrg_idx][lidx];
                for (vertex = 0; vertex < vertex_num[mrg_idx]; vertex++)
                {
                    core->affine_mv[lidx][vertex][MV_X] = aff_mrg_mvp[mrg_idx][lidx][vertex][MV_X];
                    core->affine_mv[lidx][vertex][MV_Y] = aff_mrg_mvp[mrg_idx][lidx][vertex][MV_Y];
                }
            }
            else
            {
                core->refi[lidx] = REFI_INVALID;
                core->mv[lidx][MV_X] = 0;
                core->mv[lidx][MV_Y] = 0;
            }
        }
    }
    else if (core->pred_mode == MODE_INTER) // affine inter motion vector
    {
        int vertex;
        int vertex_num = core->affine_flag + 1;
        int inter_dir_idx;
        for (inter_dir_idx = 0; inter_dir_idx < 2; inter_dir_idx++)
        {
            /* 0: forward, 1: backward */
            if (((core->inter_dir + 1) >> inter_dir_idx) & 1)
            {
                evc_get_affine_motion_scaling(ctx->ptr, core->scup, inter_dir_idx, core->refi[inter_dir_idx],
                    ctx->dpm.num_refp[inter_dir_idx], ctx->map_mv, ctx->map_refi, ctx->refp, \
                    cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, affine_mvp, refi
                    , ctx->map_scu, ctx->map_affine, vertex_num, core->avail_lr
                    , ctx->log2_max_cuwh
#if DMVR_LAG
                    , ctx->map_unrefined_mv
#endif
                );

                for (vertex = 0; vertex < vertex_num; vertex++)
                {
                    core->affine_mv[inter_dir_idx][vertex][MV_X] = affine_mvp[core->mvp_idx[inter_dir_idx]][vertex][MV_X] + core->affine_mvd[inter_dir_idx][vertex][MV_X];
                    core->affine_mv[inter_dir_idx][vertex][MV_Y] = affine_mvp[core->mvp_idx[inter_dir_idx]][vertex][MV_Y] + core->affine_mvd[inter_dir_idx][vertex][MV_Y];
                    if (vertex == 0)
                    {
                        affine_mvp[core->mvp_idx[inter_dir_idx]][1][MV_X] += core->affine_mvd[inter_dir_idx][vertex][MV_X];
                        affine_mvp[core->mvp_idx[inter_dir_idx]][1][MV_Y] += core->affine_mvd[inter_dir_idx][vertex][MV_Y];
                        affine_mvp[core->mvp_idx[inter_dir_idx]][2][MV_X] += core->affine_mvd[inter_dir_idx][vertex][MV_X];
                        affine_mvp[core->mvp_idx[inter_dir_idx]][2][MV_Y] += core->affine_mvd[inter_dir_idx][vertex][MV_Y];
                    }
                }
            }
            else
            {
                core->refi[inter_dir_idx] = REFI_INVALID;
                for (vertex = 0; vertex < vertex_num; vertex++)
                {
                    core->affine_mv[inter_dir_idx][vertex][MV_X] = 0;
                    core->affine_mv[inter_dir_idx][vertex][MV_Y] = 0;
                }

                core->refi[inter_dir_idx] = REFI_INVALID;
                core->mv[inter_dir_idx][MV_X] = 0;
                core->mv[inter_dir_idx][MV_Y] = 0;
            }
        }
    }
#if !M49023_ADMVP_IMPROVE
#if AFFINE_UPDATE
    core->refi_sp[REFP_0] = REFI_INVALID;
    core->refi_sp[REFP_1] = REFI_INVALID;

    core->mv_sp[REFP_0][MV_X] = 0;
    core->mv_sp[REFP_0][MV_Y] = 0;
    core->mv_sp[REFP_1][MV_X] = 0;
    core->mv_sp[REFP_1][MV_Y] = 0;


    int neb_addr[MAX_NUM_POSSIBLE_SCAND], valid_flag[MAX_NUM_POSSIBLE_SCAND];

    for (k = 0; k < MAX_NUM_POSSIBLE_SCAND; k++)
    {
        valid_flag[k] = 0;
    }
#if ADMVP
    evc_check_motion_availability2(core->scup, cuw, cuh, ctx->w_scu, ctx->h_scu, neb_addr, valid_flag, ctx->map_scu, core->avail_lr, 1
#if IBC
        , 0
#endif  
    );
#else
    evc_check_motion_availability(core->scup, cuw, cuh, ctx->w_scu, ctx->h_scu, neb_addr, valid_flag, ctx->map_scu, core->avail_lr, 1);
#endif

    for (k = 0; k < 5; k++)
    {
        if (valid_flag[k])
        {
            core->refi_sp[REFP_0] = REFI_IS_VALID(ctx->map_refi[neb_addr[k]][REFP_0]) ? ctx->map_refi[neb_addr[k]][REFP_0] : REFI_INVALID;
            core->mv_sp[REFP_0][MV_X] = ctx->map_mv[neb_addr[k]][REFP_0][MV_X];
            core->mv_sp[REFP_0][MV_Y] = ctx->map_mv[neb_addr[k]][REFP_0][MV_Y];

            if (ctx->sh.slice_type == SLICE_B)
            {
                core->refi_sp[REFP_1] = REFI_IS_VALID(ctx->map_refi[neb_addr[k]][REFP_1]) ? ctx->map_refi[neb_addr[k]][REFP_1] : REFI_INVALID;
                core->mv_sp[REFP_1][MV_X] = ctx->map_mv[neb_addr[k]][REFP_1][MV_X];
                core->mv_sp[REFP_1][MV_Y] = ctx->map_mv[neb_addr[k]][REFP_1][MV_Y];
            }

            break;
        }
    }
#endif
#endif
}
#endif

static int evcd_eco_unit(EVCD_CTX * ctx, EVCD_CORE * core, int x, int y, int log2_cuw, int log2_cuh
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int ret, cuw, cuh;
#if M50761_CHROMA_NOT_SPLIT
    ctx->tree_cons = tree_cons;
#endif
    core->log2_cuw = log2_cuw;
    core->log2_cuh = log2_cuh;
    core->x_scu = PEL2SCU(x);
    core->y_scu = PEL2SCU(y);
    core->scup = core->x_scu + core->y_scu * ctx->w_scu;

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ptr: ");
    EVC_TRACE_INT(ctx->ptr);
    EVC_TRACE_STR("x pos ");
    EVC_TRACE_INT(x);
    EVC_TRACE_STR("y pos ");
    EVC_TRACE_INT(y);
    EVC_TRACE_STR("width ");
    EVC_TRACE_INT(cuw);
    EVC_TRACE_STR("height ");
    EVC_TRACE_INT(cuh);
#if M50761_CHROMA_NOT_SPLIT
    EVC_TRACE_STR("tree status ");
    EVC_TRACE_INT(ctx->tree_cons.tree_type);
    EVC_TRACE_STR("mode status ");
    EVC_TRACE_INT(ctx->tree_cons.mode_cons);
#endif
    EVC_TRACE_STR("\n");

#if ATS_INTRA_PROCESS
    core->ats_intra_cu = core->ats_intra_tu_h = core->ats_intra_tu_v = 0;
#endif

    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu);
    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
#if IBC
        , ctx->sps.ibc_flag, ctx->sps.ibc_log_max_size
#endif
    );


    /* parse CU info */
    ret = evcd_eco_cu(ctx, core);
    evc_assert_g(ret == EVC_OK, ERR);

#if TRACE_ENC_CU_DATA
    static int core_counter = 1;
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("RDO check id ");
    EVC_TRACE_INT(core_counter++);
    EVC_TRACE_STR("\n");
#endif

#if TRACE_ENC_HISTORIC
    //if (core->pred_mode != MODE_INTRA)
    {
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("Historic (");
        EVC_TRACE_INT((int)core->history_buffer.currCnt);
        EVC_TRACE_STR("): ");
        for (int i = 0; i < core->history_buffer.currCnt; ++i)
        {
            EVC_TRACE_STR("(");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_0][MV_X]);
            EVC_TRACE_STR(", ");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_0][MV_Y]);
            EVC_TRACE_STR("; ");
            EVC_TRACE_INT((int)core->history_buffer.history_refi_table[i][REFP_0]);
            EVC_TRACE_STR("), (");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_1][MV_X]);
            EVC_TRACE_STR(", ");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_1][MV_Y]);
            EVC_TRACE_STR("; ");
            EVC_TRACE_INT((int)core->history_buffer.history_refi_table[i][REFP_1]);
            EVC_TRACE_STR("); ");
        }
        EVC_TRACE_STR("\n");
    }
#endif
    /* inverse transform and dequantization */
    if(core->pred_mode != MODE_SKIP)
    {
        evcd_itdq(ctx, core);
    }

    evcd_set_dec_info(ctx, core
#if ENC_DEC_TRACE
#if IBC
                      , (core->pred_mode == MODE_INTRA || core->pred_mode == MODE_IBC)
#else
                      , core->pred_mode == MODE_INTRA
#endif
#endif
    );

    /* prediction */
#if IBC
    if (core->pred_mode == MODE_IBC)
    {
        core->avail_cu = evc_get_avail_ibc(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, cuw, cuh, ctx->map_scu);
        evc_IBC_mc(x, y, log2_cuw, log2_cuh, core->mv[0], ctx->pic, core->pred[0]
#if M50761_CHROMA_NOT_SPLIT
            , ctx->tree_cons
#endif
        );
#if M48879_IMPROVEMENT_INTRA 
        get_nbr_yuv(x, y, cuw, cuh, ctx, core);
#else
        get_nbr_yuv(x, y, cuw, cuh, core->avail_cu, ctx->pic, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu);
#endif
    }
    else
#endif
    if(core->pred_mode != MODE_INTRA)
    {    
        core->avail_cu = evc_get_avail_inter(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, cuw, cuh, ctx->map_scu);
#if DMVR
        if (ctx->sps.tool_dmvr)
        {
            core->DMVRenable = 0;

            if (core->pred_mode == MODE_SKIP && !core->mmvd_flag)
                core->DMVRenable = 1;

            if (core->inter_dir == PRED_DIR)
                core->DMVRenable = 1;
#if AFFINE
            if (core->affine_flag)
                core->DMVRenable = 0;
#endif
        }
#endif
    
#if AFFINE
        if(core->affine_flag)
        {
            evcd_get_affine_motion(ctx, core);

            evc_affine_mc(x, y, ctx->w, ctx->h, cuw, cuh, core->refi, core->affine_mv, ctx->refp, core->pred, core->affine_flag + 1
#if EIF
                          , core->eif_tmp_buffer
#endif
            );
        }
        else
        {
#endif
            if (core->pred_mode == MODE_SKIP)
            {
                evcd_get_skip_motion(ctx, core);
            }
            else
            {
                {
                    if (core->inter_dir == PRED_DIR)
                    {
                        if (ctx->sps.tool_amis == 0)
                        {
#if ADMVP
                            s8 refidx;
#endif
                            evc_get_mv_dir(ctx->refp[0], ctx->ptr, core->scup + ((1 << (core->log2_cuw - MIN_CU_LOG2)) - 1) + ((1 << (core->log2_cuh - MIN_CU_LOG2)) - 1) * ctx->w_scu, core->scup, ctx->w_scu, ctx->h_scu, core->mv
#if ADMVP
                                , &refidx
#endif
                                , ctx->sps.tool_admvp
                            );
                            core->refi[REFP_0] = 0;
                            core->refi[REFP_1] = 0;
                        }
                        else if (core->mvr_idx == 0)
                        {
                            evcd_get_direct_motion(ctx, core);
                        }
                    }
                    else if (core->inter_dir == PRED_DIR_MMVD)
                    {
                        evcd_get_mmvd_motion(ctx, core);
                    }
                    else
                    {
                        evcd_get_inter_motion(ctx, core);
                    }
                }
            }
#if M50761_REMOVE_BIBLOCKS_8x4 
            if (!check_bi_applicability_rdo(ctx->sh.slice_type, 1 << core->log2_cuw, 1 << core->log2_cuh) && REFI_IS_VALID(core->refi[REFP_0]) && REFI_IS_VALID(core->refi[REFP_1]))
            {
                evc_assert(process_bi_mv((s16 *)core->mv, core->refi));
            }
#endif
#if DMVR
            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, core->refi, core->mv, ctx->refp, core->pred, ctx->ptr, core->dmvr_template, core->dmvr_ref_pred_interpolated
                   , core->dmvr_half_pred_interpolated
                   , (core->DMVRenable == 1)
#if DMVR
                   && ctx->sps.tool_dmvr
#endif
#if DMVR_PADDING
                   , core->dmvr_padding_buf
#endif
#if DMVR_FLAG
                   , &core->dmvr_flag
#if DMVR_LAG
                   , core->dmvr_mv
#endif
#endif
                   , ctx->sps.tool_amis
#else
            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, core->refi, core->mv, ctx->refp, core->pred
#endif
            );
#if AFFINE
        }
#endif

#if DMVR && HISTORY_LCU_COPY_BUG_FIX
        evcd_set_dec_info(ctx, core
#if ENC_DEC_TRACE
                          , 1
#endif
        );
#endif
#if AFFINE && ADMVP && AFFINE_UPDATE 
#if M49023_ADMVP_IMPROVE && !M50662_AFFINE_MV_HISTORY_TABLE
        if (core->pred_mode != MODE_INTRA && !core->affine_flag
#if IBC
            && core->pred_mode != MODE_IBC
#endif
#if M50761_CHROMA_NOT_SPLIT
            && evcd_check_luma(ctx)
#endif
            )
#else
        if (core->pred_mode != MODE_INTRA
#if IBC
            && core->pred_mode != MODE_IBC
#endif
#if M50761_CHROMA_NOT_SPLIT
            && evcd_check_luma(ctx)
#endif
            )
#endif
        {
            update_history_buffer_parse_affine(core, ctx->sh.slice_type
            );
        }

#endif        // #if AFFINE && ADMVP && AFFINE_UPDATE 
#if DMVR && !HISTORY_LCU_COPY_BUG_FIX
        evcd_set_dec_info(ctx, core
#if ENC_DEC_TRACE
                          , 1
#endif
        );
#endif
    }
    else
    {
        core->avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->log2_cuw, core->log2_cuh, ctx->map_scu);
     
#if M48879_IMPROVEMENT_INTRA
        get_nbr_yuv(x, y, cuw, cuh, ctx, core);
#else
        get_nbr_yuv(x, y, cuw, cuh, core->avail_cu, ctx->pic, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu);
#endif
      
        if (ctx->sps.tool_eipd)
        {
#if M50761_CHROMA_NOT_SPLIT
            if (evcd_check_luma(ctx))
            {
#endif
            evc_ipred(core->nb[0][0] + 2, core->nb[0][1] + cuh, core->nb[0][2] + 2, core->avail_lr, core->pred[0][Y_C], core->ipm[0], cuw, cuh, core->avail_cu
#if M48879_IMPROVEMENT_INTRA
                , ctx->sps.sps_suco_flag
#endif
            );
#if M50761_CHROMA_NOT_SPLIT
            }
            if (evcd_check_chroma(ctx))
            {
#endif
            evc_ipred_uv(core->nb[1][0] + 2, core->nb[1][1] + (cuh >> 1), core->nb[1][2] + 2, core->avail_lr, core->pred[0][U_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu
#if M48879_IMPROVEMENT_INTRA
                , ctx->sps.sps_suco_flag
#endif
            );
            evc_ipred_uv(core->nb[2][0] + 2, core->nb[2][1] + (cuh >> 1), core->nb[2][2] + 2, core->avail_lr, core->pred[0][V_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu
#if M48879_IMPROVEMENT_INTRA
                , ctx->sps.sps_suco_flag
#endif
            );
#if M50761_CHROMA_NOT_SPLIT
            }
#endif
        }
        else
        {
#if M50761_CHROMA_NOT_SPLIT
            if (evcd_check_luma(ctx))
            {
#endif
            evc_ipred_b(core->nb[0][0] + 2, core->nb[0][1] + cuh, core->nb[0][2] + 2, core->avail_lr, core->pred[0][Y_C], core->ipm[0], cuw, cuh, core->avail_cu);
#if M50761_CHROMA_NOT_SPLIT
            }
            if (evcd_check_chroma(ctx))
            {
#endif
            evc_ipred_uv_b(core->nb[1][0] + 2, core->nb[1][1] + (cuh >> 1), core->nb[1][2] + 2, core->avail_lr, core->pred[0][U_C], core->ipm[0], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
            evc_ipred_uv_b(core->nb[2][0] + 2, core->nb[2][1] + (cuh >> 1), core->nb[2][2] + 2, core->avail_lr, core->pred[0][V_C], core->ipm[0], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
#if M50761_CHROMA_NOT_SPLIT
            }
#endif
        }
    }
#if GRAB_STAT
    encd_stat_cu(core->x_scu << MIN_CU_LOG2, core->y_scu << MIN_CU_LOG2, 1 << core->log2_cuw, 1 << core->log2_cuh, core->scup, ctx, core
#if M50761_CHROMA_NOT_SPLIT
        , ctx->tree_cons
#endif
    );
#endif

    /* reconstruction */
    evc_recon_yuv(x, y, cuw, cuh, core->coef, core->pred[0], core->is_coef, ctx->pic
#if ATS_INTER_PROCESS
#if IBC
      , core->pred_mode == MODE_IBC ? 0 : core->ats_inter_info
#else
                  , core->ats_inter_info
#endif
#endif
#if M50761_CHROMA_NOT_SPLIT
        , ctx->tree_cons
#endif
    );
#if IBC
    if (core->pred_mode != MODE_IBC)
    {
#endif
#if HTDF
    if(ctx->sps.tool_htdf == 1 && (core->is_coef[Y_C] || core->pred_mode == MODE_INTRA)
#if M50761_CHROMA_NOT_SPLIT
        && evcd_check_luma(ctx)
#endif
        )
    {
        u16 avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, log2_cuw, log2_cuh, ctx->map_scu);
        evc_htdf(ctx->pic->y + (y * ctx->pic->s_l) + x, ctx->sh.qp, cuw, cuh, ctx->pic->s_l, core->pred_mode == MODE_INTRA
            , ctx->pic->y + (y * ctx->pic->s_l) + x, ctx->pic->s_l, avail_cu);
    }
#endif
#if IBC
    }
#endif
    return EVC_OK;
ERR:
    return ret;
}

static int evcd_eco_tree(EVCD_CTX * ctx, EVCD_CORE * core, int x0, int y0, int log2_cuw, int log2_cuh, int cup, int cud, EVC_BSR * bs, EVCD_SBAC * sbac, int next_split
                         , int parent_suco, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth
#if DQP
                         , int cu_qp_delta_code
#endif
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int ret;
    s8  split_mode;
    int cuw, cuh;
    s8  suco_flag = 0;
    int bound;
    int split_mode_child[4] = {NO_SPLIT, NO_SPLIT, NO_SPLIT, NO_SPLIT};
    int split_allow[6];

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;
#if M50761_CHROMA_NOT_SPLIT
    ctx->tree_cons = tree_cons;
#endif

    if(cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE)
    {
        if(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h)
        {
            if(next_split)
            {
                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("x pos ");
                EVC_TRACE_INT(core->x_pel + ((cup % (ctx->max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
                EVC_TRACE_STR("y pos ");
                EVC_TRACE_INT(core->y_pel + ((cup / (ctx->max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
                EVC_TRACE_STR("width ");
                EVC_TRACE_INT(cuw);
                EVC_TRACE_STR("height ");
                EVC_TRACE_INT(cuh);
                EVC_TRACE_STR("depth ");
                EVC_TRACE_INT(cud);

                split_mode = evcd_eco_split_mode(ctx, bs, sbac, cuw, cuh, parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
            }
            else
            {
                split_mode = NO_SPLIT;
            }
        }
        else
        {
            int boundary = !(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);
            int boundary_b = boundary && (y0 + cuh > ctx->h) && !(x0 + cuw > ctx->w);
            int boundary_r = boundary && (x0 + cuw > ctx->w) && !(y0 + cuh > ctx->h);

#if M50761_BOUNDARY_FORCE_CLEANUP
            split_mode = NO_SPLIT;
            if (ctx->sps.sps_btt_flag)
            {
                if (boundary)
                {
                    if (boundary_r)
                    {
                        // right side
                        if (log2_cuh >= (2 + log2_cuw))
                        {
                            split_mode = SPLIT_BI_HOR;
                        }
                        else
                        {
                            split_mode = SPLIT_BI_VER;
                        }
                    }
                    else
                    {
                        // Bottom and right-bottom corner
                        if (log2_cuw >= (2 + log2_cuh))
                        {
                            split_mode = SPLIT_BI_VER;
                        }
                        else
                        {
                            split_mode = SPLIT_BI_HOR;
                        }
                    }
                }
            }
#endif
            if(cuw == cuh)
            {
                if(!ctx->sps.sps_btt_flag)
                {
                    EVC_TRACE_COUNTER;
                    EVC_TRACE_STR("x pos ");
                    EVC_TRACE_INT(core->x_pel + ((cup % (ctx->max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
                    EVC_TRACE_STR("y pos ");
                    EVC_TRACE_INT(core->y_pel + ((cup / (ctx->max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
                    EVC_TRACE_STR("width ");
                    EVC_TRACE_INT(cuw);
                    EVC_TRACE_STR("height ");
                    EVC_TRACE_INT(cuh);
                    EVC_TRACE_STR("depth ");
                    EVC_TRACE_INT(cud);

                    split_mode = evcd_eco_split_mode(ctx, bs, sbac, cuw, cuh, parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
                }
#if !M50761_BOUNDARY_FORCE_CLEANUP
                else if(boundary_b)
                {
                    split_mode = SPLIT_BI_HOR;
                }
                else if(boundary_r)
                {
                    split_mode = SPLIT_BI_VER;
                }
                else if(!boundary_b && !boundary_r)
                {
                    split_mode = SPLIT_QUAD;
                }
#endif
            }
            else
            {
                if(cuw > cuh)
                {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                    if(boundary_b)
                    {
                        if (log2_cuw == 7 && log2_cuh == 6)
                        {
                            split_mode = SPLIT_BI_VER;
                        }
                        else
                        {
                            split_mode = SPLIT_BI_HOR;
                        }

                    }
                    else if(boundary_r)
                    {
                        split_mode = SPLIT_BI_VER;
                    }
                    else
                    {
                        assert(0);

                    }
#endif
                }
                else
                {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                    if(boundary_b)
                    {
                        split_mode = SPLIT_BI_HOR;
                    }
                    else if(boundary_r)
                    {
                        if (log2_cuw == 6 && log2_cuh == 7)
                        {
                            split_mode = SPLIT_BI_HOR;
                        }
                        else
                        {
                            split_mode = SPLIT_BI_VER;
                        }
                    }
                    else
                    {
                        assert(0);
                    }
#endif
                }
            }
        }
    }
    else
    {
        split_mode = NO_SPLIT;
    }

#if DQP
    if(ctx->sps.sps_btt_flag)
    {
        if(ctx->pps.cu_qp_delta_enabled_flag && ctx->sps.dquant_flag)
        {
            if (split_mode == NO_SPLIT && (log2_cuh + log2_cuw >= ctx->pps.cu_qp_delta_area) && cu_qp_delta_code != 2)
            {
                if (log2_cuh == 7 || log2_cuw == 7)
                {
                    cu_qp_delta_code = 2;
                }
                else
                {
                    cu_qp_delta_code = 1;
                }
                core->cu_qp_delta_is_coded = 0;
            }
            else if ((((split_mode == SPLIT_TRI_VER || split_mode == SPLIT_TRI_HOR) && (log2_cuh + log2_cuw == ctx->pps.cu_qp_delta_area + 1)) ||
                (log2_cuh + log2_cuw == ctx->pps.cu_qp_delta_area && cu_qp_delta_code != 2)))
            {
                cu_qp_delta_code = 2;
                core->cu_qp_delta_is_coded = 0;
            }
        }
    }
#endif

    evc_set_split_mode(split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, core->split_mode);
    same_layer_split[node_idx] = split_mode;

    bound = !(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);

    suco_flag = evcd_eco_suco_flag(bs, sbac, ctx, core, cuw, cuh, split_mode, bound, ctx->log2_max_cuwh, parent_suco);
    evc_set_suco_flag(suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, core->suco_flag);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x0, y0, cuw, cuh, cup, cud, ctx->log2_max_cuwh - MIN_CU_LOG2, &split_struct
#if M50761_CHROMA_NOT_SPLIT
            ,  tree_cons /*, ctx->tgh.tile_group_type */
#endif
        );
#if M50761_CHROMA_NOT_SPLIT
        BOOL mode_cons_changed = evc_signal_mode_cons(&ctx->tree_cons, &split_struct.tree_cons)
#if CHROMA_NOT_SPLIT_EXCLUDE_IBC
            && !ctx->sps.ibc_flag
#endif
            ;
        if (split_mode != SPLIT_QUAD)       // Only for main profile
        {
            if (mode_cons_changed)
            {
                MODE_CONS mode = eOnlyIntra;
                if (ctx->sh.slice_type != SLICE_I && (evc_get_mode_cons_by_split(split_mode, cuw, cuh) == eAll))
                {
                    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
#if IBC
                        , ctx->sps.ibc_flag, ctx->sps.ibc_log_max_size
#endif
                    );

                    mode = evcd_eco_mode_constr(ctx);
                }
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

        evc_split_get_suco_order(evc_split_is_vertical(split_mode) ? suco_flag : 0, split_mode, suco_order);
        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int log2_sub_cuw = split_struct.log_cuw[cur_part_num];
            int log2_sub_cuh = split_struct.log_cuh[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                ret = evcd_eco_tree(ctx, core, x_pos, y_pos, log2_sub_cuw, log2_sub_cuh, split_struct.cup[cur_part_num], split_struct.cud[cur_part_num], bs, sbac, 1
                                    , suco_flag, split_mode, split_mode_child, part_num, split_allow
                                    , INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, bound)
#if DQP
                    , cu_qp_delta_code
#endif
#if M50761_CHROMA_NOT_SPLIT
                                     , split_struct.tree_cons
#endif
                );
                evc_assert_g(ret == EVC_OK, ERR);
            }
#if M50761_CHROMA_NOT_SPLIT
            ctx->tree_cons = tree_cons;
#endif
        }
#if M50761_CHROMA_NOT_SPLIT
        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
        {
            TREE_CONS local_tree_cons = split_struct.tree_cons;
            local_tree_cons.tree_type = TREE_C;
            evc_assert(evc_check_only_intra(split_struct.tree_cons));
            ret = evcd_eco_unit(ctx, core, x0, y0, log2_cuw, log2_cuh
#if M50761_CHROMA_NOT_SPLIT
                , local_tree_cons
#endif
            );
            evc_assert_g(ret == EVC_OK, ERR);
            ctx->tree_cons = tree_cons;
        }
#endif
    }
    else
    {
#if DQP
        core->cu_qp_delta_code = cu_qp_delta_code;
#endif
        ret = evcd_eco_unit(ctx, core, x0, y0, log2_cuw, log2_cuh
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
        evc_assert_g(ret == EVC_OK, ERR);
    }

    return EVC_OK;
ERR:
    return ret;
}

static void deblock_tree(EVCD_CTX * ctx, EVC_PIC * pic, int x, int y, int cuw, int cuh, int cud, int cup, int is_hor
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
    lcu_num = (x >> ctx->log2_max_cuwh) + (y >> ctx->log2_max_cuwh) * ctx->w_lcu;
    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_split[lcu_num]);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_suco[lcu_num]);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x, y, cuw, cuh, cup, cud, ctx->log2_max_cuwh - MIN_CU_LOG2, &split_struct
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons /*, ctx->tgh.tile_group_type */
#endif
        );
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
                MODE_CONS mode = evcd_derive_mode_cons(ctx, PEL2SCU(x) + PEL2SCU(y) * ctx->w_scu);
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

        evc_split_get_suco_order(evc_split_is_vertical(split_mode) ? suco_flag : 0, split_mode, suco_order);
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
                evc_deblock_cu_ver(pic, x,               y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi,
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

int evcd_deblock_h263(EVCD_CTX * ctx)
{
    int i, j;
    u32 k;
#if M49023_DBF_IMPROVE
    ctx->pic->pic_deblock_alpha_offset = ctx->sh.sh_deblock_alpha_offset;
    ctx->pic->pic_deblock_beta_offset = ctx->sh.sh_deblock_beta_offset;
#endif
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
            deblock_tree(ctx, ctx->pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1
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
            deblock_tree(ctx, ctx->pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
            );
        }
    }

    return EVC_OK;
}

#if ALF
int evcd_alf(EVCD_CTX * ctx, EVC_PIC * pic)
{
    AdaptiveLoopFilter* p = (AdaptiveLoopFilter*)(ctx->alf);

#if ALF_PARAMETER_APS
    call_dec_alf_process_aps(p, ctx, pic);
#else
    call_ALFProcess(p, ctx, pic);
#endif

    return EVC_OK;
}
#endif

int evcd_dec_slice(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVC_BSR   *bs;
    EVCD_SBAC *sbac;
    int         ret;
    int         size;

    size = sizeof(s8) * ctx->f_scu * REFP_NUM;
    evc_mset_x64a(ctx->map_refi, -1, size);

    size = sizeof(s16) * ctx->f_scu * REFP_NUM * MV_D;
    evc_mset_x64a(ctx->map_mv, 0, size);

#if DMVR_LAG
    size = sizeof(s16) * ctx->f_scu * REFP_NUM * MV_D;
    evc_mset_x64a(ctx->map_unrefined_mv, 0, size);
#endif

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    /* reset SBAC */
    evcd_eco_sbac_reset(bs, ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);

#if DQP
    /* reset dqp used map data */
    {
        int size;
        size = sizeof(s8) * ctx->f_scu;
        evc_mset_x64a(ctx->map_dqp_used, DQP_UNUSED, size);
    }
#endif
#if DQP
    ctx->sh.qp_prev = ctx->sh.qp;
#endif

#if GRAB_STAT
    evc_stat_set_enc_state(FALSE);
#endif
    while(1)
    {
        int same_layer_split[4];
        int split_allow[6] = {0, 0, 0, 0, 0, 1};
        evc_assert_rv(core->lcu_num < ctx->f_lcu, EVC_ERR_UNEXPECTED);
#if M50662_HISTORY_CTU_ROW_RESET
        if (core->x_pel == 0)
        {
            ret = evcd_hmvp_init(core);
            evc_assert_rv(ret == EVC_OK, ret);
        }
#endif
        /* invoke coding_tree() recursion */
        evc_mset(core->split_mode, 0, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
#if APS_ALF_CTU_FLAG
        evc_AlfSliceParam* alfSliceParam = &(ctx->sh.alf_sh_param);
        if ((alfSliceParam->isCtbAlfOn) && (ctx->sh.alf_on))
        {
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("Usage of ALF: ");
#if ALF_CTU_MAP_DYNAMIC
            *(alfSliceParam->alfCtuEnableFlag + core->lcu_num) = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ctb_alf_flag);
            EVC_TRACE_INT((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)));
#else
            alfSliceParam->alfCtuEnableFlag[0][core->lcu_num] = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ctb_alf_flag);
            EVC_TRACE_INT((int)(alfTileGroupParam->alfCtuEnableFlag[0][core->lcu_num]));
#endif
            EVC_TRACE_STR("\n");
        }
#endif
        ret = evcd_eco_tree(ctx, core, core->x_pel, core->y_pel, ctx->log2_max_cuwh, ctx->log2_max_cuwh, 0, 0, bs, sbac, 1
                            , 0, NO_SPLIT, same_layer_split, 0, split_allow, 0, 0
#if DQP
                            , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                             , evc_get_default_tree_cons()
#endif
        );
        evc_assert_g(EVC_SUCCEEDED(ret), ERR);

        /* set split flags to map */
        evc_mcpy(ctx->map_split[core->lcu_num], core->split_mode, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
        evc_mcpy(ctx->map_suco[core->lcu_num], core->suco_flag, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);

        /* read end_of_picture_flag */
        if(evcd_eco_slice_end_flag(bs, sbac))
        {
            break;
        }

        core->lcu_num++;
        core->x_lcu++;

        if(core->x_lcu == ctx->w_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }

        core->x_pel = core->x_lcu << ctx->log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->log2_max_cuwh;
    }


    return EVC_OK;

ERR:
    return ret;
}

int evcd_ready(EVCD_CTX *ctx)
{
    int ret = EVC_OK;
    EVCD_CORE *core = NULL;

    evc_assert(ctx);

    core = core_alloc();
    evc_assert_gv(core != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

    ctx->core = core;

    return EVC_OK;
ERR:
    if(core)
    {
        core_free(core);
    }

    return ret;
}

void evcd_flush(EVCD_CTX * ctx)
{
    if(ctx->core)
    {
        core_free(ctx->core);
        ctx->core = NULL;
    }
}

int evcd_dec_nalu(EVCD_CTX * ctx, EVC_BITB * bitb, EVCD_STAT * stat)
{
    EVC_BSR  *bs = &ctx->bs;
    EVC_SPS  *sps = &ctx->sps;
    EVC_PPS  *pps = &ctx->pps;
#if ALF_PARAMETER_APS
    EVC_APS  *aps = &ctx->aps;
#endif
    EVC_SH   *sh = &ctx->sh;
    EVC_NALU *nalu = &ctx->nalu;
    int        ret;

    ret = EVC_OK;

    /* set error status */
    ctx->bs_err = bitb->err;
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
    if (sh->slice_type != SLICE_I)
    {
#endif
        EVC_TRACE_SET(1);
#if TRACE_RDO_EXCLUDE_I
    }
    else
    {
        EVC_TRACE_SET(0);
    }
#endif
#endif
#if GRAB_STAT
    evc_stat_set_enc_state(FALSE);
#endif
    /* bitstream reader initialization */
    evc_bsr_init(bs, bitb->addr, bitb->ssize, NULL);
    SET_SBAC_DEC(bs, &ctx->sbac_dec);

    /* parse nalu header */
    ret = evcd_eco_nalu(bs, nalu);
    evc_assert_rv(EVC_SUCCEEDED(ret), ret);
#if ALF_PARAMETER_APS
    ctx->aps_temp = -1;
#endif
    if(nalu->nal_unit_type_plus1 - 1 == EVC_SPS_NUT)
    {
        ret = evcd_eco_sps(bs, sps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        ret = sequence_init(ctx, sps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
#if GRAB_STAT
        enc_stat_header(ctx->w, ctx->h);
#endif

#if ALF
        //TDB: check if should be here
        sh->alf_on = sps->tool_alf;
#endif
#if M48879_IMPROVEMENT_INTER
        sh->mmvd_group_enable_flag = sps->tool_mmvd;
#endif
    }
    else if (nalu->nal_unit_type_plus1 - 1 == EVC_PPS_NUT)
    {
        ret = evcd_eco_pps(bs, sps, pps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
    }
#if ALF_PARAMETER_APS
    else if (nalu->nal_unit_type_plus1 - 1 == EVC_APS_NUT)
    {
#if ALF_CTU_MAP_DYNAMIC
        aps->alf_aps_param.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
        memset(aps->alf_aps_param.alfCtuEnableFlag, 0, N_C * ctx->f_lcu * sizeof(u8));
#endif
        ret = evcd_eco_aps(bs, aps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        aps->alf_aps_param.prevIdx = aps->aps_id;
#if M50662_LUMA_CHROMA_SEPARATE_APS
        aps->alf_aps_param.prevIdxComp[0] = aps->aps_id_y;
        aps->alf_aps_param.prevIdxComp[1] = aps->aps_id_ch;
#endif
        store_dec_aps_to_buffer(ctx);
        ctx->aps_temp = 0;
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
    }
    else if (nalu->nal_unit_type_plus1 - 1 < EVC_SPS_NUT)
#else
    else if (nalu->nalu_type == EVC_CT_SLICE)
#endif
    {
        /* decode slice header */
#if ALF
        sh->num_ctb = ctx->f_lcu;
#endif
#if ALF_CTU_MAP_DYNAMIC
        sh->alf_sh_param.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
        memset(sh->alf_sh_param.alfCtuEnableFlag, 1, N_C * ctx->f_lcu * sizeof(u8));
#endif
        ret = evcd_eco_sh(bs, &ctx->sps, &ctx->pps, sh);

        /* HLS_RPL Test printing the content of RPL0 and RPL1 for each slice*/
/*
        printf("\nCurrent slice POC: %d RPL0 Index: %d RPL1 Index: %d\n", sh->poc, sh->rpl_l0_idx, sh->rpl_l1_idx);
        printf(" Number of ref pics in RPL0: %d Number of active ref pics in RPL0 %d [", sh->rpl_l0.ref_pic_num, sh->rpl_l0.ref_pic_active_num);
        for (int ii = 0; ii < sh->rpl_l0.ref_pic_num; ii++)
        {
            printf("%d ", sh->rpl_l0.ref_pics[ii]);
        }
        printf("]\n");
        printf(" Number of ref pics in RPL1: %d Number of active ref pics in RPL1 %d [", sh->rpl_l1.ref_pic_num, sh->rpl_l1.ref_pic_active_num);
        for (int ii = 0; ii < sh->rpl_l1.ref_pic_num; ii++)
        {
            printf("%d ", sh->rpl_l1.ref_pics[ii]);
        }
        printf("]\n");
*/

        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        ret = slice_init(ctx, ctx->core, sh);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        if(!sps->tool_pocs)
        {
            if (ctx->dtr == 0) // TBD: Check instead if picture is IDR
            {
                sh->poc = 0;
                ctx->prev_doc_offset = 0;
                ctx->prev_pic_order_cnt_val = sh->poc;
                ctx->slice_ref_flag = is_ref_pic(ctx, sh);
            }
            else
            {
                ctx->slice_ref_flag = is_ref_pic(ctx, sh);
                poc_derivation(ctx, sh);
            }
        }
        else
        {
            ctx->slice_ref_flag = 1;
        }
#if TRACE_START_POC
        if (ctx->ptr == TRACE_START_POC)
        {
            fp_trace_started = 1;
            EVC_TRACE_SET(1);
        }
#endif
#if GRAB_STAT
        evc_stat_set_poc(ctx->sh.poc);
#endif

        if (!sps->tool_rpl)
        {
            /* initialize reference pictures */
            ret = evc_picman_refp_init(&ctx->dpm, ctx->sps.max_num_ref_pics, sh->slice_type, ctx->ptr, ctx->sh.layer_id, ctx->last_intra_ptr, ctx->refp);
        }
        else
        {
            /* reference picture marking */
            ret = evc_picman_refpic_marking(&ctx->dpm, sh);
            evc_assert_rv(ret == EVC_OK, ret);

            /* reference picture lists construction */
            ret = evc_picman_refp_rpl_based_init(&ctx->dpm, sh, ctx->refp);
        }
        evc_assert_rv(ret == EVC_OK, ret);

        /* get available frame buffer for decoded image */
        ctx->pic = evc_picman_get_empty_pic(&ctx->dpm, &ret);
        evc_assert_rv(ctx->pic, ret);

        /* get available frame buffer for decoded image */
        ctx->map_refi = ctx->pic->map_refi;
        ctx->map_mv = ctx->pic->map_mv;
#if DMVR_LAG
        ctx->map_unrefined_mv = ctx->pic->map_unrefined_mv;
#endif
        /* decode slice layer */
        ret = ctx->fn_dec_slice(ctx, ctx->core);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        /* deblocking filter */
        if(ctx->sh.deblocking_filter_on)
        {
            ret = ctx->fn_deblock(ctx);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        }


#if ALF
        /* adaptive loop filter */
        if( ctx->sh.alf_on )
        {
            ret = ctx->fn_alf(ctx,  ctx->pic);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        }
#endif

#if USE_DRAW_PARTITION_DEC
        evcd_draw_partition(ctx, ctx->pic);
#endif

#if PIC_PAD_SIZE_L > 0
        /* expand pixels to padding area */
        ctx->fn_picbuf_expand(ctx, ctx->pic);
#endif

        /* put decoded picture to DPB */
        ret = evc_picman_put_pic(&ctx->dpm, ctx->pic, ctx->nalu.nal_unit_type_plus1 - 1 == EVC_IDR_NUT, ctx->ptr, ctx->dtr, ctx->sh.layer_id, 1, ctx->refp, ctx->slice_ref_flag, sps->tool_rpl, ctx->ref_pic_gap_length);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        slice_deinit(ctx);
    }
    else if (nalu->nal_unit_type_plus1 - 1 == EVC_SEI_NUT)
    {
        if (ctx->use_pic_sign && ctx->pic_sign_exist)
        {
            ret = evcd_picbuf_check_signature(ctx->pic, ctx->pic_sign);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);

            ctx->pic_sign_exist = 0;
        }
    }
    else
    {
        assert(!"wrong NALU type");
    }
    
    make_stat(ctx, nalu->nal_unit_type_plus1 - 1, stat);

    return ret;
}

int evcd_pull_frm(EVCD_CTX *ctx, EVC_IMGB **imgb)
{
    int ret;
    EVC_PIC *pic;

    *imgb = NULL;

    pic = evc_picman_out_pic(&ctx->dpm, &ret);

    if(pic)
    {
        evc_assert_rv(pic->imgb != NULL, EVC_ERR);

        /* increase reference count */
        pic->imgb->addref(pic->imgb);
        *imgb = pic->imgb;
    }
    return ret;
}

int evcd_platform_init(EVCD_CTX *ctx)
{
    ctx->fn_ready         = evcd_ready;
    ctx->fn_flush         = evcd_flush;
    ctx->fn_dec_cnk       = evcd_dec_nalu;
    ctx->fn_dec_slice     = evcd_dec_slice;
    ctx->fn_pull          = evcd_pull_frm;
    ctx->fn_deblock       = evcd_deblock_h263;
    ctx->fn_picbuf_expand = evcd_picbuf_expand;
    ctx->pf               = NULL;

    return EVC_OK;
}

void evcd_platform_deinit(EVCD_CTX * ctx)
{
    evc_assert(ctx->pf == NULL);

    ctx->fn_ready         = NULL;
    ctx->fn_flush         = NULL;
    ctx->fn_dec_cnk       = NULL;
    ctx->fn_dec_slice     = NULL;
    ctx->fn_pull          = NULL;
    ctx->fn_deblock       = NULL;
#if ALF
    AdaptiveLoopFilter* p = (AdaptiveLoopFilter*)(ctx->alf);
    if (p!=NULL)
    {
        call_destroy_ALF(p);
    }
    if (ctx->alf != NULL)
    {
        delete_ALF(ctx->alf);
        ctx->fn_alf = NULL;
    }
#endif
    ctx->fn_picbuf_expand = NULL;
}

EVCD evcd_create(EVCD_CDSC * cdsc, int * err)
{
    EVCD_CTX *ctx = NULL;
    int ret;

#if ENC_DEC_TRACE
    fp_trace = fopen("dec_trace.txt", "w+");
#endif
#if GRAB_STAT
    evc_stat_init("dec_stat.vtmbmsstats", esu_only_enc, 0, -1, encd_stat_cu);
#endif

    ctx = ctx_alloc();
    evc_assert_gv(ctx != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mcpy(&ctx->cdsc, cdsc, sizeof(EVCD_CDSC));

    /* additional initialization for each platform, if needed */
    ret = evcd_platform_init(ctx);
    evc_assert_g(ret == EVC_OK, ERR);

    ret = evc_scan_tbl_init();
    evc_assert_g(ret == EVC_OK, ERR);

    if(ctx->fn_ready)
    {
        ret = ctx->fn_ready(ctx);
        evc_assert_g(ret == EVC_OK, ERR);
    }

#if ALF
    ctx->fn_alf           = evcd_alf;
#endif

    /* Set CTX variables to default value */
    ctx->magic = EVCD_MAGIC_CODE;
    ctx->id = (EVCD)ctx;

#if ATS_INTRA_PROCESS
    evc_init_multi_tbl();
    evc_init_multi_inv_tbl();
#endif

    return (ctx->id);
ERR:
    if(ctx)
    {
        if(ctx->fn_flush) ctx->fn_flush(ctx);
        evcd_platform_deinit(ctx);
        ctx_free(ctx);
    }

    if(err) *err = ret;

    return NULL;
}

void evcd_delete(EVCD id)
{
    EVCD_CTX *ctx;

    EVCD_ID_TO_CTX_R(id, ctx);

#if ENC_DEC_TRACE
    fclose(fp_trace);
#endif

#if GRAB_STAT
    evc_stat_finish();
#endif

#if NS_MEMORY_LEAK_FIX
    free_ns_dec();
#endif

    sequence_deinit(ctx);

    if(ctx->fn_flush) ctx->fn_flush(ctx);

    /* addtional deinitialization for each platform, if needed */
    evcd_platform_deinit(ctx);

    ctx_free(ctx);

    evc_scan_tbl_delete();
}

int evcd_config(EVCD id, int cfg, void * buf, int * size)
{
    EVCD_CTX *ctx;

    EVCD_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);

    switch(cfg)
    {
        /* set config ************************************************************/
        case EVCD_CFG_SET_USE_PIC_SIGNATURE:
            ctx->use_pic_sign = (*((int *)buf)) ? 1 : 0;
            break;

        /* get config ************************************************************/
        default:
            evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
    }
    return EVC_OK;
}

int evcd_decode(EVCD id, EVC_BITB * bitb, EVCD_STAT * stat)
{
    EVCD_CTX *ctx;

    EVCD_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_dec_cnk, EVC_ERR_UNEXPECTED);

    return ctx->fn_dec_cnk(ctx, bitb, stat);
}

int evcd_pull(EVCD id, EVC_IMGB ** img)
{
    EVCD_CTX *ctx;

    EVCD_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_pull, EVC_ERR_UNKNOWN);

    return ctx->fn_pull(ctx, img);
}
