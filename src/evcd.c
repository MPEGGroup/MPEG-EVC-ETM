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

    /* initialize reference picture manager */
    ctx->pa.fn_alloc = evcd_picbuf_alloc;
    ctx->pa.fn_free = evcd_picbuf_free;
    ctx->pa.w = ctx->w;
    ctx->pa.h = ctx->h;
    ctx->pa.pad_l = PIC_PAD_SIZE_L;
    ctx->pa.pad_c = PIC_PAD_SIZE_C;
#if HLS_M47668
    ctx->ref_pic_gap_length = (int)pow(2.0, sps->log2_ref_pic_gap_length);
#endif

    ret = evc_picman_init(&ctx->dpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, sps->closed_gop, &ctx->pa);
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

static void tile_group_deinit(EVCD_CTX * ctx)
{
}

static int tile_group_init(EVCD_CTX * ctx, EVCD_CORE * core, EVC_TGH * tgh)
{
    core->lcu_num = 0;
    core->x_lcu = 0;
    core->y_lcu = 0;
    core->x_pel = 0;
    core->y_pel = 0;
    core->qp_y = tgh->qp + 6 * (BIT_DEPTH - 8);
    core->qp_u = evc_tbl_qp_chroma_ajudst[tgh->qp_u] + 6 * (BIT_DEPTH - 8);
    core->qp_v = evc_tbl_qp_chroma_ajudst[tgh->qp_v] + 6 * (BIT_DEPTH - 8);

    ctx->dtr_prev_low = tgh->dtr;
    ctx->dtr = tgh->dtr;
    ctx->ptr = tgh->dptr + tgh->dtr; /* PTR */

    /* clear maps */
    evc_mset_x64a(ctx->map_scu, 0, sizeof(u32) * ctx->f_scu);
#if AFFINE
    evc_mset_x64a(ctx->map_affine, 0, sizeof(u32) * ctx->f_scu);
#endif
#if ATS_INTER_PROCESS
    evc_mset_x64a(ctx->map_ats_inter, 0, sizeof(u8) * ctx->f_scu);
#endif
    evc_mset_x64a(ctx->map_cu_mode, 0, sizeof(u32) * ctx->f_scu);
    if(ctx->tgh.tile_group_type == TILE_GROUP_I)
    {
        ctx->last_intra_ptr = ctx->ptr;
    }

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

    return EVC_OK;
}

#if HLS_M47668
int is_ref_pic(EVCD_CTX * ctx, EVC_TGH * tgh)
{
    return (tgh->layer_id == 0 || tgh->layer_id < ctx->sps.log2_sub_gop_length);
}

static const s8 poc_offset_from_doc_offset[5][16] =
{
    { 0,  -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { 0,  -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { 0,  -4,   -6,   -2,   -7,   -5,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    { 0,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { 0,  -8,   -12,   -4,  -14,  -10,  -6,   -2,  -15,  -13,  -11,   -9,   -7,   -5,   -3,   -1}   /* gop_size = 16 */
};

int poc_derivation(EVCD_CTX * ctx, EVC_TGH * tgh)
{
    int sub_gop_length = (int)pow(2.0, ctx->sps.log2_sub_gop_length);
    int expected_temporal_id = 0;
    int doc_offset, poc_offset;
    if (tgh->layer_id == 0)
    {
        tgh->poc = ctx->prev_pic_order_cnt_val + sub_gop_length;
        ctx->prev_doc_offset = 0;
        ctx->prev_pic_order_cnt_val = tgh->poc;
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
    while (tgh->layer_id != expected_temporal_id)
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
    //poc_offset = (int)(sub_gop_length * ((2.0 * doc_offset + 1) / (int)pow(2.0, tgh->layer_id) - 2));
    poc_offset = poc_offset_from_doc_offset[sub_gop_length >> 2][doc_offset];
    tgh->poc = ctx->prev_pic_order_cnt_val + poc_offset;
    ctx->prev_doc_offset = doc_offset;

    return EVC_OK;
}
#endif

static void make_stat(EVCD_CTX * ctx, int btype, EVCD_STAT * stat)
{
    int i, j;
    stat->read = 0;
    stat->ctype = btype;
    stat->stype = 0;
    stat->fnum = -1;
    if(ctx)
    {
        stat->read = EVC_BSR_GET_READ_BYTE(&ctx->bs);
        if(btype == EVC_CT_TILE_GROUP)
        {
            stat->fnum = ctx->pic_cnt;
            stat->stype = ctx->tgh.tile_group_type;

            /* increase decoded picture count */
            ctx->pic_cnt++;

            stat->poc = ctx->ptr;

            stat->tid = ctx->tgh.layer_id;

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
#if USE_IBC
                       , core->pred_mode == MODE_IBC ? 0 : core->ats_intra_cu
                       , core->pred_mode == MODE_IBC ? 0 : ((core->ats_intra_tu_h << 1) | core->ats_intra_tu_v)
#else
                       , core->ats_intra_cu, ((core->ats_intra_tu_h << 1) | core->ats_intra_tu_v)
#endif
#endif
#if ATS_INTER_PROCESS
#if USE_IBC
                         , core->pred_mode == MODE_IBC ? 0 : core->ats_inter_info
#else
                       , core->ats_inter_info
#endif
#endif
    );
}

static void get_nbr_yuv(int x, int y, int cuw, int cuh, u16 avail_cu, EVC_PIC *pic_rec, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 *map_scu, int w_scu, int h_scu)
{
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
}

#if ADMVP
static void update_history_buffer_parse(EVCD_CORE *core, int tile_group_type)
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
static void update_history_buffer_parse_affine(EVCD_CORE *core, int tile_group_type)
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

        if(core->affine_flag)
        {
            // some spatial neighbor may be unavailable
            if((tile_group_type == TILE_GROUP_P && REFI_IS_VALID(core->refi_sp[REFP_0])) ||
                (tile_group_type == TILE_GROUP_B && (REFI_IS_VALID(core->refi_sp[REFP_0]) || REFI_IS_VALID(core->refi_sp[REFP_1]))))
            {
                evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt - 1], core->mv_sp, REFP_NUM * MV_D * sizeof(s16));
                evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt - 1], core->refi_sp, REFP_NUM * sizeof(s8));
#if TRACE_ENC_CU_DATA
                core->history_buffer.history_cu_table[core->history_buffer.currCnt - 1] = core->trace_idx;
#endif
            }
        }
        else
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
        if(core->affine_flag)
        {
            if((tile_group_type == TILE_GROUP_P && REFI_IS_VALID(core->refi_sp[REFP_0])) ||
                (tile_group_type == TILE_GROUP_B && (REFI_IS_VALID(core->refi_sp[REFP_0]) || REFI_IS_VALID(core->refi_sp[REFP_1]))))
            {
                evc_mcpy(core->history_buffer.history_mv_table[core->history_buffer.currCnt], core->mv_sp, REFP_NUM * MV_D * sizeof(s16));
                evc_mcpy(core->history_buffer.history_refi_table[core->history_buffer.currCnt], core->refi_sp, REFP_NUM * sizeof(s8));
            }
        }
        else
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

static int evcd_eco_unit(EVCD_CTX * ctx, EVCD_CORE * core, int x, int y, int log2_cuw, int log2_cuh)
{
    int ret, cuw, cuh;
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
    EVC_TRACE_STR("\n");

#if ATS_INTRA_PROCESS
    core->ats_intra_cu = core->ats_intra_tu_h = core->ats_intra_tu_v = 0;
#endif

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
                      , core->pred_mode == MODE_INTRA
#endif
    );

    /* prediction */
#if USE_IBC
    if (core->pred_mode == MODE_IBC)
    {
      evc_IBC_mc(x, y, log2_cuw, log2_cuh, core->mv[0], ctx->pic, core->pred[0]);
      get_nbr_yuv(x, y, cuw, cuh, core->avail_cu, ctx->pic, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu);
    }
    else
#endif
    if(core->pred_mode != MODE_INTRA)
    {
#if AFFINE
        if(core->affine_flag)
        {
            evc_affine_mc(x, y, ctx->w, ctx->h, cuw, cuh, core->refi, core->affine_mv, ctx->refp, core->pred, core->affine_flag + 1
#if EIF
                          , core->eif_tmp_buffer
#endif
            );
        }
        else
        {
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
        if(core->pred_mode != MODE_INTRA
#if USE_IBC
          && core->pred_mode != MODE_IBC
#endif
          )
        {
            update_history_buffer_parse_affine(core, ctx->tgh.tile_group_type
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
        get_nbr_yuv(x, y, cuw, cuh, core->avail_cu, ctx->pic, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu);
    }
    else
    {
        get_nbr_yuv(x, y, cuw, cuh, core->avail_cu, ctx->pic, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu);

        if (ctx->sps.tool_eipd)
        {
            evc_ipred(core->nb[0][0] + 2, core->nb[0][1] + cuh, core->nb[0][2] + 2, core->avail_lr, core->pred[0][Y_C], core->ipm[0], cuw, cuh, core->avail_cu);
            evc_ipred_uv(core->nb[1][0] + 2, core->nb[1][1] + (cuh >> 1), core->nb[1][2] + 2, core->avail_lr, core->pred[0][U_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
            evc_ipred_uv(core->nb[2][0] + 2, core->nb[2][1] + (cuh >> 1), core->nb[2][2] + 2, core->avail_lr, core->pred[0][V_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
        }
        else
        {
            evc_ipred_b(core->nb[0][0] + 2, core->nb[0][1] + cuh, core->nb[0][2] + 2, core->avail_lr, core->pred[0][Y_C], core->ipm[0], cuw, cuh, core->avail_cu);
            evc_ipred_uv_b(core->nb[1][0] + 2, core->nb[1][1] + (cuh >> 1), core->nb[1][2] + 2, core->avail_lr, core->pred[0][U_C], core->ipm[0], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
            evc_ipred_uv_b(core->nb[2][0] + 2, core->nb[2][1] + (cuh >> 1), core->nb[2][2] + 2, core->avail_lr, core->pred[0][V_C], core->ipm[0], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
        }
    }

    /* reconstruction */
    evc_recon_yuv(x, y, cuw, cuh, core->coef, core->pred[0], core->is_coef, ctx->pic
#if ATS_INTER_PROCESS
#if USE_IBC
      , core->pred_mode == MODE_IBC ? 0 : core->ats_inter_info
#else
                  , core->ats_inter_info
#endif
#endif
    );
#if USE_IBC
    if (core->pred_mode != MODE_IBC)
    {
#endif
#if HTDF
    if(ctx->sps.tool_htdf == 1 && (core->is_coef[Y_C]
#if HTDF_CBF0_INTRA
                              || core->pred_mode == MODE_INTRA
#endif
                              )
       )
    {
        evc_htdf(ctx->pic->y + (y * ctx->pic->s_l) + x, ctx->tgh.qp, cuw, cuh, ctx->pic->s_l, core->pred_mode == MODE_INTRA, core->nb[0][0] + 2, core->nb[0][1] + cuh - 1, core->nb[0][2] + 2, core->avail_cu);
    }
#endif
#if USE_IBC
    }
#endif
    return EVC_OK;
ERR:
    return ret;
}

static int evcd_eco_tree(EVCD_CTX * ctx, EVCD_CORE * core, int x0, int y0, int log2_cuw, int log2_cuh, int cup, int cud, EVC_BSR * bs, EVCD_SBAC * sbac, int next_split
                         , int parent_suco, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth)
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
                else if(boundary_b)
                {
                    if(cuw == 128 && cuh == 128)
                        split_mode = SPLIT_BI_VER;
                    else
                        split_mode = SPLIT_BI_HOR;
                }
                else if(boundary_r)
                {
                    if(cuw == 128 && cuh == 128)
                        split_mode = SPLIT_BI_HOR;
                    else
                        split_mode = SPLIT_BI_VER;
                }
                else if(boundary && !boundary_b && !boundary_r)
                {
                    split_mode = SPLIT_QUAD;
                }
            }
            else
            {
                if(cuw > cuh)
                {
                    if(boundary)
                    {
                        if(boundary_b)
                        {
                            split_mode = SPLIT_BI_HOR;
                        }
                        else if(boundary_r)
                        {
                            split_mode = SPLIT_BI_VER;
                        }
                        else
                        {
                            split_mode = SPLIT_QUAD;
                        }
                    }
                    else
                    {
                        split_mode = evcd_eco_split_mode(ctx, bs, sbac, cuw, cuh, parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
                    }
                }
                else
                {
                    if(boundary)
                    {
                        if(boundary_b)
                        {
                            split_mode = SPLIT_BI_HOR;
                        }
                        else if(boundary_r)
                        {
                            split_mode = SPLIT_BI_VER;
                        }
                        else
                        {
                            split_mode = SPLIT_QUAD;
                        }
                    }
                    else
                    {
                        split_mode = evcd_eco_split_mode(ctx, bs, sbac, cuw, cuh, parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
                    }
                }
            }
        }
    }
    else
    {
        split_mode = NO_SPLIT;
    }

    evc_set_split_mode(split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, core->split_mode);
    same_layer_split[node_idx] = split_mode;

    bound = !(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);

    suco_flag = evcd_eco_suco_flag(bs, sbac, ctx, core, cuw, cuh, split_mode, bound, ctx->log2_max_cuwh, parent_suco);
    evc_set_suco_flag(suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, core->suco_flag);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x0, y0, cuw, cuh, cup, cud, ctx->log2_max_cuwh - MIN_CU_LOG2, &split_struct);
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
                                    , INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, bound));
                evc_assert_g(ret == EVC_OK, ERR);
            }
        }
    }
    else
    {
        ret = evcd_eco_unit(ctx, core, x0, y0, log2_cuw, log2_cuh);
        evc_assert_g(ret == EVC_OK, ERR);
    }

    return EVC_OK;
ERR:
    return ret;
}

static void deblock_tree(EVCD_CTX * ctx, EVC_PIC * pic, int x, int y, int cuw, int cuh, int cud, int cup, int is_hor)
{
    s8  split_mode;
    int lcu_num;
    s8  suco_flag = 0;

    lcu_num = (x >> ctx->log2_max_cuwh) + (y >> ctx->log2_max_cuwh) * ctx->w_lcu;
    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_split[lcu_num]);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_suco[lcu_num]);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x, y, cuw, cuh, cup, cud, ctx->log2_max_cuwh - MIN_CU_LOG2, &split_struct);
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
                deblock_tree(ctx, pic, x_pos, y_pos, sub_cuw, sub_cuh, split_struct.cud[cur_part_num], split_struct.cup[cur_part_num], is_hor);
            }
        }
    }
    else
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
                evc_deblock_cu_hor(pic, x, y              , cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
                evc_deblock_cu_hor(pic, x, y + MAX_TR_SIZE, cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
            }
            else
            {
                evc_deblock_cu_hor(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
#if ATS_INTER_PROCESS // deblock
                if (ats_inter_idx && is_ats_inter_horizontal(ats_inter_idx))
                {
                    int y_offset = is_ats_inter_quad_size(ats_inter_idx) ? cuh / 4 : cuh / 2;
                    y_offset = ats_inter_pos == 0 ? y_offset : cuh - y_offset;
                    if ((y + y_offset) % 8 == 0)
                    {
                        evc_deblock_cu_hor(pic, x, y + y_offset, cuw, cuh - y_offset, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
                    }
                }
#endif
            }
        }
        else
        {
            if (cuw > MAX_TR_SIZE)
            {
                evc_deblock_cu_ver(pic, x,               y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                    , ctx->map_cu_mode
#endif
                    , ctx->refp
                );
                evc_deblock_cu_ver(pic, x + MAX_TR_SIZE, y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                    , ctx->map_cu_mode
#endif
                    , ctx->refp
                );
            }
            else
            {
                evc_deblock_cu_ver(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                                   , ctx->map_cu_mode
#endif
                                   , ctx->refp
                );
#if ATS_INTER_PROCESS // deblock
                if (ats_inter_idx && !is_ats_inter_horizontal(ats_inter_idx))
                {
                    int x_offset = is_ats_inter_quad_size(ats_inter_idx) ? cuw / 4 : cuw / 2;
                    x_offset = ats_inter_pos == 0 ? x_offset : cuw - x_offset;
                    if ((x + x_offset) % 8 == 0)
                    {
                        evc_deblock_cu_ver(pic, x + x_offset, y, cuw - x_offset, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                                           , ctx->map_cu_mode
#endif
                                           , ctx->refp
                        );
                    }
                }
#endif
            }
        }
    }
}

int evcd_deblock_h263(EVCD_CTX * ctx)
{
    int i, j;
    u32 k;

    for(k = 0; k < ctx->f_scu; k++)
    {
        MCU_CLR_COD(ctx->map_scu[k]);
    }

    /* horizontal filtering */
    for(j = 0; j < ctx->h_lcu; j++)
    {
        for(i = 0; i < ctx->w_lcu; i++)
        {
            deblock_tree(ctx, ctx->pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1);
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
            deblock_tree(ctx, ctx->pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0);
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

int evcd_dec_tile_group(EVCD_CTX * ctx, EVCD_CORE * core)
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
    evcd_eco_sbac_reset(bs, ctx->tgh.tile_group_type, ctx->tgh.qp, ctx->sps.tool_cm_init);

    while(1)
    {
        int same_layer_split[4];
        int split_allow[6] = {0, 0, 0, 0, 0, 1};
        evc_assert_rv(core->lcu_num < ctx->f_lcu, EVC_ERR_UNEXPECTED);

        /* invoke coding_tree() recursion */
        evc_mset(core->split_mode, 0, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
#if APS_ALF_CTU_FLAG
        evc_AlfTileGroupParam* alfTileGroupParam = &(ctx->tgh.alf_tgh_param);
        if ((alfTileGroupParam->isCtbAlfOn) && (ctx->tgh.alf_on))
        {
#if ALF_CTU_MAP_DYNAMIC
            *(alfTileGroupParam->alfCtuEnableFlag + core->lcu_num) = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ctb_alf_flag);
#else
            alfTileGroupParam->alfCtuEnableFlag[0][core->lcu_num] = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ctb_alf_flag);
#endif
        }
#endif
        ret = evcd_eco_tree(ctx, core, core->x_pel, core->y_pel, ctx->log2_max_cuwh, ctx->log2_max_cuwh, 0, 0, bs, sbac, 1
                            , 0, NO_SPLIT, same_layer_split, 0, split_allow, 0, 0);
        evc_assert_g(EVC_SUCCEEDED(ret), ERR);

        /* set split flags to map */
        evc_mcpy(ctx->map_split[core->lcu_num], core->split_mode, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
        evc_mcpy(ctx->map_suco[core->lcu_num], core->suco_flag, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);

        /* read end_of_picture_flag */
        if(evcd_eco_tile_group_end_flag(bs, sbac))
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

    /* parse user data */
    if(ctx->tgh.udata_exist)
    {
        ret = evcd_eco_udata(ctx, bs);
        evc_assert_g(EVC_SUCCEEDED(ret), ERR);
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

int evcd_dec_cnk(EVCD_CTX * ctx, EVC_BITB * bitb, EVCD_STAT * stat)
{
    EVC_BSR  *bs = &ctx->bs;
    EVC_SPS  *sps = &ctx->sps;
    EVC_PPS  *pps = &ctx->pps;
#if ALF_PARAMETER_APS
    EVC_APS  *aps = &ctx->aps;
#endif
    EVC_TGH   *tgh = &ctx->tgh;
    EVC_CNKH *cnkh = &ctx->cnkh;
    int        ret;

    ret = EVC_OK;

    if(stat)
    {
        evc_mset(stat, 0, sizeof(EVCD_STAT));
    }

    /* set error status */
    ctx->bs_err = bitb->err;
#if TRACE_RDO_EXCLUDE_I
    if (tgh->tile_group_type != TILE_GROUP_I)
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
    /* bitstream reader initialization */
    evc_bsr_init(bs, bitb->addr, bitb->ssize, NULL);
    SET_SBAC_DEC(bs, &ctx->sbac_dec);

    /* parse chunk header */
    ret = evcd_eco_cnkh(bs, cnkh);
    evc_assert_rv(EVC_SUCCEEDED(ret), ret);
    /* check evc version */
    evc_assert_rv(cnkh->ver == EVC_VER_1, EVC_ERR_UNSUPPORTED);
#if ALF_PARAMETER_APS
    ctx->aps_temp = -1;
#endif
    if(cnkh->ctype == EVC_CT_SPS)
    {
        ret = evcd_eco_sps(bs, sps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        ret = sequence_init(ctx, sps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        ret = evcd_eco_pps(bs, sps, pps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

#if ALF
        //TDB: check if should be here
        tgh->alf_on = sps->tool_alf;
#endif
    }
#if ALF_PARAMETER_APS
    else if (cnkh->ctype == EVC_CT_APS)
    {
#if ALF_CTU_MAP_DYNAMIC
        aps->alf_aps_param.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
        memset(aps->alf_aps_param.alfCtuEnableFlag, 0, N_C * ctx->f_lcu * sizeof(u8));
#endif
        ret = evcd_eco_aps(bs, aps);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        aps->alf_aps_param.prevIdx = aps->aps_id;
        store_dec_aps_to_buffer(ctx);
        ctx->aps_temp = 0;

        /* parse chunk header */
        ret = evcd_eco_cnkh(bs, cnkh);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        /* check evc version */
        evc_assert_rv(cnkh->ver == EVC_VER_1, EVC_ERR_UNSUPPORTED);
    }
    if (cnkh->ctype == EVC_CT_TILE_GROUP)
#else
    else if (cnkh->ctype == EVC_CT_TILE_GROUP)
#endif
    {
        /* decode tile_group header */
#if ALF
        tgh->num_ctb = ctx->f_lcu;
#endif
#if ALF_CTU_MAP_DYNAMIC
        tgh->alf_tgh_param.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
        memset(tgh->alf_tgh_param.alfCtuEnableFlag, 1, N_C * ctx->f_lcu * sizeof(u8));
#endif
        ret = evcd_eco_tgh(bs, &ctx->sps, &ctx->pps, tgh);

        /* HLS_RPL Test printing the content of RPL0 and RPL1 for each tile_group*/
/*
        printf("\nCurrent tile_group POC: %d RPL0 Index: %d RPL1 Index: %d\n", tgh->poc, tgh->rpl_l0_idx, tgh->rpl_l1_idx);
        printf(" Number of ref pics in RPL0: %d Number of active ref pics in RPL0 %d [", tgh->rpl_l0.ref_pic_num, tgh->rpl_l0.ref_pic_active_num);
        for (int ii = 0; ii < tgh->rpl_l0.ref_pic_num; ii++)
        {
            printf("%d ", tgh->rpl_l0.ref_pics[ii]);
        }
        printf("]\n");
        printf(" Number of ref pics in RPL1: %d Number of active ref pics in RPL1 %d [", tgh->rpl_l1.ref_pic_num, tgh->rpl_l1.ref_pic_active_num);
        for (int ii = 0; ii < tgh->rpl_l1.ref_pic_num; ii++)
        {
            printf("%d ", tgh->rpl_l1.ref_pics[ii]);
        }
        printf("]\n");
*/

        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        ret = tile_group_init(ctx, ctx->core, tgh);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
#if HLS_M47668
        if(!sps->tool_pocs)
        {
            if (ctx->dtr == 0) // TBD: Check instead if picture is IDR
            {
                tgh->poc = 0;
                ctx->prev_doc_offset = 0;
                ctx->prev_pic_order_cnt_val = tgh->poc;
                ctx->tile_group_ref_flag = is_ref_pic(ctx, tgh);
            }
            else
            {
                ctx->tile_group_ref_flag = is_ref_pic(ctx, tgh);
                poc_derivation(ctx, tgh);
            }
        }
        else
        {
            ctx->tile_group_ref_flag = 1;
        }
#endif

        if (sps->picture_num_present_flag)
        {
            /* initialize reference pictures */
            ret = evc_picman_refp_init(&ctx->dpm, ctx->sps.num_ref_pics_act, tgh->tile_group_type, ctx->ptr, ctx->tgh.layer_id, ctx->last_intra_ptr, ctx->refp);
        }
        else
        {
            /* reference picture marking */
            ret = evc_picman_refpic_marking(&ctx->dpm, tgh);
            evc_assert_rv(ret == EVC_OK, ret);

            /* reference picture lists construction */
            ret = evc_picman_refp_rpl_based_init(&ctx->dpm, tgh, ctx->refp);
        }
        evc_assert_rv(ret == EVC_OK, ret);

        if (sps->picture_num_present_flag)
        {
            if ((tgh->rmpni_on && ctx->tgh.tile_group_type != TILE_GROUP_I))
            {
                ret = evc_picman_refp_reorder(&ctx->dpm, ctx->sps.num_ref_pics_act, tgh->tile_group_type, ctx->ptr, ctx->refp, ctx->last_intra_ptr, tgh->rmpni);
                evc_assert_rv(ret == EVC_OK, ret);
            }
        }
        /* get available frame buffer for decoded image */
        ctx->pic = evc_picman_get_empty_pic(&ctx->dpm, &ret);
        evc_assert_rv(ctx->pic, ret);

        /* get available frame buffer for decoded image */
        ctx->map_refi = ctx->pic->map_refi;
        ctx->map_mv = ctx->pic->map_mv;
#if DMVR_LAG
        ctx->map_unrefined_mv = ctx->pic->map_unrefined_mv;
#endif
        /* decode tile_group layer */
        ret = ctx->fn_dec_tile_group(ctx, ctx->core);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        /* deblocking filter */
        if(ctx->tgh.deblocking_filter_on)
        {
            ret = ctx->fn_deblock(ctx);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        }


#if ALF
        /* adaptive loop filter */
        if( ctx->tgh.alf_on )
        {
            ret = ctx->fn_alf(ctx,  ctx->pic);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        }
#endif

#if USE_DRAW_PARTITION_DEC
        evcd_draw_partition(ctx, ctx->pic);
#endif

        if(ctx->use_pic_sign && ctx->pic_sign_exist)
        {
            ret = evcd_picbuf_check_signature(ctx->pic, ctx->pic_sign);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);

            ctx->pic_sign_exist = 0; /* reset flag */
        }

#if PIC_PAD_SIZE_L > 0
        /* expand pixels to padding area */
        ctx->fn_picbuf_expand(ctx, ctx->pic);
#endif

        /* put decoded picture to DPB */
#if HLS_M47668
        ret = evc_picman_put_pic(&ctx->dpm, ctx->pic, ctx->tgh.tile_group_type, ctx->ptr, ctx->dtr, ctx->tgh.layer_id, 1, ctx->refp, ctx->tile_group_ref_flag, sps->picture_num_present_flag, ctx->ref_pic_gap_length);
#else
        ret = evc_picman_put_pic(&ctx->dpm, ctx->pic, ctx->tgh.tile_group_type, ctx->ptr, ctx->dtr, ctx->tgh.layer_id, 1, ctx->refp, (ctx->tgh.mmco_on ? &ctx->tgh.mmco : NULL), sps->picture_num_present_flag);
#endif
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);

        tile_group_deinit(ctx);
    }
#if ALF_PARAMETER_APS
    if ( ! ( (cnkh->ctype == EVC_CT_TILE_GROUP) || (cnkh->ctype == EVC_CT_SPS) || (cnkh->ctype == EVC_CT_APS) ) )
#else
    else
#endif
    {
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    make_stat(ctx, cnkh->ctype, stat);

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
    ctx->fn_dec_cnk       = evcd_dec_cnk;
    ctx->fn_dec_tile_group     = evcd_dec_tile_group;
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
    ctx->fn_dec_tile_group     = NULL;
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
