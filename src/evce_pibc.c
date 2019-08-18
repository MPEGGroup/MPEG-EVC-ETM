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
#include "evce_ibc_hash_wrapper.h"

#if IBC

#define ENABLE_IBC_CHROMA_REFINE 0

#define SWAP(a, b, t) { (t) = (a); (a) = (b); (b) = (t); }

#define CHROMA_REFINEMENT_CANDIDATES         8  /* 8 candidates BV to choose from */
__inline u32 get_exp_golomb_bits(u32 abs_mvd)
{
  int bits = 0;
  int len_i, len_c, nn;

  /* abs(mvd) */
  nn = ((abs_mvd + 1) >> 1);
  for (len_i = 0; len_i < 16 && nn != 0; len_i++)
  {
    nn >>= 1;
  }
  len_c = (len_i << 1) + 1;

  bits += len_c;

  /* sign */
  if (abs_mvd)
  {
    bits++;
  }

  return bits;
}
static double pibc_residue_rdo(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh,
    pel pred[2][N_C][MAX_CU_DIM], s16 coef[N_C][MAX_CU_DIM], u8 mvp_idx, s16 match_pos[MV_D])
{
    EVCE_PIBC *pi = &ctx->pibc;
    int   *nnz, tnnz, w[N_C], h[N_C], log2_w[N_C], log2_h[N_C];
    int    cuw;
    int    cuh;
    pel(*rec)[MAX_CU_DIM];
    s64    dist[N_C];
    double cost, cost_best = MAX_COST;
    int nnz_store[N_C];
    int    bit_cnt;
    int    i;
    pel   *org[N_C];
    double cost_comp_best = MAX_COST;
    int    idx_best[N_C] = { 0, };

#if RDO_DBK
    u8     is_from_mv_field = 0;
#endif
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif
    rec = pi->unfiltered_rec_buf;
    nnz = core->nnz;
    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;
    w[Y_C] = 1 << log2_cuw;
    h[Y_C] = 1 << log2_cuh;
    w[U_C] = w[V_C] = 1 << (log2_cuw - 1);
    h[U_C] = h[V_C] = 1 << (log2_cuh - 1);
    log2_w[Y_C] = log2_cuw;
    log2_h[Y_C] = log2_cuh;
    log2_w[U_C] = log2_w[V_C] = log2_cuw - 1;
    log2_h[U_C] = log2_h[V_C] = log2_cuh - 1;
    org[Y_C] = pi->o[Y_C] + (y * pi->s_o[Y_C]) + x;
    org[U_C] = pi->o[U_C] + ((y >> 1) * pi->s_o[U_C]) + (x >> 1);
    org[V_C] = pi->o[V_C] + ((y >> 1) * pi->s_o[V_C]) + (x >> 1);

    evc_IBC_mc(x, y, log2_cuw, log2_cuh, match_pos, pi->pic_m, pred[0]);

    /* get residual */
    evce_diff_pred(x, y, log2_cuw, log2_cuh, pi->pic_o, pred[0], coef);

//    /* transform and quantization */
    tnnz = evce_sub_block_tq(coef, log2_cuw, log2_cuh, pi->qp_y, pi->qp_u, pi->qp_v, pi->tile_group_type, nnz
      , core->nnz_sub, 0, ctx->lambda[0], ctx->lambda[1], ctx->lambda[2], RUN_L | RUN_CB | RUN_CR, ctx->sps.tool_cm_init, ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
      , 0, 0
#endif
#if ATS_INTER_PROCESS
      , 0
#endif
#if ADCC
      , ctx->sps.tool_adcc
#endif
    );
    if (tnnz)
    {

        for (i = 0; i < N_C; i++)
        {
            int size = (cuw * cuh) >> (i == 0 ? 0 : 2);

            evc_mcpy(pi->inv_coef[i], coef[i], sizeof(s16) * size);

            //cbf_idx[i] = 0;
            nnz_store[i] = nnz[i];
        }

        evc_sub_block_itdq(pi->inv_coef, log2_cuw, log2_cuh, pi->qp_y, pi->qp_u, pi->qp_v, nnz, core->nnz_sub, ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
          , 0, 0
#endif
#if ATS_INTER_PROCESS
          , 0
#endif
        );

        for (i = 0; i < N_C; i++)
        {
            evc_recon(pi->inv_coef[i], pred[0][i], nnz[i], w[i], h[i], w[i], rec[i]
#if ATS_INTER_PROCESS
              , 0
#endif
            );

            dist[i] = evce_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], w[i], pi->s_o[i]);

        }
#if RDO_DBK
        //filter rec and calculate ssd
        calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, rec, cuw, x, y, core->avail_lr, 0, nnz[Y_C] != 0, NULL, pi->mv, is_from_mv_field
#if ATS_INTER_PROCESS
          , 0
#endif
        );
        for (i = 0; i < N_C; i++)
        {
            dist[i] += ctx->delta_dist[i];
        }
#endif

        nnz[Y_C] = nnz_store[Y_C];
        nnz[U_C] = nnz_store[U_C];
        nnz[V_C] = nnz_store[V_C];

        cost = (double)dist[Y_C] + (((double)dist[U_C] * ctx->dist_chroma_weight[0]) + ((double)dist[V_C] * ctx->dist_chroma_weight[1]));


        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
        evce_sbac_bit_reset(&core->s_temp_run);

        evce_rdo_bit_cnt_cu_ibc(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->mvd, coef, mvp_idx, pi->ibc_flag);

        bit_cnt = evce_get_bit_number(&core->s_temp_run);
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

        if (cost < cost_best)
        {
            cost_best = cost;
            SBAC_STORE(core->s_temp_best, core->s_temp_run);
        }

        SBAC_LOAD(core->s_temp_prev_comp_best, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

        for (i = 0; i < N_C; i++)
        {
            nnz[i] = nnz_store[i];
            if (nnz[i] == 0 && nnz_store[i] != 0)
            {
                evc_mset(coef[i], 0, sizeof(s16) * ((cuw * cuh) >> (i == 0 ? 0 : 2)));
            }
        }
    }
    else
    {
        for (i = 0; i < N_C; i++)
        {
            nnz[i] = 0;
        }

        for (i = 0; i < N_C; i++)
        {
            evc_recon(coef[i], pred[0][i], nnz[i], w[i], h[i], w[i], rec[i]
#if ATS_INTER_PROCESS
              , 0
#endif
            );

            dist[i] = evce_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], w[i], pi->s_o[i]);

        }

        calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, rec, cuw, x, y, core->avail_lr, 0, 0, NULL, pi->mv, is_from_mv_field
#if ATS_INTER_PROCESS
          , 0
#endif
        );
        for (i = 0; i < N_C; i++)
        {
            dist[i] += ctx->delta_dist[i];
        }

        cost_best = (double)dist[Y_C] + (ctx->dist_chroma_weight[0] * (double)dist[U_C]) + (ctx->dist_chroma_weight[1] * (double)dist[V_C]);


        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

        evce_sbac_bit_reset(&core->s_temp_run);

        evce_rdo_bit_cnt_cu_ibc(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->mvd, coef, mvp_idx, pi->ibc_flag);

        bit_cnt = evce_get_bit_number(&core->s_temp_run);
        cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
        SBAC_STORE(core->s_temp_best, core->s_temp_run);

    }

    return cost_best;
}

static void clip_ibc_mv(int rc_mv[2], int pic_width, int pic_height, int lcu_width, int lcu_height, int cu_pos_x, int cu_pos_y)
{
    int offset = 8;
    int hor_max = (pic_width + offset - cu_pos_x - 1);
    int hor_min = (-lcu_width - offset - cu_pos_x + 1);

    int ver_max = (pic_height + offset - cu_pos_y - 1);
    int ver_min = (-lcu_height - offset - cu_pos_y + 1);

    rc_mv[0] = EVC_MIN(hor_max, EVC_MAX(hor_min, rc_mv[0]));
    rc_mv[1] = EVC_MIN(ver_max, EVC_MAX(ver_min, rc_mv[1]));
}

static void ibc_set_search_range(EVCE_CTX *ctx, EVCE_CORE *core, int cu_pel_x, int cu_pel_y, int log2_cuw, int log2_cuh,
    const int local_search_range_x, const int local_search_range_y, int mv_search_range_left[2], int mv_search_range_right[2])
{
    int search_left = 0;
    int search_right = 0;
    int search_top = 0;
    int search_bottom = 0;

    const int roi_width = (1 << log2_cuw);
    const int roi_height = (1 << log2_cuh);

    const int pic_width = ctx->w;
    const int pic_height = ctx->h;

    search_left = -EVC_MIN(cu_pel_x, local_search_range_x);
    search_top = -EVC_MIN(cu_pel_y, local_search_range_y);

    search_right = EVC_MIN(pic_width - cu_pel_x - roi_width, local_search_range_x);
    search_bottom = EVC_MIN(pic_height - cu_pel_y - roi_height, local_search_range_y);

    mv_search_range_left[0] = search_left;
    mv_search_range_left[1] = search_top;
    mv_search_range_right[0] = search_right;
    mv_search_range_right[1] = search_bottom;

    clip_ibc_mv(mv_search_range_left, pic_width, pic_height, ctx->max_cuwh, ctx->max_cuwh,
        cu_pel_x, cu_pel_y);
    clip_ibc_mv(mv_search_range_right, pic_width, pic_height, ctx->max_cuwh, ctx->max_cuwh,
        cu_pel_x, cu_pel_y);
}

static void init_LOG_LUT(EVCE_PIBC *pi)
{
    int size = sizeof(s8) * (MAX_CU_SIZE + 1);
    evc_mset(pi->ctu_log2_tbl, 0, size);
    // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
    int c = 0;
    for (int i = 0, n = 0; i <= MAX_CU_SIZE; i++)
    {
        if (i == (1 << n))
        {
            c = n;
            n++;
        }

        pi->ctu_log2_tbl[i] = c;
    }
}

static void update_ibc_mv_cand(u32 sad, int x, int y, u32 *sad_best_cand, s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D])
{
    int j = CHROMA_REFINEMENT_CANDIDATES - 1;

    if (sad < sad_best_cand[CHROMA_REFINEMENT_CANDIDATES - 1])
    {
        for (int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
        {
            if (sad < sad_best_cand[t])
                j = t;
        }

        for (int k = CHROMA_REFINEMENT_CANDIDATES - 1; k > j; k--)
        {
            sad_best_cand[k] = sad_best_cand[k - 1];

            mv_cand[k][0] = mv_cand[k - 1][0];
            mv_cand[k][1] = mv_cand[k - 1][1];
        }
        sad_best_cand[j] = sad;
        mv_cand[j][0] = x;
        mv_cand[j][1] = y;
    }
}

#if ENABLE_IBC_CHROMA_REFINE
static int refine_ibc_chroma_mv(EVCE_CTX *ctx,
    EVCE_CORE *core,
    EVCE_PIBC *pi,
    int cu_x,
    int cu_y,
    int log2_cuw,
    int log2_cuh,
    int pic_width,
    int pic_height,
    u32 *sad_best_cand,
    s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D])
{
    int best_cand_idx = 0;
    u32 sad_best = EVC_UINT32_MAX;

    u32 temp_sad = 0;

    int luma_cuw = 0, luma_cuh = 0;
    int chroma_cuw = 0, chroma_cuh = 0;

    pel pred[N_C][MAX_CU_DIM];

    pel *org = NULL;
    pel *ref = NULL;

    int ref_stride = 0, org_stride = 0;
    int chroma_cu_x = 0, chroma_cu_y = 0;

    EVC_PIC *ref_pic = NULL;

    luma_cuw = 1 << log2_cuw;
    luma_cuh = 1 << log2_cuh;
    chroma_cuw = luma_cuw >> 1;
    chroma_cuh = luma_cuh >> 1;

    chroma_cu_x = cu_x >> 1;
    chroma_cu_y = cu_y >> 1;
    org_stride = pi->pic_o->s_c;

    ref_pic = pi->pic_m;

    ref_stride = ref_pic->s_c;

    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
        if ((!mv_cand[cand][0]) && (!mv_cand[cand][1]))
            continue;

        if (((int)(cu_y + mv_cand[cand][1] + luma_cuh) >= pic_height) || ((cu_y + mv_cand[cand][1]) < 0))
            continue;

        if (((int)(cu_x + mv_cand[cand][0] + luma_cuw) >= pic_width) || ((cu_x + mv_cand[cand][0]) < 0))
            continue;

        temp_sad = sad_best_cand[cand];

        evc_IBC_mc(cu_x, cu_y, log2_cuw, log2_cuh, mv_cand[cand], ref_pic, pred);

        org = pi->pic_o->u + chroma_cu_y * org_stride + chroma_cu_x;
        ref = pred[U_C];

        temp_sad += evce_sad_16b(log2_cuw - 1, log2_cuh - 1, org, ref, org_stride, chroma_cuw);

        org = pi->pic_o->v + chroma_cu_y * org_stride + chroma_cu_x;
        ref = pred[V_C];

        temp_sad += evce_sad_16b(log2_cuw - 1, log2_cuh - 1, org, ref, org_stride, chroma_cuw);

        if (temp_sad < sad_best)
        {
            sad_best = temp_sad;
            best_cand_idx = cand;
        }
    }

    return best_cand_idx;
}
#endif


int get_ibc_mv_bits(int mvd_x, int mvd_y)
{
    int bits = 0;
    bits = (mvd_x > 2048 || mvd_x <= -2048) ? get_exp_golomb_bits(EVC_ABS(mvd_x)) : evce_tbl_mv_bits[mvd_x];
    bits += (mvd_y > 2048 || mvd_y <= -2048) ? get_exp_golomb_bits(EVC_ABS(mvd_y)) : evce_tbl_mv_bits[mvd_y];
    return bits;
}

static u32 getIComponentBits(int val)
{
    if (!val) return 1;

    u32 length = 1;
    u32 temp = (val <= 0) ? (-val << 1) + 1 : (val << 1);

    while (1 != temp)
    {
        temp >>= 1;
        length += 2;
    }

    return length;
}

u32 get_bv_cost_bits(int mv_x, int mv_y)
{
    return getIComponentBits(mv_x) + getIComponentBits(mv_y);
}

static int pibc_search_estimation(EVCE_CTX *ctx, EVCE_CORE *core, EVCE_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D])
{
    int mv_search_range_left[2] = { 0 };
    int mv_search_range_right[2] = { 0 };

    int srch_rng_hor_left = 0;
    int srch_rng_hor_right = 0;
    int srch_rng_ver_top = 0;
    int srch_rng_ver_bottom = 0;

    const unsigned int lcu_width = ctx->max_cuwh;
    const int pu_pel_offset_x = 0;
    const int pu_pel_offset_y = 0;

    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);

    //Distortion  sad;
    u32 sad = 0;
    u32 sad_best = EVC_UINT32_MAX;
    u32 rui_cost = EVC_UINT32_MAX;
    int bestX = 0;
    int bestY = 0;
    int mv_bits = 0, best_mv_bits = 0;

    EVC_PIC *ref_pic = ctx->pibc.pic_m;

    pel *org = pi->o[Y_C] + cu_y * pi->s_o[Y_C] + cu_x;

    pel *rec = ref_pic->y + cu_y * ref_pic->s_l + cu_x;
    pel *ref = rec;

    int best_cand_idx = 0;

    u32 sad_best_cand[CHROMA_REFINEMENT_CANDIDATES];
    s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D];

    ibc_set_search_range(ctx, core, cu_x, cu_y, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
        ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right);

    srch_rng_hor_left = mv_search_range_left[0];
    srch_rng_hor_right = mv_search_range_right[0];
    srch_rng_ver_top = mv_search_range_left[1];
    srch_rng_ver_bottom = mv_search_range_right[1];

    mvp[MV_X] = 0;
    mvp[MV_Y] = 0;

    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
        sad_best_cand[cand] = EVC_UINT32_MAX;
        mv_cand[cand][0] = 0;
        mv_cand[cand][1] = 0;
    }

    const int pic_width = ctx->w;
    const int pic_height = ctx->h;

    {
        u32 tempSadBest = 0;

        int srLeft = srch_rng_hor_left, srRight = srch_rng_hor_right, srTop = srch_rng_ver_top, srBottom = srch_rng_ver_bottom;

        const int boundY = (0 - roi_height - pu_pel_offset_y);
        for (int y = EVC_MAX(srch_rng_ver_top, 0 - cu_pel_y); y <= boundY; ++y)
        {
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, 0, y, lcu_width))
            {
                continue;
            }

            mv_bits = get_bv_cost_bits(0, y);
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
            ref = rec + ref_pic->s_l * y;
            sad += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);

            update_ibc_mv_cand(sad, 0, y, sad_best_cand, mv_cand);
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
                goto end;
            }
        }

        const int boundX = EVC_MAX(srch_rng_hor_left, -cu_pel_x);
        for (int x = 0 - roi_width - pu_pel_offset_x; x >= boundX; --x)
        {
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, 0, lcu_width))
            {
                continue;
            }

            mv_bits = get_bv_cost_bits(x, 0);
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
            ref = rec + x;
            sad += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);

            update_ibc_mv_cand(sad, x, 0, sad_best_cand, mv_cand);
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
                goto end;
            }
        }

        bestX = mv_cand[0][0];
        bestY = mv_cand[0][1];
        sad_best = sad_best_cand[0];
        sad = GET_MV_COST(ctx, mv_bits);
        if ((!bestX && !bestY) || (sad_best - sad <= 32))
        {
#if ENABLE_IBC_CHROMA_REFINE
            //chroma refine
            best_cand_idx = refine_ibc_chroma_mv(ctx, core, pi, cu_x, cu_y, log2_cuw, log2_cuh, pic_width, pic_height, sad_best_cand, mv_cand);
#else
            best_cand_idx = 0;
#endif
            bestX = mv_cand[best_cand_idx][0];
            bestY = mv_cand[best_cand_idx][1];
            sad_best = sad_best_cand[best_cand_idx];
            mv[0] = bestX;
            mv[1] = bestY;
            rui_cost = sad_best;
            goto end;
        }

        if ((1 << log2_cuw) < 16 && (1 << log2_cuh) < 16)
        {
            for (int y = EVC_MAX(srch_rng_ver_top, -cu_pel_y); y <= srch_rng_ver_bottom; y += 2)
            {
                if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
                    continue;

                for (int x = EVC_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x++)
                {
                    if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
                        continue;

                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
                    {
                        continue;
                    }

                    mv_bits = get_bv_cost_bits(x, y);
                    sad = GET_MV_COST(ctx, mv_bits);

                    /* get sad */
                    ref = rec + y * ref_pic->s_l + x;
                    sad += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);

                    update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
                }
            }

            bestX = mv_cand[0][0];
            bestY = mv_cand[0][1];
            sad_best = sad_best_cand[0];

            mv_bits = get_bv_cost_bits(bestX, bestY);
            sad = GET_MV_COST(ctx, mv_bits);

            if (sad_best - sad <= 16)
            {
#if ENABLE_IBC_CHROMA_REFINE
                //chroma refine
                best_cand_idx = refine_ibc_chroma_mv(ctx, core, pi, cu_x, cu_y, log2_cuw, log2_cuh, pic_width, pic_height, sad_best_cand, mv_cand);
#else
                best_cand_idx = 0;
#endif         
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[best_cand_idx];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
                goto end;
            }

            for (int y = (EVC_MAX(srch_rng_ver_top, -cu_pel_y) + 1); y <= srch_rng_ver_bottom; y += 2)
            {
                if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
                    continue;

                for (int x = EVC_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x += 2)
                {
                    if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
                        continue;

                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
                    {
                        continue;
                    }

                    mv_bits = get_bv_cost_bits(x, y);
                    sad = GET_MV_COST(ctx, mv_bits);

                    /* get sad */
                    ref = rec + y * ref_pic->s_l + x;
                    sad += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);

                    update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 5)
                    {
#if ENABLE_IBC_CHROMA_REFINE
                        //chroma refine & return
                        best_cand_idx = refine_ibc_chroma_mv(ctx, core, pi, cu_x, cu_y, log2_cuw, log2_cuh, pic_width, pic_height, sad_best_cand, mv_cand);
#else
                        best_cand_idx = 0;
#endif
                        bestX = mv_cand[best_cand_idx][0];
                        bestY = mv_cand[best_cand_idx][1];
                        sad_best = sad_best_cand[best_cand_idx];
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
                        goto end;
                    }
                }
            }

            bestX = mv_cand[0][0];
            bestY = mv_cand[0][1];
            sad_best = sad_best_cand[0];

            mv_bits = get_bv_cost_bits(bestX, bestY);
            sad = GET_MV_COST(ctx, mv_bits);

            if ((sad_best >= tempSadBest) || ((sad_best - sad) <= 32))
            {
#if ENABLE_IBC_CHROMA_REFINE
                //chroma refine
                best_cand_idx = refine_ibc_chroma_mv(ctx, core, pi, cu_x, cu_y, log2_cuw, log2_cuh, pic_width, pic_height, sad_best_cand, mv_cand);
#else
                best_cand_idx = 0;
#endif
                bestX = mv_cand[best_cand_idx][0];
                bestY = mv_cand[best_cand_idx][1];
                sad_best = sad_best_cand[best_cand_idx];
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
                goto end;
            }

            tempSadBest = sad_best_cand[0];

            for (int y = (EVC_MAX(srch_rng_ver_top, -cu_pel_y) + 1); y <= srch_rng_ver_bottom; y += 2)
            {
                if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
                    continue;

                for (int x = (EVC_MAX(srch_rng_hor_left, -cu_pel_x) + 1); x <= srch_rng_hor_right; x += 2)
                {

                    if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
                        continue;

                    if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
                    {
                        continue;
                    }

                    mv_bits = get_bv_cost_bits(x, y);
                    sad = GET_MV_COST(ctx, mv_bits);

                    /* get sad */
                    ref = rec + y * ref_pic->s_l + x;
                    sad += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);

                    update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 5)
                    {
#if ENABLE_IBC_CHROMA_REFINE
                        //chroma refine & return
                        best_cand_idx = refine_ibc_chroma_mv(ctx, core, pi, cu_x, cu_y, log2_cuw, log2_cuh, pic_width, pic_height, sad_best_cand, mv_cand);
#else
                        best_cand_idx = 0;
#endif                       
                        bestX = mv_cand[best_cand_idx][0];
                        bestY = mv_cand[best_cand_idx][1];
                        sad_best = sad_best_cand[best_cand_idx];
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
                        goto end;
                    }
                }
            }
        }
    }

#if ENABLE_IBC_CHROMA_REFINE
    //chroma refine
    best_cand_idx = refine_ibc_chroma_mv(ctx, core, pi, cu_x, cu_y, log2_cuw, log2_cuh, pic_width, pic_height, sad_best_cand, mv_cand);
#else
    best_cand_idx = 0;
#endif

    bestX = mv_cand[best_cand_idx][0];
    bestY = mv_cand[best_cand_idx][1];
    sad_best = sad_best_cand[best_cand_idx];
    mv[0] = bestX;
    mv[1] = bestY;
    rui_cost = sad_best;

end:
    return rui_cost;
}

static u32 pibc_me_search(EVCE_CTX *ctx, EVCE_CORE *core, EVCE_PIBC *pi, int x, int y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D])
{
    u32 cost = 0;
    s16 mv_temp[MV_D] = { 0, 0 };

    if (ctx->param.ibc_hash_search_flag)
    {
        cost = search_ibc_hash_match(ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp);
    }

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        // if hash search does not work or is not enabled
        cost = pibc_search_estimation(ctx, core, pi, x, y, log2_cuw, log2_cuh, mvp, mv_temp);
    }

    mv[0] = mv_temp[0];
    mv[1] = mv_temp[1];

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        return EVC_UINT32_MAX;
    }

    return cost;
}

static double pibc_analyze_cu(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh,
  EVCE_MODE *mi, s16 coef[N_C][MAX_CU_DIM], pel *rec[N_C], int s_rec[N_C])
{
  EVCE_PIBC *pi;
  u32 mecost, best_mecost;
  s16(*mvp)[MV_D], *mv, *mvd;
  int cuw, cuh, i, j;
  u8 mvp_idx = 0;
  double cost, cost_best = MAX_COST;
  double cost_ibc;
  u8 found_available_ibc = 0;
#if ATS_INTER_PROCESS
  core->ats_inter_info = 0;
#endif
  pi = &ctx->pibc;
  cuw = (1 << log2_cuw);
  cuh = (1 << log2_cuh);

  mv = pi->mv[0];
  mvd = pi->mvd;

  best_mecost = EVC_UINT32_MAX;

  mvp = pi->mvp;

  mvp_idx = 0;

  /* motion search ********************/
  u8 skip_me = 0;

  if (skip_me)
  {
    mecost = EVC_UINT32_MAX;
    mv[MV_X] = mvp[mvp_idx][MV_X];
    mv[MV_Y] = mvp[mvp_idx][MV_Y];
  }
  else
  {
    mecost = pibc_me_search(ctx, core, pi, x, y, log2_cuw, log2_cuh, mvp[mvp_idx], mv);
  }

  if (mv[MV_X] != 0 || mv[MV_Y] != 0)
  {
    found_available_ibc = 1;
    if (mecost < best_mecost)
    {
      best_mecost = mecost;
    }

    pi->mv[1][MV_X] = mv[MV_X];
    pi->mv[1][MV_Y] = mv[MV_Y];

    mvd[MV_X] = mv[MV_X];
    mvd[MV_Y] = mv[MV_Y];

    pi->mvp_idx = mvp_idx;

    pi->pred_mode = MODE_IBC;
    pi->ibc_flag = 1;

    cost = cost_ibc = pibc_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred, pi->coef, mvp_idx, pi->mv[1]);

    if (cost < cost_best)
    {
      pi->mvp_idx = mvp_idx;
      cost_ibc = cost_best = cost;

      for (j = 0; j < N_C; j++)
      {
        int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
        pi->nnz_best[j] = core->nnz[j];
      }
    }
  }

  if (found_available_ibc)
  {
    /* reconstruct */
    for (j = 0; j < N_C; j++)
    {
      int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
      evc_mcpy(coef[j], pi->coef[j], sizeof(s16) * size_tmp);
    }


    for (i = 0; i < N_C; i++)
    {
      rec[i] = pi->unfiltered_rec_buf[i];
      s_rec[i] = (i == 0 ? cuw : cuw >> 1);
      core->nnz[i] = pi->nnz_best[i];
    }

    return cost_ibc;
  }
  else
  {
    return MAX_COST;
  }
}

static int pibc_init_frame(EVCE_CTX *ctx)
{
    EVCE_PIBC *pi;
    EVC_PIC     *pic;
    int size;

    pi = &ctx->pibc;

    pic = pi->pic_o = PIC_ORIG(ctx);
    pi->o[Y_C] = pic->y;
    pi->o[U_C] = pic->u;
    pi->o[V_C] = pic->v;

    pi->s_o[Y_C] = pic->s_l;
    pi->s_o[U_C] = pic->s_c;
    pi->s_o[V_C] = pic->s_c;

    pic = pi->pic_m = PIC_MODE(ctx);
    pi->m[Y_C] = pic->y;
    pi->m[U_C] = pic->u;
    pi->m[V_C] = pic->v;

    pi->s_m[Y_C] = pic->s_l;
    pi->s_m[U_C] = pic->s_c;
    pi->s_m[V_C] = pic->s_c;

    pi->tile_group_type = ctx->tile_group_type;

    pi->refi[0] = 0;
    pi->refi[1] = REFI_INVALID;

    pi->w_scu = ctx->w_scu;

    size = sizeof(pel) * N_C * MAX_CU_DIM;
    evc_mset(pi->unfiltered_rec_buf, 0, size);

    size = sizeof(pel) * REFP_NUM * N_C * MAX_CU_DIM;
    evc_mset(pi->pred, 0, size);

    /* MV predictor */
    size = sizeof(s16) * MAX_NUM_MVP * MV_D;
    evc_mset(pi->mvp, 0, size);

    size = sizeof(s16) * MV_D;
    evc_mset(pi->mv, 0, size);

    size = sizeof(s16) * MV_D;
    evc_mset(pi->mvd, 0, size);

    init_LOG_LUT(pi);

    return EVC_OK;
}

void reset_ibc_search_range(EVCE_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh)
{
    int hashHitRatio = 0;
    ctx->pibc.search_range_x = ctx->param.ibc_search_range_x;
    ctx->pibc.search_range_y = ctx->param.ibc_search_range_y;

    hashHitRatio = get_hash_hit_ratio(ctx, ctx->ibc_hash_handle, cu_x, cu_y, log2_cuw, log2_cuh); // in percent

    if (hashHitRatio < 5) // 5%
    {
        ctx->pibc.search_range_x >>= 1;
        ctx->pibc.search_range_y >>= 1;
    }
}

static int pibc_init_lcu(EVCE_CTX *ctx, EVCE_CORE *core)
{
    EVCE_PIBC *pi;

    pi = &ctx->pibc;
    pi->lambda_mv = (u32)floor(65536.0 * ctx->sqrt_lambda[0]);
    pi->qp_y = core->qp_y;
    pi->qp_u = core->qp_u;
    pi->qp_v = core->qp_v;

    return EVC_OK;
}

static int pibc_set_complexity(EVCE_CTX *ctx, int complexity)
{
    EVCE_PIBC *pi;

    pi = &ctx->pibc;

    /* default values *************************************************/
    pi->search_range_x = ctx->param.ibc_search_range_x;
    pi->search_range_y = ctx->param.ibc_search_range_y;

    ctx->fn_pibc_analyze_cu = pibc_analyze_cu;

    pi->complexity = complexity;

    return EVC_OK;
}

int evce_pibc_create(EVCE_CTX *ctx, int complexity)
{
    /* set function addresses */
    ctx->fn_pibc_init_frame = pibc_init_frame;
    ctx->fn_pibc_init_lcu = pibc_init_lcu;
    ctx->fn_pibc_set_complexity = pibc_set_complexity;

    /* set maximum/minimum value of search range */
    ctx->pibc.min_clip[MV_X] = -MAX_CU_SIZE + 1;
    ctx->pibc.min_clip[MV_Y] = -MAX_CU_SIZE + 1;
    ctx->pibc.max_clip[MV_X] = ctx->param.w - 1;
    ctx->pibc.max_clip[MV_Y] = ctx->param.h - 1;

    return ctx->fn_pibc_set_complexity(ctx, complexity);
}
#endif