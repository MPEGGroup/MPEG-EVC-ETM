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

#include "evce_def.h"
#include <math.h>

/* Define the Search Range for int-pel */
#define SEARCH_RANGE_IPEL_RA               384
#define SEARCH_RANGE_IPEL_LD               64
/* Define the Search Range for sub-pel ME */
#define SEARCH_RANGE_SPEL                  3

#define MV_COST(pi, mv_bits) (u32)(((pi)->lambda_mv * mv_bits + (1 << 15)) >> 16)
#define SWAP(a, b, t) { (t) = (a); (a) = (b); (b) = (t); }

#if AFFINE_SIMD
#define CALC_EQUAL_COEFF_8PXLS(x1,x2,y1,y2,tmp0,tmp1,tmp2,tmp3,inter0,inter1,inter2,inter3,load_location)      \
{                                                                                                              \
inter0 = _mm_mul_epi32(x1, y1);                                                                                \
inter1 = _mm_mul_epi32(tmp0, tmp2);                                                                            \
inter2 = _mm_mul_epi32(x2, y2);                                                                                \
inter3 = _mm_mul_epi32(tmp1, tmp3);                                                                            \
inter2 = _mm_add_epi64(inter0, inter2);                                                                        \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter0 = _mm_loadl_epi64(load_location);                                                                       \
inter3 = _mm_add_epi64(inter2, inter3);                                                                        \
inter1 = _mm_srli_si128(inter3, 8);                                                                            \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter3 = _mm_add_epi64(inter0, inter3);                                                                        \
}

#define CALC_EQUAL_COEFF_4PXLS(x1,y1,tmp0,tmp1,inter0,inter1,inter2,load_location)        \
{                                                                                         \
inter0 = _mm_mul_epi32(x1, y1);                                                           \
inter1 = _mm_mul_epi32(tmp0, tmp1);                                                       \
inter2 = _mm_loadl_epi64(load_location);                                                  \
inter1 = _mm_add_epi64(inter0, inter1);                                                   \
inter0 = _mm_srli_si128(inter1, 8);                                                       \
inter0 = _mm_add_epi64(inter0, inter1);                                                   \
inter0 = _mm_add_epi64(inter2, inter0);                                                   \
}
#endif

/* q-pel search pattern */
static s8 tbl_search_pattern_qpel_8point[8][2] =
{
    {-1,  0}, { 0,  1}, { 1,  0}, { 0, -1},
    {-1,  1}, { 1,  1}, {-1, -1}, { 1, -1}
};

static const s8 tbl_diapos_partial[2][16][2] =
{
    {
        {-2, 0}, {-1, 1}, {0, 2}, {1, 1}, {2, 0}, {1, -1}, {0, -2}, {-1, -1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
    },
    {
        {-4, 0}, {-3, 1}, {-2, 2}, {-1, 3}, {0, 4}, {1, 3}, {2, 2}, {3, 1}, {4, 0}, {3, -1}, {2, -2}, {1, -3}, {0, -4}, {-1, -3}, {-2, -2}, {-3, -1}
    }
};

static s8 tbl_search_pattern_hpel_partial[8][2] =
{
    {-2, 0}, {-2, 2}, {0, 2}, {2, 2}, {2, 0}, {2, -2}, {0, -2}, {-2, -2}
};

__inline static u32 get_exp_golomb_bits(u32 abs_mvd)
{
    int bits = 0;
    int len_i, len_c, nn;

    /* abs(mvd) */
    nn = ((abs_mvd + 1) >> 1);
    for(len_i = 0; len_i < 16 && nn != 0; len_i++)
    {
        nn >>= 1;
    }
    len_c = (len_i << 1) + 1;

    bits += len_c;

    /* sign */
    if(abs_mvd)
    {
        bits++;
    }

    return bits;
}

static int get_mv_bits_with_mvr(int mvd_x, int mvd_y, int num_refp, int refi, u8 mvr_idx, int sps_amvr_flag)
{
    int bits = 0;
    bits = ((mvd_x >> mvr_idx) > 2048 || (mvd_x >> mvr_idx) <= -2048) ? get_exp_golomb_bits(EVC_ABS(mvd_x) >> mvr_idx) : evce_tbl_mv_bits[mvd_x >> mvr_idx];
    bits += ((mvd_y >> mvr_idx) > 2048 || (mvd_y >> mvr_idx) <= -2048) ? get_exp_golomb_bits(EVC_ABS(mvd_y) >> mvr_idx) : evce_tbl_mv_bits[mvd_y >> mvr_idx];
    bits += evce_tbl_refi_bits[num_refp][refi];
    if(sps_amvr_flag)
    {
        bits += mvr_idx + 1;
    }
    return bits;
}

static void get_range_ipel(EVCE_PINTER * pi, s16 mvc[MV_D], s16 range[MV_RANGE_DIM][MV_D], int bi, int ri, int lidx)
{
    if(pi->sps_amvr_flag)
    {
        int offset = pi->gop_size >> 1;
        int max_qpel_sr = pi->max_search_range >> 3;
        int max_hpel_sr = pi->max_search_range >> 2;
        int max_ipel_sr = pi->max_search_range >> 1;
        int max_spel_sr = pi->max_search_range;
        int max_search_range = EVC_CLIP3(pi->max_search_range >> 2, pi->max_search_range, (pi->max_search_range * EVC_ABS(pi->ptr - (int)pi->refp[ri][lidx].ptr) + offset) / pi->gop_size);
        int offset_x, offset_y, rangexy;
        int range_offset = 3 * (1 << (pi->curr_mvr - 1));

        if(pi->curr_mvr == 0)
        {
            rangexy = EVC_CLIP3(max_qpel_sr >> 2, max_qpel_sr, (max_qpel_sr * EVC_ABS(pi->ptr - (int)pi->refp[ri][lidx].ptr) + offset) / pi->gop_size);
        }
        else if(pi->curr_mvr == 1)
        {
            rangexy = EVC_CLIP3(max_hpel_sr >> 2, max_hpel_sr, (max_hpel_sr * EVC_ABS(pi->ptr - (int)pi->refp[ri][lidx].ptr) + offset) / pi->gop_size);
        }
        else if(pi->curr_mvr == 2)
        {
            rangexy = EVC_CLIP3(max_ipel_sr >> 2, max_ipel_sr, (max_ipel_sr * EVC_ABS(pi->ptr - (int)pi->refp[ri][lidx].ptr) + offset) / pi->gop_size);
        }
        else
        {
            rangexy = EVC_CLIP3(max_spel_sr >> 2, max_spel_sr, (max_spel_sr * EVC_ABS(pi->ptr - (int)pi->refp[ri][lidx].ptr) + offset) / pi->gop_size);
        }

        if(rangexy > max_search_range)
        {
            rangexy = max_search_range;
        }

        if(pi->curr_mvr > 0)
        {
            if((abs(mvc[MV_X] - pi->max_imv[MV_X]) + range_offset) > rangexy)
            {
                offset_x = rangexy;
            }
            else
            {
                offset_x = abs(mvc[MV_X] - pi->max_imv[MV_X]) + range_offset;
            }

            if((abs(mvc[MV_Y] - pi->max_imv[MV_Y]) + range_offset) > rangexy)
            {
                offset_y = rangexy;
            }
            else
            {
                offset_y = abs(mvc[MV_Y] - pi->max_imv[MV_Y]) + range_offset;
            }
        }
        else
        {
            offset_x = rangexy;
            offset_y = rangexy;
        }

        /* define search range for int-pel search and clip it if needs */
        range[MV_RANGE_MIN][MV_X] = EVC_CLIP3(pi->min_clip[MV_X], pi->max_clip[MV_X], mvc[MV_X] - offset_x);
        range[MV_RANGE_MAX][MV_X] = EVC_CLIP3(pi->min_clip[MV_X], pi->max_clip[MV_X], mvc[MV_X] + offset_x);
        range[MV_RANGE_MIN][MV_Y] = EVC_CLIP3(pi->min_clip[MV_Y], pi->max_clip[MV_Y], mvc[MV_Y] - offset_y);
        range[MV_RANGE_MAX][MV_Y] = EVC_CLIP3(pi->min_clip[MV_Y], pi->max_clip[MV_Y], mvc[MV_Y] + offset_y);
    }
    else
    {
        int offset = pi->gop_size >> 1;
        int max_search_range = EVC_CLIP3(pi->max_search_range >> 2, pi->max_search_range, (pi->max_search_range * EVC_ABS(pi->ptr - (int)pi->refp[ri][lidx].ptr) + offset) / pi->gop_size);
        int search_range_x = bi ? BI_STEP : max_search_range;
        int search_range_y = bi ? BI_STEP : max_search_range;

        /* define search range for int-pel search and clip it if needs */
        range[MV_RANGE_MIN][MV_X] = EVC_CLIP3(pi->min_clip[MV_X], pi->max_clip[MV_X], mvc[MV_X] - search_range_x);
        range[MV_RANGE_MAX][MV_X] = EVC_CLIP3(pi->min_clip[MV_X], pi->max_clip[MV_X], mvc[MV_X] + search_range_x);
        range[MV_RANGE_MIN][MV_Y] = EVC_CLIP3(pi->min_clip[MV_Y], pi->max_clip[MV_Y], mvc[MV_Y] - search_range_y);
        range[MV_RANGE_MAX][MV_Y] = EVC_CLIP3(pi->min_clip[MV_Y], pi->max_clip[MV_Y], mvc[MV_Y] + search_range_y);
    }

    evc_assert(range[MV_RANGE_MIN][MV_X] <= range[MV_RANGE_MAX][MV_X]);
    evc_assert(range[MV_RANGE_MIN][MV_Y] <= range[MV_RANGE_MAX][MV_Y]);
}

/* Get original dummy buffer for bi prediction */
static void get_org_bi(pel * org, pel * pred, int s_o, int cuw, int cuh, s16 * org_bi)
{
    int i, j;

    for(j = 0; j < cuh; j++)
    {
        for(i = 0; i < cuw; i++)
        {
            org_bi[i] = ((s16)(org[i]) << 1) - (s16)pred[i];
        }

        org += s_o;
        pred += cuw;
        org_bi += cuw;
    }
}

static u32 me_raster(EVCE_PINTER * pi, int x, int y, int log2_cuw, int log2_cuh, s8 refi, int lidx, s16 range[MV_RANGE_DIM][MV_D], s16 gmvp[MV_D], s16 mv[MV_D])
{
    EVC_PIC *ref_pic;
    pel      *org, *ref;
    u8        mv_bits, best_mv_bits;
    u32       cost_best, cost;
    int       i, j;
    s16       mv_x, mv_y;
    s32       search_step_x = max(RASTER_SEARCH_STEP, (1 << (log2_cuw - 1))); /* Adaptive step size : Half of CU dimension */
    s32       search_step_y = max(RASTER_SEARCH_STEP, (1 << (log2_cuh - 1))); /* Adaptive step size : Half of CU dimension */
    s16       center_mv[MV_D];
    s32       search_step;
    search_step_x = search_step_y = max(RASTER_SEARCH_STEP, (1 << (min(log2_cuh, log2_cuw) - 1)));

    org = pi->o[Y_C] + y * pi->s_o[Y_C] + x;
    ref_pic = pi->refp[refi][lidx].pic;
    best_mv_bits = 0;
    cost_best = EVC_UINT32_MAX;

#if MULTI_REF_ME_STEP
    for(i = range[MV_RANGE_MIN][MV_Y]; i <= range[MV_RANGE_MAX][MV_Y]; i += (search_step_y * (refi + 1)))
    {
        for(j = range[MV_RANGE_MIN][MV_X]; j <= range[MV_RANGE_MAX][MV_X]; j += (search_step_x * (refi + 1)))
#else
    for (i = range[MV_RANGE_MIN][MV_Y]; i <= range[MV_RANGE_MAX][MV_Y]; i += search_step_y)
    {
        for (j = range[MV_RANGE_MIN][MV_X]; j <= range[MV_RANGE_MAX][MV_X]; j += search_step_x)
#endif
        {
            mv_x = j;
            mv_y = i;

            if(pi->curr_mvr > 2)
            {
                int shift = pi->curr_mvr - 2;
                int offset = 1 << (shift - 1);
                mv_x = mv_x >= 0 ? ((mv_x + offset) >> shift) << shift : -(((-mv_x + offset) >> shift) << shift);
                mv_y = mv_y >= 0 ? ((mv_y + offset) >> shift) << shift : -(((-mv_y + offset) >> shift) << shift);
            }

            /* get MVD bits */
            mv_bits = get_mv_bits_with_mvr((mv_x << 2) - gmvp[MV_X], (mv_y << 2) - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr, pi->sps_amvr_flag);

            /* get MVD cost_best */
            cost = MV_COST(pi, mv_bits);

            ref = ref_pic->y + mv_x + mv_y * ref_pic->s_l;


            /* get sad */
            cost += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);

            /* check if motion cost_best is less than minimum cost_best */
            if(cost < cost_best)
            {
                mv[MV_X] = ((mv_x - x) << 2);
                mv[MV_Y] = ((mv_y - y) << 2);
                cost_best = cost;
                best_mv_bits = mv_bits;
            }
        }
    }

    /* Grid search around best mv for all dyadic step sizes till integer pel */
#if MULTI_REF_ME_STEP
    search_step = (refi + 1) * max(search_step_x, search_step_y) >> 1;
#else
    search_step = max(search_step_x, search_step_y) >> 1;
#endif

    while(search_step > 0)
    {
        center_mv[MV_X] = mv[MV_X];
        center_mv[MV_Y] = mv[MV_Y];

        for(i = -search_step; i <= search_step; i += search_step)
        {
            for(j = -search_step; j <= search_step; j += search_step)
            {
                mv_x = (center_mv[MV_X] >> 2) + x + j;
                mv_y = (center_mv[MV_Y] >> 2) + y + i;

                if((mv_x < range[MV_RANGE_MIN][MV_X]) || (mv_x > range[MV_RANGE_MAX][MV_X]))
                    continue;
                if((mv_y < range[MV_RANGE_MIN][MV_Y]) || (mv_y > range[MV_RANGE_MAX][MV_Y]))
                    continue;

                if(pi->curr_mvr > 2)
                {
                    int rounding = 0;
                    rounding = 1 << (pi->curr_mvr - 3);
                    if(mv_x > 0)
                    {
                        mv_x = ((mv_x + rounding) >> (pi->curr_mvr - 2)) << (pi->curr_mvr - 2);
                    }
                    else
                    {
                        mv_x = ((abs(mv_x) + rounding) >> (pi->curr_mvr - 2)) << (pi->curr_mvr - 2);
                        mv_x = -1 * mv_x;
                    }
                    if(mv_y > 0)
                    {
                        mv_y = ((mv_y + rounding) >> (pi->curr_mvr - 2)) << (pi->curr_mvr - 2);
                    }
                    else
                    {
                        mv_y = ((abs(mv_y) + rounding) >> (pi->curr_mvr - 2)) << (pi->curr_mvr - 2);
                        mv_y = -1 * mv_y;
                    }
                }

                /* get MVD bits */
                mv_bits = get_mv_bits_with_mvr((mv_x << 2) - gmvp[MV_X], (mv_y << 2) - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr, pi->sps_amvr_flag);

                /* get MVD cost_best */
                cost = MV_COST(pi, mv_bits);

                ref = ref_pic->y + mv_x + mv_y * ref_pic->s_l;

                /* get sad */
                cost += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);
                /* check if motion cost_best is less than minimum cost_best */
                if(cost < cost_best)
                {
                    mv[MV_X] = ((mv_x - x) << 2);
                    mv[MV_Y] = ((mv_y - y) << 2);
                    cost_best = cost;
                    best_mv_bits = mv_bits;
                }
            }
        }

        /* Halve the step size */
        search_step >>= 1;
    }

    if(best_mv_bits > 0)
    {
        pi->mot_bits[lidx] = best_mv_bits;
    }

    return cost_best;
}

static u32 me_ipel_diamond(EVCE_PINTER *pi, int x, int y, int log2_cuw, int log2_cuh, s8 refi, int lidx, s16 range[MV_RANGE_DIM][MV_D], s16 gmvp[MV_D], s16 mvi[MV_D], s16 mv[MV_D], int bi, int *beststep, int faststep)
{
    EVC_PIC      *ref_pic;
    pel           *org, *ref;
    u32            cost, cost_best = EVC_UINT32_MAX;
    int            mv_bits, best_mv_bits;
    s16            mv_x, mv_y, mv_best_x, mv_best_y;
    int            lidx_r = (lidx == REFP_0) ? REFP_1 : REFP_0;
    s16           *org_bi = pi->org_bi;
    s16            mvc[MV_D];
    int            step, i, j;
    int            min_cmv_x, min_cmv_y, max_cmv_x, max_cmv_y;
    s16            imv_x, imv_y;
    int            mvsize = 1;

    org = pi->o[Y_C] + y * pi->s_o[Y_C] + x;
    ref_pic = pi->refp[refi][lidx].pic;
    mv_best_x = (mvi[MV_X] >> 2);
    mv_best_y = (mvi[MV_Y] >> 2);
    best_mv_bits = 0;
    step = 0;
    mv_best_x = EVC_CLIP3(pi->min_clip[MV_X], pi->max_clip[MV_X], mv_best_x);
    mv_best_y = EVC_CLIP3(pi->min_clip[MV_Y], pi->max_clip[MV_Y], mv_best_y);

    if(pi->curr_mvr > 2)
    {
        int shift = pi->curr_mvr - 2;
        int offset = 1 << (shift - 1);
        mv_best_x = mv_best_x >= 0 ? ((mv_best_x + offset) >> shift) << shift : -(((-mv_best_x + offset) >> shift) << shift);
        mv_best_y = mv_best_y >= 0 ? ((mv_best_y + offset) >> shift) << shift : -(((-mv_best_y + offset) >> shift) << shift);
    }

    imv_x = mv_best_x;
    imv_y = mv_best_y;

    while(1)
    {
        if(step <= 2)
        {
            if(pi->curr_mvr > 2)
            {
                min_cmv_x = (mv_best_x <= range[MV_RANGE_MIN][MV_X]) ? mv_best_x : mv_best_x - ((bi == BI_NORMAL ? (BI_STEP - 2) : 1) << (pi->curr_mvr - 1));
                min_cmv_y = (mv_best_y <= range[MV_RANGE_MIN][MV_Y]) ? mv_best_y : mv_best_y - ((bi == BI_NORMAL ? (BI_STEP - 2) : 1) << (pi->curr_mvr - 1));
                max_cmv_x = (mv_best_x >= range[MV_RANGE_MAX][MV_X]) ? mv_best_x : mv_best_x + ((bi == BI_NORMAL ? (BI_STEP - 2) : 1) << (pi->curr_mvr - 1));
                max_cmv_y = (mv_best_y >= range[MV_RANGE_MAX][MV_Y]) ? mv_best_y : mv_best_y + ((bi == BI_NORMAL ? (BI_STEP - 2) : 1) << (pi->curr_mvr - 1));
            }
            else
            {
                min_cmv_x = (mv_best_x <= range[MV_RANGE_MIN][MV_X]) ? mv_best_x : mv_best_x - (bi == BI_NORMAL ? BI_STEP : 2);
                min_cmv_y = (mv_best_y <= range[MV_RANGE_MIN][MV_Y]) ? mv_best_y : mv_best_y - (bi == BI_NORMAL ? BI_STEP : 2);
                max_cmv_x = (mv_best_x >= range[MV_RANGE_MAX][MV_X]) ? mv_best_x : mv_best_x + (bi == BI_NORMAL ? BI_STEP : 2);
                max_cmv_y = (mv_best_y >= range[MV_RANGE_MAX][MV_Y]) ? mv_best_y : mv_best_y + (bi == BI_NORMAL ? BI_STEP : 2);
            }

            if(pi->curr_mvr > 2)
            {
                mvsize = 1 << (pi->curr_mvr - 2);
            }
            else
            {
                mvsize = 1;
            }

            for(i = min_cmv_y; i <= max_cmv_y; i += mvsize)
            {
                for(j = min_cmv_x; j <= max_cmv_x; j += mvsize)
                {
                    mv_x = j;
                    mv_y = i;

                    if(mv_x > range[MV_RANGE_MAX][MV_X] ||
                       mv_x < range[MV_RANGE_MIN][MV_X] ||
                       mv_y > range[MV_RANGE_MAX][MV_Y] ||
                       mv_y < range[MV_RANGE_MIN][MV_Y])
                    {
                        cost = EVC_UINT32_MAX;
                    }
                    else
                    {
                        /* get MVD bits */
                        mv_bits = get_mv_bits_with_mvr((mv_x << 2) - gmvp[MV_X], (mv_y << 2) - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr, pi->sps_amvr_flag);

                        if(bi)
                        {
                            mv_bits += pi->mot_bits[lidx_r];
                        }

                        /* get MVD cost_best */
                        cost = MV_COST(pi, mv_bits);

                        ref = ref_pic->y + mv_x + mv_y * ref_pic->s_l;

                        if(bi)
                        {
                            /* get sad */
                            cost += evce_sad_bi_16b(log2_cuw, log2_cuh, org_bi, ref, 1 << log2_cuw, ref_pic->s_l);
                        }
                        else
                        {
                            /* get sad */
                            cost += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);
                        }

                        /* check if motion cost_best is less than minimum cost_best */
                        if(cost < cost_best)
                        {
                            mv_best_x = mv_x;
                            mv_best_y = mv_y;
                            *beststep = 2;
                            cost_best = cost;
                            best_mv_bits = mv_bits;
                        }
                    }
                }
            }

            mvc[MV_X] = mv_best_x;
            mvc[MV_Y] = mv_best_y;

            get_range_ipel(pi, mvc, range, (bi != BI_NORMAL) ? 0 : 1, refi, lidx);

            step += 2;
        }
        else
        {
            int meidx = step > 8 ? 2 : 1;
            int multi;

            if(pi->curr_mvr > 2)
            {
                multi = step * (1 << (pi->curr_mvr - 2));
            }
            else
            {
                multi = step;
            }

            for(i = 0; i < 16; i++)
            {
                if(meidx == 1 && i > 8)
                {
                    continue;
                }
                if((step == 4) && (i == 1 || i == 3 || i == 5 || i == 7))
                {
                    continue;
                }

                mv_x = imv_x + ((multi >> meidx) * tbl_diapos_partial[meidx - 1][i][MV_X]);
                mv_y = imv_y + ((multi >> meidx) * tbl_diapos_partial[meidx - 1][i][MV_Y]);

                if(mv_x > range[MV_RANGE_MAX][MV_X] ||
                   mv_x < range[MV_RANGE_MIN][MV_X] ||
                   mv_y > range[MV_RANGE_MAX][MV_Y] ||
                   mv_y < range[MV_RANGE_MIN][MV_Y])
                {
                    cost = EVC_UINT32_MAX;
                }
                else
                {
                    /* get MVD bits */
                    mv_bits = get_mv_bits_with_mvr((mv_x << 2) - gmvp[MV_X], (mv_y << 2) - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr, pi->sps_amvr_flag);

                    if(bi)
                    {
                        mv_bits += pi->mot_bits[lidx_r];
                    }

                    /* get MVD cost_best */
                    cost = MV_COST(pi, mv_bits);

                    ref = ref_pic->y + mv_x + mv_y * ref_pic->s_l;
                    if(bi)
                    {
                        /* get sad */
                        cost += evce_sad_bi_16b(log2_cuw, log2_cuh, org_bi, ref, 1 << log2_cuw, ref_pic->s_l);
                    }
                    else
                    {
                        /* get sad */
                        cost += evce_sad_16b(log2_cuw, log2_cuh, org, ref, pi->s_o[Y_C], ref_pic->s_l);
                    }

                    /* check if motion cost_best is less than minimum cost_best */
                    if(cost < cost_best)
                    {
                        mv_best_x = mv_x;
                        mv_best_y = mv_y;
                        *beststep = step;
                        cost_best = cost;
                        best_mv_bits = mv_bits;
                    }
                }
            }
        }

        if(step >= faststep)
        {
            break;
        }

        if(bi == BI_NORMAL)
        {
            break;
        }

        step <<= 1;
    }

    /* set best MV */
    mv[MV_X] = ((mv_best_x - x) << 2);
    mv[MV_Y] = ((mv_best_y - y) << 2);

    if(bi != BI_NORMAL && best_mv_bits > 0 )
    {
        pi->mot_bits[lidx] = best_mv_bits;
    }

    return cost_best;
}

static u32 me_spel_pattern(EVCE_PINTER *pi, int x, int y, int log2_cuw, int log2_cuh, s8 refi, int lidx, s16 gmvp[MV_D], s16 mvi[MV_D], s16 mv[MV_D], int bi)
{
    pel     *org, *ref, *pred;
    s16     *org_bi;
    u32      cost, cost_best = EVC_UINT32_MAX;
    s16      mv_x, mv_y, cx, cy;
    int      lidx_r = (lidx == REFP_0) ? REFP_1 : REFP_0;
    int      i, mv_bits, cuw, cuh, s_org, s_ref, best_mv_bits;

    s_org = pi->s_o[Y_C];
    org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
    s_ref = pi->refp[refi][lidx].pic->s_l;
    ref = pi->refp[refi][lidx].pic->y;
    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;
    org_bi = pi->org_bi;
    pred = pi->pred_buf;
    best_mv_bits = 0;

    /* make MV to be global coordinate */
    cx = mvi[MV_X] + (x << 2);
    cy = mvi[MV_Y] + (y << 2);

    /* intial value */
    mv[MV_X] = mvi[MV_X];
    mv[MV_Y] = mvi[MV_Y];

    /* search upto hpel-level from here */
    /* search of large diamond pattern */
    for(i = 0; i < pi->search_pattern_hpel_cnt; i++)
    {
        mv_x = cx + pi->search_pattern_hpel[i][0];
        mv_y = cy + pi->search_pattern_hpel[i][1];

        /* get MVD bits */
        mv_bits = get_mv_bits_with_mvr(mv_x - gmvp[MV_X], mv_y - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr, pi->sps_amvr_flag);

        if(bi)
        {
            mv_bits += pi->mot_bits[lidx_r];
        }

        /* get MVD cost_best */
        cost = MV_COST(pi, mv_bits);

        /* get the interpolated(predicted) image */
#if MC_PRECISION_ADD
        evc_mc_l(ref, (mv_x << 2), (mv_y << 2), s_ref, cuw, pred, cuw, cuh);
#else
        evc_mc_l(ref, mv_x, mv_y, s_ref, cuw, pred, cuw, cuh);
#endif

        if(bi)
        {
            /* get sad */
            cost += evce_sad_bi_16b(log2_cuw, log2_cuh, org_bi, pred, cuw, cuw);
        }
        else
        {
            /* get sad */
            cost += evce_sad_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw);
        }

        /* check if motion cost_best is less than minimum cost_best */
        if(cost < cost_best)
        {
            mv[MV_X] = mv_x - (x << 2);
            mv[MV_Y] = mv_y - (y << 2);
            cost_best = cost;
        }
    }

    /* search upto qpel-level from here*/
    /* search of small diamond pattern */
    if(pi->me_level > ME_LEV_HPEL && pi->curr_mvr == 0)
    {
        /* make MV to be absolute coordinate */
        cx = mv[MV_X] + (x << 2);
        cy = mv[MV_Y] + (y << 2);

        for(i = 0; i < pi->search_pattern_qpel_cnt; i++)
        {
            mv_x = cx + pi->search_pattern_qpel[i][0];
            mv_y = cy + pi->search_pattern_qpel[i][1];

            /* get MVD bits */
            mv_bits = get_mv_bits_with_mvr(mv_x - gmvp[MV_X], mv_y - gmvp[MV_Y], pi->num_refp, refi, pi->curr_mvr, pi->sps_amvr_flag);

            if(bi)
            {
                mv_bits += pi->mot_bits[lidx_r];
            }

            /* get MVD cost_best */
            cost = MV_COST(pi, mv_bits);

            /* get the interpolated(predicted) image */
#if MC_PRECISION_ADD
            evc_mc_l(ref, (mv_x << 2), (mv_y << 2), s_ref, cuw, pred, cuw, cuh);
#else
            evc_mc_l(ref, mv_x, mv_y, s_ref, cuw, pred, cuw, cuh);
#endif

            if(bi)
            {
                /* get sad */
                cost += evce_sad_bi_16b(log2_cuw, log2_cuh, org_bi, pred, cuw, cuw);
            }
            else
            {
                /* get sad */
                cost += evce_sad_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw);
            }

            /* check if motion cost_best is less than minimum cost_best */
            if(cost < cost_best)
            {
                mv[MV_X] = mv_x - (x << 2);
                mv[MV_Y] = mv_y - (y << 2);
                cost_best = cost;
                best_mv_bits = mv_bits;
            }
        }
    }

    if(!bi && best_mv_bits > 0)
    {
        pi->mot_bits[lidx] = best_mv_bits;
    }

    return cost_best;
}

static u32 pinter_me_epzs(EVCE_PINTER * pi, int x, int y, int log2_cuw, int log2_cuh, s8 * refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi)
{
    s16 mvc[MV_D];  /* MV center for search */
    s16 gmvp[MV_D]; /* MVP in frame cordinate */
    s16 range[MV_RANGE_DIM][MV_D]; /* search range after clipping */
    s16 mvi[MV_D];
    s16 mvt[MV_D];
    u32 cost, cost_best = EVC_UINT32_MAX;
    s8 ri = 0;  /* reference buffer index */
    int tmpstep = 0;
    int beststep = 0;

    gmvp[MV_X] = mvp[MV_X] + (x << 2);
    gmvp[MV_Y] = mvp[MV_Y] + (y << 2);

    if(bi == BI_NORMAL)
    {
        mvi[MV_X] = mv[MV_X] + (x << 2);
        mvi[MV_Y] = mv[MV_Y] + (y << 2);
        mvc[MV_X] = x + (mv[MV_X] >> 2);
        mvc[MV_Y] = y + (mv[MV_Y] >> 2);
    }
    else
    {
        mvi[MV_X] = mvp[MV_X] + (x << 2);
        mvi[MV_Y] = mvp[MV_Y] + (y << 2);
        mvc[MV_X] = x + (mvp[MV_X] >> 2);
        mvc[MV_Y] = y + (mvp[MV_Y] >> 2);
    }

    ri = *refi;

    mvc[MV_X] = EVC_CLIP3(pi->min_clip[MV_X], pi->max_clip[MV_X], mvc[MV_X]);
    mvc[MV_Y] = EVC_CLIP3(pi->min_clip[MV_Y], pi->max_clip[MV_Y], mvc[MV_Y]);

    get_range_ipel(pi, mvc, range, (bi != BI_NORMAL) ? 0 : 1, ri, lidx);

    cost = me_ipel_diamond(pi, x, y, log2_cuw, log2_cuh, ri, lidx, range, gmvp, mvi, mvt, bi, &tmpstep, MAX_FIRST_SEARCH_STEP);
    if(cost < cost_best)
    {
        cost_best = cost;
        mv[MV_X] = mvt[MV_X];
        mv[MV_Y] = mvt[MV_Y];
        if(abs(mvp[MV_X] - mv[MV_X]) < 2 && abs(mvp[MV_Y] - mv[MV_Y]) < 2)
        {
            beststep = 0;
        }
        else
        {
            beststep = tmpstep;
        }
    }

    if(bi == BI_NON && beststep > RASTER_SEARCH_THD)
    {
        cost = me_raster(pi, x, y, log2_cuw, log2_cuh, ri, lidx, range, gmvp, mvt);
        if(cost < cost_best)
        {
            beststep = RASTER_SEARCH_THD;

            cost_best = cost;

            mv[MV_X] = mvt[MV_X];
            mv[MV_Y] = mvt[MV_Y];
        }
    }

    if(bi != BI_NORMAL && beststep > REFINE_SEARCH_THD)
    {
        mvc[MV_X] = x + (mv[MV_X] >> 2);
        mvc[MV_Y] = y + (mv[MV_Y] >> 2);

        get_range_ipel(pi, mvc, range, (bi != BI_NORMAL) ? 0 : 1, ri, lidx);

        mvi[MV_X] = mv[MV_X] + (x << 2);
        mvi[MV_Y] = mv[MV_Y] + (y << 2);
        cost = me_ipel_diamond(pi, x, y, log2_cuw, log2_cuh, ri, lidx, range, gmvp, mvi, mvt, bi, &tmpstep, MAX_REFINE_SEARCH_STEP);
        if(cost < cost_best)
        {
            cost_best = cost;

            mv[MV_X] = mvt[MV_X];
            mv[MV_Y] = mvt[MV_Y];
        }
    }

    if(pi->me_level > ME_LEV_IPEL && (pi->curr_mvr == 0 || pi->curr_mvr == 1))
    {
        /* sub-pel ME */
        cost = me_spel_pattern(pi, x, y, log2_cuw, log2_cuh, ri, lidx, gmvp, mv, mvt, bi);

        if(cost < cost_best)
        {
            cost_best = cost;

            mv[MV_X] = mvt[MV_X];
            mv[MV_Y] = mvt[MV_Y];
        }
    }

    return cost_best;
}

static void evc_mc_mmvd(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[2][N_C][MAX_CU_DIM])
{
    EVC_PIC *ref_pic;
    int       qpel_gmv_x, qpel_gmv_y;
    int       bidx = 0;
    s16       mv_t[REFP_NUM][MV_D];

    mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, mv_t);

    if(REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_0][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_0][MV_Y];

#if MC_PRECISION_ADD
        evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[0][Y_C], w, h);
#else
        evc_bl_mc_l(ref_pic->y, (qpel_gmv_x), (qpel_gmv_y), ref_pic->s_l, w, pred[0][Y_C], w, h);
#endif
        bidx++;
    }

    /* check identical motion */
    if(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
    {
        if(refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr &&  mv_t[REFP_0][MV_X] == mv_t[REFP_1][MV_X] && mv_t[REFP_0][MV_Y] == mv_t[REFP_1][MV_Y])
        {
            return;
        }
    }

    if(REFI_IS_VALID(refi[REFP_1]))
    {
        /* backward */
        ref_pic = refp[refi[REFP_1]][REFP_1].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_1][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_1][MV_Y];

#if MC_PRECISION_ADD
        evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[bidx][Y_C], w, h);
#else
        evc_bl_mc_l(ref_pic->y, (qpel_gmv_x), (qpel_gmv_y), ref_pic->s_l, w, pred[bidx][Y_C], w, h);
#endif
        bidx++;
    }

    if(bidx == 2)
    {
#if OPT_SIMD_MC_L
        average_16b_no_clip_sse(pred[0][Y_C], pred[1][Y_C], pred[0][Y_C], w, w, w, w, h);
#else    
        p0 = pred[0][Y_C];
        p1 = pred[1][Y_C];
        for(j = 0; j < h; j++)
        {
            for(i = 0; i < w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
            }
            p0 += w;
            p1 += w;
        }
#endif
    }
}

__inline static int mmvd_bit_unary_sym(u32 sym, u32 num_ctx, u32 max_num)
{
    int bits = 0;
    u32 ctx_idx = 0;
    int symbol = 0;

    if(max_num > 1)
    {
        for(ctx_idx = 0; ctx_idx < max_num - 1; ++ctx_idx)
        {
            symbol = (ctx_idx == sym) ? 0 : 1;
            bits++;

            if(symbol == 0)
            {
                break;
            }
        }
    }

    return bits;
}

__inline static int mmvd_info_bit_cost(int mvp_idx, int type)
{
    int bits = 0;
    int var0, var1, var2;
    int dev0 = 0;
    int var;

    if(type == 1)
    {
        if(mvp_idx >= (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM))
        {
            mvp_idx = mvp_idx - (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            dev0 = mvp_idx / (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            mvp_idx = mvp_idx - dev0 * (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            var = 1;
        }
        else
        {
            var = 0;
        }

        /* mmvd_group_idx */
        bits += 1;
        if(var == 1)
        {
            bits += 1;
        }
    }
    else
    {
        var = 0;
        dev0 = 0;
    }

    var0 = mvp_idx / MMVD_MAX_REFINE_NUM;
    var1 = (mvp_idx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
    var2 = mvp_idx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;

    bits += mmvd_bit_unary_sym(var, NUM_SBAC_CTX_MMVD_MERGE_IDX, MMVD_BASE_MV_NUM); /* mmvd_merge_idx */
    bits += mmvd_bit_unary_sym(var, NUM_SBAC_CTX_MMVD_DIST_IDX, MMVD_DIST_NUM); /* mmvd_distance_idx */

    /* mmvd_direction_idx */
    if(var2 == 0)
    {
        bits += 2;
    }
    else if(var2 == 1)
    {
        bits += 2;
    }
    else if(var2 == 2)
    {
        bits += 2;
    }
    else if(var2 == 3)
    {
        bits += 2;
    }

    return bits;
}

static double pinter_residue_rdo_mmvd(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, pel pred[2][N_C][MAX_CU_DIM], int pidx)
{
    EVCE_PINTER   *pi = &ctx->pinter;
    int             w, h, log2_w, log2_h;
    int             bit_cnt;
    double          cost = 0.0;
    pel            *y_org;

    w = 1 << log2_cuw;
    h = 1 << log2_cuh;    
    log2_w = log2_cuw;
    log2_h = log2_cuh;

    /* prediction */
    evc_mc_mmvd(x, y, ctx->w, ctx->h, w, h, pi->refi[pidx], pi->mv[pidx], pi->refp, pred);

    /* get distortion */
    y_org = pi->o[Y_C] + x + y * pi->s_o[Y_C];    
    cost = evce_satd_16b(log2_w, log2_h, pred[0][Y_C], y_org, w, pi->s_o[Y_C]);

    /* get bits */
#if 1
    bit_cnt = mmvd_info_bit_cost(pi->mmvd_idx[pidx], !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));
#else
    SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
    evce_sbac_bit_reset(&core->s_temp_run);
    evce_eco_mmvd_info(&core->bs_temp, pi->mmvd_idx[pidx], !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));
    bit_cnt = evce_get_bit_number(&core->s_temp_run);
#endif
    
    /* get RD cost */    
    cost += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);

    return cost;
}

#if ATS_INTER_PROCESS
void copy_tu_from_cu(s16 tu_resi[N_C][MAX_CU_DIM], s16 cu_resi[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 ats_inter_info)
{
    int j;
    int cuw = 1 << log2_cuw;
    int log2_tuw, log2_tuh;
    int tuw, tuh;
    int tu_offset_x, tu_offset_y;

    get_tu_size(ats_inter_info, log2_cuw, log2_cuh, &log2_tuw, &log2_tuh);
    get_tu_pos_offset(ats_inter_info, log2_cuw, log2_cuh, &tu_offset_x, &tu_offset_y);
    tuw = 1 << log2_tuw;
    tuh = 1 << log2_tuh;

    //Y
    for (j = tu_offset_y; j < tu_offset_y + tuh; j++)
    {
        memcpy(tu_resi[Y_C] + (j - tu_offset_y) * tuw, cu_resi[Y_C] + tu_offset_x + j * cuw, sizeof(s16)*tuw);
    }

    //UV
    tu_offset_x >>= 1;
    tu_offset_y >>= 1;
    tuw >>= 1;
    tuh >>= 1;
    cuw >>= 1;
    for (j = tu_offset_y; j < tu_offset_y + tuh; j++)
    {
        memcpy(tu_resi[U_C] + (j - tu_offset_y) * tuw, cu_resi[U_C] + tu_offset_x + j * cuw, sizeof(s16)*tuw);
        memcpy(tu_resi[V_C] + (j - tu_offset_y) * tuw, cu_resi[V_C] + tu_offset_x + j * cuw, sizeof(s16)*tuw);
    }
}

void get_ats_inter_info_rdo_order(EVCE_CORE *core, u8 ats_inter_avail, int* num_rdo, u8* ats_inter_info_list)
{
    int i;
    u8 idx = 0;
    if (ats_inter_avail == 0)
    {
        ats_inter_info_list[idx++] = 0;
    }
    else
    {
        //add non-ats_inter mode
        ats_inter_info_list[idx++] = 0;

        //add ats_inter mode
        for (i = 0; i < 4; i++)
        {
            if ((ats_inter_avail >> i) & 0x1)
            {
                ats_inter_info_list[idx++] = get_ats_inter_info(i + 1, 0);
                ats_inter_info_list[idx++] = get_ats_inter_info(i + 1, 1);
            }
        }

        //toDO: add reordering fast algorithm based on estimated RDCost
    }

    *num_rdo = idx;
}

//fast algorithms for ATS_inter
void calc_min_cost_ats_inter(EVCE_CTX *ctx, EVCE_CORE *core, pel pred[N_C][MAX_CU_DIM], pel** org, int* s_pred, int* s_org,
    u8 ats_inter_avail, s64* dist_no_resi, int* num_rdo, u8* ats_inter_info_list, s64* ats_inter_est_dist)
{
    int cuw = 1 << core->log2_cuw;
    int cuh = 1 << core->log2_cuh;
    int num_part_x = min(16, cuw) / 4;
    int num_part_y = min(16, cuh) / 4;
    int log2_length_x[3];
    int log2_length_y[3];
    s64 dist[4][4], dist_blk, dist_temp[9];
    s64 sum_dist = 0;
    u8  ats_inter_info_list_temp[9];
    int comp, i, j, idx;
    int blk_luma_w = cuw / num_part_x;
    int blk_luma_h = cuh / num_part_y;
    int ats_inter_rdo_idx_list[4];
    int ats_inter_rdo_idx_num = 0;
    int num_half_ats_inter = ((ats_inter_avail & 0x1) ? 2 : 0) + ((ats_inter_avail & 0x2) ? 2 : 0);
    int num_quad_ats_inter = ((ats_inter_avail & 0x4) ? 2 : 0) + ((ats_inter_avail & 0x8) ? 2 : 0);
    assert(num_half_ats_inter + num_quad_ats_inter == *num_rdo - 1);

    if (!ats_inter_avail)
        return;

    //ATS_INTER fast algorithm 1.1: not try ATS_INTER if the residual is too small to compensate bits for encoding residual info
    if (dist_no_resi[Y_C] + dist_no_resi[U_C] * ctx->dist_chroma_weight[0] + dist_no_resi[V_C] * ctx->dist_chroma_weight[1]
        < RATE_TO_COST_LAMBDA(ctx->lambda[0], 20)) //20 extra bits for ATS_INTER residual encoding
    {
        *num_rdo = 1;
        return;
    }

    //ATS_INTER fast algorithm 1.2: derive estimated minDist of ATS_INTER = zero-residual part distortion + non-zero residual part distortion / 16
    memset(dist, 0, sizeof(s64) * 16);
    for (comp = Y_C; comp < N_C; comp++)
    {
        int blk_w, blk_h;
        log2_length_x[comp] = evc_tbl_log2[blk_luma_w] - (comp > 0 ? 1 : 0);
        log2_length_y[comp] = evc_tbl_log2[blk_luma_h] - (comp > 0 ? 1 : 0);
        blk_w = 1 << log2_length_x[comp];
        blk_h = 1 << log2_length_y[comp];

        for (j = 0; j < num_part_y; j++)
        {
            for (i = 0; i < num_part_x; i++)
            {
                int offset_pred = j * blk_h * s_pred[comp] + i * blk_w;
                int offset_org = j * blk_h * s_org[comp] + i * blk_w;
                dist_blk = evce_ssd_16b(log2_length_x[comp], log2_length_y[comp], pred[comp] + offset_pred, org[comp] + offset_org, s_pred[comp], s_org[comp]);
                dist_blk = comp > 0 ? (s64)(dist_blk * ctx->dist_chroma_weight[comp - 1]) : dist_blk;
                dist[j][i] += dist_blk;
                sum_dist += dist_blk;
            }
        }
    }
    assert(abs((int)(sum_dist - (dist_no_resi[Y_C] + dist_no_resi[U_C] * ctx->dist_chroma_weight[0] + dist_no_resi[V_C] * ctx->dist_chroma_weight[1]))) < 32);

    //estimate rd cost for each ATS_INTER mode
    ats_inter_est_dist[0] = sum_dist;
    for (idx = 1; idx < 9; idx++)
    {
        ats_inter_est_dist[idx] = UINT_MAX;
    }
    for (idx = 1; idx < *num_rdo; idx++)
    {
        u8 ats_inter_info = ats_inter_info_list[idx];
        int log2_tuw, log2_tuh, tux, tuy, tuw, tuh;
        s64 dist_tu = 0;
        get_tu_size(ats_inter_info, core->log2_cuw, core->log2_cuh, &log2_tuw, &log2_tuh);
        get_tu_pos_offset(ats_inter_info, core->log2_cuw, core->log2_cuh, &tux, &tuy);
        tuw = 1 << log2_tuw;
        tuh = 1 << log2_tuh;
        for (j = tuy / blk_luma_h; j < (tuy + tuh) / blk_luma_h; j++)
        {
            for (i = tux / blk_luma_w; i < (tux + tuw) / blk_luma_w; i++)
            {
                dist_tu += dist[j][i];
            }
        }
        ats_inter_est_dist[idx] = (dist_tu / 16) + (sum_dist - dist_tu);
    }
    //try 2 half ATS_INTER modes with the lowest distortion
    memcpy(dist_temp, ats_inter_est_dist, sizeof(s64) * 9);
    if (num_half_ats_inter > 0)
    {
        for (i = ats_inter_rdo_idx_num; i < ats_inter_rdo_idx_num + 2; i++)
        {
            s64 min_dist = UINT_MAX;
            for (idx = 1; idx < 1 + num_half_ats_inter; idx++)
            {
                if (dist_temp[idx] < min_dist)
                {
                    min_dist = dist_temp[idx];
                    ats_inter_rdo_idx_list[i] = idx;
                }
            }
            dist_temp[ats_inter_rdo_idx_list[i]] = UINT_MAX;
        }
        ats_inter_rdo_idx_num += 2;
    }
    if (num_quad_ats_inter > 0)
    {
        for (i = ats_inter_rdo_idx_num; i < ats_inter_rdo_idx_num + 2; i++)
        {
            s64 min_dist = UINT_MAX;
            for (idx = 1 + num_half_ats_inter; idx < 1 + num_half_ats_inter + num_quad_ats_inter; idx++)
            {
                if (dist_temp[idx] < min_dist)
                {
                    min_dist = dist_temp[idx];
                    ats_inter_rdo_idx_list[i] = idx;
                }
            }
            dist_temp[ats_inter_rdo_idx_list[i]] = UINT_MAX;
        }
        ats_inter_rdo_idx_num += 2;
    }
    
    memcpy(dist_temp, ats_inter_est_dist, sizeof(s64) * 9);
    memcpy(ats_inter_info_list_temp, ats_inter_info_list, sizeof(u8) * 9);
    for (idx = 1; idx < 1 + ats_inter_rdo_idx_num; idx++)
    {
        ats_inter_info_list[idx] = ats_inter_info_list_temp[ats_inter_rdo_idx_list[idx - 1]];
        ats_inter_est_dist[idx] = dist_temp[ats_inter_rdo_idx_list[idx - 1]];
    }
    for (idx = 1 + ats_inter_rdo_idx_num; idx < *num_rdo; idx++)
    {
        ats_inter_info_list[idx] = 255;
        ats_inter_est_dist[idx] = UINT_MAX;
    }
    *num_rdo = ats_inter_rdo_idx_num + 1;
}

u8 skip_ats_inter_by_rd_cost(EVCE_CTX *ctx, s64* ats_inter_est_dist, u8* ats_inter_info_list, int curr_idx, double cost_best, s64 dist_ats_inter0, double cost_ats_inter0, u8 root_cbf_ats_inter0)
{
    //ATS_INTER fast algorithm 2.2 : estimate a minimum RD cost of a ATS_INTER mode based on the luma distortion of uncoded part and coded part (assuming distorted can be reduced to 1/16);
    //                         if this cost is larger than the best cost, no need to try a specific ATS_INTER mode
    double cost_curr_ats_inter = ats_inter_est_dist[curr_idx] + RATE_TO_COST_LAMBDA(ctx->lambda[0], 11);
    if (cost_curr_ats_inter > cost_best)
    {
        return 1;
    }

    if (cost_ats_inter0 != MAX_COST)
    {
        u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info_list[curr_idx]);
        if (!root_cbf_ats_inter0)
        {
            //ATS_INTER fast algorithm 3: skip ATS_INTER when the residual is too small (estCost is more accurate than fast algorithm 1, counting PU mode bits)
            int weight = is_ats_inter_quad_size(ats_inter_idx) ? 6 : 9;
            s64 dist_resi_part = ((ats_inter_est_dist[0] - ats_inter_est_dist[curr_idx]) * weight) >> 4;
            //prediction info bits cost + minimum residual bits cost + estimate distortion
            double est_cost = (cost_ats_inter0 - dist_ats_inter0) + RATE_TO_COST_LAMBDA(ctx->lambda[0], 10) + (ats_inter_est_dist[curr_idx] + dist_resi_part);
            if (est_cost > cost_ats_inter0 || est_cost > cost_best)
            {
                return 2;
            }
        }
        else
        {
            //ATS_INTER fast algorithm 4: skip ATS_INTER when an estimated RD cost is larger than the bestCost
            double weight = is_ats_inter_quad_size(ats_inter_idx) ? 0.4 : 0.6;
            double est_cost = (cost_ats_inter0 - dist_ats_inter0) * weight + ats_inter_est_dist[curr_idx];
            if (est_cost > cost_best)
            {
                return 3;
            }
        }
    }
    return 0;
}

//save & load functions for ATS_inter
void search_ats_inter_info_saved(EVCE_CTX *ctx, EVCE_CORE *core, u32 dist_pu, int log2_cuw, int log2_cuh, int x, int y, u8* ats_inter_info_match)
{
    int posx = (x - core->x_pel) >> MIN_CU_LOG2;
    int posy = (y - core->y_pel) >> MIN_CU_LOG2;
    int widx = log2_cuw - 2;
    int hidx = log2_cuh - 2;
    int num_route = ATS_INTER_SL_NUM;
    int stride1 = MAX_TR_LOG2 - MIN_CU_LOG2 + 1;
    int stride2 = ctx->max_cuwh >> MIN_CU_LOG2;
    int stridew = stride2 * stride2 * stride1;
    int strideh = stride2 * stride2;
    int stridex = stride2;
    int offset1 = widx * stridew + hidx * strideh + posx * stridex + posy;
    int offset2 = offset1 * num_route;
    int i;
    *ats_inter_info_match = 255;

    for (i = 0; i < ctx->ats_inter_num_pred[offset1]; i++)
    {
        if (ctx->ats_inter_pred_dist[offset2 + i] == dist_pu)
        {
            *ats_inter_info_match = ctx->ats_inter_info_pred[offset2 + i];
            break;
        }
    }
}

void save_ats_inter_info_pred(EVCE_CTX *ctx, EVCE_CORE *core, u32 dist_pu, u8 ats_inter_info_pu, int log2_cuw, int log2_cuh, int x, int y)
{
    int posx = (x - core->x_pel) >> MIN_CU_LOG2;
    int posy = (y - core->y_pel) >> MIN_CU_LOG2;
    int widx = log2_cuw - 2;
    int hidx = log2_cuh - 2;
    int num_route = ATS_INTER_SL_NUM;
    int stride1 = MAX_TR_LOG2 - MIN_CU_LOG2 + 1;
    int stride2 = ctx->max_cuwh >> MIN_CU_LOG2;
    int stridew = stride2 * stride2 * stride1;
    int strideh = stride2 * stride2;
    int stridex = stride2;
    int offset1 = widx * stridew + hidx * strideh + posx * stridex + posy;
    int offset2 = offset1 * num_route;
    int num_data = ctx->ats_inter_num_pred[offset1];

    if (num_data < num_route)
    {
        ctx->ats_inter_info_pred[offset2 + num_data] = ats_inter_info_pu;
        ctx->ats_inter_pred_dist[offset2 + num_data] = dist_pu;
        ctx->ats_inter_num_pred[offset1]++;
    }
}
#endif

static s16    coef_t[N_C][MAX_CU_DIM];

static double pinter_residue_rdo(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh,
                                 pel pred[2][N_C][MAX_CU_DIM], s16 coef[N_C][MAX_CU_DIM], int pidx, u8 *mvp_idx
#if DMVR
                                 , BOOL apply_dmvr
#endif
)
{
    EVCE_PINTER *pi = &ctx->pinter;
    int   *nnz, tnnz, w[N_C], h[N_C], log2_w[N_C], log2_h[N_C];
    int    cuw;
    int    cuh;
    pel  (*rec)[MAX_CU_DIM];
    s64    dist[2][N_C];
    double cost, cost_best = MAX_COST;
    int    cbf_idx[N_C], nnz_store[N_C];
    int    nnz_sub_store[N_C][MAX_SUB_TB_NUM] = {{0},};
    int    bit_cnt;
    int    i, idx_y, idx_u, idx_v;
    pel   *org[N_C];
    double cost_comp_best = MAX_COST;
    int    idx_best[N_C] = {0, };
    int    j;
#if RDO_DBK
    u8     is_from_mv_field = 0;
#endif
#if ATS_INTER_PROCESS // rdo
    s64    dist_no_resi[N_C];
    int    log2_tuw, log2_tuh;
    u8     ats_inter_info_best = 255;
    u8     ats_inter_info_list[9];
    int    num_rdo;
    int    nnz_best[N_C] = { -1, -1, -1 };
    int    ats_inter_mode_idx;
    u8     ats_inter_avail = check_ats_inter_info_coded(1 << log2_cuw, 1 << log2_cuh, MODE_INTER, ctx->sps.tool_ats_inter);
    s64    ats_inter_est_dist[9];
    s64    dist_ats_inter0 = UINT_MAX;
    double cost_ats_inter0 = MAX_COST;
    u8     root_cbf_ats_inter0 = 255;
    u8     ats_inter_info_match = 255;
    u8     num_rdo_tried = 0;
    s64    dist_idx = -1;
    get_ats_inter_info_rdo_order(core, ats_inter_avail, &num_rdo, ats_inter_info_list);
    core->ats_inter_info = 0;
#endif

#if AFFINE
    if(core->affine_flag)
    {
        pi->mvr_idx[pidx] = 0;
        pi->bi_idx[pidx] = BI_NON;
    }
#endif
    rec = pi->rec[pidx];
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

#if RDO_DBK && AFFINE
    if(core->affine_flag)
    {
        is_from_mv_field = 1;
    }
#endif

    /* prediction */
#if AFFINE
    if(core->affine_flag)
    {
        evc_affine_mc(x, y, ctx->w, ctx->h, w[0], h[0], pi->refi[pidx], pi->affine_mv[pidx], pi->refp, pred, core->affine_flag + 1
#if EIF
                      , core->eif_tmp_buffer
#endif
        );
    }
    else
    {
#endif

#if DMVR
        evc_mc(x, y, ctx->w, ctx->h, w[0], h[0], pi->refi[pidx], pi->mv[pidx], pi->refp, pred, ctx->ptr, pi->dmvr_template, pi->dmvr_ref_pred_interpolated
               , pi->dmvr_half_pred_interpolated
               , apply_dmvr && ctx->sps.tool_dmvr
#if DMVR_PADDING
               , pi->dmvr_padding_buf
#endif
#if DMVR_FLAG
               , &core->dmvr_flag
#if DMVR_LAG
               , pi->dmvr_mv[pidx]
#endif
#endif
               , ctx->sps.tool_amis
#else
        evc_mc(x, y, ctx->w, ctx->h, w[0], h[0], pi->refi[pidx], pi->mv[pidx], pi->refp, pred
#endif
        );
#if AFFINE
    }
#endif

    /* get residual */
#if ATS_INTER_PROCESS
    evce_diff_pred(x, y, log2_cuw, log2_cuh, pi->pic_o, pred[0], pi->resi);
    for (i = 0; i < N_C; i++)
    {
        dist[0][i] = evce_ssd_16b(log2_w[i], log2_h[i], pred[0][i], org[i], w[i], pi->s_o[i]);
        dist_no_resi[i] = dist[0][i];
    }
#else
    evce_diff_pred(x, y, log2_cuw, log2_cuh, pi->pic_o, pred[0], coef);
#endif

#if ATS_INTER_PROCESS
    //load best in history
    if (ats_inter_avail)
    {
        int shift_val = min(log2_cuw + log2_cuh, 9);
        dist_idx = dist_no_resi[Y_C] + dist_no_resi[U_C] + dist_no_resi[V_C];
        dist_idx = (dist_idx + (s64)(1 << (shift_val - 1))) >> shift_val;
        search_ats_inter_info_saved(ctx, core, (u32)dist_idx, log2_cuw, log2_cuh, x, y, &ats_inter_info_match);
    }
    if (ats_inter_avail && ats_inter_info_match == 255)
    {
        calc_min_cost_ats_inter(ctx, core, pred[0], org, w, pi->s_o, ats_inter_avail, dist_no_resi, &num_rdo, ats_inter_info_list, ats_inter_est_dist);
    }

    for (ats_inter_mode_idx = 0; ats_inter_mode_idx < num_rdo; ats_inter_mode_idx++)
    {
        core->ats_inter_info = ats_inter_info_list[ats_inter_mode_idx];
        assert(get_ats_inter_idx(core->ats_inter_info) >= 0 && get_ats_inter_idx(core->ats_inter_info) <= 4);
        assert(get_ats_inter_pos(core->ats_inter_info) >= 0 && get_ats_inter_pos(core->ats_inter_info) <= 1);

        //early skp fast algorithm here
        if (ats_inter_info_match != 255 && core->ats_inter_info != ats_inter_info_match)
        {
            continue;
        }
        if (ats_inter_mode_idx > 0 && ats_inter_info_match == 255)
        {
            assert(pidx == AFF_DIR || pidx == PRED_DIR || pidx == PRED_DIR_MMVD || root_cbf_ats_inter0 != 255);
            if (skip_ats_inter_by_rd_cost(ctx, ats_inter_est_dist, ats_inter_info_list, ats_inter_mode_idx, core->cost_best, dist_ats_inter0, cost_ats_inter0, root_cbf_ats_inter0))
            {
                continue;
            }
        }

        //try this ATS_INTER mode
        num_rdo_tried++;

        //prepare tu residual
        copy_tu_from_cu(coef, pi->resi, log2_cuw, log2_cuh, core->ats_inter_info);
#endif

    /* transform and quantization */
    tnnz = evce_sub_block_tq(coef, log2_cuw, log2_cuh, pi->qp_y, pi->qp_u, pi->qp_v, pi->tile_group_type, nnz
                             , core->nnz_sub, 0, ctx->lambda[0], ctx->lambda[1], ctx->lambda[2], RUN_L | RUN_CB | RUN_CR, ctx->sps.tool_cm_init, ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
                             , 0, 0
#endif
#if ATS_INTER_PROCESS
                             , core->ats_inter_info
#endif
#if COEFF_CODE_ADCC
        , ctx->sps.tool_adcc
#endif
    );

    if(tnnz)
    {
        for(i = 0; i < N_C; i++)
        {
            int size = (cuw * cuh) >> (i == 0 ? 0 : 2);
#if ATS_INTER_PROCESS
            int ats_inter_idx = get_ats_inter_idx(core->ats_inter_info);
            size = (core->ats_inter_info == 0) ? size : (size >> (is_ats_inter_quad_size(ats_inter_idx) ? 2 : 1));
#endif
            evc_mcpy(coef_t[i], coef[i], sizeof(s16) * size);

            cbf_idx[i] = 0;
            nnz_store[i] = nnz[i];
            evc_mcpy(nnz_sub_store[i], core->nnz_sub[i], sizeof(int) * MAX_SUB_TB_NUM);
        }

        evc_sub_block_itdq(coef_t, log2_cuw, log2_cuh, pi->qp_y, pi->qp_u, pi->qp_v, nnz, core->nnz_sub, ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
                           , 0, 0
#endif
#if ATS_INTER_PROCESS
                           , core->ats_inter_info
#endif
        );

#if RDO_DBK
#if ATS_INTER_PROCESS
        if (core->ats_inter_info == 0)
        {
#endif
        calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pred[0], cuw, x, y, core->avail_lr, 0, 0, pi->refi[pidx], pi->mv[pidx], is_from_mv_field
#if ATS_INTER_PROCESS
                                        , core->ats_inter_info
#endif
        );
#if ATS_INTER_PROCESS
        }
#endif
#endif

        for(i = 0; i < N_C; i++)
        {
#if !ATS_INTER_PROCESS
            dist[0][i] = evce_ssd_16b(log2_w[i], log2_h[i], pred[0][i], org[i], w[i], pi->s_o[i]);
#endif

            if(nnz[i])
            {
                evc_recon(coef_t[i], pred[0][i], nnz[i], w[i], h[i], w[i], rec[i]
#if ATS_INTER_PROCESS
                          , core->ats_inter_info
#endif
                );
#if HTDF
                if(ctx->sps.tool_htdf == 1 && i == Y_C)
                {
                    const int s_mod = pi->s_m[Y_C];
#if HW_HTDF_CLEANUP
                    u16 avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, log2_cuw, log2_cuh, ctx->map_scu);
                    evc_htdf(rec[i], ctx->tgh.qp, cuw, cuh, cuw, FALSE, pi->m[Y_C] + (y * s_mod) + x, s_mod, avail_cu);
#else
                    evc_get_nbr(x, y, cuw, cuh, pi->m[Y_C] + (y * s_mod) + x, s_mod, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, Y_C);
                    evc_htdf(rec[i], ctx->tgh.qp, cuw, cuh, cuw, FALSE, core->nb[Y_C][0] + 2, core->nb[Y_C][1] + cuh - 1, core->nb[Y_C][2] + 2, core->avail_cu);
#endif
                }
#endif
                dist[1][i] = evce_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], w[i], pi->s_o[i]);
            }
            else
            {
#if ATS_INTER_PROCESS
                dist[1][i] = dist_no_resi[i];
#else
                dist[1][i] = dist[0][i];
#endif
            }
#if RDO_DBK
#if ATS_INTER_PROCESS
            if (core->ats_inter_info == 0)
            {
#endif
            dist[0][i] += ctx->delta_dist[i];
#if ATS_INTER_PROCESS
            }
#endif
#endif
        }
#if RDO_DBK
        //complete rec
        for(i = 0; i < N_C; i++)
        {
            if(nnz[i] == 0)
            {
                evc_recon(coef_t[i], pred[0][i], nnz[i], w[i], h[i], w[i], rec[i]
#if ATS_INTER_PROCESS
                         , core->ats_inter_info
#endif
                );
            }
        }
        //filter rec and calculate ssd
        calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, rec, cuw, x, y, core->avail_lr, 0, nnz[Y_C] != 0, pi->refi[pidx], pi->mv[pidx], is_from_mv_field
#if ATS_INTER_PROCESS
                                        , core->ats_inter_info
#endif
        );
        for(i = 0; i < N_C; i++)
        {
            dist[1][i] += ctx->delta_dist[i];
        }
#endif

        if(1
#if AFFINE
           && pidx != AFF_DIR
#endif
           && pidx != PRED_DIR_MMVD
#if MERGE
           && pidx != PRED_DIR
#endif
#if ATS_INTER_PROCESS
           && core->ats_inter_info == 0
#endif
           )
        {
            /* test all zero case */
            idx_y = 0;
            idx_u = 0;
            idx_v = 0;
            nnz[Y_C] = 0;
            nnz[U_C] = 0;
            nnz[V_C] = 0;
            evc_mset(core->nnz_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);

            cost = (double)dist[idx_y][Y_C] + (((double)dist[idx_u][U_C] * ctx->dist_chroma_weight[0]) + ((double)dist[idx_v][V_C] * ctx->dist_chroma_weight[1]));

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
            evce_sbac_bit_reset(&core->s_temp_run);

            if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
            {
                pi->mvd[pidx][REFP_0][MV_X] >>= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_0][MV_Y] >>= pi->mvr_idx[pidx];
            }
            if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
            {
                pi->mvd[pidx][REFP_1][MV_X] >>= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_1][MV_Y] >>= pi->mvr_idx[pidx];
            }

            evce_rdo_bit_cnt_cu_inter(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->refi[pidx], pi->mvd[pidx], coef, pidx, mvp_idx, pi->mvr_idx[pidx], pi->bi_idx[pidx]
#if AFFINE
                                      , pi->affine_mvd[pidx]
#endif
            );

            if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
            {
                pi->mvd[pidx][REFP_0][MV_X] <<= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_0][MV_Y] <<= pi->mvr_idx[pidx];
            }
            if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
            {
                pi->mvd[pidx][REFP_1][MV_X] <<= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_1][MV_Y] <<= pi->mvr_idx[pidx];
            }

            bit_cnt = evce_get_bit_number(&core->s_temp_run);
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

            if(cost < cost_best)
            {
                cost_best = cost;
                cbf_idx[Y_C] = idx_y;
                cbf_idx[U_C] = idx_u;
                cbf_idx[V_C] = idx_v;
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
                ats_inter_info_best = core->ats_inter_info;
                core->cost_best = cost < core->cost_best ? cost : core->cost_best;
                if (ats_inter_mode_idx == 0)
                {
                    dist_ats_inter0 = (s64)(cost_best - RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt));
                    cost_ats_inter0 = cost_best;
                    root_cbf_ats_inter0 = 0;
                }
#endif
            }
        } // forced zero

        /* test as it is */
        idx_y = nnz_store[Y_C] > 0 ? 1 : 0;
        idx_u = nnz_store[U_C] > 0 ? 1 : 0;
        idx_v = nnz_store[V_C] > 0 ? 1 : 0;
        nnz[Y_C] = nnz_store[Y_C];
        nnz[U_C] = nnz_store[U_C];
        nnz[V_C] = nnz_store[V_C];
        evc_mcpy(core->nnz_sub, nnz_sub_store, sizeof(int) * N_C * MAX_SUB_TB_NUM);

        cost = (double)dist[idx_y][Y_C] + (((double)dist[idx_u][U_C] * ctx->dist_chroma_weight[0]) + ((double)dist[idx_v][V_C] * ctx->dist_chroma_weight[1]));

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
        evce_sbac_bit_reset(&core->s_temp_run);

        if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
        {
            pi->mvd[pidx][REFP_0][MV_X] >>= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_0][MV_Y] >>= pi->mvr_idx[pidx];
        }
        if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
        {
            pi->mvd[pidx][REFP_1][MV_X] >>= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_1][MV_Y] >>= pi->mvr_idx[pidx];
        }

        evce_rdo_bit_cnt_cu_inter(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->refi[pidx], pi->mvd[pidx], coef, pidx, mvp_idx, pi->mvr_idx[pidx], pi->bi_idx[pidx]
#if AFFINE
                                  , pi->affine_mvd[pidx]
#endif
        );

        if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
        {
            pi->mvd[pidx][REFP_0][MV_X] <<= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_0][MV_Y] <<= pi->mvr_idx[pidx];
        }
        if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
        {
            pi->mvd[pidx][REFP_1][MV_X] <<= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_1][MV_Y] <<= pi->mvr_idx[pidx];
        }

        bit_cnt = evce_get_bit_number(&core->s_temp_run);
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

        if(cost < cost_best)
        {
            cost_best = cost;
            cbf_idx[Y_C] = idx_y;
            cbf_idx[U_C] = idx_u;
            cbf_idx[V_C] = idx_v;
            SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
            ats_inter_info_best = core->ats_inter_info;
            core->cost_best = cost < core->cost_best ? cost : core->cost_best;
            if (ats_inter_mode_idx == 0)
            {
                dist_ats_inter0 = (s64)(cost_best - RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt));
                cost_ats_inter0 = cost_best;
                root_cbf_ats_inter0 = (idx_y + idx_u + idx_v) != 0;
            }
#endif
        }

        SBAC_LOAD(core->s_temp_prev_comp_best, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
        /* cbf test for each component */
        for(i = 0; i < N_C; i++)
        {
            if(nnz_store[i] > 0)
            {
                cost_comp_best = MAX_COST;
                SBAC_LOAD(core->s_temp_prev_comp_run, core->s_temp_prev_comp_best);
                for(j = 0; j < 2; j++)
                {
                    cost = dist[j][i] * (i == 0 ? 1 : ctx->dist_chroma_weight[i - 1]);
                    nnz[i] = j ? nnz_store[i] : 0;
                    if (j)
                    {
                        evc_mcpy(core->nnz_sub[i], nnz_sub_store[i], sizeof(int) * MAX_SUB_TB_NUM);
                    }
                    else
                    {
                        evc_mset(core->nnz_sub[i], 0, sizeof(int) * MAX_SUB_TB_NUM);
                    }

                    SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_run);
                    evce_sbac_bit_reset(&core->s_temp_run);
                    evce_rdo_bit_cnt_cu_inter_comp(core, coef, i, pidx
#if ATS_INTRA_PROCESS || COEFF_CODE_ADCC
                                                  , ctx
#endif
                    );

                    bit_cnt = evce_get_bit_number(&core->s_temp_run);
                    cost += RATE_TO_COST_LAMBDA(ctx->lambda[i], bit_cnt);
                    if(cost < cost_comp_best)
                    {
                        cost_comp_best = cost;
                        idx_best[i] = j;
                        SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                    }
                }
            }
            else
            {
                idx_best[i] = 0;
            }
        }

        if(idx_best[Y_C] != 0 || idx_best[U_C] != 0 || idx_best[V_C] != 0)
        {
            idx_y = idx_best[Y_C];
            idx_u = idx_best[U_C];
            idx_v = idx_best[V_C];
            nnz[Y_C] = idx_y ? nnz_store[Y_C] : 0;
            nnz[U_C] = idx_u ? nnz_store[U_C] : 0;
            nnz[V_C] = idx_v ? nnz_store[V_C] : 0;
            for (i = 0; i < N_C; i++)
            {
                if (idx_best[i])
                {
                    evc_mcpy(core->nnz_sub[i], nnz_sub_store[i], sizeof(int) * MAX_SUB_TB_NUM);
                }
                else
                {
                    evc_mset(core->nnz_sub[i], 0, sizeof(int) * MAX_SUB_TB_NUM);
                }
            }
        }

        if(nnz[Y_C] != nnz_store[Y_C] || nnz[U_C] != nnz_store[U_C] || nnz[V_C] != nnz_store[V_C])
        {
            cost = (double)dist[idx_y][Y_C] + (((double)dist[idx_u][U_C] * ctx->dist_chroma_weight[0]) + ((double)dist[idx_v][V_C] * ctx->dist_chroma_weight[1]));

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
            evce_sbac_bit_reset(&core->s_temp_run);

            if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
            {
                pi->mvd[pidx][REFP_0][MV_X] >>= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_0][MV_Y] >>= pi->mvr_idx[pidx];
            }
            if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
            {
                pi->mvd[pidx][REFP_1][MV_X] >>= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_1][MV_Y] >>= pi->mvr_idx[pidx];
            }

            evce_rdo_bit_cnt_cu_inter(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->refi[pidx], pi->mvd[pidx], coef, pidx, mvp_idx, pi->mvr_idx[pidx], pi->bi_idx[pidx]
#if AFFINE
                                      , pi->affine_mvd[pidx]
#endif
            );

            if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
            {
                pi->mvd[pidx][REFP_0][MV_X] <<= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_0][MV_Y] <<= pi->mvr_idx[pidx];
            }
            if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
            {
                pi->mvd[pidx][REFP_1][MV_X] <<= pi->mvr_idx[pidx];
                pi->mvd[pidx][REFP_1][MV_Y] <<= pi->mvr_idx[pidx];
            }

            bit_cnt = evce_get_bit_number(&core->s_temp_run);
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

            if(cost < cost_best)
            {
                cost_best = cost;
                cbf_idx[Y_C] = idx_y;
                cbf_idx[U_C] = idx_u;
                cbf_idx[V_C] = idx_v;
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
                ats_inter_info_best = core->ats_inter_info;
                core->cost_best = cost < core->cost_best ? cost : core->cost_best;
                if (ats_inter_mode_idx == 0)
                {
                    dist_ats_inter0 = (s64)(cost_best - RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt));
                    cost_ats_inter0 = cost_best;
                    root_cbf_ats_inter0 = (idx_y + idx_u + idx_v) != 0;
                }
#endif
            }
        }

        for(i = 0; i < N_C; i++)
        {
            nnz[i] = (cbf_idx[i] ? nnz_store[i] : 0);
            if (cbf_idx[i])
            {
                evc_mcpy(core->nnz_sub[i], nnz_sub_store[i], sizeof(int) * MAX_SUB_TB_NUM);
            }
            else
            {
                evc_mset(core->nnz_sub[i], 0, sizeof(int) * MAX_SUB_TB_NUM);
            }
            if(nnz[i] == 0 && nnz_store[i] != 0)
            {
                evc_mset(core->nnz_sub[i], 0, sizeof(int) * MAX_SUB_TB_NUM);
                evc_mset(coef[i], 0, sizeof(s16) * ((cuw * cuh) >> (i == 0 ? 0 : 2)));
            }
        }
#if ATS_INTER_PROCESS
        //save the best coeff
        if (ats_inter_info_best == core->ats_inter_info && ats_inter_avail)
        {
            for (i = 0; i < N_C; i++)
            {
                nnz_best[i] = nnz[i];
                if (nnz[i] > 0)
                {
                    memcpy(pi->coff_save[i], coef[i], sizeof(s16) * ((cuw * cuh) >> (i == 0 ? 0 : 2)));
                }
            }
        }
#endif
    }
    else
    {
        if(ctx->sps.tool_amis == 1 && (0
#if AFFINE
           || pidx == AFF_DIR
#endif
           || pidx == PRED_DIR_MMVD
#if MERGE
           || pidx == PRED_DIR
#endif
           ))
        {
#if ATS_INTER_PROCESS
            if (ats_inter_info_match != 0 && ats_inter_info_match != 255 && core->ats_inter_info)
            {
                return MAX_COST;
            }
            continue;
#else
            return MAX_COST;
#endif
        }
#if ATS_INTER_PROCESS
        core->ats_inter_info = 0;
        if (cost_best != MAX_COST)
        {
            continue;
        }
#endif

        for(i = 0; i < N_C; i++)
        {
            nnz[i] = 0;
            evc_mset(core->nnz_sub[i], 0, sizeof(int) * MAX_SUB_TB_NUM);
        }

#if RDO_DBK
        calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pred[0], cuw, x, y, core->avail_lr, 0, 0, pi->refi[pidx], pi->mv[pidx], is_from_mv_field
#if ATS_INTER_PROCESS
                                        , core->ats_inter_info
#endif
        );
#endif
        for(i = 0; i < N_C; i++)
        {
#if ATS_INTER_PROCESS
            dist[0][i] = dist_no_resi[i];
#else
            dist[0][i] = evce_ssd_16b(log2_w[i], log2_h[i], pred[0][i], org[i], w[i], pi->s_o[i]);
#endif
#if RDO_DBK
            dist[0][i] += ctx->delta_dist[i];
#endif
        }
        cost_best = (double)dist[0][Y_C] + (ctx->dist_chroma_weight[0] * (double)dist[0][U_C]) + (ctx->dist_chroma_weight[1] * (double)dist[0][V_C]);

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

        evce_sbac_bit_reset(&core->s_temp_run);

        if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
        {
            pi->mvd[pidx][REFP_0][MV_X] >>= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_0][MV_Y] >>= pi->mvr_idx[pidx];
        }
        if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
        {
            pi->mvd[pidx][REFP_1][MV_X] >>= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_1][MV_Y] >>= pi->mvr_idx[pidx];
        }

        evce_rdo_bit_cnt_cu_inter(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->refi[pidx], pi->mvd[pidx], coef, pidx, mvp_idx, pi->mvr_idx[pidx], pi->bi_idx[pidx]
#if AFFINE
                                  , pi->affine_mvd[pidx]
#endif
        );

        if(IS_INTER_TILE_GROUP(ctx->tgh.tile_group_type) && REFI_IS_VALID(pi->refi[pidx][REFP_0]))
        {
            pi->mvd[pidx][REFP_0][MV_X] <<= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_0][MV_Y] <<= pi->mvr_idx[pidx];
        }
        if(ctx->tgh.tile_group_type == TILE_GROUP_B && REFI_IS_VALID(pi->refi[pidx][REFP_1]))
        {
            pi->mvd[pidx][REFP_1][MV_X] <<= pi->mvr_idx[pidx];
            pi->mvd[pidx][REFP_1][MV_Y] <<= pi->mvr_idx[pidx];
        }

        bit_cnt = evce_get_bit_number(&core->s_temp_run);
        cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
        SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
        assert(core->ats_inter_info == 0);
        ats_inter_info_best = core->ats_inter_info;
        nnz_best[Y_C] = nnz_best[U_C] = nnz_best[V_C] = 0;
        core->cost_best = cost_best < core->cost_best ? cost_best : core->cost_best;
        if (ats_inter_mode_idx == 0)
        {
            dist_ats_inter0 = (s64)(cost_best - (s64)RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt));
            cost_ats_inter0 = cost_best;
            root_cbf_ats_inter0 = 0;
        }
#endif
    }
#if ATS_INTER_PROCESS
    }
#endif

#if ATS_INTER_PROCESS
    if (ats_inter_avail)
    {
        assert(log2_cuw <= MAX_TR_LOG2 && log2_cuh <= MAX_TR_LOG2);
        if (ctx->sps.tool_amis == 1 && (0
#if AFFINE
            || pidx == AFF_DIR
#endif
            || pidx == PRED_DIR_MMVD
#if MERGE
            || pidx == PRED_DIR
#endif
            ))
        {
            if (nnz_best[Y_C] + nnz_best[U_C] + nnz_best[V_C] <= 0)
            {
                core->ats_inter_info = 0;
                return MAX_COST;
            }
        }

        //if no residual, the best mode shall not be ATS_INTER mode
        ats_inter_info_best = (nnz_best[Y_C] + nnz_best[U_C] + nnz_best[V_C] == 0) ? 0 : ats_inter_info_best;
        assert(cost_best != MAX_COST);
        assert(ats_inter_info_best != 255);
        core->ats_inter_info = ats_inter_info_best;
        get_tu_size(core->ats_inter_info, log2_cuw, log2_cuh, &log2_tuw, &log2_tuh);
        for (i = 0; i < N_C; i++)
        {
            int tuw = 1 << log2_tuw;
            int tuh = 1 << log2_tuh;
            assert(nnz_best[i] != -1);
            core->nnz_sub[i][0] = nnz[i] = nnz_best[i];
            if (nnz[i] > 0)
            {
                memcpy(coef[i], pi->coff_save[i], sizeof(s16) * ((tuw * tuh) >> (i == 0 ? 0 : 2)));
#if ATS_INTER_DEBUG
                int num_coef = 0;
                for (int a = 0; a < ((tuw * tuh) >> (i == 0 ? 0 : 2)); a++)
                {
                    num_coef += coef[i][a] != 0;
                }
                assert(num_coef == nnz[i]);
#endif
            }
            else
            {
                evc_mset(coef[i], 0, sizeof(s16) * ((cuw * cuh) >> (i == 0 ? 0 : 2))); //not necessary
            }
        }
        //save the best to history memory
        if (ats_inter_info_match == 255 && num_rdo_tried > 1)
        {
            assert(dist_idx != -1);
            save_ats_inter_info_pred(ctx, core, (u32)dist_idx, ats_inter_info_best, log2_cuw, log2_cuh, x, y);
        }
    }
#endif

    return cost_best;
}

static double analyze_skip_baseline(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh)
{
    EVCE_PINTER *pi = &ctx->pinter;
    pel          *y_org, *u_org, *v_org;
    s16          mvp[REFP_NUM][MV_D];
    s8           refi[REFP_NUM];
    double       cost, cost_best = MAX_COST;
    int          cuw, cuh, idx0, idx1, cnt, bit_cnt;
    s64          cy, cu, cv;

    s16          dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif
    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);
    y_org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
    u_org = pi->o[U_C] + (x >> 1) + ((y >> 1) * pi->s_o[U_C]);
    v_org = pi->o[V_C] + (x >> 1) + ((y >> 1) * pi->s_o[V_C]);

    evc_get_motion(core->scup, REFP_0, ctx->map_refi, ctx->map_mv, pi->refp, cuw, cuh, ctx->w_scu, core->avail_cu, pi->refi_pred[REFP_0], pi->mvp[REFP_0]);
    if(ctx->tile_group_type == TILE_GROUP_B)
    {
        evc_get_motion(core->scup, REFP_1, ctx->map_refi, ctx->map_mv, pi->refp, cuw, cuh, ctx->w_scu, core->avail_cu, pi->refi_pred[REFP_1], pi->mvp[REFP_1]);
    }

    pi->mvp_idx[PRED_SKIP][REFP_0] = 0;
    pi->mvp_idx[PRED_SKIP][REFP_1] = 0;

    for(idx0 = 0; idx0 < 4; idx0++)
    {
        cnt = (ctx->tile_group_type == TILE_GROUP_B ? 4 : 1);
        for(idx1 = 0; idx1 < cnt; idx1++)
        {
            if(idx0 != idx1)
            {
                continue;
            }

            mvp[REFP_0][MV_X] = pi->mvp[REFP_0][idx0][MV_X];
            mvp[REFP_0][MV_Y] = pi->mvp[REFP_0][idx0][MV_Y];
            mvp[REFP_1][MV_X] = pi->mvp[REFP_1][idx1][MV_X];
            mvp[REFP_1][MV_Y] = pi->mvp[REFP_1][idx1][MV_Y];

            SET_REFI(refi, pi->refi_pred[REFP_0][idx0], ctx->tgh.tile_group_type == TILE_GROUP_B ? pi->refi_pred[REFP_1][idx1] : REFI_INVALID);
            if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
            {
                continue;
            }

            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, mvp, pi->refp, pi->pred[PRED_NUM],
                   ctx->ptr, pi->dmvr_template, pi->dmvr_ref_pred_interpolated,
                   pi->dmvr_half_pred_interpolated, FALSE, pi->dmvr_padding_buf, &core->dmvr_flag, dmvr_mv, ctx->sps.tool_amis);

            cy = evce_ssd_16b(log2_cuw, log2_cuh, pi->pred[PRED_NUM][0][Y_C], y_org, cuw, pi->s_o[Y_C]);
            cu = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][U_C], u_org, cuw >> 1, pi->s_o[U_C]);
            cv = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][V_C], v_org, cuw >> 1, pi->s_o[V_C]);

#if RDO_DBK
            calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pi->pred[PRED_NUM][0], cuw, x, y, core->avail_lr, 0, 0, refi, mvp, 0
#if ATS_INTER_PROCESS
                                            , core->ats_inter_info
#endif
            );
            cy += ctx->delta_dist[Y_C];
            cu += ctx->delta_dist[U_C];
            cv += ctx->delta_dist[V_C];
#endif

            cost = (double)cy + (ctx->dist_chroma_weight[0] * (double)cu) + (ctx->dist_chroma_weight[1] * (double)cv);

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

            evce_sbac_bit_reset(&core->s_temp_run);
            evce_rdo_bit_cnt_cu_skip(ctx, core, ctx->tgh.tile_group_type, core->scup, idx0, idx1, 0, ctx->sps.tool_mmvd);

            bit_cnt = evce_get_bit_number(&core->s_temp_run);
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

            if(cost < cost_best)
            {
                int j;
                cost_best = cost;
                pi->mvp_idx[PRED_SKIP][REFP_0] = idx0;
                pi->mvp_idx[PRED_SKIP][REFP_1] = idx1;
                pi->mv[PRED_SKIP][REFP_0][MV_X] = mvp[REFP_0][MV_X];
                pi->mv[PRED_SKIP][REFP_0][MV_Y] = mvp[REFP_0][MV_Y];
                pi->mv[PRED_SKIP][REFP_1][MV_X] = mvp[REFP_1][MV_X];
                pi->mv[PRED_SKIP][REFP_1][MV_Y] = mvp[REFP_1][MV_Y];
                pi->mvd[PRED_SKIP][REFP_0][MV_X] = 0;
                pi->mvd[PRED_SKIP][REFP_0][MV_Y] = 0;
                pi->mvd[PRED_SKIP][REFP_1][MV_X] = 0;
                pi->mvd[PRED_SKIP][REFP_1][MV_Y] = 0;
                pi->refi[PRED_SKIP][REFP_0] = refi[REFP_0];
                pi->refi[PRED_SKIP][REFP_1] = refi[REFP_1];
#if ATS_INTER_PROCESS
                core->cost_best = cost < core->cost_best ? cost : core->cost_best;
#endif

                for(j = 0; j < N_C; j++)
                {
                    int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                    evc_mcpy(pi->pred[PRED_SKIP][0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                }

                SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
                pi->ats_inter_info_mode[PRED_SKIP] = 0;
#endif
            }
        }
    }

    return cost_best;
}

static double analyze_skip(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh)
{
    EVCE_PINTER *pi = &ctx->pinter;
    pel          *y_org, *u_org, *v_org;
    s16          mvp[REFP_NUM][MV_D];
#if DMVR_LAG
    s16          dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];    
#endif
#if DMVR_FLAG
    int          best_dmvr = 0;
#endif
    s8           refi[REFP_NUM];
    double       cost, cost_best = MAX_COST;
#if MERGE
    double       ad_best_costs[MAX_NUM_MVP];
    int          j;
#endif
    int          cuw, cuh, idx0, idx1, cnt, bit_cnt;
    s64          cy, cu, cv;
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);
    y_org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
    u_org = pi->o[U_C] + (x >> 1) + ((y >> 1) * pi->s_o[U_C]);
    v_org = pi->o[V_C] + (x >> 1) + ((y >> 1) * pi->s_o[V_C]);

    core->mmvd_flag = 0;

#if MERGE
    for(j = 0; j < MAX_NUM_MVP; j++)
    {
        ad_best_costs[j] = MAX_COST;
    }
#endif

#if ADMVP
    if(ctx->sps.tool_admvp == 0)
        evc_get_motion_skip(ctx->ptr, ctx->tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, pi->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, pi->refi_pred, pi->mvp, ctx->map_scu, core->avail_cu
                            , ctx->map_unrefined_mv
                            , core->history_buffer, ctx->sps.tool_admvp
#if USE_IBC
          , core->ibc_flag
#endif
        );
    else
#endif
        evc_get_motion_skip(ctx->ptr, ctx->tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, pi->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, pi->refi_pred, pi->mvp, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                            , ctx->map_unrefined_mv
#endif
#if ADMVP
                            , core->history_buffer, ctx->sps.tool_admvp
#endif
#if USE_IBC
          , core->ibc_flag
#endif

        );

    pi->mvp_idx[PRED_SKIP][REFP_0] = 0;
    pi->mvp_idx[PRED_SKIP][REFP_1] = 0;
#if ADMVP
    for (idx0 = 0; idx0 < (cuw*cuh<= NUM_SAMPLES_BLOCK ? MAX_NUM_MVP_SMALL_CU:MAX_NUM_MVP); idx0++)
#else
    for(idx0 = 0; idx0 < MAX_NUM_MVP; idx0++)
#endif
    {
#if ADMVP
        cnt = (ctx->tile_group_type == TILE_GROUP_B ? (cuw * cuh <= NUM_SAMPLES_BLOCK ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP) : 1);
#else
        cnt = (ctx->tile_group_type == TILE_GROUP_B ? MAX_NUM_MVP : 1);
#endif
        for(idx1 = 0; idx1 < cnt; idx1++)
        {
            if(idx0 != idx1)
            {
                continue;
            }

            mvp[REFP_0][MV_X] = pi->mvp[REFP_0][idx0][MV_X];
            mvp[REFP_0][MV_Y] = pi->mvp[REFP_0][idx0][MV_Y];
            mvp[REFP_1][MV_X] = pi->mvp[REFP_1][idx1][MV_X];
            mvp[REFP_1][MV_Y] = pi->mvp[REFP_1][idx1][MV_Y];

            SET_REFI(refi, pi->refi_pred[REFP_0][idx0], ctx->tgh.tile_group_type == TILE_GROUP_B ? pi->refi_pred[REFP_1][idx1] : REFI_INVALID);
            if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
            {
                continue;
            }

#if DMVR
            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, mvp, pi->refp, pi->pred[PRED_NUM], ctx->ptr, pi->dmvr_template, pi->dmvr_ref_pred_interpolated
                   , pi->dmvr_half_pred_interpolated, TRUE && ctx->sps.tool_dmvr
#if DMVR_PADDING
                   , pi->dmvr_padding_buf
#endif
#if DMVR_FLAG
                   , &core->dmvr_flag
#if DMVR_LAG
                   , dmvr_mv
#endif
#endif
                   , ctx->sps.tool_amis
#else
            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, mvp, pi->refp, pi->pred[PRED_NUM]
#endif
            );

            cy = evce_ssd_16b(log2_cuw, log2_cuh, pi->pred[PRED_NUM][0][Y_C], y_org, cuw, pi->s_o[Y_C]);
            cu = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][U_C], u_org, cuw >> 1, pi->s_o[U_C]);
            cv = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][V_C], v_org, cuw >> 1, pi->s_o[V_C]);

#if RDO_DBK
            calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pi->pred[PRED_NUM][0], cuw, x, y, core->avail_lr, 0, 0, refi, mvp, 0
#if ATS_INTER_PROCESS
                                            , core->ats_inter_info
#endif
            );
            cy += ctx->delta_dist[Y_C];
            cu += ctx->delta_dist[U_C];
            cv += ctx->delta_dist[V_C];
#endif

            cost = (double)cy + (ctx->dist_chroma_weight[0] * (double)cu) + (ctx->dist_chroma_weight[1] * (double)cv);

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

            evce_sbac_bit_reset(&core->s_temp_run);
            evce_rdo_bit_cnt_cu_skip(ctx, core, ctx->tgh.tile_group_type, core->scup, idx0, idx1, 0, ctx->sps.tool_mmvd);

            bit_cnt = evce_get_bit_number(&core->s_temp_run);
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

            if(cost < cost_best)
            {
                int j;
                cost_best = cost;
                pi->mvp_idx[PRED_SKIP][REFP_0] = idx0;
                pi->mvp_idx[PRED_SKIP][REFP_1] = idx1;
                pi->mv[PRED_SKIP][REFP_0][MV_X] = mvp[REFP_0][MV_X];
                pi->mv[PRED_SKIP][REFP_0][MV_Y] = mvp[REFP_0][MV_Y];
                pi->mv[PRED_SKIP][REFP_1][MV_X] = mvp[REFP_1][MV_X];
                pi->mv[PRED_SKIP][REFP_1][MV_Y] = mvp[REFP_1][MV_Y];
#if ATS_INTER_PROCESS
                core->cost_best = cost < core->cost_best ? cost : core->cost_best;
#endif
#if DMVR_FLAG
                best_dmvr = core->dmvr_flag;
                core->dmvr_flag = 0;
#endif
#if DMVR_LAG                
                if(best_dmvr)
                {
                    u16 idx = 0, i, j;
                    for(j = 0; j < core->cuh >> MIN_CU_LOG2; j++)
                    {
                        for(i = 0; i < core->cuw >> MIN_CU_LOG2; i++)
                        {
                            pi->dmvr_mv[PRED_SKIP][idx + i][REFP_0][MV_X] = dmvr_mv[idx + i][REFP_0][MV_X];
                            pi->dmvr_mv[PRED_SKIP][idx + i][REFP_0][MV_Y] = dmvr_mv[idx + i][REFP_0][MV_Y];
                            pi->dmvr_mv[PRED_SKIP][idx + i][REFP_1][MV_X] = dmvr_mv[idx + i][REFP_1][MV_X];
                            pi->dmvr_mv[PRED_SKIP][idx + i][REFP_1][MV_Y] = dmvr_mv[idx + i][REFP_1][MV_Y];
                        }
                        idx += core->cuw >> MIN_CU_LOG2;
                    }
                }
#endif
                pi->mvd[PRED_SKIP][REFP_0][MV_X] = 0;
                pi->mvd[PRED_SKIP][REFP_0][MV_Y] = 0;
                pi->mvd[PRED_SKIP][REFP_1][MV_X] = 0;
                pi->mvd[PRED_SKIP][REFP_1][MV_Y] = 0;
                pi->refi[PRED_SKIP][REFP_0] = refi[REFP_0];
                pi->refi[PRED_SKIP][REFP_1] = refi[REFP_1];

                for(j = 0; j < N_C; j++)
                {
                    int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                    evc_mcpy(pi->pred[PRED_SKIP][0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                }

                SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
                pi->ats_inter_info_mode[PRED_SKIP] = 0;
#endif
            }
#if MERGE
            ad_best_costs[idx0] = cost;
#endif
        }
    }

#if MERGE
    {
        assert(ctx->tile_group_type == TILE_GROUP_B);
        /* removes the cost above threshold and remove the duplicates */
#if ADMVP
        for (idx0 = 0; idx0 < (cuw * cuh <= NUM_SAMPLES_BLOCK ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP); idx0++)
#else
        for(idx0 = 0; idx0 < MAX_NUM_MVP; idx0++)
#endif
        {
            /* removes the cost above threshold */
            if(ad_best_costs[idx0] > (cost_best * FAST_MERGE_THR))
                core->au8_eval_mvp_idx[idx0] = 0;
            else
                core->au8_eval_mvp_idx[idx0] = 1;
        }

        /* remove the duplicates and keep the best */
#if ADMVP
        for (idx0 = 0; idx0 < (cuw * cuh <= NUM_SAMPLES_BLOCK ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP); idx0++)
#else
        for (idx0 = 0; idx0 < MAX_NUM_MVP; idx0++)
#endif
        {
            if(core->au8_eval_mvp_idx[idx0] == 1)
            {
#if ADMVP
                for (j = idx0 + 1; j < (cuw * cuh <= NUM_SAMPLES_BLOCK ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP); j++)
#else
                for(j = idx0 + 1; j < MAX_NUM_MVP; j++)
#endif
                {
                    if(core->au8_eval_mvp_idx[j] == 1)
                    {
                        u8 is_duplicate = 0;
                        if(pi->refi_pred[REFP_0][idx0] == pi->refi_pred[REFP_0][j])
                        {
                            if((pi->mvp[REFP_0][idx0][MV_X] == pi->mvp[REFP_0][j][MV_X]) &&
                               (pi->mvp[REFP_0][idx0][MV_Y] == pi->mvp[REFP_0][j][MV_Y]))
                            {
                                is_duplicate = 1;
                            }
                        }
                        if(is_duplicate && (pi->refi_pred[REFP_1][idx0] == pi->refi_pred[REFP_1][j]))
                        {
                            if((pi->mvp[REFP_1][idx0][MV_X] == pi->mvp[REFP_1][j][MV_X]) &&
                               (pi->mvp[REFP_1][idx0][MV_Y] == pi->mvp[REFP_1][j][MV_Y]))
                            {
                                if(ad_best_costs[j] < ad_best_costs[idx0])
                                {
                                    /* removes idx0 */
                                    core->au8_eval_mvp_idx[idx0] = 0;
                                    break;
                                }
                                else
                                {
                                    /* removes j */
                                    core->au8_eval_mvp_idx[j] = 0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
#endif
#if DMVR_FLAG
     core->dmvr_flag = best_dmvr;
#endif
    return cost_best;
}

#if MERGE
static double analyze_merge(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh)
{
    EVCE_PINTER *pi = &ctx->pinter;
    s16          mvp[REFP_NUM][MV_D];
    s8           refi[REFP_NUM];
    double       cost, cost_best = MAX_COST;
    int          cuw, cuh, idx0;
    int          j;
    int          pidx = PRED_DIR, pidx1 = PRED_NUM;
    int          tmp_mvp0 = 0, tmp_mvp1 = 0, tmp_mvx0 = 0, tmp_mvx1 = 0, tmp_mvy0 = 0, tmp_mvy1 = 0, tmp_ref0 = 0, tmp_ref1 = 0;
#if DMVR_LAG
    int          tmp_dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
#endif

#if DMVR
    BOOL apply_dmvr;
#endif
#if DMVR_FLAG
    int best_dmvr = 0;
#endif
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);
    core->mmvd_flag = 0;

#if ADMVP
    if(ctx->sps.tool_admvp == 0)
        evc_get_motion_skip(ctx->ptr, ctx->tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, pi->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, pi->refi_pred, pi->mvp, ctx->map_scu, core->avail_cu
                            , ctx->map_unrefined_mv
                            , core->history_buffer, ctx->sps.tool_admvp
#if USE_IBC
          , core->ibc_flag
#endif
        );
    else
#endif
        evc_get_motion_skip(ctx->ptr, ctx->tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, pi->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, pi->refi_pred, pi->mvp, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                            , ctx->map_unrefined_mv
#endif
#if ADMVP
                            , core->history_buffer, ctx->sps.tool_admvp
#endif
#if USE_IBC
          , core->ibc_flag
#endif
        );
#if ADMVP
    for(idx0 = 0; idx0 < (cuw*cuh <= NUM_SAMPLES_BLOCK ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP); idx0++)
#else
    for(idx0 = 0; idx0 < MAX_NUM_MVP; idx0++)
#endif
    {
#if MERGE
        if(0 == core->au8_eval_mvp_idx[idx0])
        {
            continue;
        }
#endif
        mvp[REFP_0][MV_X] = pi->mvp[REFP_0][idx0][MV_X];
        mvp[REFP_0][MV_Y] = pi->mvp[REFP_0][idx0][MV_Y];
        mvp[REFP_1][MV_X] = pi->mvp[REFP_1][idx0][MV_X];
        mvp[REFP_1][MV_Y] = pi->mvp[REFP_1][idx0][MV_Y];

        SET_REFI(refi, pi->refi_pred[REFP_0][idx0], ctx->tgh.tile_group_type == TILE_GROUP_B ? pi->refi_pred[REFP_1][idx0] : REFI_INVALID);
        if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }

        pi->mvp_idx[pidx][REFP_0] = idx0;
        pi->mvp_idx[pidx][REFP_1] = idx0;
        pi->mv[pidx][REFP_0][MV_X] = mvp[REFP_0][MV_X];
        pi->mv[pidx][REFP_0][MV_Y] = mvp[REFP_0][MV_Y];
        pi->mv[pidx][REFP_1][MV_X] = mvp[REFP_1][MV_X];
        pi->mv[pidx][REFP_1][MV_Y] = mvp[REFP_1][MV_Y];
        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;
        pi->refi[pidx][REFP_0] = refi[REFP_0];
        pi->refi[pidx][REFP_1] = refi[REFP_1];

        apply_dmvr = TRUE;
        cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pi->coef[pidx], pidx, pi->mvp_idx[pidx]
#if DMVR
                                  , apply_dmvr
#endif
        );

        if(cost < cost_best)
        {
            tmp_mvp0 = idx0;
            tmp_mvp1 = idx0;
            tmp_mvx0 = pi->mv[pidx][REFP_0][MV_X];
            tmp_mvy0 = pi->mv[pidx][REFP_0][MV_Y];
            tmp_mvx1 = pi->mv[pidx][REFP_1][MV_X];
            tmp_mvy1 = pi->mv[pidx][REFP_1][MV_Y];
#if DMVR_FLAG
            best_dmvr = core->dmvr_flag;
            core->dmvr_flag = 0;
#endif
#if DMVR_LAG
            if(best_dmvr)
            {
                u16 idx = 0, i, j;
                for(j = 0; j < core->cuh >> MIN_CU_LOG2; j++)
                {
                    for(i = 0; i < core->cuw >> MIN_CU_LOG2; i++)
                    {
                        tmp_dmvr_mv[idx + i][REFP_0][MV_X] = pi->dmvr_mv[pidx][idx + i][REFP_0][MV_X];
                        tmp_dmvr_mv[idx + i][REFP_0][MV_Y] = pi->dmvr_mv[pidx][idx + i][REFP_0][MV_Y];
                        tmp_dmvr_mv[idx + i][REFP_1][MV_X] = pi->dmvr_mv[pidx][idx + i][REFP_1][MV_X];
                        tmp_dmvr_mv[idx + i][REFP_1][MV_Y] = pi->dmvr_mv[pidx][idx + i][REFP_1][MV_Y];
                    }
                    idx += core->cuw >> MIN_CU_LOG2;
                }
            }
#endif
            tmp_ref0 = pi->refi[pidx][REFP_0];
            tmp_ref1 = pi->refi[pidx][REFP_1];

            cost_best = cost;

            for(j = 0; j < N_C; j++)
            {
                int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                pi->nnz_best[pidx][j] = core->nnz[j];
                evc_mcpy(pi->nnz_sub_best[pidx][j], core->nnz_sub[j], MAX_SUB_TB_NUM * sizeof(int));
                evc_mcpy(pi->pred[pidx1][0][j], pi->pred[pidx][0][j], size_tmp * sizeof(pel));
                evc_mcpy(pi->coef[pidx1][j], pi->coef[pidx][j], size_tmp * sizeof(s16));
            }
            SBAC_STORE(core->s_temp_best_merge, core->s_temp_best);
#if ATS_INTER_PROCESS
            pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
        }
    }

    pi->mvp_idx[pidx][REFP_0] = tmp_mvp0;
    pi->mvp_idx[pidx][REFP_1] = tmp_mvp1;
    pi->mv[pidx][REFP_0][MV_X] = tmp_mvx0;
    pi->mv[pidx][REFP_0][MV_Y] = tmp_mvy0;
    pi->mv[pidx][REFP_1][MV_X] = tmp_mvx1;
    pi->mv[pidx][REFP_1][MV_Y] = tmp_mvy1;
#if DMVR_FLAG
    core->dmvr_flag = best_dmvr;
#endif
#if DMVR_LAG
    if(core->dmvr_flag)
    {
        u16 idx = 0, i, j;
        for(j = 0; j < core->cuh >> MIN_CU_LOG2; j++)
        {
            for(i = 0; i < core->cuw >> MIN_CU_LOG2; i++)
            {
                pi->dmvr_mv[pidx][idx + i][REFP_0][MV_X] = tmp_dmvr_mv[idx + i][REFP_0][MV_X];
                pi->dmvr_mv[pidx][idx + i][REFP_0][MV_Y] = tmp_dmvr_mv[idx + i][REFP_0][MV_Y];
                pi->dmvr_mv[pidx][idx + i][REFP_1][MV_X] = tmp_dmvr_mv[idx + i][REFP_1][MV_X];
                pi->dmvr_mv[pidx][idx + i][REFP_1][MV_Y] = tmp_dmvr_mv[idx + i][REFP_1][MV_Y];
            }
            idx += core->cuw >> MIN_CU_LOG2;
        }
    }
#endif
    pi->mvd[pidx][REFP_0][MV_X] = 0;
    pi->mvd[pidx][REFP_0][MV_Y] = 0;
    pi->mvd[pidx][REFP_1][MV_X] = 0;
    pi->mvd[pidx][REFP_1][MV_Y] = 0;
    pi->refi[pidx][REFP_0] = tmp_ref0;
    pi->refi[pidx][REFP_1] = tmp_ref1;

    return cost_best;
}
#endif

static double analyze_t_direct(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh)
{
    EVCE_PINTER *pi = &ctx->pinter;
    double      cost;
    int         pidx;
    s8          refidx = 0;
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    pidx = PRED_DIR;
    evc_get_mv_dir(pi->refp[0], ctx->ptr, core->scup + ((1 << (log2_cuw - MIN_CU_LOG2)) - 1) + ((1 << (log2_cuh - MIN_CU_LOG2)) - 1) * ctx->w_scu, core->scup, ctx->w_scu, ctx->h_scu, pi->mv[pidx], &refidx, ctx->sps.tool_admvp);

    pi->mvd[pidx][REFP_0][MV_X] = 0;
    pi->mvd[pidx][REFP_0][MV_Y] = 0;
    pi->mvd[pidx][REFP_1][MV_X] = 0;
    pi->mvd[pidx][REFP_1][MV_Y] = 0;

    SET_REFI(pi->refi[pidx], 0, 0);

    cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pi->coef[pidx], pidx, pi->mvp_idx[pidx], FALSE);

    evc_mcpy(pi->nnz_best[pidx], core->nnz, sizeof(int) * N_C);
#if ATS_INTER_PROCESS
    pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
    return cost;
}

static double analyze_skip_mmvd(EVCE_CTX * ctx, EVCE_CORE * core, int x, int y, int log2_cuw, int log2_cuh, int real_mv[][2][3])
{
    EVCE_PINTER *pi = &ctx->pinter;
    pel          *y_org, *u_org, *v_org;
    s16           mvp[REFP_NUM][MV_D];
    s8            refi[REFP_NUM];
    double        cost, cost_best = MAX_COST;
    int           cuw, cuh, bit_cnt;
    s64           cy, cu, cv;
    int           c_num = 0;
    int           t_base_num = 0;
    int           best_idx_num = -1;
    int           iii;
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    core->mmvd_flag = 1;

    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);
    y_org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
    u_org = pi->o[U_C] + (x >> 1) + ((y >> 1) * pi->s_o[U_C]);
    v_org = pi->o[V_C] + (x >> 1) + ((y >> 1) * pi->s_o[V_C]);

    pi->mvp_idx[PRED_SKIP_MMVD][REFP_0] = 0;
    pi->mvp_idx[PRED_SKIP_MMVD][REFP_1] = 0;

    t_base_num = MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM;

    if((pi->tile_group_type == TILE_GROUP_B) || (pi->tile_group_type == TILE_GROUP_P))
    {
        t_base_num = pi->best_index[PRED_DIR_MMVD][MMVD_SKIP_CON_NUM - 1];
    }
    for(iii = 0; iii < t_base_num; iii++)
    {
        if((pi->tile_group_type == TILE_GROUP_B) || (pi->tile_group_type == TILE_GROUP_P))
        {
            c_num = pi->best_index[PRED_DIR_MMVD][iii];
        }
        else
        {
            c_num = iii;
        }
        if(c_num == -1)
        {
            continue;
        }

        mvp[REFP_0][MV_X] = real_mv[c_num][0][MV_X];
        mvp[REFP_0][MV_Y] = real_mv[c_num][0][MV_Y];
        mvp[REFP_1][MV_X] = real_mv[c_num][1][MV_X];
        mvp[REFP_1][MV_Y] = real_mv[c_num][1][MV_Y];
        if((real_mv[c_num][0][2] == -1) && (real_mv[c_num][1][2] == -1))
        {
            continue;
        }
        pi->refi[PRED_SKIP_MMVD][0] = real_mv[c_num][0][2];
        pi->refi[PRED_SKIP_MMVD][1] = real_mv[c_num][1][2];

        SET_REFI(refi, real_mv[c_num][0][2], ctx->tgh.tile_group_type == TILE_GROUP_B ? real_mv[c_num][1][2] : REFI_INVALID);
        if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }

#if DMVR
        evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, mvp, pi->refp, pi->pred[PRED_NUM], ctx->ptr, pi->dmvr_template, pi->dmvr_ref_pred_interpolated
               , pi->dmvr_half_pred_interpolated
               , FALSE
#if DMVR_PADDING
               , pi->dmvr_padding_buf
#endif
#if DMVR_FLAG
               , &core->dmvr_flag
#if DMVR_LAG
               , NULL
#endif
#endif
               , ctx->sps.tool_amis
#else
        evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, mvp, pi->refp, pi->pred[PRED_NUM]
#endif
        );
        
        cy = evce_ssd_16b(log2_cuw, log2_cuh, pi->pred[PRED_NUM][0][Y_C], y_org, cuw, pi->s_o[Y_C]);
        cu = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][U_C], u_org, cuw >> 1, pi->s_o[U_C]);
        cv = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][V_C], v_org, cuw >> 1, pi->s_o[V_C]);

#if RDO_DBK
        calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pi->pred[PRED_NUM][0], cuw, x, y, core->avail_lr, 0, 0, refi, mvp, 0
#if ATS_INTER_PROCESS
                                        , core->ats_inter_info
#endif
        );
        cy += ctx->delta_dist[Y_C];
        cu += ctx->delta_dist[U_C];
        cv += ctx->delta_dist[V_C];
#endif

        cost = (double)cy + (ctx->dist_chroma_weight[0] * (double)cu) + (ctx->dist_chroma_weight[1] * (double)cv);

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

        evce_sbac_bit_reset(&core->s_temp_run);
        evce_rdo_bit_cnt_cu_skip(ctx, core, ctx->tgh.tile_group_type, core->scup, 0, 0, c_num, ctx->sps.tool_mmvd);
        bit_cnt = evce_get_bit_number(&core->s_temp_run);
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

        if(cost < cost_best)
        {
            int j;
            cost_best = cost;
            best_idx_num = c_num;
            pi->mmvd_idx[PRED_SKIP_MMVD] = c_num;

            pi->mv[PRED_SKIP_MMVD][REFP_0][MV_X] = mvp[REFP_0][MV_X];
            pi->mv[PRED_SKIP_MMVD][REFP_0][MV_Y] = mvp[REFP_0][MV_Y];
            pi->mv[PRED_SKIP_MMVD][REFP_1][MV_X] = mvp[REFP_1][MV_X];
            pi->mv[PRED_SKIP_MMVD][REFP_1][MV_Y] = mvp[REFP_1][MV_Y];
            pi->refi[PRED_SKIP_MMVD][REFP_0] = refi[REFP_0];
            pi->refi[PRED_SKIP_MMVD][REFP_1] = refi[REFP_1];
#if ATS_INTER_PROCESS
            core->cost_best = cost < core->cost_best ? cost : core->cost_best;
#endif

            for(j = 0; j < N_C; j++)
            {
                int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                evc_mcpy(pi->pred[PRED_SKIP_MMVD][0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
            }
            SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if ATS_INTER_PROCESS
            pi->ats_inter_info_mode[PRED_SKIP_MMVD] = 0;
#endif
        }
    }
    mvp[REFP_0][MV_X] = real_mv[best_idx_num][0][MV_X];
    mvp[REFP_0][MV_Y] = real_mv[best_idx_num][0][MV_Y];
    mvp[REFP_1][MV_X] = real_mv[best_idx_num][1][MV_X];
    mvp[REFP_1][MV_Y] = real_mv[best_idx_num][1][MV_Y];
    pi->refi[PRED_SKIP_MMVD][REFP_0] = real_mv[best_idx_num][0][2];
    pi->refi[PRED_SKIP_MMVD][REFP_1] = real_mv[best_idx_num][1][2];

    pi->mvd[PRED_SKIP_MMVD][REFP_0][MV_X] = 0;
    pi->mvd[PRED_SKIP_MMVD][REFP_0][MV_Y] = 0;
    pi->mvd[PRED_SKIP_MMVD][REFP_1][MV_X] = 0;
    pi->mvd[PRED_SKIP_MMVD][REFP_1][MV_Y] = 0;

    return cost_best;
}

static double analyze_merge_mmvd(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, int real_mv[][2][3])
{
    EVCE_PINTER *pi = &ctx->pinter;
    s8  refi[REFP_NUM];
    int pidx, i;
    int c_num = 0;
    int t_base_num = 0;
    double direct_cost[10];
    int current_idx = 0;
    double min_cost = MAX_COST;
    double temp_cost = 0.0;
    int moving_index = 0;
    int current_array[MMVD_GRP_NUM * MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM];
    int i1, i2, i3;
    int cur_temp = 0;
    int ttt = 0;
    int consider_num = 0;
    int best_candi = -1;
    double min_cost_temp = MAX_COST;
#if ATS_INTER_PROCESS
    double cost_best_save = core->cost_best;
    core->ats_inter_info = 0;
#endif

    pidx = PRED_DIR_MMVD;
    SET_REFI(pi->refi[pidx], 0, ctx->tgh.tile_group_type == TILE_GROUP_B ? 0 : REFI_INVALID);

    for (i = 0; i < MMVD_SKIP_CON_NUM; i++)
    {
        pi->best_index[pidx][i] = -1;
        direct_cost[i] = MAX_COST;
    }
    t_base_num = MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM;

    pi->mvp_idx[pidx][REFP_0] = 0;
    pi->mvp_idx[pidx][REFP_1] = 0;


    for(i1 = 0; i1 < MMVD_DIST_NUM; i1++)
    {
        for(i2 = 0; i2 < MMVD_BASE_MV_NUM; i2++)
        {
            for(i3 = 0; i3 < 4; i3++)
            {
                int idx_tmp = i1 * 4 + i2 * MMVD_MAX_REFINE_NUM + i3;
                current_array[cur_temp] = idx_tmp;
                current_array[cur_temp + t_base_num] = idx_tmp + t_base_num;
                current_array[cur_temp + t_base_num * 2] = idx_tmp + t_base_num * 2;
                cur_temp++;
            }
        }
    }
    
    for(moving_index = 0; moving_index < t_base_num; moving_index++)
    {
        int temp;
        int i2_t;
        int i1_t;

        c_num = current_array[moving_index];
        i2_t = c_num / 24;
        temp = c_num - i2_t * 24;
        i1_t = temp / 4;
        temp = temp - i1_t * 4;

        pi->mv[pidx][REFP_0][MV_X] = real_mv[c_num][0][MV_X];
        pi->mv[pidx][REFP_0][MV_Y] = real_mv[c_num][0][MV_Y];
        pi->mv[pidx][REFP_1][MV_X] = real_mv[c_num][1][MV_X];
        pi->mv[pidx][REFP_1][MV_Y] = real_mv[c_num][1][MV_Y];

        if((real_mv[c_num][0][2] == -1) && (real_mv[c_num][1][2] == -1))
        {
            continue;
        }
        pi->refi[pidx][0] = real_mv[c_num][0][2];
        pi->refi[pidx][1] = real_mv[c_num][1][2];

        SET_REFI(refi, real_mv[c_num][0][2], ctx->tgh.tile_group_type == TILE_GROUP_B ? real_mv[c_num][1][2] : REFI_INVALID);
        if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }

        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;

        pi->mmvd_idx[pidx] = c_num;

        temp_cost = pinter_residue_rdo_mmvd(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pidx);

        if(temp_cost < direct_cost[current_idx])
        {
            int s, t;
            if(current_idx == 0)
            {
                direct_cost[current_idx] = temp_cost;
                pi->best_index[pidx][current_idx] = c_num;
                current_idx++;
            }
            else
            {
                int insert_or_not = -1;
                for(s = current_idx; s >= 1; s--)
                {
                    if((direct_cost[s] > temp_cost) && (temp_cost >= direct_cost[s - 1]))
                    {
                        t = current_idx;
                        do
                        {
                            direct_cost[t] = direct_cost[t - 1];
                            pi->best_index[pidx][t] = pi->best_index[pidx][t - 1];
                            t = t - 1;
                        } while(t >= s);

                        direct_cost[s] = temp_cost;
                        pi->best_index[pidx][s] = c_num;

                        current_idx++;
                        if (current_idx >= MMVD_SKIP_CON_NUM)
                        {
                            current_idx = (MMVD_SKIP_CON_NUM - 1);
                        }
                        insert_or_not = 1;
                        break;
                    }
                }
                if(insert_or_not != 1)
                {
                    for(t = current_idx; t >= 1; t--)
                    {
                        direct_cost[t] = direct_cost[t - 1];
                        pi->best_index[pidx][t] = pi->best_index[pidx][t - 1];
                    }
                    direct_cost[0] = temp_cost;
                    pi->best_index[pidx][0] = c_num;

                    current_idx++;
                    if (current_idx >= MMVD_SKIP_CON_NUM)
                    {
                        current_idx = (MMVD_SKIP_CON_NUM - 1);
                    }
                }
            }
        }
    }

    for(moving_index = t_base_num; moving_index < 2 * t_base_num; moving_index++)
    {
        int temp;
        int i2_t;
        int i1_t;
        int i0_t;
        c_num = current_array[moving_index];
        i0_t = c_num - 1 * (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);
        i2_t = i0_t / MMVD_MAX_REFINE_NUM;
        temp = i0_t - i2_t*MMVD_MAX_REFINE_NUM;
        i1_t = temp / 4;
        temp = temp - i1_t * 4;

        if(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr)
        {
            continue;
        }

        pi->mv[pidx][REFP_0][MV_X] = real_mv[c_num][0][MV_X];
        pi->mv[pidx][REFP_0][MV_Y] = real_mv[c_num][0][MV_Y];
        pi->mv[pidx][REFP_1][MV_X] = real_mv[c_num][1][MV_X];
        pi->mv[pidx][REFP_1][MV_Y] = real_mv[c_num][1][MV_Y];

        if ((real_mv[c_num][0][2] == -1) && (real_mv[c_num][1][2] == -1))
        {
            continue;
        }
        pi->refi[pidx][0] = real_mv[c_num][0][2];
        pi->refi[pidx][1] = real_mv[c_num][1][2];

        SET_REFI(refi, real_mv[c_num][0][2], ctx->tgh.tile_group_type == TILE_GROUP_B ? real_mv[c_num][1][2] : REFI_INVALID);
        if (!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }

        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;

        pi->mmvd_idx[pidx] = c_num;

        temp_cost = pinter_residue_rdo_mmvd(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pidx);

        if(temp_cost < direct_cost[current_idx])
        {
            int s, t;
            if(current_idx == 0)
            {
                direct_cost[current_idx] = temp_cost;
                pi->best_index[pidx][current_idx] = c_num;
                current_idx++;
            }
            else
            {
                int insert_or_not = -1;
                for(s = current_idx; s >= 1; s--)
                {
                    if((direct_cost[s] > temp_cost) && (temp_cost >= direct_cost[s - 1]))
                    {
                        t = current_idx;
                        do
                        {
                            direct_cost[t] = direct_cost[t - 1];
                            pi->best_index[pidx][t] = pi->best_index[pidx][t - 1];
                            t = t - 1;
                        } while(t >= s);

                        direct_cost[s] = temp_cost;
                        pi->best_index[pidx][s] = c_num;

                        current_idx++;
                        if(current_idx >= MMVD_SKIP_CON_NUM)
                        {
                            current_idx = (MMVD_SKIP_CON_NUM - 1);
                        }
                        insert_or_not = 1;
                        break;
                    }
                }
                if(insert_or_not != 1)
                {
                    for(t = current_idx; t >= 1; t--)
                    {
                        direct_cost[t] = direct_cost[t - 1];
                        pi->best_index[pidx][t] = pi->best_index[pidx][t - 1];
                    }
                    direct_cost[0] = temp_cost;
                    pi->best_index[pidx][0] = c_num;

                    current_idx++;
                    if(current_idx >= MMVD_SKIP_CON_NUM)
                    {
                        current_idx = (MMVD_SKIP_CON_NUM - 1);
                    }
                }
            }
        }
    }

    for(moving_index = 2 * t_base_num; moving_index < 3 * t_base_num; moving_index++)
    {
        int temp;
        int i2_t;
        int i1_t;
        int i0_t;
        c_num = current_array[moving_index];
        i0_t = c_num - 2 * (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);
        i2_t = i0_t / MMVD_MAX_REFINE_NUM;
        temp = i0_t - i2_t*MMVD_MAX_REFINE_NUM;
        i1_t = temp / 4;
        temp = temp - i1_t * 4;

        if(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr)
        {
            continue;
        }

        pi->mv[pidx][REFP_0][MV_X] = real_mv[c_num][0][MV_X];
        pi->mv[pidx][REFP_0][MV_Y] = real_mv[c_num][0][MV_Y];
        pi->mv[pidx][REFP_1][MV_X] = real_mv[c_num][1][MV_X];
        pi->mv[pidx][REFP_1][MV_Y] = real_mv[c_num][1][MV_Y];

        if ((real_mv[c_num][0][2] == -1) && (real_mv[c_num][1][2] == -1))
        {
            continue;
        }
        pi->refi[pidx][0] = real_mv[c_num][0][2];
        pi->refi[pidx][1] = real_mv[c_num][1][2];

        SET_REFI(refi, real_mv[c_num][0][2], ctx->tgh.tile_group_type == TILE_GROUP_B ? real_mv[c_num][1][2] : REFI_INVALID);
        if (!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }

        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;

        pi->mmvd_idx[pidx] = c_num;

        temp_cost = pinter_residue_rdo_mmvd(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pidx);

        if(temp_cost < direct_cost[current_idx])
        {
            int s, t;
            if(current_idx == 0)
            {
                direct_cost[current_idx] = temp_cost;
                pi->best_index[pidx][current_idx] = c_num;
                current_idx++;
            }
            else
            {
                int insert_or_not = -1;
                for(s = current_idx; s >= 1; s--)
                {
                    if((direct_cost[s] > temp_cost) && (temp_cost >= direct_cost[s - 1]))
                    {
                        t = current_idx;
                        do
                        {
                            direct_cost[t] = direct_cost[t - 1];
                            pi->best_index[pidx][t] = pi->best_index[pidx][t - 1];
                            t = t - 1;
                        } while(t >= s);

                        direct_cost[s] = temp_cost;
                        pi->best_index[pidx][s] = c_num;

                        current_idx++;
                        if(current_idx >= MMVD_SKIP_CON_NUM)
                        {
                            current_idx = (MMVD_SKIP_CON_NUM - 1);
                        }
                        insert_or_not = 1;
                        break;
                    }
                }
                if(insert_or_not != 1)
                {
                    for(t = current_idx; t >= 1; t--)
                    {
                        direct_cost[t] = direct_cost[t - 1];
                        pi->best_index[pidx][t] = pi->best_index[pidx][t - 1];
                    }
                    direct_cost[0] = temp_cost;
                    pi->best_index[pidx][0] = c_num;

                    current_idx++;
                    if(current_idx >= MMVD_SKIP_CON_NUM)
                    {
                        current_idx = (MMVD_SKIP_CON_NUM - 1);
                    }
                }
            }
        }
    }

    min_cost = 0.0;
    consider_num = 1;
    for(ttt = 1; ttt < current_idx; ttt++)
    {
        if((direct_cost[0] * MMVD_THRESHOLD) < direct_cost[ttt])
        {
            break;
        }
        else
        {
            consider_num++;
        }
    }

    pi->best_index[pidx][MMVD_SKIP_CON_NUM - 1] = consider_num;

    min_cost = MAX_COST;
    min_cost_temp = MAX_COST;
    temp_cost = MAX_COST;
    for(ttt = 0; ttt < consider_num; ttt++)
    {
        c_num = pi->best_index[pidx][ttt];

        pi->mv[pidx][REFP_0][MV_X] = real_mv[c_num][0][MV_X];
        pi->mv[pidx][REFP_0][MV_Y] = real_mv[c_num][0][MV_Y];
        pi->mv[pidx][REFP_1][MV_X] = real_mv[c_num][1][MV_X];
        pi->mv[pidx][REFP_1][MV_Y] = real_mv[c_num][1][MV_Y];
        pi->refi[pidx][0] = real_mv[c_num][0][2];
        pi->refi[pidx][1] = real_mv[c_num][1][2];

        SET_REFI(refi, real_mv[c_num][0][2], ctx->tgh.tile_group_type == TILE_GROUP_B ? real_mv[c_num][1][2] : REFI_INVALID);
        if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }
        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;

        pi->mmvd_idx[pidx] = c_num;

        temp_cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pi->coef[pidx], pidx, pi->mvp_idx[pidx]
#if DMVR
                                       , FALSE
#endif
        );

        evc_mcpy(pi->nnz_best[pidx], core->nnz, sizeof(int) * N_C);
        evc_mcpy(pi->nnz_sub_best[pidx], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);

        if(min_cost_temp > temp_cost)
        {
            min_cost_temp = temp_cost;
            best_candi = ttt;
        }
    }
#if ATS_INTER_PROCESS
    //Note: temp_cost could be smaller than min_cost
    //I doubt whether the next for loop is needed
    core->cost_best = cost_best_save;
#endif
    for(ttt = best_candi; ttt < best_candi + 1; ttt++)
    {
        c_num = pi->best_index[pidx][ttt];

        pi->mv[pidx][REFP_0][MV_X] = real_mv[c_num][0][MV_X];
        pi->mv[pidx][REFP_0][MV_Y] = real_mv[c_num][0][MV_Y];
        pi->mv[pidx][REFP_1][MV_X] = real_mv[c_num][1][MV_X];
        pi->mv[pidx][REFP_1][MV_Y] = real_mv[c_num][1][MV_Y];
        pi->refi[pidx][0] = real_mv[c_num][0][2];
        pi->refi[pidx][1] = real_mv[c_num][1][2];

        SET_REFI(refi, real_mv[c_num][0][2], ctx->tgh.tile_group_type == TILE_GROUP_B ? real_mv[c_num][1][2] : REFI_INVALID);
        if(!REFI_IS_VALID(refi[REFP_0]) && !REFI_IS_VALID(refi[REFP_1]))
        {
            continue;
        }
        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;

        pi->mmvd_idx[pidx] = c_num;

        min_cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pi->coef[pidx], pidx, pi->mvp_idx[pidx]
#if DMVR
                                      , FALSE
#endif
        );
        pi->mmvd_idx[pidx] = c_num;
        evc_mcpy(pi->nnz_best[pidx], core->nnz, sizeof(int) * N_C);
        evc_mcpy(pi->nnz_sub_best[pidx], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
#if ATS_INTER_PROCESS
        pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
        core->cost_best = min_cost < core->cost_best ? min_cost : core->cost_best;
#endif
    }

    return min_cost;
}

static double analyze_bi(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, double * cost_inter)
{
    EVCE_PINTER *pi = &ctx->pinter;
    s8         refi[REFP_NUM] = {REFI_INVALID, REFI_INVALID};
    int        t1;
    u32        best_mecost = EVC_UINT32_MAX;
    int        refi_best = 0, refi_cur;
    int        changed = 0;
    u32        mecost;
    pel        *org;
    pel       (*pred)[N_C][MAX_CU_DIM];
    int         cuw, cuh, t0;
    double      cost;
    int         lidx_ref, lidx_cnd, mvp_idx = 0;
    int         pidx, pidx_ref, pidx_cnd, i;
    const int   mvr_offset = pi->curr_mvr * ORG_PRED_NUM;
    u8          bi_idx = BI_NORMAL + (pi->curr_bi % 3);
    int         bi_start = 0;
    int         bi_end = pi->num_refp;
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);

    if(bi_idx == BI_FL0 || bi_idx == BI_FL1)
    {
        pi->mot_bits[REFP_0] = 0;
        pi->mot_bits[REFP_1] = 0;

        if(bi_idx == BI_FL0) pidx = PRED_FL0_BI + mvr_offset;
        else pidx = PRED_FL1_BI + mvr_offset;


        pi->mvr_idx[pidx] = pi->curr_mvr;
        pi->bi_idx[pidx] = bi_idx;
        pi->mvp_idx[pidx][REFP_0] = 0;
        pi->mvp_idx[pidx][REFP_1] = 0;

        lidx_ref = (bi_idx == BI_FL1) ? REFP_0 : REFP_1;
        lidx_cnd = (bi_idx == BI_FL1) ? REFP_1 : REFP_0;

        pi->refi[pidx][lidx_ref] = REFI_INVALID;
        pi->refi[pidx][lidx_cnd] = evc_get_first_refi(core->scup, lidx_cnd, ctx->map_refi, ctx->map_mv, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, pi->mvr_idx[pidx], core->avail_lr
#if DMVR_LAG
                                                      , ctx->map_unrefined_mv
#endif
#if ADMVP
                                                      , core->history_buffer
                                                      , ctx->sps.tool_admvp
#endif
        );

        pi->mv[pidx][lidx_ref][MV_X] = pi->mvp_scale[lidx_ref][pi->refi[pidx][lidx_ref]][pi->mvp_idx[pidx][lidx_ref]][MV_X];
        pi->mv[pidx][lidx_ref][MV_Y] = pi->mvp_scale[lidx_ref][pi->refi[pidx][lidx_ref]][pi->mvp_idx[pidx][lidx_ref]][MV_Y];
        pi->mv[pidx][lidx_cnd][MV_X] = pi->mvp_scale[lidx_cnd][pi->refi[pidx][lidx_cnd]][pi->mvp_idx[pidx][lidx_cnd]][MV_X];
        pi->mv[pidx][lidx_cnd][MV_Y] = pi->mvp_scale[lidx_cnd][pi->refi[pidx][lidx_cnd]][pi->mvp_idx[pidx][lidx_cnd]][MV_Y];

        /* get MVP lidx_cnd */
        org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
        pred = pi->pred[pidx];

        refi[REFP_0] = pi->refi[pidx][REFP_0];
        refi[REFP_1] = pi->refi[pidx][REFP_1];


        /* predict reference */
#if DMVR
        evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, pi->mv[pidx], pi->refp, pred, 0, pi->dmvr_template, pi->dmvr_ref_pred_interpolated
               , pi->dmvr_half_pred_interpolated
               , FALSE
#if DMVR_PADDING
               , pi->dmvr_padding_buf
#endif
#if DMVR_FLAG
               , &core->dmvr_flag
#if DMVR_LAG
               , NULL
#endif
#endif
               , ctx->sps.tool_amis
#else
        evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, pi->mv[pidx], pi->refp, pred
#endif
        );

        get_org_bi(org, pred[0][Y_C], pi->s_o[Y_C], cuw, cuh, pi->org_bi);
        refi[lidx_ref] = evc_get_first_refi(core->scup, lidx_ref, ctx->map_refi, ctx->map_mv, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, pi->mvr_idx[pidx], core->avail_lr
#if DMVR_LAG
                                            , ctx->map_unrefined_mv
#endif
#if ADMVP
                                            , core->history_buffer
                                            , ctx->sps.tool_admvp
#endif
        );
        refi[lidx_cnd] = REFI_INVALID;

        for(refi_cur = refi[lidx_ref]; refi_cur < refi[lidx_ref] + 1; refi_cur++)
        {
            refi[lidx_ref] = refi_cur;
            mecost = pi->fn_me(pi, x, y, log2_cuw, log2_cuh, &refi[lidx_ref], lidx_ref, pi->mvp_scale[lidx_ref][refi_cur][pi->mvp_idx[pidx][lidx_ref]], pi->mv_scale[lidx_ref][refi_cur], bi_idx);
            if(mecost < best_mecost)
            {
                refi_best = refi_cur;
                best_mecost = mecost;
                pi->mv[pidx][lidx_ref][MV_X] = pi->mv_scale[lidx_ref][refi_cur][MV_X];
                pi->mv[pidx][lidx_ref][MV_Y] = pi->mv_scale[lidx_ref][refi_cur][MV_Y];
            }
        }

        pi->refi[pidx][lidx_ref] = refi_best;

        pi->mv[pidx][REFP_0][MV_X] = (pi->mv[pidx][REFP_0][MV_X] >> pi->curr_mvr) << pi->curr_mvr;
        pi->mv[pidx][REFP_0][MV_Y] = (pi->mv[pidx][REFP_0][MV_Y] >> pi->curr_mvr) << pi->curr_mvr;
        pi->mv[pidx][REFP_1][MV_X] = (pi->mv[pidx][REFP_1][MV_X] >> pi->curr_mvr) << pi->curr_mvr;
        pi->mv[pidx][REFP_1][MV_Y] = (pi->mv[pidx][REFP_1][MV_Y] >> pi->curr_mvr) << pi->curr_mvr;

        pi->mvd[pidx][REFP_0][MV_X] = pi->mv[pidx][REFP_0][MV_X] - pi->mvp_scale[REFP_0][pi->refi[pidx][REFP_0]][pi->mvp_idx[pidx][REFP_0]][MV_X];
        pi->mvd[pidx][REFP_0][MV_Y] = pi->mv[pidx][REFP_0][MV_Y] - pi->mvp_scale[REFP_0][pi->refi[pidx][REFP_0]][pi->mvp_idx[pidx][REFP_0]][MV_Y];
        pi->mvd[pidx][REFP_1][MV_X] = pi->mv[pidx][REFP_1][MV_X] - pi->mvp_scale[REFP_1][pi->refi[pidx][REFP_1]][pi->mvp_idx[pidx][REFP_1]][MV_X];
        pi->mvd[pidx][REFP_1][MV_Y] = pi->mv[pidx][REFP_1][MV_Y] - pi->mvp_scale[REFP_1][pi->refi[pidx][REFP_1]][pi->mvp_idx[pidx][REFP_1]][MV_Y];
    }
    else
    {
        pidx = (pi->curr_bi == 3 ? PRED_BI_REF : PRED_BI) + mvr_offset;

        if(cost_inter[PRED_L0 + mvr_offset] <= cost_inter[PRED_L1 + mvr_offset])
        {
            lidx_ref = REFP_0;
            lidx_cnd = REFP_1;
            pidx_ref = PRED_L0 + mvr_offset;
            pidx_cnd = PRED_L1 + mvr_offset;
        }
        else
        {
            lidx_ref = REFP_1;
            lidx_cnd = REFP_0;
            pidx_ref = PRED_L1 + mvr_offset;
            pidx_cnd = PRED_L0 + mvr_offset;
        }
        pi->mvr_idx[pidx] = pi->curr_mvr;
        if(ctx->sps.tool_amis == 1)
        {
            pi->mvp_idx[pidx][REFP_0] = 0;
            pi->mvp_idx[pidx][REFP_1] = 0;
        }
        else
        {
            pi->mvp_idx[pidx][REFP_0] = pi->mvp_idx[PRED_L0][REFP_0];
            pi->mvp_idx[pidx][REFP_1] = pi->mvp_idx[PRED_L1][REFP_1];
        }
        pi->refi[pidx][REFP_0] = pi->refi[PRED_L0 + mvr_offset][REFP_0];
        pi->refi[pidx][REFP_1] = pi->refi[PRED_L1 + mvr_offset][REFP_1];

        pi->bi_idx[pidx] = bi_idx;
        if(pi->curr_bi == 3)
        {
            if(EVC_ABS(pi->mvp_scale[lidx_ref][pi->refi[pidx][lidx_ref]][pi->mvp_idx[pidx][lidx_ref]][MV_X] - pi->mv[pidx_ref][lidx_ref][MV_X]) < 3 &&
               EVC_ABS(pi->mvp_scale[lidx_ref][pi->refi[pidx][lidx_ref]][pi->mvp_idx[pidx][lidx_ref]][MV_Y] - pi->mv[pidx_ref][lidx_ref][MV_Y]) < 3 &&
               EVC_ABS(pi->mvp_scale[lidx_cnd][pi->refi[pidx][lidx_cnd]][pi->mvp_idx[pidx][lidx_cnd]][MV_X] - pi->mv[pidx_cnd][lidx_cnd][MV_X]) < 3 &&
               EVC_ABS(pi->mvp_scale[lidx_cnd][pi->refi[pidx][lidx_cnd]][pi->mvp_idx[pidx][lidx_cnd]][MV_Y] - pi->mv[pidx_cnd][lidx_cnd][MV_Y]) < 3)
            {
                return MAX_COST;
            }
            pi->mv[pidx][lidx_ref][MV_X] = pi->mvp_scale[lidx_ref][pi->refi[pidx][lidx_ref]][pi->mvp_idx[pidx][lidx_ref]][MV_X];
            pi->mv[pidx][lidx_ref][MV_Y] = pi->mvp_scale[lidx_ref][pi->refi[pidx][lidx_ref]][pi->mvp_idx[pidx][lidx_ref]][MV_Y];
            pi->mv[pidx][lidx_cnd][MV_X] = pi->mvp_scale[lidx_cnd][pi->refi[pidx][lidx_cnd]][pi->mvp_idx[pidx][lidx_cnd]][MV_X];
            pi->mv[pidx][lidx_cnd][MV_Y] = pi->mvp_scale[lidx_cnd][pi->refi[pidx][lidx_cnd]][pi->mvp_idx[pidx][lidx_cnd]][MV_Y];
        }
        else
        {
            pi->mv[pidx][lidx_ref][MV_X] = pi->mv[pidx_ref][lidx_ref][MV_X];
            pi->mv[pidx][lidx_ref][MV_Y] = pi->mv[pidx_ref][lidx_ref][MV_Y];
            pi->mv[pidx][lidx_cnd][MV_X] = pi->mv[pidx_cnd][lidx_cnd][MV_X];
            pi->mv[pidx][lidx_cnd][MV_Y] = pi->mv[pidx_cnd][lidx_cnd][MV_Y];
        }

        /* get MVP lidx_cnd */
        org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
        pred = pi->pred[pidx];

        t0 = (lidx_ref == REFP_0) ? pi->refi[pidx][lidx_ref] : REFI_INVALID;
        t1 = (lidx_ref == REFP_1) ? pi->refi[pidx][lidx_ref] : REFI_INVALID;
        SET_REFI(refi, t0, t1);


        for(i = 0; i < BI_ITER; i++)
        {
            /* predict reference */
#if DMVR
            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, pi->mv[pidx], pi->refp, pred, 0, pi->dmvr_template, pi->dmvr_ref_pred_interpolated
                   , pi->dmvr_half_pred_interpolated
                   , FALSE
#if DMVR_PADDING
                   , pi->dmvr_padding_buf
#endif
#if DMVR_FLAG
                   , &core->dmvr_flag
#if DMVR_LAG
                   , NULL
#endif
#endif
                   , ctx->sps.tool_amis
#else
            evc_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, pi->mv[pidx], pi->refp, pred
#endif
            );

            get_org_bi(org, pred[0][Y_C], pi->s_o[Y_C], cuw, cuh, pi->org_bi);

            SWAP(refi[lidx_ref], refi[lidx_cnd], t0);
            SWAP(lidx_ref, lidx_cnd, t0);
            SWAP(pidx_ref, pidx_cnd, t0);

            mvp_idx = pi->mvp_idx[pidx][lidx_ref];
            changed = 0;

            if(pi->curr_bi == 3)
            {
                bi_start = refi[lidx_ref];
                bi_end = refi[lidx_ref] + 1;
            }

            for(refi_cur = bi_start; refi_cur < bi_end; refi_cur++)
            {
                refi[lidx_ref] = refi_cur;
                mecost = pi->fn_me(pi, x, y, log2_cuw, log2_cuh, &refi[lidx_ref], lidx_ref, pi->mvp[lidx_ref][mvp_idx], pi->mv_scale[lidx_ref][refi_cur], 1);
                if(mecost < best_mecost)
                {
                    refi_best = refi_cur;
                    best_mecost = mecost;

                    changed = 1;
                    t0 = (lidx_ref == REFP_0) ? refi_best : pi->refi[pidx][lidx_cnd];
                    t1 = (lidx_ref == REFP_1) ? refi_best : pi->refi[pidx][lidx_cnd];
                    SET_REFI(pi->refi[pidx], t0, t1);

                    pi->mv[pidx][lidx_ref][MV_X] = pi->mv_scale[lidx_ref][refi_cur][MV_X];
                    pi->mv[pidx][lidx_ref][MV_Y] = pi->mv_scale[lidx_ref][refi_cur][MV_Y];
                }
            }

            t0 = (lidx_ref == REFP_0) ? refi_best : REFI_INVALID;
            t1 = (lidx_ref == REFP_1) ? refi_best : REFI_INVALID;
            SET_REFI(refi, t0, t1);

            if(!changed)
            {
                break;
            }
        }

        pi->mv[pidx][REFP_0][MV_X] = (pi->mv[pidx][REFP_0][MV_X] >> pi->curr_mvr) << pi->curr_mvr;
        pi->mv[pidx][REFP_0][MV_Y] = (pi->mv[pidx][REFP_0][MV_Y] >> pi->curr_mvr) << pi->curr_mvr;
        pi->mv[pidx][REFP_1][MV_X] = (pi->mv[pidx][REFP_1][MV_X] >> pi->curr_mvr) << pi->curr_mvr;
        pi->mv[pidx][REFP_1][MV_Y] = (pi->mv[pidx][REFP_1][MV_Y] >> pi->curr_mvr) << pi->curr_mvr;

        pi->mvd[pidx][REFP_0][MV_X] = pi->mv[pidx][REFP_0][MV_X] - pi->mvp_scale[REFP_0][pi->refi[pidx][REFP_0]][pi->mvp_idx[pidx][REFP_0]][MV_X];
        pi->mvd[pidx][REFP_0][MV_Y] = pi->mv[pidx][REFP_0][MV_Y] - pi->mvp_scale[REFP_0][pi->refi[pidx][REFP_0]][pi->mvp_idx[pidx][REFP_0]][MV_Y];
        pi->mvd[pidx][REFP_1][MV_X] = pi->mv[pidx][REFP_1][MV_X] - pi->mvp_scale[REFP_1][pi->refi[pidx][REFP_1]][pi->mvp_idx[pidx][REFP_1]][MV_X];
        pi->mvd[pidx][REFP_1][MV_Y] = pi->mv[pidx][REFP_1][MV_Y] - pi->mvp_scale[REFP_1][pi->refi[pidx][REFP_1]][pi->mvp_idx[pidx][REFP_1]][MV_Y];
    }

    cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pi->coef[pidx], pidx, pi->mvp_idx[pidx]
#if DMVR
                              , FALSE
#endif
    );

    evc_mcpy(pi->nnz_best[pidx], core->nnz, sizeof(int) * N_C);
    evc_mcpy(pi->nnz_sub_best[pidx], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
#if ATS_INTER_PROCESS
    pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
    return cost;
}

static int pinter_init_frame(EVCE_CTX *ctx)
{
    EVCE_PINTER *pi;
    EVC_PIC     *pic;
    int size;

    pi             = &ctx->pinter;

    pic            = pi->pic_o = PIC_ORIG(ctx);
    pi->o[Y_C]     = pic->y;
    pi->o[U_C]     = pic->u;
    pi->o[V_C]     = pic->v;
                   
    pi->s_o[Y_C]   = pic->s_l;
    pi->s_o[U_C]   = pic->s_c;
    pi->s_o[V_C]   = pic->s_c;
                   
    pic            = pi->pic_m = PIC_MODE(ctx);
    pi->m[Y_C]     = pic->y;
    pi->m[U_C]     = pic->u;
    pi->m[V_C]     = pic->v;
                   
    pi->s_m[Y_C]   = pic->s_l;
    pi->s_m[U_C]   = pic->s_c;
    pi->s_m[V_C]   = pic->s_c;

    pi->refp       = ctx->refp;
    pi->tile_group_type = ctx->tile_group_type;

    pi->map_mv     = ctx->map_mv;
#if DMVR_LAG
    pi->map_unrefined_mv = ctx->map_unrefined_mv;
#endif
    pi->w_scu      = ctx->w_scu;

    size = sizeof(pel) * MAX_CU_DIM;
    evc_mset(pi->pred_buf, 0, size);

    size = sizeof(pel) * N_C * MAX_CU_DIM;
    evc_mset(pi->rec_buf, 0, size);

    size = sizeof(s8) * PRED_NUM * REFP_NUM;
    evc_mset(pi->refi, 0, size);

    size = sizeof(s8) * REFP_NUM * MAX_NUM_MVP;
    evc_mset(pi->refi_pred, 0, size);

    size = sizeof(s8) * REFP_NUM * MAX_NUM_MVP;
    evc_mset(pi->mvp_idx, 0, size);

    size = sizeof(s16) * REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME * MAX_NUM_MVP * MV_D;
    evc_mset(pi->mvp_scale, 0, size);

    size = sizeof(s16) * REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME * MV_D;
    evc_mset(pi->mv_scale, 0, size);

    size = sizeof(u8) * PRED_NUM * REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME;
    evc_mset(pi->mvp_idx_temp_for_bi, 0, size);

    size = sizeof(int) * PRED_NUM * 4;
    evc_mset(pi->best_index, 0, size);

    size = sizeof(s16) * PRED_NUM;
    evc_mset(pi->mmvd_idx, 0, size);

    size = sizeof(s8) * PRED_NUM;
    evc_mset(pi->mvr_idx, 0, size);

    size = sizeof(int) * MV_D;
    evc_mset(pi->max_imv, 0, size);

    size = sizeof(s8) * PRED_NUM * REFP_NUM;
    evc_mset(pi->first_refi, 0, size);

    size = sizeof(u8) * PRED_NUM;
    evc_mset(pi->bi_idx, 0, size);

#if AFFINE
    size = sizeof(s16) * REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME * MAX_NUM_MVP * VER_NUM * MV_D;
    evc_mset(pi->affine_mvp_scale, 0, size);

    size = sizeof(s16) * REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME * VER_NUM * MV_D;
    evc_mset(pi->affine_mv_scale, 0, size);

    size = sizeof(u8) * REFP_NUM * MAX_NUM_ACTIVE_REF_FRAME;
    evc_mset(pi->mvp_idx_scale, 0, size);

    size = sizeof(u8) * REFP_NUM * MAX_NUM_MVP * VER_NUM * MV_D;
    evc_mset(pi->affine_mvp, 0, size);

    size = sizeof(s16) * PRED_NUM * REFP_NUM * VER_NUM * MV_D;
    evc_mset(pi->affine_mv, 0, size);

    size = sizeof(s16) * PRED_NUM * REFP_NUM * VER_NUM * MV_D;
    evc_mset(pi->affine_mvd, 0, size);

    size = sizeof(pel) * MAX_CU_DIM;
    evc_mset(pi->p_error, 0, size);

    size = sizeof(int) * 2 * MAX_CU_DIM;
    evc_mset(pi->i_gradient, 0, size);
#endif

#if ATS_INTER_PROCESS
    size = sizeof(s16) * N_C * MAX_CU_DIM;
    evc_mset(pi->resi, 0, size);

    size = sizeof(s16) * N_C * MAX_CU_DIM;
    evc_mset(pi->coff_save, 0, size);

    size = sizeof(u8) * PRED_NUM;
    evc_mset(pi->ats_inter_info_mode, 0, size);
#endif

    /* MV predictor */
    size = sizeof(s16) * REFP_NUM * MAX_NUM_MVP * MV_D;
    evc_mset(pi->mvp, 0, size);

    size = sizeof(s16) * PRED_NUM * REFP_NUM * MV_D;
    evc_mset(pi->mv, 0, size);

#if DMVR_LAG
    size = sizeof(s16) * MAX_CU_CNT_IN_LCU * PRED_NUM * REFP_NUM * MV_D;
    evc_mset(pi->dmvr_mv, 0, size);
#endif
    size = sizeof(s16) * PRED_NUM * REFP_NUM * MV_D;
    evc_mset(pi->mvd, 0, size);

    size = sizeof(s16) * MAX_CU_DIM;
    evc_mset(pi->org_bi, 0, size);

    size = sizeof(s32) * REFP_NUM;
    evc_mset(pi->mot_bits, 0, size);

    size = sizeof(pel) * (PRED_NUM + 1) * 2 * N_C * MAX_CU_DIM;
    evc_mset(pi->pred, 0, size);

#if DMVR
    size = sizeof(pel) * MAX_CU_DIM;
    evc_mset(pi->dmvr_template, 0, size);

    size = sizeof(pel) * REFP_NUM * (MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) *
        (MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT));
    evc_mset(pi->dmvr_ref_pred_interpolated, 0, size);
#endif

    return EVC_OK;
}

static int pinter_init_lcu(EVCE_CTX *ctx, EVCE_CORE *core)
{
    EVCE_PINTER *pi;

    pi            = &ctx->pinter;
    pi->lambda_mv = (u32)floor(65536.0 * ctx->sqrt_lambda[0]);
    pi->qp_y      = core->qp_y;
    pi->qp_u      = core->qp_u;
    pi->qp_v      = core->qp_v;
    pi->ptr       = ctx->ptr;
    pi->gop_size  = ctx->param.gop_size;

    return EVC_OK;
}

static void check_best_mvp(EVCE_CTX *ctx, EVCE_CORE *core, s32 tile_group_type, s8 refi[REFP_NUM],
                           int lidx, int pidx, s16(*mvp)[2], s16 *mv, s16 *mvd, u8*mvp_idx)
{
    double cost, best_cost;
    int idx, best_idx;
    u32 bit_cnt;
    s16 mvd_tmp[REFP_NUM][MV_D];

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[core->log2_cuw - 2][core->log2_cuh - 2]);

    evce_sbac_bit_reset(&core->s_temp_run);

    mvd_tmp[lidx][MV_X] = mv[MV_X] - mvp[*mvp_idx][MV_X];
    mvd_tmp[lidx][MV_Y] = mv[MV_Y] - mvp[*mvp_idx][MV_Y];

    evce_rdo_bit_cnt_mvp(ctx, core, tile_group_type, refi, mvd_tmp, pidx, *mvp_idx);
    bit_cnt = evce_get_bit_number(&core->s_temp_run);

    best_cost = RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

    best_idx = *mvp_idx;

#if INCREASE_MVP_NUM
    for(idx = 0; idx < ORG_MAX_NUM_MVP; idx++)
#else
    for(idx = 0; idx < MAX_NUM_MVP; idx++)
#endif
    {
        if(idx == *mvp_idx)
        {
            continue;
        }

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[core->log2_cuw - 2][core->log2_cuh - 2]);

        evce_sbac_bit_reset(&core->s_temp_run);

        mvd_tmp[lidx][MV_X] = mv[MV_X] - mvp[idx][MV_X];
        mvd_tmp[lidx][MV_Y] = mv[MV_Y] - mvp[idx][MV_Y];

        evce_rdo_bit_cnt_mvp(ctx, core, tile_group_type, refi, mvd_tmp, pidx, idx);
        bit_cnt = evce_get_bit_number(&core->s_temp_run);

        cost = RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
        if(cost < best_cost)
        {
            best_idx = idx;
        }
    }

    *mvp_idx = best_idx;
    mvd[MV_X] = mv[MV_X] - mvp[*mvp_idx][MV_X];
    mvd[MV_Y] = mv[MV_Y] - mvp[*mvp_idx][MV_Y];
}

#if AFFINE
static void scaled_horizontal_sobel_filter(pel *pred,
                                           int pred_stride,
                                           int *derivate,
                                           int derivate_buf_stride,
                                           int width,
                                           int height
)
{
#if AFFINE_SIMD
    int j, col, row;

    __m128i mm_pred[4];
    __m128i mm2x_pred[2];
    __m128i mm_intermediates[4];
    __m128i mm_derivate[2];

    assert(!(height % 2));
    assert(!(width % 4));

    /* Derivates of the rows and columns at the boundary are done at the end of this function */
    /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
    for(col = 1; (col + 2) < width; col += 2)
    {
        mm_pred[0] = _mm_loadl_epi64((const __m128i *)(&pred[0 * pred_stride + col - 1]));
        mm_pred[1] = _mm_loadl_epi64((const __m128i *)(&pred[1 * pred_stride + col - 1]));

        mm_pred[0] = _mm_cvtepi16_epi32(mm_pred[0]);
        mm_pred[1] = _mm_cvtepi16_epi32(mm_pred[1]);

        for(row = 1; row < (height - 1); row += 2)
        {
            mm_pred[2] = _mm_loadl_epi64((const __m128i *)(&pred[(row + 1) * pred_stride + col - 1]));
            mm_pred[3] = _mm_loadl_epi64((const __m128i *)(&pred[(row + 2) * pred_stride + col - 1]));

            mm_pred[2] = _mm_cvtepi16_epi32(mm_pred[2]);
            mm_pred[3] = _mm_cvtepi16_epi32(mm_pred[3]);

            mm2x_pred[0] = _mm_slli_epi32(mm_pred[1], 1);
            mm2x_pred[1] = _mm_slli_epi32(mm_pred[2], 1);

            mm_intermediates[0] = _mm_add_epi32(mm2x_pred[0], mm_pred[0]);
            mm_intermediates[2] = _mm_add_epi32(mm2x_pred[1], mm_pred[1]);

            mm_intermediates[0] = _mm_add_epi32(mm_intermediates[0], mm_pred[2]);
            mm_intermediates[2] = _mm_add_epi32(mm_intermediates[2], mm_pred[3]);

            mm_pred[0] = mm_pred[2];
            mm_pred[1] = mm_pred[3];

            mm_intermediates[1] = _mm_srli_si128(mm_intermediates[0], 8);
            mm_intermediates[3] = _mm_srli_si128(mm_intermediates[2], 8);

            mm_derivate[0] = _mm_sub_epi32(mm_intermediates[1], mm_intermediates[0]);
            mm_derivate[1] = _mm_sub_epi32(mm_intermediates[3], mm_intermediates[2]);

            _mm_storel_epi64((__m128i *)(&derivate[col + (row + 0) * derivate_buf_stride]), mm_derivate[0]);
            _mm_storel_epi64((__m128i *)(&derivate[col + (row + 1) * derivate_buf_stride]), mm_derivate[1]);
        }
    }

    for(j = 1; j < (height - 1); j++)
    {
        derivate[j * derivate_buf_stride] = derivate[j * derivate_buf_stride + 1];
        derivate[j * derivate_buf_stride + (width - 1)] = derivate[j * derivate_buf_stride + (width - 2)];
    }

    evc_mcpy
    (
        derivate,
        derivate + derivate_buf_stride,
        width * sizeof(derivate[0])
    );

    evc_mcpy
    (
        derivate + (height - 1) * derivate_buf_stride,
        derivate + (height - 2) * derivate_buf_stride,
        width * sizeof(derivate[0])
    );
#else
    int j, k;

    for(j = 1; j < height - 1; j++)
    {
        for(k = 1; k < width - 1; k++)
        {
            int center = j * pred_stride + k;

            derivate[j * derivate_buf_stride + k] =
                pred[center + 1 - pred_stride] -
                pred[center - 1 - pred_stride] +
                (pred[center + 1] * 2) -
                (pred[center - 1] * 2) +
                pred[center + 1 + pred_stride] -
                pred[center - 1 + pred_stride];
        }

        derivate[j * derivate_buf_stride] = derivate[j * derivate_buf_stride + 1];
        derivate[j * derivate_buf_stride + width - 1] = derivate[j * derivate_buf_stride + width - 2];
    }

    derivate[0] = derivate[derivate_buf_stride + 1];
    derivate[width - 1] = derivate[derivate_buf_stride + width - 2];
    derivate[(height - 1) * derivate_buf_stride] = derivate[(height - 2) * derivate_buf_stride + 1];
    derivate[(height - 1) * derivate_buf_stride + width - 1] = derivate[(height - 2) * derivate_buf_stride + (width - 2)];

    for(j = 1; j < width - 1; j++)
    {
        derivate[j] = derivate[derivate_buf_stride + j];
        derivate[(height - 1) * derivate_buf_stride + j] = derivate[(height - 2) * derivate_buf_stride + j];
    }
#endif
}

static void scaled_vertical_sobel_filter(pel *pred,
                                         int pred_stride,
                                         int *derivate,
                                         int derivate_buf_stride,
                                         int width,
                                         int height
)
{
#if AFFINE_SIMD
    int j, col, row;

    __m128i mm_pred[4];
    __m128i mm_intermediates[6];
    __m128i mm_derivate[2];

    assert(!(height % 2));
    assert(!(width % 4));

    /* Derivates of the rows and columns at the boundary are done at the end of this function */
    /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
    for(col = 1; col < (width - 1); col += 2)
    {
        mm_pred[0] = _mm_loadl_epi64((const __m128i *)(&pred[0 * pred_stride + col - 1]));
        mm_pred[1] = _mm_loadl_epi64((const __m128i *)(&pred[1 * pred_stride + col - 1]));

        mm_pred[0] = _mm_cvtepi16_epi32(mm_pred[0]);
        mm_pred[1] = _mm_cvtepi16_epi32(mm_pred[1]);

        for(row = 1; row < (height - 1); row += 2)
        {
            mm_pred[2] = _mm_loadl_epi64((const __m128i *)(&pred[(row + 1) * pred_stride + col - 1]));
            mm_pred[3] = _mm_loadl_epi64((const __m128i *)(&pred[(row + 2) * pred_stride + col - 1]));

            mm_pred[2] = _mm_cvtepi16_epi32(mm_pred[2]);
            mm_pred[3] = _mm_cvtepi16_epi32(mm_pred[3]);

            mm_intermediates[0] = _mm_sub_epi32(mm_pred[2], mm_pred[0]);
            mm_intermediates[3] = _mm_sub_epi32(mm_pred[3], mm_pred[1]);

            mm_pred[0] = mm_pred[2];
            mm_pred[1] = mm_pred[3];

            mm_intermediates[1] = _mm_srli_si128(mm_intermediates[0], 4);
            mm_intermediates[4] = _mm_srli_si128(mm_intermediates[3], 4);
            mm_intermediates[2] = _mm_srli_si128(mm_intermediates[0], 8);
            mm_intermediates[5] = _mm_srli_si128(mm_intermediates[3], 8);

            mm_intermediates[1] = _mm_slli_epi32(mm_intermediates[1], 1);
            mm_intermediates[4] = _mm_slli_epi32(mm_intermediates[4], 1);

            mm_intermediates[0] = _mm_add_epi32(mm_intermediates[0], mm_intermediates[2]);
            mm_intermediates[3] = _mm_add_epi32(mm_intermediates[3], mm_intermediates[5]);

            mm_derivate[0] = _mm_add_epi32(mm_intermediates[0], mm_intermediates[1]);
            mm_derivate[1] = _mm_add_epi32(mm_intermediates[3], mm_intermediates[4]);

            _mm_storel_epi64((__m128i *) (&derivate[col + (row + 0) * derivate_buf_stride]), mm_derivate[0]);
            _mm_storel_epi64((__m128i *) (&derivate[col + (row + 1) * derivate_buf_stride]), mm_derivate[1]);
        }
    }

    for(j = 1; j < (height - 1); j++)
    {
        derivate[j * derivate_buf_stride] = derivate[j * derivate_buf_stride + 1];
        derivate[j * derivate_buf_stride + (width - 1)] = derivate[j * derivate_buf_stride + (width - 2)];
    }

    evc_mcpy
    (
        derivate,
        derivate + derivate_buf_stride,
        width * sizeof(derivate[0])
    );

    evc_mcpy
    (
        derivate + (height - 1) * derivate_buf_stride,
        derivate + (height - 2) * derivate_buf_stride,
        width * sizeof(derivate[0])
    );
#else
    int k, j;
    for(k = 1; k < width - 1; k++)
    {
        for(j = 1; j < height - 1; j++)
        {
            int center = j * pred_stride + k;

            derivate[j * derivate_buf_stride + k] =
                pred[center + pred_stride - 1] -
                pred[center - pred_stride - 1] +
                (pred[center + pred_stride] * 2) -
                (pred[center - pred_stride] * 2) +
                pred[center + pred_stride + 1] -
                pred[center - pred_stride + 1];
        }

        derivate[k] = derivate[derivate_buf_stride + k];
        derivate[(height - 1) * derivate_buf_stride + k] = derivate[(height - 2) * derivate_buf_stride + k];
    }

    derivate[0] = derivate[derivate_buf_stride + 1];
    derivate[width - 1] = derivate[derivate_buf_stride + width - 2];
    derivate[(height - 1) * derivate_buf_stride] = derivate[(height - 2) * derivate_buf_stride + 1];
    derivate[(height - 1) * derivate_buf_stride + width - 1] = derivate[(height - 2) * derivate_buf_stride + (width - 2)];

    for(j = 1; j < height - 1; j++)
    {
        derivate[j * derivate_buf_stride] = derivate[j * derivate_buf_stride + 1];
        derivate[j * derivate_buf_stride + width - 1] = derivate[j * derivate_buf_stride + width - 2];
    }
#endif
}

static void equal_coeff_computer(pel *residue,
                                 int residue_stride,
                                 int **derivate,
                                 int derivate_buf_stride,
                                 s64(*equal_coeff)[7],
                                 int width,
                                 int height,
                                 int vertex_num
)
{
#if AFFINE_SIMD
    int j, k;
    int idx1 = 0, idx2 = 0;

    __m128i mm_two, mm_four;
    __m128i mm_tmp[4];
    __m128i mm_intermediate[4];
    __m128i mm_idx_k, mm_idx_j[2];
    __m128i mm_residue[2];

    // Add directly to indexes to get new index
    mm_two = _mm_set1_epi32(2);
    mm_four = _mm_set1_epi32(4);

    if(vertex_num == 3)
    {
        __m128i mm_c[12];

        //  mm_c - map 
        //  C for 1st row of pixels
        //  mm_c[0] = iC[0][i] | iC[0][i+1] | iC[0][i+2] | iC[0][i+3]
        //  mm_c[1] = iC[1][i] | iC[1][i+1] | iC[1][i+2] | iC[1][i+3]
        //  mm_c[2] = iC[2][i] | iC[2][i+1] | iC[2][i+2] | iC[2][i+3]
        //  mm_c[3] = iC[3][i] | iC[3][i+1] | iC[3][i+2] | iC[3][i+3]
        //  mm_c[4] = iC[4][i] | iC[4][i+1] | iC[4][i+2] | iC[4][i+3]
        //  mm_c[5] = iC[5][i] | iC[5][i+1] | iC[5][i+2] | iC[5][i+3]

        //  C for 2nd row of pixels
        //  mm_c[6] = iC[6][i] | iC[6][i+1] | iC[6][i+2] | iC[6][i+3]
        //  mm_c[7] = iC[7][i] | iC[7][i+1] | iC[7][i+2] | iC[7][i+3]
        //  mm_c[8] = iC[8][i] | iC[8][i+1] | iC[8][i+2] | iC[8][i+3]
        //  mm_c[9] = iC[9][i] | iC[9][i+1] | iC[9][i+2] | iC[9][i+3]
        //  mm_c[10] = iC[10][i] | iC[10][i+1] | iC[10][i+2] | iC[10][i+3]
        //  mm_c[11] = iC[11][i] | iC[11][i+1] | iC[11][i+2] | iC[11][i+3]

        idx1 = -2 * derivate_buf_stride - 4;
        idx2 = -derivate_buf_stride - 4;
        mm_idx_j[0] = _mm_set1_epi32(-2);
        mm_idx_j[1] = _mm_set1_epi32(-1);

        for(j = 0; j < height; j += 2)
        {
            mm_idx_j[0] = _mm_add_epi32(mm_idx_j[0], mm_two);
            mm_idx_j[1] = _mm_add_epi32(mm_idx_j[1], mm_two);
            mm_idx_k = _mm_set_epi32(-1, -2, -3, -4);
            idx1 += (derivate_buf_stride << 1);
            idx2 += (derivate_buf_stride << 1);

            for(k = 0; k < width; k += 4)
            {
                idx1 += 4;
                idx2 += 4;

                mm_idx_k = _mm_add_epi32(mm_idx_k, mm_four);

                // 1st row
                mm_c[0] = _mm_loadu_si128((const __m128i*)&derivate[0][idx1]);
                mm_c[2] = _mm_loadu_si128((const __m128i*)&derivate[1][idx1]);
                // 2nd row
                mm_c[6] = _mm_loadu_si128((const __m128i*)&derivate[0][idx2]);
                mm_c[8] = _mm_loadu_si128((const __m128i*)&derivate[1][idx2]);

                // 1st row
                mm_c[1] = _mm_mullo_epi32(mm_idx_k, mm_c[0]);
                mm_c[3] = _mm_mullo_epi32(mm_idx_k, mm_c[2]);
                mm_c[4] = _mm_mullo_epi32(mm_idx_j[0], mm_c[0]);
                mm_c[5] = _mm_mullo_epi32(mm_idx_j[0], mm_c[2]);

                // 2nd row
                mm_c[7] = _mm_mullo_epi32(mm_idx_k, mm_c[6]);
                mm_c[9] = _mm_mullo_epi32(mm_idx_k, mm_c[8]);
                mm_c[10] = _mm_mullo_epi32(mm_idx_j[1], mm_c[6]);
                mm_c[11] = _mm_mullo_epi32(mm_idx_j[1], mm_c[8]);

                // Residue
                mm_residue[0] = _mm_loadl_epi64((const __m128i*)&residue[idx1]);
                mm_residue[1] = _mm_loadl_epi64((const __m128i*)&residue[idx2]);

                mm_residue[0] = _mm_cvtepi16_epi32(mm_residue[0]);
                mm_residue[1] = _mm_cvtepi16_epi32(mm_residue[1]);

                mm_residue[0] = _mm_slli_epi32(mm_residue[0], 3);
                mm_residue[1] = _mm_slli_epi32(mm_residue[1], 3);

                // Calculate residue coefficients first
                mm_tmp[2] = _mm_srli_si128(mm_residue[0], 4);
                mm_tmp[3] = _mm_srli_si128(mm_residue[1], 4);

                // 1st row
                mm_tmp[0] = _mm_srli_si128(mm_c[0], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[6], 4);
                // 7th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][6]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][6], mm_intermediate[3]);

                // 2nd row
                mm_tmp[0] = _mm_srli_si128(mm_c[1], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[7], 4);
                // 7th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[7], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][6]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][6], mm_intermediate[3]);

                // 3rd row
                mm_tmp[0] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[8], 4);
                // 7th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[8], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][6]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][6], mm_intermediate[3]);

                // 4th row
                mm_tmp[0] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[9], 4);
                // 7th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[3], mm_c[9], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[4][6]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][6], mm_intermediate[3]);

                // 5th row
                mm_tmp[0] = _mm_srli_si128(mm_c[4], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[10], 4);
                // 7th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[4], mm_c[10], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[5][6]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][6], mm_intermediate[3]);

                // 6th row
                mm_tmp[0] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[11], 4);
                // 7th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[5], mm_c[11], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[6][6]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][6], mm_intermediate[3]);

                //Start calculation of coefficient matrix
                // 1st row
                mm_tmp[0] = _mm_srli_si128(mm_c[0], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[6], 4);

                // 1st col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_c[0], mm_c[6], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][0]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][0], mm_intermediate[3]);
                // 2nd col of row and 1st col of 2nd row
                mm_tmp[2] = _mm_srli_si128(mm_c[1], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[7], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_c[1], mm_c[7], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][1]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][1], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][0], mm_intermediate[3]);
                // 3rd col of row and 1st col of 3rd row
                mm_tmp[2] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[8], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_c[2], mm_c[8], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][2]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][2], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][0], mm_intermediate[3]);
                // 4th col of row and 1st col of 4th row
                mm_tmp[2] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[9], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_c[3], mm_c[9], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][3], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][0], mm_intermediate[3]);
                // 5th col of row and 1st col of the 5th row
                mm_tmp[2] = _mm_srli_si128(mm_c[4], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[10], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_c[4], mm_c[10], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][4], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][0], mm_intermediate[3]);
                // 6th col of row and 1st col of the 6th row
                mm_tmp[2] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[11], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[6], mm_c[5], mm_c[11], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][5]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][5], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][0], mm_intermediate[3]);

                // 2nd row
                mm_tmp[0] = _mm_srli_si128(mm_c[1], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[7], 4);

                // 2nd col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[7], mm_c[1], mm_c[7], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][1]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][1], mm_intermediate[3]);
                // 3rd col of row and 2nd col of 3rd row
                mm_tmp[2] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[8], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[7], mm_c[2], mm_c[8], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][2]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][2], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][1], mm_intermediate[3]);
                // 4th col of row and 2nd col of 4th row
                mm_tmp[2] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[9], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[7], mm_c[3], mm_c[9], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][3], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][1], mm_intermediate[3]);
                // 5th col of row and 1st col of the 5th row
                mm_tmp[2] = _mm_srli_si128(mm_c[4], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[10], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[7], mm_c[4], mm_c[10], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][4], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][1], mm_intermediate[3]);
                // 6th col of row and 1st col of the 6th row
                mm_tmp[2] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[11], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[7], mm_c[5], mm_c[11], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][5]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][5], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][1], mm_intermediate[3]);

                // 3rd row
                mm_tmp[0] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[8], 4);

                //3rd Col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[8], mm_c[2], mm_c[8], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][2]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][2], mm_intermediate[3]);
                // 4th col of row and 3rd col of 4th row
                mm_tmp[2] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[9], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[8], mm_c[3], mm_c[9], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][3], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][2], mm_intermediate[3]);
                // 5th col of row and 1st col of the 5th row
                mm_tmp[2] = _mm_srli_si128(mm_c[4], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[10], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[8], mm_c[4], mm_c[10], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][4], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][2], mm_intermediate[3]);
                // 6th col of row and 1st col of the 6th row
                mm_tmp[2] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[11], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[8], mm_c[5], mm_c[11], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][5]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][5], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][2], mm_intermediate[3]);

                // 4th row
                mm_tmp[0] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[9], 4);

                // 4th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[3], mm_c[9], mm_c[3], mm_c[9], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[4][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][3], mm_intermediate[3]);
                // 5th col of row and 1st col of the 5th row
                mm_tmp[2] = _mm_srli_si128(mm_c[4], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[10], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[3], mm_c[9], mm_c[4], mm_c[10], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[4][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][4], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][3], mm_intermediate[3]);
                // 6th col of row and 1st col of the 6th row
                mm_tmp[2] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[11], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[3], mm_c[9], mm_c[5], mm_c[11], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[4][5]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][5], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][3], mm_intermediate[3]);

                // 5th row
                mm_tmp[0] = _mm_srli_si128(mm_c[4], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[10], 4);
                // 5th col of row and 1st col of the 5th row
                CALC_EQUAL_COEFF_8PXLS(mm_c[4], mm_c[10], mm_c[4], mm_c[10], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[5][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][4], mm_intermediate[3]);
                // 6th col of row and 1st col of the 6th row
                mm_tmp[2] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[11], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[4], mm_c[10], mm_c[5], mm_c[11], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[5][5]);
                _mm_storel_epi64((__m128i*)&equal_coeff[5][5], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][4], mm_intermediate[3]);

                // 6th row
                mm_tmp[0] = _mm_srli_si128(mm_c[5], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[11], 4);
                // 5th col of row and 1st col of the 5th row
                CALC_EQUAL_COEFF_8PXLS(mm_c[5], mm_c[11], mm_c[5], mm_c[11], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[6][5]);
                _mm_storel_epi64((__m128i*)&equal_coeff[6][5], mm_intermediate[3]);
            }

            idx1 -= (width);
            idx2 -= (width);
        }
    }
    else
    {
        __m128i mm_c[8];

        //  mm_c - map 
        //  C for 1st row of pixels
        //  mm_c[0] = iC[0][i] | iC[0][i+1] | iC[0][i+2] | iC[0][i+3]
        //  mm_c[1] = iC[1][i] | iC[1][i+1] | iC[1][i+2] | iC[1][i+3]
        //  mm_c[2] = iC[2][i] | iC[2][i+1] | iC[2][i+2] | iC[2][i+3]
        //  mm_c[3] = iC[3][i] | iC[3][i+1] | iC[3][i+2] | iC[3][i+3]
        //  C for 2nd row of pixels
        //  mm_c[4] = iC[0][i] | iC[0][i+1] | iC[0][i+2] | iC[0][i+3]
        //  mm_c[5] = iC[1][i] | iC[1][i+1] | iC[1][i+2] | iC[1][i+3]
        //  mm_c[6] = iC[2][i] | iC[2][i+1] | iC[2][i+2] | iC[2][i+3]
        //  mm_c[7] = iC[3][i] | iC[3][i+1] | iC[3][i+2] | iC[3][i+3]

        idx1 = -2 * derivate_buf_stride - 4;
        idx2 = -derivate_buf_stride - 4;
        mm_idx_j[0] = _mm_set1_epi32(-2);
        mm_idx_j[1] = _mm_set1_epi32(-1);

        for(j = 0; j < height; j += 2)
        {
            mm_idx_j[0] = _mm_add_epi32(mm_idx_j[0], mm_two);
            mm_idx_j[1] = _mm_add_epi32(mm_idx_j[1], mm_two);
            mm_idx_k = _mm_set_epi32(-1, -2, -3, -4);
            idx1 += (derivate_buf_stride << 1);
            idx2 += (derivate_buf_stride << 1);

            for(k = 0; k < width; k += 4)
            {
                idx1 += 4;
                idx2 += 4;

                mm_idx_k = _mm_add_epi32(mm_idx_k, mm_four);

                mm_c[0] = _mm_loadu_si128((const __m128i*)&derivate[0][idx1]);
                mm_c[2] = _mm_loadu_si128((const __m128i*)&derivate[1][idx1]);
                mm_c[4] = _mm_loadu_si128((const __m128i*)&derivate[0][idx2]);
                mm_c[6] = _mm_loadu_si128((const __m128i*)&derivate[1][idx2]);

                mm_c[1] = _mm_mullo_epi32(mm_idx_k, mm_c[0]);
                mm_c[3] = _mm_mullo_epi32(mm_idx_j[0], mm_c[0]);
                mm_c[5] = _mm_mullo_epi32(mm_idx_k, mm_c[4]);
                mm_c[7] = _mm_mullo_epi32(mm_idx_j[1], mm_c[4]);

                mm_residue[0] = _mm_loadl_epi64((const __m128i*)&residue[idx1]);
                mm_residue[1] = _mm_loadl_epi64((const __m128i*)&residue[idx2]);

                mm_tmp[0] = _mm_mullo_epi32(mm_idx_j[0], mm_c[2]);
                mm_tmp[1] = _mm_mullo_epi32(mm_idx_k, mm_c[2]);
                mm_tmp[2] = _mm_mullo_epi32(mm_idx_j[1], mm_c[6]);
                mm_tmp[3] = _mm_mullo_epi32(mm_idx_k, mm_c[6]);

                mm_residue[0] = _mm_cvtepi16_epi32(mm_residue[0]);
                mm_residue[1] = _mm_cvtepi16_epi32(mm_residue[1]);

                mm_c[1] = _mm_add_epi32(mm_c[1], mm_tmp[0]);
                mm_c[3] = _mm_sub_epi32(mm_c[3], mm_tmp[1]);
                mm_c[5] = _mm_add_epi32(mm_c[5], mm_tmp[2]);
                mm_c[7] = _mm_sub_epi32(mm_c[7], mm_tmp[3]);

                mm_residue[0] = _mm_slli_epi32(mm_residue[0], 3);
                mm_residue[1] = _mm_slli_epi32(mm_residue[1], 3);

                //Start calculation of coefficient matrix
                // 1st row
                mm_tmp[0] = _mm_srli_si128(mm_c[0], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[4], 4);

                // 1st col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[4], mm_c[0], mm_c[4], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][0]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][0], mm_intermediate[3]);
                // 2nd col of row and 1st col of 2nd row
                mm_tmp[2] = _mm_srli_si128(mm_c[1], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[5], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[4], mm_c[1], mm_c[5], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][1]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][1], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][0], mm_intermediate[3]);
                // 3rd col of row and 1st col of 3rd row
                mm_tmp[2] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[6], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[4], mm_c[2], mm_c[6], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][2]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][2], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][0], mm_intermediate[3]);
                // 4th col of row and 1st col of 4th row
                mm_tmp[2] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[7], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[4], mm_c[3], mm_c[7], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][3], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][0], mm_intermediate[3]);
                // 5th col of row
                mm_tmp[2] = _mm_srli_si128(mm_residue[0], 4);
                mm_tmp[3] = _mm_srli_si128(mm_residue[1], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[0], mm_c[4], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[1][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[1][4], mm_intermediate[3]);

                // 2nd row
                mm_tmp[0] = _mm_srli_si128(mm_c[1], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[5], 4);

                // 2nd col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[5], mm_c[1], mm_c[5], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][1]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][1], mm_intermediate[3]);
                // 3rd col of row and 2nd col of 3rd row
                mm_tmp[2] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[6], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[5], mm_c[2], mm_c[6], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][2]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][2], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][1], mm_intermediate[3]);
                // 4th col of row and 2nd col of 4th row
                mm_tmp[2] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[7], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[5], mm_c[3], mm_c[7], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][3], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][1], mm_intermediate[3]);
                // 5th col of row
                mm_tmp[2] = _mm_srli_si128(mm_residue[0], 4);
                mm_tmp[3] = _mm_srli_si128(mm_residue[1], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[1], mm_c[5], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[2][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[2][4], mm_intermediate[3]);

                // 3rd row
                mm_tmp[0] = _mm_srli_si128(mm_c[2], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[6], 4);

                //3rd Col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[6], mm_c[2], mm_c[6], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][2]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][2], mm_intermediate[3]);
                // 4th col of row and 3rd col of 4th row
                mm_tmp[2] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[3] = _mm_srli_si128(mm_c[7], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[6], mm_c[3], mm_c[7], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][3], mm_intermediate[3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][2], mm_intermediate[3]);
                // 5th col of row
                mm_tmp[2] = _mm_srli_si128(mm_residue[0], 4);
                mm_tmp[3] = _mm_srli_si128(mm_residue[1], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[2], mm_c[6], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[3][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[3][4], mm_intermediate[3]);

                // 4th row
                mm_tmp[0] = _mm_srli_si128(mm_c[3], 4);
                mm_tmp[1] = _mm_srli_si128(mm_c[7], 4);

                // 4th col of row
                CALC_EQUAL_COEFF_8PXLS(mm_c[3], mm_c[7], mm_c[3], mm_c[7], mm_tmp[0], mm_tmp[1], mm_tmp[0], mm_tmp[1], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[4][3]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][3], mm_intermediate[3]);
                // 5th col of row
                mm_tmp[2] = _mm_srli_si128(mm_residue[0], 4);
                mm_tmp[3] = _mm_srli_si128(mm_residue[1], 4);
                CALC_EQUAL_COEFF_8PXLS(mm_c[3], mm_c[7], mm_residue[0], mm_residue[1], mm_tmp[0], mm_tmp[1], mm_tmp[2], mm_tmp[3], mm_intermediate[0], mm_intermediate[1], mm_intermediate[2], mm_intermediate[3], (const __m128i*)&equal_coeff[4][4]);
                _mm_storel_epi64((__m128i*)&equal_coeff[4][4], mm_intermediate[3]);
            }

            idx1 -= (width);
            idx2 -= (width);
        }
    }
#else
    int affine_param_num = (vertex_num << 1);
    int j, k, col, row;

    for (j = 0; j != height; j++)
    {
        for (k = 0; k != width; k++)
        {
            s64 intermediates[2];
            int iC[6];
            int iIdx = j * derivate_buf_stride + k;

            if (vertex_num == 2)
            {
                iC[0] = derivate[0][iIdx];
                iC[1] = k * derivate[0][iIdx];
                iC[1] += j * derivate[1][iIdx];
                iC[2] = derivate[1][iIdx];
                iC[3] = j * derivate[0][iIdx];
                iC[3] -= k * derivate[1][iIdx];
            }
            else
            {
                iC[0] = derivate[0][iIdx];
                iC[1] = k * derivate[0][iIdx];
                iC[2] = derivate[1][iIdx];
                iC[3] = k * derivate[1][iIdx];
                iC[4] = j * derivate[0][iIdx];
                iC[5] = j * derivate[1][iIdx];
            }

            for (col = 0; col < affine_param_num; col++)
            {
                intermediates[0] = iC[col];

                for (row = 0; row < affine_param_num; row++)
                {
                    intermediates[1] = intermediates[0] * iC[row];

                    equal_coeff[col + 1][row] += intermediates[1];
                }

                intermediates[1] = intermediates[0] * residue[iIdx];

                equal_coeff[col + 1][affine_param_num] += intermediates[1] * 8;
            }
        }
    }
#endif
}

void solve_equal(double(*equal_coeff)[7], int order, double* affine_para)
{
    int i, j, k;

    // row echelon
    for(i = 1; i < order; i++)
    {
        // find column max
        double temp = fabs(equal_coeff[i][i - 1]);
        int temp_idx = i;
        for(j = i + 1; j < order + 1; j++)
        {
            if(fabs(equal_coeff[j][i - 1]) > temp)
            {
                temp = fabs(equal_coeff[j][i - 1]);
                temp_idx = j;
            }
        }

        // swap line
        if(temp_idx != i)
        {
            for(j = 0; j < order + 1; j++)
            {
                equal_coeff[0][j] = equal_coeff[i][j];
                equal_coeff[i][j] = equal_coeff[temp_idx][j];
                equal_coeff[temp_idx][j] = equal_coeff[0][j];
            }
        }

        // elimination first column
        for(j = i + 1; j < order + 1; j++)
        {
            for(k = i; k < order + 1; k++)
            {
                equal_coeff[j][k] = equal_coeff[j][k] - equal_coeff[i][k] * equal_coeff[j][i - 1] / equal_coeff[i][i - 1];
            }
        }
    }

    affine_para[order - 1] = equal_coeff[order][order] / equal_coeff[order][order - 1];
    for(i = order - 2; i >= 0; i--)
    {
        double temp = 0;
        for(j = i + 1; j < order; j++)
        {
            temp += equal_coeff[i + 1][j] * affine_para[j];
        }
        affine_para[i] = (equal_coeff[i + 1][order] - temp) / equal_coeff[i + 1][i];
    }
}

static int get_affine_mv_bits(s16 mv[VER_NUM][MV_D], s16 mvp[VER_NUM][MV_D], int num_refp, int refi, int vertex_num)
{
    int bits = 0;
    int vertex;

    int b_zero = 1;
    bits = 1;
    for(vertex = 0; vertex < vertex_num; vertex++)
    {
        int mvd_x = mv[vertex][MV_X] - mvp[vertex][MV_X];
        int mvd_y = mv[vertex][MV_Y] - mvp[vertex][MV_Y];
        if(mvd_x != 0 || mvd_y != 0)
        {
            b_zero = 0;
            break;
        }
    }
    if(b_zero)
    {
        return bits;
    }

    for(vertex = 0; vertex < vertex_num; vertex++)
    {
        int mvd_x = mv[vertex][MV_X] - mvp[vertex][MV_X];
        int mvd_y = mv[vertex][MV_Y] - mvp[vertex][MV_Y];
        if (vertex)
        {
          mvd_x -= (mv[0][MV_X] - mvp[0][MV_X]);
          mvd_y -= (mv[0][MV_Y] - mvp[0][MV_Y]);
        }
        bits += (mvd_x > 2048 || mvd_x <= -2048) ? get_exp_golomb_bits(EVC_ABS(mvd_x)) : evce_tbl_mv_bits[mvd_x];
        bits += (mvd_y > 2048 || mvd_y <= -2048) ? get_exp_golomb_bits(EVC_ABS(mvd_y)) : evce_tbl_mv_bits[mvd_y];
    }
    bits += evce_tbl_refi_bits[num_refp][refi];
    return bits;
}

static u32 pinter_affine_me_gradient(EVCE_PINTER * pi, int x, int y, int log2_cuw, int log2_cuh, s8 * refi, int lidx, s16 mvp[VER_NUM][MV_D], s16 mv[VER_NUM][MV_D], int bi, int vertex_num
#if EIF
                                     , pel *tmp_buffer_for_eif
#endif
)
{
    s16 mvt[VER_NUM][MV_D];
    s16 mvd[VER_NUM][MV_D];

    int cuw = 1 << log2_cuw;
    int cuh = 1 << log2_cuh;

    u32 cost, cost_best = EVC_UINT32_MAX;

    s8 ri = *refi;
    EVC_PIC* refp = pi->refp[ri][lidx].pic;

    pel *pred = pi->pred_buf;
    pel *org = bi ? pi->org_bi : (pi->o[Y_C] + x + y * pi->s_o[Y_C]);
    pel s_org = bi ? cuw : pi->s_o[Y_C];

    int mv_bits, best_bits;
    int vertex, iter;
    int iter_num = bi ? AF_ITER_BI : AF_ITER_UNI;
    int para_num = (vertex_num << 1) + 1;
    int affine_param_num = para_num - 1;

    double affine_para[6];
    double delta_mv[6];

    s64    equal_coeff_t[7][7];
    double equal_coeff[7][7];

    pel    *error = pi->p_error;
    int    *derivate[2];
    derivate[0] = pi->i_gradient[0];
    derivate[1] = pi->i_gradient[1];

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;

    /* set start mv */
    for(vertex = 0; vertex < vertex_num; vertex++)
    {
        mvt[vertex][MV_X] = mv[vertex][MV_X];
        mvt[vertex][MV_Y] = mv[vertex][MV_Y];
        mvd[vertex][MV_X] = 0;
        mvd[vertex][MV_Y] = 0;
    }

    /* do motion compensation with start mv */
    evc_affine_mc_l(x, y, refp->w_l, refp->h_l, cuw, cuh, mvt, refp, pred, vertex_num
#if EIF
                    , tmp_buffer_for_eif
#endif
    );

    /* get mvd bits*/
    best_bits = get_affine_mv_bits(mvt, mvp, pi->num_refp, ri, vertex_num);
    if(bi)
    {
        best_bits += pi->mot_bits[1 - lidx];
    }
    cost_best = MV_COST(pi, best_bits);

    /* get satd */
    cost_best += evce_satd_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw) >> bi;

    if(vertex_num == 3)
    {
        iter_num = bi ? (AF_ITER_BI - 2) : (AF_ITER_UNI - 2);
    }

    for(iter = 0; iter < iter_num; iter++)
    {
        int row, col;
        int all_zero = 0;

        evce_diff_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw, cuw, error);

        // sobel x direction
        // -1 0 1
        // -2 0 2
        // -1 0 1

        scaled_horizontal_sobel_filter(pred, cuw, derivate[0], cuw, cuw, cuh);

        // sobel y direction
        // -1 -2 -1
        //  0  0  0
        //  1  2  1

        scaled_vertical_sobel_filter(pred, cuw, derivate[1], cuw, cuw, cuh);

        // solve delta x and y
        for(row = 0; row < para_num; row++)
        {
            evc_mset(&equal_coeff_t[row][0], 0, para_num * sizeof(s64));
        }

        equal_coeff_computer(error, cuw, derivate, cuw, equal_coeff_t, cuw, cuh, vertex_num);
        for(row = 0; row < para_num; row++)
        {
            for(col = 0; col < para_num; col++)
            {
                equal_coeff[row][col] = (double)equal_coeff_t[row][col];
            }
        }
        solve_equal(equal_coeff, affine_param_num, affine_para);

        // convert to delta mv
        if(vertex_num == 3)
        {
            // for MV0
            delta_mv[0] = affine_para[0];
            delta_mv[2] = affine_para[2];
            // for MV1
            delta_mv[1] = affine_para[1] * cuw + affine_para[0];
            delta_mv[3] = affine_para[3] * cuw + affine_para[2];
            // for MV2
            delta_mv[4] = affine_para[4] * cuh + affine_para[0];
            delta_mv[5] = affine_para[5] * cuh + affine_para[2];

            mvd[0][MV_X] = (s16)(delta_mv[0] * 4 + (delta_mv[0] >= 0 ? 0.5 : -0.5));
            mvd[0][MV_Y] = (s16)(delta_mv[2] * 4 + (delta_mv[2] >= 0 ? 0.5 : -0.5));
            mvd[1][MV_X] = (s16)(delta_mv[1] * 4 + (delta_mv[1] >= 0 ? 0.5 : -0.5));
            mvd[1][MV_Y] = (s16)(delta_mv[3] * 4 + (delta_mv[3] >= 0 ? 0.5 : -0.5));
            mvd[2][MV_X] = (s16)(delta_mv[4] * 4 + (delta_mv[4] >= 0 ? 0.5 : -0.5));
            mvd[2][MV_Y] = (s16)(delta_mv[5] * 4 + (delta_mv[5] >= 0 ? 0.5 : -0.5));
        }
        else
        {
            // for MV0
            delta_mv[0] = affine_para[0];
            delta_mv[2] = affine_para[2];
            // for MV1
            delta_mv[1] = affine_para[1] * cuw + affine_para[0];
            delta_mv[3] = -affine_para[3] * cuw + affine_para[2];

            mvd[0][MV_X] = (s16)(delta_mv[0] * 4 + (delta_mv[0] >= 0 ? 0.5 : -0.5));
            mvd[0][MV_Y] = (s16)(delta_mv[2] * 4 + (delta_mv[2] >= 0 ? 0.5 : -0.5));
            mvd[1][MV_X] = (s16)(delta_mv[1] * 4 + (delta_mv[1] >= 0 ? 0.5 : -0.5));
            mvd[1][MV_Y] = (s16)(delta_mv[3] * 4 + (delta_mv[3] >= 0 ? 0.5 : -0.5));
        }

        // check early terminate
        for(vertex = 0; vertex < vertex_num; vertex++)
        {
            if(mvd[vertex][MV_X] != 0 || mvd[vertex][MV_Y] != 0)
            {
                all_zero = 0;
                break;
            }
            all_zero = 1;
        }
        if(all_zero)
        {
            break;
        }

        /* update mv */
        for(vertex = 0; vertex < vertex_num; vertex++)
        {
            mvt[vertex][MV_X] += mvd[vertex][MV_X];
            mvt[vertex][MV_Y] += mvd[vertex][MV_Y];
        }

        /* do motion compensation with updated mv */
        evc_affine_mc_l(x, y, refp->w_l, refp->h_l, cuw, cuh, mvt, refp, pred, vertex_num
#if EIF
                        , tmp_buffer_for_eif
#endif
        );

        /* get mvd bits*/
        mv_bits = get_affine_mv_bits(mvt, mvp, pi->num_refp, ri, vertex_num);
        if(bi)
        {
            mv_bits += pi->mot_bits[1 - lidx];
        }
        cost = MV_COST(pi, mv_bits);

        /* get satd */
        cost += evce_satd_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw) >> bi;

        /* save best mv */
        if(cost < cost_best)
        {
            cost_best = cost;
            best_bits = mv_bits;
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                mv[vertex][MV_X] = mvt[vertex][MV_X];
                mv[vertex][MV_Y] = mvt[vertex][MV_Y];
            }
        }
    }

    return (cost_best - MV_COST(pi, best_bits));
}

static void check_best_affine_mvp(EVCE_CTX * ctx, EVCE_CORE * core, s32 tile_group_type, s8 refi[REFP_NUM],
                                  int lidx, int pidx, s16(*mvp)[VER_NUM][MV_D], s16(*mv)[MV_D], s16(*mvd)[MV_D], u8* mvp_idx, int vertex_num)
{
    double cost, best_cost;
    int idx, best_idx;
    int vertex;
    u32 bit_cnt;
    s16 mvd_tmp[REFP_NUM][VER_NUM][MV_D];

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[core->log2_cuw - 2][core->log2_cuh - 2]);
    evce_sbac_bit_reset(&core->s_temp_run);

    for(vertex = 0; vertex < vertex_num; vertex++)
    {
        mvd_tmp[lidx][vertex][MV_X] = mv[vertex][MV_X] - mvp[*mvp_idx][vertex][MV_X];
        mvd_tmp[lidx][vertex][MV_Y] = mv[vertex][MV_Y] - mvp[*mvp_idx][vertex][MV_Y];
        if(vertex)
        {
            mvd_tmp[lidx][vertex][MV_X] -= mvd_tmp[lidx][0][MV_X];
            mvd_tmp[lidx][vertex][MV_Y] -= mvd_tmp[lidx][0][MV_Y];
        }
    }
    evce_rdo_bit_cnt_affine_mvp(ctx, core, tile_group_type, refi, mvd_tmp, pidx, *mvp_idx, vertex_num);
    bit_cnt = evce_get_bit_number(&core->s_temp_run);

    best_cost = RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

    best_idx = *mvp_idx;

    for(idx = 0; idx < AFF_MAX_NUM_MVP; idx++)
    {
        if(idx == *mvp_idx)
        {
            continue;
        }

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[core->log2_cuw - 2][core->log2_cuh - 2]);
        evce_sbac_bit_reset(&core->s_temp_run);

        for(vertex = 0; vertex < vertex_num; vertex++)
        {
            mvd_tmp[lidx][vertex][MV_X] = mv[vertex][MV_X] - mvp[idx][vertex][MV_X];
            mvd_tmp[lidx][vertex][MV_Y] = mv[vertex][MV_Y] - mvp[idx][vertex][MV_Y];
            if (vertex)
            {
                mvd_tmp[lidx][vertex][MV_X] -= mvd_tmp[lidx][0][MV_X];
                mvd_tmp[lidx][vertex][MV_Y] -= mvd_tmp[lidx][0][MV_Y];
            }
        }
        evce_rdo_bit_cnt_affine_mvp(ctx, core, tile_group_type, refi, mvd_tmp, pidx, idx, vertex_num);
        bit_cnt = evce_get_bit_number(&core->s_temp_run);

        cost = RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
        if(cost < best_cost)
        {
            best_idx = idx;
        }
    }

    *mvp_idx = best_idx;
    for(vertex = 0; vertex < vertex_num; vertex++)
    {
        mvd[vertex][MV_X] = mv[vertex][MV_X] - mvp[*mvp_idx][vertex][MV_X];
        mvd[vertex][MV_Y] = mv[vertex][MV_Y] - mvp[*mvp_idx][vertex][MV_Y];
    }
}

static double analyze_affine_bi(EVCE_CTX * ctx, EVCE_CORE * core, EVCE_PINTER * pi,
                                int x, int y, int log2_cuw, int log2_cuh, double * cost_inter, int pred_mode, int vertex_num)
{
    s8         refi[REFP_NUM] = {REFI_INVALID, REFI_INVALID};
    int        t1;
    u32        best_mecost = EVC_UINT32_MAX;
    int        refi_best = 0, refi_cur;
    int        changed = 0;
    u32        mecost;
    pel        *org;
    pel(*pred)[N_C][MAX_CU_DIM];

    int         cuw, cuh, t0;
    double      cost;
    int         lidx_ref, lidx_cnd;
    u8          mvp_idx = 0;
    int         pidx, pidx_ref, pidx_cnd, i;
    int         vertex;
    int         mebits;
    int         memory_access[REFP_NUM];
    int         mem = MAX_MEMORY_ACCESS_BI * (1 << log2_cuw) * (1 << log2_cuh);
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    {
        cuw = (1 << log2_cuw);
        cuh = (1 << log2_cuh);

        if(vertex_num == 3)
        {
            pidx = AFF_6_BI;
            if(cost_inter[AFF_6_L0] <= cost_inter[AFF_6_L1])
            {
                lidx_ref = REFP_0;
                lidx_cnd = REFP_1;
                pidx_ref = AFF_6_L0;
                pidx_cnd = AFF_6_L1;
            }
            else
            {
                lidx_ref = REFP_1;
                lidx_cnd = REFP_0;
                pidx_ref = AFF_6_L1;
                pidx_cnd = AFF_6_L0;
            }
            pi->mvp_idx[pidx][REFP_0] = pi->mvp_idx[AFF_6_L0][REFP_0];
            pi->mvp_idx[pidx][REFP_1] = pi->mvp_idx[AFF_6_L1][REFP_1];
            pi->refi[pidx][REFP_0] = pi->refi[AFF_6_L0][REFP_0];
            pi->refi[pidx][REFP_1] = pi->refi[AFF_6_L1][REFP_1];
        }
        else
        {
            pidx = AFF_BI;
            if(cost_inter[AFF_L0] <= cost_inter[AFF_L1])
            {
                lidx_ref = REFP_0;
                lidx_cnd = REFP_1;
                pidx_ref = AFF_L0;
                pidx_cnd = AFF_L1;
            }
            else
            {
                lidx_ref = REFP_1;
                lidx_cnd = REFP_0;
                pidx_ref = AFF_L1;
                pidx_cnd = AFF_L0;
            }
            pi->mvp_idx[pidx][REFP_0] = pi->mvp_idx[AFF_L0][REFP_0];
            pi->mvp_idx[pidx][REFP_1] = pi->mvp_idx[AFF_L1][REFP_1];
            pi->refi[pidx][REFP_0] = pi->refi[AFF_L0][REFP_0];
            pi->refi[pidx][REFP_1] = pi->refi[AFF_L1][REFP_1];
        }

        for(vertex = 0; vertex < vertex_num; vertex++)
        {
            pi->affine_mv[pidx][lidx_ref][vertex][MV_X] = pi->affine_mv[pidx_ref][lidx_ref][vertex][MV_X];
            pi->affine_mv[pidx][lidx_ref][vertex][MV_Y] = pi->affine_mv[pidx_ref][lidx_ref][vertex][MV_Y];
            pi->affine_mv[pidx][lidx_cnd][vertex][MV_X] = pi->affine_mv[pidx_ref][lidx_cnd][vertex][MV_X];
            pi->affine_mv[pidx][lidx_cnd][vertex][MV_Y] = pi->affine_mv[pidx_ref][lidx_cnd][vertex][MV_Y];
        }

        /* get MVP lidx_cnd */
        org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
        pred = pi->pred[pidx];

        t0 = (lidx_ref == REFP_0) ? pi->refi[pidx][lidx_ref] : REFI_INVALID;
        t1 = (lidx_ref == REFP_1) ? pi->refi[pidx][lidx_ref] : REFI_INVALID;
        SET_REFI(refi, t0, t1);

        for(i = 0; i < AFFINE_BI_ITER; i++)
        {
            /* predict reference */
            evc_affine_mc(x, y, ctx->w, ctx->h, cuw, cuh, refi, pi->affine_mv[pidx], pi->refp, pred, vertex_num
#if EIF
                          , core->eif_tmp_buffer
#endif
            );

            get_org_bi(org, pred[0][Y_C], pi->s_o[Y_C], cuw, cuh, pi->org_bi);

            SWAP(refi[lidx_ref], refi[lidx_cnd], t0);
            SWAP(lidx_ref, lidx_cnd, t0);
            SWAP(pidx_ref, pidx_cnd, t0);

            mvp_idx = pi->mvp_idx[pidx][lidx_ref];
            changed = 0;
            for(refi_cur = 0; refi_cur < pi->num_refp; refi_cur++)
            {
                refi[lidx_ref] = refi_cur;
                mvp_idx = pi->mvp_idx_scale[lidx_ref][refi_cur];
                mecost = pi->fn_affine_me(pi, x, y, log2_cuw, log2_cuh, &refi[lidx_ref], lidx_ref, \
                                          pi->affine_mvp_scale[lidx_ref][refi_cur][mvp_idx], pi->affine_mv_scale[lidx_ref][refi_cur], 1, vertex_num
#if EIF
                                          , core->eif_tmp_buffer
#endif
                );

                // update MVP bits
                check_best_affine_mvp(ctx, core, pi->tile_group_type, refi, lidx_ref, pidx, pi->affine_mvp_scale[lidx_ref][refi_cur], pi->affine_mv_scale[lidx_ref][refi_cur], pi->affine_mvd[pidx][lidx_ref], &mvp_idx, vertex_num);

                mebits = get_affine_mv_bits(pi->affine_mv_scale[lidx_ref][refi_cur], pi->affine_mvp_scale[lidx_ref][refi_cur][mvp_idx], pi->num_refp, refi_cur, vertex_num);
                mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][mvp_idx];
                mebits += pi->mot_bits[1 - lidx_ref]; // add opposite bits

                mecost += MV_COST(pi, mebits);
                pi->mvp_idx_scale[lidx_ref][refi_cur] = mvp_idx;

                if(mecost < best_mecost)
                {
                    pi->mot_bits[lidx_ref] = mebits - pi->mot_bits[1 - lidx_ref];
                    pi->mvp_idx[pidx][lidx_ref] = mvp_idx;
                    refi_best = refi_cur;
                    best_mecost = mecost;
                    changed = 1;
                    t0 = (lidx_ref == REFP_0) ? refi_best : pi->refi[pidx][lidx_cnd];
                    t1 = (lidx_ref == REFP_1) ? refi_best : pi->refi[pidx][lidx_cnd];
                    SET_REFI(pi->refi[pidx], t0, t1);

                    for(vertex = 0; vertex < vertex_num; vertex++)
                    {
                        pi->affine_mv[pidx][lidx_ref][vertex][MV_X] = pi->affine_mv_scale[lidx_ref][refi_cur][vertex][MV_X];
                        pi->affine_mv[pidx][lidx_ref][vertex][MV_Y] = pi->affine_mv_scale[lidx_ref][refi_cur][vertex][MV_Y];
                    }
                }
            }

            t0 = (lidx_ref == REFP_0) ? refi_best : REFI_INVALID;
            t1 = (lidx_ref == REFP_1) ? refi_best : REFI_INVALID;
            SET_REFI(refi, t0, t1);

            if(!changed) break;
        }

        for(vertex = 0; vertex < vertex_num; vertex++)
        {
            pi->affine_mvd[pidx][REFP_0][vertex][MV_X] = pi->affine_mv[pidx][REFP_0][vertex][MV_X] - pi->affine_mvp_scale[REFP_0][pi->refi[pidx][REFP_0]][pi->mvp_idx[pidx][REFP_0]][vertex][MV_X];
            pi->affine_mvd[pidx][REFP_0][vertex][MV_Y] = pi->affine_mv[pidx][REFP_0][vertex][MV_Y] - pi->affine_mvp_scale[REFP_0][pi->refi[pidx][REFP_0]][pi->mvp_idx[pidx][REFP_0]][vertex][MV_Y];
            pi->affine_mvd[pidx][REFP_1][vertex][MV_X] = pi->affine_mv[pidx][REFP_1][vertex][MV_X] - pi->affine_mvp_scale[REFP_1][pi->refi[pidx][REFP_1]][pi->mvp_idx[pidx][REFP_1]][vertex][MV_X];
            pi->affine_mvd[pidx][REFP_1][vertex][MV_Y] = pi->affine_mv[pidx][REFP_1][vertex][MV_Y] - pi->affine_mvp_scale[REFP_1][pi->refi[pidx][REFP_1]][pi->mvp_idx[pidx][REFP_1]][vertex][MV_Y];
            if(vertex)
            {
                pi->affine_mvd[pidx][REFP_0][vertex][MV_X] -= pi->affine_mvd[pidx][REFP_0][0][MV_X];
                pi->affine_mvd[pidx][REFP_0][vertex][MV_Y] -= pi->affine_mvd[pidx][REFP_0][0][MV_Y];
                pi->affine_mvd[pidx][REFP_1][vertex][MV_X] -= pi->affine_mvd[pidx][REFP_1][0][MV_X];
                pi->affine_mvd[pidx][REFP_1][vertex][MV_Y] -= pi->affine_mvd[pidx][REFP_1][0][MV_Y];
            }
        }
    }

    for(i = 0; i < REFP_NUM; i++)
    {
        if(vertex_num == 3)
        {
            pi->affine_mv[pidx][i][3][MV_X] = pi->affine_mv[pidx][i][1][MV_X] + pi->affine_mv[pidx][i][2][MV_X] - pi->affine_mv[pidx][i][0][MV_X];
            pi->affine_mv[pidx][i][3][MV_Y] = pi->affine_mv[pidx][i][1][MV_Y] + pi->affine_mv[pidx][i][2][MV_Y] - pi->affine_mv[pidx][i][0][MV_Y];
        }
        else
        {
            pi->affine_mv[pidx][i][2][MV_X] = pi->affine_mv[pidx][i][0][MV_X] - (pi->affine_mv[pidx][i][1][MV_Y] + pi->affine_mv[pidx][i][0][MV_Y]) * cuh / cuh;
            pi->affine_mv[pidx][i][2][MV_Y] = pi->affine_mv[pidx][i][0][MV_Y] + (pi->affine_mv[pidx][i][1][MV_X] + pi->affine_mv[pidx][i][0][MV_X]) * cuh / cuh;
            pi->affine_mv[pidx][i][3][MV_X] = pi->affine_mv[pidx][i][0][MV_X] - (pi->affine_mv[pidx][i][1][MV_Y] + pi->affine_mv[pidx][i][0][MV_Y]) * cuh / cuh;
            pi->affine_mv[pidx][i][3][MV_Y] = pi->affine_mv[pidx][i][0][MV_Y] + (pi->affine_mv[pidx][i][1][MV_X] + pi->affine_mv[pidx][i][0][MV_X]) * cuh / cuh;
        }
        memory_access[i] = evc_get_affine_memory_access(pi->affine_mv[pidx][i], cuw, cuh);
    }
    if(memory_access[0] > mem || memory_access[1] > mem)
    {
        return MAX_COST;
    }
    else
    {
        cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[pidx], pi->coef[pidx], pidx, pi->mvp_idx[pidx]
#if DMVR
                                  , FALSE
#endif
        );
        evc_mcpy(pi->nnz_best[pidx], core->nnz, sizeof(int) * N_C);
        evc_mcpy(pi->nnz_sub_best[pidx], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
#if ATS_INTER_PROCESS
        pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
        return cost;
    }
}

static double analyze_affine_merge(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, int pidx)
{
    EVCE_PINTER *pi = &ctx->pinter;
    pel          *y_org, *u_org, *v_org;
    s16          mrg_list_cp_mv[AFF_MAX_CAND][REFP_NUM][VER_NUM][MV_D];
    s8           mrg_list_refi[AFF_MAX_CAND][REFP_NUM];
    int          mrg_list_cp_num[AFF_MAX_CAND];

    double       cost, cost_best = MAX_COST;
    int          cuw, cuh, idx, bit_cnt, mrg_cnt, best_idx = 0;
    s64          cy, cu, cv;
    int          j;
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif

    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);
    y_org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
    u_org = pi->o[U_C] + (x >> 1) + ((y >> 1) * pi->s_o[U_C]);
    v_org = pi->o[V_C] + (x >> 1) + ((y >> 1) * pi->s_o[V_C]);

    mrg_cnt = evc_get_affine_merge_candidate(ctx->ptr, ctx->tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, pi->refp, cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mrg_list_refi, mrg_list_cp_mv, mrg_list_cp_num, ctx->map_scu, ctx->map_affine
#if DMVR_LAG
                                             , ctx->map_unrefined_mv
#endif
    );

    if(mrg_cnt == 0)
        return MAX_COST;

    for(idx = 0; idx < mrg_cnt; idx++)
    {
        int memory_access[REFP_NUM];
        int allowed = 1;
        int i;
        for ( i = 0; i < REFP_NUM; i++ )
        {
            if ( REFI_IS_VALID( mrg_list_refi[idx][i] ) )
            {
                if ( mrg_list_cp_num[idx] == 3 ) // derive RB
                {
                    mrg_list_cp_mv[idx][i][3][MV_X] = mrg_list_cp_mv[idx][i][1][MV_X] + mrg_list_cp_mv[idx][i][2][MV_X] - mrg_list_cp_mv[idx][i][0][MV_X];
                    mrg_list_cp_mv[idx][i][3][MV_Y] = mrg_list_cp_mv[idx][i][1][MV_Y] + mrg_list_cp_mv[idx][i][2][MV_Y] - mrg_list_cp_mv[idx][i][0][MV_Y];
                }
                else // derive LB, RB
                {
                    mrg_list_cp_mv[idx][i][2][MV_X] = mrg_list_cp_mv[idx][i][0][MV_X] - (mrg_list_cp_mv[idx][i][1][MV_Y] - mrg_list_cp_mv[idx][i][0][MV_Y]) * (s16)cuh / (s16)cuw;
                    mrg_list_cp_mv[idx][i][2][MV_Y] = mrg_list_cp_mv[idx][i][0][MV_Y] + (mrg_list_cp_mv[idx][i][1][MV_X] - mrg_list_cp_mv[idx][i][0][MV_X]) * (s16)cuh / (s16)cuw;
                    mrg_list_cp_mv[idx][i][3][MV_X] = mrg_list_cp_mv[idx][i][1][MV_X] - (mrg_list_cp_mv[idx][i][1][MV_Y] - mrg_list_cp_mv[idx][i][0][MV_Y]) * (s16)cuh / (s16)cuw;
                    mrg_list_cp_mv[idx][i][3][MV_Y] = mrg_list_cp_mv[idx][i][1][MV_Y] + (mrg_list_cp_mv[idx][i][1][MV_X] - mrg_list_cp_mv[idx][i][0][MV_X]) * (s16)cuh / (s16)cuw;
                }
                memory_access[i] = evc_get_affine_memory_access( mrg_list_cp_mv[idx][i], cuw, cuh );
            }
        }

        if ( REFI_IS_VALID( mrg_list_refi[idx][0] ) && REFI_IS_VALID( mrg_list_refi[idx][1] ) )
        {
            int mem = MAX_MEMORY_ACCESS_BI * cuw * cuh;
            if ( memory_access[0] > mem || memory_access[1] > mem )
            {
                allowed = 0;
            }
        }
        else
        {
            int valid_idx = REFI_IS_VALID( mrg_list_refi[idx][0] ) ? 0 : 1;
            int mem = MAX_MEMORY_ACCESS_UNI * cuw * cuh;
            if ( memory_access[valid_idx] > mem )
            {
                allowed = 0;
            }
        }
        if ( !allowed )
        {
            continue;
        }

        // set motion information for MC
        core->affine_flag = mrg_list_cp_num[idx] - 1;
        pi->mvp_idx[pidx][REFP_0] = idx;
        pi->mvp_idx[pidx][REFP_1] = 0;
        for(j = 0; j < mrg_list_cp_num[idx]; j++)
        {
            pi->affine_mv[pidx][REFP_0][j][MV_X] = mrg_list_cp_mv[idx][REFP_0][j][MV_X];
            pi->affine_mv[pidx][REFP_0][j][MV_Y] = mrg_list_cp_mv[idx][REFP_0][j][MV_Y];
            pi->affine_mv[pidx][REFP_1][j][MV_X] = mrg_list_cp_mv[idx][REFP_1][j][MV_X];
            pi->affine_mv[pidx][REFP_1][j][MV_Y] = mrg_list_cp_mv[idx][REFP_1][j][MV_Y];
        }
        pi->refi[pidx][REFP_0] = mrg_list_refi[idx][REFP_0];
        pi->refi[pidx][REFP_1] = mrg_list_refi[idx][REFP_1];

        if(pidx == AFF_DIR)
        {
            cost = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[PRED_NUM], pi->coef[PRED_NUM], pidx, pi->mvp_idx[pidx]
#if DMVR
                                      , FALSE
#endif
            );
        }
        else
        {
#if ATS_INTER_PROCESS
            assert(core->ats_inter_info == 0);
#endif
            evc_affine_mc(x, y, ctx->w, ctx->h, cuw, cuh, mrg_list_refi[idx], mrg_list_cp_mv[idx], pi->refp, pi->pred[PRED_NUM], mrg_list_cp_num[idx]
#if EIF
                          , core->eif_tmp_buffer
#endif
            );

            cy = evce_ssd_16b(log2_cuw, log2_cuh, pi->pred[PRED_NUM][0][Y_C], y_org, cuw, pi->s_o[Y_C]);
            cu = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][U_C], u_org, cuw >> 1, pi->s_o[U_C]);
            cv = evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->pred[PRED_NUM][0][V_C], v_org, cuw >> 1, pi->s_o[V_C]);

#if RDO_DBK
            evc_set_affine_mvf(ctx, core, cuw, cuh, mrg_list_refi[idx], mrg_list_cp_mv[idx], mrg_list_cp_num[idx]);
            calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pi->pred[PRED_NUM][0], cuw, x, y, core->avail_lr, 0, 0, mrg_list_refi[idx], pi->mv[pidx], 1
#if ATS_INTER_PROCESS
                                            , core->ats_inter_info
#endif
            );
            cy += ctx->delta_dist[Y_C];
            cu += ctx->delta_dist[U_C];
            cv += ctx->delta_dist[V_C];
#endif

            cost = (double)cy + (ctx->dist_chroma_weight[0] * (double)cu) + (ctx->dist_chroma_weight[1] * (double)cv);

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

            evce_sbac_bit_reset(&core->s_temp_run);
            evce_rdo_bit_cnt_cu_skip(ctx, core, ctx->tgh.tile_group_type, core->scup, idx, 0, 0, ctx->sps.tool_mmvd);

            bit_cnt = evce_get_bit_number(&core->s_temp_run);
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
#if ATS_INTER_PROCESS
            core->cost_best = cost < core->cost_best ? cost : core->cost_best;
#endif
        }

        // store best pred and coeff
        if(cost < cost_best)
        {
            cost_best = cost;
            best_idx = idx;

            evc_mcpy(pi->nnz_best[pidx], core->nnz, sizeof(int) * N_C);
            evc_mcpy(pi->nnz_sub_best[pidx], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
#if ATS_INTER_PROCESS
            pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif

            for(j = 0; j < N_C; j++)
            {
                int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                evc_mcpy(pi->pred[pidx][0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                evc_mcpy(pi->coef[pidx][j], pi->coef[PRED_NUM][j], size_tmp * sizeof(s16));
            }

            SBAC_STORE(core->s_temp_best, core->s_temp_run);
        }
    }

    // set best motion information
    if(mrg_cnt >= 1)
    {
        core->affine_flag = mrg_list_cp_num[best_idx] - 1;

        pi->mvp_idx[pidx][REFP_0] = best_idx;
        pi->mvp_idx[pidx][REFP_1] = 0;
        for(j = 0; j < mrg_list_cp_num[best_idx]; j++)
        {
            pi->affine_mv[pidx][REFP_0][j][MV_X] = mrg_list_cp_mv[best_idx][REFP_0][j][MV_X];
            pi->affine_mv[pidx][REFP_0][j][MV_Y] = mrg_list_cp_mv[best_idx][REFP_0][j][MV_Y];
            pi->affine_mv[pidx][REFP_1][j][MV_X] = mrg_list_cp_mv[best_idx][REFP_1][j][MV_X];
            pi->affine_mv[pidx][REFP_1][j][MV_Y] = mrg_list_cp_mv[best_idx][REFP_1][j][MV_Y];
        }
        pi->refi[pidx][REFP_0] = mrg_list_refi[best_idx][REFP_0];
        pi->refi[pidx][REFP_1] = mrg_list_refi[best_idx][REFP_1];

        pi->mv[pidx][REFP_0][MV_X] = 0;
        pi->mv[pidx][REFP_0][MV_Y] = 0;
        pi->mv[pidx][REFP_1][MV_X] = 0;
        pi->mv[pidx][REFP_1][MV_Y] = 0;

        pi->mvd[pidx][REFP_0][MV_X] = 0;
        pi->mvd[pidx][REFP_0][MV_Y] = 0;
        pi->mvd[pidx][REFP_1][MV_X] = 0;
        pi->mvd[pidx][REFP_1][MV_Y] = 0;
    }

    return cost_best;
}
#endif

static double pinter_analyze_cu_baseline(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, EVCE_MODE *mi, s16 coef[N_C][MAX_CU_DIM], pel *rec[N_C], int s_rec[N_C])
{
    EVCE_PINTER *pi;
    s8 *refi;
    s8 refi_temp = 0;
    u32 mecost, best_mecost;
    pel(*pred)[N_C][MAX_CU_DIM];
    s16(*coef_t)[MAX_CU_DIM];
    s16(*mvp)[MV_D], *mv, *mvd;
    int cuw, cuh, t0, t1, best_idx = PRED_SKIP, i, j;
    u8 mvp_idx[REFP_NUM] = {0, 0};
    s8 refi_cur = 0;
    double cost, cost_best = MAX_COST;
    double cost_inter[PRED_NUM];
    int lidx, pidx;
#if ATS_INTER_PROCESS
    int log2_tuw, log2_tuh;
#endif

    pi = &ctx->pinter;
    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);

    for(i = 0; i < PRED_NUM; i++)
    {
        cost_inter[i] = MAX_COST;
    }

    /* skip mode */
    cost = cost_inter[PRED_SKIP] = analyze_skip_baseline(ctx, core, x, y, log2_cuw, log2_cuh);

    if(cost < cost_best)
    {
        core->cu_mode = MODE_SKIP;
        best_idx = PRED_SKIP;
        cost_inter[best_idx] = cost_best = cost;
        SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
        evc_mset(pi->nnz_best[PRED_SKIP], 0, sizeof(int) * N_C);
        evc_mcpy(pi->nnz_sub_best[PRED_SKIP], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
    }

    if(pi->tile_group_type == TILE_GROUP_B)
    {
        cost = cost_inter[PRED_DIR] = analyze_t_direct(ctx, core, x, y, log2_cuw, log2_cuh);
        if(cost < cost_best)
        {
            core->cu_mode = MODE_DIR;
            best_idx = PRED_DIR;
            cost_inter[best_idx] = cost_best = cost;

            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
            evc_mcpy(pi->nnz_sub_best[PRED_DIR], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
        }
    }

    /* Motion Search *********************************************************/
    for(lidx = 0; lidx <= ((pi->tile_group_type == TILE_GROUP_P) ? PRED_L0 : PRED_L1); lidx++)
    {
        pidx = lidx;
        refi = pi->refi[pidx];
        mv = pi->mv[pidx][lidx];
        mvd = pi->mvd[pidx][lidx];
        pred = pi->pred[pidx];
        coef_t = pi->coef[pidx];

        pi->num_refp = ctx->rpm.num_refp[lidx];

        best_mecost = EVC_UINT32_MAX;

        for(refi_cur = 0; refi_cur < pi->num_refp; refi_cur++)
        {
            mvp = pi->mvp_scale[lidx][refi_cur];
            evc_get_motion(core->scup, lidx, ctx->map_refi, ctx->map_mv, pi->refp, core->cuw, core->cuh, ctx->w_scu, core->avail_cu, pi->refi_pred[lidx], mvp);
            mvp_idx[lidx] = pi->mvp_idx[PRED_SKIP][lidx];

            /* motion search ********************/
            u8 skip_me = 0;
            if(skip_me)
            {
                mecost = EVC_UINT32_MAX;
                mv[MV_X] = mvp[mvp_idx[lidx]][MV_X];
                mv[MV_Y] = mvp[mvp_idx[lidx]][MV_Y];
            }
            else
            {
                mecost = pi->fn_me(pi, x, y, log2_cuw, log2_cuh, &refi_cur, lidx, mvp[mvp_idx[lidx]], mv, 0);
            }

            pi->mv_scale[lidx][refi_cur][MV_X] = mv[MV_X];
            pi->mv_scale[lidx][refi_cur][MV_Y] = mv[MV_Y];
            if(mecost < best_mecost)
            {
                best_mecost = mecost;
                refi_temp = refi_cur;
            }
        }

        refi_cur = refi_temp;
        mv[MV_X] = pi->mv_scale[lidx][refi_cur][MV_X];
        mv[MV_Y] = pi->mv_scale[lidx][refi_cur][MV_Y];
        mvp = pi->mvp_scale[lidx][refi_cur];

        t0 = (lidx == 0) ? refi_cur : REFI_INVALID;
        t1 = (lidx == 1) ? refi_cur : REFI_INVALID;
        SET_REFI(refi, t0, t1);

        mvd[MV_X] = mv[MV_X] - mvp[mvp_idx[lidx]][MV_X];
        mvd[MV_Y] = mv[MV_Y] - mvp[mvp_idx[lidx]][MV_Y];

        check_best_mvp(ctx, core, pi->tile_group_type, refi, lidx, pidx, mvp, mv, mvd, &mvp_idx[lidx]);

        pi->mvp_idx[pidx][lidx] = mvp_idx[lidx];

        cost = cost_inter[pidx] = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[PRED_NUM], pi->coef[PRED_NUM], pidx, mvp_idx, FALSE);

        if(cost < cost_best)
        {
            core->cu_mode = MODE_INTER;
            best_idx = pidx;

            pi->mvp_idx[best_idx][lidx] = mvp_idx[lidx];
            cost_inter[best_idx] = cost_best = cost;
            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);

            for(j = 0; j < N_C; j++)
            {
                int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                pi->nnz_best[pidx][j] = core->nnz[j];
                evc_mcpy(pi->nnz_sub_best[pidx][j], core->nnz_sub[j], sizeof(int) * MAX_SUB_TB_NUM);
                evc_mcpy(pred[0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                evc_mcpy(coef_t[j], pi->coef[PRED_NUM][j], size_tmp * sizeof(s16));
            }
#if ATS_INTER_PROCESS
            pi->ats_inter_info_mode[best_idx] = core->ats_inter_info;
#endif
        }
    }

    if(pi->tile_group_type == TILE_GROUP_B)
    {
        pidx = PRED_BI;
        cost = cost_inter[pidx] = analyze_bi(ctx, core, x, y, log2_cuw, log2_cuh, cost_inter);

        if(cost < cost_best)
        {
            core->cu_mode = MODE_INTER;
            best_idx = pidx;
            cost_inter[best_idx] = cost_best = cost;
            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
            evc_mcpy(pi->nnz_sub_best[best_idx], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
        }
    }

    /* reconstruct */
    for(j = 0; j < N_C; j++)
    {
        int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
        evc_mcpy(coef[j], pi->coef[best_idx][j], sizeof(s16) * size_tmp);
        evc_mcpy(pi->residue[j], pi->coef[best_idx][j], sizeof(s16) * size_tmp);
    }
#if ATS_INTER_PROCESS
    core->ats_inter_info = pi->ats_inter_info_mode[best_idx];
    get_tu_size(get_ats_inter_idx(core->ats_inter_info), log2_cuw, log2_cuh, &log2_tuw, &log2_tuh);
#if ATS_INTER_DEBUG
    assert(core->cost_best == cost_inter[best_idx]);
    if( pi->nnz_best[best_idx][Y_C] && log2_cuw <= MAX_TR_LOG2 && log2_cuh <= MAX_TR_LOG2 )
    {
        int sum_y_coef = 0;
        for (int a = 0; a < (1 << (log2_tuw + log2_tuh)); a++)
        {
            sum_y_coef += coef[Y_C][a] != 0;
        }
        assert(sum_y_coef == pi->nnz_best[best_idx][Y_C]);
    }
#endif
#endif

    evc_sub_block_itdq(pi->residue, log2_cuw, log2_cuh, pi->qp_y, pi->qp_u, pi->qp_v, pi->nnz_best[best_idx], pi->nnz_sub_best[best_idx], ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
                       , 0, 0
#endif
#if ATS_INTER_PROCESS
                       , core->ats_inter_info
#endif
    );

    for(i = 0; i < N_C; i++)
    {
        rec[i] = pi->rec[best_idx][i];
        s_rec[i] = (i == 0 ? cuw : cuw >> 1);
        evc_recon(pi->residue[i], pi->pred[best_idx][0][i], pi->nnz_best[best_idx][i], s_rec[i], (i == 0 ? cuh : cuh >> 1), s_rec[i], rec[i]
#if ATS_INTER_PROCESS
                  , core->ats_inter_info
#endif
        );
        core->nnz[i] = pi->nnz_best[best_idx][i];
        evc_mcpy(core->nnz_sub[i], pi->nnz_sub_best[best_idx][i], sizeof(int) * MAX_SUB_TB_NUM);
    }

    mi->pred_y_best = pi->pred[best_idx][0][0];

    /* save all cu inforamtion ********************/
    for(lidx = 0; lidx < REFP_NUM; lidx++)
    {
        mi->refi[lidx] = pi->refi[best_idx][lidx];
        mi->mvp_idx[lidx] = pi->mvp_idx[best_idx][lidx];
        {
            mi->mv[lidx][MV_X] = pi->mv[best_idx][lidx][MV_X];
            mi->mv[lidx][MV_Y] = pi->mv[best_idx][lidx][MV_Y];
        }

        mi->mvd[lidx][MV_X] = pi->mvd[best_idx][lidx][MV_X];
        mi->mvd[lidx][MV_Y] = pi->mvd[best_idx][lidx][MV_Y];
    }

    return cost_inter[best_idx];
}

static double pinter_analyze_cu(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, EVCE_MODE *mi, s16 coef[N_C][MAX_CU_DIM], pel *rec[N_C], int s_rec[N_C])
{
    EVCE_PINTER *pi;
    s8 *refi;
    s8 refi_temp = 0;
    u32 mecost, best_mecost;
    pel (*pred)[N_C][MAX_CU_DIM];
    s16 (*coef_t)[MAX_CU_DIM];
    s16 (*mvp)[MV_D], *mv, *mvd;
    int cuw, cuh, t0, t1, best_idx = PRED_SKIP, i, j;
    u8 mvp_idx[REFP_NUM] = {0, 0};
    s8 refi_cur = 0;
    double cost, cost_best = MAX_COST;
    double cost_inter[PRED_NUM];
    int lidx, pidx;
#if ATS_INTER_PROCESS
    int log2_tuw, log2_tuh;
#endif
#if DMVR_FLAG
    int best_dmvr = 0;
#endif
#if AFFINE
    int best_affine_mode = 0;
    u8 affine_applicable = 0;
    int allow_affine = ctx->sps.tool_affine;
    int mebits, best_bits = 0;
    int save_translation_mv[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
    u32 cost_trans[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME];
    s16 mv_trans[MAX_NUM_ACTIVE_REF_FRAME][REFP_NUM][MV_D];
    s16 tmp_mv_array[VER_NUM][MV_D];
    int memory_access;
    int allowed = 1;
    int mem = MAX_MEMORY_ACCESS_UNI * (1 << log2_cuw) * (1 << log2_cuh);
#endif
    int k;
    int REF_SET[3][MAX_NUM_ACTIVE_REF_FRAME] = {{0,0,},};
    int real_mv[MMVD_GRP_NUM * MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM][2][3];
    int num_amvr = MAX_NUM_MVR;

    if(ctx->sps.tool_amvr)
    {
        if(core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].visit)
        {
            num_amvr = core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].mvr_idx + 1;

            if(num_amvr > MAX_NUM_MVR)
            {
                num_amvr = MAX_NUM_MVR;
            }
        }
    }
    else
    {
        num_amvr = 1; /* only allow 1/4 pel of resolution */
    }

    pi = &ctx->pinter;
    cuw = (1 << log2_cuw);
    cuh = (1 << log2_cuh);

#if AFFINE
    core->affine_flag = 0;
    if(core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].visit)
    {
        if(core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].affine_flag == 0)
        {
            allow_affine = 0;
        }
    }

    // init translation mv for affine
    for(i = 0; i < REFP_NUM; i++)
    {
        for(j = 0; j < MAX_NUM_ACTIVE_REF_FRAME; j++)
        {
            save_translation_mv[i][j][MV_X] = 0;
            save_translation_mv[i][j][MV_Y] = 0;
        }
    }
#endif

    for(i = 0; i < PRED_NUM; i++)
    {
        cost_inter[i] = MAX_COST;
        pi->mvr_idx[i] = 0;
        pi->bi_idx[i] = BI_NON;
    }

#if AFFINE
    affine_applicable = 1;
#endif

    if(ctx->sps.tool_mmvd && ((pi->tile_group_type == TILE_GROUP_B) || (pi->tile_group_type == TILE_GROUP_P)))
    {
        for(k = 0; k < MAX_NUM_ACTIVE_REF_FRAME; k++)
        {
            REF_SET[0][k] = ctx->refp[k][0].ptr;
            REF_SET[1][k] = ctx->refp[k][1].ptr;
        }
        REF_SET[2][0] = ctx->ptr;

        evc_get_mmvd_mvp_list(ctx->map_refi, ctx->refp[0], ctx->map_mv, ctx->w_scu, ctx->h_scu, core->scup, core->avail_cu, log2_cuw, log2_cuh, ctx->tile_group_type, real_mv, ctx->map_scu, REF_SET, core->avail_lr
#if ADMVP
                              , core->history_buffer, ctx->sps.tool_admvp
#endif
        );
    }

    /* skip mode */
    cost = cost_inter[PRED_SKIP] = analyze_skip(ctx, core, x, y, log2_cuw, log2_cuh);

    if(cost < cost_best)
    {
#if DMVR_FLAG
        best_dmvr = core->dmvr_flag;
        core->dmvr_flag = 0;
#endif
        core->cu_mode = MODE_SKIP;
        best_idx = PRED_SKIP;
        cost_inter[best_idx] = cost_best = cost;
        SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
        evc_mset(pi->nnz_best[PRED_SKIP], 0, sizeof(int) * N_C);
        evc_mcpy(pi->nnz_sub_best[PRED_SKIP], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
    }

    if(pi->tile_group_type == TILE_GROUP_B)
    {
#if MERGE
        cost = cost_inter[PRED_DIR] = analyze_merge(ctx, core, x, y, log2_cuw, log2_cuh);
        if(cost < cost_best)
        {
            core->cu_mode = MODE_DIR;
            best_idx = PRED_DIR;
            cost_inter[best_idx] = cost_best = cost;
#if DMVR_FLAG
            best_dmvr = core->dmvr_flag;
            core->dmvr_flag = 0;
#endif

            for(i = 0; i < N_C; i++)
            {
                int size_tmp = (cuw * cuh) >> (i == 0 ? 0 : 2);
                evc_mcpy(pi->pred[best_idx][0][i], pi->pred[PRED_NUM][0][i], size_tmp * sizeof(pel));
                evc_mcpy(pi->coef[best_idx][i], pi->coef[PRED_NUM][i], size_tmp * sizeof(s16));
            }
            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best_merge);
        }
#else
        cost = cost_inter[PRED_DIR] = analyze_t_direct(ctx, core, x, y, log2_cuw, log2_cuh);
        if(cost < cost_best)
        {
            core->cu_mode = MODE_DIR;
            best_idx = PRED_DIR;
            cost_inter[best_idx] = cost_best = cost;

            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
        }
#endif
    }

    if(ctx->sps.tool_mmvd && ((pi->tile_group_type == TILE_GROUP_B) || (pi->tile_group_type == TILE_GROUP_P)))
    {
        /* MMVD mode for merge */
        cost = cost_inter[PRED_DIR_MMVD] = analyze_merge_mmvd(ctx, core, x, y, log2_cuw, log2_cuh, real_mv);
        if(cost < cost_best)
        {
            core->cu_mode = MODE_DIR_MMVD;
            best_idx = PRED_DIR_MMVD;
            cost_inter[best_idx] = cost_best = cost;
#if DMVR_FLAG
            best_dmvr = 0;
#endif
            cost_best = cost;
            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
        }

        /* MMVD mode for skip */
        cost = cost_inter[PRED_SKIP_MMVD] = analyze_skip_mmvd(ctx, core, x, y, log2_cuw, log2_cuh, real_mv);
        if(cost < cost_best)
        {
            core->cu_mode = MODE_SKIP_MMVD;
            best_idx = PRED_SKIP_MMVD;
#if DMVR_FLAG
            best_dmvr = 0;
#endif
            cost_inter[best_idx] = cost_best = cost;
            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
            evc_mset(pi->nnz_best[PRED_SKIP_MMVD], 0, sizeof(int) * N_C);
            evc_mcpy(pi->nnz_sub_best[PRED_SKIP_MMVD], core->nnz_sub, sizeof(int) * N_C * MAX_SUB_TB_NUM);
        }
    }

#if ET_AMVP
    if (best_idx == PRED_SKIP)
    {
        //skip_flag + pred_mode + mrg_flag + ref_list_idx + ref_idx + amvp_idx + mvd_bits + cbf0
        //1           1           1          2              1         1          6          1    = 14
        int penalty = 5;
        int bits_th_uni = 14 + penalty;
        int base_dist = 1 << (log2_cuw + log2_cuh + 2);
        if (cost_best < (bits_th_uni * ctx->lambda[0] + base_dist))
            mode_skip_curr[MSL_LIS0] = mode_skip_curr[MSL_LIS1] = 1;

        //skip_flag + pred_mode + mrg_flag + ref_list_idx + (ref_idx + amvp_idx + mvd_bits) * 2 + cbf0
        //1           1           1          1              (1         1          6) * 2          1    = 21
        int bits_th_bi = 21 + penalty;
        if (cost_best < (bits_th_bi * ctx->lambda[0] + base_dist))
            mode_skip_curr[MSL_BI] = 1;
    }
    else if (best_idx == PRED_DIR)
    {
        //skip_flag + pred_mode + mrg_flag + ref_list_idx + ref_idx + amvp_idx + mvd_bits + cbf1 + coeff
        //1           1           1          2              1         1          6          3      5     = 21
        int penalty = 5;
        int bits_th_uni = 21 + penalty;
        int base_dist = 1 << (log2_cuw + log2_cuh + 2);
        if (cost_best < (bits_th_uni * ctx->lambda[0] + base_dist))
            mode_skip_curr[MSL_LIS0] = mode_skip_curr[MSL_LIS1] = 1;

        //skip_flag + pred_mode + mrg_flag + ref_list_idx + (ref_idx + amvp_idx + mvd_bits) * 2 + cbf1 + coeff
        //1           1           1          1              (1         1          6) * 2          3      5    = 28
        int bits_th_bi = 28 + penalty;
        if (cost_best < (bits_th_bi * ctx->lambda[0] + base_dist))
            mode_skip_curr[MSL_BI] = 1;
    }
#endif

#if DMVR && AFFINE
    if(!(core->cu_mode == MODE_SKIP)
       || ctx->sps.tool_mmvd == 0
#if DMVR
       || ctx->sps.tool_dmvr == 0
#endif
#if AFFINE
       || ctx->sps.tool_affine == 0
#endif
       )
    {
#endif
        for(pi->curr_mvr = 0; pi->curr_mvr < num_amvr; pi->curr_mvr++)
        {
            const int mvr_offset = pi->curr_mvr * ORG_PRED_NUM;

            /* Motion Search *********************************************************/
            for(lidx = 0; lidx <= ((pi->tile_group_type == TILE_GROUP_P) ? PRED_L0 : PRED_L1); lidx++)
            {
                pidx = lidx + mvr_offset;
                pi->mvr_idx[pidx] = pi->curr_mvr;
                refi = pi->refi[pidx];
                mv = pi->mv[pidx][lidx];
                mvd = pi->mvd[pidx][lidx];
                pred = pi->pred[pidx];
                coef_t = pi->coef[pidx];

                pi->num_refp = ctx->rpm.num_refp[lidx];

                best_mecost = EVC_UINT32_MAX;

                for(refi_cur = 0; refi_cur < pi->num_refp; refi_cur++)
                {
                    mvp = pi->mvp_scale[lidx][refi_cur];

                    evc_get_motion_from_mvr(pi->curr_mvr, ctx->ptr, core->scup, lidx, refi_cur, pi->num_refp, ctx->map_mv, ctx->map_refi, pi->refp, core->cuw, core->cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mvp, pi->refi_pred[lidx], ctx->map_scu, core->avail_lr
#if DMVR_LAG
                                            , ctx->map_unrefined_mv
#endif
#if ADMVP
                                            , core->history_buffer
                                            , ctx->sps.tool_admvp
#endif
                    );
                    mvp_idx[lidx] = 0;

                    /* motion search ********************/
                    u8 skip_me = 0;
#if MODE_SAVE_LOAD_UPDATE
                    if(match_idx != -1)
                    {
                        if(history_data->ref_idx[match_idx][lidx] != refi_cur && history_data->ref_idx[match_idx][lidx] < 255)
                            skip_me = 1;
                    }
#endif
#if ET_ME_REFIDX1
                    int th_mvd = ctx->h >> 6;
                    if (refi_cur > 0 && best_mecost != EVC_UINT32_MAX && abs(pi->mvd[lidx][0][MV_X] + pi->mvd[lidx][0][MV_Y]) < th_mvd)
                        skip_me = 1;
#endif
                    if(skip_me)
                    {
                        mecost = EVC_UINT32_MAX;
                        mv[MV_X] = mvp[mvp_idx[lidx]][MV_X];
                        mv[MV_Y] = mvp[mvp_idx[lidx]][MV_Y];
                    }
                    else
                    {
                        mecost = pi->fn_me(pi, x, y, log2_cuw, log2_cuh, &refi_cur, lidx, mvp[mvp_idx[lidx]], mv, 0);
                    }

                    pi->mv_scale[lidx][refi_cur][MV_X] = mv[MV_X];
                    pi->mv_scale[lidx][refi_cur][MV_Y] = mv[MV_Y];
                    if(mecost < best_mecost)
                    {
                        best_mecost = mecost;
                        refi_temp = refi_cur;
                    }
#if AFFINE
                    {
                        if(pi->curr_mvr == 0)
                        {
                            save_translation_mv[lidx][refi_cur][MV_X] = mv[MV_X];
                            save_translation_mv[lidx][refi_cur][MV_Y] = mv[MV_Y];
                        }
                    }
#endif
                }
#if MODE_SAVE_LOAD_UPDATE
                if(history_data->num_visit_save < NUM_MODE_SL_PATH && match_idx == -1)
                    history_data->ref_idx[history_data->num_visit_save][lidx] = refi_temp;
#endif

                refi_cur = refi_temp;
                mv[MV_X] = pi->mv_scale[lidx][refi_cur][MV_X];
                mv[MV_Y] = pi->mv_scale[lidx][refi_cur][MV_Y];
                mvp = pi->mvp_scale[lidx][refi_cur];

                t0 = (lidx == 0) ? refi_cur : REFI_INVALID;
                t1 = (lidx == 1) ? refi_cur : REFI_INVALID;
                SET_REFI(refi, t0, t1);

                mv[MV_X] = (mv[MV_X] >> pi->curr_mvr) << pi->curr_mvr;
                mv[MV_Y] = (mv[MV_Y] >> pi->curr_mvr) << pi->curr_mvr;

                mvd[MV_X] = mv[MV_X] - mvp[mvp_idx[lidx]][MV_X];
                mvd[MV_Y] = mv[MV_Y] - mvp[mvp_idx[lidx]][MV_Y];

                pi->mvp_idx[pidx][lidx] = mvp_idx[lidx];

                cost = cost_inter[pidx] = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[PRED_NUM], pi->coef[PRED_NUM], pidx, mvp_idx
#if DMVR
                                                             , FALSE
#endif
                );

                if(cost < cost_best)
                {
                    core->cu_mode = MODE_INTER;
                    best_idx = pidx;
                    pi->mvr_idx[best_idx] = pi->curr_mvr;
                    pi->mvp_idx[best_idx][lidx] = mvp_idx[lidx];
                    cost_inter[best_idx] = cost_best = cost;
                    SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
#if DMVR_FLAG
                    best_dmvr = 0;
#endif

                    for(j = 0; j < N_C; j++)
                    {
                        int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                        pi->nnz_best[pidx][j] = core->nnz[j];
                        evc_mcpy(pi->nnz_sub_best[pidx][j], core->nnz_sub[j], sizeof(int) * MAX_SUB_TB_NUM);
                        evc_mcpy(pred[0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                        evc_mcpy(coef_t[j], pi->coef[PRED_NUM][j], size_tmp * sizeof(s16));
                    }
#if ATS_INTER_PROCESS
                    pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
                }
            }
#if ADMVP
            if(check_bi_applicability(pi->tile_group_type, cuw, cuh))
#else
            if(pi->tile_group_type == TILE_GROUP_B)
#endif
            {
                int max_num_bi = MAX_NUM_BI;
                int pred_mode = 0;

                if(core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].visit)
                {
                    max_num_bi = (core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].bi_idx == 2 || 
                                  core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].bi_idx == 3) ? MAX_NUM_BI : 1;
                }

                for(pi->curr_bi = 0; pi->curr_bi < max_num_bi; pi->curr_bi++)
                {
                    if(pi->curr_bi >= SKIP_BI_IDX && ((core->cu_mode == MODE_SKIP) || (core->cu_mode == MODE_SKIP_MMVD) || (core->cu_mode == MODE_DIR_MMVD)))
                    {
                        continue;
                    }
                    if(pi->curr_bi > 0 && cost_inter[PRED_BI] > (1.17) * cost_inter[PRED_L0] && cost_inter[PRED_BI] > (1.17) * cost_inter[PRED_L1])
                    {
                        continue;
                    }
                    pred_mode = (pi->curr_bi == 0) ? PRED_BI : ((pi->curr_bi == 1) ? PRED_FL0_BI : (pi->curr_bi == 2) ? PRED_FL1_BI : PRED_BI_REF);
                    pidx = pred_mode + mvr_offset;
                    cost = cost_inter[pidx] = analyze_bi(ctx, core, x, y, log2_cuw, log2_cuh, cost_inter);

                    if(cost < cost_best)
                    {
                        core->cu_mode = MODE_INTER;
                        best_idx = pidx;
                        pi->mvr_idx[best_idx] = pi->curr_mvr;
                        pi->bi_idx[best_idx] = BI_NORMAL + (pi->curr_bi % 3);
                        cost_inter[best_idx] = cost_best = cost;
#if DMVR_FLAG
                        best_dmvr = 0;
#endif
                        SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
                    }
                }
            }

            if(pi->curr_mvr >= SKIP_MVR_IDX && ((core->cu_mode == MODE_SKIP) || (core->cu_mode == MODE_SKIP_MMVD)))
            {
                break;
            }

            if(pi->curr_mvr >= FAST_MVR_IDX)
            {
                if(abs(pi->mvd[best_idx][REFP_0][MV_X]) <= 0 &&
                   abs(pi->mvd[best_idx][REFP_0][MV_Y]) <= 0 &&
                   abs(pi->mvd[best_idx][REFP_1][MV_X]) <= 0 &&
                   abs(pi->mvd[best_idx][REFP_1][MV_Y]) <= 0)
                {
                    break;
                }
            }

            if(abs(pi->mv[best_idx][REFP_0][MV_X]) > abs(pi->mv[best_idx][REFP_1][MV_X]))
            {
                pi->max_imv[MV_X] = (abs(pi->mv[best_idx][REFP_0][MV_X]) + 1) >> 2;
                if(pi->mv[best_idx][REFP_0][MV_X] < 0)
                {
                    pi->max_imv[MV_X] = -1 * pi->max_imv[MV_X];
                }
            }
            else
            {
                pi->max_imv[MV_X] = (abs(pi->mv[best_idx][REFP_1][MV_X]) + 1) >> 2;
                if(pi->mv[best_idx][REFP_1][MV_X] < 0)
                {
                    pi->max_imv[MV_X] = -1 * pi->max_imv[MV_X];
                }
            }

            if(abs(pi->mv[best_idx][REFP_0][MV_Y]) > abs(pi->mv[best_idx][REFP_1][MV_Y]))
            {
                pi->max_imv[MV_Y] = (abs(pi->mv[best_idx][REFP_0][MV_Y]) + 1) >> 2;
                if(pi->mv[best_idx][REFP_0][MV_Y] < 0)
                {
                    pi->max_imv[MV_Y] = -1 * pi->max_imv[MV_Y];
                }
            }
            else
            {
                pi->max_imv[MV_Y] = (abs(pi->mv[best_idx][REFP_1][MV_Y]) + 1) >> 2;
                if(pi->mv[best_idx][REFP_1][MV_Y] < 0)
                {
                    pi->max_imv[MV_Y] = -1 * pi->max_imv[MV_Y];
                }
            }
        }
#if DMVR && AFFINE
    }
#endif

#if AFFINE
#if HLS_M47668
    if(ctx->tile_group_depth < 4)
#else
    if (ctx->layer_id < 4)
#endif
    {
        if(allow_affine && cuw >= 8 && cuh >= 8)
        {
            s16(*affine_mvp)[VER_NUM][MV_D], (*affine_mv)[MV_D], (*affine_mvd)[MV_D];
            int vertex = 0;
            int vertex_num;

            /* AFFINE skip mode */
            core->mmvd_flag = 0;
            cost = cost_inter[AFF_SKIP] = analyze_affine_merge(ctx, core, x, y, log2_cuw, log2_cuh, AFF_SKIP);

            if(cost < cost_best)
            {
                best_affine_mode = core->affine_flag;
                core->cu_mode = MODE_SKIP;
                best_idx = AFF_SKIP;
#if DMVR_FLAG
                best_dmvr = 0;
#endif
                cost_inter[best_idx] = cost_best = cost;
                SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
                evc_mset(pi->nnz_best[AFF_SKIP], 0, sizeof(int) * N_C);
                evc_mset(pi->nnz_sub_best[AFF_SKIP], 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);
            }

            /* AFFINE direct mode */
            cost = cost_inter[AFF_DIR] = analyze_affine_merge(ctx, core, x, y, log2_cuw, log2_cuh, AFF_DIR);

            if(cost < cost_best)
            {
                best_affine_mode = core->affine_flag;
#if DMVR_FLAG
                best_dmvr = 0;
#endif
                core->cu_mode = MODE_DIR;
                best_idx = AFF_DIR;
                cost_inter[best_idx] = cost_best = cost;
                SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
            }

            if(affine_applicable && cuw >= 16 && cuh >= 16)
            {
#if AFFINE 
                if(!(core->cu_mode == MODE_SKIP) && !(core->cu_mode == MODE_SKIP_MMVD)) //fast skip affine
                {
#endif
                    /* AFFINE 4 paramters Motion Search *********************************************************/
                    core->affine_flag = 1;
                    vertex_num = 2;
                    for(lidx = 0; lidx <= ((pi->tile_group_type == TILE_GROUP_P) ? PRED_L0 : PRED_L1); lidx++)
                    {
                        pidx = lidx + AFF_L0;
                        refi = pi->refi[pidx];
                        affine_mv = pi->affine_mv[pidx][lidx];
                        affine_mvd = pi->affine_mvd[pidx][lidx];

                        pred = pi->pred[pidx];
                        coef_t = pi->coef[pidx];
                        pi->num_refp = ctx->rpm.num_refp[lidx];

                        best_mecost = EVC_UINT32_MAX;

                        for(refi_cur = 0; refi_cur < pi->num_refp; refi_cur++)
                        {
                            affine_mvp = pi->affine_mvp_scale[lidx][refi_cur];

                            evc_get_affine_motion_scaling(ctx->ptr, core->scup, lidx, refi_cur, pi->num_refp, ctx->map_mv, ctx->map_refi, pi->refp,
                                                          core->cuw, core->cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, affine_mvp, pi->refi_pred[lidx],
                                                          ctx->map_scu, ctx->map_affine, vertex_num, core->avail_lr
#if DMVR_LAG
                                                          , ctx->map_unrefined_mv
#endif
                            );
                            {
                                u32 mvp_best = EVC_UINT32_MAX;
                                u32 mvp_temp = EVC_UINT32_MAX;
                                s8  refi_t[REFP_NUM];

                                EVC_PIC* refp = pi->refp[refi_cur][lidx].pic;
                                pel *pred = pi->pred_buf;
                                pel *org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
                                pel s_org = pi->s_o[Y_C];

                                for(i = 0; i < AFF_MAX_NUM_MVP; i++)
                                {
                                    evc_affine_mc_l(x, y, refp->w_l, refp->h_l, cuw, cuh, affine_mvp[i], refp, pred, vertex_num
#if EIF
                                                    , core->eif_tmp_buffer
#endif
                                    );
                                    mvp_temp = evce_satd_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw);
                                    mebits = 1; // zero mvd flag
                                    mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][i]; // mvp idx
                                    mvp_temp += MV_COST(pi, mebits);

                                    if(mvp_temp < mvp_best)
                                    {
                                        mvp_idx[lidx] = i;
                                        mvp_best = mvp_temp;
                                    }
                                }

                                mv_trans[refi_cur][lidx][MV_X] = save_translation_mv[lidx][refi_cur][MV_X];
                                mv_trans[refi_cur][lidx][MV_Y] = save_translation_mv[lidx][refi_cur][MV_Y];

                                refi_t[lidx] = refi_cur;
                                refi_t[1 - lidx] = -1;
                                mv_clip(x, y, ctx->w, ctx->h, cuw, cuh, refi_t, mv_trans[refi_cur], mv_trans[refi_cur]);

                                for(vertex = 0; vertex < vertex_num; vertex++)
                                {
                                    tmp_mv_array[vertex][MV_X] = mv_trans[refi_cur][lidx][MV_X];
                                    tmp_mv_array[vertex][MV_Y] = mv_trans[refi_cur][lidx][MV_Y];
                                }

                                evc_affine_mc_l(x, y, refp->w_l, refp->h_l, cuw, cuh, tmp_mv_array, refp, pred, vertex_num
#if EIF
                                                , core->eif_tmp_buffer
#endif
                                );
                                cost_trans[lidx][refi_cur] = evce_satd_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw);
                                mebits = get_affine_mv_bits(tmp_mv_array, affine_mvp[mvp_idx[lidx]], pi->num_refp, refi_cur, vertex_num);
                                mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][mvp_idx[lidx]];
                                mvp_temp = cost_trans[lidx][refi_cur] + MV_COST(pi, mebits);

                                if(mvp_temp < mvp_best)
                                {
                                    for(vertex = 0; vertex < vertex_num; vertex++)
                                    {
                                        affine_mv[vertex][MV_X] = mv_trans[refi_cur][lidx][MV_X];
                                        affine_mv[vertex][MV_Y] = mv_trans[refi_cur][lidx][MV_Y];
                                    }
                                }
                                else
                                {
                                    for(vertex = 0; vertex < vertex_num; vertex++)
                                    {
                                        affine_mv[vertex][MV_X] = affine_mvp[mvp_idx[lidx]][vertex][MV_X];
                                        affine_mv[vertex][MV_Y] = affine_mvp[mvp_idx[lidx]][vertex][MV_Y];
                                    }
                                }
                            }

                            /* affine motion search */
                            mecost = pi->fn_affine_me(pi, x, y, log2_cuw, log2_cuh, &refi_cur, lidx, affine_mvp[mvp_idx[lidx]], affine_mv, 0, vertex_num
#if EIF
                                                      , core->eif_tmp_buffer
#endif
                            );

                            // update MVP bits
                            t0 = (lidx == 0) ? refi_cur : REFI_INVALID;
                            t1 = (lidx == 1) ? refi_cur : REFI_INVALID;
                            SET_REFI(refi, t0, t1);
                            check_best_affine_mvp(ctx, core, pi->tile_group_type, refi, lidx, pidx, affine_mvp, affine_mv, affine_mvd, &mvp_idx[lidx], vertex_num);

                            mebits = get_affine_mv_bits(affine_mv, affine_mvp[mvp_idx[lidx]], pi->num_refp, refi_cur, vertex_num);
                            mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][mvp_idx[lidx]];
                            mecost += MV_COST(pi, mebits);

                            pi->mvp_idx_scale[lidx][refi_cur] = mvp_idx[lidx];

                            /* save affine per ref me results */
                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                pi->affine_mv_scale[lidx][refi_cur][vertex][MV_X] = affine_mv[vertex][MV_X];
                                pi->affine_mv_scale[lidx][refi_cur][vertex][MV_Y] = affine_mv[vertex][MV_Y];
                            }
                            if(mecost < best_mecost)
                            {
                                best_mecost = mecost;
                                best_bits = mebits;
                                refi_temp = refi_cur;
                            }
                        }

                        /* save affine per list me results */
                        refi_cur = refi_temp;
                        for(vertex = 0; vertex < vertex_num; vertex++)
                        {
                            affine_mv[vertex][MV_X] = pi->affine_mv_scale[lidx][refi_cur][vertex][MV_X];
                            affine_mv[vertex][MV_Y] = pi->affine_mv_scale[lidx][refi_cur][vertex][MV_Y];
                        }

                        affine_mvp = pi->affine_mvp_scale[lidx][refi_cur];
                        t0 = (lidx == 0) ? refi_cur : REFI_INVALID;
                        t1 = (lidx == 1) ? refi_cur : REFI_INVALID;
                        SET_REFI(refi, t0, t1);

                        /* get affine mvd */
                        mvp_idx[lidx] = pi->mvp_idx_scale[lidx][refi_cur];
                        for(vertex = 0; vertex < vertex_num; vertex++)
                        {
                            affine_mvd[vertex][MV_X] = affine_mv[vertex][MV_X] - affine_mvp[mvp_idx[lidx]][vertex][MV_X];
                            affine_mvd[vertex][MV_Y] = affine_mv[vertex][MV_Y] - affine_mvp[mvp_idx[lidx]][vertex][MV_Y];
                            if (vertex)
                            {
                                affine_mvd[vertex][MV_X] -= affine_mvd[0][MV_X];
                                affine_mvd[vertex][MV_Y] -= affine_mvd[0][MV_Y];
                            }
                        }
                        pi->mot_bits[lidx] = best_bits;
                        pi->mvp_idx[pidx][lidx] = mvp_idx[lidx];

                        affine_mv[2][MV_X] = affine_mv[0][MV_X] - (affine_mv[1][MV_Y] - affine_mv[0][MV_Y]) * cuh / cuw;
                        affine_mv[2][MV_Y] = affine_mv[0][MV_Y] + (affine_mv[1][MV_X] - affine_mv[0][MV_X]) * cuh / cuw;
                        affine_mv[3][MV_X] = affine_mv[1][MV_X] - (affine_mv[1][MV_Y] - affine_mv[0][MV_Y]) * cuh / cuw;
                        affine_mv[3][MV_Y] = affine_mv[1][MV_Y] + (affine_mv[1][MV_X] - affine_mv[0][MV_X]) * cuh / cuw;
                        allowed = 1;
                        memory_access = evc_get_affine_memory_access(affine_mv, cuw, cuh);
                        if(memory_access > mem)
                        {
                            allowed = 0;
                        }

                        if(allowed)
                        {
                            cost = cost_inter[pidx] = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[PRED_NUM], pi->coef[PRED_NUM], pidx, mvp_idx
#if DMVR
                                                                         , FALSE
#endif
                            );


                            if(cost < cost_best)
                            {
                                best_affine_mode = core->affine_flag;
#if DMVR_FLAG
                                best_dmvr = 0;
#endif
                                core->cu_mode = MODE_INTER;
                                best_idx = pidx;
                                pi->mvp_idx[best_idx][lidx] = mvp_idx[lidx];
                                cost_inter[best_idx] = cost_best = cost;
                                pi->bi_idx[best_idx] = BI_NON;

                                SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);

                                for(j = 0; j < N_C; j++)
                                {
                                    int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                                    pi->nnz_best[pidx][j] = core->nnz[j];
                                    evc_mcpy(pi->nnz_sub_best[pidx][j], core->nnz_sub[j], sizeof(int) * MAX_SUB_TB_NUM);
                                    evc_mcpy(pred[0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                                    evc_mcpy(coef_t[j], pi->coef[PRED_NUM][j], size_tmp * sizeof(s16));
                                }
#if ATS_INTER_PROCESS
                                pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
                            }
                        }
                    }

                    if(pi->tile_group_type == TILE_GROUP_B)
                    {
                        pidx = AFF_BI;
                        cost = cost_inter[pidx] = analyze_affine_bi(ctx, core, pi, x, y, log2_cuw, log2_cuh, cost_inter, AFF_BI, vertex_num);

                        if(cost < cost_best)
                        {
                            best_affine_mode = core->affine_flag;
#if DMVR_FLAG
                            best_dmvr = 0;
#endif
                            core->cu_mode = MODE_INTER;
                            best_idx = pidx;
                            cost_inter[best_idx] = cost_best = cost;
                            pi->bi_idx[best_idx] = BI_NORMAL;

                            SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
                        }
                    }

                    if((best_idx >= AFF_L0) && (best_idx <= AFF_6_BI))
                    {
                        /* AFFINE 6 paramters Motion Search *********************************************************/
                        core->affine_flag = 2;
                        vertex_num = 3;
                        for(lidx = 0; lidx <= ((pi->tile_group_type == TILE_GROUP_P) ? PRED_L0 : PRED_L1); lidx++)
                        {
                            pidx = lidx + AFF_6_L0;
                            refi = pi->refi[pidx];
                            affine_mv = pi->affine_mv[pidx][lidx];
                            affine_mvd = pi->affine_mvd[pidx][lidx];

                            pred = pi->pred[pidx];
                            coef_t = pi->coef[pidx];
                            pi->num_refp = ctx->rpm.num_refp[lidx];

                            best_mecost = EVC_UINT32_MAX;

                            for(refi_cur = 0; refi_cur < pi->num_refp; refi_cur++)
                            {
                                affine_mvp = pi->affine_mvp_scale[lidx][refi_cur];

                                evc_get_affine_motion_scaling(ctx->ptr, core->scup, lidx, refi_cur, pi->num_refp, ctx->map_mv, ctx->map_refi, pi->refp,
                                                              core->cuw, core->cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, affine_mvp, pi->refi_pred[lidx],
                                                              ctx->map_scu, ctx->map_affine, vertex_num, core->avail_lr
#if DMVR_LAG
                                                              , ctx->map_unrefined_mv
#endif
                                );

                                {
                                    u32 mvp_best = EVC_UINT32_MAX;
                                    u32 mvp_temp = EVC_UINT32_MAX;

                                    EVC_PIC* refp = pi->refp[refi_cur][lidx].pic;
                                    pel *pred = pi->pred_buf;
                                    pel *org = pi->o[Y_C] + x + y * pi->s_o[Y_C];
                                    pel s_org = pi->s_o[Y_C];
                                    for(i = 0; i < AFF_MAX_NUM_MVP; i++)
                                    {
                                        evc_affine_mc_l(x, y, refp->w_l, refp->h_l, cuw, cuh, affine_mvp[i], refp, pred, vertex_num
#if EIF
                                                        , core->eif_tmp_buffer
#endif
                                        );

                                        mvp_temp = evce_satd_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw);
                                        mebits = 1; // zero mvd flag
                                        mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][i]; // mvp idx
                                        mvp_temp += MV_COST(pi, mebits);

                                        if(mvp_temp < mvp_best)
                                        {
                                            mvp_idx[lidx] = i;
                                            mvp_best = mvp_temp;
                                        }
                                    }

                                    affine_mv[0][MV_X] = pi->affine_mv_scale[lidx][refi_cur][0][MV_X];
                                    affine_mv[0][MV_Y] = pi->affine_mv_scale[lidx][refi_cur][0][MV_Y];
                                    affine_mv[1][MV_X] = pi->affine_mv_scale[lidx][refi_cur][1][MV_X];
                                    affine_mv[1][MV_Y] = pi->affine_mv_scale[lidx][refi_cur][1][MV_Y];
                                    affine_mv[2][MV_X] = affine_mv[0][MV_X] - (affine_mv[1][MV_Y] - affine_mv[0][MV_Y]) * cuh / cuw;
                                    affine_mv[2][MV_Y] = affine_mv[0][MV_Y] + (affine_mv[1][MV_X] - affine_mv[0][MV_X]) * cuh / cuw;
                                    evc_affine_mc_l(x, y, refp->w_l, refp->h_l, cuw, cuh, affine_mv, refp, pred, vertex_num
#if EIF
                                                    , core->eif_tmp_buffer
#endif
                                    );

                                    mvp_temp = evce_satd_16b(log2_cuw, log2_cuh, org, pred, s_org, cuw);

                                    // 4 parameter AFFINE MV
                                    mebits = get_affine_mv_bits(affine_mv, affine_mvp[mvp_idx[lidx]], pi->num_refp, refi_cur, vertex_num);
                                    mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][mvp_idx[lidx]]; // mvp idx
                                    mvp_temp += MV_COST(pi, mebits);
                                    // translation MV
                                    for(vertex = 0; vertex < vertex_num; vertex++)
                                    {
                                        tmp_mv_array[vertex][MV_X] = mv_trans[refi_cur][lidx][MV_X];
                                        tmp_mv_array[vertex][MV_Y] = mv_trans[refi_cur][lidx][MV_Y];
                                    }
                                    mebits = get_affine_mv_bits(tmp_mv_array, affine_mvp[mvp_idx[lidx]], pi->num_refp, refi_cur, vertex_num);
                                    mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][mvp_idx[lidx]];
                                    cost_trans[lidx][refi_cur] += MV_COST(pi, mebits);

                                    if(mvp_best <= mvp_temp && mvp_best <= cost_trans[lidx][refi_cur])
                                    {
                                        for(vertex = 0; vertex < vertex_num; vertex++)
                                        {
                                            affine_mv[vertex][MV_X] = affine_mvp[mvp_idx[lidx]][vertex][MV_X];
                                            affine_mv[vertex][MV_Y] = affine_mvp[mvp_idx[lidx]][vertex][MV_Y];
                                        }
                                    }
                                    else if(mvp_best <= mvp_temp && cost_trans[lidx][refi_cur] < mvp_best)
                                    {
                                        for(vertex = 0; vertex < vertex_num; vertex++)
                                        {
                                            affine_mv[vertex][MV_X] = mv_trans[refi_cur][lidx][MV_X];
                                            affine_mv[vertex][MV_Y] = mv_trans[refi_cur][lidx][MV_Y];
                                        }
                                    }
                                }

                                /* affine motion search */
                                mecost = pi->fn_affine_me(pi, x, y, log2_cuw, log2_cuh, &refi_cur, lidx, affine_mvp[mvp_idx[lidx]], affine_mv, 0, vertex_num
#if EIF
                                                          , core->eif_tmp_buffer
#endif
                                );

                                // update ME bits
                                t0 = (lidx == 0) ? refi_cur : REFI_INVALID;
                                t1 = (lidx == 1) ? refi_cur : REFI_INVALID;
                                SET_REFI(refi, t0, t1);
                                check_best_affine_mvp(ctx, core, pi->tile_group_type, refi, lidx, pidx, affine_mvp, affine_mv, affine_mvd, &mvp_idx[lidx], vertex_num);
                                mebits = get_affine_mv_bits(affine_mv, affine_mvp[mvp_idx[lidx]], pi->num_refp, refi_cur, vertex_num);
                                mebits += evce_tbl_mvp_idx_bits[AFF_MAX_NUM_MVP][mvp_idx[lidx]];
                                mecost += MV_COST(pi, mebits);

                                pi->mvp_idx_scale[lidx][refi_cur] = mvp_idx[lidx];

                                /* save affine per ref me results */
                                for(vertex = 0; vertex < vertex_num; vertex++)
                                {
                                    pi->affine_mv_scale[lidx][refi_cur][vertex][MV_X] = affine_mv[vertex][MV_X];
                                    pi->affine_mv_scale[lidx][refi_cur][vertex][MV_Y] = affine_mv[vertex][MV_Y];
                                }
                                if(mecost < best_mecost)
                                {
                                    best_mecost = mecost;
                                    best_bits = mebits;
                                    refi_temp = refi_cur;
                                }
                            }

                            /* save affine per list me results */
                            refi_cur = refi_temp;
                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                affine_mv[vertex][MV_X] = pi->affine_mv_scale[lidx][refi_cur][vertex][MV_X];
                                affine_mv[vertex][MV_Y] = pi->affine_mv_scale[lidx][refi_cur][vertex][MV_Y];
                            }

                            affine_mvp = pi->affine_mvp_scale[lidx][refi_cur];
                            t0 = (lidx == 0) ? refi_cur : REFI_INVALID;
                            t1 = (lidx == 1) ? refi_cur : REFI_INVALID;
                            SET_REFI(refi, t0, t1);

                            /* get affine mvd */
                            mvp_idx[lidx] = pi->mvp_idx_scale[lidx][refi_cur];
                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                affine_mvd[vertex][MV_X] = affine_mv[vertex][MV_X] - affine_mvp[mvp_idx[lidx]][vertex][MV_X];
                                affine_mvd[vertex][MV_Y] = affine_mv[vertex][MV_Y] - affine_mvp[mvp_idx[lidx]][vertex][MV_Y];
                                if (vertex)
                                {
                                    affine_mvd[vertex][MV_X] -= affine_mvd[0][MV_X];
                                    affine_mvd[vertex][MV_Y] -= affine_mvd[0][MV_Y];
                                }
                            }
                            pi->mot_bits[lidx] = best_bits;
                            pi->mvp_idx[pidx][lidx] = mvp_idx[lidx];

                            affine_mv[3][MV_X] = affine_mv[1][MV_X] + affine_mv[2][MV_X] - affine_mv[0][MV_X];
                            affine_mv[3][MV_Y] = affine_mv[1][MV_Y] + affine_mv[2][MV_Y] - affine_mv[0][MV_Y];
                            allowed = 1;
                            memory_access = evc_get_affine_memory_access(affine_mv, cuw, cuh);
                            if(memory_access > mem)
                            {
                                allowed = 0;
                            }

                            if(allowed)
                            {
                                cost = cost_inter[pidx] = pinter_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred[PRED_NUM], pi->coef[PRED_NUM], pidx, mvp_idx
#if DMVR
                                                                             , FALSE
#endif
                                );

                                if(cost < cost_best)
                                {
                                    best_affine_mode = core->affine_flag;
#if DMVR_FLAG
                                    best_dmvr = 0;
#endif
                                    core->cu_mode = MODE_INTER;
                                    best_idx = pidx;
                                    pi->mvp_idx[best_idx][lidx] = mvp_idx[lidx];
                                    cost_inter[best_idx] = cost_best = cost;
                                    pi->bi_idx[best_idx] = BI_NON;

                                    SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);

                                    for(j = 0; j < N_C; j++)
                                    {
                                        int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                                        pi->nnz_best[pidx][j] = core->nnz[j];
                                        evc_mcpy(pi->nnz_sub_best[pidx][j], core->nnz_sub[j], sizeof(int) * MAX_SUB_TB_NUM);
                                        evc_mcpy(pred[0][j], pi->pred[PRED_NUM][0][j], size_tmp * sizeof(pel));
                                        evc_mcpy(coef_t[j], pi->coef[PRED_NUM][j], size_tmp * sizeof(s16));
                                    }
#if ATS_INTER_PROCESS
                                    pi->ats_inter_info_mode[pidx] = core->ats_inter_info;
#endif
                                }
                            }
                        }

                        if(pi->tile_group_type == TILE_GROUP_B)
                        {
                            pidx = AFF_6_BI;
                            cost = cost_inter[pidx] = analyze_affine_bi(ctx, core, pi, x, y, log2_cuw, log2_cuh, cost_inter, AFF_6_BI, vertex_num);
                            if(cost < cost_best)
                            {
                                best_affine_mode = core->affine_flag;
#if DMVR_FLAG
                                best_dmvr = 0;
#endif
                                core->cu_mode = MODE_INTER;
                                best_idx = pidx;
                                cost_inter[best_idx] = cost_best = cost;
                                pi->bi_idx[best_idx] = BI_NORMAL;

                                SBAC_STORE(core->s_next_best[log2_cuw - 2][log2_cuh - 2], core->s_temp_best);
                            }
                        }
                    }
                }
            }
        }
    }
#endif

    /* reconstruct */
    for(j = 0; j < N_C; j++)
    {
        int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
        evc_mcpy(coef[j], pi->coef[best_idx][j], sizeof(s16) * size_tmp);
        evc_mcpy(pi->residue[j], pi->coef[best_idx][j], sizeof(s16) * size_tmp);
    }
#if ATS_INTER_PROCESS
    core->ats_inter_info = pi->ats_inter_info_mode[best_idx];
    get_tu_size(get_ats_inter_idx(core->ats_inter_info), log2_cuw, log2_cuh, &log2_tuw, &log2_tuh);
#if ATS_INTER_DEBUG
    assert(core->cost_best == cost_inter[best_idx]);
    if (pi->nnz_best[best_idx][Y_C] && log2_cuw <= MAX_TR_LOG2 && log2_cuh <= MAX_TR_LOG2 )
    {
        int sum_y_coef = 0;
        for (int a = 0; a < (1 << (log2_tuw + log2_tuh)); a++)
        {
            sum_y_coef += coef[Y_C][a] != 0;
        }
        assert(sum_y_coef == pi->nnz_best[best_idx][Y_C]);
    }
#endif
#endif

    evc_sub_block_itdq(pi->residue, log2_cuw, log2_cuh, pi->qp_y, pi->qp_u, pi->qp_v, pi->nnz_best[best_idx], pi->nnz_sub_best[best_idx], ctx->sps.tool_iqt
#if ATS_INTRA_PROCESS
                       , 0, 0
#endif
#if ATS_INTER_PROCESS
                       , core->ats_inter_info
#endif
    );

    for(i = 0; i < N_C; i++)
    {
        rec[i] = pi->rec[best_idx][i];
        s_rec[i] = (i == 0 ? cuw : cuw >> 1);
        evc_recon(pi->residue[i], pi->pred[best_idx][0][i], pi->nnz_best[best_idx][i], s_rec[i], (i == 0 ? cuh : cuh >> 1), s_rec[i], rec[i]
#if ATS_INTER_PROCESS
                  , core->ats_inter_info
#endif
        );
#if HTDF
        if (ctx->sps.tool_htdf == 1 && i == Y_C && pi->nnz_best[best_idx][i])
        {
            const int s_mod = pi->s_m[Y_C];
#if HW_HTDF_CLEANUP
            u16 avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, log2_cuw, log2_cuh, ctx->map_scu);
            evc_htdf(rec[i], ctx->tgh.qp, cuw, cuh, cuw, FALSE, pi->m[Y_C] + (y * s_mod) + x, s_mod, avail_cu);
#else
            evc_get_nbr(x, y, cuw, cuh, pi->m[Y_C] + (y * s_mod) + x, s_mod, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, Y_C);
            evc_htdf(rec[i], ctx->tgh.qp, cuw, cuh, cuw, FALSE, core->nb[Y_C][0] + 2, core->nb[Y_C][1] + cuh - 1, core->nb[Y_C][2] + 2, core->avail_cu);
#endif
        }
#endif
        core->nnz[i] = pi->nnz_best[best_idx][i];
        evc_mcpy(core->nnz_sub[i], pi->nnz_sub_best[best_idx][i], sizeof(int) * MAX_SUB_TB_NUM);
    }

    mi->pred_y_best = pi->pred[best_idx][0][0];

    /* save all cu inforamtion ********************/
#if AFFINE // save affine information
    if(best_idx >= AFF_L0 && best_idx <= AFF_6_BI)
    {
        int vertex;
        int vertex_num;

        core->affine_flag = best_affine_mode;
        vertex_num = core->affine_flag + 1;
        for(lidx = 0; lidx < REFP_NUM; lidx++)
        {
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                mi->affine_mv[lidx][vertex][MV_X] = pi->affine_mv[best_idx][lidx][vertex][MV_X];
                mi->affine_mv[lidx][vertex][MV_Y] = pi->affine_mv[best_idx][lidx][vertex][MV_Y];
                mi->affine_mvd[lidx][vertex][MV_X] = pi->affine_mvd[best_idx][lidx][vertex][MV_X];
                mi->affine_mvd[lidx][vertex][MV_Y] = pi->affine_mvd[best_idx][lidx][vertex][MV_Y];
            }
        }
    }
    else
    {
        core->affine_flag = 0;
    }
#endif
#if DMVR_FLAG
    core->dmvr_flag = best_dmvr;
#endif
    for(lidx = 0; lidx < REFP_NUM; lidx++)
    {
        mi->refi[lidx] = pi->refi[best_idx][lidx];
        mi->mvp_idx[lidx] = pi->mvp_idx[best_idx][lidx];
#if DMVR_LAG
        if(core->dmvr_flag)
        {
            assert(core->cu_mode == MODE_SKIP || core->cu_mode == MODE_DIR);
            u16 idx = 0, i, j;
            for(j = 0; j < core->cuh >> MIN_CU_LOG2; j++)
            {
                for(i = 0; i < core->cuw >> MIN_CU_LOG2; i++)
                {
                    mi->dmvr_mv[idx + i][lidx][MV_X] = pi->dmvr_mv[best_idx][idx + i][lidx][MV_X];
                    mi->dmvr_mv[idx + i][lidx][MV_Y] = pi->dmvr_mv[best_idx][idx + i][lidx][MV_Y];
                }
                idx += core->cuw >> MIN_CU_LOG2;
            }
        }
#endif
        {
            mi->mv[lidx][MV_X] = pi->mv[best_idx][lidx][MV_X];
            mi->mv[lidx][MV_Y] = pi->mv[best_idx][lidx][MV_Y];
        }

        mi->mvd[lidx][MV_X] = pi->mvd[best_idx][lidx][MV_X];
        mi->mvd[lidx][MV_Y] = pi->mvd[best_idx][lidx][MV_Y];
    }

    mi->mmvd_idx = pi->mmvd_idx[best_idx];
    mi->mvr_idx = pi->mvr_idx[best_idx];
    mi->bi_idx = pi->bi_idx[best_idx];

    if(!core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].visit)
    {
        core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].mvr_idx = mi->mvr_idx;
        core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].bi_idx = mi->bi_idx;
    }

#if AFFINE
    if(!core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].visit)
    {
        core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].affine_flag = best_affine_mode;
    }
#endif

    return cost_inter[best_idx];
}

static int pinter_set_complexity(EVCE_CTX *ctx, int complexity)
{
    EVCE_PINTER *pi;

    pi = &ctx->pinter;

    /* default values *************************************************/
    pi->max_search_range = ctx->param.max_b_frames == 0 ? SEARCH_RANGE_IPEL_LD : SEARCH_RANGE_IPEL_RA;
    pi->search_range_ipel[MV_X] = pi->max_search_range;
    pi->search_range_ipel[MV_Y] = pi->max_search_range;

    pi->search_range_spel[MV_X] = SEARCH_RANGE_SPEL;
    pi->search_range_spel[MV_Y] = SEARCH_RANGE_SPEL;

    pi->search_pattern_hpel = tbl_search_pattern_hpel_partial;
    pi->search_pattern_hpel_cnt = 8;

    pi->search_pattern_qpel = tbl_search_pattern_qpel_8point;
    pi->search_pattern_qpel_cnt = 8;

    if(ctx->cdsc.tool_amis == 0)
    {
        ctx->fn_pinter_analyze_cu = pinter_analyze_cu_baseline;
    }
    else
    {
        ctx->fn_pinter_analyze_cu = pinter_analyze_cu;
    }

    pi->me_level = ME_LEV_QPEL;

    pi->fn_me = pinter_me_epzs;
#if AFFINE
    pi->fn_affine_me = pinter_affine_me_gradient;
#endif

    pi->complexity = complexity;

    pi->sps_amvr_flag = ctx->cdsc.tool_amvr;

    return EVC_OK;
}

int evce_pinter_create(EVCE_CTX *ctx, int complexity)
{
    /* set function addresses */
    ctx->fn_pinter_init_frame = pinter_init_frame;
    ctx->fn_pinter_init_lcu = pinter_init_lcu;
    ctx->fn_pinter_set_complexity = pinter_set_complexity;

    /* set maximum/minimum value of search range */
    ctx->pinter.min_clip[MV_X] = -MAX_CU_SIZE + 1;
    ctx->pinter.min_clip[MV_Y] = -MAX_CU_SIZE + 1;
    ctx->pinter.max_clip[MV_X] = ctx->param.w - 1;
    ctx->pinter.max_clip[MV_Y] = ctx->param.h - 1;

    return ctx->fn_pinter_set_complexity(ctx, complexity);
}
