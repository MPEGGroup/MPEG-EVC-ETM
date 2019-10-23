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

#include "evc_def.h"
#if GRAB_STAT
#include "evc_debug.h"
#endif


#define TX_SHIFT1(log2_size)   ((log2_size) - 1 + BIT_DEPTH - 8)
#define TX_SHIFT2(log2_size)   ((log2_size) + 6)

#if ENC_DEC_TRACE
FILE *fp_trace;
#if TRACE_RDO
#if TRACE_RDO_EXCLUDE_I
int fp_trace_print = 0;
#else
int fp_trace_print = 1;
#endif
#else
int fp_trace_print = 0;
#endif
int fp_trace_counter = 0;
#endif
#if TRACE_START_POC
int fp_trace_started = 0;
#endif

int evc_atomic_inc(volatile int *pcnt)
{
    int ret;
    ret = *pcnt;
    ret++;
    *pcnt = ret;
    return ret;
}

int evc_atomic_dec(volatile int *pcnt)
{
    int ret;
    ret = *pcnt;
    ret--;
    *pcnt = ret;
    return ret;
}

BOOL is_ptr_aligned(void* ptr, int num_bytes)
{
  int mask = num_bytes - 1;

  return ((uintptr_t)ptr & mask) == 0;
}

EVC_PIC * evc_picbuf_alloc(int w, int h, int pad_l, int pad_c, int *err)
{
    EVC_PIC *pic = NULL;
    EVC_IMGB *imgb = NULL;
    int ret, opt, align[EVC_IMGB_MAX_PLANE], pad[EVC_IMGB_MAX_PLANE];
    int w_scu, h_scu, f_scu, size;

    /* allocate PIC structure */
    pic = evc_malloc(sizeof(EVC_PIC));
    evc_assert_gv(pic != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset(pic, 0, sizeof(EVC_PIC));

    opt = EVC_IMGB_OPT_NONE;

    /* set align value*/
    align[0] = MIN_CU_SIZE;
    align[1] = MIN_CU_SIZE >> 1;
    align[2] = MIN_CU_SIZE >> 1;

    /* set padding value*/
    pad[0] = pad_l;
    pad[1] = pad_c;
    pad[2] = pad_c;

    imgb = evc_imgb_create(w, h, EVC_COLORSPACE_YUV420_10LE, opt, pad, align);
    evc_assert_gv(imgb != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

    /* set EVC_PIC */
    pic->buf_y = imgb->baddr[0];
    pic->buf_u = imgb->baddr[1];
    pic->buf_v = imgb->baddr[2];
    pic->y     = imgb->a[0];
    pic->u     = imgb->a[1];
    pic->v     = imgb->a[2];

    pic->w_l   = imgb->w[0];
    pic->h_l   = imgb->h[0];
    pic->w_c   = imgb->w[1];
    pic->h_c   = imgb->h[1];

    pic->s_l   = STRIDE_IMGB2PIC(imgb->s[0]);
    pic->s_c   = STRIDE_IMGB2PIC(imgb->s[1]);

    pic->pad_l = pad_l;
    pic->pad_c = pad_c;

    pic->imgb  = imgb;

    /* allocate maps */
    w_scu = (pic->w_l + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    h_scu = (pic->h_l + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    f_scu = w_scu * h_scu;

    size = sizeof(s8) * f_scu * REFP_NUM;
    pic->map_refi = evc_malloc_fast(size);
    evc_assert_gv(pic->map_refi, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset_x64a(pic->map_refi, -1, size);

    size = sizeof(s16) * f_scu * REFP_NUM * MV_D;
    pic->map_mv = evc_malloc_fast(size);
    evc_assert_gv(pic->map_mv, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset_x64a(pic->map_mv, 0, size);

#if DMVR_LAG
    size = sizeof(s16) * f_scu * REFP_NUM * MV_D;
    pic->map_unrefined_mv = evc_malloc_fast(size);
    evc_assert_gv(pic->map_unrefined_mv, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset_x64a(pic->map_unrefined_mv, 0, size);
#endif

    if(err)
    {
        *err = EVC_OK;
    }
    return pic;

ERR:
    if(pic)
    {
        evc_mfree(pic->map_mv);
#if DMVR_LAG
        evc_mfree(pic->map_unrefined_mv);
#endif
        evc_mfree(pic->map_refi);
        evc_mfree(pic);
    }
    if(err) *err = ret;
    return NULL;
}

void evc_picbuf_free(EVC_PIC *pic)
{
    EVC_IMGB *imgb;

    if(pic)
    {
        imgb = pic->imgb;

        if(imgb)
        {
            imgb->release(imgb);

            pic->y = NULL;
            pic->u = NULL;
            pic->v = NULL;
            pic->w_l = 0;
            pic->h_l = 0;
            pic->w_c = 0;
            pic->h_c = 0;
            pic->s_l = 0;
            pic->s_c = 0;
        }
        evc_mfree(pic->map_mv);
#if DMVR_LAG
        evc_mfree(pic->map_unrefined_mv);
#endif
        evc_mfree(pic->map_refi);
        evc_mfree(pic);
    }
}

static void picbuf_expand(pel *a, int s, int w, int h, int exp)
{
    int i, j;
    pel pixel;
    pel *src, *dst;

    /* left */
    src = a;
    dst = a - exp;

    for(i = 0; i < h; i++)
    {
        pixel = *src; /* get boundary pixel */
        for(j = 0; j < exp; j++)
        {
            dst[j] = pixel;
        }
        dst += s;
        src += s;
    }

    /* right */
    src = a + (w - 1);
    dst = a + w;

    for(i = 0; i < h; i++)
    {
        pixel = *src; /* get boundary pixel */
        for(j = 0; j < exp; j++)
        {
            dst[j] = pixel;
        }
        dst += s;
        src += s;
    }

    /* upper */
    src = a - exp;
    dst = a - exp - (exp * s);

    for(i = 0; i < exp; i++)
    {
        evc_mcpy(dst, src, s*sizeof(pel));
        dst += s;
    }

    /* below */
    src = a + ((h - 1)*s) - exp;
    dst = a + ((h - 1)*s) - exp + s;

    for(i = 0; i < exp; i++)
    {
        evc_mcpy(dst, src, s*sizeof(pel));
        dst += s;
    }
}

void evc_picbuf_expand(EVC_PIC *pic, int exp_l, int exp_c)
{
    picbuf_expand(pic->y, pic->s_l, pic->w_l, pic->h_l, exp_l);
    picbuf_expand(pic->u, pic->s_c, pic->w_c, pic->h_c, exp_c);
    picbuf_expand(pic->v, pic->s_c, pic->w_c, pic->h_c, exp_c);
}

void scaling_mv(int ratio, s16 mvp[MV_D], s16 mv[MV_D])
{
    int tmp_mv;
    tmp_mv = mvp[MV_X] * ratio;
    tmp_mv = tmp_mv == 0 ? 0 : tmp_mv > 0 ? (tmp_mv + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION : -((-tmp_mv + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION);
    mv[MV_X] = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, tmp_mv);

    tmp_mv = mvp[MV_Y] * ratio;
    tmp_mv = tmp_mv == 0 ? 0 : tmp_mv > 0 ? (tmp_mv + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION : -((-tmp_mv + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION);
    mv[MV_Y] = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, tmp_mv);
}

void evc_get_mmvd_mvp_list(s8(*map_refi)[REFP_NUM], EVC_REFP refp[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, int h_scu, int scup, u16 avail, int log2_cuw, int log2_cuh, int slice_t, int real_mv[][2][3], u32 *map_scu, int REF_SET[][MAX_NUM_ACTIVE_REF_FRAME], u16 avail_lr
#if ADMVP
                           , EVC_HISTORY_BUFFER history_buffer, int admvp_flag
#endif
#if M49023_ADMVP_IMPROVE 
    , EVC_SH* sh
#endif
#if M50761_TMVP_8X8_GRID
    , int log2_max_cuwh
#endif
)
{
    int idx0, idx1, cnt;
    int ref_mvd = 0;
    int ref_mvd1 = 0;
    int list0_weight;
    int list1_weight;
    int ref_sign = 0;
#if M50632_IMPROVEMENT_MMVD
    int ref_sign1 = 0;
#endif
    int ref_mvd_cands[8] = {1, 2, 4, 8, 16, 32, 64, 128};
    int hor0var[MMVD_MAX_REFINE_NUM] = {0};
    int ver0var[MMVD_MAX_REFINE_NUM] = {0};
    int hor1var[MMVD_MAX_REFINE_NUM] = {0};
    int ver1Var[MMVD_MAX_REFINE_NUM] = {0};
    int base_mv_idx = 0;
    int base_skip[MMVD_BASE_MV_NUM];
    int c_num = 0;
    int base_mv[25][2][3];
    int k;
    int nn;
    s16 mvp[MAX_NUM_MVP][MV_D];
    s16 mvp1[MAX_NUM_MVP][MV_D];
    s8 refi[MAX_NUM_MVP];
    s8 refi1[MAX_NUM_MVP];
    s16 smvp[REFP_NUM][MAX_NUM_MVP][MV_D];
    s8 srefi[REFP_NUM][MAX_NUM_MVP];
    int base_mv_t[25][2][3];
    int base_type[3][MAX_NUM_MVP];
    int cur_set;
    int total_num = MMVD_BASE_MV_NUM*MMVD_MAX_REFINE_NUM;
    int sld1[MMVD_BASE_MV_NUM];
    int sld2[MMVD_BASE_MV_NUM];
    int sld[MMVD_BASE_MV_NUM*MMVD_BASE_MV_NUM][2];
    int s, z, a;
    int c_win = 0;
    int comp;
    int cuw = (1 << log2_cuw);
    int cuh = (1 << log2_cuh);
    int list0_r;
    int list1_r;
    int poc0, poc1, poc_c;
    int poc0_t, poc1_t;
#if M50632_IMPROVEMENT_MMVD
    int base_mv_p[25][3][3];
#endif
#if M48879_IMPROVEMENT_INTER
    int small_cu = 0;
    if (cuw*cuh <= NUM_SAMPLES_BLOCK)
        small_cu = 1;
#endif
    for(s = 0; s < MMVD_BASE_MV_NUM; s++)
    {
        sld1[s] = s;
        sld2[s] = s;
    }

    for(z = 0; z < MMVD_BASE_MV_NUM; z++)
    {
        comp = z;
        for(s = 0; s < MMVD_BASE_MV_NUM; s++)
        {
            a = s + comp;
            if(a >= MMVD_BASE_MV_NUM)
            {
                a = a - MMVD_BASE_MV_NUM;
            }
            sld[c_win][0] = sld1[s];
            sld[c_win][1] = sld2[a];
            c_win++;
        }
    }
#if ADMVP
#if M49023_ADMVP_IMPROVE
    if (admvp_flag == 0)
        evc_get_motion_skip_baseline(slice_t, scup, map_refi, map_mv, refp, cuw, cuh, w_scu, srefi, smvp, avail);
    else
#endif
#endif
#if M49023_ADMVP_IMPROVE
        evc_get_motion_merge_main(REF_SET[2][0], slice_t, scup, map_refi, map_mv, refp, cuw, cuh, w_scu, h_scu, srefi, smvp, map_scu, avail_lr
#if DMVR_LAG
            , NULL
#endif
#if ADMVP
            , history_buffer
#endif
#if IBC
            , 0
#endif
#if M49023_ADMVP_IMPROVE
            , (EVC_REFP(*)[2])refp
            , sh
#endif
#if M50761_TMVP_8X8_GRID
            , log2_max_cuwh
#endif
        );
#endif

    for (z = 0; z < MAX_NUM_MVP; z++)
    {
        mvp[z][MV_X] = smvp[REFP_0][z][MV_X];
        mvp[z][MV_Y] = smvp[REFP_0][z][MV_Y];
        mvp1[z][MV_X] = smvp[REFP_1][z][MV_X];
        mvp1[z][MV_Y] = smvp[REFP_1][z][MV_Y];
        refi[z] = srefi[REFP_0][z];
        refi1[z] = srefi[REFP_1][z];
    }

    if (slice_t == SLICE_B)
    {
        for(k = 0; k < MMVD_BASE_MV_NUM; k++)
        {
            base_mv[c_num][0][MV_X] = mvp[sld[k][0]][MV_X];
            base_mv[c_num][0][MV_Y] = mvp[sld[k][0]][MV_Y];
            base_mv[c_num][1][MV_X] = mvp1[sld[k][1]][MV_X];
            base_mv[c_num][1][MV_Y] = mvp1[sld[k][1]][MV_Y];
            base_mv[c_num][0][2] = refi[sld[k][0]];
            base_mv[c_num][1][2] = refi1[sld[k][1]];

            c_num++;
        }
    }
    else
    {
#if INCREASE_MVP_NUM
        for (idx0 = 0; idx0 < ORG_MAX_NUM_MVP; idx0++)
#else
        for (idx0 = 0; idx0 < MAX_NUM_MVP; idx0++)
#endif
        {
#if INCREASE_MVP_NUM
            cnt = (slice_t == SLICE_B ? ORG_MAX_NUM_MVP : 1);
#else
            cnt = (slice_t == SLICE_B ? MAX_NUM_MVP : 1);
#endif
            for (idx1 = 0; idx1 < cnt; idx1++)
            {
                base_mv[c_num][0][MV_X] = mvp[idx0][MV_X];
                base_mv[c_num][0][MV_Y] = mvp[idx0][MV_Y];
                base_mv[c_num][1][MV_X] = mvp1[idx1][MV_X];
                base_mv[c_num][1][MV_Y] = mvp1[idx1][MV_Y];
                base_mv[c_num][0][2] = refi[idx0];
                base_mv[c_num][1][2] = refi1[idx1];

                c_num++;
            }
        }
    }

    for(k = 0; k < MMVD_BASE_MV_NUM; k++)
    {
        base_skip[k] = 1;
    }
#if !M50632_IMPROVEMENT_MMVD
    ref_sign = 1;
#endif
#if INCREASE_MVP_NUM
    for (k = 0; k < ORG_MAX_NUM_MVP; k++)
#else
    for (k = 0; k < MAX_NUM_MVP; k++)
#endif
    {
#if M50632_IMPROVEMENT_MMVD
        ref_sign = 1;
        ref_sign1 = 1;
#endif
        base_mv_t[k][0][0] = base_mv[k][0][0];
        base_mv_t[k][0][1] = base_mv[k][0][1];
        base_mv_t[k][0][2] = base_mv[k][0][2];

        base_mv_t[k][1][0] = base_mv[k][1][0];
        base_mv_t[k][1][1] = base_mv[k][1][1];
        base_mv_t[k][1][2] = base_mv[k][1][2];

        list0_r = base_mv_t[k][0][2];
        list1_r = base_mv_t[k][1][2];

        if ((base_mv_t[k][0][2] >= 0) && (base_mv_t[k][1][2] >= 0))
        {
            base_type[0][k] = 0;
            base_type[1][k] = 1;
            base_type[2][k] = 2;
        }
        else if ((base_mv_t[k][0][2] >= 0) && !(base_mv_t[k][1][2] >= 0))
        {
#if M50632_IMPROVEMENT_MMVD
            if (slice_t == SLICE_P)
            {
                int cur_ref_num = REF_SET[2][1];

                base_type[0][k] = 1;
                base_type[1][k] = 1;
                base_type[2][k] = 1;

                if (cur_ref_num == 1)
                {
                    base_mv_p[k][0][2] = base_mv_t[k][0][2];
                    base_mv_p[k][1][2] = base_mv_t[k][0][2];
                    base_mv_p[k][2][2] = base_mv_t[k][0][2];
                }
                else
                {
                    int app_idx = 0;
                    int prior_ref[2] = { -1,-1 };
                    int v;
                    for (v = 0; v < min(cur_ref_num, MAX_NUM_ACTIVE_REF_FRAME); v++)
                    {
                        if (v != base_mv_t[k][0][2])
                        {
                            prior_ref[app_idx] = v;
                            app_idx++;
                        }
                        if (app_idx == cur_ref_num)
                            break;
                    }
                    if (cur_ref_num == 2)
                    {
                        app_idx = 0;

                        base_mv_p[k][0][2] = base_mv_t[k][0][2];
                        base_mv_p[k][1][2] = prior_ref[app_idx];
                        base_mv_p[k][2][2] = base_mv_t[k][0][2];
                    }
                    else
                    {
                        app_idx = 0;

                        base_mv_p[k][0][2] = base_mv_t[k][0][2];
                        base_mv_p[k][1][2] = prior_ref[app_idx];
                        app_idx++;
                        base_mv_p[k][2][2] = prior_ref[app_idx];
                    }
                }

                if (cur_ref_num == 1)
                {
                    base_mv_p[k][0][0] = base_mv_t[k][0][0];
                    base_mv_p[k][0][1] = base_mv_t[k][0][1];

                    base_mv_p[k][1][0] = base_mv_t[k][0][0] + 3;
                    base_mv_p[k][1][1] = base_mv_t[k][0][1];

                    base_mv_p[k][2][0] = base_mv_t[k][0][0] - 3;
                    base_mv_p[k][2][1] = base_mv_t[k][0][1];
                }
                else if (cur_ref_num == 2)
                {
                    base_mv_p[k][0][0] = base_mv_t[k][0][0];
                    base_mv_p[k][0][1] = base_mv_t[k][0][1];

                    poc0 = REF_SET[0][base_mv_p[k][0][2]];
                    poc_c = REF_SET[2][0];
                    poc1 = REF_SET[0][base_mv_p[k][1][2]];

                    list0_weight = ((poc_c - poc0) << MVP_SCALING_PRECISION) / ((poc_c - poc1));
                    ref_sign = 1;
                    base_mv_p[k][1][0] = EVC_CLIP3(-32768, 32767, ref_sign * ((EVC_ABS(list0_weight * base_mv_t[k][0][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));
                    base_mv_p[k][1][1] = EVC_CLIP3(-32768, 32767, ref_sign1 * ((EVC_ABS(list0_weight * base_mv_t[k][0][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));

                    base_mv_p[k][2][0] = base_mv_t[k][0][0] + 3;
                    base_mv_p[k][2][1] = base_mv_t[k][0][1];
                }
                else if (cur_ref_num >= 3)
                {
                    base_mv_p[k][0][0] = base_mv_t[k][0][0];
                    base_mv_p[k][0][1] = base_mv_t[k][0][1];
                    base_mv_p[k][0][2] = base_mv_p[k][0][2];

                    poc0 = REF_SET[0][base_mv_p[k][0][2]];
                    poc_c = REF_SET[2][0];
                    poc1 = REF_SET[0][base_mv_p[k][1][2]];

                    list0_weight = ((poc_c - poc0) << MVP_SCALING_PRECISION) / ((poc_c - poc1));
                    ref_sign = 1;
                    base_mv_p[k][1][0] = EVC_CLIP3(-32768, 32767, ref_sign * ((EVC_ABS(list0_weight * base_mv_t[k][0][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));
                    base_mv_p[k][1][1] = EVC_CLIP3(-32768, 32767, ref_sign1 * ((EVC_ABS(list0_weight * base_mv_t[k][0][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));

                    poc0 = REF_SET[0][base_mv_p[k][0][2]];
                    poc_c = REF_SET[2][0];
                    poc1 = REF_SET[0][base_mv_p[k][2][2]];

                    list0_weight = ((poc_c - poc0) << MVP_SCALING_PRECISION) / ((poc_c - poc1));
                    ref_sign = 1;
                    base_mv_p[k][2][0] = EVC_CLIP3(-32768, 32767, ref_sign * ((EVC_ABS(list0_weight * base_mv_t[k][0][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));
                    base_mv_p[k][2][1] = EVC_CLIP3(-32768, 32767, ref_sign1 * ((EVC_ABS(list0_weight * base_mv_t[k][0][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));
                }
            }
            else
            {
#endif
            base_type[0][k] = 1;
            base_type[1][k] = 0;
            base_type[2][k] = 2;

            poc0 = REF_SET[0][list0_r];
            poc_c = REF_SET[2][0];
            poc1_t = (poc_c - poc0) + poc_c;
            list0_weight = 1 << MVP_SCALING_PRECISION;
            list1_weight = 1 << MVP_SCALING_PRECISION;

            if ((poc1_t == REF_SET[1][0]) || (poc1_t == REF_SET[1][1]))
            {
                if (poc1_t == REF_SET[1][0])
                {
                    base_mv_t[k][1][2] = 0;
                }
                else if (poc1_t == REF_SET[1][1])
                {
                    base_mv_t[k][1][2] = 1;
                }
                poc1 = REF_SET[1][base_mv_t[k][1][2]];
            }
            else
            {
                base_mv_t[k][1][2] = 0;
                poc1 = REF_SET[1][base_mv_t[k][1][2]];
            }
#if M50632_IMPROVEMENT_MMVD
            list1_weight = ((poc_c - poc1) << MVP_SCALING_PRECISION) / ((poc_c - poc0));
            if ((list1_weight * base_mv_t[k][0][0]) < 0)
            {
                ref_sign = -1;
            }

            base_mv_t[k][1][0] = EVC_CLIP3(-32768, 32767, ref_sign * ((EVC_ABS(list1_weight * base_mv_t[k][0][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));

            list1_weight = ((poc_c - poc1) << MVP_SCALING_PRECISION) / ((poc_c - poc0));
            if ((list1_weight * base_mv_t[k][0][1]) < 0)
            {
                ref_sign1 = -1;
            }

            base_mv_t[k][1][1] = EVC_CLIP3(-32768, 32767, ref_sign1 * ((EVC_ABS(list1_weight * base_mv_t[k][0][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));
#else
            list1_weight = (EVC_ABS(poc1 - poc_c) << MVP_SCALING_PRECISION) / (EVC_ABS(poc0 - poc_c));
            if ((poc0 - poc_c) * (poc_c - poc1) > 0)
            {
                ref_sign = -1;
            }

            base_mv_t[k][1][0] = ref_sign*(list1_weight * (base_mv_t[k][0][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
            base_mv_t[k][1][1] = ref_sign*(list1_weight * (base_mv_t[k][0][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
#endif
#if M50632_IMPROVEMENT_MMVD
            }
#endif        
}
        else if (!(base_mv_t[k][0][2] >= 0) && (base_mv_t[k][1][2] >= 0))
        {
            base_type[0][k] = 2;
            base_type[1][k] = 0;
            base_type[2][k] = 1;
            poc1 = REF_SET[1][list1_r];
            poc_c = REF_SET[2][0];
            poc0_t = (poc_c - poc1) + poc_c;
            list0_weight = 1 << MVP_SCALING_PRECISION;
            list1_weight = 1 << MVP_SCALING_PRECISION;

            if ((poc0_t == REF_SET[0][0]) || (poc0_t == REF_SET[0][1]))
            {
                if (poc0_t == REF_SET[0][0])
                {
                    base_mv_t[k][0][2] = 0;
                }
                else if (poc0_t == REF_SET[0][1])
                {
                    base_mv_t[k][0][2] = 1;
                }
                poc0 = REF_SET[0][base_mv_t[k][0][2]];
            }
            else
            {
                base_mv_t[k][0][2] = 0;
                poc0 = REF_SET[0][base_mv_t[k][0][2]];
            }
#if M50632_IMPROVEMENT_MMVD
            list0_weight = ((poc_c - poc0) << MVP_SCALING_PRECISION) / ((poc_c - poc1));
            if ((list0_weight * base_mv_t[k][1][0]) < 0)
            {
                ref_sign = -1;
            }
            base_mv_t[k][0][0] = EVC_CLIP3(-32768, 32767, ref_sign * ((EVC_ABS(list0_weight * base_mv_t[k][1][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));

            list0_weight = ((poc_c - poc0) << MVP_SCALING_PRECISION) / ((poc_c - poc1));
            if ((list0_weight * base_mv_t[k][1][1]) < 0)
            {
                ref_sign1 = -1;
            }
            base_mv_t[k][0][1] = EVC_CLIP3(-32768, 32767, ref_sign1 * ((EVC_ABS(list0_weight * base_mv_t[k][1][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION));
#else
            list0_weight = (EVC_ABS(poc0 - poc_c) << MVP_SCALING_PRECISION) / (EVC_ABS(poc1 - poc_c));
            if ((poc0 - poc_c) * (poc_c - poc1) > 0)
            {
                ref_sign = -1;
            }

            base_mv_t[k][0][0] = ref_sign *(list0_weight * (base_mv_t[k][1][0]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
            base_mv_t[k][0][1] = ref_sign *(list0_weight * (base_mv_t[k][1][1]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
#endif
        }
        else
        {
            base_type[0][k] = 3;
            base_type[1][k] = 3;
            base_type[2][k] = 3;
        }
    }

    for(k = 0; k < MMVD_BASE_MV_NUM - 1; k++)
    {
        if (base_skip[k] == 1)
        {
            for(nn = k + 1; nn < MMVD_BASE_MV_NUM; nn++)
            {

                if ((base_mv[k][0][2] != -1) && (base_mv[nn][0][2] != -1))
                {
                    if ((base_mv[k][1][2] != -1) && (base_mv[nn][1][2] != -1))
                    {
                        if ((base_mv[k][0][MV_X] == base_mv[nn][0][MV_X])
                            && (base_mv[k][0][MV_Y] == base_mv[nn][0][MV_Y])
                            && (base_mv[k][0][2] == base_mv[nn][0][2])
                            && (base_mv[k][1][MV_X] == base_mv[nn][1][MV_X])
                            && (base_mv[k][1][MV_Y] == base_mv[nn][1][MV_Y])
                            && (base_mv[k][1][2] == base_mv[nn][1][2])
                            )
                        {
                            base_skip[nn] = -1;
                        }
                    }
                    else if ((base_mv[k][1][2] == -1) && (base_mv[nn][1][2] == -1))
                    {
                        if ((base_mv[k][0][MV_X] == base_mv[nn][0][MV_X])
                            && (base_mv[k][0][MV_Y] == base_mv[nn][0][MV_Y])
                            && (base_mv[k][0][2] == base_mv[nn][0][2])
                            )
                        {
                            base_skip[nn] = -1;
                        }
                    }
                }

                if ((base_mv[k][0][2] == -1) && (base_mv[nn][0][2] == -1))
                {
                    if ((base_mv[k][1][2] != -1) && (base_mv[nn][1][2] != -1))
                    {
                        if ((base_mv[k][1][MV_X] == base_mv[nn][1][MV_X])
                            && (base_mv[k][1][MV_Y] == base_mv[nn][1][MV_Y])
                            && (base_mv[k][1][2] == base_mv[nn][1][2])
                            )
                        {
                            base_skip[nn] = -1;
                        }
                    }
                    else if ((base_mv[k][1][2] == -1) && (base_mv[nn][1][2] == -1))
                    {
                        base_skip[nn] = -1;
                    }
                }
            }
        }
    }

    for(base_mv_idx = 0; base_mv_idx < MMVD_BASE_MV_NUM; base_mv_idx++)
    {
#if M48879_IMPROVEMENT_INTER
        int list0_r, list1_r;
        int poc0, poc1, poc_c;
#else
        int list0_r = base_mv[base_mv_idx][0][2];
        int list1_r = base_mv[base_mv_idx][1][2];
        int poc0, poc1, poc_c;
#endif
#if M48879_IMPROVEMENT_INTER
        int group_num = 3;

        if (small_cu)
        {
            group_num = 1;
            base_type[0][base_mv_idx] = 1;
        }

        for (cur_set = 0; cur_set < group_num; cur_set++)
        {
            if (base_type[cur_set][base_mv_idx] == 0)
            {
                base_mv[base_mv_idx][0][0] = base_mv_t[base_mv_idx][0][0];
                base_mv[base_mv_idx][0][1] = base_mv_t[base_mv_idx][0][1];
                base_mv[base_mv_idx][0][2] = base_mv_t[base_mv_idx][0][2];

                base_mv[base_mv_idx][1][0] = base_mv_t[base_mv_idx][1][0];
                base_mv[base_mv_idx][1][1] = base_mv_t[base_mv_idx][1][1];
                base_mv[base_mv_idx][1][2] = base_mv_t[base_mv_idx][1][2];
            }
            else if (base_type[cur_set][base_mv_idx] == 1)
            {
#if M50632_IMPROVEMENT_MMVD
                if (slice_t == SLICE_P)
                {
                    base_mv[base_mv_idx][0][2] = base_mv_p[base_mv_idx][cur_set][2];
                    base_mv[base_mv_idx][1][2] = -1;

                    base_mv[base_mv_idx][0][0] = base_mv_p[base_mv_idx][cur_set][0];
                    base_mv[base_mv_idx][0][1] = base_mv_p[base_mv_idx][cur_set][1];
                }
                else
                {
                    base_mv[base_mv_idx][0][2] = base_mv_t[base_mv_idx][0][2];
                    base_mv[base_mv_idx][1][2] = -1;

                    base_mv[base_mv_idx][0][0] = base_mv_t[base_mv_idx][0][0];
                    base_mv[base_mv_idx][0][1] = base_mv_t[base_mv_idx][0][1];
                }
#else
                base_mv[base_mv_idx][0][2] = base_mv_t[base_mv_idx][0][2];
                base_mv[base_mv_idx][1][2] = -1;

                base_mv[base_mv_idx][0][0] = base_mv_t[base_mv_idx][0][0];
                base_mv[base_mv_idx][0][1] = base_mv_t[base_mv_idx][0][1];
#endif
            }
            else if (base_type[cur_set][base_mv_idx] == 2)
            {
                base_mv[base_mv_idx][0][2] = -1;
                base_mv[base_mv_idx][1][2] = base_mv_t[base_mv_idx][1][2];

                base_mv[base_mv_idx][1][0] = base_mv_t[base_mv_idx][1][0];
                base_mv[base_mv_idx][1][1] = base_mv_t[base_mv_idx][1][1];
            }
            else if (base_type[cur_set][base_mv_idx] == 3)
            {
                base_mv[base_mv_idx][0][2] = -1;
                base_mv[base_mv_idx][1][2] = -1;
            }

            list0_r = base_mv[base_mv_idx][0][2];
            list1_r = base_mv[base_mv_idx][1][2];
#endif
        ref_sign = 1;
        if (slice_t == SLICE_B)
        {
            if ((list0_r != -1) && (list1_r != -1))
            {
                poc0 = REF_SET[0][list0_r];
                poc1 = REF_SET[1][list1_r];
                poc_c = REF_SET[2][0];

                if ((poc0 - poc_c) * (poc_c - poc1) > 0)
                {
                    ref_sign = -1;
                }
            }
        }

        for(k = 0; k < MMVD_MAX_REFINE_NUM; k++)
        {
            list0_weight = 1 << MVP_SCALING_PRECISION;
            list1_weight = 1 << MVP_SCALING_PRECISION;
            ref_mvd = ref_mvd_cands[(int)(k / 4)];
            ref_mvd1 = ref_mvd_cands[(int)(k / 4)];

            if ((list0_r != -1) && (list1_r != -1))
            {
                poc0 = REF_SET[0][list0_r];
                poc1 = REF_SET[1][list1_r];
                poc_c = REF_SET[2][0];

                if (EVC_ABS(poc1 - poc_c) >= EVC_ABS(poc0 - poc_c))
                {
                    list0_weight = (EVC_ABS(poc0 - poc_c) << MVP_SCALING_PRECISION) / (EVC_ABS(poc1 - poc_c));
#if M50632_IMPROVEMENT_MMVD
                    ref_mvd = EVC_CLIP3(-32768, 32767, (list0_weight * ref_mvd_cands[(int)(k / 4)] + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION);
#endif
                }
                else
                {
                    list1_weight = (EVC_ABS(poc1 - poc_c) << MVP_SCALING_PRECISION) / (EVC_ABS(poc0 - poc_c));
#if M50632_IMPROVEMENT_MMVD
                    ref_mvd1 = EVC_CLIP3(-32768, 32767, (list1_weight * ref_mvd_cands[(int)(k / 4)] + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION);
#endif
                }
#if !M50632_IMPROVEMENT_MMVD
                ref_mvd = (list0_weight * (ref_mvd_cands[(int)(k / 4)]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
                ref_mvd1 = (list1_weight * (ref_mvd_cands[(int)(k / 4)]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
#endif
                ref_mvd = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, ref_mvd);
                ref_mvd1 = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, ref_mvd1);
            }

            if ((k % 4) == 0)
            {
                hor0var[k] = ref_mvd;
                hor1var[k] = ref_mvd1 * ref_sign;
                ver0var[k] = 0;
                ver1Var[k] = 0;
            }
            else if ((k % 4) == 1)
            {
                hor0var[k] = ref_mvd * -1;
                hor1var[k] = ref_mvd1 * -1 * ref_sign;
                ver0var[k] = 0;
                ver1Var[k] = 0;
            }
            else if ((k % 4) == 2)
            {
                hor0var[k] = 0;
                hor1var[k] = 0;
                ver0var[k] = ref_mvd;
                ver1Var[k] = ref_mvd1 * ref_sign;
            }
            else
            {
                hor0var[k] = 0;
                hor1var[k] = 0;
                ver0var[k] = ref_mvd * -1;
                ver1Var[k] = ref_mvd1 * -1 * ref_sign;
            }

#if M48879_IMPROVEMENT_INTER
            real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][MV_X] = base_mv[base_mv_idx][0][MV_X] + hor0var[k];
            real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][MV_Y] = base_mv[base_mv_idx][0][MV_Y] + ver0var[k];
            real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][MV_X] = base_mv[base_mv_idx][1][MV_X] + hor1var[k];
            real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][MV_Y] = base_mv[base_mv_idx][1][MV_Y] + ver1Var[k];

            if (base_skip[base_mv_idx] == -1)
            {
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][2] = -1;
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][2] = -1;
            }
            else
            {
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][2] = base_mv[base_mv_idx][0][2];
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][2] = base_mv[base_mv_idx][1][2];
            }
#else
            real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][0][MV_X] = base_mv[base_mv_idx][0][MV_X] + hor0var[k];
            real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][0][MV_Y] = base_mv[base_mv_idx][0][MV_Y] + ver0var[k];
            real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][1][MV_X] = base_mv[base_mv_idx][1][MV_X] + hor1var[k];
            real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][1][MV_Y] = base_mv[base_mv_idx][1][MV_Y] + ver1Var[k];

            if (base_skip[base_mv_idx] == -1)
            {
                real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][0][2] = -1;
                real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][1][2] = -1;
            }
            else
            {
                real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][0][2] = base_mv[base_mv_idx][0][2];
                real_mv[base_mv_idx*MMVD_MAX_REFINE_NUM + k][1][2] = base_mv[base_mv_idx][1][2];
            }
#endif

        }
#if M48879_IMPROVEMENT_INTER
        }
#endif
    }

#if !M48879_IMPROVEMENT_INTER
    for(base_mv_idx = 0; base_mv_idx < MMVD_BASE_MV_NUM; base_mv_idx++)
    {
        int list0_r = base_mv[base_mv_idx][0][2];
        int list1_r = base_mv[base_mv_idx][1][2];
        int poc0, poc1, poc_c;

        for (cur_set = 1; cur_set < 3; cur_set++)
        {
            base_mv[base_mv_idx][0][0] = base_mv_t[base_mv_idx][0][0];
            base_mv[base_mv_idx][0][1] = base_mv_t[base_mv_idx][0][1];
            base_mv[base_mv_idx][0][2] = base_mv_t[base_mv_idx][0][2];

            base_mv[base_mv_idx][1][0] = base_mv_t[base_mv_idx][1][0];
            base_mv[base_mv_idx][1][1] = base_mv_t[base_mv_idx][1][1];
            base_mv[base_mv_idx][1][2] = base_mv_t[base_mv_idx][1][2];

            if (base_type[cur_set][base_mv_idx] == 0)
            {
                base_mv[base_mv_idx][0][2] = base_mv_t[base_mv_idx][0][2];
                base_mv[base_mv_idx][1][2] = base_mv_t[base_mv_idx][1][2];
            }
            else if (base_type[cur_set][base_mv_idx] == 1)
            {
                base_mv[base_mv_idx][0][2] = base_mv_t[base_mv_idx][0][2];
                base_mv[base_mv_idx][1][2] = -1;
            }
            else if (base_type[cur_set][base_mv_idx] == 2)
            {
                base_mv[base_mv_idx][0][2] = -1;
                base_mv[base_mv_idx][1][2] = base_mv_t[base_mv_idx][1][2];
            }
            else if (base_type[cur_set][base_mv_idx] == 3)
            {
                base_mv[base_mv_idx][0][2] = -1;
                base_mv[base_mv_idx][1][2] = -1;
            }

            list0_r = base_mv[base_mv_idx][0][2];
            list1_r = base_mv[base_mv_idx][1][2];

            ref_sign = 1;
            if (slice_t == SLICE_B)
            {
                if ((list0_r != -1) && (list1_r != -1))
                {
                    poc0 = REF_SET[0][list0_r];
                    poc1 = REF_SET[1][list1_r];
                    poc_c = REF_SET[2][0];

                    if ((poc0 - poc_c) * (poc_c - poc1) > 0)
                    {
                        ref_sign = -1;
                    }
                }
            }

            for(k = 0; k < MMVD_MAX_REFINE_NUM; k++)
            {
                list0_weight = 1 << MVP_SCALING_PRECISION;
                list1_weight = 1 << MVP_SCALING_PRECISION;
                ref_mvd = ref_mvd_cands[(int)(k / 4)];
                ref_mvd1 = ref_mvd_cands[(int)(k / 4)];

                if ((list0_r != -1) && (list1_r != -1))
                {
                    poc0 = REF_SET[0][list0_r];
                    poc1 = REF_SET[1][list1_r];
                    poc_c = REF_SET[2][0];

                    if (EVC_ABS(poc1 - poc_c) >= EVC_ABS(poc0 - poc_c))
                    {
                        list0_weight = (EVC_ABS(poc0 - poc_c) << MVP_SCALING_PRECISION) / (EVC_ABS(poc1 - poc_c));
                    }
                    else
                    {
                        list1_weight = (EVC_ABS(poc1 - poc_c) << MVP_SCALING_PRECISION) / (EVC_ABS(poc0 - poc_c));
                    }
                    ref_mvd = (list0_weight * (ref_mvd_cands[(int)(k / 4)]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;
                    ref_mvd1 = (list1_weight * (ref_mvd_cands[(int)(k / 4)]) + (1 << (MVP_SCALING_PRECISION - 1))) >> MVP_SCALING_PRECISION;

                    ref_mvd = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, ref_mvd);
                    ref_mvd1 = EVC_CLIP3(-(1 << 15), (1 << 15) - 1, ref_mvd1);
                }
                if ((k % 4) == 0)
                {
                    hor0var[k] = ref_mvd;
                    hor1var[k] = ref_mvd1 * ref_sign;
                    ver0var[k] = 0;
                    ver1Var[k] = 0;
                }
                else if ((k % 4) == 1)
                {
                    hor0var[k] = ref_mvd * -1;
                    hor1var[k] = ref_mvd1 * -1 * ref_sign;
                    ver0var[k] = 0;
                    ver1Var[k] = 0;
                }
                else if ((k % 4) == 2)
                {
                    hor0var[k] = 0;
                    hor1var[k] = 0;
                    ver0var[k] = ref_mvd;
                    ver1Var[k] = ref_mvd1 * ref_sign;
                }
                else
                {
                    hor0var[k] = 0;
                    hor1var[k] = 0;
                    ver0var[k] = ref_mvd * -1;
                    ver1Var[k] = ref_mvd1 * -1 * ref_sign;
                }

                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][MV_X] = base_mv[base_mv_idx][0][MV_X] + hor0var[k];
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][MV_Y] = base_mv[base_mv_idx][0][MV_Y] + ver0var[k];
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][MV_X] = base_mv[base_mv_idx][1][MV_X] + hor1var[k];
                real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][MV_Y] = base_mv[base_mv_idx][1][MV_Y] + ver1Var[k];

                if (base_skip[base_mv_idx] == -1)
                {
                    real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][2] = -1;
                    real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][2] = -1;
                }
                else
                {
                    real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][0][2] = base_mv[base_mv_idx][0][2];
                    real_mv[cur_set*total_num + base_mv_idx * MMVD_MAX_REFINE_NUM + k][1][2] = base_mv[base_mv_idx][1][2];
                }
            }
        }
    }
#endif
}

#if ADMVP
void evc_check_motion_availability2(int scup, int cuw, int cuh, int w_scu, int h_scu, int neb_addr[MAX_NUM_POSSIBLE_SCAND], int valid_flag[MAX_NUM_POSSIBLE_SCAND], u32* map_scu, u16 avail_lr, int num_mvp
#if IBC
  , int is_ibc
#endif
)
{
    int dx = 0;
    int dy = 0;

    int x_scu = scup % w_scu;
    int y_scu = scup / w_scu;
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;
    memset(valid_flag, 0, 5 * sizeof(int));
    
    if (avail_lr == LR_11)
    {
        neb_addr[0] = scup - 1;
        neb_addr[1] = scup + scuw;
        neb_addr[2] = scup - w_scu;
#if IBC
        if (is_ibc)
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
        else
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && !MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && !MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
#else
        valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]);
        valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
        if (num_mvp == 1)
        {
            neb_addr[3] = scup - w_scu + scuw;
            neb_addr[4] = scup - w_scu - 1;
#if IBC
            if (is_ibc)
            {
              valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
            else
            {
              valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && !MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && !MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
#else
            valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]);
            valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]);
#endif
        }
    }
    else if (avail_lr == LR_01)
    {
        neb_addr[0] = scup + (scuh - 1) * w_scu + scuw; // inverse H
        neb_addr[1] = scup - w_scu; // inverse D
        neb_addr[2] = scup - w_scu - 1;  // inverse E
#if IBC
        if (is_ibc)
        {
          valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
        else
        {
          valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && !MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && !MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
#else
        valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]);
        valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
        if (num_mvp == 1)
        {
            neb_addr[3] = scup + scuh * w_scu + scuw; // inverse I
            neb_addr[4] = scup - w_scu + scuw; // inverse A
#if IBC
            if (is_ibc)
            {
              valid_flag[3] = y_scu + scuh < h_scu && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
            else
            {
              valid_flag[3] = y_scu + scuh < h_scu && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && !MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && !MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
#else
            valid_flag[3] = y_scu + scuh < h_scu && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]);
            valid_flag[4] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]);
#endif
        }
    }
    else
    {
        neb_addr[0] = scup + (scuh - 1) * w_scu - 1; // H
        neb_addr[1] = scup - w_scu + scuw - 1; // D
        neb_addr[2] = scup - w_scu + scuw;  // E
#if IBC
        if (is_ibc)
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
        else
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && !MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && !MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
#else
        valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]);
        valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
        if (num_mvp == 1)
        {
            neb_addr[3] = scup + scuh * w_scu - 1; // I
            neb_addr[4] = scup - w_scu - 1; // A
#if IBC
            if (is_ibc)
            {
              valid_flag[3] = y_scu + scuh < h_scu && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[3]]) && MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = y_scu > 0 && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
            else
            {
              valid_flag[3] = y_scu + scuh < h_scu && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && !MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = y_scu > 0 && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && !MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
#else
            valid_flag[3] = y_scu + scuh < h_scu && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]);
            valid_flag[4] = y_scu > 0 && x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]);
#endif
        }
    }
}
#endif

#if (DMVR_LAG == 2)
int evc_use_refine_mv(int scup,int neb_scup, int w_scu)
{
    int curr_x = scup % w_scu;
    int curr_y = scup / w_scu;
    int neb_x = neb_scup % w_scu;
    int neb_y = neb_scup / w_scu;
    // first row
    if (!curr_y)
    {
        return 0;
    }
    //if current cu doesn't have its top aligned with max CU, then refined MV can't be used
    if (!(curr_y & ((MAX_CU_SIZE >> MIN_CU_LOG2) - 1)))
    {        
        //neighbor cu must be top to use refined MVs
        if (neb_y == (curr_y - 1))
        {
            // neighbor cu's refined MVs can be used only if it belongs to top /top left LCU 
            if (neb_x < (((curr_x >> (MAX_CU_LOG2 - MIN_CU_LOG2)) << (MAX_CU_LOG2 - MIN_CU_LOG2)) + (MAX_CU_SIZE >> MIN_CU_LOG2))) 
            {
                return 1;
            }
        }
    }
    return 0;
}
#endif

void evc_check_motion_availability(int scup, int cuw, int cuh, int w_scu, int h_scu, int neb_addr[MAX_NUM_POSSIBLE_SCAND], int valid_flag[MAX_NUM_POSSIBLE_SCAND], u32* map_scu, u16 avail_lr, int num_mvp
#if IBC
  , int is_ibc
#endif
)
{
    int x_scu = scup % w_scu;
    int y_scu = scup / w_scu;
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;

    if (avail_lr == LR_11)
    {
        neb_addr[0] = scup - 1;
        neb_addr[1] = scup + scuw;
        neb_addr[2] = scup - w_scu;
#if IBC
        if (is_ibc)
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
        else
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && !MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && !MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
#else
        valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]);
        valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
        if (num_mvp == 1)
        {
            neb_addr[3] = scup - w_scu + scuw;
            neb_addr[4] = scup - w_scu - 1;
#if IBC
            if (is_ibc)
            {
              valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
            else
            {
              valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && !MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && !MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
#else
            valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]);
            valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]);
#endif
        }
    }
    else if (avail_lr == LR_01)
    {
        neb_addr[0] = scup + scuw;
        neb_addr[1] = scup - w_scu + scuw - 1;
        neb_addr[2] = scup - w_scu - 1;
#if IBC
        if (is_ibc)
        {
          valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
        else
        {
          valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && !MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && !MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
#else
        valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]);
        valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]);
        valid_flag[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
        if (!valid_flag[2])
        {
            if (cuw >= cuh)
            {
                neb_addr[2] = scup - w_scu;
#if IBC
                if (is_ibc)
                {
                  valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
                else
                {
                  valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
#else
                valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
            }
            else
            {
                neb_addr[2] = scup + (scuh - 1) * w_scu + scuw;
#if IBC
                if (is_ibc)
                {
                  valid_flag[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
                else
                {
                  valid_flag[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
#else
                valid_flag[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
            }
        }

        if (num_mvp == 1)
        {
            neb_addr[3] = scup - w_scu + scuw;
            neb_addr[4] = scup + scuh * w_scu + scuw;
#if IBC
            if (is_ibc)
            {
              valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
            else
            {
              valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && !MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && !MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
#else
            valid_flag[3] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]);
            valid_flag[4] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]);
#endif
        }
    }
    else
    {
        neb_addr[0] = scup - 1;
        neb_addr[1] = scup - w_scu;
        neb_addr[2] = scup - w_scu + scuw;
#if IBC
        if (is_ibc)
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
        else
        {
          valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && !MCU_GET_IBC(map_scu[neb_addr[0]]);
          valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && !MCU_GET_IBC(map_scu[neb_addr[1]]);
          valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
        }
#else
        valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]);
        valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
        if (!valid_flag[2])
        {
            if (cuw >= cuh)
            {
                neb_addr[2] = scup - w_scu + scuw - 1;
#if IBC
                if (is_ibc)
                {
                  valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
                else
                {
                  valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
#else
                valid_flag[2] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
            }
            else
            {
                neb_addr[2] = scup + (scuh - 1) * w_scu - 1;
#if IBC
                if (is_ibc)
                {
                  valid_flag[2] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
                else
                {
                  valid_flag[2] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && !MCU_GET_IBC(map_scu[neb_addr[2]]);
                }
#else
                valid_flag[2] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]);
#endif
            }
        }

        if (num_mvp == 1)
        {
            neb_addr[3] = scup - w_scu - 1;
            neb_addr[4] = scup + scuh * w_scu - 1;
#if IBC
            if (is_ibc)
            {
              valid_flag[3] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[3]]) && MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
            else
            {
              valid_flag[3] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && !MCU_GET_IBC(map_scu[neb_addr[3]]);
              valid_flag[4] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && !MCU_GET_IBC(map_scu[neb_addr[4]]);
            }
#else
            valid_flag[3] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]);
            valid_flag[4] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]);
#endif
        }
    }
}

s8 evc_get_first_refi(int scup, int lidx, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int cuw, int cuh, int w_scu, int h_scu, u32 *map_scu, u8 mvr_idx, u16 avail_lr
#if DMVR_LAG
                      , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
#if ADMVP
                      , EVC_HISTORY_BUFFER history_buffer
                      , int admvp_flag
#endif
)
{
    int neb_addr[MAX_NUM_POSSIBLE_SCAND], valid_flag[MAX_NUM_POSSIBLE_SCAND];
    s8  refi = 0, default_refi;
    s16 default_mv[MV_D];

#if ADMVP
    evc_check_motion_availability2(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 1
#if IBC
      , 0
#endif
    );
#else
    evc_check_motion_availability(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 1
#if IBC
      , 0
#endif
    );
#endif
    evc_get_default_motion(neb_addr, valid_flag, 0, lidx, map_refi, map_mv, &default_refi, default_mv
#if DMVR_LAG
                           , map_scu
                           , map_unrefined_mv
                           , scup
                           , w_scu
#endif
#if ADMVP
                           , history_buffer
                           , admvp_flag
#endif
    );

    assert(mvr_idx < 5);
    //neb-position is coupled with mvr index
    if(valid_flag[mvr_idx])
    {
        refi = REFI_IS_VALID(map_refi[neb_addr[mvr_idx]][lidx]) ? map_refi[neb_addr[mvr_idx]][lidx] : default_refi;
    }
    else
    {
        refi = default_refi;
    }

    return refi;
}

void evc_get_default_motion(int neb_addr[MAX_NUM_POSSIBLE_SCAND], int valid_flag[MAX_NUM_POSSIBLE_SCAND], s8 cur_refi, int lidx, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], s8 *refi, s16 mv[MV_D]
#if DMVR_LAG
                            , u32 *map_scu
                            , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
                            , int scup
                            , int w_scu
#endif
#if ADMVP
                            , EVC_HISTORY_BUFFER history_buffer
                            , int admvp_flag
#endif
)
{
    int k;
    int found = 0;
    s8  tmp_refi = 0;

    *refi = 0;
    mv[MV_X] = 0;
    mv[MV_Y] = 0;
#if M48879_IMPROVEMENT_INTER
    for (k = 0; k < 2; k++)
#else
    for (k = 0; k < 5; k++)
#endif
    {
        if(valid_flag[k])
        {
            tmp_refi = REFI_IS_VALID(map_refi[neb_addr[k]][lidx]) ? map_refi[neb_addr[k]][lidx] : REFI_INVALID;
            if(tmp_refi == cur_refi)
            {
                found = 1;
                *refi = tmp_refi;
#if DMVR_LAG
                if (MCU_GET_DMVRF(map_scu[neb_addr[k]])
#if (DMVR_LAG == 2)
                    && (!evc_use_refine_mv(scup, neb_addr[k], w_scu))
#endif                
                    )
                {
                    mv[MV_X] = map_unrefined_mv[neb_addr[k]][lidx][MV_X];
                    mv[MV_Y] = map_unrefined_mv[neb_addr[k]][lidx][MV_Y];
                }
                else
#endif
                {
                mv[MV_X] = map_mv[neb_addr[k]][lidx][MV_X];
                mv[MV_Y] = map_mv[neb_addr[k]][lidx][MV_Y];
                }
                break;
            }
        }
    }

    if(!found)
    {
#if M48879_IMPROVEMENT_INTER
        for (k = 0; k < 2; k++)
#else
        for (k = 0; k < 5; k++)
#endif
        {
            if(valid_flag[k])
            {
                tmp_refi = REFI_IS_VALID(map_refi[neb_addr[k]][lidx]) ? map_refi[neb_addr[k]][lidx] : REFI_INVALID;
                if(tmp_refi != REFI_INVALID)
                {
                    found = 1;
                    *refi = tmp_refi;
#if DMVR_LAG
                    if(MCU_GET_DMVRF(map_scu[neb_addr[k]])
#if (DMVR_LAG == 2)
                       && (!evc_use_refine_mv(scup, neb_addr[k], w_scu))
#endif
                       )
                    {
                        mv[MV_X] = map_unrefined_mv[neb_addr[k]][lidx][MV_X];
                        mv[MV_Y] = map_unrefined_mv[neb_addr[k]][lidx][MV_Y];
                    }
                    else
#endif
                    {
                        mv[MV_X] = map_mv[neb_addr[k]][lidx][MV_X];
                        mv[MV_Y] = map_mv[neb_addr[k]][lidx][MV_Y];
                    }
                    break;
                }
            }
        }
    }

#if ADMVP
    if(admvp_flag)
    {
        if(!found)
        {
            for(k = 1; k <= min(history_buffer.currCnt, ALLOWED_CHECKED_AMVP_NUM); k++)
            {
                tmp_refi = REFI_IS_VALID(history_buffer.history_refi_table[history_buffer.currCnt - k][lidx]) ? history_buffer.history_refi_table[history_buffer.currCnt - k][lidx] : REFI_INVALID;
                if(tmp_refi == cur_refi)
                {
                    found = 1;
                    *refi = tmp_refi;
                    mv[MV_X] = history_buffer.history_mv_table[history_buffer.currCnt - k][lidx][MV_X];
                    mv[MV_Y] = history_buffer.history_mv_table[history_buffer.currCnt - k][lidx][MV_Y];
                    break;
                }
            }
        }

        if(!found)
        {
            for(k = 1; k <= min(history_buffer.currCnt, ALLOWED_CHECKED_AMVP_NUM); k++)
            {
                tmp_refi = REFI_IS_VALID(history_buffer.history_refi_table[history_buffer.currCnt - k][lidx]) ? history_buffer.history_refi_table[history_buffer.currCnt - k][lidx] : REFI_INVALID;
                if(tmp_refi != REFI_INVALID)
                {
                    found = 1;
                    *refi = tmp_refi;
                    mv[MV_X] = history_buffer.history_mv_table[history_buffer.currCnt - k][lidx][MV_X];
                    mv[MV_Y] = history_buffer.history_mv_table[history_buffer.currCnt - k][lidx][MV_Y];
                    break;
                }
            }
        }
#endif
    }
}

void evc_get_motion_from_mvr(u8 mvr_idx, int ptr, int scup, int lidx, s8 cur_refi, int num_refp, \
                             s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], EVC_REFP(*refp)[REFP_NUM], \
                             int cuw, int cuh, int w_scu, int h_scu, u16 avail, s16 mvp[MAX_NUM_MVP][MV_D], s8 refi[MAX_NUM_MVP], u32* map_scu, u16 avail_lr
#if DMVR_FLAG
                             , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
#if ADMVP
                             , EVC_HISTORY_BUFFER history_buffer
                             , int admvp_flag
#endif
)
{
    int i, t0, ptr_refi_cur;
    int ratio[MAX_NUM_REF_PICS];
    int neb_addr[MAX_NUM_POSSIBLE_SCAND], valid_flag[MAX_NUM_POSSIBLE_SCAND];
    int rounding = mvr_idx > 0 ? 1 << (mvr_idx - 1) : 0;
    s8 default_refi;
    s16 default_mv[MV_D];
    s16 mvp_temp[MV_D];
#if ADMVP
    evc_check_motion_availability2(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 1
#if IBC
      , 0
#endif
    );
#else
    evc_check_motion_availability(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 1
#if IBC
      , 0
#endif
    );
#endif
    evc_get_default_motion(neb_addr, valid_flag, cur_refi, lidx, map_refi, map_mv, &default_refi, default_mv
#if DMVR_LAG
                           , map_scu
                           , map_unrefined_mv
                           , scup
                           , w_scu
#endif
#if ADMVP
                           , history_buffer
                           , admvp_flag
#endif
    );

    ptr_refi_cur = refp[cur_refi][lidx].ptr;
    for(i = 0; i < num_refp; i++)
    {
        t0 = ptr - refp[i][lidx].ptr;

        if(t0 != 0)
        {
            ratio[i] = ((ptr - ptr_refi_cur) << MVP_SCALING_PRECISION) / t0;
        }

        if(ratio[i] == 0 || t0 == 0)
        {
            ratio[i] = 1 << MVP_SCALING_PRECISION;
        }
    }

    assert(mvr_idx < 5);

    if(valid_flag[mvr_idx])
    {
        refi[0] = REFI_IS_VALID(map_refi[neb_addr[mvr_idx]][lidx]) ? map_refi[neb_addr[mvr_idx]][lidx] : REFI_INVALID;
        if(refi[0] == cur_refi)
        {
#if DMVR_LAG
            if(MCU_GET_DMVRF(map_scu[neb_addr[mvr_idx]])
#if (DMVR_LAG == 2)
               && (!evc_use_refine_mv(scup, neb_addr[mvr_idx], w_scu))
#endif
               )
            {
                mvp_temp[MV_X] = map_unrefined_mv[neb_addr[mvr_idx]][lidx][MV_X];
                mvp_temp[MV_Y] = map_unrefined_mv[neb_addr[mvr_idx]][lidx][MV_Y];
            }
            else
#endif
            {
                mvp_temp[MV_X] = map_mv[neb_addr[mvr_idx]][lidx][MV_X];
                mvp_temp[MV_Y] = map_mv[neb_addr[mvr_idx]][lidx][MV_Y];
            }
        }
        else if(refi[0] == REFI_INVALID)
        {
            refi[0] = default_refi;
            if(refi[0] == cur_refi)
            {
                mvp_temp[MV_X] = default_mv[MV_X];
                mvp_temp[MV_Y] = default_mv[MV_Y];
            }
            else
            {
                scaling_mv(ratio[refi[0]], default_mv, mvp_temp);
            }
        }
        else
        {
#if DMVR_LAG
            if(MCU_GET_DMVRF(map_scu[neb_addr[mvr_idx]])
#if (DMVR_LAG == 2)
               && (!evc_use_refine_mv(scup, neb_addr[mvr_idx], w_scu))
#endif
               )
            {
                scaling_mv(ratio[refi[0]], map_unrefined_mv[neb_addr[mvr_idx]][lidx], mvp_temp);
            }
            else
#endif    
            {
                scaling_mv(ratio[refi[0]], map_mv[neb_addr[mvr_idx]][lidx], mvp_temp);
            }
        }
    }
    else
    {
        refi[0] = default_refi;
        mvp_temp[MV_X] = default_mv[MV_X];
        mvp_temp[MV_Y] = default_mv[MV_Y];
    }
    mvp[0][MV_X] = (mvp_temp[MV_X] >= 0) ? (((mvp_temp[MV_X] + rounding) >> mvr_idx) << mvr_idx) : -(((-mvp_temp[MV_X] + rounding) >> mvr_idx) << mvr_idx);
    mvp[0][MV_Y] = (mvp_temp[MV_Y] >= 0) ? (((mvp_temp[MV_Y] + rounding) >> mvr_idx) << mvr_idx) : -(((-mvp_temp[MV_Y] + rounding) >> mvr_idx) << mvr_idx);
}

void evc_get_motion(int scup, int lidx, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D],
                    EVC_REFP(*refp)[REFP_NUM],
                    int cuw, int cuh, int w_scu, u16 avail, s8 refi[MAX_NUM_MVP], s16 mvp[MAX_NUM_MVP][MV_D])
{

    if (IS_AVAIL(avail, AVAIL_LE))
    {
        refi[0] = 0;
        mvp[0][MV_X] = map_mv[scup - 1][lidx][MV_X];
        mvp[0][MV_Y] = map_mv[scup - 1][lidx][MV_Y];
    }
    else
    {
        refi[0] = 0;
        mvp[0][MV_X] = 1;
        mvp[0][MV_Y] = 1;
    }

    if (IS_AVAIL(avail, AVAIL_UP))
    {
        refi[1] = 0;
        mvp[1][MV_X] = map_mv[scup - w_scu][lidx][MV_X];
        mvp[1][MV_Y] = map_mv[scup - w_scu][lidx][MV_Y];
    }
    else
    {
        refi[1] = 0;
        mvp[1][MV_X] = 1;
        mvp[1][MV_Y] = 1;
    }

    if (IS_AVAIL(avail, AVAIL_UP_RI))
    {
        refi[2] = 0;
        mvp[2][MV_X] = map_mv[scup - w_scu + (cuw >> MIN_CU_LOG2)][lidx][MV_X];
        mvp[2][MV_Y] = map_mv[scup - w_scu + (cuw >> MIN_CU_LOG2)][lidx][MV_Y];
    }
    else
    {
        refi[2] = 0;
        mvp[2][MV_X] = 1;
        mvp[2][MV_Y] = 1;
    }
    refi[3] = 0;
    mvp[3][MV_X] = refp[0][lidx].map_mv[scup][0][MV_X];
    mvp[3][MV_Y] = refp[0][lidx].map_mv[scup][0][MV_Y];
}

#if AFFINE
void evc_get_motion_scaling(int ptr, int scup, int lidx, s8 cur_refi, int num_refp, s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], EVC_REFP(*refp)[REFP_NUM],
                            int cuw, int cuh, int w_scu, int h_scu, u16 avail, s16 mvp[MAX_NUM_MVP][MV_D], s8 refi[MAX_NUM_MVP], u32* map_scu, u16 avail_lr
#if DMVR_LAG
                            , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
)
{
    int cnt, i, t0, ptr_refi_cur, dptr_co;
    int ratio[MAX_NUM_REF_PICS] = {0,}, ratio_tmvp = 0;
    int neb_addr[MAX_NUM_POSSIBLE_SCAND], valid_flag[MAX_NUM_POSSIBLE_SCAND];
    s8(*map_refi_co)[REFP_NUM];

#if ADMVP
    evc_check_motion_availability2(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 0
#if IBC
      , 0
#endif
    );
#else
    evc_check_motion_availability(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 0
#if IBC
      , 0
#endif
    );
#endif

    ptr_refi_cur = refp[cur_refi][lidx].ptr;
    for(i = 0; i < num_refp; i++)
    {
        t0 = ptr - refp[i][lidx].ptr;

        if(t0 != 0)
        {
            ratio[i] = ((ptr - ptr_refi_cur) << MVP_SCALING_PRECISION) / t0;
        }
        if(ratio[i] == 0 || t0 == 0)
        {
            ratio[i] = 1 << MVP_SCALING_PRECISION;
        }
    }
#if INCREASE_MVP_NUM
    for(cnt = 0; cnt < ORG_MAX_NUM_MVP - 1; cnt++)
#else
    for(cnt = 0; cnt < MAX_NUM_MVP - 1; cnt++)
#endif
    {
        if(valid_flag[cnt])
        {
            refi[cnt] = REFI_IS_VALID(map_refi[neb_addr[cnt]][lidx]) ? map_refi[neb_addr[cnt]][lidx] : REFI_INVALID;
            if(refi[cnt] == cur_refi)
            {
#if DMVR_LAG
                if(MCU_GET_DMVRF(map_scu[neb_addr[cnt]])
#if (DMVR_LAG == 2)
                   && (!evc_use_refine_mv(scup, neb_addr[cnt], w_scu))
#endif
                   )
                {
                    mvp[cnt][MV_X] = map_unrefined_mv[neb_addr[cnt]][lidx][MV_X];
                    mvp[cnt][MV_Y] = map_unrefined_mv[neb_addr[cnt]][lidx][MV_Y];
                }
                else
#endif
                {

                    mvp[cnt][MV_X] = map_mv[neb_addr[cnt]][lidx][MV_X];
                    mvp[cnt][MV_Y] = map_mv[neb_addr[cnt]][lidx][MV_Y];
                }
            }
            else if(refi[cnt] == REFI_INVALID)
            {
                mvp[cnt][MV_X] = 0;
                mvp[cnt][MV_Y] = 0;
            }
            else
            {
#if DMVR_LAG
                if(MCU_GET_DMVRF(map_scu[neb_addr[cnt]])
#if (DMVR_LAG == 2)
                   && (!evc_use_refine_mv(scup, neb_addr[cnt], w_scu))
#endif
                   )
                {
                    scaling_mv(ratio[refi[cnt]], map_unrefined_mv[neb_addr[cnt]][lidx], mvp[cnt]);
                }
                else
#endif
                {
                    scaling_mv(ratio[refi[cnt]], map_mv[neb_addr[cnt]][lidx], mvp[cnt]);
                }
            }
        }
        else
        {
            refi[cnt] = 0;
            mvp[cnt][MV_X] = 0;
            mvp[cnt][MV_Y] = 0;
        }
    }

    refi[cnt] = 0;
    map_refi_co = refp[0][REFP_1].map_refi;
    if(REFI_IS_VALID(map_refi_co[scup][REFP_0]))
    {
        dptr_co = refp[0][REFP_1].ptr - refp[0][REFP_1].list_ptr[0];
        ratio_tmvp = 1 << MVP_SCALING_PRECISION;
        if(dptr_co == 0)
        {
            mvp[cnt][MV_X] = 0;
            mvp[cnt][MV_Y] = 0;
        }
        else
        {
            ratio_tmvp = ((ptr - ptr_refi_cur) << MVP_SCALING_PRECISION) / dptr_co;
            scaling_mv(ratio_tmvp, refp[0][REFP_1].map_mv[scup][REFP_0], mvp[cnt]);
        }
    }
    else
    {
        refi[cnt] = 0;
        mvp[cnt][MV_X] = 0;
        mvp[cnt][MV_Y] = 0;
    }
}
#endif

#if MERGE_MVP
#if ADMVP
static int evc_get_right_below_scup_qc_merge(int scup, int cuw, int cuh, int w_scu, int h_scu, int bottom_right
#if M50761_TMVP_8X8_GRID
    , int log2_max_cuwh
#endif
)
{
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;

    int x_scu = scup % w_scu + scuw - 1;
    int y_scu = scup / w_scu + scuh - 1;

    if (bottom_right == 0)            // fetch bottom sample
    {
        if (y_scu + 1 >= h_scu)
            return -1;
#if M50761_TMVP_8X8_GRID
        else if ( ((y_scu + 1) << MIN_CU_LOG2 >> log2_max_cuwh) != (y_scu << MIN_CU_LOG2 >> log2_max_cuwh) )
            return -1; // check same CTU row, align to spec
        else
            return ((y_scu + 1) >> 1 << 1) * w_scu + (x_scu >> 1 << 1);
#else
        else
            return (y_scu + 1)*w_scu + x_scu;
#endif
    }
    else if (bottom_right == 1)        // fetch bottom-to-right sample
    {
        if (x_scu + 1 >= w_scu)
            return -1;
#if M50761_TMVP_8X8_GRID
        else if ( ((x_scu + 1) << MIN_CU_LOG2 >> log2_max_cuwh) != (x_scu << MIN_CU_LOG2 >> log2_max_cuwh) )
            return -1; // check same CTU column, align to spec
        else
            return (y_scu >> 1 << 1) * w_scu + ((x_scu + 1) >> 1 << 1);
#else
        else
            return y_scu*w_scu + (x_scu + 1);
#endif
    }
    return -1;
}

static int evc_get_right_below_scup_qc_merge_suco(int scup, int cuw, int cuh, int w_scu, int h_scu, int bottom_right
#if M50761_TMVP_8X8_GRID
    , int log2_max_cuwh
#endif
)
{
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;

    int x_scu = scup % w_scu - 1;
    int y_scu = scup / w_scu + scuh - 1;

    if (bottom_right == 0)            // fetch bottom sample
    {
        if ( y_scu + 1 >= h_scu )
            return -1;
#if M50761_TMVP_8X8_GRID
        else if ( ((y_scu + 1) << MIN_CU_LOG2 >> log2_max_cuwh) != (y_scu << MIN_CU_LOG2 >> log2_max_cuwh) )
            return -1; // check same CTU row, align to spec
        else
            return ((y_scu + 1) >> 1 << 1) * w_scu + ((x_scu + 1) >> 1 << 1);  // bottom sample
#else
        else
            return (y_scu + 1)*w_scu + x_scu + 1;  // bottom sample
#endif
    }
    else if (bottom_right == 1)        // fetch bottom-to-left sample
    {
        if (x_scu < 0)
            return -1;
#if M50761_TMVP_8X8_GRID
        else if ( ((x_scu + 1) << MIN_CU_LOG2 >> log2_max_cuwh) != (x_scu << MIN_CU_LOG2 >> log2_max_cuwh) )
            return -1; // check same CTU column, align to spec
        else
            return (y_scu >> 1 << 1) * w_scu + (x_scu >> 1 << 1);
#else
        else
            return y_scu * w_scu + x_scu;
#endif
    }
    return -1;
}
#endif

static int evc_get_right_below_scup(int scup, int cuw, int cuh, int w_scu, int h_scu)
{
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;

    int x_scu = scup % w_scu + scuw - 1;
    int y_scu = scup / w_scu + scuh - 1;

    if (x_scu + 1 < w_scu && y_scu + 1 < h_scu)
    {
        return (y_scu + 1)*w_scu + (x_scu + 1);
    }
    else if (x_scu + 1 < w_scu)
    {
        return y_scu*w_scu + (x_scu + 1);
    }
    else if (y_scu + 1 < h_scu)
    {
        return (y_scu + 1)*w_scu + x_scu;
    }
    else
    {
        return y_scu*w_scu + x_scu;
    }
}
#endif

#if M50761_REMOVE_BIBLOCKS_8x4
BOOL check_bi_applicability_rdo(int tile_group_type, int cuw, int cuh)
{
    BOOL is_applicable = FALSE;
    if ((tile_group_type == SLICE_B) &&
        allowed_block_size(cuw, cuh))
    {
        is_applicable = TRUE;
    }
    return is_applicable;
}
#endif

#if ADMVP
BOOL check_bi_applicability(int slice_type, int cuw, int cuh)
{
    BOOL is_applicable = FALSE;
    if((slice_type == SLICE_B) &&
       !((max(cuw, cuh) < 8 && min(cuw, cuh) < 8))
       )
    {
        is_applicable = TRUE;
    }
    return is_applicable;
}
#endif

__inline static void check_redundancy(int slice_type, s16 mvp[REFP_NUM][MAX_NUM_MVP][MV_D], s8 refi[REFP_NUM][MAX_NUM_MVP], int *count)
{
    int i;
    int cnt = *count;

    if(cnt > 0)
    {
        if(refi != NULL)
        {
            for(i = (cnt)-1; i >= 0; i--)
            {
                if(refi[REFP_0][cnt] == refi[REFP_0][i] && mvp[REFP_0][cnt][MV_X] == mvp[REFP_0][i][MV_X] && mvp[REFP_0][cnt][MV_Y] == mvp[REFP_0][i][MV_Y])
                {
                    if(slice_type != SLICE_B || (refi[REFP_1][cnt] == refi[REFP_1][i] && mvp[REFP_1][cnt][MV_X] == mvp[REFP_1][i][MV_X] && mvp[REFP_1][cnt][MV_Y] == mvp[REFP_1][i][MV_Y]))
                    {
                        cnt--;
                        break;
                    }
                }
            }
        }
        else
        {
            for(i = cnt - 1; i >= 0; i--)
            {
                if(mvp[REFP_0][cnt][MV_X] == mvp[REFP_0][i][MV_X] && mvp[REFP_0][cnt][MV_Y] == mvp[REFP_0][i][MV_Y])
                {
                    if(slice_type != SLICE_B || (mvp[REFP_1][cnt][MV_X] == mvp[REFP_1][i][MV_X] && mvp[REFP_1][cnt][MV_Y] == mvp[REFP_1][i][MV_Y]))
                    {
                        cnt--;
                        break;
                    }
                }
            }
        }
        *count = cnt;
    }
}

#if M49023_ADMVP_IMPROVE
void evc_get_motion_merge_main(int ptr, int slice_type, int scup, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D],
    EVC_REFP refp[REFP_NUM], int cuw, int cuh, int w_scu, int h_scu, s8 refi[REFP_NUM][MAX_NUM_MVP], s16 mvp[REFP_NUM][MAX_NUM_MVP][MV_D], u32 *map_scu, u16 avail_lr
#if DMVR_LAG
    , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
    , EVC_HISTORY_BUFFER history_buffer
#if IBC
    , u8 ibc_flag
#endif
    , EVC_REFP(*refplx)[REFP_NUM]
#if M49023_ADMVP_IMPROVE 
    , EVC_SH* sh
#endif
#if M50761_TMVP_8X8_GRID
    , int log2_max_cuwh
#endif
)
{
    BOOL tmpvBottomRight = 0; // Bottom first
    int small_cu = 0;
    if (cuw*cuh <= NUM_SAMPLES_BLOCK)
        small_cu = 1;
    int k, cnt = 0;
    int neb_addr[MAX_NUM_POSSIBLE_SCAND], valid_flag[MAX_NUM_POSSIBLE_SCAND];
    s16 tmvp[REFP_NUM][MV_D];
    int scup_tmp;
#if M49023_ADMVP_IMPROVE
    int cur_num, i, idx0, idx1;
#else
    int cur_num, i, idx0, idx1, s, z;
#endif
    int c_win = 0;
#if !M49023_ADMVP_IMPROVE
    int priority_list0[MAX_NUM_MVP*MAX_NUM_MVP];
    int priority_list1[MAX_NUM_MVP*MAX_NUM_MVP];
#endif
    evc_mset(mvp, 0, MAX_NUM_MVP * REFP_NUM * MV_D * sizeof(s16));
    evc_mset(refi, REFI_INVALID, MAX_NUM_MVP * REFP_NUM * sizeof(s8));

    s8 refidx = REFI_INVALID;

    for (k = 0; k < MAX_NUM_POSSIBLE_SCAND; k++)
    {
        valid_flag[k] = 0;
    }
    evc_check_motion_availability2(scup, cuw, cuh, w_scu, h_scu, neb_addr, valid_flag, map_scu, avail_lr, 1
#if IBC
        , ibc_flag
#endif
    );

    for (k = 0; k < 5; k++)
    {
        if (valid_flag[k])
        {
#if DMVR_LAG

            if ((NULL != map_unrefined_mv) && MCU_GET_DMVRF(map_scu[neb_addr[k]])
#if (DMVR_LAG == 2)
                && (!evc_use_refine_mv(scup, neb_addr[k], w_scu))
#endif
                )
            {
                assert(map_unrefined_mv[neb_addr[k]][REFP_0][MV_X] != SHRT_MAX);
                assert(map_unrefined_mv[neb_addr[k]][REFP_0][MV_Y] != SHRT_MAX);
                refi[REFP_0][cnt] = REFI_IS_VALID(map_refi[neb_addr[k]][REFP_0]) ? map_refi[neb_addr[k]][REFP_0] : REFI_INVALID;
                mvp[REFP_0][cnt][MV_X] = map_unrefined_mv[neb_addr[k]][REFP_0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = map_unrefined_mv[neb_addr[k]][REFP_0][MV_Y];
                if (slice_type == SLICE_B)
                {
                    assert(map_unrefined_mv[neb_addr[k]][REFP_1][MV_X] != SHRT_MAX);
                    assert(map_unrefined_mv[neb_addr[k]][REFP_1][MV_Y] != SHRT_MAX);
                    refi[REFP_1][cnt] = REFI_IS_VALID(map_refi[neb_addr[k]][REFP_1]) ? map_refi[neb_addr[k]][REFP_1] : REFI_INVALID;
                    mvp[REFP_1][cnt][MV_X] = map_unrefined_mv[neb_addr[k]][REFP_1][MV_X];
                    mvp[REFP_1][cnt][MV_Y] = map_unrefined_mv[neb_addr[k]][REFP_1][MV_Y];
                }
            }
            else
#endif
            {
                refi[REFP_0][cnt] = REFI_IS_VALID(map_refi[neb_addr[k]][REFP_0]) ? map_refi[neb_addr[k]][REFP_0] : REFI_INVALID;
                mvp[REFP_0][cnt][MV_X] = map_mv[neb_addr[k]][REFP_0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = map_mv[neb_addr[k]][REFP_0][MV_Y];
                if (!check_bi_applicability(slice_type, cuw, cuh))
                {
                    refi[REFP_1][cnt] = REFI_INVALID;
                    mvp[REFP_1][cnt][MV_X] = 0;
                    mvp[REFP_1][cnt][MV_Y] = 0;
                }
                else

                {
                    refi[REFP_1][cnt] = REFI_IS_VALID(map_refi[neb_addr[k]][REFP_1]) ? map_refi[neb_addr[k]][REFP_1] : REFI_INVALID;
                    mvp[REFP_1][cnt][MV_X] = map_mv[neb_addr[k]][REFP_1][MV_X];
                    mvp[REFP_1][cnt][MV_Y] = map_mv[neb_addr[k]][REFP_1][MV_Y];
                }
            }
            check_redundancy(slice_type, mvp, refi, &cnt);
            cnt++;
        }
        if (cnt == (small_cu ? MAX_NUM_MVP_SMALL_CU - 1 : MAX_NUM_MVP - 1))
        {
            break;
        }
    }

    int tmvp_cnt_pos0 = 0, tmvp_cnt_pos1 = 0;
    int tmvp_added = 0;

#if M49023_ADMVP_IMPROVE
    if (!tmvp_added)
    {// TMVP-central
        s8 availablePredIdx = 0;
#if M50761_TMVP_8X8_GRID
        int x_scu = (scup % w_scu);
        int y_scu = (scup / w_scu);
        int scu_col = ((x_scu + (cuw >> 1 >> MIN_CU_LOG2)) >> 1 << 1) + ((y_scu + (cuh >> 1 >> MIN_CU_LOG2)) >> 1 << 1) * w_scu; // 8x8 grid
        evc_get_mv_collocated(
            refplx,
            ptr, scu_col, scup, w_scu, h_scu, tmvp, &availablePredIdx
#if M49023_ADMVP_IMPROVE 
            , sh
#endif
        );
#else
        evc_get_mv_collocated(
            refplx,
            ptr, scup + ((cuw >> 1) >> MIN_CU_LOG2) + ((cuh >> 1) >> MIN_CU_LOG2) * w_scu, scup, w_scu, h_scu, tmvp, &availablePredIdx
#if M49023_ADMVP_IMPROVE 
            , sh
#endif
        );
#endif
        if ((availablePredIdx == 1) || (availablePredIdx == 3))
        {
            refi[REFP_0][cnt] = 0;
            mvp[REFP_0][cnt][MV_X] = tmvp[REFP_0][MV_X];
            mvp[REFP_0][cnt][MV_Y] = tmvp[REFP_0][MV_Y];
        }
        else
        {
            refi[REFP_0][cnt] = REFI_INVALID;
            mvp[REFP_0][cnt][MV_X] = 0;
            mvp[REFP_0][cnt][MV_Y] = 0;
        }
        if (((availablePredIdx == 2) || (availablePredIdx == 3)) && check_bi_applicability(slice_type, cuw, cuh))
        {
            refi[REFP_1][cnt] = 0;
            mvp[REFP_1][cnt][MV_X] = tmvp[REFP_1][MV_X];
            mvp[REFP_1][cnt][MV_Y] = tmvp[REFP_1][MV_Y];
        }
        else
        {
            refi[REFP_1][cnt] = REFI_INVALID;
            mvp[REFP_1][cnt][MV_X] = 0;
            mvp[REFP_1][cnt][MV_Y] = 0;
        }

        tmvp_cnt_pos0 = cnt;
        if (availablePredIdx != 0)
        {
            check_redundancy(slice_type, mvp, refi, &cnt);
            cnt++;
            tmvp_cnt_pos1 = cnt;
            if (tmvp_cnt_pos1 == tmvp_cnt_pos0 + 1)
                tmvp_added = 1;
            if (cnt >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))
            {
                return;
            }
        }
    } // TMVP-central
    if (!tmvp_added)
    {// Bottom first
        s8 availablePredIdx = 0;
        tmpvBottomRight = 0;
        if (avail_lr == LR_01)
            scup_tmp = evc_get_right_below_scup_qc_merge_suco(scup, cuw, cuh, w_scu, h_scu, tmpvBottomRight
#if M50761_TMVP_8X8_GRID
                , log2_max_cuwh
#endif
            );
        else
            scup_tmp = evc_get_right_below_scup_qc_merge(scup, cuw, cuh, w_scu, h_scu, tmpvBottomRight
#if M50761_TMVP_8X8_GRID
                , log2_max_cuwh
#endif
            );
        if (scup_tmp != -1)  // if available, add it to candidate list
        {
            evc_get_mv_collocated(
                refplx,
                ptr, scup_tmp, scup, w_scu, h_scu, tmvp, &availablePredIdx
#if M49023_ADMVP_IMPROVE 
                , sh
#endif
            );
            if ((availablePredIdx == 1) || (availablePredIdx == 3))
            {
                refi[REFP_0][cnt] = 0;
                mvp[REFP_0][cnt][MV_X] = tmvp[REFP_0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = tmvp[REFP_0][MV_Y];
            }
            else
            {
                refi[REFP_0][cnt] = REFI_INVALID;
                mvp[REFP_0][cnt][MV_X] = 0;
                mvp[REFP_0][cnt][MV_Y] = 0;
            }
            if (((availablePredIdx == 2) || (availablePredIdx == 3)) && check_bi_applicability(slice_type, cuw, cuh))
            {
                refi[REFP_1][cnt] = 0;
                mvp[REFP_1][cnt][MV_X] = tmvp[REFP_1][MV_X];
                mvp[REFP_1][cnt][MV_Y] = tmvp[REFP_1][MV_Y];
            }
            else
            {
                refi[REFP_1][cnt] = REFI_INVALID;
                mvp[REFP_1][cnt][MV_X] = 0;
                mvp[REFP_1][cnt][MV_Y] = 0;
            }

            tmvp_cnt_pos0 = cnt;
            if (availablePredIdx != 0)
            {
                check_redundancy(slice_type, mvp, refi, &cnt);
                cnt++;
                tmvp_cnt_pos1 = cnt;
                if (tmvp_cnt_pos1 == tmvp_cnt_pos0 + 1)
                    tmvp_added = 1;
                if (cnt >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))
                {
                    return;
                }
            }
        }
    }
    if (!tmvp_added)
    {
        s8 availablePredIdx = 0;
        if (avail_lr == LR_01)
            scup_tmp = evc_get_right_below_scup_qc_merge_suco(scup, cuw, cuh, w_scu, h_scu, !tmpvBottomRight
#if M50761_TMVP_8X8_GRID
                , log2_max_cuwh
#endif
            );
        else
            scup_tmp = evc_get_right_below_scup_qc_merge(scup, cuw, cuh, w_scu, h_scu, !tmpvBottomRight
#if M50761_TMVP_8X8_GRID
                , log2_max_cuwh
#endif
            );
        if (scup_tmp != -1)  // if available, add it to candidate list
        {
            evc_get_mv_collocated(
                refplx,
                ptr, scup_tmp, scup, w_scu, h_scu, tmvp, &availablePredIdx
#if M49023_ADMVP_IMPROVE 
                , sh
#endif
            );
            if ((availablePredIdx == 1) || (availablePredIdx == 3))
            {
                refi[REFP_0][cnt] = 0;
                mvp[REFP_0][cnt][MV_X] = tmvp[REFP_0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = tmvp[REFP_0][MV_Y];
            }
            else
            {
                refi[REFP_0][cnt] = REFI_INVALID;
                mvp[REFP_0][cnt][MV_X] = 0;
                mvp[REFP_0][cnt][MV_Y] = 0;
            }
            if (((availablePredIdx == 2) || (availablePredIdx == 3)) && check_bi_applicability(slice_type, cuw, cuh))
            {
                refi[REFP_1][cnt] = 0;
                mvp[REFP_1][cnt][MV_X] = tmvp[REFP_1][MV_X];
                mvp[REFP_1][cnt][MV_Y] = tmvp[REFP_1][MV_Y];
            }
            else
            {
                refi[REFP_1][cnt] = REFI_INVALID;
                mvp[REFP_1][cnt][MV_X] = 0;
                mvp[REFP_1][cnt][MV_Y] = 0;
            }

            tmvp_cnt_pos0 = cnt;
            if (availablePredIdx != 0)
            {
                check_redundancy(slice_type, mvp, refi, &cnt);
                cnt++;
                tmvp_cnt_pos1 = cnt;
                if (tmvp_cnt_pos1 == tmvp_cnt_pos0 + 1)
                    tmvp_added = 1;
                if (cnt >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))
                {
                    return;
                }
            }
        }
    }
#endif
#if M49023_ADMVP_IMPROVE
    if (cnt < (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))
#else
    if (cnt < (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP - 1))
#endif
    {
        // take the MV from last to first
#if M49023_ADMVP_IMPROVE
        for (k = 3; k <= min(history_buffer.currCnt, ALLOWED_CHECKED_NUM); k += 4)
#else
        for (k = 3; k <= min(history_buffer.currCnt, small_cu ? ALLOWED_CHECKED_NUM - (MAX_NUM_MVP - MAX_NUM_MVP_SMALL_CU) * 4 : ALLOWED_CHECKED_NUM); k += 4)
#endif
            //        small size: for (k = 3; k <= min(history_buffer.currCnt, ALLOWED_CHECKED_NUM - 2 * 4 ); k += 4)
            //        large size: for (k = 3; k <= min(history_buffer.currCnt, ALLOWED_CHECKED_NUM); k += 4)
        {
            if (slice_type == SLICE_P)
            {
                refi[REFP_0][cnt] = history_buffer.history_refi_table[history_buffer.currCnt - k][REFP_0];
                mvp[REFP_0][cnt][MV_X] = history_buffer.history_mv_table[history_buffer.currCnt - k][REFP_0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = history_buffer.history_mv_table[history_buffer.currCnt - k][REFP_0][MV_Y];

                check_redundancy(slice_type, mvp, refi, &cnt);
                cnt++;
                if (cnt >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))
                {
                    return;
                }
            }

            if (slice_type == SLICE_B)
            {
                refi[REFP_0][cnt] = history_buffer.history_refi_table[history_buffer.currCnt - k][REFP_0];
                mvp[REFP_0][cnt][MV_X] = history_buffer.history_mv_table[history_buffer.currCnt - k][REFP_0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = history_buffer.history_mv_table[history_buffer.currCnt - k][REFP_0][MV_Y];

                if (!check_bi_applicability(slice_type, cuw, cuh))
                {
                    refi[REFP_1][cnt] = REFI_INVALID;
                    mvp[REFP_1][cnt][MV_X] = 0;
                    mvp[REFP_1][cnt][MV_Y] = 0;
                }
                else
                {
                    refi[REFP_1][cnt] = history_buffer.history_refi_table[history_buffer.currCnt - k][REFP_1];
                    mvp[REFP_1][cnt][MV_X] = history_buffer.history_mv_table[history_buffer.currCnt - k][REFP_1][MV_X];
                    mvp[REFP_1][cnt][MV_Y] = history_buffer.history_mv_table[history_buffer.currCnt - k][REFP_1][MV_Y];
                }

                check_redundancy(slice_type, mvp, refi, &cnt);
                cnt++;
                if (cnt >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))

                {
                    return;
                }
            }
        }
    }
    // B slice mv combination
    if (check_bi_applicability(slice_type, cuw, cuh))
    {
#if !M49023_ADMVP_IMPROVE
        for (z = 1; z <= 2 * (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP) - 3; z++)
        {
            for (s = z / 2; s >= 0; s--)
            {
                if ((s == z - s) || (s >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP)) || ((z - s) >= (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP)))

                {
                    continue;
                }
                priority_list0[c_win] = s;
                priority_list1[c_win] = z - s;
                c_win++;
                priority_list0[c_win] = z - s;
                priority_list1[c_win] = s;
                c_win++;
            }
        }
#endif
#if M49023_ADMVP_IMPROVE
        int priority_list0[MAX_NUM_MVP*MAX_NUM_MVP] = { 0, 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3, 0, 4, 1, 4, 2, 4, 3, 4 };
        int priority_list1[MAX_NUM_MVP*MAX_NUM_MVP] = { 1, 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2, 4, 0, 4, 1, 4, 2, 4, 3 };
#endif
        cur_num = cnt;
        for (i = 0; i < cur_num*(cur_num - 1) && cnt != (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP); i++)
        {
            idx0 = priority_list0[i];
            idx1 = priority_list1[i];

            if (REFI_IS_VALID(refi[REFP_0][idx0]) && REFI_IS_VALID(refi[REFP_1][idx1]))
            {
                refi[REFP_0][cnt] = refi[REFP_0][idx0];
                mvp[REFP_0][cnt][MV_X] = mvp[REFP_0][idx0][MV_X];
                mvp[REFP_0][cnt][MV_Y] = mvp[REFP_0][idx0][MV_Y];

                refi[REFP_1][cnt] = refi[REFP_1][idx1];
                mvp[REFP_1][cnt][MV_X] = mvp[REFP_1][idx1][MV_X];
                mvp[REFP_1][cnt][MV_Y] = mvp[REFP_1][idx1][MV_Y];

                check_redundancy(slice_type, mvp, refi, &cnt);
                cnt++;
            }
        }
        if (cnt == (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP))
        {
            return;
        }
    }

    for (k = cnt; k < (small_cu ? MAX_NUM_MVP_SMALL_CU : MAX_NUM_MVP); k++)
    {
        refi[REFP_0][k] = 0;
        mvp[REFP_0][k][MV_X] = 0;
        mvp[REFP_0][k][MV_Y] = 0;
        if (!check_bi_applicability(slice_type, cuw, cuh))
        {
            refi[REFP_1][k] = REFI_INVALID;
            mvp[REFP_1][k][MV_X] = 0;
            mvp[REFP_1][k][MV_Y] = 0;
        }
        else
        {
            refi[REFP_1][k] = 0;
            mvp[REFP_1][k][MV_X] = 0;
            mvp[REFP_1][k][MV_Y] = 0;
        }
    }
}
void evc_get_motion_skip_baseline(int slice_type, int scup, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], EVC_REFP refp[REFP_NUM], int cuw, int cuh, int w_scu, s8 refi[REFP_NUM][MAX_NUM_MVP], s16 mvp[REFP_NUM][MAX_NUM_MVP][MV_D], u16 avail_lr)
{
#if ADMVP
    evc_mset(mvp, 0, MAX_NUM_MVP * REFP_NUM * MV_D * sizeof(s16));
    evc_mset(refi, REFI_INVALID, MAX_NUM_MVP * REFP_NUM * sizeof(s8));
    evc_get_motion(scup, REFP_0, map_refi, map_mv, (EVC_REFP(*)[2])refp, cuw, cuh, w_scu, avail_lr, refi[REFP_0], mvp[REFP_0]);
    if (slice_type == SLICE_B)
    {
        evc_get_motion(scup, REFP_1, map_refi, map_mv, (EVC_REFP(*)[2])refp, cuw, cuh, w_scu, avail_lr, refi[REFP_1], mvp[REFP_1]);
    }
#endif
}
#endif

#if ADMVP        
void evc_clip_mv_pic(int x, int y, int maxX, int maxY, s16 mvp[REFP_NUM][MV_D])
{
    int minXY = - PIC_PAD_SIZE_L;
    mvp[REFP_0][MV_X] = (x + mvp[REFP_0][MV_X]) < minXY ? -(x + minXY) : mvp[REFP_0][MV_X];
    mvp[REFP_1][MV_X] = (x + mvp[REFP_1][MV_X]) < minXY ? -(x + minXY) : mvp[REFP_1][MV_X];
    mvp[REFP_0][MV_Y] = (y + mvp[REFP_0][MV_Y]) < minXY ? -(y + minXY) : mvp[REFP_0][MV_Y];
    mvp[REFP_1][MV_Y] = (y + mvp[REFP_1][MV_Y]) < minXY ? -(y + minXY) : mvp[REFP_1][MV_Y];

    mvp[REFP_0][MV_X] = (x + mvp[REFP_0][MV_X]) > maxX ? (maxX - x) : mvp[REFP_0][MV_X];
    mvp[REFP_1][MV_X] = (x + mvp[REFP_1][MV_X]) > maxX ? (maxX - x) : mvp[REFP_1][MV_X];
    mvp[REFP_0][MV_Y] = (y + mvp[REFP_0][MV_Y]) > maxY ? (maxY - y) : mvp[REFP_0][MV_Y];
    mvp[REFP_1][MV_Y] = (y + mvp[REFP_1][MV_Y]) > maxY ? (maxY - y) : mvp[REFP_1][MV_Y];
}

void evc_get_mv_dir(EVC_REFP refp[REFP_NUM], u32 ptr, int scup, int c_scu, u16 w_scu, u16 h_scu, s16 mvp[REFP_NUM][MV_D]
#if ADMVP
                    , s8 *refidx
#endif
                    , int sps_admvp_flag
)
#else
void evc_get_mv_dir(EVC_REFP refp[REFP_NUM], u32 ptr, int scup, u16 w_scu, s16 mvp[REFP_NUM][MV_D]
#if ADMVP
                    , s8 *refidx
#endif
)
#endif
{
#if ADMVP                            
    int maxX = PIC_PAD_SIZE_L + (w_scu << MIN_CU_LOG2) - 1;
    int maxY = PIC_PAD_SIZE_L + (h_scu << MIN_CU_LOG2) - 1;
    int x = (c_scu % w_scu) << MIN_CU_LOG2;
    int y = (c_scu / w_scu) << MIN_CU_LOG2;
#endif
    s16 mvc[MV_D];
    int dptr_co, dptr_L0, dptr_L1;

    mvc[MV_X] = refp[REFP_1].map_mv[scup][0][MV_X];
    mvc[MV_Y] = refp[REFP_1].map_mv[scup][0][MV_Y];
#if ADMVP
    *refidx = refp[REFP_1].map_refi[scup][0];
#endif

    dptr_co = refp[REFP_1].ptr - refp[REFP_1].list_ptr[0];
    dptr_L0 = ptr - refp[REFP_0].ptr;
    dptr_L1 = refp[REFP_1].ptr - ptr;

    if(dptr_co == 0)
    {
        mvp[REFP_0][MV_X] = 0;
        mvp[REFP_0][MV_Y] = 0;
        mvp[REFP_1][MV_X] = 0;
        mvp[REFP_1][MV_Y] = 0;
    }
    else
#if ADMVP
    {
        if(sps_admvp_flag == 1)
        {
            if((dptr_L0 & (dptr_L0 - 1)) == 0 && (dptr_L1 & (dptr_L1 - 1)) == 0 && (dptr_co & (dptr_co - 1)) == 0)
            {
                mvp[REFP_0][MV_X] = dptr_L0 * mvc[MV_X] / dptr_co;
                mvp[REFP_0][MV_Y] = dptr_L0 * mvc[MV_Y] / dptr_co;
                mvp[REFP_1][MV_X] = -dptr_L1 * mvc[MV_X] / dptr_co;
                mvp[REFP_1][MV_Y] = -dptr_L1 * mvc[MV_Y] / dptr_co;
#if ADMVP
                evc_clip_mv_pic(x, y, maxX, maxY, mvp);
#endif
            }
            else
            {
                mvp[REFP_0][MV_X] = 0;
                mvp[REFP_0][MV_Y] = 0;
                mvp[REFP_1][MV_X] = 0;
                mvp[REFP_1][MV_Y] = 0;
                *refidx = REFI_INVALID;
            }
        }
        else
        {
            mvp[REFP_0][MV_X] = dptr_L0 * mvc[MV_X] / dptr_co;
            mvp[REFP_0][MV_Y] = dptr_L0 * mvc[MV_Y] / dptr_co;
            mvp[REFP_1][MV_X] = -dptr_L1 * mvc[MV_X] / dptr_co;
            mvp[REFP_1][MV_Y] = -dptr_L1 * mvc[MV_Y] / dptr_co;
#if ADMVP && 0 // is this a fix?
            evc_clip_mv_pic(x, y, maxX, maxY, mvp);
#endif
        }
    }
#else
    {
        mvp[REFP_0][MV_X] = dptr_L0 * mvc[MV_X] / dptr_co;
        mvp[REFP_0][MV_Y] = dptr_L0 * mvc[MV_Y] / dptr_co;
        mvp[REFP_1][MV_X] = -dptr_L1 * mvc[MV_X] / dptr_co;
        mvp[REFP_1][MV_Y] = -dptr_L1 * mvc[MV_Y] / dptr_co;
#if ADMVP
        evc_clip_mv_pic(x, y, maxX, maxY, mvp);
#endif
    }
#endif
}

int evc_get_avail_cu(int neb_scua[MAX_NEB2], u32 * map_cu)
{
    int slice_num_x;
    u16 avail_cu = 0;

    evc_assert(neb_scua[NEB_X] >= 0);

    slice_num_x = MCU_GET_SN(map_cu[neb_scua[NEB_X]]);

    /* left */
    if(neb_scua[NEB_A] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_A]])))
    {
        avail_cu |= AVAIL_LE;
    }
    /* up */
    if(neb_scua[NEB_B] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_B]])))
    {
        avail_cu |= AVAIL_UP;
    }
    /* up-right */
    if(neb_scua[NEB_C] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_C]])))
    {
        if(MCU_GET_COD(map_cu[neb_scua[NEB_C]]))
        {
            avail_cu |= AVAIL_UP_RI;
        }
    }
    /* up-left */
    if(neb_scua[NEB_D] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_D]])))
    {
        avail_cu |= AVAIL_UP_LE;
    }
    /* low-left */
    if(neb_scua[NEB_E] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_E]])))
    {
        if(MCU_GET_COD(map_cu[neb_scua[NEB_E]]))
        {
            avail_cu |= AVAIL_LO_LE;
        }
    }

    /* right */
    if(neb_scua[NEB_H] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_H]])))
    {
        avail_cu |= AVAIL_RI;
    }
    /* low-right */
    if(neb_scua[NEB_I] >= 0 &&
       (slice_num_x == MCU_GET_SN(map_cu[neb_scua[NEB_I]])))
    {
        if(MCU_GET_COD(map_cu[neb_scua[NEB_I]]))
        {
            avail_cu |= AVAIL_LO_RI;
        }
    }

    return avail_cu;
}

u16 evc_get_avail_inter(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int cuw, int cuh, u32 * map_scu)
{
    u16 avail = 0;
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;

    if(x_scu > 0 && !MCU_GET_IF(map_scu[scup - 1]) && MCU_GET_COD(map_scu[scup - 1])
#if IBC
      && !MCU_GET_IBC(map_scu[scup - 1])
#endif
      )
    {
        SET_AVAIL(avail, AVAIL_LE);

        if(y_scu + scuh < h_scu  && MCU_GET_COD(map_scu[scup + (scuh * w_scu) - 1]) && !MCU_GET_IF(map_scu[scup + (scuh * w_scu) - 1])
#if IBC
          && !MCU_GET_IBC(map_scu[scup + (scuh * w_scu) - 1])
#endif
          )
        {
            SET_AVAIL(avail, AVAIL_LO_LE);
        }
    }

    if(y_scu > 0)
    {
        if(!MCU_GET_IF(map_scu[scup - w_scu])
#if IBC
          && !MCU_GET_IBC(map_scu[scup - w_scu])
#endif
          )
        {
            SET_AVAIL(avail, AVAIL_UP);
        }

        if(!MCU_GET_IF(map_scu[scup - w_scu + scuw - 1])
#if IBC
          && !MCU_GET_IBC(map_scu[scup - w_scu + scuw - 1])
#endif
          )
        {
            SET_AVAIL(avail, AVAIL_RI_UP);
        }

        if(x_scu > 0 && !MCU_GET_IF(map_scu[scup - w_scu - 1]) && MCU_GET_COD(map_scu[scup - w_scu - 1])
#if IBC
          && !MCU_GET_IBC(map_scu[scup - w_scu - 1])
#endif
          )
        {
            SET_AVAIL(avail, AVAIL_UP_LE);
        }
        // xxu check??
        if(x_scu + scuw < w_scu  && MCU_IS_COD_NIF(map_scu[scup - w_scu + scuw]) && MCU_GET_COD(map_scu[scup - w_scu + scuw]))
        {
            SET_AVAIL(avail, AVAIL_UP_RI);
        }
    }

    if(x_scu + scuw < w_scu && !MCU_GET_IF(map_scu[scup + scuw]) && MCU_GET_COD(map_scu[scup + scuw])
#if IBC
      && !MCU_GET_IBC(map_scu[scup + scuw])
#endif
      )
    {
        SET_AVAIL(avail, AVAIL_RI);

        if(y_scu + scuh < h_scu  && MCU_GET_COD(map_scu[scup + (scuh * w_scu) + scuw]) && !MCU_GET_IF(map_scu[scup + (scuh * w_scu) + scuw])
#if IBC
          && !MCU_GET_IBC(map_scu[scup + (scuh * w_scu) + scuw])
#endif
          )
        {
            SET_AVAIL(avail, AVAIL_LO_RI);
        }
    }

    return avail;
}

u16 evc_get_avail_intra(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int log2_cuw, int log2_cuh, u32 * map_scu)
{
    u16 avail = 0;
    int log2_scuw, log2_scuh, scuw, scuh;

    log2_scuw = log2_cuw - MIN_CU_LOG2;
    log2_scuh = log2_cuh - MIN_CU_LOG2;
    scuw = 1 << log2_scuw;
    scuh = 1 << log2_scuh;

    if(x_scu > 0 && MCU_GET_COD(map_scu[scup - 1]))
    {
        SET_AVAIL(avail, AVAIL_LE);

        if(y_scu + scuh + scuw - 1 < h_scu  && MCU_GET_COD(map_scu[scup + (w_scu * (scuw + scuh)) - w_scu - 1]))
        {
            SET_AVAIL(avail, AVAIL_LO_LE);
        }
    }

    if(y_scu > 0)
    {
        SET_AVAIL(avail, AVAIL_UP);
        SET_AVAIL(avail, AVAIL_RI_UP);

        if(x_scu > 0 && MCU_GET_COD(map_scu[scup - w_scu - 1]))
        {
            SET_AVAIL(avail, AVAIL_UP_LE);
        }

        if(x_scu + scuw < w_scu  && MCU_GET_COD(map_scu[scup - w_scu + scuw]))
        {
            SET_AVAIL(avail, AVAIL_UP_RI);
        }
    }

    if(x_scu + scuw < w_scu && MCU_GET_COD(map_scu[scup + scuw]))
    {
        SET_AVAIL(avail, AVAIL_RI);

        if(y_scu + scuh + scuw - 1 < h_scu  && MCU_GET_COD(map_scu[scup + (w_scu * (scuw + scuh - 1)) + scuw]))
        {
            SET_AVAIL(avail, AVAIL_LO_RI);
        }
    }

    return avail;
}
#if IBC
u16 evc_get_avail_ibc(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int cuw, int cuh, u32 * map_scu)
{
  u16 avail = 0;
  int scuw = cuw >> MIN_CU_LOG2;
  int scuh = cuh >> MIN_CU_LOG2;

  if (x_scu > 0 && MCU_GET_IBC(map_scu[scup - 1]) && MCU_GET_COD(map_scu[scup - 1]))
  {
    SET_AVAIL(avail, AVAIL_LE);

    if (y_scu + scuh < h_scu  && MCU_GET_COD(map_scu[scup + (scuh * w_scu) - 1]) && MCU_GET_IBC(map_scu[scup + (scuh * w_scu) - 1]))
    {
      SET_AVAIL(avail, AVAIL_LO_LE);
    }
  }

  if (y_scu > 0)
  {
    if (MCU_GET_IBC(map_scu[scup - w_scu]))
    {
      SET_AVAIL(avail, AVAIL_UP);
    }

    if (MCU_GET_IBC(map_scu[scup - w_scu + scuw - 1]))
    {
      SET_AVAIL(avail, AVAIL_RI_UP);
    }

    if (x_scu > 0 && MCU_GET_IBC(map_scu[scup - w_scu - 1]) && MCU_GET_COD(map_scu[scup - w_scu - 1]))
    {
      SET_AVAIL(avail, AVAIL_UP_LE);
    }

    if (x_scu + scuw < w_scu  && MCU_IS_COD_NIF(map_scu[scup - w_scu + scuw]) && MCU_GET_COD(map_scu[scup - w_scu + scuw]))
    {
      SET_AVAIL(avail, AVAIL_UP_RI);
    }
  }

  if (x_scu + scuw < w_scu && MCU_GET_IBC(map_scu[scup + scuw]) && MCU_GET_COD(map_scu[scup + scuw]))
  {
    SET_AVAIL(avail, AVAIL_RI);

    if (y_scu + scuh < h_scu  && MCU_GET_COD(map_scu[scup + (scuh * w_scu) + scuw]) && MCU_GET_IBC(map_scu[scup + (scuh * w_scu) + scuw]))
    {
      SET_AVAIL(avail, AVAIL_LO_RI);
    }
  }

  return avail;
}
#endif

int evc_picbuf_signature(EVC_PIC *pic, u8 *signature)
{
    return evc_md5_imgb(pic->imgb, signature);
}

/* MD5 functions */
#define MD5FUNC(f, w, x, y, z, msg1, s,msg2 )  ( w += f(x, y, z) + msg1 + msg2,  w = w<<s | w>>(32-s),  w += x )
#define FF(x, y, z) (z ^ (x & (y ^ z)))
#define GG(x, y, z) (y ^ (z & (x ^ y)))
#define HH(x, y, z) (x ^ y ^ z)
#define II(x, y, z) (y ^ (x | ~z))

static void evc_md5_trans(u32 *buf, u32 *msg)
{
    register u32 a, b, c, d;

    a = buf[0];
    b = buf[1];
    c = buf[2];
    d = buf[3];

    MD5FUNC(FF, a, b, c, d, msg[ 0],  7, 0xd76aa478); /* 1 */
    MD5FUNC(FF, d, a, b, c, msg[ 1], 12, 0xe8c7b756); /* 2 */
    MD5FUNC(FF, c, d, a, b, msg[ 2], 17, 0x242070db); /* 3 */
    MD5FUNC(FF, b, c, d, a, msg[ 3], 22, 0xc1bdceee); /* 4 */

    MD5FUNC(FF, a, b, c, d, msg[ 4],  7, 0xf57c0faf); /* 5 */
    MD5FUNC(FF, d, a, b, c, msg[ 5], 12, 0x4787c62a); /* 6 */
    MD5FUNC(FF, c, d, a, b, msg[ 6], 17, 0xa8304613); /* 7 */
    MD5FUNC(FF, b, c, d, a, msg[ 7], 22, 0xfd469501); /* 8 */

    MD5FUNC(FF, a, b, c, d, msg[ 8],  7, 0x698098d8); /* 9 */
    MD5FUNC(FF, d, a, b, c, msg[ 9], 12, 0x8b44f7af); /* 10 */
    MD5FUNC(FF, c, d, a, b, msg[10], 17, 0xffff5bb1); /* 11 */
    MD5FUNC(FF, b, c, d, a, msg[11], 22, 0x895cd7be); /* 12 */

    MD5FUNC(FF, a, b, c, d, msg[12],  7, 0x6b901122); /* 13 */
    MD5FUNC(FF, d, a, b, c, msg[13], 12, 0xfd987193); /* 14 */
    MD5FUNC(FF, c, d, a, b, msg[14], 17, 0xa679438e); /* 15 */
    MD5FUNC(FF, b, c, d, a, msg[15], 22, 0x49b40821); /* 16 */

    /* Round 2 */
    MD5FUNC(GG, a, b, c, d, msg[ 1],  5, 0xf61e2562); /* 17 */
    MD5FUNC(GG, d, a, b, c, msg[ 6],  9, 0xc040b340); /* 18 */
    MD5FUNC(GG, c, d, a, b, msg[11], 14, 0x265e5a51); /* 19 */
    MD5FUNC(GG, b, c, d, a, msg[ 0], 20, 0xe9b6c7aa); /* 20 */

    MD5FUNC(GG, a, b, c, d, msg[ 5],  5, 0xd62f105d); /* 21 */
    MD5FUNC(GG, d, a, b, c, msg[10],  9,  0x2441453); /* 22 */
    MD5FUNC(GG, c, d, a, b, msg[15], 14, 0xd8a1e681); /* 23 */
    MD5FUNC(GG, b, c, d, a, msg[ 4], 20, 0xe7d3fbc8); /* 24 */

    MD5FUNC(GG, a, b, c, d, msg[ 9],  5, 0x21e1cde6); /* 25 */
    MD5FUNC(GG, d, a, b, c, msg[14],  9, 0xc33707d6); /* 26 */
    MD5FUNC(GG, c, d, a, b, msg[ 3], 14, 0xf4d50d87); /* 27 */
    MD5FUNC(GG, b, c, d, a, msg[ 8], 20, 0x455a14ed); /* 28 */

    MD5FUNC(GG, a, b, c, d, msg[13],  5, 0xa9e3e905); /* 29 */
    MD5FUNC(GG, d, a, b, c, msg[ 2],  9, 0xfcefa3f8); /* 30 */
    MD5FUNC(GG, c, d, a, b, msg[ 7], 14, 0x676f02d9); /* 31 */
    MD5FUNC(GG, b, c, d, a, msg[12], 20, 0x8d2a4c8a); /* 32 */

    /* Round 3 */
    MD5FUNC(HH, a, b, c, d, msg[ 5],  4, 0xfffa3942); /* 33 */
    MD5FUNC(HH, d, a, b, c, msg[ 8], 11, 0x8771f681); /* 34 */
    MD5FUNC(HH, c, d, a, b, msg[11], 16, 0x6d9d6122); /* 35 */
    MD5FUNC(HH, b, c, d, a, msg[14], 23, 0xfde5380c); /* 36 */

    MD5FUNC(HH, a, b, c, d, msg[ 1],  4, 0xa4beea44); /* 37 */
    MD5FUNC(HH, d, a, b, c, msg[ 4], 11, 0x4bdecfa9); /* 38 */
    MD5FUNC(HH, c, d, a, b, msg[ 7], 16, 0xf6bb4b60); /* 39 */
    MD5FUNC(HH, b, c, d, a, msg[10], 23, 0xbebfbc70); /* 40 */

    MD5FUNC(HH, a, b, c, d, msg[13],  4, 0x289b7ec6); /* 41 */
    MD5FUNC(HH, d, a, b, c, msg[ 0], 11, 0xeaa127fa); /* 42 */
    MD5FUNC(HH, c, d, a, b, msg[ 3], 16, 0xd4ef3085); /* 43 */
    MD5FUNC(HH, b, c, d, a, msg[ 6], 23,  0x4881d05); /* 44 */

    MD5FUNC(HH, a, b, c, d, msg[ 9],  4, 0xd9d4d039); /* 45 */
    MD5FUNC(HH, d, a, b, c, msg[12], 11, 0xe6db99e5); /* 46 */
    MD5FUNC(HH, c, d, a, b, msg[15], 16, 0x1fa27cf8); /* 47 */
    MD5FUNC(HH, b, c, d, a, msg[ 2], 23, 0xc4ac5665); /* 48 */

    /* Round 4 */
    MD5FUNC(II, a, b, c, d, msg[ 0],  6, 0xf4292244); /* 49 */
    MD5FUNC(II, d, a, b, c, msg[ 7], 10, 0x432aff97); /* 50 */
    MD5FUNC(II, c, d, a, b, msg[14], 15, 0xab9423a7); /* 51 */
    MD5FUNC(II, b, c, d, a, msg[ 5], 21, 0xfc93a039); /* 52 */

    MD5FUNC(II, a, b, c, d, msg[12],  6, 0x655b59c3); /* 53 */
    MD5FUNC(II, d, a, b, c, msg[ 3], 10, 0x8f0ccc92); /* 54 */
    MD5FUNC(II, c, d, a, b, msg[10], 15, 0xffeff47d); /* 55 */
    MD5FUNC(II, b, c, d, a, msg[ 1], 21, 0x85845dd1); /* 56 */

    MD5FUNC(II, a, b, c, d, msg[ 8],  6, 0x6fa87e4f); /* 57 */
    MD5FUNC(II, d, a, b, c, msg[15], 10, 0xfe2ce6e0); /* 58 */
    MD5FUNC(II, c, d, a, b, msg[ 6], 15, 0xa3014314); /* 59 */
    MD5FUNC(II, b, c, d, a, msg[13], 21, 0x4e0811a1); /* 60 */

    MD5FUNC(II, a, b, c, d, msg[ 4],  6, 0xf7537e82); /* 61 */
    MD5FUNC(II, d, a, b, c, msg[11], 10, 0xbd3af235); /* 62 */
    MD5FUNC(II, c, d, a, b, msg[ 2], 15, 0x2ad7d2bb); /* 63 */
    MD5FUNC(II, b, c, d, a, msg[ 9], 21, 0xeb86d391); /* 64 */

    buf[0] += a;
    buf[1] += b;
    buf[2] += c;
    buf[3] += d;
}

void evc_md5_init(EVC_MD5 *md5)
{
    md5->h[0] = 0x67452301;
    md5->h[1] = 0xefcdab89;
    md5->h[2] = 0x98badcfe;
    md5->h[3] = 0x10325476;

    md5->bits[0] = 0;
    md5->bits[1] = 0;
}

void evc_md5_update(EVC_MD5 *md5, void *buf_t, u32 len)
{
    u8 *buf;
    u32 i, idx, part_len;

    buf = (u8*)buf_t;

    idx = (u32)((md5->bits[0] >> 3) & 0x3f);

    md5->bits[0] += (len << 3);
    if(md5->bits[0] < (len << 3))
    {
        (md5->bits[1])++;
    }

    md5->bits[1] += (len >> 29);
    part_len = 64 - idx;

    if(len >= part_len)
    {
        evc_mcpy(md5->msg + idx, buf, part_len);
        evc_md5_trans(md5->h, (u32 *)md5->msg);

        for(i = part_len; i + 63 < len; i += 64)
        {
            evc_md5_trans(md5->h, (u32 *)(buf + i));
        }
        idx = 0;
    }
    else
    {
        i = 0;
    }

    if(len - i > 0)
    {
        evc_mcpy(md5->msg + idx, buf + i, len - i);
    }
}

void evc_md5_update_16(EVC_MD5 *md5, void *buf_t, u32 len)
{
    u16 *buf;
    u32 i, idx, part_len, j;
    u8 t[512];

    buf = (u16 *)buf_t;
    idx = (u32)((md5->bits[0] >> 3) & 0x3f);

    len = len * 2;
    for(j = 0; j < len; j += 2)
    {
        t[j] = (u8)(*(buf));
        t[j + 1] = *(buf) >> 8;
        buf++;
    }

    md5->bits[0] += (len << 3);
    if(md5->bits[0] < (len << 3))
    {
        (md5->bits[1])++;
    }

    md5->bits[1] += (len >> 29);
    part_len = 64 - idx;

    if(len >= part_len)
    {
        evc_mcpy(md5->msg + idx, t, part_len);
        evc_md5_trans(md5->h, (u32 *)md5->msg);

        for(i = part_len; i + 63 < len; i += 64)
        {
            evc_md5_trans(md5->h, (u32 *)(t + i));
        }
        idx = 0;
    }
    else
    {
        i = 0;
    }

    if(len - i > 0)
    {
        evc_mcpy(md5->msg + idx, t + i, len - i);
    }
}

void evc_md5_finish(EVC_MD5 *md5, u8 digest[16])
{
    u8 *pos;
    int cnt;

    cnt = (md5->bits[0] >> 3) & 0x3F;
    pos = md5->msg + cnt;
    *pos++ = 0x80;
    cnt = 64 - 1 - cnt;

    if(cnt < 8)
    {
        evc_mset(pos, 0, cnt);
        evc_md5_trans(md5->h, (u32 *)md5->msg);
        evc_mset(md5->msg, 0, 56);
    }
    else
    {
        evc_mset(pos, 0, cnt - 8);
    }

    evc_mcpy((md5->msg + 14 * sizeof(u32)), &md5->bits[0], sizeof(u32));
    evc_mcpy((md5->msg + 15 * sizeof(u32)), &md5->bits[1], sizeof(u32));

    evc_md5_trans(md5->h, (u32 *)md5->msg);
    evc_mcpy(digest, md5->h, 16);
    evc_mset(md5, 0, sizeof(EVC_MD5));
}

int evc_md5_imgb(EVC_IMGB *imgb, u8 digest[16])
{
    EVC_MD5 md5;
    int i, j;

    evc_md5_init(&md5);

    if(EVC_COLORSPACE_IS_YUV_PLANAR(imgb->cs))
    {
        for(i = 0; i < imgb->np; i++)
        {
            for(j = imgb->y[i]; j < imgb->h[i]; j++)
            {
                evc_md5_update(&md5, ((u8 *)imgb->a[i]) + j*imgb->s[i] +
                                PEL2BYTE(imgb->x[i])
                                , imgb->w[i] * 2
                                );
            }
        }
    }
    else
    {
        evc_assert_rv(0, EVC_ERR_UNSUPPORTED_COLORSPACE);
    }
    evc_md5_finish(&md5, digest);
    return EVC_OK;
}

void init_scan(u16 *scan, int size_x, int size_y, int scan_type)
{
    int x, y, l, pos, num_line;

    pos = 0;
    num_line = size_x + size_y - 1;

    if(scan_type == COEF_SCAN_ZIGZAG)
    {
        /* starting point */
        scan[pos] = 0;
        pos++;

        /* loop */
        for(l = 1; l < num_line; l++)
        {
            if(l % 2) /* decreasing loop */
            {
                x = EVC_MIN(l, size_x - 1);
                y = EVC_MAX(0, l - (size_x - 1));

                while(x >= 0 && y < size_y)
                {
                    scan[pos] = y * size_x + x;
                    pos++;
                    x--;
                    y++;
                }
            }
            else /* increasing loop */
            {
                y = EVC_MIN(l, size_y - 1);
                x = EVC_MAX(0, l - (size_y - 1));
                while(y >= 0 && x < size_x)
                {
                    scan[pos] = y * size_x + x;
                    pos++;
                    x++;
                    y--;
                }
            }
        }
    }
}

int evc_scan_tbl_init()
{
    int x, y, scan_type;
    int size_y, size_x;

    for(scan_type = 0; scan_type < COEF_SCAN_TYPE_NUM; scan_type++)
    {
        if (scan_type != COEF_SCAN_ZIGZAG)
            continue;
        for(y = 0; y < MAX_CU_LOG2 - 1; y++)
        {
            size_y = 1 << (y + 1);
            for(x = 0; x < MAX_CU_LOG2 - 1; x++)
            {
                size_x = 1 << (x + 1);
                evc_scan_tbl[scan_type][x][y] = (u16*)evc_malloc_fast(size_y * size_x * sizeof(u16));
                init_scan(evc_scan_tbl[scan_type][x][y], size_x, size_y, scan_type);
#if ADCC
                evc_inv_scan_tbl[scan_type][x][y] = (u16*)evc_malloc_fast(size_y * size_x * sizeof(u16));
                evc_init_inverse_scan_sr(evc_inv_scan_tbl[scan_type][x][y], evc_scan_tbl[scan_type][x][y], size_x, size_y, scan_type);
#endif
            }
        }
    }
    return EVC_OK;
}

int evc_scan_tbl_delete()
{
    int x, y, scan_type;

    for(scan_type = 0; scan_type < COEF_SCAN_TYPE_NUM; scan_type++)
    {
        for(y = 0; y < MAX_CU_LOG2 - 1; y++)
        {
            for(x = 0; x < MAX_CU_LOG2 - 1; x++)
            {
                if(evc_scan_tbl[scan_type][x][y] != NULL)
                {
                    free(evc_scan_tbl[scan_type][x][y]);
                }
#if ADCC
                if (evc_inv_scan_tbl[scan_type][x][y] != NULL)
                {
                    free(evc_inv_scan_tbl[scan_type][x][y]);
                }
#endif
            }
        }
    }
    return EVC_OK;
}

int evc_get_split_mode(s8 *split_mode, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    int ret = EVC_OK;
    int pos = cup + (((cuh >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cuw >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cuw) - CONV_LOG2(cuh));

    if(cuw < 8 && cuh < 8)
    {
        *split_mode = NO_SPLIT;
        return ret;
    }

    *split_mode = split_mode_buf[cud][shape][pos];

    return ret;
}

int evc_set_split_mode(s8 split_mode, int cud, int cup, int cuw, int cuh, int lcu_s, s8 (*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    int ret = EVC_OK;
    int pos = cup + (((cuh >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cuw >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cuw) - CONV_LOG2(cuh));

    if(cuw < 8 && cuh < 8)
    {
        return ret;
    }

    split_mode_buf[cud][shape][pos] = split_mode;

    return ret;
}

void evc_check_split_mode(int *split_allow, int log2_cuw, int log2_cuh, int boundary, int boundary_b, int boundary_r, int log2_max_cuwh
                          , const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth
                          , int x, int y, int im_w, int im_h
                          , u8 *remaining_split, int sps_btt_flag
#if M50761_CHROMA_NOT_SPLIT
                          , TREE_CONS tree_cons
#endif
)
{
    if(!sps_btt_flag)
    {
        evc_mset(split_allow, 0, sizeof(int) * SPLIT_CHECK_NUM);
        split_allow[SPLIT_QUAD] = 1;
        return;
    }

    int log2_sub_cuw, log2_sub_cuh;
    int long_side, ratio;
    int cu_max, from_boundary_b, from_boundary_r;
    cu_max = 1 << (log2_max_cuwh - 1);
    from_boundary_b = (y >= im_h - im_h%cu_max) && !(x >= im_w - im_w % cu_max);
    from_boundary_r = (x >= im_w - im_w % cu_max) && !(y >= im_h - im_h%cu_max);

    evc_mset(split_allow, 0, sizeof(int) * SPLIT_CHECK_NUM);
    {
        split_allow[SPLIT_QUAD] = 0;

#if M50761_BOUNDARY_FORCE_CLEANUP
        if (boundary)
        {
            if (boundary_r)
            {
                // right side
                if (log2_cuh >= (2 + log2_cuw) )
                {
                    split_allow[SPLIT_BI_HOR] = 1;
                }
                else
                {
                    split_allow[SPLIT_BI_VER] = 1;
                }
            }
            else
            {
                // Bottom and right-bottom corner
                if (log2_cuw >= (2 + log2_cuh) )
                {
                    split_allow[SPLIT_BI_VER] = 1;
                }
                else
                {
                    split_allow[SPLIT_BI_HOR] = 1;
                }
            }
        }
#endif

        if(log2_cuw == log2_cuh)
        {
            if(boundary_b)
            {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                split_allow[SPLIT_BI_HOR] = 1;
#endif
            }
            else if(boundary_r)
            {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                split_allow[SPLIT_BI_VER] = 1;
#endif
            }
            else if(boundary && !boundary_b && !boundary_r)
            {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                split_allow[SPLIT_QUAD] = 1;
#endif
            }
            else
            {
                split_allow[SPLIT_BI_HOR] = ALLOW_SPLIT_RATIO(log2_cuw, 1);
                split_allow[SPLIT_BI_VER] = ALLOW_SPLIT_RATIO(log2_cuw, 1);
#if M50632_SIMPLIFICATION_TT
                split_allow[SPLIT_TRI_VER] = (log2_cuw >= log2_cuh) & ALLOW_SPLIT_TRI(log2_cuw);
                split_allow[SPLIT_TRI_HOR] = (log2_cuh >= log2_cuw) & ALLOW_SPLIT_TRI(log2_cuh);
#else                
                split_allow[SPLIT_TRI_VER] = ALLOW_SPLIT_RATIO(log2_cuw, 2) & ALLOW_SPLIT_TRI(log2_cuw);
                split_allow[SPLIT_TRI_HOR] = split_allow[SPLIT_TRI_VER];
#endif
            }
        }
        else
        {
            if(log2_cuw > log2_cuh)
            {
                if(boundary)
                {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                    log2_sub_cuw = log2_cuw - 1;
                    log2_sub_cuh = log2_cuh;
                    long_side = log2_sub_cuw > log2_sub_cuh ? log2_sub_cuw : log2_sub_cuh;
                    ratio = EVC_ABS(log2_sub_cuw - log2_sub_cuh);
                    if(boundary_b)
                    {
                        if (log2_cuw == 7 && log2_cuh == 6)
                        {
                            split_allow[SPLIT_BI_VER] = 1;
                        }
                        else
                        {
                            split_allow[SPLIT_BI_HOR] = 1;
                        }
                    }
                    else if(boundary_r)
                    {
                        split_allow[SPLIT_BI_VER] = 1;
                    }
                    else
                    {
                        assert(0);
                    }
#endif
                }
                else
                {
                    split_allow[SPLIT_BI_HOR] = ALLOW_SPLIT_RATIO(log2_cuw, log2_cuw - log2_cuh + 1);

                    log2_sub_cuw = log2_cuw - 1;
                    log2_sub_cuh = log2_cuh;
                    long_side = log2_sub_cuw > log2_sub_cuh ? log2_sub_cuw : log2_sub_cuh;
                    ratio = EVC_ABS(log2_sub_cuw - log2_sub_cuh);

                    split_allow[SPLIT_BI_VER] = ALLOW_SPLIT_RATIO(long_side, ratio);
                    if(from_boundary_b && (ratio == 3 || ratio == 4))
                        split_allow[SPLIT_BI_VER] = 1;
#if M50632_SIMPLIFICATION_TT
                    split_allow[SPLIT_TRI_VER] = (log2_cuw >= log2_cuh) & ALLOW_SPLIT_TRI(log2_cuw);
                    split_allow[SPLIT_TRI_HOR] = (log2_cuh >= log2_cuw) & ALLOW_SPLIT_TRI(log2_cuh);
#else
                    log2_sub_cuw = log2_cuw - 2;
                    log2_sub_cuh = log2_cuh;
                    long_side = log2_sub_cuw > log2_sub_cuh ? log2_sub_cuw : log2_sub_cuh;
                    ratio = EVC_ABS(log2_sub_cuw - log2_sub_cuh);

                    split_allow[SPLIT_TRI_VER] = ALLOW_SPLIT_RATIO(long_side, ratio) & ALLOW_SPLIT_TRI(log2_cuw);
                    split_allow[SPLIT_TRI_HOR] = 0;
#endif
                    if (from_boundary_b && (log2_cuw == 7 || ratio == 3))
                    {
                        split_allow[SPLIT_TRI_VER] = 1;
                    }
                }
            }
            else
            {
                if(boundary)
                {
#if !M50761_BOUNDARY_FORCE_CLEANUP
                    log2_sub_cuh = log2_cuh - 1;
                    log2_sub_cuw = log2_cuw;
                    long_side = log2_sub_cuw > log2_sub_cuh ? log2_sub_cuw : log2_sub_cuh;
                    ratio = EVC_ABS(log2_sub_cuw - log2_sub_cuh);
                    if(boundary_b)
                    {
                        split_allow[SPLIT_BI_HOR] = 1;
                    }
                    else if(boundary_r)
                    {
                        if (log2_cuw == 6 && log2_cuh == 7)
                        {
                            split_allow[SPLIT_BI_HOR] = 1;
                        }
                        else
                        {
                            split_allow[SPLIT_BI_VER] = 1;
                        }
                    }
                    else
                    {
                        assert(0);
                    }
#endif
                }
                else
                {
                    log2_sub_cuh = log2_cuh - 1;
                    log2_sub_cuw = log2_cuw;
                    long_side = log2_sub_cuw > log2_sub_cuh ? log2_sub_cuw : log2_sub_cuh;
                    ratio = EVC_ABS(log2_sub_cuw - log2_sub_cuh);

                    split_allow[SPLIT_BI_HOR] = ALLOW_SPLIT_RATIO(long_side, ratio);
                    if(from_boundary_r && (ratio == 3 || ratio == 4))
                        split_allow[SPLIT_BI_HOR] = 1;

                    split_allow[SPLIT_BI_VER] = ALLOW_SPLIT_RATIO(log2_cuh, log2_cuh - log2_cuw + 1);
#if M50632_SIMPLIFICATION_TT
                    split_allow[SPLIT_TRI_VER] = (log2_cuw >= log2_cuh) & ALLOW_SPLIT_TRI(log2_cuw);
                    split_allow[SPLIT_TRI_HOR] = (log2_cuh >= log2_cuw) & ALLOW_SPLIT_TRI(log2_cuh);
#else
                    split_allow[SPLIT_TRI_VER] = 0;

                    log2_sub_cuh = log2_cuh - 2;
                    log2_sub_cuw = log2_cuw;
                    long_side = log2_sub_cuw > log2_sub_cuh ? log2_sub_cuw : log2_sub_cuh;
                    ratio = EVC_ABS(log2_sub_cuw - log2_sub_cuh);

                    split_allow[SPLIT_TRI_HOR] = ALLOW_SPLIT_RATIO(long_side, ratio) & ALLOW_SPLIT_TRI(log2_cuh);
#endif
                    if (from_boundary_r && (log2_cuh == 7 || ratio == 3))
                    {
                        split_allow[SPLIT_TRI_HOR] = 1;
                    }
                }
            }
        }
    }
#if M50761_CHROMA_NOT_SPLIT
    if (tree_cons.mode_cons == eOnlyInter)
    {
        int cuw = 1 << log2_cuw;
        int cuh = 1 << log2_cuh;
        for (int mode = SPLIT_BI_VER; mode < SPLIT_QUAD; ++mode)
            split_allow[mode] &= evc_get_mode_cons_by_split(mode, cuw, cuh) == eAll;

        if (sps_btt_flag)
            split_allow[SPLIT_QUAD] &= evc_get_mode_cons_by_split(SPLIT_QUAD, cuw, cuh) == eAll;
    }
#endif
}

#if DQP
u8 * evc_get_dqp_used(int x_scu, int y_scu, int w_scu, u8 * map_dqp_used, int dqp_depth)
{
    u8 * dqp;
    int x_scu_pos, y_scu_pos, scup_pos;

    x_scu_pos = x_scu;
    y_scu_pos = y_scu;
    scup_pos = x_scu_pos + y_scu_pos * w_scu;
    dqp = &map_dqp_used[scup_pos];
    return  dqp;
}
#endif

int evc_get_suco_flag(s8* suco_flag, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*suco_flag_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    int ret = EVC_OK;
    int pos = cup + (((cuh >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cuw >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cuw) - CONV_LOG2(cuh));

    if(cuw < 8 && cuh < 8)
    {
        *suco_flag = 0;
        return ret;
    }

    *suco_flag = suco_flag_buf[cud][shape][pos];

    return ret;
}

int evc_set_suco_flag(s8  suco_flag, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*suco_flag_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    int ret = EVC_OK;
    int pos = cup + (((cuh >> 1) >> MIN_CU_LOG2) * (lcu_s >> MIN_CU_LOG2) + ((cuw >> 1) >> MIN_CU_LOG2));
    int shape = SQUARE + (CONV_LOG2(cuw) - CONV_LOG2(cuh));

    if(cuw < 8 && cuh < 8)
    {
        return ret;
    }

    suco_flag_buf[cud][shape][pos] = suco_flag;

    return ret;
}

u8 evc_check_suco_cond(int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, u8 suco_max_depth, u8 suco_depth)
{
    int suco_minsize = 1 << max((log2_max_cuwh - suco_max_depth - suco_depth), MIN_CU_LOG2);
#if M48879_IMPROVEMENT_ENC_OPT
    int suco_maxsize = 1 << min((log2_max_cuwh - suco_max_depth), 6);
#else
    int suco_maxsize = 1 << (log2_max_cuwh - suco_max_depth);
#endif

    if(EVC_MIN(cuw, cuh) < suco_minsize || EVC_MAX(cuw, cuh) > suco_maxsize)
    {
        return 0;
    }

    if(boundary)
    {
        return 0;
    }

    if(split_mode==NO_SPLIT || split_mode==SPLIT_BI_HOR || split_mode==SPLIT_TRI_HOR)
    {
        return 0;
    }

    if(split_mode != SPLIT_QUAD && cuw <= cuh)
    {
        return 0;
    }

    return 1;
}

u16 evc_check_nev_avail(int x_scu, int y_scu, int cuw, int cuh, int w_scu, int h_scu, u32 * map_scu)
{
    int scup = y_scu *  w_scu + x_scu;
    int scuw = cuw >> MIN_CU_LOG2;
    u16 avail_lr = 0;

    if(x_scu > 0 && MCU_GET_COD(map_scu[scup - 1]))
    {
        avail_lr+=1;
    }

    if(x_scu + scuw < w_scu && MCU_GET_COD(map_scu[scup+scuw]))
    {
        avail_lr+=2;
    }

    return avail_lr;
}

void evc_get_ctx_some_flags(int x_scu, int y_scu, int cuw, int cuh, int w_scu, u32* map_scu, u32* map_cu_mode, u8* ctx, u8 slice_type, int sps_cm_init_flag
#if IBC    
  , u8 ibc_flag, u8 ibc_log_max_size
#endif
)
{
    int nev_info[NUM_CNID][3];
    int scun[3], avail[3];
    int scup = y_scu * w_scu + x_scu;
    int scuw = cuw >> MIN_CU_LOG2, scuh = cuh >> MIN_CU_LOG2;
    int num_pos_avail;
    int i, j;
#if IBC
    if ((slice_type == SLICE_I && ibc_flag == 0)
      || (slice_type == SLICE_I && (cuw > (1 << ibc_log_max_size) || cuh > (1 << ibc_log_max_size))))
    {
      return;
    }
#else
    if(slice_type == SLICE_I)
    {
        return;
    }
#endif
    for(i = 0; i < NUM_CNID; i++)
    {
        nev_info[i][0] = nev_info[i][1] = nev_info[i][2] = 0;
        ctx[i] = 0;
    }

    scun[0] = scup - w_scu;
    scun[1] = scup - 1 + (scuh - 1)*w_scu;
    scun[2] = scup + scuw + (scuh - 1)*w_scu;
    avail[0] = y_scu == 0 ? 0 : MCU_GET_COD(map_scu[scun[0]]);
    avail[1] = x_scu == 0 ? 0 : MCU_GET_COD(map_scu[scun[1]]);
    avail[2] = x_scu + scuw >= w_scu ? 0 : MCU_GET_COD(map_scu[scun[2]]);

    num_pos_avail = 0;
    for(j = 0; j < 3; j++)
    {
        if(avail[j])
        {
#if IBC
          nev_info[CNID_SKIP_FLAG][j] = MCU_GET_SF(map_scu[scun[j]]);
          nev_info[CNID_PRED_MODE][j] = MCU_GET_IF(map_scu[scun[j]]);

          if (slice_type != SLICE_I)
          {
#if AFFINE
            nev_info[CNID_AFFN_FLAG][j] = MCU_GET_AFF(map_scu[scun[j]]);
#endif
          }

          if (ibc_flag == 1)
          {
            nev_info[CNID_IBC_FLAG][j] = MCU_GET_IBC(map_scu[scun[j]]);
          }
#else
            if(slice_type != SLICE_I)
            {
                nev_info[CNID_SKIP_FLAG][j] = MCU_GET_SF(map_scu[scun[j]]);
                nev_info[CNID_PRED_MODE][j] = MCU_GET_IF(map_scu[scun[j]]);
#if AFFINE
                nev_info[CNID_AFFN_FLAG][j] = MCU_GET_AFF(map_scu[scun[j]]);
#endif
            }
#endif
            num_pos_avail++;
        }
    }

    //decide ctx
    for(i = 0; i < NUM_CNID; i++)
    {
        if(num_pos_avail == 0)
        {
            ctx[i] = 0;
        }
        else
        {
            ctx[i] = nev_info[i][0] + nev_info[i][1] + nev_info[i][2];

            if(i == CNID_SKIP_FLAG)
            {
                if(sps_cm_init_flag == 1)
                {
                    ctx[i] = min(ctx[i], NUM_SBAC_CTX_SKIP_FLAG - 1);
                }
                else
                {
                    ctx[i] = 0;
                }
            }
#if IBC
            else if (i == CNID_IBC_FLAG)
            {
              if (sps_cm_init_flag == 1)
              {
                ctx[i] = min(ctx[i], NUM_SBAC_CTX_IBC_FLAG - 1);
              }
              else
              {
                ctx[i] = 0;
              }
            }
#endif
            else if(i == CNID_PRED_MODE)
            {
                if(sps_cm_init_flag == 1)
                {
                    ctx[i] = min(ctx[i], NUM_PRED_MODE_CTX - 1);
                }
                else
                {
                    ctx[i] = 0;
                }
            }
#if M50761_CHROMA_NOT_SPLIT
            else if (i == CNID_MODE_CONS)
            {
                if (sps_cm_init_flag == 1)
                {
                    ctx[i] = min(ctx[i], NUM_MODE_CONS_CTX - 1);
                }
                else
                {
                    ctx[i] = 0;
                }
            }
#endif
#if AFFINE
            else if(i == CNID_AFFN_FLAG)
            {
                if(sps_cm_init_flag == 1)
                {
                    ctx[i] = min(ctx[i], NUM_SBAC_CTX_AFFINE_FLAG - 1);
                }
                else
                {
                    ctx[i] = 0;
                }
            }
#endif
        }
    }
}

#if M48933_AFFINE
void evc_mv_rounding_s32( s32 hor, int ver, s32 * rounded_hor, s32 * rounded_ver, s32 right_shift, int left_shift )
{
    int offset = (right_shift > 0) ? (1 << (right_shift - 1)) : 0;
    *rounded_hor = ((hor + offset - (hor >= 0)) >> right_shift) << left_shift;
    *rounded_ver = ((ver + offset - (ver >= 0)) >> right_shift) << left_shift;
}

#if M50761_AFFINE_ADAPT_SUB_SIZE
void derive_affine_subblock_size_bi( s16 ac_mv[REFP_NUM][VER_NUM][MV_D], s8 refi[REFP_NUM], int cuw, int cuh, int *sub_w, int *sub_h, int vertex_num )
{
    int w = cuw;
    int h = cuh;
#if MC_PRECISION_ADD
    int mc_prec_add = MC_PRECISION_ADD;
#else
    int mc_prec_add = 0;
#endif
    int mv_wx, mv_wy;
    int l = 0;

    *sub_w = cuw;
    *sub_h = cuh;

    for ( l = 0; l < REFP_NUM; l++ )
    {
        if ( REFI_IS_VALID( refi[l] ) )
        {
#if M50761_AFFINE_SUB_SIZE_LUT
            int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;

            // convert to 2^(storeBit + bit) precision
            dmv_hor_x = ((ac_mv[l][1][MV_X] - ac_mv[l][0][MV_X]) << 7) >> evc_tbl_log2[cuw];     // deltaMvHor
            dmv_hor_y = ((ac_mv[l][1][MV_Y] - ac_mv[l][0][MV_Y]) << 7) >> evc_tbl_log2[cuw];
            if (vertex_num == 3)
            {
              dmv_ver_x = ((ac_mv[l][2][MV_X] - ac_mv[l][0][MV_X]) << 7) >> evc_tbl_log2[cuh]; // deltaMvVer
              dmv_ver_y = ((ac_mv[l][2][MV_Y] - ac_mv[l][0][MV_Y]) << 7) >> evc_tbl_log2[cuh];
            }
            else
            {
              dmv_ver_x = -dmv_hor_y;                                                    // deltaMvVer
              dmv_ver_y = dmv_hor_x;
            }

            mv_wx = max(abs(dmv_hor_x), abs(dmv_hor_y)), mv_wy = max(abs(dmv_ver_x), abs(dmv_ver_y));
            int sub_lut[4] = { 32, 16, 8, 8 };
            if (mv_wx > 4)
            {
              w = 4;
            }
            else if (mv_wx == 0)
            {
              w = cuw;
            }
            else
            {
              w = sub_lut[mv_wx - 1];
            }

            if (mv_wy > 4)
            {
              h = 4;
            }
            else if (mv_wy == 0)
            {
              h = cuh;
            }
            else
            {
              h = sub_lut[mv_wy - 1];
            }
#else
            mv_wx = max( abs( ac_mv[l][1][MV_X] - ac_mv[l][0][MV_X] ), abs( ac_mv[l][1][MV_Y] - ac_mv[l][0][MV_Y] ) );
            if ( mv_wx )
            {
                w = max( (int)((cuw >> mc_prec_add) / mv_wx), 1 );
                while ( cuw % w )
                {
                    w--;
                }
                w = max( AFFINE_MIN_BLOCK_SIZE, w );
            }

            if ( vertex_num == 2 )
            {
                h = min( w, cuh );
            }
            else
            {
                mv_wy = max( abs( ac_mv[l][2][MV_X] - ac_mv[l][0][MV_X] ), abs( ac_mv[l][2][MV_Y] - ac_mv[l][0][MV_Y] ) );
                if ( mv_wy )
                {
                    h = max( (int)((cuh >> mc_prec_add) / mv_wy), 1 );
                    while ( cuh % h )
                    {
                        h--;
                    }
                    h = max( AFFINE_MIN_BLOCK_SIZE, h );
                }
            }
#endif
            *sub_w = min( *sub_w, w );
            *sub_h = min( *sub_h, h );
        }
    }

#if M50761_EIF_HW_RESTRICTIONS_SET_B || M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV // Dmytro comment HW
    int apply_eif = check_eif_applicability_bi( ac_mv, refi, cuw, cuh, vertex_num );

    if ( !apply_eif )
    {
      *sub_w = max( *sub_w, AFFINE_ADAPT_EIF_SIZE );
      *sub_h = max( *sub_h, AFFINE_ADAPT_EIF_SIZE );
    }
#endif // Dmytro comment HW
}
#endif

void derive_affine_subblock_size( s16 ac_mv[VER_NUM][MV_D], int cuw, int cuh, int *sub_w, int *sub_h, int vertex_num )
{
    int w = cuw;
    int h = cuh;
#if MC_PRECISION_ADD
    int mc_prec_add = MC_PRECISION_ADD;
#else
    int mc_prec_add = 0;
#endif
    int mv_wx, mv_wy;

#if M50761_AFFINE_SUB_SIZE_LUT
    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = ((ac_mv[1][MV_X] - ac_mv[0][MV_X]) << 7) >> evc_tbl_log2[cuw];     // deltaMvHor
    dmv_hor_y = ((ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << 7) >> evc_tbl_log2[cuw];
    if (vertex_num == 3)
    {
      dmv_ver_x = ((ac_mv[2][MV_X] - ac_mv[0][MV_X]) << 7) >> evc_tbl_log2[cuh]; // deltaMvVer
      dmv_ver_y = ((ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << 7) >> evc_tbl_log2[cuh];
    }
    else
    {
      dmv_ver_x = -dmv_hor_y;                                                    // deltaMvVer
      dmv_ver_y = dmv_hor_x;
    }

    mv_wx = max(abs(dmv_hor_x), abs(dmv_hor_y)), mv_wy = max(abs(dmv_ver_x), abs(dmv_ver_y));
    int sub_lut[4] = { 32, 16, 8, 8 };
    if (mv_wx > 4)
    {
      w = 4;
    }
    else if (mv_wx == 0)
    {
      w = cuw;
    }
    else
    {
      w = sub_lut[mv_wx - 1];
    }

    if (mv_wy > 4)
    {
      h = 4;
    }
    else if (mv_wy == 0)
    {
      h = cuh;
    }
    else
    {
      h = sub_lut[mv_wy - 1];
    }
#else
    mv_wx = max( abs( ac_mv[1][MV_X] - ac_mv[0][MV_X] ), abs( ac_mv[1][MV_Y] - ac_mv[0][MV_Y] ) );
    if ( mv_wx )
    {
        w = max( (int)((cuw >> mc_prec_add) / mv_wx), 1 );
        while ( cuw % w )
        {
            w--;
        }
        w = max( AFFINE_MIN_BLOCK_SIZE, w );
    }

    if ( vertex_num == 2 )
    {
        h = min( w, cuh );
    }
    else
    {
        mv_wy = max( abs( ac_mv[2][MV_X] - ac_mv[0][MV_X] ), abs( ac_mv[2][MV_Y] - ac_mv[0][MV_Y] ) );
        if ( mv_wy )
        {
            h = max( (int)((cuh >> mc_prec_add) / mv_wy), 1 );
            while ( cuh % h )
            {
                h--;
            }
            h = max( AFFINE_MIN_BLOCK_SIZE, h );
        }
    }
#endif

    *sub_w = w;
    *sub_h = h;

#if M50761_EIF_HW_RESTRICTIONS_SET_B || M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV // Dmytro comment HW
    int apply_eif = check_eif_applicability_uni( ac_mv, cuw, cuh, vertex_num );

    if ( !apply_eif )
    {
      *sub_w = max( *sub_w, AFFINE_ADAPT_EIF_SIZE );
      *sub_h = max( *sub_h, AFFINE_ADAPT_EIF_SIZE );
    }
#endif // Dmytro comment HW

}
#endif

#if M50761_EIF_HW_RESTRICTIONS_SET_B || M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV // Dmytro comment HW

void calculate_affine_motion_model_parameters( s16 ac_mv[VER_NUM][MV_D], int cuw, int cuh, int vertex_num, int d_hor[MV_D], int d_ver[MV_D], int mv_additional_precision )
{
  assert( MV_X == 0 && MV_Y == 1 );
  assert( vertex_num == 2 || vertex_num == 3 );

  // convert to 2^(storeBit + bit) precision

  for ( int comp = MV_X; comp < MV_D; ++comp )
    d_hor[comp] = ( ( ac_mv[1][comp] - ac_mv[0][comp] ) << mv_additional_precision ) >> evc_tbl_log2[cuw];

  for ( int comp = MV_X; comp < MV_D; ++comp )
  {
    if ( vertex_num == 3 )
      d_ver[comp] = ( ( ac_mv[2][comp] - ac_mv[0][comp] ) << mv_additional_precision ) >> evc_tbl_log2[cuh]; // deltaMvVer
    else
      d_ver[comp] = comp == MV_X ? -d_hor[1 - comp] : d_hor[1 - comp];
  }
}

void calculate_bounding_box_size( int w, int h, s16 ac_mv[VER_NUM][MV_D], int d_hor[MV_D], int d_ver[MV_D], int mv_precision, int *b_box_w, int *b_box_h )
{
  int corners[MV_D][4] = { 0, };

  corners[MV_X][0] = 0;
  corners[MV_X][1] = corners[MV_X][0] + ( w + 1 ) * ( d_hor[MV_X] + ( 1 << mv_precision) ) ;
  corners[MV_X][2] = corners[MV_X][0] + ( h + 1 ) * d_ver[MV_X];
  corners[MV_X][3] = corners[MV_X][1] + corners[MV_X][2] - corners[MV_X][0];

  corners[MV_Y][0] = 0;
  corners[MV_Y][1] = corners[MV_Y][0] + ( w + 1 ) * d_hor[MV_Y];
  corners[MV_Y][2] = corners[MV_Y][0] + ( h + 1 ) * ( d_ver[MV_Y] + ( 1 << mv_precision ) );
  corners[MV_Y][3] = corners[MV_Y][1] + corners[MV_Y][2] - corners[MV_Y][0];

  int max[MV_D] = { 0, }, min[MV_D] = { 0, }, diff[MV_D] = { 0, };

  for ( int coord = MV_X; coord < MV_D; ++coord )
  {
    max[coord] = max( max( corners[coord][0], corners[coord][1] ), max( corners[coord][2], corners[coord][3] ) );

    min[coord] = min( min( corners[coord][0], corners[coord][1] ), min( corners[coord][2], corners[coord][3] ) );

    diff[coord] = ( max[coord] - min[coord] + ( 1 << mv_precision ) - 1 ) >> mv_precision; //ceil
  }

  *b_box_w = diff[MV_X] + 1 + 1;
  *b_box_h = diff[MV_Y] + 1 + 1;
}

BOOL check_eif_num_fetched_lines_restrictions( s16 ac_mv[VER_NUM][MV_D], int d_hor[MV_D], int d_ver[MV_D], int mv_precision )
{
  
#if EIF_FORBID_NON_CONTINUOUS_MEMORY_ACCESS
  if ( d_ver[MV_Y] < -1 << mv_precision )
    return FALSE;
#endif

#if EIF_NUM_FETCHED_LINES_BASIC_RESTRICTION
  if( ( max( 0, d_ver[MV_Y] ) + abs( d_hor[MV_Y] ) ) * ( 1 + EIF_HW_SUBBLOCK_SIZE ) > ( EIF_NUM_ALLOWED_FETCHED_LINES_FOR_THE_FIRST_LINE - 2 ) << mv_precision )
    return FALSE;
#endif

  return TRUE;
}

BOOL check_eif_applicability_uni( s16 ac_mv[VER_NUM][MV_D], int cuw, int cuh, int vertex_num )
{
  int mv_additional_precision = MAX_CU_LOG2;
  int mv_precision = 2 + mv_additional_precision;

  int d_hor[MV_D] = { 0, 0 }, d_ver[MV_D] = { 0, 0 };

  calculate_affine_motion_model_parameters( ac_mv, cuw, cuh, vertex_num, d_hor, d_ver, mv_additional_precision );

  if ( !check_eif_num_fetched_lines_restrictions( ac_mv, d_hor, d_ver, mv_precision ) )
    return FALSE;

#if EIF_MEMORY_BANDWIDTH_RESTRICTION
  int bounding_box_w = 0, bounding_box_h = 0;
  calculate_bounding_box_size( EIF_HW_SUBBLOCK_SIZE, EIF_HW_SUBBLOCK_SIZE, ac_mv, d_hor, d_ver, mv_precision, &bounding_box_w, &bounding_box_h );

if ( bounding_box_w * bounding_box_h > MAX_MEMORY_ACCESS_BI )
  return FALSE;
#endif

  return TRUE;
}


BOOL check_eif_applicability_bi( s16 ac_mv[REFP_NUM][VER_NUM][MV_D], s8 refi[REFP_NUM], int cuw, int cuh, int vertex_num )
{
  int mv_additional_precision = MAX_CU_LOG2;
  int mv_precision = 2 + mv_additional_precision;

  for ( int lidx = 0; lidx <= PRED_L1; lidx++ )
  {
    if ( REFI_IS_VALID( refi[lidx] ) && !check_eif_applicability_uni( ac_mv[lidx], cuw, cuh, vertex_num ) )
      return FALSE;
  }

  return TRUE;
}
#endif // Dmytro comment HW

#if AFFINE
/*******************************************/
/* Neighbor location: Graphical indication */
/*                                         */
/*          B2 UP___________B1 B0          */
/*          LE|               |            */
/*            |               |            */
/*            |               |            */
/*            |      cu    cuh|            */
/*            |               |            */
/*            |               |            */
/*          A1|_____cuw_______|            */
/*          A0                             */
/*                                         */
/*******************************************/

#define SAME_MV(MV0, MV1) ((MV0[MV_X] == MV1[MV_X]) && (MV0[MV_Y] == MV1[MV_Y]))
#define SAME_MVF(refi0, vx0, vy0, refi1, vx1, vy1)   ((refi0 == refi1) && (vx0 == vx1) && (vy0 == vy1))

#if !EIF_MEMORY_BANDWIDTH_RESTRICTION
int evc_get_affine_memory_access(s16 mv[VER_NUM][MV_D], int cuw, int cuh)
{
    int max_x = max(mv[0][MV_X], max(mv[1][MV_X] + cuw, max(mv[2][MV_X], mv[3][MV_X] + cuw))) >> 2;
    int min_x = min(mv[0][MV_X], min(mv[1][MV_X] + cuw, min(mv[2][MV_X], mv[3][MV_X] + cuw))) >> 2;

    int max_y = max(mv[0][MV_Y], max(mv[1][MV_Y], max(mv[2][MV_Y] + cuh, mv[3][MV_Y] + cuh))) >> 2;
    int min_y = min(mv[0][MV_Y], min(mv[1][MV_Y], min(mv[2][MV_Y] + cuh, mv[3][MV_Y] + cuh))) >> 2;

#if EIF
    return (abs(max_x - min_x) + 4) *  (abs(max_y - min_y) + 4);
#else
    return (abs(max_x - min_x) + 7) *  (abs(max_y - min_y) + 7);
#endif
}
#endif

int evc_derive_affine_constructed_candidate(int ptr, EVC_REFP (*refp)[REFP_NUM], int cuw, int cuh, int cp_valid[VER_NUM], s16 cp_mv[REFP_NUM][VER_NUM][MV_D], int cp_refi[REFP_NUM][VER_NUM], int cp_idx[VER_NUM], int model_idx, int ver_num, s16 mrg_list_cp_mv[AFF_MAX_CAND][REFP_NUM][VER_NUM][MV_D], s8 mrg_list_refi[AFF_MAX_CAND][REFP_NUM], int *mrg_idx, int mrg_list_cp_num[AFF_MAX_CAND])
{
    int lidx, i;
    int valid_model[2] = {0, 0};
    s32 cpmv_tmp[REFP_NUM][VER_NUM][MV_D];
    int tmp_hor, tmp_ver;
    int shiftHtoW = 7 + evc_tbl_log2[cuw] - evc_tbl_log2[cuh]; // x * cuWidth / cuHeight

    // early terminate
    if(*mrg_idx >= AFF_MAX_CAND)
    {
        return 0;
    }

    // check valid model and decide ref idx
    if(ver_num == 2)
    {
        int idx0 = cp_idx[0], idx1 = cp_idx[1];

        if(!cp_valid[idx0] || !cp_valid[idx1])
        {
            return 0;
        }

        for(lidx = 0; lidx < REFP_NUM; lidx++)
        {
            if(REFI_IS_VALID(cp_refi[lidx][idx0]) && REFI_IS_VALID(cp_refi[lidx][idx1]) && cp_refi[lidx][idx0] == cp_refi[lidx][idx1])
            {
                valid_model[lidx] = 1;
            }
        }
    }
    else if(ver_num == 3)
    {
        int idx0 = cp_idx[0], idx1 = cp_idx[1], idx2 = cp_idx[2];

        if(!cp_valid[idx0] || !cp_valid[idx1] || !cp_valid[idx2])
        {
            return 0;
        }

        for(lidx = 0; lidx < REFP_NUM; lidx++)
        {
            if(REFI_IS_VALID(cp_refi[lidx][idx0]) && REFI_IS_VALID(cp_refi[lidx][idx1]) && REFI_IS_VALID(cp_refi[lidx][idx2]) && cp_refi[lidx][idx0] == cp_refi[lidx][idx1] && cp_refi[lidx][idx0] == cp_refi[lidx][idx2])
            {
                valid_model[lidx] = 1;
            }
        }
    }
    else
    {
        evc_assert( 0 );
    }

    // set merge index and vertex num for valid model
    if(valid_model[0] || valid_model[1])
    {
        mrg_list_cp_num[*mrg_idx] = ver_num;
    }
    else
    {
        return 0;
    }
    
    for(lidx = 0; lidx < REFP_NUM; lidx++)
    {
        if(valid_model[lidx])
        {
            mrg_list_refi[*mrg_idx][lidx] = cp_refi[lidx][cp_idx[0]];
            for ( i = 0; i < ver_num; i++ )
            {
                cpmv_tmp[lidx][cp_idx[i]][MV_X] = (s32)cp_mv[lidx][cp_idx[i]][MV_X];
                cpmv_tmp[lidx][cp_idx[i]][MV_Y] = (s32)cp_mv[lidx][cp_idx[i]][MV_Y];
            }

            // convert to LT, RT[, [LB], [RB]]
            switch(model_idx)
            {
            case 0: // 0 : LT, RT, LB
                break;
            case 1: // 1 : LT, RT, RB
                cpmv_tmp[lidx][2][MV_X] = cpmv_tmp[lidx][3][MV_X] + cpmv_tmp[lidx][0][MV_X] - cpmv_tmp[lidx][1][MV_X];
                cpmv_tmp[lidx][2][MV_Y] = cpmv_tmp[lidx][3][MV_Y] + cpmv_tmp[lidx][0][MV_Y] - cpmv_tmp[lidx][1][MV_Y];
                break;
            case 2: // 1 : LT, LB, RB
                cpmv_tmp[lidx][1][MV_X] = cpmv_tmp[lidx][3][MV_X] + cpmv_tmp[lidx][0][MV_X] - cpmv_tmp[lidx][2][MV_X];
                cpmv_tmp[lidx][1][MV_Y] = cpmv_tmp[lidx][3][MV_Y] + cpmv_tmp[lidx][0][MV_Y] - cpmv_tmp[lidx][2][MV_Y];
                break;
            case 3: // 4 : RT, LB, RB
                cpmv_tmp[lidx][0][MV_X] = cpmv_tmp[lidx][1][MV_X] + cpmv_tmp[lidx][2][MV_X] - cpmv_tmp[lidx][3][MV_X];
                cpmv_tmp[lidx][0][MV_Y] = cpmv_tmp[lidx][1][MV_Y] + cpmv_tmp[lidx][2][MV_Y] - cpmv_tmp[lidx][3][MV_Y];
                break;
            case 4: // 5 : LT, RT
                break;
            case 5: // 6 : LT, LB
                tmp_hor = +((cpmv_tmp[lidx][2][MV_Y] - cpmv_tmp[lidx][0][MV_Y]) << shiftHtoW) + (cpmv_tmp[lidx][0][MV_X] << 7);
                tmp_ver = -((cpmv_tmp[lidx][2][MV_X] - cpmv_tmp[lidx][0][MV_X]) << shiftHtoW) + (cpmv_tmp[lidx][0][MV_Y] << 7);
                evc_mv_rounding_s32( tmp_hor, tmp_ver, &cpmv_tmp[lidx][1][MV_X], &cpmv_tmp[lidx][1][MV_Y], 7, 0 );
                break;
            default:
                evc_assert( 0 );
            }

            for ( i = 0; i < ver_num; i++ )
            {
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_X] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, cpmv_tmp[lidx][i][MV_X] );
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_Y] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, cpmv_tmp[lidx][i][MV_Y] );
            }
        }
        else
        {
            mrg_list_refi[*mrg_idx][lidx] = -1;
            for (i = 0; i < ver_num; i++)
            {
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_X] = 0;
                mrg_list_cp_mv[*mrg_idx][lidx][i][MV_Y] = 0;
            }
        }
    }

    (*mrg_idx)++;

    return 1;
}

void evc_derive_affine_model_mv(int scup, int scun, int lidx, s16(*map_mv)[REFP_NUM][MV_D], int cuw, int cuh, int w_scu, int h_scu, s16 mvp[VER_NUM][MV_D], u32 *map_affine, int cur_cp_num
#if M48933_AFFINE
                                , int log2_max_cuwh
#endif
#if DMVR_LAG
                                , u32 *map_scu
                                , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
)
{
    s16 neb_mv[VER_NUM][MV_D] = {{0, }, };
    int i;
    int neb_addr[VER_NUM];
    int neb_log_w = MCU_GET_AFF_LOGW(map_affine[scun]);
    int neb_log_h = MCU_GET_AFF_LOGH(map_affine[scun]);
    int neb_w = 1 << neb_log_w;
    int neb_h = 1 << neb_log_h;
    int neb_x, neb_y;
    int cur_x, cur_y;
    int max_bit = 7;
    int diff_w = max_bit - neb_log_w;
    int diff_h = max_bit - neb_log_h;
    int dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, hor_base, ver_base;
    s32 tmp_hor, tmp_ver;
    int neb_cp_num = (MCU_GET_AFF( map_scu[scun] ) == 1) ? 2 : 3;

    neb_addr[0] = scun - MCU_GET_AFF_XOFF(map_affine[scun]) - w_scu * MCU_GET_AFF_YOFF(map_affine[scun]);
    neb_addr[1] = neb_addr[0] + ((neb_w >> MIN_CU_LOG2) - 1);
    neb_addr[2] = neb_addr[0] + ((neb_h >> MIN_CU_LOG2) - 1) * w_scu;
    neb_addr[3] = neb_addr[2] + ((neb_w >> MIN_CU_LOG2) - 1);

    neb_x = (neb_addr[0] % w_scu) << MIN_CU_LOG2;
    neb_y = (neb_addr[0] / w_scu) << MIN_CU_LOG2;
    cur_x = (scup % w_scu) << MIN_CU_LOG2;
    cur_y = (scup / w_scu) << MIN_CU_LOG2;

#if M48933_AFFINE
    for ( i = 0; i < VER_NUM; i++ )
#else
    for(i = 0; i < cur_cp_num; i++)
#endif
    {
#if DMVR_LAG
        if (MCU_GET_DMVRF(map_scu[neb_addr[i]]) 
#if (DMVR_LAG == 2)
            && (!evc_use_refine_mv(scup, neb_addr[i], w_scu))
#endif
            )
        {
            neb_mv[i][MV_X] = map_unrefined_mv[neb_addr[i]][lidx][MV_X];
            neb_mv[i][MV_Y] = map_unrefined_mv[neb_addr[i]][lidx][MV_Y];
        }
        else
#endif
        {
            neb_mv[i][MV_X] = map_mv[neb_addr[i]][lidx][MV_X];
            neb_mv[i][MV_Y] = map_mv[neb_addr[i]][lidx][MV_Y];
        }
    }

#if M48933_AFFINE
    int is_top_ctu_boundary = FALSE;
    if ( (neb_y + neb_h) % (1 << log2_max_cuwh) == 0 && (neb_y + neb_h) == cur_y )
    {
        is_top_ctu_boundary = TRUE;
        neb_y += neb_h;

        neb_mv[0][MV_X] = neb_mv[2][MV_X];
        neb_mv[0][MV_Y] = neb_mv[2][MV_Y];
        neb_mv[1][MV_X] = neb_mv[3][MV_X];
        neb_mv[1][MV_Y] = neb_mv[3][MV_Y];
    }
#endif

    dmv_hor_x = (neb_mv[1][MV_X] - neb_mv[0][MV_X]) << diff_w;    // deltaMvHor
    dmv_hor_y = (neb_mv[1][MV_Y] - neb_mv[0][MV_Y]) << diff_w;
#if M48933_AFFINE
    if (cur_cp_num == 3 && !is_top_ctu_boundary )
#else
    if(cur_cp_num == 3)
#endif
    {
        dmv_ver_x = (neb_mv[2][MV_X] - neb_mv[0][MV_X]) << diff_h;  // deltaMvVer
        dmv_ver_y = (neb_mv[2][MV_Y] - neb_mv[0][MV_Y]) << diff_h;
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                      // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }
    hor_base = neb_mv[0][MV_X] << max_bit;
    ver_base = neb_mv[0][MV_Y] << max_bit;

    // derive CPMV 0
    tmp_hor = dmv_hor_x * (cur_x - neb_x) + dmv_ver_x * (cur_y - neb_y) + hor_base;
    tmp_ver = dmv_hor_y * (cur_x - neb_x) + dmv_ver_y * (cur_y - neb_y) + ver_base;
    evc_mv_rounding_s32( tmp_hor, tmp_ver, &tmp_hor, &tmp_ver, max_bit, 0 );
    mvp[0][MV_X] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, tmp_hor );
    mvp[0][MV_Y] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, tmp_ver );

    // derive CPMV 1
    tmp_hor = dmv_hor_x * (cur_x - neb_x + cuw) + dmv_ver_x * (cur_y - neb_y) + hor_base;
    tmp_ver = dmv_hor_y * (cur_x - neb_x + cuw) + dmv_ver_y * (cur_y - neb_y) + ver_base;
    evc_mv_rounding_s32( tmp_hor, tmp_ver, &tmp_hor, &tmp_ver, max_bit, 0 );
    mvp[1][MV_X] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, tmp_hor );
    mvp[1][MV_Y] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, tmp_ver );

    // derive CPMV 2
    if ( cur_cp_num == 3 )
    {
        tmp_hor = dmv_hor_x * (cur_x - neb_x) + dmv_ver_x * (cur_y - neb_y + cuh) + hor_base;
        tmp_ver = dmv_hor_y * (cur_x - neb_x) + dmv_ver_y * (cur_y - neb_y + cuh) + ver_base;
        evc_mv_rounding_s32( tmp_hor, tmp_ver, &tmp_hor, &tmp_ver, max_bit, 0 );
        mvp[2][MV_X] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, tmp_hor );
        mvp[2][MV_Y] = (s16)EVC_CLIP3( EVC_INT16_MIN, EVC_INT16_MAX, tmp_ver );
    }
}

/* inter affine mode */
void evc_get_affine_motion_scaling(int ptr, int scup, int lidx, s8 cur_refi, int num_refp, \
                                   s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], EVC_REFP(*refp)[REFP_NUM], \
                                   int cuw, int cuh, int w_scu, int h_scu, u16 avail, s16 mvp[MAX_NUM_MVP][VER_NUM][MV_D], s8 refi[MAX_NUM_MVP]
                                   , u32* map_scu, u32* map_affine, int vertex_num, u16 avail_lr
#if M48933_AFFINE
                                   , int log2_max_cuwh
#endif
#if DMVR_LAG
                                   , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
)
{
    int x_scu = scup % w_scu;
    int y_scu = scup / w_scu;
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;
    int cnt_lt = 0, cnt_rt = 0, cnt_lb = 0;
    int i, j, k;
    s16 mvp_tmp[VER_NUM][MV_D];
    int neb_addr[3];
    int valid_flag[3];
    int cnt_tmp = 0;
    s16 mvp_cand_lt[AFFINE_MAX_NUM_LT][MV_D];
    int neb_addr_lt[AFFINE_MAX_NUM_LT];
    int valid_flag_lt[AFFINE_MAX_NUM_LT];
    s16 mvp_cand_rt[AFFINE_MAX_NUM_RT][MV_D];
    int neb_addr_rt[AFFINE_MAX_NUM_RT];
    int valid_flag_rt[AFFINE_MAX_NUM_RT];
    s16 mvp_cand_lb[AFFINE_MAX_NUM_LB][MV_D];
    int neb_addr_lb[AFFINE_MAX_NUM_LB];
    int valid_flag_lb[AFFINE_MAX_NUM_LB];
#if M48879_IMPROVEMENT_SUCO
    int cnt_rb = 0;
    s16 mvp_cand_rb[AFFINE_MAX_NUM_RB][MV_D];
    int neb_addr_rb[AFFINE_MAX_NUM_RB];
    int valid_flag_rb[AFFINE_MAX_NUM_RB];
#endif
    //-------------------  INIT  -------------------//
#if INCREASE_MVP_NUM
    for(i = 0; i < ORG_MAX_NUM_MVP; i++)
#else
    for(i = 0; i < MAX_NUM_MVP; i++)
#endif
    {
        for(j = 0; j < VER_NUM; j++)
        {
            mvp[i][j][MV_X] = 0;
            mvp[i][j][MV_Y] = 0;
        }
        refi[i] = 0;
    }

    //-------------------  Model based affine MVP  -------------------//
    
    // left inherited affine MVP, first of {A0, A1}
    neb_addr[0] = scup + w_scu * scuh - 1;       // A0
    neb_addr[1] = scup + w_scu * (scuh - 1) - 1; // A1
    valid_flag[0] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && MCU_GET_AFF(map_scu[neb_addr[0]]);
    valid_flag[1] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && MCU_GET_AFF(map_scu[neb_addr[1]]);

    for(k = 0; k < 2; k++)
    {
        if(valid_flag[k] && REFI_IS_VALID(map_refi[neb_addr[k]][lidx])
           && map_refi[neb_addr[k]][lidx] == cur_refi)
        {
            refi[cnt_tmp] = map_refi[neb_addr[k]][lidx];
            evc_derive_affine_model_mv(scup, neb_addr[k], lidx, map_mv, cuw, cuh, w_scu, h_scu, mvp_tmp, map_affine, vertex_num
#if M48933_AFFINE
                                       , log2_max_cuwh
#endif
#if DMVR_LAG
                                       , map_scu
                                       , map_unrefined_mv
#endif
            );
            mvp[cnt_tmp][0][MV_X] = mvp_tmp[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_tmp[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_tmp[1][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_tmp[1][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_tmp[2][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_tmp[2][MV_Y];
            cnt_tmp++;
            break;
        }
    }
    if(cnt_tmp >= AFF_MAX_NUM_MVP)
    {
        return;
    }

    // above inherited affine MVP, first of {B0, B1, B2}
    neb_addr[0] = scup - w_scu + scuw;           // B0
    neb_addr[1] = scup - w_scu + scuw - 1;       // B1
    neb_addr[2] = scup - w_scu - 1;              // B2
    valid_flag[0] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && MCU_GET_AFF(map_scu[neb_addr[0]]);
    valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && MCU_GET_AFF(map_scu[neb_addr[1]]);
    valid_flag[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && MCU_GET_AFF(map_scu[neb_addr[2]]);
    for(k = 0; k < 3; k++)
    {
        if(valid_flag[k] && REFI_IS_VALID(map_refi[neb_addr[k]][lidx])
           && map_refi[neb_addr[k]][lidx] == cur_refi)
        {
            refi[cnt_tmp] = map_refi[neb_addr[k]][lidx];
            evc_derive_affine_model_mv(scup, neb_addr[k], lidx, map_mv, cuw, cuh, w_scu, h_scu, mvp_tmp, map_affine, vertex_num
#if M48933_AFFINE
                                       , log2_max_cuwh
#endif
#if DMVR_LAG
                                       , map_scu
                                       ,  map_unrefined_mv
#endif
            );
            mvp[cnt_tmp][0][MV_X] = mvp_tmp[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_tmp[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_tmp[1][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_tmp[1][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_tmp[2][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_tmp[2][MV_Y];
            cnt_tmp++;
            break;
        }
    }
    if(cnt_tmp >= AFF_MAX_NUM_MVP)
    {
        return;
    }
#if M48879_IMPROVEMENT_SUCO
    // right inherited affine MVP, first of {C0, C1}
    neb_addr[0] = scup + w_scu * scuh + scuw;       // C0
    neb_addr[1] = scup + w_scu * (scuh - 1) + scuw; // C1
    valid_flag[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && MCU_GET_AFF(map_scu[neb_addr[0]]);
    valid_flag[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && MCU_GET_AFF(map_scu[neb_addr[1]]);

    for (k = 0; k < 2; k++)
    {
        if (valid_flag[k] && REFI_IS_VALID(map_refi[neb_addr[k]][lidx])
            && map_refi[neb_addr[k]][lidx] == cur_refi)
        {
            refi[cnt_tmp] = map_refi[neb_addr[k]][lidx];
            evc_derive_affine_model_mv(scup, neb_addr[k], lidx, map_mv, cuw, cuh, w_scu, h_scu, mvp_tmp, map_affine, vertex_num
#if M48933_AFFINE
                , log2_max_cuwh
#endif
#if DMVR_LAG
                , map_scu
                , map_unrefined_mv
#endif
            );
            mvp[cnt_tmp][0][MV_X] = mvp_tmp[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_tmp[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_tmp[1][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_tmp[1][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_tmp[2][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_tmp[2][MV_Y];
            cnt_tmp++;
            break;
        }
    }
    if (cnt_tmp >= AFF_MAX_NUM_MVP)
    {
        return;
    }
#endif
    //-------------------  LT  -------------------//
    for(i = 0; i < AFFINE_MAX_NUM_LT; i++)
    {
        mvp_cand_lt[i][MV_X] = 0;
        mvp_cand_lt[i][MV_Y] = 0;
    }
#if M50662_AFFINE_ALTERNATIVE_SCANNING_ORDER
    neb_addr_lt[0] = scup - w_scu - 1;
    neb_addr_lt[1] = scup - w_scu;
    neb_addr_lt[2] = scup - 1;

    valid_flag_lt[0] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[0]]) && !MCU_GET_IF(map_scu[neb_addr_lt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[0]]);
    valid_flag_lt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[1]]) && !MCU_GET_IF(map_scu[neb_addr_lt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[1]]);
    valid_flag_lt[2] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[2]]) && !MCU_GET_IF(map_scu[neb_addr_lt[2]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[2]]);
#else
    neb_addr_lt[0] = scup - 1;          // LE
    neb_addr_lt[1] = scup - w_scu;      // UP
    neb_addr_lt[2] = scup - w_scu - 1;  // B2
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
    valid_flag_lt[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[0]]) && !MCU_GET_IF(map_scu[neb_addr_lt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[0]]);
    valid_flag_lt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[1]]) && !MCU_GET_IF(map_scu[neb_addr_lt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[1]]);
    valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[2]]) && !MCU_GET_IF(map_scu[neb_addr_lt[2]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[2]]);
#else
    valid_flag_lt[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[0]]) && !MCU_GET_IF(map_scu[neb_addr_lt[0]]);
    valid_flag_lt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[1]]) && !MCU_GET_IF(map_scu[neb_addr_lt[1]]);
    valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[2]]) && !MCU_GET_IF(map_scu[neb_addr_lt[2]]);
#endif
#endif

    for(k = 0; k < AFFINE_MAX_NUM_LT; k++)
    {
        if(valid_flag_lt[k] && REFI_IS_VALID(map_refi[neb_addr_lt[k]][lidx]))
        {
            refi[cnt_lt] = map_refi[neb_addr_lt[k]][lidx];
            if(refi[cnt_lt] == cur_refi)
            {
#if DMVR_LAG
                if(MCU_GET_DMVRF(map_scu[neb_addr_lt[k]])
#if (DMVR_LAG == 2)
                   && (!evc_use_refine_mv(scup, neb_addr_lt[k], w_scu))
#endif
                   )
                {
                    mvp_cand_lt[cnt_lt][MV_X] = map_unrefined_mv[neb_addr_lt[k]][lidx][MV_X];
                    mvp_cand_lt[cnt_lt][MV_Y] = map_unrefined_mv[neb_addr_lt[k]][lidx][MV_Y];
                }
                else
#endif
                {
                    mvp_cand_lt[cnt_lt][MV_X] = map_mv[neb_addr_lt[k]][lidx][MV_X];
                    mvp_cand_lt[cnt_lt][MV_Y] = map_mv[neb_addr_lt[k]][lidx][MV_Y];
                }
                cnt_lt++;
                break;
            }
        }
    }

    //-------------------  RT  -------------------//
    for(i = 0; i < AFFINE_MAX_NUM_RT; i++)
    {
        mvp_cand_rt[i][MV_X] = 0;
        mvp_cand_rt[i][MV_Y] = 0;
    }
#if M50662_AFFINE_ALTERNATIVE_SCANNING_ORDER
    neb_addr_rt[0] = scup - w_scu + scuw;
    neb_addr_rt[1] = scup - w_scu + scuw - 1;

    valid_flag_rt[0] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[0]]) && !MCU_GET_IF(map_scu[neb_addr_rt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[0]]);
    valid_flag_rt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_rt[1]]) && !MCU_GET_IF(map_scu[neb_addr_rt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[1]]);
#else
    neb_addr_rt[0] = scup - w_scu + scuw - 1;     // B1
    neb_addr_rt[1] = scup - w_scu + scuw;         // B0
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
    valid_flag_rt[0] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_rt[0]]) && !MCU_GET_IF(map_scu[neb_addr_rt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[0]]);
    valid_flag_rt[1] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[1]]) && !MCU_GET_IF(map_scu[neb_addr_rt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[1]]);
#else
    valid_flag_rt[0] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_rt[0]]) && !MCU_GET_IF(map_scu[neb_addr_rt[0]]);
    valid_flag_rt[1] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[1]]) && !MCU_GET_IF(map_scu[neb_addr_rt[1]]);
#endif
#endif
#if M48879_IMPROVEMENT_SUCO
    neb_addr_rt[2] = scup + scuw;
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
    valid_flag_rt[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[2]]) && !MCU_GET_IF(map_scu[neb_addr_rt[2]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[2]]);
#else
    valid_flag_rt[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[2]]) && !MCU_GET_IF(map_scu[neb_addr_rt[2]]);
#endif
#endif

    for(k = 0; k < AFFINE_MAX_NUM_RT; k++)
    {
        if(valid_flag_rt[k] && REFI_IS_VALID(map_refi[neb_addr_rt[k]][lidx]))
        {
            refi[cnt_rt] = map_refi[neb_addr_rt[k]][lidx];
            if(refi[cnt_rt] == cur_refi)
            {
#if DMVR_LAG
                if(MCU_GET_DMVRF(map_scu[neb_addr_rt[k]])
#if (DMVR_LAG == 2)
                   && (!evc_use_refine_mv(scup, neb_addr_rt[k], w_scu))
#endif
                   )
                {
                    mvp_cand_rt[cnt_rt][MV_X] = map_unrefined_mv[neb_addr_rt[k]][lidx][MV_X];
                    mvp_cand_rt[cnt_rt][MV_Y] = map_unrefined_mv[neb_addr_rt[k]][lidx][MV_Y];
                }
                else
#endif
                {
                    mvp_cand_rt[cnt_rt][MV_X] = map_mv[neb_addr_rt[k]][lidx][MV_X];
                    mvp_cand_rt[cnt_rt][MV_Y] = map_mv[neb_addr_rt[k]][lidx][MV_Y];
                }
                cnt_rt++;
                break;
            }
        }
    }

    //-------------------  LB  -------------------//
    for(i = 0; i < AFFINE_MAX_NUM_LB; i++)
    {
        mvp_cand_lb[i][MV_X] = 0;
        mvp_cand_lb[i][MV_Y] = 0;
    }

    neb_addr_lb[0] = scup + w_scu * scuh - 1;        // A0
    neb_addr_lb[1] = scup + w_scu * (scuh - 1) - 1;  // A1
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
    valid_flag_lb[0] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_lb[0]]) && !MCU_GET_IF(map_scu[neb_addr_lb[0]]) && !MCU_GET_IBC(map_scu[neb_addr_lb[0]]);
    valid_flag_lb[1] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lb[1]]) && !MCU_GET_IF(map_scu[neb_addr_lb[1]]) && !MCU_GET_IBC(map_scu[neb_addr_lb[1]]);
#else
    valid_flag_lb[0] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_lb[0]]) && !MCU_GET_IF(map_scu[neb_addr_lb[0]]);
    valid_flag_lb[1] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lb[1]]) && !MCU_GET_IF(map_scu[neb_addr_lb[1]]);
#endif

    for(k = 0; k < AFFINE_MAX_NUM_LB; k++)
    {
        if(valid_flag_lb[k] && REFI_IS_VALID(map_refi[neb_addr_lb[k]][lidx]))
        {
            refi[cnt_lb] = map_refi[neb_addr_lb[k]][lidx];
            if(refi[cnt_lb] == cur_refi)
            {
#if M50761_DMVR_BUGFIX_REFINED_MV_RESTRICTION
              if (MCU_GET_DMVRF(map_scu[neb_addr_lb[k]]))
              {
                mvp_cand_lb[cnt_lb][MV_X] = map_unrefined_mv[neb_addr_lb[k]][lidx][MV_X];
                mvp_cand_lb[cnt_lb][MV_Y] = map_unrefined_mv[neb_addr_lb[k]][lidx][MV_Y];
              }
              else
              {
                mvp_cand_lb[cnt_lb][MV_X] = map_mv[neb_addr_lb[k]][lidx][MV_X];
                mvp_cand_lb[cnt_lb][MV_Y] = map_mv[neb_addr_lb[k]][lidx][MV_Y];
              }
#else

                mvp_cand_lb[cnt_lb][MV_X] = map_mv[neb_addr_lb[k]][lidx][MV_X];
                mvp_cand_lb[cnt_lb][MV_Y] = map_mv[neb_addr_lb[k]][lidx][MV_Y];
#endif
                cnt_lb++;
                break;
            }
        }
    }
#if M48879_IMPROVEMENT_SUCO
    //-------------------  RB  -------------------//
    for (i = 0; i < AFFINE_MAX_NUM_RB; i++)
    {
        mvp_cand_rb[i][MV_X] = 0;
        mvp_cand_rb[i][MV_Y] = 0;
    }

    neb_addr_rb[0] = scup + w_scu * scuh + scuw;
    neb_addr_rb[1] = scup + w_scu * (scuh - 1) + scuw;
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
    valid_flag_rb[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_rb[0]]) && !MCU_GET_IF(map_scu[neb_addr_rb[0]]) && !MCU_GET_IBC(map_scu[neb_addr_rb[0]]);
    valid_flag_rb[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rb[1]]) && !MCU_GET_IF(map_scu[neb_addr_rb[1]]) && !MCU_GET_IF(map_scu[neb_addr_rb[1]]);
#else
    valid_flag_rb[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_rb[0]]) && !MCU_GET_IF(map_scu[neb_addr_rb[0]]);
    valid_flag_rb[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rb[1]]) && !MCU_GET_IF(map_scu[neb_addr_rb[1]]);
#endif

    for (k = 0; k < AFFINE_MAX_NUM_RB; k++)
    {
        if (valid_flag_rb[k] && REFI_IS_VALID(map_refi[neb_addr_rb[k]][lidx]))
        {
            refi[cnt_rb] = map_refi[neb_addr_rb[k]][lidx];
            if (refi[cnt_rb] == cur_refi)
            {
#if M50761_DMVR_BUGFIX_REFINED_MV_RESTRICTION
              if (MCU_GET_DMVRF(map_scu[neb_addr_rb[k]]))
              {
                mvp_cand_rb[cnt_rb][MV_X] = map_unrefined_mv[neb_addr_rb[k]][lidx][MV_X];
                mvp_cand_rb[cnt_rb][MV_Y] = map_unrefined_mv[neb_addr_rb[k]][lidx][MV_Y];
              }
              else
              {
                mvp_cand_rb[cnt_rb][MV_X] = map_mv[neb_addr_rb[k]][lidx][MV_X];
                mvp_cand_rb[cnt_rb][MV_Y] = map_mv[neb_addr_rb[k]][lidx][MV_Y];
              }
#else
                mvp_cand_rb[cnt_rb][MV_X] = map_mv[neb_addr_rb[k]][lidx][MV_X];
                mvp_cand_rb[cnt_rb][MV_Y] = map_mv[neb_addr_rb[k]][lidx][MV_Y];
#endif
                cnt_rb++;
                break;
            }
        }
    }
#endif

    //-------------------  organize  -------------------//
    {
#if M48933_AFFINE
#if M48879_IMPROVEMENT_SUCO
        if (cnt_lt && cnt_rt && (vertex_num == 2 || (cnt_lb || cnt_rb)))
#else
        if ( cnt_lt && cnt_rt && (vertex_num == 2 || cnt_lb) )
#endif
#else
        if(cnt_lt && cnt_rt)
#endif
        {
            mvp[cnt_tmp][0][MV_X] = mvp_cand_lt[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_cand_lt[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_cand_rt[0][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_cand_rt[0][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_cand_lb[0][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_cand_lb[0][MV_Y];
#if M48879_IMPROVEMENT_SUCO
            if (cnt_lb == 0 && cnt_rb > 0)
            {
                mvp[cnt_tmp][2][MV_X] = (s16)EVC_CLIP3(EVC_INT16_MIN, EVC_INT16_MAX, mvp_cand_rb[0][MV_X] + mvp_cand_lt[0][MV_X] - mvp_cand_rt[0][MV_X]);
                mvp[cnt_tmp][2][MV_Y] = (s16)EVC_CLIP3(EVC_INT16_MIN, EVC_INT16_MAX, mvp_cand_rb[0][MV_Y] + mvp_cand_lt[0][MV_Y] - mvp_cand_rt[0][MV_Y]);
            }
#endif

#if !M48933_AFFINE
            if(vertex_num == 2 || cnt_lb == 0)
            {
                mvp[cnt_tmp][2][MV_X] = mvp[cnt_tmp][0][MV_X] + (mvp[cnt_tmp][0][MV_Y] - mvp[cnt_tmp][1][MV_Y]) * cuh / cuw;
                mvp[cnt_tmp][2][MV_Y] = mvp[cnt_tmp][0][MV_Y] + (mvp[cnt_tmp][1][MV_X] - mvp[cnt_tmp][0][MV_X]) * cuh / cuw;
            }
#endif
            cnt_tmp++;
        }
        if(cnt_tmp == AFF_MAX_NUM_MVP)
        {
            return;
        }

        // Add translation mv, left
        if(cnt_lb)
        {
            mvp[cnt_tmp][0][MV_X] = mvp_cand_lb[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_cand_lb[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_cand_lb[0][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_cand_lb[0][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_cand_lb[0][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_cand_lb[0][MV_Y];
            cnt_tmp++;
        }
#if M48879_IMPROVEMENT_SUCO
        // Add translation mv, right
        else if (cnt_rb)
        {
            mvp[cnt_tmp][0][MV_X] = mvp_cand_rb[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_cand_rb[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_cand_rb[0][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_cand_rb[0][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_cand_rb[0][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_cand_rb[0][MV_Y];
            cnt_tmp++;
        }
#endif
        if(cnt_tmp == AFF_MAX_NUM_MVP)
        {
            return;
        }

        // Add translation mv, above
        if(cnt_rt)
        {
            mvp[cnt_tmp][0][MV_X] = mvp_cand_rt[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_cand_rt[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_cand_rt[0][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_cand_rt[0][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_cand_rt[0][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_cand_rt[0][MV_Y];
            cnt_tmp++;
        }
        if(cnt_tmp == AFF_MAX_NUM_MVP)
        {
            return;
        }

        // Add translation mv, above left
        if(cnt_lt)
        {
            mvp[cnt_tmp][0][MV_X] = mvp_cand_lt[0][MV_X];
            mvp[cnt_tmp][0][MV_Y] = mvp_cand_lt[0][MV_Y];
            mvp[cnt_tmp][1][MV_X] = mvp_cand_lt[0][MV_X];
            mvp[cnt_tmp][1][MV_Y] = mvp_cand_lt[0][MV_Y];
            mvp[cnt_tmp][2][MV_X] = mvp_cand_lt[0][MV_X];
            mvp[cnt_tmp][2][MV_Y] = mvp_cand_lt[0][MV_Y];
            cnt_tmp++;
        }
        if(cnt_tmp == AFF_MAX_NUM_MVP)
        {
            return;
        }

        // Add zero MVP
        for(; cnt_tmp < AFF_MAX_NUM_MVP; cnt_tmp++)
        {
            mvp[cnt_tmp][0][MV_X] = 0;
            mvp[cnt_tmp][0][MV_Y] = 0;
            mvp[cnt_tmp][1][MV_X] = 0;
            mvp[cnt_tmp][1][MV_Y] = 0;
            mvp[cnt_tmp][2][MV_X] = 0;
            mvp[cnt_tmp][2][MV_Y] = 0;
            cnt_tmp++;
        }
    }
}

/* merge affine mode */
int evc_get_affine_merge_candidate(int ptr, int slice_type, int scup, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], EVC_REFP(*refp)[REFP_NUM], int cuw, int cuh, int w_scu, int h_scu, u16 avail,
                                   s8 mrg_list_refi[AFF_MAX_CAND][REFP_NUM], s16 mrg_list_cpmv[AFF_MAX_CAND][REFP_NUM][VER_NUM][MV_D], int mrg_list_cp_num[AFF_MAX_CAND], u32* map_scu, u32* map_affine
#if M48933_AFFINE
                                   , int log2_max_cuwh
#endif
#if DMVR_LAG
                                   , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
#if M48879_IMPROVEMENT_SUCO
    , u16 avail_lr
#endif
#if M50761_TMVP_ALIGN_SPEC || M50662_AFFINE_IBC_TMVP_SUCO_FIX
    , EVC_SH * sh
#endif
)
{
    int lidx, i, k;
    int x_scu = scup % w_scu;
    int y_scu = scup / w_scu;
    int scuw = cuw >> MIN_CU_LOG2;
    int scuh = cuh >> MIN_CU_LOG2;
    int cnt = 0;
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
    s16 tmvp[REFP_NUM][MV_D];
    s8  availablePredIdx = 0;
#endif
    //-------------------  Model based affine MVP  -------------------//
    {
        int neb_addr[5];
        int valid_flag[5];
        int top_left[7];
#if M48879_IMPROVEMENT_SUCO
        if (avail_lr == LR_01)
        {
            neb_addr[0] = scup + w_scu * (scuh - 1) + scuw; // A1
            neb_addr[1] = scup - w_scu;                     // B1
            neb_addr[2] = scup - w_scu - 1;                 // B0
            neb_addr[3] = scup + w_scu * scuh + scuw;       // A0
            neb_addr[4] = scup - w_scu + scuw;              // B2

            valid_flag[0] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && MCU_GET_AFF(map_scu[neb_addr[0]]);
            valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && MCU_GET_AFF(map_scu[neb_addr[1]]);
            valid_flag[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && MCU_GET_AFF(map_scu[neb_addr[2]]);
            valid_flag[3] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && MCU_GET_AFF(map_scu[neb_addr[3]]);
            valid_flag[4] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && MCU_GET_AFF(map_scu[neb_addr[4]]);
        }
        else
        {
#endif
        neb_addr[0] = scup + w_scu * (scuh - 1) - 1; // A1
        neb_addr[1] = scup - w_scu + scuw - 1;       // B1
        neb_addr[2] = scup - w_scu + scuw;           // B0
        neb_addr[3] = scup + w_scu * scuh - 1;       // A0
        neb_addr[4] = scup - w_scu - 1;              // B2

        valid_flag[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr[0]]) && !MCU_GET_IF(map_scu[neb_addr[0]]) && MCU_GET_AFF(map_scu[neb_addr[0]]);
        valid_flag[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[1]]) && !MCU_GET_IF(map_scu[neb_addr[1]]) && MCU_GET_AFF(map_scu[neb_addr[1]]);
        valid_flag[2] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr[2]]) && !MCU_GET_IF(map_scu[neb_addr[2]]) && MCU_GET_AFF(map_scu[neb_addr[2]]);
        valid_flag[3] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr[3]]) && !MCU_GET_IF(map_scu[neb_addr[3]]) && MCU_GET_AFF(map_scu[neb_addr[3]]);
        valid_flag[4] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr[4]]) && !MCU_GET_IF(map_scu[neb_addr[4]]) && MCU_GET_AFF(map_scu[neb_addr[4]]);
#if M48879_IMPROVEMENT_SUCO
        }
#endif
        for(k = 0; k < 5; k++)
        {
            if(valid_flag[k])
            {
                top_left[k] = neb_addr[k] - MCU_GET_AFF_XOFF( map_affine[neb_addr[k]] ) - w_scu * MCU_GET_AFF_YOFF( map_affine[neb_addr[k]] );
            }
        }

        if ( valid_flag[2] && valid_flag[1] && top_left[1] == top_left[2] )
        {
            valid_flag[2] = 0;
        }

        if(valid_flag[3] && valid_flag[0] && top_left[0] == top_left[3])
        {
            valid_flag[3] = 0;
        }

        if((valid_flag[4] && valid_flag[0] && top_left[4] == top_left[0]) || (valid_flag[4] && valid_flag[1] && top_left[4] == top_left[1]))
        {
            valid_flag[4] = 0;
        }

        for(k = 0; k < 5; k++)
        {
            if(valid_flag[k])
            {
                // set vertex number: affine flag == 1, set to 2 vertex, otherwise, set to 3 vertex
                mrg_list_cp_num[cnt] = (MCU_GET_AFF(map_scu[neb_addr[k]]) == 1) ? 2 : 3;

                for(lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    if(REFI_IS_VALID(map_refi[neb_addr[k]][lidx]))
                    {
                        mrg_list_refi[cnt][lidx] = map_refi[neb_addr[k]][lidx];
                        evc_derive_affine_model_mv(scup, neb_addr[k], lidx, map_mv, cuw, cuh, w_scu, h_scu, mrg_list_cpmv[cnt][lidx], map_affine, mrg_list_cp_num[cnt]
#if M48933_AFFINE
                                                   , log2_max_cuwh
#endif
#if DMVR_LAG
                                                   , map_scu
                                                   , map_unrefined_mv
#endif
                        );
                    }
                    else // set to default value
                    {
                        mrg_list_refi[cnt][lidx] = -1;
                        for(i = 0; i < VER_NUM; i++)
                        {
                            mrg_list_cpmv[cnt][lidx][i][MV_X] = 0;
                            mrg_list_cpmv[cnt][lidx][i][MV_Y] = 0;
                        }
                    }
                }
                cnt++;
            }

            if(cnt >= AFF_MODEL_CAND) // one candidate in current stage
            {
                break;
            }
        }
    }

    //-------------------  control point based affine MVP  -------------------//
    {
#if !M50761_TMVP_ALIGN_SPEC || !M50662_AFFINE_IBC_TMVP_SUCO_FIX
        s8(*map_refi_co)[REFP_NUM];
        int dptr_co = 0;
        int ratio_tmvp = 1;
#endif

        s16 cp_mv[REFP_NUM][VER_NUM][MV_D];
        int cp_refi[REFP_NUM][VER_NUM];
        int cp_valid[VER_NUM];

        int neb_addr_lt[AFFINE_MAX_NUM_LT];
        int neb_addr_rt[AFFINE_MAX_NUM_RT];
        int neb_addr_lb[AFFINE_MAX_NUM_LB];
        int neb_addr_rb[AFFINE_MAX_NUM_RB];

        int valid_flag_lt[AFFINE_MAX_NUM_LT];
        int valid_flag_rt[AFFINE_MAX_NUM_RT];
        int valid_flag_lb[AFFINE_MAX_NUM_LB];
        int valid_flag_rb[AFFINE_MAX_NUM_RB];

        //------------------  INIT  ------------------//
        for(i = 0; i < VER_NUM; i++)
        {
            for(lidx = 0; lidx < REFP_NUM; lidx++)
            {
                cp_mv[lidx][i][MV_X] = 0;
                cp_mv[lidx][i][MV_Y] = 0;
                cp_refi[lidx][i] = -1;
            }
            cp_valid[i] = 0;
        }

        //-------------------  LT  -------------------//
#if M50662_AFFINE_ALTERNATIVE_SCANNING_ORDER
        neb_addr_lt[0] = scup - w_scu - 1;
        neb_addr_lt[1] = scup - w_scu;
        neb_addr_lt[2] = scup - 1;

        valid_flag_lt[0] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[0]]) && !MCU_GET_IF(map_scu[neb_addr_lt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[0]]);
        valid_flag_lt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[1]]) && !MCU_GET_IF(map_scu[neb_addr_lt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[1]]);
        valid_flag_lt[2] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[2]]) && !MCU_GET_IF(map_scu[neb_addr_lt[2]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[2]]);
#else
        neb_addr_lt[0] = scup - 1;          // LE
        neb_addr_lt[1] = scup - w_scu;      // UP
        neb_addr_lt[2] = scup - w_scu - 1;  // B2
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
        valid_flag_lt[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[0]]) && !MCU_GET_IF(map_scu[neb_addr_lt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[0]]);
        valid_flag_lt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[1]]) && !MCU_GET_IF(map_scu[neb_addr_lt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[1]]);
        valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[2]]) && !MCU_GET_IF(map_scu[neb_addr_lt[2]]) && !MCU_GET_IBC(map_scu[neb_addr_lt[2]]);
#else
        valid_flag_lt[0] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[0]]) && !MCU_GET_IF(map_scu[neb_addr_lt[0]]);
        valid_flag_lt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[1]]) && !MCU_GET_IF(map_scu[neb_addr_lt[1]]);
        valid_flag_lt[2] = x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lt[2]]) && !MCU_GET_IF(map_scu[neb_addr_lt[2]]);
#endif
#endif
        for (k = 0; k < AFFINE_MAX_NUM_LT; k++)
        {
            if (valid_flag_lt[k])
            {
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    cp_refi[lidx][0] = map_refi[neb_addr_lt[k]][lidx];
                    if (MCU_GET_DMVRF(map_scu[neb_addr_lt[k]])
#if (DMVR_LAG == 2)
                      && (!evc_use_refine_mv(scup, neb_addr_lt[k], w_scu))
#endif
                      )
                    {
                        cp_mv[lidx][0][MV_X] = map_unrefined_mv[neb_addr_lt[k]][lidx][MV_X];
                        cp_mv[lidx][0][MV_Y] = map_unrefined_mv[neb_addr_lt[k]][lidx][MV_Y];
                    }
                    else
                    {
                        cp_mv[lidx][0][MV_X] = map_mv[neb_addr_lt[k]][lidx][MV_X];
                        cp_mv[lidx][0][MV_Y] = map_mv[neb_addr_lt[k]][lidx][MV_Y];
                    }
                }
                cp_valid[0] = 1;
                break;
            }
        }

        //-------------------  RT  -------------------//
#if M50662_AFFINE_ALTERNATIVE_SCANNING_ORDER
        neb_addr_rt[0] = scup - w_scu + scuw;
        neb_addr_rt[1] = scup - w_scu + scuw - 1;

        valid_flag_rt[0] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[0]]) && !MCU_GET_IF(map_scu[neb_addr_rt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[0]]);
        valid_flag_rt[1] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_rt[1]]) && !MCU_GET_IF(map_scu[neb_addr_rt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[1]]);
#else
        neb_addr_rt[0] = scup - w_scu + scuw - 1;     // B1
        neb_addr_rt[1] = scup - w_scu + scuw;         // B0

#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
        valid_flag_rt[0] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_rt[0]]) && !MCU_GET_IF(map_scu[neb_addr_rt[0]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[0]]);
        valid_flag_rt[1] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[1]]) && !MCU_GET_IF(map_scu[neb_addr_rt[1]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[1]]);
#else
        valid_flag_rt[0] = y_scu > 0 && MCU_GET_COD(map_scu[neb_addr_rt[0]]) && !MCU_GET_IF(map_scu[neb_addr_rt[0]]);
        valid_flag_rt[1] = y_scu > 0 && x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[1]]) && !MCU_GET_IF(map_scu[neb_addr_rt[1]]);
#endif
#endif
#if M48879_IMPROVEMENT_SUCO
        neb_addr_rt[2] = scup + scuw;                 // RIGHT
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
        valid_flag_rt[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[2]]) && !MCU_GET_IF(map_scu[neb_addr_rt[2]]) && !MCU_GET_IBC(map_scu[neb_addr_rt[2]]);
#else
        valid_flag_rt[2] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rt[2]]) && !MCU_GET_IF(map_scu[neb_addr_rt[2]]);
#endif
#endif

        for (k = 0; k < AFFINE_MAX_NUM_RT; k++)
        {
            if (valid_flag_rt[k])
            {
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    cp_refi[lidx][1] = map_refi[neb_addr_rt[k]][lidx];
                    if (MCU_GET_DMVRF(map_scu[neb_addr_rt[k]])
#if (DMVR_LAG == 2)
                      && (!evc_use_refine_mv(scup, neb_addr_rt[k], w_scu))
#endif
                        )
                    {
                        cp_mv[lidx][0][MV_X] = map_unrefined_mv[neb_addr_rt[k]][lidx][MV_X];
                        cp_mv[lidx][0][MV_Y] = map_unrefined_mv[neb_addr_rt[k]][lidx][MV_Y];
                    }
                    else
                    {
                        cp_mv[lidx][1][MV_X] = map_mv[neb_addr_rt[k]][lidx][MV_X];
                        cp_mv[lidx][1][MV_Y] = map_mv[neb_addr_rt[k]][lidx][MV_Y];
                    }
                }
                cp_valid[1] = 1;
                break;
            }
        }

        //-------------------  LB  -------------------//
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
        if (avail_lr == LR_10 || avail_lr == LR_11)
        {
#endif
        neb_addr_lb[0] = scup + w_scu * scuh - 1;        // A0
        neb_addr_lb[1] = scup + w_scu * (scuh - 1) - 1;  // A1
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
        valid_flag_lb[0] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_lb[0]]) && !MCU_GET_IF(map_scu[neb_addr_lb[0]]) && !MCU_GET_IBC(map_scu[neb_addr_lb[0]]);
        valid_flag_lb[1] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lb[1]]) && !MCU_GET_IF(map_scu[neb_addr_lb[1]]) && !MCU_GET_IBC(map_scu[neb_addr_lb[1]]);
#else
        valid_flag_lb[0] = x_scu > 0 && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_lb[0]]) && !MCU_GET_IF(map_scu[neb_addr_lb[0]]);
        valid_flag_lb[1] = x_scu > 0 && MCU_GET_COD(map_scu[neb_addr_lb[1]]) && !MCU_GET_IF(map_scu[neb_addr_lb[1]]);
#endif

        for (k = 0; k < AFFINE_MAX_NUM_LB; k++)
        {
            if (valid_flag_lb[k])
            {
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    cp_refi[lidx][2] = map_refi[neb_addr_lb[k]][lidx];
#if M50761_DMVR_BUGFIX_REFINED_MV_RESTRICTION
                    if (MCU_GET_DMVRF(map_scu[neb_addr_lb[k]]))
                    {
                      cp_mv[lidx][2][MV_X] = map_unrefined_mv[neb_addr_lb[k]][lidx][MV_X];
                      cp_mv[lidx][2][MV_Y] = map_unrefined_mv[neb_addr_lb[k]][lidx][MV_Y];
                    }
                    else
                    {
                      cp_mv[lidx][2][MV_X] = map_mv[neb_addr_lb[k]][lidx][MV_X];
                      cp_mv[lidx][2][MV_Y] = map_mv[neb_addr_lb[k]][lidx][MV_Y];
                    }
#else

                    cp_mv[lidx][2][MV_X] = map_mv[neb_addr_lb[k]][lidx][MV_X];
                    cp_mv[lidx][2][MV_Y] = map_mv[neb_addr_lb[k]][lidx][MV_Y];
#endif
                }
                cp_valid[2] = 1;
                break;
            }
        }
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
        }
        else
        {
            neb_addr_lb[0] = scup + w_scu * scuh - 1;
            valid_flag_lb[0] = x_scu > 0 && y_scu + scuh < h_scu;
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
            if (valid_flag_lb[0])
            {
                evc_get_mv_collocated(
                    refp,
                    ptr, neb_addr_lb[0], scup, w_scu, h_scu, tmvp, &availablePredIdx
#if M49023_ADMVP_IMPROVE 
                    , sh
#endif
                );
                if ((availablePredIdx == 1) || (availablePredIdx == 3))
                {
                    cp_refi[REFP_0][2] = 0;
                    cp_mv[REFP_0][2][MV_X] = tmvp[REFP_0][MV_X];
                    cp_mv[REFP_0][2][MV_Y] = tmvp[REFP_0][MV_Y];
                }
                else
                {
                    cp_refi[0][2] = REFI_INVALID;
                    cp_mv[REFP_0][2][MV_X] = 0;
                    cp_mv[REFP_0][2][MV_Y] = 0;
                }
                if ((availablePredIdx == 2) || (availablePredIdx == 3))
                {
                    cp_refi[REFP_1][2] = 0;
                    cp_mv[REFP_1][2][MV_X] = tmvp[REFP_1][MV_X];
                    cp_mv[REFP_1][2][MV_Y] = tmvp[REFP_1][MV_Y];
                }
                else
                {
                    cp_refi[REFP_1][2] = REFI_INVALID;
                    cp_mv[REFP_1][2][MV_X] = 0;
                    cp_mv[REFP_1][2][MV_Y] = 0;
                }
            }
            if (REFI_IS_VALID(cp_refi[REFP_0][2]) || REFI_IS_VALID(cp_refi[REFP_1][2]))
            {
                cp_valid[2] = 1;
            }
#else
            if (valid_flag_lb[0])
            {
                map_refi_co = refp[0][REFP_1].map_refi; // col picture is ref idx 0 and list 1
                for (lidx = 0; lidx < REFP_NUM; lidx++)
                {
                    if (REFI_IS_VALID(map_refi_co[neb_addr_lb[0]][lidx]))
                    {
                        dptr_co = refp[0][REFP_1].ptr - refp[0][REFP_1].list_ptr[map_refi_co[neb_addr_lb[0]][lidx]];
                        if (dptr_co == 0)
                        {
                            break;
                        }

                        ratio_tmvp = (int)((ptr - refp[0][REFP_1].ptr) << MVP_SCALING_PRECISION) / dptr_co;
                        cp_refi[lidx][2] = 0; // ref idx
                        scaling_mv(ratio_tmvp, refp[0][REFP_1].map_mv[neb_addr_lb[0]][lidx], cp_mv[lidx][2]);
                        cp_valid[2] = 1;
                    }
                }
            }
#endif
        }
#endif
        //-------------------  RB  -------------------//
#if M48879_IMPROVEMENT_SUCO
        if (avail_lr == LR_01 || avail_lr == LR_11)
        {
            neb_addr_rb[0] = scup + w_scu * scuh + scuw;
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
            valid_flag_rb[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_rb[0]]) && !MCU_GET_IF(map_scu[neb_addr_rb[0]]) && !MCU_GET_IBC(map_scu[neb_addr_rb[0]]);
#else
            valid_flag_rb[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && MCU_GET_COD(map_scu[neb_addr_rb[0]]) && !MCU_GET_IF(map_scu[neb_addr_rb[0]]);
#endif

            neb_addr_rb[1] = scup + w_scu * (scuh - 1) + scuw;
#if M50662_AFFINE_IBC_TMVP_SUCO_FIX
            valid_flag_rb[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rb[1]]) && !MCU_GET_IF(map_scu[neb_addr_rb[1]]) && !MCU_GET_IBC(map_scu[neb_addr_rb[1]]);
#else
            valid_flag_rb[1] = x_scu + scuw < w_scu && MCU_GET_COD(map_scu[neb_addr_rb[1]]) && !MCU_GET_IF(map_scu[neb_addr_rb[1]]);
#endif

            for (k = 0; k < AFFINE_MAX_NUM_RB; k++)
            {
                if (valid_flag_rb[k])
                {
                    for (lidx = 0; lidx < REFP_NUM; lidx++)
                    {

                        cp_refi[lidx][3] = map_refi[neb_addr_rb[k]][lidx];
#if M50761_DMVR_BUGFIX_REFINED_MV_RESTRICTION
                        if (MCU_GET_DMVRF(map_scu[neb_addr_rb[k]]))
                        {
                          cp_mv[lidx][3][MV_X] = map_unrefined_mv[neb_addr_rb[k]][lidx][MV_X];
                          cp_mv[lidx][3][MV_Y] = map_unrefined_mv[neb_addr_rb[k]][lidx][MV_Y];
                        }
                        else
                        {
                          cp_mv[lidx][3][MV_X] = map_mv[neb_addr_rb[k]][lidx][MV_X];
                          cp_mv[lidx][3][MV_Y] = map_mv[neb_addr_rb[k]][lidx][MV_Y];
                        }
#else
                        cp_mv[lidx][3][MV_X] = map_mv[neb_addr_rb[k]][lidx][MV_X];
                        cp_mv[lidx][3][MV_Y] = map_mv[neb_addr_rb[k]][lidx][MV_Y];
#endif
                    }
                    break;
                }
            }
        }
        else
        {
#endif

#if M50761_TMVP_ALIGN_SPEC
        s32 isSameCtuLine = ((y_scu + scuh) << MIN_CU_LOG2 >> log2_max_cuwh) == (y_scu << MIN_CU_LOG2 >> log2_max_cuwh);
        valid_flag_rb[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu && isSameCtuLine;

        if ( valid_flag_rb[0] )
        {
            s16 tmvp[REFP_NUM][MV_D];
            s8 availablePredIdx = 0;

            neb_addr_rb[0] = ((x_scu + scuw) >> 1 << 1) + ((y_scu + scuh) >> 1 << 1) * w_scu; // 8x8 grid
            evc_get_mv_collocated( refp, ptr, neb_addr_rb[0], scup, w_scu, h_scu, tmvp, &availablePredIdx, sh );

            if ( (availablePredIdx == 1) || (availablePredIdx == 3) )
            {
                cp_refi[0][3] = 0;
                cp_mv[0][3][MV_X] = tmvp[REFP_0][MV_X];
                cp_mv[0][3][MV_Y] = tmvp[REFP_0][MV_Y];
            }
            else
            {
                cp_refi[0][3] = REFI_INVALID;
                cp_mv[0][3][MV_X] = 0;
                cp_mv[0][3][MV_Y] = 0;
            }

            if ( ((availablePredIdx == 2) || (availablePredIdx == 3)) && slice_type == SLICE_B )
            {
                cp_refi[1][3] = 0;
                cp_mv[1][3][MV_X] = tmvp[REFP_1][MV_X];
                cp_mv[1][3][MV_Y] = tmvp[REFP_1][MV_Y];
            }
            else
            {
                cp_refi[1][3] = REFI_INVALID;
                cp_mv[1][3][MV_X] = 0;
                cp_mv[1][3][MV_Y] = 0;
            }
        }
#else
        neb_addr_rb[0] = scup + w_scu * scuh + scuw;     // Col
        valid_flag_rb[0] = x_scu + scuw < w_scu && y_scu + scuh < h_scu;

        if (valid_flag_rb[0])
        {
            map_refi_co = refp[0][REFP_1].map_refi; // col picture is ref idx 0 and list 1
            for (lidx = 0; lidx < REFP_NUM; lidx++)
            {
                if(slice_type == SLICE_B && REFI_IS_VALID(map_refi_co[neb_addr_rb[0]][lidx]))
                {
                    dptr_co = refp[0][REFP_1].ptr - refp[0][REFP_1].list_ptr[map_refi_co[neb_addr_rb[0]][lidx]];
                    if (dptr_co == 0)
                    {
                        break;
                    }

                    ratio_tmvp = (int)((ptr - refp[0][REFP_1].ptr) << MVP_SCALING_PRECISION) / dptr_co;
                    cp_refi[lidx][3] = 0; // ref idx
                    scaling_mv(ratio_tmvp, refp[0][REFP_1].map_mv[neb_addr_rb[0]][lidx], cp_mv[lidx][3]);   //SEMIH:this is temporal MV, it is ok.
                }
            }
        }
#endif
#if M48879_IMPROVEMENT_SUCO
        }
#endif
        if (REFI_IS_VALID(cp_refi[REFP_0][3]) || REFI_IS_VALID(cp_refi[REFP_1][3]))
        {
            cp_valid[3] = 1;
        }

        //-------------------  insert model  -------------------//
        int const_order[6] = { 0, 1, 2, 3, 4, 5 };
        int const_num = 6;

        int idx = 0;
        int const_model[6][VER_NUM] =
        {
            { 0, 1, 2 },          // 0: LT, RT, LB
            { 0, 1, 3 },          // 1: LT, RT, RB
            { 0, 2, 3 },          // 2: LT, LB, RB
            { 1, 2, 3 },          // 3: RT, LB, RB
            { 0, 1 },             // 4: LT, RT
            { 0, 2 },             // 5: LT, LB
        };

        int cp_num[6] = { 3, 3, 3, 3, 2, 2 };
        for ( idx = 0; idx < const_num; idx++ )
        {
            int const_idx = const_order[idx];
            evc_derive_affine_constructed_candidate( ptr, refp, cuw, cuh, cp_valid, cp_mv, cp_refi, const_model[const_idx], const_idx, cp_num[const_idx], mrg_list_cpmv, mrg_list_refi, &cnt, mrg_list_cp_num );
        }
    }

    // Zero padding
    int cnt_wo_padding = cnt;
    {
        int cp_idx;
        for ( ; cnt < AFF_MAX_CAND; cnt++ )
        {
            mrg_list_cp_num[cnt] = 2;
            for ( lidx = 0; lidx < REFP_NUM; lidx++ )
            {
                for ( cp_idx = 0; cp_idx < 2; cp_idx++ )
                {
                    mrg_list_cpmv[cnt][lidx][cp_idx][MV_X] = 0;
                    mrg_list_cpmv[cnt][lidx][cp_idx][MV_Y] = 0;
                }
            }
            mrg_list_refi[cnt][REFP_0] = 0;
            mrg_list_refi[cnt][REFP_1] = (slice_type == SLICE_B) ? 0 : REFI_INVALID;
        }
    }

    return cnt_wo_padding; // return value only used for encoder
}
#endif

#if ADCC
void evc_get_ctx_last_pos_xy_para(int ch_type, int width, int height, int *result_offset_x, int *result_offset_y, int *result_shift_x, int *result_shift_y)
{
    int convertedWidth = CONV_LOG2(width) - 2;
    int convertedHeight = CONV_LOG2(height) - 2;
    convertedWidth = (convertedWidth < 0) ? 0 : convertedWidth;
    convertedHeight = (convertedHeight < 0) ? 0 : convertedHeight;

    *result_offset_x = (ch_type != Y_C) ? 0 : ((convertedWidth * 3) + ((convertedWidth + 1) >> 2));
    *result_offset_y = (ch_type != Y_C) ? 0 : ((convertedHeight * 3) + ((convertedHeight + 1) >> 2));
    *result_shift_x = (ch_type != Y_C) ? convertedWidth - CONV_LOG2(width >> 4) : ((convertedWidth + 3) >> 2);
    *result_shift_y = (ch_type != Y_C) ? convertedHeight - CONV_LOG2(height >> 4) : ((convertedHeight + 3) >> 2);

    if (ch_type == Y_C)
    {
        if (convertedWidth >= 4)
        {
            *result_offset_x += ((width >> 6) << 1) + (width >> 7);
        }
        if (convertedHeight >= 4)
        {
            *result_offset_y += ((height >> 6) << 1) + (height >> 7);
        }
    }
}

int evc_get_ctx_gt0_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int diag = pos_x + pos_y;
    int num_gt0 = 0;
    int ctx_idx;
    int ctx_ofs;

    if(pos_x < width_m1)
    {
        num_gt0 += pdata[1] != 0;
        if(pos_x < width_m1 - 1)
        {
            num_gt0 += pdata[2] != 0;
        }
        if(pos_y < height_m1)
        {
            num_gt0 += pdata[width + 1] != 0;
        }
    }

    if(pos_y < height_m1)
    {
        num_gt0 += pdata[width] != 0;
        if(pos_y < height_m1 - 1)
        {
            num_gt0 += pdata[2 * width] != 0;
        }
    }

    ctx_idx = EVC_MIN(num_gt0, 4) + 1;

    if(diag < 2)
    {
        ctx_idx = EVC_MIN(ctx_idx, 2);
    }

    if(ch_type == Y_C)
    {
        ctx_ofs = diag < 2 ? 0 : (diag < 5 ? 2 : 7);
    }
    else
    {
        ctx_ofs = diag < 2 ? 0 : 2;
    }

    return ctx_ofs + ctx_idx;
}

int evc_get_ctx_gtA_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int num_gtA = 0;

    if(pos_x < width_m1)
    {
        num_gtA += EVC_ABS16(pdata[1]) > 1;
        if(pos_x < width_m1 - 1)
        {
            num_gtA += EVC_ABS16(pdata[2]) > 1;
        }
        if(pos_y < height_m1)
        {
            num_gtA += EVC_ABS16(pdata[width + 1]) > 1;
        }
    }

    if(pos_y < height_m1)
    {
        num_gtA += EVC_ABS16(pdata[width]) > 1;
        if(pos_y < height_m1 - 1)
        {
            num_gtA += EVC_ABS16(pdata[2 * width]) > 1;
        }
    }

    num_gtA = EVC_MIN(num_gtA, 3) + 1;
    if(ch_type == Y_C)
    {
        num_gtA += (pos_x == 0 && pos_y == 0) ? 0 : ((pos_x <= sr_x / 2 && pos_y <= sr_y / 2) ? 4 : 8);
    }
    return num_gtA;
}

int evc_get_ctx_gtB_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int diag = pos_x + pos_y;
    int num_gtB = 0;

    if(pos_x < width_m1)
    {
        num_gtB += EVC_ABS16(pdata[1]) > 2;
        if(pos_x < width_m1 - 1)
        {
            num_gtB += EVC_ABS16(pdata[2]) > 2;
        }
        if(pos_y < height_m1)
        {
            num_gtB += EVC_ABS16(pdata[width + 1]) > 2;
        }
    }

    if(pos_y < height_m1)
    {
        num_gtB += EVC_ABS16(pdata[width]) > 2;
        if(pos_y < height_m1 - 1)
        {
            num_gtB += EVC_ABS16(pdata[2 * width]) > 2;
        }
    }

    num_gtB = EVC_MIN(num_gtB, 3) + 1;
    if(ch_type == Y_C)
    {
        num_gtB += (pos_x == 0 && pos_y == 0) ? 0 : ((pos_x <= sr_x / 2 && pos_y <= sr_y / 2) ? 4 : 8);
    }
    return num_gtB;
}

int evc_get_ctx_remain_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);

    pdata = pcoeff + pos_x + (pos_y << log2_w);



    int numPos = 0;
    int sumAbsAll = 0;
    if (pos_x < width_m1)
    {
        sumAbsAll += abs(pdata[1]);
        numPos += pdata[1] != 0;
        if (pos_x < width_m1 - 1)
        {
            sumAbsAll += abs(pdata[2]);
            numPos += pdata[2] != 0;
        }
        if (pos_y < height_m1)
        {
            sumAbsAll += abs(pdata[width + 1]);
            numPos += pdata[width + 1] != 0;
        }
    }
    if (pos_y < height_m1)
    {
        sumAbsAll += abs(pdata[width]);
        numPos += pdata[width] != 0;
        if (pos_y < height_m1 - 1)
        {
            sumAbsAll += abs(pdata[2 * width]);
            numPos += pdata[2 * width] != 0;
        }
    }

    int uiVal = (sumAbsAll - numPos);
    int iOrder;
    for (iOrder = 0; iOrder < MAX_GR_ORDER_RESIDUAL; iOrder++)
    {
        if ((1 << (iOrder + 3)) >(uiVal + 4))
        {
            break;
        }
    }
    return (iOrder == MAX_GR_ORDER_RESIDUAL ? (MAX_GR_ORDER_RESIDUAL - 1) : iOrder);
}

int get_rice_para(s16 *pcoeff, int blkpos, int width, int height, int base_level)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    int sum_abs = 0;

    if (pos_x < width_m1)
    {
        sum_abs += EVC_ABS16(pdata[1]);
        if (pos_x < width_m1 - 1)
        {
            sum_abs += EVC_ABS16(pdata[2]);
        }
        if (pos_y < height_m1)
        {
            sum_abs += EVC_ABS16(pdata[width + 1]);
        }
    }

    if (pos_y < height_m1)
    {
        sum_abs += EVC_ABS16(pdata[width]);
        if (pos_y < height_m1 - 1)
        {
            sum_abs += EVC_ABS16(pdata[2 * width]);
        }
    }
    sum_abs = EVC_MAX(EVC_MIN(sum_abs - 5 * base_level, 31), 0);
    return g_go_rice_para_coeff[sum_abs];
}

void evc_init_scan_sr(int *scan, int size_x, int size_y, int width, int height, int scan_type)
{
    int x, y, l, pos, num_line;

    pos = 0;
    num_line = size_x + size_y - 1;
    if(scan_type == COEF_SCAN_ZIGZAG)
    {
        /* starting point */
        scan[pos] = 0;
        pos++;

        /* loop */
        for(l = 1; l < num_line; l++)
        {
            if(l % 2) /* decreasing loop */
            {
                x = EVC_MIN(l, size_x - 1);
                y = EVC_MAX(0, l - (size_x - 1));

                while(x >= 0 && y < size_y)
                {
                    scan[pos] = y * width + x;
                    pos++;
                    x--;
                    y++;
                }
            }
            else /* increasing loop */
            {
                y = EVC_MIN(l, size_y - 1);
                x = EVC_MAX(0, l - (size_y - 1));
                while(y >= 0 && x < size_x)
                {
                    scan[pos] = y * width + x;
                    pos++;
                    x++;
                    y--;
                }
            }
        }
    }
}

void evc_init_inverse_scan_sr(u16 *scan_inv, u16 *scan_orig, int width, int height, int scan_type)
{
    int x, num_line;

    num_line = width*height;
    if ( (scan_type == COEF_SCAN_ZIGZAG) || (scan_type == COEF_SCAN_DIAG) || (scan_type == COEF_SCAN_DIAG_CG) )
    {
        for ( x = 0; x < num_line; x++)
        {
            int scan_pos = scan_orig[x];
            assert(scan_pos >= 0);
            assert(scan_pos < num_line);
            scan_inv[scan_pos] = x;
        }
    }
    else
    {
        printf("Not supported scan_type\n");
    }
}

#endif

int evc_get_transform_shift(int log2_size, int type)
{
    return (type == 0) ? TX_SHIFT1(log2_size) : TX_SHIFT2(log2_size);
}

void evc_eco_sbac_ctx_initialize(SBAC_CTX_MODEL *ctx, s16 *ctx_init_model, u16 num_ctx, u8 slice_type, u8 slice_qp)
{
    int i, slope, offset;
    u16 cmps, p0, p1;
    int qp = EVC_CLIP3(0, 51, slice_qp);
    int is_inter_slice = (slice_type == SLICE_B || slice_type == SLICE_P);
#if CTX_REPRESENTATION_IMPROVEMENT
    ctx_init_model += (is_inter_slice * num_ctx * 1);
#else
    ctx_init_model += (is_inter_slice * num_ctx * 2);
#endif
    for(i = 0; i < num_ctx; i++)
    {
#if CTX_REPRESENTATION_IMPROVEMENT
        int tmp = *(ctx_init_model);
        int m;
        int SLOPE_BITS = 4;
        m = (tmp & 14) >> 1;  //(tmp & 0b1110)
        m = m << 5;
        m = (tmp & 1) ? -m : m;
        
        tmp = *(ctx_init_model) >> SLOPE_BITS;
        int c;
        c = (tmp & 62) >> 1;    //(tmp & 0b111110)
        c = c << 8;
        c = (tmp & 1) ? -c : c;;
        c += 4096;
        slope = m;
        offset = c;
#else
        slope = *(ctx_init_model);
        offset = *(ctx_init_model + 1);
#endif

#if PROB_INIT_FIX
        int p = slope * qp + offset;
        int shift = MCABAC_PROB_BITS - 13;
        if (shift < 0)
          p >>= (-shift);
        else
          p <<= shift;
        p0 = (u16)EVC_CLIP3(1, MAX_PROB - 1, p);
#else
        p0 = (u16)EVC_CLIP3(1, MAX_PROB - 1, slope * qp + offset);
#endif
        p1 = p0;
        cmps = 1;
        if(p0 + p1 > MAX_PROB)
        {
            p0 = MAX_PROB - p0;
            p1 = MAX_PROB - p1;
            cmps = 0;
        }
        ctx[i] = (p0 << MPS_SHIFT) + (p1 << 1) + cmps;
#if CTX_REPRESENTATION_IMPROVEMENT
        ctx_init_model += 1;
#else
        ctx_init_model += 2;
#endif
    }
}

int evc_split_part_count(int split_mode)
{
    switch(split_mode)
    {
        case SPLIT_BI_VER:
        case SPLIT_BI_HOR:
            return 2;
        case SPLIT_TRI_VER:
        case SPLIT_TRI_HOR:
            return 3;
        case SPLIT_QUAD:
            return 4;
        default:
            // NO_SPLIT
            return 0;
    }
}

int evc_split_get_part_size(int split_mode, int part_num, int length)
{
    int ans = length;
    switch(split_mode)
    {
        case SPLIT_QUAD:
        case SPLIT_BI_HOR:
        case SPLIT_BI_VER:
            ans = length >> 1;
            break;
        case SPLIT_TRI_HOR:
        case SPLIT_TRI_VER:
            if(part_num == 1)
                ans = length >> 1;
            else
                ans = length >> 2;
            break;
    }
    return ans;
}

int evc_split_get_part_size_idx(int split_mode, int part_num, int length_idx)
{
    int ans = length_idx;
    switch(split_mode)
    {
        case SPLIT_QUAD:
        case SPLIT_BI_HOR:
        case SPLIT_BI_VER:
            ans = length_idx - 1;
            break;
        case SPLIT_TRI_HOR:
        case SPLIT_TRI_VER:
            if(part_num == 1)
                ans = length_idx - 1;
            else
                ans = length_idx - 2;
            break;
    }
    return ans;
}

void evc_split_get_part_structure(int split_mode, int x0, int y0, int cuw, int cuh, int cup, int cud, int log2_culine, EVC_SPLIT_STRUCT* split_struct
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons /*, u8 slice_type */
#endif
)
{
    int i;
    int log_cuw, log_cuh;
    int cup_w, cup_h;

    split_struct->part_count = evc_split_part_count(split_mode);
    log_cuw = CONV_LOG2(cuw);
    log_cuh = CONV_LOG2(cuh);
    split_struct->x_pos[0] = x0;
    split_struct->y_pos[0] = y0;
    split_struct->cup[0] = cup;
#if M50761_CHROMA_NOT_SPLIT
    split_struct->tree_cons = tree_cons;
    if (!tree_cons.changed && !evc_tree_split_allowed(cuw, cuh, split_mode))
    {
        split_struct->tree_cons.changed = TRUE;
    }
#endif

    switch(split_mode)
    {
        case NO_SPLIT:
        {
            split_struct->width[0] = cuw;
            split_struct->height[0] = cuh;
            split_struct->log_cuw[0] = log_cuw;
            split_struct->log_cuh[0] = log_cuh;
        }
        break;

        case SPLIT_QUAD:
        {
            split_struct->width[0] = cuw >> 1;
            split_struct->height[0] = cuh >> 1;
            split_struct->log_cuw[0] = log_cuw - 1;
            split_struct->log_cuh[0] = log_cuh - 1;
            for(i = 1; i < split_struct->part_count; ++i)
            {
                split_struct->width[i] = split_struct->width[0];
                split_struct->height[i] = split_struct->height[0];
                split_struct->log_cuw[i] = split_struct->log_cuw[0];
                split_struct->log_cuh[i] = split_struct->log_cuh[0];
            }
            split_struct->x_pos[1] = x0 + split_struct->width[0];
            split_struct->y_pos[1] = y0;
            split_struct->x_pos[2] = x0;
            split_struct->y_pos[2] = y0 + split_struct->height[0];
            split_struct->x_pos[3] = split_struct->x_pos[1];
            split_struct->y_pos[3] = split_struct->y_pos[2];
            cup_w = (split_struct->width[0] >> MIN_CU_LOG2);
            cup_h = ((split_struct->height[0] >> MIN_CU_LOG2) << log2_culine);
            split_struct->cup[1] = cup + cup_w;
            split_struct->cup[2] = cup + cup_h;
            split_struct->cup[3] = split_struct->cup[1] + cup_h;
            split_struct->cud[0] = cud + 2;
            split_struct->cud[1] = cud + 2;
            split_struct->cud[2] = cud + 2;
            split_struct->cud[3] = cud + 2;
        }
        break;

        default:
        {
            if(evc_split_is_vertical(split_mode))
            {
                for(i = 0; i < split_struct->part_count; ++i)
                {
                    split_struct->width[i] = evc_split_get_part_size(split_mode, i, cuw);
                    split_struct->log_cuw[i] = evc_split_get_part_size_idx(split_mode, i, log_cuw);
                    split_struct->height[i] = cuh;
                    split_struct->log_cuh[i] = log_cuh;
                    if(i)
                    {
                        split_struct->x_pos[i] = split_struct->x_pos[i - 1] + split_struct->width[i - 1];
                        split_struct->y_pos[i] = split_struct->y_pos[i - 1];
                        split_struct->cup[i] = split_struct->cup[i - 1] + (split_struct->width[i - 1] >> MIN_CU_LOG2);
                    }
                }
            }
            else
            {
                for(i = 0; i < split_struct->part_count; ++i)
                {
                    split_struct->width[i] = cuw;
                    split_struct->log_cuw[i] = log_cuw;
                    split_struct->height[i] = evc_split_get_part_size(split_mode, i, cuh);
                    split_struct->log_cuh[i] = evc_split_get_part_size_idx(split_mode, i, log_cuh);
                    if(i)
                    {
                        split_struct->y_pos[i] = split_struct->y_pos[i - 1] + split_struct->height[i - 1];
                        split_struct->x_pos[i] = split_struct->x_pos[i - 1];
                        split_struct->cup[i] = split_struct->cup[i - 1] + ((split_struct->height[i - 1] >> MIN_CU_LOG2) << log2_culine);
                    }
                }
            }
            switch(split_mode)
            {
                case SPLIT_BI_VER:
                    split_struct->cud[0] = cud + 1;
                    split_struct->cud[1] = cud + 1;
                    break;
                case SPLIT_BI_HOR:
                    split_struct->cud[0] = cud + 1;
                    split_struct->cud[1] = cud + 1;
                    break;
                default:
                    // Triple tree case
                    split_struct->cud[0] = cud + 2;
                    split_struct->cud[1] = cud + 1;
                    split_struct->cud[2] = cud + 2;
                    break;
            }
        }
        break;
    }
}

void evc_split_get_split_rdo_order(int cuw, int cuh, SPLIT_MODE splits[MAX_SPLIT_NUM])
{
    if(cuw < cuh)
    {
        splits[1] = SPLIT_BI_HOR;
        splits[2] = SPLIT_BI_VER;
    }
    else
    {
        splits[1] = SPLIT_BI_VER;
        splits[2] = SPLIT_BI_HOR;
    }
    splits[3] = SPLIT_TRI_VER;
    splits[4] = SPLIT_TRI_HOR;
    splits[5] = SPLIT_QUAD;
    splits[0] = NO_SPLIT;
}

SPLIT_DIR evc_split_get_direction(SPLIT_MODE mode)
{
    switch(mode)
    {
        case SPLIT_BI_HOR:
        case SPLIT_TRI_HOR:
            return SPLIT_HOR;
        default:
            return SPLIT_VER;
    }
}

int evc_split_is_vertical(SPLIT_MODE mode)
{
    return evc_split_get_direction(mode) == SPLIT_VER ? 1 : 0;
}

int evc_split_is_horizontal(SPLIT_MODE mode)
{
    return evc_split_get_direction(mode) == SPLIT_HOR ? 1 : 0;
}

void evc_split_get_suco_order(int suco_flag, SPLIT_MODE mode, int suco_order[SPLIT_MAX_PART_COUNT])
{
    int i, i2;
    if(suco_flag)
    {
        // Reverse order of partitions
        switch(mode)
        {
            case SPLIT_QUAD:
                suco_order[0] = 1;
                suco_order[1] = 0;
                suco_order[2] = 3;
                suco_order[3] = 2;
                break;
            default:
                i2 = 0;
                for(i = evc_split_part_count(mode); i > 0; --i)
                {
                    suco_order[i2++] = i - 1;
                }
        }
    }
    else
    {
        // Direct order of partitions
        for(i = 0; i < evc_split_part_count(mode); ++i)
        {
            suco_order[i] = i;
        }
    }
}

int  evc_split_is_TT(SPLIT_MODE mode)
{
    return (mode == SPLIT_TRI_HOR) || (mode == SPLIT_TRI_VER) ? 1 : 0;
}

int  evc_split_is_BT(SPLIT_MODE mode)
{
    return (mode == SPLIT_BI_HOR) || (mode == SPLIT_BI_VER) ? 1 : 0;
}

#if SIMD_CLIP
__inline void do_clip(__m128i *vreg, __m128i *vbdmin, __m128i *vbdmax) { *vreg = _mm_min_epi16(*vbdmax, _mm_max_epi16(*vbdmin, *vreg)); }

void clip_simd(const pel* src, int src_stride, pel *dst, int dst_stride, int width, int height, const int clp_rng_min, const int clp_rng_max )
{
    __m128i vzero = _mm_setzero_si128();
    __m128i vbdmin = _mm_set1_epi16(clp_rng_min);
    __m128i vbdmax = _mm_set1_epi16(clp_rng_max);

    if ((width & 3) == 0)
    {
        for (int row = 0; row < height; row++)
        {
            for (int col = 0; col < width; col += 4)
            {
                __m128i val;
                val = _mm_loadl_epi64((const __m128i *)&src[col]);
                val = _mm_cvtepi16_epi32(val);
                val = _mm_packs_epi32(val, vzero);
                do_clip(&val, &vbdmin, &vbdmax);
                _mm_storel_epi64((__m128i *)&dst[col], val);
            }
            src += src_stride;
            dst += dst_stride;
        }
    }
    else
    {
        for (int row = 0; row < height; row++)
        {
            for (int col = 0; col < width; col ++)
            {
                dst[col] = EVC_CLIP3(clp_rng_min , clp_rng_max, src[col]);
            }
            src += src_stride;
            dst += dst_stride;
        }
    }
}
#endif

void evc_block_copy(s16 * src, int src_stride, s16 * dst, int dst_stride, int log2_copy_w, int log2_copy_h)
{
    int h;
    int copy_size = (1 << log2_copy_w) * (int)sizeof(s16);
    s16 *tmp_src = src;
    s16 *tmp_dst = dst;
    for (h = 0; h < (1<< log2_copy_h); h++)
    {
        memcpy(tmp_dst, tmp_src, copy_size);
        tmp_dst += dst_stride;
        tmp_src += src_stride;
    }
}

#if ATS_INTER_PROCESS
u8 check_ats_inter_info_coded(int cuw, int cuh, int pred_mode, int tool_ats)
{
    int min_size = 8;
    int max_size = 1 << MAX_TR_LOG2;
    u8  mode_hori, mode_vert, mode_hori_quad, mode_vert_quad;
#if IBC
    if (!tool_ats || pred_mode == MODE_INTRA || cuw > max_size || cuh > max_size || pred_mode == MODE_IBC)
#else
    if (!tool_ats || pred_mode == MODE_INTRA || cuw > max_size || cuh > max_size)
#endif
    {
        mode_hori = mode_vert = mode_hori_quad = mode_vert_quad = 0;
    }
    else
    {
        //vertical mode
        mode_vert = cuw >= min_size ? 1 : 0;
        mode_vert_quad = cuw >= min_size * 2 ? 1 : 0;
        mode_hori = cuh >= min_size ? 1 : 0;
        mode_hori_quad = cuh >= min_size * 2 ? 1 : 0;
    }
    return (mode_vert << 0) + (mode_hori << 1) + (mode_vert_quad << 2) + (mode_hori_quad << 3);
}

void get_tu_size(u8 ats_inter_info, int log2_cuw, int log2_cuh, int* log2_tuw, int* log2_tuh)
{
    u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
    if (ats_inter_idx == 0)
    {
        *log2_tuw = log2_cuw;
        *log2_tuh = log2_cuh;
        return;
    }

    assert(ats_inter_idx <= 4);
    if (is_ats_inter_horizontal(ats_inter_idx))
    {
        *log2_tuw = log2_cuw;
        *log2_tuh = is_ats_inter_quad_size(ats_inter_idx) ? log2_cuh - 2 : log2_cuh - 1;
    }
    else
    {
        *log2_tuw = is_ats_inter_quad_size(ats_inter_idx) ? log2_cuw - 2 : log2_cuw - 1;
        *log2_tuh = log2_cuh;
    }
}

void get_tu_pos_offset(u8 ats_inter_info, int log2_cuw, int log2_cuh, int* x_offset, int* y_offset)
{
    u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
    u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
    int cuw = 1 << log2_cuw;
    int cuh = 1 << log2_cuh;

    if (ats_inter_idx == 0)
    {
        *x_offset = 0;
        *y_offset = 0;
        return;
    }

    if (is_ats_inter_horizontal(ats_inter_idx))
    {
        *x_offset = 0;
        *y_offset = ats_inter_pos == 0 ? 0 : cuh - (is_ats_inter_quad_size(ats_inter_idx) ? cuh / 4 : cuh / 2);
    }
    else
    {
        *x_offset = ats_inter_pos == 0 ? 0 : cuw - (is_ats_inter_quad_size(ats_inter_idx) ? cuw / 4 : cuw / 2);
        *y_offset = 0;
    }
}

#if ATS_INTRA_PROCESS
void get_ats_inter_trs(u8 ats_inter_info, int log2_cuw, int log2_cuh, u8* ats_cu, u8* ats_tu)
{
    if (ats_inter_info == 0)
    {
        return;
    }

    if (log2_cuw > 5 || log2_cuh > 5)
    {
        *ats_cu = 0;
        *ats_tu = 0;
    }
    else
    {
        u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
        u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
        u8 t_idx_h, t_idx_v;

        //Note: 1 is DCT8 and 0 is DST7
#if ATS_INTER_DEBUG
        assert(evc_tbl_tr_subset_intra[0] == DST7 && evc_tbl_tr_subset_intra[1] == DCT8);
#endif
        if (is_ats_inter_horizontal(ats_inter_idx))
        {
            t_idx_h = 0;
            t_idx_v = ats_inter_pos == 0 ? 1 : 0;
        }
        else
        {
            t_idx_v = 0;
            t_idx_h = ats_inter_pos == 0 ? 1 : 0;
        }
        *ats_cu = 1;
        *ats_tu = (t_idx_h << 1) | t_idx_v;
    }
}
#endif

void set_cu_cbf_flags(u8 cbf_y, u8 ats_inter_info, int log2_cuw, int log2_cuh, u32 *map_scu, int w_scu)
{
    u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
    u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
    int x_offset, y_offset, log2_tuw, log2_tuh;
    int x, y, w, h;
    int w_cus = 1 << (log2_cuw - MIN_CU_LOG2);
    int h_cus = 1 << (log2_cuh - MIN_CU_LOG2);
    u32 *cur_map;
    if (ats_inter_info)
    {
        get_tu_pos_offset(ats_inter_info, log2_cuw, log2_cuh, &x_offset, &y_offset);
        get_tu_size(ats_inter_info, log2_cuw, log2_cuh, &log2_tuw, &log2_tuh);
        x_offset >>= MIN_CU_LOG2;
        y_offset >>= MIN_CU_LOG2;
        w = 1 << (log2_tuw - MIN_CU_LOG2);
        h = 1 << (log2_tuh - MIN_CU_LOG2);

        // Clear CbF of CU
        cur_map = map_scu;
        for (y = 0; y < h_cus; ++y, cur_map += w_scu)
        {
            for (x = 0; x < w_cus; ++x)
            {
                MCU_CLR_CBFL(cur_map[x]);
            }
        }

        if (cbf_y)
        {
            // Set CbF only on coded part
            cur_map = map_scu + y_offset * w_scu + x_offset;
            for (y = 0; y < h; ++y, cur_map += w_scu)
            {
                for (x = 0; x < w; ++x)
                {
                    MCU_SET_CBFL(cur_map[x]);
                }
            }
        }
    }
    else
    {
        assert(0);
    }
}
#endif

#if REMOVE_UNIBLOCKS_4x4
char is_inter_applicable_log2(int log2_cuw, int log2_cuh)
{
    return (log2_cuw > MIN_CU_LOG2 || log2_cuh > MIN_CU_LOG2) ? 1 : 0;
}

char is_inter_applicable(int cuw, int cuh)
{
    return (cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE) ? 1 : 0;
}
#endif


#if M49023_ADMVP_IMPROVE
void evc_get_mv_collocated(
    EVC_REFP(*refp)[REFP_NUM]
    , u32 ptr, int scup, int c_scu, u16 w_scu, u16 h_scu, s16 mvp[REFP_NUM][MV_D]
    , s8 *availablePredIdx
#if M49023_ADMVP_IMPROVE 
    , EVC_SH* sh
#endif
)
{
    *availablePredIdx = 0;
#if M49023_ADMVP_IMPROVE
    int temporal_mvp_asigned_flag = sh->temporal_mvp_asigned_flag;
    int collocated_from_list_idx = (sh->slice_type == SLICE_P) ? REFP_0 : REFP_1;  // Specifies source (List ID) of the collocated picture, equialent of the collocated_from_l0_flag
    int collocated_from_ref_idx = 0;        // Specifies source (RefID_ of the collocated picture, equialent of the collocated_ref_idx
    int collocated_mvp_source_list_idx = REFP_0;  // Specifies source (List ID) in collocated pic that provides MV information (Applicability is function of NoBackwardPredFlag)
    if (sh->temporal_mvp_asigned_flag)
    {
        collocated_from_list_idx = sh->collocated_from_list_idx;
        collocated_from_ref_idx = sh->collocated_from_ref_idx;
        collocated_mvp_source_list_idx = sh->collocated_mvp_source_list_idx;
    }
#else
    int collocated_from_list_idx = REFP_1;  // Specifies source (List ID) of the collocated picture, equialent of the collocated_from_l0_flag
    int collocated_from_ref_idx = 0;        // Specifies source (RefID_ of the collocated picture, equialent of the collocated_ref_idx
    int collocated_mvp_source_list_idx = REFP_0;  // Specifies source (List ID) in collocated pic that provides MV information (Applicability is function of NoBackwardPredFlag)
#endif
    EVC_REFP colPic = (refp[collocated_from_ref_idx][collocated_from_list_idx]);  // col picture is ref idx 0 and list 1

    int neb_addr_coll = scup;     // Col 

    int dptr_co[REFP_NUM] = { 0, 0 };
    int dptr[REFP_NUM] = { 0, 0 };
    int ver_refi[REFP_NUM] = { -1, -1 };
    memset(mvp, 0, sizeof(s16) * REFP_NUM * MV_D);


    s8(*map_refi_co)[REFP_NUM] = colPic.map_refi;
    dptr[REFP_0] = ptr - refp[0][REFP_0].ptr;
    dptr[REFP_1] = ptr - refp[0][REFP_1].ptr;
#if M49023_ADMVP_IMPROVE
    if (!temporal_mvp_asigned_flag)
#endif
    {
        dptr_co[REFP_0] = colPic.ptr - colPic.list_ptr[map_refi_co[neb_addr_coll][REFP_0]]; //POC1
        dptr_co[REFP_1] = colPic.ptr - colPic.list_ptr[map_refi_co[neb_addr_coll][REFP_1]]; //POC2

        for (int lidx = 0; lidx < REFP_NUM; lidx++)
        {
            s8 refidx = map_refi_co[neb_addr_coll][lidx];
            if (dptr_co[lidx] != 0 && REFI_IS_VALID(refidx))
            {
                int ratio_tmvp = ((dptr[lidx]) << MVP_SCALING_PRECISION) / dptr_co[lidx];
                ver_refi[lidx] = 0; // ref idx
                s16 *mvc = colPic.map_mv[neb_addr_coll][lidx];
                scaling_mv(ratio_tmvp, mvc, mvp[lidx]);
            }
            else
            {
                mvp[lidx][MV_X] = 0;
                mvp[lidx][MV_Y] = 0;
            }
        }
    }
    else
    {
        // collocated_mvp_source_list_idx = REFP_0; // specified above
        s8 refidx = map_refi_co[neb_addr_coll][collocated_mvp_source_list_idx];
        dptr_co[REFP_0] = colPic.ptr - colPic.list_ptr[refidx];
        {
            if (dptr_co[REFP_0] != 0 && REFI_IS_VALID(refidx))
            {
                ver_refi[REFP_0] = 0;
                ver_refi[REFP_1] = 0;
                s16 *mvc = colPic.map_mv[neb_addr_coll][collocated_mvp_source_list_idx]; //  collocated_mvp_source_list_idx == 0 for RA
                int ratio_tmvp = ((dptr[REFP_0]) << MVP_SCALING_PRECISION) / dptr_co[REFP_0];
                scaling_mv(ratio_tmvp, mvc, mvp[REFP_0]);

                ratio_tmvp = ((dptr[REFP_1]) << MVP_SCALING_PRECISION) / dptr_co[REFP_0];
                scaling_mv(ratio_tmvp, mvc, mvp[REFP_1]);
            }
            else
            {
                mvp[REFP_0][MV_X] = 0;
                mvp[REFP_0][MV_Y] = 0;
                mvp[REFP_1][MV_X] = 0;
                mvp[REFP_1][MV_Y] = 0;
            }
        }
    }


    {
        int maxX = PIC_PAD_SIZE_L + (w_scu << MIN_CU_LOG2) - 1;
        int maxY = PIC_PAD_SIZE_L + (h_scu << MIN_CU_LOG2) - 1;
        int x = (c_scu % w_scu) << MIN_CU_LOG2;
        int y = (c_scu / w_scu) << MIN_CU_LOG2;
        evc_clip_mv_pic(x, y, maxX, maxY, mvp);
    }

    int flag = REFI_IS_VALID(ver_refi[REFP_0]) + (REFI_IS_VALID(ver_refi[REFP_1]) << 1);
    *availablePredIdx = flag; // combines flag and indication on what type of prediction is ( 0 - not available, 1 = uniL0, 2 = uniL1, 3 = Bi)
}
#endif

#if M50761_REMOVE_BIBLOCKS_8x4
char process_bi_mv(s16 *mv, s8* refi)
{
    refi[1] = REFI_INVALID;
    return 1;
}

char allowed_block_size(int cuw, int cuh)
{
    return !((max(cuw, cuh) == 8 && min(cuw, cuh) == 4));
}
#endif


#if M50761_CHROMA_NOT_SPLIT
int evc_get_luma_cup(int x_scu, int y_scu, int cu_w_scu, int cu_h_scu, int w_scu)
{
    return (y_scu + (cu_h_scu >> 1)) * w_scu + x_scu + (cu_w_scu >> 1);
    //return (y_scu + (cu_h_scu - 1)) * w_scu + x_scu + (cu_w_scu - 1);
}

u8 evc_check_chroma_split_allowed(int luma_width, int luma_height)
{
    return (luma_width * luma_height) >= (16 * 4)  ? 1 : 0;
}


u8 evc_tree_split_allowed(int w, int h, SPLIT_MODE split)
{
    switch (split)
    {
    case SPLIT_QUAD:
        return evc_check_chroma_split_allowed(w >> 1, h >> 1);
    case SPLIT_BI_VER:
        return evc_check_chroma_split_allowed(w >> 1, h);
    case SPLIT_BI_HOR:
        return evc_check_chroma_split_allowed(w, h >> 1);
    case SPLIT_TRI_VER:
        return evc_check_chroma_split_allowed(w >> 2, h);
    case SPLIT_TRI_HOR:
        return evc_check_chroma_split_allowed(w, h >> 2);
    default:
        evc_assert(0);
        return 0;
    }

}

enum TQC_RUN evc_get_run(enum TQC_RUN run_list, TREE_CONS tree_cons)
{
    enum TQC_RUN ans = 0;
    if (evc_check_luma(tree_cons))
    {
        ans |= run_list & RUN_L;
    }

    if (evc_check_chroma(tree_cons))
    {
        ans |= run_list & RUN_CB;
        ans |= run_list & RUN_CR;
    }
    return ans;
}


u8 evc_check_luma(TREE_CONS tree_cons)
{
    return tree_cons.tree_type != TREE_C;
}

u8 evc_check_chroma(TREE_CONS tree_cons)
{
    return tree_cons.tree_type != TREE_L;
}

u8 evc_check_all(TREE_CONS tree_cons)
{
    return tree_cons.tree_type == TREE_LC;
}

u8 evc_check_only_intra(TREE_CONS tree_cons)
{
    return tree_cons.mode_cons == eOnlyIntra;
}

u8 evc_check_only_inter(TREE_CONS tree_cons)
{
    return tree_cons.mode_cons == eOnlyInter;
}

u8 evc_check_all_preds(TREE_CONS tree_cons)
{
    return tree_cons.mode_cons == eAll;
}

TREE_CONS evc_get_default_tree_cons()
{
    TREE_CONS ans;
    ans.changed = FALSE;
    ans.mode_cons = eAll;
    ans.tree_type = TREE_LC;
    return ans;
}

void evc_set_tree_mode(TREE_CONS* dest, MODE_CONS mode)
{
    dest->mode_cons = mode;
    switch (mode)
    {
    case eOnlyIntra:
        dest->tree_type = TREE_L;
        break;
    default:
        dest->tree_type = TREE_LC;
        break;
    }
}

BOOL evc_signal_mode_cons(TREE_CONS* parent, TREE_CONS* cur_split)
{
    return !parent->changed && cur_split->changed;
}

MODE_CONS evc_get_mode_cons_by_split(SPLIT_MODE split_mode, int cuw, int cuh)
{
    int small_cuw = cuw;
    int small_cuh = cuh;
    switch (split_mode)
    {
    case SPLIT_BI_HOR:
        small_cuh >>= 1;
        break;
    case SPLIT_BI_VER:
        small_cuw >>= 1;
        break;
    case SPLIT_TRI_HOR:
        small_cuh >>= 2;
        break;
    case SPLIT_TRI_VER:
        small_cuw >>= 2;
        break;
    case SPLIT_QUAD:
        small_cuw >>= 1;
        small_cuh >>= 1;
        break;
    default:
        evc_assert(0);
    }
    return (small_cuh == 4 && small_cuw == 4) ? eOnlyIntra : eAll;
}
#endif




#if GRAB_STAT
void enc_stat_header(int pic_w, int pic_h)
{
    evc_stat_write_comment("VTMBMS Block Statistics");
    evc_stat_write_comment("Sequence size: [%dx%4d]", pic_w, pic_h);
    evc_stat_write_type("PredMode", "Flag", NULL);
    evc_stat_write_type("AffineFlag", "Flag", NULL);
    evc_stat_write_type("MMVDFlag", "Flag", NULL);
    evc_stat_write_type("MV0", "Vector", "Scale: 4");
    evc_stat_write_type("REF0", "Flag", NULL);
    evc_stat_write_type("MV1", "Vector", "Scale: 4");
    evc_stat_write_type("REF1", "Flag", NULL);
    evc_stat_write_type("BiBlock", "Flag", NULL);
    evc_stat_write_type("ats_intra_cu", "Flag", NULL);
    evc_stat_write_type("ats_inter_info", "Flag", NULL);
    evc_stat_write_type("CBF_luma", "Flag", NULL);
}
#endif
