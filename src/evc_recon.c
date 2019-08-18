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
#include "evc_recon.h"

#include <math.h>


void evc_recon(s16 *coef, pel *pred, int is_coef, int cuw, int cuh, int s_rec, pel *rec
#if ATS_INTER_PROCESS
               , u8 ats_inter_info
#endif
)
{
    int i, j;
    s16 t0;

    if(is_coef == 0) /* just copy pred to rec */
    {
        for(i = 0; i < cuh; i++)
        {
            for(j = 0; j < cuw; j++)
            {
                rec[i * s_rec + j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pred[i * cuw + j]);
            }
        }
#if SIMD_CLIP
        clip_simd(rec, s_rec, rec, s_rec, cuw, cuh, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
    else  /* add b/w pred and coef and copy it into rec */
    {
#if ATS_INTER_PROCESS
        if (ats_inter_info != 0)
        {
            u8  ats_inter_idx = get_ats_inter_idx(ats_inter_info);
            u8  ats_inter_pos = get_ats_inter_pos(ats_inter_info);
            assert(ats_inter_idx >= 1 && ats_inter_idx <= 4);
            int tu0_w, tu0_h;
            int tu1_w, tu1_h;
            pel resi;
            if (!is_ats_inter_horizontal(ats_inter_idx))
            {
                tu0_w = is_ats_inter_quad_size(ats_inter_idx) ? (cuw / 4) : (cuw / 2);
                tu0_w = ats_inter_pos == 0 ? tu0_w : cuw - tu0_w;
                tu1_w = cuw - tu0_w;
                for (i = 0; i < cuh; i++)
                {
                    for (j = 0; j < tu0_w; j++)
                    {
                        resi = ats_inter_pos == 0 ? coef[i * tu0_w + j] : 0;
                        t0 = resi + pred[i * cuw + j];
                        rec[i * s_rec + j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, t0);
                    }
                    for (j = tu0_w; j < cuw; j++)
                    {
                        resi = ats_inter_pos == 1 ? coef[i * tu1_w + j - tu0_w] : 0;
                        t0 = resi + pred[i * cuw + j];
                        rec[i * s_rec + j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, t0);
                    }
                }
            }
            else
            {
                tu0_h = is_ats_inter_quad_size(ats_inter_idx) ? (cuh / 4) : (cuh / 2);
                tu0_h = ats_inter_pos == 0 ? tu0_h : cuh - tu0_h;
                tu1_h = cuh - tu0_h;
                for (j = 0; j < cuw; j++)
                {
                    for (i = 0; i < tu0_h; i++)
                    {
                        resi = ats_inter_pos == 0 ? coef[i * cuw + j] : 0;
                        t0 = resi + pred[i * cuw + j];
                        rec[i * s_rec + j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, t0);
                    }
                    for (i = tu0_h; i < cuh; i++)
                    {
                        resi = ats_inter_pos == 1 ? coef[(i - tu0_h) * cuw + j] : 0;
                        t0 = resi + pred[i * cuw + j];
                        rec[i * s_rec + j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, t0);
                    }
                }
            }
            return;
        }
#endif
        for(i = 0; i < cuh; i++)
        {
            for(j = 0; j < cuw; j++)
            {
                t0 = coef[i * cuw + j] + pred[i * cuw + j];
                rec[i * s_rec + j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, t0);
            }
        }
#if SIMD_CLIP
        clip_simd(rec, s_rec, rec, s_rec, cuw, cuh, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_recon_yuv(int x, int y, int cuw, int cuh, s16 coef[N_C][MAX_CU_DIM], pel pred[N_C][MAX_CU_DIM], int nnz[N_C], EVC_PIC *pic
#if ATS_INTER_PROCESS
                   , u8 ats_inter_info
#endif
)
{
    pel * rec;
    int s_rec, off;

    /* Y */
    s_rec = pic->s_l;
    rec = pic->y + (y * s_rec) + x;
    evc_recon(coef[Y_C], pred[Y_C], nnz[Y_C], cuw, cuh, s_rec, rec
#if ATS_INTER_PROCESS
              , ats_inter_info
#endif
    );

    /* chroma */
    cuw >>= 1;
    cuh >>= 1;
    off = (x >> 1) + (y >> 1) * pic->s_c;
    evc_recon(coef[U_C], pred[U_C], nnz[U_C], cuw, cuh, pic->s_c, pic->u + off
#if ATS_INTER_PROCESS
              , ats_inter_info
#endif
    );
    evc_recon(coef[V_C], pred[V_C], nnz[V_C], cuw, cuh, pic->s_c, pic->v + off
#if ATS_INTER_PROCESS
              , ats_inter_info
#endif
    );
}

#if HTDF

#define HTDF_LUT_QP_NUM                                   5   // num of LUTs
#define HTDF_LUT_SIZE_LOG2                                4   // table size in bits
#define HTDF_LUT_MIN_QP                                   20  // LUT min QP 
#define HTDF_LUT_STEP_QP_LOG2                             3   // LUT QP step
#define HTDF_FAST_TBL                                     1   // bit mask check & abs operations, SW friendly implementation
#define HTDF_BIT_RND4                                     2
#define HTDF_CNT_SCALE                                    2
#define HTDF_CNT_SCALE_RND                                (1 << (HTDF_CNT_SCALE - 1))

u8 HTDF_table_thr_log2[HTDF_LUT_QP_NUM] = { 6, 7, 7, 8, 8 };

static const 
u8 HTDF_table[HTDF_LUT_QP_NUM][1 << HTDF_LUT_SIZE_LOG2] = {
{ 0, 0, 2,  6, 10, 14, 19, 23, 28, 32,  36,  41,  45,  49,  53,  57, },
{ 0, 0, 5, 12, 20, 29, 38, 47, 56, 65,  73,  82,  90,  98, 107, 115, },
{ 0, 0, 1,  4,  9, 16, 24, 32, 41, 50,  59,  68,  77,  86,  94, 103, },
{ 0, 0, 3,  9, 19, 32, 47, 64, 81, 99, 117, 135, 154, 179, 205, 230, },
{ 0, 0, 0,  2,  6, 11, 18, 27, 38, 51,  64,  96, 128, 160, 192, 224, },
};

INLINE int read_table(const int z, const u8 *tbl, const int thr, const int table_shift, const int table_round)
{
#if HTDF_FAST_TBL
    const unsigned Shift = sizeof(int) * 8 - 1;
    const int sg0 = z >> Shift;                                   // sign(z)
    const int v0 = (z + sg0) ^ sg0;                               // abs(z) 
    const int r0 = v0 << 0;                                       // scaled abs(z)
    const int idx = ((v0 + table_round)&thr) >> table_shift;
    const int w0 = r0 + ((tbl[idx] - r0)&((v0 - thr) >> Shift));  // tbl(abs(z))
    return (w0 + sg0) ^ sg0;                                      // +-tbl(abs(z))
#else
    return  (z > 0 ? (z < thr ? tbl[(z + table_round) >> table_shift] : z) : (-z < thr ? -tbl[((-z) + table_round) >> table_shift] : z));
#endif
}

typedef struct
{
    int r;
    int c;
}tHtdfOffset;

static tHtdfOffset Scan[4] = { { 0,0 },{ 0,1 },{ 1,0 },{ 1,1 } };

void evc_htdf_filter_block(pel *block, pel *acc_block, const u8 *tbl, int stride_block, int stride_acc, int width, int height, int tbl_thr_log2)
{
    const int p0 = Scan[0].r*stride_block + Scan[0].c;
    const int p1 = Scan[1].r*stride_block + Scan[1].c;
    const int p2 = Scan[2].r*stride_block + Scan[2].c;
    const int p3 = Scan[3].r*stride_block + Scan[3].c;

    const int p0_out = Scan[0].r*stride_acc + Scan[0].c;
    const int p1_out = Scan[1].r*stride_acc + Scan[1].c;
    const int p2_out = Scan[2].r*stride_acc + Scan[2].c;
    const int p3_out = Scan[3].r*stride_acc + Scan[3].c;

    const int table_shift = tbl_thr_log2 - HTDF_LUT_SIZE_LOG2;
    const int table_round = (1 << table_shift) >> 1;
    const int thr = (1 << tbl_thr_log2) - (1 << table_shift);

    for (int r = 0; r < height - 1; ++r)
    {
        pel *in = &block[r*stride_block];
        pel *out = &acc_block[r*stride_acc];

        for (int c = 0; c < width - 1; ++c, in++, out++)
        {
            const int x0 = in[p0];
            const int x1 = in[p1];
            const int x2 = in[p2];
            const int x3 = in[p3];

            // forward transform
            const int y0 = x0 + x2;
            const int y1 = x1 + x3;
            const int y2 = x0 - x2;
            const int y3 = x1 - x3;

            const int t0 = y0 + y1;
            const int t1 = y0 - y1;
            const int t2 = y2 + y3;
            const int t3 = y2 - y3;

            // filtering
            const int z0 = t0;  // skip DC
            const int z1 = read_table(t1, tbl, thr, table_shift, table_round);
            const int z2 = read_table(t2, tbl, thr, table_shift, table_round);
            const int z3 = read_table(t3, tbl, thr, table_shift, table_round);

            // backward transform
            const int iy0 = z0 + z2;
            const int iy1 = z1 + z3;
            const int iy2 = z0 - z2;
            const int iy3 = z1 - z3;

            out[p0_out] += ((iy0 + iy1) >> HTDF_BIT_RND4);
            out[p1_out] += ((iy0 - iy1) >> HTDF_BIT_RND4);
            out[p2_out] += ((iy2 + iy3) >> HTDF_BIT_RND4);
            out[p3_out] += ((iy2 - iy3) >> HTDF_BIT_RND4);

            // normalization
            in[p0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (out[p0_out] + HTDF_CNT_SCALE_RND) >> HTDF_CNT_SCALE);
        }
    }
}

void filter_block_luma(pel *block, const u8 HTDF_table[HTDF_LUT_QP_NUM][1 << HTDF_LUT_SIZE_LOG2], int width, int height, int stride, int qp)
{
    pel acc_block[(MAX_CU_SIZE + 2)*(MAX_CU_SIZE + 2)];

    memset(acc_block, 0, stride*height * sizeof(*acc_block));

    int idx = (qp - HTDF_LUT_MIN_QP + (1 << (HTDF_LUT_STEP_QP_LOG2 - 1))) >> HTDF_LUT_STEP_QP_LOG2;
    idx = max(idx, 0);
    idx = min(idx, HTDF_LUT_QP_NUM - 1);

    evc_htdf_filter_block(block, acc_block, HTDF_table[idx], stride, width, width, height, HTDF_table_thr_log2[idx]);
}

BOOL evc_htdf_skip_condition(int width, int height, int IntraBlockFlag, int *qp)
{
    if(*qp <= 17)
        return TRUE;

    if((width == 4) && (height == 4))
        return TRUE;

    int min_size = min(width, height);
    int max_size = max(width, height);

    if(max_size >= 128)
        return TRUE;

    if(IntraBlockFlag == 0)
    {
        if(min_size >= 32)
            return TRUE;
    }
    else
    {
        if((width == height) && (min_size >= 32))
            *qp -= 1 << HTDF_LUT_STEP_QP_LOG2;
    }

    return FALSE;
}

void evc_htdf(s16* rec, int qp, int w, int h, int s, BOOL intra_block_flag, pel* rec_pic, int s_pic, int avail_cu)
{
    if (evc_htdf_skip_condition(w, h, intra_block_flag, &qp))
        return;

    pel tempblock[(MAX_CU_SIZE + 2) * (MAX_CU_SIZE + 2)];
    int width_ext  = w + 2;
    int height_ext = h + 2;

    for (int i = 0; i < h; ++i)
        memcpy(tempblock + (i + 1) * width_ext + 1, rec + i * s, w * sizeof(rec[0]));

    if (IS_AVAIL(avail_cu, AVAIL_LE))
    {
        for (int i = 1; i < height_ext - 1; ++i)
            tempblock[i * width_ext] = rec_pic[(i - 1) * s_pic - 1];
    }
    else
    {
        for (int i = 1; i < height_ext - 1; ++i)
            tempblock[i * width_ext] = rec[(i - 1) * s];
    }
    if (IS_AVAIL(avail_cu, AVAIL_RI))
    {
        for (int i = 1; i < height_ext - 1; ++i)
            tempblock[i * width_ext + width_ext - 1] = rec_pic[(i - 1) * s_pic + w];
    }
    else
    {
        for (int i = 1; i < height_ext - 1; ++i)
            tempblock[i * width_ext + width_ext - 1] = rec[(i - 1) * s + w - 1];
    }
    if (IS_AVAIL(avail_cu, AVAIL_UP))
    {
        memcpy(tempblock + 1, rec_pic - s_pic, w * sizeof(rec_pic[0]));
    }
    else
    {
        memcpy(tempblock + 1, rec, w * sizeof(rec[0]));
    }
    memcpy(tempblock + 1 + (height_ext - 1) * width_ext, rec + (h - 1) * s, w * sizeof(rec[0]));

    tempblock[0] = IS_AVAIL(avail_cu, AVAIL_UP_LE) ? rec_pic[-1 - 1 * s_pic] : rec[0];
    tempblock[width_ext - 1] = IS_AVAIL(avail_cu, AVAIL_UP_RI) ? rec_pic[w - 1 * s_pic] : rec[w - 1];
    tempblock[width_ext * (height_ext - 1)] = IS_AVAIL(avail_cu, AVAIL_LO_LE) ? rec_pic[-1 + h * s_pic] : rec[(h - 1) * s];
    tempblock[width_ext - 1 + width_ext * (height_ext - 1)] = IS_AVAIL(avail_cu, AVAIL_LO_RI) ? rec_pic[w + h * s_pic] : rec[w - 1 + (h - 1) * s];

    filter_block_luma(tempblock, HTDF_table, width_ext, height_ext, width_ext, qp);

    for (int i = 0; i < h; ++i)
        memcpy(rec + i * s, tempblock + (i + 1) * width_ext + 1, w * sizeof(rec[0]));
}
#endif
