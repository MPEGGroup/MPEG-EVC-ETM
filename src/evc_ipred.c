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

#include "evcd_def.h"

#if M48879_IMPROVEMENT_INTRA
void evc_get_nbr_b(int x, int y, int cuw, int cuh, pel *src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 * map_scu, int w_scu, int h_scu, int ch_type, int constrained_intra_pred)
{
    int  i, j;
    int  scuw = (ch_type == Y_C) ? (cuw >> MIN_CU_LOG2) : (cuw >> (MIN_CU_LOG2 - 1));
    int  scuh = (ch_type == Y_C) ? (cuh >> MIN_CU_LOG2) : (cuh >> (MIN_CU_LOG2 - 1));
    int  unit_size = (ch_type == Y_C) ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1);
    int  x_scu = PEL2SCU(ch_type == Y_C ? x : x << 1);
    int  y_scu = PEL2SCU(ch_type == Y_C ? y : y << 1);
    pel *tmp = src;
    pel *left = nb[ch_type][0] + 2;
    pel *up = nb[ch_type][1] + cuh;

    if (IS_AVAIL(avail_cu, AVAIL_UP_LE) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - w_scu - 1])))
    {
        evc_mcpy(up - 1, src - s_src - 1, cuw * sizeof(pel));
    }
    else
    {
        up[-1] = 1 << (BIT_DEPTH - 1);
    }

    for (i = 0; i < (scuw + scuh); i++)   
    {
        int is_avail = (y_scu > 0) && (x_scu + i < w_scu);
        if (is_avail && MCU_GET_COD(map_scu[scup - w_scu + i]) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - w_scu + i])))
        {
            evc_mcpy(up + i * unit_size, src - s_src + i * unit_size, unit_size * sizeof(pel));
        }
        else
        {
            evc_mset_16b(up + i * unit_size, 1 << (BIT_DEPTH - 1), unit_size);
        }
    }
    
    src--;
    for (i = 0; i < (scuh + scuw); ++i)
    {
        int is_avail = (x_scu > 0) && (y_scu + i < h_scu);
        if (is_avail && MCU_GET_COD(map_scu[scup - 1 + i * w_scu]) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - 1 + i * w_scu])))
        {
            for (j = 0; j < unit_size; ++j)
            {
                left[i * unit_size + j] = *src;
                src += s_src;
            }
        }
        else
        {
            evc_mset_16b(left + i * unit_size, 1 << (BIT_DEPTH - 1), unit_size);
            src += (s_src * unit_size);
        }
    }
    left[-1] = up[-1];
}
#endif

void evc_get_nbr(int x, int y, int cuw, int cuh, pel *src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 * map_scu, int w_scu, int h_scu, int ch_type
#if M48879_IMPROVEMENT_INTRA
                 , int constrained_intra_pred
#endif
)
{
    int  i, j;
    int  scuw = (ch_type == Y_C) ? (cuw >> MIN_CU_LOG2) : (cuw >> (MIN_CU_LOG2 - 1));
    int  scuh = (ch_type == Y_C) ? (cuh >> MIN_CU_LOG2) : (cuh >> (MIN_CU_LOG2 - 1));
    int  unit_size = (ch_type == Y_C) ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1);
    int  x_scu = PEL2SCU(ch_type == Y_C ? x : x << 1);
    int  y_scu = PEL2SCU(ch_type == Y_C ? y : y << 1);
    pel *tmp = src;
    pel *left = nb[ch_type][0] + 2;
    pel *up = nb[ch_type][1] + cuh;
    pel *right = nb[ch_type][2] + 2;

#if M48879_IMPROVEMENT_INTRA
    if (IS_AVAIL(avail_cu, AVAIL_UP_LE) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - w_scu - 1])))
    {
        evc_mcpy(up - 1, src - s_src - 1, cuw * sizeof(pel));
    }
    else
    {
        up[-1] = 1 << (BIT_DEPTH - 1);
    }

    for (i = 0; i < (scuw + scuh); i++)
    {
        int is_avail = (y_scu > 0) && (x_scu + i < w_scu);
        if (is_avail && MCU_GET_COD(map_scu[scup - w_scu + i]) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - w_scu + i])))
        {
            evc_mcpy(up + i * unit_size, src - s_src + i * unit_size, unit_size * sizeof(pel));
        }
        else
        {
            evc_mset_16b(up + i * unit_size, up[i * unit_size - 1], unit_size);
        }
    }

    if (x_scu > 0)
    {
        for (i = 0; i < scuh; i++)
        {
            if (scup > 0 && y_scu > 0 && (x_scu - 1 - i >= 0) && MCU_GET_COD(map_scu[scup - w_scu - 1 - i]) 
                && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - w_scu - 1 - i])))
            {
                evc_mcpy(up - (i + 1) * unit_size, src - s_src - (i + 1) * unit_size, unit_size * sizeof(pel));
            }
            else
            {
                evc_mset_16b(up - (i + 1) * unit_size, up[-i * unit_size], unit_size);
            }
        }
    }
    else
    {
        evc_mset_16b(up - cuh, up[0], cuh);
    }

    src--;
    left[-1] = up[-1];

    for (i = 0; i < (scuh + scuw); ++i)
    {
        int is_avail = (x_scu > 0) && (y_scu + i < h_scu);
        if (is_avail && MCU_GET_COD(map_scu[scup - 1 + i * w_scu]) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup - 1 + i * w_scu])))
        {
            for (j = 0; j < unit_size; ++j)
            {
                left[i * unit_size + j] = *src;
                src += s_src;
            }
        }
        else
        {
            evc_mset_16b(left + i * unit_size, left[i * unit_size - 1], unit_size);
            src += (s_src * unit_size);
        }
    }

    left[-2] = left[-1];

    src = tmp;

    src += cuw;
    right[-1] = up[cuw];

    for (i = 0; i < (scuh + scuw); i++)
    {
        int is_avail = (x_scu < w_scu) && (y_scu + i < h_scu);
        if (is_avail && MCU_GET_COD(map_scu[scup + scuw + i * w_scu]) && (!constrained_intra_pred || MCU_GET_IF(map_scu[scup + scuw + i * w_scu])))
        {
            for (j = 0; j < unit_size; ++j)
            {
                right[i * unit_size + j] = *src;
                src += s_src;
            }
        }
        else
        {
            evc_mset_16b(right + i * unit_size, right[i * unit_size - 1], unit_size);
            src += (s_src * unit_size);
        }
    }

    right[-2] = right[-1];
#else
    if(IS_AVAIL(avail_cu, AVAIL_UP))
    {
        evc_mcpy(up, src - s_src, cuw * sizeof(pel));

        for(i = 0; i < scuh; i++)
        {
            if(x_scu + scuw + i < w_scu && MCU_GET_COD(map_scu[scup - w_scu + scuw + i]))
            {
                evc_mcpy(up + cuw + i * unit_size, src - s_src + cuw + i * unit_size, unit_size * sizeof(pel));
            }
            else
            {
                evc_mset_16b(up + cuw + i * unit_size, up[cuw + i * unit_size - 1], unit_size);
            }
        }

        if(x_scu > 0)
        {
            for(i = 0; i < scuh; i++)
            {
                if(x_scu - 1 - i >= 0 && MCU_GET_COD(map_scu[scup - w_scu - 1 - i]))
                {
                    evc_mcpy(up - (i + 1) * unit_size, src - s_src - (i + 1) * unit_size, unit_size * sizeof(pel));
                }
                else
                {
                    evc_mset_16b(up - (i + 1) * unit_size, up[-i * unit_size], unit_size);
                }
            }
        }
        else
        {
            evc_mset_16b(up - cuh, up[0], cuh);
        }
    }
    else
    {
        evc_mset_16b(up - cuh, 1 << (BIT_DEPTH - 1), (cuw + cuh * 2));
    }

    if(IS_AVAIL(avail_cu, AVAIL_LE))
    {
        src--;
        for(i = 0; i < cuh; ++i)
        {
            left[i] = *src;
            src += s_src;
        }

        for(i = 0; i < scuw; i++)
        {
            if(y_scu + scuh + i < h_scu && MCU_GET_COD(map_scu[scup - 1 + (scuh + i) *w_scu]))
            {
                int j;
                for(j = 0; j < unit_size; ++j)
                {
                    left[cuh + i * unit_size + j] = *src;
                    src += s_src;
                }
            }
            else
            {
                evc_mset_16b(left + cuh + i * unit_size, left[cuh + i * unit_size - 1], unit_size);
                src += (s_src * unit_size);
            }
        }

        left[-1] = up[-1];
        left[-2] = up[-1];
    }
    else
    {
        if(x_scu > 0 && y_scu > 0 && MCU_GET_COD(map_scu[scup - w_scu - 1]))
        {
            evc_mset_16b(left - 2, up[-1], cuh + cuw + 2);
        }
        else
        {
            evc_mset_16b(left - 2, 1 << (BIT_DEPTH - 1), cuh + cuw + 2);
        }
    }

    src = tmp;

    if(IS_AVAIL(avail_cu, AVAIL_RI))
    {
        src += cuw;
        for(i = 0; i < cuh; i++)
        {
            right[i] = *src;
            src += s_src;
        }

        for(i = 0; i < scuw; i++)
        {
            if(y_scu + scuh + i < h_scu && MCU_GET_COD(map_scu[scup + scuw + (scuh + i) * w_scu]))
            {
                for(j = 0; j < unit_size; ++j)
                {
                    right[cuh + i * unit_size + j] = *src;
                    src += s_src;
                }
            }
            else
            {
                evc_mset_16b(right + cuh + i * unit_size, right[cuh + i * unit_size - 1], unit_size);
                src += (s_src * unit_size);
            }
        }

        right[-1] = up[cuw];
        right[-2] = up[cuw];
    }
    else
    {
        if(x_scu + scuw < w_scu  && y_scu > 0 && MCU_GET_COD(map_scu[scup - w_scu + scuw]))
        {
            evc_mset_16b(right - 2, up[cuw], cuh + cuw + 2);
        }
        else
        {
            evc_mset_16b(right - 2, 1 << (BIT_DEPTH - 1), cuh + cuw + 2);
        }
    }
#endif
}

#if M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE || M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
static const int lut_size_plus1[MAX_CU_LOG2 + 1] = { 2048, 1365, 819, 455, 241, 124, 63, 32 }; // 1/(w+1) = k >> 12
#endif //M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE || M48933_INTRA_PRED_NO_DIV_IN_DC_MODE

void ipred_hor(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int w, int h)
{
    int i, j;

    if(avail_lr == LR_11)
    {
#if M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE
        const int multi_w = lut_size_plus1[evc_tbl_log2[w]];
        const int shift_w = 12;
#endif //M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pel vle, vri;

                vle = src_le[0];
                vri = src_ri[0];
#if M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE
                dst[j] = ((vle * (w - j) + vri * (j + 1) + (w >> 1)) * multi_w) >> shift_w;
#else //!M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE
                dst[j] = divide_tbl((vle * (w - j) + vri * (j + 1) + (w >> 1)), (w + 1));
#endif //M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE
            }
            dst += w; src_le++; src_ri++;
        }
    }
    else if(avail_lr == LR_01)
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                dst[j] = src_ri[0];
            }
            dst += w; src_ri++;
        }
    }
    else
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                dst[j] = src_le[0];
            }
            dst += w; src_le++;
        }
    }
}

void ipred_vert(pel *src_le, pel *src_up, pel * src_ri, u16 avail_lr, pel *dst, int w, int h)
{
    int i, j;

    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            dst[j] = src_up[j];
        }
        dst += w;
    }
}

#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
static int evc_get_dc(const int numerator, const int w, const int h)
{
    const int log2_w = evc_tbl_log2[w];
    const int log2_h = evc_tbl_log2[h];
    const int shift_w = 12;
    
    int basic_shift = log2_w, log2_asp_ratio = 0;

    if (log2_w > log2_h)
    {
        basic_shift = log2_h;
        log2_asp_ratio = log2_w - log2_h;
    }
    else if (log2_w < log2_h)
    {
        basic_shift = log2_w;
        log2_asp_ratio = log2_h - log2_w;
    }

  return (numerator * lut_size_plus1[log2_asp_ratio]) >> (basic_shift + shift_w);
}
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE

void ipred_dc_b(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int w, int h, u16 avail_cu)
{
    int dc = 0;
    int wh, i, j;

#if M48879_IMPROVEMENT_INTRA
    for (i = 0; i < h; i++) dc += src_le[i];
    for (j = 0; j < w; j++) dc += src_up[j];
    dc = (dc + w) >> (evc_tbl_log2[w] + 1);
#else

    if(avail_lr == LR_11)
    {
        for(i = 0; i < h; i++) dc += src_le[i];
        for(i = 0; i < h; i++) dc += src_ri[i];
        if(IS_AVAIL(avail_cu, AVAIL_UP))
        {
            for(j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = evc_get_dc(dc + ((w + h + h) >> 1), w, h << 1);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = divide_tbl((dc + ((w + h + h) >> 1)), (w + h + h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        }
        else
        {
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = (dc + h) >> (evc_tbl_log2[h] + 1);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = divide_tbl((dc + h), (h + h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        }
    }
    else if(avail_lr == LR_01)
    {
        for(i = 0; i < h; i++) dc += src_ri[i];
        if(IS_AVAIL(avail_cu, AVAIL_UP))
        {
            for(j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = evc_get_dc(dc + ((w + h) >> 1), w, h);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = divide_tbl((dc + ((w + h) >> 1)), (w + h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        }
        else
        {
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = (dc + (h >> 1)) >> evc_tbl_log2[h];
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = divide_tbl((dc + ((h) >> 1)), (h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        }
    }
    else if(avail_lr == LR_10)
    {
        for(i = 0; i < h; i++) dc += src_le[i];
        if(IS_AVAIL(avail_cu, AVAIL_UP))
        {
            for(j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = evc_get_dc(dc + ((w + h) >> 1), w, h);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = (dc + ((w + h) >> 1)) / (w + h);
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        }
        else
        {
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = (dc + (h >> 1)) >> evc_tbl_log2[h];
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
            dc = (dc + (h >> 1)) / h;
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        }
    }
    else
    {
        for(j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = (dc + (w >> 1)) >> evc_tbl_log2[w];
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = (dc + w / 2) / w;
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
    }
#endif
    wh = w * h;

    for(i = 0; i < wh; i++)
    {
        dst[i] = (pel)dc;
    }
}

void ipred_dc(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int w, int h, u16 avail_cu, int sps_suco_flag)
{
    int dc = 0;
    int wh, i, j;

    if (!sps_suco_flag)
    {
        avail_lr = LR_10;
    }

    if (avail_lr == LR_11)
    {
        for (i = 0; i < h; i++) dc += src_le[i];
        for (i = 0; i < h; i++) dc += src_ri[i];
        for (j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = evc_get_dc(dc + ((w + h + h) >> 1), w, h << 1);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = divide_tbl((dc + ((w + h + h) >> 1)), (w + h + h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
    }
    else if (avail_lr == LR_01)
    {
        for (i = 0; i < h; i++) dc += src_ri[i];
        for (j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = evc_get_dc(dc + ((w + h) >> 1), w, h);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = divide_tbl((dc + ((w + h) >> 1)), (w + h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
    }
    else if (avail_lr == LR_10)
    {
        for (i = 0; i < h; i++) dc += src_le[i];
        for (j = 0; j < w; j++) dc += src_up[j];
#if M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = evc_get_dc(dc + ((w + h) >> 1), w, h);
#else //!M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
        dc = divide_tbl((dc + ((w + h) >> 1)), (w + h));
#endif //M48933_INTRA_PRED_NO_DIV_IN_DC_MODE
    }
    else
    {
        for (j = 0; j < w; j++) dc += src_up[j];
        dc = (dc + (w >> 1)) >> evc_tbl_log2[w];
    }

    wh = w * h;

    for (i = 0; i < wh; i++)
    {
        dst[i] = (pel)dc;
    }
}

void ipred_plane(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int w, int h)
{
    pel *rsrc;
    int  coef_h = 0, coef_v = 0;
    int  a, b, c, x, y;
    int  w2 = w >> 1;
    int  h2 = h >> 1;
    int  ib_mult[6] = {13, 17, 5, 11, 23, 47};
    int  ib_shift[6] = {7, 10, 11, 15, 19, 23};
    int  idx_w = evc_tbl_log2[w] < 2 ? 0 : evc_tbl_log2[w] - 2;
    int  idx_h = evc_tbl_log2[h] < 2 ? 0 : evc_tbl_log2[h] - 2;
    int  im_h, is_h, im_v, is_v, temp, temp2;

    im_h = ib_mult[idx_w];
    is_h = ib_shift[idx_w];
    im_v = ib_mult[idx_h];
    is_v = ib_shift[idx_h];

    if(avail_lr == LR_01 || avail_lr == LR_11)
    {
        rsrc = src_up + w2;
        for(x = 1; x < w2 + 1; x++)
        {
            coef_h += x * (rsrc[-x] - rsrc[x]);
        }

        rsrc = src_ri + (h2 - 1);
        for(y = 1; y < h2 + 1; y++)
        {
            coef_v += y * (rsrc[y] - rsrc[-y]);
        }

        a = (src_ri[h - 1] + src_up[0]) << 4;
        b = ((coef_h << 5) * im_h + (1 << (is_h - 1))) >> is_h;
        c = ((coef_v << 5) * im_v + (1 << (is_v - 1))) >> is_v;

        temp = a - (h2 - 1) * c - (w2 - 1) * b + 16;

        for(y = 0; y < h; y++)
        {
            temp2 = temp;
            for(x = w - 1; x >= 0; x--)
            {
                dst[x] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, temp2 >> 5);
                temp2 += b;
            }
            temp += c; dst += w;
        }
    }
    else
    {
        rsrc = src_up + (w2 - 1);
        for(x = 1; x < w2 + 1; x++)
        {
            coef_h += x * (rsrc[x] - rsrc[-x]);
        }

        rsrc = src_le + (h2 - 1);
        for(y = 1; y < h2 + 1; y++)
        {
            coef_v += y * (rsrc[y] - rsrc[-y]);
        }

        a = (src_le[h - 1] + src_up[w - 1]) << 4;
        b = ((coef_h << 5) * im_h + (1 << (is_h - 1))) >> is_h;
        c = ((coef_v << 5) * im_v + (1 << (is_v - 1))) >> is_v;

        temp = a - (h2 - 1) * c - (w2 - 1) * b + 16;

        for(y = 0; y < h; y++)
        {
            temp2 = temp;
            for(x = 0; x < w; x++)
            {
                dst[x] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, temp2 >> 5);
                temp2 += b;
            }
            temp += c; dst += w;
        }
    }
}

#if !(M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE || M48933_INTRA_PRED_NO_DIV_IN_DC_MODE)
int lut_size_plus1[MAX_CU_LOG2 + 1] = {2048, 1365, 819, 455, 241, 124, 63, 32};// 1/(w+1) = k >> 12
#endif //!(M48933_INTRA_PRED_NO_DIV_IN_HOR_MODE || M48933_INTRA_PRED_NO_DIV_IN_DC_MODE)

void ipred_bi(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int w, int h)
{
    int x, y;
    int ishift_x = evc_tbl_log2[w];
    int ishift_y = evc_tbl_log2[h];
    int ishift = EVC_MIN(ishift_x, ishift_y);
    int ishift_xy = ishift_x + ishift_y + 1;
    int offset = 1 << (ishift_x + ishift_y);
    int a, b, c, wt, wxy, tmp;
    int predx;
    int ref_up[MAX_CU_SIZE], ref_le[MAX_CU_SIZE], up[MAX_CU_SIZE], le[MAX_CU_SIZE], wy[MAX_CU_SIZE];
    int ref_ri[MAX_CU_SIZE], ri[MAX_CU_SIZE];
    int dst_tmp[MAX_CU_SIZE][MAX_CU_SIZE];
    int wc, tbl_wc[6] = {-1, 341, 205, 114, 60, 31};
    int log2_w = evc_tbl_log2[w];
    int log2_h = evc_tbl_log2[h];
    int multi_w = lut_size_plus1[log2_w];
    int shift_w = 12;

    wc = ishift_x > ishift_y ? ishift_x - ishift_y : ishift_y - ishift_x;
    evc_assert(wc <= 5);
    wc = tbl_wc[wc];

    for(x = 0; x < w; x++) ref_up[x] = src_up[x];
    for(y = 0; y < h; y++) ref_le[y] = src_le[y];
    for(y = 0; y < h; y++) ref_ri[y] = src_ri[y];

    if(avail_lr == LR_11)
    {
        for(y = 0; y < h; y++)
        {
            for(x = 0; x < w; x++)
            {
                dst_tmp[y][x] = (ref_le[y] * (w - x) + ref_ri[y] * (x + 1) + (w >> 1)) * multi_w >> shift_w;
            }
        }

        for(x = 0; x < w; x++)
        {
            for(y = 0; y < h; y++)
            {
                tmp = (ref_up[x] * (h - 1 - y) + dst_tmp[h - 1][x] * (y + 1) + (h >> 1)) >> log2_h;
                dst[y * w + x] = (dst_tmp[y][x] + tmp + 1) >> 1;
            }
        }
    }
    else if(avail_lr == LR_01)
    {
        a = src_up[-1];
        b = src_ri[h];
        c = (w == h) ? (a + b + 1) >> 1 : (((a << ishift_x) + (b << ishift_y)) * wc + (1 << (ishift + 9))) >> (ishift + 10);
        wt = (c << 1) - a - b;

        for(x = w - 1; x >= 0; x--)
        {
            up[x] = b - ref_up[x];
            ref_up[x] <<= ishift_y;
        }
        tmp = 0;
        for(y = 0; y < h; y++)
        {
            ri[y] = a - ref_ri[y];
            ref_ri[y] <<= ishift_x;
            wy[y] = tmp;
            tmp += wt;
        }

        for(y = 0; y < h; y++)
        {
            predx = ref_ri[y];
            wxy = 0;
            for(x = w - 1; x >= 0; x--)
            {
                predx += ri[y];
                ref_up[x] += up[x];
                dst[x] = ((predx << ishift_y) + (ref_up[x] << ishift_x) + wxy + offset) >> ishift_xy;

                dst[x] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, dst[x]);

                wxy += wy[y];
            }
            dst += w;
        }
    }
    else
    {
        a = src_up[w];
        b = src_le[h];
        c = (w == h) ? (a + b + 1) >> 1 : (((a << ishift_x) + (b << ishift_y)) * wc + (1 << (ishift + 9))) >> (ishift + 10);
        wt = (c << 1) - a - b;

        for(x = 0; x < w; x++)
        {
            up[x] = b - ref_up[x];
            ref_up[x] <<= ishift_y;
        }
        tmp = 0;
        for(y = 0; y < h; y++)
        {
            le[y] = a - ref_le[y];
            ref_le[y] <<= ishift_x;
            wy[y] = tmp;
            tmp += wt;
        }

        for(y = 0; y < h; y++)
        {
            predx = ref_le[y];
            wxy = 0;
            for(x = 0; x < w; x++)
            {
                predx += le[y];
                ref_up[x] += up[x];

                dst[x] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (((predx << ishift_y) + (ref_up[x] << ishift_x) + wxy + offset) >> ishift_xy));

                wxy += wy[y];
            }
            dst += w;
        }
    }
}

#define GET_REF_POS(mt,d_in,d_out,offset) \
    (d_out) = ((d_in) * (mt)) >> 10;\
    (offset) = (((d_in) * (mt)) >> 5) - ((d_out) << 5);

#define ADI_4T_FILTER_BITS                 7
#define ADI_4T_FILTER_OFFSET              (1<<(ADI_4T_FILTER_BITS-1))

__inline
pel ipred_ang_val(pel * src_up, pel * src_le, pel * src_ri, u16 avail_lr, int ipm, int i, int j, int w, int pos_min, int pos_max, int h)
{
    int offset;
    int t_dx, t_dy;
    int x, y, xn, yn, xn_n1, yn_n1, xn_p2, yn_p2;
    const int dxy = (ipm > IPD_HOR || ipm < IPD_VER) ? -1 : 1;
    const int * filter;
    const int(*tbl_filt)[4];
    int filter_offset, filter_bits;
    const int * mt = evc_tbl_ipred_dxdy[ipm];
    pel * src_ch = NULL;
    int num_selections = 0;
    int use_x;
    int p, pn, pn_n1, pn_p2;
    pel temp_pel = 0;

    x = INT_MAX;
    y = INT_MAX;

    tbl_filt = evc_tbl_ipred_adi;
    filter_offset = ADI_4T_FILTER_OFFSET;
    filter_bits = ADI_4T_FILTER_BITS;

    evc_assert(ipm < IPD_CNT);

    if(ipm < IPD_VER)
    {
        /* case x-line */
        GET_REF_POS(mt[0], j + 1, t_dx, offset);

        if((avail_lr == LR_01 || avail_lr == LR_11) && i >= (w - t_dx))
        {
            GET_REF_POS(mt[1], w - i, t_dy, offset);
            x = w;
            y = j - t_dy;
        }
        else
        {
            x = i + t_dx;
            y = -1;
        }
    }
    else if(ipm > IPD_HOR)
    {
        if (avail_lr == LR_01 || avail_lr == LR_11)
        {
            GET_REF_POS(mt[1], w - i, t_dy, offset);            

            if(j < t_dy)
            {
                GET_REF_POS(mt[0], w - i, t_dx, offset);
                x = i + t_dx;
                y = -1;
            }
            else
            {
                x = w;
                y = j - t_dy;
            }
        }
        else
        {
            GET_REF_POS(mt[1], i + 1, t_dy, offset);
            x = -1;
            y = j + t_dy;
        }
    }
    else
    {
        GET_REF_POS(mt[1], i + 1, t_dy, offset);

        if(j < t_dy)
        {
            GET_REF_POS(mt[0], j + 1, t_dx, offset);
            x = i - t_dx;
            y = -1;
        }
        else
        {
            if (avail_lr == LR_01)
            {
                GET_REF_POS(mt[1], w - i, t_dy, offset);
                x = w;
                y = j + t_dy;
            }
            else
            {
                x = -1;
                y = j - t_dy;
            }
        }
    }

    evc_assert(x != INT_MAX);
    evc_assert(y != INT_MAX);

    if(y == -1)
    {
        if(dxy < 0)
        {
            xn_n1 = x - 1;
            xn = x + 1;
            xn_p2 = x + 2;
        }
        else
        {
            xn_n1 = x + 1;
            xn = x - 1;
            xn_p2 = x - 2;
        }

        use_x = 1;
        ++num_selections;
        src_ch = src_up;
    }
    else if(x == -1)
    {
        if(dxy < 0)
        {
            yn_n1 = y - 1;
            yn = y + 1;
            yn_p2 = y + 2;
        }
        else
        {
            yn_n1 = y + 1;
            yn = y - 1;
            yn_p2 = y - 2;
        }

        use_x = 0;
        ++num_selections;
        src_ch = src_le;
    }
    else if(x == w)
    {
        if(dxy > 0)
        {
            yn_n1 = y - 1;
            yn = y + 1;
            yn_p2 = y + 2;
        }
        else
        {
            yn_n1 = y + 1;
            yn = y - 1;
            yn_p2 = y - 2;
        }

        use_x = 0;
        ++num_selections;
        src_ch = src_ri;
    }

    evc_assert(num_selections == 1);
    evc_assert(src_ch != NULL);

    if(use_x)
    {
        pn_n1 = xn_n1;
        p = x;
        pn = xn;
        pn_p2 = xn_p2;
    }
    else
    {
        pn_n1 = yn_n1;
        p = y;
        pn = yn;
        pn_p2 = yn_p2;
    }

    pn_n1 = EVC_MAX(EVC_MIN(pn_n1, pos_max), pos_min);
    p = EVC_MAX(EVC_MIN(p, pos_max), pos_min);
    pn = EVC_MAX(EVC_MIN(pn, pos_max), pos_min);
    pn_p2 = EVC_MAX(EVC_MIN(pn_p2, pos_max), pos_min);
    filter = (tbl_filt + offset)[0];

    temp_pel = (src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] + src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] + filter_offset) >> filter_bits;

    return EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, temp_pel);
}

void ipred_ang(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int w, int h, int ipm)
{
    int i, j;
    const int pos_max = w + h - 1;
#if M48879_IMPROVEMENT_INTRA
    const int pos_min = - 1;
#else
    const int pos_min = -h;
#endif

    for(j = 0; j < h; j++)
    {
        for(i = 0; i < w; i++)
        {
            dst[i] = ipred_ang_val(src_up, src_le, src_ri, avail_lr, ipm, i, j, w, pos_min, pos_max, h);
        }
        dst += w;
    }
}

void ipred_ul(pel *src_le, pel *src_up, pel * src_ri, u16 avail_lr, pel *dst, int w, int h)
{
    int i, j;
    for (i = 0; i<h; i++)
    {
        for (j = 0; j<w; j++)
        {
            int diag = i - j;
            if (diag > 0) {
                dst[j] = src_le[diag - 1];
            }
            else if (diag == 0)
            {
                dst[j] = src_up[-1];
            }
            else
            {
                dst[j] = src_up[-diag - 1];
            }
        }
        dst += w;
    }
}

void ipred_ur(pel *src_le, pel *src_up, pel * src_ri, u16 avail_lr, pel *dst, int w, int h)
{
    int i, j;
    for (i = 0; i<h; i++)
    {
        for (j = 0; j<w; j++)
        {
            dst[j] = (src_up[i + j + 1] + src_le[i + j + 1]) >> 1;
        }
        dst += w;
    }
}

/* intra prediction for baseline profile */
void evc_ipred_b(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm, int w, int h, u16 avail_cu)
{
    switch(ipm)
    {
        case IPD_VER_B:
            ipred_vert(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_HOR_B:
            ipred_hor(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_DC_B:
            ipred_dc_b(src_le, src_up, src_ri, avail_lr, dst, w, h, avail_cu);
            break;
        case IPD_UL_B:
            ipred_ul(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;

        case IPD_UR_B:
            ipred_ur(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        default:
#if M48879_IMPROVEMENT_INTRA
            printf("\n illegal intra prediction mode\n");
#else
            ipred_ang(src_le, src_up, src_ri, avail_lr, dst, w, h, ipm);
#endif
            break;
    }
}

void evc_ipred(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm, int w, int h, u16 avail_cu
#if M48879_IMPROVEMENT_INTRA
    , int sps_suco_flag
#endif
)
{
    switch(ipm)
    {
        case IPD_VER:
            ipred_vert(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_HOR:
            ipred_hor(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_DC:
            ipred_dc(src_le, src_up, src_ri, avail_lr, dst, w, h, avail_cu
#if M48879_IMPROVEMENT_INTRA
                , sps_suco_flag
#endif
            );
            break;
        case IPD_PLN:
            ipred_plane(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;

        case IPD_BI:
            ipred_bi(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        default:
            ipred_ang(src_le, src_up, src_ri, avail_lr, dst, w, h, ipm);
            break;
    }
}

void evc_ipred_uv_b(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm_c, int ipm, int w, int h, u16 avail_cu)
{
    switch(ipm_c)
    {

        case IPD_DC_C_B:
            ipred_dc_b(src_le, src_up, src_ri, avail_lr, dst, w, h, avail_cu);
            break;
        case IPD_HOR_C_B:
            ipred_hor(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_VER_C_B:
            ipred_vert(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_UL_C_B:
            ipred_ul(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;

        case IPD_UR_C_B:
            ipred_ur(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        default:
            printf("\n illegal chroma intra prediction mode\n");
            break;
    }
}

void evc_ipred_uv(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm_c, int ipm, int w, int h, u16 avail_cu
#if M48879_IMPROVEMENT_INTRA
    , int sps_suco_flag
#endif
)
{
    if(ipm_c == IPD_DM_C && EVC_IPRED_CHK_CONV(ipm))
    {
        ipm_c = EVC_IPRED_CONV_L2C(ipm);
    }

    switch(ipm_c)
    {
        case IPD_DM_C:
            switch(ipm)
            {
                case IPD_PLN:
                    ipred_plane(src_le, src_up, src_ri, avail_lr, dst, w, h);
                    break;
                default:
                    ipred_ang(src_le, src_up, src_ri, avail_lr, dst, w, h, ipm);
                    break;
            }
            break;

        case IPD_DC_C:
            ipred_dc(src_le, src_up, src_ri, avail_lr, dst, w, h, avail_cu
#if M48879_IMPROVEMENT_INTRA
                , sps_suco_flag
#endif
            );
            break;
        case IPD_HOR_C:
            ipred_hor(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        case IPD_VER_C:
            ipred_vert(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;

        case IPD_BI_C:
            ipred_bi(src_le, src_up, src_ri, avail_lr, dst, w, h);
            break;
        default:
            printf("\n illegal chroma intra prediction mode\n");
            break;
    }
}

#if M48879_IMPROVEMENT_INTRA
int intra_mode_list[IPD_CNT] = {
    IPD_DC, IPD_BI, IPD_VER, IPD_PLN, IPD_HOR,
    IPD_VER - 1, IPD_VER + 1, IPD_VER - 2, IPD_VER + 2, IPD_VER - 3, IPD_VER + 3,
    IPD_HOR - 1,IPD_HOR + 1, IPD_HOR - 2, IPD_HOR + 2, IPD_HOR - 3, IPD_HOR + 3,
    IPD_DIA_R,
    IPD_DIA_L, IPD_DIA_L - 3, IPD_DIA_L - 2, IPD_DIA_L - 1,
    IPD_DIA_U, IPD_DIA_U + 1, IPD_DIA_U + 2,
    IPD_VER + 5, IPD_VER + 4,
    IPD_HOR - 4, IPD_HOR - 5,
    IPD_VER - 5, IPD_VER - 4,
    IPD_HOR + 5, IPD_HOR + 4,
};
#else
int intra_mode_list[4][IPD_CNT] = {
    {
        IPD_DC, IPD_BI, IPD_VER, IPD_PLN, IPD_HOR,
        IPD_VER - 1, IPD_VER + 1, IPD_VER - 2, IPD_VER + 2, IPD_VER - 3, IPD_VER + 3,
        IPD_HOR - 1,IPD_HOR + 1, IPD_HOR - 2, IPD_HOR + 2, IPD_HOR - 3, IPD_HOR + 3,
        IPD_VER + 5, IPD_VER + 4,
        IPD_VER - 5, IPD_VER - 4,
        IPD_DIA_R, 
        IPD_DIA_L, IPD_DIA_L - 3, IPD_DIA_L - 2, IPD_DIA_L - 1,
        IPD_DIA_U, IPD_DIA_U + 1, IPD_DIA_U + 2, 
        IPD_HOR - 4, IPD_HOR - 5,
        IPD_HOR + 5, IPD_HOR + 4,
    },
    {
        IPD_DC, IPD_BI, IPD_VER, IPD_PLN, IPD_HOR,
        IPD_VER - 1, IPD_VER + 1, IPD_VER - 2, IPD_VER + 2, IPD_VER - 3, IPD_VER + 3,
        IPD_HOR - 1,IPD_HOR + 1, IPD_HOR - 2, IPD_HOR + 2, IPD_HOR - 3, IPD_HOR + 3,
        IPD_DIA_R, 
        IPD_VER + 5, IPD_VER + 4,
        IPD_HOR - 4, IPD_HOR - 5,
        IPD_DIA_L, IPD_DIA_L - 3, IPD_DIA_L - 2, IPD_DIA_L - 1,
        IPD_DIA_U, IPD_DIA_U + 1, IPD_DIA_U + 2,
        IPD_VER - 5, IPD_VER - 4,
        IPD_HOR + 5, IPD_HOR + 4,
    },
    {
        IPD_DC, IPD_BI, IPD_VER, IPD_PLN, IPD_HOR,
        IPD_VER - 1, IPD_VER + 1, IPD_VER - 2, IPD_VER + 2, IPD_VER - 3, IPD_VER + 3,
        IPD_HOR - 1,IPD_HOR + 1, IPD_HOR - 2, IPD_HOR + 2, IPD_HOR - 3, IPD_HOR + 3,
        IPD_DIA_L, IPD_DIA_L - 3, IPD_DIA_L - 2, IPD_DIA_L - 1,
        IPD_DIA_U, IPD_DIA_U + 1, IPD_DIA_U + 2,
        IPD_VER - 5, IPD_VER - 4,
        IPD_HOR + 5, IPD_HOR + 4,
        IPD_VER + 5, IPD_VER + 4,
        IPD_HOR - 4, IPD_HOR - 5,
        IPD_DIA_R 

    },
    {
        IPD_DC, IPD_BI, IPD_VER, IPD_PLN, IPD_HOR,
        IPD_VER - 1, IPD_VER + 1, IPD_VER - 2, IPD_VER + 2, IPD_VER - 3, IPD_VER + 3,
        IPD_HOR - 1,IPD_HOR + 1, IPD_HOR - 2, IPD_HOR + 2, IPD_HOR - 3, IPD_HOR + 3,
        IPD_DIA_R, 
        IPD_DIA_L, IPD_DIA_L - 3, IPD_DIA_L - 2, IPD_DIA_L - 1,
        IPD_DIA_U, IPD_DIA_U + 1, IPD_DIA_U + 2,
        IPD_VER + 5, IPD_VER + 4,
        IPD_HOR - 4, IPD_HOR - 5,
        IPD_VER - 5, IPD_VER - 4,
        IPD_HOR + 5, IPD_HOR + 4,
    },
};
#endif

void evc_get_mpm_b(int x_scu, int y_scu, int cuw, int cuh, u32 *map_scu, s8 *map_ipm, int scup, int w_scu,
                   u8 ** mpm, u16 avail_lr, u8 mpm_ext[8], u8 pms[IPD_CNT] /* 10 third MPM */)
{
    u8 ipm_l = IPD_DC, ipm_u = IPD_DC;

    if(x_scu > 0 && MCU_GET_IF(map_scu[scup - 1]) && MCU_GET_COD(map_scu[scup - 1]))
    {
        ipm_l = map_ipm[scup - 1] + 1;
    }
    if(y_scu > 0 && MCU_GET_IF(map_scu[scup - w_scu]) && MCU_GET_COD(map_scu[scup - w_scu]))
    {
        ipm_u = map_ipm[scup - w_scu] + 1;
    }
    *mpm = (u8*)&evey_tbl_mpm[ipm_l][ipm_u];
}

void evc_get_mpm(int x_scu, int y_scu, int cuw, int cuh, u32 *map_scu, s8 *map_ipm, int scup, int w_scu,
    u8 mpm[2],
                   u16 avail_lr, u8 mpm_ext[8], u8 pms[IPD_CNT] /* 10 third MPM */)
{
    u8 ipm_l = IPD_DC, ipm_u = IPD_DC;
    u8 ipm_r = IPD_DC;
    int scuw = cuw >> MIN_CU_LOG2;
    int valid_l = 0, valid_u = 0;
    int valid_r = 0;
    int i;
    int mode_idx = 0;
    int check = 8;
    int included_mode[IPD_CNT];
#if M48879_IMPROVEMENT_INTRA
    int *default_mode_list = intra_mode_list;
#else
    int *default_mode_list = intra_mode_list[avail_lr];
#endif

    evc_mset(included_mode, 0, sizeof(included_mode));

    if(x_scu > 0 && MCU_GET_IF(map_scu[scup - 1]) && MCU_GET_COD(map_scu[scup - 1]))
    {
        ipm_l = map_ipm[scup - 1];
        valid_l = 1;
    }

    if(y_scu > 0 && MCU_GET_IF(map_scu[scup - w_scu]) && MCU_GET_COD(map_scu[scup - w_scu]))
    {
        ipm_u = map_ipm[scup - w_scu];
        valid_u = 1;
    }

    if(x_scu + scuw < w_scu && MCU_GET_IF(map_scu[scup + scuw]) && MCU_GET_COD(map_scu[scup + scuw]))
    {
        ipm_r = map_ipm[scup + scuw];

        if(valid_l && valid_u)
        {
            if(ipm_l == ipm_u)
            {
                ipm_u = ipm_r;
            }
            else
            {
                valid_r = 1;
            }
        }
        else if(valid_l == 0)
        {
            ipm_l = ipm_r;
        }
        else if(valid_u == 0)
        {
            ipm_u = ipm_r;
        }
        if(valid_r)
        {
            if((ipm_l == ipm_r) || (ipm_u == ipm_r))
            {
                valid_r = 0;
            }
        }
    }

    mpm[0] = EVC_MIN(ipm_l, ipm_u);
    mpm[1] = EVC_MAX(ipm_l, ipm_u);

    if(mpm[0] == mpm[1])
    {
        mpm[0] = IPD_DC;
        mpm[1] = (mpm[1] == IPD_DC) ? IPD_BI : mpm[1];
    }

    if(valid_r)
    {
        int j;
        if(mpm[0] < 3 && mpm[1] < 3)
        {
            if(ipm_r < 3)
            {
                if(mpm[0] == IPD_DC)
                {
                    mpm_ext[0] = ((mpm[1] == IPD_BI) ? IPD_PLN : IPD_BI);
                }
                else if(mpm[0] == IPD_PLN)
                {
                    mpm_ext[0] = IPD_DC;
                }
                mpm_ext[1] = IPD_VER;
                mpm_ext[2] = IPD_HOR;
                mpm_ext[3] = IPD_DIA_R;
                mpm_ext[4] = IPD_DIA_L;
                mpm_ext[5] = IPD_DIA_U;
                mpm_ext[6] = IPD_VER + 4;
                mpm_ext[7] = IPD_HOR - 4;
            }
            else
            {
                int list[10] = {IPD_VER, IPD_HOR, IPD_DIA_R, IPD_PLN, IPD_DIA_L, IPD_DIA_U, IPD_VER + 4, IPD_HOR - 4, IPD_VER - 4, IPD_HOR + 4};
                if(mpm[0] == IPD_DC)
                {
                    mpm_ext[0] = ((mpm[1] == IPD_BI) ? IPD_PLN : IPD_BI);
                }
                else if(mpm[0] == IPD_PLN)
                {
                    mpm_ext[0] = IPD_DC;
                }
                mpm_ext[1] = ipm_r;
                mpm_ext[2] = ((ipm_r == 3 || ipm_r == 4) ? ipm_r + 1 : ipm_r - 2);
                mpm_ext[3] = ((ipm_r == IPD_CNT - 1 || ipm_r == IPD_CNT - 2) ? ipm_r - 1 : ipm_r + 2);
                int cnt_cand = 4;
                for (i = 0; i < 10; i++)
                {
                    for (j = 0; j < cnt_cand; j++)
                    {
                        if (list[i] == mpm_ext[j] || list[i] == mpm[0] || list[i] == mpm[1])
                        {
                            break;
                        }
                        if (j == cnt_cand - 1)
                        {
                            mpm_ext[cnt_cand] = list[i];
                            cnt_cand++;
                            break;
                        }
                    }
                    if (cnt_cand > 7)
                    {
                        break;
                    }
                }
            }
        }
        else if(mpm[0] < 3)
        {
            if(ipm_r < 3)
            {
                if(mpm[0] == IPD_PLN)
                {
                    mpm_ext[0] = IPD_BI;
                    mpm_ext[1] = IPD_DC;
                }
                else
                {
                    mpm_ext[0] = (mpm[0] == IPD_BI ? IPD_DC : IPD_BI);
                    mpm_ext[1] = IPD_PLN;
                }
                if(mpm[1] > IPD_CNT - 3)
                {
                    mpm_ext[2] = (mpm[1] == IPD_CNT - 1 ? IPD_CNT - 2 : IPD_CNT - 1);
                    mpm_ext[3] = IPD_CNT - 3;
                    mpm_ext[4] = IPD_CNT - 4;
                    mpm_ext[5] = IPD_CNT - 5;
                    mpm_ext[6] = IPD_HOR;
                    mpm_ext[7] = IPD_DIA_R;
                }
                else if(mpm[1] < 5)
                {
                    mpm_ext[2] = (mpm[1] == 3 ? 4 : 3);
                    mpm_ext[3] = 5;
                    mpm_ext[4] = 6;
                    mpm_ext[5] = 7;
                    mpm_ext[6] = IPD_VER;
                    mpm_ext[7] = IPD_DIA_R;
                }
                else
                {
                    mpm_ext[2] = mpm[1] + 2;
                    mpm_ext[3] = mpm[1] - 2;
                    mpm_ext[4] = mpm[1] + 1;
                    mpm_ext[5] = mpm[1] - 1;
                    if(mpm[1] <= 23 && mpm[1] >= 13)
                    {
                        mpm_ext[6] = mpm[1] - 5;
                        mpm_ext[7] = mpm[1] + 5;
                    }
                    else
                    {
                        mpm_ext[6] = (mpm[1] > 23) ? mpm[1] - 5 : mpm[1] + 5;
                        mpm_ext[7] = (mpm[1] > 23) ? mpm[1] - 10 : mpm[1] + 10;
                    }
                }
            }
            else
            {
                int list[15] = { 0, 0, 0, 0, 0, 0, 0, IPD_VER, IPD_HOR, IPD_DIA_R, IPD_PLN, IPD_DIA_L, IPD_DIA_U, IPD_VER + 4, IPD_HOR - 4};
                int cnt_cand = 0;
                list[0] = ((ipm_r == 3 || ipm_r == 4) ? ipm_r + 1 : ipm_r - 2);
                list[1] = ((ipm_r == IPD_CNT - 1 || ipm_r == IPD_CNT - 2) ? ipm_r - 1 : ipm_r + 2);
                list[2] = ((mpm[1] == 3 || mpm[1] == 4) ? mpm[1] + 1 : mpm[1] - 2);
                list[3] = ((mpm[1] == IPD_CNT - 1 || mpm[1] == IPD_CNT - 2) ? mpm[1] - 1 : mpm[1] + 2);
                list[4] = (ipm_r + mpm[1] + 1) >> 1;
                list[5] = (list[4] + ipm_r + 1) >> 1;
                list[6] = (list[4] + mpm[1] + 1) >> 1;

                if(mpm[0] == IPD_PLN)
                {
                    mpm_ext[0] = IPD_BI;
                    mpm_ext[1] = IPD_DC;
                }
                else
                {
                    mpm_ext[0] = (mpm[0] == IPD_BI ? IPD_DC : IPD_BI);
                    mpm_ext[1] = IPD_PLN;
                }
                mpm_ext[2] = ipm_r;

                cnt_cand = 3;
                for(i = 0; i < 15; i++)
                {
                    for(j = 0; j < cnt_cand; j++)
                    {
                        if(list[i] == mpm_ext[j] || list[i] == mpm[0] || list[i] == mpm[1])
                        {
                            break;
                        }
                        if(j == cnt_cand - 1)
                        {
                            mpm_ext[cnt_cand] = list[i];
                            cnt_cand++;
                            break;
                        }
                    }
                    if(cnt_cand > 7)
                    {
                        break;
                    }
                }
            }
        }
        else
        {
            if(ipm_r < 3)
            {
                int list[15] = {0, 0, 0, 0, 0, 0, 0, IPD_VER, IPD_HOR, IPD_DIA_R, IPD_PLN, IPD_DIA_L, IPD_DIA_U, IPD_VER + 4, IPD_HOR - 4};
                int cnt_cand = 0;
                list[0] = ((mpm[0] == 3 || mpm[0] == 4) ? mpm[0] + 1 : mpm[0] - 2);
                list[1] = ((mpm[0] == IPD_CNT - 2) ? mpm[0] - 1 : mpm[0] + 2);
                list[2] = ((mpm[1] == 4) ? mpm[1] + 1 : mpm[1] - 2);
                list[3] = ((mpm[1] == IPD_CNT - 1 || mpm[1] == IPD_CNT - 2) ? mpm[1] - 1 : mpm[1] + 2);
                list[4] = (mpm[0] + mpm[1] + 1) >> 1;
                list[5] = (list[4] + mpm[0] + 1) >> 1;
                list[6] = (list[4] + mpm[1] + 1) >> 1;

                mpm_ext[0] = ipm_r;
                mpm_ext[1] = (ipm_r == IPD_BI) ? IPD_DC : IPD_BI;

                cnt_cand = 2;
                for(i = 0; i < 15; i++)
                {
                    for(j = 0; j < cnt_cand; j++)
                    {
                        if(list[i] == mpm_ext[j] || list[i] == mpm[0] || list[i] == mpm[1])
                        {
                            break;
                        }
                        if(j == cnt_cand - 1)
                        {
                            mpm_ext[cnt_cand] = list[i];
                            cnt_cand++;
                            break;
                        }
                    }
                    if(cnt_cand > 7)
                    {
                        break;
                    }
                }
            }
            else
            {
                int list[16] = {0, 0, 0, 0, 0, 0, 0, 0, IPD_VER, IPD_HOR, IPD_DIA_R, IPD_PLN, IPD_DIA_L, IPD_DIA_U, IPD_VER + 4, IPD_HOR - 4};
                int cnt_cand = 0;
                list[0] = ((mpm[0] == 3 || mpm[0] == 4) ? mpm[0] + 1 : mpm[0] - 2);
                list[1] = ((mpm[0] == IPD_CNT - 2) ? mpm[0] - 1 : mpm[0] + 2);
                list[2] = ((mpm[1] == 4) ? mpm[1] + 1 : mpm[1] - 2);
                list[3] = ((mpm[1] == IPD_CNT - 1 || mpm[1] == IPD_CNT - 2) ? mpm[1] - 1 : mpm[1] + 2);
                list[4] = ((ipm_r == 3 || ipm_r == 4) ? ipm_r + 1 : ipm_r - 2);
                list[5] = ((ipm_r == IPD_CNT - 1 || ipm_r == IPD_CNT - 2) ? ipm_r - 1 : ipm_r + 2);
                list[6] = ((ipm_r < mpm[1]) ? (mpm[0] + ipm_r + 1) >> 1 : (mpm[0] + mpm[1] + 1) >> 1);
                list[7] = ((ipm_r < mpm[0]) ? (mpm[0] + mpm[1] + 1) >> 1 : (mpm[1] + ipm_r + 1) >> 1);

                mpm_ext[0] = IPD_BI;
                mpm_ext[1] = IPD_DC;
                mpm_ext[2] = ipm_r;

                cnt_cand = 3;
                for(i = 0; i < 16; i++)
                {
                    for(j = 0; j < cnt_cand; j++)
                    {
                        if(list[i] == mpm_ext[j] || list[i] == mpm[0] || list[i] == mpm[1])
                        {
                            break;
                        }
                        if(j == cnt_cand - 1)
                        {
                            mpm_ext[cnt_cand] = list[i];
                            cnt_cand++;
                            break;
                        }
                    }
                    if(cnt_cand > 7)
                    {
                        break;
                    }
                }
            }
        }
    }
    else
    {
        int j;
        if(mpm[0] < 3 && mpm[1] < 3)
        {
            int cnt_cand = 4;
            
            if(mpm[0] == IPD_DC)
            {
                mpm_ext[0] = ((mpm[1] == IPD_BI) ? IPD_PLN : IPD_BI);
            }
            else if(mpm[0] == IPD_PLN)
            {
                mpm_ext[0] = IPD_DC;
            }
            mpm_ext[1] = IPD_VER;
            mpm_ext[2] = IPD_HOR;
            mpm_ext[3] = IPD_DIA_R;
            mpm_ext[4] = IPD_DIA_L;
            mpm_ext[5] = IPD_DIA_U;
            mpm_ext[6] = IPD_VER + 4;
            mpm_ext[7] = IPD_HOR - 4;
        }
        else if(mpm[0] < 3)
        {
            if(mpm[0] == IPD_PLN)
            {
                mpm_ext[0] = IPD_BI;
                mpm_ext[1] = IPD_DC;
            }
            else
            {
                mpm_ext[0] = (mpm[0] == IPD_BI ? IPD_DC : IPD_BI);
                mpm_ext[1] = IPD_PLN;
            }

            if(mpm[1] > IPD_CNT - 3)
            {
                mpm_ext[2] = (mpm[1] == IPD_CNT - 1 ? IPD_CNT - 2 : IPD_CNT - 1);
                mpm_ext[3] = IPD_CNT - 3;
                mpm_ext[4] = IPD_CNT - 4;
                mpm_ext[5] = IPD_CNT - 5;
                mpm_ext[6] = IPD_HOR;
                mpm_ext[7] = IPD_DIA_R;
            }

            else if(mpm[1] < 5)
            {
                mpm_ext[2] = (mpm[1] == 3 ? 4 : 3);
                mpm_ext[3] = 5;
                mpm_ext[4] = 6;
                mpm_ext[5] = 7;
                mpm_ext[6] = IPD_VER;
                mpm_ext[7] = IPD_DIA_R;
            }
            else
            {
                mpm_ext[2] = mpm[1] + 2;
                mpm_ext[3] = mpm[1] - 2;
                mpm_ext[4] = mpm[1] + 1;
                mpm_ext[5] = mpm[1] - 1;

                if(mpm[1] <= 23 && mpm[1] >= 13)
                {
                    mpm_ext[6] = mpm[1] - 5;
                    mpm_ext[7] = mpm[1] + 5;
                }
                else
                {
                    mpm_ext[6] = (mpm[1] > 23) ? mpm[1] - 5 : mpm[1] + 5;
                    mpm_ext[7] = (mpm[1] > 23) ? mpm[1] - 10 : mpm[1] + 10;
                }
            }
        }
        else
        {
            int list[15] = {0, 0, 0, 0, 0, 0, 0, IPD_VER, IPD_HOR, IPD_DIA_R, IPD_PLN, IPD_DIA_L, IPD_DIA_U, IPD_VER + 4, IPD_HOR - 4};
            int cnt_cand = 0;
            list[0] = ((mpm[0] == 3 || mpm[0] == 4) ? mpm[0] + 1 : mpm[0] - 2);
            list[1] = ((mpm[0] == IPD_CNT - 2) ? mpm[0] - 1 : mpm[0] + 2);
            list[2] = ((mpm[1] == 4) ? mpm[1] + 1 : mpm[1] - 2);
            list[3] = ((mpm[1] == IPD_CNT - 1 || mpm[1] == IPD_CNT - 2) ? mpm[1] - 1 : mpm[1] + 2);
            list[4] = (mpm[0] + mpm[1] + 1) >> 1;
            list[5] = (list[4] + mpm[0] + 1) >> 1;
            list[6] = (list[4] + mpm[1] + 1) >> 1;

            mpm_ext[0] = IPD_BI;
            mpm_ext[1] = IPD_DC;

            cnt_cand = 2;
            for(i = 0; i < 15; i++)
            {
                for(j = 0; j < cnt_cand; j++)
                {
                    if(list[i] == mpm_ext[j] || list[i] == mpm[0] || list[i] == mpm[1])
                    {
                        break;
                    }
                    if(j == cnt_cand - 1)
                    {
                        mpm_ext[cnt_cand] = list[i];
                        cnt_cand++;
                        break;
                    }
                }
                if(cnt_cand > 7)
                {
                    break;
                }
            }
        }
    }

    for(i = 0; i < 2; i++)
    {
        if(!included_mode[mpm[i]])
        {
            included_mode[mpm[i]] = 1;
            pms[mode_idx] = mpm[i];
            mode_idx++;
        }
    }

    for(i = 0; i < check; i++)
    {
        if(!included_mode[mpm_ext[i]])
        {
            included_mode[mpm_ext[i]] = 1;
            pms[mode_idx] = mpm_ext[i];
            mode_idx++;
        }
    }

    for(i = 0; i < IPD_CNT; i++)
    {
        if(!included_mode[default_mode_list[i]])
        {
            included_mode[default_mode_list[i]] = 1;
            pms[mode_idx] = default_mode_list[i];
            mode_idx++;
        }
    }
    assert(mode_idx == IPD_CNT);
}
