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

#include <math.h>

#include "evc_def.h"
#include "evc_tbl.h"

#define ITX_SHIFT1   (7)                     /* shift after 1st IT stage */
#define ITX_SHIFT2   (12 - (BIT_DEPTH - 8))  /* shift after 2nd IT stage */

#define ITX_CLIP(x) \
    (s16)(((x)<MIN_TX_VAL)? MIN_TX_VAL: (((x)>MAX_TX_VAL)? MAX_TX_VAL: (x)))

#define MAX_TX_DYNAMIC_RANGE_32               31
#define MAX_TX_VAL_32                       2147483647
#define MIN_TX_VAL_32                      (-2147483647-1)
#define ITX_CLIP_32(x) \
    (s32)(((x)<=MIN_TX_VAL_32)? MIN_TX_VAL_32: (((x)>=MAX_TX_VAL_32)? MAX_TX_VAL_32: (x)))
#if X86_SSE
#define MAC_8PEL_MEM(src1, src2, m01, m02, m03, m04, mac) \
    m01 = _mm_loadu_si128((__m128i*)(src1)); \
    m02 = _mm_loadu_si128((__m128i*)(src2)); \
    \
    m03 = _mm_cvtepi16_epi32(m01); \
    m04 = _mm_cvtepi16_epi32(m02); \
    \
    m03 = _mm_mullo_epi32(m03, m04); \
    \
    m01 = _mm_srli_si128(m01, 8); \
    m02 = _mm_srli_si128(m02, 8); \
    \
    m01 = _mm_cvtepi16_epi32(m01); \
    m02 = _mm_cvtepi16_epi32(m02); \
    \
    m04 = _mm_mullo_epi32(m01, m02); \
    \
    mac = _mm_add_epi32(mac, m03); \
    mac = _mm_add_epi32(mac, m04);

#define MAC_8PEL_REG(mcoef, src2, mac) \
    mac = _mm_add_epi32(mac,  _mm_madd_epi16(mcoef,\
        _mm_loadu_si128((__m128i*)(src2))));

#define MAC_LINE(idx, w, mcoef, src2, mac, mtot, lane) \
    mac = _mm_setzero_si128(); \
    for (idx = 0; idx<((w)>>3); idx++) \
    { \
        MAC_8PEL_REG(mcoef[idx], src2 + (idx<<3), mac); \
    } \
    mac = _mm_hadd_epi32(mac, mac); \
    mac = _mm_hadd_epi32(mac, mac); \
    mtot = _mm_insert_epi32(mtot, _mm_extract_epi32(mac, 0), lane);

/* 32bit in xmm to 16bit clip with round-off */
#define ADD_SHIFT_CLIP_S32_TO_S16_4PEL(mval, madd, shift) \
    mval = _mm_srai_epi32(_mm_add_epi32(mval, madd), shift); \
    mval = _mm_packs_epi32(mval, mval);

/* top macro for inverse transforms */
#define ITX_MATRIX(coef, blk, tsize, line, shift, itm_tbl, skip_line) \
{\
    int i, j, k, h, w; \
    const s16 *itm; \
    s16 * c; \
\
    __m128i mc[8]; \
    __m128i mac, mtot=_mm_setzero_si128(), madd; \
\
    if(skip_line) \
    { \
        h = line - skip_line; \
        w = tsize; \
    } \
    else  \
    { \
        h = line; \
        w = tsize; \
    } \
\
    madd = _mm_set1_epi32(1 << (shift - 1)); \
\
    for (i = 0; i<h; i++) \
    { \
        itm = (itm_tbl); \
        c = coef + i; \
\
        for (k = 0; k<(w>>3); k++) \
        { \
            mc[k] = _mm_setr_epi16(c[0], \
                c[(1)*line], \
                c[(2)*line], \
                c[(3)*line], \
                c[(4)*line], \
                c[(5)*line], \
                c[(6)*line], \
                c[(7)*line]); \
            c += line << 3; \
        } \
\
        for (j = 0; j<(tsize>>2); j++) \
        { \
            MAC_LINE(k, w, mc, itm, mac, mtot, 0); \
            itm += tsize; \
\
            MAC_LINE(k, w, mc, itm, mac, mtot, 1); \
            itm += tsize; \
\
            MAC_LINE(k, w, mc, itm, mac, mtot, 2); \
            itm += tsize; \
\
            MAC_LINE(k, w, mc, itm, mac, mtot, 3); \
            itm += tsize; \
\
            ADD_SHIFT_CLIP_S32_TO_S16_4PEL(mtot, madd, shift); \
            _mm_storel_epi64((__m128i*)(blk + (j<<2)), mtot); \
        } \
        blk += tsize; \
    } \
}
#endif

static void itx_pb2b(void *src, void *dst, int shift, int line, int step)
{
    int j;
    s64 E, O;
    int add = shift == 0 ? 0 : 1 << (shift - 1);
#define RUN_ITX_PB2(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* E and O */\
        E = *((type_src *)src + 0 * line + j) + *((type_src *)src + 1 * line + j);\
        O = *((type_src *)src + 0 * line + j) - *((type_src *)src + 1 * line + j);\
        \
        if(step == 0)\
        {\
            *((type_dst *)dst + j * 2 + 0) = ITX_CLIP_32((evc_tbl_tm2[0][0] * E + add) >> shift); \
            *((type_dst *)dst + j * 2 + 1) = ITX_CLIP_32((evc_tbl_tm2[1][0] * O + add) >> shift); \
        }\
        else\
        {\
            *((type_dst *)dst + j * 2 + 0) = ITX_CLIP((evc_tbl_tm2[0][0] * E + add) >> shift); \
            *((type_dst *)dst + j * 2 + 1) = ITX_CLIP((evc_tbl_tm2[1][0] * O + add) >> shift); \
        }\
    }
    if (step == 0)
    {
        RUN_ITX_PB2(src, dst, s16, s32);
    }
    else
    {
        RUN_ITX_PB2(src, dst, s32, s16);
    }
}

static void itx_pb4b(void *src, void *dst, int shift, int line, int step)
{
    int j;
    s64 E[2], O[2];
    int add = shift == 0 ? 0 : 1 << (shift - 1);

#define RUN_ITX_PB4(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */\
        O[0] = evc_tbl_tm4[1][0] * *((type_src * )src + 1 * line + j) + evc_tbl_tm4[3][0] * *((type_src * )src + 3 * line + j);\
        O[1] = evc_tbl_tm4[1][1] * *((type_src * )src + 1 * line + j) + evc_tbl_tm4[3][1] * *((type_src * )src + 3 * line + j);\
        E[0] = evc_tbl_tm4[0][0] * *((type_src * )src + 0 * line + j) + evc_tbl_tm4[2][0] * *((type_src * )src + 2 * line + j);\
        E[1] = evc_tbl_tm4[0][1] * *((type_src * )src + 0 * line + j) + evc_tbl_tm4[2][1] * *((type_src * )src + 2 * line + j);\
        \
        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */\
        if (step == 0)\
        {\
            *((type_dst * )dst + j * 4 + 0) = ITX_CLIP_32((E[0] + O[0] + add) >> shift);\
            *((type_dst * )dst + j * 4 + 1) = ITX_CLIP_32((E[1] + O[1] + add) >> shift);\
            *((type_dst * )dst + j * 4 + 2) = ITX_CLIP_32((E[1] - O[1] + add) >> shift);\
            *((type_dst * )dst + j * 4 + 3) = ITX_CLIP_32((E[0] - O[0] + add) >> shift);\
        }\
        else\
        {\
            *((type_dst * )dst + j * 4 + 0) = ITX_CLIP((E[0] + O[0] + add) >> shift);\
            *((type_dst * )dst + j * 4 + 1) = ITX_CLIP((E[1] + O[1] + add) >> shift);\
            *((type_dst * )dst + j * 4 + 2) = ITX_CLIP((E[1] - O[1] + add) >> shift);\
            *((type_dst * )dst + j * 4 + 3) = ITX_CLIP((E[0] - O[0] + add) >> shift);\
        }\
    }

    if (step == 0)
    {
        RUN_ITX_PB4(src, dst, s16, s32);
    }
    else
    {
        RUN_ITX_PB4(src, dst, s32, s16);
    }

}

static void itx_pb8b(void *src, void *dst, int shift, int line, int step)
{
    int j, k;
    s64 E[4], O[4];
    s64 EE[2], EO[2];
    int add = shift == 0 ? 0 : 1 << (shift - 1);
#define RUN_ITX_PB8(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */\
        for (k = 0; k < 4; k++)\
        {\
            O[k] = evc_tbl_tm8[1][k] * *((type_src * )src + 1 * line + j) + evc_tbl_tm8[3][k] * *((type_src * )src + 3 * line + j)\
                 + evc_tbl_tm8[5][k] * *((type_src * )src + 5 * line + j) + evc_tbl_tm8[7][k] * *((type_src * )src + 7 * line + j);\
        }\
        \
        EO[0] = evc_tbl_tm8[2][0] * *((type_src * )src + 2 * line + j) + evc_tbl_tm8[6][0] * *((type_src * )src + 6 * line + j);\
        EO[1] = evc_tbl_tm8[2][1] * *((type_src * )src + 2 * line + j) + evc_tbl_tm8[6][1] * *((type_src * )src + 6 * line + j);\
        EE[0] = evc_tbl_tm8[0][0] * *((type_src * )src + 0 * line + j) + evc_tbl_tm8[4][0] * *((type_src * )src + 4 * line + j);\
        EE[1] = evc_tbl_tm8[0][1] * *((type_src * )src + 0 * line + j) + evc_tbl_tm8[4][1] * *((type_src * )src + 4 * line + j);\
        \
        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */\
        E[0] = EE[0] + EO[0];\
        E[3] = EE[0] - EO[0];\
        E[1] = EE[1] + EO[1];\
        E[2] = EE[1] - EO[1];\
        \
        if(step == 0)\
        {\
            for (k = 0; k < 4; k++)\
            {\
                *((type_dst * )dst + j * 8 + k    ) = ITX_CLIP_32((E[k] + O[k] + add) >> shift);\
                *((type_dst * )dst + j * 8 + k + 4) = ITX_CLIP_32((E[3 - k] - O[3 - k] + add) >> shift);\
            }\
        }\
        else\
        {\
            for (k = 0; k < 4; k++)\
            {\
                *((type_dst * )dst + j * 8 + k    ) = ITX_CLIP((E[k] + O[k] + add) >> shift);\
                *((type_dst * )dst + j * 8 + k + 4) = ITX_CLIP((E[3 - k] - O[3 - k] + add) >> shift);\
            }\
        }\
    }

    if (step == 0)
    {
        RUN_ITX_PB8(src, dst, s16, s32);
    }
    else
    {
        RUN_ITX_PB8(src, dst, s32, s16);
    }
}

static void itx_pb16b(void *src, void *dst, int shift, int line, int step)
{
    int j, k;
    s64 E[8], O[8];
    s64 EE[4], EO[4];
    s64 EEE[2], EEO[2];
    int add = shift == 0 ? 0 : 1 << (shift - 1);
#define RUN_ITX_PB16(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */\
        for (k = 0; k < 8; k++)\
        {\
            O[k] = evc_tbl_tm16[1][k]  * *((type_src * )src + 1  * line + j) + evc_tbl_tm16[3][k]  * *((type_src * )src + 3  * line + j) +\
                   evc_tbl_tm16[5][k]  * *((type_src * )src + 5  * line + j) + evc_tbl_tm16[7][k]  * *((type_src * )src + 7  * line + j) +\
                   evc_tbl_tm16[9][k]  * *((type_src * )src + 9  * line + j) + evc_tbl_tm16[11][k] * *((type_src * )src + 11 * line + j) +\
                   evc_tbl_tm16[13][k] * *((type_src * )src + 13 * line + j) + evc_tbl_tm16[15][k] * *((type_src * )src + 15 * line + j);\
        }\
        \
        for (k = 0; k < 4; k++)\
        {\
            EO[k] = evc_tbl_tm16[2][k]  * *((type_src * )src + 2  * line + j) + evc_tbl_tm16[6][k]  * *((type_src * )src + 6  * line + j) +\
                    evc_tbl_tm16[10][k] * *((type_src * )src + 10 * line + j) + evc_tbl_tm16[14][k] * *((type_src * )src + 14 * line + j);\
        }\
        \
        EEO[0] = evc_tbl_tm16[4][0] * *((type_src * )src + 4 * line + j) + evc_tbl_tm16[12][0] * *((type_src * )src + 12 * line + j);\
        EEE[0] = evc_tbl_tm16[0][0] * *((type_src * )src + 0 * line + j) + evc_tbl_tm16[8][0]  * *((type_src * )src + 8  * line + j);\
        EEO[1] = evc_tbl_tm16[4][1] * *((type_src * )src + 4 * line + j) + evc_tbl_tm16[12][1] * *((type_src * )src + 12 * line + j);\
        EEE[1] = evc_tbl_tm16[0][1] * *((type_src * )src + 0 * line + j) + evc_tbl_tm16[8][1]  * *((type_src * )src + 8  * line + j);\
        \
        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */\
        for (k = 0; k < 2; k++)\
        {\
            EE[k] = EEE[k] + EEO[k];\
            EE[k + 2] = EEE[1 - k] - EEO[1 - k];\
        }\
        for (k = 0; k < 4; k++)\
        {\
            E[k] = EE[k] + EO[k];\
            E[k + 4] = EE[3 - k] - EO[3 - k];\
        }\
        if(step == 0)\
        {\
            for (k = 0; k < 8; k++)\
            {\
                *((type_dst * )dst + j * 16 + k    ) = ITX_CLIP_32((E[k] + O[k] + add) >> shift); \
                *((type_dst * )dst + j * 16 + k + 8) = ITX_CLIP_32((E[7 - k] - O[7 - k] + add) >> shift); \
            }\
        }\
        else\
        {\
            for (k = 0; k < 8; k++)\
            {\
                *((type_dst * )dst + j * 16 + k    ) = ITX_CLIP((E[k] + O[k] + add) >> shift); \
                *((type_dst * )dst + j * 16 + k + 8) = ITX_CLIP((E[7 - k] - O[7 - k] + add) >> shift); \
            }\
        }\
    }

    if (step == 0)
    {
        RUN_ITX_PB16(src, dst, s16, s32);
    }
    else
    {
        RUN_ITX_PB16(src, dst, s32, s16);    
    }
}

static void itx_pb32b(void *src, void *dst, int shift, int line, int step)
{
    int j, k;
    s64 E[16], O[16];
    s64 EE[8], EO[8];
    s64 EEE[4], EEO[4];
    s64 EEEE[2], EEEO[2];
    int add = shift == 0 ? 0 : 1 << (shift - 1);
#define RUN_ITX_PB32(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        for (k = 0; k < 16; k++) \
        {\
            O[k] = evc_tbl_tm32[1][k]  * *((type_src * )src + 1  * line + j)  + \
                   evc_tbl_tm32[3][k]  * *((type_src * )src + 3  * line + j)  + \
                   evc_tbl_tm32[5][k]  * *((type_src * )src + 5  * line + j)  + \
                   evc_tbl_tm32[7][k]  * *((type_src * )src + 7  * line + j)  + \
                   evc_tbl_tm32[9][k]  * *((type_src * )src + 9  * line + j)  + \
                   evc_tbl_tm32[11][k] * *((type_src * )src + 11 * line + j) + \
                   evc_tbl_tm32[13][k] * *((type_src * )src + 13 * line + j) + \
                   evc_tbl_tm32[15][k] * *((type_src * )src + 15 * line + j) + \
                   evc_tbl_tm32[17][k] * *((type_src * )src + 17 * line + j) + \
                   evc_tbl_tm32[19][k] * *((type_src * )src + 19 * line + j) + \
                   evc_tbl_tm32[21][k] * *((type_src * )src + 21 * line + j) + \
                   evc_tbl_tm32[23][k] * *((type_src * )src + 23 * line + j) + \
                   evc_tbl_tm32[25][k] * *((type_src * )src + 25 * line + j) + \
                   evc_tbl_tm32[27][k] * *((type_src * )src + 27 * line + j) + \
                   evc_tbl_tm32[29][k] * *((type_src * )src + 29 * line + j) + \
                   evc_tbl_tm32[31][k] * *((type_src * )src + 31 * line + j);\
        }\
        \
        for (k = 0; k < 8; k++)\
        {\
            EO[k] = evc_tbl_tm32[2][k]  * *((type_src * )src + 2  * line + j) + \
                    evc_tbl_tm32[6][k]  * *((type_src * )src + 6  * line + j) + \
                    evc_tbl_tm32[10][k] * *((type_src * )src + 10 * line + j) + \
                    evc_tbl_tm32[14][k] * *((type_src * )src + 14 * line + j) + \
                    evc_tbl_tm32[18][k] * *((type_src * )src + 18 * line + j) + \
                    evc_tbl_tm32[22][k] * *((type_src * )src + 22 * line + j) + \
                    evc_tbl_tm32[26][k] * *((type_src * )src + 26 * line + j) + \
                    evc_tbl_tm32[30][k] * *((type_src * )src + 30 * line + j);\
        }\
        \
        for (k = 0; k < 4; k++)\
        {\
            EEO[k] = evc_tbl_tm32[4][k]  * *((type_src * )src + 4  * line + j) + \
                     evc_tbl_tm32[12][k] * *((type_src * )src + 12 * line + j) + \
                     evc_tbl_tm32[20][k] * *((type_src * )src + 20 * line + j) + \
                     evc_tbl_tm32[28][k] * *((type_src * )src + 28 * line + j);\
        }\
        \
        EEEO[0] = evc_tbl_tm32[8][0] * *((type_src * )src + 8 * line + j) + evc_tbl_tm32[24][0] * *((type_src * )src + 24 * line + j);\
        EEEO[1] = evc_tbl_tm32[8][1] * *((type_src * )src + 8 * line + j) + evc_tbl_tm32[24][1] * *((type_src * )src + 24 * line + j);\
        EEEE[0] = evc_tbl_tm32[0][0] * *((type_src * )src + 0 * line + j) + evc_tbl_tm32[16][0] * *((type_src * )src + 16 * line + j);\
        EEEE[1] = evc_tbl_tm32[0][1] * *((type_src * )src + 0 * line + j) + evc_tbl_tm32[16][1] * *((type_src * )src + 16 * line + j);\
        \
        EEE[0] = EEEE[0] + EEEO[0];\
        EEE[3] = EEEE[0] - EEEO[0];\
        EEE[1] = EEEE[1] + EEEO[1];\
        EEE[2] = EEEE[1] - EEEO[1];\
        for (k = 0; k<4; k++)\
        {\
            EE[k] = EEE[k] + EEO[k];\
            EE[k + 4] = EEE[3 - k] - EEO[3 - k];\
        }\
        for (k = 0; k<8; k++)\
        {\
            E[k] = EE[k] + EO[k];\
            E[k + 8] = EE[7 - k] - EO[7 - k];\
        }\
        if (step == 0)\
        {\
            for (k = 0; k < 16; k++)\
            {\
                *((type_dst * )dst + j * 32 + k     ) = ITX_CLIP_32((E[k] + O[k] + add) >> shift);\
                *((type_dst * )dst + j * 32 + k + 16) = ITX_CLIP_32((E[15 - k] - O[15 - k] + add) >> shift);\
            }\
        }\
        else\
        {\
            for (k = 0; k < 16; k++)\
            {\
                *((type_dst * )dst + j * 32 + k     ) = ITX_CLIP((E[k] + O[k] + add) >> shift);\
                *((type_dst * )dst + j * 32 + k + 16) = ITX_CLIP((E[15 - k] - O[15 - k] + add) >> shift);\
            }\
        }\
    }

    if (step == 0)
    {
        RUN_ITX_PB32(src, dst, s16, s32);
    }
    else
    {
        RUN_ITX_PB32(src, dst, s32, s16);
    }

}

static void itx_pb64b(void *src, void *dst, int shift, int line, int step)
{
    const int tx_size = 64;
    const s8 *tm = evc_tbl_tm64[0];
    int j, k;
    s64 E[32], O[32];
    s64 EE[16], EO[16];
    s64 EEE[8], EEO[8];
    s64 EEEE[4], EEEO[4];
    s64 EEEEE[2], EEEEO[2];
    int add = shift == 0 ? 0 : 1 << (shift - 1);
#define RUN_ITX_PB64(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++) \
    { \
        for (k = 0; k < 32; k++) \
        { \
            O[k] = tm[1  * 64 + k] * *((type_src * )src +      line) + tm[3  * 64 + k] * *((type_src * )src + 3  * line) + \
                   tm[5  * 64 + k] * *((type_src * )src + 5  * line) + tm[7  * 64 + k] * *((type_src * )src + 7  * line) + \
                   tm[9  * 64 + k] * *((type_src * )src + 9  * line) + tm[11 * 64 + k] * *((type_src * )src + 11 * line) + \
                   tm[13 * 64 + k] * *((type_src * )src + 13 * line) + tm[15 * 64 + k] * *((type_src * )src + 15 * line) + \
                   tm[17 * 64 + k] * *((type_src * )src + 17 * line) + tm[19 * 64 + k] * *((type_src * )src + 19 * line) + \
                   tm[21 * 64 + k] * *((type_src * )src + 21 * line) + tm[23 * 64 + k] * *((type_src * )src + 23 * line) + \
                   tm[25 * 64 + k] * *((type_src * )src + 25 * line) + tm[27 * 64 + k] * *((type_src * )src + 27 * line) + \
                   tm[29 * 64 + k] * *((type_src * )src + 29 * line) + tm[31 * 64 + k] * *((type_src * )src + 31 * line) + \
                   tm[33 * 64 + k] * *((type_src * )src + 33 * line) + tm[35 * 64 + k] * *((type_src * )src + 35 * line) + \
                   tm[37 * 64 + k] * *((type_src * )src + 37 * line) + tm[39 * 64 + k] * *((type_src * )src + 39 * line) + \
                   tm[41 * 64 + k] * *((type_src * )src + 41 * line) + tm[43 * 64 + k] * *((type_src * )src + 43 * line) + \
                   tm[45 * 64 + k] * *((type_src * )src + 45 * line) + tm[47 * 64 + k] * *((type_src * )src + 47 * line) + \
                   tm[49 * 64 + k] * *((type_src * )src + 49 * line) + tm[51 * 64 + k] * *((type_src * )src + 51 * line) + \
                   tm[53 * 64 + k] * *((type_src * )src + 53 * line) + tm[55 * 64 + k] * *((type_src * )src + 55 * line) + \
                   tm[57 * 64 + k] * *((type_src * )src + 57 * line) + tm[59 * 64 + k] * *((type_src * )src + 59 * line) + \
                   tm[61 * 64 + k] * *((type_src * )src + 61 * line) + tm[63 * 64 + k] * *((type_src * )src + 63 * line);  \
        } \
        \
        for (k = 0; k < 16; k++) \
        { \
            EO[k] = tm[2  * 64 + k] * *((type_src * )src + 2  * line) + tm[6  * 64 + k] * *((type_src * )src + 6  * line) + \
                    tm[10 * 64 + k] * *((type_src * )src + 10 * line) + tm[14 * 64 + k] * *((type_src * )src + 14 * line) + \
                    tm[18 * 64 + k] * *((type_src * )src + 18 * line) + tm[22 * 64 + k] * *((type_src * )src + 22 * line) + \
                    tm[26 * 64 + k] * *((type_src * )src + 26 * line) + tm[30 * 64 + k] * *((type_src * )src + 30 * line) + \
                    tm[34 * 64 + k] * *((type_src * )src + 34 * line) + tm[38 * 64 + k] * *((type_src * )src + 38 * line) + \
                    tm[42 * 64 + k] * *((type_src * )src + 42 * line) + tm[46 * 64 + k] * *((type_src * )src + 46 * line) + \
                    tm[50 * 64 + k] * *((type_src * )src + 50 * line) + tm[54 * 64 + k] * *((type_src * )src + 54 * line) + \
                    tm[58 * 64 + k] * *((type_src * )src + 58 * line) + tm[62 * 64 + k] * *((type_src * )src + 62 * line);  \
        } \
        \
        for (k = 0; k < 8; k++) \
        {\
            EEO[k] = tm[4  * 64 + k] * *((type_src * )src + 4  * line) + tm[12 * 64 + k] * *((type_src * )src + 12 * line) + \
                     tm[20 * 64 + k] * *((type_src * )src + 20 * line) + tm[28 * 64 + k] * *((type_src * )src + 28 * line) + \
                     tm[36 * 64 + k] * *((type_src * )src + 36 * line) + tm[44 * 64 + k] * *((type_src * )src + 44 * line) + \
                     tm[52 * 64 + k] * *((type_src * )src + 52 * line) + tm[60 * 64 + k] * *((type_src * )src + 60 * line);  \
        } \
        \
        for (k = 0; k<4; k++)\
        {\
            EEEO[k] = tm[8  * 64 + k] * *((type_src * )src + 8  * line) + tm[24 * 64 + k] * *((type_src * )src + 24 * line) + \
                      tm[40 * 64 + k] * *((type_src * )src + 40 * line) + tm[56 * 64 + k] * *((type_src * )src + 56 * line);  \
        }\
        EEEEO[0] = tm[16 * 64 + 0] * *((type_src * )src + 16 * line) + tm[48 * 64 + 0] * *((type_src * )src + 48 * line);\
        EEEEO[1] = tm[16 * 64 + 1] * *((type_src * )src + 16 * line) + tm[48 * 64 + 1] * *((type_src * )src + 48 * line);\
        EEEEE[0] = tm[0  * 64 + 0] * *((type_src * )src + 0        ) + tm[32 * 64 + 0] * *((type_src * )src + 32 * line);\
        EEEEE[1] = tm[0  * 64 + 1] * *((type_src * )src + 0        ) + tm[32 * 64 + 1] * *((type_src * )src + 32 * line);\
        \
        for (k = 0; k < 2; k++)\
        {\
            EEEE[k] = EEEEE[k] + EEEEO[k];\
            EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];\
        }\
        for (k = 0; k < 4; k++)\
        {\
            EEE[k] = EEEE[k] + EEEO[k];\
            EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];\
        }\
        for (k = 0; k < 8; k++)\
        {\
            EE[k] = EEE[k] + EEO[k];\
            EE[k + 8] = EEE[7 - k] - EEO[7 - k];\
        }\
        for (k = 0; k < 16; k++)\
        {\
            E[k] = EE[k] + EO[k];\
            E[k + 16] = EE[15 - k] - EO[15 - k];\
        }\
        if (step == 0)\
        {\
            for (k = 0; k < 32; k++)\
            {\
                *((type_dst * )dst + k     ) = ITX_CLIP_32((E[k] + O[k] + add) >> shift);\
                *((type_dst * )dst + k + 32) = ITX_CLIP_32((E[31 - k] - O[31 - k] + add) >> shift);\
            }\
        }\
        else\
        {\
            for (k = 0; k < 32; k++)\
            {\
                *((type_dst * )dst + k     ) = ITX_CLIP((E[k] + O[k] + add) >> shift);\
                *((type_dst * )dst + k + 32) = ITX_CLIP((E[31 - k] - O[31 - k] + add) >> shift);\
            }\
        }\
        src = (type_src * )src + 1;\
        dst = (type_dst * )dst + tx_size;\
    }

    if (step == 0)
    {
        RUN_ITX_PB64(src, dst, s16, s32);
    }
    else
    {
        RUN_ITX_PB64(src, dst, s32, s16);
    }
    
}

void evc_itrans_ats_intra_DST7_B4(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DST7_B8(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DST7_B16(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DST7_B32(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DCT8_B4(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DCT8_B8(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DCT8_B16(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);
void evc_itrans_ats_intra_DCT8_B32(s16 *coeff, s16 *block, int shift, int line, int skip_line, int skip_line_2);

typedef void INV_TRANS(s16 *, s16 *, int, int, int, int);

INV_TRANS *evc_itrans_map_tbl[16][5] =
{
    { NULL, evc_itrans_ats_intra_DCT8_B4, evc_itrans_ats_intra_DCT8_B8, evc_itrans_ats_intra_DCT8_B16, evc_itrans_ats_intra_DCT8_B32 },
    { NULL, evc_itrans_ats_intra_DST7_B4, evc_itrans_ats_intra_DST7_B8, evc_itrans_ats_intra_DST7_B16, evc_itrans_ats_intra_DST7_B32 },
};

void evc_itrans_ats_intra(s16 *coef, int log2_cuw, int log2_cuh, u8 ats_tu, int skip_w, int skip_h);
void evc_it_MxN_ats_intra(s16 *coef, int tuw, int tuh, int bit_depth, const int max_log2_tr_dynamic_range, u8 ats_intra_tridx, int skip_w, int skip_h);

void evc_init_multi_tbl()
{
    int c, k, n, i;

    c = 2;
    for (i = 0; i < 7; i++)
    {
        const double s = sqrt((double)c) * (64);
        s16 *tm = NULL;

        switch (i)
        {
        case 0: tm = evc_tbl_tr2[0][0]; break;
        case 1: tm = evc_tbl_tr4[0][0]; break;
        case 2: tm = evc_tbl_tr8[0][0]; break;
        case 3: tm = evc_tbl_tr16[0][0]; break;
        case 4: tm = evc_tbl_tr32[0][0]; break;
        case 5: tm = evc_tbl_tr64[0][0]; break;
        case 6: tm = evc_tbl_tr128[0][0]; break;
        case 7: exit(0); break;
        }

        for (k = 0; k < c; k++)
        {
            for (n = 0; n < c; n++)
            {
                double v;

                /* DCT-VIII */
                v = cos(PI*(k + 0.5)*(n + 0.5) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
                tm[DCT8*c*c + k*c + n] = (s16)(s * v + (v > 0 ? 0.5 : -0.5));

                /* DST-VII */
                v = sin(PI*(k + 0.5)*(n + 1) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
                tm[DST7*c*c + k*c + n] = (s16)(s * v + (v > 0 ? 0.5 : -0.5));
            }
        }
        c <<= 1;
    }
}

void evc_init_multi_inv_tbl()
{
    int c, k, n, i;

    c = 2;
    for (i = 0; i < 7; i++)
    {
        const double s = sqrt((double)c) * (64);
        s16 *tm = NULL;

        switch (i)
        {
        case 0: tm = evc_tbl_inv_tr2[0][0]; break;
        case 1: tm = evc_tbl_inv_tr4[0][0]; break;
        case 2: tm = evc_tbl_inv_tr8[0][0]; break;
        case 3: tm = evc_tbl_inv_tr16[0][0]; break;
        case 4: tm = evc_tbl_inv_tr32[0][0]; break;
        case 5: tm = evc_tbl_inv_tr64[0][0]; break;
        case 6: tm = evc_tbl_inv_tr128[0][0]; break;
        case 7: exit(0); break;
        }

        for (k = 0; k < c; k++)
        {
            for (n = 0; n < c; n++)
            {
                double v;

                /* DCT-VIII */
                v = cos(PI*(k + 0.5)*(n + 0.5) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
                tm[DCT8*c*c + n*c + k] = (s16)(s * v + (v > 0 ? 0.5 : -0.5));

                /* DST-VII */
                v = sin(PI*(k + 0.5)*(n + 1) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
                tm[DST7*c*c + n*c + k] = (s16)(s * v + (v > 0 ? 0.5 : -0.5));
            }
        }
        c <<= 1;
    }
}

void evc_itrans_ats_intra_DST7_B4(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
    int i, c[4];
    int rnd_factor = 1 << (shift - 1);
    const s16 *tm = evc_tbl_tr4[DST7][0];
    const int reduced_line = line - skip_line;

    for (i = 0; i < reduced_line; i++)
    {
        /* Intermediate Variables */
        c[0] = coef[0] + coef[2 * line];
        c[1] = coef[2 * line] + coef[3 * line];
        c[2] = coef[0] - coef[3 * line];
        c[3] = tm[2] * coef[1 * line];

        block[0] = EVC_CLIP3(-32768, 32767, (tm[0] * c[0] + tm[1] * c[1] + c[3] + rnd_factor) >> shift);
        block[1] = EVC_CLIP3(-32768, 32767, (tm[1] * c[2] - tm[0] * c[1] + c[3] + rnd_factor) >> shift);
        block[2] = EVC_CLIP3(-32768, 32767, (tm[2] * (coef[0] - coef[2 * line] + coef[3 * line]) + rnd_factor) >> shift);
        block[3] = EVC_CLIP3(-32768, 32767, (tm[1] * c[0] + tm[0] * c[2] - c[3] + rnd_factor) >> shift);

        block += 4;
        coef++;
    }

    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 2) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DST7_B8(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
#if X86_SSE
    ITX_MATRIX(coef, block, 8, line, shift, evc_tbl_inv_tr8[DST7][0], skip_line);
#else
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 8;
    const s16 *tm = evc_tbl_tr8[DST7][0];
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < cut_off; k++)
            {
                sum += coef[k*line] * tm[k*tr_size + j];
            }
            block[j] = EVC_CLIP3(-32768, 32767, (int)(sum + rnd_factor) >> shift);
        }
        block += tr_size;
        coef++;
    }
#endif
    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 3) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DST7_B16(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
#if X86_SSE
    ITX_MATRIX(coef, block, 16, line, shift, evc_tbl_inv_tr16[DST7][0], skip_line);
#else
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 16;
    const s16 *tm = evc_tbl_tr16[DST7][0];
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < cut_off; k++)
            {
                sum += coef[k*line] * tm[k*tr_size + j];
            }
            block[j] = EVC_CLIP3(-32768, 32767, (int)(sum + rnd_factor) >> shift);
        }
        block += tr_size;
        coef++;
    }
#endif
    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 4) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DST7_B32(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
#if X86_SSE
    ITX_MATRIX(coef, block, 32, line, shift, evc_tbl_inv_tr32[DST7][0], skip_line);
#else
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 32;
    const s16 *tm = evc_tbl_tr32[DST7][0];
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < cut_off; k++)
            {
                sum += coef[k*line] * tm[k*tr_size + j];
            }
            block[j] = EVC_CLIP3(-32768, 32767, (int)(sum + rnd_factor) >> shift);
        }
        block += tr_size;
        coef++;
    }
#endif
    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 5) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DCT8_B4(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
    int i;
    int rnd_factor = 1 << (shift - 1);
    const s16 *tm = evc_tbl_tr4[DCT8][0];
    int c[4];
    const int reduced_line = line - skip_line;

    for (i = 0; i < reduced_line; i++)
    {
        /* Intermediate Variables */
        c[0] = coef[0] + coef[3 * line];
        c[1] = coef[2 * line] + coef[0];
        c[2] = coef[3 * line] - coef[2 * line];
        c[3] = tm[1] * coef[1 * line];

        block[0] = EVC_CLIP3(-32768, 32767, (tm[3] * c[0] + tm[2] * c[1] + c[3] + rnd_factor) >> shift);
        block[1] = EVC_CLIP3(-32768, 32767, (tm[1] * (coef[0 * line] - coef[2 * line] - coef[3 * line]) + rnd_factor) >> shift);
        block[2] = EVC_CLIP3(-32768, 32767, (tm[3] * c[2] + tm[2] * c[0] - c[3] + rnd_factor) >> shift);
        block[3] = EVC_CLIP3(-32768, 32767, (tm[3] * c[1] - tm[2] * c[2] - c[3] + rnd_factor) >> shift);

        block += 4;
        coef++;
    }

    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 2) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DCT8_B8(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
#if X86_SSE
    ITX_MATRIX(coef, block, 8, line, shift, evc_tbl_inv_tr8[DCT8][0], skip_line);
#else
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 8;
    const s16 *tm = evc_tbl_tr8[DCT8][0];
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < cut_off; k++)
            {
                sum += coef[k*line] * tm[k*tr_size + j];
            }
            block[j] = EVC_CLIP3(-32768, 32767, (int)(sum + rnd_factor) >> shift);
        }
        block += tr_size;
        coef++;
    }
#endif
    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 3) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DCT8_B16(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
#if X86_SSE
    ITX_MATRIX(coef, block, 16, line, shift, evc_tbl_inv_tr16[DCT8][0], skip_line);
#else
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 16;
    const s16 *tm = evc_tbl_tr16[DCT8][0];
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < cut_off; k++)
            {
                sum += coef[k*line] * tm[k*tr_size + j];
            }
            block[j] = EVC_CLIP3(-32768, 32767, (int)(sum + rnd_factor) >> shift);
        }
        block += tr_size;
        coef++;
    }
#endif
    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 4) * sizeof(s16));
    }
}

void evc_itrans_ats_intra_DCT8_B32(s16 *coef, s16 *block, int shift, int line, int skip_line, int skip_line_2)
{
#if X86_SSE
    ITX_MATRIX(coef, block, 32, line, shift, evc_tbl_inv_tr32[DCT8][0], skip_line);
#else
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 32;
    const s16 *tm = evc_tbl_tr32[DCT8][0];
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < cut_off; k++)
            {
                sum += coef[k*line] * tm[k*tr_size + j];
            }
            block[j] = EVC_CLIP3(-32768, 32767, (int)(sum + rnd_factor) >> shift);
        }
        block += tr_size;
        coef++;
    }
#endif
    if (skip_line)
    {
        evc_mset(block, 0, (skip_line << 5) * sizeof(s16));
    }
}

void evc_it_MxN_ats_intra(s16 *coef, int tuw, int tuh, int bit_depth, const int max_log2_tr_dynamic_range, u8 ats_intra_tridx, int skip_w, int skip_h)
{
    const int TRANSFORM_MATRIX_SHIFT = 6;
    const int shift_1st = TRANSFORM_MATRIX_SHIFT + 1;
    const int shift_2nd = (TRANSFORM_MATRIX_SHIFT + max_log2_tr_dynamic_range - 1) - bit_depth;
    const u8 log2_minus1_w = CONV_LOG2(tuw) - 1;
    const u8 log2_minus1_h = CONV_LOG2(tuh) - 1;
    s16 t[MAX_TR_DIM]; /* temp buffer */
    u8  t_idx_h = 0, t_idx_v = 0;

    t_idx_h = evc_tbl_tr_subset_intra[ats_intra_tridx >> 1];
    t_idx_v = evc_tbl_tr_subset_intra[ats_intra_tridx & 1];

    evc_itrans_map_tbl[t_idx_v][log2_minus1_h](coef, t, shift_1st, tuw, skip_w, skip_h);
    evc_itrans_map_tbl[t_idx_h][log2_minus1_w](t, coef, shift_2nd, tuh, 0, skip_w);
}

static void itx_pb2(s16 *src, s16 *dst, int shift, int line)
{
    int j;
    int E, O;
    int add = shift == 0 ? 0 : 1 << (shift - 1);
    for (j = 0; j < line; j++)
    {
        /* E and O */
        E = src[0 * line + j] + src[1 * line + j];
        O = src[0 * line + j] - src[1 * line + j];

        dst[j * 2 + 0] = ITX_CLIP((evc_tbl_tm2[0][0] * E + add) >> shift);
        dst[j * 2 + 1] = ITX_CLIP((evc_tbl_tm2[1][0] * O + add) >> shift);
    }
}

static void itx_pb4(s16 *src, s16 *dst, int shift, int line)
{
    int j;
    int E[2], O[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        O[0] = evc_tbl_tm4[1][0] * src[1 * line + j] + evc_tbl_tm4[3][0] * src[3 * line + j];
        O[1] = evc_tbl_tm4[1][1] * src[1 * line + j] + evc_tbl_tm4[3][1] * src[3 * line + j];
        E[0] = evc_tbl_tm4[0][0] * src[0 * line + j] + evc_tbl_tm4[2][0] * src[2 * line + j];
        E[1] = evc_tbl_tm4[0][1] * src[0 * line + j] + evc_tbl_tm4[2][1] * src[2 * line + j];

        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        dst[j * 4 + 0] = ITX_CLIP((E[0] + O[0] + add) >> shift);
        dst[j * 4 + 1] = ITX_CLIP((E[1] + O[1] + add) >> shift);
        dst[j * 4 + 2] = ITX_CLIP((E[1] - O[1] + add) >> shift);
        dst[j * 4 + 3] = ITX_CLIP((E[0] - O[0] + add) >> shift);
    }
}

static void itx_pb8(s16 *src, s16 *dst, int shift, int line)
{
    int j, k;
    int E[4], O[4];
    int EE[2], EO[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for (k = 0; k < 4; k++)
        {
            O[k] = evc_tbl_tm8[1][k] * src[1 * line + j] + evc_tbl_tm8[3][k] * src[3 * line + j] + evc_tbl_tm8[5][k] * src[5 * line + j] + evc_tbl_tm8[7][k] * src[7 * line + j];
        }

        EO[0] = evc_tbl_tm8[2][0] * src[2 * line + j] + evc_tbl_tm8[6][0] * src[6 * line + j];
        EO[1] = evc_tbl_tm8[2][1] * src[2 * line + j] + evc_tbl_tm8[6][1] * src[6 * line + j];
        EE[0] = evc_tbl_tm8[0][0] * src[0 * line + j] + evc_tbl_tm8[4][0] * src[4 * line + j];
        EE[1] = evc_tbl_tm8[0][1] * src[0 * line + j] + evc_tbl_tm8[4][1] * src[4 * line + j];

        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        E[0] = EE[0] + EO[0];
        E[3] = EE[0] - EO[0];
        E[1] = EE[1] + EO[1];
        E[2] = EE[1] - EO[1];

        for (k = 0; k < 4; k++)
        {
            dst[j * 8 + k] = ITX_CLIP((E[k] + O[k] + add) >> shift);
            dst[j * 8 + k + 4] = ITX_CLIP((E[3 - k] - O[3 - k] + add) >> shift);
        }
    }
}

static void itx_pb16(s16 *src, s16 *dst, int shift, int line)
{
    int j, k;
    int E[8], O[8];
    int EE[4], EO[4];
    int EEE[2], EEO[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for (k = 0; k < 8; k++)
        {
            O[k] = evc_tbl_tm16[1][k] * src[1 * line + j] + evc_tbl_tm16[3][k] * src[3 * line + j] + evc_tbl_tm16[5][k] * src[5 * line + j] + evc_tbl_tm16[7][k] * src[7 * line + j] +
                evc_tbl_tm16[9][k] * src[9 * line + j] + evc_tbl_tm16[11][k] * src[11 * line + j] + evc_tbl_tm16[13][k] * src[13 * line + j] + evc_tbl_tm16[15][k] * src[15 * line + j];
        }

        for (k = 0; k < 4; k++)
        {
            EO[k] = evc_tbl_tm16[2][k] * src[2 * line + j] + evc_tbl_tm16[6][k] * src[6 * line + j] + evc_tbl_tm16[10][k] * src[10 * line + j] + evc_tbl_tm16[14][k] * src[14 * line + j];
        }

        EEO[0] = evc_tbl_tm16[4][0] * src[4 * line + j] + evc_tbl_tm16[12][0] * src[12 * line + j];
        EEE[0] = evc_tbl_tm16[0][0] * src[0 * line + j] + evc_tbl_tm16[8][0] * src[8 * line + j];
        EEO[1] = evc_tbl_tm16[4][1] * src[4 * line + j] + evc_tbl_tm16[12][1] * src[12 * line + j];
        EEE[1] = evc_tbl_tm16[0][1] * src[0 * line + j] + evc_tbl_tm16[8][1] * src[8 * line + j];

        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        for (k = 0; k < 2; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 2] = EEE[1 - k] - EEO[1 - k];
        }
        for (k = 0; k < 4; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k + 4] = EE[3 - k] - EO[3 - k];
        }
        for (k = 0; k < 8; k++)
        {
            dst[j * 16 + k] = ITX_CLIP((E[k] + O[k] + add) >> shift);
            dst[j * 16 + k + 8] = ITX_CLIP((E[7 - k] - O[7 - k] + add) >> shift);
        }
    }
}

static void itx_pb32(s16 *src, s16 *dst, int shift, int line)
{
    int j, k;
    int E[16], O[16];
    int EE[8], EO[8];
    int EEE[4], EEO[4];
    int EEEE[2], EEEO[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        for (k = 0; k < 16; k++)
        {
            O[k] = evc_tbl_tm32[1][k] * src[1 * line + j] + \
                evc_tbl_tm32[3][k] * src[3 * line + j] + \
                evc_tbl_tm32[5][k] * src[5 * line + j] + \
                evc_tbl_tm32[7][k] * src[7 * line + j] + \
                evc_tbl_tm32[9][k] * src[9 * line + j] + \
                evc_tbl_tm32[11][k] * src[11 * line + j] + \
                evc_tbl_tm32[13][k] * src[13 * line + j] + \
                evc_tbl_tm32[15][k] * src[15 * line + j] + \
                evc_tbl_tm32[17][k] * src[17 * line + j] + \
                evc_tbl_tm32[19][k] * src[19 * line + j] + \
                evc_tbl_tm32[21][k] * src[21 * line + j] + \
                evc_tbl_tm32[23][k] * src[23 * line + j] + \
                evc_tbl_tm32[25][k] * src[25 * line + j] + \
                evc_tbl_tm32[27][k] * src[27 * line + j] + \
                evc_tbl_tm32[29][k] * src[29 * line + j] + \
                evc_tbl_tm32[31][k] * src[31 * line + j];
        }

        for (k = 0; k < 8; k++)
        {
            EO[k] = evc_tbl_tm32[2][k] * src[2 * line + j] + \
                evc_tbl_tm32[6][k] * src[6 * line + j] + \
                evc_tbl_tm32[10][k] * src[10 * line + j] + \
                evc_tbl_tm32[14][k] * src[14 * line + j] + \
                evc_tbl_tm32[18][k] * src[18 * line + j] + \
                evc_tbl_tm32[22][k] * src[22 * line + j] + \
                evc_tbl_tm32[26][k] * src[26 * line + j] + \
                evc_tbl_tm32[30][k] * src[30 * line + j];
        }

        for (k = 0; k < 4; k++)
        {
            EEO[k] = evc_tbl_tm32[4][k] * src[4 * line + j] + \
                evc_tbl_tm32[12][k] * src[12 * line + j] + \
                evc_tbl_tm32[20][k] * src[20 * line + j] + \
                evc_tbl_tm32[28][k] * src[28 * line + j];
        }

        EEEO[0] = evc_tbl_tm32[8][0] * src[8 * line + j] + evc_tbl_tm32[24][0] * src[24 * line + j];
        EEEO[1] = evc_tbl_tm32[8][1] * src[8 * line + j] + evc_tbl_tm32[24][1] * src[24 * line + j];
        EEEE[0] = evc_tbl_tm32[0][0] * src[0 * line + j] + evc_tbl_tm32[16][0] * src[16 * line + j];
        EEEE[1] = evc_tbl_tm32[0][1] * src[0 * line + j] + evc_tbl_tm32[16][1] * src[16 * line + j];

        EEE[0] = EEEE[0] + EEEO[0];
        EEE[3] = EEEE[0] - EEEO[0];
        EEE[1] = EEEE[1] + EEEO[1];
        EEE[2] = EEEE[1] - EEEO[1];
        for (k = 0; k<4; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 4] = EEE[3 - k] - EEO[3 - k];
        }
        for (k = 0; k<8; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k + 8] = EE[7 - k] - EO[7 - k];
        }
        for (k = 0; k<16; k++)
        {
            dst[j * 32 + k] = ITX_CLIP((E[k] + O[k] + add) >> shift);
            dst[j * 32 + k + 16] = ITX_CLIP((E[15 - k] - O[15 - k] + add) >> shift);
        }
    }
}

static void itx_pb64(s16 *src, s16 *dst, int shift, int line)
{
    const int tx_size = 64;
    const s8 *tm = evc_tbl_tm64[0];
    int j, k;
    int E[32], O[32];
    int EE[16], EO[16];
    int EEE[8], EEO[8];
    int EEEE[4], EEEO[4];
    int EEEEE[2], EEEEO[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        for (k = 0; k < 32; k++)
        {
            O[k] = tm[1 * 64 + k] * src[line] + tm[3 * 64 + k] * src[3 * line] + tm[5 * 64 + k] * src[5 * line] + tm[7 * 64 + k] * src[7 * line] +
                tm[9 * 64 + k] * src[9 * line] + tm[11 * 64 + k] * src[11 * line] + tm[13 * 64 + k] * src[13 * line] + tm[15 * 64 + k] * src[15 * line] +
                tm[17 * 64 + k] * src[17 * line] + tm[19 * 64 + k] * src[19 * line] + tm[21 * 64 + k] * src[21 * line] + tm[23 * 64 + k] * src[23 * line] +
                tm[25 * 64 + k] * src[25 * line] + tm[27 * 64 + k] * src[27 * line] + tm[29 * 64 + k] * src[29 * line] + tm[31 * 64 + k] * src[31 * line] +
                tm[33 * 64 + k] * src[33 * line] + tm[35 * 64 + k] * src[35 * line] + tm[37 * 64 + k] * src[37 * line] + tm[39 * 64 + k] * src[39 * line] +
                tm[41 * 64 + k] * src[41 * line] + tm[43 * 64 + k] * src[43 * line] + tm[45 * 64 + k] * src[45 * line] + tm[47 * 64 + k] * src[47 * line] +
                tm[49 * 64 + k] * src[49 * line] + tm[51 * 64 + k] * src[51 * line] + tm[53 * 64 + k] * src[53 * line] + tm[55 * 64 + k] * src[55 * line] +
                tm[57 * 64 + k] * src[57 * line] + tm[59 * 64 + k] * src[59 * line] + tm[61 * 64 + k] * src[61 * line] + tm[63 * 64 + k] * src[63 * line];
        }

        for (k = 0; k < 16; k++)
        {
            EO[k] = tm[2 * 64 + k] * src[2 * line] + tm[6 * 64 + k] * src[6 * line] + tm[10 * 64 + k] * src[10 * line] + tm[14 * 64 + k] * src[14 * line] +
                tm[18 * 64 + k] * src[18 * line] + tm[22 * 64 + k] * src[22 * line] + tm[26 * 64 + k] * src[26 * line] + tm[30 * 64 + k] * src[30 * line] +
                tm[34 * 64 + k] * src[34 * line] + tm[38 * 64 + k] * src[38 * line] + tm[42 * 64 + k] * src[42 * line] + tm[46 * 64 + k] * src[46 * line] +
                tm[50 * 64 + k] * src[50 * line] + tm[54 * 64 + k] * src[54 * line] + tm[58 * 64 + k] * src[58 * line] + tm[62 * 64 + k] * src[62 * line];
        }

        for (k = 0; k < 8; k++)
        {
            EEO[k] = tm[4 * 64 + k] * src[4 * line] + tm[12 * 64 + k] * src[12 * line] + tm[20 * 64 + k] * src[20 * line] + tm[28 * 64 + k] * src[28 * line] +
                tm[36 * 64 + k] * src[36 * line] + tm[44 * 64 + k] * src[44 * line] + tm[52 * 64 + k] * src[52 * line] + tm[60 * 64 + k] * src[60 * line];
        }

        for (k = 0; k<4; k++)
        {
            EEEO[k] = tm[8 * 64 + k] * src[8 * line] + tm[24 * 64 + k] * src[24 * line] + tm[40 * 64 + k] * src[40 * line] + tm[56 * 64 + k] * src[56 * line];
        }
        EEEEO[0] = tm[16 * 64 + 0] * src[16 * line] + tm[48 * 64 + 0] * src[48 * line];
        EEEEO[1] = tm[16 * 64 + 1] * src[16 * line] + tm[48 * 64 + 1] * src[48 * line];
        EEEEE[0] = tm[0 * 64 + 0] * src[0] + tm[32 * 64 + 0] * src[32 * line];
        EEEEE[1] = tm[0 * 64 + 1] * src[0] + tm[32 * 64 + 1] * src[32 * line];

        for (k = 0; k < 2; k++)
        {
            EEEE[k] = EEEEE[k] + EEEEO[k];
            EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];
        }
        for (k = 0; k < 4; k++)
        {
            EEE[k] = EEEE[k] + EEEO[k];
            EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
        }
        for (k = 0; k < 8; k++)
        {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 8] = EEE[7 - k] - EEO[7 - k];
        }
        for (k = 0; k < 16; k++)
        {
            E[k] = EE[k] + EO[k];
            E[k + 16] = EE[15 - k] - EO[15 - k];
        }
        for (k = 0; k < 32; k++)
        {
            dst[k] = ITX_CLIP((E[k] + O[k] + add) >> shift);
            dst[k + 32] = ITX_CLIP((E[31 - k] - O[31 - k] + add) >> shift);
        }
        src++;
        dst += tx_size;
    }
}

typedef void(*EVC_ITXB)(void *coef, void *t, int shift, int line, int step);
static EVC_ITXB tbl_itxb[MAX_TR_LOG2] =
{
    itx_pb2b,
    itx_pb4b,
    itx_pb8b,
    itx_pb16b,
    itx_pb32b,
    itx_pb64b
};

typedef void(*EVC_ITX)(s16 *coef, s16 *t, int shift, int line);

static EVC_ITX tbl_itx[MAX_TR_LOG2] =
{
    itx_pb2,
    itx_pb4,
    itx_pb8,
    itx_pb16,
    itx_pb32,
    itx_pb64
};

void evc_itrans(s16 *coef, int log2_cuw, int log2_cuh, int iqt_flag)
{
    if(iqt_flag)
    {
        s16 t[MAX_TR_DIM]; /* temp buffer */
        tbl_itx[log2_cuh - 1](coef, t, ITX_SHIFT1, 1 << log2_cuw);
        tbl_itx[log2_cuw - 1](t, coef, ITX_SHIFT2, 1 << log2_cuh);
    }
    else
    {
        s32 tb[MAX_TR_DIM]; /* temp buffer */
        tbl_itxb[log2_cuh - 1](coef, tb, 0, 1 << log2_cuw, 0);
        tbl_itxb[log2_cuw - 1](tb, coef, (ITX_SHIFT1 + ITX_SHIFT2), 1 << log2_cuh, 1);
    }
}


void evc_itrans_ats_intra(s16* coef, int log2_w, int log2_h, u8 ats_tu, int skip_w, int skip_h)
{
    evc_it_MxN_ats_intra(coef, (1 << log2_w), (1 << log2_h), BIT_DEPTH, 15, ats_tu, skip_w, skip_h);
}

static void evc_dquant(s16 *coef, int log2_w, int log2_h, int scale, s32 offset, u8 shift)
{
    int i;
    s64 lev;

    const int ns_scale = ((log2_w + log2_h) & 1) ? 181 : 1;
    for(i = 0; i < (1 << (log2_w + log2_h)); i++)
    {
        lev = (coef[i] * (scale * (s64)ns_scale) + offset) >> shift;
        coef[i] = (s16)EVC_CLIP(lev, -32768, 32767);
    }
}

void evc_itdq(s16 *coef, int log2_w, int log2_h, int scale, int iqt_flag, u8 ats_intra_cu, u8 ats_tu)
{
    s32 offset;
    u8 shift;
    s8 tr_shift;
    int log2_size = (log2_w + log2_h) >> 1;
    const int ns_shift = ((log2_w + log2_h) & 1) ? 8 : 0;

    int skip_w = 1 << log2_w;
    int skip_h = 1 << log2_h;
    int max_x = 0;
    int max_y = 0;
    s16* coef_tmp = coef;
    int i, j;
    int cuw = 1 << log2_w;
    int cuh = 1 << log2_h;

    tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - log2_size;
    shift = QUANT_IQUANT_SHIFT - QUANT_SHIFT - tr_shift;
    shift += ns_shift;
    offset = (shift == 0) ? 0 : (1 << (shift - 1));

    evc_dquant(coef, log2_w, log2_h, scale, offset, shift);
    
    for(j = 0; j < cuh; j++)
    {
        for(i = 0; i < cuw; i++)
        {
            if(coef_tmp[i] != 0)
            {
                if(i > max_x)
                {
                    max_x = i;
                }
                if(j > max_y)
                {
                    max_y = j;
                }
            }
        }
        coef_tmp += cuw;
    }

    skip_w = cuw - 1 - max_x;
    skip_h = cuh - 1 - max_y;

    if(ats_intra_cu)
    {
        evc_itrans_ats_intra(coef, log2_w, log2_h, ats_tu, skip_w, skip_h);
    }
    else
    {
        evc_itrans(coef, log2_w, log2_h, iqt_flag);
    }
}

void evc_sub_block_itdq(s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 qp_y, u8 qp_u, u8 qp_v, int flag[N_C], int nnz_sub[N_C][MAX_SUB_TB_NUM], int iqt_flag
                        , u8 ats_intra_cu, u8 ats_tu, u8 ats_inter_info)
{
    s16 *coef_temp[N_C];
    s16 coef_temp_buf[N_C][MAX_TR_DIM];
    int i, j, c;
    int log2_w_sub = (log2_cuw > MAX_TR_LOG2) ? MAX_TR_LOG2 : log2_cuw;
    int log2_h_sub = (log2_cuh > MAX_TR_LOG2) ? MAX_TR_LOG2 : log2_cuh;
    int loop_w = (log2_cuw > MAX_TR_LOG2) ? (1 << (log2_cuw - MAX_TR_LOG2)) : 1;
    int loop_h = (log2_cuh > MAX_TR_LOG2) ? (1 << (log2_cuh - MAX_TR_LOG2)) : 1;
    int stride = (1 << log2_cuw);
    int sub_stride = (1 << log2_w_sub);
    u8 qp[N_C] = { qp_y, qp_u, qp_v };
    int scale = 0;
    u8 ats_intra_cu_on = 0;
    u8 ats_tu_mode = 0;

    if (ats_inter_info)
    {
        get_tu_size(ats_inter_info, log2_cuw, log2_cuh, &log2_w_sub, &log2_h_sub);
        sub_stride = (1 << log2_w_sub);
    }

    for(j = 0; j < loop_h; j++)
    {
        for(i = 0; i < loop_w; i++)
        {
            for(c = 0; c < N_C; c++)
            {
                ats_intra_cu_on = (c == 0)? ats_intra_cu : 0;
                ats_tu_mode = (c == 0) ? ats_tu : 0;

                if (c == 0)
                {
                    get_ats_inter_trs(ats_inter_info, log2_cuw, log2_cuh, &ats_intra_cu_on, &ats_tu_mode);
                }

                if(nnz_sub[c][(j << 1) | i])
                {
                    int pos_sub_x = i * (1 << (log2_w_sub - !!c));
                    int pos_sub_y = j * (1 << (log2_h_sub - !!c)) * (stride >> (!!c));

                    if(loop_h + loop_w > 2)
                    {
                        evc_block_copy(coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), coef_temp_buf[c], sub_stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
                        coef_temp[c] = coef_temp_buf[c];
                    }
                    else
                    {
                        coef_temp[c] = coef[c];
                    }

                    if(iqt_flag)
                    {
                        scale = evc_tbl_dq_scale[qp[c] % 6] << (qp[c] / 6);
                    }
                    else
                    {
                        scale = evc_tbl_dq_scale_b[qp[c] % 6] << (qp[c] / 6);
                    }

                    evc_itdq(coef_temp[c], log2_w_sub - !!c, log2_h_sub - !!c, scale, iqt_flag, ats_intra_cu_on, ats_tu_mode);

                    if(loop_h + loop_w > 2)
                    {
                        evc_block_copy(coef_temp_buf[c], sub_stride >> (!!c), coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
                    }
                }
            }
        }

    }
}
