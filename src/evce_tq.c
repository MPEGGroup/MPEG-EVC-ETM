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


#define QUANT(c, scale, offset, shift) ((s16)((((c)*(scale)) + (offset)) >> (shift)))

static s64 err_scale_tbl[6][MAX_CU_DEPTH];

extern int rdoq_est_all_cbf[2];
extern int rdoq_est_cbf[NUM_QT_CBF_CTX][2];
extern int rdoq_est_gt0[NUM_CTX_GT0][2];
extern int rdoq_est_gtA[NUM_CTX_GTA][2];
extern int rdoq_est_scanr_x[NUM_CTX_SCANR][2];
extern int rdoq_est_scanr_y[NUM_CTX_SCANR][2];
extern s32 rdoq_est_run[NUM_SBAC_CTX_RUN][2];
extern s32 rdoq_est_level[NUM_SBAC_CTX_LEVEL][2];
extern s32 rdoq_est_last[NUM_SBAC_CTX_LAST][2];

const int quant_scale[6] = {26214, 23302, 20560, 18396, 16384, 14564};

void evce_trans_DST7_B4(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DST7_B8(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DST7_B16(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DST7_B32(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DCT8_B4(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DCT8_B8(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DCT8_B16(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
void evce_trans_DCT8_B32(s16* block, s16* coeff, s32 shift, s32 line, int skip_line, int skip_line_2);
typedef void Trans(s16*, s16*, s32, s32, int, int);

Trans* evce_trans_map_tbl[16][5] =
{
    { NULL, evce_trans_DCT8_B4, evce_trans_DCT8_B8, evce_trans_DCT8_B16, evce_trans_DCT8_B32 },
    { NULL, evce_trans_DST7_B4, evce_trans_DST7_B8, evce_trans_DST7_B16, evce_trans_DST7_B32 },
};

static void tx_pb2b(void * src, void * dst, int shift, int line, int step)
{
    int j;
    s64 E, O;
    int add = shift == 0 ? 0 : 1 << (shift - 1);

#define RUN_TX_PB2(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* E and O */\
        E = *((type_src * )src + j * 2 + 0) + *((type_src *)src + j * 2 + 1);\
        O = *((type_src * )src + j * 2 + 0) - *((type_src *)src + j * 2 + 1);\
        \
        *((type_dst * )dst + 0 * line + j) = (type_dst)((evc_tbl_tm2[0][0] * E + add) >> shift);\
        *((type_dst * )dst + 1 * line + j) = (type_dst)((evc_tbl_tm2[1][0] * O + add) >> shift);\
    }

    if(step == 0)
    {
        RUN_TX_PB2(src, dst, s16, s32);
    }
    else
    {
        RUN_TX_PB2(src, dst, s32, s16);
    }
}

static void tx_pb4b(void * src, void * dst, int shift, int line, int step)
{
    int j;
    s64 E[2], O[2];
    int add = 1 << (shift - 1);

#define RUN_TX_PB4(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* E and O */\
        E[0] = *((type_src * )src + j * 4 + 0) + *((type_src * )src + j * 4 + 3);\
        O[0] = *((type_src * )src + j * 4 + 0) - *((type_src * )src + j * 4 + 3);\
        E[1] = *((type_src * )src + j * 4 + 1) + *((type_src * )src + j * 4 + 2);\
        O[1] = *((type_src * )src + j * 4 + 1) - *((type_src * )src + j * 4 + 2);\
        \
        *((type_dst * )dst + 0 * line + j) = (type_dst)((evc_tbl_tm4[0][0] * E[0] + evc_tbl_tm4[0][1] * E[1] + add) >> shift);\
        *((type_dst * )dst + 2 * line + j) = (type_dst)((evc_tbl_tm4[2][0] * E[0] + evc_tbl_tm4[2][1] * E[1] + add) >> shift);\
        *((type_dst * )dst + 1 * line + j) = (type_dst)((evc_tbl_tm4[1][0] * O[0] + evc_tbl_tm4[1][1] * O[1] + add) >> shift);\
        *((type_dst * )dst + 3 * line + j) = (type_dst)((evc_tbl_tm4[3][0] * O[0] + evc_tbl_tm4[3][1] * O[1] + add) >> shift);\
    }

    if(step == 0)
    {
        RUN_TX_PB4(src, dst, s16, s32);
    }
    else
    {
        RUN_TX_PB4(src, dst, s32, s16);
    }
}

static void tx_pb8b(void * src, void * dst, int shift, int line, int step)
{
    int j, k;
    s64 E[4], O[4];
    s64 EE[2], EO[2];
    int add = 1 << (shift - 1);

#define RUN_TX_PB8(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* E and O*/\
        for (k = 0; k < 4; k++)\
        {\
            E[k] = *((type_src * )src + j * 8 + k) + *((type_src * )src + j * 8 + 7 - k);\
            O[k] = *((type_src * )src + j * 8 + k) - *((type_src * )src + j * 8 + 7 - k);\
        }\
        /* EE and EO */\
        EE[0] = E[0] + E[3];\
        EO[0] = E[0] - E[3];\
        EE[1] = E[1] + E[2];\
        EO[1] = E[1] - E[2];\
        \
        *((type_dst * )dst + 0 * line + j) = (type_dst)((evc_tbl_tm8[0][0] * EE[0] + evc_tbl_tm8[0][1] * EE[1] + add) >> shift);\
        *((type_dst * )dst + 4 * line + j) = (type_dst)((evc_tbl_tm8[4][0] * EE[0] + evc_tbl_tm8[4][1] * EE[1] + add) >> shift);\
        *((type_dst * )dst + 2 * line + j) = (type_dst)((evc_tbl_tm8[2][0] * EO[0] + evc_tbl_tm8[2][1] * EO[1] + add) >> shift);\
        *((type_dst * )dst + 6 * line + j) = (type_dst)((evc_tbl_tm8[6][0] * EO[0] + evc_tbl_tm8[6][1] * EO[1] + add) >> shift);\
        \
        *((type_dst * )dst + 1 * line + j) = (type_dst)((evc_tbl_tm8[1][0] * O[0] + evc_tbl_tm8[1][1] * O[1] + evc_tbl_tm8[1][2] * O[2] + evc_tbl_tm8[1][3] * O[3] + add) >> shift);\
        *((type_dst * )dst + 3 * line + j) = (type_dst)((evc_tbl_tm8[3][0] * O[0] + evc_tbl_tm8[3][1] * O[1] + evc_tbl_tm8[3][2] * O[2] + evc_tbl_tm8[3][3] * O[3] + add) >> shift);\
        *((type_dst * )dst + 5 * line + j) = (type_dst)((evc_tbl_tm8[5][0] * O[0] + evc_tbl_tm8[5][1] * O[1] + evc_tbl_tm8[5][2] * O[2] + evc_tbl_tm8[5][3] * O[3] + add) >> shift);\
        *((type_dst * )dst + 7 * line + j) = (type_dst)((evc_tbl_tm8[7][0] * O[0] + evc_tbl_tm8[7][1] * O[1] + evc_tbl_tm8[7][2] * O[2] + evc_tbl_tm8[7][3] * O[3] + add) >> shift);\
    }

    if(step == 0)
    {
        RUN_TX_PB8(src, dst, s16, s32);
    }
    else
    {
        RUN_TX_PB8(src, dst, s32, s16);
    }
}

static void tx_pb16b(void * src, void * dst, int shift, int line, int step)
{
    int j, k;
    s64 E[8], O[8];
    s64 EE[4], EO[4];
    s64 EEE[2], EEO[2];
    int add = 1 << (shift - 1);

#define RUN_TX_PB16(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* E and O*/\
        for (k = 0; k < 8; k++)\
        {\
            E[k] = *((type_src * )src + j * 16 + k) + *((type_src * )src + j * 16 + 15 - k);\
            O[k] = *((type_src * )src + j * 16 + k) - *((type_src * )src + j * 16 + 15 - k);\
        }\
        /* EE and EO */\
        for (k = 0; k < 4; k++)\
        {\
            EE[k] = E[k] + E[7 - k];\
            EO[k] = E[k] - E[7 - k];\
        }\
        /* EEE and EEO */\
        EEE[0] = EE[0] + EE[3];\
        EEO[0] = EE[0] - EE[3];\
        EEE[1] = EE[1] + EE[2];\
        EEO[1] = EE[1] - EE[2];\
        \
        *((type_dst * )dst + 0  * line + j) = (type_dst)((evc_tbl_tm16[0][0]  * EEE[0] + evc_tbl_tm16[0][1]  * EEE[1] + add) >> shift);\
        *((type_dst * )dst + 8  * line + j) = (type_dst)((evc_tbl_tm16[8][0]  * EEE[0] + evc_tbl_tm16[8][1]  * EEE[1] + add) >> shift);\
        *((type_dst * )dst + 4  * line + j) = (type_dst)((evc_tbl_tm16[4][0]  * EEO[0] + evc_tbl_tm16[4][1]  * EEO[1] + add) >> shift);\
        *((type_dst * )dst + 12 * line + j) = (type_dst)((evc_tbl_tm16[12][0] * EEO[0] + evc_tbl_tm16[12][1] * EEO[1] + add) >> shift);\
        \
        for (k = 2; k < 16; k += 4)\
        {\
            *((type_dst * )dst + k * line + j) = (type_dst)((evc_tbl_tm16[k][0] * EO[0] + evc_tbl_tm16[k][1] * EO[1] + evc_tbl_tm16[k][2] * EO[2] + evc_tbl_tm16[k][3] * EO[3] + add) >> shift);\
        }\
        \
        for (k = 1; k < 16; k += 2)\
        {\
            *((type_dst * )dst + k * line + j) = (type_dst)((evc_tbl_tm16[k][0] * O[0] + evc_tbl_tm16[k][1] * O[1] + evc_tbl_tm16[k][2] * O[2] + evc_tbl_tm16[k][3] * O[3] + \
                evc_tbl_tm16[k][4] * O[4] + evc_tbl_tm16[k][5] * O[5] + evc_tbl_tm16[k][6] * O[6] + evc_tbl_tm16[k][7] * O[7] + add) >> shift);\
        }\
    }

    if(step == 0)
    {
        RUN_TX_PB16(src, dst, s16, s32);
    }
    else
    {
        RUN_TX_PB16(src, dst, s32, s16);
    }
}

static void tx_pb32b(void * src, void * dst, int shift, int line, int step)
{
    int j, k;
    s64 E[16], O[16];
    s64 EE[8], EO[8];
    s64 EEE[4], EEO[4];
    s64 EEEE[2], EEEO[2];
    int add = 1 << (shift - 1);

#define RUN_TX_PB32(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        /* E and O*/\
        for (k = 0; k < 16; k++)\
        {\
            E[k] = *((type_src *)src + j * 32 + k) + *((type_src *)src + j * 32 + 31 - k);\
            O[k] = *((type_src *)src + j * 32 + k) - *((type_src *)src + j * 32 + 31 - k);\
        }\
        /* EE and EO */\
        for (k = 0; k < 8; k++)\
        {\
            EE[k] = E[k] + E[15 - k];\
            EO[k] = E[k] - E[15 - k];\
        }\
        /* EEE and EEO */\
        for (k = 0; k < 4; k++)\
        {\
            EEE[k] = EE[k] + EE[7 - k];\
            EEO[k] = EE[k] - EE[7 - k];\
        }\
        /* EEEE and EEEO */\
        EEEE[0] = EEE[0] + EEE[3];\
        EEEO[0] = EEE[0] - EEE[3];\
        EEEE[1] = EEE[1] + EEE[2];\
        EEEO[1] = EEE[1] - EEE[2];\
        \
        *((type_dst *)dst + 0  * line + j) = (type_dst)((evc_tbl_tm32[0][0]  * EEEE[0] + evc_tbl_tm32[0][1]  * EEEE[1] + add) >> shift);\
        *((type_dst *)dst + 16 * line + j) = (type_dst)((evc_tbl_tm32[16][0] * EEEE[0] + evc_tbl_tm32[16][1] * EEEE[1] + add) >> shift);\
        *((type_dst *)dst + 8  * line + j) = (type_dst)((evc_tbl_tm32[8][0]  * EEEO[0] + evc_tbl_tm32[8][1]  * EEEO[1] + add) >> shift);\
        *((type_dst *)dst + 24 * line + j) = (type_dst)((evc_tbl_tm32[24][0] * EEEO[0] + evc_tbl_tm32[24][1] * EEEO[1] + add) >> shift);\
        for (k = 4; k < 32; k += 8)\
        {\
            *((type_dst *)dst + k*line + j) = (type_dst)((evc_tbl_tm32[k][0] * EEO[0] + evc_tbl_tm32[k][1] * EEO[1] + evc_tbl_tm32[k][2] * EEO[2] + evc_tbl_tm32[k][3] * EEO[3] + add) >> shift);\
        }\
        for (k = 2; k < 32; k += 4)\
        {\
            *((type_dst *)dst + k*line + j) = (type_dst)((evc_tbl_tm32[k][0] * EO[0] + evc_tbl_tm32[k][1] * EO[1] + evc_tbl_tm32[k][2] * EO[2] + evc_tbl_tm32[k][3] * EO[3] +\
                evc_tbl_tm32[k][4] * EO[4] + evc_tbl_tm32[k][5] * EO[5] + evc_tbl_tm32[k][6] * EO[6] + evc_tbl_tm32[k][7] * EO[7] + add) >> shift);\
        }\
        for (k = 1; k < 32; k += 2)\
        {\
            *((type_dst *)dst + k*line + j) = (type_dst)((evc_tbl_tm32[k][0] * O[0] + evc_tbl_tm32[k][1] * O[1] + evc_tbl_tm32[k][2] * O[2] + evc_tbl_tm32[k][3] * O[3] +\
                evc_tbl_tm32[k][4] * O[4] + evc_tbl_tm32[k][5] * O[5] + evc_tbl_tm32[k][6] * O[6] + evc_tbl_tm32[k][7] * O[7] +\
                evc_tbl_tm32[k][8] * O[8] + evc_tbl_tm32[k][9] * O[9] + evc_tbl_tm32[k][10] * O[10] + evc_tbl_tm32[k][11] * O[11] +\
                evc_tbl_tm32[k][12] * O[12] + evc_tbl_tm32[k][13] * O[13] + evc_tbl_tm32[k][14] * O[14] + evc_tbl_tm32[k][15] * O[15] + add) >> shift);\
        }\
    }

    if(step == 0)
    {
        RUN_TX_PB32(src, dst, s16, s32);
    }
    else
    {
        RUN_TX_PB32(src, dst, s32, s16);
    }
}

static void tx_pb64b(void *src, void *dst, int shift, int line, int step)
{
    const int tx_size = 64;
    const s8 * tm = evc_tbl_tm64[0];
    int j, k;
    s64 E[32], O[32];
    s64 EE[16], EO[16];
    s64 EEE[8], EEO[8];
    s64 EEEE[4], EEEO[4];
    s64 EEEEE[2], EEEEO[2];
    int add = 1 << (shift - 1);

#define RUN_TX_PB64(src, dst, type_src, type_dst) \
    for (j = 0; j < line; j++)\
    {\
        for (k = 0; k < 32; k++)\
        {\
            E[k] =  *((type_src *)src + k) + *((type_src *)src + 63 - k);\
            O[k] =  *((type_src *)src + k) - *((type_src *)src + 63 - k);\
        }\
        for (k = 0; k<16; k++)\
        {\
            EE[k] = E[k] + E[31 - k];\
            EO[k] = E[k] - E[31 - k];\
        }\
        for (k = 0; k<8; k++)\
        {\
            EEE[k] = EE[k] + EE[15 - k];\
            EEO[k] = EE[k] - EE[15 - k];\
        }\
        for (k = 0; k<4; k++)\
        {\
            EEEE[k] = EEE[k] + EEE[7 - k];\
            EEEO[k] = EEE[k] - EEE[7 - k];\
        }\
        EEEEE[0] = EEEE[0] + EEEE[3];\
        EEEEO[0] = EEEE[0] - EEEE[3];\
        EEEEE[1] = EEEE[1] + EEEE[2];\
        EEEEO[1] = EEEE[1] - EEEE[2];\
        \
        *((type_dst *)dst + 0        ) = (type_dst)((tm[0 * 64 + 0]  * EEEEE[0] + tm[0 * 64 + 1]  * EEEEE[1] + add) >> shift);\
        *((type_dst *)dst + 16 * line) = (type_dst)((tm[16 * 64 + 0] * EEEEO[0] + tm[16 * 64 + 1] * EEEEO[1] + add) >> shift);\
        *((type_dst *)dst + 32 * line) = 0;\
        *((type_dst *)dst + 48 * line) = 0;\
        \
        for (k = 8; k<64; k += 16)\
        {\
            if(k > 31)\
            {\
                *((type_dst *)dst + k*line) = 0;\
            }\
            else\
            {\
                *((type_dst *)dst + k*line) = (type_dst)((tm[k * 64 + 0] * EEEO[0] + tm[k * 64 + 1] * EEEO[1] + tm[k * 64 + 2] * EEEO[2] + tm[k * 64 + 3] * EEEO[3] + add) >> shift);\
            }\
        }\
        for (k = 4; k<64; k += 8)\
        {\
            if(k > 31)\
            {\
                *((type_dst *)dst + k*line) = 0;\
            }\
            else\
            {\
                *((type_dst *)dst + k * line) = (type_dst)((tm[k * 64 + 0] * EEO[0] + tm[k * 64 + 1] * EEO[1] + tm[k * 64 + 2] * EEO[2] + tm[k * 64 + 3] * EEO[3] + \
                    tm[k * 64 + 4] * EEO[4] + tm[k * 64 + 5] * EEO[5] + tm[k * 64 + 6] * EEO[6] + tm[k * 64 + 7] * EEO[7] + add) >> shift); \
            }\
        }\
        for (k = 2; k<64; k += 4)\
        {\
            if (k > 31)\
            {\
                *((type_dst *)dst + k * line) = 0; \
            }\
            else\
            {\
                *((type_dst *)dst + k * line) = (type_dst)((tm[k * 64 + 0] * EO[0] + tm[k * 64 + 1] * EO[1] + tm[k * 64 + 2] * EO[2] + tm[k * 64 + 3] * EO[3] + \
                    tm[k * 64 + 4] * EO[4] + tm[k * 64 + 5] * EO[5] + tm[k * 64 + 6] * EO[6] + tm[k * 64 + 7] * EO[7] + \
                    tm[k * 64 + 8] * EO[8] + tm[k * 64 + 9] * EO[9] + tm[k * 64 + 10] * EO[10] + tm[k * 64 + 11] * EO[11] + \
                    tm[k * 64 + 12] * EO[12] + tm[k * 64 + 13] * EO[13] + tm[k * 64 + 14] * EO[14] + tm[k * 64 + 15] * EO[15] + add) >> shift); \
            }\
        }\
        for (k = 1; k<64; k += 2)\
        {\
            if (k > 31)\
            {\
                *((type_dst *)dst + k * line) = 0; \
            }\
            else\
            {\
                *((type_dst *)dst + k * line) = (type_dst)((tm[k * 64 + 0] * O[0] + tm[k * 64 + 1] * O[1] + tm[k * 64 + 2] * O[2] + tm[k * 64 + 3] * O[3] + \
                    tm[k * 64 + 4] * O[4] + tm[k * 64 + 5] * O[5] + tm[k * 64 + 6] * O[6] + tm[k * 64 + 7] * O[7] + \
                    tm[k * 64 + 8] * O[8] + tm[k * 64 + 9] * O[9] + tm[k * 64 + 10] * O[10] + tm[k * 64 + 11] * O[11] + \
                    tm[k * 64 + 12] * O[12] + tm[k * 64 + 13] * O[13] + tm[k * 64 + 14] * O[14] + tm[k * 64 + 15] * O[15] + \
                    tm[k * 64 + 16] * O[16] + tm[k * 64 + 17] * O[17] + tm[k * 64 + 18] * O[18] + tm[k * 64 + 19] * O[19] + \
                    tm[k * 64 + 20] * O[20] + tm[k * 64 + 21] * O[21] + tm[k * 64 + 22] * O[22] + tm[k * 64 + 23] * O[23] + \
                    tm[k * 64 + 24] * O[24] + tm[k * 64 + 25] * O[25] + tm[k * 64 + 26] * O[26] + tm[k * 64 + 27] * O[27] + \
                    tm[k * 64 + 28] * O[28] + tm[k * 64 + 29] * O[29] + tm[k * 64 + 30] * O[30] + tm[k * 64 + 31] * O[31] + add) >> shift); \
            }\
        }\
        src = (type_src *)src + tx_size;\
        dst = (type_dst *)dst + 1;\
    }

    if(step == 0)
    {
        RUN_TX_PB64(src, dst, s16, s32);
    }
    else
    {
        RUN_TX_PB64(src, dst, s32, s16);
    }
}

static void tx_pb2(s16 * src, s16 * dst, int shift, int line)
{
    int j;
    int E, O;
    int add = shift == 0 ? 0 : 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        /* E and O */
        E = src[j * 2 + 0] + src[j * 2 + 1];
        O = src[j * 2 + 0] - src[j * 2 + 1];

        dst[0 * line + j] = (evc_tbl_tm2[0][0] * E + add) >> shift;
        dst[1 * line + j] = (evc_tbl_tm2[1][0] * O + add) >> shift;
    }
}

static void tx_pb4(s16 * src, s16 * dst, int shift, int line)
{
    int j;
    int E[2], O[2];
    int add = 1 << (shift - 1);
    for (j = 0; j < line; j++)
    {
        /* E and O */
        E[0] = src[j * 4 + 0] + src[j * 4 + 3];
        O[0] = src[j * 4 + 0] - src[j * 4 + 3];
        E[1] = src[j * 4 + 1] + src[j * 4 + 2];
        O[1] = src[j * 4 + 1] - src[j * 4 + 2];

        dst[0 * line + j] = (evc_tbl_tm4[0][0] * E[0] + evc_tbl_tm4[0][1] * E[1] + add) >> shift;
        dst[2 * line + j] = (evc_tbl_tm4[2][0] * E[0] + evc_tbl_tm4[2][1] * E[1] + add) >> shift;
        dst[1 * line + j] = (evc_tbl_tm4[1][0] * O[0] + evc_tbl_tm4[1][1] * O[1] + add) >> shift;
        dst[3 * line + j] = (evc_tbl_tm4[3][0] * O[0] + evc_tbl_tm4[3][1] * O[1] + add) >> shift;
    }
}

static void tx_pb8(s16 * src, s16 * dst, int shift, int line)
{
    int j, k;
    int E[4], O[4];
    int EE[2], EO[2];
    int add = 1 << (shift - 1);
    for (j = 0; j < line; j++)
    {
        /* E and O*/
        for (k = 0; k < 4; k++)
        {
            E[k] = src[j * 8 + k] + src[j * 8 + 7 - k];
            O[k] = src[j * 8 + k] - src[j * 8 + 7 - k];
        }
        /* EE and EO */
        EE[0] = E[0] + E[3];
        EO[0] = E[0] - E[3];
        EE[1] = E[1] + E[2];
        EO[1] = E[1] - E[2];

        dst[0 * line + j] = (evc_tbl_tm8[0][0] * EE[0] + evc_tbl_tm8[0][1] * EE[1] + add) >> shift;
        dst[4 * line + j] = (evc_tbl_tm8[4][0] * EE[0] + evc_tbl_tm8[4][1] * EE[1] + add) >> shift;
        dst[2 * line + j] = (evc_tbl_tm8[2][0] * EO[0] + evc_tbl_tm8[2][1] * EO[1] + add) >> shift;
        dst[6 * line + j] = (evc_tbl_tm8[6][0] * EO[0] + evc_tbl_tm8[6][1] * EO[1] + add) >> shift;

        dst[1 * line + j] = (evc_tbl_tm8[1][0] * O[0] + evc_tbl_tm8[1][1] * O[1] + evc_tbl_tm8[1][2] * O[2] + evc_tbl_tm8[1][3] * O[3] + add) >> shift;
        dst[3 * line + j] = (evc_tbl_tm8[3][0] * O[0] + evc_tbl_tm8[3][1] * O[1] + evc_tbl_tm8[3][2] * O[2] + evc_tbl_tm8[3][3] * O[3] + add) >> shift;
        dst[5 * line + j] = (evc_tbl_tm8[5][0] * O[0] + evc_tbl_tm8[5][1] * O[1] + evc_tbl_tm8[5][2] * O[2] + evc_tbl_tm8[5][3] * O[3] + add) >> shift;
        dst[7 * line + j] = (evc_tbl_tm8[7][0] * O[0] + evc_tbl_tm8[7][1] * O[1] + evc_tbl_tm8[7][2] * O[2] + evc_tbl_tm8[7][3] * O[3] + add) >> shift;
    }
}

static void tx_pb16(s16 * src, s16 * dst, int shift, int line)
{
    int j, k;
    int E[8], O[8];
    int EE[4], EO[4];
    int EEE[2], EEO[2];
    int add = 1 << (shift - 1);

    for (j = 0; j < line; j++)
    {
        /* E and O*/
        for (k = 0; k < 8; k++)
        {
            E[k] = src[j * 16 + k] + src[j * 16 + 15 - k];
            O[k] = src[j * 16 + k] - src[j * 16 + 15 - k];
        }
        /* EE and EO */
        for (k = 0; k < 4; k++)
        {
            EE[k] = E[k] + E[7 - k];
            EO[k] = E[k] - E[7 - k];
        }
        /* EEE and EEO */
        EEE[0] = EE[0] + EE[3];
        EEO[0] = EE[0] - EE[3];
        EEE[1] = EE[1] + EE[2];
        EEO[1] = EE[1] - EE[2];

        dst[0 * line + j] = (evc_tbl_tm16[0][0] * EEE[0] + evc_tbl_tm16[0][1] * EEE[1] + add) >> shift;
        dst[8 * line + j] = (evc_tbl_tm16[8][0] * EEE[0] + evc_tbl_tm16[8][1] * EEE[1] + add) >> shift;
        dst[4 * line + j] = (evc_tbl_tm16[4][0] * EEO[0] + evc_tbl_tm16[4][1] * EEO[1] + add) >> shift;
        dst[12 * line + j] = (evc_tbl_tm16[12][0] * EEO[0] + evc_tbl_tm16[12][1] * EEO[1] + add) >> shift;

        for (k = 2; k < 16; k += 4)
        {
            dst[k*line + j] = (evc_tbl_tm16[k][0] * EO[0] + evc_tbl_tm16[k][1] * EO[1] + evc_tbl_tm16[k][2] * EO[2] + evc_tbl_tm16[k][3] * EO[3] + add) >> shift;
        }

        for (k = 1; k < 16; k += 2)
        {
            dst[k*line + j] = (evc_tbl_tm16[k][0] * O[0] + evc_tbl_tm16[k][1] * O[1] + evc_tbl_tm16[k][2] * O[2] + evc_tbl_tm16[k][3] * O[3] +
                evc_tbl_tm16[k][4] * O[4] + evc_tbl_tm16[k][5] * O[5] + evc_tbl_tm16[k][6] * O[6] + evc_tbl_tm16[k][7] * O[7] + add) >> shift;
        }
    }
}

static void tx_pb32(s16 * src, s16 * dst, int shift, int line)
{
    int j, k;
    int E[16], O[16];
    int EE[8], EO[8];
    int EEE[4], EEO[4];
    int EEEE[2], EEEO[2];
    int add = 1 << (shift - 1);
    for (j = 0; j < line; j++)
    {
        /* E and O*/
        for (k = 0; k < 16; k++)
        {
            E[k] = src[j * 32 + k] + src[j * 32 + 31 - k];
            O[k] = src[j * 32 + k] - src[j * 32 + 31 - k];
        }
        /* EE and EO */
        for (k = 0; k < 8; k++)
        {
            EE[k] = E[k] + E[15 - k];
            EO[k] = E[k] - E[15 - k];
        }
        /* EEE and EEO */
        for (k = 0; k < 4; k++)
        {
            EEE[k] = EE[k] + EE[7 - k];
            EEO[k] = EE[k] - EE[7 - k];
        }
        /* EEEE and EEEO */
        EEEE[0] = EEE[0] + EEE[3];
        EEEO[0] = EEE[0] - EEE[3];
        EEEE[1] = EEE[1] + EEE[2];
        EEEO[1] = EEE[1] - EEE[2];

        dst[0 * line + j] = (evc_tbl_tm32[0][0] * EEEE[0] + evc_tbl_tm32[0][1] * EEEE[1] + add) >> shift;
        dst[16 * line + j] = (evc_tbl_tm32[16][0] * EEEE[0] + evc_tbl_tm32[16][1] * EEEE[1] + add) >> shift;
        dst[8 * line + j] = (evc_tbl_tm32[8][0] * EEEO[0] + evc_tbl_tm32[8][1] * EEEO[1] + add) >> shift;
        dst[24 * line + j] = (evc_tbl_tm32[24][0] * EEEO[0] + evc_tbl_tm32[24][1] * EEEO[1] + add) >> shift;
        for (k = 4; k < 32; k += 8)
        {
            dst[k*line + j] = (evc_tbl_tm32[k][0] * EEO[0] + evc_tbl_tm32[k][1] * EEO[1] + evc_tbl_tm32[k][2] * EEO[2] + evc_tbl_tm32[k][3] * EEO[3] + add) >> shift;
        }
        for (k = 2; k < 32; k += 4)
        {
            dst[k*line + j] = (evc_tbl_tm32[k][0] * EO[0] + evc_tbl_tm32[k][1] * EO[1] + evc_tbl_tm32[k][2] * EO[2] + evc_tbl_tm32[k][3] * EO[3] +
                evc_tbl_tm32[k][4] * EO[4] + evc_tbl_tm32[k][5] * EO[5] + evc_tbl_tm32[k][6] * EO[6] + evc_tbl_tm32[k][7] * EO[7] + add) >> shift;
        }
        for (k = 1; k < 32; k += 2)
        {
            dst[k*line + j] = (evc_tbl_tm32[k][0] * O[0] + evc_tbl_tm32[k][1] * O[1] + evc_tbl_tm32[k][2] * O[2] + evc_tbl_tm32[k][3] * O[3] +
                evc_tbl_tm32[k][4] * O[4] + evc_tbl_tm32[k][5] * O[5] + evc_tbl_tm32[k][6] * O[6] + evc_tbl_tm32[k][7] * O[7] +
                evc_tbl_tm32[k][8] * O[8] + evc_tbl_tm32[k][9] * O[9] + evc_tbl_tm32[k][10] * O[10] + evc_tbl_tm32[k][11] * O[11] +
                evc_tbl_tm32[k][12] * O[12] + evc_tbl_tm32[k][13] * O[13] + evc_tbl_tm32[k][14] * O[14] + evc_tbl_tm32[k][15] * O[15] + add) >> shift;
        }
    }
}

static void tx_pb64(s16 *src, s16 *dst, int shift, int line)
{
    const int tx_size = 64;
    const s8 * tm = evc_tbl_tm64[0];

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
            E[k] = src[k] + src[63 - k];
            O[k] = src[k] - src[63 - k];
        }
        for (k = 0; k<16; k++)
        {
            EE[k] = E[k] + E[31 - k];
            EO[k] = E[k] - E[31 - k];
        }
        for (k = 0; k<8; k++)
        {
            EEE[k] = EE[k] + EE[15 - k];
            EEO[k] = EE[k] - EE[15 - k];
        }
        for (k = 0; k<4; k++)
        {
            EEEE[k] = EEE[k] + EEE[7 - k];
            EEEO[k] = EEE[k] - EEE[7 - k];
        }
        EEEEE[0] = EEEE[0] + EEEE[3];
        EEEEO[0] = EEEE[0] - EEEE[3];
        EEEEE[1] = EEEE[1] + EEEE[2];
        EEEEO[1] = EEEE[1] - EEEE[2];

        dst[0] = (tm[0 * 64 + 0] * EEEEE[0] + tm[0 * 64 + 1] * EEEEE[1] + add) >> shift;
        dst[16 * line] = (tm[16 * 64 + 0] * EEEEO[0] + tm[16 * 64 + 1] * EEEEO[1] + add) >> shift;

        dst[32 * line] = 0;
        dst[48 * line] = 0;

        for (k = 8; k<64; k += 16)
        {
            if (k > 31)
            {
                dst[k*line] = 0;
            }
            else
            {
                dst[k*line] = (tm[k * 64 + 0] * EEEO[0] + tm[k * 64 + 1] * EEEO[1] + tm[k * 64 + 2] * EEEO[2] + tm[k * 64 + 3] * EEEO[3] + add) >> shift;
            }
        }
        for (k = 4; k<64; k += 8)
        {
            if (k > 31)
            {
                dst[k*line] = 0;
            }
            else
            {
                dst[k*line] = (tm[k * 64 + 0] * EEO[0] + tm[k * 64 + 1] * EEO[1] + tm[k * 64 + 2] * EEO[2] + tm[k * 64 + 3] * EEO[3] +
                    tm[k * 64 + 4] * EEO[4] + tm[k * 64 + 5] * EEO[5] + tm[k * 64 + 6] * EEO[6] + tm[k * 64 + 7] * EEO[7] + add) >> shift;
            }
        }
        for (k = 2; k<64; k += 4)
        {
            if (k > 31)
            {
                dst[k*line] = 0;
            }
            else
            {
                dst[k*line] = (tm[k * 64 + 0] * EO[0] + tm[k * 64 + 1] * EO[1] + tm[k * 64 + 2] * EO[2] + tm[k * 64 + 3] * EO[3] +
                    tm[k * 64 + 4] * EO[4] + tm[k * 64 + 5] * EO[5] + tm[k * 64 + 6] * EO[6] + tm[k * 64 + 7] * EO[7] +
                    tm[k * 64 + 8] * EO[8] + tm[k * 64 + 9] * EO[9] + tm[k * 64 + 10] * EO[10] + tm[k * 64 + 11] * EO[11] +
                    tm[k * 64 + 12] * EO[12] + tm[k * 64 + 13] * EO[13] + tm[k * 64 + 14] * EO[14] + tm[k * 64 + 15] * EO[15] + add) >> shift;
            }
        }
        for (k = 1; k<64; k += 2)
        {
            if (k > 31)
            {
                dst[k*line] = 0;
            }
            else
            {
                dst[k*line] = (tm[k * 64 + 0] * O[0] + tm[k * 64 + 1] * O[1] + tm[k * 64 + 2] * O[2] + tm[k * 64 + 3] * O[3] +
                    tm[k * 64 + 4] * O[4] + tm[k * 64 + 5] * O[5] + tm[k * 64 + 6] * O[6] + tm[k * 64 + 7] * O[7] +
                    tm[k * 64 + 8] * O[8] + tm[k * 64 + 9] * O[9] + tm[k * 64 + 10] * O[10] + tm[k * 64 + 11] * O[11] +
                    tm[k * 64 + 12] * O[12] + tm[k * 64 + 13] * O[13] + tm[k * 64 + 14] * O[14] + tm[k * 64 + 15] * O[15] +
                    tm[k * 64 + 16] * O[16] + tm[k * 64 + 17] * O[17] + tm[k * 64 + 18] * O[18] + tm[k * 64 + 19] * O[19] +
                    tm[k * 64 + 20] * O[20] + tm[k * 64 + 21] * O[21] + tm[k * 64 + 22] * O[22] + tm[k * 64 + 23] * O[23] +
                    tm[k * 64 + 24] * O[24] + tm[k * 64 + 25] * O[25] + tm[k * 64 + 26] * O[26] + tm[k * 64 + 27] * O[27] +
                    tm[k * 64 + 28] * O[28] + tm[k * 64 + 29] * O[29] + tm[k * 64 + 30] * O[30] + tm[k * 64 + 31] * O[31] + add) >> shift;
            }
        }
        src += tx_size;
        dst++;
    }
}

/********************************** DST-VII **********************************/
void evce_trans_DST7_B4(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i;
    int rnd_factor = 1 << (shift - 1);
    const s16 *tm = evc_tbl_tr4[DST7][0];
    int c[4];
    s16 *tmp = coef;
    const int reduced_line = line - skip_line;

    for (i = 0; i < reduced_line; i++)
    {
        /* Intermediate Variables */

        c[0] = block[0] + block[3];
        c[1] = block[1] + block[3];
        c[2] = block[0] - block[1];
        c[3] = tm[2] * block[2];

        coef[0] = (tm[0] * c[0] + tm[1] * c[1] + c[3] + rnd_factor) >> shift;
        coef[line] = (tm[2] * (block[0] + block[1] - block[3]) + rnd_factor) >> shift;
        coef[2 * line] = (tm[0] * c[2] + tm[1] * c[0] - c[3] + rnd_factor) >> shift;
        coef[3 * line] = (tm[1] * c[2] - tm[0] * c[1] + c[3] + rnd_factor) >> shift;
        block += 4;
        coef++;
    }

    if (skip_line)
    {
        coef = tmp + reduced_line;
        for (i = 0; i < 4; i++)
        {
            evc_mset(coef, 0, sizeof(s16)* skip_line);
            coef += line;
        }
    }
}

void evce_trans_DST7_B8(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 8;
    const s16 *tm;
    s16 *coef_tmp;
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        coef_tmp = coef;
        tm = evc_tbl_tr8[DST7][0];
        for (j = 0; j < cut_off; j++)
        {
            sum = 0;
            for (k = 0; k < tr_size; k++)
            {
                sum += tm[k] * block[k];
            }
            coef_tmp[i] = (sum + rnd_factor) >> shift;
            coef_tmp += line;
            tm += tr_size;
        }
        block += tr_size;
    }

    if (skip_line)
    {
        coef_tmp = coef + reduced_line;
        for (j = 0; j < cut_off; j++)
        {
            evc_mset(coef_tmp, 0, sizeof(s16)* skip_line);
            coef_tmp += line;
        }
    }

    if (skip_line_2)
    {
        coef_tmp = coef + line * cut_off;
        evc_mset(coef_tmp, 0, sizeof(s16)* line * skip_line_2);
    }
}

void evce_trans_DST7_B16(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 16;
    const s16 *tm;
    s16 *coef_tmp;
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        coef_tmp = coef;
        tm = evc_tbl_tr16[DST7][0];
        for (j = 0; j < cut_off; j++)
        {
            sum = 0;
            for (k = 0; k < tr_size; k++)
            {
                sum += tm[k] * block[k];
            }
            coef_tmp[i] = (sum + rnd_factor) >> shift;
            coef_tmp += line;
            tm += tr_size;
        }
        block += tr_size;
    }

    if (skip_line)
    {
        coef_tmp = coef + reduced_line;
        for (j = 0; j < cut_off; j++)
        {
            evc_mset(coef_tmp, 0, sizeof(s16)* skip_line);
            coef_tmp += line;
        }
    }

    if (skip_line_2)
    {
        coef_tmp = coef + line * cut_off;
        evc_mset(coef_tmp, 0, sizeof(s16)* line * skip_line_2);
    }
}

void evce_trans_DST7_B32(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 32;
    const s16 *tm;
    s16 *coef_tmp;
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        coef_tmp = coef;
        tm = evc_tbl_tr32[DST7][0];
        for (j = 0; j < cut_off; j++)
        {
            sum = 0;
            for (k = 0; k < tr_size; k++)
            {
                sum += block[k] * tm[k];
            }
            coef_tmp[i] = (sum + rnd_factor) >> shift;
            tm += tr_size;
            coef_tmp += line;
        }
        block += tr_size;
    }

    if (skip_line)
    {
        coef_tmp = coef + reduced_line;
        for (j = 0; j < cut_off; j++)
        {
            evc_mset(coef_tmp, 0, sizeof(s16)* skip_line);
            coef_tmp += line;
        }
    }

    if (skip_line_2)
    {
        coef_tmp = coef + line * cut_off;
        evc_mset(coef_tmp, 0, sizeof(s16)* line * skip_line_2);
    }
}

/********************************** DCT-VIII **********************************/
void evce_trans_DCT8_B4(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i;
    int rnd_factor = 1 << (shift - 1);
    const s16 *tm = evc_tbl_tr4[DCT8][0];
    int c[4];
    s16 *tmp = coef;
    const int reduced_line = line - skip_line;

    for (i = 0; i < reduced_line; i++)
    {
        /* Intermediate Variables */
        c[0] = block[0] + block[3];
        c[1] = block[2] + block[0];
        c[2] = block[3] - block[2];
        c[3] = tm[1] * block[1];

        coef[0] = (tm[3] * c[0] + tm[2] * c[1] + c[3] + rnd_factor) >> shift;
        coef[line] = (tm[1] * (block[0] - block[2] - block[3]) + rnd_factor) >> shift;
        coef[2 * line] = (tm[3] * c[2] + tm[2] * c[0] - c[3] + rnd_factor) >> shift;
        coef[3 * line] = (tm[3] * c[1] - tm[2] * c[2] - c[3] + rnd_factor) >> shift;
        block += 4;
        coef++;
    }

    if (skip_line)
    {
        coef = tmp + reduced_line;
        for (i = 0; i < 4; i++)
        {
            evc_mset(coef, 0, sizeof(s16)* skip_line);
            coef += line;
        }
    }
}

void evce_trans_DCT8_B8(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 8;
    const s16 *tm;
    s16 *coef_tmp;
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        coef_tmp = coef;
        tm = evc_tbl_tr8[DCT8][0];
        for (j = 0; j < cut_off; j++)
        {
            sum = 0;
            for (k = 0; k < tr_size; k++)
            {
                sum += tm[k] * block[k];
            }
            coef_tmp[i] = (sum + rnd_factor) >> shift;
            coef_tmp += line;
            tm += tr_size;
        }
        block += tr_size;
    }

    if (skip_line)
    {
        coef_tmp = coef + reduced_line;
        for (j = 0; j < cut_off; j++)
        {
            evc_mset(coef_tmp, 0, sizeof(s16)* skip_line);
            coef_tmp += line;
        }
    }

    if (skip_line_2)
    {
        coef_tmp = coef + line * cut_off;
        evc_mset(coef_tmp, 0, sizeof(s16)* line * skip_line_2);
    }
}

void evce_trans_DCT8_B16(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 16;
    const s16 *tm;
    s16 *coef_tmp;
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        coef_tmp = coef;
        tm = evc_tbl_tr16[DCT8][0];
        for (j = 0; j < tr_size; j++)
        {
            sum = 0;
            for (k = 0; k < tr_size; k++)
            {
                sum += tm[k] * block[k];
            }
            coef_tmp[i] = (sum + rnd_factor) >> shift;
            coef_tmp += line;
            tm += tr_size;
        }
        block += tr_size;
    }

    if (skip_line)
    {
        coef_tmp = coef + reduced_line;
        for (j = 0; j < cut_off; j++)
        {
            evc_mset(coef_tmp, 0, sizeof(s16)* skip_line);
            coef_tmp += line;
        }
    }

    if (skip_line_2)
    {
        coef_tmp = coef + line * cut_off;
        evc_mset(coef_tmp, 0, sizeof(s16)* line * skip_line_2);
    }
}

void evce_trans_DCT8_B32(s16 *block, s16 *coef, s32 shift, s32 line, int skip_line, int skip_line_2)  /* input block, output coef */
{
    int i, j, k, sum;
    int rnd_factor = 1 << (shift - 1);
    const int tr_size = 32;
    const s16 *tm;
    s16 *coef_tmp;
    const int reduced_line = line - skip_line;
    const int cut_off = tr_size - skip_line_2;

    for (i = 0; i < reduced_line; i++)
    {
        coef_tmp = coef;
        tm = evc_tbl_tr32[DCT8][0];
        for (j = 0; j < cut_off; j++)
        {
            sum = 0;
            for (k = 0; k < tr_size; k++)
            {
                sum += block[k] * tm[k];
            }
            coef_tmp[i] = (sum + rnd_factor) >> shift;
            tm += tr_size;
            coef_tmp += line;
        }
        block += tr_size;
    }

    if (skip_line)
    {
        coef_tmp = coef + reduced_line;
        for (j = 0; j < cut_off; j++)
        {
            evc_mset(coef_tmp, 0, sizeof(s16)* skip_line);
            coef_tmp += line;
        }
    }

    if (skip_line_2)
    {
        coef_tmp = coef + line * cut_off;
        evc_mset(coef_tmp, 0, sizeof(s16)* line * skip_line_2);
    }
}

void evc_t_MxN_ats_intra(s16 *coef, int tuw, int tuh, int bit_depth, u8 ats_intra_mode, u8 ats_intra_tridx)
{
    const int shift_1st = CONV_LOG2(tuw) - 1 + bit_depth - 8;
    const int shift_2nd = CONV_LOG2(tuh) + 6;
    const u8 log2_minus1_w = CONV_LOG2(tuw) - 1;
    const u8 log2_minus1_h = CONV_LOG2(tuh) - 1;
    s16 t[MAX_TR_DIM]; /* temp buffer */
    u8  t_idx_h = 0, t_idx_v = 0;
    int skip_w = 0;
    int skip_h = 0;

    t_idx_h = evc_tbl_tr_subset_intra[ats_intra_tridx >> 1];
    t_idx_v = evc_tbl_tr_subset_intra[ats_intra_tridx & 1];

    evce_trans_map_tbl[t_idx_h][log2_minus1_w](coef, t, shift_1st, tuh, 0, skip_w);
    evce_trans_map_tbl[t_idx_v][log2_minus1_h](t, coef, shift_2nd, tuw, skip_w, skip_h);
}

typedef void(*EVC_TXB)(void * coef, void * t, int shift, int line, int step);
static EVC_TXB evce_tbl_txb[MAX_TR_LOG2] =
{
    tx_pb2b,
    tx_pb4b,
    tx_pb8b,
    tx_pb16b,
    tx_pb32b,
    tx_pb64b
};

typedef void(*EVC_TX)(s16 * coef, s16 * t, int shift, int line);
static EVC_TX evce_tbl_tx[MAX_TR_LOG2] =
{
    tx_pb2,
    tx_pb4,
    tx_pb8,
    tx_pb16,
    tx_pb32,
    tx_pb64
};

void evce_trans_ats_intra(s16* coef, int log2_cuw, int log2_cuh, u8 ats_intra_cu, u8 ats_tu)
{
    evc_t_MxN_ats_intra(coef, (1 << log2_cuw), (1 << log2_cuh), BIT_DEPTH, ats_intra_cu, ats_tu);
}

void evce_trans(s16 * coef, int log2_cuw, int log2_cuh, int iqt_flag)
{    
    int shift1 = evc_get_transform_shift(log2_cuw, 0);
    int shift2 = evc_get_transform_shift(log2_cuh, 1);

    if(iqt_flag == 1)
    {
        s16 t[MAX_TR_DIM]; /* temp buffer */
        evce_tbl_tx[log2_cuw - 1](coef, t, shift1, 1 << log2_cuh);
        evce_tbl_tx[log2_cuh - 1](t, coef, shift2, 1 << log2_cuw);
    }
    else
    {
        s32 tb[MAX_TR_DIM]; /* temp buffer */
        evce_tbl_txb[log2_cuw - 1](coef, tb, 0, 1 << log2_cuh, 0);
        evce_tbl_txb[log2_cuh - 1](tb, coef, (shift1 + shift2), 1 << log2_cuw, 1);
    }
}

void evce_init_err_scale()
{
    double err_scale;
    int qp;
    int i;

    for (qp = 0; qp < 6; qp++)
    {
        int q_value = quant_scale[qp];

        for (i = 0; i < MAX_CU_DEPTH; i++)
        {
            int tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - (i + 1);

            err_scale = (double)(1 << SCALE_BITS) * pow(2.0, -tr_shift);
            err_scale = err_scale / q_value / (1 << ((BIT_DEPTH - 8)));
            err_scale_tbl[qp][i] = (s64)(err_scale * (double)(1 << ERR_SCALE_PRECISION_BITS));
        }
    }
}

#define GET_I_COST(rate, lamba)  (rate*lamba)
#define GET_IEP_RATE             (32768)

__inline static s64 get_rate_positionLastXY(int pos_x, int pos_y, int width, int height, int ch_type, s64 lambda, int sps_cm_init_flag)
{
    int group_idx_x;
    int group_idx_y;
    int blk_offset_x, blk_offset_y, shift_x, shift_y;
    int bin, cnt;
    int rate = 0;
    int offset_x = (ch_type == Y_C ? 0 : (sps_cm_init_flag == 1 ? NUM_CTX_SCANR_LUMA : 11));
    int offset_y = (ch_type == Y_C ? 0 : (sps_cm_init_flag == 1 ? NUM_CTX_SCANR_LUMA : 11));

    group_idx_x = g_group_idx[pos_x];
    group_idx_y = g_group_idx[pos_y];
    if (sps_cm_init_flag == 1)
    {
        evc_get_ctx_last_pos_xy_para(ch_type, width, height, &blk_offset_x, &blk_offset_y, &shift_x, &shift_y);
    }
    else
    {
        blk_offset_x = 0;
        blk_offset_y = 0;
        shift_x = 0;
        shift_y = 0;
    }
    //------------------

    // posX

    for (bin = 0; bin < group_idx_x; bin++)
    {
        rate += rdoq_est_scanr_x[offset_x + blk_offset_x + (bin >> shift_x)][1];
    }
    if (group_idx_x < g_group_idx[width - 1])
    {
        rate += rdoq_est_scanr_x[offset_x + blk_offset_x + (bin >> shift_x)][0];
    }

    // posY

    for (bin = 0; bin < group_idx_y; bin++)
    {
        rate += rdoq_est_scanr_y[offset_y + blk_offset_y + (bin >> shift_y)][1];
    }
    if (group_idx_y < g_group_idx[height - 1])
    {
        rate += rdoq_est_scanr_y[offset_y + blk_offset_y + (bin >> shift_y)][0];
    }

    // EP-coded part

    if (group_idx_x > 3)
    {
        cnt = (group_idx_x - 2) >> 1;
        pos_x = pos_x - g_min_in_group[group_idx_x];
        rate += (cnt * GET_IEP_RATE);
    }
    if (group_idx_y > 3)
    {
        cnt = (group_idx_y - 2) >> 1;
        pos_y = pos_y - g_min_in_group[group_idx_y];
        rate += (cnt * GET_IEP_RATE);
    }

    return GET_I_COST(rate, lambda);
}

__inline static s64 get_rate_gt0(int significance, int ctx_gt0, s64 lambda)
{
    s64 rate = rdoq_est_gt0[ctx_gt0][significance];
    return GET_I_COST(rate, lambda);
}

__inline static int get_ic_rate(int abs_level, int ctx_gtA, int ctx_gtB, int rparam, int c1_idx, int c2_idx, int num_gtA, int num_gtB)
{
    int rate = GET_IEP_RATE; // cost of sign bit
    int base_level = (c1_idx < num_gtA) ? (2 + (c2_idx < num_gtB ? 1 : 0)) : 1;

    if (abs_level >= base_level)
    {
        int symbol = abs_level - base_level;
        int length;

        if (symbol < (g_go_rice_range[rparam] << rparam))
        {
            length = symbol >> rparam;
            rate += (length + 1 + rparam) << 15;
        }
        else
        {
            length = rparam;
            symbol = symbol - (g_go_rice_range[rparam] << rparam);
            while (symbol >= (1 << length))
            {
                symbol -= (1 << (length++));
            }
            rate += (g_go_rice_range[rparam] + length + 1 - rparam + length) << 15;
        }

        if (c1_idx < num_gtA)
        {
            rate += rdoq_est_gtA[ctx_gtA][1];
            if (c2_idx < num_gtB)
            {
                rate += rdoq_est_gtA[ctx_gtB][1];
            }
        }
    }
    else if (abs_level == 1)
    {
        rate += rdoq_est_gtA[ctx_gtA][0];
    }
    else if (abs_level == 2)
    {
        rate += rdoq_est_gtA[ctx_gtA][1];
        rate += rdoq_est_gtA[ctx_gtB][0];
    }
    else
    {
        rate = 0;
    }

    return  rate;
}

__inline static int get_coded_level(
    s64*    rd_coded_cost,          //< reference to coded cost
    s64*    rd_coded_cost0,         //< reference to cost when coefficient is 0
    s64*    rd_coded_cost_sig,       //< rd_coded_cost_sig reference to cost of significant coefficient
    s64     level_double,           //< reference to unscaled quantized level
    int     max_abs_level,          //< scaled quantized level
    int     ctx_gt0,          //< current ctxInc for coeff_abs_significant_flag
    int     ctx_gtA,          //< current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
    int     ctx_gtB,          //< current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
    int     rparam,          //< current Rice parameter for coeff_abs_level_minus3
    int     c1_idx,                  //< 
    int     c2_idx,                  //< 
    int     num_gtA,
    int     num_gtB,
    int     qbits,                 //< quantization step size
    s64     error_scale,             //< 
    s64     lambda,
    int     bypass_sigmap
)
{
    s64 curr_cost_sig = 0;
    s64 curr_cost;
    int best_abs_level = 0;
    int min_abs_level;
    int abs_level;
    int rate_best = 0;
    int rate_max = 0;
    int rate = 0;

    if (bypass_sigmap == 0 && max_abs_level < 3)
    {
        *rd_coded_cost_sig = get_rate_gt0(0, ctx_gt0, lambda);
        *rd_coded_cost = *rd_coded_cost0 + *rd_coded_cost_sig;

        if (max_abs_level == 0)
        {
            return best_abs_level;
        }
    }
    else
    {
        *rd_coded_cost = EVC_INT64_MAX;
    }

    if (bypass_sigmap == 0)
    {
        curr_cost_sig = get_rate_gt0(1, ctx_gt0, lambda);
    }

    min_abs_level = (max_abs_level > 1 ? max_abs_level - 1 : 1);
    for (abs_level = max_abs_level; abs_level >= min_abs_level; abs_level--)
    {
        s64 err = (s64)(level_double - ((s64)abs_level << qbits));
        rate = get_ic_rate(abs_level, ctx_gtA, ctx_gtB, rparam, c1_idx, c2_idx, num_gtA, num_gtB);
        err = (err * error_scale) >> ERR_SCALE_PRECISION_BITS;
        curr_cost = err * err + GET_I_COST(rate, lambda);
        curr_cost += curr_cost_sig;

        if (curr_cost < *rd_coded_cost)
        {
            best_abs_level = abs_level;
            *rd_coded_cost = curr_cost;
            *rd_coded_cost_sig = curr_cost_sig;
            rate_best = rate;
        }
        if (abs_level == max_abs_level)
        {
            rate_max = rate;
        }
    }

    return best_abs_level;
}

__inline static int get_ctx_gt012_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int *num1, int *num2)
{
    const s16 *pdata = pcoeff + blkpos;
    const int width_m1 = width - 1;
    const int height_m1 = height - 1;
    const int log2_w = CONV_LOG2(width);
    const int pos_y = blkpos >> log2_w;
    const int pos_x = blkpos - (pos_y << log2_w);
    const int diag = pos_x + pos_y;
    int num_gt0 = 0;
    int num_gtA = 0;
    int num_gtB = 0;
    int ctx_idx;
    int ctx_ofs;
    s16 tmp;

    if (pos_x < width_m1)
    {
        tmp = EVC_ABS16(pdata[1]);
        num_gt0 += !!(tmp);
        num_gtA += (tmp > 1 ? 1 : 0);
        num_gtB += (tmp > 2 ? 1 : 0);
        if (pos_x < width_m1 - 1)
        {
            tmp = EVC_ABS16(pdata[2]);
            num_gt0 += !!(tmp);
            num_gtA += (tmp > 1 ? 1 : 0);
            num_gtB += (tmp > 2 ? 1 : 0);
        }
        if (pos_y < height_m1)
        {
            tmp = EVC_ABS16(pdata[width + 1]);
            num_gt0 += !!(tmp);
            num_gtA += (tmp > 1 ? 1 : 0);
            num_gtB += (tmp > 2 ? 1 : 0);
        }
    }
    if (pos_y < height_m1)
    {
        tmp = EVC_ABS16(pdata[width]);
        num_gt0 += !!(tmp);
        num_gtA += (tmp > 1 ? 1 : 0);
        num_gtB += (tmp > 2 ? 1 : 0);
        if (pos_y < height_m1 - 1)
        {
            tmp = EVC_ABS16(pdata[2 * width]);
            num_gt0 += !!(tmp);
            num_gtA += (tmp > 1 ? 1 : 0);
            num_gtB += (tmp > 2 ? 1 : 0);
        }
    }

    ctx_idx = EVC_MIN(num_gt0, 4) + 1;
#if 0// COEFF_REAL_LAST  JEM alinged

    if (ch_type == Y_C)
    {
        ctx_idx += diag < 3 ? 10 : (diag < 10 ? 5 : 0);
    }
#else // Context derivation by SRCC
    if (diag < 2)
    {
        ctx_idx = EVC_MIN(ctx_idx, 2);
    }

    if (ch_type == Y_C)
    {
        ctx_ofs = diag < 2 ? 0 : (diag < 5 ? 2 : 7);
    }
    else
    {
        ctx_ofs = diag < 2 ? 0 : 2;
    }

    *num1 = EVC_MIN(num_gtA, 3) + 1;
    *num2 = EVC_MIN(num_gtB, 3) + 1;
    if (ch_type == Y_C)
    {
        *num1 += (diag < 3) ? 0 : ((diag < 10) ? 4 : 8);
        *num2 += (diag < 3) ? 0 : ((diag < 10) ? 4 : 8);
    }
#endif

    return ctx_ofs + ctx_idx;
}

static __inline s64 get_ic_rate_cost_rl(u32 abs_level, u32 run, s32 ctx_run, u32 ctx_level, s64 lambda)
{
    s32 rate;
    if(abs_level == 0)
    {
        rate = 0;
        if(run == 0)
        {
            rate += rdoq_est_run[ctx_run][1];
        }
        else
        {
            rate += rdoq_est_run[ctx_run + 1][1];
        }
    }
    else
    {
        rate = GET_IEP_RATE;
        if(run == 0)
        {
            rate += rdoq_est_run[ctx_run][0];
        }
        else
        {
            rate += rdoq_est_run[ctx_run + 1][0];
        }

        if(abs_level == 1)
        {
            rate += rdoq_est_level[ctx_level][0];
        }
        else
        {
            rate += rdoq_est_level[ctx_level][1];
            rate += rdoq_est_level[ctx_level + 1][1] * (s32)(abs_level - 2);
            rate += rdoq_est_level[ctx_level + 1][0];
        }
    }
    return (s64)GET_I_COST(rate, lambda);
}

static __inline u32 get_coded_level_rl(s64* rd64_uncoded_cost, s64* rd64_coded_cost, s64 level_double, u32 max_abs_level,
                                       u32 run, u16 ctx_run, u16 ctx_level, s32 q_bits, s64 err_scale, s64 lambda)
{
    u32 best_abs_level = 0;
    s64 err1 = (level_double * err_scale) >> ERR_SCALE_PRECISION_BITS;
    u32 min_abs_level;
    u32 abs_level;

    *rd64_uncoded_cost = err1 * err1;
    *rd64_coded_cost = *rd64_uncoded_cost + get_ic_rate_cost_rl(0, run, ctx_run, ctx_level, lambda);

    min_abs_level = (max_abs_level > 1 ? max_abs_level - 1 : 1);
    for(abs_level = max_abs_level; abs_level >= min_abs_level; abs_level--)
    {
        s64 i64Delta = level_double - ((s64)abs_level << q_bits);
        s64 err = (i64Delta * err_scale) >> ERR_SCALE_PRECISION_BITS;
        s64 dCurrCost = err * err + get_ic_rate_cost_rl(abs_level, run, ctx_run, ctx_level, lambda);

        if(dCurrCost < *rd64_coded_cost)
        {
            best_abs_level = abs_level;
            *rd64_coded_cost = dCurrCost;
        }
    }
    return best_abs_level;
}

int evce_rdoq_run_length_cc(u8 qp, double d_lambda, u8 is_intra, s16 *src_coef, s16 *dst_tmp, int log2_cuw, int log2_cuh, int ch_type, int sps_cm_init_flag)
{
    const int qp_rem = qp % 6;
    const int ns_shift = ((log2_cuw + log2_cuh) & 1) ? 7 : 0;
    const int ns_scale = ((log2_cuw + log2_cuh) & 1) ? 181 : 1;
    const int ns_offset = ((log2_cuw + log2_cuh) & 1) ? (1 << (ns_shift - 1)) : 0;
    const int q_value = (quant_scale[qp_rem] * ns_scale + ns_offset) >> ns_shift;
    const int log2_size = (log2_cuw + log2_cuh) >> 1;
    const int tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - (log2_size);
    const u32 max_num_coef = 1 << (log2_cuw + log2_cuh);
    const u16 *scan = evc_scan_tbl[COEF_SCAN_ZIGZAG][log2_cuw - 1][log2_cuh - 1];
    const int ctx_last = (ch_type == Y_C) ? 0 : 1;
    const int q_bits = QUANT_SHIFT + tr_shift + (qp / 6);
    int nnz = 0;
    int sum_all = 0;

    u32 scan_pos;
    u32 run;
    u32 prev_level;
    s32 ctx_qt_cbf;
    u32 best_last_idx_p1 = 0;
    s16 tmp_coef[MAX_TR_DIM];
    s64 tmp_level_double[MAX_TR_DIM];
    s16 tmp_dst_coef[MAX_TR_DIM];
    const s64 lambda = (s64)(d_lambda * (double)(1 << SCALE_BITS) + 0.5);
    s64 err_scale = err_scale_tbl[qp_rem][log2_size - 1];
    s64 d64_best_cost = 0;
    s64 d64_base_cost = 0;
    s64 d64_coded_cost = 0;
    s64 d64_uncoded_cost = 0;
    s64 d64_block_uncoded_cost = 0;
    s64 err;

    /* ===== quantization ===== */
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];
        s64 level_double = src_coef[blk_pos];
        u32 max_abs_level;
        s8 lower_int;
        s64 temp_level;

        temp_level = ((s64)EVC_ABS(src_coef[blk_pos]) * (s64)q_value);

        level_double = (int)EVC_MIN(((s64)temp_level), (s64)EVC_INT32_MAX - (s64)(1 << (q_bits - 1)));
        tmp_level_double[blk_pos] = level_double;
        max_abs_level = (u32)(level_double >> q_bits);
        lower_int = ((level_double - ((s64)max_abs_level << q_bits)) < (s64)(1 << (q_bits - 1))) ? 1 : 0;

        if (!lower_int)
        {
            max_abs_level++;
        }

        err = (level_double * err_scale) >> ERR_SCALE_PRECISION_BITS;
        d64_block_uncoded_cost += err * err;
        tmp_coef[blk_pos] = src_coef[blk_pos] > 0 ? (s16)max_abs_level : -(s16)(max_abs_level);
        sum_all += max_abs_level;
    }

    evc_mset(dst_tmp, 0, sizeof(s16)*max_num_coef);

    if (sum_all == 0)
    {
        return nnz;
    }

    if (!is_intra && ch_type == Y_C)
    {
        d64_best_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_all_cbf[0], lambda);
        d64_base_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_all_cbf[1], lambda);
    }
    else
    {
        ctx_qt_cbf = ch_type;
        d64_best_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][0], lambda);
        d64_base_cost = d64_block_uncoded_cost + GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][1], lambda);
    }

    run = 0;
    prev_level = 6;

    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];
        u32 level;
#if CTX_MODEL_FOR_RESIDUAL_IN_BASE
        int ctx_run = ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);
        int ctx_level = ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);
#else
        int ctx_run = sps_cm_init_flag == 1 ? ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12) : (ch_type == Y_C ? 0 : 2);
        int ctx_level = sps_cm_init_flag == 1 ? ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12) : (ch_type == Y_C ? 0 : 2);
#endif

        level = get_coded_level_rl(&d64_uncoded_cost, &d64_coded_cost, tmp_level_double[blk_pos], EVC_ABS(tmp_coef[blk_pos]), run, ctx_run, ctx_level, q_bits, err_scale, lambda);
        tmp_dst_coef[blk_pos] = tmp_coef[blk_pos] < 0 ? -(s32)(level) : level;
        d64_base_cost -= d64_uncoded_cost;
        d64_base_cost += d64_coded_cost;

        if (level)
        {
            /* ----- check for last flag ----- */
            s64 d64_cost_last_zero = GET_I_COST(rdoq_est_last[ctx_last][0], lambda);
            s64 d64_cost_last_one = GET_I_COST(rdoq_est_last[ctx_last][1], lambda);
            s64 d64_cur_is_last_cost = d64_base_cost + d64_cost_last_one;
            d64_base_cost += d64_cost_last_zero;

            if (d64_cur_is_last_cost < d64_best_cost)
            {
                d64_best_cost = d64_cur_is_last_cost;
                best_last_idx_p1 = scan_pos + 1;
            }
            run = 0;
            prev_level = level;
        }
        else
        {
            run++;
        }
    }

    /* ===== clean uncoded coeficients ===== */
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];

        if (scan_pos < best_last_idx_p1)
        {
            if (tmp_dst_coef[blk_pos])
            {
                nnz++;
            }
        }
        else
        {
            tmp_dst_coef[blk_pos] = 0;
        }

        dst_tmp[blk_pos] = tmp_dst_coef[blk_pos];
    }

    return nnz;
}

int ifvce_rdoq_method_ccA(u8 qp, double d_lambda, u8 is_intra, s16 *src_coef, s16 *dst_tmp, int log2_cuw, int log2_cuh, int ch_type, int sps_cm_init_flag)
{
    const int ns_shift = ((log2_cuw + log2_cuh) & 1) ? 7 : 0;
    const int ns_scale = ((log2_cuw + log2_cuh) & 1) ? 181 : 1;
    const int qp_rem = qp % 6;
    const int q_value = (quant_scale[qp_rem] * ns_scale) >> ns_shift;
    const int log2_size = (log2_cuw + log2_cuh) >> 1;
    const int tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - (log2_size);

    s64 err_scale = err_scale_tbl[qp_rem][log2_size - 1];
    s64 lambda = (s64)(d_lambda * (double)(1 << SCALE_BITS) + 0.5);
    int q_bits;
    const int width = (1 << log2_cuw);
    const int height = (1 << log2_cuh);
    const int max_num_coef = width * height;
    int scan_type = COEF_SCAN_ZIGZAG;
    int log2_block_size = min(log2_cuw, log2_cuh);
    u16 *scan;
    int scan_pos_last = -1;
    int ipos;
    int cg_log2_size = LOG2_CG_SIZE;
    int cg_size = 1 << cg_log2_size;
    int last_scan_set;
    int sub_set;

    int offset1 = (sps_cm_init_flag == 1) ? ((ch_type == Y_C) ? 0 : NUM_CTX_GTA_LUMA) : ((ch_type == Y_C) ? 0 : 1);
    int offset0 = (sps_cm_init_flag == 1) ? ((ch_type == Y_C) ? (log2_block_size <= 2 ? 0 : NUM_CTX_GT0_LUMA_TU << (EVC_MIN(1, (log2_block_size - 3)))) : NUM_CTX_GT0_LUMA) : (ch_type == Y_C ? 0 : 1);
    int c1_idx = 0;
    int c2_idx = 0;
    s64 cost_base = 0;
    s64 cost_best = 0;
    int best_last_idx_p1 = 0;
    int found_last = 0;
    s64 cbf_cost = 0;
    int nnz = 0;
    int rice_param = 0;
    s64 dcost_block_uncoded = 0;
    s64 pdcost_coeff[MAX_TR_DIM];
    s64 pdcost_sig[MAX_TR_DIM];
    s64 pdcost_coeff0[MAX_TR_DIM];
    static int sig_rate_delta[MAX_TR_DIM];
    static int delta_u[MAX_TR_DIM];
    static s16 coef_dst[MAX_TR_DIM];
    int sum_all = 0;
    int blk_pos;
    static s64 tmp_level_double[MAX_TR_DIM];
    int num_nz = 0;
    int is_last_x = 0;
    int is_last_y = 0;
    int is_last_nz = 0;
    int num_gtA, num_gtB;
    s64 sig_last_cost[MAX_TR_DIM];
    s64 sig_last_cost0[MAX_TR_DIM];
    s64 sig_cost_delta[MAX_TR_DIM];
    int last_pos_in_scan = -1;
    int numNonZeroCoefs = 0;
    int last_pos_in_raster_from_scan = -1;
    int scan_pos = 0;
    q_bits = QUANT_SHIFT + tr_shift + (qp / 6);

    scan = evc_scan_tbl[scan_type][log2_cuw - 1][log2_cuh - 1];
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        int max_abs_level;
        s64 err;
        s64 temp_level;
        int level_double;
        blk_pos = scan[scan_pos];
        temp_level = ((s64)EVC_ABS(src_coef[blk_pos]) * (s64)q_value);
        level_double = (int)EVC_MIN(((s64)temp_level), (s64)EVC_INT32_MAX - (s64)(1 << (q_bits - 1)));
        tmp_level_double[blk_pos] = (s64)level_double;
        max_abs_level = EVC_MIN(MAX_TX_VAL, ((level_double + ((int)1 << (q_bits - 1))) >> q_bits));
        err = (s64)level_double;
        err = (err * err_scale) >> ERR_SCALE_PRECISION_BITS;
        pdcost_coeff0[blk_pos] = err * err;
        dcost_block_uncoded += pdcost_coeff0[blk_pos];
        coef_dst[blk_pos] = (s16)max_abs_level;
        sum_all += max_abs_level;

        if(max_abs_level != 0)
        {
            num_nz++;
            last_pos_in_scan = scan_pos;
            last_pos_in_raster_from_scan = blk_pos;
        }
    }
    if(sum_all == 0)
    {
        evc_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
        return 0;
    }

    last_scan_set = last_pos_in_scan >> cg_log2_size;
    scan_pos_last = last_pos_in_raster_from_scan;
    num_gtA = CAFLAG_NUMBER;
    num_gtB = CBFLAG_NUMBER;
    rice_param = 0;

    ipos = last_pos_in_scan;

    cost_base = dcost_block_uncoded;

    for(sub_set = last_scan_set; sub_set >= 0; sub_set--)
    {
        int sub_pos = sub_set << cg_log2_size;

        c1_idx = 0;
        c2_idx = 0;

        for(; ipos >= sub_pos; ipos--)
        {
            //===== coefficient level estimation =====
            int  level;
            int  ctx_gt0 = 0;
            int  ctx_gtA = 0;
            int  ctx_gtB = 0;

            blk_pos = scan[ipos];
            {
                s64 level_double = tmp_level_double[blk_pos];
                int max_abs_level = coef_dst[blk_pos];
                int bypass_sigmap = blk_pos == scan_pos_last ? 1 : 0;
                int base_level = (c1_idx < num_gtA) ? (2 + (c2_idx < num_gtB ? 1 : 0)) : 1;
                if (sps_cm_init_flag == 1)
                {
                    ctx_gt0 = get_ctx_gt012_inc(coef_dst, blk_pos, width, height, ch_type, &ctx_gtA, &ctx_gtB);
                }

                ctx_gt0 += offset0;
                if (max_abs_level != 0 && is_last_nz == 0)
                {
                    ctx_gtA = 0;
                    ctx_gtB = 0;
                }
                ctx_gtA += offset1;
                ctx_gtB += offset1;
                rice_param = get_rice_para(coef_dst, blk_pos, width, height, base_level);
                level = get_coded_level(&pdcost_coeff[blk_pos], &pdcost_coeff0[blk_pos], &pdcost_sig[blk_pos], level_double, max_abs_level, ctx_gt0, ctx_gtA, ctx_gtB, rice_param,
                                        c1_idx, c2_idx, num_gtA, num_gtB, q_bits, err_scale, lambda, bypass_sigmap);

                sig_rate_delta[blk_pos] = rdoq_est_gt0[ctx_gt0][1] - rdoq_est_gt0[ctx_gt0][0];
                delta_u[blk_pos] = (int)((level_double - (((s64)level) << q_bits)) >> (q_bits - 8));
                sig_cost_delta[blk_pos] = GET_I_COST(sig_rate_delta[blk_pos], lambda);
                sig_last_cost[blk_pos] = GET_I_COST(rdoq_est_gt0[offset0][!!(level)], lambda);
                sig_last_cost0[blk_pos] = GET_I_COST(rdoq_est_gt0[offset0][0], lambda);
                coef_dst[blk_pos] = (s16)level;

                if(level > 0)
                {
                    if (is_last_nz == 0)
                    {
                        is_last_nz = 1;
                    }

                    c1_idx++;
                    if(level > 1)
                    {
                        c2_idx++;
                    }
                }
                else if(max_abs_level)
                {
                    num_nz--;
                    if(num_nz == 0)
                    {
                        evc_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
                        return 0;
                    }
                }
            }
        }
    }

    if(num_nz == 0)
    {
        evc_mset(dst_tmp, 0, sizeof(s16) * max_num_coef);
        return 0;
    }

    {
        s64 in_sr_cost0 = 0;
        s64 in_sr_cost = 0;

        cost_base = 0;

        for (ipos = last_pos_in_scan; ipos >= 0; ipos--)
        {
            blk_pos = scan[ipos];
            in_sr_cost += pdcost_coeff[blk_pos];
            in_sr_cost0 += pdcost_coeff0[blk_pos];
        }

        cost_base = dcost_block_uncoded - in_sr_cost0 + in_sr_cost;
    }

    cost_best = 0;

    if(is_intra == 0 && ch_type == Y_C)
    {
        cost_best = dcost_block_uncoded + GET_I_COST(rdoq_est_all_cbf[0], lambda);
        cbf_cost = GET_I_COST(rdoq_est_all_cbf[1], lambda);
        cost_base += cbf_cost;
    }
    else
    {
        int ctx_qt_cbf = ch_type;
        cost_best = dcost_block_uncoded + GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][0], lambda);
        cbf_cost = GET_I_COST(rdoq_est_cbf[ctx_qt_cbf][1], lambda);
        cost_base += cbf_cost;
    }

    {
        best_last_idx_p1 = 0;
        found_last = 0;
        for (ipos = last_pos_in_scan; ipos >= 0; ipos--)
        {
            blk_pos = scan[ipos];
            if (coef_dst[blk_pos] > 0)
            {
                u32 pos_y = blk_pos >> log2_cuw;
                u32 pos_x = blk_pos - (pos_y << log2_cuw);

                s64 cost_last = get_rate_positionLastXY(pos_x, pos_y, width, height, ch_type, lambda, sps_cm_init_flag);
                s64 total_cost = cost_base + cost_last - pdcost_sig[blk_pos];

                if (total_cost < cost_best)
                {
                    best_last_idx_p1 = ipos + 1;
                    cost_best = total_cost;
                }
                if (coef_dst[blk_pos] > 1)
                {
                    found_last = 1;
                    break;
                }
                cost_base -= pdcost_coeff[blk_pos];
                cost_base += pdcost_coeff0[blk_pos];
            }
            else
            {
                cost_base -= pdcost_sig[blk_pos];
            }

            if (found_last > 0)
            {
                break;
            }
        }
    }

    nnz = 0;
    for (ipos = 0; ipos < best_last_idx_p1; ipos++)
    {
        u32 blk_pos = scan[ipos];
        s16 level = coef_dst[blk_pos];
        dst_tmp[blk_pos] = (src_coef[blk_pos] < 0) ? -level : level;
        nnz += !!(level);
    }

    //===== clean uncoded coefficients =====
    for (ipos = best_last_idx_p1; ipos < max_num_coef; ipos++)
    {
        dst_tmp[scan[ipos]] = 0;
    }
    return nnz;
}

int evce_quant_nnz(u8 qp, double lambda, int is_intra, s16 * coef, int log2_cuw, int log2_cuh, u16 scale, int ch_type, int slice_type, int sps_cm_init_flag, int tool_adcc)
{
    int nnz = 0;

    if(USE_RDOQ)
    {
        s64 lev;
        s64 offset;
        int i;
        int shift;
        int tr_shift;
        int log2_size = (log2_cuw + log2_cuh) >> 1;
        const int ns_shift = ((log2_cuw + log2_cuh) & 1) ? 7 : 0;
        const int ns_scale = ((log2_cuw + log2_cuh) & 1) ? 181 : 1;
        s64 zero_coeff_threshold;
        BOOL is_coded = 0;

        tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - log2_size + ns_shift;
        shift = QUANT_SHIFT + tr_shift + (qp / 6);

#define FAST_RDOQ_INTRA_RND_OFST  201 //171
#define FAST_RDOQ_INTER_RND_OFST  153 //85
        offset = (s64)((slice_type == SLICE_I) ? FAST_RDOQ_INTRA_RND_OFST : FAST_RDOQ_INTER_RND_OFST) << (s64)(shift - 9);
        zero_coeff_threshold = ((s64)1 << (s64)shift) - offset;

        for(i = 0; i < (1 << (log2_cuw + log2_cuh)); i++)
        {
            lev = (s64)EVC_ABS(coef[i]) * (s64)scale * ns_scale;
            if(lev >= zero_coeff_threshold)
            {
                is_coded = 1;
                break;
            }
        }

        if(!is_coded)
        {
            memset(coef, 0, sizeof(coef[0])*((s64)1 << (log2_cuw + log2_cuh)));
            return nnz;
        }
    }

    if(USE_RDOQ)
    {
        if (tool_adcc)
        {
            nnz = ifvce_rdoq_method_ccA(qp, lambda, is_intra, coef, coef, log2_cuw, log2_cuh, ch_type, sps_cm_init_flag);
        }
        else
        {
            nnz = evce_rdoq_run_length_cc(qp, lambda, is_intra, coef, coef, log2_cuw, log2_cuh, ch_type, sps_cm_init_flag);
        }
    }
    else
    {
        s64 lev;
        s64 offset;
        int sign;
        int i;
        int shift;
        int tr_shift;
        int log2_size = (log2_cuw + log2_cuh) >> 1;
        const int ns_shift = ((log2_cuw + log2_cuh) & 1) ? 7 : 0;
        const int ns_scale = ((log2_cuw + log2_cuh) & 1) ? 181 : 1;

        tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - log2_size + ns_shift;
        shift = QUANT_SHIFT + tr_shift + (qp / 6);
        offset = (s64)((slice_type == SLICE_I) ? 171 : 85) << (s64)(shift - 9);

        for(i = 0; i < (1 << (log2_cuw + log2_cuh)); i++)
        {
            sign = EVC_SIGN_GET(coef[i]);
            lev = (s64)EVC_ABS(coef[i]) * (s64)scale;
            lev = (s16)(((s64)lev * ns_scale + offset) >> shift);
            coef[i] = (s16)EVC_SIGN_SET(lev, sign);
            nnz += !!(coef[i]);
        }
    }

    return nnz;
}


int evce_tq_nnz(u8 qp, double lambda, s16 * coef, int log2_cuw, int log2_cuh, u16 scale, int slice_type, int ch_type, int is_intra, int sps_cm_init_flag, int iqt_flag
                , u8 ats_intra_cu, u8 ats_tu, int tool_adcc)
{
    if (ats_intra_cu)
    {
        evce_trans_ats_intra(coef, log2_cuw, log2_cuh, ats_intra_cu, ats_tu);
    }
    else
    {
        evce_trans(coef, log2_cuw, log2_cuh, iqt_flag);
    }

    return evce_quant_nnz(qp, lambda, is_intra, coef, log2_cuw, log2_cuh, scale, ch_type, slice_type, sps_cm_init_flag, tool_adcc);
}

int evce_sub_block_tq(s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 qp_y, u8 qp_u, u8 qp_v, int slice_type, int nnz[N_C]
                      , int nnz_sub[N_C][MAX_SUB_TB_NUM], int is_intra, double lambda_y, double lambda_u, double lambda_v, int run_stats, int sps_cm_init_flag, int iqt_flag
                      , u8 ats_intra_cu, u8 ats_tu, u8 ats_inter_info, int tool_adcc
#if M50761_CHROMA_NOT_SPLIT
                     , TREE_CONS tree_cons
#endif
)
{
#if M50761_CHROMA_NOT_SPLIT
    run_stats = evc_get_run(run_stats, tree_cons);
#endif
    int run[N_C] = {run_stats & 1, (run_stats >> 1) & 1, (run_stats >> 2) & 1};
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
    double lambda[N_C] = { lambda_y , lambda_u , lambda_v };
    int nnz_temp[N_C] = {0};
    evc_mset(nnz_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);
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

                if(run[c])
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

                    int scale = quant_scale[qp[c] % 6];
                    nnz_sub[c][(j << 1) | i] = evce_tq_nnz(qp[c], lambda[c], coef_temp[c], log2_w_sub - !!c, log2_h_sub - !!c, scale, slice_type, c, is_intra, sps_cm_init_flag, iqt_flag
                            , ats_intra_cu_on, ats_tu_mode, tool_adcc);
                    nnz_temp[c] += nnz_sub[c][(j << 1) | i];

                    if(loop_h + loop_w > 2)
                    {
                        evc_block_copy(coef_temp_buf[c], sub_stride >> (!!c), coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
                    }
                }
            }
        }
    }

    for(c = 0; c < N_C; c++)
    {
        nnz[c] = run[c] ? nnz_temp[c] : 0;
    }

    return (nnz[Y_C] + nnz[U_C] + nnz[V_C]);
}
