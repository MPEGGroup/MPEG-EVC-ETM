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
#if USE_RDOQ
static s64 err_scale_tbl[6][MAX_CU_DEPTH];
#else
static double err_scale_tbl[6][MAX_CU_DEPTH];
#endif

extern int rdoq_est_all_cbf[2];
extern int rdoq_est_cbf[NUM_QT_CBF_CTX][2];

extern s32 rdoq_est_run[NUM_SBAC_CTX_RUN][2];
extern s32 rdoq_est_level[NUM_SBAC_CTX_LEVEL][2];
extern s32 rdoq_est_last[NUM_SBAC_CTX_LAST][2];

const int quant_scale[6] = {26214, 23302, 20560, 18396, 16384, 14564};


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
#if TU_ZONAL_CODING
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
#else
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
        *((type_dst *)dst + 32 * line) = (type_dst)((tm[32 * 64 + 0] * EEEEE[0] + tm[32 * 64 + 1] * EEEEE[1] + add) >> shift);\
        *((type_dst *)dst + 48 * line) = (type_dst)((tm[48 * 64 + 0] * EEEEO[0] + tm[48 * 64 + 1] * EEEEO[1] + add) >> shift);\
        \
        for (k = 8; k<64; k += 16)\
        {\
            *((type_dst *)dst + k*line) = (type_dst)((tm[k * 64 + 0] * EEEO[0] + tm[k * 64 + 1] * EEEO[1] + tm[k * 64 + 2] * EEEO[2] + tm[k * 64 + 3] * EEEO[3] + add) >> shift);\
        }\
        for (k = 4; k<64; k += 8)\
        {\
            *((type_dst *)dst + k*line) = (type_dst)((tm[k * 64 + 0] * EEO[0] + tm[k * 64 + 1] * EEO[1] + tm[k * 64 + 2] * EEO[2] + tm[k * 64 + 3] * EEO[3] +\
                tm[k * 64 + 4] * EEO[4] + tm[k * 64 + 5] * EEO[5] + tm[k * 64 + 6] * EEO[6] + tm[k * 64 + 7] * EEO[7] + add) >> shift);\
        }\
        for (k = 2; k<64; k += 4)\
        {\
            *((type_dst *)dst + k*line) = (type_dst)((tm[k * 64 + 0] * EO[0] + tm[k * 64 + 1] * EO[1] + tm[k * 64 + 2] * EO[2] + tm[k * 64 + 3] * EO[3] +\
                tm[k * 64 + 4] * EO[4] + tm[k * 64 + 5] * EO[5] + tm[k * 64 + 6] * EO[6] + tm[k * 64 + 7] * EO[7] +\
                tm[k * 64 + 8] * EO[8] + tm[k * 64 + 9] * EO[9] + tm[k * 64 + 10] * EO[10] + tm[k * 64 + 11] * EO[11] +\
                tm[k * 64 + 12] * EO[12] + tm[k * 64 + 13] * EO[13] + tm[k * 64 + 14] * EO[14] + tm[k * 64 + 15] * EO[15] + add) >> shift);\
        }\
        for (k = 1; k<64; k += 2)\
        {\
            *((type_dst *)dst + k*line) = (type_dst)((tm[k * 64 + 0] * O[0] + tm[k * 64 + 1] * O[1] + tm[k * 64 + 2] * O[2] + tm[k * 64 + 3] * O[3] +\
                tm[k * 64 + 4] * O[4] + tm[k * 64 + 5] * O[5] + tm[k * 64 + 6] * O[6] + tm[k * 64 + 7] * O[7] +\
                tm[k * 64 + 8] * O[8] + tm[k * 64 + 9] * O[9] + tm[k * 64 + 10] * O[10] + tm[k * 64 + 11] * O[11] +\
                tm[k * 64 + 12] * O[12] + tm[k * 64 + 13] * O[13] + tm[k * 64 + 14] * O[14] + tm[k * 64 + 15] * O[15] +\
                tm[k * 64 + 16] * O[16] + tm[k * 64 + 17] * O[17] + tm[k * 64 + 18] * O[18] + tm[k * 64 + 19] * O[19] +\
                tm[k * 64 + 20] * O[20] + tm[k * 64 + 21] * O[21] + tm[k * 64 + 22] * O[22] + tm[k * 64 + 23] * O[23] +\
                tm[k * 64 + 24] * O[24] + tm[k * 64 + 25] * O[25] + tm[k * 64 + 26] * O[26] + tm[k * 64 + 27] * O[27] +\
                tm[k * 64 + 28] * O[28] + tm[k * 64 + 29] * O[29] + tm[k * 64 + 30] * O[30] + tm[k * 64 + 31] * O[31] + add) >> shift);\
        }\
        src = (type_src *)src + tx_size;\
        dst = (type_dst *)dst + 1;\
    }
#endif
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
#if TU_ZONAL_CODING
        dst[32 * line] = 0;
        dst[48 * line] = 0;
#else
        dst[32 * line] = (tm[32 * 64 + 0] * EEEEE[0] + tm[32 * 64 + 1] * EEEEE[1] + add) >> shift;
        dst[48 * line] = (tm[48 * 64 + 0] * EEEEO[0] + tm[48 * 64 + 1] * EEEEO[1] + add) >> shift;
#endif

        for (k = 8; k<64; k += 16)
        {
#if TU_ZONAL_CODING
            if (k > 31)
                dst[k*line] = 0;
            else
#endif
            dst[k*line] = (tm[k * 64 + 0] * EEEO[0] + tm[k * 64 + 1] * EEEO[1] + tm[k * 64 + 2] * EEEO[2] + tm[k * 64 + 3] * EEEO[3] + add) >> shift;
        }
        for (k = 4; k<64; k += 8)
        {
#if TU_ZONAL_CODING
            if (k > 31)
                dst[k*line] = 0;
            else
#endif
            dst[k*line] = (tm[k * 64 + 0] * EEO[0] + tm[k * 64 + 1] * EEO[1] + tm[k * 64 + 2] * EEO[2] + tm[k * 64 + 3] * EEO[3] +
                tm[k * 64 + 4] * EEO[4] + tm[k * 64 + 5] * EEO[5] + tm[k * 64 + 6] * EEO[6] + tm[k * 64 + 7] * EEO[7] + add) >> shift;
        }
        for (k = 2; k<64; k += 4)
        {
#if TU_ZONAL_CODING
            if (k > 31)
                dst[k*line] = 0;
            else
#endif
            dst[k*line] = (tm[k * 64 + 0] * EO[0] + tm[k * 64 + 1] * EO[1] + tm[k * 64 + 2] * EO[2] + tm[k * 64 + 3] * EO[3] +
                tm[k * 64 + 4] * EO[4] + tm[k * 64 + 5] * EO[5] + tm[k * 64 + 6] * EO[6] + tm[k * 64 + 7] * EO[7] +
                tm[k * 64 + 8] * EO[8] + tm[k * 64 + 9] * EO[9] + tm[k * 64 + 10] * EO[10] + tm[k * 64 + 11] * EO[11] +
                tm[k * 64 + 12] * EO[12] + tm[k * 64 + 13] * EO[13] + tm[k * 64 + 14] * EO[14] + tm[k * 64 + 15] * EO[15] + add) >> shift;
        }
        for (k = 1; k<64; k += 2)
        {
#if TU_ZONAL_CODING
            if (k > 31)
                dst[k*line] = 0;
            else
#endif
            dst[k*line] = (tm[k * 64 + 0] * O[0] + tm[k * 64 + 1] * O[1] + tm[k * 64 + 2] * O[2] + tm[k * 64 + 3] * O[3] +
                tm[k * 64 + 4] * O[4] + tm[k * 64 + 5] * O[5] + tm[k * 64 + 6] * O[6] + tm[k * 64 + 7] * O[7] +
                tm[k * 64 + 8] * O[8] + tm[k * 64 + 9] * O[9] + tm[k * 64 + 10] * O[10] + tm[k * 64 + 11] * O[11] +
                tm[k * 64 + 12] * O[12] + tm[k * 64 + 13] * O[13] + tm[k * 64 + 14] * O[14] + tm[k * 64 + 15] * O[15] +
                tm[k * 64 + 16] * O[16] + tm[k * 64 + 17] * O[17] + tm[k * 64 + 18] * O[18] + tm[k * 64 + 19] * O[19] +
                tm[k * 64 + 20] * O[20] + tm[k * 64 + 21] * O[21] + tm[k * 64 + 22] * O[22] + tm[k * 64 + 23] * O[23] +
                tm[k * 64 + 24] * O[24] + tm[k * 64 + 25] * O[25] + tm[k * 64 + 26] * O[26] + tm[k * 64 + 27] * O[27] +
                tm[k * 64 + 28] * O[28] + tm[k * 64 + 29] * O[29] + tm[k * 64 + 30] * O[30] + tm[k * 64 + 31] * O[31] + add) >> shift;
        }
        src += tx_size;
        dst++;
    }
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

#if USE_RDOQ
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
#else
static double get_err_scale(int qp, int log2_size)
{
    return err_scale_tbl[qp][log2_size - 1];
}

void evce_init_err_scale()
{
    double err_scale;
    int qp;
    int i;

    for(qp = 0; qp < 6; qp++)
    {
        int q_value = quant_scale[qp];

        for(i = 0; i < MAX_CU_DEPTH; i++)
        {
            int tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - (i + 1);
            err_scale = (double)(1 << SCALE_BITS) * pow(2.0, -2.0 * tr_shift);
            err_scale = err_scale / q_value / q_value / (1 << (2 * (BIT_DEPTH - 8)));
            err_scale_tbl[qp][i] = err_scale;
        }
    }
}
#endif

#define GET_I_COST(rate, lamba)  (rate*lamba)
#define GET_IEP_RATE             (32768)

#if USE_RDOQ
static __inline s64 get_ic_rate_cost_rl(u32 abs_level, u32 run, s32 ctx_run, u32 ctx_level, s64 lambda)
#else
static __inline double get_ic_rate_cost_rl(u32 abs_level, u32 run, s32 ctx_run, u32 ctx_level, double lambda)
#endif
{
#if USE_RDOQ
    s32 rate;
#else
    double rate;
#endif
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
#if USE_RDOQ
    return (s64)GET_I_COST(rate, lambda);
#else
    return GET_I_COST(rate, lambda);
#endif
}

#if USE_RDOQ
static __inline u32 get_coded_level_rl(s64* rd64_uncoded_cost, s64* rd64_coded_cost, s64 level_double, u32 max_abs_level,
                                       u32 run, u16 ctx_run, u16 ctx_level, s32 q_bits, s64 err_scale, s64 lambda)
#else
static __inline u32 get_coded_level_rl(double* rd64_uncoded_cost, double* rd64_coded_cost, s64 level_double, u32 max_abs_level,
                                       u32 run, u16 ctx_run, u16 ctx_level, s32 q_bits, double err_scale, double lambda)
#endif
{
    u32 best_abs_level = 0;
#if USE_RDOQ
    s64 err1 = (level_double * err_scale) >> ERR_SCALE_PRECISION_BITS;
#else
    double err1 = (double)level_double;
#endif
    u32 min_abs_level;
    u32 abs_level;

#if USE_RDOQ
    *rd64_uncoded_cost = err1 * err1;
#else
    *rd64_uncoded_cost = err1 * err1 * err_scale;
#endif
    *rd64_coded_cost = *rd64_uncoded_cost + get_ic_rate_cost_rl(0, run, ctx_run, ctx_level, lambda);

    min_abs_level = (max_abs_level > 1 ? max_abs_level - 1 : 1);
    for(abs_level = max_abs_level; abs_level >= min_abs_level; abs_level--)
    {
#if USE_RDOQ
        s64 i64Delta = level_double - ((s64)abs_level << q_bits);
        s64 err = (i64Delta * err_scale) >> ERR_SCALE_PRECISION_BITS;
        s64 dCurrCost = err * err + get_ic_rate_cost_rl(abs_level, run, ctx_run, ctx_level, lambda);
#else
        double i64Delta = (double)(level_double - ((s64)abs_level << q_bits));
        double err = (double)(i64Delta);
        double dCurrCost = err * err * err_scale + get_ic_rate_cost_rl(abs_level, run, ctx_run, ctx_level, lambda);
#endif

        if(dCurrCost < *rd64_coded_cost)
        {
            best_abs_level = abs_level;
            *rd64_coded_cost = dCurrCost;
        }
    }
    return best_abs_level;
}

#if USE_RDOQ
int evce_rdoq_run_length_cc(u8 qp, double d_lambda, u8 is_intra, s16 *src_coef, s16 *dst_tmp, int log2_cuw, int log2_cuh, int ch_type, int sps_cm_init_flag
#if AQS
                            , u16 qs_scale
#endif
)
#else
int evce_rdoq_run_length_cc(u8 qp, double lambda, u8 is_intra, s16 *src_coef, s16 *dst_tmp, int log2_cuw, int log2_cuh, int ch_type, int sps_cm_init_flag
#if AQS
                            , u16 qs_scale
#endif
)
#endif
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
#if USE_RDOQ
    const s64 lambda = (s64)(d_lambda * (double)(1 << SCALE_BITS) + 0.5);
    s64 err_scale = err_scale_tbl[qp_rem][log2_size - 1];
    s64 d64_best_cost = 0;
    s64 d64_base_cost = 0;
    s64 d64_coded_cost = 0;
    s64 d64_uncoded_cost = 0;
    s64 d64_block_uncoded_cost = 0;
    s64 err;
#else
    double err_scale = (double)get_err_scale(qp_rem, log2_size);
    double d64_best_cost = 0;
    double d64_base_cost = 0;
    double d64_coded_cost = 0;
    double d64_uncoded_cost = 0;
    double d64_block_uncoded_cost = 0;
    double err;
#endif
#if AQS
    int qs_scale_inv_shift14 = (ch_type == Y_C) ? (((1 << (14 + ESM_SHIFT)) + (qs_scale >> 1)) / qs_scale) : (1 << 14);
#endif
    /* ===== quantization ===== */
    for (scan_pos = 0; scan_pos < max_num_coef; scan_pos++)
    {
        u32 blk_pos = scan[scan_pos];
        s64 level_double = src_coef[blk_pos];
        u32 max_abs_level;
        s8 lower_int;
        s64 temp_level;

        temp_level = ((s64)EVC_ABS(src_coef[blk_pos]) * (s64)q_value);
#if AQS //in rdoq
        temp_level = (temp_level * qs_scale_inv_shift14 + (1 << 13)) >> 14;
#endif
        level_double = (int)EVC_MIN(((s64)temp_level), (s64)EVC_INT32_MAX - (s64)(1 << (q_bits - 1)));
        tmp_level_double[blk_pos] = level_double;
        max_abs_level = (u32)(level_double >> q_bits);
        lower_int = ((level_double - ((s64)max_abs_level << q_bits)) < (s64)(1 << (q_bits - 1))) ? 1 : 0;

        if (!lower_int)
        {
            max_abs_level++;
        }

#if USE_RDOQ
        err = (level_double * err_scale) >> ERR_SCALE_PRECISION_BITS;
        d64_block_uncoded_cost += err * err;
#else
        err = (double)level_double;
        d64_block_uncoded_cost += err * err * err_scale;
#endif
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
#if USE_RDOQ
            s64 d64_cost_last_zero = GET_I_COST(rdoq_est_last[ctx_last][0], lambda);
            s64 d64_cost_last_one = GET_I_COST(rdoq_est_last[ctx_last][1], lambda);
            s64 d64_cur_is_last_cost = d64_base_cost + d64_cost_last_one;
#else
            double d64_cost_last_zero = GET_I_COST(rdoq_est_last[ctx_last][0], lambda);
            double d64_cost_last_one = GET_I_COST(rdoq_est_last[ctx_last][1], lambda);
            double d64_cur_is_last_cost = d64_base_cost + d64_cost_last_one;
#endif
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

int evce_quant_nnz(u8 qp, double lambda, int is_intra, s16 * coef, int log2_cuw, int log2_cuh, u16 scale, int ch_type, int tile_group_type, int sps_cm_init_flag
#if AQS
                   , u16 qs_scale
#endif
)
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
        offset = (s64)((tile_group_type == TILE_GROUP_I) ? FAST_RDOQ_INTRA_RND_OFST : FAST_RDOQ_INTER_RND_OFST) << (s64)(shift - 9);
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
        nnz = evce_rdoq_run_length_cc(qp, lambda, is_intra, coef, coef, log2_cuw, log2_cuh, ch_type
#if AQS
                                      , qs_scale
#endif
                                      , sps_cm_init_flag
        );
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
#if AQS
        int qs_scale_inv_shift14 = (ch_type == Y_C) ? (((1 << (14 + ESM_SHIFT)) + (qs_scale >> 1)) / qs_scale) : (1 << 14);
#endif
        tr_shift = MAX_TX_DYNAMIC_RANGE - BIT_DEPTH - log2_size + ns_shift;
        shift = QUANT_SHIFT + tr_shift + (qp / 6);
        offset = (s64)((tile_group_type == TILE_GROUP_I) ? 171 : 85) << (s64)(shift - 9);

        for(i = 0; i < (1 << (log2_cuw + log2_cuh)); i++)
        {
            sign = EVC_SIGN_GET(coef[i]);
            lev = (s64)EVC_ABS(coef[i]) * (s64)scale;
#if AQS //in normal quantization
            //Note: "offset<<14" may beyond 32 bit, so it shall must convert to s64
            lev = (s16)(((s64)lev * ns_scale * qs_scale_inv_shift14 + ((s64)offset << 14)) >> (shift + 14));
#else
            lev = (s16)(((s64)lev * ns_scale + offset) >> shift);
#endif
            coef[i] = (s16)EVC_SIGN_SET(lev, sign);
            nnz += !!(coef[i]);
        }
    }

    return nnz;
}


int evce_tq_nnz(u8 qp, double lambda, s16 * coef, int log2_cuw, int log2_cuh, u16 scale, int tile_group_type, int ch_type, int is_intra, int sps_cm_init_flag, int iqt_flag
#if AQS
                , u16 qs_scale
#endif
)
{
    evce_trans(coef, log2_cuw, log2_cuh, iqt_flag);

    return evce_quant_nnz(qp, lambda, is_intra, coef, log2_cuw, log2_cuh, scale, ch_type, tile_group_type, sps_cm_init_flag
#if AQS
                          , qs_scale
#endif
    );
}

int evce_sub_block_tq(s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 qp_y, u8 qp_u, u8 qp_v, int tile_group_type, int nnz[N_C]
                      , int nnz_sub[N_C][MAX_SUB_TB_NUM], int is_intra, double lambda_y, double lambda_u, double lambda_v
#if AQS
                      , u16 qs_scale
#endif
                      , int run_stats
                      , int sps_cm_init_flag
                      , int iqt_flag
)
{
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

    for(j = 0; j < loop_h; j++)
    {
        for(i = 0; i < loop_w; i++)
        {
            for(c = 0; c < N_C; c++)
            {
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
                    nnz_sub[c][(j << 1) | i] = evce_tq_nnz(qp[c], lambda[c], coef_temp[c], log2_w_sub - !!c, log2_h_sub - !!c, scale, tile_group_type, c, is_intra
#if AQS
                                                           , qs_scale
#endif
                                                           , sps_cm_init_flag, iqt_flag
                    );
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
