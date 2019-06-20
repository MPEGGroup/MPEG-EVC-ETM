﻿/* The copyright in this software is being made available under the BSD
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

#include "evc_tbl.h"
#include "evc_mc.h"
#include "evc_util.h"
#include <assert.h>

#define MC_FILTER_BASE        0
#define MC_FILTER_MAIN        1

#define MAC_SFT_N0            (6)
#define MAC_ADD_N0            (1<<5)

#define MAC_SFT_0N            MAC_SFT_N0
#define MAC_ADD_0N            MAC_ADD_N0

#define MAC_SFT_NN_S1         (2)
#define MAC_ADD_NN_S1         (0)
#define MAC_SFT_NN_S2         (10)
#define MAC_ADD_NN_S2         (1<<9)

#define MAC_8TAP(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3)+(c)[4]*(r4)+\
    (c)[5]*(r5)+(c)[6]*(r6)+(c)[7]*(r7))
#define MAC_8TAP_N0(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((MAC_8TAP(c, r0, r1, r2, r3, r4, r5, r6, r7) + MAC_ADD_N0) >> MAC_SFT_N0)
#define MAC_8TAP_0N(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((MAC_8TAP(c, r0, r1, r2, r3, r4, r5, r6, r7) + MAC_ADD_0N) >> MAC_SFT_0N)
#define MAC_8TAP_NN_S1(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((MAC_8TAP(c,r0,r1,r2,r3,r4,r5,r6,r7) + MAC_ADD_NN_S1) >> MAC_SFT_NN_S1)
#define MAC_8TAP_NN_S2(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((MAC_8TAP(c,r0,r1,r2,r3,r4,r5,r6,r7) + MAC_ADD_NN_S2) >> MAC_SFT_NN_S2)

#define MAC_4TAP(c, r0, r1, r2, r3) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3))
#define MAC_4TAP_N0(c, r0, r1, r2, r3) \
    ((MAC_4TAP(c, r0, r1, r2, r3) + MAC_ADD_N0) >> MAC_SFT_N0)
#define MAC_4TAP_0N(c, r0, r1, r2, r3) \
    ((MAC_4TAP(c, r0, r1, r2, r3) + MAC_ADD_0N) >> MAC_SFT_0N)
#define MAC_4TAP_NN_S1(c, r0, r1, r2, r3) \
    ((MAC_4TAP(c, r0, r1, r2, r3) + MAC_ADD_NN_S1) >> MAC_SFT_NN_S1)
#define MAC_4TAP_NN_S2(c, r0, r1, r2, r3) \
    ((MAC_4TAP(c, r0, r1, r2, r3) + MAC_ADD_NN_S2) >> MAC_SFT_NN_S2)

#define MAC_BL(c, r0, r1) \
    ((c)[0]*(r0)+(c)[1]*(r1))
#define MAC_BL_N0(c, r0, r1) \
    ((MAC_BL(c, r0, r1) + MAC_ADD_N0) >> MAC_SFT_N0)
#define MAC_BL_0N(c, r0, r1) \
    ((MAC_BL(c, r0, r1) + MAC_ADD_0N) >> MAC_SFT_0N)
#define MAC_BL_NN_S1(c, r0, r1) \
    ((MAC_BL(c, r0, r1) + MAC_ADD_NN_S1) >> MAC_SFT_NN_S1)
#define MAC_BL_NN_S2(c, r0, r1) \
    ((MAC_BL(c, r0, r1) + MAC_ADD_NN_S2) >> MAC_SFT_NN_S2)

/* padding for store intermediate values, which should be larger than
1+ half of filter tap */
#define MC_IBUF_PAD_C          4
#define MC_IBUF_PAD_L          8
#define MC_IBUF_PAD_BL         2

static int g_mc_ftr = MC_FILTER_MAIN;

#if MC_PRECISION_ADD
#if OPT_SIMD_MC_BL
static const s16 tbl_bl_mc_l_coeff[4 << MC_PRECISION_ADD][2] =
#else
static const s8 tbl_bl_mc_l_coeff[4 << MC_PRECISION_ADD][2] =
#endif
#else
#if OPT_SIMD_MC_BL
static const s16 tbl_bl_mc_l_coeff[4][2] =
#else
static const s8 tbl_bl_mc_l_coeff[4][2] =
#endif
#endif
{
{ 64,  0 },
#if MC_PRECISION_ADD
{ 60,  4 },
{ 56,  8 },
{ 52, 12 },
#endif
{ 48, 16 },
#if MC_PRECISION_ADD
{ 44, 20 },
{ 40, 24 },
{ 36, 28 },
#endif
{ 32, 32 },
#if MC_PRECISION_ADD
{ 28, 36 },
{ 24, 40 },
{ 20, 44 },
#endif
{ 16, 48 },
#if MC_PRECISION_ADD
{ 12, 52 },
{ 8,  56 },
{ 4,  60 }
#endif
};

#ifdef X86_SSE
#define SSE_MC_FILTER_L_8PEL(src, s_src, dst, add, shift, coef, clip, min, max)\
{ \
    __m128i mac1, mac2, filt, mt1, mt2, mt3; \
    mac1 = mac2 = _mm_set1_epi32(add);    \
\
    mt1 = _mm_loadu_si128((__m128i*)(src));\
    mt2 = _mm_mullo_epi16(mt1, coef[0]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[0]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)));\
    mt2 = _mm_mullo_epi16(mt1, coef[1]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[1]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*2));\
    mt2 = _mm_mullo_epi16(mt1, coef[2]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[2]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*3));\
    mt2 = _mm_mullo_epi16(mt1, coef[3]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[3]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*4));\
    mt2 = _mm_mullo_epi16(mt1, coef[4]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[4]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*5));\
    mt2 = _mm_mullo_epi16(mt1, coef[5]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[5]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*6));\
    mt2 = _mm_mullo_epi16(mt1, coef[6]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[6]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*7));\
    mt2 = _mm_mullo_epi16(mt1, coef[7]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[7]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mac1 = _mm_srai_epi32(mac1, shift);\
    mac2 = _mm_srai_epi32(mac2, shift);\
    filt = _mm_packs_epi32(mac2, mac1);\
\
    if(clip){\
    mt1 = _mm_cmplt_epi16(filt, max); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, max));\
    mt1 = _mm_cmpgt_epi16(filt, min); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, min));\
    }\
\
    _mm_storeu_si128((__m128i*)(dst), filt);\
}

#define SSE_MC_FILTER_L_4PEL(src, s_src, dst, add, shift, coef, clip, min, max)\
{ \
    __m128i mac1, mac2, filt, mt1, mt2, mt3; \
    mac1 = mac2 = _mm_set1_epi32(add);    \
\
    mt1 = _mm_loadl_epi64((__m128i*)(src));\
    mt2 = _mm_mullo_epi16(mt1, coef[0]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[0]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)));\
    mt2 = _mm_mullo_epi16(mt1, coef[1]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[1]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*2));\
    mt2 = _mm_mullo_epi16(mt1, coef[2]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[2]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*3));\
    mt2 = _mm_mullo_epi16(mt1, coef[3]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[3]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*4));\
    mt2 = _mm_mullo_epi16(mt1, coef[4]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[4]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*5));\
    mt2 = _mm_mullo_epi16(mt1, coef[5]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[5]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*6));\
    mt2 = _mm_mullo_epi16(mt1, coef[6]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[6]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*7));\
    mt2 = _mm_mullo_epi16(mt1, coef[7]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[7]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mac1 = _mm_srai_epi32(mac1, shift);\
    mac2 = _mm_srai_epi32(mac2, shift);\
    filt = _mm_packs_epi32(mac2, mac1);\
\
    if(clip){\
    mt1 = _mm_cmplt_epi16(filt, max); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, max));\
    mt1 = _mm_cmpgt_epi16(filt, min); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, min));\
    }\
\
    _mm_storel_epi64((__m128i*)(dst), filt);\
}

#define SSE_MC_FILTER_L_8PEL_BILIN(src, s_src, dst, add, shift, coef, clip, min, max)\
{ \
    __m128i mac1, mac2, filt, mt1, mt2, mt3; \
    mac1 = mac2 = _mm_set1_epi32(add);    \
\
    mt1 = _mm_loadu_si128((__m128i*)(src));\
    mt2 = _mm_mullo_epi16(mt1, coef[0]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[0]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)));\
    mt2 = _mm_mullo_epi16(mt1, coef[1]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[1]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mac1 = _mm_srai_epi32(mac1, shift);\
    mac2 = _mm_srai_epi32(mac2, shift);\
    filt = _mm_packs_epi32(mac2, mac1);\
\
    if(clip){\
    mt1 = _mm_cmplt_epi16(filt, max); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, max));\
    mt1 = _mm_cmpgt_epi16(filt, min); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, min));\
    }\
\
    _mm_storeu_si128((__m128i*)(dst), filt);\
}

#define SSE_MC_FILTER_L_4PEL_BILIN(src, s_src, dst, add, shift, coef, clip, min, max)\
{ \
    __m128i mac1, mac2, filt, mt1, mt2, mt3; \
    mac1 = mac2 = _mm_set1_epi32(add);    \
\
    mt1 = _mm_loadl_epi64((__m128i*)(src));\
    mt2 = _mm_mullo_epi16(mt1, coef[0]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[0]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)));\
    mt2 = _mm_mullo_epi16(mt1, coef[1]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[1]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mac1 = _mm_srai_epi32(mac1, shift);\
    mac2 = _mm_srai_epi32(mac2, shift);\
    filt = _mm_packs_epi32(mac2, mac1);\
\
    if(clip){\
    mt1 = _mm_cmplt_epi16(filt, max); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, max));\
    mt1 = _mm_cmpgt_epi16(filt, min); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, min));\
    }\
\
    _mm_storel_epi64((__m128i*)(dst), filt);\
}

#define SSE_MC_FILTER_C_8PEL(src, s_src, dst, add, shift, coef, clip, min, max)\
{ \
    __m128i mac1, mac2, filt, mt1, mt2, mt3; \
    mac1 = mac2 = _mm_set1_epi32(add);  \
\
    mt1 = _mm_loadu_si128((__m128i*)(src));\
    mt2 = _mm_mullo_epi16(mt1, coef[0]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[0]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)));\
    mt2 = _mm_mullo_epi16(mt1, coef[1]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[1]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*2));\
    mt2 = _mm_mullo_epi16(mt1, coef[2]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[2]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadu_si128((__m128i*)((src) + (s_src)*3));\
    mt2 = _mm_mullo_epi16(mt1, coef[3]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[3]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mac1 = _mm_srai_epi32(mac1, shift);\
    mac2 = _mm_srai_epi32(mac2, shift);\
    filt = _mm_packs_epi32(mac2, mac1);\
\
    if(clip){\
    mt1 = _mm_cmplt_epi16(filt, max); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, max));\
    mt1 = _mm_cmpgt_epi16(filt, min); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, min));\
    }\
\
    _mm_storeu_si128((__m128i*)(dst), filt);\
}

#define SSE_MC_FILTER_C_4PEL(src, s_src, dst, add, shift, coef, clip, min, max)\
{ \
    __m128i mac1, mac2, filt, mt1, mt2, mt3; \
    mac1 = mac2 = _mm_set1_epi32(add);  \
\
    mt1 = _mm_loadl_epi64((__m128i*)(src));\
    mt2 = _mm_mullo_epi16(mt1, coef[0]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[0]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)));\
    mt2 = _mm_mullo_epi16(mt1, coef[1]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[1]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*2));\
    mt2 = _mm_mullo_epi16(mt1, coef[2]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[2]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mt1 = _mm_loadl_epi64((__m128i*)((src) + (s_src)*3));\
    mt2 = _mm_mullo_epi16(mt1, coef[3]);\
    mt3 = _mm_mulhi_epi16(mt1, coef[3]);\
    mac1 = _mm_add_epi32(mac1, _mm_unpackhi_epi16(mt2, mt3));\
    mac2 = _mm_add_epi32(mac2, _mm_unpacklo_epi16(mt2, mt3));\
\
    mac1 = _mm_srai_epi32(mac1, shift);\
    mac2 = _mm_srai_epi32(mac2, shift);\
    filt = _mm_packs_epi32(mac2, mac1);\
\
    if(clip){ \
    mt1 = _mm_cmplt_epi16(filt, max); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, max));\
    mt1 = _mm_cmpgt_epi16(filt, min); \
    filt = _mm_or_si128( _mm_and_si128(mt1, filt), _mm_andnot_si128(mt1, min));\
    }\
\
    _mm_storel_epi64((__m128i*)(dst), filt);\
}
#endif

/****************************************************************************
 * motion compensation for luma
 ****************************************************************************/

#if EIF
static const s8 tbl_mc_l_coeff_ds[2][5] =
{
#if EIF_3TAP
  { -8, 0, 80, 0, -8 },
#else
  { -12, 18, 52, 18, -12 },
#endif
};
#endif

#if OPT_SIMD_MC_BL
static const s8 shuffle_2Tap[16] = {0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9};

void mc_filter_bilin_horz_sse(s16 const *ref,
                              int src_stride,
                              s16 *pred,
                              int dst_stride,
                              const short *coeff,
                              int width,
                              int height,
                              int min_val,
                              int max_val,
                              int offset,
                              int shift,
                              s8  is_last)
{
    int row, col, rem_w, rem_h;
    int src_stride2, src_stride3;
    s16 const *inp_copy;
    s16 *dst_copy;

    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16(min_val);
    __m128i mm_max = _mm_set1_epi16(max_val);

    __m128i row1, row11, row2, row22, row3, row33, row4, row44;
    __m128i res0, res1, res2, res3;
    __m128i coeff0_1_8x16b, shuffle;

    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    src_stride2 = (src_stride << 1);
    src_stride3 = (src_stride * 3);

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadl_epi64((__m128i*)coeff);      /*w0 w1 x x x x x x*/
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);  /*w0 w1 w0 w1 w0 w1 w0 w1*/

    shuffle = _mm_loadu_si128((__m128i*)shuffle_2Tap);

    rem_h = (height & 0x3);

    if(rem_w > 7)
    {
        for(row = height; row > 3; row -= 4)
        {
            int cnt = 0;
            for(col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                             /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));                        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt));       /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row22 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt + 1));  /*b1 b2 b3 b4 b5 b6 b7 b8*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                row33 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 1));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                row44 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 1));

                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);            /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);          /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
                row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
                row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
                row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);

                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row2 = _mm_add_epi32(row2, offset_4x32b);
                row22 = _mm_add_epi32(row22, offset_4x32b);
                row3 = _mm_add_epi32(row3, offset_4x32b);
                row33 = _mm_add_epi32(row33, offset_4x32b);
                row4 = _mm_add_epi32(row4, offset_4x32b);
                row44 = _mm_add_epi32(row44, offset_4x32b);

                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                row2 = _mm_srai_epi32(row2, shift);
                row22 = _mm_srai_epi32(row22, shift);
                row3 = _mm_srai_epi32(row3, shift);
                row33 = _mm_srai_epi32(row33, shift);
                row4 = _mm_srai_epi32(row4, shift);
                row44 = _mm_srai_epi32(row44, shift);

                row1 = _mm_packs_epi32(row1, row2);
                row11 = _mm_packs_epi32(row11, row22);
                row3 = _mm_packs_epi32(row3, row4);
                row33 = _mm_packs_epi32(row33, row44);

                res0 = _mm_unpacklo_epi16(row1, row11);
                res1 = _mm_unpackhi_epi16(row1, row11);
                res2 = _mm_unpacklo_epi16(row3, row33);
                res3 = _mm_unpackhi_epi16(row3, row33);

                if(is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res1 = _mm_min_epi16(res1, mm_max);
                    res2 = _mm_min_epi16(res2, mm_max);
                    res3 = _mm_min_epi16(res3, mm_max);

                    res0 = _mm_max_epi16(res0, mm_min);
                    res1 = _mm_max_epi16(res1, mm_min);
                    res2 = _mm_max_epi16(res2, mm_min);
                    res3 = _mm_max_epi16(res3, mm_min);
                }

                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 2 + cnt), res2);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 3 + cnt), res3);

                cnt += 8; /* To pointer updates*/
            }

            inp_copy += (src_stride << 2);
            dst_copy += (dst_stride << 2);
        }

        /*extra height to be done --- one row at a time*/
        for(row = 0; row < rem_h; row++)
        {
            int cnt = 0;
            for(col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));       /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));  /*a1 a2 a3 a4 a5 a6 a7 a8*/

                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);              /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);            /*a1+a2 a3+a4 a5+a6 a7+a8*/

                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);

                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);

                row1 = _mm_packs_epi32(row1, row11);    /*a0 a2 a4 a6 a1 a3 a5 a7*/

                res0 = _mm_unpackhi_epi64(row1, row1);  /*a1 a3 a5 a7*/
                res1 = _mm_unpacklo_epi16(row1, res0);  /*a0 a1 a2 a3 a4 a5 a6 a7*/

                if(is_last)
                {
                    res1 = _mm_min_epi16(res1, mm_max);
                    res1 = _mm_max_epi16(res1, mm_min);
                }

                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res1);

                cnt += 8;
            }

            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x7;

    if(rem_w > 3)
    {
        inp_copy = ref + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);

        for(row = height; row > 3; row -= 4)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadu_si128((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
            row2 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride));  /*a1 a2 a3 a4 a5 a6 a7 a8*/
            row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2));
            row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3));

            row1 = _mm_shuffle_epi8(row1, shuffle);  /*a0 a1 a1 a2 a2 a3 a3 a4 */
            row2 = _mm_shuffle_epi8(row2, shuffle);
            row3 = _mm_shuffle_epi8(row3, shuffle);
            row4 = _mm_shuffle_epi8(row4, shuffle);

            row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
            row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
            row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);

            row1 = _mm_add_epi32(row1, offset_4x32b);
            row2 = _mm_add_epi32(row2, offset_4x32b);
            row3 = _mm_add_epi32(row3, offset_4x32b);
            row4 = _mm_add_epi32(row4, offset_4x32b);

            row1 = _mm_srai_epi32(row1, shift);
            row2 = _mm_srai_epi32(row2, shift);
            row3 = _mm_srai_epi32(row3, shift);
            row4 = _mm_srai_epi32(row4, shift);

            res0 = _mm_packs_epi32(row1, row2);
            res1 = _mm_packs_epi32(row3, row4);

            if(is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res1 = _mm_min_epi16(res1, mm_max);

                res0 = _mm_max_epi16(res0, mm_min);
                res1 = _mm_max_epi16(res1, mm_min);
            }

            /* to store the 8 pixels res. */
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 2), res1);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 3), _mm_unpackhi_epi64(res1, res1));

            inp_copy += (src_stride << 2);
            dst_copy += (dst_stride << 2);
        }

        for(row = 0; row < rem_h; row++)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadu_si128((__m128i*)(inp_copy));  /*a0 a1 a2 a3 a4 a5 a6 a7*/

            res0 = _mm_shuffle_epi8(row1, shuffle);        /*a0 a1 a1 a2 a2 a3 a3 a4 */
            res0 = _mm_madd_epi16(res0, coeff0_1_8x16b);   /*a0+a1 a1+a2 a2+a3 a3+a4*/
            res0 = _mm_add_epi32(res0, offset_4x32b);
            res0 = _mm_srai_epi32(res0, shift);
            res0 = _mm_packs_epi32(res0, res0);

            if(is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
            }

            _mm_storel_epi64((__m128i *)(dst_copy), res0);

            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x3;

    if(rem_w)
    {
        int sum, sum1;

        inp_copy = ref + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);

        for(row = height; row > 3; row -= 4)
        {
            for(col = 0; col < rem_w; col++)
            {
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + col));                        /*a0 a1 x x x x x x*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + col));  /*b0 b1 x x x x x x*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + col));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + col));

                row1 = _mm_unpacklo_epi32(row1, row2);  /*a0 a1 b0 b1*/
                row3 = _mm_unpacklo_epi32(row3, row4);  /*c0 c1 d0 d1*/
                row1 = _mm_unpacklo_epi64(row1, row3);  /*a0 a1 b0 b1 c0 c1 d0 d1*/

                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);  /*a0+a1 b0+b1 c0+c1 d0+d1*/

                row1 = _mm_add_epi32(row1, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                res0 = _mm_packs_epi32(row1, row1);

                if(is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                }

                /*extract 32 bit integer form register and store it in dst_copy*/
                sum = _mm_extract_epi32(res0, 0);
                sum1 = _mm_extract_epi32(res0, 1);

                dst_copy[col] = (s16)(sum & 0xffff);
                dst_copy[col + dst_stride] = (s16)(sum >> 16);
                dst_copy[col + (dst_stride << 1)] = (s16)(sum1 & 0xffff);
                dst_copy[col + (dst_stride * 3)] = (s16)(sum1 >> 16);
            }
            inp_copy += (src_stride << 2);
            dst_copy += (dst_stride << 2);
        }

        for(row = 0; row < rem_h; row++)
        {
            for(col = 0; col < rem_w; col++)
            {
                s16 val;
                int sum;

                sum = inp_copy[col + 0] * coeff[0];
                sum += inp_copy[col + 1] * coeff[1];

                val = (sum + offset) >> shift;
                dst_copy[col] = (is_last ? (EVC_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += src_stride;
            dst_copy += dst_stride;
        }
    }
}

void mc_filter_bilin_vert_sse(s16 const *ref,
                              int src_stride,
                              s16 *pred,
                              int dst_stride,
                              const short *coeff,
                              int width,
                              int height,
                              int min_val,
                              int max_val,
                              int offset,
                              int shift,
                              s8  is_last)
{
    int row, col, rem_w, rem_h;
    int src_stride2, src_stride3, src_stride4;
    s16 const *inp_copy;
    s16 *dst_copy;

    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16(min_val);
    __m128i mm_max = _mm_set1_epi16(max_val);

    __m128i row1, row11, row2, row22, row3, row33, row4, row44, row5;
    __m128i res0, res1, res2, res3;
    __m128i coeff0_1_8x16b;

    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    src_stride2 = (src_stride << 1);
    src_stride3 = (src_stride * 3);
    src_stride4 = (src_stride << 2);

    coeff0_1_8x16b = _mm_loadl_epi64((__m128i*)coeff);      /*w0 w1 x x x x x x*/
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);  /*w0 w1 w0 w1 w0 w1 w0 w1*/

    rem_h = height & 0x3;

    if (rem_w > 7)
    {
        for (row = height; row > 3; row -= 4)
        {
            int cnt = 0;
            for (col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt));  /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                row5 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride4 + cnt));

                row11 = _mm_unpacklo_epi16(row1, row2);   /*a0 b0 a1 b1 a2 b2 a3 b3*/
                row1 = _mm_unpackhi_epi16(row1, row2);    /*a4 b4 a5 b5 a6 b6 a7 b7*/
                row22 = _mm_unpacklo_epi16(row2, row3);
                row2 = _mm_unpackhi_epi16(row2, row3);
                row33 = _mm_unpacklo_epi16(row3, row4);
                row3 = _mm_unpackhi_epi16(row3, row4);
                row44 = _mm_unpacklo_epi16(row4, row5);
                row4 = _mm_unpackhi_epi16(row4, row5);

                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);    /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
                row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
                row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
                row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);

                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row22 = _mm_add_epi32(row22, offset_4x32b);
                row2 = _mm_add_epi32(row2, offset_4x32b);
                row33 = _mm_add_epi32(row33, offset_4x32b);
                row3 = _mm_add_epi32(row3, offset_4x32b);
                row44 = _mm_add_epi32(row44, offset_4x32b);
                row4 = _mm_add_epi32(row4, offset_4x32b);

                row11 = _mm_srai_epi32(row11, shift);
                row1 = _mm_srai_epi32(row1, shift);
                row22 = _mm_srai_epi32(row22, shift);
                row2 = _mm_srai_epi32(row2, shift);
                row33 = _mm_srai_epi32(row33, shift);
                row3 = _mm_srai_epi32(row3, shift);
                row44 = _mm_srai_epi32(row44, shift);
                row4 = _mm_srai_epi32(row4, shift);

                res0 = _mm_packs_epi32(row11, row1);
                res1 = _mm_packs_epi32(row22, row2);
                res2 = _mm_packs_epi32(row33, row3);
                res3 = _mm_packs_epi32(row44, row4);

                if (is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res1 = _mm_min_epi16(res1, mm_max);
                    res2 = _mm_min_epi16(res2, mm_max);
                    res3 = _mm_min_epi16(res3, mm_max);

                    res0 = _mm_max_epi16(res0, mm_min);
                    res1 = _mm_max_epi16(res1, mm_min);
                    res2 = _mm_max_epi16(res2, mm_min);
                    res3 = _mm_max_epi16(res3, mm_min);
                }

                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 2 + cnt), res2);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 3 + cnt), res3);

                cnt += 8;  /* To pointer updates*/
            }

            inp_copy += (src_stride << 2);
            dst_copy += (dst_stride << 2);
        }

        /*extra height to be done --- one row at a time*/
        for (row = 0; row < rem_h; row++)
        {
            int cnt = 0;
            for (col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt));  /*b0 b1 b2 b3 b4 b5 b6 b7*/

                row11 = _mm_unpacklo_epi16(row1, row2);  /*a0 b0 a1 b1 a2 b2 a3 b3*/
                row1 = _mm_unpackhi_epi16(row1, row2);   /*a4 b4 a5 b5 a6 b6 a7 b7*/

                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a1+a2 a3+a4 a5+a6 a7+a8*/

                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);

                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);

                res1 = _mm_packs_epi32(row11, row1);

                if (is_last)
                {
                    res1 = _mm_min_epi16(res1, mm_max);
                    res1 = _mm_max_epi16(res1, mm_min);
                }

                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res1);

                cnt += 8;
            }

            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x7;

    if (rem_w > 3)
    {
        inp_copy = ref + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);

        for (row = height; row > 3; row -= 4)
        {
            /*load 4 pixel values */
            row1 = _mm_loadl_epi64((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 x x x x*/
            row2 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride));  /*b0 b1 b2 b3 x x x x*/
            row3 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2));
            row4 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3));
            row5 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride4));

            row11 = _mm_unpacklo_epi16(row1, row2);  /*a0 b0 a1 b1 a2 b2 a3 b3*/
            row22 = _mm_unpacklo_epi16(row2, row3);
            row33 = _mm_unpacklo_epi16(row3, row4);
            row44 = _mm_unpacklo_epi16(row4, row5);

            row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
            row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
            row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);

            row11 = _mm_add_epi32(row11, offset_4x32b);
            row22 = _mm_add_epi32(row22, offset_4x32b);
            row33 = _mm_add_epi32(row33, offset_4x32b);
            row44 = _mm_add_epi32(row44, offset_4x32b);

            row11 = _mm_srai_epi32(row11, shift);
            row22 = _mm_srai_epi32(row22, shift);
            row33 = _mm_srai_epi32(row33, shift);
            row44 = _mm_srai_epi32(row44, shift);

            res0 = _mm_packs_epi32(row11, row22);
            res1 = _mm_packs_epi32(row33, row44);

            if (is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res1 = _mm_min_epi16(res1, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
                res1 = _mm_max_epi16(res1, mm_min);
            }

            /* to store the 8 pixels res. */
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 2), res1);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 3), _mm_unpackhi_epi64(res1, res1));

            inp_copy += (src_stride << 2);
            dst_copy += (dst_stride << 2);
        }

        for (row = 0; row < rem_h; row++)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadl_epi64((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 x x x x*/
            row2 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride));  /*b0 b1 b2 b3 x x x x*/

            row11 = _mm_unpacklo_epi16(row1, row2);         /*a0 b0 a1 b1 a2 b2 a3 b3*/
            row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row11 = _mm_add_epi32(row11, offset_4x32b);
            row11 = _mm_srai_epi32(row11, shift);
            row11 = _mm_packs_epi32(row11, row11);

            if (is_last)
            {
                row11 = _mm_min_epi16(row11, mm_max);
                row11 = _mm_max_epi16(row11, mm_min);
            }

            _mm_storel_epi64((__m128i *)(dst_copy), row11);

            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x3;

    if (rem_w)
    {
        inp_copy = ref + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);

        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                s16 val;
                int sum;

                sum = inp_copy[col + 0 * src_stride] * coeff[0];
                sum += inp_copy[col + 1 * src_stride] * coeff[1];

                val = (sum + offset) >> shift;
                dst_copy[col] = (is_last ? (EVC_CLIP3(min_val, max_val, val)) : val);
            }

            inp_copy += src_stride;
            dst_copy += dst_stride;
        }
    }
}
#endif

#if MC_PRECISION_ADD
#if OPT_SIMD_MC_L
s16 tbl_mc_l_coeff[2][4 << MC_PRECISION_ADD][8] =
#else
static const s8 tbl_mc_l_coeff[2][4 << MC_PRECISION_ADD][8] =
#endif
{
    {
        {  0, 0,   0, 64,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 1,  -5, 52, 20,  -5,  1,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 2, -10, 40, 40, -10,  2,  0 }, 
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 1,  -5, 20, 52,  -5,  1,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
        {  0, 0,   0,  0,  0,   0,  0,  0 },
    },
    {
        {  0, 0,   0, 64,  0,   0,  0,  0 },
        {  0, 1,  -3, 63,  4,  -2,  1,  0 },
        { -1, 2,  -5, 62,  8,  -3,  1,  0 },
        { -1, 3,  -8, 60, 13,  -4,  1,  0 },
        { -1, 3,  -9, 57, 19,  -7,  3, -1 },
        { -1, 4, -11, 52, 26,  -8,  3, -1 },
        { -1, 3,  -9, 47, 31, -10,  4, -1 },
        { -1, 4, -11, 45, 34, -10,  4, -1 },
        { -1, 4, -10, 39, 39, -10,  4, -1 },
        { -1, 4, -10, 34, 45, -11,  4, -1 },
        { -1, 4, -10, 31, 47,  -9,  3, -1 },
        { -1, 3,  -8, 26, 52, -11,  4, -1 },
        { -1, 3, -7,  19, 57,  -9,  3, -1 },
        {  0, 1,  -4, 13, 60,  -8,  3, -1 },
        {  0, 1,  -3,  8, 62,  -5,  2, -1 },
        {  0, 1,  -2,  4, 63,  -3,  1,  0 },
    },
};
#else
#if OPT_SIMD_MC_L
static const s16 tbl_mc_l_coeff[2][4][8] =
#else
static const s8 tbl_mc_l_coeff[2][4][8] =
#endif
{
    {
        { 0, 0,   0, 64,  0,   0, 0, 0 },
        { 0, 1,  -5, 52, 20,  -5, 1, 0 },
        { 0, 2, -10, 40, 40, -10, 2, 0 },
        { 0, 1,  -5, 20, 52,  -5, 1, 0 },
    },
    {
        { 0,  0,  0, 64,  0,  0,  0,  0},
        {-1,  3, -9, 57, 19, -7,  3, -1},
        {-1,  4,-10, 39, 39,-10,  4, -1},
        {-1,  3, -7, 19, 57, -9,  3, -1},
    },
};
#endif

#if OPT_SIMD_MC_L
void average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, int s_src, int s_ref, int s_dst, int wd, int ht)
{
    s16 *p0, *p1, *p2;
    int rem_h = ht;
    int rem_w;
    int i, j;

    __m128i src_8x16b, src_8x16b_1, src_8x16b_2, src_8x16b_3;
    __m128i pred_8x16b, pred_8x16b_1, pred_8x16b_2, pred_8x16b_3;
    __m128i temp_0, temp_1, temp_2, temp_3;
    __m128i offset_8x16b;

    /* Can be changed for a generic avg fun. or taken as an argument! */
    int offset = 1;
    int shift = 1;

    assert(BIT_DEPTH <= 14);

    p0 = src;
    p1 = ref;
    p2 = dst;

    offset_8x16b = _mm_set1_epi16(offset);

    /* Mult. of 4 Loop */
    if (rem_h >= 4)
    {
        for (i = 0; i < rem_h; i += 4)
        {
            p0 = src + (i * s_src);
            p1 = ref + (i * s_ref);
            p2 = dst + (i * s_dst);

            rem_w = wd;

            /* Mult. of 8 Loop */
            if (rem_w >= 8)
            {
                for (j = 0; j < rem_w; j += 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    src_8x16b_2 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 2)));
                    src_8x16b_3 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 3)));

                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    pred_8x16b_2 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 2)));
                    pred_8x16b_3 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 3)));

                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                    temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);

                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                    temp_3 = _mm_add_epi16(temp_3, offset_8x16b);

                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    temp_2 = _mm_srai_epi16(temp_2, shift);
                    temp_3 = _mm_srai_epi16(temp_3, shift);

                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    _mm_storeu_si128((__m128i *)(p2 + 2 * s_dst), temp_2);
                    _mm_storeu_si128((__m128i *)(p2 + 3 * s_dst), temp_3);

                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }

            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w >= 4)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                src_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 2)));
                src_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 3)));

                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                pred_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 2)));
                pred_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 3)));

                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);

                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                temp_3 = _mm_add_epi16(temp_3, offset_8x16b);

                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                temp_2 = _mm_srai_epi16(temp_2, shift);
                temp_3 = _mm_srai_epi16(temp_3, shift);

                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                _mm_storel_epi64((__m128i *)(p2 + 2 * s_dst), temp_2);
                _mm_storel_epi64((__m128i *)(p2 + 3 * s_dst), temp_3);

                p0 += 4;
                p1 += 4;
                p2 += 4;
            }

            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] + p1[j + 0 * s_ref] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] + p1[j + 1 * s_ref] + offset) >> shift;
                    p2[j + 2 * s_dst] = (p0[j + 2 * s_src] + p1[j + 2 * s_ref] + offset) >> shift;
                    p2[j + 3 * s_dst] = (p0[j + 3 * s_src] + p1[j + 3 * s_ref] + offset) >> shift;
                }
            }
        }
    }

    /* Remaining rows */
    rem_h &= 0x3;

    /* One 2 row case */
    if (rem_h >= 2)
    {
        p0 = src + ((ht >> 2) << 2) * s_src;
        p1 = ref + ((ht >> 2) << 2) * s_ref;
        p2 = dst + ((ht >> 2) << 2) * s_dst;

        /* One 2 row case */
        {
            rem_w = wd;

            /* Mult. of 8 Loop */
            if (rem_w >= 8)
            {
                for (j = 0; j < rem_w; j += 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));

                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));

                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);

                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);

                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);

                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);

                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }

            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w >= 4)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));

                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));

                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);

                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);

                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);

                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);

                p0 += 4;
                p1 += 4;
                p2 += 4;
            }

            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] + p1[j + 0 * s_ref] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] + p1[j + 1 * s_ref] + offset) >> shift;
                }
            }
        }
    }

    /* Remaining 1 row */
    if (rem_h &= 0x1)
    {
        p0 = src + ((ht >> 1) << 1) * s_src;
        p1 = ref + ((ht >> 1) << 1) * s_ref;
        p2 = dst + ((ht >> 1) << 1) * s_dst;

        /* One 1 row case */
        {
            rem_w = wd;

            /* Mult. of 8 Loop */
            if (rem_w >= 8)
            {
                for (j = 0; j < rem_w; j += 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));

                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));

                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);

                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);

                    temp_0 = _mm_srai_epi16(temp_0, shift);

                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);

                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }

            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w >= 4)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));

                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));

                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);

                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);

                temp_0 = _mm_srai_epi16(temp_0, shift);

                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);

                p0 += 4;
                p1 += 4;
                p2 += 4;
            }

            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j] = (p0[j] + p1[j] + offset) >> shift;
                }
            }
        }
    }
}

static void mc_filter_l_8pel_horz_clip_sse(s16 *ref,
                                           int src_stride,
                                           s16 *pred,
                                           int dst_stride,
                                           const s16 *coeff,
                                           int width,
                                           int height,
                                           int min_val,
                                           int max_val,
                                           int offset,
                                           int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;

    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i    mm_min = _mm_set1_epi16(min_val);
    __m128i mm_max = _mm_set1_epi16(max_val);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;

    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);

    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);

    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));

                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));

                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));

                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));

                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);

                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    //if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }

                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);

                    cnt += 8; /* To pointer updates*/
                }

                inp_copy += src_stride; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }

        rem_w &= 0x7;

        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);

            for (row = 0; row < height; row += 2)
            {
                /*load 8 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride));    /* row = 1 */

                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));

                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));

                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));

                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 1));

                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 3));

                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 5));

                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 7));

                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);

                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);

                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);

                //if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);

                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }

                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (src_stride << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }

        rem_w &= 0x3;

        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;

            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);

            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7

            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + col));

                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);

                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);

                    //if (is_last)
                    {
                        sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                        sum1 = (sum1 < min_val) ? min_val : (sum1>max_val ? max_val : sum1);
                    }
                    dst_copy[col] = (sum);
                    dst_copy[col + dst_stride] = (sum1);
                }
                inp_copy += (src_stride << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));

                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    /* row = 0 */
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));

                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));

                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));

                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);

                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    //if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }

                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);

                    cnt += 8; /* To pointer updates*/
                }

                inp_copy += src_stride; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }

        rem_w &= 0x7;

        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);

            for (row = 0; row < (height - 1); row += 2)
            {

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride));

                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));

                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));

                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));

                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));

                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 1));

                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                /* row =1 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 3));

                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 5));

                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 7));

                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);

                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);

                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);

                //if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }

                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (src_stride << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }

            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));

            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));

            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));

            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));

            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));

            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

            res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

            //if (is_last)
            {
                res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
            }

            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }

        rem_w &= 0x3;

        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;

            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);

            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7

            for (row = 0; row < (height - 1); row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + col));

                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);

                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);

                    //if (is_last)
                    {
                        sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                        sum1 = (sum1 < min_val) ? min_val : (sum1>max_val ? max_val : sum1);
                    }
                    dst_copy[col] = (sum);
                    dst_copy[col + dst_stride] = (sum1);
                }
                inp_copy += (src_stride << 1);
                dst_copy += (dst_stride << 1);
            }

            for (col = 0; col < rem_w; col++)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));

                src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);

                src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                src_temp2_16x8b = _mm_srli_si128(src_temp1_16x8b, 4);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, src_temp2_16x8b);

                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);

                sum = (s16)_mm_extract_epi16(src_temp1_16x8b, 0);

                //if (is_last)
                {
                    sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                }
                dst_copy[col] = (sum);
            }
        }
    }
}

static void mc_filter_l_8pel_horz_no_clip_sse(s16 *ref,
                                              int src_stride,
                                              s16 *pred,
                                              int dst_stride,
                                              const s16 *coeff,
                                              int width,
                                              int height,
                                              int offset,
                                              int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;

    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */

    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;

    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);

    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);

    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));

                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));

                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));

                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));

                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);

                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);

                    cnt += 8; /* To pointer updates*/
                }

                inp_copy += src_stride; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }

        rem_w &= 0x7;

        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);

            for (row = 0; row < height; row += 2)
            {
                /*load 8 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride));    /* row = 1 */

                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));

                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));

                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));

                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 1));

                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 3));

                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 5));

                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 7));

                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);

                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);

                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);

                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (src_stride << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }

        rem_w &= 0x3;

        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;

            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);

            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7

            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + col));

                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);

                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);

                    dst_copy[col] = (sum);
                    dst_copy[col + dst_stride] = (sum1);
                }
                inp_copy += (src_stride << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));

                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    /* row = 0 */
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));

                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));

                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));

                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);

                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);

                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);

                    cnt += 8; /* To pointer updates*/
                }

                inp_copy += src_stride; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }

        rem_w &= 0x7;

        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);

            for (row = 0; row < (height - 1); row += 2)
            {

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride));

                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));

                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));

                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));

                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));

                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 1));

                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                /* row =1 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 3));

                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 5));

                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + 7));

                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);

                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);

                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);

                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (src_stride << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }

            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));

            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));

            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));

            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));

            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));

            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

            res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);

            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }

        rem_w &= 0x3;

        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;

            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);

            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7

            for (row = 0; row < (height - 1); row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + col));

                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);

                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);

                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);

                    dst_copy[col] = (sum);
                    dst_copy[col + dst_stride] = (sum1);
                }
                inp_copy += (src_stride << 1);
                dst_copy += (dst_stride << 1);
            }

            for (col = 0; col < rem_w; col++)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));

                src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);

                src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                src_temp2_16x8b = _mm_srli_si128(src_temp1_16x8b, 4);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, src_temp2_16x8b);

                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);

                sum = (s16)_mm_extract_epi16(src_temp1_16x8b, 0);

                dst_copy[col] = (sum);
            }
        }
    }
}

static void mc_filter_l_8pel_vert_clip_sse(s16 *ref,
                                           int src_stride,
                                           s16 *pred,
                                           int dst_stride,
                                           const s16 *coeff,
                                           int width,
                                           int height,
                                           int min_val,
                                           int max_val,
                                           int offset,
                                           int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;

    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s3_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b, s2_6_16x8b, s2_7_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_3_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b, s3_7_16x8b;

    __m128i mm_min = _mm_set1_epi16(min_val);
    __m128i mm_max = _mm_set1_epi16(max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */

    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);

    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);

    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /*load 8 pixel values.*/
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));

                /*load 8 pixel values*/
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt));

                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);

                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /*load 8 pixel values*/
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride << 1) + cnt));

                /*load 8 pixel values*/
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride * 3) + cnt));

                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);

                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);

                /*load 8 pixel values*/
                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride << 2) + cnt));

                /*load 8 pixel values*/
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride * 5) + cnt));

                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);

                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff4_5_8x16b);

                /*load 8 pixel values*/
                s2_6_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride * 6) + cnt));

                /*load 8 pixel values*/
                s2_7_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride * 7) + cnt));

                s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_7_16x8b = _mm_unpackhi_epi16(s2_6_16x8b, s2_7_16x8b);

                s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
                s7_8x16b = _mm_madd_epi16(s3_7_16x8b, coeff6_7_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s6_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);

                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);

                /* i2_tmp = CLIP_U8(i2_tmp);*/
                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);

                //if (is_last)
                {
                    s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
                }

                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);

                cnt += 8;
            }
            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x7;

    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);

        for (row = 0; row < height; row++)
        {
            /*load 8 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));

            /*load 8 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (src_stride)));

            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);

            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 8 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * src_stride)));

            /*load 8 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * src_stride)));

            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);

            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            /*load 8 pixel values*/
            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * src_stride)));

            /*load 8 pixel values*/
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * src_stride)));

            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);

            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
            /*load 8 pixel values*/
            s2_6_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (6 * src_stride)));

            /*load 8 pixel values*/
            s2_7_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (7 * src_stride)));

            s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);

            s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);

            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s5_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
            s6_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);

            s7_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);

            /*(i2_tmp + OFFSET_14_MINUS_BIT_DEPTH) >> SHIFT_14_MINUS_BIT_DEPTH */
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);

            /* i2_tmp = CLIP_U8(i2_tmp);*/
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);

            //if (is_last)
            {
                s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);

            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x3;

    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);

        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                s16 val;
                int sum;

                sum = inp_copy[col + 0 * src_stride] * coeff[0];
                sum += inp_copy[col + 1 * src_stride] * coeff[1];
                sum += inp_copy[col + 2 * src_stride] * coeff[2];
                sum += inp_copy[col + 3 * src_stride] * coeff[3];
                sum += inp_copy[col + 4 * src_stride] * coeff[4];
                sum += inp_copy[col + 5 * src_stride] * coeff[5];
                sum += inp_copy[col + 6 * src_stride] * coeff[6];
                sum += inp_copy[col + 7 * src_stride] * coeff[7];

                val = (sum + offset) >> shift;
                //if (is_last)
                {
                    val = EVC_CLIP3(min_val, max_val, val);
                }
                dst_copy[col] = val;
            }

            inp_copy += src_stride;
            dst_copy += dst_stride;
        }
    }
}

#endif

#if OPT_SIMD_MC_C
static void mc_filter_c_4pel_horz_sse(s16 *ref,
                                      int src_stride,
                                      s16 *pred,
                                      int dst_stride,
                                      const s16 *coeff,
                                      int width,
                                      int height,
                                      int min_val,
                                      int max_val,
                                      int offset,
                                      int shift,
                                      s8  is_last)
{
    int row, col, rem_w, rem_h, cnt;
    int src_stride2, src_stride3;
    s16 *inp_copy;
    s16 *dst_copy;

    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */

    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i    mm_min = _mm_set1_epi16(min_val);
    __m128i mm_max = _mm_set1_epi16(max_val);
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, mm_mask;

    __m128i res0, res1, res2, res3;
    __m128i row11, row12, row13, row14, row21, row22, row23, row24;
    __m128i row31, row32, row33, row34, row41, row42, row43, row44;

    src_stride2 = (src_stride << 1);
    src_stride3 = (src_stride * 3);

    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);

    {
        rem_h = height & 0x3;
        rem_w = width;

        coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);  /*w2 w3 w2 w3 w2 w3 w2 w3*/
        coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);        /*w0 w1 w0 w1 w0 w1 w0 w1*/

        /* 8 pixels at a time */
        if (rem_w > 7)
        {
            cnt = 0;
            for (row = 0; row < height; row += 4)
            {
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row12 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row13 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row14 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                                                                                    /*load pixel values from row 2*/
                    row21 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt));
                    row22 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt + 1));
                    row23 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt + 2));
                    row24 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt + 3));

                    /*load pixel values from row 3*/
                    row31 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                    row32 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 1));
                    row33 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 2));
                    row34 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 3));
                    /*load pixel values from row 4*/
                    row41 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                    row42 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 1));
                    row43 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 2));
                    row44 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 3));

                    row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                    row12 = _mm_madd_epi16(row12, coeff0_1_8x16b);       /*a1+a2 a3+a4 a5+a6 a7+a8*/
                    row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a4+a5 a6+a7 a8+a9*/
                    row14 = _mm_madd_epi16(row14, coeff2_3_8x16b);       /*a3+a4 a5+a6 a7+a8 a9+a10*/
                    row21 = _mm_madd_epi16(row21, coeff0_1_8x16b);
                    row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                    row23 = _mm_madd_epi16(row23, coeff2_3_8x16b);
                    row24 = _mm_madd_epi16(row24, coeff2_3_8x16b);
                    row31 = _mm_madd_epi16(row31, coeff0_1_8x16b);
                    row32 = _mm_madd_epi16(row32, coeff0_1_8x16b);
                    row33 = _mm_madd_epi16(row33, coeff2_3_8x16b);
                    row34 = _mm_madd_epi16(row34, coeff2_3_8x16b);
                    row41 = _mm_madd_epi16(row41, coeff0_1_8x16b);
                    row42 = _mm_madd_epi16(row42, coeff0_1_8x16b);
                    row43 = _mm_madd_epi16(row43, coeff2_3_8x16b);
                    row44 = _mm_madd_epi16(row44, coeff2_3_8x16b);

                    row11 = _mm_add_epi32(row11, row13);
                    row12 = _mm_add_epi32(row12, row14);
                    row21 = _mm_add_epi32(row21, row23);
                    row22 = _mm_add_epi32(row22, row24);
                    row31 = _mm_add_epi32(row31, row33);
                    row32 = _mm_add_epi32(row32, row34);
                    row41 = _mm_add_epi32(row41, row43);
                    row42 = _mm_add_epi32(row42, row44);

                    row11 = _mm_add_epi32(row11, offset_4x32b);
                    row12 = _mm_add_epi32(row12, offset_4x32b);
                    row21 = _mm_add_epi32(row21, offset_4x32b);
                    row22 = _mm_add_epi32(row22, offset_4x32b);
                    row31 = _mm_add_epi32(row31, offset_4x32b);
                    row32 = _mm_add_epi32(row32, offset_4x32b);
                    row41 = _mm_add_epi32(row41, offset_4x32b);
                    row42 = _mm_add_epi32(row42, offset_4x32b);

                    row11 = _mm_srai_epi32(row11, shift);
                    row12 = _mm_srai_epi32(row12, shift);
                    row21 = _mm_srai_epi32(row21, shift);
                    row22 = _mm_srai_epi32(row22, shift);
                    row31 = _mm_srai_epi32(row31, shift);
                    row32 = _mm_srai_epi32(row32, shift);
                    row41 = _mm_srai_epi32(row41, shift);
                    row42 = _mm_srai_epi32(row42, shift);

                    row11 = _mm_packs_epi32(row11, row21);
                    row12 = _mm_packs_epi32(row12, row22);
                    row31 = _mm_packs_epi32(row31, row41);
                    row32 = _mm_packs_epi32(row32, row42);

                    res0 = _mm_unpacklo_epi16(row11, row12);
                    res1 = _mm_unpackhi_epi16(row11, row12);
                    res2 = _mm_unpacklo_epi16(row31, row32);
                    res3 = _mm_unpackhi_epi16(row31, row32);

                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));

                        mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res1, mm_max);
                        res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));

                        mm_mask = _mm_cmpgt_epi16(res2, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res2 = _mm_or_si128(_mm_and_si128(mm_mask, res2), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res2, mm_max);
                        res2 = _mm_or_si128(_mm_and_si128(mm_mask, res2), _mm_andnot_si128(mm_mask, mm_max));

                        mm_mask = _mm_cmpgt_epi16(res3, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res3 = _mm_or_si128(_mm_and_si128(mm_mask, res3), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res3, mm_max);
                        res3 = _mm_or_si128(_mm_and_si128(mm_mask, res3), _mm_andnot_si128(mm_mask, mm_max));
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                    _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                    _mm_storeu_si128((__m128i *)(dst_copy + (dst_stride << 1) + cnt), res2);
                    _mm_storeu_si128((__m128i *)(dst_copy + (dst_stride * 3) + cnt), res3);

                    cnt += 8;
                }

                cnt = 0;
                inp_copy += (src_stride << 2); /* pointer updates*/
                dst_copy += (dst_stride << 2); /* pointer updates*/
            }

            /*remaining ht */
            for (row = 0; row < rem_h; row++)
            {
                cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row12 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row13 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row14 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/

                    row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                    row12 = _mm_madd_epi16(row12, coeff0_1_8x16b);       /*a1+a2 a3+a4 a5+a6 a7+a8*/
                    row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a4+a5 a6+a7 a8+a9*/
                    row14 = _mm_madd_epi16(row14, coeff2_3_8x16b);       /*a3+a4 a5+a6 a7+a8 a9+a10*/

                    row11 = _mm_add_epi32(row11, row13); /*a0+a1+a2+a3 a2+a3+a4+a5 a4+a5+a6+a7 a6+a7+a8+a9*/
                    row12 = _mm_add_epi32(row12, row14); /*a1+a2+a3+a4 a3+a4+a5+a6 a5+a6+a7+a8 a7+a8+a9+a10*/

                    row11 = _mm_add_epi32(row11, offset_4x32b);
                    row12 = _mm_add_epi32(row12, offset_4x32b);

                    row11 = _mm_srai_epi32(row11, shift);
                    row12 = _mm_srai_epi32(row12, shift);

                    row11 = _mm_packs_epi32(row11, row12);

                    res3 = _mm_unpackhi_epi64(row11, row11);
                    res0 = _mm_unpacklo_epi16(row11, res3);

                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                    }

                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);

                    cnt += 8;
                }
                inp_copy += (src_stride); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }

        rem_w &= 0x7;

        /* one 4 pixel wd for multiple rows */
        if (rem_w > 3)
        {
            inp_copy = ref + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);

            for (row = 0; row < height; row += 4)
            {
                /*load pixel values from row 1*/
                row11 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row12 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row13 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row14 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                                                                        /*load pixel values from row 2*/
                row21 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride));
                row22 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride + 1));
                row23 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride + 2));
                row24 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride + 3));

                /*load pixel values from row 3*/
                row31 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2));
                row32 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 1));
                row33 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 2));
                row34 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 3));
                /*load pixel values from row 4*/
                row41 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3));
                row42 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 1));
                row43 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 2));
                row44 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 3));

                row11 = _mm_unpacklo_epi32(row11, row12);
                row13 = _mm_unpacklo_epi32(row13, row14);
                row21 = _mm_unpacklo_epi32(row21, row22);
                row23 = _mm_unpacklo_epi32(row23, row24);
                row31 = _mm_unpacklo_epi32(row31, row32);
                row33 = _mm_unpacklo_epi32(row33, row34);
                row41 = _mm_unpacklo_epi32(row41, row42);
                row43 = _mm_unpacklo_epi32(row43, row44);

                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);
                row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);
                row21 = _mm_madd_epi16(row21, coeff0_1_8x16b);
                row23 = _mm_madd_epi16(row23, coeff2_3_8x16b);
                row31 = _mm_madd_epi16(row31, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff2_3_8x16b);
                row41 = _mm_madd_epi16(row41, coeff0_1_8x16b);
                row43 = _mm_madd_epi16(row43, coeff2_3_8x16b);

                row11 = _mm_add_epi32(row11, row13);
                row21 = _mm_add_epi32(row21, row23);
                row31 = _mm_add_epi32(row31, row33);
                row41 = _mm_add_epi32(row41, row43);

                row11 = _mm_add_epi32(row11, offset_4x32b);
                row21 = _mm_add_epi32(row21, offset_4x32b);
                row31 = _mm_add_epi32(row31, offset_4x32b);
                row41 = _mm_add_epi32(row41, offset_4x32b);

                row11 = _mm_srai_epi32(row11, shift);
                row21 = _mm_srai_epi32(row21, shift);
                row31 = _mm_srai_epi32(row31, shift);
                row41 = _mm_srai_epi32(row41, shift);

                res0 = _mm_packs_epi32(row11, row21);
                res1 = _mm_packs_epi32(row31, row41);

                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res0, mm_max);
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));

                    mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res1, mm_max);
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res0);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
                _mm_storel_epi64((__m128i *)(dst_copy + (dst_stride << 1)), res1);
                _mm_storel_epi64((__m128i *)(dst_copy + (dst_stride * 3)), _mm_unpackhi_epi64(res1, res1));

                inp_copy += (src_stride << 2); /* pointer updates*/
                dst_copy += (dst_stride << 2); /* pointer updates*/
            }

            for (row = 0; row < rem_h; row++)
            {
                /*load pixel values from row 1*/
                row11 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row12 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row13 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row14 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/

                row11 = _mm_unpacklo_epi32(row11, row12);        /*a0 a1 a1 a2 a2 a3 a3 a4*/
                row13 = _mm_unpacklo_epi32(row13, row14);        /*a2 a3 a3 a4 a4 a5 a5 a6*/

                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a1+a2 a2+a3 a3+a4*/
                row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a3+a4 a4+a5 a5+a6*/

                row11 = _mm_add_epi32(row11, row13);    /*r00 r01  r02  r03*/

                row11 = _mm_add_epi32(row11, offset_4x32b);

                row11 = _mm_srai_epi32(row11, shift);

                res1 = _mm_packs_epi32(row11, row11);

                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res1, mm_max);
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res1);

                inp_copy += (src_stride); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }

        rem_w &= 0x3;
        if (rem_w)
        {
            inp_copy = ref + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);

            for (row = 0; row < height; row++)
            {
                for (col = 0; col < rem_w; col++)
                {
                    s16 val;
                    int sum;

                    sum = inp_copy[col + 0] * coeff[0];
                    sum += inp_copy[col + 1] * coeff[1];
                    sum += inp_copy[col + 2] * coeff[2];
                    sum += inp_copy[col + 3] * coeff[3];

                    val = (sum + offset) >> shift;
                    dst_copy[col] = (is_last ? (EVC_CLIP3(min_val, max_val, val)) : val);
                }
                inp_copy += (src_stride); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
    }
}

static void mc_filter_c_4pel_vert_sse(s16 *ref,
                                      int src_stride,
                                      s16 *pred,
                                      int dst_stride,
                                      const s16 *coeff,
                                      int width,
                                      int height,
                                      int min_val,
                                      int max_val,
                                      int offset,
                                      int shift,
                                      s8  is_last)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;

    __m128i coeff0_1_8x16b, coeff2_3_8x16b, mm_mask;
    __m128i s0_8x16b, s1_8x16b, s4_8x16b, s5_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_4_16x8b, s3_5_16x8b;

    __m128i mm_min = _mm_set1_epi16(min_val);
    __m128i mm_max = _mm_set1_epi16(max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */

    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);

    if(rem_w > 7)
    {
        for(row = 0; row < height; row++)
        {
            int cnt = 0;
            for(col = width; col > 7; col -= 8)
            {
                /* a0 a1 a2 a3 a4 a5 a6 a7 */
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                /* b0 b1 b2 b3 b4 b5 b6 b7 */
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + src_stride + cnt));
                /* a0 b0 a1 b1 a2 b2 a3 b3 */
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a4 b4 ... a7 b7 */
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a0+b0 a1+b1 a2+b2 a3+b3*/
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);

                /* c0 c1 c2 c3 c4 c5 c6 c7 */
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride << 1) + cnt));
                /* d0 d1 d2 d3 d4 d5 d6 d7 */
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (src_stride * 3) + cnt));
                /* c0 d0 c1 d1 c2 d2 c3 d3 */
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                /* c0+d0 c1+d1 c2+d2 c3+d3*/
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);

                /* a0+b0+c0+d0 ... a3+b3+c3+d3 */
                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                /* a4+b4+c4+d4 ... a7+b7+c7+d7 */
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);

                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);

                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);

                if(is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
                }

                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);

                cnt += 8;
            }
            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x7;

    if(rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);

        for(row = 0; row < height; row++)
        {
            /*load 4 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            /*load 4 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (src_stride)));

            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);

            /*load 4 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * src_stride)));
            /*load 4 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * src_stride)));

            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);

            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);

            s7_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);

            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);

            if(is_last)
            {
                mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);

            inp_copy += (src_stride);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x3;

    if(rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);

        for(row = 0; row < height; row++)
        {
            for(col = 0; col < rem_w; col++)
            {
                s16 val;
                int sum;

                sum = inp_copy[col + 0 * src_stride] * coeff[0];
                sum += inp_copy[col + 1 * src_stride] * coeff[1];
                sum += inp_copy[col + 2 * src_stride] * coeff[2];
                sum += inp_copy[col + 3 * src_stride] * coeff[3];

                val = (sum + offset) >> shift;
                dst_copy[col] = (is_last ? (EVC_CLIP3(min_val, max_val, val)) : val);
            }

            inp_copy += src_stride;
            dst_copy += dst_stride;
        }
    }
}
#endif

void evc_mc_l_00(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
    int i, j;

#if MC_PRECISION_ADD
    gmv_x >>= 4;
    gmv_y >>= 4;
#else
    gmv_x >>= 2;
    gmv_y >>= 2;
#endif

    ref += gmv_y * s_ref + gmv_x;

#if defined(X86_SSE)
    if(((w & 0x7)==0) && ((h & 1)==0))
    {
        __m128i m00, m01;

        for(i=0; i<h; i+=2)
        {
            for(j=0; j<w; j+=8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));

                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref  += s_ref * 2;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i m00;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else
#endif
    {
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
}

void evc_mc_l_n0(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
    int i, j, dx;
    s32 pt;
#if MC_PRECISION_ADD
    dx = gmv_x & 15;
    ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4) - 3;
#else
    dx = gmv_x & 0x3;
    ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2) - 3;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_L
    if (1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_l_8pel_horz_clip_sse(ref, s_ref, pred, s_pred, tbl_mc_l_coeff[g_mc_ftr][dx], w, h, min, max, MAC_ADD_N0, MAC_SFT_N0);
    }
#else
    if ((w & 0x7) == 0)
    {
        __m128i coef[8], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=8)
            {
                SSE_MC_FILTER_L_8PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i coef[8], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_L_4PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_8TAP_N0(tbl_mc_l_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]);
                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);
            }
            ref += s_ref;
            pred += s_pred;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_mc_l_0n(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
    int i, j, dy;
    s32 pt;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dy = gmv_y & 15;
    ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4);
#else
    dy = gmv_y & 0x3;
    ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_L
    if(1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_l_8pel_vert_clip_sse(ref, s_ref, pred, s_pred, tbl_mc_l_coeff[g_mc_ftr][dy], w, h, min, max, MAC_ADD_0N, MAC_SFT_0N);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[8], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();
        for(i = 0; i < 8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 8)
            {
                SSE_MC_FILTER_L_8PEL(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
    else if((w & 0x3) == 0)
    {
        __m128i coef[8], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i = 0; i < 8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 4)
            {
                SSE_MC_FILTER_L_4PEL(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
#endif
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_8TAP_0N(tbl_mc_l_coeff[g_mc_ftr][dy], ref[j], ref[s_ref + j], ref[s_ref * 2 + j], ref[s_ref * 3 + j], ref[s_ref * 4 + j], ref[s_ref * 5 + j], ref[s_ref * 6 + j], ref[s_ref * 7 + j]);
                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);
            }
            ref += s_ref;
            pred += s_pred;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_mc_l_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L)*(MAX_CU_SIZE + MC_IBUF_PAD_L)];
    s16        *b;
    int         i, j, dx, dy;
    s32         pt;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dx = gmv_x & 15;
    dy = gmv_y & 15;
    ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4) - 3;
#else
    dx = gmv_x & 0x3;
    dy = gmv_y & 0x3;
    ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_L
    if(1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_mc_l_coeff[g_mc_ftr][dx], w, (h + 7), MAC_ADD_NN_S1, MAC_SFT_NN_S1);
        mc_filter_l_8pel_vert_clip_sse(buf, w, pred, s_pred, tbl_mc_l_coeff[g_mc_ftr][dy], w, h, min, max, MAC_ADD_NN_S2, MAC_SFT_NN_S2);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[8], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);
        b = buf;

        for(i=0; i<h+7; i++)
        {
            for(j=0; j<w; j+=8)
            {
                SSE_MC_FILTER_L_8PEL(ref+j, 1, b+j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
            }
            ref  += s_ref;
            b     += w;
        }

        for(i=0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);
        b = buf;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=8)
            {
                SSE_MC_FILTER_L_8PEL(b+j, w, pred+j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
            }
            pred += s_pred;
            b     += w;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i coef[8], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);
        b = buf;

        for(i=0; i<h+7; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_L_4PEL(ref+j, 1, b+j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
            }
            ref  += s_ref;
            b     += w;
        }

        for(i=0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);
        b = buf;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_L_4PEL(b+j, w, pred+j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
            }
            pred += s_pred;
            b     += w;
        }
    }
#endif
    else
#endif
    {
        b = buf;
        for(i = 0; i < h + 7; i++)
        {
            for(j = 0; j < w; j++)
            {
                b[j] = MAC_8TAP_NN_S1(tbl_mc_l_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]);
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_8TAP_NN_S2(tbl_mc_l_coeff[g_mc_ftr][dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]);
                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);
            }
            pred += s_pred;
            b += w;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

#if DMVR_PADDING
void evc_mc_dmvr_l_00(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
  int i, j;

#if MC_PRECISION_ADD
  gmv_x >>= 4;
  gmv_y >>= 4;
#else
  gmv_x >>= 2;
  gmv_y >>= 2;
#endif

  //ref += gmv_y * s_ref + gmv_x;

#if defined(X86_SSE)
  if (((w & 0x7) == 0) && ((h & 1) == 0))
  {
    __m128i m00, m01;

    for (i = 0; i<h; i += 2)
    {
      for (j = 0; j<w; j += 8)
      {
        m00 = _mm_loadu_si128((__m128i*)(ref + j));
        m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));

        _mm_storeu_si128((__m128i*)(pred + j), m00);
        _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
      }
      pred += s_pred * 2;
      ref += s_ref * 2;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i m00;

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        m00 = _mm_loadl_epi64((__m128i*)(ref + j));
        _mm_storel_epi64((__m128i*)(pred + j), m00);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
  else
#endif
  {
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pred[j] = ref[j];
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
}

void evc_mc_dmvr_l_n0(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
  int i, j, dx;
  s32 pt;

#if MC_PRECISION_ADD
  dx = gmv_x & 15;
  //ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4) - 3;
  ref =  ref - 3;
#else
  dx = gmv_x & 0x3;
  ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2) - 3;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_L
  if (1)
  {

    int max = ((1 << BIT_DEPTH) - 1);
    int min = 0;


    mc_filter_l_8pel_horz_clip_sse(ref, s_ref, pred, s_pred, tbl_mc_l_coeff[g_mc_ftr][dx], w, h, min, max,
      MAC_ADD_N0, MAC_SFT_N0);
  }
#else
  if ((w & 0x7) == 0)
  {
    __m128i coef[8], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 8)
      {
        SSE_MC_FILTER_L_8PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i coef[8], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_L_4PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
#endif
  else
#endif
  {
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pt = MAC_8TAP_N0(tbl_mc_l_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]);

        pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

      }
      ref += s_ref;
      pred += s_pred;
    }
#if SIMD_CLIP
    pred = dst_cpy;
    clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
  }
}

void evc_mc_dmvr_l_0n(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
  int i, j, dy;
  s32 pt;
#if SIMD_CLIP
  pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
  dy = gmv_y & 15;
  //ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4);
  ref = ref - ( 3 * s_ref);
#else
  dy = gmv_y & 0x3;
  ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_L
  if (1)
  {

    int max = ((1 << BIT_DEPTH) - 1);
    int min = 0;

    mc_filter_l_8pel_vert_clip_sse(ref, s_ref, pred, s_pred, tbl_mc_l_coeff[g_mc_ftr][dy], w, h, min, max,
      MAC_ADD_0N, MAC_SFT_0N);
  }
#else
  if ((w & 0x7) == 0)
  {
    __m128i coef[8], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();
    for (i = 0; i < 8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);

    for (i = 0; i < h; i++)
    {
      for (j = 0; j < w; j += 8)
      {
        SSE_MC_FILTER_L_8PEL(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i coef[8], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i < 8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);

    for (i = 0; i < h; i++)
    {
      for (j = 0; j < w; j += 4)
      {
        SSE_MC_FILTER_L_4PEL(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
#endif
  else
#endif
  {
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pt = MAC_8TAP_0N(tbl_mc_l_coeff[g_mc_ftr][dy], ref[j], ref[s_ref + j], ref[s_ref * 2 + j], ref[s_ref * 3 + j], ref[s_ref * 4 + j], ref[s_ref * 5 + j], ref[s_ref * 6 + j], ref[s_ref * 7 + j]);

        pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

      }
      ref += s_ref;
      pred += s_pred;
    }
#if SIMD_CLIP
    pred = dst_cpy;
    clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
  }
}

void evc_mc_dmvr_l_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h
)
{
  s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L)*MAX_CU_SIZE];
  s16        *b;
  int         i, j, dx, dy;
  s32         pt;
#if SIMD_CLIP
  pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
  dx = gmv_x & 15;
  dy = gmv_y & 15;
//  ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4) - 3;
  ref = ref - (3 * s_ref + 3);
#else
  dx = gmv_x & 0x3;
  dy = gmv_y & 0x3;
  ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_L
  if (1)
  {

    int max = ((1 << BIT_DEPTH) - 1);
    int min = 0;

    mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_mc_l_coeff[g_mc_ftr][dx], w, (h + 7),
      MAC_ADD_NN_S1, MAC_SFT_NN_S1);

    mc_filter_l_8pel_vert_clip_sse(buf, w, pred, s_pred, tbl_mc_l_coeff[g_mc_ftr][dy], w, h, min, max,
      MAC_ADD_NN_S2, MAC_SFT_NN_S2);
  }
#else
  if ((w & 0x7) == 0)
  {
    __m128i coef[8], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);
    b = buf;

    for (i = 0; i<h + 7; i++)
    {
      for (j = 0; j<w; j += 8)
      {
        SSE_MC_FILTER_L_8PEL(ref + j, 1, b + j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
      }
      ref += s_ref;
      b += w;
    }

    for (i = 0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);
    b = buf;

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 8)
      {
        SSE_MC_FILTER_L_8PEL(b + j, w, pred + j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
      }
      pred += s_pred;
      b += w;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i coef[8], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dx][i]);
    b = buf;

    for (i = 0; i<h + 7; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_L_4PEL(ref + j, 1, b + j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
      }
      ref += s_ref;
      b += w;
    }

    for (i = 0; i<8; i++) coef[i] = _mm_set1_epi16(tbl_mc_l_coeff[g_mc_ftr][dy][i]);
    b = buf;

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_L_4PEL(b + j, w, pred + j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
      }
      pred += s_pred;
      b += w;
    }
  }
#endif
  else
#endif
  {
    b = buf;
    for (i = 0; i<h + 7; i++)
    {
      for (j = 0; j<w; j++)
      {
        b[j] = MAC_8TAP_NN_S1(tbl_mc_l_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]);
      }
      ref += s_ref;
      b += w;
    }

    b = buf;
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pt = MAC_8TAP_NN_S2(tbl_mc_l_coeff[g_mc_ftr][dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]);

        pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

      }
      pred += s_pred;
      b += w;
    }
#if SIMD_CLIP
    pred = dst_cpy;
    clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
  }
}
#endif

void evc_bl_mc_l_00(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
    int i, j;

#if MC_PRECISION_ADD
    gmv_x >>= 4;
    gmv_y >>= 4;
#else
    gmv_x >>= 2;
    gmv_y >>= 2;
#endif

    ref += gmv_y * s_ref + gmv_x;

#ifdef X86_SSE
    if (((w & 0x7) == 0) && ((h & 1) == 0))
    {
        __m128i m00, m01;

        for (i = 0; i < h; i += 2)
        {
            for (j = 0; j < w; j += 8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));

                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref += s_ref * 2;
        }
    }
    else if ((w & 0x3) == 0)
    {
        __m128i m00;

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
}

void evc_bl_mc_l_n0(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
    int i, j, dx;
    s32 pt;

#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dx = gmv_x & 15;
    ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4);
#else
    dx = gmv_x & 0x3;
    ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_BL
    if (1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_bilin_horz_sse(ref, s_ref, pred, s_pred, tbl_bl_mc_l_coeff[dx],
                                 w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[2], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dx][i]);

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 8)
            {
                SSE_MC_FILTER_L_8PEL_BILIN(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
    else if((w & 0x3) == 0)
    {
        __m128i coef[2], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dx][i]);

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 4)
            {
                SSE_MC_FILTER_L_4PEL_BILIN(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
#endif
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_BL_N0(tbl_bl_mc_l_coeff[dx], ref[j], ref[j + 1]);

                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

            }
            ref += s_ref;
            pred += s_pred;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_bl_mc_l_0n(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h)
{
    int i, j, dy;
    s32 pt;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif
#if MC_PRECISION_ADD
    dy = gmv_y & 15;
    ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4);
#else
    dy = gmv_y & 0x3;
    ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_BL
    if (1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_bilin_vert_sse(ref, s_ref, pred, s_pred, tbl_bl_mc_l_coeff[dy],
                                 w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[2], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for (i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dy][i]);

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j += 8)
            {
                SSE_MC_FILTER_L_8PEL_BILIN(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
    else if ((w & 0x3) == 0)
    {
        __m128i coef[2], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for (i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dy][i]);

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j += 4)
            {
                SSE_MC_FILTER_L_4PEL_BILIN(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
#endif
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_BL_0N(tbl_bl_mc_l_coeff[dy], ref[j], ref[s_ref + j]);

                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

            }
            ref += s_ref;
            pred += s_pred;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_bl_mc_l_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L)*(MAX_CU_SIZE + MC_IBUF_PAD_L)];
    s16        *b;
    int         i, j, dx, dy;
    s32         pt;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dx = gmv_x & 15;
    dy = gmv_y & 15;
    ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4);
#else
    dx = gmv_x & 0x3;
    dy = gmv_y & 0x3;
    ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_BL
    if (1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_bilin_horz_sse(ref, s_ref, buf, w, tbl_bl_mc_l_coeff[dx],
                                 w, (h + 1), min, max, MAC_ADD_NN_S1, MAC_SFT_NN_S1, 0);

        mc_filter_bilin_vert_sse(buf, w, pred, s_pred, tbl_bl_mc_l_coeff[dy],
                                 w, h, min, max, MAC_ADD_NN_S2, MAC_SFT_NN_S2, 1);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[2], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dx][i]);
        b = buf;

        for(i = 0; i < h + 1; i++)
        {
            for(j = 0; j < w; j += 8)
            {
                SSE_MC_FILTER_L_8PEL_BILIN(ref + j, 1, b + j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
            }
            ref += s_ref;
            b += w;
        }

        for(i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dy][i]);
        b = buf;

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 8)
            {
                SSE_MC_FILTER_L_8PEL_BILIN(b + j, w, pred + j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
            }
            pred += s_pred;
            b += w;
        }
    }
    else if((w & 0x3) == 0)
    {
        __m128i coef[2], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dx][i]);
        b = buf;

        for(i = 0; i < h + 1; i++)
        {
            for(j = 0; j < w; j += 4)
            {
                SSE_MC_FILTER_L_4PEL_BILIN(ref + j, 1, b + j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
            }
            ref += s_ref;
            b += w;
        }

        for(i = 0; i < 2; i++) coef[i] = _mm_set1_epi16(tbl_bl_mc_l_coeff[dy][i]);
        b = buf;

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 4)
            {
                SSE_MC_FILTER_L_4PEL_BILIN(b + j, w, pred + j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
            }
            pred += s_pred;
            b += w;
        }
    }
#endif
    else
#endif
    {
        b = buf;
        for(i = 0; i < h + 1; i++)
        {
            for(j = 0; j < w; j++)
            {
                b[j] = MAC_BL_NN_S1(tbl_bl_mc_l_coeff[dx], ref[j], ref[j + 1]);
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_BL_NN_S2(tbl_bl_mc_l_coeff[dy], b[j], b[j + w]);
                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);
            }
            pred += s_pred;
            b += w;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

/****************************************************************************
 * motion compensation for chroma
 ****************************************************************************/

#if MC_PRECISION_ADD

#if OPT_SIMD_MC_C
s16 tbl_mc_c_coeff[2][8 << MC_PRECISION_ADD][4] =
#else
static const s8 tbl_mc_c_coeff[8 << MC_PRECISION_ADD][4] =
#endif
{
    {
        {  0, 64,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -2, 58, 10, -2 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -4, 52, 20, -4 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -6, 46, 30, -6 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -8, 40, 40, -8 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -6, 30, 46, -6 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -4, 20, 52, -4 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        { -2, 10, 58, -2 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
        {  0,  0,  0,  0 },
    },
    {
        {  0, 64,  0,  0 },
        { -1, 63,  2,  0 },
        { -2, 62,  4,  0 },
        { -2, 60,  7, -1 },
        { -3, 60,  8, -1 },
        { -3, 57, 12, -2 },
        { -4, 56, 14, -2 },
        { -4, 55, 15, -2 },
        { -4, 54, 16, -2 },
        { -5, 53, 18, -2 },
        { -6, 52, 20, -2 },
        { -6, 49, 24, -3 },
        { -5, 46, 27, -4 },
        { -5, 44, 29, -4 },
        { -4, 42, 30, -4 },
        { -4, 39, 33, -4 },
        { -4, 36, 36, -4 },
        { -4, 33, 39, -4 },
        { -4, 30, 42, -4 },
        { -4, 29, 44, -5 },
        { -4, 27, 46, -5 },
        { -3, 24, 49, -6 },
        { -2, 20, 52, -6 },
        { -2, 18, 53, -5 },
        { -2, 16, 54, -4 },
        { -2, 15, 55, -4 },
        { -2, 14, 56, -4 },
        { -2, 12, 57, -3 },
        { -1,  8, 60, -3 },
        { -1,  7, 60, -2 },
        {  0,  4, 62, -2 },
        {  0,  2, 63, -1 },
    },
};
#else
#if OPT_SIMD_MC_C
static const s16 tbl_mc_c_coeff[2][8][4] =
#else
static const s8 tbl_mc_c_coeff[2][8][4] =
#endif
{
    {
        {  0, 64, 0,   0 },
        { -2, 58, 10, -2 },
        { -4, 52, 20, -4 },
        { -6, 46, 30, -6 },
        { -8, 40, 40, -8 },
        { -6, 30, 46, -6 },
        { -4, 20, 52, -4 },
        { -2, 10, 58, -2 },
    },
    {
        {  0, 64,  0,  0 },
        { -3, 60,  8, -1 },
        { -4, 54, 16, -2 },
        { -5, 46, 27, -4 },
        { -4, 36, 36, -4 },
        { -4, 27, 46, -5 },
        { -2, 16, 54, -4 },
        { -1,  8, 60, -3 },
    }
};
#endif

void evc_mc_c_00(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
    int i, j;

#if MC_PRECISION_ADD
    gmv_x >>= 5;
    gmv_y >>= 5;
#else
    gmv_x >>= 3;
    gmv_y >>= 3;
#endif

    ref += gmv_y * s_ref + gmv_x;

#ifdef X86_SSE
    if(((w & 0x7) == 0) && ((h & 1) == 0))
    {
        __m128i m00, m01;

        for(i = 0; i < h; i += 2)
        {
            for(j = 0; j < w; j += 8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));

                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref += s_ref * 2;
        }
    }
    else if(((w & 0x3) == 0))
    {
        __m128i m00;

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j += 4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref += s_ref;
        }
    }
}

void evc_mc_c_n0(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
    int       i, j, dx;
    s32       pt;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dx = gmv_x & 31;
    ref += (gmv_y >> 5) * s_ref + (gmv_x >> 5) - 1;
#else
    dx = gmv_x & 0x7;
    ref += (gmv_y >> 3) * s_ref + (gmv_x >> 3) - 1;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_C
    if(1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_c_4pel_horz_sse(ref, s_ref, pred, s_pred, tbl_mc_c_coeff[g_mc_ftr][dx], w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[4], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_C_8PEL(ref+j, 1, pred+j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i coef[4], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_C_4PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_4TAP_N0(tbl_mc_c_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3]);
                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);
            }
            pred += s_pred;
            ref += s_ref;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_mc_c_0n(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
    int i, j, dy;
    s32       pt;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dy = gmv_y & 31;
    ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5);
#else
    dy = gmv_y & 0x7;
    ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_C
    if(1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_c_4pel_vert_sse(ref, s_ref, pred, s_pred, tbl_mc_c_coeff[g_mc_ftr][dy], w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
    }
#else
    if((w & 0x7)==0)
    {
        __m128i coef[4], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_C_8PEL(ref+j, s_ref, pred+j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i coef[4], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_C_4PEL(ref+j, s_ref, pred+j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
    else
#endif
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_4TAP_0N(tbl_mc_c_coeff[g_mc_ftr][dy], ref[j], ref[s_ref + j], ref[s_ref * 2 + j], ref[s_ref * 3 + j]);
                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);
            }
            pred += s_pred;
            ref += s_ref;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

void evc_mc_c_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
    s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_C)*MAX_CU_SIZE];
    s16        *b;
    int         i, j;
    s32         pt;
    int         dx, dy;
#if SIMD_CLIP
    pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
    dx = gmv_x & 31;
    dy = gmv_y & 31;
    ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5) - 1;
#else
    dx = gmv_x & 0x7;
    dy = gmv_y & 0x7;
    ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3) - 1;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_C
    if(1)
    {
        int max = ((1 << BIT_DEPTH) - 1);
        int min = 0;

        mc_filter_c_4pel_horz_sse(ref, s_ref, buf, w, tbl_mc_c_coeff[g_mc_ftr][dx],
                                  w, (h + 3), min, max, MAC_ADD_NN_S1, MAC_SFT_NN_S1, 0);

        mc_filter_c_4pel_vert_sse(buf, w, pred, s_pred, tbl_mc_c_coeff[g_mc_ftr][dy],
                                  w, h, min, max, MAC_ADD_NN_S2, MAC_SFT_NN_S2, 1);
    }
#else
    if((w & 0x7) == 0)
    {
        __m128i coef[4], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);
        b = buf;

        for(i=0; i<h+3; i++)
        {
            for(j=0; j<w; j+=8)
            {
                SSE_MC_FILTER_C_8PEL(ref+j, 1, b+j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
            }
            ref  += s_ref;
            b     += w;
        }

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);
        b = buf;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=8)
            {
                SSE_MC_FILTER_C_8PEL(b+j, w, pred+j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
            }
            pred += s_pred;
            b     += w;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i coef[4], min, max;

        max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
        min = _mm_setzero_si128();

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);
        b = buf;

        for(i=0; i<h+3; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_C_4PEL(ref+j, 1, b+j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
            }
            ref  += s_ref;
            b     += w;
        }

        for(i=0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);
        b = buf;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                SSE_MC_FILTER_C_4PEL(b+j, w, pred+j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
            }
            pred += s_pred;
            b     += w;
        }
    }
#endif
    else
#endif
    {
        b = buf;
        for(i = 0; i < h + 3; i++)
        {
            for(j = 0; j < w; j++)
            {
                b[j] = MAC_4TAP_NN_S1(tbl_mc_c_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3]);
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                pt = MAC_4TAP_NN_S2(tbl_mc_c_coeff[g_mc_ftr][dy], b[j], b[j + w], b[j + 2 * w], b[j + 3 * w]);

                pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

            }
            pred += s_pred;
            b += w;
        }
#if SIMD_CLIP
        pred = dst_cpy;
        clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
    }
}

#if DMVR_PADDING
void evc_mc_dmvr_c_00(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
  int i, j;

#if MC_PRECISION_ADD
  gmv_x >>= 5;
  gmv_y >>= 5;
#else
  gmv_x >>= 3;
  gmv_y >>= 3;
#endif

#ifdef X86_SSE
  if (((w & 0x7) == 0) && ((h & 1) == 0))
  {
    __m128i m00, m01;

    for (i = 0; i<h; i += 2)
    {
      for (j = 0; j<w; j += 8)
      {
        m00 = _mm_loadu_si128((__m128i*)(ref + j));
        m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));

        _mm_storeu_si128((__m128i*)(pred + j), m00);
        _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
      }
      pred += s_pred * 2;
      ref += s_ref * 2;
    }
  }
  else if (((w & 0x3) == 0))
  {
    __m128i m00;

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        m00 = _mm_loadl_epi64((__m128i*)(ref + j));
        _mm_storel_epi64((__m128i*)(pred + j), m00);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
  else
#endif
  {
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pred[j] = ref[j];
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
}

void evc_mc_dmvr_c_n0(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
  int       i, j, dx;
  s32       pt;
#if SIMD_CLIP
  pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
  dx = gmv_x & 31;
  ref -= 1;
#else
  dx = gmv_x & 0x7;
  ref += (gmv_y >> 3) * s_ref + (gmv_x >> 3) - 1;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_C
  if (1)
  {

    int max = ((1 << BIT_DEPTH) - 1);
    int min = 0;


    mc_filter_c_4pel_horz_sse(ref, s_ref, pred, s_pred, tbl_mc_c_coeff[g_mc_ftr][dx],
      w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
  }
#else
  if ((w & 0x7) == 0)
  {
    __m128i coef[4], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_C_8PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i coef[4], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_C_4PEL(ref + j, 1, pred + j, MAC_ADD_N0, MAC_SFT_N0, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
#endif
  else
#endif
  {
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pt = MAC_4TAP_N0(tbl_mc_c_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3]);

        pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

      }
      pred += s_pred;
      ref += s_ref;
    }
#if SIMD_CLIP
    pred = dst_cpy;
    clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
  }
}

void evc_mc_dmvr_c_0n(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
  int i, j, dy;
  s32       pt;
#if SIMD_CLIP
  pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
  dy = gmv_y & 31;
  ref -= 1 * s_ref;
#else
  dy = gmv_y & 0x7;
  ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3);
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_C
  if (1)
  {

    int max = ((1 << BIT_DEPTH) - 1);
    int min = 0;

    mc_filter_c_4pel_vert_sse(ref, s_ref, pred, s_pred, tbl_mc_c_coeff[g_mc_ftr][dy],
      w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
  }
#else
  if ((w & 0x7) == 0)
  {
    __m128i coef[4], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_C_8PEL(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i coef[4], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_C_4PEL(ref + j, s_ref, pred + j, MAC_ADD_0N, MAC_SFT_0N, coef, 1, min, max);
      }
      pred += s_pred;
      ref += s_ref;
    }
  }
#endif
  else
#endif
  {
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pt = MAC_4TAP_0N(tbl_mc_c_coeff[g_mc_ftr][dy], ref[j], ref[s_ref + j], ref[s_ref * 2 + j], ref[s_ref * 3 + j]);

        pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

      }
      pred += s_pred;
      ref += s_ref;
    }
#if SIMD_CLIP
    pred = dst_cpy;
    clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
  }
}

void evc_mc_dmvr_c_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h)
{
  s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_C)*MAX_CU_SIZE];
  s16        *b;
  int         i, j;
  s32         pt;
  int         dx, dy;
#if SIMD_CLIP
  pel *dst_cpy = pred;
#endif

#if MC_PRECISION_ADD
  dx = gmv_x & 31;
  dy = gmv_y & 31;
  //ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5) - 1;
  ref -= (1 * s_ref + 1);
#else
  dx = gmv_x & 0x7;
  dy = gmv_y & 0x7;
  ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3) - 1;
#endif

#ifdef X86_SSE
#if OPT_SIMD_MC_C
  if (1)
  {

    int max = ((1 << BIT_DEPTH) - 1);
    int min = 0;

    mc_filter_c_4pel_horz_sse(ref, s_ref, buf, w, tbl_mc_c_coeff[g_mc_ftr][dx],
      w, (h + 3), min, max, MAC_ADD_NN_S1, MAC_SFT_NN_S1, 0);

    mc_filter_c_4pel_vert_sse(buf, w, pred, s_pred, tbl_mc_c_coeff[g_mc_ftr][dy],
      w, h, min, max, MAC_ADD_NN_S2, MAC_SFT_NN_S2, 1);
  }
#else
  if ((w & 0x7) == 0)
  {
    __m128i coef[4], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);
    b = buf;

    for (i = 0; i<h + 3; i++)
    {
      for (j = 0; j<w; j += 8)
      {
        SSE_MC_FILTER_C_8PEL(ref + j, 1, b + j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
      }
      ref += s_ref;
      b += w;
    }

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);
    b = buf;

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 8)
      {
        SSE_MC_FILTER_C_8PEL(b + j, w, pred + j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
      }
      pred += s_pred;
      b += w;
    }
  }
  else if ((w & 0x3) == 0)
  {
    __m128i coef[4], min, max;

    max = _mm_set1_epi16((1 << BIT_DEPTH) - 1);
    min = _mm_setzero_si128();

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dx][i]);
    b = buf;

    for (i = 0; i<h + 3; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_C_4PEL(ref + j, 1, b + j, MAC_ADD_NN_S1, MAC_SFT_NN_S1, coef, 0, min, max);
      }
      ref += s_ref;
      b += w;
    }

    for (i = 0; i<4; i++) coef[i] = _mm_set1_epi16(tbl_mc_c_coeff[g_mc_ftr][dy][i]);
    b = buf;

    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j += 4)
      {
        SSE_MC_FILTER_C_4PEL(b + j, w, pred + j, MAC_ADD_NN_S2, MAC_SFT_NN_S2, coef, 1, min, max);
      }
      pred += s_pred;
      b += w;
    }
  }
#endif
  else
#endif
  {
    b = buf;
    for (i = 0; i<h + 3; i++)
    {
      for (j = 0; j<w; j++)
      {
        b[j] = MAC_4TAP_NN_S1(tbl_mc_c_coeff[g_mc_ftr][dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3]);
      }
      ref += s_ref;
      b += w;
    }

    b = buf;
    for (i = 0; i<h; i++)
    {
      for (j = 0; j<w; j++)
      {
        pt = MAC_4TAP_NN_S2(tbl_mc_c_coeff[g_mc_ftr][dy], b[j], b[j + w], b[j + 2 * w], b[j + 3 * w]);

        pred[j] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, pt);

      }
      pred += s_pred;
      b += w;
    }
#if SIMD_CLIP
    pred = dst_cpy;
    clip_simd(pred, s_pred, pred, s_pred, w, h, adapt_clip_min[adapt_clip_comp], adapt_clip_max[adapt_clip_comp]);
#endif
  }
}
#endif

EVC_MC_L evc_tbl_mc_l[2][2] =
{
    {
        evc_mc_l_00, /* dx == 0 && dy == 0 */
        evc_mc_l_0n  /* dx == 0 && dy != 0 */
    },
    {
        evc_mc_l_n0, /* dx != 0 && dy == 0 */
        evc_mc_l_nn  /* dx != 0 && dy != 0 */
    }
};

EVC_MC_C evc_tbl_mc_c[2][2] =
{
    {
        evc_mc_c_00, /* dx == 0 && dy == 0 */
        evc_mc_c_0n  /* dx == 0 && dy != 0 */
    },
    {
        evc_mc_c_n0, /* dx != 0 && dy == 0 */
        evc_mc_c_nn  /* dx != 0 && dy != 0 */
    }
};

#if DMVR_PADDING
EVC_MC_L evc_tbl_dmvr_mc_l[2][2] =

{
  {
    evc_mc_dmvr_l_00, /* dx == 0 && dy == 0 */
    evc_mc_dmvr_l_0n  /* dx == 0 && dy != 0 */
  },
  {
    evc_mc_dmvr_l_n0, /* dx != 0 && dy == 0 */
    evc_mc_dmvr_l_nn  /* dx != 0 && dy != 0 */
  }
};

EVC_MC_C evc_tbl_dmvr_mc_c[2][2] =
{
  {
    evc_mc_dmvr_c_00, /* dx == 0 && dy == 0 */
    evc_mc_dmvr_c_0n  /* dx == 0 && dy != 0 */
  },
  {
    evc_mc_dmvr_c_n0, /* dx != 0 && dy == 0 */
    evc_mc_dmvr_c_nn  /* dx != 0 && dy != 0 */
  }
};
#endif

/* luma and chroma will remain the same */
EVC_MC_L evc_tbl_bl_mc_l[2][2] =
{
    {
        evc_bl_mc_l_00,
        evc_bl_mc_l_0n
    },
    {
        evc_bl_mc_l_n0,
        evc_bl_mc_l_nn
    }
};

void mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], s16(*mv_t)[MV_D])
{
    int min_clip[MV_D], max_clip[MV_D];

    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;
    min_clip[MV_X] = (-MAX_CU_SIZE) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE) << 2;

    mv_t[REFP_0][MV_X] = mv[REFP_0][MV_X];
    mv_t[REFP_0][MV_Y] = mv[REFP_0][MV_Y];
    mv_t[REFP_1][MV_X] = mv[REFP_1][MV_X];
    mv_t[REFP_1][MV_Y] = mv[REFP_1][MV_Y];

    if(REFI_IS_VALID(refi[REFP_0]))
    {
        if(x + mv[REFP_0][MV_X] < min_clip[MV_X]) mv_t[REFP_0][MV_X] = min_clip[MV_X] - x;
        if(y + mv[REFP_0][MV_Y] < min_clip[MV_Y]) mv_t[REFP_0][MV_Y] = min_clip[MV_Y] - y;
        if(x + mv[REFP_0][MV_X] + w - 4 > max_clip[MV_X]) mv_t[REFP_0][MV_X] = max_clip[MV_X] - x - w + 4;
        if(y + mv[REFP_0][MV_Y] + h - 4 > max_clip[MV_Y]) mv_t[REFP_0][MV_Y] = max_clip[MV_Y] - y - h + 4;
    }
    if(REFI_IS_VALID(refi[REFP_1]))
    {
        if(x + mv[REFP_1][MV_X] < min_clip[MV_X]) mv_t[REFP_1][MV_X] = min_clip[MV_X] - x;
        if(y + mv[REFP_1][MV_Y] < min_clip[MV_Y]) mv_t[REFP_1][MV_Y] = min_clip[MV_Y] - y;
        if(x + mv[REFP_1][MV_X] + w - 4 > max_clip[MV_X]) mv_t[REFP_1][MV_X] = max_clip[MV_X] - x - w + 4;
        if(y + mv[REFP_1][MV_Y] + h - 4 > max_clip[MV_Y]) mv_t[REFP_1][MV_Y] = max_clip[MV_Y] - y - h + 4;
    }
}

#if DMVR
typedef enum EVC_POINT_INDEX
{
    CENTER = 0,
    CENTER_TOP,
    CENTER_BOTTOM,
    LEFT_CENTER,
    RIGHT_CENTER,
    LEFT_TOP,
    RIGHT_TOP,
    LEFT_BOTTOM,
    RIGHT_BOTTOM,
    REF_PRED_POINTS_INDEXS_NUM
} EVC_POINT_INDEX;
enum NSAD_SRC
{
    SRC1 = 0,
    SRC2,
    SRC_NUM
};
enum NSAD_BORDER_LINE
{
    LINE_TOP = 0,
    LINE_BOTTOM,
    LINE_LEFT,
    LINE_RIGHT,
    LINE_NUM
};

#if OPT_SIMD_DMVR_MR_SAD
static int dmvr_sad_mr_16b_sse(int w, int h, void * src1, void * src2,
                               int s_src1, int s_src2, s16 delta)
{
    int  i, j, rem_w;
    int mr_sad = 0;

    short *pu2_inp;
    short *pu2_ref;

    __m128i src1_8x16b_0, src1_8x16b_1, src1_8x16b_2, src1_8x16b_3;
    __m128i src2_8x16b_0, src2_8x16b_1, src2_8x16b_2, src2_8x16b_3;

    __m128i /*mm_zero, */mm_result1, mm_result2, mm_delta;

    assert(BIT_DEPTH <= 14);

    mm_delta = _mm_set1_epi16(delta);

    pu2_inp = src1;
    pu2_ref = src2;
    rem_w = w;

    mm_result1 = _mm_setzero_si128();
    mm_result2 = _mm_setzero_si128();

    /* 16 pixels at a time */
    if (rem_w > 15)
    {
        for (i = h; i > 1; i -= 2)
        {
            for (j = 0; j < w; j += 16)
            {
                src2_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_ref));
                src2_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_ref + 8));
                src2_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_ref + s_src2));
                src2_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_ref + s_src2 + 8));

                src1_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_inp));
                src1_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_inp + 8));
                src1_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_inp + s_src1));
                src1_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_inp + s_src1 + 8));

                src2_8x16b_0 = _mm_slli_epi16(src2_8x16b_0, 1);
                src2_8x16b_1 = _mm_slli_epi16(src2_8x16b_1, 1);
                src2_8x16b_2 = _mm_slli_epi16(src2_8x16b_2, 1);
                src2_8x16b_3 = _mm_slli_epi16(src2_8x16b_3, 1);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, mm_delta);
                src1_8x16b_1 = _mm_sub_epi16(src1_8x16b_1, mm_delta);
                src1_8x16b_2 = _mm_sub_epi16(src1_8x16b_2, mm_delta);
                src1_8x16b_3 = _mm_sub_epi16(src1_8x16b_3, mm_delta);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, src2_8x16b_0);
                src1_8x16b_1 = _mm_sub_epi16(src1_8x16b_1, src2_8x16b_1);
                src1_8x16b_2 = _mm_sub_epi16(src1_8x16b_2, src2_8x16b_2);
                src1_8x16b_3 = _mm_sub_epi16(src1_8x16b_3, src2_8x16b_3);

                src1_8x16b_0 = _mm_abs_epi16(src1_8x16b_0);
                src1_8x16b_1 = _mm_abs_epi16(src1_8x16b_1);
                src1_8x16b_2 = _mm_abs_epi16(src1_8x16b_2);
                src1_8x16b_3 = _mm_abs_epi16(src1_8x16b_3);

                src1_8x16b_0 = _mm_add_epi16(src1_8x16b_0, src1_8x16b_1);
                src1_8x16b_2 = _mm_add_epi16(src1_8x16b_2, src1_8x16b_3);

                src1_8x16b_1 = _mm_srli_si128(src1_8x16b_0, 8);
                src1_8x16b_3 = _mm_srli_si128(src1_8x16b_2, 8);
                src1_8x16b_0 = _mm_cvtepi16_epi32(src1_8x16b_0);
                src1_8x16b_2 = _mm_cvtepi16_epi32(src1_8x16b_2);
                src1_8x16b_1 = _mm_cvtepi16_epi32(src1_8x16b_1);
                src1_8x16b_3 = _mm_cvtepi16_epi32(src1_8x16b_3);

                src1_8x16b_0 = _mm_add_epi32(src1_8x16b_0, src1_8x16b_1);
                src1_8x16b_2 = _mm_add_epi32(src1_8x16b_2, src1_8x16b_3);

                mm_result1 = _mm_add_epi32(mm_result1, src1_8x16b_0);
                mm_result2 = _mm_add_epi32(mm_result2, src1_8x16b_2);

                pu2_inp += 16;
                pu2_ref += 16;
            }

            pu2_inp = pu2_inp - ((w >> 4) << 4) + s_src1 * 2;
            pu2_ref = pu2_ref - ((w >> 4) << 4) + s_src2 * 2;
        }

        /* Last 1 row */
        if (h & 0x1)
        {
            for (j = 0; j < w; j += 16)
            {
                src2_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_ref));
                src2_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_ref + 8));

                src1_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_inp));
                src1_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_inp + 8));

                src2_8x16b_0 = _mm_slli_epi16(src2_8x16b_0, 1);
                src2_8x16b_1 = _mm_slli_epi16(src2_8x16b_1, 1);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, mm_delta);
                src1_8x16b_1 = _mm_sub_epi16(src1_8x16b_1, mm_delta);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, src2_8x16b_0);
                src1_8x16b_1 = _mm_sub_epi16(src1_8x16b_1, src2_8x16b_1);

                src1_8x16b_0 = _mm_abs_epi16(src1_8x16b_0);
                src1_8x16b_1 = _mm_abs_epi16(src1_8x16b_1);

                src1_8x16b_0 = _mm_add_epi16(src1_8x16b_0, src1_8x16b_1);

                src1_8x16b_1 = _mm_srli_si128(src1_8x16b_0, 8);
                src1_8x16b_0 = _mm_cvtepi16_epi32(src1_8x16b_0);
                src1_8x16b_1 = _mm_cvtepi16_epi32(src1_8x16b_1);

                mm_result1 = _mm_add_epi32(mm_result1, src1_8x16b_0);
                mm_result2 = _mm_add_epi32(mm_result2, src1_8x16b_1);

                pu2_inp += 16;
                pu2_ref += 16;
            }
        }
    }

    rem_w &= 0xF;

    /* 8 pixels at a time */
    if (rem_w > 7)
    {
        pu2_inp = (short *)src1 + ((w >> 4) << 4);
        pu2_ref = (short *)src2 + ((w >> 4) << 4);

        for (i = h; i > 3; i -= 4)
        {
            for (j = 0; j < w; j += 8)
            {
                src2_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_ref));
                src2_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_ref + s_src2));
                src2_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_ref + (s_src2 * 2)));
                src2_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_ref + (s_src2 * 3)));

                src1_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_inp));
                src1_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_inp + s_src1));
                src1_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_inp + (s_src1 * 2)));
                src1_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_inp + (s_src1 * 3)));

                src2_8x16b_0 = _mm_slli_epi16(src2_8x16b_0, 1);
                src2_8x16b_1 = _mm_slli_epi16(src2_8x16b_1, 1);
                src2_8x16b_2 = _mm_slli_epi16(src2_8x16b_2, 1);
                src2_8x16b_3 = _mm_slli_epi16(src2_8x16b_3, 1);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, mm_delta);
                src1_8x16b_1 = _mm_sub_epi16(src1_8x16b_1, mm_delta);
                src1_8x16b_2 = _mm_sub_epi16(src1_8x16b_2, mm_delta);
                src1_8x16b_3 = _mm_sub_epi16(src1_8x16b_3, mm_delta);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, src2_8x16b_0);
                src1_8x16b_1 = _mm_sub_epi16(src1_8x16b_1, src2_8x16b_1);
                src1_8x16b_2 = _mm_sub_epi16(src1_8x16b_2, src2_8x16b_2);
                src1_8x16b_3 = _mm_sub_epi16(src1_8x16b_3, src2_8x16b_3);

                src1_8x16b_0 = _mm_abs_epi16(src1_8x16b_0);
                src1_8x16b_1 = _mm_abs_epi16(src1_8x16b_1);
                src1_8x16b_2 = _mm_abs_epi16(src1_8x16b_2);
                src1_8x16b_3 = _mm_abs_epi16(src1_8x16b_3);

                src1_8x16b_0 = _mm_add_epi16(src1_8x16b_0, src1_8x16b_1);
                src1_8x16b_2 = _mm_add_epi16(src1_8x16b_2, src1_8x16b_3);

                src1_8x16b_1 = _mm_srli_si128(src1_8x16b_0, 8);
                src1_8x16b_3 = _mm_srli_si128(src1_8x16b_2, 8);
                src1_8x16b_0 = _mm_cvtepi16_epi32(src1_8x16b_0);
                src1_8x16b_2 = _mm_cvtepi16_epi32(src1_8x16b_2);
                src1_8x16b_1 = _mm_cvtepi16_epi32(src1_8x16b_1);
                src1_8x16b_3 = _mm_cvtepi16_epi32(src1_8x16b_3);

                src1_8x16b_0 = _mm_add_epi32(src1_8x16b_0, src1_8x16b_1);
                src1_8x16b_2 = _mm_add_epi32(src1_8x16b_2, src1_8x16b_3);

                mm_result1 = _mm_add_epi32(mm_result1, src1_8x16b_0);
                mm_result2 = _mm_add_epi32(mm_result2, src1_8x16b_2);

                pu2_inp += 8;
                pu2_ref += 8;
            }

            pu2_inp = pu2_inp - ((rem_w >> 3) << 3) + s_src1 * 4;
            pu2_ref = pu2_ref - ((rem_w >> 3) << 3) + s_src2 * 4;
        }

        /* Rem. Last rows (max 3) */
        for (i = 0; i < (h & 3); i++)
        {
            for (j = 0; j < w; j += 8)
            {
                src2_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_ref));
                src1_8x16b_0 = _mm_loadu_si128((__m128i *) (pu2_inp));

                src2_8x16b_0 = _mm_slli_epi16(src2_8x16b_0, 1);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, mm_delta);
                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, src2_8x16b_0);

                src1_8x16b_0 = _mm_abs_epi16(src1_8x16b_0);

                src1_8x16b_1 = _mm_srli_si128(src1_8x16b_0, 8);
                src1_8x16b_0 = _mm_cvtepi16_epi32(src1_8x16b_0);
                src1_8x16b_1 = _mm_cvtepi16_epi32(src1_8x16b_1);

                mm_result1 = _mm_add_epi32(mm_result1, src1_8x16b_0);
                mm_result2 = _mm_add_epi32(mm_result2, src1_8x16b_1);

                pu2_inp += 8;
                pu2_ref += 8;
            }

            pu2_inp = pu2_inp - ((rem_w >> 3) << 3) + s_src1;
            pu2_ref = pu2_ref - ((rem_w >> 3) << 3) + s_src2;
        }
    }

    rem_w &= 0x7;

    /* 4 pixels at a time */
    if (rem_w > 3)
    {
        pu2_inp = (short *)src1 + ((w >> 3) << 3);
        pu2_ref = (short *)src2 + ((w >> 3) << 3);

        for (i = h; i > 3; i -= 4)
        {
            for (j = 0; j < w; j += 4)
            {
                src2_8x16b_0 = _mm_loadl_epi64((__m128i *) (pu2_ref));
                src2_8x16b_1 = _mm_loadl_epi64((__m128i *) (pu2_ref + s_src2));
                src2_8x16b_2 = _mm_loadl_epi64((__m128i *) (pu2_ref + (s_src2 * 2)));
                src2_8x16b_3 = _mm_loadl_epi64((__m128i *) (pu2_ref + (s_src2 * 3)));

                src1_8x16b_0 = _mm_loadl_epi64((__m128i *) (pu2_inp));
                src1_8x16b_1 = _mm_loadl_epi64((__m128i *) (pu2_inp + s_src1));
                src1_8x16b_2 = _mm_loadl_epi64((__m128i *) (pu2_inp + (s_src1 * 2)));
                src1_8x16b_3 = _mm_loadl_epi64((__m128i *) (pu2_inp + (s_src1 * 3)));

                src2_8x16b_0 = _mm_unpacklo_epi16(src2_8x16b_0, src2_8x16b_1);
                src2_8x16b_2 = _mm_unpacklo_epi16(src2_8x16b_2, src2_8x16b_3);

                src1_8x16b_0 = _mm_unpacklo_epi16(src1_8x16b_0, src1_8x16b_1);
                src1_8x16b_2 = _mm_unpacklo_epi16(src1_8x16b_2, src1_8x16b_3);

                src2_8x16b_0 = _mm_slli_epi16(src2_8x16b_0, 1);
                src2_8x16b_2 = _mm_slli_epi16(src2_8x16b_2, 1);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, mm_delta);
                src1_8x16b_2 = _mm_sub_epi16(src1_8x16b_2, mm_delta);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, src2_8x16b_0);
                src1_8x16b_2 = _mm_sub_epi16(src1_8x16b_2, src2_8x16b_2);

                src1_8x16b_0 = _mm_abs_epi16(src1_8x16b_0);
                src1_8x16b_2 = _mm_abs_epi16(src1_8x16b_2);

                src1_8x16b_0 = _mm_add_epi16(src1_8x16b_0, src1_8x16b_2);

                src1_8x16b_1 = _mm_srli_si128(src1_8x16b_0, 8);
                src1_8x16b_0 = _mm_cvtepi16_epi32(src1_8x16b_0);
                src1_8x16b_1 = _mm_cvtepi16_epi32(src1_8x16b_1);

                mm_result1 = _mm_add_epi32(mm_result1, src1_8x16b_0);
                mm_result2 = _mm_add_epi32(mm_result2, src1_8x16b_1);

                pu2_inp += 4;
                pu2_ref += 4;
            }

            pu2_inp = pu2_inp - ((rem_w >> 2) << 2) + s_src1 * 4;
            pu2_ref = pu2_ref - ((rem_w >> 2) << 2) + s_src2 * 4;
        }

        /* Rem. Last rows (max 3) */
        for(i = 0; i < (h & 3); i++)
        {
            for(j = 0; j < w; j += 4)
            {
                src2_8x16b_0 = _mm_loadl_epi64((__m128i *) (pu2_ref));
                src1_8x16b_0 = _mm_loadl_epi64((__m128i *) (pu2_inp));

                src2_8x16b_0 = _mm_slli_epi16(src2_8x16b_0, 1);

                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, mm_delta);
                src1_8x16b_0 = _mm_sub_epi16(src1_8x16b_0, src2_8x16b_0);

                src1_8x16b_0 = _mm_abs_epi16(src1_8x16b_0);

                src1_8x16b_0 = _mm_cvtepi16_epi32(src1_8x16b_0);

                mm_result1 = _mm_add_epi32(mm_result1, src1_8x16b_0);

                pu2_inp += 4;
                pu2_ref += 4;
            }

            pu2_inp = pu2_inp - ((rem_w >> 2) << 2) + s_src1;
            pu2_ref = pu2_ref - ((rem_w >> 2) << 2) + s_src2;
        }
    }

    rem_w &= 0x3;
    if (rem_w)
    {
        pu2_inp = (short *)src1 + ((w >> 2) << 2);
        pu2_ref = (short *)src2 + ((w >> 2) << 2);

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < rem_w; j++)
            {
                mr_sad += abs(pu2_inp[j] - (pu2_ref[j] << 1) - delta);
            }
            pu2_inp += s_src1;
            pu2_ref += s_src2;
        }
    }

    mm_result1 = _mm_add_epi32(mm_result1, mm_result2);
    {
        int *val = (int*)&mm_result1;
        mr_sad += (val[0] + val[1] + val[2] + val[3]);
    }

    return (mr_sad);
}
#endif

s32 simple_sad(int w, int h, pel *src1, pel *src2, int s_src1, int s_src2, s32(*average_for_next_interation)[REF_PRED_POINTS_INDEXS_NUM], s32 *avg_borders_array, s32 *avg_extra_borders_array, EVC_POINT_INDEX point_index
               , s32 *center_point_avgs_l0_l1, u8 l0_or_l1, BOOL do_not_sad)
{
    int i, j;
    s32 sad = 0;
    s32 avg2 = 0;
    pel *src1_org = src1;
    pel *src2_org = src2;
    s32 delta;
 
    s32 mean_a;
    s32 mean_b;
    s32 mean_c;
    s32 mean_t;

    if (avg_borders_array == NULL)
    {
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                avg2 += src2[j];
            }
            src2 += s_src2;
        }
    }
    else
    {
        avg2 = center_point_avgs_l0_l1[l0_or_l1];
        switch(point_index)
        {
            case CENTER:
                // borders calculation
                // top border
                src1 = src2_org;
                src2 = src1 - REF_PRED_EXTENTION_PEL_COUNT * s_src2;
                for(j = 0; j < w; j++)
                {
                    avg_borders_array[LINE_TOP] += src1[j];
                    avg_extra_borders_array[LINE_TOP] += src2[j];
                }
                // bottom border
                src1 = src2_org + (h - REF_PRED_EXTENTION_PEL_COUNT) * s_src2;
                src2 = src1 + REF_PRED_EXTENTION_PEL_COUNT * s_src2;
                for(j = 0; j < w; j++)
                {
                    avg_borders_array[LINE_BOTTOM] += src1[j];
                    avg_extra_borders_array[LINE_BOTTOM] += src2[j];
                }
                // left border
                src1 = src2_org;
                src2 = src1 - REF_PRED_EXTENTION_PEL_COUNT;
                for(j = 0; j < h; j++)
                {
                    avg_borders_array[LINE_LEFT] += *src1;
                    avg_extra_borders_array[LINE_LEFT] += *src2;
                    src1 += s_src2;
                    src2 += s_src2;
                }
                // right border
                src1 = src2_org + w - REF_PRED_EXTENTION_PEL_COUNT;
                src2 = src1 + REF_PRED_EXTENTION_PEL_COUNT;
                for(j = 0; j < h; j++)
                {
                    avg_borders_array[LINE_RIGHT] += *src1;
                    avg_extra_borders_array[LINE_RIGHT] += *src2;
                    src1 += s_src2;
                    src2 += s_src2;
                }
                // preparing src2 averages for the next iterations
                average_for_next_interation[SRC2][CENTER_BOTTOM] = avg2 + avg_extra_borders_array[LINE_BOTTOM] - avg_borders_array[LINE_TOP];

                // center top
                average_for_next_interation[SRC2][CENTER_TOP] = avg2 + avg_extra_borders_array[LINE_TOP] - avg_borders_array[LINE_BOTTOM];

                // left center 
                average_for_next_interation[SRC2][LEFT_CENTER] = avg2 + avg_extra_borders_array[LINE_LEFT] - avg_borders_array[LINE_RIGHT];

                // right center 
                average_for_next_interation[SRC2][RIGHT_CENTER] = avg2 + avg_extra_borders_array[LINE_RIGHT] - avg_borders_array[LINE_LEFT];
                break;
            case CENTER_TOP:
            case CENTER_BOTTOM:
            case LEFT_CENTER:
            case RIGHT_CENTER:
                avg2 = average_for_next_interation[SRC2][point_index];
                break;
            case LEFT_TOP:
                // substruct right and bottom lines
                avg2 = average_for_next_interation[SRC2][CENTER];
                avg2 -= avg_borders_array[LINE_RIGHT];
                avg2 -= avg_borders_array[LINE_BOTTOM];
                // add left and top lines
                avg2 += avg_extra_borders_array[LINE_LEFT];
                avg2 += avg_extra_borders_array[LINE_TOP];
                // add left top corner pel, due to not added by lines
                avg2 += *src2;
                src2 += w;
                // substract right edge top line point
                avg2 -= *src2;
                src2 += h * s_src2;
                // add right bottom corner pel, due to substructed by lines
                avg2 += *src2;
                src2 -= w;
                // substract bottom edge left line point
                avg2 -= *src2;
                break;
            case RIGHT_TOP:
                // substruct left and bottom lines
                avg2 = average_for_next_interation[SRC2][CENTER];
                avg2 -= avg_borders_array[LINE_LEFT];
                avg2 -= avg_borders_array[LINE_BOTTOM];
                // add right and top lines
                avg2 += avg_extra_borders_array[LINE_RIGHT];
                avg2 += avg_extra_borders_array[LINE_TOP];
                src2 -= REF_PRED_EXTENTION_PEL_COUNT;
                // substract left edge top line point
                avg2 -= *src2;
                src2 += w;
                // add right top corner pel, due to not added by lines
                avg2 += *src2;
                src2 += h * s_src2;
                // substract right edge bottom line point
                avg2 -= *src2;
                src2 -= w;
                // add left bottom corner pel, due to substructed by lines
                avg2 += *src2;
                break;
            case LEFT_BOTTOM:
                // substruct right and top lines
                avg2 = average_for_next_interation[SRC2][CENTER];
                avg2 -= avg_borders_array[LINE_RIGHT];
                avg2 -= avg_borders_array[LINE_TOP];
                // add left and bottom lines
                avg2 += avg_extra_borders_array[LINE_LEFT];
                avg2 += avg_extra_borders_array[LINE_BOTTOM];
                src2 -= REF_PRED_EXTENTION_PEL_COUNT * s_src2;
                // substract left edge top line point
                avg2 -= *src2;
                src2 += w;
                // add right top corner pel, due to substructed by lines
                avg2 += *src2;
                src2 += h * s_src2;
                // substract bottom edge right line point
                avg2 -= *src2;
                src2 -= w;
                // add left bottom corner pel, due to not added by lines
                avg2 += *src2;
                break;
            case RIGHT_BOTTOM:
                // substruct left and top lines
                avg2 = average_for_next_interation[SRC2][CENTER];
                avg2 -= avg_borders_array[LINE_LEFT];
                avg2 -= avg_borders_array[LINE_TOP];
                // add right and bottom lines
                avg2 += avg_extra_borders_array[LINE_RIGHT];
                avg2 += avg_extra_borders_array[LINE_BOTTOM];
                src2 -= (REF_PRED_EXTENTION_PEL_COUNT * s_src2 + REF_PRED_EXTENTION_PEL_COUNT);
                // add left top corner pel, due to substructed by lines
                avg2 += *src2;
                src2 += w;
                // substract right edge top line point
                avg2 -= *src2;
                src2 += h * s_src2;
                // add right bottom corner pel, due to not added by lines
                avg2 += *src2;
                src2 -= w;
                // substract left edge bottom line point
                avg2 -= *src2;
                break;
            default:
                assert(!"Undefined case");
                break;
        }
        average_for_next_interation[SRC2][point_index] = avg2;
    }
    if (do_not_sad)
    {
        return 0;
    }
    src1 = src1_org;
    src2 = src2_org;

    mean_a = (((center_point_avgs_l0_l1[2 + !l0_or_l1] << 5) / (w*h)) + 16) >> 5;
    mean_b = (((center_point_avgs_l0_l1[2 + l0_or_l1] << 5) / (w*h)) + 16) >> 5;
    mean_c = (((avg2 << 5) / (w*h)) + 16) >> 5;
    mean_t = mean_a + mean_b;
    delta = mean_t - 2 * mean_c;

#if OPT_SIMD_DMVR_MR_SAD
    sad = dmvr_sad_mr_16b_sse(w, h, src1, src2, s_src1, s_src2, delta);
#else
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            sad += abs(src1[j] - (src2[j] << 1) - delta);
        }
        src1 += s_src1;
        src2 += s_src2;
    }
#endif

    return sad;
}

void generate_template(pel *dst, pel *src1, s16 *src2, s32 src1_stride, s32 src2_stride, int w, int h)
{
    int i, j;
    for (i = 0; i < h; ++i)
    {
        for (j = 0; j < w; ++j)
        {
            dst[j] = (src1[j] + src2[j]);
        }
        dst += w;
        src1 += src1_stride;
        src2 += src2_stride;
    }
}

static void mv_clip_only_one_ref(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[MV_D], s16(*mv_t))
{
    int min_clip[MV_D], max_clip[MV_D];

    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;
    min_clip[MV_X] = (-MAX_CU_SIZE) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE) << 2;

    mv_t[MV_X] = mv[MV_X];
    mv_t[MV_Y] = mv[MV_Y];

    if (x + mv[MV_X] < min_clip[MV_X]) mv_t[MV_X] = min_clip[MV_X] - x;
    if (y + mv[MV_Y] < min_clip[MV_Y]) mv_t[MV_Y] = min_clip[MV_Y] - y;
    if (x + mv[MV_X] + w - 4 > max_clip[MV_X]) mv_t[MV_X] = max_clip[MV_X] - x - w + 4;
    if (y + mv[MV_Y] + h - 4 > max_clip[MV_Y]) mv_t[MV_Y] = max_clip[MV_Y] - y - h + 4;
}

static BOOL mv_clip_only_one_ref_dmvr(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[MV_D], s16(*mv_t))
{
  BOOL clip_flag = 0;
  int min_clip[MV_D], max_clip[MV_D];

  x <<= 2;
  y <<= 2;
  w <<= 2;
  h <<= 2;
  min_clip[MV_X] = (-MAX_CU_SIZE) << 2;
  min_clip[MV_Y] = (-MAX_CU_SIZE) << 2;
  max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE) << 2;
  max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE) << 2;


  mv_t[MV_X] = mv[MV_X];
  mv_t[MV_Y] = mv[MV_Y];

  if (x + mv[MV_X] < min_clip[MV_X])
  {
    clip_flag = 1;
    mv_t[MV_X] = min_clip[MV_X] - x;
  }
  if (y + mv[MV_Y] < min_clip[MV_Y])
  {
    clip_flag = 1;
    mv_t[MV_Y] = min_clip[MV_Y] - y;
  }
  if (x + mv[MV_X] + w - 4 > max_clip[MV_X])
  {
    clip_flag = 1;
    mv_t[MV_X] = max_clip[MV_X] - x - w + 4;
  }
  if (y + mv[MV_Y] + h - 4 > max_clip[MV_Y])
  {
    clip_flag = 1;
    mv_t[MV_Y] = max_clip[MV_Y] - y - h + 4;
  }
  return clip_flag;
}



pel* refinement_motion_vectors_in_one_ref(int x, int y, int pic_w, int pic_h, int w, int h, const EVC_PIC *ref_pic, s16(*mv), pel dmvr_current_template[MAX_CU_SIZE*MAX_CU_SIZE], pel(*dmvr_ref_pred_interpolated), int stride
                                          , BOOL calculate_center, s32 *center_cost
                                          , s32 *center_point_avgs_l0_l1, u8 l0_or_l1
                                          )
{
    int BEST_COST_FROM_INTEGER = 0;
    int COST_FROM_L0 = 1;

    s16    ref_pred_mv_scaled_step = REF_PRED_EXTENTION_PEL_COUNT << 2;
    s32 offset = 0;

    s32 average_for_next_interation[SRC_NUM][REF_PRED_POINTS_INDEXS_NUM] =
    {
        { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    };

    s32 avg_extra_borders_array[LINE_NUM] = { 0, 0, 0, 0 };
    s32 avg_borders_array[LINE_NUM] = { 0, 0, 0, 0 };
    u8    points_flag[REF_PRED_POINTS_CROSS] = { FALSE, FALSE, FALSE, FALSE, FALSE };
    s32    cost[REF_PRED_POINTS_NUM] = { 1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30 };
    s32 cost_best = 1 << 30;
    EVC_POINT_INDEX point_index;
    EVC_POINT_INDEX best_cost_point = CENTER;

    // in a plus sign order, starting with the central point
    // cost calculation firstly only for 5 points in the plus sign
    for(point_index = CENTER; point_index < REF_PRED_POINTS_CROSS; ++point_index)
    {
        if(l0_or_l1 == REFP_1 && point_index == CENTER)
        {
            cost[point_index] = center_cost[COST_FROM_L0];
            simple_sad(w, h, dmvr_current_template, dmvr_ref_pred_interpolated + offset, w, stride, average_for_next_interation, avg_borders_array, avg_extra_borders_array, point_index
                       , center_point_avgs_l0_l1, l0_or_l1, TRUE
                       );
        }
        else
        {
            cost[point_index] = simple_sad(w, h, dmvr_current_template, dmvr_ref_pred_interpolated + offset, w, stride, average_for_next_interation, avg_borders_array, avg_extra_borders_array, point_index
                , center_point_avgs_l0_l1, l0_or_l1, FALSE);
        }
        switch(point_index)
        {
            case CENTER:
                if(l0_or_l1 == REFP_0)
                {
                    center_cost[COST_FROM_L0] = cost[point_index];
                }
      
                offset -= REF_PRED_EXTENTION_PEL_COUNT * stride;
                break;
            case CENTER_TOP:
                offset += (REF_PRED_EXTENTION_PEL_COUNT << 1) * stride;
                break;
            case CENTER_BOTTOM:
                offset -= (REF_PRED_EXTENTION_PEL_COUNT * stride + REF_PRED_EXTENTION_PEL_COUNT);
                break;
            case LEFT_CENTER:
                offset += (REF_PRED_EXTENTION_PEL_COUNT << 1);
                break;
            case RIGHT_CENTER:
                break;

            default:
                assert(!"Undefined case");
                break;
        }
    }
    // check which additional point we need
    if(cost[CENTER_TOP] < cost[CENTER_BOTTOM])
    {
        points_flag[CENTER_TOP] = TRUE;
    }
    else
    {
        points_flag[CENTER_BOTTOM] = TRUE;
    }

    if(cost[LEFT_CENTER] < cost[RIGHT_CENTER])
    {
        points_flag[LEFT_CENTER] = TRUE;
    }
    else
    {
        points_flag[RIGHT_CENTER] = TRUE;
    }

    // left top point
    if(points_flag[CENTER_TOP] && points_flag[LEFT_CENTER])
    {
        // obtain left top point position from last used right center point
        offset -= (REF_PRED_EXTENTION_PEL_COUNT * stride + (REF_PRED_EXTENTION_PEL_COUNT << 1));
        cost[LEFT_TOP] = simple_sad(w, h, dmvr_current_template, dmvr_ref_pred_interpolated + offset, w, stride, average_for_next_interation, avg_borders_array, avg_extra_borders_array, LEFT_TOP
                                    , center_point_avgs_l0_l1, l0_or_l1, FALSE);
    }
    // right top point
    else if(points_flag[CENTER_TOP] && points_flag[RIGHT_CENTER])
    {
        // obtain right top point position from last used right center point
        offset -= (REF_PRED_EXTENTION_PEL_COUNT * stride);
        cost[RIGHT_TOP] = simple_sad(w, h, dmvr_current_template, dmvr_ref_pred_interpolated + offset, w, stride, average_for_next_interation, avg_borders_array, avg_extra_borders_array, RIGHT_TOP
                                     , center_point_avgs_l0_l1, l0_or_l1, FALSE);

    }
    // left bottom point
    else if(points_flag[CENTER_BOTTOM] && points_flag[LEFT_CENTER])
    {
        // obtain left bottom point position from last used right center point
        offset -= (-(REF_PRED_EXTENTION_PEL_COUNT * stride) + (REF_PRED_EXTENTION_PEL_COUNT << 1));
        cost[LEFT_BOTTOM] = simple_sad(w, h, dmvr_current_template, dmvr_ref_pred_interpolated + offset, w, stride, average_for_next_interation, avg_borders_array, avg_extra_borders_array, LEFT_BOTTOM
                                       , center_point_avgs_l0_l1, l0_or_l1, FALSE);
    }
    // right bottom point
    else if(points_flag[CENTER_BOTTOM] && points_flag[RIGHT_CENTER])
    {
        // obtain right bottom point position from last used right center point
        offset += (REF_PRED_EXTENTION_PEL_COUNT * stride);
        cost[RIGHT_BOTTOM] = simple_sad(w, h, dmvr_current_template, dmvr_ref_pred_interpolated + offset, w, stride, average_for_next_interation, avg_borders_array, avg_extra_borders_array, RIGHT_BOTTOM
                                        , center_point_avgs_l0_l1, l0_or_l1, FALSE);
    }

    for(point_index = CENTER; point_index < REF_PRED_POINTS_NUM; ++point_index)
    {
        if(cost[point_index] < cost_best)
        {
            cost_best = cost[point_index];
            best_cost_point = point_index;
        }
    }

    center_cost[BEST_COST_FROM_INTEGER] = cost[best_cost_point];
    center_point_avgs_l0_l1[l0_or_l1] = average_for_next_interation[SRC2][best_cost_point];
    // refine oroginal MV
    switch(best_cost_point)
    {
        case CENTER:
            offset = 0;

            break;
        case CENTER_TOP:
            offset = -REF_PRED_EXTENTION_PEL_COUNT * stride;

            mv[MV_Y] -= ref_pred_mv_scaled_step;
            break;
        case CENTER_BOTTOM:
            offset = REF_PRED_EXTENTION_PEL_COUNT * stride;

            mv[MV_Y] += ref_pred_mv_scaled_step;
            break;
        case LEFT_CENTER:
            offset = -REF_PRED_EXTENTION_PEL_COUNT;

            mv[MV_X] -= ref_pred_mv_scaled_step;
            break;
        case RIGHT_CENTER:
            offset = REF_PRED_EXTENTION_PEL_COUNT;

            mv[MV_X] += ref_pred_mv_scaled_step;
            break;
        case LEFT_TOP:
            offset = -REF_PRED_EXTENTION_PEL_COUNT * stride - REF_PRED_EXTENTION_PEL_COUNT;

            mv[MV_X] -= ref_pred_mv_scaled_step;
            mv[MV_Y] -= ref_pred_mv_scaled_step;
            break;
        case RIGHT_TOP:
            offset = -REF_PRED_EXTENTION_PEL_COUNT * stride + REF_PRED_EXTENTION_PEL_COUNT;

            mv[MV_X] += ref_pred_mv_scaled_step;
            mv[MV_Y] -= ref_pred_mv_scaled_step;
            break;
        case LEFT_BOTTOM:
            offset = REF_PRED_EXTENTION_PEL_COUNT * stride - REF_PRED_EXTENTION_PEL_COUNT;

            mv[MV_X] -= ref_pred_mv_scaled_step;
            mv[MV_Y] += ref_pred_mv_scaled_step;
            break;
        case RIGHT_BOTTOM:
            offset = REF_PRED_EXTENTION_PEL_COUNT * stride + REF_PRED_EXTENTION_PEL_COUNT;

            mv[MV_X] += ref_pred_mv_scaled_step;
            mv[MV_Y] += ref_pred_mv_scaled_step;
            break;

        default:
            assert(!"Undefined case");
            break;
    }

    return dmvr_ref_pred_interpolated + offset;
}

void copy_to_dst(int w, int h, pel *src, pel *dst, int s_stride, int d_stride)
{
    int i;
    for (i = 0; i < h; i++)
    {
        evc_mcpy(dst, src, sizeof(pel) * w);
        src += s_stride;
        dst += d_stride;
    }
}

void predict_new_line(int x, int y, int pic_w, int pic_h, const EVC_PIC *ref_pic, const s16(*mv), const s16(*mv_current), pel *preds_array, const s16(*mv_offsets), int stride, int w, int h
                      , BOOL all_sides
                      )
{
    s16       ref_pred_mv_scaled_step = REF_PRED_EXTENTION_PEL_COUNT << 2;
    int       qpel_gmv_x, qpel_gmv_y;
    s16       mv_t[MV_D];
    s16       new_mv_t[MV_D];
    s32       pred_buffer_offset;

    if (mv_offsets[MV_X]
        || all_sides
        )
    {
        // go right
        if (mv_offsets[MV_X] > 0
            || all_sides
            )
        {
            new_mv_t[MV_X] = mv[MV_X] + ref_pred_mv_scaled_step * w;
            new_mv_t[MV_Y] = mv[MV_Y] - ref_pred_mv_scaled_step;
            mv_clip_only_one_ref(x, y, pic_w, pic_h, w, h, new_mv_t, mv_t);
            qpel_gmv_x = (x << 2) + mv_t[MV_X];
            qpel_gmv_y = (y << 2) + mv_t[MV_Y];
            pred_buffer_offset = -REF_PRED_EXTENTION_PEL_COUNT * stride + REF_PRED_EXTENTION_PEL_COUNT * w;

#if MC_PRECISION_ADD
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, stride, preds_array + pred_buffer_offset, REF_PRED_EXTENTION_PEL_COUNT, h + (REF_PRED_EXTENTION_PEL_COUNT << 1));
#else
            evc_bl_mc_l(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, stride, preds_array + pred_buffer_offset, REF_PRED_EXTENTION_PEL_COUNT, h + (REF_PRED_EXTENTION_PEL_COUNT << 1));
#endif
        }
        // go left

        if(mv_offsets[MV_X] < 0
           || all_sides
           )
        {
            new_mv_t[MV_X] = mv[MV_X] - ref_pred_mv_scaled_step;
            new_mv_t[MV_Y] = mv[MV_Y] - ref_pred_mv_scaled_step;
            mv_clip_only_one_ref(x, y, pic_w, pic_h, w, h, new_mv_t, mv_t);
            qpel_gmv_x = (x << 2) + mv_t[MV_X];
            qpel_gmv_y = (y << 2) + mv_t[MV_Y];
            pred_buffer_offset = -REF_PRED_EXTENTION_PEL_COUNT * stride - REF_PRED_EXTENTION_PEL_COUNT;

#if MC_PRECISION_ADD
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, stride, preds_array + pred_buffer_offset, REF_PRED_EXTENTION_PEL_COUNT, h + (REF_PRED_EXTENTION_PEL_COUNT << 1));
#else
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x), (qpel_gmv_y), ref_pic->s_l, stride, preds_array + pred_buffer_offset, REF_PRED_EXTENTION_PEL_COUNT, h + (REF_PRED_EXTENTION_PEL_COUNT << 1));
#endif
        }
    }

    if(mv_offsets[MV_Y]
       || all_sides
       )
    {
        // go down
        if (mv_offsets[MV_Y] > 0
            || all_sides
            )
        {
            new_mv_t[MV_X] = mv[MV_X] - ref_pred_mv_scaled_step;
            new_mv_t[MV_Y] = mv[MV_Y] + ref_pred_mv_scaled_step * (h);
            mv_clip_only_one_ref(x, y, pic_w, pic_h, w, h, new_mv_t, mv_t);
            qpel_gmv_x = (x << 2) + mv_t[MV_X];
            qpel_gmv_y = (y << 2) + mv_t[MV_Y];
            pred_buffer_offset = REF_PRED_EXTENTION_PEL_COUNT * stride * h - REF_PRED_EXTENTION_PEL_COUNT;

#if MC_PRECISION_ADD
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, stride, preds_array + pred_buffer_offset, w + (REF_PRED_EXTENTION_PEL_COUNT << 1), REF_PRED_EXTENTION_PEL_COUNT);
#else
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x), (qpel_gmv_y), ref_pic->s_l, stride, preds_array + pred_buffer_offset, w + (REF_PRED_EXTENTION_PEL_COUNT << 1), REF_PRED_EXTENTION_PEL_COUNT);
#endif
        }
        // go up
        if (mv_offsets[MV_Y] < 0
            || all_sides
            )
        {
            new_mv_t[MV_X] = mv[MV_X] - ref_pred_mv_scaled_step;
            new_mv_t[MV_Y] = mv[MV_Y] - ref_pred_mv_scaled_step;
            mv_clip_only_one_ref(x, y, pic_w, pic_h, w, h, new_mv_t, mv_t);
            qpel_gmv_x = (x << 2) + mv_t[MV_X];
            qpel_gmv_y = (y << 2) + mv_t[MV_Y];
            pred_buffer_offset = -REF_PRED_EXTENTION_PEL_COUNT * stride - REF_PRED_EXTENTION_PEL_COUNT;

#if MC_PRECISION_ADD
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, stride, preds_array + pred_buffer_offset, w + (REF_PRED_EXTENTION_PEL_COUNT << 1), REF_PRED_EXTENTION_PEL_COUNT);
#else
            evc_bl_mc_l(ref_pic->y, (qpel_gmv_x), (qpel_gmv_y), ref_pic->s_l, stride, preds_array + pred_buffer_offset, w + (REF_PRED_EXTENTION_PEL_COUNT << 1), REF_PRED_EXTENTION_PEL_COUNT);
#endif
        }
    }
}

s32 evc_DMVR_cost(int w, int h, pel *src1, pel *src2, int s_src1, int s_src2
#if USE_MR_SAD
  , int mean_l0, int mean_l1
#endif
)
{
  s32 sad = 0;
  s32 i, j;
  s32 row_sum_l0, row_sum_l1;
  s32 delta = 0;
  pel *src1_temp;
  pel *src2_temp;
#if USE_MR_SAD
  delta = (mean_l0 - mean_l1) / (w*h);
#endif
  src1_temp = src1;
  src2_temp = src2;
  for (i = 0; i < h; i++)
  {
    row_sum_l0 = 0;
    row_sum_l1 = 0;
    for (j = 0; j < w; j++)
    {
      sad += abs(src1_temp[j] - src2_temp[j] - delta);
      row_sum_l0 += src1_temp[j];
      row_sum_l1 += src2_temp[j];
    }
    delta = ((row_sum_l0 - row_sum_l1) / w);
    src1_temp += s_src1;
    src2_temp += s_src2;
  }
  return sad;
}
#if USE_MR_SAD
void evc_block_sum(pel *blk, int s_blk, int w, int h, int *sum)
{
  pel *blk_temp = blk;
  *sum = 0;
  if (w == 1)
  {
    for (int j = 0; j < h; j++)
    {
      *sum += *blk_temp;
      blk_temp += s_blk;
    }
  }
  else
  {
    for (int j = 0; j < h; j++)
    {
      for (int i = 0; i < w; i += 4)
      {
        *sum += blk_temp[i + 0];
        *sum += blk_temp[i + 1];
        *sum += blk_temp[i + 2];
        *sum += blk_temp[i + 3];
      }
      blk_temp += s_blk;
    }
  }
}
#endif
void evc_DMVR_refine(int w, int h, pel *ref_l0, int s_ref_l0, pel *ref_l1, int s_ref_l1, 
#if USE_MR_SAD
  s32 *centre_mean_l0, s32 *centre_mean_l1,
#endif
  s32 *minCost, s16 *delta_mvX, s16 *delta_mvY, s32 *SAD_Array)
{
  enum SAD_POINT_INDEX idx;
  s32 LineMeanL0[4] = { 0, 0, 0, 0 };
  s32 LineMeanL1[4] = { 0, 0, 0, 0 };
  s32 meanL0 = 0, meanL1 = 0;
  s32 searchOffsetX[5] = { 0,  0, 1, -1, 0 };
  s32 searchOffsetY[5] = { 1, -1, 0,  0, 0 };
  pel *ref_l0_Orig = ref_l0;
  pel *ref_l1_Orig = ref_l1;
#if USE_MR_SAD
  s32, best_meanL0, best_meanL1;
  best_meanL0 = *centre_mean_l0;
  best_meanL1 = *centre_mean_l1;
#endif
  for (idx = SAD_BOTTOM; idx <= SAD_TOP_LEFT; ++idx)
  {
    int sum = 0;
    ref_l0 = ref_l0_Orig + searchOffsetX[idx] + (searchOffsetY[idx] * s_ref_l0);
    ref_l1 = ref_l1_Orig - searchOffsetX[idx] - (searchOffsetY[idx] * s_ref_l1);
#if USE_MR_SAD
    switch (idx)
    {
    case SAD_BOTTOM:
      evc_block_sum(ref_l0 + (h - 1)*s_ref_l0, s_ref_l0, w, 1, &sum);
      meanL0 = sum;
      evc_block_sum(ref_l0 - s_ref_l0, s_ref_l0, w, 1, &sum);
      LineMeanL0[3] = meanL0 - sum;
      meanL0 += *centre_mean_l0 - sum;

      evc_block_sum(ref_l1, s_ref_l1, w, 1, &sum);
      meanL1 = sum;
      evc_block_sum(ref_l1 + h*s_ref_l1, s_ref_l1, w, 1, &sum);
      LineMeanL1[2] = meanL1 - sum;
      meanL1 += *centre_mean_l1 - sum;
      break;
    case SAD_TOP:
      evc_block_sum(ref_l0, s_ref_l0, w, 1, &sum);
      meanL0 = sum;
      evc_block_sum(ref_l0 + h*s_ref_l0, s_ref_l0, w, 1, &sum);
      LineMeanL0[2] = meanL0 - sum;
      meanL0 += *centre_mean_l0 - sum;

      evc_block_sum(ref_l1 + (h - 1)*s_ref_l1, s_ref_l1, w, 1, &sum);
      meanL1 = sum;
      evc_block_sum(ref_l1 - s_ref_l1, s_ref_l1, w, 1, &sum);
      LineMeanL1[3] = meanL1 - sum;
      meanL1 += *centre_mean_l1 - sum;
      break;
    case SAD_RIGHT:
      evc_block_sum(ref_l0 + w - 1, s_ref_l0, 1, h, &sum);
      meanL0 = sum;
      evc_block_sum(ref_l0 - 1, s_ref_l0, 1, h, &sum);
      LineMeanL0[1] = meanL0 - sum;
      meanL0 += *centre_mean_l0 - sum;

      evc_block_sum(ref_l1, s_ref_l1, 1, h, &sum);
      meanL1 = sum;
      evc_block_sum(ref_l1 + w, s_ref_l1, 1, h, &sum);
      LineMeanL1[0] = meanL1 - sum;
      meanL1 += *centre_mean_l1 - sum;
      break;
    case SAD_LEFT:
      evc_block_sum(ref_l0, s_ref_l0, 1, h, &sum);
      meanL0 = sum;
      evc_block_sum(ref_l0 + w, s_ref_l0, 1, h, &sum);
      LineMeanL0[0] = meanL0 - sum;
      meanL0 += *centre_mean_l0 - sum;

      evc_block_sum(ref_l1 + w - 1, s_ref_l1, 1, h, &sum);
      meanL1 = sum;
      evc_block_sum(ref_l1 - 1, s_ref_l1, 1, h, &sum);
      LineMeanL1[1] = meanL1 - sum;
      meanL1 += *centre_mean_l1 - sum;
      break;
    default:
      assert(idx == SAD_TOP_LEFT);
      assert(s_ref_l1 == s_ref_l0);
      const s32 x = (searchOffsetX[idx] == -1) ? 0 : -1;
      const s32 y = (searchOffsetY[idx] == -1) ? 0 : -s_ref_l0;
      const s32 yIndex = (searchOffsetY[idx] == -1) ? 0 : -1;

      const s32 xL1 = (x) ? 0 : -1;
      const s32 yL1 = (y) ? 0 : -s_ref_l0;
      const s32 yIndexL1 = (yIndex) ? 0 : -1;

      const s32 sign = searchOffsetX[idx] * searchOffsetY[idx];

      meanL0 = *centre_mean_l0 + LineMeanL0[-x] + LineMeanL0[2 - yIndex] + sign * ref_l0[x + y] - sign * ref_l0[y + (s32)w + x]
        - sign * ref_l0[x + h*s_ref_l0 + y] + sign * ref_l0[h*s_ref_l0 + (s32)w + y + x];
      meanL1 = *centre_mean_l1 + LineMeanL1[-xL1] + LineMeanL1[2 - yIndexL1] + sign * ref_l1[xL1 + yL1] - sign * ref_l1[yL1 + (s32)w + xL1]
        - sign * ref_l1[xL1 + h*s_ref_l1 + yL1] + sign * ref_l1[h*s_ref_l1 + (s32)w + yL1 + xL1];
    }/*end of switch*/
#endif
    s32 cost = evc_DMVR_cost(w, h, ref_l0, ref_l1, s_ref_l0, s_ref_l1
#if USE_MR_SAD
      , meanL0, meanL1
#endif
    );
    *(SAD_Array + idx) = cost;
    if (idx == SAD_LEFT)
    {
      s32 down = -1, right = -1;
      if (*(SAD_Array + SAD_BOTTOM) < *(SAD_Array + SAD_TOP))
      {
        down = 1;
      }
      if (*(SAD_Array + SAD_RIGHT) < *(SAD_Array + SAD_LEFT))
      {
        right = 1;
      }
      searchOffsetX[SAD_TOP_LEFT] = right;
      searchOffsetY[SAD_TOP_LEFT] = down;
    }
    if (cost < *minCost)
    {
      *minCost = cost;
#if USE_MR_SAD
      best_meanL0 = meanL0;
      best_meanL1 = meanL1;
#endif
      *delta_mvX = searchOffsetX[idx];
      *delta_mvY = searchOffsetY[idx];
    }
  }/*end of search point loop*/
  ref_l0 = ref_l0_Orig;
  ref_l1 = ref_l1_Orig;
#if USE_MR_SAD
  *centre_mean_l0 = best_meanL0;
  *centre_mean_l1 = best_meanL1;
#endif
}
INLINE s32 div_for_maxq7(s64 N, s64 D)
{
  s32 sign, q;

  sign = 0;
  if (N < 0)
{
    sign = 1;
    N = -N;
  }

  q = 0;
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  if (N >= (D >> 1))
    q++;

  if (sign)
    return (-q);
  return(q);
}
void evc_SubPelErrorSrfc(
  int *sadBuffer,
  int *deltaMv
)
{
  s64 iNum, iDenom;
  int iMvDelta_SubPel;
  int MvSubPel_lvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
                                                        /*horizontal*/
  iNum = (s64)((sadBuffer[1] - sadBuffer[3]) << MvSubPel_lvl);
  iDenom = (s64)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

  if (0 != iDenom)
  {
    if ((sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]))
    {
      iMvDelta_SubPel = div_for_maxq7(iNum, iDenom);
      deltaMv[0] = (iMvDelta_SubPel);
    }
    else
    {
      if (sadBuffer[1] == sadBuffer[0])
      {
        deltaMv[0] = -8;// half pel
      }
      else
      {
        deltaMv[0] = 8;// half pel
      }
    }
  }
  /*vertical*/
  iNum = (s64)((sadBuffer[2] - sadBuffer[4]) << MvSubPel_lvl);
  iDenom = (s64)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));
  if (0 != iDenom)
  {
    if ((sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]))
    {
      iMvDelta_SubPel = div_for_maxq7(iNum, iDenom);
      deltaMv[1] = (iMvDelta_SubPel);
    }
    else
    {
      if (sadBuffer[2] == sadBuffer[0])
      {
        deltaMv[1] = -8;// half pel
      }
      else
      {
        deltaMv[1] = 8;// half pel
      }
    }
  }
  return;
}

#if DMVR_PADDING
void copy_buffer(pel *src, int src_stride, pel *dst, int dst_stride, int width, int height)
{
  int numBytes = width * sizeof(pel);
  for (int i = 0; i < height; i++)
  {
    memcpy(dst + i * dst_stride, src + i * src_stride, numBytes);
  }
}

void padding(pel *ptr, int iStride, int iWidth, int iHeight, int PadLeftsize, int PadRightsize, int PadTopsize, int PadBottomSize)
{
  /*left padding*/
  pel *ptr_temp = ptr;
  int offset = 0;
  for (int i = 0; i < iHeight; i++)
  {
    offset = iStride * i;
    for (int j = 1; j <= PadLeftsize; j++)
    {
      *(ptr_temp - j + offset) = *(ptr_temp + offset);
    }
  }
  /*Right padding*/
  ptr_temp = ptr + (iWidth - 1);
  for (int i = 0; i < iHeight; i++)
  {
    offset = iStride * i;
    for (int j = 1; j <= PadRightsize; j++)
    {
      *(ptr_temp + j + offset) = *(ptr_temp + offset);
    }
  }
  /*Top padding*/
  int numBytes = (iWidth + PadLeftsize + PadRightsize) * sizeof(pel);
  ptr_temp = (ptr - PadLeftsize);
  for (int i = 1; i <= PadTopsize; i++)
  {
    memcpy(ptr_temp - (i * iStride), (ptr_temp), numBytes);
  }
  /*Bottom padding*/
  numBytes = (iWidth + PadLeftsize + PadRightsize) * sizeof(pel);
  ptr_temp = (ptr + (iStride * (iHeight - 1)) - PadLeftsize);
  for (int i = 1; i <= PadBottomSize; i++)
  {
    memcpy(ptr_temp + (i * iStride), (ptr_temp), numBytes);
  }
}

void prefetch_for_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*mv)[MV_D], EVC_REFP(*refp)[REFP_NUM]
  , int iteration
  , pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
)
{
  s16          mv_temp[REFP_NUM][MV_D];
  int l_w = w, l_h = h;
  int c_w = w >> 1, c_h = h >> 1;
  int num_extra_pixel_left_for_filter;
  for (int i = 0; i < REFP_NUM; ++i)
  {
    int filtersize = NTAPS_LUMA;
    num_extra_pixel_left_for_filter = ((filtersize >> 1) - 1);
    int offset = ((DMVR_ITER_COUNT) * (PAD_BUFFER_STRIDE + 1));
    int padsize = DMVR_PAD_LENGTH;
    int          qpel_gmv_x, qpel_gmv_y;
    EVC_PIC    *ref_pic;
    mv_clip_only_one_ref_dmvr(x, y, pic_w, pic_h, w, h, mv[i], mv_temp[i]);
    
    qpel_gmv_x = ((x << 2) + mv_temp[i][MV_X]) << 2;
    qpel_gmv_y = ((y << 2) + mv_temp[i][MV_Y]) << 2;

    ref_pic = refp[refi[i]][i].pic;
    pel *ref = ref_pic->y + ((qpel_gmv_y >> 4) - num_extra_pixel_left_for_filter) * ref_pic->s_l +
                             (qpel_gmv_x >> 4) - num_extra_pixel_left_for_filter;

    pel *dst = dmvr_padding_buf[i][0] + offset;
    copy_buffer(ref, ref_pic->s_l, dst, PAD_BUFFER_STRIDE, (l_w + filtersize), (l_h + filtersize));

    padding(dst, PAD_BUFFER_STRIDE, (l_w + filtersize - 1), (l_h + filtersize - 1), padsize,
      padsize, padsize, padsize);

    // chroma
    filtersize = NTAPS_CHROMA;
    num_extra_pixel_left_for_filter = ((filtersize >> 1) - 1);
    offset = (DMVR_ITER_COUNT);
    offset = offset *(PAD_BUFFER_STRIDE + 1);
    padsize = DMVR_PAD_LENGTH >> 1;

    ref = ref_pic->u + ((qpel_gmv_y >> 5) - 1) * ref_pic->s_c + (qpel_gmv_x >> 5) - 1;
    dst = dmvr_padding_buf[i][1] + offset;
    copy_buffer(ref, ref_pic->s_c, dst, PAD_BUFFER_STRIDE, (c_w  + filtersize), (c_h + filtersize));
    padding(dst, PAD_BUFFER_STRIDE, (c_w + filtersize - 1), (c_h + filtersize - 1), padsize,
      padsize, padsize, padsize);

    ref = ref_pic->v + ((qpel_gmv_y >> 5) - 1) * ref_pic->s_c + (qpel_gmv_x >> 5) - 1;
    dst = dmvr_padding_buf[i][2] + offset;
    copy_buffer(ref, ref_pic->s_c, dst, PAD_BUFFER_STRIDE, (c_w + filtersize), (c_h + filtersize));
    padding(dst, PAD_BUFFER_STRIDE, (c_w + filtersize - 1), (c_h + filtersize - 1), padsize,
      padsize, padsize, padsize);
  }
}

#endif

void final_paddedMC_forDMVR(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*inital_mv)[MV_D], s16(*refined_mv)[MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[REFP_NUM][N_C][MAX_CU_DIM]
, int sub_pred_offset_x
, int sub_pred_offset_y
, int cu_pred_stride
#if DMVR_PADDING
, pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
#endif
)
{
  int i;
  EVC_PIC    *ref_pic;
  s16          mv_temp[REFP_NUM][MV_D];
  for (i = 0; i < REFP_NUM; ++i)
  {
    int          qpel_gmv_x, qpel_gmv_y;

    ref_pic = refp[refi[i]][i].pic;

    s16 temp_uncliped_mv[MV_D] = { refined_mv[i][MV_X] >> 2, refined_mv[i][MV_Y] >> 2 };

    BOOL clip_flag = mv_clip_only_one_ref_dmvr(x, y, pic_w, pic_h, w, h, temp_uncliped_mv, mv_temp[i]);

    if (clip_flag)
    {
      qpel_gmv_x = (x << 4) + (mv_temp[i][MV_X] << 2);
      qpel_gmv_y = (y << 4) + (mv_temp[i][MV_Y] << 2);
    }
    else
    {
      qpel_gmv_x = (x << 4) + (refined_mv[i][MV_X]);
      qpel_gmv_y = (y << 4) + (refined_mv[i][MV_Y]);
    }

#if MC_PRECISION_ADD
#if DMVR_PADDING
    int delta_x_l = 0;
    int delta_y_l = 0;
    int delta_x_c = 0;
    int delta_y_c = 0;
    int offset = 0;
    int filter_size = NTAPS_LUMA;
    int num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);

    if (clip_flag == 0)
    {
      // int pixel movement from inital mv
      delta_x_l = (refined_mv[i][MV_X] >> 4) - (inital_mv[i][MV_X] >> 2);
      delta_y_l = (refined_mv[i][MV_Y] >> 4) - (inital_mv[i][MV_Y] >> 2);

      delta_x_c = (refined_mv[i][MV_X] >> 5) - (inital_mv[i][MV_X] >> 3);
      delta_y_c = (refined_mv[i][MV_Y] >> 5) - (inital_mv[i][MV_Y] >> 3);
    }
    else
    {
      // int pixel movement from inital mv
      delta_x_l = (mv_temp[i][MV_X] >> 2) - (inital_mv[i][MV_X] >> 2);
      delta_y_l = (mv_temp[i][MV_Y] >> 2) - (inital_mv[i][MV_Y] >> 2);

      delta_x_c = (mv_temp[i][MV_X] >> 3) - (inital_mv[i][MV_X] >> 3);
      delta_y_c = (mv_temp[i][MV_Y] >> 3) - (inital_mv[i][MV_Y] >> 3);
    }
    offset = (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
    offset += (delta_y_l)* PAD_BUFFER_STRIDE;
    offset += (delta_x_l);

    pel *src = dmvr_padding_buf[i][0] + offset + sub_pred_offset_x + sub_pred_offset_y * PAD_BUFFER_STRIDE;;
    pel *temp = pred[i][Y_C] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
    evc_dmvr_mc_l(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp, w, h);
    filter_size = NTAPS_CHROMA;
    num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);
    offset = (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
    offset += (delta_y_c)* PAD_BUFFER_STRIDE;
    offset += (delta_x_c);
    src = dmvr_padding_buf[i][1] + offset + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * PAD_BUFFER_STRIDE;;
    temp = pred[i][U_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
    evc_dmvr_mc_c(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride >> 1, temp, w >> 1, h >> 1);

    src = dmvr_padding_buf[i][2] + offset + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * PAD_BUFFER_STRIDE;;
    temp = pred[i][V_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
    evc_dmvr_mc_c(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride >> 1, temp, w >> 1, h >> 1);
#else
    pel *temp = pred[i][Y_C] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
    evc_mc_l(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, cu_pred_stride, temp, w, h);
    temp = pred[i][U_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
    evc_mc_c(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cu_pred_stride >> 1, temp, w >> 1, h >> 1);
    temp = pred[i][V_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
    evc_mc_c(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cu_pred_stride >> 1, temp, w >> 1, h >> 1);
#endif
#else
    assert(false);  
#endif
    {
      /*  s16          mv_t[REFP_NUM][MV_D];
        mv_clip_only_one_ref(x, y, pic_w, pic_h, w, h, mv_temp[i], mv_t[i]);
        qpel_gmv_x = (x << 2) + mv_t[i][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[i][MV_Y];
        evc_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[i][Y_C], w, h);
        evc_mc_c(ref_pic->u, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[i][U_C], w >> 1, h >> 1);
        evc_mc_c(ref_pic->v, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[i][V_C], w >> 1, h >> 1);*/
    }
  }
}


void processDMVR(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*mv)[MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[REFP_NUM][N_C][MAX_CU_DIM], \
  int poc_c, pel *dmvr_current_template, pel dmvr_ref_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))]
  , pel dmvr_half_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)]
  , int iteration
#if DMVR_PADDING
  , pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
#endif
#if DMVR_LAG
  , s16 dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D]
#endif
)
{
#if DMVR_SUBCU
  s16 sub_pu_L0[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];
  s16 sub_pu_L1[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];
#endif

  int stride = w + (iteration << 1);
  s16 ref_pred_mv_scaled_step = 2;

  s16 tempMv[MV_D];
  s16 refined_mv[REFP_NUM][MV_D];

  s16 starting_mv[REFP_NUM][MV_D];
  mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, starting_mv);

  int          qpel_gmv_x, qpel_gmv_y;
  EVC_PIC    *ref_pic;

  // centre address holder for pred
  pel          *preds_array[REFP_NUM];
  preds_array[REFP_0] = dmvr_ref_pred_interpolated[REFP_0];
  preds_array[REFP_1] = dmvr_ref_pred_interpolated[REFP_1];

  // REF_PIC_LIST_0
  ref_pic = refp[refi[REFP_0]][REFP_0].pic;
  //produce iteration lines extra
  tempMv[MV_X] = starting_mv[REFP_0][MV_X] - (iteration << ref_pred_mv_scaled_step);
  tempMv[MV_Y] = starting_mv[REFP_0][MV_Y] - (iteration << ref_pred_mv_scaled_step);
  qpel_gmv_x = (x << 2) + tempMv[MV_X];
  qpel_gmv_y = (y << 2) + tempMv[MV_Y];
  evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, stride, preds_array[REFP_0], (w + iteration * 2), (h + iteration * 2));

  // REF_PIC_LIST_1
  ref_pic = refp[refi[REFP_1]][REFP_1].pic;
  //produce iteration lines extra
  tempMv[MV_X] = starting_mv[REFP_1][MV_X] - (iteration << ref_pred_mv_scaled_step);
  tempMv[MV_Y] = starting_mv[REFP_1][MV_Y] - (iteration << ref_pred_mv_scaled_step);
  qpel_gmv_x = (x << 2) + tempMv[MV_X];
  qpel_gmv_y = (y << 2) + tempMv[MV_Y];
  evc_bl_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, stride, preds_array[REFP_1], (w + iteration * 2), (h + iteration * 2));

  // go to the center point
  pel *preds_centre_array[REFP_NUM];
  preds_centre_array[REFP_0] = preds_array[REFP_0] + (stride * iteration + iteration);
  preds_centre_array[REFP_1] = preds_array[REFP_1] + (stride * iteration + iteration);

  int minCost = INT_MAX;

  int lastDirection = -1;

  int arrayCost[SAD_COUNT];

#if USE_MR_SAD
  int avg_l0 = 0;
  int avg_l1 = 0;
#endif

  int dx, dy;
#if DMVR_SUBCU
  dy = min(h, DMVR_SUBCU_SIZE);
  dx = min(w, DMVR_SUBCU_SIZE);
#else
  dy = h;
  dx = w;
#endif

  int num = 0;
  int subPuStartX, subPuStartY, startX, startY;
  for (startY = 0, subPuStartY = y; subPuStartY < (y + h); subPuStartY = subPuStartY + dy, startY += dy)
  {
    for (startX = 0, subPuStartX = x; subPuStartX < (x + w); subPuStartX = subPuStartX + dx, startX += dx)
    {
      s16 total_delta_mv[MV_D] = { 0, 0 };
      BOOL notZeroCost = 1;

      pel *addr_subpu_l0 = preds_centre_array[REFP_0] + startX + startY * stride;
      pel *addr_subpu_l1 = preds_centre_array[REFP_1] + startX + startY * stride;

      for (int i = 0; i < iteration; i++)
      {
        s16 delta_mv[MV_D] = { 0, 0 };
        pel *addr_l0 = addr_subpu_l0 + (total_delta_mv[MV_X] + total_delta_mv[MV_Y] * stride);
        pel *addr_l1 = addr_subpu_l1 - (total_delta_mv[MV_X] + total_delta_mv[MV_Y] * stride);

        for (int loop = 0; loop < SAD_COUNT; loop++)
        {
          arrayCost[loop] = INT_MAX;
        }

        if (i == 0)
        {
#if USE_MR_SAD
          evc_block_sum(addr_l0, stride, dx, dy, &avg_l0);
          evc_block_sum(addr_l1, stride, dx, dy, &avg_l1);
#endif

          minCost = evc_DMVR_cost(dx, dy, addr_l0, addr_l1, stride, stride
#if USE_MR_SAD /*meanl0, mean l1*/
            , avg_l0
            , avg_l1
#endif
          );
        }

        if (minCost == 0)
        {
          notZeroCost = 0;
          break;
        }
        arrayCost[SAD_CENTER] = minCost;
        evc_DMVR_refine(dx, dy, addr_l0, stride, addr_l1, stride
#if USE_MR_SAD
          , &avg_l0, &avg_l1
#endif      
          , &minCost
          , &delta_mv[MV_X], &delta_mv[MV_Y]
          , arrayCost);

        if (delta_mv[MV_X] == 0 && delta_mv[MV_Y] == 0)
        {
          break;
        }
        total_delta_mv[MV_X] += delta_mv[MV_X];
        total_delta_mv[MV_Y] += delta_mv[MV_Y];
      }

      total_delta_mv[MV_X] = (total_delta_mv[MV_X] << 4);
      total_delta_mv[MV_Y] = (total_delta_mv[MV_Y] << 4);

      if (notZeroCost && (minCost == arrayCost[SAD_CENTER]))
      {
        int sadbuffer[5];
        int deltaMv[MV_D] = { 0, 0 };
        sadbuffer[0] = arrayCost[SAD_CENTER];
        sadbuffer[1] = arrayCost[SAD_LEFT];
        sadbuffer[2] = arrayCost[SAD_TOP];
        sadbuffer[3] = arrayCost[SAD_RIGHT];
        sadbuffer[4] = arrayCost[SAD_BOTTOM];
        evc_SubPelErrorSrfc(sadbuffer, deltaMv);

        total_delta_mv[MV_X] += deltaMv[MV_X];
        total_delta_mv[MV_Y] += deltaMv[MV_Y];
      }

      refined_mv[REFP_0][MV_X] = (starting_mv[REFP_0][MV_X] << 2) + (total_delta_mv[MV_X]);
      refined_mv[REFP_0][MV_Y] = (starting_mv[REFP_0][MV_Y] << 2) + (total_delta_mv[MV_Y]);

      refined_mv[REFP_1][MV_X] = (starting_mv[REFP_1][MV_X] << 2) - (total_delta_mv[MV_X]);
      refined_mv[REFP_1][MV_Y] = (starting_mv[REFP_1][MV_Y] << 2) - (total_delta_mv[MV_Y]);

      /*final_paddedMC_forDMVR(subPuStartX, subPuStartY, pic_w, pic_h, dx, dy, refi, starting_mv, refined_mv, refp, pred
        , startX
        , startY
        , w
      );*/
#if DMVR_SUBCU
      sub_pu_L0[num][MV_X] = refined_mv[REFP_0][MV_X];
      sub_pu_L0[num][MV_Y] = refined_mv[REFP_0][MV_Y];

      sub_pu_L1[num][MV_X] = refined_mv[REFP_1][MV_X];
      sub_pu_L1[num][MV_Y] = refined_mv[REFP_1][MV_Y];
      num++;
#endif
#if DMVR_LAG
      u32 idx = (startX >> MIN_CU_LOG2) + ((startY >> MIN_CU_LOG2) * (w >> MIN_CU_LOG2));
      int i, j;
      for (j = 0; j < dy >> MIN_CU_LOG2; j++)
      {
        for (i = 0; i < dx >> MIN_CU_LOG2; i++)
        {
          dmvr_mv[idx + i][REFP_0][MV_X] = refined_mv[REFP_0][MV_X] >> 2;
          dmvr_mv[idx + i][REFP_0][MV_Y] = refined_mv[REFP_0][MV_Y] >> 2;

          dmvr_mv[idx + i][REFP_1][MV_X] = refined_mv[REFP_1][MV_X] >> 2;
          dmvr_mv[idx + i][REFP_1][MV_Y] = refined_mv[REFP_1][MV_Y] >> 2;
      }
        idx += w >> MIN_CU_LOG2;
    }
#endif
    }
  }

  // produce padded buffer for exact MC
#if DMVR_PADDING
  prefetch_for_mc(x, y, pic_w, pic_h, w, h, refi, starting_mv, refp, iteration, dmvr_padding_buf);
#endif

  num = 0;
  for (int startY = 0, subPuStartY = y; subPuStartY < (y + h); subPuStartY = subPuStartY + dy, startY += dy)
  {
    for (int startX = 0, subPuStartX = x; subPuStartX < (x + w); subPuStartX = subPuStartX + dx, startX += dx)
    {
#if DMVR_SUBCU
      s16 dmvr_mv[REFP_NUM][MV_D] = { { sub_pu_L0[num][MV_X], sub_pu_L0[num][MV_Y] },
                                      { sub_pu_L1[num][MV_X], sub_pu_L1[num][MV_Y] }
                                     };
#else
      s16 dmvr_mv[REFP_NUM][MV_D] = { { refined_mv[REFP_0][MV_X], refined_mv[REFP_0][MV_Y] }, 
                                      { refined_mv[REFP_1][MV_X], refined_mv[REFP_1][MV_Y] } 
                                    };
#endif
      final_paddedMC_forDMVR(subPuStartX, subPuStartY, pic_w, pic_h, dx, dy, refi, starting_mv, dmvr_mv, refp, pred
        , startX
        , startY
        , w
#if DMVR_PADDING
        , dmvr_padding_buf
#endif
      );
#if DMVR_SUBCU
      num++;
#endif
    }
  }
}

void evc_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*mv)[MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[REFP_NUM][N_C][MAX_CU_DIM], \
             int poc_c, pel *dmvr_current_template, pel dmvr_ref_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))]
             , pel dmvr_half_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)]
             , BOOL apply_DMVR

#if DMVR_PADDING
             , pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
#endif
#if DMVR_FLAG 
             , u8 *cu_dmvr_flag
#if DMVR_LAG
            , s16 dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D]
#endif
#endif
            , int sps_amis_flag
#else
void evc_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[2][N_C][MAX_CU_DIM]
#endif // DMVR
             )
{
    EVC_PIC    *ref_pic;
#if !OPT_SIMD_MC_L
    pel         *p2, *p3;
#endif
    int          qpel_gmv_x, qpel_gmv_y;
#if !OPT_SIMD_MC_L || DMVR 
#endif
    int          bidx = 0;
    s16          mv_t[REFP_NUM][MV_D];

    mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, mv_t);

#if DMVR
    int          poc0 = refp[refi[REFP_0]][REFP_0].ptr;
    int          poc1 = refp[refi[REFP_1]][REFP_1].ptr;
    s16          mv_refine[REFP_NUM][MV_D] = {{mv[REFP_0][MV_X], mv[REFP_0][MV_Y]},
                                              {mv[REFP_1][MV_X], mv[REFP_1][MV_Y]}};

    s16          inital_mv[REFP_NUM][MV_D] = { { mv[REFP_0][MV_X], mv[REFP_0][MV_Y] },
                                               { mv[REFP_1][MV_X], mv[REFP_1][MV_Y] }};

    BOOL         dmvr_poc_condition = ((BOOL)((poc_c - poc0)*(poc_c - poc1) < 0)) && (abs(poc_c - poc0) == abs(poc_c - poc1));

    s32          extend_width = (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT;
    s32          extend_width_minus1 = DMVR_NEW_VERSION_ITER_COUNT * REF_PRED_EXTENTION_PEL_COUNT;
    int          stride = w + (extend_width << 1);
    s16          mv_offsets[REFP_NUM][MV_D] = {{0,},};
    s32          center_point_avgs_l0_l1[2 * REFP_NUM] = {0, 0, 0, 0}; // center_point_avgs_l0_l1[2,3] for "A" and "B" current center point average
    int iterations_count = DMVR_ITER_COUNT;
    apply_DMVR = apply_DMVR && dmvr_poc_condition;
    apply_DMVR = apply_DMVR && (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]));
    apply_DMVR = apply_DMVR && !(refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr &&  mv_t[REFP_0][MV_X] == mv_t[REFP_1][MV_X] && mv_t[REFP_0][MV_Y] == mv_t[REFP_1][MV_Y]);
    apply_DMVR = apply_DMVR && (!((w == 4 && h <= 8) || (w <= 8 && h == 4)));
#endif
#if DMVR_FLAG
    *cu_dmvr_flag = 0;
#endif

    if(sps_amis_flag == 1)
    {
        g_mc_ftr = MC_FILTER_MAIN;
    }
    else
    {
        g_mc_ftr = MC_FILTER_BASE;
    }

    if(REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_0][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_0][MV_Y];

#if MC_PRECISION_ADD

        if(!apply_DMVR)
        {
            evc_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[0][Y_C], w, h);
        }
#if DMVR
        if(!REFI_IS_VALID(refi[REFP_1]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(ref_pic->u, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[0][U_C], w >> 1, h >> 1);
            evc_mc_c(ref_pic->v, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[0][V_C], w >> 1, h >> 1);
        }
#else

#if DMVR
        if(!apply_DMVR)
#endif
        {
            evc_mc_l(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, w, pred[0][Y_C], w, h);
        }

#if DMVR
        if(!REFI_IS_VALID(refi[REFP_1]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[0][U_C], w >> 1, h >> 1);
            evc_mc_c(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[0][V_C], w >> 1, h >> 1);
        }
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

        if(!apply_DMVR)
        {
            evc_mc_l(ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[bidx][Y_C], w, h);
        }
#if DMVR
        if(!REFI_IS_VALID(refi[REFP_0]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(ref_pic->u, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[bidx][U_C], w >> 1, h >> 1);
            evc_mc_c(ref_pic->v, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[bidx][V_C], w >> 1, h >> 1);
        }
#else

#if DMVR
        if(!apply_DMVR)
#endif
        {
            evc_mc_l(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, w, pred[bidx][Y_C], w, h);
        }

#if DMVR
        if(!REFI_IS_VALID(refi[REFP_0]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[bidx][U_C], w >> 1, h >> 1);
            evc_mc_c(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[bidx][V_C], w >> 1, h >> 1);
        }
#endif
        bidx++;
    }

    if(bidx == 2)
    {
#if DMVR
        BOOL template_needs_update = FALSE;
        s32 center_cost[2] = {1 << 30, 1 << 30};

        //only if the references are located on opposite sides of the current frame
        if(apply_DMVR && dmvr_poc_condition)
        {
          if (apply_DMVR)
          {
#if DMVR_FLAG
            *cu_dmvr_flag = 1;
#endif
            processDMVR(x, y, pic_w, pic_h, w, h, refi, mv, refp, pred, poc_c, dmvr_current_template, dmvr_ref_pred_interpolated
                , dmvr_half_pred_interpolated
                , iterations_count
#if DMVR_PADDING
                , dmvr_padding_buf
#endif
#if DMVR_LAG
                , dmvr_mv
#endif
            );
          }

          mv[REFP_0][MV_X] = inital_mv[REFP_0][MV_X];
          mv[REFP_0][MV_Y] = inital_mv[REFP_0][MV_Y];

          mv[REFP_1][MV_X] = inital_mv[REFP_1][MV_X];
          mv[REFP_1][MV_Y] = inital_mv[REFP_1][MV_Y];
        } //if (apply_DMVR && ((poc_c - poc0)*(poc_c - poc1) < 0))

#endif // DMVR

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

#if OPT_SIMD_MC_L
        w >>= 1;
        h >>= 1;
        average_16b_no_clip_sse(pred[0][U_C], pred[1][U_C], pred[0][U_C], w, w, w, w, h);
        average_16b_no_clip_sse(pred[0][V_C], pred[1][V_C], pred[0][V_C], w, w, w, w, h);
#else
        {
          pel *p0, *p1;
          int i, j;
          p0 = pred[0][U_C];
          p1 = pred[1][U_C];
          p2 = pred[0][V_C];
          p3 = pred[1][V_C];
          w >>= 1;
          h >>= 1;
          for (j = 0; j < h; j++)
          {
            for (i = 0; i < w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
                p2[i] = (p2[i] + p3[i] + 1) >> 1;
            }
            p0 += w;
            p1 += w;
            p2 += w;
            p3 += w;
          }
        }
#endif
    }
}

#if AFFINE
static void derive_affine_subblock_size(s16 ac_mv[VER_NUM][MV_D], int cuw, int cuh, int *sub_w, int *sub_h, int vertex_num)
{
    int w = cuw;
    int h = cuh;
#if MC_PRECISION_ADD
    int mc_prec_add = MC_PRECISION_ADD;
#else
    int mc_prec_add = 0;
#endif
    int mv_wx, mv_wy;

    mv_wx = max(abs(ac_mv[1][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[1][MV_Y] - ac_mv[0][MV_Y]));
    if (mv_wx)
    {
        w = max((int)((cuw >> mc_prec_add) / mv_wx), 1);
        while (cuw % w)
        {
            w--;
        }
        w = max(AFFINE_MIN_BLOCK_SIZE, w);
    }

    if (vertex_num == 2)
    {
        h = min(w, cuh);
    }
    else
    {
        mv_wy = max(abs(ac_mv[2][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[2][MV_Y] - ac_mv[0][MV_Y]));
        if (mv_wy)
        {
            h = max((int)((cuh >> mc_prec_add) / mv_wy), 1);
            while (cuh % h)
            {
                h--;
            }
            h = max(AFFINE_MIN_BLOCK_SIZE, h);
        }
    }

    *sub_w = w;
    *sub_h = h;
}

void evc_affine_mc_l(int x, int y, int pic_w, int pic_h, int cuw, int cuh, s16 ac_mv[VER_NUM][MV_D], EVC_PIC* ref_pic, pel pred[MAX_CU_DIM], int vertex_num
#if EIF
                      , pel* tmp_buffer
#endif
                      )
{
    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred;
    int sub_w, sub_h;
    int w, h;
    int half_w, half_h;
    int bit = 7;

#if MC_PRECISION_ADD
    int mc_prec = 2 + MC_PRECISION_ADD;
    int shift = bit - MC_PRECISION_ADD;
#else
    int mc_prec = 4;
    int shift = bit;
#endif
    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
    int mv_scale_hor = ac_mv[0][MV_X] << bit;
    int mv_scale_ver = ac_mv[0][MV_Y] << bit;
    int mv_scale_tmp_hor, mv_scale_tmp_ver;
    int hor_max, hor_min, ver_max, ver_min;
#if EIF
    int b_eif = 0;
    int mv_w = max(abs(ac_mv[1][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[1][MV_Y] - ac_mv[0][MV_Y]));

    if(mv_w)
    {
        w = max((int)((cuw >> 2) / mv_w), 1);
        if(w < AFFINE_ADAPT_EIF_SIZE)
            b_eif = 1;
    }
    if(vertex_num == 3 && !b_eif)
    {
        int mv_h = max(abs(ac_mv[2][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[2][MV_Y] - ac_mv[0][MV_Y]));
        if(mv_h)
        {
            h = max((int)((cuh >> 2) / mv_h), 1);
            if(h < AFFINE_ADAPT_EIF_SIZE)
                b_eif = 1;
        }
    }

    if(b_eif)
    {
        evc_affine_mc_eif(x, y, pic_w, pic_h, cuw, cuh, ac_mv, ref_pic, pred, vertex_num, tmp_buffer, Y_C);
        return;
    }
#endif

    // get clip MV Range
    hor_max = (pic_w + MAX_CU_SIZE - x - cuw + 1) << mc_prec;
    ver_max = (pic_h + MAX_CU_SIZE - y - cuh + 1) << mc_prec;
    hor_min = (-MAX_CU_SIZE - x) << mc_prec;
    ver_min = (-MAX_CU_SIZE - y) << mc_prec;

    // get sub block size
    derive_affine_subblock_size(ac_mv, cuw, cuh, &sub_w, &sub_h, vertex_num);
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = ((ac_mv[1][MV_X] - ac_mv[0][MV_X]) << bit) >> evc_tbl_log2[cuw];     // deltaMvHor
    dmv_hor_y = ((ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << bit) >> evc_tbl_log2[cuw];
    if (vertex_num == 3)
    {
        dmv_ver_x = ((ac_mv[2][MV_X] - ac_mv[0][MV_X]) << bit) >> evc_tbl_log2[cuh]; // deltaMvVer
        dmv_ver_y = ((ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << bit) >> evc_tbl_log2[cuh];
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                                       // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

    // get prediction block by block
    for(h = 0; h < cuh; h += sub_h)
    {
        for(w = 0; w < cuw; w += sub_w)
        {
            int pos_x = w + half_w;
            int pos_y = h + half_h;

            mv_scale_tmp_hor = (mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y) >> shift;
            mv_scale_tmp_ver = (mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y) >> shift;

            // clip
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));

            qpel_gmv_x = ((x + w) << mc_prec) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << mc_prec) + mv_scale_tmp_ver;

            evc_mc_l(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, cuw, (pred_y + w), sub_w, sub_h);
        }
        pred_y += (cuw * sub_h);
    }
}

void evc_affine_mc_lc(int x, int y, int pic_w, int pic_h, int cuw, int cuh, s16 ac_mv[VER_NUM][MV_D], EVC_PIC* ref_pic, pel pred[N_C][MAX_CU_DIM], int vertex_num
#if EIF
                       , pel* tmp_buffer_for_eif
#endif
                       )
{
    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred[Y_C], *pred_u = pred[U_C], *pred_v = pred[V_C];
    int sub_w, sub_h;
    int w, h;
    int half_w, half_h;
    int bit = 7;

#if MC_PRECISION_ADD
    int mc_prec = 2 + MC_PRECISION_ADD;
    int shift = bit - MC_PRECISION_ADD;
#else
    int mc_prec = 4;
    int shift = bit;
#endif
    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
    int mv_scale_hor = ac_mv[0][MV_X] << bit;
    int mv_scale_ver = ac_mv[0][MV_Y] << bit;
    int mv_scale_tmp_hor, mv_scale_tmp_ver;
    int hor_max, hor_min, ver_max, ver_min;
#if EIF
    int b_eif = 0;
    int mv_w = max(abs(ac_mv[1][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[1][MV_Y] - ac_mv[0][MV_Y]));

    if(mv_w)
    {
        w = max((int)((cuw >> 2) / mv_w), 1);
        if(w < AFFINE_ADAPT_EIF_SIZE)
            b_eif = 1;
    }
    if(vertex_num == 3 && !b_eif)
    {
        int mv_h = max(abs(ac_mv[2][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[2][MV_Y] - ac_mv[0][MV_Y]));
        if(mv_h)
        {
            h = max((int)((cuh >> 2) / mv_h), 1);
            if(h < AFFINE_ADAPT_EIF_SIZE)
                b_eif = 1;
        }
    }

    if(b_eif)
    {
        evc_affine_mc_eif(x, y, pic_w, pic_h, cuw, cuh, ac_mv, ref_pic, pred[Y_C], vertex_num, tmp_buffer_for_eif, Y_C);
        evc_affine_mc_eif(x, y, pic_w, pic_h, cuw, cuh, ac_mv, ref_pic, pred[U_C], vertex_num, tmp_buffer_for_eif, U_C);
        evc_affine_mc_eif(x, y, pic_w, pic_h, cuw, cuh, ac_mv, ref_pic, pred[V_C], vertex_num, tmp_buffer_for_eif, V_C);
        return;
    }
#endif

    // get clip MV Range
    hor_max = (pic_w + MAX_CU_SIZE - x - cuw + 1) << mc_prec;
    ver_max = (pic_h + MAX_CU_SIZE - y - cuh + 1) << mc_prec;
    hor_min = (-MAX_CU_SIZE - x) << mc_prec;
    ver_min = (-MAX_CU_SIZE - y) << mc_prec;

    // get sub block size
    derive_affine_subblock_size(ac_mv, cuw, cuh, &sub_w, &sub_h, vertex_num);
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = ((ac_mv[1][MV_X] - ac_mv[0][MV_X]) << bit) >> evc_tbl_log2[cuw];     // deltaMvHor
    dmv_hor_y = ((ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << bit) >> evc_tbl_log2[cuw];
    if(vertex_num == 3)
    {
        dmv_ver_x = ((ac_mv[2][MV_X] - ac_mv[0][MV_X]) << bit) >> evc_tbl_log2[cuh]; // deltaMvVer
        dmv_ver_y = ((ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << bit) >> evc_tbl_log2[cuh];
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                                       // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

    // get prediction block by block
    for(h = 0; h < cuh; h += sub_h)
    {
        for(w = 0; w < cuw; w += sub_w)
        {
            int pos_x = w + half_w;
            int pos_y = h + half_h;

            mv_scale_tmp_hor = (mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y) >> shift;
            mv_scale_tmp_ver = (mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y) >> shift;

            // clip
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));

            qpel_gmv_x = ((x + w) << mc_prec) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << mc_prec) + mv_scale_tmp_ver;

            evc_mc_l(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, cuw, (pred_y + w), sub_w, sub_h);

#if (AFFINE_MIN_BLOCK_SIZE == 1)
            if((w & 1) == 0 && (h & 1) == 0)
            {
                evc_mc_c(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_u + (w >> 1), max((sub_w >> 1), 1), max((sub_h >> 1), 1));
                evc_mc_c(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_v + (w >> 1), max((sub_w >> 1), 1), max((sub_h >> 1), 1));
            }
#else
            evc_mc_c(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_u + (w >> 1), sub_w >> 1, sub_h >> 1);
            evc_mc_c(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_v + (w >> 1), sub_w >> 1, sub_h >> 1);
#endif
        }

        pred_y += (cuw * sub_h);
        pred_u += (cuw * sub_h) >> 2;
        pred_v += (cuw * sub_h) >> 2;
    }
}

#if EIF

#define EIF_MV_PEL_SHIFT                      ( EIF_MV_ADDITIONAL_PRECISION + 2 )

#define EIF_ONE                               ( 1 << EIF_MV_PEL_SHIFT )
#define EIF_FRAC_MASK                         ( EIF_ONE - 1 )
#define EIF_ROUNDING_OFFSET                   ( 1 << (2 * EIF_MV_PEL_SHIFT - 1) )

#if EIF_SIMD
void eif_interpolation_simd_no_clipping(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, pel* p_cur_pos_in_ref, int ref_stride, pel *dst, int dst_stride, pel* p_tmp_buffer, int tmp_buffer_stride)
{
    int c[3];
    int upscaled_data_stride;
    double_pel* buf_line1 = (double_pel*)p_tmp_buffer;
    double_pel* buf_line2 = (double_pel*)p_tmp_buffer + tmp_buffer_stride;
    pel* buf_tmp0 = (pel*)((double_pel*)p_tmp_buffer + tmp_buffer_stride * 2);

    upscaled_data_stride = block_width * 2 + 4;
    upscaled_data_stride = EIF_NUM_PELS_IN_SSE_REG * ((upscaled_data_stride + EIF_NUM_PELS_IN_SSE_REG - 1) / EIF_NUM_PELS_IN_SSE_REG); //align by NUM_PELS_IN_SSE_REG

    d_mv_hor_x += EIF_ONE / 2;
    d_mv_ver_y += EIF_ONE / 2;

    c[2] = tbl_mc_l_coeff_ds[0][2];
    c[1] = tbl_mc_l_coeff_ds[0][1];
    c[0] = tbl_mc_l_coeff_ds[0][0];

    const __m128i msk16 = _mm_set1_epi16(EIF_FRAC_MASK);             //mask for fractional part in fixed point of mv1
    const __m128i msk32 = _mm_set1_epi32(EIF_FRAC_MASK);             //mask for fractional part in fixed point of mv
    const __m128i ofs = _mm_set1_epi32(EIF_ROUNDING_OFFSET);            //1 in fixed point of mv

    const __m128i mvhordx = _mm_set1_epi16(d_mv_hor_x * 4); // increment for +dx and -dx for 4 pixels step
    const __m128i mvhordy = _mm_set1_epi32(d_mv_hor_y * 4);    //horizontal step by Y  //TIM: epi32? why not 16?

    const __m128i mv1D = _mm_set1_epi32(EIF_ONE); // One for (1-dx) for 4 pixels step
    const __m128i mv1W = _mm_set1_epi16(EIF_ONE); // One for (1-dx) for 4 pixels step

    const int vshift = EIF_IF_FILTER_PREC_HP * 2;
    const int voffset = 1 << (vshift - 1);


    const __m128i mmx = _mm_set1_epi32((1 << 10) - 1);       // Max Value for clipping
    const __m128i mmn = _mm_set1_epi32(0);            // Min Value for clipping

    const __m128i fcE = _mm_setr_epi16(
        c[0], 0, 0, 0,                               // ce`(0,1,2,1,0) for even sub-points
        c[2], c[1], c[0], 0);                  // co`(0,1,2,1,0) for odd sub-points

    const __m128i fcO = _mm_setr_epi16(
        c[0], c[1], c[2], c[1],             //for odd sub-points
        0, 0, c[0], c[1]);                        //for even sub-points

    const __m128i fcv = _mm_setr_epi32(
        c[0], c[0], c[2], c[2]);             //c0,c0,c2,c2

    const __m128i fc1 = _mm_set1_epi16(c[1]);     //c[1]
    const __m128i fcee = _mm_mullo_epi16(fc1, fcE); //c0 * ce`(0,1,2,1,0),  c(0,1,2,1,0) is 5-tap
    const __m128i fcoe = _mm_mullo_epi16(fc1, fcO);  //c0 * co`(0,1,2,1,0),  c(0,1,2,1,0) is 5-tap
    const __m128i ofsds = _mm_set1_epi32(voffset);     //0.5 in fixed point for higher resolution

    assert(block_width % 4 == 0);

    for(int h = 0; h < block_height * 2 + 4; ++h)
    {
        //---------------------- prepare two lines from source --------------------------------------

        int mv_x1 = mv_x0;
        int mv_y1 = mv_y0;

        __m128i mvdy0, mvdx0;
        __m128i mvy0 = _mm_set1_epi32(mv_y0 & EIF_FRAC_MASK);
        __m128i mvx0 = _mm_set1_epi32(mv_x0 & EIF_FRAC_MASK);
        __m128i m0123 = _mm_setr_epi32(0, 1, 2, 3);  // hosizontal shift for dx & dy
        __m128i mvdy1 = _mm_set1_epi32(d_mv_hor_y);    // dy
        __m128i mvdx1 = _mm_set1_epi32(d_mv_hor_x);    // dx

        __m128i mt1, mt2, mt3, l0;
        __m128i mp1, mp2, mp3, mma, mcl;

        __m128i t1;
        __m128i fce, fco;

        pel* r0;
        pel* r1;
        pel* cur_pos_in_dst;

        mvdy1 = _mm_mullo_epi32(mvdy1, m0123);
        mvdx1 = _mm_mullo_epi32(mvdx1, m0123);
        mvdy1 = _mm_add_epi32(mvy0, mvdy1);
        mvdx1 = _mm_add_epi32(mvx0, mvdx1);
        mvdy1 = _mm_and_si128(mvdy1, msk32);     // get fractional by mask
        mvdx1 = _mm_and_si128(mvdx1, msk32);     // get fractional by mask
        mvdx1 = _mm_packs_epi32(mvdx1, mvdx1);            //

#define EIF_READ_CHUNK_LOG 2  //2 or 3
#define EIF_READ_CHUNK (1<<EIF_READ_CHUNK_LOG)  //4 or 8
        for (int j = 0; j < (block_width * 2 + 4); ++j)
        {
            buf_line1[j] = *((double_pel*)(p_cur_pos_in_ref + (mv_y1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x1 >> EIF_MV_PEL_SHIFT)));
            buf_line2[j] = *((double_pel*)(p_cur_pos_in_ref + (mv_y1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x1 >> EIF_MV_PEL_SHIFT) + ref_stride));
            mv_x1 += d_mv_hor_x;
            mv_y1 += d_mv_hor_y;
        }
        //---------------------------------------------------------------------------------------

        r0 = (pel*)buf_line1;
        r1 = (pel*)buf_line2;
        cur_pos_in_dst = dst;

        mp3 = _mm_set1_epi32(0);
        mcl = _mm_set1_epi32(0);

        if ((h & 1) == 0)
        {
            fce = fcE; //1 * ce`(0,1,2,1,0),  c(0,1,2,1,0) is 5-tap
            fco = fcO; //1 * co`(0,1,2,1,0),  c(0,1,2,1,0) is 5-tap
        }
        else
        {
            fce = fcee; //c0 * ce`(0,1,2,1,0),  c(0,1,2,1,0) is 5-tap
            fco = fcoe; //c0 * co`(0,1,2,1,0),  c(0,1,2,1,0) is 5-tap
        }

        //horizontal loop
        buf_tmp0 = (pel*)( (double_pel*)p_tmp_buffer + tmp_buffer_stride * 2 );

        for (int w = 0; w < block_width * 2 + 4; w += 4 )
        {
            mvdx1 = _mm_and_si128(mvdx1, msk16);     // get fractional by mask of dx
            mvdy1 = _mm_and_si128(mvdy1, msk32);     // get fractional by mask
            mvdy0 = _mm_sub_epi32(mv1D, mvdy1);      //(1-dy)
            mvdx0 = _mm_sub_epi16(mv1W, mvdx1);      //(1-dx)
            mvdx0 = _mm_unpacklo_epi16(mvdx0, mvdx1);//(1-dx),dx ... 8W

            // r - reference, b - bilinear, d - high-pass horizontally, filter - (c0,c1,c2,c1,c0)
            //loading rxy with int precision
            mt1 = _mm_load_si128((__m128i*)(r0));       //r00, r10 for r(0..4) = 4 pels
            mt2 = _mm_load_si128((__m128i*)(r1));       //r01, r11 for r(0..4) = 4 pels
            //bilinear interpolation
            mt1 = _mm_madd_epi16(mt1, mvdx0);          //(1-dx)*r00+dx*r10 .. for r(0..4)
            mt2 = _mm_madd_epi16(mt2, mvdx0);          //(1-dx)*r01+dx*r11 .. for r(0..4)
            mt1 = _mm_mullo_epi32(mt1, mvdy0);         //(1-dy) * ((1-dx)*r00+dx*r10)  .. for r(0..4)
            mt2 = _mm_mullo_epi32(mt2, mvdy1);         //(dy)   * ((1-dx)*r01+dx*r11)  .. for r(0..4)
            mt1 = _mm_add_epi32(mt1, mt2);              //(1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11) .. for r(0..4)
            mt1 = _mm_add_epi32(mt1, ofs);             //(1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11) + ofs  .. for r(0..4)
            l0 = _mm_srli_epi32(mt1, EIF_MV_PEL_SHIFT * 2);      //b = ((1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11)  + ofs) >> shift.. for r(0..4)

            //5-tap high-pass
            //horizontal high-pass filtering
            mma = _mm_load_si128((__m128i*)(buf_tmp0)); //vertical accumulator (z-2,z-2,z,z)
            mt1 = _mm_packus_epi32(l0, l0);           //dublicate b1 -> [b1,b1]  where b0=(b0,b1,b2,b3) and b1=(b4,b5,b6,b7)
            mp1 = _mm_madd_epi16(mt1, fce);            //get cx*(c2,c1,c0,0))*b1 | cx*(c0,0,0,0)*b1 -> 4 support pixels
            t1 = _mm_add_epi32(mp3, mp1);              // get cx*(c0c1,c2c1)*b0 + cx*(c0,0)*b1  | cx*(0,c0c1)*b0 + cx*(c2c1,c0)*b1   -> 4 support pixels
            mt2 = _mm_hadd_epi32(t1, t1);             // d0 = cx*(c0c1c2c1)*b0 + cx*(c0)*b1 | d1 = cx*(c0c1)*b0 + cx*(c2c1c0)*b1
            mp3 = _mm_madd_epi16(mt1, fco);            // get cx*(c0c1 c2c1)*b1 | cx*(0 c0c1)*b1

            if ( h & 1 )// even lines just *c1 during horizontal high-pass filtering
            {
                mma = _mm_add_epi32(mma, mt2);             // + c1*(d0,d1,d0,d1)
            }
            else
            {
                mp2 = _mm_mullo_epi32(mt2, fcv);         // c0*(l0,l1), c2*(l0,l1)
                mma = _mm_add_epi32(mma, mp2);             // + c0*(l0,l1) = dst, + c2*(l0,l1)
                mt3 = _mm_srai_epi32(mma, vshift);      // dst >> vshift
                mt2 = _mm_add_epi32(mp2, ofsds);         // c0*(l0,l1) + offset
                //clipping
                mcl = _mm_alignr_epi8(mt3, mcl, 8);      // collecting values for clipping
                if (h > 2 && w > 0 && ((w & 4) == 0))
                {
                    mp2 = _mm_max_epi32(mcl, mmn);
                    mcl = _mm_min_epi32(mp2, mmx);
                    mp1 = _mm_packus_epi32(mcl, mcl);       // 32->16 with clipping
                    _mm_storel_epi64( (__m128i*) cur_pos_in_dst, mp1);    // store dst(0,1)
                    cur_pos_in_dst += 4;
                }
                mma = _mm_alignr_epi8(mt2, mma, 8);      // shift dst out from accumulator and add new one
            }
            _mm_store_si128( (__m128i*) buf_tmp0, mma);             // store vertical accumulators

            //fractional increment
            mvdx1 = _mm_add_epi16(mvdx1, mvhordx);   //incrementing dx
            mvdy1 = _mm_add_epi32(mvdy1, mvhordy);   //incrementing dy

            r0 += 8;
            r1 += 8;
            buf_tmp0 += 8;  //t_inc=0 initially to skip first writing to memory - just initialization
        }
        mv_x0 += d_mv_ver_x;
        mv_y0 += d_mv_ver_y;
        if ((h > 2) && (h & 1) == 0)
        {
            dst += block_width;
        }
    }
}

#if EIF_3TAP
//3-tap 1-pass filter, high pass currently working in 16 bit precision only
void eif_3tap_interpolation_simd_no_clipping(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, pel* p_cur_pos_in_ref, int ref_stride, pel *dst, int dst_stride, pel* p_tmp_buffer, int tmp_buffer_stride)
{
    int upscaled_data_stride;
    __m128i msk16;
    __m128i msk32;
    __m128i ofs;
    __m128i mvhordx;
    __m128i mvhordy;
    __m128i mv1D;
    __m128i mv1W;

    int voffset;
    int vshift;
    int hshift;
    int hoffset;
    __m128i hofs;
    pel max_val;
    int bit_depth;
    __m128i mmx;
    __m128i mmn;
    __m128i ofsds;
    __m128i ma1, ma2;

    int h;

    double_pel* buf_line1 = (double_pel*)(p_tmp_buffer + 4);
    double_pel* buf_line2 = (double_pel*)(p_tmp_buffer + tmp_buffer_stride + 4);
    pel* buf_tmp0 = (pel*)((double_pel*)p_tmp_buffer + tmp_buffer_stride * 2);
    upscaled_data_stride = block_width * 2 + 4;
    upscaled_data_stride = EIF_NUM_PELS_IN_SSE_REG * ((upscaled_data_stride + EIF_NUM_PELS_IN_SSE_REG - 1) / EIF_NUM_PELS_IN_SSE_REG); //align by NUM_PELS_IN_SSE_REG

    d_mv_hor_x = 2 * d_mv_hor_x + EIF_ONE;
    d_mv_hor_y = 2 * d_mv_hor_y;
    d_mv_ver_x = 2 * d_mv_ver_x;
    d_mv_ver_y = 2 * d_mv_ver_y + EIF_ONE;

    msk16 = _mm_set1_epi16(EIF_FRAC_MASK);             //mask for fractional part in fixed point of mv1
    msk32 = _mm_set1_epi32(EIF_FRAC_MASK);             //mask for fractional part in fixed point of mv
    ofs = _mm_set1_epi32(EIF_ROUNDING_OFFSET);            //1 in fixed point of mv

    mvhordx = _mm_set1_epi16((d_mv_hor_x * 4) & EIF_FRAC_MASK); // increment for +dx and -dx for 4 pixels step
    mvhordy = _mm_set1_epi32((d_mv_hor_y * 4) & EIF_FRAC_MASK);    //horizontal step by Y  //TIM: epi32? why not 16?

    mv1D = _mm_set1_epi32(EIF_ONE); // One for (1-dx) for 4 pixels step
    mv1W = _mm_set1_epi16(EIF_ONE); // One for (1-dx) for 4 pixels step

    bit_depth = 10; //take from outside

    hshift = 1;
    hoffset = 1 << (hshift - 1);
    hofs = _mm_set1_epi16(hoffset);       // Max Value for clipping
    vshift = (3 * 2) - hshift - 1;// EIF_IF_FILTER_PREC_HP * 2 - hshift - 1(keeping 16 bit vertically);
    voffset = 1 << (vshift - 1);
    max_val = (1 << bit_depth) - 1;


    mmx = _mm_set1_epi16(max_val);       // Max Value for clipping
    mmn = _mm_set1_epi16(0);            // Min Value for clipping

    ma1 = _mm_setzero_si128();
    ma2 = ma1;

    ofsds = _mm_set1_epi16(voffset);     //0.5 if fixed point for higher resolution

    assert(block_width % 8 == 0);

    for (h = 0; h < block_height + 2; ++h)
    {
        //---------------------- prepare two lines from source --------------------------------------

        int mv_x1 = mv_x0;
        int mv_x2 = mv_x0 + 1 * d_mv_hor_x;
        int mv_x3 = mv_x0 + (2 + 2) * d_mv_hor_x;
        int mv_x4 = mv_x0 + (3 + 2) * d_mv_hor_x;

        int mv_y1 = mv_y0;
        int mv_y2 = mv_y0 + 1 * d_mv_hor_y;
        int mv_y3 = mv_y0 + (2 + 2) * d_mv_hor_y;
        int mv_y4 = mv_y0 + (3 + 2) * d_mv_hor_y;
        int j = 0;

        __m128i mvdy0, mvdx0;
        __m128i mvy0 = _mm_set1_epi32((mv_y0 + 2 * d_mv_hor_y) & EIF_FRAC_MASK);
        __m128i mvx0 = _mm_set1_epi32((mv_x0 + 2 * d_mv_hor_x) & EIF_FRAC_MASK);
        __m128i m0123 = _mm_setr_epi32(0, 1, 2, 3);  // hosizontal shift for dx & dy
        __m128i mvdy1 = _mm_set1_epi32(d_mv_hor_y);    // dy
        __m128i mvdx1 = _mm_set1_epi32(d_mv_hor_x);    // dx
        __m128i mt1, mt2, mt3, l0, l1;
        __m128i mp1, mp2, mp3, mcl;

        __m128i t1;
        __m128i mc0, t0, mc1, b0, b1, b2;
        int w = 0;

        pel* r0;
        pel* r1;
        pel* cur_pos_in_dst;

        mvdy1 = _mm_mullo_epi32(mvdy1, m0123);
        mvdx1 = _mm_mullo_epi32(mvdx1, m0123);
        mvdy1 = _mm_add_epi32(mvy0, mvdy1);
        mvdx1 = _mm_add_epi32(mvx0, mvdx1);
        mvdy1 = _mm_and_si128(mvdy1, msk32);     // get fractional by mask
        mvdx1 = _mm_and_si128(mvdx1, msk32);     // get fractional by mask
        mvdx1 = _mm_packs_epi32(mvdx1, mvdx1);            //

        pel *ref1 = p_cur_pos_in_ref + (mv_y1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x1 >> EIF_MV_PEL_SHIFT);
        pel *ref2 = p_cur_pos_in_ref + (mv_y2 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x2 >> EIF_MV_PEL_SHIFT);
        buf_line1[4 * j + 0] = *((double_pel*)ref1);
        buf_line1[4 * j + 1] = *((double_pel*)ref2);
        buf_line2[4 * j + 0] = *((double_pel*)(ref1 + ref_stride));
        buf_line2[4 * j + 1] = *((double_pel*)(ref2 + ref_stride));

        mv_x1 += 2 * d_mv_hor_x;
        mv_x2 += 2 * d_mv_hor_x;
        mv_y1 += 2 * d_mv_hor_y;
        mv_y2 += 2 * d_mv_hor_y;
        for (; j < (block_width) / 4; ++j)
        {
            pel *ref1 = p_cur_pos_in_ref + (mv_y1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x1 >> EIF_MV_PEL_SHIFT);
            pel *ref2 = p_cur_pos_in_ref + (mv_y2 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x2 >> EIF_MV_PEL_SHIFT);
            pel *ref3 = p_cur_pos_in_ref + (mv_y3 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x3 >> EIF_MV_PEL_SHIFT);
            pel *ref4 = p_cur_pos_in_ref + (mv_y4 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x4 >> EIF_MV_PEL_SHIFT);

            buf_line1[4 * j + 0 + 2] = *((double_pel*)ref1);
            buf_line1[4 * j + 1 + 2] = *((double_pel*)ref2);
            buf_line1[4 * j + 2 + 2] = *((double_pel*)ref3);
            buf_line1[4 * j + 3 + 2] = *((double_pel*)ref4);

            buf_line2[4 * j + 0 + 2] = *((double_pel*)(ref1 + ref_stride));
            buf_line2[4 * j + 1 + 2] = *((double_pel*)(ref2 + ref_stride));
            buf_line2[4 * j + 2 + 2] = *((double_pel*)(ref3 + ref_stride));
            buf_line2[4 * j + 3 + 2] = *((double_pel*)(ref4 + ref_stride));

            mv_x1 += 4 * d_mv_hor_x;
            mv_x2 += 4 * d_mv_hor_x;
            mv_x3 += 4 * d_mv_hor_x;
            mv_x4 += 4 * d_mv_hor_x;

            mv_y1 += 4 * d_mv_hor_y;
            mv_y2 += 4 * d_mv_hor_y;
            mv_y3 += 4 * d_mv_hor_y;
            mv_y4 += 4 * d_mv_hor_y;
        }
        //---------------------------------------------------------------------------------------

        r0 = (pel*)buf_line1;
        r1 = (pel*)buf_line2;
        cur_pos_in_dst = dst;

        mp3 = _mm_set1_epi32(0);
        mcl = _mm_set1_epi32(0);

        //horizontal loop
        buf_tmp0 = (pel*)((double_pel*)p_tmp_buffer + tmp_buffer_stride * 2);
        //predinitialization of first 2 support pixels
        //...
        int frac_ref_pos_X = mv_x0 & EIF_FRAC_MASK;
        int frac_ref_pos_Y = mv_y0 & EIF_FRAC_MASK;
        pel bm2 = ((u32)r0[0] * (EIF_ONE - frac_ref_pos_X) * (EIF_ONE - frac_ref_pos_Y) +
                   (u32)r0[1] * frac_ref_pos_X * (EIF_ONE - frac_ref_pos_Y) +
                   (u32)r1[0] * (EIF_ONE - frac_ref_pos_X) * frac_ref_pos_Y +
                   (u32)r1[1] * frac_ref_pos_X * frac_ref_pos_Y + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);
        frac_ref_pos_X = (mv_x0 + d_mv_hor_x) & EIF_FRAC_MASK;
        frac_ref_pos_Y = (mv_y0 + d_mv_hor_y) & EIF_FRAC_MASK;
        pel bm1 = ((u32)r0[2] * (EIF_ONE - frac_ref_pos_X) * (EIF_ONE - frac_ref_pos_Y) +
                   (u32)r0[3] * frac_ref_pos_X * (EIF_ONE - frac_ref_pos_Y) +
                   (u32)r1[2] * (EIF_ONE - frac_ref_pos_X) * frac_ref_pos_Y +
                   (u32)r1[3] * frac_ref_pos_X * frac_ref_pos_Y + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);

        b0 = _mm_setr_epi16(0, 0, 0, 0, 0, 0, bm2, bm1);
        r0 += 4;
        r1 += 4;
        for (; w < block_width; w += 8)
        {
            mvdx1 = _mm_and_si128(mvdx1, msk16);     // get fractional by mask of dx
            mvdy1 = _mm_and_si128(mvdy1, msk32);     // get fractional by mask
            mvdy0 = _mm_sub_epi32(mv1D, mvdy1);      //(1-dy)
            mvdx0 = _mm_sub_epi16(mv1W, mvdx1);      //(1-dx)
            mvdx0 = _mm_unpacklo_epi16(mvdx0, mvdx1);//(1-dx),dx ... 8W

            // r - reference, b - bilinear, d - high-pass filtering horizontally, filter - (c0,c1,c2,c1,c0)
            //loading rxy with int precision
            mt1 = _mm_load_si128((__m128i*)(r0));       //r00, r10 for r(0..4) = 4 pels
            mt2 = _mm_load_si128((__m128i*)(r1));       //r01, r11 for r(0..4) = 4 pels
            //bilinear interpolation
            mt1 = _mm_madd_epi16(mt1, mvdx0);          //(1-dx)*r00+dx*r10 .. for r(0..4)
            mt2 = _mm_madd_epi16(mt2, mvdx0);          //(1-dx)*r01+dx*r11 .. for r(0..4)
            mt1 = _mm_mullo_epi32(mt1, mvdy0);         //(1-dy) * ((1-dx)*r00+dx*r10)  .. for r(0..4)
            mt2 = _mm_mullo_epi32(mt2, mvdy1);         //(dy)   * ((1-dx)*r01+dx*r11)  .. for r(0..4)
            mt1 = _mm_add_epi32(mt1, mt2);              //(1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11) .. for r(0..4)
            mt1 = _mm_add_epi32(mt1, ofs);             //(1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11) + ofs  .. for r(0..4)
            l0 = _mm_srli_epi32(mt1, EIF_MV_PEL_SHIFT * 2);      //b = ((1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11)  + ofs) >> shift.. for r(0..4)

            mvdx1 = _mm_add_epi16(mvdx1, mvhordx);   //incrementing dx
            mvdy1 = _mm_add_epi32(mvdy1, mvhordy);   //incrementing dy
            mvdx1 = _mm_and_si128(mvdx1, msk16);     // get fractional by mask of dx
            mvdy1 = _mm_and_si128(mvdy1, msk32);     // get fractional by mask
            mvdy0 = _mm_sub_epi32(mv1D, mvdy1);      //(1-dy)
            mvdx0 = _mm_sub_epi16(mv1W, mvdx1);      //(1-dx)
            mvdx0 = _mm_unpacklo_epi16(mvdx0, mvdx1);//(1-dx),dx ... 8W
            r0 += 8;
            r1 += 8;

            // r - reference, b - bilinear, d - high-pass filtering horizontally, filter - (c0,c1,c2,c1,c0)
            //loading rxy with int precision
            mt1 = _mm_load_si128((__m128i*)(r0));       //r00, r10 for r(0..4) = 4 pels
            mt2 = _mm_load_si128((__m128i*)(r1));       //r01, r11 for r(0..4) = 4 pels
            //bilinear interpolation
            mt1 = _mm_madd_epi16(mt1, mvdx0);          //(1-dx)*r00+dx*r10 .. for r(0..4)
            mt2 = _mm_madd_epi16(mt2, mvdx0);          //(1-dx)*r01+dx*r11 .. for r(0..4)
            mt1 = _mm_mullo_epi32(mt1, mvdy0);         //(1-dy) * ((1-dx)*r00+dx*r10)  .. for r(0..4)
            mt2 = _mm_mullo_epi32(mt2, mvdy1);         //(dy)   * ((1-dx)*r01+dx*r11)  .. for r(0..4)
            mt1 = _mm_add_epi32(mt1, mt2);              //(1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11) .. for r(0..4)
            mt1 = _mm_add_epi32(mt1, ofs);             //(1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11) + ofs  .. for r(0..4)
            l1 = _mm_srli_epi32(mt1, EIF_MV_PEL_SHIFT * 2);      //b = ((1 - dy) * ((1 - dx)*r00 + dx*r10) + (dy)   * ((1-dx)*r01+dx*r11)  + ofs) >> shift.. for r(0..4)
            //3-tap high-pass
            //horizontal filtering
            ma1 = _mm_load_si128((__m128i*)(buf_tmp0)); //vertical accumulator (z-2)
            ma2 = _mm_load_si128((__m128i*)(buf_tmp0 + 8)); //vertical accumulator (z-1)
            mt3 = _mm_packus_epi32(l0, l1);           //[b0,b1]  where b0=(b0,b1,b2,b3) and b1=(b4,b5,b6,b7)
            b1 = _mm_alignr_epi8(mt3, b0, 14);         // b(-1)
            b2 = _mm_alignr_epi8(mt3, b0, 12);        // b(-2)
            b0 = mt3;
            mc0 = _mm_add_epi32(b2, b0);              // b(-2)*|-1| + b(0)*|-1|
            t1 = _mm_slli_epi16(b1, 3);               // b(-1)*8
            t0 = _mm_add_epi16(b1, b1);              //b(-1)*2
            mc1 = _mm_add_epi16(t1, t0);              //b(-1)*10
            mp2 = _mm_sub_epi16(mc1, mc0);              //b(-2)*(-1) + b(-1)*10 + b(0)*(-1)

            //reduce precision to stay in 16 bit for high-pass
            mt2 = _mm_add_epi16(mp2, hofs);
            mp2 = _mm_srai_epi16(mt2, hshift);          // bh(0)
            //vertical high-pass filtering  for 16 bit arithmetic with losing 1 bit precision
            t1 = _mm_slli_epi16(ma2, 2);               // bh(-1)*4
            t0 = _mm_add_epi16(t1, ma2);              // bh(-1)*5 = (c1*bh(-1) + 1) >> 1 when c1==10
            mc1 = _mm_add_epi16(ma1, mp2);              // (|c0|*bh(-2) + |c0|*bh(0))
#if EIF_3TAP_HALFFIX
            mp3 = _mm_srai_epi16(mc1, hshift);          // (|c0|*bh(-2) + |c0|*bh(0)) >> 1 when |c0|==1     Could be done with mm_avg by substracting lowest possible negative value from hofs and adding it to ofsds
#else
            mc0 = _mm_add_epi16(mc1, hofs);             // (|c0|*bh(-2) + |c0|*bh(0) + 1)
            mp3 = _mm_srai_epi16(mc0, hshift);          // (|c0|*bh(-2) + |c0|*bh(0)) >> 1 when |c0|==1     Could be done with mm_avg by substracting lowest possible negative value from hofs and adding it to ofsds
#endif
            t1 = _mm_sub_epi16(t0, mp3);              // (c1*bh(-1) - (|c0|*bh(-2) + |c0|*bh(0) + 1) >> 1)
            mp3 = _mm_add_epi16(t1, ofsds);           // (c1*bh(-1) - (|c0|*bh(-2) + |c0|*bh(0) + 1) >> 1) + vofs
            mt3 = _mm_srai_epi16(mp3, vshift);          // dst >> vshift

            //clipping
            mp1 = _mm_max_epi16(mt3, mmn);
            mcl = _mm_min_epi16(mp1, mmx);
            _mm_storeu_si128((__m128i*)cur_pos_in_dst, mcl);    // store dst(0..7)            ORIGINAL

            cur_pos_in_dst += 8;

            _mm_store_si128((__m128i*)buf_tmp0, ma2);               //  bh(-2) = bh(-1) store vertical accumulators
            _mm_store_si128((__m128i*)(buf_tmp0 + 8), mp2);           // bh(-2) = bh(-1)store vertical accumulators

            //fractional increment
            mvdx1 = _mm_add_epi16(mvdx1, mvhordx);     //incrementing dx
            mvdy1 = _mm_add_epi32(mvdy1, mvhordy);     //incrementing dy

            r0 += 8;
            r1 += 8;

            buf_tmp0 += 16;
        }

        mv_x0 += d_mv_ver_x;
        mv_y0 += d_mv_ver_y;
        if (h > 1)
            dst += block_width;
    }
}
#endif
#endif  //EIF_SIMD

BOOL can_mv_clipping_occurs(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, int mv_x_max, int mv_y_max, int mv_x_min, int mv_y_min)
{
    int mv_x_corners[2][2];
    int mv_y_corners[2][2];
    BOOL mv_x_clip_occurs = FALSE;
    BOOL mv_y_clip_occurs = FALSE;
    int i, j;

    block_width = block_width * 2 + 4;
    block_height = block_height * 2 + 4;

    mv_x_corners[0][0] = mv_x0;
    mv_x_corners[0][1] = mv_x0 + block_width * d_mv_hor_x;
    mv_x_corners[1][0] = mv_x0 + block_height * d_mv_ver_x;
    mv_x_corners[1][1] = mv_x0 + block_width * d_mv_hor_x + block_height * d_mv_ver_x;

    for(i = 0; i < 2; ++i)
        for(j = 0; j < 2; ++j)
        {
            if(mv_x_corners[i][j] > mv_x_max || mv_x_corners[i][j] < mv_x_min)  //TODO(@Tim) handle eq case 
                mv_x_clip_occurs = TRUE;
        }

    //TODO: copy-paste
    mv_y_corners[0][0] = mv_y0;
    mv_y_corners[0][1] = mv_y0 + block_width * d_mv_hor_y;
    mv_y_corners[1][0] = mv_y0 + block_height * d_mv_ver_y;
    mv_y_corners[1][1] = mv_y0 + block_width * d_mv_hor_y + block_height * d_mv_ver_y;

    for(i = 0; i < 2; ++i)
        for(j = 0; j < 2; ++j)
        {
            if(mv_y_corners[i][j] > mv_y_max || mv_y_corners[i][j] < mv_y_min)  //TODO(@Tim) handle eq case
                mv_y_clip_occurs = TRUE;
        }

    return mv_x_clip_occurs || mv_y_clip_occurs;
}

void evc_affine_mc_eif(int x, int y, int pic_w, int pic_h, int block_width, int block_height, s16 initial_mv_array[VER_NUM][MV_D], EVC_PIC *ref_pic, pel pred[MAX_CU_DIM], int vertex_num, pel *p_tmp_buffer, s8 comp)
{  
    // get clip MV Range
    int bc = comp > Y_C;
    int mv_x_max = ((pic_w + MAX_CU_SIZE - x - block_width)  << EIF_MV_PEL_SHIFT) >> bc;
    int mv_y_max = ((pic_h + MAX_CU_SIZE - y - block_height) << EIF_MV_PEL_SHIFT) >> bc;
    int mv_x_min = ((-MAX_CU_SIZE - x) << EIF_MV_PEL_SHIFT) >> bc;
    int mv_y_min = ((-MAX_CU_SIZE - y) << EIF_MV_PEL_SHIFT) >> bc;
    int d_mv_hor_x = 0, d_mv_hor_y = 0, d_mv_ver_x = 0, d_mv_ver_y = 0;
    pel* p_ref = NULL;
    int ref_stride = 0;
    int prep_data_for_bi_stride = 2 * (2 * block_width + 4);

    assert(EIF_MV_ADDITIONAL_PRECISION > evc_tbl_log2[block_width] + 1);

    int mv_x0 = initial_mv_array[0][MV_X] << (EIF_MV_ADDITIONAL_PRECISION - bc);
    int mv_y0 = initial_mv_array[0][MV_Y] << (EIF_MV_ADDITIONAL_PRECISION - bc);

    block_width >>= bc;
    block_height >>= bc;

    if(vertex_num == 3)
    {
        d_mv_hor_x = ((initial_mv_array[1][MV_X] - initial_mv_array[0][MV_X]) << (EIF_MV_ADDITIONAL_PRECISION - bc)) >> (evc_tbl_log2[block_width] + 1);
        d_mv_hor_y = ((initial_mv_array[1][MV_Y] - initial_mv_array[0][MV_Y]) << (EIF_MV_ADDITIONAL_PRECISION - bc)) >> (evc_tbl_log2[block_width] + 1);

        d_mv_ver_x = ((initial_mv_array[2][MV_X] - initial_mv_array[0][MV_X]) << (EIF_MV_ADDITIONAL_PRECISION - bc)) >> (evc_tbl_log2[block_height] + 1);
        d_mv_ver_y = ((initial_mv_array[2][MV_Y] - initial_mv_array[0][MV_Y]) << (EIF_MV_ADDITIONAL_PRECISION - bc)) >> (evc_tbl_log2[block_height] + 1);
    }
    else /*if (vertex_num == 2)*/
    {
        d_mv_hor_x = ((initial_mv_array[1][MV_X] - initial_mv_array[0][MV_X]) << (EIF_MV_ADDITIONAL_PRECISION - bc)) >> (evc_tbl_log2[block_width] + 1);
        d_mv_hor_y = ((initial_mv_array[1][MV_Y] - initial_mv_array[0][MV_Y]) << (EIF_MV_ADDITIONAL_PRECISION - bc)) >> (evc_tbl_log2[block_width] + 1);
        d_mv_ver_x = -d_mv_hor_y;
        d_mv_ver_y = d_mv_hor_x;
    }
    /*else
    assert(!"Invalid vertex num");*/

    x >>= bc;
    y >>= bc;

    if(comp == Y_C)
    {
        p_ref = ref_pic->y;
        ref_stride = ref_pic->s_l;
    }
    else {
        ref_stride = ref_pic->s_c;
        if (comp == U_C)
            p_ref = ref_pic->u;
        else if(comp == V_C)
            p_ref = ref_pic->v;
    }

    pel* p_cur_pos_in_ref = p_ref + y * ref_stride + x - ref_stride - 1;

    int upscaled_data_stride = block_width * 2 + 4;
    upscaled_data_stride = EIF_NUM_PELS_IN_SSE_REG * ((upscaled_data_stride + EIF_NUM_PELS_IN_SSE_REG - 1) / EIF_NUM_PELS_IN_SSE_REG); //align by NUM_PELS_IN_SSE_REG

    mv_x0 -= 2 * d_mv_hor_x + 2 * d_mv_ver_x;
    mv_y0 -= 2 * d_mv_hor_y + 2 * d_mv_ver_y;
#if !EIF_SIMD
    assert((upscaled_data_stride & (EIF_NUM_PELS_IN_SSE_REG - 1)) == 0);  //TODO(@Tim): move under SIMD condition
#endif
    assert((prep_data_for_bi_stride & (EIF_NUM_PELS_IN_SSE_REG - 1)) == 0);
#if EIF_SIMD
    assert(is_ptr_aligned(p_tmp_buffer, EIF_NUM_PELS_IN_SSE_REG * sizeof(pel)));
#endif

    if (can_mv_clipping_occurs(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, mv_x_max, mv_y_max, mv_x_min, mv_y_min))
    {
#if EIF_3TAP
        eif_1pass_3tap_clipping(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride, mv_x_max, mv_y_max, mv_x_min, mv_y_min);
#else
        eif_1pass_clipping(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride, mv_x_max, mv_y_max, mv_x_min, mv_y_min);
#endif
    }
    else
    {
#if EIF_SIMD
#if EIF_3TAP
        if (block_width >= 8)
        {
            eif_3tap_interpolation_simd_no_clipping(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride);
        }
        else
        {
            eif_1pass_3tap(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride);
        }
#else
        eif_interpolation_simd_no_clipping(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride);
#endif
#elif EIF_3TAP
        eif_1pass_3tap(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride);
#else
        eif_1pass(block_width, block_height, mv_x0, mv_y0, d_mv_hor_x, d_mv_hor_y, d_mv_ver_x, d_mv_ver_y, p_cur_pos_in_ref, ref_stride, pred, upscaled_data_stride, p_tmp_buffer + upscaled_data_stride * (2 * block_height + 4), prep_data_for_bi_stride);
#endif
    }
}

#if EIF_3TAP
void eif_1pass_3tap(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, pel* p_cur_pos_in_ref, int ref_stride, pel *dst, int dst_stride, pel* p_tmp_buffer, int tmp_buffer_stride)
{
    double_pel buf_tmp0_base[2 * (MAX_CU_SIZE + 2)];
    double_pel buf_tmp1_base[2 * (MAX_CU_SIZE + 2)];
    int c[3];


    pel min_val = 0;
    pel max_val = (1 << 10) - 1;


    d_mv_hor_x = 2 * d_mv_hor_x + EIF_ONE;
    d_mv_hor_y = 2 * d_mv_hor_y;
    d_mv_ver_x = 2 * d_mv_ver_x;
    d_mv_ver_y = 2 * d_mv_ver_y + EIF_ONE;

    c[2] = tbl_mc_l_coeff_ds[0][2];
    c[0] = tbl_mc_l_coeff_ds[0][0];

    int hshift = 1;
    int hoffset = 1 << (hshift - 1);
    int vshift = (3 * 2) - hshift;
    c[2] = c[2] >> 3;
    c[0] = c[0] >> 3;

    const int voffset = 1 << (vshift - 1);

    for (int h = 0; h < block_height + 2; ++h)
    {
        int mvdy1 = (mv_y0)& EIF_FRAC_MASK;
        int mvdx1 = (mv_x0)& EIF_FRAC_MASK;

        int prevl0 = 0;
        int prev2l0 = 0;
        int valhor0;
        int valver0;
        int dstval;

        double_pel* buf_tmp0 = buf_tmp0_base;
        double_pel* buf_tmp1 = buf_tmp1_base;
        pel* cur_pos_in_dst = dst;
        pel* r = (p_cur_pos_in_ref + (mv_y0 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x0 >> EIF_MV_PEL_SHIFT));

        //horizontal loop
        for (int w = 0; w < block_width + 2; ++w)
        {
            //bilinear interpolation
            r += (mvdy1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mvdx1 >> EIF_MV_PEL_SHIFT);
            mvdy1 = mvdy1 & EIF_FRAC_MASK;
            mvdx1 = mvdx1 & EIF_FRAC_MASK;

            pel l0 = ((u32)r[0] * (EIF_ONE - mvdx1) * (EIF_ONE - mvdy1) +
                      (u32)r[1] * mvdx1 * (EIF_ONE - mvdy1) +
                      (u32)r[0 + ref_stride] * (EIF_ONE - mvdx1) * mvdy1 +
                      (u32)r[1 + ref_stride] * mvdx1 * mvdy1 + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);

            //            s32 a = ((s32)r[0] << EIF_MV_PEL_SHIFT) + ((s32)r[0 + ref_stride] - (s32)r[0]) * mvdy1;
            //            s32 b = ((s32)r[1] << EIF_MV_PEL_SHIFT) + ((s32)r[1 + ref_stride] - (s32)r[1]) * mvdy1;
            //            pel l0 = (u32)(((a << EIF_MV_PEL_SHIFT) + (b - a) * mvdx1 + EIF_ROUNDING_OFFSET) >> (2 * EIF_MV_PEL_SHIFT));

            //high-pass filtering
            valhor0 = (c[0] * prev2l0 + c[2] * prevl0 + c[0] * l0 + hoffset) >> hshift;

#if EIF_3TAP_HALFFIX
            valver0 = c[0] * ((valhor0 + buf_tmp0[0] + 0) >> 1) + (c[2] >> 1) * buf_tmp1[0];
#else
            valver0 = c[0] * ((valhor0 + buf_tmp0[0] + 1) >> 1) + (c[2] >> 1) * buf_tmp1[0];
#endif
            buf_tmp0[0] = buf_tmp1[0];
            buf_tmp1[0] = valhor0;
            dstval = (valver0 + (voffset>>1)) >> (vshift-1);

            if (h > 1 && w > 1)
            {
                dstval = min(dstval, max_val);
                dstval = max(dstval, min_val);
                cur_pos_in_dst[0] = dstval;
                cur_pos_in_dst += 1;
            }
            mvdx1 = mvdx1 + d_mv_hor_x;   //incrementing dx
            mvdy1 = mvdy1 + d_mv_hor_y;   //incrementing dy

            buf_tmp0 += 2;
            buf_tmp1 += 2;

            prev2l0 = prevl0;
            prevl0 = l0;
        }
        mv_x0 += d_mv_ver_x;
        mv_y0 += d_mv_ver_y;
        if (h > 1)
        {
            dst += block_width;
        }
    }
}

void eif_1pass_3tap_clipping(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, pel* p_cur_pos_in_ref, int ref_stride, pel *dst, int dst_stride, pel* p_tmp_buffer, int tmp_buffer_stride, int mv_x_max, int mv_y_max, int mv_x_min, int mv_y_min)
{
    double_pel buf_tmp0_base[2 * (MAX_CU_SIZE + 2)];
    double_pel buf_tmp1_base[2 * (MAX_CU_SIZE + 2)];
    int c[3];
    pel min_val = 0;
    pel max_val = (1 << 10) - 1;

    d_mv_hor_x = 2 * d_mv_hor_x + EIF_ONE;
    d_mv_hor_y = 2 * d_mv_hor_y;
    d_mv_ver_x = 2 * d_mv_ver_x;
    d_mv_ver_y = 2 * d_mv_ver_y + EIF_ONE;

    c[2] = tbl_mc_l_coeff_ds[0][2];
    c[0] = tbl_mc_l_coeff_ds[0][0];

    int hshift = 1;
    int hoffset = 1 << (hshift - 1);
    int vshift = (3 * 2) - hshift;
    c[2] = c[2] >> 3;
    c[0] = c[0] >> 3;

    const int voffset = 1 << (vshift - 1);

    for (int h = 0; h < block_height + 2; ++h)
    {
        int mv_y = mv_y0;
        int mv_x = mv_x0;
        int prevl0 = 0;
        int prev2l0 = 0;
        int valhor0;
        int valver0;
        int dstval;

        double_pel* buf_tmp0 = buf_tmp0_base;
        double_pel* buf_tmp1 = buf_tmp1_base;
        pel* cur_pos_in_dst = dst;
        pel* r = (p_cur_pos_in_ref + (mv_y0 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x0 >> EIF_MV_PEL_SHIFT));

        //horizontal loop
        for (int w = 0; w < block_width + 2; ++w)
        {
            int mv_xc = min(mv_x_max, max(mv_x_min, mv_x));
            int mv_yc = min(mv_y_max, max(mv_y_min, mv_y));
            int mv_xcf = mv_xc & EIF_FRAC_MASK;
            int mv_ycf = mv_yc & EIF_FRAC_MASK;
            //bilinear interpolation
            r = (p_cur_pos_in_ref + (mv_yc >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_xc >> EIF_MV_PEL_SHIFT));

            pel l0 = ((u32)r[0] * (EIF_ONE - mv_xcf) * (EIF_ONE - mv_ycf) +
                      (u32)r[1] * mv_xcf * (EIF_ONE - mv_ycf) +
                      (u32)r[0 + ref_stride] * (EIF_ONE - mv_xcf) * mv_ycf +
                      (u32)r[1 + ref_stride] * mv_xcf * mv_ycf + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);
            //            s64 a = ((s64)r[0] << EIF_MV_PEL_SHIFT) + ((s64)r[0 + ref_stride] - (s64)r[0]) * (mv_yc & EIF_FRAC_MASK);
            //            s64 b = ((s64)r[1] << EIF_MV_PEL_SHIFT) + ((s64)r[1 + ref_stride] - (s64)r[1]) * (mv_yc & EIF_FRAC_MASK);
            //            s32 l0 = (s32)(((a << EIF_MV_PEL_SHIFT) + (b - a) * (mv_xc & EIF_FRAC_MASK) + EIF_ROUNDING_OFFSET) >> (2 * EIF_MV_PEL_SHIFT));

            //high-pass filtering
            valhor0 = (c[0] * prev2l0 + c[2] * prevl0 + c[0] * l0 + hoffset) >> hshift;

#if EIF_3TAP_HALFFIX
            valver0 = c[0] * ((valhor0 + buf_tmp0[0] + 0) >> 1) + (c[2] >> 1) * buf_tmp1[0];
#else
            valver0 = c[0] * ((valhor0 + buf_tmp0[0] + 1) >> 1) + (c[2] >> 1) * buf_tmp1[0];
#endif
            buf_tmp0[0] = buf_tmp1[0];
            buf_tmp1[0] = valhor0;
            dstval = (valver0 + (voffset>>1)) >> (vshift-1);

            if (h > 1 && w > 1)
            {
                dstval = min(dstval, max_val);
                dstval = max(dstval, min_val);
                cur_pos_in_dst[0] = dstval;
                cur_pos_in_dst += 1;
            }
            mv_x = mv_x + d_mv_hor_x;   //incrementing dx
            mv_y = mv_y + d_mv_hor_y;   //incrementing dy

            buf_tmp0 += 2;
            buf_tmp1 += 2;

            prev2l0 = prevl0;
            prevl0 = l0;
        }
        mv_x0 += d_mv_ver_x;
        mv_y0 += d_mv_ver_y;
        if (h > 1)
        {
            dst += block_width;
        }
    }
}
#endif

void eif_1pass_clipping(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, pel* p_cur_pos_in_ref, int ref_stride, pel *dst, int dst_stride, pel* p_tmp_buffer, int tmp_buffer_stride, int mv_x_max, int mv_y_max, int mv_x_min, int mv_y_min)
{
    double_pel buf_tmp0_base[2*(MAX_CU_SIZE + 2)];
    double_pel buf_tmp1_base[2*(MAX_CU_SIZE + 2)];
    int c[3];

    d_mv_hor_x += EIF_ONE / 2;
    d_mv_ver_y += EIF_ONE / 2;

    c[2] = tbl_mc_l_coeff_ds[0][2];
    c[1] = tbl_mc_l_coeff_ds[0][1];
    c[0] = tbl_mc_l_coeff_ds[0][0];

    const int vshift = EIF_IF_FILTER_PREC_HP * 2;
    const int voffset = 1 << (vshift - 1);

    pel min_val = 0;
    pel max_val = (1 << 10) - 1;

    for (int h = 0; h < block_height * 2 + 4; ++h)
    {
        int mv_y = mv_y0;
        int mv_x = mv_x0;
        int prevl0 = 0;
        int prevl1 = 0;
        int prev2l0 = 0;
        int prev2l1 = 0;
        int valhor0;
        int valver0;
        int dstval;

        //horizontal loop
        double_pel* buf_tmp0 = buf_tmp0_base;
        double_pel* buf_tmp1 = buf_tmp1_base;
        pel* cur_pos_in_dst = dst;
        pel* r = (p_cur_pos_in_ref + (mv_y0 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x0 >> EIF_MV_PEL_SHIFT));
        for (int w = 0; w < block_width * 2 + 4; w += 2)
        {
            int mv_xc = min(mv_x_max, max(mv_x_min, mv_x));
            int mv_yc = min(mv_y_max, max(mv_y_min, mv_y));
            int mv_xcf = mv_xc & EIF_FRAC_MASK;
            int mv_ycf = mv_yc & EIF_FRAC_MASK;
            //bilinear interpolation
            r = (p_cur_pos_in_ref + (mv_yc >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_xc >> EIF_MV_PEL_SHIFT));

            pel l0 = ((u32)r[0] * (EIF_ONE - mv_xcf) * (EIF_ONE - mv_ycf) +
                      (u32)r[1] * mv_xcf * (EIF_ONE - mv_ycf) +
                      (u32)r[0 + ref_stride] * (EIF_ONE - mv_xcf) * mv_ycf +
                      (u32)r[1 + ref_stride] * mv_xcf * mv_ycf + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);
            //            s64 a = ((s64)r[0] << EIF_MV_PEL_SHIFT) + ((s64)r[0 + ref_stride] - (s64)r[0]) * (mv_yc & EIF_FRAC_MASK);
            //            s64 b = ((s64)r[1] << EIF_MV_PEL_SHIFT) + ((s64)r[1 + ref_stride] - (s64)r[1]) * (mv_yc & EIF_FRAC_MASK);
            //            s32 l0 = (s32)(((a << EIF_MV_PEL_SHIFT) + (b - a) * (mv_xc & EIF_FRAC_MASK) + EIF_ROUNDING_OFFSET) >> (2 * EIF_MV_PEL_SHIFT));

            mv_x += d_mv_hor_x;   //incrementing dx
            mv_y += d_mv_hor_y;   //incrementing dy

            mv_xc = min(mv_x_max, max(mv_x_min, mv_x));
            mv_yc = min(mv_y_max, max(mv_y_min, mv_y));
            mv_xcf = mv_xc & EIF_FRAC_MASK;
            mv_ycf = mv_yc & EIF_FRAC_MASK;

            r = (p_cur_pos_in_ref + (mv_yc >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_xc >> EIF_MV_PEL_SHIFT));

            pel l1 = ((u32)r[0] * (EIF_ONE - mv_xcf) * (EIF_ONE - mv_ycf) +
                      (u32)r[1] * mv_xcf * (EIF_ONE - mv_ycf) +
                      (u32)r[0 + ref_stride] * (EIF_ONE - mv_xcf) * mv_ycf +
                      (u32)r[1 + ref_stride] * mv_xcf * mv_ycf + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);
            //            a = ((s64)r[0] << EIF_MV_PEL_SHIFT) + ((s64)r[0 + ref_stride] - (s64)r[0]) * (mv_yc & EIF_FRAC_MASK);
            //            b = ((s64)r[1] << EIF_MV_PEL_SHIFT) + ((s64)r[1 + ref_stride] - (s64)r[1]) * (mv_yc & EIF_FRAC_MASK);
            //            s32 l1 = (s32)(((a << EIF_MV_PEL_SHIFT) + (b - a) * (mv_xc & EIF_FRAC_MASK) + EIF_ROUNDING_OFFSET) >> (2 * EIF_MV_PEL_SHIFT));

            //high-pass filtering
            valhor0 = c[0] * prev2l0 + c[1] * prev2l1 + c[2] * prevl0 + c[1] * prevl1 + c[0] * l0;

            if (h & 1)
            {
                buf_tmp0[0] = buf_tmp0[0] + c[1] * valhor0;
                buf_tmp1[0] = buf_tmp1[0] + c[1] * valhor0;
            }
            else
            {
                valver0 = c[0] * valhor0 + buf_tmp0[0];
                buf_tmp0[0] = buf_tmp1[0] + c[2] * valhor0;
                buf_tmp1[0] = valhor0 * c[0];
                dstval = (valver0 + voffset) >> vshift;
                if (h > 2 && w > 2)
                {
                    dstval = min(dstval, max_val);
                    dstval = max(dstval, min_val);
                    cur_pos_in_dst[0] = dstval;
                    cur_pos_in_dst += 1;
                }
            }
            //fractional increment
            mv_x += d_mv_hor_x;   //incrementing dx
            mv_y += d_mv_hor_y;   //incrementing dy

            buf_tmp0 += 2;
            buf_tmp1 += 2;

            prev2l0 = prevl0;
            prev2l1 = prevl1;
            prevl0 = l0;
            prevl1 = l1;
        }
        mv_x0 += d_mv_ver_x;
        mv_y0 += d_mv_ver_y;
        if ((h > 2) && (h & 1) == 0)
            dst += block_width;
    }
}

void eif_1pass(int block_width, int block_height, int mv_x0, int mv_y0, int d_mv_hor_x, int d_mv_hor_y, int d_mv_ver_x, int d_mv_ver_y, pel* p_cur_pos_in_ref, int ref_stride, pel *dst, int dst_stride, pel* p_tmp_buffer, int tmp_buffer_stride)
{
    double_pel buf_tmp0_base[2*(MAX_CU_SIZE + 2)];
    double_pel buf_tmp1_base[2*(MAX_CU_SIZE + 2)];
    int c[3];

    d_mv_hor_x += EIF_ONE / 2;
    d_mv_ver_y += EIF_ONE / 2;

    c[2] = tbl_mc_l_coeff_ds[0][2];
    c[1] = tbl_mc_l_coeff_ds[0][1];
    c[0] = tbl_mc_l_coeff_ds[0][0];

    const int vshift = EIF_IF_FILTER_PREC_HP * 2;
    const int voffset = 1 << (vshift - 1);

    pel min_val = 0;
    pel max_val = (1 << 10) - 1;
    
    for (int h = 0; h < block_height * 2 + 4; ++h)
    {
        int mvdy1 = (mv_y0)& EIF_FRAC_MASK;
        int mvdx1 = (mv_x0)& EIF_FRAC_MASK;
        int prevl0 = 0;
        int prevl1 = 0;
        int prev2l0 = 0;
        int prev2l1 = 0;
        int valhor0;
        int valver0;
        int dstval;

        //horizontal loop
        double_pel* buf_tmp0 = buf_tmp0_base;
        double_pel* buf_tmp1 = buf_tmp1_base;
        pel* cur_pos_in_dst = dst;
        pel* r = (p_cur_pos_in_ref + (mv_y0 >> EIF_MV_PEL_SHIFT) * ref_stride + (mv_x0 >> EIF_MV_PEL_SHIFT));
        for (int w = 0; w < block_width * 2 + 4; w += 2)
        {
            //bilinear interpolation
            r += (mvdy1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mvdx1 >> EIF_MV_PEL_SHIFT);
            mvdy1 &= EIF_FRAC_MASK;
            mvdx1 &= EIF_FRAC_MASK;

            pel l0 = ((u32)r[0] * (EIF_ONE - mvdx1) * (EIF_ONE - mvdy1) +
                      (u32)r[1] * mvdx1 * (EIF_ONE - mvdy1) +
                      (u32)r[0 + ref_stride] * (EIF_ONE - mvdx1) * mvdy1 +
                      (u32)r[1 + ref_stride] * mvdx1 * mvdy1 + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);
            //          s64 a = ((s64)r[0] << EIF_MV_PEL_SHIFT) + ((s64)r[0 + ref_stride] - (s64)r[0]) * mvdy1;
            //          s64 b = ((s64)r[1] << EIF_MV_PEL_SHIFT) + ((s64)r[1 + ref_stride] - (s64)r[1]) * mvdy1;
            //          s32 l0 = (s32)(((a << EIF_MV_PEL_SHIFT) + (b - a) * mvdx1 + EIF_ROUNDING_OFFSET) >> (2 * EIF_MV_PEL_SHIFT));

            mvdx1 += d_mv_hor_x;   //incrementing dx
            mvdy1 += d_mv_hor_y;   //incrementing dy

            r += (mvdy1 >> EIF_MV_PEL_SHIFT) * ref_stride + (mvdx1 >> EIF_MV_PEL_SHIFT);
            mvdy1 &= EIF_FRAC_MASK;
            mvdx1 &= EIF_FRAC_MASK;
            pel l1 = ((u32)r[0] * (EIF_ONE - mvdx1) * (EIF_ONE - mvdy1) +
                      (u32)r[1] * mvdx1 * (EIF_ONE - mvdy1) +
                      (u32)r[0 + ref_stride] * (EIF_ONE - mvdx1) * mvdy1 +
                      (u32)r[1 + ref_stride] * mvdx1 * mvdy1 + EIF_ROUNDING_OFFSET) >> (EIF_MV_PEL_SHIFT * 2);
            //          a = ((s64)r[0] << EIF_MV_PEL_SHIFT) + ((s64)r[0 + ref_stride] - (s64)r[0]) * mvdy1;
            //          b = ((s64)r[1] << EIF_MV_PEL_SHIFT) + ((s64)r[1 + ref_stride] - (s64)r[1]) * mvdy1;
            //          s32 l1 = (s32)(((a << EIF_MV_PEL_SHIFT) + (b - a) * mvdx1 + EIF_ROUNDING_OFFSET) >> (2 * EIF_MV_PEL_SHIFT));

            //high-pass filtering
            valhor0 = c[0] * prev2l0 + c[1] * prev2l1 + c[2] * prevl0 + c[1] * prevl1 + c[0] * l0;

            if (h & 1)
            {
                buf_tmp0[0] = buf_tmp0[0] + c[1] * valhor0;
                buf_tmp1[0] = buf_tmp1[0] + c[1] * valhor0;
            }
            else
            {
                valver0 = c[0] * valhor0 + buf_tmp0[0];
                buf_tmp0[0] = buf_tmp1[0] + c[2] * valhor0;
                buf_tmp1[0] = valhor0 * c[0];
                dstval = (valver0 + voffset) >> vshift;
                if (h > 2 && w > 2)
                {
                    dstval = min(dstval, max_val);
                    dstval = max(dstval, min_val);
                    cur_pos_in_dst[0] = dstval;
                    cur_pos_in_dst += 1;
                }
            }
            //fractional increment
            mvdx1 += d_mv_hor_x;   //incrementing dx
            mvdy1 += d_mv_hor_y;   //incrementing dy

            buf_tmp0 += 2;
            buf_tmp1 += 2;

            prev2l0 = prevl0;
            prev2l1 = prevl1;
            prevl0 = l0;
            prevl1 = l1;
        }
        mv_x0 += d_mv_ver_x;
        mv_y0 += d_mv_ver_y;
        if ((h > 2) && (h & 1) == 0)
            dst += block_width;
    }
}
#endif //EIF

void evc_affine_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][VER_NUM][MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[2][N_C][MAX_CU_DIM], int vertex_num
#if EIF
                    , pel* tmp_buffer
#endif

                    )
{
    EVC_PIC *ref_pic;
    pel      *p0, *p1, *p2, *p3;
    int       i, j, bidx = 0;

    if(REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        evc_affine_mc_lc(x, y, pic_w, pic_h, w, h, mv[REFP_0], ref_pic, pred[0], vertex_num
#if EIF
                          , tmp_buffer
#endif
                          );

        bidx++;
    }

    if(REFI_IS_VALID(refi[REFP_1]))
    {
        /* backward */
        ref_pic = refp[refi[REFP_1]][REFP_1].pic;
        evc_affine_mc_lc(x, y, pic_w, pic_h, w, h, mv[REFP_1], ref_pic, pred[bidx], vertex_num
#if EIF
                          , tmp_buffer
#endif
                          );

        bidx++;
    }

    if(bidx == 2)
    {
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
        p0 = pred[0][U_C];
        p1 = pred[1][U_C];
        p2 = pred[0][V_C];
        p3 = pred[1][V_C];
        w >>= 1;
        h >>= 1;
        for(j = 0; j < h; j++)
        {
            for(i = 0; i < w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
                p2[i] = (p2[i] + p3[i] + 1) >> 1;
            }
            p0 += w;
            p1 += w;
            p2 += w;
            p3 += w;
        }
    }
}
#endif
