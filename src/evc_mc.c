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
#if M50662_AFFINE_BANDWIDTH_CLIPMV
static int g_aff_mvDevBB2_125[5] = { 64, 128, 272, 560, 1136 };
static int g_aff_sizeBB2_125[5] = { 272, 544, 1088, 2176, 4352 };
#endif
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

#if EIF_NEW_BILINEAR
static const s16 tbl_bl_eif_32_phases_mc_l_coeff[32][2] =
{
  { 64, 0  },
  { 62, 2  },
  { 60, 4  },
  { 58, 6  },
  { 56, 8  },
  { 54, 10 },
  { 52, 12 },
  { 50, 14 },
  { 48, 16 },
  { 46, 18 },
  { 44, 20 },
  { 42, 22 },
  { 40, 24 },
  { 38, 26 },
  { 36, 28 },
  { 34, 30 },
  { 32, 32 },
  { 30, 34 },
  { 28, 36 },
  { 26, 38 },
  { 24, 40 },
  { 22, 42 },
  { 20, 44 },
  { 18, 46 },
  { 16, 48 },
  { 14, 50 },
  { 12, 52 },
  { 10, 54 },
  { 8,  56 },
  { 6,  58 },
  { 4,  60 },
  { 2,  62 }
};
#endif

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
        { -1, 4, -10, 58, 17,  -5,  1,  0 },
        { -1, 4, -11, 52, 26,  -8,  3, -1 },
        { -1, 3,  -9, 47, 31, -10,  4, -1 },
        { -1, 4, -11, 45, 34, -10,  4, -1 },
        { -1, 4, -11, 40, 40, -11,  4, -1 },
        { -1, 4, -10, 34, 45, -11,  4, -1 },
        { -1, 4, -10, 31, 47,  -9,  3, -1 },
        { -1, 3,  -8, 26, 52, -11,  4, -1 },
        {  0, 1,  -5, 17, 58, -10,  4, -1 },
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
        { -2, 58, 10, -2 },
        { -3, 57, 12, -2 },
        { -4, 56, 14, -2 },
        { -4, 55, 15, -2 },
        { -4, 54, 16, -2 },
        { -5, 53, 18, -2 },
        { -6, 52, 20, -2 },
        { -6, 49, 24, -3 },
        { -6, 46, 28, -4 },
        { -5, 44, 29, -4 },
        { -4, 42, 30, -4 },
        { -4, 39, 33, -4 },
        { -4, 36, 36, -4 },
        { -4, 33, 39, -4 },
        { -4, 30, 42, -4 },
        { -4, 29, 44, -5 },
        { -4, 28, 46, -6 },
        { -3, 24, 49, -6 },
        { -2, 20, 52, -6 },
        { -2, 18, 53, -5 },
        { -2, 16, 54, -4 },
        { -2, 15, 55, -4 },
        { -2, 14, 56, -4 },
        { -2, 12, 57, -3 },
        { -2, 10, 58, -2 },
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
__inline s32 div_for_maxq7(s64 N, s64 D)
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

void prefetch_for_mc(int x, int y, 
#if M50761_DMVR_SIMP_SUBPUPAD
  int pu_x, int pu_y, int pu_w, int pu_h,
#endif
  int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*mv)[MV_D], EVC_REFP(*refp)[REFP_NUM]
  , int iteration
  , pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
)
{
  s16          mv_temp[REFP_NUM][MV_D];
#if M50761_DMVR_SIMP_SUBPUPAD
  int l_w = pu_w, l_h = pu_h;
  int c_w = pu_w >> 1, c_h = pu_h >> 1;
  int topleft_x_offset = pu_x - x;
  int topleft_y_offset = pu_y - y;
#else
  int l_w = w, l_h = h; //SEMIH: sub-pu width and height
  int c_w = w >> 1, c_h = h >> 1;//SEMIH: sub-pu width and height
#endif

  int num_extra_pixel_left_for_filter;
  for (int i = 0; i < REFP_NUM; ++i)
  {
    int filtersize = NTAPS_LUMA;
    num_extra_pixel_left_for_filter = ((filtersize >> 1) - 1);
#if M50761_DMVR_SIMP_SUBPUPAD
    int offset = (DMVR_ITER_COUNT + topleft_y_offset) * PAD_BUFFER_STRIDE + topleft_x_offset + DMVR_ITER_COUNT;
#else
    int offset = ((DMVR_ITER_COUNT) * (PAD_BUFFER_STRIDE + 1)); //SEMIH: add here the sub-pu offset
#endif
    int padsize = DMVR_PAD_LENGTH;
    int          qpel_gmv_x, qpel_gmv_y;
    EVC_PIC    *ref_pic;
    mv_clip_only_one_ref_dmvr(x, y, pic_w, pic_h, w, h, mv[i], mv_temp[i]);
#if M50761_DMVR_SIMP_SUBPUPAD
    qpel_gmv_x = ((pu_x << 2) + mv_temp[i][MV_X]) << 2;
    qpel_gmv_y = ((pu_y << 2) + mv_temp[i][MV_Y]) << 2;
#else
    qpel_gmv_x = ((x << 2) + mv_temp[i][MV_X]) << 2;  //SEMIH: add here the sub-pu offset 
    qpel_gmv_y = ((y << 2) + mv_temp[i][MV_Y]) << 2;  //SEMIH: add here the sub-pu offset
#endif
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
#if M50761_DMVR_SIMP_SUBPUPAD
    offset = (DMVR_ITER_COUNT + (topleft_y_offset>>1)) * PAD_BUFFER_STRIDE + (topleft_x_offset>>1) + DMVR_ITER_COUNT;
#else
    offset = (DMVR_ITER_COUNT);
    offset = offset *(PAD_BUFFER_STRIDE + 1); //SEMIH: add here the sub-pu offset
#endif
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
    evc_mc_c(qpel_gmv_x, qpel_gmv_y, ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cu_pred_stride >> 1, temp, w >> 1, h >> 1);
    temp = pred[i][V_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
    evc_mc_c(qpel_gmv_x, qpel_gmv_y, ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cu_pred_stride >> 1, temp, w >> 1, h >> 1);
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
#if !M50761_DMVR_SIMP_SUBPUPAD
  prefetch_for_mc(x, y, pic_w, pic_h, w, h, refi, starting_mv, refp, iteration, dmvr_padding_buf);
#endif
#endif

  num = 0;
  for (int startY = 0, subPuStartY = y; subPuStartY < (y + h); subPuStartY = subPuStartY + dy, startY += dy)
  {
    for (int startX = 0, subPuStartX = x; subPuStartX < (x + w); subPuStartX = subPuStartX + dx, startX += dx)
    {
#if M50761_DMVR_SIMP_SUBPUPAD
      prefetch_for_mc(x, y, subPuStartX, subPuStartY, dx, dy, pic_w, pic_h, w, h, refi, starting_mv, refp, iteration, dmvr_padding_buf);
#endif
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
    s16          mv_before_clipping[REFP_NUM][MV_D]; //store it to pass it to interpolation function for deriving correct interpolation filter

    mv_before_clipping[REFP_0][MV_X]=mv[REFP_0][MV_X];
    mv_before_clipping[REFP_0][MV_Y]=mv[REFP_0][MV_Y];
    mv_before_clipping[REFP_1][MV_X]=mv[REFP_1][MV_X];
    mv_before_clipping[REFP_1][MV_Y]=mv[REFP_1][MV_Y];

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
    
#if !M50761_DMVR_RESTRICT_SMALL_BLOCKS
    apply_DMVR = apply_DMVR && (!((w == 4 && h <= 8) || (w <= 8 && h == 4)));
#else
    apply_DMVR = apply_DMVR && w >= 8 && h >= 8;
#endif

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
            evc_mc_l(mv_before_clipping[REFP_0][MV_X]<<2, mv_before_clipping[REFP_0][MV_Y]<<2, ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[0][Y_C], w, h);
        }
#if DMVR
        if(!REFI_IS_VALID(refi[REFP_1]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(mv_before_clipping[REFP_0][MV_X]<<2, mv_before_clipping[REFP_0][MV_Y]<<2, ref_pic->u, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[0][U_C], w >> 1, h >> 1);
            evc_mc_c(mv_before_clipping[REFP_0][MV_X]<<2, mv_before_clipping[REFP_0][MV_Y]<<2, ref_pic->v, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[0][V_C], w >> 1, h >> 1);
        }
#else

#if DMVR
        if(!apply_DMVR)
#endif
        {
            evc_mc_l(mv_before_clipping[REFP_0][MV_X], mv_before_clipping[REFP_0][MV_Y], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, w, pred[0][Y_C], w, h);
        }

#if DMVR
        if(!REFI_IS_VALID(refi[REFP_1]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(mv_before_clipping[REFP_0][MV_X], mv_before_clipping[REFP_0][MV_Y], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[0][U_C], w >> 1, h >> 1);
            evc_mc_c(mv_before_clipping[REFP_0][MV_X], mv_before_clipping[REFP_0][MV_Y], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[0][V_C], w >> 1, h >> 1);
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
            evc_mc_l(mv_before_clipping[REFP_1][MV_X]<<2, mv_before_clipping[REFP_1][MV_Y]<<2, ref_pic->y, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_l, w, pred[bidx][Y_C], w, h);
        }
#if DMVR
        if(!REFI_IS_VALID(refi[REFP_0]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(mv_before_clipping[REFP_1][MV_X]<<2, mv_before_clipping[REFP_1][MV_Y]<<2,ref_pic->u, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[bidx][U_C], w >> 1, h >> 1);
            evc_mc_c(mv_before_clipping[REFP_1][MV_X]<<2, mv_before_clipping[REFP_1][MV_Y]<<2,ref_pic->v, (qpel_gmv_x << 2), (qpel_gmv_y << 2), ref_pic->s_c, w >> 1, pred[bidx][V_C], w >> 1, h >> 1);
        }
#else

#if DMVR
        if(!apply_DMVR)
#endif
        {
            evc_mc_l(mv_before_clipping[REFP_1][MV_X], mv_before_clipping[REFP_1][MV_Y], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, w, pred[bidx][Y_C], w, h);
        }

#if DMVR
        if(!REFI_IS_VALID(refi[REFP_0]) || !apply_DMVR || !dmvr_poc_condition)
#endif
        {
            evc_mc_c(mv_before_clipping[REFP_1][MV_X], mv_before_clipping[REFP_1][MV_Y],ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[bidx][U_C], w >> 1, h >> 1);
            evc_mc_c(mv_before_clipping[REFP_1][MV_X], mv_before_clipping[REFP_1][MV_Y],ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, w >> 1, pred[bidx][V_C], w >> 1, h >> 1);
        }
#endif
        bidx++;
    }

    if(bidx == 2)
    {
#if DMVR
        BOOL template_needs_update = FALSE;
        s32 center_cost[2] = {1 << 30, 1 << 30};
#if M50761_REMOVE_BIBLOCKS_8x4
        evc_assert(check_bi_applicability_rdo(SLICE_B, w, h));
#endif

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
#if IBC
void evc_IBC_mc(int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D], EVC_PIC *ref_pic, pel pred[N_C][MAX_CU_DIM]
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
  int i = 0, j = 0;
  int size = 0;

  int cuw = 0, cuh = 0;
  int stride = 0;
  int mv_x = 0, mv_y = 0;

  pel *dst = NULL;
  pel *ref = NULL;

  cuw = 1 << log2_cuw;
  cuh = 1 << log2_cuh;
  mv_x = mv[0];
  mv_y = mv[1];
  stride = ref_pic->s_l;

#if M50761_CHROMA_NOT_SPLIT
  if (evc_check_luma(tree_cons))
  {
#endif
  dst = pred[0];
  ref = ref_pic->y + (mv_y + y) * stride + (mv_x + x);
  size = sizeof(pel) * cuw;

  for (i = 0; i < cuh; i++)
  {
    evc_mcpy(dst, ref, size);
    ref += stride;
    dst += cuw;
  }
#if M50761_CHROMA_NOT_SPLIT
  }
  if (evc_check_chroma(tree_cons))
  {
#endif
  cuw >>= 1;
  cuh >>= 1;
  x >>= 1;
  y >>= 1;
  mv_x >>= 1;
  mv_y >>= 1;
  log2_cuw--;
  log2_cuh--;
  stride = ref_pic->s_c;

  dst = pred[1];
  ref = ref_pic->u + (mv_y + y) * stride + (mv_x + x);
  size = sizeof(pel) * cuw;
  for (i = 0; i < cuh; i++)
  {
    evc_mcpy(dst, ref, size);
    ref += stride;
    dst += cuw;
  }

  dst = pred[2];
  ref = ref_pic->v + (mv_y + y) * stride + (mv_x + x);
  size = sizeof(pel) * cuw;
  for (i = 0; i < cuh; i++)
  {
    evc_mcpy(dst, ref, size);
    ref += stride;
    dst += cuw;
  }
#if M50761_CHROMA_NOT_SPLIT
  }
#endif
}
#endif
#if AFFINE
#if !M48933_AFFINE
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
#endif
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV
BOOL check_eif_clipMV_flag(s16 ac_mv[VER_NUM][MV_D], int d_hor[MV_D], int d_ver[MV_D], int mv_precision)
{

#if EIF_FORBID_NON_CONTINUOUS_MEMORY_ACCESS
    if (d_ver[MV_Y] < -1 << mv_precision)
        return FALSE;
#endif

#if EIF_NUM_FETCHED_LINES_BASIC_RESTRICTION
    if ((max(0, d_ver[MV_Y]) + abs(d_hor[MV_Y])) * (1 + EIF_HW_SUBBLOCK_SIZE) > (EIF_NUM_ALLOWED_FETCHED_LINES_FOR_THE_FIRST_LINE - 2) << mv_precision)
        return FALSE;
#endif

    return TRUE;
}
#endif

#if M50662_AFFINE_BANDWIDTH_CLIPMV
void evc_derive_mv_clip_range(int cuw, int cuh, int mv_scale_hor, int mv_scale_ver, int dmv_hor_x, int dmv_hor_y, int dmv_ver_x, int dmv_ver_y, int *hor_max, int *hor_min, int *ver_max, int *ver_min)
{
    // get clip MV Range
    s32 affine_center_mv_hor, affine_center_mv_ver;
    int pos_x = cuw >> 1;
    int pos_y = cuh >> 1;

    affine_center_mv_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
    affine_center_mv_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

    evc_mv_rounding_s32(affine_center_mv_hor, affine_center_mv_ver, &affine_center_mv_hor, &affine_center_mv_ver, 5, 0);
    affine_center_mv_hor = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, affine_center_mv_hor);
    affine_center_mv_ver = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, affine_center_mv_ver);

    s32 mv_hor_min, mv_ver_min, mv_hor_max, mv_ver_max;

    mv_hor_min = affine_center_mv_hor - g_aff_mvDevBB2_125[evc_tbl_log2[cuw] - 3];
    mv_ver_min = affine_center_mv_ver - g_aff_mvDevBB2_125[evc_tbl_log2[cuh] - 3];
    mv_hor_max = affine_center_mv_hor + g_aff_mvDevBB2_125[evc_tbl_log2[cuw] - 3];
    mv_ver_max = affine_center_mv_ver + g_aff_mvDevBB2_125[evc_tbl_log2[cuh] - 3];

#if FIX_AFFINE_CLIP
    mv_hor_max = max(mv_hor_max, *hor_min);
    mv_ver_max = max(mv_ver_max, *ver_min);
    mv_hor_min = min(mv_hor_min, *hor_max);
    mv_ver_min = min(mv_ver_min, *ver_max);
#endif

    *hor_max = min(*hor_max, mv_hor_max);
    *ver_max = min(*ver_max, mv_ver_max);
    *hor_min = max(*hor_min, mv_hor_min);
    *ver_min = max(*ver_min, mv_ver_min);
}
void evc_derive_mv_clip_range_2D(int cuw, int cuh, int mv_scale_hor, int mv_scale_ver, int dmv_hor_x, int dmv_hor_y, int dmv_ver_x, int dmv_ver_y, int *hor_max, int *hor_min, int *ver_max, int *ver_min, int vertex_num, s16 ac_mv[VER_NUM][MV_D])
{
    // get clip MV Range
    s32 affine_center_mv_hor, affine_center_mv_ver;
    int pos_x = cuw >> 1;
    int pos_y = cuh >> 1;

    affine_center_mv_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
    affine_center_mv_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

    evc_mv_rounding_s32(affine_center_mv_hor, affine_center_mv_ver, &affine_center_mv_hor, &affine_center_mv_ver, 5, 0);
    affine_center_mv_hor = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, affine_center_mv_hor);
    affine_center_mv_ver = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, affine_center_mv_ver);

    int mv_corners[4][MV_D];

    mv_corners[0][MV_X] = ac_mv[0][MV_X];
    mv_corners[0][MV_Y] = ac_mv[0][MV_Y];
    mv_corners[1][MV_X] = ac_mv[1][MV_X];
    mv_corners[1][MV_Y] = ac_mv[1][MV_Y];
    if (vertex_num == 3)
    {
        mv_corners[2][MV_X] = ac_mv[2][MV_X];
        mv_corners[2][MV_Y] = ac_mv[2][MV_Y];
    }
    else
    {
        mv_corners[2][MV_X] = mv_scale_hor + dmv_ver_x * cuh;
        mv_corners[2][MV_Y] = mv_scale_ver + dmv_ver_y * cuh;

        evc_mv_rounding_s32(mv_corners[2][MV_X], mv_corners[2][MV_Y], &mv_corners[2][MV_X], &mv_corners[2][MV_Y], 5, 0);
        mv_corners[2][MV_X] = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_corners[2][MV_X]);
        mv_corners[2][MV_Y] = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_corners[2][MV_Y]);
    }
    mv_corners[3][MV_X] = mv_scale_hor + dmv_hor_x * cuw + dmv_ver_x * cuh;
    mv_corners[3][MV_Y] = mv_scale_ver + dmv_hor_y * cuw + dmv_ver_y * cuh;

    evc_mv_rounding_s32(mv_corners[3][MV_X], mv_corners[3][MV_Y], &mv_corners[3][MV_X], &mv_corners[3][MV_Y], 5, 0);
    mv_corners[3][MV_X] = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_corners[3][MV_X]);
    mv_corners[3][MV_Y] = EVC_CLIP3(-(2 << 17), (2 << 17) - 1, mv_corners[3][MV_Y]);

    int mv_max[MV_D];
    int mv_min[MV_D];
    mv_max[MV_X] = max(max(mv_corners[0][MV_X], mv_corners[1][MV_X]), max(mv_corners[2][MV_X], mv_corners[3][MV_X]));
    mv_max[MV_Y] = max(max(mv_corners[0][MV_Y], mv_corners[1][MV_Y]), max(mv_corners[2][MV_Y], mv_corners[3][MV_Y]));
    mv_min[MV_X] = min(min(mv_corners[0][MV_X], mv_corners[1][MV_X]), min(mv_corners[2][MV_X], mv_corners[3][MV_X]));
    mv_min[MV_Y] = min(min(mv_corners[0][MV_Y], mv_corners[1][MV_Y]), min(mv_corners[2][MV_Y], mv_corners[3][MV_Y]));
    int mv_spread[MV_D];
    mv_spread[MV_X] = mv_max[MV_X] - mv_min[MV_X];
    mv_spread[MV_Y] = mv_max[MV_Y] - mv_min[MV_Y];

    double ratio = MEMORY_BANDWIDTH_THRESHOLD;
    int bounding_block_width = g_aff_sizeBB2_125[evc_tbl_log2[cuw] - 3];
    int bounding_block_height = g_aff_sizeBB2_125[evc_tbl_log2[cuw] - 3];
    s32 mv_hor_min, mv_ver_min, mv_hor_max, mv_ver_max;
    if ((mv_spread[MV_X] + (cuw << 4) + 32) * (mv_spread[MV_Y] + (cuh << 4) + 32) > (bounding_block_width * bounding_block_height))
    {
        mv_hor_min = affine_center_mv_hor - g_aff_mvDevBB2_125[evc_tbl_log2[cuw] - 3];
        mv_ver_min = affine_center_mv_ver - g_aff_mvDevBB2_125[evc_tbl_log2[cuh] - 3];
        mv_hor_max = affine_center_mv_hor + g_aff_mvDevBB2_125[evc_tbl_log2[cuw] - 3];
        mv_ver_max = affine_center_mv_ver + g_aff_mvDevBB2_125[evc_tbl_log2[cuh] - 3];
    }
    else if ((mv_spread[MV_X] + (cuw << 4)) > (bounding_block_width - 32) || (mv_spread[MV_Y] + (cuh << 4)) > (bounding_block_height - 32))
    {
        if ((mv_spread[MV_X] + (cuw << 4)) > (bounding_block_width - 32))
        {
            mv_hor_min = mv_min[MV_X];
            mv_hor_max = mv_max[MV_X];
            mv_ver_min = *ver_min;
            mv_ver_max = *ver_max;
        }
        else
        {
            mv_hor_min = *hor_min;
            mv_hor_max = *hor_max;
            mv_ver_min = mv_min[MV_Y];
            mv_ver_max = mv_max[MV_Y];
        }
    }
    else
    {
        mv_hor_min = *hor_min;
        mv_hor_max = *hor_max;
        mv_ver_min = *ver_min;
        mv_ver_max = *ver_max;
    }

#if FIX_AFFINE_CLIP
    mv_hor_max = max(mv_hor_max, *hor_min);
    mv_ver_max = max(mv_ver_max, *ver_min);
    mv_hor_min = min(mv_hor_min, *hor_max);
    mv_ver_min = min(mv_ver_min, *ver_max);
#endif

    *hor_max = min(*hor_max, mv_hor_max);
    *ver_max = min(*ver_max, mv_ver_max);
    *hor_min = max(*hor_min, mv_hor_min);
    *ver_min = max(*ver_min, mv_ver_min);
}
#endif
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
    int bit = MAX_CU_LOG2;

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

#if EIF
#if M50761_AFFINE_ADAPT_SUB_SIZE || M50761_AFFINE_SUB_SIZE_LUT
    int b_eif = sub_w < AFFINE_ADAPT_EIF_SIZE || sub_h < AFFINE_ADAPT_EIF_SIZE;
#if M50662_AFFINE_BANDWIDTH_CLIPMV
    int d_hor[MV_D] = { dmv_hor_x, dmv_hor_y }, d_ver[MV_D] = { dmv_ver_x, dmv_ver_y };
    int mv_precision = MAX_CU_LOG2 + MC_PRECISION_ADD;
    BOOL clipMV = FALSE;
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV   // to be updated by HW
    //clipMV = b_eif && check_eif_clipMV_flag(ac_mv, d_hor, d_ver, mv_precision);
    clipMV = check_eif_clipMV_flag(ac_mv, d_hor, d_ver, mv_precision);
#endif
#endif
#else
    int b_eif = 0;
    int mv_w = max(abs(ac_mv[1][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[1][MV_Y] - ac_mv[0][MV_Y]));

    if (mv_w)
    {
      w = max((int)((cuw >> 2) / mv_w), 1);
      if (w < AFFINE_ADAPT_EIF_SIZE)
        b_eif = 1;
    }
    if (vertex_num == 3 && !b_eif)
    {
      int mv_h = max(abs(ac_mv[2][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[2][MV_Y] - ac_mv[0][MV_Y]));
      if (mv_h)
      {
        h = max((int)((cuh >> 2) / mv_h), 1);
        if (h < AFFINE_ADAPT_EIF_SIZE)
          b_eif = 1;
      }
    }
#endif
#if M50662_AFFINE_BANDWIDTH_CLIPMV
    if (b_eif)
    {
        evc_derive_mv_clip_range(cuw, cuh, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, &hor_max, &hor_min, &ver_max, &ver_min);
        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->y, ref_pic->s_l, pred, cuw, tmp_buffer, bit + 2, Y_C);
        return;
    }
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV
    else if (clipMV)
    {
        evc_derive_mv_clip_range_2D(cuw, cuh, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, &hor_max, &hor_min, &ver_max, &ver_min, vertex_num, ac_mv);
        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->y, ref_pic->s_l, pred, cuw, tmp_buffer, bit + 2, Y_C);
        return;
    }
#endif
#else
    if (b_eif)
    {
      evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
                 hor_max, ver_max, hor_min, ver_min, ref_pic->y , ref_pic->s_l, pred, cuw, tmp_buffer, bit + 2, Y_C);
      return;
    }
#endif
#endif
    int mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori;
    // get prediction block by block
    for(h = 0; h < cuh; h += sub_h)
    {
        for(w = 0; w < cuw; w += sub_w)
        {
#if M48933_AFFINE
            mv_scale_tmp_hor = (mv_scale_hor + dmv_hor_x * half_w + dmv_ver_x * half_h);
            mv_scale_tmp_ver = (mv_scale_ver + dmv_hor_y * half_w + dmv_ver_y * half_h);
            evc_mv_rounding_s32( mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, shift, 0 );
            mv_scale_tmp_hor = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor );
            mv_scale_tmp_ver = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver );
#else
            mv_scale_tmp_hor = (mv_scale_hor + dmv_hor_x * half_w + dmv_ver_x * half_h) >> shift;
            mv_scale_tmp_ver = (mv_scale_ver + dmv_hor_y * half_w + dmv_ver_y * half_h) >> shift;
#endif
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            // clip
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));

            qpel_gmv_x = ((x + w) << mc_prec) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << mc_prec) + mv_scale_tmp_ver;

            evc_mc_l(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, cuw, (pred_y + w), sub_w, sub_h);
        }
        pred_y += (cuw * sub_h);
    }
}

void evc_affine_mc_lc( int x, int y, int pic_w, int pic_h, int cuw, int cuh, s16 ac_mv[VER_NUM][MV_D], EVC_PIC* ref_pic, pel pred[N_C][MAX_CU_DIM], int vertex_num
#if M50761_AFFINE_ADAPT_SUB_SIZE
    , int sub_w, int sub_h
#endif
#if EIF
    , pel* tmp_buffer_for_eif
#endif
)
{
    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred[Y_C], *pred_u = pred[U_C], *pred_v = pred[V_C];
#if !M50761_AFFINE_ADAPT_SUB_SIZE
    int sub_w, sub_h;
#endif
    int w, h;
    int half_w, half_h;
    int bit = MAX_CU_LOG2;

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

    // get clip MV Range
    hor_max = (pic_w + MAX_CU_SIZE - x - cuw + 1) << mc_prec;
    ver_max = (pic_h + MAX_CU_SIZE - y - cuh + 1) << mc_prec;
    hor_min = (-MAX_CU_SIZE - x) << mc_prec;
    ver_min = (-MAX_CU_SIZE - y) << mc_prec;

    // get sub block size
#if !M50761_AFFINE_ADAPT_SUB_SIZE
    derive_affine_subblock_size( ac_mv, cuw, cuh, &sub_w, &sub_h, vertex_num );
#endif
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = ((ac_mv[1][MV_X] - ac_mv[0][MV_X]) << bit) >> evc_tbl_log2[cuw];     // deltaMvHor
    dmv_hor_y = ((ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << bit) >> evc_tbl_log2[cuw];
    if ( vertex_num == 3 )
    {
        dmv_ver_x = ((ac_mv[2][MV_X] - ac_mv[0][MV_X]) << bit) >> evc_tbl_log2[cuh]; // deltaMvVer
        dmv_ver_y = ((ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << bit) >> evc_tbl_log2[cuh];
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                                       // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

#if EIF
#if M50761_AFFINE_ADAPT_SUB_SIZE || M50761_AFFINE_SUB_SIZE_LUT
    int b_eif = sub_w < AFFINE_ADAPT_EIF_SIZE || sub_h < AFFINE_ADAPT_EIF_SIZE;
#if M50662_AFFINE_BANDWIDTH_CLIPMV
    int d_hor[MV_D] = { dmv_hor_x, dmv_hor_y }, d_ver[MV_D] = { dmv_ver_x, dmv_ver_y };
    int mv_precision = MAX_CU_LOG2 + MC_PRECISION_ADD;
    BOOL clipMV = FALSE;
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV  // to be updated by HW
    //clipMV = b_eif && check_eif_clipMV_flag(ac_mv, d_hor, d_ver, mv_precision);
    clipMV = check_eif_clipMV_flag(ac_mv, d_hor, d_ver, mv_precision);
#endif
#endif
#else
    int b_eif = 0;
    int mv_w = max(abs(ac_mv[1][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[1][MV_Y] - ac_mv[0][MV_Y]));

    if (mv_w)
    {
      w = max((int)((cuw >> 2) / mv_w), 1);
      if (w < AFFINE_ADAPT_EIF_SIZE)
        b_eif = 1;
    }
    if (vertex_num == 3 && !b_eif)
    {
      int mv_h = max(abs(ac_mv[2][MV_X] - ac_mv[0][MV_X]), abs(ac_mv[2][MV_Y] - ac_mv[0][MV_Y]));
      if (mv_h)
      {
        h = max((int)((cuh >> 2) / mv_h), 1);
        if (h < AFFINE_ADAPT_EIF_SIZE)
          b_eif = 1;
      }
    }
#endif
#if M50662_AFFINE_BANDWIDTH_CLIPMV
    if (b_eif)
    {
        evc_derive_mv_clip_range(cuw, cuh, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, &hor_max, &hor_min, &ver_max, &ver_min);

        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->y, ref_pic->s_l, pred[Y_C], cuw, tmp_buffer_for_eif, bit + 2, Y_C);

        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->u, ref_pic->s_c, pred[U_C], cuw >> 1, tmp_buffer_for_eif, bit + 2, U_C);

        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->v, ref_pic->s_c, pred[V_C], cuw >> 1, tmp_buffer_for_eif, bit + 2, V_C);
        return;
    }
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV
    else if (clipMV)
    {
        evc_derive_mv_clip_range_2D(cuw, cuh, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, &hor_max, &hor_min, &ver_max, &ver_min, vertex_num, ac_mv);

        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->y, ref_pic->s_l, pred[Y_C], cuw, tmp_buffer_for_eif, bit + 2, Y_C);

        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->u, ref_pic->s_c, pred[U_C], cuw >> 1, tmp_buffer_for_eif, bit + 2, U_C);

        evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
            hor_max, ver_max, hor_min, ver_min, ref_pic->v, ref_pic->s_c, pred[V_C], cuw >> 1, tmp_buffer_for_eif, bit + 2, V_C);
        return;
    }
#endif
#else
    if (b_eif)
    {
      evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
                 hor_max, ver_max, hor_min, ver_min, ref_pic->y, ref_pic->s_l, pred[Y_C], cuw, tmp_buffer_for_eif, bit + 2, Y_C);

      evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
                 hor_max, ver_max, hor_min, ver_min, ref_pic->u, ref_pic->s_c, pred[U_C], cuw >> 1, tmp_buffer_for_eif, bit + 2, U_C);

      evc_eif_mc(cuw, cuh, x, y, mv_scale_hor, mv_scale_ver, dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y,
                 hor_max, ver_max, hor_min, ver_min, ref_pic->v, ref_pic->s_c, pred[V_C], cuw >> 1, tmp_buffer_for_eif, bit + 2, V_C);
      return;
    }
#endif
#endif
    int mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori;

    // get prediction block by block
    for(h = 0; h < cuh; h += sub_h)
    {
        for(w = 0; w < cuw; w += sub_w)
        {
#if M48933_AFFINE
            mv_scale_tmp_hor = (mv_scale_hor + dmv_hor_x * half_w + dmv_ver_x * half_h);
            mv_scale_tmp_ver = (mv_scale_ver + dmv_hor_y * half_w + dmv_ver_y * half_h);
            evc_mv_rounding_s32( mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, shift, 0 );
            mv_scale_tmp_hor = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor );
            mv_scale_tmp_ver = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver );
#else
            mv_scale_tmp_hor = (mv_scale_hor + dmv_hor_x * half_w + dmv_ver_x * half_h) >> shift;
            mv_scale_tmp_ver = (mv_scale_ver + dmv_hor_y * half_w + dmv_ver_y * half_h) >> shift;
#endif
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            // clip
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));

            qpel_gmv_x = ((x + w) << mc_prec) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << mc_prec) + mv_scale_tmp_ver;

            evc_mc_l(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->s_l, cuw, (pred_y + w), sub_w, sub_h);

#if (AFFINE_MIN_BLOCK_SIZE == 1)
            if((w & 1) == 0 && (h & 1) == 0)
            {
                evc_mc_c(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_u + (w >> 1), max((sub_w >> 1), 1), max((sub_h >> 1), 1));
                evc_mc_c(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_v + (w >> 1), max((sub_w >> 1), 1), max((sub_h >> 1), 1));
            }
#else
            evc_mc_c(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_u + (w >> 1), sub_w >> 1, sub_h >> 1);
            evc_mc_c(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->s_c, cuw >> 1, pred_v + (w >> 1), sub_w >> 1, sub_h >> 1);
#endif
        }

        pred_y += (cuw * sub_h);
        pred_u += (cuw * sub_h) >> 2;
        pred_v += (cuw * sub_h) >> 2;
    }
}

#if EIF

BOOL can_mv_clipping_occurs(int block_width, int block_height, int mv0[MV_D], int d_x[MV_D], int d_y[MV_D], int mv_max[MV_D], int mv_min[MV_D])
{
  int mv_corners[2][2][MV_D];
  BOOL mv_clip_occurs[MV_D] = { FALSE, FALSE };

  int mv[MV_D] = { mv0[MV_X] - d_x[MV_X] - d_y[MV_X], mv0[MV_Y] - d_x[MV_Y] - d_y[MV_Y] }; //set to pos (-1, -1)

  block_width = block_width + 1;
  block_height = block_height + 1;

  assert(MV_Y - MV_X == 1);

  for (int coord = MV_X; coord <= MV_Y; ++coord)
  {
    mv_corners[0][0][coord] = mv[coord];
    mv_corners[0][1][coord] = mv[coord] + block_width  * d_x[coord];
    mv_corners[1][0][coord] = mv[coord] + block_height * d_y[coord];
    mv_corners[1][1][coord] = mv[coord] + block_width  * d_x[coord] + block_height * d_y[coord];

    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j)
      {
        if (mv_corners[i][j][coord] > mv_max[coord] || mv_corners[i][j][coord] < mv_min[coord])
          mv_clip_occurs[coord] = TRUE;
      }
  }

  return mv_clip_occurs[MV_X] || mv_clip_occurs[MV_Y];
}

void evc_eif_filter( int block_width, int block_height, pel* p_tmp_buf, int tmp_buf_stride, pel *p_dst, int dst_stride, int shifts[4], int offsets[4], int bit_depth )
{
  pel* p_buf = p_tmp_buf + 1;

  for (int y = 0; y <= block_height + 1; ++y, p_buf += tmp_buf_stride)
  {
    pel* t = p_buf;

    for (int x = 1; x <= block_width; ++x, ++t)
      t[-1] = (-t[-1] + (t[0] * 10) - t[1] + offsets[2]) >> shifts[2];
  }

  p_buf = p_tmp_buf + tmp_buf_stride;

  for (int y = 0; y < block_height; ++y, p_buf += tmp_buf_stride, p_dst += dst_stride)
  {
    pel* p_dst_buf = p_dst;
    pel* t = p_buf;

    for (int x = 0; x < block_width; ++x, ++t, ++p_dst_buf)
    {
      pel res = (-t[-tmp_buf_stride] + (t[0] * 10) - t[tmp_buf_stride] + offsets[3]) >> shifts[3];

      *p_dst_buf = min((1 << bit_depth) - 1, max(0, res));
    }
  }
}

void evc_eif_bilinear_clip(int block_width, int block_height, int mv0[MV_D], int d_x[MV_D], int d_y[MV_D], int mv_max[MV_D], int mv_min[MV_D], pel* p_ref, int ref_stride, pel* p_dst, int dst_stride, int shifts[4], int offsets[4])
{
  int mv[MV_D] = { mv0[MV_X], mv0[MV_Y] };

#if EIF_NEW_BILINEAR
  const pel fracMask = ( 1 << EIF_MV_PRECISION_BILINEAR ) - 1;
#else
  const char mv_precision = EIF_MV_PRECISION_INTERNAL;
  const pel one = 1 << EIF_MV_PRECISION_INTERNAL;
  const pel fracMask = one - 1;

  int a[2][2] = { { 0, 0 },{ 0, 0 }, };
#endif

  pel* p_buf = p_dst;

  int tmp_mv_for_line[MV_D] = { mv0[MV_X] - d_x[MV_X] - d_y[MV_X], mv0[MV_Y] - d_x[MV_Y] - d_y[MV_Y] }; //set to pos (-1, -1)

  for (int y = -1; y <= block_height; ++y, p_buf += dst_stride, tmp_mv_for_line[MV_X] += d_y[MV_X], tmp_mv_for_line[MV_Y] += d_y[MV_Y])
  {
    int tmp_mv[MV_D] = { tmp_mv_for_line[MV_X], tmp_mv_for_line[MV_Y] };

    for (int x = -1; x <= block_width; ++x, tmp_mv[MV_X] += d_x[MV_X], tmp_mv[MV_Y] += d_x[MV_Y])
    {
      mv[MV_X] = min(mv_max[MV_X], max(mv_min[MV_X], tmp_mv[MV_X]));
      mv[MV_Y] = min(mv_max[MV_Y], max(mv_min[MV_Y], tmp_mv[MV_Y]));

#if EIF_NEW_BILINEAR
      mv[MV_X] >>= ( EIF_MV_PRECISION_INTERNAL - EIF_MV_PRECISION_BILINEAR );
      mv[MV_Y] >>= ( EIF_MV_PRECISION_INTERNAL - EIF_MV_PRECISION_BILINEAR );

      int xInt = x + ( mv[MV_X] >> EIF_MV_PRECISION_BILINEAR );
      int yInt = y + ( mv[MV_Y] >> EIF_MV_PRECISION_BILINEAR );
#else
      int xInt = x + ( mv[MV_X] >> EIF_MV_PRECISION_INTERNAL );
      int yInt = y + ( mv[MV_Y] >> EIF_MV_PRECISION_INTERNAL );
#endif

      pel xFrac = mv[MV_X] & fracMask;
      pel yFrac = mv[MV_Y] & fracMask;

      pel* r = p_ref + yInt * ref_stride + xInt;

#if EIF_NEW_BILINEAR

#if EIF_MV_PRECISION_BILINEAR == 4
      pel s1 = MAC_BL_NN_S1( tbl_bl_mc_l_coeff[xFrac], r[0], r[1] );
      pel s2 = MAC_BL_NN_S1( tbl_bl_mc_l_coeff[xFrac], r[ref_stride], r[ref_stride + 1] );

      p_buf[x + 1] = MAC_BL_NN_S2( tbl_bl_mc_l_coeff[yFrac], s1, s2 );
#elif  EIF_MV_PRECISION_BILINEAR == 5
      pel s1 = MAC_BL_NN_S1( tbl_bl_eif_32_phases_mc_l_coeff[xFrac], r[0], r[1] );
      pel s2 = MAC_BL_NN_S1( tbl_bl_eif_32_phases_mc_l_coeff[xFrac], r[ref_stride], r[ref_stride + 1] );

      p_buf[x + 1] = MAC_BL_NN_S2( tbl_bl_eif_32_phases_mc_l_coeff[yFrac], s1, s2 );
#else
      pel tmpPel = r[0] - r[1] - r[ref_stride] + r[ref_stride + 1];
      tmpPel = ( tmpPel * yFrac + ( ( r[1] - r[0] ) << EIF_MV_PRECISION_BILINEAR ) + offsets[0] ) >> shifts[0];

      pel tmpPel2 = r[ref_stride] - r[0];
      double_pel tmpDoublePel = tmpPel * xFrac + ( ( tmpPel2 * yFrac ) << ( EIF_MV_PRECISION_BILINEAR - shifts[0] ) ) + ( r[0] << ( 2 * EIF_MV_PRECISION_BILINEAR - shifts[0] ) );

      p_buf[x + 1] = ( tmpDoublePel + offsets[1] ) >> shifts[1];
#endif

#else

      pel tmp = (r[0] * (one - xFrac) + offsets[0]) >> shifts[0];
      a[0][0] = tmp * (one - yFrac);

      tmp = (r[1] * xFrac + offsets[0]) >> shifts[0];
      a[0][1] = tmp * (one - yFrac);

      tmp = (r[0 + ref_stride] * (one - xFrac) + offsets[0]) >> shifts[0];
      a[1][0] = tmp * yFrac;

      tmp = (r[1 + ref_stride] * xFrac + offsets[0]) >> shifts[0];
      a[1][1] = tmp * yFrac;

      pel b = (a[0][0] + a[0][1] + a[1][0] + a[1][1] + offsets[1]) >> shifts[1];

      p_buf[x + 1] = b;
#endif
    }
  }
}

void evc_eif_bilinear_no_clip(int block_width, int block_height, int mv0[MV_D], int d_x[MV_D], int d_y[MV_D], pel* p_ref, int ref_stride, pel* p_dst, int dst_stride, int shifts[4], int offsets[4])
{
#if EIF_NEW_BILINEAR
  int mv[MV_D] = { mv0[MV_X], mv0[MV_Y] };

  const pel fracMask = (1 << EIF_MV_PRECISION_BILINEAR) - 1;
#else
  const char mv_precision = EIF_MV_PRECISION_INTERNAL;
  const pel one = 1 << EIF_MV_PRECISION_INTERNAL;
  const pel fracMask = one - 1;

  int a[2][2] = { { 0, 0 },{ 0, 0 }, };
#endif

  pel* p_buf = p_dst;

  int tmp_mv_for_line[MV_D] = { mv0[MV_X] - d_x[MV_X] - d_y[MV_X], mv0[MV_Y] - d_x[MV_Y] - d_y[MV_Y] }; //set to pos (-1, -1)

  for (int y = -1; y <= block_height; ++y, p_buf += dst_stride, tmp_mv_for_line[MV_X] += d_y[MV_X], tmp_mv_for_line[MV_Y] += d_y[MV_Y])
  {
    int tmp_mv[MV_D] = { tmp_mv_for_line[MV_X], tmp_mv_for_line[MV_Y] };

    for (int x = -1; x <= block_width; ++x, tmp_mv[MV_X] += d_x[MV_X], tmp_mv[MV_Y] += d_x[MV_Y])
    {

#if EIF_NEW_BILINEAR
      mv[MV_X] = tmp_mv[MV_X] >> ( EIF_MV_PRECISION_INTERNAL - EIF_MV_PRECISION_BILINEAR );
      mv[MV_Y] = tmp_mv[MV_Y] >> ( EIF_MV_PRECISION_INTERNAL - EIF_MV_PRECISION_BILINEAR );

      int xInt = x + ( mv[MV_X] >> EIF_MV_PRECISION_BILINEAR );
      int yInt = y + ( mv[MV_Y] >> EIF_MV_PRECISION_BILINEAR );

      pel xFrac = mv[MV_X] & fracMask;
      pel yFrac = mv[MV_Y] & fracMask;
#else
      int xInt = x + (tmp_mv[MV_X] >> EIF_MV_PRECISION_INTERNAL);
      int yInt = y + (tmp_mv[MV_Y] >> EIF_MV_PRECISION_INTERNAL);

      pel xFrac = tmp_mv[MV_X] & fracMask;
      pel yFrac = tmp_mv[MV_Y] & fracMask;
#endif

      pel* r = p_ref + yInt * ref_stride + xInt;

#if EIF_NEW_BILINEAR

#if EIF_MV_PRECISION_BILINEAR == 4
      pel s1 = MAC_BL_NN_S1( tbl_bl_mc_l_coeff[xFrac], r[0], r[1] );
      pel s2 = MAC_BL_NN_S1( tbl_bl_mc_l_coeff[xFrac], r[ref_stride], r[ref_stride + 1] );

      p_buf[x + 1] = MAC_BL_NN_S2( tbl_bl_mc_l_coeff[yFrac], s1, s2 );
#elif  EIF_MV_PRECISION_BILINEAR == 5
      pel s1 = MAC_BL_NN_S1( tbl_bl_eif_32_phases_mc_l_coeff[xFrac], r[0], r[1] );
      pel s2 = MAC_BL_NN_S1( tbl_bl_eif_32_phases_mc_l_coeff[xFrac], r[ref_stride], r[ref_stride + 1] );

      p_buf[x + 1] = MAC_BL_NN_S2( tbl_bl_eif_32_phases_mc_l_coeff[yFrac], s1, s2 );
#else
      pel tmpPel = r[0] - r[1] - r[ref_stride] + r[ref_stride + 1];
      tmpPel = ( tmpPel * yFrac + ( ( r[1] - r[0] ) << EIF_MV_PRECISION_BILINEAR ) + offsets[0] ) >> shifts[0];

      pel tmpPel2 = r[ref_stride] - r[0];
      double_pel tmpDoublePel = tmpPel * xFrac + ( ( tmpPel2 * yFrac ) << ( EIF_MV_PRECISION_BILINEAR - shifts[0] ) ) + ( r[0] << ( 2 * EIF_MV_PRECISION_BILINEAR - shifts[0] ) );

      p_buf[x + 1] = ( tmpDoublePel + offsets[1] ) >> shifts[1];
#endif

#else
      pel tmp = (r[0] * (one - xFrac) + offsets[0]) >> shifts[0];
      a[0][0] = tmp * (one - yFrac);

      tmp = (r[1] * xFrac + offsets[0]) >> shifts[0];
      a[0][1] = tmp * (one - yFrac);

      tmp = (r[0 + ref_stride] * (one - xFrac) + offsets[0]) >> shifts[0];
      a[1][0] = tmp * yFrac;

      tmp = (r[1 + ref_stride] * xFrac + offsets[0]) >> shifts[0];
      a[1][1] = tmp * yFrac;

      pel b = (a[0][0] + a[0][1] + a[1][0] + a[1][1] + offsets[1]) >> shifts[1];

      p_buf[x + 1] = b;
#endif
    }
  }
}

void evc_eif_mc(int block_width, int block_height, int x, int y, int mv_scale_hor, int mv_scale_ver, int dmv_hor_x, int dmv_hor_y, int dmv_ver_x, int dmv_ver_y,
                int hor_max, int ver_max, int hor_min, int ver_min, pel* p_ref, int ref_stride, pel *p_dst, int dst_stride, pel* p_tmp_buf, char affine_mv_prec, s8 comp)
{
  assert(EIF_MV_PRECISION_INTERNAL >= affine_mv_prec);  //For current affine internal MV precision is (2 + bit) bits; 2 means qpel
  assert(EIF_MV_PRECISION_INTERNAL >= 2 + MC_PRECISION_ADD);  //For current affine internal MV precision is (2 + bit) bits; 2 means qpel

  int mv0[MV_D] = { mv_scale_hor << ( EIF_MV_PRECISION_INTERNAL - affine_mv_prec ),
                    mv_scale_ver << ( EIF_MV_PRECISION_INTERNAL - affine_mv_prec ) };
  int d_x[MV_D] = { dmv_hor_x    << ( EIF_MV_PRECISION_INTERNAL - affine_mv_prec ),
                    dmv_hor_y    << ( EIF_MV_PRECISION_INTERNAL - affine_mv_prec ) };
  int d_y[MV_D] = { dmv_ver_x    << ( EIF_MV_PRECISION_INTERNAL - affine_mv_prec ),
                    dmv_ver_y    << ( EIF_MV_PRECISION_INTERNAL - affine_mv_prec ) };
  int mv_max[MV_D] = { hor_max   << ( EIF_MV_PRECISION_INTERNAL - (2 + MC_PRECISION_ADD)),
                       ver_max   << ( EIF_MV_PRECISION_INTERNAL - (2 + MC_PRECISION_ADD)) };
  int mv_min[MV_D] = { hor_min   << ( EIF_MV_PRECISION_INTERNAL - (2 + MC_PRECISION_ADD)),
                         ver_min << ( EIF_MV_PRECISION_INTERNAL - (2 + MC_PRECISION_ADD)) };
  
  if (comp > Y_C)
  {
    mv0[MV_X] >>= 1;    mv0[MV_Y] >>= 1;
    mv_max[MV_X] >>= 1; mv_max[MV_Y] >>= 1;
    mv_min[MV_X] >>= 1; mv_min[MV_Y] >>= 1;
    block_width >>= 1;
    block_height >>= 1;
    x >>= 1;
    y >>= 1;
  }

  p_ref += ref_stride * y + x;

  const int tmp_buf_stride = MAX_CU_SIZE + 2;
  int bit_depth = 10;

  assert(bit_depth < 16);

#if EIF_NEW_BILINEAR

#if EIF_MV_PRECISION_BILINEAR == 4 || EIF_MV_PRECISION_BILINEAR == 5
  int shifts[4] = { 0, 0, max( bit_depth + 5 - 16, 0 ), 6 - max( bit_depth + 5 - 16, 0 ) };
#else
  int shifts[4] = { bit_depth + EIF_MV_PRECISION_BILINEAR - 13, EIF_MV_PRECISION_BILINEAR + 1, 4, 14 - bit_depth }; //4 -- number of bits in 10 ; all pels are positive after bilinear interpolation
#endif

#else
  int shifts[4] = { bit_depth + EIF_MV_PRECISION_INTERNAL - 15, EIF_MV_PRECISION_INTERNAL + 2, 4, 15 - bit_depth }; //4 -- number of bits in 10 ; all pels are positive after bilinear interpolation
#endif
  int offsets[4] = { 0, 0, 0, 0 };

  for (int i = 0; i < 4; ++i)
    offsets[i] = 1 << (shifts[i] - 1);

  BOOL is_mv_clip_needed = can_mv_clipping_occurs(block_width, block_height, mv0, d_x, d_y, mv_max, mv_min);

  if (is_mv_clip_needed)
    evc_eif_bilinear_clip(block_width, block_height, mv0, d_x, d_y, mv_max, mv_min, p_ref, ref_stride, p_tmp_buf, tmp_buf_stride, shifts, offsets);
  else
    evc_eif_bilinear_no_clip(block_width, block_height, mv0, d_x, d_y, p_ref, ref_stride, p_tmp_buf, tmp_buf_stride, shifts, offsets);

  evc_eif_filter(block_width, block_height, p_tmp_buf, tmp_buf_stride, p_dst, dst_stride, shifts, offsets, bit_depth);
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

#if M50761_AFFINE_ADAPT_SUB_SIZE
    // derive sub-block size
    int sub_w = 4, sub_h = 4;
    derive_affine_subblock_size_bi( mv, refi, w, h, &sub_w, &sub_h, vertex_num );
#endif

    if(REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        evc_affine_mc_lc(x, y, pic_w, pic_h, w, h, mv[REFP_0], ref_pic, pred[0], vertex_num
#if M50761_AFFINE_ADAPT_SUB_SIZE
                          , sub_w, sub_h
#endif
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
#if M50761_AFFINE_ADAPT_SUB_SIZE
                          , sub_w, sub_h
#endif
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
