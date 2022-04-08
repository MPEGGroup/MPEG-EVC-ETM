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

#include "evce_def.h"
#include <math.h>

/* SAD for 16bit **************************************************************/
static int sad_16b(int w, int h, void *src1, void *src2, int s_src1, int s_src2, int bit_depth)
{
    s16 *s1;
    s16 *s2;

    int i, j, sad;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sad = 0;

    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            sad += EVC_ABS16((s16)s1[j] - (s16)s2[j]);
        }
        s1 += s_src1;
        s2 += s_src2;
    }

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

#if X86_SSE
#define SSE_SAD_16B_4PEL(src1, src2, s00, s01, sac0) \
    s00 = _mm_loadl_epi64((__m128i*)(src1)); \
    s01 = _mm_loadl_epi64((__m128i*)(src2));\
    s00 = _mm_sub_epi16(s00, s01); \
    s00 = _mm_abs_epi16(s00); \
    s00 = _mm_cvtepi16_epi32(s00); \
    \
    sac0 = _mm_add_epi32(sac0, s00);

#define SSE_SAD_16B_8PEL(src1, src2, s00, s01, sac0, sac1) \
    s00 = _mm_loadu_si128((__m128i*)(src1)); \
    s01 = _mm_loadu_si128((__m128i*)(src2)); \
    s00 = _mm_sub_epi16(s00, s01); \
    s01 = _mm_abs_epi16(s00); \
    \
    s00 = _mm_srli_si128(s01, 8); \
    s00 = _mm_cvtepi16_epi32(s00); \
    s01 = _mm_cvtepi16_epi32(s01); \
    \
    sac0 = _mm_add_epi32(sac0, s00); \
    sac1 = _mm_add_epi32(sac1, s01);

static int sad_16b_sse_4x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sac0 = _mm_setzero_si128();

    SSE_SAD_16B_4PEL(s1, s2, s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0);

    sad = _mm_extract_epi32(sac0, 0);
    sad += _mm_extract_epi32(sac0, 1);
    sad += _mm_extract_epi32(sac0, 2);
    sad += _mm_extract_epi32(sac0, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

static int sad_16b_sse_4x2n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0;
    int i;
    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sac0 = _mm_setzero_si128();

    for(i = 0; i < h >> 1; i++)
    {
        SSE_SAD_16B_4PEL(s1, s2, s00, s01, sac0);
        SSE_SAD_16B_4PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0);
        s1 += s_src1 << 1;
        s2 += s_src2 << 1;
    }

    sad = _mm_extract_epi32(sac0, 0);
    sad += _mm_extract_epi32(sac0, 1);
    sad += _mm_extract_epi32(sac0, 2);
    sad += _mm_extract_epi32(sac0, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

static int sad_16b_sse_4x4(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0;

    s1  = (s16 *)src1;
    s2  = (s16 *)src2;

    sac0 = _mm_setzero_si128();

    SSE_SAD_16B_4PEL(s1, s2, s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + (s_src1*2), s2 + (s_src2*2), s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + (s_src1*3), s2 + (s_src2*3), s00, s01, sac0);

    sad  = _mm_extract_epi32(sac0, 0);
    sad += _mm_extract_epi32(sac0, 1);
    sad += _mm_extract_epi32(sac0, 2);
    sad += _mm_extract_epi32(sac0, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

#if !OPT_SIMD_SAD
static int sad_16b_sse_8x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;
    sad = 0;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);

    s00 = _mm_add_epi32(sac0, sac1);

    sad += _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_8x4(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;
    sad = 0;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 2), s2 + (s_src2 * 2), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 3), s2 + (s_src2 * 3), s00, s01, sac0, sac1);

    s00 = _mm_add_epi32(sac0, sac1);

    sad += _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_8x4n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int i, sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;
    sad = 0;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    for(i = 0; i < h >> 2; i++)
    {
        SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
        SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
        SSE_SAD_16B_8PEL(s1 + (s_src1 * 2), s2 + (s_src2 * 2), s00, s01, sac0, sac1);
        SSE_SAD_16B_8PEL(s1 + (s_src1 * 3), s2 + (s_src2 * 3), s00, s01, sac0, sac1);
        s1 += s_src1 << 2;
        s2 += s_src2 << 2;
    }

    s00 = _mm_add_epi32(sac0, sac1);

    sad += _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_8x8(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2    = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1*2), s2 + (s_src2*2), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1*3), s2 + (s_src2*3), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1*4), s2 + (s_src2*4), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1*5), s2 + (s_src2*5), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1*6), s2 + (s_src2*6), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1*7), s2 + (s_src2*7), s00, s01, sac0, sac1);

    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

static int sad_16b_sse_16x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + 8, s2 + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1 + 8, s2 + s_src2 + 8, s00, s01, sac0, sac1);

    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

#if !OPT_SIMD_SAD
static int sad_16b_sse_16x4(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;

    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + 8, s2 + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1 + 8, s2 + s_src2 + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 2), s2 + (s_src2 * 2), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 2) + 8, s2 + (s_src2 * 2) + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 3), s2 + (s_src2 * 3), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 3) + 8, s2 + (s_src2 * 3) + 8, s00, s01, sac0, sac1);

    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_16x8(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + 8, s2 + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1 + 8, s2 + s_src2 + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 2), s2 + (s_src2 * 2), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 2) + 8, s2 + (s_src2 * 2) + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 3), s2 + (s_src2 * 3), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 3) + 8, s2 + (s_src2 * 3) + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 4), s2 + (s_src2 * 4), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 4) + 8, s2 + (s_src2 * 4) + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 5), s2 + (s_src2 * 5), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 5) + 8, s2 + (s_src2 * 5) + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 6), s2 + (s_src2 * 6), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 6) + 8, s2 + (s_src2 * 6) + 8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + (s_src1 * 7), s2 + (s_src2 * 7), s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + (s_src1 * 7) + 8, s2 + (s_src2 * 7) + 8, s00, s01, sac0, sac1);

    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_16x16(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16 * s1;
    s16 * s2;

    __m128i s00, s01, sac0, sac1;

    s1  = (s16 *)src1;
    s2  = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    SSE_SAD_16B_8PEL(s1,               s2,               s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+8,             s2+8,             s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1 + s_src1,      s2 + s_src2,      s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1 + s_src1+8,    s2 + s_src2+8,    s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*2),    s2+(s_src2*2),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*2)+8,  s2+(s_src2*2)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*3),    s2+(s_src2*3),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*3)+8,  s2+(s_src2*3)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*4),    s2+(s_src2*4),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*4)+8,  s2+(s_src2*4)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*5),    s2+(s_src2*5),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*5)+8,  s2+(s_src2*5)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*6),    s2+(s_src2*6),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*6)+8,  s2+(s_src2*6)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*7),    s2+(s_src2*7),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*7)+8,  s2+(s_src2*7)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*8),    s2+(s_src2*8),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*8)+8,  s2+(s_src2*8)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*9),    s2+(s_src2*9),    s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*9)+8,  s2+(s_src2*9)+8,  s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*10),   s2+(s_src2*10),   s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*10)+8, s2+(s_src2*10)+8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*11),   s2+(s_src2*11),   s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*11)+8, s2+(s_src2*11)+8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*12),   s2+(s_src2*12),   s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*12)+8, s2+(s_src2*12)+8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*13),   s2+(s_src2*13),   s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*13)+8, s2+(s_src2*13)+8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*14),   s2+(s_src2*14),   s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*14)+8, s2+(s_src2*14)+8, s00, s01, sac0, sac1);

    SSE_SAD_16B_8PEL(s1+(s_src1*15),   s2+(s_src2*15),   s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL(s1+(s_src1*15)+8, s2+(s_src2*15)+8, s00, s01, sac0, sac1);


    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_16nx4n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    int i, j;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    for(i = 0; i<(h >> 2); i++)
    {
        for(j = 0; j<(w >> 4); j++)
        {
            SSE_SAD_16B_8PEL(s1, s2, s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1 + 8, s2 + 8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1 + s_src1 + 8, s2 + s_src2 + 8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1 + (s_src1 * 2), s2 + (s_src2 * 2), s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1 + (s_src1 * 2) + 8, s2 + (s_src2 * 2) + 8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1 + (s_src1 * 3), s2 + (s_src2 * 3), s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1 + (s_src1 * 3) + 8, s2 + (s_src2 * 3) + 8, s00, s01, sac0, sac1);

            s1 += 16;
            s2 += 16;
        }
        s1 += (s_src1 << 2) - ((w >> 4) << 4);
        s2 += (s_src2 << 2) - ((w >> 4) << 4);
    }

    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if !OPT_SIMD_SAD
static int sad_16b_sse_16nx16n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    int i, j;
    s16 * s1;
    s16 * s2;
    __m128i s00, s01, sac0, sac1;

    s1  = (s16 *)src1;
    s2  = (s16 *)src2;

    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();

    for(i=0; i<(h>>4); i++)
    {
        for(j=0; j<(w>>4); j++)
        {
            SSE_SAD_16B_8PEL(s1,   s2,   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+8, s2+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1 + s_src1,   s2 + s_src2,   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1 + s_src1+8, s2 + s_src2+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*2),   s2+(s_src2*2),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*2)+8, s2+(s_src2*2)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*3),   s2+(s_src2*3),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*3)+8, s2+(s_src2*3)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*4),   s2+(s_src2*4),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*4)+8, s2+(s_src2*4)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*5),   s2+(s_src2*5),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*5)+8, s2+(s_src2*5)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*6),   s2+(s_src2*6),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*6)+8, s2+(s_src2*6)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*7),   s2+(s_src2*7),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*7)+8, s2+(s_src2*7)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*8),   s2+(s_src2*8),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*8)+8, s2+(s_src2*8)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*9),   s2+(s_src2*9),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*9)+8, s2+(s_src2*9)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*10),   s2+(s_src2*10),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*10)+8, s2+(s_src2*10)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*11),   s2+(s_src2*11),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*11)+8, s2+(s_src2*11)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*12),   s2+(s_src2*12),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*12)+8, s2+(s_src2*12)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*13),   s2+(s_src2*13),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*13)+8, s2+(s_src2*13)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*14),   s2+(s_src2*14),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*14)+8, s2+(s_src2*14)+8, s00, s01, sac0, sac1);

            SSE_SAD_16B_8PEL(s1+(s_src1*15),   s2+(s_src2*15),   s00, s01, sac0, sac1);
            SSE_SAD_16B_8PEL(s1+(s_src1*15)+8, s2+(s_src2*15)+8, s00, s01, sac0, sac1);

            s1 += 16;
            s2 += 16;
        }
        s1 += (s_src1 << 4) - ((w >> 4) << 4);
        s2 += (s_src2 << 4) - ((w >> 4) << 4);
    }

    s00 = _mm_add_epi32(sac0, sac1);

    sad  = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif

#if OPT_SIMD_SAD
static int sad_16b_sse_8x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    __m128i src_8x16b;
    __m128i src_8x16b_1;

    __m128i pred_8x16b;
    __m128i pred_8x16b_1;

    __m128i temp;
    __m128i temp_1;
    __m128i temp_3;

    __m128i temp_dummy;
    __m128i result;

    short *pu2_inp;
    short *pu2_ref;

    int sad = 0;

    assert(bit_depth <= 14);
    assert(w == 8); /* fun usage expects w ==8, but assumption is width has to be multiple of 8 */
    assert(h == 2); /* fun usage expects h ==2, but assumption is height has to be multiple of 2 */

    pu2_inp = src1;
    pu2_ref = src2;

    temp_dummy = _mm_setzero_si128();
    result = _mm_setzero_si128();

    {
        src_8x16b = _mm_loadu_si128((__m128i *) (pu2_inp));
        src_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_inp + s_src1));

        pred_8x16b = _mm_loadu_si128((__m128i *) (pu2_ref));
        pred_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_ref + s_src2));

        temp = _mm_sub_epi16(src_8x16b, pred_8x16b);
        temp_1 = _mm_sub_epi16(src_8x16b_1, pred_8x16b_1);

        temp = _mm_abs_epi16(temp);
        temp_1 = _mm_abs_epi16(temp_1);

        temp = _mm_adds_epu16(temp, temp_1);

        temp_1 = _mm_unpackhi_epi16(temp, temp_dummy);
        temp_3 = _mm_unpacklo_epi16(temp, temp_dummy);

        temp = _mm_add_epi32(temp_1, temp_3);

        result = _mm_add_epi32(result, temp);

    }

    {
        int *val = (int*)&result;
        sad = val[0] + val[1] + val[2] + val[3];
    }

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

static int sad_16b_sse_8x4n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    __m128i src_8x16b;
    __m128i src_8x16b_1;
    __m128i src_8x16b_2;
    __m128i src_8x16b_3;

    __m128i pred_8x16b;
    __m128i pred_8x16b_1;
    __m128i pred_8x16b_2;
    __m128i pred_8x16b_3;

    __m128i temp;
    __m128i temp_1;
    __m128i temp_2;
    __m128i temp_3;
    __m128i temp_4;
    __m128i temp_5;

    __m128i temp_dummy;
    __m128i result, result_1;

    short *pu2_inp;
    short *pu2_ref;

    int  i;
    int sad = 0;

    assert(bit_depth <= 14);
    assert(w == 8); /* fun usage expects w ==8, but assumption is width has to be multiple of 8 */
    assert(!(h & 3)); /* height has to be multiple of 4 */

    pu2_inp = src1;
    pu2_ref = src2;

    temp_dummy = _mm_setzero_si128();
    result = _mm_setzero_si128();
    result_1 = _mm_setzero_si128();

    {
        for (i = 0; i < h / 4; i++)
        {
            src_8x16b = _mm_loadu_si128((__m128i *) (pu2_inp));
            src_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_inp + s_src1));
            src_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_inp + (s_src1 * 2)));
            src_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_inp + (s_src1 * 3)));

            pred_8x16b = _mm_loadu_si128((__m128i *) (pu2_ref));
            pred_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_ref + s_src2));
            pred_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_ref + (s_src2 * 2)));
            pred_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_ref + (s_src2 * 3)));

            temp = _mm_sub_epi16(src_8x16b, pred_8x16b);
            temp_1 = _mm_sub_epi16(src_8x16b_1, pred_8x16b_1);
            temp_2 = _mm_sub_epi16(src_8x16b_2, pred_8x16b_2);
            temp_3 = _mm_sub_epi16(src_8x16b_3, pred_8x16b_3);

            temp = _mm_abs_epi16(temp);
            temp_1 = _mm_abs_epi16(temp_1);
            temp_2 = _mm_abs_epi16(temp_2);
            temp_3 = _mm_abs_epi16(temp_3);

            temp = _mm_add_epi16(temp, temp_1);
            temp_2 = _mm_add_epi16(temp_2, temp_3);

            temp_1 = _mm_unpackhi_epi16(temp, temp_dummy);
            temp_3 = _mm_unpacklo_epi16(temp, temp_dummy);
            temp_4 = _mm_unpackhi_epi16(temp_2, temp_dummy);
            temp_5 = _mm_unpacklo_epi16(temp_2, temp_dummy);

            temp = _mm_add_epi32(temp_1, temp_3);
            temp_2 = _mm_add_epi32(temp_4, temp_5);

            result = _mm_add_epi32(result, temp);
            result_1 = _mm_add_epi32(result_1, temp_2);

            pu2_inp += (4 * s_src1);
            pu2_ref += (4 * s_src2);
        }
        result = _mm_add_epi32(result, result_1);
    }

    {
        int *val = (int*)&result;
        sad = val[0] + val[1] + val[2] + val[3];
    }
#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}

static int sad_16b_sse_16nx4n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    __m128i src_8x16b;
    __m128i src_8x16b_1;
    __m128i src_8x16b_2;
    __m128i src_8x16b_3;

    __m128i pred_8x16b;
    __m128i pred_8x16b_1;
    __m128i pred_8x16b_2;
    __m128i pred_8x16b_3;

    __m128i temp;
    __m128i temp_1;
    __m128i temp_2;
    __m128i temp_3;
    __m128i temp_4;
    __m128i temp_5;

    __m128i temp_dummy;
    __m128i result, result_1;

    short *pu2_inp;
    short *pu2_ref;

    int  i, j;
    int sad = 0;

    assert(bit_depth <= 14);
    assert(!(w & 15)); /*fun used only for multiple of 16, but internal assumption is only 8 */
    assert(!(h & 3)); /* height has to be multiple of 4 */

    pu2_inp = src1;
    pu2_ref = src2;

    temp_dummy = _mm_setzero_si128();
    result = _mm_setzero_si128();
    result_1 = _mm_setzero_si128();

    {
        for (i = 0; i < h / 4; i++)
          {
            int count = 0;

            for (j = w; j > 7; j -= 8)
            {
                src_8x16b = _mm_loadu_si128((__m128i *) (pu2_inp + count));
                src_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_inp + count + s_src1));
                src_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_inp + count + (s_src1 * 2)));
                src_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_inp + count + (s_src1 * 3)));

                pred_8x16b = _mm_loadu_si128((__m128i *) (pu2_ref + count));
                pred_8x16b_1 = _mm_loadu_si128((__m128i *) (pu2_ref + count + s_src2));
                pred_8x16b_2 = _mm_loadu_si128((__m128i *) (pu2_ref + count + (s_src2 * 2)));
                pred_8x16b_3 = _mm_loadu_si128((__m128i *) (pu2_ref + count + (s_src2 * 3)));

                temp = _mm_sub_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_sub_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_sub_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_sub_epi16(src_8x16b_3, pred_8x16b_3);

                temp = _mm_abs_epi16(temp);
                temp_1 = _mm_abs_epi16(temp_1);
                temp_2 = _mm_abs_epi16(temp_2);
                temp_3 = _mm_abs_epi16(temp_3);

                temp = _mm_add_epi16(temp, temp_1);
                temp_2 = _mm_add_epi16(temp_2, temp_3);

                temp_1 = _mm_unpackhi_epi16(temp, temp_dummy);
                temp_3 = _mm_unpacklo_epi16(temp, temp_dummy);
                temp_4 = _mm_unpackhi_epi16(temp_2, temp_dummy);
                temp_5 = _mm_unpacklo_epi16(temp_2, temp_dummy);

                temp = _mm_add_epi32(temp_1, temp_3);
                temp_2 = _mm_add_epi32(temp_4, temp_5);

                result = _mm_add_epi32(result, temp);
                result_1 = _mm_add_epi32(result_1, temp_2);

                count += 8;
            }

            pu2_inp += (4 * s_src1);
            pu2_ref += (4 * s_src2);
        }
        result = _mm_add_epi32(result, result_1);
    }

    {
        int *val = (int*)&result;
        sad = val[0] + val[1] + val[2] + val[3];
    }

#if FULL_BITDEPTH_RDO
    return (sad);
#else
    return (sad >> (bit_depth - 8));
#endif
}
#endif
#endif /* X86_SSE */

/* index: [log2 of width][log2 of height] */
const EVCE_FN_SAD evce_tbl_sad_16b[8][8] =
{
#if X86_SSE
    /* width == 1 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        sad_16b, /* height == 1 */
        sad_16b_sse_4x2,  /* height == 2 */
        sad_16b_sse_4x4,  /* height == 4 */
        sad_16b_sse_4x2n, /* height == 8 */
        sad_16b_sse_4x2n, /* height == 16 */
        sad_16b_sse_4x2n, /* height == 32 */
        sad_16b_sse_4x2n, /* height == 64 */
        sad_16b_sse_4x2n, /* height == 128 */
    },
    /* width == 8 */
    {
        sad_16b, /* height == 1 */
        sad_16b_sse_8x2,  /* height == 2 */
#if OPT_SIMD_SAD
        sad_16b_sse_8x4n,  /* height == 4 */
        sad_16b_sse_8x4n,  /* height == 8 */
#else
        sad_16b_sse_8x4,  /* height == 4 */
        sad_16b_sse_8x8,  /* height == 8 */
#endif
        sad_16b_sse_8x4n, /* height == 16 */
        sad_16b_sse_8x4n, /* height == 32 */
        sad_16b_sse_8x4n, /* height == 64 */
        sad_16b_sse_8x4n, /* height == 128 */
    },
    /* width == 16 */
    {
        sad_16b,    /* height == 1 */
        sad_16b_sse_16x2,    /* height == 2 */
#if OPT_SIMD_SAD
        sad_16b_sse_16nx4n,  /* height == 4 */
        sad_16b_sse_16nx4n,  /* height == 8 */
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
#else
        sad_16b_sse_16x4,    /* height == 4 */
        sad_16b_sse_16x8,    /* height == 8 */
        sad_16b_sse_16x16,   /* height == 16 */
        sad_16b_sse_16nx16n, /* height == 32 */
        sad_16b_sse_16nx16n, /* height == 64 */
        sad_16b_sse_16nx16n, /* height == 128 */
#endif
    },
    /* width == 32 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b_sse_16nx4n,  /* height == 4 */
        sad_16b_sse_16nx4n,  /* height == 8 */
#if OPT_SIMD_SAD
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
#else
        sad_16b_sse_16nx16n, /* height == 16 */
        sad_16b_sse_16nx16n, /* height == 32 */
        sad_16b_sse_16nx16n, /* height == 64 */
        sad_16b_sse_16nx16n, /* height == 128 */
#endif
    },
    /* width == 64 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b_sse_16nx4n,  /* height == 4 */
        sad_16b_sse_16nx4n,  /* height == 8 */
#if OPT_SIMD_SAD
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
#else
        sad_16b_sse_16nx16n, /* height == 16 */
        sad_16b_sse_16nx16n, /* height == 32 */
        sad_16b_sse_16nx16n, /* height == 64 */
        sad_16b_sse_16nx16n, /* height == 128 */
#endif
    },
    /* width == 128 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b,    /* height == 4 */
        sad_16b,    /* height == 8 */
#if OPT_SIMD_SAD
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
#else
        sad_16b_sse_16nx16n, /* height == 16 */
        sad_16b_sse_16nx16n, /* height == 32 */
        sad_16b_sse_16nx16n, /* height == 64 */
        sad_16b_sse_16nx16n, /* height == 128 */
#endif
    }
#else /* default */
    /* width == 1 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 8 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 16 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 32 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 64 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 128 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    }
#endif
};


/* DIFF **********************************************************************/
static void diff_16b(int w, int h, void *src1, void *src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 *s1;
    s16 *s2;

    int i, j;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            diff[j] = (s16)s1[j] - (s16)s2[j];
        }
        diff += s_diff;
        s1 += s_src1;
        s2 += s_src2;
    }
}

#if X86_SSE
#define SSE_DIFF_16B_4PEL(src1, src2, diff, m00, m01, m02) \
    m00 = _mm_loadl_epi64((__m128i*)(src1)); \
    m01 = _mm_loadl_epi64((__m128i*)(src2)); \
    m02 = _mm_sub_epi16(m00, m01); \
    _mm_storel_epi64((__m128i*)(diff), m02);

#define SSE_DIFF_16B_8PEL(src1, src2, diff, m00, m01, m02) \
    m00 = _mm_loadu_si128((__m128i*)(src1)); \
    m01 = _mm_loadu_si128((__m128i*)(src2)); \
    m02 = _mm_sub_epi16(m00, m01); \
    _mm_storeu_si128((__m128i*)(diff), m02);

static void diff_16b_sse_4x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    __m128i m01, m02, m03, m04, m05, m06;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    SSE_DIFF_16B_4PEL(s1, s2, diff, m01, m02, m03);
    SSE_DIFF_16B_4PEL(s1 + s_src1, s2 + s_src2, diff + s_diff, m04, m05, m06);
}

static void diff_16b_sse_4x4(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    __m128i m01, m02, m03, m04, m05, m06, m07, m08, m09, m10, m11, m12;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    SSE_DIFF_16B_4PEL(s1, s2, diff, m01, m02, m03);
    SSE_DIFF_16B_4PEL(s1 + s_src1, s2 + s_src2, diff + s_diff, m04, m05, m06);
    SSE_DIFF_16B_4PEL(s1 + s_src1*2, s2 + s_src2*2, diff + s_diff*2, m07, m08, m09);
    SSE_DIFF_16B_4PEL(s1 + s_src1*3, s2 + s_src2*3, diff + s_diff*3, m10, m11, m12);
}

static void diff_16b_sse_8x8(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    __m128i m01, m02, m03, m04, m05, m06, m07, m08, m09, m10, m11, m12;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
    SSE_DIFF_16B_8PEL(s1 + s_src1, s2 + s_src2, diff + s_diff, m04, m05, m06);
    SSE_DIFF_16B_8PEL(s1 + s_src1*2, s2 + s_src2*2, diff + s_diff*2, m07, m08, m09);
    SSE_DIFF_16B_8PEL(s1 + s_src1*3, s2 + s_src2*3, diff + s_diff*3, m10, m11, m12);
    SSE_DIFF_16B_8PEL(s1 + s_src1*4, s2 + s_src2*4, diff + s_diff*4, m01, m02, m03);
    SSE_DIFF_16B_8PEL(s1 + s_src1*5, s2 + s_src2*5, diff + s_diff*5, m04, m05, m06);
    SSE_DIFF_16B_8PEL(s1 + s_src1*6, s2 + s_src2*6, diff + s_diff*6, m07, m08, m09);
    SSE_DIFF_16B_8PEL(s1 + s_src1*7, s2 + s_src2*7, diff + s_diff*7, m10, m11, m12);
}

static void diff_16b_sse_8nx2n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    int i, j;
    __m128i m01, m02, m03, m04, m05, m06;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i = 0; i < h >> 1; i++)
    {
        for(j = 0; j < (w >> 3); j++)
        {
            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1 + s_src1, s2 + s_src2, diff + s_diff, m04, m05, m06);
            s1 += 8; s2 += 8; diff += 8;
        }

        s1   += ((s_src1<<1) - ((w>>3)<<3));
        s2   += ((s_src2<<1) - ((w>>3)<<3));
        diff += ((s_diff<<1) - ((w>>3)<<3));
    }
}

static void diff_16b_sse_16nx2n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    int i, j;
    __m128i m01, m02, m03, m04, m05, m06;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i = 0; i < h>>1; i++)
    {
        for(j = 0; j < (w >> 4); j++)
        {
            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1 + s_src1, s2 + s_src2, diff + s_diff, m04, m05, m06);
            s1 += 8; s2 += 8; diff += 8;

            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1 + s_src1, s2 + s_src2, diff + s_diff, m04, m05, m06);
            s1 += 8; s2 += 8; diff += 8;
        }

        s2   += ((s_src2<<1) - ((w>>4)<<4));
        s1   += ((s_src1<<1) - ((w>>4)<<4));
        diff += ((s_diff<<1) - ((w>>4)<<4));
    }
}

static void diff_16b_sse_32nx4n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int s_diff, s16 * diff, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    int i, j;
    __m128i m01, m02, m03, m04, m05, m06, m07, m08, m09, m10, m11, m12;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i = 0; i < (h>>2); i++)
    {
        for(j = 0; j < (w>>5); j++)
        {
            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1+s_src1, s2+s_src2, diff+s_diff, m04, m05, m06);
            SSE_DIFF_16B_8PEL(s1+s_src1*2, s2+s_src2*2, diff+s_diff*2, m07, m08, m09);
            SSE_DIFF_16B_8PEL(s1+s_src1*3, s2+s_src2*3, diff+s_diff*3, m10, m11, m12);
            s1 += 8; s2 += 8; diff+= 8;

            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1+s_src1, s2+s_src2, diff+s_diff, m04, m05, m06);
            SSE_DIFF_16B_8PEL(s1+s_src1*2, s2+s_src2*2, diff+s_diff*2, m07, m08, m09);
            SSE_DIFF_16B_8PEL(s1+s_src1*3, s2+s_src2*3, diff+s_diff*3, m10, m11, m12);
            s1 += 8; s2 += 8; diff+= 8;

            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1+s_src1, s2+s_src2, diff+s_diff, m04, m05, m06);
            SSE_DIFF_16B_8PEL(s1+s_src1*2, s2+s_src2*2, diff+s_diff*2, m07, m08, m09);
            SSE_DIFF_16B_8PEL(s1+s_src1*3, s2+s_src2*3, diff+s_diff*3, m10, m11, m12);
            s1 += 8; s2 += 8; diff+= 8;

            SSE_DIFF_16B_8PEL(s1, s2, diff, m01, m02, m03);
            SSE_DIFF_16B_8PEL(s1+s_src1, s2+s_src2, diff+s_diff, m04, m05, m06);
            SSE_DIFF_16B_8PEL(s1+s_src1*2, s2+s_src2*2, diff+s_diff*2, m07, m08, m09);
            SSE_DIFF_16B_8PEL(s1+s_src1*3, s2+s_src2*3, diff+s_diff*3, m10, m11, m12);
            s1 += 8; s2 += 8; diff+= 8;
        }

        s1   += ((s_src1<<2) - ((w>>5)<<5));
        s2   += ((s_src2<<2) - ((w>>5)<<5));
        diff += ((s_diff<<2) - ((w>>5)<<5));
    }
}
#endif /* X86_SSE */

const EVCE_FN_DIFF evce_tbl_diff_16b[8][8] =
{
#if X86_SSE
    /* width == 1 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        diff_16b, /* height == 1 */
        diff_16b_sse_4x2,  /* height == 2 */
        diff_16b_sse_4x4,  /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 8 */
    {
        diff_16b,  /* height == 1 */
        diff_16b_sse_8nx2n, /* height == 2 */
        diff_16b_sse_8nx2n, /* height == 4 */
        diff_16b_sse_8x8,   /* height == 8 */
        diff_16b_sse_8nx2n, /* height == 16 */
        diff_16b_sse_8nx2n, /* height == 32 */
        diff_16b_sse_8nx2n, /* height == 64 */
        diff_16b_sse_8nx2n, /* height == 128 */
    },
    /* width == 16 */
    {
        diff_16b,   /* height == 1 */
        diff_16b_sse_16nx2n, /* height == 2 */
        diff_16b_sse_16nx2n, /* height == 4 */
        diff_16b_sse_16nx2n, /* height == 8 */
        diff_16b_sse_16nx2n, /* height == 16 */
        diff_16b_sse_16nx2n, /* height == 32 */
        diff_16b_sse_16nx2n, /* height == 64 */
        diff_16b_sse_16nx2n, /* height == 128 */
    },
    /* width == 32 */
    {
        diff_16b,   /* height == 1 */
        diff_16b_sse_16nx2n, /* height == 2 */
        diff_16b_sse_32nx4n, /* height == 4 */
        diff_16b_sse_32nx4n, /* height == 8 */
        diff_16b_sse_32nx4n, /* height == 16 */
        diff_16b_sse_32nx4n, /* height == 32 */
        diff_16b_sse_32nx4n, /* height == 64 */
        diff_16b_sse_32nx4n, /* height == 128 */
    },
    /* width == 64 */
    {
        diff_16b,   /* height == 1 */
        diff_16b_sse_16nx2n, /* height == 2 */
        diff_16b_sse_32nx4n, /* height == 4 */
        diff_16b_sse_32nx4n, /* height == 8 */
        diff_16b_sse_32nx4n, /* height == 16 */
        diff_16b_sse_32nx4n, /* height == 32 */
        diff_16b_sse_32nx4n, /* height == 64 */
        diff_16b_sse_32nx4n, /* height == 128 */
    },
    /* width == 128 */
    {
        diff_16b,   /* height == 1 */
        diff_16b_sse_16nx2n, /* height == 2 */
        diff_16b_sse_32nx4n, /* height == 4 */
        diff_16b_sse_32nx4n, /* height == 8 */
        diff_16b_sse_32nx4n, /* height == 16 */
        diff_16b_sse_32nx4n, /* height == 32 */
        diff_16b_sse_32nx4n, /* height == 64 */
        diff_16b_sse_32nx4n, /* height == 128 */
    }
#else
    /* width == 1 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 8 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 16 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 32 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 64 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    },
    /* width == 128 */
    {
        diff_16b, /* height == 1 */
        diff_16b, /* height == 2 */
        diff_16b, /* height == 4 */
        diff_16b, /* height == 8 */
        diff_16b, /* height == 16 */
        diff_16b, /* height == 32 */
        diff_16b, /* height == 64 */
        diff_16b, /* height == 128 */
    }
#endif
};

/* SSD ***********************************************************************/
static s64 ssd_16b(int w, int h, void *src1, void *src2, int s_src1, int s_src2, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    int     i, j, diff;
    s64   ssd;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    ssd = 0;

    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            diff = s1[j] - s2[j];
            ssd += (diff * diff) >> shift;
        }
        s1 += s_src1;
        s2 += s_src2;
    }
    return ssd;
}

#if X86_SSE
#define SSE_SSD_16B_4PEL(src1, src2, shift, s00, s01, s00a) \
    s00 = _mm_loadl_epi64((__m128i*)(src1)); \
    s01 = _mm_loadl_epi64((__m128i*)(src2));\
    s00 = _mm_sub_epi16(s00, s01); \
    s00 = _mm_cvtepi16_epi32(s00); \
    s01 = _mm_mullo_epi32(s00, s00); \
    s00 = _mm_srli_epi32(s01, shift); \
    s00a = _mm_add_epi32(s00a, s00);

#define SSE_SSD_16B_8PEL(src1, src2, shift, s00, s01, s02, s00a) \
    s00 = _mm_loadu_si128((__m128i*)(src1)); \
    s01 = _mm_loadu_si128((__m128i*)(src2)); \
    s02 = _mm_sub_epi16(s00, s01); \
    \
    s00 = _mm_cvtepi16_epi32(s02); \
    s00 = _mm_mullo_epi32(s00, s00); \
    \
    s01 = _mm_srli_si128(s02, 8); \
    s01 = _mm_cvtepi16_epi32(s01); \
    s01 = _mm_mullo_epi32(s01, s01); \
    \
    s00 = _mm_srli_epi32(s00, shift); \
    s01 = _mm_srli_epi32(s01, shift); \
    s00a = _mm_add_epi32(s00a, s00); \
    s00a = _mm_add_epi32(s00a, s01);

static s64 ssd_16b_sse_4x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif

    __m128i s00, s01, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_4PEL(s1, s2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_4x4(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif

    __m128i s00, s01, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_4PEL(s1, s2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1*2, s2 + s_src2*2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1*3, s2 + s_src2*3, shift, s00, s01, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_4x8(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_4PEL(s1, s2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 2, s2 + s_src2 * 2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 3, s2 + s_src2 * 3, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 4, s2 + s_src2 * 4, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 5, s2 + s_src2 * 5, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 6, s2 + s_src2 * 6, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 7, s2 + s_src2 * 7, shift, s00, s01, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_4x16(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif

    __m128i s00, s01, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_4PEL(s1, s2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  2, s2 + s_src2 *  2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  3, s2 + s_src2 *  3, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  4, s2 + s_src2 *  4, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  5, s2 + s_src2 *  5, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  6, s2 + s_src2 *  6, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  7, s2 + s_src2 *  7, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  8, s2 + s_src2 *  8, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 *  9, s2 + s_src2 *  9, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 10, s2 + s_src2 * 10, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 11, s2 + s_src2 * 11, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 12, s2 + s_src2 * 12, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 13, s2 + s_src2 * 13, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 14, s2 + s_src2 * 14, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 15, s2 + s_src2 * 15, shift, s00, s01, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_4x32(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_4PEL(s1, s2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 2, s2 + s_src2 * 2, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 3, s2 + s_src2 * 3, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 4, s2 + s_src2 * 4, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 5, s2 + s_src2 * 5, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 6, s2 + s_src2 * 6, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 7, s2 + s_src2 * 7, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 8, s2 + s_src2 * 8, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 9, s2 + s_src2 * 9, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 10, s2 + s_src2 * 10, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 11, s2 + s_src2 * 11, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 12, s2 + s_src2 * 12, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 13, s2 + s_src2 * 13, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 14, s2 + s_src2 * 14, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 15, s2 + s_src2 * 15, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 16, s2 + s_src2 * 16, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 17, s2 + s_src2 * 17, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 18, s2 + s_src2 * 18, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 19, s2 + s_src2 * 19, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 20, s2 + s_src2 * 20, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 21, s2 + s_src2 * 21, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 22, s2 + s_src2 * 22, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 23, s2 + s_src2 * 23, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 24, s2 + s_src2 * 24, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 25, s2 + s_src2 * 25, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 26, s2 + s_src2 * 26, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 27, s2 + s_src2 * 27, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 28, s2 + s_src2 * 28, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 29, s2 + s_src2 * 29, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 30, s2 + s_src2 * 30, shift, s00, s01, s00a);
    SSE_SSD_16B_4PEL(s1 + s_src1 * 31, s2 + s_src2 * 31, shift, s00, s01, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_8x2(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s02, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_8PEL(s1, s2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s02, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_8x4(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s02, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_8PEL(s1, s2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1 * 2, s2 + s_src2 * 2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1 * 3, s2 + s_src2 * 3, shift, s00, s01, s02, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_8x8(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd;
    s16 * s1;
    s16 * s2;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s02, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    s00a = _mm_setzero_si128();

    SSE_SSD_16B_8PEL(s1, s2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1*2, s2 + s_src2*2, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1*3, s2 + s_src2*3, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1*4, s2 + s_src2*4, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1*5, s2 + s_src2*5, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1*6, s2 + s_src2*6, shift, s00, s01, s02, s00a);
    SSE_SSD_16B_8PEL(s1 + s_src1*7, s2 + s_src2*7, shift, s00, s01, s02, s00a);

    ssd = _mm_extract_epi32(s00a, 0);
    ssd += _mm_extract_epi32(s00a, 1);
    ssd += _mm_extract_epi32(s00a, 2);
    ssd += _mm_extract_epi32(s00a, 3);

    return ssd;
}

static s64 ssd_16b_sse_8nx2n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd = 0;
    s16 * s1;
    s16 * s2;
    int     i, j;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif

    __m128i s00, s01, s02, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i = 0; i<(h >> 1); i++)
    {
        s00a = _mm_setzero_si128();
        for(j = 0; j<(w >> 3); j++)
        {
            SSE_SSD_16B_8PEL(s1, s2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s02, s00a);

            s1 += 8;
            s2 += 8;
        }
        s1 += (s_src1 << 1) - ((w >> 3) << 8);
        s2 += (s_src2 << 1) - ((w >> 3) << 8);

        ssd += _mm_extract_epi32(s00a, 0);
        ssd += _mm_extract_epi32(s00a, 1);
        ssd += _mm_extract_epi32(s00a, 2);
        ssd += _mm_extract_epi32(s00a, 3);
    }

    return ssd;
}

static s64 ssd_16b_sse_8nx4n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd = 0;
    s16 * s1;
    s16 * s2;
    int     i, j;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s02, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i = 0; i<(h >> 2); i++)
    {
        s00a = _mm_setzero_si128();

        for(j = 0; j<(w >> 3); j++)
        {
            SSE_SSD_16B_8PEL(s1, s2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1 * 2, s2 + s_src2 * 2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1 * 3, s2 + s_src2 * 3, shift, s00, s01, s02, s00a);

            s1 += 8;
            s2 += 8;
        }
        s1 += (s_src1 << 2) - ((w >> 3) << 3);
        s2 += (s_src2 << 2) - ((w >> 3) << 3);

        ssd += _mm_extract_epi32(s00a, 0);
        ssd += _mm_extract_epi32(s00a, 1);
        ssd += _mm_extract_epi32(s00a, 2);
        ssd += _mm_extract_epi32(s00a, 3);
    }
    return ssd;
}

static s64 ssd_16b_sse_8nx8n(int w, int h, void * src1, void * src2, int s_src1, int s_src2, int bit_depth)
{
    s64   ssd = 0;
    s16 * s1;
    s16 * s2;
    int     i, j;
#if FULL_BITDEPTH_RDO
    const int shift = 0;
#else
    const int shift = (bit_depth - 8) << 1;
#endif
    __m128i s00, s01, s02, s00a;

    s1 = (s16 *)src1;
    s2 = (s16 *)src2;

    for(i=0; i<(h>>3); i++)
    {
        s00a = _mm_setzero_si128();

        for(j=0; j<(w>>3); j++)
        {
            SSE_SSD_16B_8PEL(s1, s2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1, s2 + s_src2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1*2, s2 + s_src2*2, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1*3, s2 + s_src2*3, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1*4, s2 + s_src2*4, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1*5, s2 + s_src2*5, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1*6, s2 + s_src2*6, shift, s00, s01, s02, s00a);
            SSE_SSD_16B_8PEL(s1 + s_src1*7, s2 + s_src2*7, shift, s00, s01, s02, s00a);
            s1 += 8;
            s2 += 8;
        }
        s1 += (s_src1<<3) - ((w>>3)<<3);
        s2 += (s_src2<<3) - ((w>>3)<<3);

        ssd += _mm_extract_epi32(s00a, 0);
        ssd += _mm_extract_epi32(s00a, 1);
        ssd += _mm_extract_epi32(s00a, 2);
        ssd += _mm_extract_epi32(s00a, 3);
    }

    return ssd;
}
#endif /* X86_SSE */

const EVCE_FN_SSD evce_tbl_ssd_16b[8][8] =
{
#if X86_SSE
    /* width == 1 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b_sse_4x2,  /* height == 2 */
        ssd_16b_sse_4x4,  /* height == 4 */
        ssd_16b_sse_4x8,  /* height == 8 */
        ssd_16b_sse_4x16, /* height == 16 */
        ssd_16b_sse_4x32, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 8 */
    {
        ssd_16b,  /* height == 1 */
        ssd_16b_sse_8x2,   /* height == 2 */
        ssd_16b_sse_8x4,   /* height == 4 */
        ssd_16b_sse_8x8,   /* height == 8 */
        ssd_16b_sse_8nx8n, /* height == 16 */
        ssd_16b_sse_8nx8n, /* height == 32 */
        ssd_16b_sse_8nx8n, /* height == 64 */
        ssd_16b_sse_8nx8n, /* height == 128 */
    },
    /* width == 16 */
    {
        ssd_16b,  /* height == 1 */
        ssd_16b_sse_8nx2n, /* height == 2 */
        ssd_16b_sse_8nx4n, /* height == 4 */
        ssd_16b_sse_8nx8n, /* height == 8 */
        ssd_16b_sse_8nx8n, /* height == 16 */
        ssd_16b_sse_8nx8n, /* height == 32 */
        ssd_16b_sse_8nx8n, /* height == 64 */
        ssd_16b_sse_8nx8n, /* height == 128 */
    },
    /* width == 32 */
    {
        ssd_16b,  /* height == 1 */
        ssd_16b_sse_8nx2n, /* height == 2 */
        ssd_16b_sse_8nx4n, /* height == 4 */
        ssd_16b_sse_8nx8n, /* height == 8 */
        ssd_16b_sse_8nx8n, /* height == 16 */
        ssd_16b_sse_8nx8n, /* height == 32 */
        ssd_16b_sse_8nx8n, /* height == 64 */
        ssd_16b_sse_8nx8n, /* height == 128 */
    },
    /* width == 64 */
    {
        ssd_16b,  /* height == 1 */
        ssd_16b,  /* height == 2 */
        ssd_16b_sse_8nx4n, /* height == 4 */
        ssd_16b_sse_8nx8n, /* height == 8 */
        ssd_16b_sse_8nx8n, /* height == 16 */
        ssd_16b_sse_8nx8n, /* height == 32 */
        ssd_16b_sse_8nx8n, /* height == 64 */
        ssd_16b_sse_8nx8n, /* height == 128 */
    },
    /* width == 128 */
    {
        ssd_16b,  /* height == 1 */
        ssd_16b_sse_8nx2n, /* height == 2 */
        ssd_16b_sse_8nx4n, /* height == 4 */
        ssd_16b_sse_8nx8n, /* height == 8 */
        ssd_16b_sse_8nx8n, /* height == 16 */
        ssd_16b_sse_8nx8n, /* height == 32 */
        ssd_16b_sse_8nx8n, /* height == 64 */
        ssd_16b_sse_8nx8n, /* height == 128 */
    }
#else
    /* width == 1 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 8 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 16 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 32 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 64 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    },
    /* width == 128 */
    {
        ssd_16b, /* height == 1 */
        ssd_16b, /* height == 2 */
        ssd_16b, /* height == 4 */
        ssd_16b, /* height == 8 */
        ssd_16b, /* height == 16 */
        ssd_16b, /* height == 32 */
        ssd_16b, /* height == 64 */
        ssd_16b, /* height == 128 */
    }
#endif
};

/* SATD **********************************************************************/
int evc_had_2x2(pel *org, pel *cur, int s_org, int s_cur, int step)
{
    int satd = 0;
    int diff[4], m[4];

    diff[0] = org[0        ] - cur[0        ];
    diff[1] = org[1        ] - cur[1        ];
    diff[2] = org[s_org    ] - cur[0 + s_cur];
    diff[3] = org[s_org + 1] - cur[1 + s_cur];
    m[0] = diff[0] + diff[2];
    m[1] = diff[1] + diff[3];
    m[2] = diff[0] - diff[2];
    m[3] = diff[1] - diff[3];
    satd += (EVC_ABS(m[0] + m[1]) >> 2);
    satd += EVC_ABS(m[0] - m[1]);
    satd += EVC_ABS(m[2] + m[3]);
    satd += EVC_ABS(m[2] - m[3]);

    return satd;
}

int evc_had_4x4(pel *org, pel *cur, int s_org, int s_cur, int step, int bit_depth)
{
#if X86_SSE 
    if(bit_depth <= 10)
    {
        int satd = 0;
        __m128i r0 = (_mm_loadl_epi64((const __m128i*)&org[0]));
        __m128i r1 = (_mm_loadl_epi64((const __m128i*)&org[s_org]));
        __m128i r2 = (_mm_loadl_epi64((const __m128i*)&org[2 * s_org]));
        __m128i r3 = (_mm_loadl_epi64((const __m128i*)&org[3 * s_org]));
        __m128i r4 = (_mm_loadl_epi64((const __m128i*)&cur[0]));
        __m128i r5 = (_mm_loadl_epi64((const __m128i*)&cur[s_cur]));
        __m128i r6 = (_mm_loadl_epi64((const __m128i*)&cur[2 * s_cur]));
        __m128i r7 = (_mm_loadl_epi64((const __m128i*)&cur[3 * s_cur]));
        __m128i sum;
        __m128i zero;

        r0 = _mm_sub_epi16(r0, r4);
        r1 = _mm_sub_epi16(r1, r5);
        r2 = _mm_sub_epi16(r2, r6);
        r3 = _mm_sub_epi16(r3, r7);

        // first stage
        r4 = r0;
        r5 = r1;

        r0 = _mm_add_epi16(r0, r3);
        r1 = _mm_add_epi16(r1, r2);

        r4 = _mm_sub_epi16(r4, r3);
        r5 = _mm_sub_epi16(r5, r2);

        r2 = r0;
        r3 = r4;

        r0 = _mm_add_epi16(r0, r1);
        r2 = _mm_sub_epi16(r2, r1);
        r3 = _mm_sub_epi16(r3, r5);
        r5 = _mm_add_epi16(r5, r4);

        // shuffle - flip matrix for vertical transform
        r0 = _mm_unpacklo_epi16(r0, r5);
        r2 = _mm_unpacklo_epi16(r2, r3);

        r3 = r0;
        r0 = _mm_unpacklo_epi32(r0, r2);
        r3 = _mm_unpackhi_epi32(r3, r2);

        r1 = r0;
        r2 = r3;
        r1 = _mm_srli_si128(r1, 8);
        r3 = _mm_srli_si128(r3, 8);

        // second stage
        r4 = r0;
        r5 = r1;

        r0 = _mm_add_epi16(r0, r3);
        r1 = _mm_add_epi16(r1, r2);

        r4 = _mm_sub_epi16(r4, r3);
        r5 = _mm_sub_epi16(r5, r2);

        r2 = r0;
        r3 = r4;

        r0 = _mm_add_epi16(r0, r1);
        r2 = _mm_sub_epi16(r2, r1);
        r3 = _mm_sub_epi16(r3, r5);
        r5 = _mm_add_epi16(r5, r4);

        // abs
        sum = _mm_abs_epi16(r0);

        s16* p = (s16*)&sum;
        p[0] = p[0] >> 2;

        sum = _mm_add_epi16(sum, _mm_abs_epi16(r2));
        sum = _mm_add_epi16(sum, _mm_abs_epi16(r3));
        sum = _mm_add_epi16(sum, _mm_abs_epi16(r5));

        zero = _mm_set1_epi16(0);
        sum = _mm_unpacklo_epi16(sum, zero);
        sum = _mm_hadd_epi32(sum, sum);
        sum = _mm_hadd_epi32(sum, sum);

        satd = _mm_cvtsi128_si32(sum);

        satd = ((satd + 1) >> 1);

        return satd;
    }
    else
    {
        int k;
        int satd = 0;
        int diff[16], m[16], d[16];

        for( k = 0; k < 16; k+=4 )
        {
            diff[k+0] = org[0] - cur[0];
            diff[k+1] = org[1] - cur[1];
            diff[k+2] = org[2] - cur[2];
            diff[k+3] = org[3] - cur[3];

            cur += s_cur;
            org += s_org;
        }

        m[ 0] = diff[ 0] + diff[12];
        m[ 1] = diff[ 1] + diff[13];
        m[ 2] = diff[ 2] + diff[14];
        m[ 3] = diff[ 3] + diff[15];
        m[ 4] = diff[ 4] + diff[ 8];
        m[ 5] = diff[ 5] + diff[ 9];
        m[ 6] = diff[ 6] + diff[10];
        m[ 7] = diff[ 7] + diff[11];
        m[ 8] = diff[ 4] - diff[ 8];
        m[ 9] = diff[ 5] - diff[ 9];
        m[10] = diff[ 6] - diff[10];
        m[11] = diff[ 7] - diff[11];
        m[12] = diff[ 0] - diff[12];
        m[13] = diff[ 1] - diff[13];
        m[14] = diff[ 2] - diff[14];
        m[15] = diff[ 3] - diff[15];

        d[ 0] = m[ 0] + m[ 4];
        d[ 1] = m[ 1] + m[ 5];
        d[ 2] = m[ 2] + m[ 6];
        d[ 3] = m[ 3] + m[ 7];
        d[ 4] = m[ 8] + m[12];
        d[ 5] = m[ 9] + m[13];
        d[ 6] = m[10] + m[14];
        d[ 7] = m[11] + m[15];
        d[ 8] = m[ 0] - m[ 4];
        d[ 9] = m[ 1] - m[ 5];
        d[10] = m[ 2] - m[ 6];
        d[11] = m[ 3] - m[ 7];
        d[12] = m[12] - m[ 8];
        d[13] = m[13] - m[ 9];
        d[14] = m[14] - m[10];
        d[15] = m[15] - m[11];

        m[ 0] = d[ 0] + d[ 3];
        m[ 1] = d[ 1] + d[ 2];
        m[ 2] = d[ 1] - d[ 2];
        m[ 3] = d[ 0] - d[ 3];
        m[ 4] = d[ 4] + d[ 7];
        m[ 5] = d[ 5] + d[ 6];
        m[ 6] = d[ 5] - d[ 6];
        m[ 7] = d[ 4] - d[ 7];
        m[ 8] = d[ 8] + d[11];
        m[ 9] = d[ 9] + d[10];
        m[10] = d[ 9] - d[10];
        m[11] = d[ 8] - d[11];
        m[12] = d[12] + d[15];
        m[13] = d[13] + d[14];
        m[14] = d[13] - d[14];
        m[15] = d[12] - d[15];

        d[ 0] = m[ 0] + m[ 1];
        d[ 1] = m[ 0] - m[ 1];
        d[ 2] = m[ 2] + m[ 3];
        d[ 3] = m[ 3] - m[ 2];
        d[ 4] = m[ 4] + m[ 5];
        d[ 5] = m[ 4] - m[ 5];
        d[ 6] = m[ 6] + m[ 7];
        d[ 7] = m[ 7] - m[ 6];
        d[ 8] = m[ 8] + m[ 9];
        d[ 9] = m[ 8] - m[ 9];
        d[10] = m[10] + m[11];
        d[11] = m[11] - m[10];
        d[12] = m[12] + m[13];
        d[13] = m[12] - m[13];
        d[14] = m[14] + m[15];
        d[15] = m[15] - m[14];

        satd += (EVC_ABS(d[0]) >> 2);
        for (k = 1; k < 16; k++)
        {
            satd += EVC_ABS(d[k]);
        }
        satd = ((satd + 1) >> 1);

        return satd;
    }
#else
    int k;
    int satd = 0;
    int diff[16], m[16], d[16];
    for( k = 0; k < 16; k+=4 )
    {
        diff[k+0] = org[0] - cur[0];
        diff[k+1] = org[1] - cur[1];
        diff[k+2] = org[2] - cur[2];
        diff[k+3] = org[3] - cur[3];
        cur += s_cur;
        org += s_org;
    }
    m[ 0] = diff[ 0] + diff[12];
    m[ 1] = diff[ 1] + diff[13];
    m[ 2] = diff[ 2] + diff[14];
    m[ 3] = diff[ 3] + diff[15];
    m[ 4] = diff[ 4] + diff[ 8];
    m[ 5] = diff[ 5] + diff[ 9];
    m[ 6] = diff[ 6] + diff[10];
    m[ 7] = diff[ 7] + diff[11];
    m[ 8] = diff[ 4] - diff[ 8];
    m[ 9] = diff[ 5] - diff[ 9];
    m[10] = diff[ 6] - diff[10];
    m[11] = diff[ 7] - diff[11];
    m[12] = diff[ 0] - diff[12];
    m[13] = diff[ 1] - diff[13];
    m[14] = diff[ 2] - diff[14];
    m[15] = diff[ 3] - diff[15];
    d[ 0] = m[ 0] + m[ 4];
    d[ 1] = m[ 1] + m[ 5];
    d[ 2] = m[ 2] + m[ 6];
    d[ 3] = m[ 3] + m[ 7];
    d[ 4] = m[ 8] + m[12];
    d[ 5] = m[ 9] + m[13];
    d[ 6] = m[10] + m[14];
    d[ 7] = m[11] + m[15];
    d[ 8] = m[ 0] - m[ 4];
    d[ 9] = m[ 1] - m[ 5];
    d[10] = m[ 2] - m[ 6];
    d[11] = m[ 3] - m[ 7];
    d[12] = m[12] - m[ 8];
    d[13] = m[13] - m[ 9];
    d[14] = m[14] - m[10];
    d[15] = m[15] - m[11];
    m[ 0] = d[ 0] + d[ 3];
    m[ 1] = d[ 1] + d[ 2];
    m[ 2] = d[ 1] - d[ 2];
    m[ 3] = d[ 0] - d[ 3];
    m[ 4] = d[ 4] + d[ 7];
    m[ 5] = d[ 5] + d[ 6];
    m[ 6] = d[ 5] - d[ 6];
    m[ 7] = d[ 4] - d[ 7];
    m[ 8] = d[ 8] + d[11];
    m[ 9] = d[ 9] + d[10];
    m[10] = d[ 9] - d[10];
    m[11] = d[ 8] - d[11];
    m[12] = d[12] + d[15];
    m[13] = d[13] + d[14];
    m[14] = d[13] - d[14];
    m[15] = d[12] - d[15];
    d[ 0] = m[ 0] + m[ 1];
    d[ 1] = m[ 0] - m[ 1];
    d[ 2] = m[ 2] + m[ 3];
    d[ 3] = m[ 3] - m[ 2];
    d[ 4] = m[ 4] + m[ 5];
    d[ 5] = m[ 4] - m[ 5];
    d[ 6] = m[ 6] + m[ 7];
    d[ 7] = m[ 7] - m[ 6];
    d[ 8] = m[ 8] + m[ 9];
    d[ 9] = m[ 8] - m[ 9];
    d[10] = m[10] + m[11];
    d[11] = m[11] - m[10];
    d[12] = m[12] + m[13];
    d[13] = m[12] - m[13];
    d[14] = m[14] + m[15];
    d[15] = m[15] - m[14];
    satd += (EVC_ABS(d[0]) >> 2);
    for (k = 1; k < 16; k++)
    {
        satd += EVC_ABS(d[k]);
    }
    satd = ((satd + 1) >> 1);
    return satd;
#endif
}

int evc_had_8x8(pel *org, pel *cur, int s_org, int s_cur, int step, int bit_depth)
{
#if X86_SSE 
    if(bit_depth <= 10)
    {
#if OPT_SIMD_HAD_SAD
    {
        int sad = 0;
        /* all 128 bit registers are named with a suffix mxnb, where m is the */
        /* number of n bits packed in the register                            */
        __m128i src0_8x16b, src1_8x16b, src2_8x16b, src3_8x16b;
        __m128i src4_8x16b, src5_8x16b, src6_8x16b, src7_8x16b;
        __m128i pred0_8x16b, pred1_8x16b, pred2_8x16b, pred3_8x16b;
        __m128i pred4_8x16b, pred5_8x16b, pred6_8x16b, pred7_8x16b;
        __m128i out0_8x16b, out1_8x16b, out2_8x16b, out3_8x16b;
        __m128i out4_8x16b, out5_8x16b, out6_8x16b, out7_8x16b;

        /**********************Residue Calculation********************************/
        src0_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src1_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src2_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src3_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src4_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src5_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src6_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src7_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;

        pred0_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred1_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred2_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred3_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred4_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred5_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred6_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred7_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;

        src0_8x16b = _mm_sub_epi16(src0_8x16b, pred0_8x16b);
        src1_8x16b = _mm_sub_epi16(src1_8x16b, pred1_8x16b);
        src2_8x16b = _mm_sub_epi16(src2_8x16b, pred2_8x16b);
        src3_8x16b = _mm_sub_epi16(src3_8x16b, pred3_8x16b);
        src4_8x16b = _mm_sub_epi16(src4_8x16b, pred4_8x16b);
        src5_8x16b = _mm_sub_epi16(src5_8x16b, pred5_8x16b);
        src6_8x16b = _mm_sub_epi16(src6_8x16b, pred6_8x16b);
        src7_8x16b = _mm_sub_epi16(src7_8x16b, pred7_8x16b);
        /**********************Residue Calculation********************************/

        /**************** 8x8 horizontal transform *******************************/
        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src0_8x16b, src1_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src2_8x16b, src3_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src4_8x16b, src5_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src0_8x16b, src1_8x16b);
        src2_8x16b = _mm_unpackhi_epi16(src2_8x16b, src3_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src4_8x16b, src5_8x16b);
        src6_8x16b = _mm_unpackhi_epi16(src6_8x16b, src7_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src2_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src2_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src6_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src6_8x16b);

        out0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        out1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        out2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        out3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        out4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        out5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        out6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        out7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/

        /* r0 + r1 */
        pred0_8x16b = _mm_add_epi16(out0_8x16b, out1_8x16b);
        /* r2 + r3 */
        pred2_8x16b = _mm_add_epi16(out2_8x16b, out3_8x16b);
        /* r4 + r5 */
        pred4_8x16b = _mm_add_epi16(out4_8x16b, out5_8x16b);
        /* r6 + r7 */
        pred6_8x16b = _mm_add_epi16(out6_8x16b, out7_8x16b);

        /* r0 + r1 + r2 + r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 + r6 + r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
        src0_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
        src4_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 + r1 - r2 - r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 - r6 - r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
        src2_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
        src6_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 */
        pred0_8x16b = _mm_sub_epi16(out0_8x16b, out1_8x16b);
        /* r2 - r3 */
        pred2_8x16b = _mm_sub_epi16(out2_8x16b, out3_8x16b);
        /* r4 - r5 */
        pred4_8x16b = _mm_sub_epi16(out4_8x16b, out5_8x16b);
        /* r6 - r7 */
        pred6_8x16b = _mm_sub_epi16(out6_8x16b, out7_8x16b);

        /* r0 - r1 + r2 - r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 + r6 - r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
        src1_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
        src5_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 - r2 + r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 - r6 + r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
        src3_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
        src7_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src0_8x16b, src1_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src2_8x16b, src3_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src4_8x16b, src5_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src0_8x16b, src1_8x16b);
        src2_8x16b = _mm_unpackhi_epi16(src2_8x16b, src3_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src4_8x16b, src5_8x16b);
        src6_8x16b = _mm_unpackhi_epi16(src6_8x16b, src7_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src2_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src2_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src6_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src6_8x16b);

        src0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        src1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        src2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        src3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        src4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        src5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        src6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        src7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/
        /**************** 8x8 horizontal transform *******************************/
        if(bit_depth == 8)
        {
            /************************* 8x8 Vertical Transform*************************/
            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi16(src0_8x16b, src1_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi16(src2_8x16b, src3_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi16(src4_8x16b, src5_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi16(src6_8x16b, src7_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out4_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out2_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out6_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi16(src0_8x16b, src1_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi16(src2_8x16b, src3_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi16(src4_8x16b, src5_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi16(src6_8x16b, src7_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out1_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out5_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out3_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out7_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /****************************SATD calculation ****************************/
            src0_8x16b = _mm_abs_epi16(out0_8x16b);
            src1_8x16b = _mm_abs_epi16(out1_8x16b);
            src2_8x16b = _mm_abs_epi16(out2_8x16b);
            src3_8x16b = _mm_abs_epi16(out3_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out4_8x16b);
            src1_8x16b = _mm_abs_epi16(out5_8x16b);
            src2_8x16b = _mm_abs_epi16(out6_8x16b);
            src3_8x16b = _mm_abs_epi16(out7_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            sad = (sad + 2) >> 2;

            return sad;
        }
        else
        {
            __m128i out0a_8x16b, out1a_8x16b, out2a_8x16b, out3a_8x16b;
            __m128i out4a_8x16b, out5a_8x16b, out6a_8x16b, out7a_8x16b;
            __m128i tmp0_8x16b, tmp1_8x16b, tmp2_8x16b, tmp3_8x16b;
            __m128i tmp4_8x16b, tmp5_8x16b, tmp6_8x16b, tmp7_8x16b;

            /************************* 8x8 Vertical Transform*************************/
            tmp0_8x16b = _mm_srli_si128(src0_8x16b, 8);
            tmp1_8x16b = _mm_srli_si128(src1_8x16b, 8);
            tmp2_8x16b = _mm_srli_si128(src2_8x16b, 8);
            tmp3_8x16b = _mm_srli_si128(src3_8x16b, 8);
            tmp4_8x16b = _mm_srli_si128(src4_8x16b, 8);
            tmp5_8x16b = _mm_srli_si128(src5_8x16b, 8);
            tmp6_8x16b = _mm_srli_si128(src6_8x16b, 8);
            tmp7_8x16b = _mm_srli_si128(src7_8x16b, 8);

            /*************************First 4 pixels ********************************/
            src0_8x16b = _mm_cvtepi16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepi16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepi16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepi16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepi16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepi16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepi16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepi16_epi32(src7_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out4_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out2_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out6_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src0_8x16b, src1_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src2_8x16b, src3_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src4_8x16b, src5_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src6_8x16b, src7_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out1_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out5_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out3_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out7_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /*************************First 4 pixels ********************************/

            /**************************Next 4 pixels *******************************/
            src0_8x16b = _mm_cvtepi16_epi32(tmp0_8x16b);
            src1_8x16b = _mm_cvtepi16_epi32(tmp1_8x16b);
            src2_8x16b = _mm_cvtepi16_epi32(tmp2_8x16b);
            src3_8x16b = _mm_cvtepi16_epi32(tmp3_8x16b);
            src4_8x16b = _mm_cvtepi16_epi32(tmp4_8x16b);
            src5_8x16b = _mm_cvtepi16_epi32(tmp5_8x16b);
            src6_8x16b = _mm_cvtepi16_epi32(tmp6_8x16b);
            src7_8x16b = _mm_cvtepi16_epi32(tmp7_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out4a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out2a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out6a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src0_8x16b, src1_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src2_8x16b, src3_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src4_8x16b, src5_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src6_8x16b, src7_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out1a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out5a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out3a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out7a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /**************************Next 4 pixels *******************************/
            /************************* 8x8 Vertical Transform*************************/

            /****************************SATD calculation ****************************/
            src0_8x16b = _mm_abs_epi32(out0_8x16b);
            src1_8x16b = _mm_abs_epi32(out1_8x16b);
            src2_8x16b = _mm_abs_epi32(out2_8x16b);
            src3_8x16b = _mm_abs_epi32(out3_8x16b);
            src4_8x16b = _mm_abs_epi32(out4_8x16b);
            src5_8x16b = _mm_abs_epi32(out5_8x16b);
            src6_8x16b = _mm_abs_epi32(out6_8x16b);
            src7_8x16b = _mm_abs_epi32(out7_8x16b);

            s32* p = (s32*)&src0_8x16b;
            p[0] = p[0] >> 2;

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out0a_8x16b);
            src1_8x16b = _mm_abs_epi32(out1a_8x16b);
            src2_8x16b = _mm_abs_epi32(out2a_8x16b);
            src3_8x16b = _mm_abs_epi32(out3a_8x16b);
            src4_8x16b = _mm_abs_epi32(out4a_8x16b);
            src5_8x16b = _mm_abs_epi32(out5a_8x16b);
            src6_8x16b = _mm_abs_epi32(out6a_8x16b);
            src7_8x16b = _mm_abs_epi32(out7a_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            sad = (sad + 2) >> 2;

            return sad;
        }
    }
#else
    int k, i;
    int satd = 0;
    __m128i m1[8], m2[8];
    __m128i n1[8][2];
    __m128i n2[8][2];
    __m128i sum;

    for(k = 0; k < 8; k++)
    {
        __m128i r0 = (_mm_loadu_si128((__m128i*)org));
        __m128i r1 = (_mm_lddqu_si128((__m128i*)cur));
        m2[k] = _mm_sub_epi16(r0, r1);
        org += s_org;
        cur += s_cur;        
    }

    //horizontal
    m1[0] = _mm_add_epi16(m2[0], m2[4]);
    m1[1] = _mm_add_epi16(m2[1], m2[5]);
    m1[2] = _mm_add_epi16(m2[2], m2[6]);
    m1[3] = _mm_add_epi16(m2[3], m2[7]);
    m1[4] = _mm_sub_epi16(m2[0], m2[4]);
    m1[5] = _mm_sub_epi16(m2[1], m2[5]);
    m1[6] = _mm_sub_epi16(m2[2], m2[6]);
    m1[7] = _mm_sub_epi16(m2[3], m2[7]);

    m2[0] = _mm_add_epi16(m1[0], m1[2]);
    m2[1] = _mm_add_epi16(m1[1], m1[3]);
    m2[2] = _mm_sub_epi16(m1[0], m1[2]);
    m2[3] = _mm_sub_epi16(m1[1], m1[3]);
    m2[4] = _mm_add_epi16(m1[4], m1[6]);
    m2[5] = _mm_add_epi16(m1[5], m1[7]);
    m2[6] = _mm_sub_epi16(m1[4], m1[6]);
    m2[7] = _mm_sub_epi16(m1[5], m1[7]);

    m1[0] = _mm_add_epi16(m2[0], m2[1]);
    m1[1] = _mm_sub_epi16(m2[0], m2[1]);
    m1[2] = _mm_add_epi16(m2[2], m2[3]);
    m1[3] = _mm_sub_epi16(m2[2], m2[3]);
    m1[4] = _mm_add_epi16(m2[4], m2[5]);
    m1[5] = _mm_sub_epi16(m2[4], m2[5]);
    m1[6] = _mm_add_epi16(m2[6], m2[7]);
    m1[7] = _mm_sub_epi16(m2[6], m2[7]);

    m2[0] = _mm_unpacklo_epi16(m1[0], m1[1]);
    m2[1] = _mm_unpacklo_epi16(m1[2], m1[3]);
    m2[2] = _mm_unpacklo_epi16(m1[4], m1[5]);
    m2[3] = _mm_unpacklo_epi16(m1[6], m1[7]);
    m2[4] = _mm_unpackhi_epi16(m1[0], m1[1]);
    m2[5] = _mm_unpackhi_epi16(m1[2], m1[3]);
    m2[6] = _mm_unpackhi_epi16(m1[4], m1[5]);
    m2[7] = _mm_unpackhi_epi16(m1[6], m1[7]);

    m1[0] = _mm_unpacklo_epi32(m2[0], m2[1]);
    m1[1] = _mm_unpackhi_epi32(m2[0], m2[1]);
    m1[2] = _mm_unpacklo_epi32(m2[2], m2[3]);
    m1[3] = _mm_unpackhi_epi32(m2[2], m2[3]);
    m1[4] = _mm_unpacklo_epi32(m2[4], m2[5]);
    m1[5] = _mm_unpackhi_epi32(m2[4], m2[5]);
    m1[6] = _mm_unpacklo_epi32(m2[6], m2[7]);
    m1[7] = _mm_unpackhi_epi32(m2[6], m2[7]);

    m2[0] = _mm_unpacklo_epi64(m1[0], m1[2]);
    m2[1] = _mm_unpackhi_epi64(m1[0], m1[2]);
    m2[2] = _mm_unpacklo_epi64(m1[1], m1[3]);
    m2[3] = _mm_unpackhi_epi64(m1[1], m1[3]);
    m2[4] = _mm_unpacklo_epi64(m1[4], m1[6]);
    m2[5] = _mm_unpackhi_epi64(m1[4], m1[6]);
    m2[6] = _mm_unpacklo_epi64(m1[5], m1[7]);
    m2[7] = _mm_unpackhi_epi64(m1[5], m1[7]);

    for(i = 0; i < 8; i++)
    {
        n2[i][0] = _mm_cvtepi16_epi32(m2[i]);
        n2[i][1] = _mm_cvtepi16_epi32(_mm_shuffle_epi32(m2[i], 0xEE));
    }

    for(i = 0; i < 2; i++)
    {
        n1[0][i] = _mm_add_epi32(n2[0][i], n2[4][i]);
        n1[1][i] = _mm_add_epi32(n2[1][i], n2[5][i]);
        n1[2][i] = _mm_add_epi32(n2[2][i], n2[6][i]);
        n1[3][i] = _mm_add_epi32(n2[3][i], n2[7][i]);
        n1[4][i] = _mm_sub_epi32(n2[0][i], n2[4][i]);
        n1[5][i] = _mm_sub_epi32(n2[1][i], n2[5][i]);
        n1[6][i] = _mm_sub_epi32(n2[2][i], n2[6][i]);
        n1[7][i] = _mm_sub_epi32(n2[3][i], n2[7][i]);

        n2[0][i] = _mm_add_epi32(n1[0][i], n1[2][i]);
        n2[1][i] = _mm_add_epi32(n1[1][i], n1[3][i]);
        n2[2][i] = _mm_sub_epi32(n1[0][i], n1[2][i]);
        n2[3][i] = _mm_sub_epi32(n1[1][i], n1[3][i]);
        n2[4][i] = _mm_add_epi32(n1[4][i], n1[6][i]);
        n2[5][i] = _mm_add_epi32(n1[5][i], n1[7][i]);
        n2[6][i] = _mm_sub_epi32(n1[4][i], n1[6][i]);
        n2[7][i] = _mm_sub_epi32(n1[5][i], n1[7][i]);

        n1[0][i] = _mm_abs_epi32(_mm_add_epi32(n2[0][i], n2[1][i]));
        n1[1][i] = _mm_abs_epi32(_mm_sub_epi32(n2[0][i], n2[1][i]));
        n1[2][i] = _mm_abs_epi32(_mm_add_epi32(n2[2][i], n2[3][i]));
        n1[3][i] = _mm_abs_epi32(_mm_sub_epi32(n2[2][i], n2[3][i]));
        n1[4][i] = _mm_abs_epi32(_mm_add_epi32(n2[4][i], n2[5][i]));
        n1[5][i] = _mm_abs_epi32(_mm_sub_epi32(n2[4][i], n2[5][i]));
        n1[6][i] = _mm_abs_epi32(_mm_add_epi32(n2[6][i], n2[7][i]));
        n1[7][i] = _mm_abs_epi32(_mm_sub_epi32(n2[6][i], n2[7][i]));
    }

    s32* p = (s32*)&n1[0][0];
    p[0] = p[0] >> 2;

    for(i = 0; i < 8; i++)
    {
        m1[i] = _mm_add_epi32(n1[i][0], n1[i][1]);
    }
    
    m1[0] = _mm_add_epi32(m1[0], m1[1]);
    m1[2] = _mm_add_epi32(m1[2], m1[3]);
    m1[4] = _mm_add_epi32(m1[4], m1[5]);
    m1[6] = _mm_add_epi32(m1[6], m1[7]);

    m1[0] = _mm_add_epi32(m1[0], m1[2]);
    m1[4] = _mm_add_epi32(m1[4], m1[6]);
    sum = _mm_add_epi32(m1[0], m1[4]);

    sum = _mm_hadd_epi32(sum, sum);
    sum = _mm_hadd_epi32(sum, sum);

    satd = _mm_cvtsi128_si32(sum);
    satd = ((satd + 2) >> 2);

    return satd;
#endif
    }
    else
    {
        int k, i, j, jj;
        int satd = 0;
        int diff[64], m1[8][8], m2[8][8], m3[8][8];
        for(k = 0; k < 64; k += 8)
        {
            diff[k + 0] = org[0] - cur[0];
            diff[k + 1] = org[1] - cur[1];
            diff[k + 2] = org[2] - cur[2];
            diff[k + 3] = org[3] - cur[3];
            diff[k + 4] = org[4] - cur[4];
            diff[k + 5] = org[5] - cur[5];
            diff[k + 6] = org[6] - cur[6];
            diff[k + 7] = org[7] - cur[7];
            cur += s_cur;
            org += s_org;
        }
        for(j = 0; j < 8; j++)
        {
            jj = j << 3;
            m2[j][0] = diff[jj] + diff[jj + 4];
            m2[j][1] = diff[jj + 1] + diff[jj + 5];
            m2[j][2] = diff[jj + 2] + diff[jj + 6];
            m2[j][3] = diff[jj + 3] + diff[jj + 7];
            m2[j][4] = diff[jj] - diff[jj + 4];
            m2[j][5] = diff[jj + 1] - diff[jj + 5];
            m2[j][6] = diff[jj + 2] - diff[jj + 6];
            m2[j][7] = diff[jj + 3] - diff[jj + 7];
            m1[j][0] = m2[j][0] + m2[j][2];
            m1[j][1] = m2[j][1] + m2[j][3];
            m1[j][2] = m2[j][0] - m2[j][2];
            m1[j][3] = m2[j][1] - m2[j][3];
            m1[j][4] = m2[j][4] + m2[j][6];
            m1[j][5] = m2[j][5] + m2[j][7];
            m1[j][6] = m2[j][4] - m2[j][6];
            m1[j][7] = m2[j][5] - m2[j][7];
            m2[j][0] = m1[j][0] + m1[j][1];
            m2[j][1] = m1[j][0] - m1[j][1];
            m2[j][2] = m1[j][2] + m1[j][3];
            m2[j][3] = m1[j][2] - m1[j][3];
            m2[j][4] = m1[j][4] + m1[j][5];
            m2[j][5] = m1[j][4] - m1[j][5];
            m2[j][6] = m1[j][6] + m1[j][7];
            m2[j][7] = m1[j][6] - m1[j][7];
        }
        for(i = 0; i < 8; i++)
        {
            m3[0][i] = m2[0][i] + m2[4][i];
            m3[1][i] = m2[1][i] + m2[5][i];
            m3[2][i] = m2[2][i] + m2[6][i];
            m3[3][i] = m2[3][i] + m2[7][i];
            m3[4][i] = m2[0][i] - m2[4][i];
            m3[5][i] = m2[1][i] - m2[5][i];
            m3[6][i] = m2[2][i] - m2[6][i];
            m3[7][i] = m2[3][i] - m2[7][i];
            m1[0][i] = m3[0][i] + m3[2][i];
            m1[1][i] = m3[1][i] + m3[3][i];
            m1[2][i] = m3[0][i] - m3[2][i];
            m1[3][i] = m3[1][i] - m3[3][i];
            m1[4][i] = m3[4][i] + m3[6][i];
            m1[5][i] = m3[5][i] + m3[7][i];
            m1[6][i] = m3[4][i] - m3[6][i];
            m1[7][i] = m3[5][i] - m3[7][i];
            m2[0][i] = m1[0][i] + m1[1][i];
            m2[1][i] = m1[0][i] - m1[1][i];
            m2[2][i] = m1[2][i] + m1[3][i];
            m2[3][i] = m1[2][i] - m1[3][i];
            m2[4][i] = m1[4][i] + m1[5][i];
            m2[5][i] = m1[4][i] - m1[5][i];
            m2[6][i] = m1[6][i] + m1[7][i];
            m2[7][i] = m1[6][i] - m1[7][i];
        }
        satd += EVC_ABS(m2[0][0]) >> 2;
        for(j = 1; j < 8; j++)
        {
            satd += EVC_ABS(m2[0][j]);
        }
        for(i = 1; i < 8; i++)
        {
            for(j = 0; j < 8; j++)
            {
                satd += EVC_ABS(m2[i][j]);
            }
        }
        satd = ((satd + 2) >> 2);
        return satd;
    }
#else
    int k, i, j, jj;
    int satd = 0;
    int diff[64], m1[8][8], m2[8][8], m3[8][8];

    for(k = 0; k < 64; k += 8)
    {
        diff[k+0] = org[0] - cur[0];
        diff[k+1] = org[1] - cur[1];
        diff[k+2] = org[2] - cur[2];
        diff[k+3] = org[3] - cur[3];
        diff[k+4] = org[4] - cur[4];
        diff[k+5] = org[5] - cur[5];
        diff[k+6] = org[6] - cur[6];
        diff[k+7] = org[7] - cur[7];

        cur += s_cur;
        org += s_org;
    }

    /* horizontal */
    for(j = 0; j < 8; j++)
    {
        jj = j << 3;
        m2[j][0] = diff[jj  ] + diff[jj+4];
        m2[j][1] = diff[jj+1] + diff[jj+5];
        m2[j][2] = diff[jj+2] + diff[jj+6];
        m2[j][3] = diff[jj+3] + diff[jj+7];
        m2[j][4] = diff[jj  ] - diff[jj+4];
        m2[j][5] = diff[jj+1] - diff[jj+5];
        m2[j][6] = diff[jj+2] - diff[jj+6];
        m2[j][7] = diff[jj+3] - diff[jj+7];

        m1[j][0] = m2[j][0] + m2[j][2];
        m1[j][1] = m2[j][1] + m2[j][3];
        m1[j][2] = m2[j][0] - m2[j][2];
        m1[j][3] = m2[j][1] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][6];
        m1[j][5] = m2[j][5] + m2[j][7];
        m1[j][6] = m2[j][4] - m2[j][6];
        m1[j][7] = m2[j][5] - m2[j][7];

        m2[j][0] = m1[j][0] + m1[j][1];
        m2[j][1] = m1[j][0] - m1[j][1];
        m2[j][2] = m1[j][2] + m1[j][3];
        m2[j][3] = m1[j][2] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][5];
        m2[j][5] = m1[j][4] - m1[j][5];
        m2[j][6] = m1[j][6] + m1[j][7];
        m2[j][7] = m1[j][6] - m1[j][7];
    }

    /* vertical */
    for(i = 0; i < 8; i++)
    {
        m3[0][i] = m2[0][i] + m2[4][i];
        m3[1][i] = m2[1][i] + m2[5][i];
        m3[2][i] = m2[2][i] + m2[6][i];
        m3[3][i] = m2[3][i] + m2[7][i];
        m3[4][i] = m2[0][i] - m2[4][i];
        m3[5][i] = m2[1][i] - m2[5][i];
        m3[6][i] = m2[2][i] - m2[6][i];
        m3[7][i] = m2[3][i] - m2[7][i];

        m1[0][i] = m3[0][i] + m3[2][i];
        m1[1][i] = m3[1][i] + m3[3][i];
        m1[2][i] = m3[0][i] - m3[2][i];
        m1[3][i] = m3[1][i] - m3[3][i];
        m1[4][i] = m3[4][i] + m3[6][i];
        m1[5][i] = m3[5][i] + m3[7][i];
        m1[6][i] = m3[4][i] - m3[6][i];
        m1[7][i] = m3[5][i] - m3[7][i];

        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
    }

    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 8; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }

    satd = ((satd + 2) >> 2);

    return satd;
#endif
}

int evc_had_16x8(pel *org, pel *cur, int s_org, int s_cur, int step, int bit_depth)
{
#if X86_SSE 
    if(bit_depth <= 10)
    {
#if OPT_SIMD_HAD_SAD
    {
        int sad = 0;

        /* all 128 bit registers are named with a suffix mxnb, where m is the */
        /* number of n bits packed in the register                            */
        __m128i src0_8x16b, src1_8x16b, src2_8x16b, src3_8x16b;
        __m128i src4_8x16b, src5_8x16b, src6_8x16b, src7_8x16b;
        __m128i src8_8x16b, src9_8x16b, src10_8x16b, src11_8x16b;
        __m128i src12_8x16b, src13_8x16b, src14_8x16b, src15_8x16b;
        __m128i pred0_8x16b, pred1_8x16b, pred2_8x16b, pred3_8x16b;
        __m128i pred4_8x16b, pred5_8x16b, pred6_8x16b, pred7_8x16b;
        __m128i pred8_8x16b, pred9_8x16b, pred10_8x16b, pred11_8x16b;
        __m128i pred12_8x16b, pred13_8x16b, pred14_8x16b, pred15_8x16b;
        __m128i out0_8x16b, out1_8x16b, out2_8x16b, out3_8x16b;
        __m128i out4_8x16b, out5_8x16b, out6_8x16b, out7_8x16b;
        __m128i out8_8x16b, out9_8x16b, out10_8x16b, out11_8x16b;
        __m128i out12_8x16b, out13_8x16b, out14_8x16b, out15_8x16b;

        /**********************Residue Calculation********************************/
        src0_8x16b = _mm_loadu_si128((__m128i *) org);
        src1_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;
        src2_8x16b = _mm_loadu_si128((__m128i *) org);
        src3_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;
        src4_8x16b = _mm_loadu_si128((__m128i *) org);
        src5_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;
        src6_8x16b = _mm_loadu_si128((__m128i *) org);
        src7_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;

        pred0_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred1_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;
        pred2_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred3_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;
        pred4_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred5_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;
        pred6_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred7_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;

        src0_8x16b = _mm_sub_epi16(src0_8x16b, pred0_8x16b);
        src1_8x16b = _mm_sub_epi16(src1_8x16b, pred1_8x16b);
        src2_8x16b = _mm_sub_epi16(src2_8x16b, pred2_8x16b);
        src3_8x16b = _mm_sub_epi16(src3_8x16b, pred3_8x16b);
        src4_8x16b = _mm_sub_epi16(src4_8x16b, pred4_8x16b);
        src5_8x16b = _mm_sub_epi16(src5_8x16b, pred5_8x16b);
        src6_8x16b = _mm_sub_epi16(src6_8x16b, pred6_8x16b);
        src7_8x16b = _mm_sub_epi16(src7_8x16b, pred7_8x16b);

        src8_8x16b = _mm_loadu_si128((__m128i *) org);
        src9_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;
        src10_8x16b = _mm_loadu_si128((__m128i *) org);
        src11_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;
        src12_8x16b = _mm_loadu_si128((__m128i *) org);
        src13_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;
        src14_8x16b = _mm_loadu_si128((__m128i *) org);
        src15_8x16b = _mm_loadu_si128((__m128i *)(org + 8));
        org = org + s_org;

        pred8_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred9_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;
        pred10_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred11_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;
        pred12_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred13_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;
        pred14_8x16b = _mm_loadu_si128((__m128i *) cur);
        pred15_8x16b = _mm_loadu_si128((__m128i *)(cur + 8));
        cur = cur + s_cur;

        src8_8x16b = _mm_sub_epi16(src8_8x16b, pred8_8x16b);
        src9_8x16b = _mm_sub_epi16(src9_8x16b, pred9_8x16b);
        src10_8x16b = _mm_sub_epi16(src10_8x16b, pred10_8x16b);
        src11_8x16b = _mm_sub_epi16(src11_8x16b, pred11_8x16b);
        src12_8x16b = _mm_sub_epi16(src12_8x16b, pred12_8x16b);
        src13_8x16b = _mm_sub_epi16(src13_8x16b, pred13_8x16b);
        src14_8x16b = _mm_sub_epi16(src14_8x16b, pred14_8x16b);
        src15_8x16b = _mm_sub_epi16(src15_8x16b, pred15_8x16b);
        /**********************Residue Calculation********************************/

        /**************** 8x8 horizontal transform *******************************/
        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src0_8x16b, src1_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src2_8x16b, src3_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src4_8x16b, src5_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src0_8x16b, src1_8x16b);
        src2_8x16b = _mm_unpackhi_epi16(src2_8x16b, src3_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src4_8x16b, src5_8x16b);
        src6_8x16b = _mm_unpackhi_epi16(src6_8x16b, src7_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src2_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src2_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src6_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src6_8x16b);

        out0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        out1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        out2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        out3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        out4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        out5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        out6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        out7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/

        /* r0 + r1 */
        pred0_8x16b = _mm_add_epi16(out0_8x16b, out1_8x16b);
        /* r2 + r3 */
        pred2_8x16b = _mm_add_epi16(out2_8x16b, out3_8x16b);
        /* r4 + r5 */
        pred4_8x16b = _mm_add_epi16(out4_8x16b, out5_8x16b);
        /* r6 + r7 */
        pred6_8x16b = _mm_add_epi16(out6_8x16b, out7_8x16b);

        /* r0 + r1 + r2 + r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 + r6 + r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
        src0_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
        src4_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 + r1 - r2 - r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 - r6 - r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
        src2_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
        src6_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 */
        pred0_8x16b = _mm_sub_epi16(out0_8x16b, out1_8x16b);
        /* r2 - r3 */
        pred2_8x16b = _mm_sub_epi16(out2_8x16b, out3_8x16b);
        /* r4 - r5 */
        pred4_8x16b = _mm_sub_epi16(out4_8x16b, out5_8x16b);
        /* r6 - r7 */
        pred6_8x16b = _mm_sub_epi16(out6_8x16b, out7_8x16b);

        /* r0 - r1 + r2 - r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 + r6 - r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
        src1_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
        src5_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 - r2 + r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 - r6 + r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
        src3_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
        src7_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src0_8x16b, src1_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src2_8x16b, src3_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src4_8x16b, src5_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src0_8x16b, src1_8x16b);
        src2_8x16b = _mm_unpackhi_epi16(src2_8x16b, src3_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src4_8x16b, src5_8x16b);
        src6_8x16b = _mm_unpackhi_epi16(src6_8x16b, src7_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src2_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src2_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src6_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src6_8x16b);

        src0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        src1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        src2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        src3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        src4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        src5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        src6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        src7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/
        /**************** 8x8 horizontal transform *******************************/

        /**************** 8x8 horizontal transform *******************************/
        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src8_8x16b, src9_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src10_8x16b, src11_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src12_8x16b, src13_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src14_8x16b, src15_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src8_8x16b, src9_8x16b);
        src10_8x16b = _mm_unpackhi_epi16(src10_8x16b, src11_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src12_8x16b, src13_8x16b);
        src14_8x16b = _mm_unpackhi_epi16(src14_8x16b, src15_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src10_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src10_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src14_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src14_8x16b);

        out0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        out1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        out2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        out3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        out4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        out5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        out6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        out7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/

        /* r0 + r1 */
        pred0_8x16b = _mm_add_epi16(out0_8x16b, out1_8x16b);
        /* r2 + r3 */
        pred2_8x16b = _mm_add_epi16(out2_8x16b, out3_8x16b);
        /* r4 + r5 */
        pred4_8x16b = _mm_add_epi16(out4_8x16b, out5_8x16b);
        /* r6 + r7 */
        pred6_8x16b = _mm_add_epi16(out6_8x16b, out7_8x16b);

        /* r0 + r1 + r2 + r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 + r6 + r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
        src8_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
        src12_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 + r1 - r2 - r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 - r6 - r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
        src10_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
        src14_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 */
        pred0_8x16b = _mm_sub_epi16(out0_8x16b, out1_8x16b);
        /* r2 - r3 */
        pred2_8x16b = _mm_sub_epi16(out2_8x16b, out3_8x16b);
        /* r4 - r5 */
        pred4_8x16b = _mm_sub_epi16(out4_8x16b, out5_8x16b);
        /* r6 - r7 */
        pred6_8x16b = _mm_sub_epi16(out6_8x16b, out7_8x16b);

        /* r0 - r1 + r2 - r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 + r6 - r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
        src9_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
        src13_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 - r2 + r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 - r6 + r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
        src11_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
        src15_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src8_8x16b, src9_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src10_8x16b, src11_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src12_8x16b, src13_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src14_8x16b, src15_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src8_8x16b, src9_8x16b);
        src10_8x16b = _mm_unpackhi_epi16(src10_8x16b, src11_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src12_8x16b, src13_8x16b);
        src14_8x16b = _mm_unpackhi_epi16(src14_8x16b, src15_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src10_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src10_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src14_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src14_8x16b);

        src8_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        src9_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        src10_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        src11_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        src12_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        src13_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        src14_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        src15_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/
        /**************** 8x8 horizontal transform *******************************/

        /****************Horizontal Transform Addition****************************/
        out0_8x16b = _mm_add_epi16(src0_8x16b, src1_8x16b);
        out1_8x16b = _mm_sub_epi16(src0_8x16b, src1_8x16b);

        out2_8x16b = _mm_add_epi16(src2_8x16b, src3_8x16b);
        out3_8x16b = _mm_sub_epi16(src2_8x16b, src3_8x16b);

        out4_8x16b = _mm_add_epi16(src4_8x16b, src5_8x16b);
        out5_8x16b = _mm_sub_epi16(src4_8x16b, src5_8x16b);

        out6_8x16b = _mm_add_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_sub_epi16(src6_8x16b, src7_8x16b);

        out8_8x16b = _mm_add_epi16(src8_8x16b, src9_8x16b);
        out9_8x16b = _mm_sub_epi16(src8_8x16b, src9_8x16b);

        out10_8x16b = _mm_add_epi16(src10_8x16b, src11_8x16b);
        out11_8x16b = _mm_sub_epi16(src10_8x16b, src11_8x16b);

        out12_8x16b = _mm_add_epi16(src12_8x16b, src13_8x16b);
        out13_8x16b = _mm_sub_epi16(src12_8x16b, src13_8x16b);

        out14_8x16b = _mm_add_epi16(src14_8x16b, src15_8x16b);
        out15_8x16b = _mm_sub_epi16(src14_8x16b, src15_8x16b);
        /****************Horizontal Transform Addition****************************/

        src0_8x16b = out0_8x16b;
        src1_8x16b = out1_8x16b;
        src2_8x16b = out2_8x16b;
        src3_8x16b = out3_8x16b;
        src4_8x16b = out4_8x16b;
        src5_8x16b = out5_8x16b;
        src6_8x16b = out6_8x16b;
        src7_8x16b = out7_8x16b;
        src8_8x16b = out8_8x16b;
        src9_8x16b = out9_8x16b;
        src10_8x16b = out10_8x16b;
        src11_8x16b = out11_8x16b;
        src12_8x16b = out12_8x16b;
        src13_8x16b = out13_8x16b;
        src14_8x16b = out14_8x16b;
        src15_8x16b = out15_8x16b;

        if(bit_depth == 8)
        {
            /************************* 8x8 Vertical Transform*************************/
            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi16(src0_8x16b, src2_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi16(src4_8x16b, src6_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi16(src8_8x16b, src10_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi16(src12_8x16b, src14_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out8_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out4_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out12_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi16(src0_8x16b, src2_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi16(src4_8x16b, src6_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi16(src8_8x16b, src10_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi16(src12_8x16b, src14_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out2_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out10_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out6_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out14_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /************************* 8x8 Vertical Transform*************************/
            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi16(src1_8x16b, src3_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi16(src5_8x16b, src7_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi16(src9_8x16b, src11_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi16(src13_8x16b, src15_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out1_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out9_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out5_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out13_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi16(src1_8x16b, src3_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi16(src5_8x16b, src7_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi16(src9_8x16b, src11_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi16(src13_8x16b, src15_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out3_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out11_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out7_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out15_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /****************************SATD calculation ****************************/
            src0_8x16b = _mm_abs_epi16(out0_8x16b);
            src1_8x16b = _mm_abs_epi16(out1_8x16b);
            src2_8x16b = _mm_abs_epi16(out2_8x16b);
            src3_8x16b = _mm_abs_epi16(out3_8x16b);

            s16* p = (s16*)&src0_8x16b;
            p[0] = p[0] >> 2;

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out4_8x16b);
            src1_8x16b = _mm_abs_epi16(out5_8x16b);
            src2_8x16b = _mm_abs_epi16(out6_8x16b);
            src3_8x16b = _mm_abs_epi16(out7_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out8_8x16b);
            src1_8x16b = _mm_abs_epi16(out9_8x16b);
            src2_8x16b = _mm_abs_epi16(out10_8x16b);
            src3_8x16b = _mm_abs_epi16(out11_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out12_8x16b);
            src1_8x16b = _mm_abs_epi16(out13_8x16b);
            src2_8x16b = _mm_abs_epi16(out14_8x16b);
            src3_8x16b = _mm_abs_epi16(out15_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            sad = (int)(sad / sqrt(16.0 * 8) * 2);

            return sad;
        }
        else
        {
            __m128i out0a_8x16b, out1a_8x16b, out2a_8x16b, out3a_8x16b;
            __m128i out4a_8x16b, out5a_8x16b, out6a_8x16b, out7a_8x16b;
            __m128i out8a_8x16b, out9a_8x16b, out10a_8x16b, out11a_8x16b;
            __m128i out12a_8x16b, out13a_8x16b, out14a_8x16b, out15a_8x16b;
            __m128i tmp0_8x16b, tmp1_8x16b, tmp2_8x16b, tmp3_8x16b;
            __m128i tmp4_8x16b, tmp5_8x16b, tmp6_8x16b, tmp7_8x16b;
            __m128i tmp8_8x16b, tmp9_8x16b, tmp10_8x16b, tmp11_8x16b;
            __m128i tmp12_8x16b, tmp13_8x16b, tmp14_8x16b, tmp15_8x16b;

            /************************* 8x8 Vertical Transform*************************/
            tmp0_8x16b = _mm_srli_si128(src0_8x16b, 8);
            tmp2_8x16b = _mm_srli_si128(src2_8x16b, 8);
            tmp4_8x16b = _mm_srli_si128(src4_8x16b, 8);
            tmp6_8x16b = _mm_srli_si128(src6_8x16b, 8);
            tmp8_8x16b = _mm_srli_si128(src8_8x16b, 8);
            tmp10_8x16b = _mm_srli_si128(src10_8x16b, 8);
            tmp12_8x16b = _mm_srli_si128(src12_8x16b, 8);
            tmp14_8x16b = _mm_srli_si128(src14_8x16b, 8);

            /*************************First 4 pixels ********************************/
            src0_8x16b = _mm_cvtepi16_epi32(src0_8x16b);
            src2_8x16b = _mm_cvtepi16_epi32(src2_8x16b);
            src4_8x16b = _mm_cvtepi16_epi32(src4_8x16b);
            src6_8x16b = _mm_cvtepi16_epi32(src6_8x16b);
            src8_8x16b = _mm_cvtepi16_epi32(src8_8x16b);
            src10_8x16b = _mm_cvtepi16_epi32(src10_8x16b);
            src12_8x16b = _mm_cvtepi16_epi32(src12_8x16b);
            src14_8x16b = _mm_cvtepi16_epi32(src14_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src8_8x16b, src10_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src12_8x16b, src14_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out8_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out4_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out12_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src0_8x16b, src2_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src4_8x16b, src6_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src8_8x16b, src10_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src12_8x16b, src14_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out2_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out10_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out6_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out14_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /*************************First 4 pixels ********************************/

            /**************************Next 4 pixels *******************************/
            src0_8x16b = _mm_cvtepi16_epi32(tmp0_8x16b);
            src2_8x16b = _mm_cvtepi16_epi32(tmp2_8x16b);
            src4_8x16b = _mm_cvtepi16_epi32(tmp4_8x16b);
            src6_8x16b = _mm_cvtepi16_epi32(tmp6_8x16b);
            src8_8x16b = _mm_cvtepi16_epi32(tmp8_8x16b);
            src10_8x16b = _mm_cvtepi16_epi32(tmp10_8x16b);
            src12_8x16b = _mm_cvtepi16_epi32(tmp12_8x16b);
            src14_8x16b = _mm_cvtepi16_epi32(tmp14_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src8_8x16b, src10_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src12_8x16b, src14_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out8a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out4a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out12a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src0_8x16b, src2_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src4_8x16b, src6_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src8_8x16b, src10_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src12_8x16b, src14_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out2a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out10a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out6a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out14a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /**************************Next 4 pixels *******************************/
            /************************* 8x8 Vertical Transform*************************/

            /************************* 8x8 Vertical Transform*************************/
            tmp1_8x16b = _mm_srli_si128(src1_8x16b, 8);
            tmp3_8x16b = _mm_srli_si128(src3_8x16b, 8);
            tmp5_8x16b = _mm_srli_si128(src5_8x16b, 8);
            tmp7_8x16b = _mm_srli_si128(src7_8x16b, 8);
            tmp9_8x16b = _mm_srli_si128(src9_8x16b, 8);
            tmp11_8x16b = _mm_srli_si128(src11_8x16b, 8);
            tmp13_8x16b = _mm_srli_si128(src13_8x16b, 8);
            tmp15_8x16b = _mm_srli_si128(src15_8x16b, 8);

            /*************************First 4 pixels ********************************/
            src1_8x16b = _mm_cvtepi16_epi32(src1_8x16b);
            src3_8x16b = _mm_cvtepi16_epi32(src3_8x16b);
            src5_8x16b = _mm_cvtepi16_epi32(src5_8x16b);
            src7_8x16b = _mm_cvtepi16_epi32(src7_8x16b);
            src9_8x16b = _mm_cvtepi16_epi32(src9_8x16b);
            src11_8x16b = _mm_cvtepi16_epi32(src11_8x16b);
            src13_8x16b = _mm_cvtepi16_epi32(src13_8x16b);
            src15_8x16b = _mm_cvtepi16_epi32(src15_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src1_8x16b, src3_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src5_8x16b, src7_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src9_8x16b, src11_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src13_8x16b, src15_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out1_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out9_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out5_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out13_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src1_8x16b, src3_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src5_8x16b, src7_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src9_8x16b, src11_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src13_8x16b, src15_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out3_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out11_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out7_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out15_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /*************************First 4 pixels ********************************/

            /*************************Next 4 pixels ********************************/
            src1_8x16b = _mm_cvtepi16_epi32(tmp1_8x16b);
            src3_8x16b = _mm_cvtepi16_epi32(tmp3_8x16b);
            src5_8x16b = _mm_cvtepi16_epi32(tmp5_8x16b);
            src7_8x16b = _mm_cvtepi16_epi32(tmp7_8x16b);
            src9_8x16b = _mm_cvtepi16_epi32(tmp9_8x16b);
            src11_8x16b = _mm_cvtepi16_epi32(tmp11_8x16b);
            src13_8x16b = _mm_cvtepi16_epi32(tmp13_8x16b);
            src15_8x16b = _mm_cvtepi16_epi32(tmp15_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src1_8x16b, src3_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src5_8x16b, src7_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src9_8x16b, src11_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src13_8x16b, src15_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out1a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out9a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out5a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out13a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src1_8x16b, src3_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src5_8x16b, src7_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src9_8x16b, src11_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src13_8x16b, src15_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out3a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out11a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out7a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out15a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /****************************SATD calculation ****************************/
            src0_8x16b = _mm_abs_epi32(out0_8x16b);
            src1_8x16b = _mm_abs_epi32(out1_8x16b);
            src2_8x16b = _mm_abs_epi32(out2_8x16b);
            src3_8x16b = _mm_abs_epi32(out3_8x16b);
            src4_8x16b = _mm_abs_epi32(out4_8x16b);
            src5_8x16b = _mm_abs_epi32(out5_8x16b);
            src6_8x16b = _mm_abs_epi32(out6_8x16b);
            src7_8x16b = _mm_abs_epi32(out7_8x16b);

            s32* p = (s32*)&src0_8x16b;
            p[0] = p[0] >> 2;

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out8_8x16b);
            src1_8x16b = _mm_abs_epi32(out9_8x16b);
            src2_8x16b = _mm_abs_epi32(out10_8x16b);
            src3_8x16b = _mm_abs_epi32(out11_8x16b);
            src4_8x16b = _mm_abs_epi32(out12_8x16b);
            src5_8x16b = _mm_abs_epi32(out13_8x16b);
            src6_8x16b = _mm_abs_epi32(out14_8x16b);
            src7_8x16b = _mm_abs_epi32(out15_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out0a_8x16b);
            src1_8x16b = _mm_abs_epi32(out1a_8x16b);
            src2_8x16b = _mm_abs_epi32(out2a_8x16b);
            src3_8x16b = _mm_abs_epi32(out3a_8x16b);
            src4_8x16b = _mm_abs_epi32(out4a_8x16b);
            src5_8x16b = _mm_abs_epi32(out5a_8x16b);
            src6_8x16b = _mm_abs_epi32(out6a_8x16b);
            src7_8x16b = _mm_abs_epi32(out7a_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out8a_8x16b);
            src1_8x16b = _mm_abs_epi32(out9a_8x16b);
            src2_8x16b = _mm_abs_epi32(out10a_8x16b);
            src3_8x16b = _mm_abs_epi32(out11a_8x16b);
            src4_8x16b = _mm_abs_epi32(out12a_8x16b);
            src5_8x16b = _mm_abs_epi32(out13a_8x16b);
            src6_8x16b = _mm_abs_epi32(out14a_8x16b);
            src7_8x16b = _mm_abs_epi32(out15a_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            sad = (int)(sad / sqrt(16.0 * 8) * 2);

            return sad;
        }
    }
#else
    int satd = 0;
    int k, l, i;
    __m128i m1[16][2], m2[16][2];
    __m128i sum = _mm_setzero_si128();
    __m128i vzero = _mm_setzero_si128();

    for(l = 0; l < 2; l++)
    {
        const pel *org_ptr = org + l * 8;
        const pel *cur_ptr = cur + l * 8;
        for(k = 0; k < 8; k++)
        {
            __m128i r0 = (_mm_loadu_si128((__m128i*)org_ptr)) ;
            __m128i r1 = (_mm_lddqu_si128((__m128i*)cur_ptr)) ;
            m2[k][l] = _mm_sub_epi16(r0, r1);
            org_ptr += s_org;
            cur_ptr += s_cur;            
        }

        //vertical
        m1[0][l] = _mm_add_epi16(m2[0][l], m2[4][l]);
        m1[1][l] = _mm_add_epi16(m2[1][l], m2[5][l]);
        m1[2][l] = _mm_add_epi16(m2[2][l], m2[6][l]);
        m1[3][l] = _mm_add_epi16(m2[3][l], m2[7][l]);
        m1[4][l] = _mm_sub_epi16(m2[0][l], m2[4][l]);
        m1[5][l] = _mm_sub_epi16(m2[1][l], m2[5][l]);
        m1[6][l] = _mm_sub_epi16(m2[2][l], m2[6][l]);
        m1[7][l] = _mm_sub_epi16(m2[3][l], m2[7][l]);

        m2[0][l] = _mm_add_epi16(m1[0][l], m1[2][l]);
        m2[1][l] = _mm_add_epi16(m1[1][l], m1[3][l]);
        m2[2][l] = _mm_sub_epi16(m1[0][l], m1[2][l]);
        m2[3][l] = _mm_sub_epi16(m1[1][l], m1[3][l]);
        m2[4][l] = _mm_add_epi16(m1[4][l], m1[6][l]);
        m2[5][l] = _mm_add_epi16(m1[5][l], m1[7][l]);
        m2[6][l] = _mm_sub_epi16(m1[4][l], m1[6][l]);
        m2[7][l] = _mm_sub_epi16(m1[5][l], m1[7][l]);

        m1[0][l] = _mm_add_epi16(m2[0][l], m2[1][l]);
        m1[1][l] = _mm_sub_epi16(m2[0][l], m2[1][l]);
        m1[2][l] = _mm_add_epi16(m2[2][l], m2[3][l]);
        m1[3][l] = _mm_sub_epi16(m2[2][l], m2[3][l]);
        m1[4][l] = _mm_add_epi16(m2[4][l], m2[5][l]);
        m1[5][l] = _mm_sub_epi16(m2[4][l], m2[5][l]);
        m1[6][l] = _mm_add_epi16(m2[6][l], m2[7][l]);
        m1[7][l] = _mm_sub_epi16(m2[6][l], m2[7][l]);
    }    

    // 4 x 8x4 blocks
    // 0 1
    // 2 3

    // transpose and do horizontal in two steps
    for(l = 0; l < 2; l++)
    {
        int off = l * 4;
        __m128i n1[16];
        __m128i n2[16];

        // transpose 8x4 -> 4x8
        // 0 1  ->  0   and        ->  2
        //          1         2 3      3

        // transpose 8x4 -> 4x8, block 0(2)
        m2[0][0] = _mm_unpacklo_epi16(m1[0 + off][0], m1[1 + off][0]);
        m2[1][0] = _mm_unpacklo_epi16(m1[2 + off][0], m1[3 + off][0]);
        m2[2][0] = _mm_unpackhi_epi16(m1[0 + off][0], m1[1 + off][0]);
        m2[3][0] = _mm_unpackhi_epi16(m1[2 + off][0], m1[3 + off][0]);

        m1[0][0] = _mm_unpacklo_epi32(m2[0][0], m2[1][0]);
        m1[1][0] = _mm_unpackhi_epi32(m2[0][0], m2[1][0]);
        m1[2][0] = _mm_unpacklo_epi32(m2[2][0], m2[3][0]);
        m1[3][0] = _mm_unpackhi_epi32(m2[2][0], m2[3][0]);

        m2[0][0] = _mm_unpacklo_epi64(m1[0][0], vzero);
        m2[1][0] = _mm_unpackhi_epi64(m1[0][0], vzero);
        m2[2][0] = _mm_unpacklo_epi64(m1[1][0], vzero);
        m2[3][0] = _mm_unpackhi_epi64(m1[1][0], vzero);
        m2[4][0] = _mm_unpacklo_epi64(m1[2][0], vzero);
        m2[5][0] = _mm_unpackhi_epi64(m1[2][0], vzero);
        m2[6][0] = _mm_unpacklo_epi64(m1[3][0], vzero);
        m2[7][0] = _mm_unpackhi_epi64(m1[3][0], vzero);

        // transpose 8x4 -> 4x8, block 1(3)
        m2[0 + 8][0] = _mm_unpacklo_epi16(m1[0 + off][1], m1[1 + off][1]);
        m2[1 + 8][0] = _mm_unpacklo_epi16(m1[2 + off][1], m1[3 + off][1]);
        m2[2 + 8][0] = _mm_unpackhi_epi16(m1[0 + off][1], m1[1 + off][1]);
        m2[3 + 8][0] = _mm_unpackhi_epi16(m1[2 + off][1], m1[3 + off][1]);

        m1[0 + 8][0] = _mm_unpacklo_epi32(m2[0 + 8][0], m2[1 + 8][0]);
        m1[1 + 8][0] = _mm_unpackhi_epi32(m2[0 + 8][0], m2[1 + 8][0]);
        m1[2 + 8][0] = _mm_unpacklo_epi32(m2[2 + 8][0], m2[3 + 8][0]);
        m1[3 + 8][0] = _mm_unpackhi_epi32(m2[2 + 8][0], m2[3 + 8][0]);

        m2[0 + 8][0] = _mm_unpacklo_epi64(m1[0 + 8][0], vzero);
        m2[1 + 8][0] = _mm_unpackhi_epi64(m1[0 + 8][0], vzero);
        m2[2 + 8][0] = _mm_unpacklo_epi64(m1[1 + 8][0], vzero);
        m2[3 + 8][0] = _mm_unpackhi_epi64(m1[1 + 8][0], vzero);
        m2[4 + 8][0] = _mm_unpacklo_epi64(m1[2 + 8][0], vzero);
        m2[5 + 8][0] = _mm_unpackhi_epi64(m1[2 + 8][0], vzero);
        m2[6 + 8][0] = _mm_unpacklo_epi64(m1[3 + 8][0], vzero);
        m2[7 + 8][0] = _mm_unpackhi_epi64(m1[3 + 8][0], vzero);

        // horizontal
        for(i = 0; i < 16; i++)
        {
            n1[i] = _mm_cvtepi16_epi32(m2[i][0]);
        }

        n2[0] = _mm_add_epi32(n1[0], n1[8]);
        n2[1] = _mm_add_epi32(n1[1], n1[9]);
        n2[2] = _mm_add_epi32(n1[2], n1[10]);
        n2[3] = _mm_add_epi32(n1[3], n1[11]);
        n2[4] = _mm_add_epi32(n1[4], n1[12]);
        n2[5] = _mm_add_epi32(n1[5], n1[13]);
        n2[6] = _mm_add_epi32(n1[6], n1[14]);
        n2[7] = _mm_add_epi32(n1[7], n1[15]);
        n2[8] = _mm_sub_epi32(n1[0], n1[8]);
        n2[9] = _mm_sub_epi32(n1[1], n1[9]);
        n2[10] = _mm_sub_epi32(n1[2], n1[10]);
        n2[11] = _mm_sub_epi32(n1[3], n1[11]);
        n2[12] = _mm_sub_epi32(n1[4], n1[12]);
        n2[13] = _mm_sub_epi32(n1[5], n1[13]);
        n2[14] = _mm_sub_epi32(n1[6], n1[14]);
        n2[15] = _mm_sub_epi32(n1[7], n1[15]);

        n1[0] = _mm_add_epi32(n2[0], n2[4]);
        n1[1] = _mm_add_epi32(n2[1], n2[5]);
        n1[2] = _mm_add_epi32(n2[2], n2[6]);
        n1[3] = _mm_add_epi32(n2[3], n2[7]);
        n1[4] = _mm_sub_epi32(n2[0], n2[4]);
        n1[5] = _mm_sub_epi32(n2[1], n2[5]);
        n1[6] = _mm_sub_epi32(n2[2], n2[6]);
        n1[7] = _mm_sub_epi32(n2[3], n2[7]);
        n1[8] = _mm_add_epi32(n2[8], n2[12]);
        n1[9] = _mm_add_epi32(n2[9], n2[13]);
        n1[10] = _mm_add_epi32(n2[10], n2[14]);
        n1[11] = _mm_add_epi32(n2[11], n2[15]);
        n1[12] = _mm_sub_epi32(n2[8], n2[12]);
        n1[13] = _mm_sub_epi32(n2[9], n2[13]);
        n1[14] = _mm_sub_epi32(n2[10], n2[14]);
        n1[15] = _mm_sub_epi32(n2[11], n2[15]);

        n2[0] = _mm_add_epi32(n1[0], n1[2]);
        n2[1] = _mm_add_epi32(n1[1], n1[3]);
        n2[2] = _mm_sub_epi32(n1[0], n1[2]);
        n2[3] = _mm_sub_epi32(n1[1], n1[3]);
        n2[4] = _mm_add_epi32(n1[4], n1[6]);
        n2[5] = _mm_add_epi32(n1[5], n1[7]);
        n2[6] = _mm_sub_epi32(n1[4], n1[6]);
        n2[7] = _mm_sub_epi32(n1[5], n1[7]);
        n2[8] = _mm_add_epi32(n1[8], n1[10]);
        n2[9] = _mm_add_epi32(n1[9], n1[11]);
        n2[10] = _mm_sub_epi32(n1[8], n1[10]);
        n2[11] = _mm_sub_epi32(n1[9], n1[11]);
        n2[12] = _mm_add_epi32(n1[12], n1[14]);
        n2[13] = _mm_add_epi32(n1[13], n1[15]);
        n2[14] = _mm_sub_epi32(n1[12], n1[14]);
        n2[15] = _mm_sub_epi32(n1[13], n1[15]);

        n1[0] = _mm_abs_epi32(_mm_add_epi32(n2[0], n2[1]));
        n1[1] = _mm_abs_epi32(_mm_sub_epi32(n2[0], n2[1]));
        n1[2] = _mm_abs_epi32(_mm_add_epi32(n2[2], n2[3]));
        n1[3] = _mm_abs_epi32(_mm_sub_epi32(n2[2], n2[3]));
        n1[4] = _mm_abs_epi32(_mm_add_epi32(n2[4], n2[5]));
        n1[5] = _mm_abs_epi32(_mm_sub_epi32(n2[4], n2[5]));
        n1[6] = _mm_abs_epi32(_mm_add_epi32(n2[6], n2[7]));
        n1[7] = _mm_abs_epi32(_mm_sub_epi32(n2[6], n2[7]));
        n1[8] = _mm_abs_epi32(_mm_add_epi32(n2[8], n2[9]));
        n1[9] = _mm_abs_epi32(_mm_sub_epi32(n2[8], n2[9]));
        n1[10] = _mm_abs_epi32(_mm_add_epi32(n2[10], n2[11]));
        n1[11] = _mm_abs_epi32(_mm_sub_epi32(n2[10], n2[11]));
        n1[12] = _mm_abs_epi32(_mm_add_epi32(n2[12], n2[13]));
        n1[13] = _mm_abs_epi32(_mm_sub_epi32(n2[12], n2[13]));
        n1[14] = _mm_abs_epi32(_mm_add_epi32(n2[14], n2[15]));
        n1[15] = _mm_abs_epi32(_mm_sub_epi32(n2[14], n2[15]));
        if (l == 0)
        {
            s32* p = (s32*)&n1[0];
            p[0] = p[0] >> 2;
        }
        // sum up
        n1[0] = _mm_add_epi32(n1[0], n1[1]);
        n1[2] = _mm_add_epi32(n1[2], n1[3]);
        n1[4] = _mm_add_epi32(n1[4], n1[5]);
        n1[6] = _mm_add_epi32(n1[6], n1[7]);
        n1[8] = _mm_add_epi32(n1[8], n1[9]);
        n1[10] = _mm_add_epi32(n1[10], n1[11]);
        n1[12] = _mm_add_epi32(n1[12], n1[13]);
        n1[14] = _mm_add_epi32(n1[14], n1[15]);

        n1[0] = _mm_add_epi32(n1[0], n1[2]);
        n1[4] = _mm_add_epi32(n1[4], n1[6]);
        n1[8] = _mm_add_epi32(n1[8], n1[10]);
        n1[12] = _mm_add_epi32(n1[12], n1[14]);

        n1[0] = _mm_add_epi32(n1[0], n1[4]);
        n1[8] = _mm_add_epi32(n1[8], n1[12]);

        n1[0] = _mm_add_epi32(n1[0], n1[8]);
        sum = _mm_add_epi32(sum, n1[0]);
    }

    sum = _mm_hadd_epi32(sum, sum);
    sum = _mm_hadd_epi32(sum, sum);

    satd = _mm_cvtsi128_si32(sum);

    satd = (int)(satd / sqrt(16.0 * 8) * 2);

    return satd;
#endif
    }
    else
    {
    int k, i, j, jj;
    int satd = 0;
    int diff[128], m1[8][16], m2[8][16];

    for(k = 0; k < 128; k += 16)
    {
        diff[k + 0] = org[0] - cur[0];
        diff[k + 1] = org[1] - cur[1];
        diff[k + 2] = org[2] - cur[2];
        diff[k + 3] = org[3] - cur[3];
        diff[k + 4] = org[4] - cur[4];
        diff[k + 5] = org[5] - cur[5];
        diff[k + 6] = org[6] - cur[6];
        diff[k + 7] = org[7] - cur[7];

        diff[k + 8]  = org[8]  - cur[8];
        diff[k + 9]  = org[9]  - cur[9];
        diff[k + 10] = org[10] - cur[10];
        diff[k + 11] = org[11] - cur[11];
        diff[k + 12] = org[12] - cur[12];
        diff[k + 13] = org[13] - cur[13];
        diff[k + 14] = org[14] - cur[14];
        diff[k + 15] = org[15] - cur[15];

        cur += s_cur;
        org += s_org;
    }

    for(j = 0; j < 8; j++)
    {
        jj = j << 4;

        m2[j][0] = diff[jj] + diff[jj + 8];
        m2[j][1] = diff[jj + 1] + diff[jj + 9];
        m2[j][2] = diff[jj + 2] + diff[jj + 10];
        m2[j][3] = diff[jj + 3] + diff[jj + 11];
        m2[j][4] = diff[jj + 4] + diff[jj + 12];
        m2[j][5] = diff[jj + 5] + diff[jj + 13];
        m2[j][6] = diff[jj + 6] + diff[jj + 14];
        m2[j][7] = diff[jj + 7] + diff[jj + 15];
        m2[j][8] = diff[jj] - diff[jj + 8];
        m2[j][9] = diff[jj + 1] - diff[jj + 9];
        m2[j][10] = diff[jj + 2] - diff[jj + 10];
        m2[j][11] = diff[jj + 3] - diff[jj + 11];
        m2[j][12] = diff[jj + 4] - diff[jj + 12];
        m2[j][13] = diff[jj + 5] - diff[jj + 13];
        m2[j][14] = diff[jj + 6] - diff[jj + 14];
        m2[j][15] = diff[jj + 7] - diff[jj + 15];

        m1[j][0] = m2[j][0] + m2[j][4];
        m1[j][1] = m2[j][1] + m2[j][5];
        m1[j][2] = m2[j][2] + m2[j][6];
        m1[j][3] = m2[j][3] + m2[j][7];
        m1[j][4] = m2[j][0] - m2[j][4];
        m1[j][5] = m2[j][1] - m2[j][5];
        m1[j][6] = m2[j][2] - m2[j][6];
        m1[j][7] = m2[j][3] - m2[j][7];
        m1[j][8] = m2[j][8] + m2[j][12];
        m1[j][9] = m2[j][9] + m2[j][13];
        m1[j][10] = m2[j][10] + m2[j][14];
        m1[j][11] = m2[j][11] + m2[j][15];
        m1[j][12] = m2[j][8] - m2[j][12];
        m1[j][13] = m2[j][9] - m2[j][13];
        m1[j][14] = m2[j][10] - m2[j][14];
        m1[j][15] = m2[j][11] - m2[j][15];

        m2[j][0] = m1[j][0] + m1[j][2];
        m2[j][1] = m1[j][1] + m1[j][3];
        m2[j][2] = m1[j][0] - m1[j][2];
        m2[j][3] = m1[j][1] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][6];
        m2[j][5] = m1[j][5] + m1[j][7];
        m2[j][6] = m1[j][4] - m1[j][6];
        m2[j][7] = m1[j][5] - m1[j][7];
        m2[j][8] = m1[j][8] + m1[j][10];
        m2[j][9] = m1[j][9] + m1[j][11];
        m2[j][10] = m1[j][8] - m1[j][10];
        m2[j][11] = m1[j][9] - m1[j][11];
        m2[j][12] = m1[j][12] + m1[j][14];
        m2[j][13] = m1[j][13] + m1[j][15];
        m2[j][14] = m1[j][12] - m1[j][14];
        m2[j][15] = m1[j][13] - m1[j][15];

        m1[j][0] = m2[j][0] + m2[j][1];
        m1[j][1] = m2[j][0] - m2[j][1];
        m1[j][2] = m2[j][2] + m2[j][3];
        m1[j][3] = m2[j][2] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][5];
        m1[j][5] = m2[j][4] - m2[j][5];
        m1[j][6] = m2[j][6] + m2[j][7];
        m1[j][7] = m2[j][6] - m2[j][7];
        m1[j][8] = m2[j][8] + m2[j][9];
        m1[j][9] = m2[j][8] - m2[j][9];
        m1[j][10] = m2[j][10] + m2[j][11];
        m1[j][11] = m2[j][10] - m2[j][11];
        m1[j][12] = m2[j][12] + m2[j][13];
        m1[j][13] = m2[j][12] - m2[j][13];
        m1[j][14] = m2[j][14] + m2[j][15];
        m1[j][15] = m2[j][14] - m2[j][15];
    }

    for(i = 0; i < 16; i++)
    {
        m2[0][i] = m1[0][i] + m1[4][i];
        m2[1][i] = m1[1][i] + m1[5][i];
        m2[2][i] = m1[2][i] + m1[6][i];
        m2[3][i] = m1[3][i] + m1[7][i];
        m2[4][i] = m1[0][i] - m1[4][i];
        m2[5][i] = m1[1][i] - m1[5][i];
        m2[6][i] = m1[2][i] - m1[6][i];
        m2[7][i] = m1[3][i] - m1[7][i];

        m1[0][i] = m2[0][i] + m2[2][i];
        m1[1][i] = m2[1][i] + m2[3][i];
        m1[2][i] = m2[0][i] - m2[2][i];
        m1[3][i] = m2[1][i] - m2[3][i];
        m1[4][i] = m2[4][i] + m2[6][i];
        m1[5][i] = m2[5][i] + m2[7][i];
        m1[6][i] = m2[4][i] - m2[6][i];
        m1[7][i] = m2[5][i] - m2[7][i];

        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
    }

    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 16; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 8; i++)
    {
        for (j = 0; j < 16; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }

    satd = (int)(satd / sqrt(16.0 * 8.0) * 2.0);

    return satd;
    }
#else
    int k, i, j, jj;
    int satd = 0;
    int diff[128], m1[8][16], m2[8][16];
    for(k = 0; k < 128; k += 16)
    {
        diff[k + 0] = org[0] - cur[0];
        diff[k + 1] = org[1] - cur[1];
        diff[k + 2] = org[2] - cur[2];
        diff[k + 3] = org[3] - cur[3];
        diff[k + 4] = org[4] - cur[4];
        diff[k + 5] = org[5] - cur[5];
        diff[k + 6] = org[6] - cur[6];
        diff[k + 7] = org[7] - cur[7];
        diff[k + 8]  = org[8]  - cur[8];
        diff[k + 9]  = org[9]  - cur[9];
        diff[k + 10] = org[10] - cur[10];
        diff[k + 11] = org[11] - cur[11];
        diff[k + 12] = org[12] - cur[12];
        diff[k + 13] = org[13] - cur[13];
        diff[k + 14] = org[14] - cur[14];
        diff[k + 15] = org[15] - cur[15];
        cur += s_cur;
        org += s_org;
    }
    for(j = 0; j < 8; j++)
    {
        jj = j << 4;
        m2[j][0] = diff[jj] + diff[jj + 8];
        m2[j][1] = diff[jj + 1] + diff[jj + 9];
        m2[j][2] = diff[jj + 2] + diff[jj + 10];
        m2[j][3] = diff[jj + 3] + diff[jj + 11];
        m2[j][4] = diff[jj + 4] + diff[jj + 12];
        m2[j][5] = diff[jj + 5] + diff[jj + 13];
        m2[j][6] = diff[jj + 6] + diff[jj + 14];
        m2[j][7] = diff[jj + 7] + diff[jj + 15];
        m2[j][8] = diff[jj] - diff[jj + 8];
        m2[j][9] = diff[jj + 1] - diff[jj + 9];
        m2[j][10] = diff[jj + 2] - diff[jj + 10];
        m2[j][11] = diff[jj + 3] - diff[jj + 11];
        m2[j][12] = diff[jj + 4] - diff[jj + 12];
        m2[j][13] = diff[jj + 5] - diff[jj + 13];
        m2[j][14] = diff[jj + 6] - diff[jj + 14];
        m2[j][15] = diff[jj + 7] - diff[jj + 15];
        m1[j][0] = m2[j][0] + m2[j][4];
        m1[j][1] = m2[j][1] + m2[j][5];
        m1[j][2] = m2[j][2] + m2[j][6];
        m1[j][3] = m2[j][3] + m2[j][7];
        m1[j][4] = m2[j][0] - m2[j][4];
        m1[j][5] = m2[j][1] - m2[j][5];
        m1[j][6] = m2[j][2] - m2[j][6];
        m1[j][7] = m2[j][3] - m2[j][7];
        m1[j][8] = m2[j][8] + m2[j][12];
        m1[j][9] = m2[j][9] + m2[j][13];
        m1[j][10] = m2[j][10] + m2[j][14];
        m1[j][11] = m2[j][11] + m2[j][15];
        m1[j][12] = m2[j][8] - m2[j][12];
        m1[j][13] = m2[j][9] - m2[j][13];
        m1[j][14] = m2[j][10] - m2[j][14];
        m1[j][15] = m2[j][11] - m2[j][15];
        m2[j][0] = m1[j][0] + m1[j][2];
        m2[j][1] = m1[j][1] + m1[j][3];
        m2[j][2] = m1[j][0] - m1[j][2];
        m2[j][3] = m1[j][1] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][6];
        m2[j][5] = m1[j][5] + m1[j][7];
        m2[j][6] = m1[j][4] - m1[j][6];
        m2[j][7] = m1[j][5] - m1[j][7];
        m2[j][8] = m1[j][8] + m1[j][10];
        m2[j][9] = m1[j][9] + m1[j][11];
        m2[j][10] = m1[j][8] - m1[j][10];
        m2[j][11] = m1[j][9] - m1[j][11];
        m2[j][12] = m1[j][12] + m1[j][14];
        m2[j][13] = m1[j][13] + m1[j][15];
        m2[j][14] = m1[j][12] - m1[j][14];
        m2[j][15] = m1[j][13] - m1[j][15];
        m1[j][0] = m2[j][0] + m2[j][1];
        m1[j][1] = m2[j][0] - m2[j][1];
        m1[j][2] = m2[j][2] + m2[j][3];
        m1[j][3] = m2[j][2] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][5];
        m1[j][5] = m2[j][4] - m2[j][5];
        m1[j][6] = m2[j][6] + m2[j][7];
        m1[j][7] = m2[j][6] - m2[j][7];
        m1[j][8] = m2[j][8] + m2[j][9];
        m1[j][9] = m2[j][8] - m2[j][9];
        m1[j][10] = m2[j][10] + m2[j][11];
        m1[j][11] = m2[j][10] - m2[j][11];
        m1[j][12] = m2[j][12] + m2[j][13];
        m1[j][13] = m2[j][12] - m2[j][13];
        m1[j][14] = m2[j][14] + m2[j][15];
        m1[j][15] = m2[j][14] - m2[j][15];
    }
    for(i = 0; i < 16; i++)
    {
        m2[0][i] = m1[0][i] + m1[4][i];
        m2[1][i] = m1[1][i] + m1[5][i];
        m2[2][i] = m1[2][i] + m1[6][i];
        m2[3][i] = m1[3][i] + m1[7][i];
        m2[4][i] = m1[0][i] - m1[4][i];
        m2[5][i] = m1[1][i] - m1[5][i];
        m2[6][i] = m1[2][i] - m1[6][i];
        m2[7][i] = m1[3][i] - m1[7][i];
        m1[0][i] = m2[0][i] + m2[2][i];
        m1[1][i] = m2[1][i] + m2[3][i];
        m1[2][i] = m2[0][i] - m2[2][i];
        m1[3][i] = m2[1][i] - m2[3][i];
        m1[4][i] = m2[4][i] + m2[6][i];
        m1[5][i] = m2[5][i] + m2[7][i];
        m1[6][i] = m2[4][i] - m2[6][i];
        m1[7][i] = m2[5][i] - m2[7][i];
        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
    }
    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 16; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 8; i++)
    {
        for (j = 0; j < 16; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }
    satd = (int)(satd / sqrt(16.0 * 8.0) * 2.0);
    return satd;
#endif
}

int evc_had_8x16(pel *org, pel *cur, int s_org, int s_cur, int step, int bit_depth)
{
#if X86_SSE 
    if(bit_depth <= 10)
    {
#if OPT_SIMD_HAD_SAD
    {
        int sad = 0;

        /* all 128 bit registers are named with a suffix mxnb, where m is the */
        /* number of n bits packed in the register                            */
        __m128i src0_8x16b, src1_8x16b, src2_8x16b, src3_8x16b;
        __m128i src4_8x16b, src5_8x16b, src6_8x16b, src7_8x16b;
        __m128i src8_8x16b, src9_8x16b, src10_8x16b, src11_8x16b;
        __m128i src12_8x16b, src13_8x16b, src14_8x16b, src15_8x16b;
        __m128i pred0_8x16b, pred1_8x16b, pred2_8x16b, pred3_8x16b;
        __m128i pred4_8x16b, pred5_8x16b, pred6_8x16b, pred7_8x16b;
        __m128i pred8_8x16b, pred9_8x16b, pred10_8x16b, pred11_8x16b;
        __m128i pred12_8x16b, pred13_8x16b, pred14_8x16b, pred15_8x16b;
        __m128i out0_8x16b, out1_8x16b, out2_8x16b, out3_8x16b;
        __m128i out4_8x16b, out5_8x16b, out6_8x16b, out7_8x16b;
        __m128i out8_8x16b, out9_8x16b, out10_8x16b, out11_8x16b;
        __m128i out12_8x16b, out13_8x16b, out14_8x16b, out15_8x16b;

        /**********************Residue Calculation********************************/
        src0_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src1_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src2_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src3_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src4_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src5_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src6_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src7_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;

        pred0_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred1_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred2_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred3_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred4_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred5_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred6_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred7_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;

        src0_8x16b = _mm_sub_epi16(src0_8x16b, pred0_8x16b);
        src1_8x16b = _mm_sub_epi16(src1_8x16b, pred1_8x16b);
        src2_8x16b = _mm_sub_epi16(src2_8x16b, pred2_8x16b);
        src3_8x16b = _mm_sub_epi16(src3_8x16b, pred3_8x16b);
        src4_8x16b = _mm_sub_epi16(src4_8x16b, pred4_8x16b);
        src5_8x16b = _mm_sub_epi16(src5_8x16b, pred5_8x16b);
        src6_8x16b = _mm_sub_epi16(src6_8x16b, pred6_8x16b);
        src7_8x16b = _mm_sub_epi16(src7_8x16b, pred7_8x16b);

        src8_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src9_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src10_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src11_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src12_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src13_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src14_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;
        src15_8x16b = _mm_loadu_si128((__m128i *) org);
        org = org + s_org;

        pred8_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred9_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred10_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred11_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred12_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred13_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred14_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;
        pred15_8x16b = _mm_loadu_si128((__m128i *) cur);
        cur = cur + s_cur;

        src8_8x16b = _mm_sub_epi16(src8_8x16b, pred8_8x16b);
        src9_8x16b = _mm_sub_epi16(src9_8x16b, pred9_8x16b);
        src10_8x16b = _mm_sub_epi16(src10_8x16b, pred10_8x16b);
        src11_8x16b = _mm_sub_epi16(src11_8x16b, pred11_8x16b);
        src12_8x16b = _mm_sub_epi16(src12_8x16b, pred12_8x16b);
        src13_8x16b = _mm_sub_epi16(src13_8x16b, pred13_8x16b);
        src14_8x16b = _mm_sub_epi16(src14_8x16b, pred14_8x16b);
        src15_8x16b = _mm_sub_epi16(src15_8x16b, pred15_8x16b);
        /**********************Residue Calculation********************************/

        /**************** 8x8 horizontal transform *******************************/
        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src0_8x16b, src1_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src2_8x16b, src3_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src4_8x16b, src5_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src0_8x16b, src1_8x16b);
        src2_8x16b = _mm_unpackhi_epi16(src2_8x16b, src3_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src4_8x16b, src5_8x16b);
        src6_8x16b = _mm_unpackhi_epi16(src6_8x16b, src7_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src2_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src2_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src6_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src6_8x16b);

        out0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        out1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        out2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        out3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        out4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        out5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        out6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        out7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/

        /* r0 + r1 */
        pred0_8x16b = _mm_add_epi16(out0_8x16b, out1_8x16b);
        /* r2 + r3 */
        pred2_8x16b = _mm_add_epi16(out2_8x16b, out3_8x16b);
        /* r4 + r5 */
        pred4_8x16b = _mm_add_epi16(out4_8x16b, out5_8x16b);
        /* r6 + r7 */
        pred6_8x16b = _mm_add_epi16(out6_8x16b, out7_8x16b);

        /* r0 + r1 + r2 + r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 + r6 + r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
        src0_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
        src4_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 + r1 - r2 - r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 - r6 - r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
        src2_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
        src6_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 */
        pred0_8x16b = _mm_sub_epi16(out0_8x16b, out1_8x16b);
        /* r2 - r3 */
        pred2_8x16b = _mm_sub_epi16(out2_8x16b, out3_8x16b);
        /* r4 - r5 */
        pred4_8x16b = _mm_sub_epi16(out4_8x16b, out5_8x16b);
        /* r6 - r7 */
        pred6_8x16b = _mm_sub_epi16(out6_8x16b, out7_8x16b);

        /* r0 - r1 + r2 - r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 + r6 - r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
        src1_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
        src5_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 - r2 + r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 - r6 + r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
        src3_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
        src7_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src0_8x16b, src1_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src2_8x16b, src3_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src4_8x16b, src5_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src0_8x16b, src1_8x16b);
        src2_8x16b = _mm_unpackhi_epi16(src2_8x16b, src3_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src4_8x16b, src5_8x16b);
        src6_8x16b = _mm_unpackhi_epi16(src6_8x16b, src7_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src2_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src2_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src6_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src6_8x16b);

        src0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        src1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        src2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        src3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        src4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        src5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        src6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        src7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/
        /**************** 8x8 horizontal transform *******************************/

        /**************** 8x8 horizontal transform *******************************/
        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src8_8x16b, src9_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src10_8x16b, src11_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src12_8x16b, src13_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src14_8x16b, src15_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src8_8x16b, src9_8x16b);
        src10_8x16b = _mm_unpackhi_epi16(src10_8x16b, src11_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src12_8x16b, src13_8x16b);
        src14_8x16b = _mm_unpackhi_epi16(src14_8x16b, src15_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src10_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src10_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src14_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src14_8x16b);

        out0_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        out1_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        out2_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        out3_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        out4_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        out5_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        out6_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        out7_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/

        /* r0 + r1 */
        pred0_8x16b = _mm_add_epi16(out0_8x16b, out1_8x16b);
        /* r2 + r3 */
        pred2_8x16b = _mm_add_epi16(out2_8x16b, out3_8x16b);
        /* r4 + r5 */
        pred4_8x16b = _mm_add_epi16(out4_8x16b, out5_8x16b);
        /* r6 + r7 */
        pred6_8x16b = _mm_add_epi16(out6_8x16b, out7_8x16b);

        /* r0 + r1 + r2 + r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 + r6 + r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
        src8_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
        src12_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 + r1 - r2 - r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 + r5 - r6 - r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
        src10_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
        src14_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 */
        pred0_8x16b = _mm_sub_epi16(out0_8x16b, out1_8x16b);
        /* r2 - r3 */
        pred2_8x16b = _mm_sub_epi16(out2_8x16b, out3_8x16b);
        /* r4 - r5 */
        pred4_8x16b = _mm_sub_epi16(out4_8x16b, out5_8x16b);
        /* r6 - r7 */
        pred6_8x16b = _mm_sub_epi16(out6_8x16b, out7_8x16b);

        /* r0 - r1 + r2 - r3 */
        pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 + r6 - r7 */
        pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
        src9_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
        src13_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /* r0 - r1 - r2 + r3 */
        pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
        /* r4 - r5 - r6 + r7 */
        pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
        /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
        src11_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
        /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
        src15_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

        /***********************    8x8 16 bit Transpose  ************************/
        out3_8x16b = _mm_unpacklo_epi16(src8_8x16b, src9_8x16b);
        pred0_8x16b = _mm_unpacklo_epi16(src10_8x16b, src11_8x16b);
        out2_8x16b = _mm_unpacklo_epi16(src12_8x16b, src13_8x16b);
        pred3_8x16b = _mm_unpacklo_epi16(src14_8x16b, src15_8x16b);
        out7_8x16b = _mm_unpackhi_epi16(src8_8x16b, src9_8x16b);
        src10_8x16b = _mm_unpackhi_epi16(src10_8x16b, src11_8x16b);
        pred7_8x16b = _mm_unpackhi_epi16(src12_8x16b, src13_8x16b);
        src14_8x16b = _mm_unpackhi_epi16(src14_8x16b, src15_8x16b);

        out1_8x16b = _mm_unpacklo_epi32(out3_8x16b, pred0_8x16b);
        out3_8x16b = _mm_unpackhi_epi32(out3_8x16b, pred0_8x16b);
        pred1_8x16b = _mm_unpacklo_epi32(out2_8x16b, pred3_8x16b);
        pred3_8x16b = _mm_unpackhi_epi32(out2_8x16b, pred3_8x16b);
        out5_8x16b = _mm_unpacklo_epi32(out7_8x16b, src10_8x16b);
        out7_8x16b = _mm_unpackhi_epi32(out7_8x16b, src10_8x16b);
        pred5_8x16b = _mm_unpacklo_epi32(pred7_8x16b, src14_8x16b);
        pred7_8x16b = _mm_unpackhi_epi32(pred7_8x16b, src14_8x16b);

        src8_8x16b = _mm_unpacklo_epi64(out1_8x16b, pred1_8x16b);
        src9_8x16b = _mm_unpackhi_epi64(out1_8x16b, pred1_8x16b);
        src10_8x16b = _mm_unpacklo_epi64(out3_8x16b, pred3_8x16b);
        src11_8x16b = _mm_unpackhi_epi64(out3_8x16b, pred3_8x16b);
        src12_8x16b = _mm_unpacklo_epi64(out5_8x16b, pred5_8x16b);
        src13_8x16b = _mm_unpackhi_epi64(out5_8x16b, pred5_8x16b);
        src14_8x16b = _mm_unpacklo_epi64(out7_8x16b, pred7_8x16b);
        src15_8x16b = _mm_unpackhi_epi64(out7_8x16b, pred7_8x16b);
        /**********************   8x8 16 bit Transpose End   *********************/
        /**************** 8x8 horizontal transform *******************************/

        /****************Horizontal Transform Addition****************************/
        out0_8x16b = _mm_add_epi16(src0_8x16b, src1_8x16b);
        out1_8x16b = _mm_sub_epi16(src0_8x16b, src1_8x16b);

        out2_8x16b = _mm_add_epi16(src2_8x16b, src3_8x16b);
        out3_8x16b = _mm_sub_epi16(src2_8x16b, src3_8x16b);

        out4_8x16b = _mm_add_epi16(src4_8x16b, src5_8x16b);
        out5_8x16b = _mm_sub_epi16(src4_8x16b, src5_8x16b);

        out6_8x16b = _mm_add_epi16(src6_8x16b, src7_8x16b);
        out7_8x16b = _mm_sub_epi16(src6_8x16b, src7_8x16b);

        out8_8x16b = _mm_add_epi16(src8_8x16b, src9_8x16b);
        out9_8x16b = _mm_sub_epi16(src8_8x16b, src9_8x16b);

        out10_8x16b = _mm_add_epi16(src10_8x16b, src11_8x16b);
        out11_8x16b = _mm_sub_epi16(src10_8x16b, src11_8x16b);

        out12_8x16b = _mm_add_epi16(src12_8x16b, src13_8x16b);
        out13_8x16b = _mm_sub_epi16(src12_8x16b, src13_8x16b);

        out14_8x16b = _mm_add_epi16(src14_8x16b, src15_8x16b);
        out15_8x16b = _mm_sub_epi16(src14_8x16b, src15_8x16b);
        /****************Horizontal Transform Addition****************************/

        src0_8x16b = out0_8x16b;
        src1_8x16b = out1_8x16b;
        src2_8x16b = out2_8x16b;
        src3_8x16b = out3_8x16b;
        src4_8x16b = out4_8x16b;
        src5_8x16b = out5_8x16b;
        src6_8x16b = out6_8x16b;
        src7_8x16b = out7_8x16b;
        src8_8x16b = out8_8x16b;
        src9_8x16b = out9_8x16b;
        src10_8x16b = out10_8x16b;
        src11_8x16b = out11_8x16b;
        src12_8x16b = out12_8x16b;
        src13_8x16b = out13_8x16b;
        src14_8x16b = out14_8x16b;
        src15_8x16b = out15_8x16b;

        if(bit_depth == 8)
        {
            /************************* 8x8 Vertical Transform*************************/
            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi16(src0_8x16b, src2_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi16(src4_8x16b, src6_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi16(src8_8x16b, src10_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi16(src12_8x16b, src14_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out8_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out4_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out12_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi16(src0_8x16b, src2_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi16(src4_8x16b, src6_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi16(src8_8x16b, src10_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi16(src12_8x16b, src14_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out2_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out10_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out6_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out14_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /************************* 8x8 Vertical Transform*************************/
            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi16(src1_8x16b, src3_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi16(src5_8x16b, src7_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi16(src9_8x16b, src11_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi16(src13_8x16b, src15_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out1_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out9_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out5_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out13_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi16(src1_8x16b, src3_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi16(src5_8x16b, src7_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi16(src9_8x16b, src11_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi16(src13_8x16b, src15_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out3_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out11_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi16(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi16(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out7_8x16b = _mm_add_epi16(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out15_8x16b = _mm_sub_epi16(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /****************************SATD calculation ****************************/
            src0_8x16b = _mm_abs_epi16(out0_8x16b);
            src1_8x16b = _mm_abs_epi16(out1_8x16b);
            src2_8x16b = _mm_abs_epi16(out2_8x16b);
            src3_8x16b = _mm_abs_epi16(out3_8x16b);

            s16* p = (s16*)&src0_8x16b;
            p[0] = p[0] >> 2;

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out4_8x16b);
            src1_8x16b = _mm_abs_epi16(out5_8x16b);
            src2_8x16b = _mm_abs_epi16(out6_8x16b);
            src3_8x16b = _mm_abs_epi16(out7_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out8_8x16b);
            src1_8x16b = _mm_abs_epi16(out9_8x16b);
            src2_8x16b = _mm_abs_epi16(out10_8x16b);
            src3_8x16b = _mm_abs_epi16(out11_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi16(out12_8x16b);
            src1_8x16b = _mm_abs_epi16(out13_8x16b);
            src2_8x16b = _mm_abs_epi16(out14_8x16b);
            src3_8x16b = _mm_abs_epi16(out15_8x16b);

            src4_8x16b = _mm_srli_si128(src0_8x16b, 8);
            src5_8x16b = _mm_srli_si128(src1_8x16b, 8);
            src6_8x16b = _mm_srli_si128(src2_8x16b, 8);
            src7_8x16b = _mm_srli_si128(src3_8x16b, 8);

            src0_8x16b = _mm_cvtepu16_epi32(src0_8x16b);
            src1_8x16b = _mm_cvtepu16_epi32(src1_8x16b);
            src2_8x16b = _mm_cvtepu16_epi32(src2_8x16b);
            src3_8x16b = _mm_cvtepu16_epi32(src3_8x16b);
            src4_8x16b = _mm_cvtepu16_epi32(src4_8x16b);
            src5_8x16b = _mm_cvtepu16_epi32(src5_8x16b);
            src6_8x16b = _mm_cvtepu16_epi32(src6_8x16b);
            src7_8x16b = _mm_cvtepu16_epi32(src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            sad = (int)(sad / sqrt(16.0 * 8) * 2);

            return sad;
        }
        else
        {
            __m128i out0a_8x16b, out1a_8x16b, out2a_8x16b, out3a_8x16b;
            __m128i out4a_8x16b, out5a_8x16b, out6a_8x16b, out7a_8x16b;
            __m128i out8a_8x16b, out9a_8x16b, out10a_8x16b, out11a_8x16b;
            __m128i out12a_8x16b, out13a_8x16b, out14a_8x16b, out15a_8x16b;
            __m128i tmp0_8x16b, tmp1_8x16b, tmp2_8x16b, tmp3_8x16b;
            __m128i tmp4_8x16b, tmp5_8x16b, tmp6_8x16b, tmp7_8x16b;
            __m128i tmp8_8x16b, tmp9_8x16b, tmp10_8x16b, tmp11_8x16b;
            __m128i tmp12_8x16b, tmp13_8x16b, tmp14_8x16b, tmp15_8x16b;

            /************************* 8x8 Vertical Transform*************************/
            tmp0_8x16b = _mm_srli_si128(src0_8x16b, 8);
            tmp2_8x16b = _mm_srli_si128(src2_8x16b, 8);
            tmp4_8x16b = _mm_srli_si128(src4_8x16b, 8);
            tmp6_8x16b = _mm_srli_si128(src6_8x16b, 8);
            tmp8_8x16b = _mm_srli_si128(src8_8x16b, 8);
            tmp10_8x16b = _mm_srli_si128(src10_8x16b, 8);
            tmp12_8x16b = _mm_srli_si128(src12_8x16b, 8);
            tmp14_8x16b = _mm_srli_si128(src14_8x16b, 8);

            /*************************First 4 pixels ********************************/
            src0_8x16b = _mm_cvtepi16_epi32(src0_8x16b);
            src2_8x16b = _mm_cvtepi16_epi32(src2_8x16b);
            src4_8x16b = _mm_cvtepi16_epi32(src4_8x16b);
            src6_8x16b = _mm_cvtepi16_epi32(src6_8x16b);
            src8_8x16b = _mm_cvtepi16_epi32(src8_8x16b);
            src10_8x16b = _mm_cvtepi16_epi32(src10_8x16b);
            src12_8x16b = _mm_cvtepi16_epi32(src12_8x16b);
            src14_8x16b = _mm_cvtepi16_epi32(src14_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src8_8x16b, src10_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src12_8x16b, src14_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out8_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out4_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out12_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src0_8x16b, src2_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src4_8x16b, src6_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src8_8x16b, src10_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src12_8x16b, src14_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out2_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out10_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out6_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out14_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /*************************First 4 pixels ********************************/

            /**************************Next 4 pixels *******************************/
            src0_8x16b = _mm_cvtepi16_epi32(tmp0_8x16b);
            src2_8x16b = _mm_cvtepi16_epi32(tmp2_8x16b);
            src4_8x16b = _mm_cvtepi16_epi32(tmp4_8x16b);
            src6_8x16b = _mm_cvtepi16_epi32(tmp6_8x16b);
            src8_8x16b = _mm_cvtepi16_epi32(tmp8_8x16b);
            src10_8x16b = _mm_cvtepi16_epi32(tmp10_8x16b);
            src12_8x16b = _mm_cvtepi16_epi32(tmp12_8x16b);
            src14_8x16b = _mm_cvtepi16_epi32(tmp14_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src8_8x16b, src10_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src12_8x16b, src14_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out0a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out8a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out4a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out12a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src0_8x16b, src2_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src4_8x16b, src6_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src8_8x16b, src10_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src12_8x16b, src14_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out2a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out10a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out6a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out14a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /**************************Next 4 pixels *******************************/
            /************************* 8x8 Vertical Transform*************************/

            /************************* 8x8 Vertical Transform*************************/
            tmp1_8x16b = _mm_srli_si128(src1_8x16b, 8);
            tmp3_8x16b = _mm_srli_si128(src3_8x16b, 8);
            tmp5_8x16b = _mm_srli_si128(src5_8x16b, 8);
            tmp7_8x16b = _mm_srli_si128(src7_8x16b, 8);
            tmp9_8x16b = _mm_srli_si128(src9_8x16b, 8);
            tmp11_8x16b = _mm_srli_si128(src11_8x16b, 8);
            tmp13_8x16b = _mm_srli_si128(src13_8x16b, 8);
            tmp15_8x16b = _mm_srli_si128(src15_8x16b, 8);

            /*************************First 4 pixels ********************************/
            src1_8x16b = _mm_cvtepi16_epi32(src1_8x16b);
            src3_8x16b = _mm_cvtepi16_epi32(src3_8x16b);
            src5_8x16b = _mm_cvtepi16_epi32(src5_8x16b);
            src7_8x16b = _mm_cvtepi16_epi32(src7_8x16b);
            src9_8x16b = _mm_cvtepi16_epi32(src9_8x16b);
            src11_8x16b = _mm_cvtepi16_epi32(src11_8x16b);
            src13_8x16b = _mm_cvtepi16_epi32(src13_8x16b);
            src15_8x16b = _mm_cvtepi16_epi32(src15_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src1_8x16b, src3_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src5_8x16b, src7_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src9_8x16b, src11_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src13_8x16b, src15_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out1_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out9_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out5_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out13_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src1_8x16b, src3_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src5_8x16b, src7_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src9_8x16b, src11_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src13_8x16b, src15_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out3_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out11_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out7_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out15_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /*************************First 4 pixels ********************************/

            /*************************Next 4 pixels ********************************/
            src1_8x16b = _mm_cvtepi16_epi32(tmp1_8x16b);
            src3_8x16b = _mm_cvtepi16_epi32(tmp3_8x16b);
            src5_8x16b = _mm_cvtepi16_epi32(tmp5_8x16b);
            src7_8x16b = _mm_cvtepi16_epi32(tmp7_8x16b);
            src9_8x16b = _mm_cvtepi16_epi32(tmp9_8x16b);
            src11_8x16b = _mm_cvtepi16_epi32(tmp11_8x16b);
            src13_8x16b = _mm_cvtepi16_epi32(tmp13_8x16b);
            src15_8x16b = _mm_cvtepi16_epi32(tmp15_8x16b);

            /* r0 + r1 */
            pred0_8x16b = _mm_add_epi32(src1_8x16b, src3_8x16b);
            /* r2 + r3 */
            pred2_8x16b = _mm_add_epi32(src5_8x16b, src7_8x16b);
            /* r4 + r5 */
            pred4_8x16b = _mm_add_epi32(src9_8x16b, src11_8x16b);
            /* r6 + r7 */
            pred6_8x16b = _mm_add_epi32(src13_8x16b, src15_8x16b);

            /* r0 + r1 + r2 + r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 + r6 + r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7 */
            out1a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 + r2 + r3 - r4 - r5 - r6 - r7 */
            out9a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 + r1 - r2 - r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 + r5 - r6 - r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 + r1 - r2 - r3 + r4 + r5 - r6 - r7 */
            out5a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 + r1 - r2 - r3 - r4 - r5 + r6 + r7 */
            out13a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 */
            pred0_8x16b = _mm_sub_epi32(src1_8x16b, src3_8x16b);
            /* r2 - r3 */
            pred2_8x16b = _mm_sub_epi32(src5_8x16b, src7_8x16b);
            /* r4 - r5 */
            pred4_8x16b = _mm_sub_epi32(src9_8x16b, src11_8x16b);
            /* r6 - r7 */
            pred6_8x16b = _mm_sub_epi32(src13_8x16b, src15_8x16b);

            /* r0 - r1 + r2 - r3 */
            pred1_8x16b = _mm_add_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 + r6 - r7 */
            pred5_8x16b = _mm_add_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 + r2 - r3 + r4 - r5 + r6 - r7 */
            out3a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 + r2 - r3 - r4 + r5 - r6 + r7 */
            out11a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);

            /* r0 - r1 - r2 + r3 */
            pred1_8x16b = _mm_sub_epi32(pred0_8x16b, pred2_8x16b);
            /* r4 - r5 - r6 + r7 */
            pred5_8x16b = _mm_sub_epi32(pred4_8x16b, pred6_8x16b);
            /* r0 - r1 - r2 + r3 + r4 - r5 - r6 + r7 */
            out7a_8x16b = _mm_add_epi32(pred1_8x16b, pred5_8x16b);
            /* r0 - r1 - r2 + r3 - r4 + r5 + r6 - r7 */
            out15a_8x16b = _mm_sub_epi32(pred1_8x16b, pred5_8x16b);
            /************************* 8x8 Vertical Transform*************************/

            /****************************SATD calculation ****************************/
            src0_8x16b = _mm_abs_epi32(out0_8x16b);
            src1_8x16b = _mm_abs_epi32(out1_8x16b);
            src2_8x16b = _mm_abs_epi32(out2_8x16b);
            src3_8x16b = _mm_abs_epi32(out3_8x16b);
            src4_8x16b = _mm_abs_epi32(out4_8x16b);
            src5_8x16b = _mm_abs_epi32(out5_8x16b);
            src6_8x16b = _mm_abs_epi32(out6_8x16b);
            src7_8x16b = _mm_abs_epi32(out7_8x16b);

            s32* p = (s32*)&src0_8x16b;
            p[0] = p[0] >> 2;

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out8_8x16b);
            src1_8x16b = _mm_abs_epi32(out9_8x16b);
            src2_8x16b = _mm_abs_epi32(out10_8x16b);
            src3_8x16b = _mm_abs_epi32(out11_8x16b);
            src4_8x16b = _mm_abs_epi32(out12_8x16b);
            src5_8x16b = _mm_abs_epi32(out13_8x16b);
            src6_8x16b = _mm_abs_epi32(out14_8x16b);
            src7_8x16b = _mm_abs_epi32(out15_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out0a_8x16b);
            src1_8x16b = _mm_abs_epi32(out1a_8x16b);
            src2_8x16b = _mm_abs_epi32(out2a_8x16b);
            src3_8x16b = _mm_abs_epi32(out3a_8x16b);
            src4_8x16b = _mm_abs_epi32(out4a_8x16b);
            src5_8x16b = _mm_abs_epi32(out5a_8x16b);
            src6_8x16b = _mm_abs_epi32(out6a_8x16b);
            src7_8x16b = _mm_abs_epi32(out7a_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            src0_8x16b = _mm_abs_epi32(out8a_8x16b);
            src1_8x16b = _mm_abs_epi32(out9a_8x16b);
            src2_8x16b = _mm_abs_epi32(out10a_8x16b);
            src3_8x16b = _mm_abs_epi32(out11a_8x16b);
            src4_8x16b = _mm_abs_epi32(out12a_8x16b);
            src5_8x16b = _mm_abs_epi32(out13a_8x16b);
            src6_8x16b = _mm_abs_epi32(out14a_8x16b);
            src7_8x16b = _mm_abs_epi32(out15a_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src1_8x16b);
            src2_8x16b = _mm_add_epi32(src2_8x16b, src3_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src5_8x16b);
            src6_8x16b = _mm_add_epi32(src6_8x16b, src7_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src2_8x16b);
            src4_8x16b = _mm_add_epi32(src4_8x16b, src6_8x16b);

            src0_8x16b = _mm_add_epi32(src0_8x16b, src4_8x16b);

            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);
            src0_8x16b = _mm_hadd_epi32(src0_8x16b, src0_8x16b);

            sad += _mm_cvtsi128_si32(src0_8x16b);

            sad = (int)(sad / sqrt(16.0 * 8) * 2);

            return sad;
        }
    }
#else
    int k, l, i;
    int satd = 0;
    __m128i m1[16], m2[16];
    __m128i sum = _mm_setzero_si128();

    for(k = 0; k < 16; k++)
    {
        __m128i r0 = (_mm_loadu_si128((__m128i*)org));
        __m128i r1 = (_mm_lddqu_si128((__m128i*)cur));
        m1[k] = _mm_sub_epi16(r0, r1);
        org += s_org;
        cur += s_cur;
    }

    // vertical
    m2[0] = _mm_add_epi16(m1[0], m1[8]);
    m2[1] = _mm_add_epi16(m1[1], m1[9]);
    m2[2] = _mm_add_epi16(m1[2], m1[10]);
    m2[3] = _mm_add_epi16(m1[3], m1[11]);
    m2[4] = _mm_add_epi16(m1[4], m1[12]);
    m2[5] = _mm_add_epi16(m1[5], m1[13]);
    m2[6] = _mm_add_epi16(m1[6], m1[14]);
    m2[7] = _mm_add_epi16(m1[7], m1[15]);
    m2[8] = _mm_sub_epi16(m1[0], m1[8]);
    m2[9] = _mm_sub_epi16(m1[1], m1[9]);
    m2[10] = _mm_sub_epi16(m1[2], m1[10]);
    m2[11] = _mm_sub_epi16(m1[3], m1[11]);
    m2[12] = _mm_sub_epi16(m1[4], m1[12]);
    m2[13] = _mm_sub_epi16(m1[5], m1[13]);
    m2[14] = _mm_sub_epi16(m1[6], m1[14]);
    m2[15] = _mm_sub_epi16(m1[7], m1[15]);

    m1[0] = _mm_add_epi16(m2[0], m2[4]);
    m1[1] = _mm_add_epi16(m2[1], m2[5]);
    m1[2] = _mm_add_epi16(m2[2], m2[6]);
    m1[3] = _mm_add_epi16(m2[3], m2[7]);
    m1[4] = _mm_sub_epi16(m2[0], m2[4]);
    m1[5] = _mm_sub_epi16(m2[1], m2[5]);
    m1[6] = _mm_sub_epi16(m2[2], m2[6]);
    m1[7] = _mm_sub_epi16(m2[3], m2[7]);
    m1[8] = _mm_add_epi16(m2[8], m2[12]);
    m1[9] = _mm_add_epi16(m2[9], m2[13]);
    m1[10] = _mm_add_epi16(m2[10], m2[14]);
    m1[11] = _mm_add_epi16(m2[11], m2[15]);
    m1[12] = _mm_sub_epi16(m2[8], m2[12]);
    m1[13] = _mm_sub_epi16(m2[9], m2[13]);
    m1[14] = _mm_sub_epi16(m2[10], m2[14]);
    m1[15] = _mm_sub_epi16(m2[11], m2[15]);

    m2[0] = _mm_add_epi16(m1[0], m1[2]);
    m2[1] = _mm_add_epi16(m1[1], m1[3]);
    m2[2] = _mm_sub_epi16(m1[0], m1[2]);
    m2[3] = _mm_sub_epi16(m1[1], m1[3]);
    m2[4] = _mm_add_epi16(m1[4], m1[6]);
    m2[5] = _mm_add_epi16(m1[5], m1[7]);
    m2[6] = _mm_sub_epi16(m1[4], m1[6]);
    m2[7] = _mm_sub_epi16(m1[5], m1[7]);
    m2[8] = _mm_add_epi16(m1[8], m1[10]);
    m2[9] = _mm_add_epi16(m1[9], m1[11]);
    m2[10] = _mm_sub_epi16(m1[8], m1[10]);
    m2[11] = _mm_sub_epi16(m1[9], m1[11]);
    m2[12] = _mm_add_epi16(m1[12], m1[14]);
    m2[13] = _mm_add_epi16(m1[13], m1[15]);
    m2[14] = _mm_sub_epi16(m1[12], m1[14]);
    m2[15] = _mm_sub_epi16(m1[13], m1[15]);

    m1[0] = _mm_add_epi16(m2[0], m2[1]);
    m1[1] = _mm_sub_epi16(m2[0], m2[1]);
    m1[2] = _mm_add_epi16(m2[2], m2[3]);
    m1[3] = _mm_sub_epi16(m2[2], m2[3]);
    m1[4] = _mm_add_epi16(m2[4], m2[5]);
    m1[5] = _mm_sub_epi16(m2[4], m2[5]);
    m1[6] = _mm_add_epi16(m2[6], m2[7]);
    m1[7] = _mm_sub_epi16(m2[6], m2[7]);
    m1[8] = _mm_add_epi16(m2[8], m2[9]);
    m1[9] = _mm_sub_epi16(m2[8], m2[9]);
    m1[10] = _mm_add_epi16(m2[10], m2[11]);
    m1[11] = _mm_sub_epi16(m2[10], m2[11]);
    m1[12] = _mm_add_epi16(m2[12], m2[13]);
    m1[13] = _mm_sub_epi16(m2[12], m2[13]);
    m1[14] = _mm_add_epi16(m2[14], m2[15]);
    m1[15] = _mm_sub_epi16(m2[14], m2[15]);

    // process horizontal in two steps ( 2 x 8x8 blocks )

    for(l = 0; l < 2; l++)
    {
        int off = l * 8;
        __m128i n1[8][2];
        __m128i n2[8][2];

        // transpose 8x8
        // blocks 0,1  and 2,3
        m2[0] = _mm_unpacklo_epi16(m1[0 + off], m1[1 + off]);
        m2[1] = _mm_unpacklo_epi16(m1[2 + off], m1[3 + off]);
        m2[2] = _mm_unpacklo_epi16(m1[4 + off], m1[5 + off]);
        m2[3] = _mm_unpacklo_epi16(m1[6 + off], m1[7 + off]);

        m2[0 + 4] = _mm_unpackhi_epi16(m1[0 + off], m1[1 + off]);
        m2[1 + 4] = _mm_unpackhi_epi16(m1[2 + off], m1[3 + off]);
        m2[2 + 4] = _mm_unpackhi_epi16(m1[4 + off], m1[5 + off]);
        m2[3 + 4] = _mm_unpackhi_epi16(m1[6 + off], m1[7 + off]);

        m1[0] = _mm_unpacklo_epi32(m2[0], m2[1]);
        m1[1] = _mm_unpackhi_epi32(m2[0], m2[1]);
        m1[2] = _mm_unpacklo_epi32(m2[2], m2[3]);
        m1[3] = _mm_unpackhi_epi32(m2[2], m2[3]);

        m2[0] = _mm_unpacklo_epi64(m1[0], m1[2]);
        m2[1] = _mm_unpackhi_epi64(m1[0], m1[2]);
        m2[2] = _mm_unpacklo_epi64(m1[1], m1[3]);
        m2[3] = _mm_unpackhi_epi64(m1[1], m1[3]);

        m1[0 + 4] = _mm_unpacklo_epi32(m2[0 + 4], m2[1 + 4]);
        m1[1 + 4] = _mm_unpackhi_epi32(m2[0 + 4], m2[1 + 4]);
        m1[2 + 4] = _mm_unpacklo_epi32(m2[2 + 4], m2[3 + 4]);
        m1[3 + 4] = _mm_unpackhi_epi32(m2[2 + 4], m2[3 + 4]);

        m2[0 + 4] = _mm_unpacklo_epi64(m1[0 + 4], m1[2 + 4]);
        m2[1 + 4] = _mm_unpackhi_epi64(m1[0 + 4], m1[2 + 4]);
        m2[2 + 4] = _mm_unpacklo_epi64(m1[1 + 4], m1[3 + 4]);
        m2[3 + 4] = _mm_unpackhi_epi64(m1[1 + 4], m1[3 + 4]);

        // horizontal calculation
        for(i = 0; i < 8; i++)
        {
            n2[i][0] = _mm_cvtepi16_epi32(m2[i]);
            n2[i][1] = _mm_cvtepi16_epi32(_mm_shuffle_epi32(m2[i], 0xEE));
        }

        for(i = 0; i < 2; i++)
        {
            n1[0][i] = _mm_add_epi32(n2[0][i], n2[4][i]);
            n1[1][i] = _mm_add_epi32(n2[1][i], n2[5][i]);
            n1[2][i] = _mm_add_epi32(n2[2][i], n2[6][i]);
            n1[3][i] = _mm_add_epi32(n2[3][i], n2[7][i]);
            n1[4][i] = _mm_sub_epi32(n2[0][i], n2[4][i]);
            n1[5][i] = _mm_sub_epi32(n2[1][i], n2[5][i]);
            n1[6][i] = _mm_sub_epi32(n2[2][i], n2[6][i]);
            n1[7][i] = _mm_sub_epi32(n2[3][i], n2[7][i]);

            n2[0][i] = _mm_add_epi32(n1[0][i], n1[2][i]);
            n2[1][i] = _mm_add_epi32(n1[1][i], n1[3][i]);
            n2[2][i] = _mm_sub_epi32(n1[0][i], n1[2][i]);
            n2[3][i] = _mm_sub_epi32(n1[1][i], n1[3][i]);
            n2[4][i] = _mm_add_epi32(n1[4][i], n1[6][i]);
            n2[5][i] = _mm_add_epi32(n1[5][i], n1[7][i]);
            n2[6][i] = _mm_sub_epi32(n1[4][i], n1[6][i]);
            n2[7][i] = _mm_sub_epi32(n1[5][i], n1[7][i]);

            n1[0][i] = _mm_abs_epi32(_mm_add_epi32(n2[0][i], n2[1][i]));
            n1[1][i] = _mm_abs_epi32(_mm_sub_epi32(n2[0][i], n2[1][i]));
            n1[2][i] = _mm_abs_epi32(_mm_add_epi32(n2[2][i], n2[3][i]));
            n1[3][i] = _mm_abs_epi32(_mm_sub_epi32(n2[2][i], n2[3][i]));
            n1[4][i] = _mm_abs_epi32(_mm_add_epi32(n2[4][i], n2[5][i]));
            n1[5][i] = _mm_abs_epi32(_mm_sub_epi32(n2[4][i], n2[5][i]));
            n1[6][i] = _mm_abs_epi32(_mm_add_epi32(n2[6][i], n2[7][i]));
            n1[7][i] = _mm_abs_epi32(_mm_sub_epi32(n2[6][i], n2[7][i]));
        }
        if (l == 0)
        {
            s32* p = (s32*)&n1[0][0];
            p[0] = p[0] >> 2;
        }

        for(i = 0; i < 8; i++)
        {
            m1[i] = _mm_add_epi32(n1[i][0], n1[i][1]);
        }

        m1[0] = _mm_add_epi32(m1[0], m1[1]);
        m1[2] = _mm_add_epi32(m1[2], m1[3]);
        m1[4] = _mm_add_epi32(m1[4], m1[5]);
        m1[6] = _mm_add_epi32(m1[6], m1[7]);

        m1[0] = _mm_add_epi32(m1[0], m1[2]);
        m1[4] = _mm_add_epi32(m1[4], m1[6]);
        sum = _mm_add_epi32(sum, _mm_add_epi32(m1[0], m1[4]));
    }

    sum = _mm_hadd_epi32(sum, sum);
    sum = _mm_hadd_epi32(sum, sum);

    satd = _mm_cvtsi128_si32(sum);

    satd = (int)(satd / sqrt(16.0 * 8) * 2);

    return satd;
#endif
    }
    else
    {
    int k, i, j, jj;
    int satd = 0;
    int diff[128], m1[16][8], m2[16][8];

    for(k = 0; k < 128; k += 8)
    {
        diff[k + 0] = org[0] - cur[0];
        diff[k + 1] = org[1] - cur[1];
        diff[k + 2] = org[2] - cur[2];
        diff[k + 3] = org[3] - cur[3];
        diff[k + 4] = org[4] - cur[4];
        diff[k + 5] = org[5] - cur[5];
        diff[k + 6] = org[6] - cur[6];
        diff[k + 7] = org[7] - cur[7];

        cur += s_cur;
        org += s_org;
    }

    for(j = 0; j < 16; j++)
    {
        jj = j << 3;

        m2[j][0] = diff[jj] + diff[jj + 4];
        m2[j][1] = diff[jj + 1] + diff[jj + 5];
        m2[j][2] = diff[jj + 2] + diff[jj + 6];
        m2[j][3] = diff[jj + 3] + diff[jj + 7];
        m2[j][4] = diff[jj] - diff[jj + 4];
        m2[j][5] = diff[jj + 1] - diff[jj + 5];
        m2[j][6] = diff[jj + 2] - diff[jj + 6];
        m2[j][7] = diff[jj + 3] - diff[jj + 7];

        m1[j][0] = m2[j][0] + m2[j][2];
        m1[j][1] = m2[j][1] + m2[j][3];
        m1[j][2] = m2[j][0] - m2[j][2];
        m1[j][3] = m2[j][1] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][6];
        m1[j][5] = m2[j][5] + m2[j][7];
        m1[j][6] = m2[j][4] - m2[j][6];
        m1[j][7] = m2[j][5] - m2[j][7];

        m2[j][0] = m1[j][0] + m1[j][1];
        m2[j][1] = m1[j][0] - m1[j][1];
        m2[j][2] = m1[j][2] + m1[j][3];
        m2[j][3] = m1[j][2] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][5];
        m2[j][5] = m1[j][4] - m1[j][5];
        m2[j][6] = m1[j][6] + m1[j][7];
        m2[j][7] = m1[j][6] - m1[j][7];
    }

    for(i = 0; i < 8; i++)
    {
        m1[0][i] = m2[0][i] + m2[8][i];
        m1[1][i] = m2[1][i] + m2[9][i];
        m1[2][i] = m2[2][i] + m2[10][i];
        m1[3][i] = m2[3][i] + m2[11][i];
        m1[4][i] = m2[4][i] + m2[12][i];
        m1[5][i] = m2[5][i] + m2[13][i];
        m1[6][i] = m2[6][i] + m2[14][i];
        m1[7][i] = m2[7][i] + m2[15][i];
        m1[8][i] = m2[0][i] - m2[8][i];
        m1[9][i] = m2[1][i] - m2[9][i];
        m1[10][i] = m2[2][i] - m2[10][i];
        m1[11][i] = m2[3][i] - m2[11][i];
        m1[12][i] = m2[4][i] - m2[12][i];
        m1[13][i] = m2[5][i] - m2[13][i];
        m1[14][i] = m2[6][i] - m2[14][i];
        m1[15][i] = m2[7][i] - m2[15][i];

        m2[0][i] = m1[0][i] + m1[4][i];
        m2[1][i] = m1[1][i] + m1[5][i];
        m2[2][i] = m1[2][i] + m1[6][i];
        m2[3][i] = m1[3][i] + m1[7][i];
        m2[4][i] = m1[0][i] - m1[4][i];
        m2[5][i] = m1[1][i] - m1[5][i];
        m2[6][i] = m1[2][i] - m1[6][i];
        m2[7][i] = m1[3][i] - m1[7][i];
        m2[8][i] = m1[8][i] + m1[12][i];
        m2[9][i] = m1[9][i] + m1[13][i];
        m2[10][i] = m1[10][i] + m1[14][i];
        m2[11][i] = m1[11][i] + m1[15][i];
        m2[12][i] = m1[8][i] - m1[12][i];
        m2[13][i] = m1[9][i] - m1[13][i];
        m2[14][i] = m1[10][i] - m1[14][i];
        m2[15][i] = m1[11][i] - m1[15][i];

        m1[0][i] = m2[0][i] + m2[2][i];
        m1[1][i] = m2[1][i] + m2[3][i];
        m1[2][i] = m2[0][i] - m2[2][i];
        m1[3][i] = m2[1][i] - m2[3][i];
        m1[4][i] = m2[4][i] + m2[6][i];
        m1[5][i] = m2[5][i] + m2[7][i];
        m1[6][i] = m2[4][i] - m2[6][i];
        m1[7][i] = m2[5][i] - m2[7][i];
        m1[8][i] = m2[8][i] + m2[10][i];
        m1[9][i] = m2[9][i] + m2[11][i];
        m1[10][i] = m2[8][i] - m2[10][i];
        m1[11][i] = m2[9][i] - m2[11][i];
        m1[12][i] = m2[12][i] + m2[14][i];
        m1[13][i] = m2[13][i] + m2[15][i];
        m1[14][i] = m2[12][i] - m2[14][i];
        m1[15][i] = m2[13][i] - m2[15][i];

        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
        m2[8][i] = m1[8][i] + m1[9][i];
        m2[9][i] = m1[8][i] - m1[9][i];
        m2[10][i] = m1[10][i] + m1[11][i];
        m2[11][i] = m1[10][i] - m1[11][i];
        m2[12][i] = m1[12][i] + m1[13][i];
        m2[13][i] = m1[12][i] - m1[13][i];
        m2[14][i] = m1[14][i] + m1[15][i];
        m2[15][i] = m1[14][i] - m1[15][i];
    }

    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 8; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 16; i++)
    {
        for (j = 0; j < 8; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }

    satd = (int)(satd / sqrt(16.0 * 8.0) * 2.0);

    return satd;
    }
#else
    int k, i, j, jj;
    int satd = 0;
    int diff[128], m1[16][8], m2[16][8];
    for(k = 0; k < 128; k += 8)
    {
        diff[k + 0] = org[0] - cur[0];
        diff[k + 1] = org[1] - cur[1];
        diff[k + 2] = org[2] - cur[2];
        diff[k + 3] = org[3] - cur[3];
        diff[k + 4] = org[4] - cur[4];
        diff[k + 5] = org[5] - cur[5];
        diff[k + 6] = org[6] - cur[6];
        diff[k + 7] = org[7] - cur[7];
        cur += s_cur;
        org += s_org;
    }
    for(j = 0; j < 16; j++)
    {
        jj = j << 3;
        m2[j][0] = diff[jj] + diff[jj + 4];
        m2[j][1] = diff[jj + 1] + diff[jj + 5];
        m2[j][2] = diff[jj + 2] + diff[jj + 6];
        m2[j][3] = diff[jj + 3] + diff[jj + 7];
        m2[j][4] = diff[jj] - diff[jj + 4];
        m2[j][5] = diff[jj + 1] - diff[jj + 5];
        m2[j][6] = diff[jj + 2] - diff[jj + 6];
        m2[j][7] = diff[jj + 3] - diff[jj + 7];
        m1[j][0] = m2[j][0] + m2[j][2];
        m1[j][1] = m2[j][1] + m2[j][3];
        m1[j][2] = m2[j][0] - m2[j][2];
        m1[j][3] = m2[j][1] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][6];
        m1[j][5] = m2[j][5] + m2[j][7];
        m1[j][6] = m2[j][4] - m2[j][6];
        m1[j][7] = m2[j][5] - m2[j][7];
        m2[j][0] = m1[j][0] + m1[j][1];
        m2[j][1] = m1[j][0] - m1[j][1];
        m2[j][2] = m1[j][2] + m1[j][3];
        m2[j][3] = m1[j][2] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][5];
        m2[j][5] = m1[j][4] - m1[j][5];
        m2[j][6] = m1[j][6] + m1[j][7];
        m2[j][7] = m1[j][6] - m1[j][7];
    }
    for(i = 0; i < 8; i++)
    {
        m1[0][i] = m2[0][i] + m2[8][i];
        m1[1][i] = m2[1][i] + m2[9][i];
        m1[2][i] = m2[2][i] + m2[10][i];
        m1[3][i] = m2[3][i] + m2[11][i];
        m1[4][i] = m2[4][i] + m2[12][i];
        m1[5][i] = m2[5][i] + m2[13][i];
        m1[6][i] = m2[6][i] + m2[14][i];
        m1[7][i] = m2[7][i] + m2[15][i];
        m1[8][i] = m2[0][i] - m2[8][i];
        m1[9][i] = m2[1][i] - m2[9][i];
        m1[10][i] = m2[2][i] - m2[10][i];
        m1[11][i] = m2[3][i] - m2[11][i];
        m1[12][i] = m2[4][i] - m2[12][i];
        m1[13][i] = m2[5][i] - m2[13][i];
        m1[14][i] = m2[6][i] - m2[14][i];
        m1[15][i] = m2[7][i] - m2[15][i];
        m2[0][i] = m1[0][i] + m1[4][i];
        m2[1][i] = m1[1][i] + m1[5][i];
        m2[2][i] = m1[2][i] + m1[6][i];
        m2[3][i] = m1[3][i] + m1[7][i];
        m2[4][i] = m1[0][i] - m1[4][i];
        m2[5][i] = m1[1][i] - m1[5][i];
        m2[6][i] = m1[2][i] - m1[6][i];
        m2[7][i] = m1[3][i] - m1[7][i];
        m2[8][i] = m1[8][i] + m1[12][i];
        m2[9][i] = m1[9][i] + m1[13][i];
        m2[10][i] = m1[10][i] + m1[14][i];
        m2[11][i] = m1[11][i] + m1[15][i];
        m2[12][i] = m1[8][i] - m1[12][i];
        m2[13][i] = m1[9][i] - m1[13][i];
        m2[14][i] = m1[10][i] - m1[14][i];
        m2[15][i] = m1[11][i] - m1[15][i];
        m1[0][i] = m2[0][i] + m2[2][i];
        m1[1][i] = m2[1][i] + m2[3][i];
        m1[2][i] = m2[0][i] - m2[2][i];
        m1[3][i] = m2[1][i] - m2[3][i];
        m1[4][i] = m2[4][i] + m2[6][i];
        m1[5][i] = m2[5][i] + m2[7][i];
        m1[6][i] = m2[4][i] - m2[6][i];
        m1[7][i] = m2[5][i] - m2[7][i];
        m1[8][i] = m2[8][i] + m2[10][i];
        m1[9][i] = m2[9][i] + m2[11][i];
        m1[10][i] = m2[8][i] - m2[10][i];
        m1[11][i] = m2[9][i] - m2[11][i];
        m1[12][i] = m2[12][i] + m2[14][i];
        m1[13][i] = m2[13][i] + m2[15][i];
        m1[14][i] = m2[12][i] - m2[14][i];
        m1[15][i] = m2[13][i] - m2[15][i];
        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
        m2[8][i] = m1[8][i] + m1[9][i];
        m2[9][i] = m1[8][i] - m1[9][i];
        m2[10][i] = m1[10][i] + m1[11][i];
        m2[11][i] = m1[10][i] - m1[11][i];
        m2[12][i] = m1[12][i] + m1[13][i];
        m2[13][i] = m1[12][i] - m1[13][i];
        m2[14][i] = m1[14][i] + m1[15][i];
        m2[15][i] = m1[14][i] - m1[15][i];
    }
    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 8; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 16; i++)
    {
        for (j = 0; j < 8; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }
    satd = (int)(satd / sqrt(16.0 * 8.0) * 2.0);
    return satd;
#endif
}

int evc_had_8x4(pel *org, pel *cur, int s_org, int s_cur, int step, int bit_depth)
{
#if X86_SSE
    if(bit_depth <= 10)
    {
    int k, i;
    int satd = 0;
    __m128i m1[8], m2[8];
    __m128i vzero = _mm_setzero_si128();
    __m128i sum;

    for(k = 0; k < 4; k++)
    {
        __m128i r0 = (_mm_loadu_si128((__m128i*)org));
        __m128i r1 = (_mm_lddqu_si128((__m128i*)cur));
        m1[k] = _mm_sub_epi16(r0, r1);
        org += s_org;
        cur += s_cur;
    }

    //vertical
    m2[0] = _mm_add_epi16(m1[0], m1[2]);
    m2[1] = _mm_add_epi16(m1[1], m1[3]);
    m2[2] = _mm_sub_epi16(m1[0], m1[2]);
    m2[3] = _mm_sub_epi16(m1[1], m1[3]);

    m1[0] = _mm_add_epi16(m2[0], m2[1]);
    m1[1] = _mm_sub_epi16(m2[0], m2[1]);
    m1[2] = _mm_add_epi16(m2[2], m2[3]);
    m1[3] = _mm_sub_epi16(m2[2], m2[3]);

    // transpose, partially
    m2[0] = _mm_unpacklo_epi16(m1[0], m1[1]);
    m2[1] = _mm_unpacklo_epi16(m1[2], m1[3]);
    m2[2] = _mm_unpackhi_epi16(m1[0], m1[1]);
    m2[3] = _mm_unpackhi_epi16(m1[2], m1[3]);

    m1[0] = _mm_unpacklo_epi32(m2[0], m2[1]);
    m1[1] = _mm_unpackhi_epi32(m2[0], m2[1]);
    m1[2] = _mm_unpacklo_epi32(m2[2], m2[3]);
    m1[3] = _mm_unpackhi_epi32(m2[2], m2[3]);

    // horizontal
    // finish transpose
    m2[0] = _mm_unpacklo_epi64(m1[0], vzero);
    m2[1] = _mm_unpackhi_epi64(m1[0], vzero);
    m2[2] = _mm_unpacklo_epi64(m1[1], vzero);
    m2[3] = _mm_unpackhi_epi64(m1[1], vzero);
    m2[4] = _mm_unpacklo_epi64(m1[2], vzero);
    m2[5] = _mm_unpackhi_epi64(m1[2], vzero);
    m2[6] = _mm_unpacklo_epi64(m1[3], vzero);
    m2[7] = _mm_unpackhi_epi64(m1[3], vzero);

    for(i = 0; i < 8; i++)
    {
        m2[i] = _mm_cvtepi16_epi32(m2[i]);
    }

    m1[0] = _mm_add_epi32(m2[0], m2[4]);
    m1[1] = _mm_add_epi32(m2[1], m2[5]);
    m1[2] = _mm_add_epi32(m2[2], m2[6]);
    m1[3] = _mm_add_epi32(m2[3], m2[7]);
    m1[4] = _mm_sub_epi32(m2[0], m2[4]);
    m1[5] = _mm_sub_epi32(m2[1], m2[5]);
    m1[6] = _mm_sub_epi32(m2[2], m2[6]);
    m1[7] = _mm_sub_epi32(m2[3], m2[7]);

    m2[0] = _mm_add_epi32(m1[0], m1[2]);
    m2[1] = _mm_add_epi32(m1[1], m1[3]);
    m2[2] = _mm_sub_epi32(m1[0], m1[2]);
    m2[3] = _mm_sub_epi32(m1[1], m1[3]);
    m2[4] = _mm_add_epi32(m1[4], m1[6]);
    m2[5] = _mm_add_epi32(m1[5], m1[7]);
    m2[6] = _mm_sub_epi32(m1[4], m1[6]);
    m2[7] = _mm_sub_epi32(m1[5], m1[7]);

    m1[0] = _mm_abs_epi32(_mm_add_epi32(m2[0], m2[1]));
    m1[1] = _mm_abs_epi32(_mm_sub_epi32(m2[0], m2[1]));
    m1[2] = _mm_abs_epi32(_mm_add_epi32(m2[2], m2[3]));
    m1[3] = _mm_abs_epi32(_mm_sub_epi32(m2[2], m2[3]));
    m1[4] = _mm_abs_epi32(_mm_add_epi32(m2[4], m2[5]));
    m1[5] = _mm_abs_epi32(_mm_sub_epi32(m2[4], m2[5]));
    m1[6] = _mm_abs_epi32(_mm_add_epi32(m2[6], m2[7]));
    m1[7] = _mm_abs_epi32(_mm_sub_epi32(m2[6], m2[7]));

    s32* p = (s32*)&m1[0];
    p[0] = p[0] >> 2;

    m1[0] = _mm_add_epi32(m1[0], m1[1]);
    m1[1] = _mm_add_epi32(m1[2], m1[3]);
    m1[2] = _mm_add_epi32(m1[4], m1[5]);
    m1[3] = _mm_add_epi32(m1[6], m1[7]);

    m1[0] = _mm_add_epi32(m1[0], m1[1]);
    m1[1] = _mm_add_epi32(m1[2], m1[3]);

    sum = _mm_add_epi32(m1[0], m1[1]);

    sum = _mm_hadd_epi32(sum, sum);
    sum = _mm_hadd_epi32(sum, sum);

    satd = _mm_cvtsi128_si32(sum);
    satd = (int)(satd / sqrt(4.0 * 8) * 2);

    return satd;
    }
    else
    {
        int k, i, j, jj;
        int satd = 0;
        int diff[32], m1[4][8], m2[4][8];
        for(k = 0; k < 32; k += 8)
        {
            diff[k + 0] = org[0] - cur[0];
            diff[k + 1] = org[1] - cur[1];
            diff[k + 2] = org[2] - cur[2];
            diff[k + 3] = org[3] - cur[3];
            diff[k + 4] = org[4] - cur[4];
            diff[k + 5] = org[5] - cur[5];
            diff[k + 6] = org[6] - cur[6];
            diff[k + 7] = org[7] - cur[7];
            cur += s_cur;
            org += s_org;
        }
        for(j = 0; j < 4; j++)
        {
            jj = j << 3;
            m2[j][0] = diff[jj] + diff[jj + 4];
            m2[j][1] = diff[jj + 1] + diff[jj + 5];
            m2[j][2] = diff[jj + 2] + diff[jj + 6];
            m2[j][3] = diff[jj + 3] + diff[jj + 7];
            m2[j][4] = diff[jj] - diff[jj + 4];
            m2[j][5] = diff[jj + 1] - diff[jj + 5];
            m2[j][6] = diff[jj + 2] - diff[jj + 6];
            m2[j][7] = diff[jj + 3] - diff[jj + 7];
            m1[j][0] = m2[j][0] + m2[j][2];
            m1[j][1] = m2[j][1] + m2[j][3];
            m1[j][2] = m2[j][0] - m2[j][2];
            m1[j][3] = m2[j][1] - m2[j][3];
            m1[j][4] = m2[j][4] + m2[j][6];
            m1[j][5] = m2[j][5] + m2[j][7];
            m1[j][6] = m2[j][4] - m2[j][6];
            m1[j][7] = m2[j][5] - m2[j][7];
            m2[j][0] = m1[j][0] + m1[j][1];
            m2[j][1] = m1[j][0] - m1[j][1];
            m2[j][2] = m1[j][2] + m1[j][3];
            m2[j][3] = m1[j][2] - m1[j][3];
            m2[j][4] = m1[j][4] + m1[j][5];
            m2[j][5] = m1[j][4] - m1[j][5];
            m2[j][6] = m1[j][6] + m1[j][7];
            m2[j][7] = m1[j][6] - m1[j][7];
        }
        for(i = 0; i < 8; i++)
        {
            m1[0][i] = m2[0][i] + m2[2][i];
            m1[1][i] = m2[1][i] + m2[3][i];
            m1[2][i] = m2[0][i] - m2[2][i];
            m1[3][i] = m2[1][i] - m2[3][i];
            m2[0][i] = m1[0][i] + m1[1][i];
            m2[1][i] = m1[0][i] - m1[1][i];
            m2[2][i] = m1[2][i] + m1[3][i];
            m2[3][i] = m1[2][i] - m1[3][i];
        }
        satd += EVC_ABS(m2[0][0]) >> 2;
        for(j = 1; j < 8; j++)
        {
            satd += EVC_ABS(m2[0][j]);
        }
        for(i = 1; i < 4; i++)
        {
            for(j = 0; j < 8; j++)
            {
                satd += EVC_ABS(m2[i][j]);
            }
        }
        satd = (int)(satd / sqrt(4.0 * 8.0) * 2.0);
        return satd;
    }
#else
    int k, i, j, jj;
    int satd = 0;
    int diff[32], m1[4][8], m2[4][8];

    for(k = 0; k < 32; k += 8)
    {
        diff[k + 0] = org[0] - cur[0];
        diff[k + 1] = org[1] - cur[1];
        diff[k + 2] = org[2] - cur[2];
        diff[k + 3] = org[3] - cur[3];
        diff[k + 4] = org[4] - cur[4];
        diff[k + 5] = org[5] - cur[5];
        diff[k + 6] = org[6] - cur[6];
        diff[k + 7] = org[7] - cur[7];

        cur += s_cur;
        org += s_org;
    }

    for(j = 0; j < 4; j++)
    {
        jj = j << 3;

        m2[j][0] = diff[jj] + diff[jj + 4];
        m2[j][1] = diff[jj + 1] + diff[jj + 5];
        m2[j][2] = diff[jj + 2] + diff[jj + 6];
        m2[j][3] = diff[jj + 3] + diff[jj + 7];
        m2[j][4] = diff[jj] - diff[jj + 4];
        m2[j][5] = diff[jj + 1] - diff[jj + 5];
        m2[j][6] = diff[jj + 2] - diff[jj + 6];
        m2[j][7] = diff[jj + 3] - diff[jj + 7];

        m1[j][0] = m2[j][0] + m2[j][2];
        m1[j][1] = m2[j][1] + m2[j][3];
        m1[j][2] = m2[j][0] - m2[j][2];
        m1[j][3] = m2[j][1] - m2[j][3];
        m1[j][4] = m2[j][4] + m2[j][6];
        m1[j][5] = m2[j][5] + m2[j][7];
        m1[j][6] = m2[j][4] - m2[j][6];
        m1[j][7] = m2[j][5] - m2[j][7];

        m2[j][0] = m1[j][0] + m1[j][1];
        m2[j][1] = m1[j][0] - m1[j][1];
        m2[j][2] = m1[j][2] + m1[j][3];
        m2[j][3] = m1[j][2] - m1[j][3];
        m2[j][4] = m1[j][4] + m1[j][5];
        m2[j][5] = m1[j][4] - m1[j][5];
        m2[j][6] = m1[j][6] + m1[j][7];
        m2[j][7] = m1[j][6] - m1[j][7];
    }

    for(i = 0; i < 8; i++)
    {
        m1[0][i] = m2[0][i] + m2[2][i];
        m1[1][i] = m2[1][i] + m2[3][i];
        m1[2][i] = m2[0][i] - m2[2][i];
        m1[3][i] = m2[1][i] - m2[3][i];

        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
    }

    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 8; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 4; i++)
    {
        for (j = 0; j < 8; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }

    satd = (int)(satd / sqrt(4.0 * 8.0) * 2.0);

    return satd;
#endif
}

int evc_had_4x8(pel *org, pel *cur, int s_org, int s_cur, int step, int bit_depth)
{
#if X86_SSE
    if(bit_depth <= 10)
    {
    int k, i;
    __m128i m1[8], m2[8];
    __m128i n1[4][2];
    __m128i n2[4][2];
    __m128i sum;
    int satd = 0;
    
    for(k = 0; k < 8; k++)
    {
        __m128i r0 = (_mm_loadl_epi64((__m128i*)org));
        __m128i r1 = (_mm_loadl_epi64((__m128i*)cur));
        m2[k] = _mm_sub_epi16(r0, r1);        
        org += s_org;
        cur += s_cur;
    }

    // vertical

    m1[0] = _mm_add_epi16(m2[0], m2[4]);
    m1[1] = _mm_add_epi16(m2[1], m2[5]);
    m1[2] = _mm_add_epi16(m2[2], m2[6]);
    m1[3] = _mm_add_epi16(m2[3], m2[7]);
    m1[4] = _mm_sub_epi16(m2[0], m2[4]);
    m1[5] = _mm_sub_epi16(m2[1], m2[5]);
    m1[6] = _mm_sub_epi16(m2[2], m2[6]);
    m1[7] = _mm_sub_epi16(m2[3], m2[7]);

    m2[0] = _mm_add_epi16(m1[0], m1[2]);
    m2[1] = _mm_add_epi16(m1[1], m1[3]);
    m2[2] = _mm_sub_epi16(m1[0], m1[2]);
    m2[3] = _mm_sub_epi16(m1[1], m1[3]);
    m2[4] = _mm_add_epi16(m1[4], m1[6]);
    m2[5] = _mm_add_epi16(m1[5], m1[7]);
    m2[6] = _mm_sub_epi16(m1[4], m1[6]);
    m2[7] = _mm_sub_epi16(m1[5], m1[7]);

    m1[0] = _mm_add_epi16(m2[0], m2[1]);
    m1[1] = _mm_sub_epi16(m2[0], m2[1]);
    m1[2] = _mm_add_epi16(m2[2], m2[3]);
    m1[3] = _mm_sub_epi16(m2[2], m2[3]);
    m1[4] = _mm_add_epi16(m2[4], m2[5]);
    m1[5] = _mm_sub_epi16(m2[4], m2[5]);
    m1[6] = _mm_add_epi16(m2[6], m2[7]);
    m1[7] = _mm_sub_epi16(m2[6], m2[7]);

    // horizontal
    // transpose

    m2[0] = _mm_unpacklo_epi16(m1[0], m1[1]);
    m2[1] = _mm_unpacklo_epi16(m1[2], m1[3]);
    m2[2] = _mm_unpacklo_epi16(m1[4], m1[5]);
    m2[3] = _mm_unpacklo_epi16(m1[6], m1[7]);

    m1[0] = _mm_unpacklo_epi32(m2[0], m2[1]);
    m1[1] = _mm_unpackhi_epi32(m2[0], m2[1]);
    m1[2] = _mm_unpacklo_epi32(m2[2], m2[3]);
    m1[3] = _mm_unpackhi_epi32(m2[2], m2[3]);

    m2[0] = _mm_unpacklo_epi64(m1[0], m1[2]);
    m2[1] = _mm_unpackhi_epi64(m1[0], m1[2]);
    m2[2] = _mm_unpacklo_epi64(m1[1], m1[3]);
    m2[3] = _mm_unpackhi_epi64(m1[1], m1[3]);
    
    for(i = 0; i < 4; i++)
    {
        n1[i][0] = _mm_cvtepi16_epi32(m2[i]);
        n1[i][1] = _mm_cvtepi16_epi32(_mm_shuffle_epi32(m2[i], 0xEE));
    }

    for(i = 0; i < 2; i++)
    {
        n2[0][i] = _mm_add_epi32(n1[0][i], n1[2][i]);
        n2[1][i] = _mm_add_epi32(n1[1][i], n1[3][i]);
        n2[2][i] = _mm_sub_epi32(n1[0][i], n1[2][i]);
        n2[3][i] = _mm_sub_epi32(n1[1][i], n1[3][i]);

        n1[0][i] = _mm_abs_epi32(_mm_add_epi32(n2[0][i], n2[1][i]));
        n1[1][i] = _mm_abs_epi32(_mm_sub_epi32(n2[0][i], n2[1][i]));
        n1[2][i] = _mm_abs_epi32(_mm_add_epi32(n2[2][i], n2[3][i]));
        n1[3][i] = _mm_abs_epi32(_mm_sub_epi32(n2[2][i], n2[3][i]));
    }

    s32* p = (s32*)&n1[0][0];
    p[0] = p[0] >> 2;

    for(i = 0; i < 4; i++)
    {
        m1[i] = _mm_add_epi32(n1[i][0], n1[i][1]);
    }

    m1[0] = _mm_add_epi32(m1[0], m1[1]);
    m1[2] = _mm_add_epi32(m1[2], m1[3]);

    sum = _mm_add_epi32(m1[0], m1[2]);

    sum = _mm_hadd_epi32(sum, sum);
    sum = _mm_hadd_epi32(sum, sum);

    satd = _mm_cvtsi128_si32(sum);
    satd = (int)(satd / sqrt(4.0 * 8) * 2);

    return satd;
    }
    else
    {
        int k, i, j, jj;
        int satd = 0;
        int diff[32], m1[8][4], m2[8][4];
        for(k = 0; k < 32; k += 4)
        {
            diff[k + 0] = org[0] - cur[0];
            diff[k + 1] = org[1] - cur[1];
            diff[k + 2] = org[2] - cur[2];
            diff[k + 3] = org[3] - cur[3];
            cur += s_cur;
            org += s_org;
        }
        for(j = 0; j < 8; j++)
        {
            jj = j << 2;
            m2[j][0] = diff[jj] + diff[jj + 2];
            m2[j][1] = diff[jj + 1] + diff[jj + 3];
            m2[j][2] = diff[jj] - diff[jj + 2];
            m2[j][3] = diff[jj + 1] - diff[jj + 3];
            m1[j][0] = m2[j][0] + m2[j][1];
            m1[j][1] = m2[j][0] - m2[j][1];
            m1[j][2] = m2[j][2] + m2[j][3];
            m1[j][3] = m2[j][2] - m2[j][3];
        }
        for(i = 0; i<4; i++)
        {
            m2[0][i] = m1[0][i] + m1[4][i];
            m2[1][i] = m1[1][i] + m1[5][i];
            m2[2][i] = m1[2][i] + m1[6][i];
            m2[3][i] = m1[3][i] + m1[7][i];
            m2[4][i] = m1[0][i] - m1[4][i];
            m2[5][i] = m1[1][i] - m1[5][i];
            m2[6][i] = m1[2][i] - m1[6][i];
            m2[7][i] = m1[3][i] - m1[7][i];
            m1[0][i] = m2[0][i] + m2[2][i];
            m1[1][i] = m2[1][i] + m2[3][i];
            m1[2][i] = m2[0][i] - m2[2][i];
            m1[3][i] = m2[1][i] - m2[3][i];
            m1[4][i] = m2[4][i] + m2[6][i];
            m1[5][i] = m2[5][i] + m2[7][i];
            m1[6][i] = m2[4][i] - m2[6][i];
            m1[7][i] = m2[5][i] - m2[7][i];
            m2[0][i] = m1[0][i] + m1[1][i];
            m2[1][i] = m1[0][i] - m1[1][i];
            m2[2][i] = m1[2][i] + m1[3][i];
            m2[3][i] = m1[2][i] - m1[3][i];
            m2[4][i] = m1[4][i] + m1[5][i];
            m2[5][i] = m1[4][i] - m1[5][i];
            m2[6][i] = m1[6][i] + m1[7][i];
            m2[7][i] = m1[6][i] - m1[7][i];
        }
        satd += EVC_ABS(m2[0][0]) >> 2;
        for(j = 1; j < 4; j++)
        {
            satd += EVC_ABS(m2[0][j]);
        }
        for(i = 1; i < 8; i++)
        {
            for(j = 0; j < 4; j++)
            {
                satd += EVC_ABS(m2[i][j]);
            }
        }
        satd = (int)(satd / sqrt(4.0 * 8.0) * 2.0);
        return satd;
    }
#else
    int k, i, j, jj;
    int satd = 0;
    int diff[32], m1[8][4], m2[8][4];

    for(k = 0; k < 32; k += 4)
    {
        diff[k + 0] = org[0] - cur[0];
        diff[k + 1] = org[1] - cur[1];
        diff[k + 2] = org[2] - cur[2];
        diff[k + 3] = org[3] - cur[3];

        cur += s_cur;
        org += s_org;
    }

    for(j = 0; j < 8; j++)
    {
        jj = j << 2;
        m2[j][0] = diff[jj] + diff[jj + 2];
        m2[j][1] = diff[jj + 1] + diff[jj + 3];
        m2[j][2] = diff[jj] - diff[jj + 2];
        m2[j][3] = diff[jj + 1] - diff[jj + 3];

        m1[j][0] = m2[j][0] + m2[j][1];
        m1[j][1] = m2[j][0] - m2[j][1];
        m1[j][2] = m2[j][2] + m2[j][3];
        m1[j][3] = m2[j][2] - m2[j][3];
    }

    for(i = 0; i<4; i++)
    {
        m2[0][i] = m1[0][i] + m1[4][i];
        m2[1][i] = m1[1][i] + m1[5][i];
        m2[2][i] = m1[2][i] + m1[6][i];
        m2[3][i] = m1[3][i] + m1[7][i];
        m2[4][i] = m1[0][i] - m1[4][i];
        m2[5][i] = m1[1][i] - m1[5][i];
        m2[6][i] = m1[2][i] - m1[6][i];
        m2[7][i] = m1[3][i] - m1[7][i];

        m1[0][i] = m2[0][i] + m2[2][i];
        m1[1][i] = m2[1][i] + m2[3][i];
        m1[2][i] = m2[0][i] - m2[2][i];
        m1[3][i] = m2[1][i] - m2[3][i];
        m1[4][i] = m2[4][i] + m2[6][i];
        m1[5][i] = m2[5][i] + m2[7][i];
        m1[6][i] = m2[4][i] - m2[6][i];
        m1[7][i] = m2[5][i] - m2[7][i];

        m2[0][i] = m1[0][i] + m1[1][i];
        m2[1][i] = m1[0][i] - m1[1][i];
        m2[2][i] = m1[2][i] + m1[3][i];
        m2[3][i] = m1[2][i] - m1[3][i];
        m2[4][i] = m1[4][i] + m1[5][i];
        m2[5][i] = m1[4][i] - m1[5][i];
        m2[6][i] = m1[6][i] + m1[7][i];
        m2[7][i] = m1[6][i] - m1[7][i];
    }

    satd += EVC_ABS(m2[0][0]) >> 2;
    for (j = 1; j < 4; j++)
    {
        satd += EVC_ABS(m2[0][j]);
    }
    for (i = 1; i < 8; i++)
    {
        for (j = 0; j < 4; j++)
        {
            satd += EVC_ABS(m2[i][j]);
        }
    }

    satd = (int)(satd / sqrt(4.0 * 8.0) * 2.0);

    return satd;
#endif
}

int evc_had(int w, int h, void *o, void *c, int s_org, int s_cur, int bit_depth)
{
    pel *org = o;
    pel *cur = c;
    int  x, y;
    int sum = 0;
    int step = 1;

    if(w > h && (h & 7) == 0 && (w & 15) == 0)
    {
        int  offset_org = s_org << 3;
        int  offset_cur = s_cur << 3;

        for(y = 0; y < h; y += 8)
        {
            for(x = 0; x < w; x += 16)
            {
                sum += evc_had_16x8(&org[x], &cur[x], s_org, s_cur, step, bit_depth);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else if(w < h && (w & 7) == 0 && (h & 15) == 0)
    {
        int  offset_org = s_org << 4;
        int  offset_cur = s_cur << 4;

        for(y = 0; y < h; y += 16)
        {
            for(x = 0; x < w; x += 8)
            {
                sum += evc_had_8x16(&org[x], &cur[x], s_org, s_cur, step, bit_depth);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else if(w > h && (h & 3) == 0 && (w & 7) == 0)
    {
        int  offset_org = s_org << 2;
        int  offset_cur = s_cur << 2;

        for(y = 0; y < h; y += 4)
        {
            for(x = 0; x < w; x += 8)
            {
                sum += evc_had_8x4(&org[x], &cur[x], s_org, s_cur, step, bit_depth);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else if(w < h && (w & 3) == 0 && (h & 7) == 0)
    {
        int  offset_org = s_org << 3;
        int  offset_cur = s_cur << 3;

        for(y = 0; y < h; y += 8)
        {
            for(x = 0; x < w; x += 4)
            {
                sum += evc_had_4x8(&org[x], &cur[x], s_org, s_cur, step, bit_depth);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else if((w % 8 == 0) && (h % 8 == 0))
    {
        int  offset_org = s_org << 3;
        int  offset_cur = s_cur << 3;

        for(y = 0; y < h; y += 8)
        {
            for(x = 0; x < w; x += 8)
            {
                sum += evc_had_8x8(&org[x], &cur[x*step], s_org, s_cur, step, bit_depth);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else if((w % 4 == 0) && (h % 4 == 0))
    {
        int  offset_org = s_org << 2;
        int  offset_cur = s_cur << 2;

        for(y = 0; y < h; y += 4)
        {
            for(x = 0; x < w; x += 4)
            {
                sum += evc_had_4x4(&org[x], &cur[x*step], s_org, s_cur, step, bit_depth);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else if((w % 2 == 0) && (h % 2 == 0) )
    {
        int  offset_org = s_org << 1;
        int  offset_cur = s_cur << 1;

        for(y = 0; y < h; y +=2)
        {
            for(x = 0; x < w; x += 2)
            {
                sum += evc_had_2x2(&org[x], &cur[x*step], s_org, s_cur, step);
            }
            org += offset_org;
            cur += offset_cur;
        }
    }
    else
    {
        evc_assert(0);
    }
#if FULL_BITDEPTH_RDO
    return (sum);
#else
    return (sum >> (bit_depth - 8));
#endif
}

/* index: [log2 of width][log2 of height] */
const EVCE_FN_SATD evce_tbl_satd_16b[8][8] =
{
    /* width == 1 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 2 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 4 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 8 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 16 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 32 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 64 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    },
    /* width == 128 */
    {
        evc_had, /* height == 1 */
        evc_had, /* height == 2 */
        evc_had, /* height == 4 */
        evc_had, /* height == 8 */
        evc_had, /* height == 16 */
        evc_had, /* height == 32 */
        evc_had, /* height == 64 */
        evc_had, /* height == 128 */
    }
};
