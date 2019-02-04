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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "AdaptiveLoopFilter.h"
#if ALF
extern "C" 
{

void portAlfFilterShape(evc_AlfFilterShape* p0, AlfFilterShape* p1)
{
    p0->filterLength = p1->filterLength;
    p0->filterSize = p1->filterSize;
    p0->numCoeff = p1->numCoeff;
    p0->filterType = p1->filterType;

    for(int i=0; i < p1->golombIdx.size(); i++)
        p0->golombIdx[i] = p1->golombIdx[i];
    for(int i=0; i < p1->pattern.size(); i++)
        p0->pattern[i] = p1->pattern[i];
    for(int i=0; i < p1->patternToLargeFilter.size(); i++)
        p0->patternToLargeFilter[i] = p1->patternToLargeFilter[i];
    for(int i=0; i < p1->weights.size(); i++)
        p0->weights[i] = p1->weights[i];
}

AlfFilterShape* new_AlfFilterShape(int size)
{
    return new AlfFilterShape(size);
}

void delete_AlfFilterShape(AlfFilterShape* p)
{
    delete p;
}

AdaptiveLoopFilter* new_ALF()
{
    return new AdaptiveLoopFilter;
}

void delete_ALF(AdaptiveLoopFilter* p)
{
    delete p;
}

void call_create_ALF(AdaptiveLoopFilter* p, const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth)
{
    p->create( picWidth, picHeight, maxCUWidth, maxCUHeight, maxCUDepth );
}

void call_destroy_ALF(AdaptiveLoopFilter* p)
{
    p->destroy();
}

void call_ALFProcess(AdaptiveLoopFilter* p, EVCD_CTX * ctx, EVC_PIC * pic)
{
    CodingStructure cs;
    cs.pCtx = (void*)ctx;
    cs.pPic = pic;

    AlfTileGroupParam alfTileGroupParam;
    evc_AlfTileGroupParam iAlfTileGroupParam = ctx->tgh.alf_tgh_param;

    //port

    alfTileGroupParam.enabledFlag[COMPONENT_Y]  = bool( iAlfTileGroupParam.enabledFlag[0] );
    alfTileGroupParam.enabledFlag[COMPONENT_Cb] = bool( iAlfTileGroupParam.enabledFlag[1] );
    alfTileGroupParam.enabledFlag[COMPONENT_Cr] = bool( iAlfTileGroupParam.enabledFlag[2] );

    alfTileGroupParam.numLumaFilters = iAlfTileGroupParam.numLumaFilters;
    alfTileGroupParam.lumaFilterType = AlfFilterType( iAlfTileGroupParam.lumaFilterType );

    memcpy(alfTileGroupParam.filterCoeffDeltaIdx, iAlfTileGroupParam.filterCoeffDeltaIdx, MAX_NUM_ALF_CLASSES*sizeof(short));
    memcpy(alfTileGroupParam.lumaCoeff,           iAlfTileGroupParam.lumaCoeff,           sizeof(short)*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_LUMA_COEFF);
    memcpy(alfTileGroupParam.chromaCoeff,         iAlfTileGroupParam.chromaCoeff,         sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
    memcpy(alfTileGroupParam.fixedFilterIdx,      iAlfTileGroupParam.fixedFilterIdx,      MAX_NUM_ALF_CLASSES*sizeof(int));

    alfTileGroupParam.fixedFilterPattern = iAlfTileGroupParam.fixedFilterPattern;

    alfTileGroupParam.coeffDeltaFlag         = bool( iAlfTileGroupParam.coeffDeltaFlag );
    alfTileGroupParam.coeffDeltaPredModeFlag = bool( iAlfTileGroupParam.coeffDeltaPredModeFlag );

    //bool is not a BOOL
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
        alfTileGroupParam.filterCoeffFlag[i] = bool( iAlfTileGroupParam.filterCoeffFlag[i] );
    }

    alfTileGroupParam.prevIdx = iAlfTileGroupParam.prevIdx;
    alfTileGroupParam.tLayer  = iAlfTileGroupParam.tLayer;
    alfTileGroupParam.temporalAlfFlag = bool( iAlfTileGroupParam.temporalAlfFlag );
    const unsigned tidx = ctx->tgh.layer_id;
    alfTileGroupParam.store2ALFBufferFlag = bool( iAlfTileGroupParam.store2ALFBufferFlag );
    alfTileGroupParam.resetALFBufferFlag = bool( iAlfTileGroupParam.resetALFBufferFlag );
    if (alfTileGroupParam.resetALFBufferFlag )
    {
        p->resetTemporalAlfBufferLine();
    }

    if (alfTileGroupParam.enabledFlag[COMPONENT_Y] && alfTileGroupParam.temporalAlfFlag)
    {
      p->loadALFParamLine(&(alfTileGroupParam), alfTileGroupParam.prevIdx, tidx);
    }
 // consistency of merge
    p->m_pendingRasInit = false;
    if( ctx->ptr > p->m_lastRasPoc )
    {
        p->m_lastRasPoc = INT_MAX;
        p->m_pendingRasInit = true;
    }
    if( ctx->tgh.tile_group_type == TILE_GROUP_I )
        p->m_lastRasPoc = ctx->ptr;

    if( p->m_pendingRasInit )
        p->resetTemporalAlfBufferLine2First();
//<-- end of consistency of merge

    alfTileGroupParam.isCtbAlfOn = bool(iAlfTileGroupParam.isCtbAlfOn == 1 ? true : false);
    memcpy(alfTileGroupParam.alfCtuEnableFlag, iAlfTileGroupParam.alfCtuEnableFlag, 3*512 * sizeof(u8));

    p->ALFProcess( cs, alfTileGroupParam );

    if (alfTileGroupParam.enabledFlag[COMPONENT_Y] && alfTileGroupParam.store2ALFBufferFlag )
    {
        p->storeALFParamLine(&(alfTileGroupParam), tidx);
    }

}

} //<-- extern "C"




AdaptiveLoopFilter::AdaptiveLoopFilter()
  : m_classifier(nullptr)
{
  for (int i = 0; i < NUM_DIRECTIONS; i++)
  {
    m_laplacian[i] = nullptr;
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_ctuEnableFlag[compIdx] = nullptr;
  }

  m_deriveClassificationBlk = deriveClassificationBlk;
  m_filter5x5Blk = filterBlk<ALF_FILTER_5>;
  m_filter7x7Blk = filterBlk<ALF_FILTER_7>;

#if ENABLE_SIMD_OPT_ALF
#ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
#endif
#endif
}



const int AdaptiveLoopFilter::m_fixedFilterCoeff[m_fixedFilterNum][13] =
{
  { 0,   2,   7, -12,  -4, -11,  -2,  31,  -9,   6,  -4,  30, 444 - (1 << (m_NUM_BITS - 1)) },
  { -26,   4,  17,  22,  -7,  19,  40,  47,  49, -28,  35,  48,  72 - (1 << (m_NUM_BITS - 1)) },
  { -24,  -8,  30,  64, -13,  18,  18,  27,  80,   0,  31,  19,  28 - (1 << (m_NUM_BITS - 1)) },
  { -4, -14,  44, 100,  -7,   6,  -4,   8,  90,  26,  26, -12,  -6 - (1 << (m_NUM_BITS - 1)) },
  { -17,  -9,  23,  -3, -15,  20,  53,  48,  16, -25,  42,  66, 114 - (1 << (m_NUM_BITS - 1)) },
  { -12,  -2,   1, -19,  -5,   8,  66,  80,  -2, -25,  20,  78, 136 - (1 << (m_NUM_BITS - 1)) },
  { 2,   8, -23, -14,  -3, -23,  64,  86,  35, -17,  -4,  79, 132 - (1 << (m_NUM_BITS - 1)) },
  { 12,   4, -39,  -7,   1, -20,  78,  13,  -8,  11, -42,  98, 310 - (1 << (m_NUM_BITS - 1)) },
  { 0,   3,  -4,   0,   2,  -7,   6,   0,   0,   3,  -8,  11, 500 - (1 << (m_NUM_BITS - 1)) },
  { 4,  -7, -25, -19,  -9,   8,  86,  65, -14,  -7,  -7,  97, 168 - (1 << (m_NUM_BITS - 1)) },
  { 3,   3,   2, -30,   6, -34,  43,  71, -10,   4, -23,  77, 288 - (1 << (m_NUM_BITS - 1)) },
  { 12,  -3, -34, -14,  -5, -14,  88,  28, -12,   8, -34, 112, 248 - (1 << (m_NUM_BITS - 1)) },
  { -1,   6,   8, -29,   7, -27,  15,  60,  -4,   6, -21,  39, 394 - (1 << (m_NUM_BITS - 1)) },
  { 8,  -1,  -7, -22,   5, -41,  63,  40, -13,   7, -28, 105, 280 - (1 << (m_NUM_BITS - 1)) },
  { 1,   3,  -5,  -1,   1, -10,  12,  -1,   0,   3,  -9,  19, 486 - (1 << (m_NUM_BITS - 1)) },
  { 10,  -1, -23, -14,  -3, -27,  78,  24, -14,   8, -28, 102, 288 - (1 << (m_NUM_BITS - 1)) },
  { 0,   0,  -1,   0,   0,  -1,   1,   0,   0,   0,   0,   1, 512 - (1 << (m_NUM_BITS - 1)) },
  { 7,   3, -19,  -7,   2, -27,  51,   8,  -6,   7, -24,  64, 394 - (1 << (m_NUM_BITS - 1)) },
  { 11, -10, -22, -22, -11, -12,  87,  49, -20,   4, -16, 108, 220 - (1 << (m_NUM_BITS - 1)) },
  { 17,  -2, -69,  -4,  -4,  22, 106,  31,  -7,  13, -63, 121, 190 - (1 << (m_NUM_BITS - 1)) },
  { 1,   4,  -1,  -7,   5, -26,  24,   0,   1,   3, -18,  51, 438 - (1 << (m_NUM_BITS - 1)) },
  { 3,   5, -10,  -2,   4, -17,  17,   1,  -2,   6, -16,  27, 480 - (1 << (m_NUM_BITS - 1)) },
  { 9,   2, -23,  -5,   6, -45,  90, -22,   1,   7, -39, 121, 308 - (1 << (m_NUM_BITS - 1)) },
  { 4,   5, -15,  -2,   4, -22,  34,  -2,  -2,   7, -22,  48, 438 - (1 << (m_NUM_BITS - 1)) },
  { 6,   8, -22,  -3,   4, -32,  57,  -3,  -4,  11, -43, 102, 350 - (1 << (m_NUM_BITS - 1)) },
  { 2,   5, -11,   1,  12, -46,  64, -32,   7,   4, -31,  85, 392 - (1 << (m_NUM_BITS - 1)) },
  { 5,   5, -12,  -8,   6, -48,  74, -13,  -1,   7, -41, 129, 306 - (1 << (m_NUM_BITS - 1)) },
  { 0,   1,  -1,   0,   1,  -3,   2,   0,   0,   1,  -3,   4, 508 - (1 << (m_NUM_BITS - 1)) },
  { -1,   3,  16, -42,   6, -16,   2, 105,   6,   6, -31,  43, 318 - (1 << (m_NUM_BITS - 1)) },
  { 7,   8, -27,  -4,  -4, -23,  46,  79,  64,  -8, -13,  68, 126 - (1 << (m_NUM_BITS - 1)) },
  { -3,  12,  -4, -34,  14,  -6, -24, 179,  56,   2, -48,  15, 194 - (1 << (m_NUM_BITS - 1)) },
  { 8,   0, -16, -25,  -1, -29,  68,  84,   3,  -3, -18,  94, 182 - (1 << (m_NUM_BITS - 1)) },
  { -3,  -1,  22, -32,   2, -20,   5,  89,   0,   9, -18,  40, 326 - (1 << (m_NUM_BITS - 1)) },
  { 14,   6, -51,  22, -10, -22,  36,  75, 106,  -4, -11,  56,  78 - (1 << (m_NUM_BITS - 1)) },
  { 1,  38, -59,  14,   8, -44, -18, 156,  80,  -1, -42,  29, 188 - (1 << (m_NUM_BITS - 1)) },
  { -1,   2,   4,  -9,   3, -13,   7,  17,  -4,   2,  -6,  17, 474 - (1 << (m_NUM_BITS - 1)) },
  { 11,  -2, -15, -36,   2, -32,  67,  89, -19,  -1, -14, 103, 206 - (1 << (m_NUM_BITS - 1)) },
  { -1,  10,   3, -28,   7, -27,   7, 117,  34,   1, -35,  51, 234 - (1 << (m_NUM_BITS - 1)) },
  { 3,   3,   4, -18,   6, -40,  36,  18,  -8,   7, -25,  86, 368 - (1 << (m_NUM_BITS - 1)) },
  { -1,   3,   9, -18,   5, -26,  12,  37, -11,   3,  -7,  32, 436 - (1 << (m_NUM_BITS - 1)) },
  { 0,  17, -38,  -9, -28, -17,  25,  48, 103,   2,  40,  69,  88 - (1 << (m_NUM_BITS - 1)) },
  { 6,   4, -11, -20,   5, -32,  51,  77,  17,   0, -25,  84, 200 - (1 << (m_NUM_BITS - 1)) },
  { 0,  -5,  28, -24,  -1, -22,  18,  -9,  17,  -1, -12, 107, 320 - (1 << (m_NUM_BITS - 1)) },
  { -10,  -4,  17, -30, -29,  31,  40,  49,  44, -26,  67,  67,  80 - (1 << (m_NUM_BITS - 1)) },
  { -30, -12,  39,  15, -21,  32,  29,  26,  71,  20,  43,  28,  32 - (1 << (m_NUM_BITS - 1)) },
  { 6,  -7,  -7, -34, -21,  15,  53,  60,  12, -26,  45,  89, 142 - (1 << (m_NUM_BITS - 1)) },
  { -1,  -5,  59, -58,  -8, -30,   2,  17,  34,  -7,  25, 111, 234 - (1 << (m_NUM_BITS - 1)) },
  { 7,   1,  -7, -20,  -9, -22,  48,  27,  -4,  -6,   0, 107, 268 - (1 << (m_NUM_BITS - 1)) },
  { -2,  22,  29, -70,  -4, -28,   2,  19,  94, -40,  14, 110, 220 - (1 << (m_NUM_BITS - 1)) },
  { 13,   0, -22, -27, -11, -15,  66,  44,  -7,  -5, -10, 121, 218 - (1 << (m_NUM_BITS - 1)) },
  { 10,   6, -22, -14,  -2, -33,  68,  15,  -9,   5, -35, 135, 264 - (1 << (m_NUM_BITS - 1)) },
  { 2,  11,   4, -32,  -3, -20,  23,  18,  17,  -1, -28,  88, 354 - (1 << (m_NUM_BITS - 1)) },
  { 0,   3,  -2,  -1,   3, -16,  16,  -3,   0,   2, -12,  35, 462 - (1 << (m_NUM_BITS - 1)) },
  { 1,   6,  -6,  -3,  10, -51,  70, -31,   5,   6, -42, 125, 332 - (1 << (m_NUM_BITS - 1)) },
  { 5,  -7,  61, -71, -36,  -6,  -2,  15,  57,  18,  14, 108, 200 - (1 << (m_NUM_BITS - 1)) },
  { 9,   1,  35, -70, -73,  28,  13,   1,  96,  40,  36,  80, 120 - (1 << (m_NUM_BITS - 1)) },
  { 11,  -7,  33, -72, -78,  48,  33,  37,  35,   7,  85,  76,  96 - (1 << (m_NUM_BITS - 1)) },
  { 4,  15,   1, -26, -24, -19,  32,  29,  -8,  -6,  21, 125, 224 - (1 << (m_NUM_BITS - 1)) },
  { 11,   8,  14, -57, -63,  21,  34,  51,   7,  -3,  69,  89, 150 - (1 << (m_NUM_BITS - 1)) },
  { 7,  16,  -7, -31, -38,  -5,  41,  44, -11, -10,  45, 109, 192 - (1 << (m_NUM_BITS - 1)) },
  { 5,  16,  16, -46, -55,   3,  22,  32,  13,   0,  48, 107, 190 - (1 << (m_NUM_BITS - 1)) },
  { 2,  10,  -3, -14,  -9, -28,  39,  15, -10,  -5,  -1, 123, 274 - (1 << (m_NUM_BITS - 1)) },
  { 3,  11,  11, -27, -17, -24,  18,  22,   2,   4,   3, 100, 300 - (1 << (m_NUM_BITS - 1)) },
  { 0,   1,   7,  -9,   3, -20,  16,   3,  -2,   0,  -9,  61, 410 - (1 << (m_NUM_BITS - 1)) },
};
const int AdaptiveLoopFilter::m_classToFilterMapping[MAX_NUM_ALF_CLASSES][ALF_FIXED_FILTER_NUM] =
{
  { 0,   1,   2,   3,   4,   5,   6,   7,   9,  19,  32,  41,  42,  44,  46,  63 },
  { 0,   1,   2,   4,   5,   6,   7,   9,  11,  16,  25,  27,  28,  31,  32,  47 },
  { 5,   7,   9,  11,  12,  14,  15,  16,  17,  18,  19,  21,  22,  27,  31,  35 },
  { 7,   8,   9,  11,  14,  15,  16,  17,  18,  19,  22,  23,  24,  25,  35,  36 },
  { 7,   8,  11,  13,  14,  15,  16,  17,  19,  20,  21,  22,  23,  24,  25,  27 },
  { 1,   2,   3,   4,   6,  19,  29,  30,  33,  34,  37,  41,  42,  44,  47,  54 },
  { 1,   2,   3,   4,   6,  11,  28,  29,  30,  31,  32,  33,  34,  37,  47,  63 },
  { 0,   1,   4,   6,  10,  12,  13,  19,  28,  29,  31,  32,  34,  35,  36,  37 },
  { 6,   9,  10,  12,  13,  16,  19,  20,  28,  31,  35,  36,  37,  38,  39,  52 },
  { 7,   8,  10,  11,  12,  13,  19,  23,  25,  27,  28,  31,  35,  36,  38,  39 },
  { 1,   2,   3,   5,  29,  30,  33,  34,  40,  43,  44,  46,  54,  55,  59,  62 },
  { 1,   2,   3,   4,  29,  30,  31,  33,  34,  37,  40,  41,  43,  44,  59,  61 },
  { 0,   1,   3,   6,  19,  28,  29,  30,  31,  32,  33,  34,  37,  41,  44,  61 },
  { 1,   6,  10,  13,  19,  28,  29,  30,  32,  33,  34,  35,  37,  41,  48,  52 },
  { 0,   5,   6,  10,  19,  27,  28,  29,  32,  37,  38,  40,  41,  47,  49,  58 },
  { 1,   2,   3,   4,  11,  29,  33,  42,  43,  44,  45,  46,  48,  55,  56,  59 },
  { 0,   1,   2,   5,   7,   9,  29,  40,  43,  44,  45,  47,  48,  56,  59,  63 },
  { 0,   4,   5,   9,  14,  19,  26,  35,  36,  43,  45,  47,  48,  49,  50,  51 },
  { 9,  11,  12,  14,  16,  19,  20,  24,  26,  36,  38,  47,  49,  50,  51,  53 },
  { 7,   8,  13,  14,  20,  21,  24,  25,  26,  27,  35,  38,  47,  50,  52,  53 },
  { 1,   2,   4,  29,  33,  40,  41,  42,  43,  44,  45,  46,  54,  55,  56,  58 },
  { 2,   4,  32,  40,  42,  43,  44,  45,  46,  54,  55,  56,  58,  59,  60,  62 },
  { 0,  19,  42,  43,  45,  46,  48,  54,  55,  56,  57,  58,  59,  60,  61,  62 },
  { 8,  13,  36,  42,  45,  46,  51,  53,  54,  57,  58,  59,  60,  61,  62,  63 },
  { 8,  13,  20,  27,  36,  38,  42,  46,  52,  53,  56,  57,  59,  61,  62,  63 },
};

void AdaptiveLoopFilter::ALFProcess(CodingStructure& cs, AlfTileGroupParam& alfTileGroupParam)
{
  if (!alfTileGroupParam.enabledFlag[COMPONENT_Y] && !alfTileGroupParam.enabledFlag[COMPONENT_Cb] && !alfTileGroupParam.enabledFlag[COMPONENT_Cr])
  {
    return;
  }

  EVCD_CTX* ctx = (EVCD_CTX*)(cs.pCtx);

  // set available filter shapes
  alfTileGroupParam.filterShapes = m_filterShapes;

  // set CTU enable flags
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
      m_ctuEnableFlag[compIdx] = &(alfTileGroupParam.alfCtuEnableFlag[compIdx][0]);
  }



  reconstructCoeff(alfTileGroupParam, CHANNEL_TYPE_LUMA, false, true);
//  reconstructCoeff(alfTileGroupParam, CHANNEL_TYPE_CHROMA, false);


  const int h = cs.pPic->h_l;
  const int w = cs.pPic->w_l;
  const int m = MAX_ALF_FILTER_LENGTH >> 1;
  const int s = w + m + m;

  pel * recYuv = cs.pPic->y;
  pel * tmpYuv = m_tempBuf + s*m + m;

  //copy
  for(int j = 0; j < h; j++ )
      memcpy( tmpYuv + j*s, recYuv + j*cs.pPic->s_l, sizeof(pel)*w );

  //extend

  pel * p = tmpYuv;
  // do left and right margins
  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < m; x++ )
    {
      *( p - m + x ) = p[0];
      p[w + x] = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= ( s + 3 );
  // p is now the (-margin, height-1)
  for( int y = 0; y < m; y++ )
  {
    memcpy( p + ( y + 1 ) * s, p, sizeof( pel ) * ( w + ( m << 1 ) ) );
  }

  // pi is still (-marginX, height-1)
  p -= ( ( h - 1 ) * s );
  // pi is now (-marginX, 0)
  for( int y = 0; y < 3; y++ )
  {
    memcpy( p - ( y + 1 ) * s, p, sizeof( pel ) * ( w + ( m << 1 ) ) );
  }

  // <-- end of extend

  //m_tempBuf.copyFrom(recYuv);
  //PelUnitBuf tmpYuv = m_tempBuf.getBuf(cs.area);
  //const PreCalcValues& pcv = *cs.pcv;
  
  int ctuIdx = 0;
  for (int yPos = 0; yPos < cs.pPic->h_l; yPos += ctx->max_cuwh )
  {
    for (int xPos = 0; xPos < cs.pPic->w_l; xPos += ctx->max_cuwh )
    {
      const int width = (xPos + ctx->max_cuwh > cs.pPic->w_l) ? (cs.pPic->w_l - xPos) : ctx->max_cuwh;
      const int height = (yPos + ctx->max_cuwh > cs.pPic->h_l) ? (cs.pPic->h_l - yPos) : ctx->max_cuwh;
//      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
      {
        Area blk(xPos, yPos, width, height);
        {
          deriveClassification(m_classifier, tmpYuv, s, blk);
          m_filter7x7Blk(m_classifier, recYuv, cs.pPic->s_l, tmpYuv, s, blk, COMPONENT_Y, m_coeffFinal, m_clpRngs.comp[COMPONENT_Y]);
        }
      }

      for (int compIdx = MAX_NUM_COMPONENT; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        ComponentID compID = ComponentID(compIdx);
        const int chromaScaleX = 1; //getComponentScaleX(compID, tmpYuv.chromaFormat);
        const int chromaScaleY = 1; //getComponentScaleY(compID, tmpYuv.chromaFormat);

        if (m_ctuEnableFlag[compIdx][ctuIdx])
        {
          Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          m_filter5x5Blk(m_classifier, recYuv, cs.pPic->s_l, tmpYuv, cs.pPic->s_l, blk, compID, alfTileGroupParam.chromaCoeff, m_clpRngs.comp[compIdx]);
        }
      }
      ctuIdx++;
    }
  }

}


void AdaptiveLoopFilter::reconstructCoeff(AlfTileGroupParam& alfTileGroupParam, ChannelType channel,
  const bool bRdo,
  const bool bRedo)
{
  int factor = bRdo ? 0 : (1 << (m_NUM_BITS - 1));
  AlfFilterType filterType = channel == CHANNEL_TYPE_LUMA ? alfTileGroupParam.lumaFilterType : ALF_FILTER_5;
  int numClasses = channel == CHANNEL_TYPE_LUMA ? MAX_NUM_ALF_CLASSES : 1;
  int numCoeff = filterType == ALF_FILTER_5 ? 7 : 13;
  int numCoeffMinus1 = numCoeff - 1;
  int numFilters = channel == CHANNEL_TYPE_LUMA ? alfTileGroupParam.numLumaFilters : 1;
  short* coeff = channel == CHANNEL_TYPE_LUMA ? alfTileGroupParam.lumaCoeff : alfTileGroupParam.chromaCoeff;

  if (alfTileGroupParam.coeffDeltaPredModeFlag && channel == CHANNEL_TYPE_LUMA)
  {
    for (int i = 1; i < numFilters; i++)
    {
      for (int j = 0; j < numCoeffMinus1; j++)
      {
        coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += coeff[(i - 1) * MAX_NUM_ALF_LUMA_COEFF + j];
      }
    }
  }


  if (isChroma(channel))
  {
    for (int filterIdx = 0; filterIdx < numFilters; filterIdx++)
    {
      int sum = 0;
      for (int i = 0; i < numCoeffMinus1; i++)
      {
        sum += (coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + i] << 1);
      }
      coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor - sum;
    }
    return;
  }

  memset(m_coeffFinal, 0, sizeof(m_coeffFinal));
  int numCoeffLargeMinus1 = MAX_NUM_ALF_LUMA_COEFF - 1;
  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {
    int filterIdx = alfTileGroupParam.filterCoeffDeltaIdx[classIdx];
    int fixedFilterIdx = alfTileGroupParam.fixedFilterIdx[classIdx];
    if (fixedFilterIdx > 0)
    {
      fixedFilterIdx = m_classToFilterMapping[classIdx][fixedFilterIdx - 1];
    }

    for (int i = 0; i < numCoeffLargeMinus1; i++)
    {
      int curCoeff = 0;
      //fixed filter
      if (fixedFilterIdx > 0)
      {
        curCoeff = m_fixedFilterCoeff[fixedFilterIdx][i];
      }
      //add coded coeff
      if (m_filterShapes[CHANNEL_TYPE_LUMA][filterType].patternToLargeFilter[i] > 0)
      {
        int coeffIdx = m_filterShapes[CHANNEL_TYPE_LUMA][filterType].patternToLargeFilter[i] - 1;
        curCoeff += coeff[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx];
      }
      m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] = curCoeff;
    }

    //last coeff
    int sum = 0;
    for (int i = 0; i < numCoeffLargeMinus1; i++)
    {
      sum += (m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] << 1);
    }
    m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffLargeMinus1] = factor - sum;
    }


  if (bRedo && alfTileGroupParam.coeffDeltaPredModeFlag)
  {
    for (int i = numFilters - 1; i > 0; i--)
    {
      for (int j = 0; j < numCoeffMinus1; j++)
      {
        coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] = coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] - coeff[(i - 1) * MAX_NUM_ALF_LUMA_COEFF + j];
      }
    }
  }
}

void AdaptiveLoopFilter::create(const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth)
{
  const ChromaFormat format = CHROMA_420;
  const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] = {10, 10};

  std::memcpy(m_inputBitDepth, inputBitDepth, sizeof(m_inputBitDepth));
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_maxCUDepth = maxCUDepth;
  m_chromaFormat = format;

  m_numCTUsInWidth = (m_picWidth / m_maxCUWidth) + ((m_picWidth % m_maxCUWidth) ? 1 : 0);
  m_numCTUsInHeight = (m_picHeight / m_maxCUHeight) + ((m_picHeight % m_maxCUHeight) ? 1 : 0);
  m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;

  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(5));
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(7));
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back(AlfFilterShape(5));


  m_tempBuf = new pel[(picWidth + 7)*(picHeight + 7)]; // +7 is of filter diameter //todo: check this

  // Laplacian based activity
  for (int i = 0; i < NUM_DIRECTIONS; i++)
  {
    m_laplacian[i] = new int*[m_CLASSIFICATION_BLK_SIZE + 5];

    for (int y = 0; y < m_CLASSIFICATION_BLK_SIZE + 5; y++)
    {
      m_laplacian[i][y] = new int[m_CLASSIFICATION_BLK_SIZE + 5];
    }
  }
  // Classification
  m_classifier = new AlfClassifier*[picHeight];
  for (int i = 0; i < picHeight; i++)
  {
    m_classifier[i] = new AlfClassifier[picWidth];
  }

}

void AdaptiveLoopFilter::destroy()
{

  m_filterShapes[CHANNEL_TYPE_LUMA].clear();
  m_filterShapes[CHANNEL_TYPE_LUMA].clear();
  m_filterShapes[CHANNEL_TYPE_CHROMA].clear();

  delete [] m_tempBuf;

  for (int i = 0; i < NUM_DIRECTIONS; i++)
  {
    if (m_laplacian[i])
    {
      for (int y = 0; y < m_CLASSIFICATION_BLK_SIZE + 5; y++)
      {
        delete[] m_laplacian[i][y];
        m_laplacian[i][y] = nullptr;
      }

      delete[] m_laplacian[i];
      m_laplacian[i] = nullptr;
    }
  }

  if (m_classifier)
  {
    for (int i = 0; i < m_picHeight; i++)
    {
      delete[] m_classifier[i];
      m_classifier[i] = nullptr;
    }

    delete[] m_classifier;
    m_classifier = nullptr;
  }

}

void AdaptiveLoopFilter::deriveClassification(AlfClassifier** classifier, const pel * srcLuma, const int srcLumaStride, const Area& blk)
{
  int height = blk.pos().y + blk.height;
  int width = blk.pos().x + blk.width;

  for (int i = blk.pos().y; i < height; i += m_CLASSIFICATION_BLK_SIZE)
  {
    int nHeight = std::min(i + m_CLASSIFICATION_BLK_SIZE, height) - i;

    for (int j = blk.pos().x; j < width; j += m_CLASSIFICATION_BLK_SIZE)
    {
      int nWidth = std::min(j + m_CLASSIFICATION_BLK_SIZE, width) - j;

      m_deriveClassificationBlk(classifier, m_laplacian, srcLuma, srcLumaStride, Area(j, i, nWidth, nHeight), m_inputBitDepth[CHANNEL_TYPE_LUMA] + 4);
    }
  }
}

void AdaptiveLoopFilter::deriveClassificationBlk(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const pel * srcLuma, const int srcStride,  const Area& blk, const int shift)
{
  static const int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  const int stride = srcStride;
  const Pel* src = srcLuma;
  const int maxActivity = 15;

  int fl = 2;
  int flP1 = fl + 1;
  int fl2 = 2 * fl;

  int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  int pixY;
  int height = blk.height + fl2;
  int width = blk.width + fl2;
  int posX = blk.pos().x;
  int posY = blk.pos().y;
  int startHeight = posY - flP1;

  for (int i = 0; i < height; i += 2)
  {
    int yoffset = (i + 1 + startHeight) * stride - flP1;
    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];

    int* pYver = laplacian[VER][i];
    int* pYhor = laplacian[HOR][i];
    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig1 = laplacian[DIAG1][i];

    for (int j = 0; j < width; j += 2)
    {
      pixY = j + 1 + posX;
      const Pel *pY = src1 + pixY;
      const Pel* pYdown = src0 + pixY;
      const Pel* pYup = src2 + pixY;
      const Pel* pYup2 = src3 + pixY;

      const Pel y0 = pY[0] << 1;
      const Pel y1 = pY[1] << 1;
      const Pel yup0 = pYup[0] << 1;
      const Pel yup1 = pYup[1] << 1;

      pYver[j] = abs(y0 - pYdown[0] - pYup[0]) + abs(y1 - pYdown[1] - pYup[1]) + abs(yup0 - pY[0] - pYup2[0]) + abs(yup1 - pY[1] - pYup2[1]);
      pYhor[j] = abs(y0 - pY[1] - pY[-1]) + abs(y1 - pY[2] - pY[0]) + abs(yup0 - pYup[1] - pYup[-1]) + abs(yup1 - pYup[2] - pYup[0]);
      pYdig0[j] = abs(y0 - pYdown[-1] - pYup[1]) + abs(y1 - pYdown[0] - pYup[2]) + abs(yup0 - pY[-1] - pYup2[1]) + abs(yup1 - pY[0] - pYup2[2]);
      pYdig1[j] = abs(y0 - pYup[-1] - pYdown[1]) + abs(y1 - pYup[0] - pYdown[2]) + abs(yup0 - pYup2[-1] - pY[1]) + abs(yup1 - pYup2[0] - pY[2]);

      if (j > 4 && (j - 6) % 4 == 0)
      {
        int jM6 = j - 6;
        int jM4 = j - 4;
        int jM2 = j - 2;

        pYver[jM6] += pYver[jM4] + pYver[jM2] + pYver[j];
        pYhor[jM6] += pYhor[jM4] + pYhor[jM2] + pYhor[j];
        pYdig0[jM6] += pYdig0[jM4] + pYdig0[jM2] + pYdig0[j];
        pYdig1[jM6] += pYdig1[jM4] + pYdig1[jM2] + pYdig1[j];
      }
    }
  }

  // classification block size
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  for (int i = 0; i < blk.height; i += clsSizeY)
  {
    int* pYver = laplacian[VER][i];
    int* pYver2 = laplacian[VER][i + 2];
    int* pYver4 = laplacian[VER][i + 4];
    int* pYver6 = laplacian[VER][i + 6];

    int* pYhor = laplacian[HOR][i];
    int* pYhor2 = laplacian[HOR][i + 2];
    int* pYhor4 = laplacian[HOR][i + 4];
    int* pYhor6 = laplacian[HOR][i + 6];

    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig02 = laplacian[DIAG0][i + 2];
    int* pYdig04 = laplacian[DIAG0][i + 4];
    int* pYdig06 = laplacian[DIAG0][i + 6];

    int* pYdig1 = laplacian[DIAG1][i];
    int* pYdig12 = laplacian[DIAG1][i + 2];
    int* pYdig14 = laplacian[DIAG1][i + 4];
    int* pYdig16 = laplacian[DIAG1][i + 6];

    for (int j = 0; j < blk.width; j += clsSizeX)
    {
      int sumV = pYver[j] + pYver2[j] + pYver4[j] + pYver6[j];
      int sumH = pYhor[j] + pYhor2[j] + pYhor4[j] + pYhor6[j];
      int sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j] + pYdig06[j];
      int sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j] + pYdig16[j];

      int tempAct = sumV + sumH;
      int activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 32) >> shift);
      int classIdx = th[activity];

      int hv1, hv0, d1, d0, hvd1, hvd0;

      if (sumV > sumH)
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }
      if (sumD0 > sumD1)
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }
      if (d1*hv0 > hv1*d0)
      {
        hvd1 = d1;
        hvd0 = d0;
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        hvd1 = hv1;
        hvd0 = hv0;
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
      }

      int directionStrength = 0;
      if (hvd1 > 2 * hvd0)
      {
        directionStrength = 1;
      }
      if (hvd1 * 2 > 9 * hvd0)
      {
        directionStrength = 2;
      }

      if (directionStrength)
      {
        classIdx += (((mainDirection & 0x1) << 1) + directionStrength) * 5;
      }

      static const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + (secondaryDirection >> 1)];

      int yOffset = i + posY;
      int xOffset = j + posX;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;
      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] = cl3[0] = cl3[1] = cl3[2] = cl3[3] = ( (classIdx << 2) + transposeIdx ) & 0xFF;
    }
  }
}

template<AlfFilterType filtType>
void AdaptiveLoopFilter::filterBlk(AlfClassifier** classifier, pel * recDst, const int dstStride, const pel* recSrc, const int srcStride, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng)
{
  const bool bChroma = ( compId != COMPONENT_Y );
  if (bChroma)
  {
    CHECK(filtType != 0, "Chroma needs to have filtType == 0");
  }

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = recSrc;
  Pel* dst = recDst + startHeight * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

  short *coef = filterSet;

  const int shift = 9;
  const int offset = 1 << (shift - 1);

  int transposeIdx = 0;
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  CHECK(startHeight % clsSizeY, "Wrong startHeight in filtering");
  CHECK(startWidth % clsSizeX, "Wrong startWidth in filtering");
  CHECK((endHeight - startHeight) % clsSizeY, "Wrong endHeight in filtering");
  CHECK((endWidth - startWidth) % clsSizeX, "Wrong endWidth in filtering");

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  std::vector<Pel> filterCoeff(MAX_NUM_ALF_LUMA_COEFF);

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;

  Pel* pRec0 = dst + startWidth;
  Pel* pRec1 = pRec0 + dstStride;

  for (int i = 0; i < endHeight - startHeight; i += clsSizeY)
  {
    if (!bChroma)
    {
      pClass = classifier[startHeight + i] + startWidth;
    }

    for (int j = 0; j < endWidth - startWidth; j += clsSizeX)
    {
      if (!bChroma)
      {
        AlfClassifier cl = pClass[j];
        transposeIdx = cl & 0x03;
        coef = filterSet + ((cl >> 2) & 0x1F) * MAX_NUM_ALF_LUMA_COEFF;
      }

      if (filtType == ALF_FILTER_7)
      {
        if (transposeIdx == 1)
        {
          filterCoeff = std::vector<Pel>{ coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10], coef[11], coef[12] };
        }
      }
      else
      {
        if (transposeIdx == 1)
        {
          filterCoeff = { coef[4], coef[1], coef[5], coef[3], coef[0], coef[2], coef[6] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[4], coef[5], coef[6] };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { coef[4], coef[3], coef[5], coef[1], coef[0], coef[2], coef[6] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6] };
        }
      }

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;
        pImg5 = pImgYPad5 + j + ii * srcStride;
        pImg6 = pImgYPad6 + j + ii * srcStride;

        pRec1 = pRec0 + j + ii * dstStride;

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          int sum = 0;
          if (filtType == ALF_FILTER_7)
          {
            sum += filterCoeff[0] * (pImg5[0] + pImg6[0]);

            sum += filterCoeff[1] * (pImg3[+1] + pImg4[-1]);
            sum += filterCoeff[2] * (pImg3[+0] + pImg4[+0]);
            sum += filterCoeff[3] * (pImg3[-1] + pImg4[+1]);

            sum += filterCoeff[4] * (pImg1[+2] + pImg2[-2]);
            sum += filterCoeff[5] * (pImg1[+1] + pImg2[-1]);
            sum += filterCoeff[6] * (pImg1[+0] + pImg2[+0]);
            sum += filterCoeff[7] * (pImg1[-1] + pImg2[+1]);
            sum += filterCoeff[8] * (pImg1[-2] + pImg2[+2]);

            sum += filterCoeff[9] * (pImg0[+3] + pImg0[-3]);
            sum += filterCoeff[10] * (pImg0[+2] + pImg0[-2]);
            sum += filterCoeff[11] * (pImg0[+1] + pImg0[-1]);
            sum += filterCoeff[12] * (pImg0[+0]);
          }
          else
          {
            sum += filterCoeff[0] * (pImg3[+0] + pImg4[+0]);

            sum += filterCoeff[1] * (pImg1[+1] + pImg2[-1]);
            sum += filterCoeff[2] * (pImg1[+0] + pImg2[+0]);
            sum += filterCoeff[3] * (pImg1[-1] + pImg2[+1]);

            sum += filterCoeff[4] * (pImg0[+2] + pImg0[-2]);
            sum += filterCoeff[5] * (pImg0[+1] + pImg0[-1]);
            sum += filterCoeff[6] * (pImg0[+0]);
          }

          sum = (sum + offset) >> shift;
          pRec1[jj] = ClipPel(sum, clpRng);

          pImg0++;
          pImg1++;
          pImg2++;
          pImg3++;
          pImg4++;
          pImg5++;
          pImg6++;
        }
      }
    }

    pRec0 += dstStride2;
    pRec1 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}

void AdaptiveLoopFilter::storeALFParam(AlfTileGroupParam* pAlfParam, unsigned tLayer, unsigned tLayerMax)
{
  for (int k = tLayer; k <= tLayerMax; k++)
  {
    unsigned idx = m_storedAlfParaNum[k] % MAX_NUM_ALFS_PER_TLAYER;
    m_storedAlfParaNum[k]++;
    pAlfParam->temporalAlfFlag = false;
    m_acStoredAlfPara[k][idx] = *pAlfParam;
  }
}

void AdaptiveLoopFilter::loadALFParam(AlfTileGroupParam* pAlfParam, unsigned idx, unsigned tLayer)
{
  bool enable[3];
  memcpy(enable, pAlfParam->enabledFlag, sizeof(pAlfParam->enabledFlag));
  *pAlfParam = m_acStoredAlfPara[tLayer][idx];
  memcpy(pAlfParam->enabledFlag, enable, sizeof(pAlfParam->enabledFlag));
  pAlfParam->temporalAlfFlag = true;
}

void AdaptiveLoopFilter::resetTemporalAlfBuffer()
{
  std::memset(m_storedAlfParaNum, 0, sizeof(m_storedAlfParaNum));
}

void AdaptiveLoopFilter::storeALFParamLine(AlfTileGroupParam* pAlfParam, unsigned tLayer)
{
  if( tLayer == 0 )
      m_IRAPFilter = *pAlfParam;

  unsigned idx = m_acAlfLineBufferCurrentSize % ALF_TEMPORAL_WITH_LINE_BUFFER;
  pAlfParam->temporalAlfFlag = false;
  pAlfParam->tLayer = tLayer;

  m_acAlfLineBuffer[idx] = *pAlfParam;
  m_acAlfLineBufferCurrentSize++;
}
void AdaptiveLoopFilter::loadALFParamLine(AlfTileGroupParam* pAlfParam, unsigned idx, unsigned tLayer)
{
  bool enable[3];
  memcpy(enable, pAlfParam->enabledFlag, sizeof(pAlfParam->enabledFlag));
  int bufIdx = 0; bool isFound = false;
  for (int i = 0; i < ALF_TEMPORAL_WITH_LINE_BUFFER; i++)
  {
    if (m_acAlfLineBuffer[i].tLayer <= tLayer)
    {
      if (idx == bufIdx)
      {
        *pAlfParam = m_acAlfLineBuffer[i];
        isFound = true;
        break;
      }
      bufIdx++;
    }
  }
  memcpy(pAlfParam->enabledFlag, enable, sizeof(pAlfParam->enabledFlag));
  pAlfParam->temporalAlfFlag = true;
  CHECK(!isFound, "ALF : loadALFParamLine cannot load from the buffer");
}
void AdaptiveLoopFilter::resetTemporalAlfBufferLine()
{
  m_acAlfLineBufferCurrentSize = 0;
}

void AdaptiveLoopFilter::resetTemporalAlfBufferLine2First()
{
  m_acAlfLineBuffer[0] = m_IRAPFilter;
  m_acAlfLineBuffer[0].temporalAlfFlag = false;
  m_acAlfLineBuffer[0].tLayer = 0;
  m_acAlfLineBufferCurrentSize = 1;
}





#endif