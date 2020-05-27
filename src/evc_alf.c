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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "evc_alf.h"

void ALFProcess(AdaptiveLoopFilter *p, CodingStructure* cs, AlfSliceParam* alfSliceParam);

short m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
int  m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
int  m_laplacian[NUM_DIRECTIONS][m_CLASSIFICATION_BLK_SIZE + 5][m_CLASSIFICATION_BLK_SIZE + 5];

AlfSliceParam m_acAlfLineBuffer[APS_MAX_NUM];

// Encoder side variables
u8  m_alfIndxInScanOrder[APS_MAX_NUM] = { 0 };
u8  m_nextFreeAlfIdxInBuffer = 0;  
u32 m_firstIdrPoc = INT_MAX;
u32 m_lastIdrPoc = INT_MAX;
u32 m_currentPoc = INT_MAX;
u32 m_currentTempLayer = INT_MAX;
u32 m_i_period;   
int m_alf_present_idr = 0;
int m_alf_idx_idr = INT_MAX;


AlfSliceParam m_IRAPFilter;
u8 m_acAlfLineBufferCurrentSize = 0;

pel* m_tempBuf, *m_tempBuf1, *m_tempBuf2;
int  m_picWidth;
int  m_picHeight;
int  m_maxCUWidth;
int  m_maxCUHeight;
int  m_maxCUDepth;
int  m_numCTUsInWidth;
int  m_numCTUsInHeight;
int  m_numCTUsInPic;
AlfClassifier** m_classifier;
ChromaFormat    m_chromaFormat;

int m_lastRasPoc = INT_MAX;
BOOL m_pendingRasInit = FALSE;
u8* m_ctuEnableFlag[MAX_NUM_COMPONENT];
ClpRngs m_clpRngs;

BOOL m_store2ALFBufferFlag;
BOOL m_resetALFBufferFlag;

AdaptiveLoopFilter* new_ALF()
{
    AdaptiveLoopFilter* r = (AdaptiveLoopFilter*)malloc(sizeof(AdaptiveLoopFilter));
    init_AdaptiveLoopFilter(r);
    return r;
}

void delete_ALF(AdaptiveLoopFilter* p)
{
    free(p);
}

void call_create_ALF(AdaptiveLoopFilter* p, const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth)
{
    AdaptiveLoopFilter_create( picWidth, picHeight, maxCUWidth, maxCUHeight, maxCUDepth );
}

void call_destroy_ALF(AdaptiveLoopFilter* p)
{
    AdaptiveLoopFilter_destroy();
}

void store_dec_aps_to_buffer(EVCD_CTX * ctx)
{
    AlfSliceParam alfSliceParam;
    evc_AlfSliceParam iAlfSliceParam = ctx->aps.alf_aps_param;

    //port
    alfSliceParam.filterShapes = NULL; // pointer will be assigned in ApplyALF;
    alfSliceParam.enabledFlag[COMPONENT_Y] = (iAlfSliceParam.enabledFlag[0]);
    alfSliceParam.enabledFlag[COMPONENT_Cb] = (iAlfSliceParam.enabledFlag[1]);
    alfSliceParam.enabledFlag[COMPONENT_Cr] = (iAlfSliceParam.enabledFlag[2]);
#if M53608_ALF_14
    alfSliceParam.chromaFilterPresent = iAlfSliceParam.chromaFilterPresent;
#endif

    alfSliceParam.numLumaFilters = iAlfSliceParam.numLumaFilters;
    alfSliceParam.lumaFilterType = (AlfFilterType)(iAlfSliceParam.lumaFilterType);
    alfSliceParam.chromaCtbPresentFlag = (iAlfSliceParam.chromaCtbPresentFlag);

    memcpy(alfSliceParam.filterCoeffDeltaIdx, iAlfSliceParam.filterCoeffDeltaIdx, MAX_NUM_ALF_CLASSES * sizeof(short));
    memcpy(alfSliceParam.lumaCoeff, iAlfSliceParam.lumaCoeff, sizeof(short)*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_LUMA_COEFF);
    memcpy(alfSliceParam.chromaCoeff, iAlfSliceParam.chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
    memcpy(alfSliceParam.fixedFilterIdx, iAlfSliceParam.fixedFilterIdx, MAX_NUM_ALF_CLASSES * sizeof(int));
#if M53608_ALF_7
    memcpy(alfSliceParam.fixedFilterUsageFlag, iAlfSliceParam.fixedFilterUsageFlag, MAX_NUM_ALF_CLASSES * sizeof(u8));
#endif
    alfSliceParam.fixedFilterPattern = iAlfSliceParam.fixedFilterPattern;

    alfSliceParam.coeffDeltaFlag = (iAlfSliceParam.coeffDeltaFlag);
    alfSliceParam.coeffDeltaPredModeFlag = (iAlfSliceParam.coeffDeltaPredModeFlag);

    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
        alfSliceParam.filterCoeffFlag[i] = (iAlfSliceParam.filterCoeffFlag[i]);
    }

    alfSliceParam.prevIdx = iAlfSliceParam.prevIdx;
    alfSliceParam.prevIdxComp[0] = iAlfSliceParam.prevIdxComp[0];
    alfSliceParam.prevIdxComp[1] = iAlfSliceParam.prevIdxComp[1];
    alfSliceParam.tLayer = iAlfSliceParam.tLayer;
    alfSliceParam.temporalAlfFlag = (iAlfSliceParam.temporalAlfFlag);

    const unsigned tidx = ctx->nalu.nuh_temporal_id;

    // Initialize un-used variables at the decoder side  TODO: Modify structure
    alfSliceParam.m_filterPoc = INT_MAX;
    alfSliceParam.m_maxIdrPoc = INT_MAX;
    alfSliceParam.m_minIdrPoc = INT_MAX;

    store_alf_paramline_from_aps(&(alfSliceParam), alfSliceParam.prevIdx);
}
void call_dec_alf_process_aps(AdaptiveLoopFilter* p, EVCD_CTX * ctx, EVC_PIC * pic)
{
    CodingStructure cs;
    cs.pCtx = (void*)ctx;
    cs.pPic = pic;

    AlfSliceParam alfSliceParam;
    alfSliceParam.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
    memset(alfSliceParam.alfCtuEnableFlag, 0, N_C * ctx->f_lcu * sizeof(u8));
    // load filter from buffer
    load_alf_paramline_from_aps_buffer2(&(alfSliceParam), ctx->sh.aps_id_y, ctx->sh.aps_id_ch
#if M53608_ALF_14
        , ctx->sh.alfChromaIdc
#endif
    );

    // load filter map buffer
    alfSliceParam.isCtbAlfOn = ctx->sh.alf_sh_param.isCtbAlfOn;
    memcpy(alfSliceParam.alfCtuEnableFlag, ctx->sh.alf_sh_param.alfCtuEnableFlag, N_C * ctx->f_lcu * sizeof(u8));
    ALFProcess(p, &cs, &alfSliceParam);
}

AlfFilterShape m_filterShapes[MAX_NUM_CHANNEL_TYPE][2];

void init_AdaptiveLoopFilter(AdaptiveLoopFilter* p)
{
  m_clpRngs.comp[0] = (ClpRng){ .min=0, .max=1023, .bd=10, .n=0 };
  m_clpRngs.comp[1] = (ClpRng){ .min=0, .max=1023, .bd=10, .n=0 };
  m_clpRngs.comp[2] = (ClpRng){ .min=0, .max=1023, .bd=10, .n=0 };
  m_clpRngs.used = FALSE;
  m_clpRngs.chroma = FALSE;

  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_ctuEnableFlag[compIdx] = NULL;
  }

  p->m_deriveClassificationBlk = deriveClassificationBlk;
  p->m_filter5x5Blk = filterBlk_5;
  p->m_filter7x7Blk = filterBlk_7;
}

void init_AlfFilterShape(void* _th, int size) {

  AlfFilterShape* th = (AlfFilterShape*)_th;

  th->filterLength = size;
  th->numCoeff = size * size / 4 + 1;
  th->filterSize = size * size / 2 + 1;

  if (size == 5)
  {
    memcpy(th->pattern, pattern5, sizeof(pattern5));
    memcpy(th->weights, weights5, sizeof(weights5));
    memcpy(th->golombIdx, golombIdx5, sizeof(golombIdx5));
    memcpy(th->patternToLargeFilter, patternToLargeFilter5, sizeof(patternToLargeFilter5));
    th->filterType = ALF_FILTER_5;
  }
  else if (size == 7)
  {
    memcpy(th->pattern, pattern7, sizeof(pattern7));
    memcpy(th->weights, weights7, sizeof(weights7));
    memcpy(th->golombIdx, golombIdx7, sizeof(golombIdx7));
    memcpy(th->patternToLargeFilter, patternToLargeFilter7, sizeof(patternToLargeFilter7));
    th->filterType = ALF_FILTER_7;
  }
  else
  {
    th->filterType = ALF_NUM_OF_FILTER_TYPES;
    CHECK(0, "Wrong ALF filter shape");
  }
}

/*
* tmpYuv -  destination, temporary buffer
* pointer tmpYuv is assumed to point to interior point inside margins
* s - its stride
* recYuv - source, recovered buffer
* s2 - its stride
* w - width
* h - height
* m - margin size
*/
void copy_and_extend_tile(pel* tmpYuv, const int s, const pel* recYuv, const int s2, const int w, const int h, const int m)
{

    //copy
    for (int j = 0; j < h; j++)
        memcpy(tmpYuv + j * s, recYuv + j * s2, sizeof(pel) * w);

    //extend
    pel * p = tmpYuv;
    // do left and right margins
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < m; x++) {
            *(p - m + x) = p[0];
            p[w + x] = p[w - 1];
        }
        p += s;
    }

    // p is now the (0,height) (bottom left of image within bigger picture
    p -= (s + m);
    // p is now the (-margin, height-1)
    for (int y = 0; y < m; y++) {
        memcpy(p + (y + 1) * s, p, sizeof(pel) * (w + (m << 1)));
    }

    // pi is still (-marginX, height-1)
    p -= ((h - 1) * s);
    // pi is now (-marginX, 0)
    for (int y = 0; y < m; y++) {
        memcpy(p - (y + 1) * s, p, sizeof(pel) * (w + (m << 1)));
    }
}

/*
 * tmpYuv -  destination, temporary buffer
 * pointer tmpYuv is assumed to point to interior point inside margins
 * s - its stride
 * recYuv - source, recovered buffer
 * s2 - its stride
 * w - width
 * h - height
 * m - margin size
 */
void copy_and_extend( pel* tmpYuv, const int s, const pel* recYuv, const int s2, const int w, const int h, const int m )
{

//copy
    for (int j = 0; j < h; j++)
        memcpy(tmpYuv + j * s, recYuv + j * s2, sizeof(pel) * w);

//extend

    pel * p = tmpYuv;
// do left and right margins
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < m; x++) {
            *(p - m + x) = p[0];
            p[w + x] = p[w - 1];
        }
        p += s;
    }

// p is now the (0,height) (bottom left of image within bigger picture
    p -= (s + m);
// p is now the (-margin, height-1)
    for (int y = 0; y < m; y++) {
        memcpy(p + (y + 1) * s, p, sizeof(pel) * (w + (m << 1)));
    }

// pi is still (-marginX, height-1)
    p -= ((h - 1) * s);
// pi is now (-marginX, 0)
    for (int y = 0; y < 3; y++) {
        memcpy(p - (y + 1) * s, p, sizeof(pel) * (w + (m << 1)));
    }

} // <-- end of copy and extend

int getMaxGolombIdx( AlfFilterType filterType )
{
  return filterType == ALF_FILTER_5 ? 2 : 3;
}

const int m_fixedFilterCoeff[m_fixedFilterNum][13] =
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
const int m_classToFilterMapping[MAX_NUM_ALF_CLASSES][ALF_FIXED_FILTER_NUM] =
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

void tile_boundary_check(int* availableL, int* availableR, int* availableT, int* availableB, const int width, const int height, int xPos, int yPos, int x_l, int x_r, int y_l, int y_r)
{
    if (xPos == x_l)
        *availableL = 0;
    else
        *availableL = 1;
    if (xPos + width == x_r)
        *availableR = 0;
    else
        *availableR = 1;
    if (yPos == y_l)
        *availableT = 0;
    else
        *availableT = 1;
    if (yPos + height == y_r)
        *availableB = 0;
    else
        *availableB = 1;
}

void ALFProcess(AdaptiveLoopFilter *p, CodingStructure* cs, AlfSliceParam* alfSliceParam)
{
    if (!alfSliceParam->enabledFlag[COMPONENT_Y] && !alfSliceParam->enabledFlag[COMPONENT_Cb] && !alfSliceParam->enabledFlag[COMPONENT_Cr])
    {
        return;
    }

    EVCD_CTX* ctx = (EVCD_CTX*)(cs->pCtx);

    // set available filter shapes
    alfSliceParam->filterShapes = &m_filterShapes[0];

    // set CTU enable flags
    for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
        m_ctuEnableFlag[compIdx] = alfSliceParam->alfCtuEnableFlag + ctx->f_lcu * compIdx;
    }

    reconstructCoeff(alfSliceParam, CHANNEL_TYPE_LUMA, FALSE, TRUE);
    if (alfSliceParam->enabledFlag[COMPONENT_Cb] || alfSliceParam->enabledFlag[COMPONENT_Cr])
    {
        reconstructCoeff(alfSliceParam, CHANNEL_TYPE_CHROMA, FALSE, FALSE);
    }
    int ii, x_l, x_r, y_l, y_r, w_tile, h_tile;
    const int h = cs->pPic->h_l;
    const int w = cs->pPic->w_l;
    const int m = MAX_ALF_FILTER_LENGTH >> 1;
    const int s = w + m + m;
    int col_bd = 0;
    u32 k = 0;
    ii = 0;
    int num_tiles_in_slice = ctx->num_tiles_in_slice;
    while (num_tiles_in_slice)
    {
        ii = ctx->tile_in_slice[k++];
        col_bd = 0;
        if (ii % (ctx->pps.num_tile_columns_minus1 + 1))
        {
            int temp = ii - 1;
            while (temp >= 0)
            {
                col_bd += ctx->tile[temp].w_ctb;
                if (!(temp % (ctx->pps.num_tile_columns_minus1 + 1))) break;
                temp--;
            }
        }
        else
        {
            col_bd = 0;
        }
        int x_loc = ((ctx->tile[ii].ctba_rs_first) % ctx->w_lcu);
        int y_loc = ((ctx->tile[ii].ctba_rs_first) / ctx->w_lcu);
        int ctuIdx = x_loc + y_loc * ctx->w_lcu;
        x_l = x_loc << ctx->log2_max_cuwh; //entry point lcu's x location
        y_l = y_loc << ctx->log2_max_cuwh; // entry point lcu's y location
        x_r = x_l + ((int)(ctx->tile[ii].w_ctb) << ctx->log2_max_cuwh);
        y_r = y_l + ((int)(ctx->tile[ii].h_ctb) << ctx->log2_max_cuwh);
        w_tile = x_r > ((int)ctx->w_scu << MIN_CU_LOG2) ? ((int)ctx->w_scu << MIN_CU_LOG2) - x_l : x_r - x_l;
        h_tile = y_r > ((int)ctx->h_scu << MIN_CU_LOG2) ? ((int)ctx->h_scu << MIN_CU_LOG2) - y_l : y_r - y_l;
        x_r = x_r > ((int)ctx->w_scu << MIN_CU_LOG2) ? ((int)ctx->w_scu << MIN_CU_LOG2) : x_r;
        y_r = y_r > ((int)ctx->h_scu << MIN_CU_LOG2) ? ((int)ctx->h_scu << MIN_CU_LOG2) : y_r;

        pel * recYuv = cs->pPic->y;
        pel * tmpYuv = m_tempBuf + s * m + m;
        //chroma (for 4:2:0 only)
        const int s1 = (w >> 1) + m + m;
        pel * recYuv1 = cs->pPic->u;
        pel * tmpYuv1 = m_tempBuf1 + s1 * m + m; //m, not m-1, is left for unification with VVC
        pel * recYuv2 = cs->pPic->v;
        pel * tmpYuv2 = m_tempBuf2 + s1 * m + m; //m, not m-1, is left for unification with VVC

        Pel * recLuma0_tile = tmpYuv + x_l + y_l * s;
        Pel * recLuma1_tile = tmpYuv1 + (x_l >> 1) + (y_l >> 1) * (s1);
        Pel * recLuma2_tile = tmpYuv2 + (x_l >> 1) + (y_l >> 1) * (s1);

        Pel * recoYuv0_tile = recYuv + x_l + y_l * cs->pPic->s_l;
        Pel * recoYuv1_tile = recYuv1 + (x_l >> 1) + (y_l >> 1) * cs->pPic->s_c;
        Pel * recoYuv2_tile = recYuv2 + (x_l >> 1) + (y_l >> 1) * cs->pPic->s_c;

        copy_and_extend_tile(recLuma0_tile, s, recoYuv0_tile, cs->pPic->s_l, w_tile, h_tile, m);
        copy_and_extend_tile(recLuma1_tile, s1, recoYuv1_tile, cs->pPic->s_c, (w_tile >> 1), (h_tile >> 1), m);
        copy_and_extend_tile(recLuma2_tile, s1, recoYuv2_tile, cs->pPic->s_c, (w_tile >> 1), (h_tile >> 1), m);

        int l_zero_offset = (MAX_CU_SIZE + m + m) * m + m;
        int l_stride = MAX_CU_SIZE + 2 * (MAX_ALF_FILTER_LENGTH >> 1);
        pel l_buffer[(MAX_CU_SIZE + 2 * (MAX_ALF_FILTER_LENGTH >> 1)) *(MAX_CU_SIZE + 2 * (MAX_ALF_FILTER_LENGTH >> 1))];
        pel *p_buffer = l_buffer + l_zero_offset;
        int l_zero_offset_chroma = ((MAX_CU_SIZE >> 1) + m + m) * m + m;
        int l_stride_chroma = (MAX_CU_SIZE >> 1) + 2 * (MAX_ALF_FILTER_LENGTH >> 1);
        pel l_buffer_cb[((MAX_CU_SIZE >> 1) + 2 * (MAX_ALF_FILTER_LENGTH >> 1)) *((MAX_CU_SIZE >> 1) + 2 * (MAX_ALF_FILTER_LENGTH >> 1))];
        pel l_buffer_cr[((MAX_CU_SIZE >> 1) + 2 * (MAX_ALF_FILTER_LENGTH >> 1)) *((MAX_CU_SIZE >> 1) + 2 * (MAX_ALF_FILTER_LENGTH >> 1))];
        pel *p_buffer_cr = l_buffer_cr + l_zero_offset_chroma;
        pel *p_buffer_cb = l_buffer_cb + l_zero_offset_chroma;

        for (int yPos = y_l; yPos < y_r; yPos += ctx->max_cuwh)
        {
            for (int xPos = x_l; xPos < x_r; xPos += ctx->max_cuwh)
            {
                const int width = (xPos + ctx->max_cuwh > cs->pPic->w_l) ? (cs->pPic->w_l - xPos) : ctx->max_cuwh;
                const int height = (yPos + ctx->max_cuwh > cs->pPic->h_l) ? (cs->pPic->h_l - yPos) : ctx->max_cuwh;
                int availableL, availableR, availableT, availableB;
                availableL = availableR = availableT = availableB = 1;
                if (!(ctx->pps.loop_filter_across_tiles_enabled_flag))
                {
                    tile_boundary_check(&availableL, &availableR, &availableT, &availableB, width, height, xPos, yPos, x_l, x_r, y_l, y_r);
                }
                else
                {
                    tile_boundary_check(&availableL, &availableR, &availableT, &availableB, width, height, xPos, yPos,
                        0, ctx->sps.pic_width_in_luma_samples - 1, 0, ctx->sps.pic_height_in_luma_samples - 1);
                }
                for (int i = m; i < height + m; i++)
                {
                    int dstPos = i * l_stride - l_zero_offset;
                    int srcPos_offset = xPos + yPos * s;
                    int stride = (width == ctx->max_cuwh ? l_stride : width + m + m);
                    memcpy(p_buffer + dstPos + m, tmpYuv + srcPos_offset + (i - m) * s, sizeof(pel) * (stride - 2 * m));
                    for (int j = 0; j < m; j++)
                    {
                        if (availableL)
                        {
                            p_buffer[dstPos + j] = tmpYuv[srcPos_offset + (i - m) * s - m + j];
                        }
                        else
                        {
                            p_buffer[dstPos + j] = tmpYuv[srcPos_offset + (i - m) * s + m - j];
                        }

                        if (availableR)
                        {
                            p_buffer[dstPos + j + width + m] = tmpYuv[srcPos_offset + (i - m) * s + width + j];
                        }
                        else
                        {
                            p_buffer[dstPos + j + width + m] = tmpYuv[srcPos_offset + (i - m) * s + width - j - 2];
                        }
                    }
                }

                for (int i = 0; i < m; i++)
                {
                    int dstPos = i * l_stride - l_zero_offset;
                    int srcPos_offset = xPos + yPos * s;
                    int stride = (width == ctx->max_cuwh ? l_stride : width + m + m);
                    if (availableT)
                        memcpy(p_buffer + dstPos, tmpYuv + srcPos_offset - (m - i) * s - m, sizeof(pel) * stride);
                    else
                        memcpy(p_buffer + dstPos, p_buffer + dstPos + (2 * m - 2 * i) * l_stride, sizeof(pel) * stride);
                }

                for (int i = height + m; i < height + m + m; i++)
                {
                    int dstPos = i * l_stride - l_zero_offset;
                    int srcPos_offset = xPos + yPos * s;
                    int stride = (width == ctx->max_cuwh ? l_stride : width + m + m);
                    if (availableB)
                    {
                        memcpy(p_buffer + dstPos, tmpYuv + srcPos_offset + (i - m) * s - m, sizeof(pel) * stride);
                    }
                    else
                    {
                        memcpy(p_buffer + dstPos, p_buffer + dstPos - (2 * (i - height - m) + 2) * l_stride, sizeof(pel) * stride);
                    }
                }

                if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
                {
                    Area blk = { 0, 0, width, height };
                    {
                        deriveClassification(m_classifier, p_buffer, l_stride, &blk);
                        p->m_filter7x7Blk(m_classifier, recYuv + xPos + yPos * (cs->pPic->s_l), cs->pPic->s_l, p_buffer, l_stride, &blk, COMPONENT_Y, m_coeffFinal, &(m_clpRngs.comp[COMPONENT_Y]));
                    }
                }

                for (int i = m; i < ((height >> 1) + m); i++)
                {
                    int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                    int srcPos_offset = (xPos >> 1) + (yPos >> 1) * s1;
                    int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> 1) + m + m);
                    memcpy(p_buffer_cb + dstPos + m, tmpYuv1 + srcPos_offset + (i - m) * s1, sizeof(pel) * (stride - 2 * m));
                    memcpy(p_buffer_cr + dstPos + m, tmpYuv2 + srcPos_offset + (i - m) * s1, sizeof(pel) * (stride - 2 * m));
                    for (int j = 0; j < m; j++)
                    {
                        if (availableL)
                        {
                            p_buffer_cb[dstPos + j] = tmpYuv1[srcPos_offset + (i - m) * s1 - m + j];
                            p_buffer_cr[dstPos + j] = tmpYuv2[srcPos_offset + (i - m) * s1 - m + j];
                        }
                        else
                        {
                            p_buffer_cb[dstPos + j] = tmpYuv1[srcPos_offset + (i - m) * s1 + m - j];
                            p_buffer_cr[dstPos + j] = tmpYuv2[srcPos_offset + (i - m) * s1 + m - j];
                        }
                        if (availableR)
                        {
                            p_buffer_cb[dstPos + j + (width >> 1) + m] = tmpYuv1[srcPos_offset + (i - m) * s1 + (width >> 1) + j];
                            p_buffer_cr[dstPos + j + (width >> 1) + m] = tmpYuv2[srcPos_offset + (i - m) * s1 + (width >> 1) + j];
                        }
                        else
                        {
                            p_buffer_cb[dstPos + j + (width >> 1) + m] = tmpYuv1[srcPos_offset + (i - m) * s1 + (width >> 1) - j - 2];
                            p_buffer_cr[dstPos + j + (width >> 1) + m] = tmpYuv2[srcPos_offset + (i - m) * s1 + (width >> 1) - j - 2];
                        }
                    }
                }

                for (int i = 0; i < m; i++)
                {
                    int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                    int srcPos_offset = (xPos >> 1) + (yPos >> 1) * s1;
                    int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> 1) + m + m);
                    if (availableT)
                    {
                        memcpy(p_buffer_cb + dstPos, tmpYuv1 + srcPos_offset - (m - i) * s1 - m, sizeof(pel) * stride);
                    }
                    else
                    {
                        memcpy(p_buffer_cb + dstPos, p_buffer_cb + dstPos + (2 * m - 2 * i) * l_stride_chroma, sizeof(pel) * stride);
                    }
                    if (availableT)
                    {
                        memcpy(p_buffer_cr + dstPos, tmpYuv2 + srcPos_offset - (m - i) * s1 - m, sizeof(pel) * stride);
                    }
                    else
                    {
                        memcpy(p_buffer_cr + dstPos, p_buffer_cr + dstPos + (2 * m - 2 * i) * l_stride_chroma, sizeof(pel) * stride);
                    }
                }

                for (int i = ((height >> 1) + m); i < ((height >> 1) + m + m); i++)
                {
                    int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                    int srcPos_offset = (xPos >> 1) + (yPos >> 1) * s1;
                    int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> 1) + m + m);
                    if (availableB)
                    {
                        memcpy(p_buffer_cb + dstPos, tmpYuv1 + srcPos_offset + (i - m) * s1 - m, sizeof(pel) * stride);
                    }
                    else
                    {
                        memcpy(p_buffer_cb + dstPos, p_buffer_cb + dstPos - (2 * (i - (height >> 1) - m) + 2) * l_stride_chroma, sizeof(pel) * stride);
                    }

                    if (availableB)
                    {
                        memcpy(p_buffer_cr + dstPos, tmpYuv2 + srcPos_offset + (i - m) * s1 - m, sizeof(pel) * stride);
                    }
                    else
                    {
                        memcpy(p_buffer_cr + dstPos, p_buffer_cr + dstPos - (2 * (i - (height >> 1) - m) + 2) * l_stride_chroma, sizeof(pel) * stride);
                    }
                }

                for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
                {
                    ComponentID compID = (ComponentID)(compIdx);
                    const int chromaScaleX = 1; //getComponentScaleX(compID, tmpYuv.chromaFormat);
                    const int chromaScaleY = 1; //getComponentScaleY(compID, tmpYuv.chromaFormat);
#if M53608_ALF_1
                    if (alfSliceParam->enabledFlag[compIdx])
                    {
                        assert(m_ctuEnableFlag[compIdx][ctuIdx] == 1);
                        Area blk = { 0, 0, width >> chromaScaleX, height >> chromaScaleY };
                        if (compIdx == 1)
                            p->m_filter5x5Blk(m_classifier, recYuv1 + (xPos >> 1) + (yPos >> 1) * (cs->pPic->s_c), cs->pPic->s_c, p_buffer_cb, l_stride_chroma, &blk, compID, alfSliceParam->chromaCoeff, &(m_clpRngs.comp[compIdx]));
                        else if (compIdx == 2)
                            p->m_filter5x5Blk(m_classifier, recYuv2 + (xPos >> 1) + (yPos >> 1) * (cs->pPic->s_c), cs->pPic->s_c, p_buffer_cr, l_stride_chroma, &blk, compID, alfSliceParam->chromaCoeff, &(m_clpRngs.comp[compIdx]));
#else
                    if (alfSliceParam->enabledFlag[compIdx] && m_ctuEnableFlag[compIdx][ctuIdx])
                    {
                        Area blk = { 0, 0, width >> chromaScaleX, height >> chromaScaleY };
                        p->m_filter5x5Blk(m_classifier, recYuv1 + (xPos >> 1) + (yPos >> 1) * (cs->pPic->s_c), cs->pPic->s_c, p_buffer_cb, l_stride_chroma, &blk, compID, alfSliceParam->chromaCoeff, &(m_clpRngs.comp[compIdx]));
                        p->m_filter5x5Blk(m_classifier, recYuv2 + (xPos >> 1) + (yPos >> 1) * (cs->pPic->s_c), cs->pPic->s_c, p_buffer_cr, l_stride_chroma, &blk, compID, alfSliceParam->chromaCoeff, &(m_clpRngs.comp[compIdx]));
#endif
                    }
                }
                x_loc++;
                if (x_loc >= ctx->tile[ii].w_ctb + col_bd)
                {
                    x_loc = ((ctx->tile[ii].ctba_rs_first) % ctx->w_lcu);
                    y_loc++;
                }
                ctuIdx = x_loc + y_loc * ctx->w_lcu;
            }
        }
        num_tiles_in_slice--;
    }
}

#if INTEGR_M53608
void reconstructCoeff(AlfSliceParam* alfSliceParam, ChannelType channel, const BOOL bRdo, const BOOL bRedo)
{
    int factor = bRdo ? 0 : (1 << (m_NUM_BITS - 1));
    AlfFilterType filterType = channel == CHANNEL_TYPE_LUMA ? alfSliceParam->lumaFilterType : ALF_FILTER_5;
    int numClasses = channel == CHANNEL_TYPE_LUMA ? MAX_NUM_ALF_CLASSES : 1;
    int numCoeff = filterType == ALF_FILTER_5 ? 7 : 13;
    int numCoeffMinus1 = numCoeff - 1;
    int numFilters = channel == CHANNEL_TYPE_LUMA ? alfSliceParam->numLumaFilters : 1;
    short* coeff = channel == CHANNEL_TYPE_LUMA ? alfSliceParam->lumaCoeff : alfSliceParam->chromaCoeff;
    if (isLuma(channel))
    {
        if (alfSliceParam->coeffDeltaPredModeFlag)
        {
            for (int i = 1; i < numFilters; i++)
            {
                for (int j = 0; j < numCoeffMinus1; j++)
                {
                    coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += coeff[(i - 1) * MAX_NUM_ALF_LUMA_COEFF + j];
                }
            }
        }

        memset(m_coeffFinal, 0, sizeof(m_coeffFinal));
        int numCoeffLargeMinus1 = MAX_NUM_ALF_LUMA_COEFF - 1;
        for (int classIdx = 0; classIdx < numClasses; classIdx++)
        {
            int filterIdx = alfSliceParam->filterCoeffDeltaIdx[classIdx];
            int fixedFilterIdx = alfSliceParam->fixedFilterIdx[classIdx];
            u8  fixedFilterUsageFlag = alfSliceParam->fixedFilterUsageFlag[classIdx];
            int fixedFilterUsed = fixedFilterUsageFlag;
            int fixedFilterMapIdx = fixedFilterIdx;
            if (fixedFilterUsed)
            {
                fixedFilterIdx = m_classToFilterMapping[classIdx][fixedFilterMapIdx];
            }

            for (int i = 0; i < numCoeffLargeMinus1; i++)
            {
                int curCoeff = 0;
                //fixed filter
                if (fixedFilterUsageFlag > 0)
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

        if (bRedo && alfSliceParam->coeffDeltaPredModeFlag)
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
    else
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
    }
#else
void reconstructCoeff(AlfSliceParam* alfSliceParam, ChannelType channel, const BOOL bRdo, const BOOL bRedo)
{
  int factor = bRdo ? 0 : (1 << (m_NUM_BITS - 1));
  AlfFilterType filterType = channel == CHANNEL_TYPE_LUMA ? alfSliceParam->lumaFilterType : ALF_FILTER_5;
  int numClasses = channel == CHANNEL_TYPE_LUMA ? MAX_NUM_ALF_CLASSES : 1;
  int numCoeff = filterType == ALF_FILTER_5 ? 7 : 13;
  int numCoeffMinus1 = numCoeff - 1;
  int numFilters = channel == CHANNEL_TYPE_LUMA ? alfSliceParam->numLumaFilters : 1;
  short* coeff = channel == CHANNEL_TYPE_LUMA ? alfSliceParam->lumaCoeff : alfSliceParam->chromaCoeff;
#if M53608_ALF_13
  if (isLuma(channel))
  {
      if (alfSliceParam->coeffDeltaPredModeFlag)
      {
          for (int i = 1; i < numFilters; i++)
          {
              for (int j = 0; j < numCoeffMinus1; j++)
              {
                  coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += coeff[(i - 1) * MAX_NUM_ALF_LUMA_COEFF + j];
              }
          }
      }

      memset(m_coeffFinal, 0, sizeof(m_coeffFinal));
      int numCoeffLargeMinus1 = MAX_NUM_ALF_LUMA_COEFF - 1;
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
          int filterIdx = alfSliceParam->filterCoeffDeltaIdx[classIdx];
          int fixedFilterIdx = alfSliceParam->fixedFilterIdx[classIdx];
#if M53608_ALF_7
          u8  fixedFilterUsageFlag = alfSliceParam->fixedFilterUsageFlag[classIdx];
          int fixedFilterUsed = fixedFilterUsageFlag;
          int fixedFilterMapIdx = fixedFilterIdx;
#else
          int fixedFilterUsed = (fixedFilterIdx == 0 ? 0 : 1);
          int fixedFilterMapIdx = fixedFilterIdx - 1;
#endif
#if M53608_ALF_7
          if (fixedFilterUsed)
#else
          if (fixedFilterIdx > 0)
#endif
          {
#if M53608_ALF_7
              fixedFilterIdx = m_classToFilterMapping[classIdx][fixedFilterMapIdx];
#else
              fixedFilterIdx = m_classToFilterMapping[classIdx][fixedFilterIdx - 1];
#endif
          }

          for (int i = 0; i < numCoeffLargeMinus1; i++)
          {
              int curCoeff = 0;
              //fixed filter
#if M53608_ALF_7
              if (fixedFilterUsageFlag > 0)
#else
              if (fixedFilterIdx > 0)
#endif
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

      if (bRedo && alfSliceParam->coeffDeltaPredModeFlag)
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
  else
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
#else
if (alfSliceParam->coeffDeltaPredModeFlag && channel == CHANNEL_TYPE_LUMA)
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
    int filterIdx = alfSliceParam->filterCoeffDeltaIdx[classIdx];
    int fixedFilterIdx = alfSliceParam->fixedFilterIdx[classIdx];
#if M53608_ALF_7
    u8  fixedFilterUsageFlag = alfSliceParam->fixedFilterUsageFlag[classIdx];
    int fixedFilterUsed = fixedFilterUsageFlag;
    int fixedFilterMapIdx = fixedFilterIdx;
#else
    int fixedFilterUsed = (fixedFilterIdx == 0 ? 0 : 1);
    int fixedFilterMapIdx = fixedFilterIdx - 1;
#endif
#if M53608_ALF_7
    if (fixedFilterUsed)
#else
    if (fixedFilterIdx > 0)
#endif
    {
#if M53608_ALF_7
        fixedFilterIdx = m_classToFilterMapping[classIdx][fixedFilterMapIdx];
#else
        fixedFilterIdx = m_classToFilterMapping[classIdx][fixedFilterIdx - 1];
#endif
    }

    for (int i = 0; i < numCoeffLargeMinus1; i++)
    {
        int curCoeff = 0;
        //fixed filter
#if M53608_ALF_7
        if (fixedFilterUsageFlag > 0)
#else
        if (fixedFilterIdx > 0)
#endif
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

if (bRedo && alfSliceParam->coeffDeltaPredModeFlag)
{
    for (int i = numFilters - 1; i > 0; i--)
    {
        for (int j = 0; j < numCoeffMinus1; j++)
        {
            coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] = coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] - coeff[(i - 1) * MAX_NUM_ALF_LUMA_COEFF + j];
        }
    }
}
#endif
}
#endif
void AdaptiveLoopFilter_create(const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth)
{
  const ChromaFormat format = CHROMA_420;
  const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] = {10, 10};

  memcpy(m_inputBitDepth, inputBitDepth, sizeof(m_inputBitDepth));
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_maxCUDepth = maxCUDepth;
  m_chromaFormat = format;

  m_numCTUsInWidth = (m_picWidth / m_maxCUWidth) + ((m_picWidth % m_maxCUWidth) ? 1 : 0);
  m_numCTUsInHeight = (m_picHeight / m_maxCUHeight) + ((m_picHeight % m_maxCUHeight) ? 1 : 0);
  m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;

  init_AlfFilterShape(&m_filterShapes[CHANNEL_TYPE_LUMA][0], 5);
  init_AlfFilterShape(&m_filterShapes[CHANNEL_TYPE_LUMA][1], 7);
  init_AlfFilterShape(&m_filterShapes[CHANNEL_TYPE_CHROMA][0], 5);

  m_tempBuf  = (pel*)malloc((picWidth + 7)*(picHeight + 7)*sizeof(pel)); // +7 is of filter diameter //todo: check this
  m_tempBuf1 = (pel*)malloc(((picWidth >> 1) + 7)*((picHeight >> 1) + 7)*sizeof(pel)); // for chroma just left for unification
  m_tempBuf2 = (pel*)malloc(((picWidth >> 1) + 7)*((picHeight >> 1) + 7)*sizeof(pel));

  // Classification
  m_classifier = (AlfClassifier**)malloc(picHeight*sizeof(AlfClassifier*));
  for (int i = 0; i < picHeight; i++)
  {
    m_classifier[i] = (AlfClassifier*)malloc(picWidth*sizeof(AlfClassifier));
    memset(m_classifier[i], 0, picWidth * sizeof(AlfClassifier)); 
  }

}

void AdaptiveLoopFilter_destroy()
{
  free(m_tempBuf);
  free(m_tempBuf1);
  free(m_tempBuf2);

  if (m_classifier)
  {
    for (int i = 0; i < m_picHeight; i++)
    {
      free(m_classifier[i]);
      m_classifier[i] = NULL;
    }

    free(m_classifier);
    m_classifier = NULL;
  }
}

void deriveClassification(AlfClassifier** classifier, const pel * srcLuma, const int srcLumaStride, const Area * blk)
{
  int height = blk->y + blk->height;
  int width = blk->x + blk->width;

  for (int i = blk->y; i < height; i += m_CLASSIFICATION_BLK_SIZE)
  {
    int nHeight = min(i + m_CLASSIFICATION_BLK_SIZE, height) - i;

    for (int j = blk->x; j < width; j += m_CLASSIFICATION_BLK_SIZE)
    {
      int nWidth = min(j + m_CLASSIFICATION_BLK_SIZE, width) - j;
      Area area = { j, i, nWidth, nHeight };
      deriveClassificationBlk(classifier, srcLuma, srcLumaStride, &area, m_inputBitDepth[CHANNEL_TYPE_LUMA] + 4);
    }
  }
}

void deriveClassificationBlk(AlfClassifier** classifier, const pel * srcLuma, const int srcStride,  const Area * blk, const int shift)
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
  int height = blk->height + fl2;
  int width = blk->width + fl2;
  int posX = blk->x;
  int posY = blk->y;
  int startHeight = posY - flP1;

  for (int i = 0; i < height; i += 2)
  {
    int yoffset = (i + 1 + startHeight) * stride - flP1;
    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];

    int* pYver =  m_laplacian[VER][i];
    int* pYhor =  m_laplacian[HOR][i];
    int* pYdig0 = m_laplacian[DIAG0][i];
    int* pYdig1 = m_laplacian[DIAG1][i];

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

  for (int i = 0; i < blk->height; i += clsSizeY)
  {
    int* pYver =  m_laplacian[VER][i];
    int* pYver2 = m_laplacian[VER][i + 2];
    int* pYver4 = m_laplacian[VER][i + 4];
    int* pYver6 = m_laplacian[VER][i + 6];

    int* pYhor =  m_laplacian[HOR][i];
    int* pYhor2 = m_laplacian[HOR][i + 2];
    int* pYhor4 = m_laplacian[HOR][i + 4];
    int* pYhor6 = m_laplacian[HOR][i + 6];

    int* pYdig0 =  m_laplacian[DIAG0][i];
    int* pYdig02 = m_laplacian[DIAG0][i + 2];
    int* pYdig04 = m_laplacian[DIAG0][i + 4];
    int* pYdig06 = m_laplacian[DIAG0][i + 6];

    int* pYdig1 =  m_laplacian[DIAG1][i];
    int* pYdig12 = m_laplacian[DIAG1][i + 2];
    int* pYdig14 = m_laplacian[DIAG1][i + 4];
    int* pYdig16 = m_laplacian[DIAG1][i + 6];

    for (int j = 0; j < blk->width; j += clsSizeX)
    {
      int sumV = pYver[j] + pYver2[j] + pYver4[j] + pYver6[j];
      int sumH = pYhor[j] + pYhor2[j] + pYhor4[j] + pYhor6[j];
      int sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j] + pYdig06[j];
      int sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j] + pYdig16[j];

      int tempAct = sumV + sumH;
#if M53608_ALF_2 
      int activity = (Pel)Clip3(0, maxActivity, tempAct >> ( BIT_DEPTH - 2 ) );
#else
      int activity = (Pel)Clip3(0, maxActivity, (tempAct * 32) >> shift);
#endif
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

void filterBlk_7(AlfClassifier** classifier, pel * recDst, const int dstStride, const pel* recSrc, const int srcStride, const Area* blk, const ComponentID compId, short* filterSet, const ClpRng* clpRng)
{
  const BOOL bChroma = FALSE;

  const int startHeight = blk->y;
  const int endHeight = blk->y + blk->height;
  const int startWidth = blk->x;
  const int endWidth = blk->x + blk->width;

  const Pel* src = recSrc;
  Pel* dst = recDst;

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

  AlfClassifier *pClass = NULL;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  Pel filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
  pImgYPad0 = src;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;
  Pel* pRec0 = dst;
  Pel* pRec1 = pRec0 + dstStride;

  for (int i = 0; i < endHeight - startHeight; i += clsSizeY)
  {
    if (!bChroma)
    {
      pClass = classifier[startHeight + i] + startWidth;
    }

    for (int j = 0; j < endWidth - startWidth; j += clsSizeX)
    {
      AlfClassifier cl = pClass[j];
      transposeIdx = cl & 0x03;
      coef = filterSet + ((cl >> 2) & 0x1F) * MAX_NUM_ALF_LUMA_COEFF;

      const int l[4][MAX_NUM_ALF_LUMA_COEFF] = {
          { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 },
          { 9, 4, 10, 8, 1, 5, 11, 7, 3, 0, 2, 6, 12 },
          { 0, 3, 2, 1, 8, 7, 6, 5, 4, 9, 10, 11, 12 },
          { 9, 8, 10, 4, 3, 7, 11, 5, 1, 0, 2, 6, 12 }
      };

      for(int i=0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
        filterCoeff[i] = coef[l[transposeIdx][i]];

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

          sum = (sum + offset) >> shift;
          pRec1[jj] = ClipPel(sum, *clpRng);

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

void filterBlk_5(AlfClassifier** classifier, pel * recDst, const int dstStride, const pel* recSrc, const int srcStride, const Area* blk, const ComponentID compId, short* filterSet, const ClpRng* clpRng)
{
  const int startHeight = blk->y;
  const int endHeight = blk->y + blk->height;
  const int startWidth = blk->x;
  const int endWidth = blk->x + blk->width;

  const Pel* src = recSrc;
  Pel* dst = recDst;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4;

  short *coef = filterSet;

  const int shift = 9;
  const int offset = 1 << (shift - 1);

  int transposeIdx = 0;
  const int clsSizeY = 1;
  const int clsSizeX = 1;

  AlfClassifier *pClass = NULL;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  Pel filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
  pImgYPad0 = src;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  Pel* pRec0 = dst;
  Pel* pRec1 = pRec0 + dstStride;

  for (int i = 0; i < endHeight - startHeight; i += clsSizeY)
  {
    for (int j = 0; j < endWidth - startWidth; j += clsSizeX)
    {
      for(int i=0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
          filterCoeff[i] = coef[i];

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;

        pRec1 = pRec0 + j + ii * dstStride;

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          int sum = 0;

            sum += filterCoeff[0] * (pImg3[+0] + pImg4[+0]);

            sum += filterCoeff[1] * (pImg1[+1] + pImg2[-1]);
            sum += filterCoeff[2] * (pImg1[+0] + pImg2[+0]);
            sum += filterCoeff[3] * (pImg1[-1] + pImg2[+1]);

            sum += filterCoeff[4] * (pImg0[+2] + pImg0[-2]);
            sum += filterCoeff[5] * (pImg0[+1] + pImg0[-1]);
            sum += filterCoeff[6] * (pImg0[+0]);

          sum = (sum + offset) >> shift;
          pRec1[jj] = ClipPel(sum, *clpRng);

          pImg0++;
          pImg1++;
          pImg2++;
          pImg3++;
          pImg4++;
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
  }
}

void copyAlfParamChroma(AlfSliceParam* dst, AlfSliceParam* src)
{
    memcpy(dst->chromaCoeff, src->chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
#if M53608_ALF_14
    dst->chromaFilterPresent = src->chromaFilterPresent;
#endif
    dst->chromaCtbPresentFlag = src->chromaCtbPresentFlag;
    dst->enabledFlag[1] = src->enabledFlag[1];
    dst->enabledFlag[2] = src->enabledFlag[2];

}

void copyAlfParam(AlfSliceParam* dst, AlfSliceParam* src)
{
  memcpy(dst->enabledFlag, src->enabledFlag, sizeof(BOOL)*MAX_NUM_COMPONENT);
#if M53608_ALF_14
  dst->chromaFilterPresent = src->chromaFilterPresent;
#endif
  memcpy(dst->lumaCoeff, src->lumaCoeff, sizeof(short)*MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF);
  memcpy(dst->chromaCoeff, src->chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
  memcpy(dst->filterCoeffDeltaIdx, src->filterCoeffDeltaIdx, sizeof(short)*MAX_NUM_ALF_CLASSES);
  memcpy(dst->filterCoeffFlag, src->filterCoeffFlag, sizeof(BOOL)*MAX_NUM_ALF_CLASSES);
  memcpy(dst->fixedFilterIdx, src->fixedFilterIdx, sizeof(int)*MAX_NUM_ALF_CLASSES);
#if M53608_ALF_7
  memcpy(dst->fixedFilterUsageFlag, src->fixedFilterUsageFlag, sizeof(u8)*MAX_NUM_ALF_CLASSES);
#endif
  dst->lumaFilterType = src->lumaFilterType;
  dst->numLumaFilters = src->numLumaFilters;
  dst->coeffDeltaFlag = src->coeffDeltaFlag;
  dst->coeffDeltaPredModeFlag = src->coeffDeltaPredModeFlag;
  dst->filterShapes = src->filterShapes;
  dst->chromaCtbPresentFlag = src->chromaCtbPresentFlag;
  dst->fixedFilterPattern = src->fixedFilterPattern;
  dst->temporalAlfFlag = src->temporalAlfFlag;
  dst->prevIdx = src->prevIdx;
  dst->prevIdxComp[0] = src->prevIdxComp[0];
  dst->prevIdxComp[1] = src->prevIdxComp[1];
  dst->tLayer = src->tLayer;

  // variables are not used at the decoder side. TODO: Modify the strcuture
  dst->m_filterPoc = src->m_filterPoc;
  dst->m_minIdrPoc = src->m_minIdrPoc;
  dst->m_maxIdrPoc = src->m_maxIdrPoc;
}
void resetAlfParam(AlfSliceParam* dst)
{
    //Reset destination
    dst->isCtbAlfOn = FALSE;
    memset(dst->enabledFlag, 0, sizeof(dst->enabledFlag)); //false is still 0
    dst->lumaFilterType = ALF_FILTER_5;
    memset(dst->lumaCoeff, 0, sizeof(dst->lumaCoeff));
    memset(dst->chromaCoeff, 0, sizeof(dst->chromaCoeff));
    memset(dst->filterCoeffDeltaIdx, 0, sizeof(dst->filterCoeffDeltaIdx));
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
        dst->filterCoeffFlag[i] = TRUE;
    dst->numLumaFilters = 1;
    dst->coeffDeltaFlag = FALSE;
    dst->coeffDeltaPredModeFlag = FALSE;
    dst->chromaCtbPresentFlag = FALSE;
    dst->fixedFilterPattern = 0;
    memset(dst->fixedFilterIdx, 0, sizeof(dst->fixedFilterIdx));
#if M53608_ALF_7
    memset(dst->fixedFilterUsageFlag, 0, sizeof(dst->fixedFilterUsageFlag));
#endif
    dst->temporalAlfFlag = FALSE;
    dst->prevIdx = 0;
    dst->prevIdxComp[0] = 0;
    dst->prevIdxComp[1] = 0;
    dst->tLayer = 0;
    dst->resetALFBufferFlag = FALSE;
    dst->store2ALFBufferFlag = FALSE;

    // variables are not used at the decoder side. TODO: Modify the strcuture
    dst->m_filterPoc = INT_MAX;  // store POC value for which filter was produced
    dst->m_minIdrPoc = INT_MAX;  // Minimal of 2 IDR POC available for current coded nalu  (to identify availability of this filter for temp prediction)
    dst->m_maxIdrPoc = INT_MAX;  // Max of 2 IDR POC available for current coded nalu  (to identify availability of this filter for temp prediction)
}

void resetIdrIndexListBufferAPS()
{
    if (m_alf_present_idr)
    {
        m_alfIndxInScanOrder[0] = m_alf_idx_idr;
        m_acAlfLineBufferCurrentSize = 1;
        m_nextFreeAlfIdxInBuffer = (m_alf_idx_idr + 1) % APS_MAX_NUM;
        m_alf_present_idr = 0;
    }
    else
    {
        m_alfIndxInScanOrder[0] = 0;
        m_acAlfLineBufferCurrentSize = 0;
        m_nextFreeAlfIdxInBuffer = 0;
    }
}

int  getProtectIdxFromList(int idx)
{
    u8 i_slice_idx = 0;
    int protectEntry = 0;
    if (m_i_period == 0)
        return protectEntry;

    // check if current idx is protected (e.g. idr filter idx)
    if ((m_acAlfLineBuffer[idx].m_filterPoc == m_acAlfLineBuffer[idx].m_maxIdrPoc))
    {
        protectEntry = 1; // previent overwrite of the protected ALF id (e.g. id of IDR pic)
    }
    if ((m_currentPoc > m_acAlfLineBuffer[idx].m_maxIdrPoc + m_i_period))
    {
        protectEntry = 0;
    }

    if ((m_currentPoc > m_lastIdrPoc) // current POC is after 2nd IDR 
        && (m_acAlfLineBuffer[idx].m_filterPoc < m_lastIdrPoc) // POC of checked ALF is before 2nd IDR
        )
    {
        protectEntry = 0;
    }

    if ((m_currentPoc > m_acAlfLineBuffer[idx].m_maxIdrPoc) // current POC is after 2nd IDR 
        && (m_acAlfLineBuffer[idx].m_filterPoc < m_acAlfLineBuffer[idx].m_maxIdrPoc) // POC of checked ALF is before 2nd IDR
        )
    {
        protectEntry = 0;
    }

    return protectEntry;
}

void storeEncALFParamLineAPS(AlfSliceParam* pAlfParam, unsigned tLayer)
{
    m_acAlfLineBufferCurrentSize++; // There is new filter, increment computed ALF buffer size
    if (m_acAlfLineBufferCurrentSize > APS_MAX_NUM)
    { // new filter to be stored in occupied location, check if this location is not protected
        while (getProtectIdxFromList(m_nextFreeAlfIdxInBuffer) && m_nextFreeAlfIdxInBuffer < APS_MAX_NUM)
        {
            m_nextFreeAlfIdxInBuffer = (m_nextFreeAlfIdxInBuffer + 1) % APS_MAX_NUM;  // Compute next availble ALF circular buffer index
        }
    }
    u8 idx = m_nextFreeAlfIdxInBuffer;  // Take in use next availble ALF circular buffer index
    pAlfParam->m_filterPoc = m_currentPoc;
    pAlfParam->m_minIdrPoc = m_firstIdrPoc;
    pAlfParam->m_maxIdrPoc = m_lastIdrPoc;
    pAlfParam->temporalAlfFlag = FALSE;
    pAlfParam->tLayer = tLayer;
    pAlfParam->chromaCtbPresentFlag = FALSE;

    if (m_acAlfLineBufferCurrentSize > APS_MAX_NUM)
    {
        // New ALF beyond ALF buffer capacity, index list is shifted left, by removing the most old  index (preserving protected indexes) from m_alfIndxInScanOrder
        for (int i = 1; i < APS_MAX_NUM; i++)
        {
            int idx_to_check = i - 1;
            if (getProtectIdxFromList(m_alfIndxInScanOrder[idx_to_check]))
            {
                continue;
            }
            m_alfIndxInScanOrder[idx_to_check] = m_alfIndxInScanOrder[i];
        }
    }

    resetAlfParam(&(m_acAlfLineBuffer[idx]));
    copyAlfParam(&(m_acAlfLineBuffer[idx]), pAlfParam);

    m_acAlfLineBufferCurrentSize = m_acAlfLineBufferCurrentSize > APS_MAX_NUM ? APS_MAX_NUM : m_acAlfLineBufferCurrentSize;  // Increment size of the circular buffer  (there are 2 buffers - ALF and indexes)
    m_alfIndxInScanOrder[m_acAlfLineBufferCurrentSize - 1] = m_nextFreeAlfIdxInBuffer;                                       // store new alf idx in the indexes circular buffer 
    m_nextFreeAlfIdxInBuffer = (m_nextFreeAlfIdxInBuffer + 1) % APS_MAX_NUM;  // Compute next availble ALF circular buffer index
}

void store_alf_paramline_from_aps(AlfSliceParam* pAlfParam, u8 idx)
{
    assert(idx < APS_MAX_NUM);
    copyAlfParam(&(m_acAlfLineBuffer[idx]), pAlfParam);
    m_acAlfLineBufferCurrentSize++;
    m_acAlfLineBufferCurrentSize = m_acAlfLineBufferCurrentSize > APS_MAX_NUM ? APS_MAX_NUM : m_acAlfLineBufferCurrentSize;  // Increment used ALF circular buffer size 
}

void load_alf_paramline_from_aps_buffer2(AlfSliceParam* pAlfParam, u8 idxY, u8 idxUV
#if M53608_ALF_14
     ,u8 alfChromaIdc
#endif
)
{
    copyAlfParam(pAlfParam, &(m_acAlfLineBuffer[idxY]));
    assert(pAlfParam->enabledFlag[0] == 1);
#if M53608_ALF_14
    if (alfChromaIdc)
    {
        copyAlfParamChroma(pAlfParam, &(m_acAlfLineBuffer[idxUV]));
        assert(pAlfParam->chromaFilterPresent == 1);
        pAlfParam->enabledFlag[1] = alfChromaIdc & 1;
        pAlfParam->enabledFlag[2] = (alfChromaIdc >> 1) & 1;
    }
    else
    {
        pAlfParam->enabledFlag[1] = 0;
        pAlfParam->enabledFlag[2] = 0;
    }
#else
    copyAlfParamChroma(pAlfParam, &(m_acAlfLineBuffer[idxUV]));
#endif
}

void load_alf_paramline_from_aps_buffer(AlfSliceParam* pAlfParam, u8 idx)
{
    copyAlfParam(pAlfParam, &(m_acAlfLineBuffer[idx]) );
}
