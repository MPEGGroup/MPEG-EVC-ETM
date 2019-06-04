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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"

#pragma warning(disable : 4996)
#if ALF

extern "C"
{

void set_resetALFBufferFlag(EncAdaptiveLoopFilter* p, int flag)
{
    m_resetALFBufferFlag = flag;
}

void set_store2ALFBufferFlag(EncAdaptiveLoopFilter* p, int flag)
{
    m_store2ALFBufferFlag = flag;
}

void delete_enc_ALF(EncAdaptiveLoopFilter* p)
{
    delete p;
}

EncAdaptiveLoopFilter* new_enc_ALF()
{
    EncAdaptiveLoopFilter* p = new EncAdaptiveLoopFilter;
    init_AdaptiveLoopFilter(&(p->m_AdaptiveLoopFilter));
    return p;
}

void call_create_enc_ALF(EncAdaptiveLoopFilter* p, const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth)
{
    p->create( picWidth, picHeight, maxCUWidth, maxCUHeight, maxCUDepth );
}

void call_destroy_enc_ALF(EncAdaptiveLoopFilter* p)
{
    p->destroy();
}

void call_enc_ALFProcess(EncAdaptiveLoopFilter* p, const double* lambdas, EVCE_CTX * ctx, EVC_PIC * pic, evc_AlfTileGroupParam* iAlfTileGroupParam)
{
    CodingStructure cs;
    cs.pCtx = (void*)ctx;
    cs.pPic = pic;

    if( m_resetALFBufferFlag )
    {
        resetTemporalAlfBufferLine();
        iAlfTileGroupParam->resetALFBufferFlag = true;
    }

#if FIX_SEQUENTIAL_CODING
    m_pendingRasInit = FALSE;
    if (ctx->ptr > m_lastRasPoc)
    {
        m_lastRasPoc = INT_MAX;
        m_pendingRasInit = TRUE;
    }
    if (ctx->tgh.tile_group_type == TILE_GROUP_I)
    {
        m_lastRasPoc = ctx->ptr;
    }

    if (m_pendingRasInit)
    {
        resetTemporalAlfBufferLine2First();
    }
#endif

    AlfTileGroupParam alfTileGroupParam;
    p->Enc_ALFProcess( cs, lambdas, &alfTileGroupParam );

    if( alfTileGroupParam.enabledFlag[0] && m_store2ALFBufferFlag )
    {
        const unsigned tidxMAX = MAX_NUM_TLAYER - 1u;
        const unsigned tidx = ctx->tgh.layer_id;
        assert(tidx <= tidxMAX);
        storeALFParamLine(&alfTileGroupParam, tidx);
        alfTileGroupParam.store2ALFBufferFlag = m_store2ALFBufferFlag;
    }


    iAlfTileGroupParam->isCtbAlfOn = BOOL(alfTileGroupParam.isCtbAlfOn ? 1 : 0);
    memcpy(iAlfTileGroupParam->alfCtuEnableFlag, alfTileGroupParam.alfCtuEnableFlag, 3*512 * sizeof(u8));
    iAlfTileGroupParam->enabledFlag[0] = BOOL( alfTileGroupParam.enabledFlag[COMPONENT_Y] );
    iAlfTileGroupParam->enabledFlag[1] = BOOL( alfTileGroupParam.enabledFlag[COMPONENT_Cb] );
    iAlfTileGroupParam->enabledFlag[2] = BOOL( alfTileGroupParam.enabledFlag[COMPONENT_Cr] );

    iAlfTileGroupParam->numLumaFilters = alfTileGroupParam.numLumaFilters;
    iAlfTileGroupParam->lumaFilterType = int(alfTileGroupParam.lumaFilterType);

    memcpy(iAlfTileGroupParam->filterCoeffDeltaIdx, alfTileGroupParam.filterCoeffDeltaIdx, MAX_NUM_ALF_CLASSES*sizeof(short));
    memcpy(iAlfTileGroupParam->lumaCoeff, alfTileGroupParam.lumaCoeff, sizeof(short)*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_LUMA_COEFF);
    memcpy(iAlfTileGroupParam->chromaCoeff, alfTileGroupParam.chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
    memcpy(iAlfTileGroupParam->fixedFilterIdx, alfTileGroupParam.fixedFilterIdx, MAX_NUM_ALF_CLASSES*sizeof(int));

    iAlfTileGroupParam->fixedFilterPattern = alfTileGroupParam.fixedFilterPattern;

    iAlfTileGroupParam->coeffDeltaFlag = BOOL(alfTileGroupParam.coeffDeltaFlag);
    iAlfTileGroupParam->coeffDeltaPredModeFlag = BOOL(alfTileGroupParam.coeffDeltaPredModeFlag);

    //bool is not a BOOL
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
        iAlfTileGroupParam->filterCoeffFlag[i] = BOOL(alfTileGroupParam.filterCoeffFlag[i]);
    }

    iAlfTileGroupParam->prevIdx = alfTileGroupParam.prevIdx;
    iAlfTileGroupParam->tLayer = alfTileGroupParam.tLayer;
    iAlfTileGroupParam->temporalAlfFlag = BOOL( alfTileGroupParam.temporalAlfFlag );
    iAlfTileGroupParam->resetALFBufferFlag = BOOL( alfTileGroupParam.resetALFBufferFlag );
    iAlfTileGroupParam->store2ALFBufferFlag = BOOL( alfTileGroupParam.store2ALFBufferFlag );

}

#if APS_ALF_SEQ_FIX
void alf_aps_enc_opt_process(EncAdaptiveLoopFilter* p, const double* lambdas, EVCE_CTX * ctx, EVC_PIC * pic, evc_AlfTileGroupParam* iAlfTileGroupParam)
{
    CodingStructure cs;
    cs.pCtx = (void*)ctx;
    cs.pPic = pic;

    if (m_resetALFBufferFlag)
    {
        iAlfTileGroupParam->resetALFBufferFlag = true;
    }
    // Initialize ALF module for current POC
    m_currentPoc = ctx->ptr;
    m_currentTempLayer = ctx->tgh.layer_id;
    if (m_resetALFBufferFlag)
    {
        // initialize firstIdrPoc
        if (m_lastIdrPoc != INT_MAX)  // LastIdr value was initialized
        {
            m_firstIdrPoc = m_lastIdrPoc;
        }
        else {
            m_firstIdrPoc = ctx->ptr;
        }
        m_lastIdrPoc = ctx->ptr;  // store current pointer of the reset poc
        m_i_period = ctx->param.i_period; // store i-period for current pic.
    }

    m_pendingRasInit = FALSE;
    if (ctx->ptr > m_lastRasPoc)
    {
        m_lastRasPoc = INT_MAX;
        m_pendingRasInit = TRUE;
    }
    if (ctx->tgh.tile_group_type == TILE_GROUP_I)
    {
        m_lastRasPoc = ctx->ptr;
    }

    if (m_pendingRasInit)
    {
        resetIdrIndexListBufferAPS();
    }

    AlfTileGroupParam alfTileGroupParam;
    p->Enc_ALFProcess(cs, lambdas, &alfTileGroupParam);

    if (alfTileGroupParam.enabledFlag[0] && m_store2ALFBufferFlag)
    {
        const unsigned tidxMAX = MAX_NUM_TLAYER - 1u;
        const unsigned tidx = ctx->tgh.layer_id;
        assert(tidx <= tidxMAX);
        storeEncALFParamLineAPS(&alfTileGroupParam, tidx);
        alfTileGroupParam.store2ALFBufferFlag = m_store2ALFBufferFlag;
    }
    if (ctx->tgh.tile_group_type == TILE_GROUP_I)
    {
        if (alfTileGroupParam.enabledFlag[0] && m_store2ALFBufferFlag)
        {
            m_alf_present_idr = 1;
            m_alf_idx_idr = alf_aps_get_current_alf_idx();
        }
        else
        {
            m_alf_present_idr = 0;
            m_alf_idx_idr = 0;
        }
    }

    iAlfTileGroupParam->isCtbAlfOn = BOOL(alfTileGroupParam.isCtbAlfOn ? 1 : 0);
    memcpy(iAlfTileGroupParam->alfCtuEnableFlag, alfTileGroupParam.alfCtuEnableFlag, 3 * 512 * sizeof(u8));
    iAlfTileGroupParam->enabledFlag[0] = BOOL(alfTileGroupParam.enabledFlag[COMPONENT_Y]);
    iAlfTileGroupParam->enabledFlag[1] = BOOL(alfTileGroupParam.enabledFlag[COMPONENT_Cb]);
    iAlfTileGroupParam->enabledFlag[2] = BOOL(alfTileGroupParam.enabledFlag[COMPONENT_Cr]);

    iAlfTileGroupParam->numLumaFilters = alfTileGroupParam.numLumaFilters;
    iAlfTileGroupParam->lumaFilterType = int(alfTileGroupParam.lumaFilterType);

    memcpy(iAlfTileGroupParam->filterCoeffDeltaIdx, alfTileGroupParam.filterCoeffDeltaIdx, MAX_NUM_ALF_CLASSES * sizeof(short));
    memcpy(iAlfTileGroupParam->lumaCoeff, alfTileGroupParam.lumaCoeff, sizeof(short)*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_LUMA_COEFF);
    memcpy(iAlfTileGroupParam->chromaCoeff, alfTileGroupParam.chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
    memcpy(iAlfTileGroupParam->fixedFilterIdx, alfTileGroupParam.fixedFilterIdx, MAX_NUM_ALF_CLASSES * sizeof(int));

    iAlfTileGroupParam->fixedFilterPattern = alfTileGroupParam.fixedFilterPattern;

    iAlfTileGroupParam->coeffDeltaFlag = BOOL(alfTileGroupParam.coeffDeltaFlag);
    iAlfTileGroupParam->coeffDeltaPredModeFlag = BOOL(alfTileGroupParam.coeffDeltaPredModeFlag);

    //bool is not a BOOL
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
        iAlfTileGroupParam->filterCoeffFlag[i] = BOOL(alfTileGroupParam.filterCoeffFlag[i]);
    }

    iAlfTileGroupParam->prevIdx = alfTileGroupParam.prevIdx;
    iAlfTileGroupParam->tLayer = alfTileGroupParam.tLayer;
    iAlfTileGroupParam->temporalAlfFlag = BOOL(alfTileGroupParam.temporalAlfFlag);
    iAlfTileGroupParam->resetALFBufferFlag = BOOL(alfTileGroupParam.resetALFBufferFlag);
    iAlfTileGroupParam->store2ALFBufferFlag = BOOL(alfTileGroupParam.store2ALFBufferFlag);

}

u8 alf_aps_get_current_alf_idx()
{
    return (m_nextFreeAlfIdxInBuffer - 1) < 0 ? APS_MAX_NUM-1 : (m_nextFreeAlfIdxInBuffer - 1);
}
#endif

} //<-- extern "C"

#define AlfCtx(c) SubCtx( Ctx::ctbAlfFlag, c )

void AlfTileGroupParam_reset(AlfTileGroupParam* p)
{
  p->isCtbAlfOn = false;
  memset( p->alfCtuEnableFlag, 1, sizeof(p->alfCtuEnableFlag) );
  memset(p->enabledFlag, 0, sizeof(p->enabledFlag)); //false is still 0
  p->lumaFilterType = ALF_FILTER_5;
  memset(p->lumaCoeff, 0, sizeof(p->lumaCoeff));
  memset(p->chromaCoeff, 0, sizeof(p->chromaCoeff));
  memset(p->filterCoeffDeltaIdx, 0, sizeof(p->filterCoeffDeltaIdx));
  for(int i=0; i<MAX_NUM_ALF_CLASSES; i++) 
      p->filterCoeffFlag[i] = TRUE;
  p->numLumaFilters = 1;
  p->coeffDeltaFlag = false;
  p->coeffDeltaPredModeFlag = false;
  p->chromaCtbPresentFlag = false;
  p->fixedFilterPattern = 0;
  memset(p->fixedFilterIdx, 0, sizeof(p->fixedFilterIdx));
  p->temporalAlfFlag = false;
  p->prevIdx = 0;
  p->tLayer = 0;
  p->resetALFBufferFlag = false;
  p->store2ALFBufferFlag = false;
  
#if APS_ALF_SEQ_FIX
  p->m_filterPoc = INT_MAX;  // store POC value for which filter was produced
  p->m_minIdrPoc = INT_MAX;  // Minimal of 2 IDR POC available for current coded chunk  (to identify availability of this filter for temp prediction)
  p->m_maxIdrPoc = INT_MAX;  // Max of 2 IDR POC available for current coded chunk  (to identify availability of this filter for temp prediction)
#endif
}

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
{

  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    m_alfCovariance[i] = nullptr;
  }
  for (int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_alfCovarianceFrame[i] = nullptr;
  }

  m_filterCoeffQuant = nullptr;
  m_filterCoeffSet = nullptr;
  m_diffFilterCoeff = nullptr;
}

void EncAdaptiveLoopFilter::create(const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth)
{
  const ChromaFormat chromaFormatIDC = CHROMA_420;

  AdaptiveLoopFilter_create( picWidth, picHeight, maxCUWidth, maxCUHeight, maxCUDepth );

  for (int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++)
  {
    ChannelType chType = (ChannelType)channelIdx;
    const int size = channelIdx ? 1 : 2;
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
    m_alfCovarianceFrame[chType] = new AlfCovariance*[size];
    for (int i = 0; i != size; i++)
    {
      m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];
      for (int k = 0; k < numClasses; k++)
      {
        m_alfCovarianceFrame[chType][i][k].create(m_filterShapes[chType][i].numCoeff);
      }
    }
  }


  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
    ChannelType chType = ChannelType(compIdx == 0 ? 0 : 1);
    int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;
    const int size = chType ==  CHANNEL_TYPE_LUMA ? 2 : 1;

    m_alfCovariance[compIdx] = new AlfCovariance**[size];

    for (int i = 0; i != size; i++)
    {
      m_alfCovariance[compIdx][i] = new AlfCovariance*[m_numCTUsInPic];
      for (int j = 0; j < m_numCTUsInPic; j++)
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance[numClasses];
        for (int k = 0; k < numClasses; k++)
        {
          m_alfCovariance[compIdx][i][j][k].create(m_filterShapes[chType][i].numCoeff);
        }
      }
    }
  }

  for (int i = 0; i != 2; i++)
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES; j++)
    {
      m_alfCovarianceMerged[i][j].create(m_filterShapes[COMPONENT_Y][i].numCoeff);
    }
  }

  m_filterCoeffQuant = new int[MAX_NUM_ALF_LUMA_COEFF];
  m_filterCoeffSet = new int*[MAX_NUM_ALF_CLASSES];
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }

}

void EncAdaptiveLoopFilter::destroy()
{
  for (int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++)
  {
    if (m_alfCovarianceFrame[channelIdx])
    {
      ChannelType chType = (ChannelType)channelIdx;
      int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
      const int size = channelIdx ? 1 : 2;
      for (int i = 0; i != size; i++)
      {
        for (int k = 0; k < numClasses; k++)
        {
          m_alfCovarianceFrame[channelIdx][i][k].destroy();
        }
        delete[] m_alfCovarianceFrame[channelIdx][i];
        m_alfCovarianceFrame[channelIdx][i] = nullptr;
      }
      delete[] m_alfCovarianceFrame[channelIdx];
      m_alfCovarianceFrame[channelIdx] = nullptr;
    }
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    if (m_ctuEnableFlagTmp[compIdx])
    {
      delete[] m_ctuEnableFlagTmp[compIdx];
      m_ctuEnableFlagTmp[compIdx] = nullptr;
    }
    if (m_alfCovariance[compIdx])
    {
      ChannelType chType = ChannelType(compIdx == 0 ? 0 : 1);
      const int size = chType == CHANNEL_TYPE_LUMA ? 2 : 1;
      int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

      for (int i = 0; i != size; i++)
      {
        for (int j = 0; j < m_numCTUsInPic; j++)
        {
          for (int k = 0; k < numClasses; k++)
          {
            m_alfCovariance[compIdx][i][j][k].destroy();
          }
          delete[] m_alfCovariance[compIdx][i][j];
          m_alfCovariance[compIdx][i][j] = nullptr;

        }
        delete[] m_alfCovariance[compIdx][i];
        m_alfCovariance[compIdx][i] = nullptr;

      }
      delete[] m_alfCovariance[compIdx];
      m_alfCovariance[compIdx] = nullptr;
    }
  }

  for (int i = 0; i != 2 /* m_filterShapes[COMPONENT_Y].size() */; i++)
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES; j++)
    {
      m_alfCovarianceMerged[i][j].destroy();
    }
  }

  if (m_filterCoeffSet)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if (m_diffFilterCoeff)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
      delete[] m_diffFilterCoeff[i];
      m_diffFilterCoeff[i] = nullptr;
    }
    delete[] m_diffFilterCoeff;
    m_diffFilterCoeff = nullptr;
  }

  delete[] m_filterCoeffQuant;
  m_filterCoeffQuant = nullptr;

  AdaptiveLoopFilter_destroy();
  }

void EncAdaptiveLoopFilter::initCABACEstimator(EVCE_CORE * core)
{
    //TBD: init CABAC estimator here
}

void EncAdaptiveLoopFilter::Enc_ALFProcess(CodingStructure& cs, const double *lambdas, AlfTileGroupParam* alfTileGroupParam)
{
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
      m_ctuEnableFlag[compIdx] = &(alfTileGroupParam->alfCtuEnableFlag[compIdx][0]);
  }

  // reset ALF parameters
  AlfTileGroupParam_reset(alfTileGroupParam);

  // set available filter shapes
  alfTileGroupParam->filterShapes = m_filterShapes;

  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA] - 8);
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA] - 8);
  m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double(1 << shiftLuma);
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double(1 << shiftChroma);
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double(1 << shiftChroma);

  const int h = cs.pPic->h_l;
  const int w = cs.pPic->w_l;
  const int m = MAX_ALF_FILTER_LENGTH >> 1;
  const int s = w + m + m;

  EVCE_CTX* ctx = (EVCE_CTX*)(cs.pCtx);

  EVC_PIC* picOrig = PIC_ORIG(ctx);
  EVC_PIC* picReco = PIC_MODE(ctx);

  pel * orgYuv0 = picOrig->y;
  pel * recoYuv0 = picReco->y;

  int oStride = picOrig->s_l;
  int rStride = picReco->s_l;
  pel * recLuma0 = m_tempBuf + s*m + m;

  //chroma (for 4:2:0 only)
  const int s1 = (w >> 1) + m + m;
  pel * recLuma1 = m_tempBuf1 + s1*m + m;
  pel * recLuma2 = m_tempBuf2 + s1*m + m;
  pel * recoYuv1 = picReco->u;
  pel * recoYuv2 = picReco->v;
  const int rStride1 = picReco->s_c;
  pel * orgYuv1 = picOrig->u;
  pel * orgYuv2 = picOrig->v;
  const int oStride1 = picOrig->s_c;

  copy_and_extend( recLuma0,  s, recoYuv0,      rStride,        w,        h, m );
  copy_and_extend( recLuma1, s1, recoYuv1, picReco->s_c, (w >> 1), (h >> 1), m );
  copy_and_extend( recLuma2, s1, recoYuv2, picReco->s_c, (w >> 1), (h >> 1), m );

  // derive classification
  Area blk = {0, 0, w, h};
  deriveClassification(m_classifier, recLuma0, s, &blk);

  YUV orgYuv, recLuma, recYuv;
  orgYuv.yuv[0] = orgYuv0;  orgYuv.s[0] = oStride;
  orgYuv.yuv[1] = orgYuv1;  orgYuv.s[1] = oStride1;
  orgYuv.yuv[2] = orgYuv2;  orgYuv.s[2] = oStride1;
  recYuv.yuv[0] = recoYuv0; recYuv.s[0] = rStride;
  recYuv.yuv[1] = recoYuv1; recYuv.s[1] = picReco->s_c;
  recYuv.yuv[2] = recoYuv2; recYuv.s[2] = picReco->s_c;
  recLuma.yuv[0] = recLuma0; recLuma.s[0] = s;
  recLuma.yuv[1] = recLuma1; recLuma.s[1] = s1;
  recLuma.yuv[2] = recLuma2; recLuma.s[2] = s1;

  // get CTB stats for filtering
  deriveStatsForFiltering(&orgYuv, &recLuma);

  // derive filter (luma)
  alfEncoder(cs, alfTileGroupParam, CHANNEL_TYPE_LUMA);
  // derive filter (chroma)

  if ( alfTileGroupParam->enabledFlag[COMPONENT_Y] )
  {
    alfEncoder(cs, alfTileGroupParam, CHANNEL_TYPE_CHROMA);
  }

  // temporal prediction

  if (ctx->tile_group_type != TILE_GROUP_I)
  {
    deriveStatsForFiltering(&orgYuv, &recLuma);
#if APS_ALF_SEQ_FIX
    alfTemporalEncoderAPS(cs, alfTileGroupParam);
#else
    alfTemporalEncoder(cs, alfTileGroupParam);
#endif
    m_resetALFBufferFlag = false; 
    alfTileGroupParam->resetALFBufferFlag = false;
    if( alfTileGroupParam->temporalAlfFlag ) {
        m_store2ALFBufferFlag = false;
        alfTileGroupParam->store2ALFBufferFlag = false;
    }
    else 
    {
        m_store2ALFBufferFlag = true;
        alfTileGroupParam->store2ALFBufferFlag = true;
    }
  }
  else {
      alfTileGroupParam->store2ALFBufferFlag = true;
      m_store2ALFBufferFlag = true;
      alfTileGroupParam->resetALFBufferFlag = true;
      m_resetALFBufferFlag = true;

  }

  // reconstruct 
  alfReconstructor(cs, alfTileGroupParam, orgYuv.yuv[0], orgYuv.s[0], recLuma.yuv[0], recLuma.s[0], COMPONENT_Y);
  alfReconstructor(cs, alfTileGroupParam, orgYuv.yuv[1],  orgYuv.s[1], recLuma.yuv[1], recLuma.s[1], COMPONENT_Cb);
  alfReconstructor(cs, alfTileGroupParam, orgYuv.yuv[2],  orgYuv.s[2], recLuma.yuv[2], recLuma.s[2], COMPONENT_Cr);

  for( int i = 0; i < ctx->f_lcu; i++ )
  {
      if( alfTileGroupParam->alfCtuEnableFlag[0][i] == 0 ) {
          alfTileGroupParam->isCtbAlfOn = true;
          break;
      } else
          alfTileGroupParam->isCtbAlfOn = false;
  }
}

double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags(CodingStructure& cs, const int iShapeIdx, ChannelType channel, const int numClasses, const int numCoeff, double& distUnfilter
  , bool recCoeff
)
{

  const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast  = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;

  if (isLuma(channel))
    CHECK(iShapeIdx != ALF_FILTER_7, "wrong ishapeIdx for luma");
  double cost = 0;
  distUnfilter = 0;
    setEnableFlag(&m_alfTileGroupParamTemp, channel, true);
  if (isChroma(channel))
  {
    m_alfTileGroupParamTemp.chromaCtbPresentFlag = false;
  }
  if (recCoeff)
  {
    reconstructCoeff(&m_alfTileGroupParamTemp, channel, true, isLuma(channel));
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
      {
        m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfTileGroupParamTemp.chromaCoeff[i];
      }
    }
  }
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
    for (int compID = compIDFirst; compID <= compIDLast; compID++)
    {
      double distUnfilterCtu = getUnfilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses);

      double costOn = 0;
      costOn = distUnfilterCtu + getFilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfTileGroupParamTemp.numLumaFilters - 1, numCoeff);
      m_ctuEnableFlag[compID][ctuIdx] = 0;
      double costOff = distUnfilterCtu; // + m_lambda[compID] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();

      if (costOn < costOff)
      {
        cost += costOn;
        m_ctuEnableFlag[compID][ctuIdx] = 1;
      }
      else
      {
        cost += costOff;
        m_ctuEnableFlag[compID][ctuIdx] = 0;
        distUnfilter += distUnfilterCtu;
      }
    }
  }
  if (isChroma(channel))
  {
    setEnableFlag(&m_alfTileGroupParamTemp, channel, m_ctuEnableFlag);
    const int alfChromaIdc = m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cr];
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[channel];
  }


  return cost;
}


void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfTileGroupParam* alfTileGroupParam, const ChannelType channel )
{

  double costMin = DBL_MAX;

  EVCE_CTX* ctx = (EVCE_CTX*)cs.pCtx;

  AlfFilterShape* alfFilterShape = alfTileGroupParam->filterShapes[channel];

  const int numClasses = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  const int size = channel == CHANNEL_TYPE_LUMA ? 2 : 1;

  int covLrgIdx = size - 1;
  for (int iShapeIdx = 0; iShapeIdx < size; iShapeIdx++)
  {
    copyAlfParam( &m_alfTileGroupParamTemp, alfTileGroupParam );
    if (isLuma(channel))
    {
      m_alfTileGroupParamTemp.lumaFilterType = (AlfFilterType)(alfFilterShape[iShapeIdx].filterType);
    }
    double cost = costMin;

    //1. get unfiltered distortion
    if (isLuma(channel))
    {
      setCtuEnableFlag(m_ctuEnableFlag, channel, 1);
      getFrameStats(channel, covLrgIdx);
    }
    cost = getUnfilteredDistortion(m_alfCovarianceFrame[channel][covLrgIdx], channel);
    cost /= 1.001; // slight preference for unfiltered choice
    if (cost < costMin)
    {
      costMin = cost;
      setEnableFlag(alfTileGroupParam, channel, false);
      setCtuEnableFlag(m_ctuEnableFlagTmp, channel, 0);
      if (isChroma(channel))
      {
        alfTileGroupParam->chromaCtbPresentFlag = false;
      }
    }

    //2. all CTUs are on
    if (isChroma(channel))
    {
      m_alfTileGroupParamTemp.chromaCtbPresentFlag = true;
    }
    setEnableFlag(&m_alfTileGroupParamTemp, channel, true);
    setCtuEnableFlag(m_ctuEnableFlag, channel, 1);
    cost = getFilterCoeffAndCost(cs, 0, channel, isLuma(channel), iShapeIdx, uiCoeffBits);

    cost += m_lambda[channel];
    if (cost < costMin)
    {
      costMin = cost;
      copyAlfTileGroupParam(alfTileGroupParam, &m_alfTileGroupParamTemp, channel);
      setCtuEnableFlag(m_ctuEnableFlagTmp, channel, 1);
    }

    //3. CTU decision

    if( channel != CHANNEL_TYPE_CHROMA ) {

    double distUnfilter = 0;
    const int iterNum = 2 * 2 + 1;

    for (int iter = 0; iter < iterNum; iter++)
    {
      if ((iter & 0x01) == 0)
      {
        cost = m_lambda[channel] * uiCoeffBits;
        cost += deriveCtbAlfEnableFlags(cs, covLrgIdx, channel, numClasses, isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF, distUnfilter, true);
        cost += m_lambda[channel]*(m_numCTUsInPic); 
        
        if (cost < costMin)
        {
          costMin = cost;
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
          copyAlfTileGroupParam(alfTileGroupParam, &m_alfTileGroupParamTemp, channel);
          alfTileGroupParam->isCtbAlfOn = true;
        }
      }
      else
      {
        cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits);
      }
    }//for iter
    }
  }//for shapeIdx
  
  m_costAlfEncoder[channel] = costMin; 
  
  copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, channel);
}

void EncAdaptiveLoopFilter::alfReconstructor(CodingStructure& cs, AlfTileGroupParam* alfTileGroupParam, const pel * orgUnitBuf, const int oStride, pel * recExtBuf, int recStride, const ComponentID compID)
{
  const ChannelType channel = compID == COMPONENT_Y ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;

  reconstructCoeff(alfTileGroupParam, channel, false, isLuma(channel));
  EVCE_CTX* ctx = (EVCE_CTX*)(cs.pCtx);
  EVC_PIC* recPic = PIC_MODE(ctx);
  
  pel * recBuf = NULL;
  switch(compID)
  {
  case COMPONENT_Y:
      recBuf = recPic->y;
      break;
  case COMPONENT_Cb:
      recBuf = recPic->u;
      break;
  case COMPONENT_Cr:
      recBuf = recPic->v;
      break;
  default:
      assert(0);
  }

  {
    if (alfTileGroupParam->enabledFlag[compID])
    {
      int ctuIdx = 0;
      const int chromaScaleX = isLuma(channel) ? 0 : 1;
      const int chromaScaleY = isLuma(channel) ? 0 : 1; //getComponentScaleY(compID, recBuf.chromaFormat);
      AlfFilterType filterType = compID == COMPONENT_Y ? ALF_FILTER_7 : ALF_FILTER_5;
      short* coeff = compID == COMPONENT_Y ? m_coeffFinal : alfTileGroupParam->chromaCoeff;
      for (int yPos = 0; yPos < recPic->h_l; yPos += ctx->max_cuwh)
      {
        for (int xPos = 0; xPos < recPic->w_l; xPos += ctx->max_cuwh)
        {
          const int width = (xPos + ctx->max_cuwh > recPic->w_l) ? (recPic->w_l - xPos) : ctx->max_cuwh;
          const int height = (yPos + ctx->max_cuwh > recPic->h_l) ? (recPic->h_l - yPos) : ctx->max_cuwh;

          Area blk = { xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY };

          if (m_ctuEnableFlag[compID][ctuIdx])
          {
            {
              int stride = isLuma(channel) ? recPic->s_l : recPic->s_c;

              if (filterType == ALF_FILTER_5)
              {
                m_AdaptiveLoopFilter.m_filter5x5Blk(m_classifier, recBuf, stride, recExtBuf, recStride, &blk, compID, coeff, &(m_clpRngs.comp[(int)compID]));
              }
              else if (filterType == ALF_FILTER_7)
              {
                m_AdaptiveLoopFilter.m_filter7x7Blk(m_classifier, recBuf, stride, recExtBuf, recStride, &blk, compID, coeff, &(m_clpRngs.comp[(int)compID]));
              }
              else
              {
                CHECK(0, "Wrong ALF filter type");
              }
            }
          }
          ctuIdx++;
        }
      }
    }
  }
}

#if APS_ALF_SEQ_FIX
void EncAdaptiveLoopFilter::alfTemporalEncoderAPS(CodingStructure& cs, AlfTileGroupParam* alfTileGroupParam)
{
    if (!alfTileGroupParam->enabledFlag[COMPONENT_Y])
    {
        setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, true);
        getFrameStats(CHANNEL_TYPE_CHROMA, 0);
        m_costAlfEncoder[CHANNEL_TYPE_CHROMA] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + lengthTruncatedUnary(0, 3) * m_lambda[CHANNEL_TYPE_CHROMA];
        setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, false);
    }

    EVCE_CTX* ctx = (EVCE_CTX*)cs.pCtx;

    double cost[MAX_NUM_CHANNEL_TYPE] = { DBL_MAX, DBL_MAX };
    const int tempLayerId = ctx->layer_id; //cs.tile_group->getTLayer();

    AlfTileGroupParam *pcStoredAlfPara = ctx->tile_group_type == TILE_GROUP_I ? NULL : m_acAlfLineBuffer;

    copyAlfParam(&m_alfTileGroupParamTemp, alfTileGroupParam);
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);

    if (pcStoredAlfPara != NULL && m_acAlfLineBufferCurrentSize > 0)
    {
        int prevIdx = 0;
        for (int bufIdx2 = 0; bufIdx2 < m_acAlfLineBufferCurrentSize && bufIdx2 < APS_MAX_NUM; bufIdx2++)
        {
            int bufIdx = bufIdx2;
            bufIdx = m_alfIndxInScanOrder[bufIdx2];
            // Identical check is implemented at the APS level, code below may be redundant or replaced by reused getProtectIdxFromList()
            {
                if ((pcStoredAlfPara[bufIdx].tLayer > tempLayerId) && (ctx->param.i_period != 0)) // this condition ensures that encoder does not break temporal scalability in RA
                {
                    continue;
                }
                if ((m_currentPoc > pcStoredAlfPara[bufIdx].m_maxIdrPoc + ctx->param.i_period) && (ctx->param.i_period != 0))
                {
                    continue;
                }

                if ((m_currentPoc > m_lastIdrPoc) // current POC is after 2nd IDR 
                    && (pcStoredAlfPara[bufIdx].m_filterPoc < m_lastIdrPoc) // POC of checked ALF is before 2nd IDR
                    )

                {
                    continue;
                }

                if ((m_currentPoc > pcStoredAlfPara[bufIdx].m_maxIdrPoc) // current POC is after 2nd IDR 
                    && (pcStoredAlfPara[bufIdx].m_filterPoc < pcStoredAlfPara[bufIdx].m_maxIdrPoc) // POC of checked ALF is before 2nd IDR
                    )

                {
                    continue;
                }
            }

            copyAlfParam(&m_alfTileGroupParamTemp, &(pcStoredAlfPara[bufIdx]));

            for (int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
            {
                ChannelType channel = (ChannelType)ch;
                if ((channel == CHANNEL_TYPE_CHROMA && !m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y]) || (channel == CHANNEL_TYPE_CHROMA && !m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cb] && !m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cr]))
                {
                    setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                    getFrameStats(channel, 0);
                    cost[channel] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + lengthTruncatedUnary(0, 3) * m_lambda[channel];
                    setEnableFlag(&m_alfTileGroupParamTemp, channel, false);
                    setCtuEnableFlag(m_ctuEnableFlag, channel, false);
                }
                else
                {
                    double costCtbEnable = DBL_MAX;
                    if (channel == CHANNEL_TYPE_CHROMA && m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y])
                    {
                        setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                        getFrameStats(CHANNEL_TYPE_CHROMA, 0);
                        costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA);
                        // reconstruct chroma coeffs
                        reconstructCoeff(&m_alfTileGroupParamTemp, channel, true, isLuma(channel));
                        for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
                        {
                            for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
                            {
                                m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfTileGroupParamTemp.chromaCoeff[i];
                            }
                        }
                        // calculate filtered cost
                        costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                        costCtbEnable += lengthTruncatedUnary(3, 3) * m_lambda[channel];
                    }

                    double distUnfilter;
                    int iShapeIdx = isLuma(channel) ? ALF_FILTER_7 : ALF_FILTER_5;
                    if (channel == CHANNEL_TYPE_LUMA)
                        cost[channel] = deriveCtbAlfEnableFlags(cs, iShapeIdx, channel, isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1, m_filterShapes[channel][iShapeIdx].numCoeff, distUnfilter, true);

                    if (channel == CHANNEL_TYPE_CHROMA && m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y])
                    {
                        if (costCtbEnable < cost[channel])
                        {
                            setEnableFlag(&m_alfTileGroupParamTemp, channel, true);
                            setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                            m_alfTileGroupParamTemp.chromaCtbPresentFlag = true;
                            cost[channel] = costCtbEnable;
                        }
                    }
                }
                if (channel == CHANNEL_TYPE_LUMA)
                {
                    cost[channel] += m_lambda[channel] * APS_MAX_NUM_IN_BITS;

                    for (int i = 0; i < ctx->f_lcu; i++)
                    {
                        if (m_alfTileGroupParamTemp.alfCtuEnableFlag[0][i] == 0) {
                            m_alfTileGroupParamTemp.isCtbAlfOn = true;
                            break;
                        }
                        else
                            m_alfTileGroupParamTemp.isCtbAlfOn = false;
                    }
                    if (m_alfTileGroupParamTemp.isCtbAlfOn)
                        cost[channel] += m_lambda[channel] * (ctx->f_lcu);
                }
            } // channel loop

            bool isCurrentBetter = (cost[CHANNEL_TYPE_LUMA] + cost[CHANNEL_TYPE_CHROMA]) < (m_costAlfEncoder[CHANNEL_TYPE_LUMA] + m_costAlfEncoder[CHANNEL_TYPE_CHROMA]);
            //      isCurrentBetter = false;
            if (isCurrentBetter)
            {
                m_costAlfEncoder[CHANNEL_TYPE_LUMA] = cost[CHANNEL_TYPE_LUMA];

                copyAlfParam(alfTileGroupParam, &m_alfTileGroupParamTemp);
                alfTileGroupParam->prevIdx = bufIdx;
                alfTileGroupParam->temporalAlfFlag = true;
                alfTileGroupParam->chromaCtbPresentFlag = false;
                copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
                copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);
            }
        } // prevIdx search
        copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
        copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA);
    } // end if 
}
#endif

void EncAdaptiveLoopFilter::alfTemporalEncoder(CodingStructure& cs, AlfTileGroupParam* alfTileGroupParam)
{
  if (!alfTileGroupParam->enabledFlag[COMPONENT_Y])
  {
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, true);
    getFrameStats(CHANNEL_TYPE_CHROMA, 0);
    m_costAlfEncoder[CHANNEL_TYPE_CHROMA] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + lengthTruncatedUnary(0, 3) * m_lambda[CHANNEL_TYPE_CHROMA];
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, false);
  }

  EVCE_CTX* ctx = (EVCE_CTX*)cs.pCtx;

  double cost[MAX_NUM_CHANNEL_TYPE] = { DBL_MAX, DBL_MAX };
  const int tempLayerId = ctx->layer_id; //cs.tile_group->getTLayer();

  AlfTileGroupParam *pcStoredAlfPara = ctx->tile_group_type == TILE_GROUP_I ? NULL : m_acAlfLineBuffer;

  copyAlfParam(&m_alfTileGroupParamTemp, alfTileGroupParam);
  copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
  copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);

  if (pcStoredAlfPara != NULL && m_acAlfLineBufferCurrentSize > 0)
  {
    int prevIdx = 0;
    for (int bufIdx = 0; bufIdx < m_acAlfLineBufferCurrentSize && bufIdx < ALF_TEMPORAL_WITH_LINE_BUFFER; bufIdx++)
    {
      if (pcStoredAlfPara[bufIdx].tLayer > tempLayerId) // this condition ensures that encoder does not break temporal scalability
      {
        continue;
      }

      copyAlfParam(&m_alfTileGroupParamTemp, &(pcStoredAlfPara[bufIdx]));

      for (int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
      {
        ChannelType channel = (ChannelType)ch;
        if ((channel == CHANNEL_TYPE_CHROMA && !m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y]) || (channel == CHANNEL_TYPE_CHROMA && !m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cb] && !m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cr]))
        {
          setCtuEnableFlag(m_ctuEnableFlag, channel, true);
          getFrameStats(channel, 0);
          cost[channel] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + lengthTruncatedUnary(0, 3) * m_lambda[channel];
          setEnableFlag(&m_alfTileGroupParamTemp, channel, false);
          setCtuEnableFlag(m_ctuEnableFlag, channel, false);
        }
        else
        {
          double costCtbEnable = DBL_MAX;
          if (channel == CHANNEL_TYPE_CHROMA && m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y])
          {
            setCtuEnableFlag(m_ctuEnableFlag, channel, true);
            getFrameStats(CHANNEL_TYPE_CHROMA, 0);
            costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA);
            // reconstruct chroma coeffs
            reconstructCoeff(&m_alfTileGroupParamTemp, channel, true, isLuma(channel));
            for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
            {
              for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
              {
                m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfTileGroupParamTemp.chromaCoeff[i];
              }
            }
            // calculate filtered cost
            costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
            costCtbEnable += lengthTruncatedUnary(3, 3) * m_lambda[channel];
          }

          double distUnfilter;
          int iShapeIdx = isLuma(channel) ? ALF_FILTER_7 : ALF_FILTER_5;
          if( channel != CHANNEL_TYPE_CHROMA )
            cost[channel] = deriveCtbAlfEnableFlags(cs, iShapeIdx, channel, isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1, m_filterShapes[channel][iShapeIdx].numCoeff, distUnfilter, true);
          
          if (channel == CHANNEL_TYPE_CHROMA && m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y])
          {
            if (costCtbEnable < cost[channel])
            {
              setEnableFlag(&m_alfTileGroupParamTemp, channel, true);
              setCtuEnableFlag(m_ctuEnableFlag, channel, true);
              m_alfTileGroupParamTemp.chromaCtbPresentFlag = true;
              cost[channel] = costCtbEnable;
            }
          }
        }
        if (channel == CHANNEL_TYPE_LUMA)
        {
#if ALF_PARAMETER_APS
            cost[channel] += m_lambda[channel] * lengthUvlc(bufIdx);
#else
          cost[channel] += m_lambda[channel] * lengthUvlc(prevIdx); 
#endif

          
          for( int i = 0; i < ctx->f_lcu; i++ )
          {
              if( m_alfTileGroupParamTemp.alfCtuEnableFlag[0][i] == 0 ) {
                  m_alfTileGroupParamTemp.isCtbAlfOn = true;
                  break;
              } else
                  m_alfTileGroupParamTemp.isCtbAlfOn = false;
          }
          if( m_alfTileGroupParamTemp.isCtbAlfOn )
              cost[channel] += m_lambda[channel]*(ctx->f_lcu);
        }
      } // channel loop

      bool isCurrentBetter = (cost[CHANNEL_TYPE_LUMA] + cost[CHANNEL_TYPE_CHROMA] ) < (m_costAlfEncoder[CHANNEL_TYPE_LUMA] + m_costAlfEncoder[CHANNEL_TYPE_CHROMA] );
      if (isCurrentBetter)
      {
        m_costAlfEncoder[CHANNEL_TYPE_LUMA] = cost[CHANNEL_TYPE_LUMA];

        copyAlfParam( alfTileGroupParam, &m_alfTileGroupParamTemp);
#if ALF_PARAMETER_APS
        alfTileGroupParam->prevIdx = bufIdx;
#else
        alfTileGroupParam->prevIdx = prevIdx;
#endif
        alfTileGroupParam->temporalAlfFlag = true;
        alfTileGroupParam->chromaCtbPresentFlag = false;   //to check
        copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
        copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);
      }
#if !ALF_PARAMETER_APS
      prevIdx++;
#endif
    } // prevIdx search
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA);
  } // end if 
}

void EncAdaptiveLoopFilter::xDeriveCovFromLgrTapFilter(AlfCovariance& covLgr, AlfCovariance& covSml, int* patternSml)
{
  covSml.pixAcc = covLgr.pixAcc;
  for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
  {
    if (patternSml[i] > 0)
    {
      covSml.y[patternSml[i] - 1] = covLgr.y[i];
      for (int j = 0; j < MAX_NUM_ALF_LUMA_COEFF; j++)
      {
        if (patternSml[j] > 0)
        {
          covSml.E[patternSml[i] - 1][patternSml[j] - 1] = covLgr.E[i][j];
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyAlfTileGroupParam(AlfTileGroupParam* alfTileGroupParamDst, AlfTileGroupParam* alfTileGroupParamSrc, ChannelType channel)
{
  if (isLuma(channel))
  {
    memcpy(alfTileGroupParamDst, alfTileGroupParamSrc, sizeof(AlfTileGroupParam));
  }
  else
  {
    alfTileGroupParamDst->enabledFlag[COMPONENT_Cb] = alfTileGroupParamSrc->enabledFlag[COMPONENT_Cb];
    alfTileGroupParamDst->enabledFlag[COMPONENT_Cr] = alfTileGroupParamSrc->enabledFlag[COMPONENT_Cr];
    alfTileGroupParamDst->chromaCtbPresentFlag = alfTileGroupParamSrc->chromaCtbPresentFlag;
    memcpy(alfTileGroupParamDst->chromaCoeff, alfTileGroupParamSrc->chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
  }
}

double EncAdaptiveLoopFilter::getFilterCoeffAndCost(CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits)
{
  const int size = channel == CHANNEL_TYPE_LUMA ? 2 : 1;

  if (bReCollectStat)
  {
    getFrameStats(channel, (int)size - 1);
  }

  double dist = distUnfilter;
  uiCoeffBits = 0;
  int uiTileGroupFlag = 0;
  AlfFilterShape alfFilterShape = m_alfTileGroupParamTemp.filterShapes[channel][iShapeIdx];
  if (isLuma(channel))
  {
    dist += mergeFiltersAndCost(&m_alfTileGroupParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][ALF_FILTER_7], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits);
  }
  else
  {
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant(m_filterCoeffQuant, m_alfCovarianceFrame[channel][iShapeIdx][0].E, m_alfCovarianceFrame[channel][iShapeIdx][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true);
    memcpy(m_filterCoeffSet[0], m_filterCoeffQuant, sizeof(*m_filterCoeffQuant) * alfFilterShape.numCoeff);
    const int alfChromaIdc = m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Cr];
    for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
    {
      m_alfTileGroupParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
    }
    uiCoeffBits += getCoeffRate(&m_alfTileGroupParamTemp, true);
    uiTileGroupFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }

  double rate = uiCoeffBits + uiTileGroupFlag;
  if (isLuma(channel) || (!m_alfTileGroupParamTemp.chromaCtbPresentFlag))
  {
    if (isChroma(channel))
    {
      CHECK(m_alfTileGroupParamTemp.chromaCtbPresentFlag, "chromaCTB is on");
    }
    else
    {
      CHECK(!m_alfTileGroupParamTemp.enabledFlag[COMPONENT_Y], "TileGroup Y is off");
    }
  }
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getCoeffRate(AlfTileGroupParam* alfTileGroupParam, bool isChroma)
{
  int iBits = 0;
  if (!isChroma)
  {
    iBits++;                                               // alf_coefficients_delta_flag
    if (!alfTileGroupParam->coeffDeltaFlag)
    {
      if (alfTileGroupParam->numLumaFilters > 1)
      {
        iBits++;                                           // coeff_delta_pred_mode_flag
      }
    }
  }

  memset(m_bitsCoeffScan, 0, sizeof(m_bitsCoeffScan));
  AlfFilterShape alfShape;
  init_AlfFilterShape(&alfShape, isChroma ? 5 : (alfTileGroupParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7));
  const int maxGolombIdx = getMaxGolombIdx((AlfFilterType)alfShape.filterType);
  const short* coeff = isChroma ? alfTileGroupParam->chromaCoeff : alfTileGroupParam->lumaCoeff;
  const int numFilters = isChroma ? 1 : alfTileGroupParam->numLumaFilters;

  // vlc for all
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (isChroma || !alfTileGroupParam->coeffDeltaFlag || alfTileGroupParam->filterCoeffFlag[ind])
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        int coeffVal = abs(coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i]);

        for (int k = 1; k < 15; k++)
        {
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
        }
      }
    }
  }

  int kMin = getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);

  // Golomb parameters
  iBits += lengthUvlc(kMin - 1);  // "min_golomb_order"
  int golombOrderIncreaseFlag = 0;

  for (int idx = 0; idx < maxGolombIdx; idx++)
  {
    golombOrderIncreaseFlag = (m_kMinTab[idx] != kMin) ? 1 : 0;
    CHECK(!(m_kMinTab[idx] <= kMin + 1), "ALF Golomb parameter not consistent");
    iBits += golombOrderIncreaseFlag;                           //golomb_order_increase_flag
    kMin = m_kMinTab[idx];
  }

  if (!isChroma)
  {
    if (alfTileGroupParam->coeffDeltaFlag)
    {
      iBits += numFilters;             //filter_coefficient_flag[i]
    }
  }

  // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (!isChroma && !alfTileGroupParam->filterCoeffFlag[ind] && alfTileGroupParam->coeffDeltaFlag)
    {
      continue;
    }

    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      iBits += lengthGolomb(coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], m_kMinTab[alfShape.golombIdx[i]]);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }
  return iBits;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion(AlfCovariance* cov, ChannelType channel)
{
  double dist = 0;
  if (isLuma(channel))
  {
    dist = getUnfilteredDistortion(cov, MAX_NUM_ALF_CLASSES);
  }
  else
  {
    dist = getUnfilteredDistortion(cov, 1); // + lengthTruncatedUnary( 0, 3 ) * m_lambda[COMPONENT_Cb];  // @hegilmez: included the rate outside this function
  }
  return dist;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion(AlfCovariance* cov, const int numClasses)
{
  double dist = 0;
  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

double EncAdaptiveLoopFilter::getFilteredDistortion(AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff)
{
  double dist = 0;

  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {

    dist += calcErrorForCoeffs(cov[classIdx].E, cov[classIdx].y, m_filterCoeffSet[classIdx], numCoeff, m_NUM_BITS);
  }

  return dist;
}


double EncAdaptiveLoopFilter::mergeFiltersAndCost(AlfTileGroupParam* alfTileGroupParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits)
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = DBL_MAX;
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;

  findBestFixedFilter(alfTileGroupParam, covFrame);

  if (alfShape.filterType == ALF_FILTER_5)
  {
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      xDeriveCovFromLgrTapFilter(covFrame[classIdx], m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][ALF_FILTER_5][classIdx], alfShape.patternToLargeFilter);
    }
    covFrame = m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][ALF_FILTER_5];
  }
  mergeClasses(covFrame, covMerged, MAX_NUM_ALF_CLASSES, m_filterIndices);

  while (numFilters >= 1)
  {
    dist = deriveFilterCoeffs(covFrame, covMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab);
    distForce0 = getDistForce0(alfShape, numFilters, errorForce0CoeffTab, codedVarBins);
    coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, predMode);
    coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFilters, codedVarBins);
    cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

    if (cost0 < cost)
    {
      cost = cost0;
    }

    if (cost <= costMin)
    {
      costMin = cost;
      numFiltersBest = numFilters;
      bestPredMode = predMode;
    }
    numFilters--;
  }

  dist = deriveFilterCoeffs(covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab);
  coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, predMode);
  distForce0 = getDistForce0(alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins);
  coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins);
  cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

  alfTileGroupParam->numLumaFilters = numFiltersBest;
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfTileGroupParam->coeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
    alfTileGroupParam->coeffDeltaPredModeFlag = bestPredMode;
  }
  else
  {
    distReturn = distForce0;
    alfTileGroupParam->coeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    for(int i=0; i<MAX_NUM_ALF_CLASSES; i++)
        alfTileGroupParam->filterCoeffFlag[i] = BOOL(codedVarBins[i]);
    alfTileGroupParam->coeffDeltaPredModeFlag = 0;

    for (int varInd = 0; varInd < numFiltersBest; varInd++)
    {
      if (codedVarBins[varInd] == 0)
      {
        memset(m_filterCoeffSet[varInd], 0, sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
      }
    }
  }
  for (int ind = 0; ind < alfTileGroupParam->numLumaFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff; i++)
    {
      if (alfTileGroupParam->coeffDeltaPredModeFlag)
      {
        alfTileGroupParam->lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_diffFilterCoeff[ind][i];
      }
      else
      {
        alfTileGroupParam->lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      }
    }
  }

  memcpy(alfTileGroupParam->filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof(short) * MAX_NUM_ALF_CLASSES);
  const int iNumFixedFilterPerClass = ALF_FIXED_FILTER_NUM;
  if (iNumFixedFilterPerClass > 0)
  {
    int fixedFilterPattern = alfTileGroupParam->fixedFilterIdx[0] > 0 ? 1 : 0;
    for (int classIdx = 1; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      int iCurrFixedFilterPattern = alfTileGroupParam->fixedFilterIdx[classIdx] > 0 ? 1 : 0;
      if (iCurrFixedFilterPattern != fixedFilterPattern)
      {
        fixedFilterPattern = 2;
        break;
      }
    }
    alfTileGroupParam->fixedFilterPattern = fixedFilterPattern;
  }

  uiCoeffBits += getNonFilterCoeffRate(alfTileGroupParam);
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate(AlfTileGroupParam* alfTileGroupParam)
{
  int len = 1   // filter_type
    + 1   // alf_coefficients_delta_flag
    + lengthTruncatedUnary(0, 3)    // chroma_idc = 0, it is signalled when ALF is enabled for luma
    + getTBlength(alfTileGroupParam->numLumaFilters - 1, MAX_NUM_ALF_CLASSES);   //numLumaFilters

                                                                            //add bits of fixed filter
  char codetab_pred[3] = { 1, 0, 2 };
  const int iNumFixedFilterPerClass = ALF_FIXED_FILTER_NUM;
  if (iNumFixedFilterPerClass > 0)
  {
    len += lengthGolomb(codetab_pred[alfTileGroupParam->fixedFilterPattern], 0);
    if (alfTileGroupParam->fixedFilterPattern == 2)
    {
      len += MAX_NUM_ALF_CLASSES;  //"fixed_filter_flag" for each class
    }
    if (alfTileGroupParam->fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
    {
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
        if (alfTileGroupParam->fixedFilterIdx[classIdx] > 0)
        {
          len += getTBlength(alfTileGroupParam->fixedFilterIdx[classIdx] - 1, iNumFixedFilterPerClass);
        }
      }
    }
  }

  if (alfTileGroupParam->numLumaFilters > 1)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
      len += getTBlength((int)alfTileGroupParam->filterCoeffDeltaIdx[i], alfTileGroupParam->numLumaFilters);  //filter_coeff_delta[i]
    }
  }
  return len;
}

int EncAdaptiveLoopFilter::lengthTruncatedUnary(int symbol, int maxSymbol)
{
  if (maxSymbol == 0)
  {
    return 0;
  }

  bool codeLast = (maxSymbol > symbol);
  int bins = 0;
  int numBins = 0;
  while (symbol--)
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if (codeLast)
  {
    bins <<= 1;
    numBins++;
  }

  return numBins;
}

int EncAdaptiveLoopFilter::getTBlength(int uiSymbol, const int uiMaxSymbol)
{
  int uiThresh;
  if (uiMaxSymbol > 256)
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while (uiThreshVal <= uiMaxSymbol)
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  assert(uiVal <= uiMaxSymbol);
  assert((uiVal << 1) > uiMaxSymbol);
  assert(uiSymbol < uiMaxSymbol);
  int b = uiMaxSymbol - uiVal;
  assert(b < uiVal);
  if (uiSymbol < uiVal - b)
  {
    return uiThresh;
  }
  else
  {
    return uiThresh + 1;
  }
}

int EncAdaptiveLoopFilter::getCostFilterCoeffForce0(AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins)
{
  const int maxGolombIdx = getMaxGolombIdx((AlfFilterType)alfShape.filterType);
  memset(m_bitsCoeffScan, 0, sizeof(m_bitsCoeffScan));

  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (!codedVarBins[ind])
    {
      continue;
    }
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      int coeffVal = abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (int k = 1; k < 15; k++)
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
      }
    }
  }

  int kMin = getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);

  // Coding parameters
  int len = kMin           //min_golomb_order
    + maxGolombIdx   //golomb_order_increase_flag
    + numFilters;    //filter_coefficient_flag[i]

                     // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (codedVarBins[ind])
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        len += lengthGolomb(abs(pDiffQFilterCoeffIntPP[ind][i]), m_kMinTab[alfShape.golombIdx[i]]); // alf_coeff_luma_delta[i][j]
      }
    }
  }

  return len;
}

int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode(AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode)
{
  int ratePredMode0 = getCostFilterCoeff(alfShape, filterSet, numFilters);

  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (ind == 0)
    {
      memcpy(filterCoeffDiff[ind], filterSet[ind], sizeof(int) * alfShape.numCoeff);
    }
    else
    {
      for (int i = 0; i < alfShape.numCoeff; i++)
      {
        filterCoeffDiff[ind][i] = filterSet[ind][i] - filterSet[ind - 1][i];
      }
    }
  }

  int ratePredMode1 = getCostFilterCoeff(alfShape, filterCoeffDiff, numFilters);

  predMode = (ratePredMode1 < ratePredMode0 && numFilters > 1) ? 1 : 0;

  return (numFilters > 1 ? 1 : 0)        // coeff_delta_pred_mode_flag
    + (predMode ? ratePredMode1 : ratePredMode0); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
}

int EncAdaptiveLoopFilter::getCostFilterCoeff(AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters)
{
  const int maxGolombIdx = getMaxGolombIdx((AlfFilterType)alfShape.filterType);

  memset(m_bitsCoeffScan, 0, sizeof(m_bitsCoeffScan));

  for (int ind = 0; ind < numFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      int coeffVal = abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (int k = 1; k < 15; k++)
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
      }
    }
  }

  int kMin = getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);

  int len = kMin           //min_golomb_order
    + maxGolombIdx;  //golomb_order_increase_flag

  len += lengthFilterCoeffs(alfShape, numFilters, pDiffQFilterCoeffIntPP, m_kMinTab);  // alf_coeff_luma_delta[i][j]

  return len;
}

int EncAdaptiveLoopFilter::lengthFilterCoeffs(AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab)
{
  int bitCnt = 0;

  for (int ind = 0; ind < numFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      bitCnt += lengthGolomb(abs(FilterCoeff[ind][i]), kMinTab[alfShape.golombIdx[i]]);
    }
  }
  return bitCnt;
}

double EncAdaptiveLoopFilter::getDistForce0(AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins)
{
  static int bitsVarBin[MAX_NUM_ALF_CLASSES];

  memset(m_bitsCoeffScan, 0, sizeof(m_bitsCoeffScan));
  for (int ind = 0; ind < numFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      int coeffVal = abs(m_filterCoeffSet[ind][i]);
      for (int k = 1; k < 15; k++)
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
      }
    }
  }

  getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);

  for (int ind = 0; ind < numFilters; ++ind)
  {
    bitsVarBin[ind] = 0;
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      bitsVarBin[ind] += lengthGolomb(abs(m_filterCoeffSet[ind][i]), m_kMinTab[alfShape.golombIdx[i]]);
    }
  }

  double distForce0 = getDistCoeffForce0(codedVarBins, errorTabForce0Coeff, bitsVarBin, numFilters);

  return distForce0;
}

int EncAdaptiveLoopFilter::getGolombKMin(AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB])
{
  int kStart;
  const int maxGolombIdx = getMaxGolombIdx((AlfFilterType)alfShape.filterType);

  int minBitsKStart = INT_MAX;
  int minKStart = -1;

  for (int k = 1; k < 8; k++)
  {
    int bitsKStart = 0; kStart = k;
    for (int scanPos = 0; scanPos < maxGolombIdx; scanPos++)
    {
      int kMin = kStart;
      int minBits = bitsCoeffScan[scanPos][kMin];

      if (bitsCoeffScan[scanPos][kStart + 1] < minBits)
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if (bitsKStart < minBitsKStart)
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for (int scanPos = 0; scanPos < maxGolombIdx; scanPos++)
  {
    int kMin = kStart;
    int minBits = bitsCoeffScan[scanPos][kMin];

    if (bitsCoeffScan[scanPos][kStart + 1] < minBits)
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  return minKStart;
}

double EncAdaptiveLoopFilter::getDistCoeffForce0(bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters)
{
  double distForce0 = 0;
  memset(codedVarBins, 0, sizeof(*codedVarBins) * MAX_NUM_ALF_CLASSES);

  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    double costDiff = errorForce0CoeffTab[filtIdx][0] - (errorForce0CoeffTab[filtIdx][1] + m_lambda[COMPONENT_Y] * bitsVarBin[filtIdx]);
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }
  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc(int uiCode)
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}

int EncAdaptiveLoopFilter::lengthGolomb(int coeffVal, int k)
{
  int m = k == 0 ? 1 : (2 << (k - 1));
  int q = coeffVal / m;
  if (coeffVal != 0)
  {
    return q + 2 + k;
  }
  else
  {
    return q + 1 + k;
  }
}

double EncAdaptiveLoopFilter::deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2])
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    tmpCov.reset();
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      if (filterIndices[classIdx] == filtIdx)
      {
        tmpCov += cov[classIdx];
      }
    }

    // Find coeffcients
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant(m_filterCoeffQuant, tmpCov.E, tmpCov.y, alfShape.numCoeff, alfShape.weights, m_NUM_BITS);
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];

    // store coeff
    memcpy(m_filterCoeffSet[filtIdx], m_filterCoeffQuant, sizeof(int)*alfShape.numCoeff);
  }
  return error;
}

double EncAdaptiveLoopFilter::deriveCoeffQuant(int *filterCoeffQuant, double **E, double *y, const int numCoeff, int* weights, const int bitDepth, const bool bChroma)
{
  const int factor = 1 << (bitDepth - 1);
  static int filterCoeffQuantMod[MAX_NUM_ALF_LUMA_COEFF];
  static double filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  gnsSolveByChol(E, y, filterCoeff, numCoeff);
  roundFiltCoeff(filterCoeffQuant, filterCoeff, numCoeff, factor);
  const int targetCoeffSumInt = 0;
  int quantCoeffSum = 0;
  for (int i = 0; i < numCoeff; i++)
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
  }

  int count = 0;
  while (quantCoeffSum != targetCoeffSumInt && count < 10)
  {
    int sign = quantCoeffSum > targetCoeffSumInt ? 1 : -1;
    int diff = (quantCoeffSum - targetCoeffSumInt) * sign;

    double errMin = DBL_MAX;
    int minInd = -1;

    for (int k = 0; k < numCoeff; k++)
    {
      if (weights[k] <= diff)
      {
        memcpy(filterCoeffQuantMod, filterCoeffQuant, sizeof(int) * numCoeff);

        filterCoeffQuantMod[k] -= sign;
        double error = calcErrorForCoeffs(E, y, filterCoeffQuantMod, numCoeff, bitDepth);

        if (error < errMin)
        {
          errMin = error;
          minInd = k;
        }
      }
    }

    if (minInd != -1)
    {
      filterCoeffQuant[minInd] -= sign;
    }

    quantCoeffSum = 0;
    for (int i = 0; i < numCoeff; i++)
    {
      quantCoeffSum += weights[i] * filterCoeffQuant[i];
    }
    ++count;
  }
  if (count == 10)
  {
    memset(filterCoeffQuant, 0, sizeof(int) * numCoeff);
  }

  //512 -> 1, (64+32+4+2)->0.199
  int max_value = 512 + 64 + 32 + 4 + 2;
  int min_value = -max_value;
  for (int i = 0; i < numCoeff - 1; i++)
  {
    filterCoeffQuant[i] = min(max_value, max(min_value, filterCoeffQuant[i]));
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }

  quantCoeffSum = 0;
  for (int i = 0; i < numCoeff - 1; i++)
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }
  filterCoeffQuant[numCoeff - 1] = -quantCoeffSum;
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);

  double error = calcErrorForCoeffs(E, y, filterCoeffQuant, numCoeff, bitDepth);
  return error;
}

double EncAdaptiveLoopFilter::calcErrorForCoeffs(double **E, double *y,  const  int *coeff, const int numCoeff, const int bitDepth)
{
  double factor = 1 << (bitDepth - 1);
  double error = 0;

  for (int i = 0; i < numCoeff; i++)   //diagonal
  {
    double sum = 0;
    for (int j = i + 1; j < numCoeff; j++)
    {
      sum += E[i][j] * coeff[j];
    }
    error += ((E[i][i] * coeff[i] + sum * 2) / factor - 2 * y[i]) * coeff[i];
  }

  return error / factor;
}

void EncAdaptiveLoopFilter::roundFiltCoeff(int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor)
{
  for (int i = 0; i < numCoeff; i++)
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int(filterCoeff[i] * sign * factor + 0.5) * sign;
  }
}

void EncAdaptiveLoopFilter::findBestFixedFilter(AlfTileGroupParam* alfTileGroupParam, AlfCovariance* cov)
{
  double factor = 1 << (m_NUM_BITS - 1);
  for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
  {
    double errorMin = cov[classIdx].pixAcc;
    alfTileGroupParam->fixedFilterIdx[classIdx] = 0;
    for (int filterIdx = 0; filterIdx < ALF_FIXED_FILTER_NUM; filterIdx++)
    {
      int fixedFilterIdx = m_classToFilterMapping[classIdx][filterIdx];
      double errorFilter = cov[classIdx].pixAcc + calcErrorForCoeffs(cov[classIdx].E, cov[classIdx].y, m_fixedFilterCoeff[fixedFilterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
      if (errorFilter < errorMin)
      {
        errorMin = errorFilter;
        alfTileGroupParam->fixedFilterIdx[classIdx] = filterIdx + 1;
      }
    }
    //update stat
    int finalFilterIdx = alfTileGroupParam->fixedFilterIdx[classIdx];
    if (finalFilterIdx > 0)
    {
      int fixedFilterIdx = m_classToFilterMapping[classIdx][finalFilterIdx - 1];
      cov[classIdx].pixAcc = errorMin;
      //update y
      for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
      {
        double sum = 0;
        for (int j = 0; j < MAX_NUM_ALF_LUMA_COEFF; j++)
        {
          sum += cov[classIdx].E[i][j] * m_fixedFilterCoeff[fixedFilterIdx][j];
        }
        sum /= factor;
        cov[classIdx].y[i] -= sum;
      }
    }
  }
}

void EncAdaptiveLoopFilter::mergeClasses(AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES])
{
  static bool availableClass[MAX_NUM_ALF_CLASSES];
  static uint8_t indexList[MAX_NUM_ALF_CLASSES];
  static uint8_t indexListTemp[MAX_NUM_ALF_CLASSES];
  int numRemaining = numClasses;

  memset(filterIndices, 0, sizeof(short) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES);

  for (int i = 0; i < numClasses; i++)
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i] = i;
    availableClass[i] = true;
    covMerged[i] = cov[i];
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  while (numRemaining > 2)
  {
    double errorMin = DBL_MAX;
    int bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for (int i = 0; i < numClasses - 1; i++)
    {
      if (availableClass[i])
      {
        for (int j = i + 1; j < numClasses; j++)
        {
          if (availableClass[j])
          {
            double error1 = calculateError(covMerged[i]);
            double error2 = calculateError(covMerged[j]);

            tmpCov.add(covMerged[i], covMerged[j]);
            double error = calculateError(tmpCov) - error1 - error2;

            if (error < errorMin)
            {
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    availableClass[bestToMergeIdx2] = false;

    for (int i = 0; i < numClasses; i++)
    {
      if (indexList[i] == bestToMergeIdx2)
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if (numRemaining <= numClasses)
    {
      memcpy(indexListTemp, indexList, sizeof(uint8_t) * numClasses);

      bool exist = false;
      int ind = 0;

      for (int j = 0; j < numClasses; j++)
      {
        exist = false;
        for (int i = 0; i < numClasses; i++)
        {
          if (indexListTemp[i] == j)
          {
            exist = true;
            break;
          }
        }

        if (exist)
        {
          for (int i = 0; i < numClasses; i++)
          {
            if (indexListTemp[i] == j)
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStats(ChannelType channel, int iShapeIdx)
{
  int numClasses = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  for (int i = 0; i < numClasses; i++)
  {
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset();
  }
  if (isLuma(channel))
  {
    getFrameStat(m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], numClasses);
  }
  else
  {
    getFrameStat(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], numClasses);
    getFrameStat(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], numClasses);
  }
}

void EncAdaptiveLoopFilter::getFrameStat(AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses)
{
  for (int i = 0; i < m_numCTUsInPic; i++)
  {
    if (ctbEnableFlags[i])
    {
      for (int j = 0; j < numClasses; j++)
      {
        frameCov[j] += ctbCov[i][j];
      }
    }
  }
}

void EncAdaptiveLoopFilter::deriveStatsForFiltering(YUV * orgYuv, YUV * recYuv)
{
  int ctuRsAddr = 0;
  const int numberOfComponents = getNumberValidComponents(m_chromaFormat);
  // init CTU stats buffers
  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID compID = ComponentID(compIdx);

    const int numClasses = compID == COMPONENT_Y ? MAX_NUM_ALF_CLASSES : 1;

    const ChannelType channel = toChannelType(compID);
    const int size = channel == CHANNEL_TYPE_LUMA ? 2 : 1;

    for (int shape = 0; shape != size; shape++)
    {
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
        {
            m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset();
        }
      }
    }
  }

  // init Frame stats buffers
  const int numberOfChannels = getNumberValidChannels(m_chromaFormat);
  for (int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++)
  {
    const ChannelType channelID = ChannelType(channelIdx);

    const int numClasses = isLuma(channelID) ? MAX_NUM_ALF_CLASSES : 1;
    const int size = isLuma(channelID) ? 2 : 1;

    for (int shape = 0; shape != size; shape++)
    {
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset();
      }
    }
  }

  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      const int width = (xPos + m_maxCUWidth > m_picWidth) ? (m_picWidth - xPos) : m_maxCUWidth;
      const int height = (yPos + m_maxCUHeight > m_picHeight) ? (m_picHeight - yPos) : m_maxCUHeight;

      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        const ComponentID compID = ComponentID(compIdx);

        //for 4:2:0 only
        int width2 = 0, height2 = 0, xPos2 = 0, yPos2 = 0;
        if( compIdx > 0 ) {
            width2 = width >> 1;
            height2 = height >> 1;
            xPos2 = xPos >> 1;
            yPos2 = yPos >> 1;
        }
        else
        {
            width2 = width;
            height2 = height;            
            xPos2 = xPos;
            yPos2 = yPos;
        }

        int  recStride = recYuv->s[compID];
        pel* rec = recYuv->yuv[compID];

        int  orgStride = orgYuv->s[compID];
        pel* org = orgYuv->yuv[compID];

        ChannelType chType = toChannelType(compID);
        const int size = isLuma(chType) ? 2 : 1;

        for (int shape = 0; shape != size; shape++)
        {
          getBlkStats((int)chType, m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, xPos2, yPos2, width2, height2);

          const int numClasses = compID == COMPONENT_Y ? MAX_NUM_ALF_CLASSES : 1;

          for (int classIdx = 0; classIdx < numClasses; classIdx++)
          {
            m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
          }
        }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStats(int ch, AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org0, const int orgStride, Pel* rec0, const int recStride,
    int x, int y, int width, int height)
{
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF];

  int transposeIdx = 0;
  int classIdx = 0;

  Pel* rec = rec0 + y*recStride + x;
  Pel* org = org0 + y*orgStride + x;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      memset(ELocal, 0, shape.numCoeff * sizeof(int));
      if( classifier /* && ( (i & 3) == 0) && ((j & 3) == 0) */ ) //todo: here addressing is for full frame classifier, to be changed to x16 times smaller
      {
        int x2 = ch ? (x << 1) : x;
        int y2 = ch ? (y << 1) : y;
        AlfClassifier cl = classifier[y2 + i][x2 + j];
        transposeIdx = cl & 0x03;
        classIdx = (cl >> 2) & 0x1F;
      }

      int yLocal = org[j] - rec[j];
      calcCovariance(ELocal, rec + j, recStride, shape.pattern, shape.filterLength >> 1, transposeIdx);
      for (int k = 0; k < shape.numCoeff; k++)
      {
        for (int l = k; l < shape.numCoeff; l++)
        {
          alfCovariace[classIdx].E[k][l] += ELocal[k] * ELocal[l];
        }
        alfCovariace[classIdx].y[k] += ELocal[k] * yLocal;
      }
      alfCovariace[classIdx].pixAcc += yLocal * yLocal;
    }
    org += orgStride;
    rec += recStride;
  }

  int numClasses = classifier ? MAX_NUM_ALF_CLASSES : 1;
  for (classIdx = 0; classIdx < numClasses; classIdx++)
  {
    for (int k = 1; k < shape.numCoeff; k++)
    {
      for (int l = 0; l < k; l++)
      {
        alfCovariace[classIdx].E[k][l] = alfCovariace[classIdx].E[l][k];
      }
    }
  }
}

void EncAdaptiveLoopFilter::calcCovariance(int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx)
{
  int k = 0;

  if (transposeIdx == 0)
  {
    for (int i = -halfFilterLength; i < 0; i++)
    {
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;

      for (int j = -halfFilterLength - i; j <= halfFilterLength + i; j++)
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
    }
    for (int j = -halfFilterLength; j < 0; j++)
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
  }
  else if (transposeIdx == 1)
  {
    for (int j = -halfFilterLength; j < 0; j++)
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++)
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
    }
    for (int i = -halfFilterLength; i < 0; i++)
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
  }
  else if (transposeIdx == 2)
  {
    for (int i = -halfFilterLength; i < 0; i++)
    {
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;

      for (int j = halfFilterLength + i; j >= -halfFilterLength - i; j--)
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
    }
    for (int j = -halfFilterLength; j < 0; j++)
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
  }
  else
  {
    for (int j = -halfFilterLength; j < 0; j++)
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--)
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
    }
    for (int i = -halfFilterLength; i < 0; i++)
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
  }
  ELocal[filterPattern[k++]] += rec[0];
}


double EncAdaptiveLoopFilter::calculateError(AlfCovariance& cov)
{
  static double c[MAX_NUM_ALF_COEFF];

  gnsSolveByChol(cov.E, cov.y, c, cov.numCoeff);

  double sum = 0;
  for (int i = 0; i < cov.numCoeff; i++)
  {
    sum += c[i] * cov.y[i];
  }

  return cov.pixAcc - sum;
}

//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
int EncAdaptiveLoopFilter::gnsCholeskyDec(double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq)
{
  static double invDiag[MAX_NUM_ALF_COEFF];  /* Vector of the inverse of diagonal entries of outMatr */

  for (int i = 0; i < numEq; i++)
  {
    for (int j = i; j < numEq; j++)
    {
      /* Compute the scaling factor */
      double scale = inpMatr[i][j];
      if (i > 0)
      {
        for (int k = i - 1; k >= 0; k--)
        {
          scale -= outMatr[k][j] * outMatr[k][i];
        }
      }

      /* Compute i'th row of outMatr */
      if (i == j)
      {
        if (scale <= REG_SQR) // if(scale <= 0 )  /* If inpMatr is singular */
        {
          return 0;
        }
        else              /* Normal operation */
          invDiag[i] = 1.0 / (outMatr[i][i] = sqrt(scale));
      }
      else
      {
        outMatr[i][j] = scale * invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }
    }
  }
  return 1; /* Signal that Cholesky factorization is successfully performed */
}

void EncAdaptiveLoopFilter::gnsTransposeBacksubstitution(double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order)
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
  for (int i = 1; i < order; i++)
  {         /* For the rows 1..order-1           */

    double sum = 0; //Holds backsubstitution from already handled rows

    for (int j = 0; j < i; j++) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = (rhs[i] - sum) / U[i][i];       /* i'th component of solution vect.  */
  }
}

void EncAdaptiveLoopFilter::gnsBacksubstitution(double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A)
{
  size--;
  A[size] = z[size] / R[size][size];

  for (int i = size - 1; i >= 0; i--)
  {
    double sum = 0;

    for (int j = i + 1; j <= size; j++)
    {
      sum += R[i][j] * A[j];
    }

    A[i] = (z[i] - sum) / R[i][i];
  }
}

int EncAdaptiveLoopFilter::gnsSolveByChol(double **LHS, double *rhs, double *x, int numEq)
{
  static double aux[MAX_NUM_ALF_COEFF];     /* Auxiliary vector */
  static double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF];    /* Upper triangular Cholesky factor of LHS */
  int res = 1;  // Signal that Cholesky factorization is successfully performed

                /* The equation to be solved is LHSx = rhs */

                /* Compute upper triangular U such that U'*U = LHS */
  if (gnsCholeskyDec(LHS, U, numEq)) /* If Cholesky decomposition has been successful */
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
    * Solve U'*aux = rhs for aux
    */
    gnsTransposeBacksubstitution(U, rhs, aux, numEq);

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution(U, aux, numEq, x);

  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for (int i = 0; i < numEq; i++)
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
    res = gnsCholeskyDec(LHS, U, numEq);

    if (!res)
    {
      memset(x, 0, sizeof(double)*numEq);
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution(U, rhs, aux, numEq);

    /* Solve U*x = aux for x */
    gnsBacksubstitution(U, aux, numEq, x);
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////
void EncAdaptiveLoopFilter::setEnableFlag(AlfTileGroupParam* alfTileGroupPara, ChannelType channel, bool val)
{
  if (channel == CHANNEL_TYPE_LUMA)
  {
    alfTileGroupPara->enabledFlag[COMPONENT_Y] = val;
  }
  else
  {
    alfTileGroupPara->enabledFlag[COMPONENT_Cb] = alfTileGroupPara->enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag(AlfTileGroupParam* alfTileGroupPara, ChannelType channel, uint8_t** ctuFlags)
{
  const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;
  for (int compId = compIDFirst; compId <= compIDLast; compId++)
  {
    alfTileGroupPara->enabledFlag[compId] = false;
    for (int i = 0; i < m_numCTUsInPic; i++)
    {
      if (ctuFlags[compId][i])
      {
        alfTileGroupPara->enabledFlag[compId] = true;
        break;
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel)
{
  if (isLuma(channel))
  {
    memcpy(ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof(uint8_t) * m_numCTUsInPic);
  }
  else
  {
    memcpy(ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof(uint8_t) * m_numCTUsInPic);
    memcpy(ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof(uint8_t) * m_numCTUsInPic);
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag(uint8_t** ctuFlags, ChannelType channel, uint8_t val)
{
  if (isLuma(channel))
  {
    memset(ctuFlags[COMPONENT_Y], val, sizeof(uint8_t) * m_numCTUsInPic);
  }
  else
  {
    memset(ctuFlags[COMPONENT_Cb], val, sizeof(uint8_t) * m_numCTUsInPic);
    memset(ctuFlags[COMPONENT_Cr], val, sizeof(uint8_t) * m_numCTUsInPic);
  }
}
#endif
