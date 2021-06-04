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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"

#pragma warning(disable : 4996) //@Chernyak: why?

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

void call_create_enc_ALF(EncAdaptiveLoopFilter* p, const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth
#if BD_CF_EXT
                         , int idc
#endif
)
{
    p->create(picWidth, picHeight, maxCUWidth, maxCUHeight, maxCUDepth
#if BD_CF_EXT
              , idc
#endif
    );
}

void call_destroy_enc_ALF(EncAdaptiveLoopFilter* p)
{
    p->destroy();
}

void alf_aps_enc_opt_process(EncAdaptiveLoopFilter* p, const double* lambdas, EVCE_CTX * ctx, EVC_PIC * pic, evc_AlfSliceParam* iAlfSliceParam)
{
    CodingStructure cs;
    cs.pCtx = (void*)ctx;
    cs.pPic = pic;
#if BD_CF_EXT
    cs.idc = ctx->sps.chroma_format_idc;
#endif

    if (m_resetALFBufferFlag)
    {
        iAlfSliceParam->resetALFBufferFlag = true;
    }
    // Initialize ALF module for current POC
    m_currentPoc = ctx->poc.poc_val;
    m_currentTempLayer = ctx->nalu.nuh_temporal_id;
    if (m_resetALFBufferFlag)
    {
        // initialize firstIdrPoc
        if (m_lastIdrPoc != INT_MAX)  // LastIdr value was initialized
        {
            m_firstIdrPoc = m_lastIdrPoc;
        }
        else {
            m_firstIdrPoc = ctx->poc.poc_val;
        }
        m_lastIdrPoc = ctx->poc.poc_val;  // store current pointer of the reset poc
        m_i_period = ctx->param.i_period; // store i-period for current pic.
    }

    m_pendingRasInit = FALSE;
    if (ctx->poc.poc_val > m_lastRasPoc)
    {
        m_lastRasPoc = INT_MAX;
        m_pendingRasInit = TRUE;
    }
    if (ctx->sh->slice_type == SLICE_I)
    {
        m_lastRasPoc = ctx->poc.poc_val;
    }

    if (m_pendingRasInit)
    {
        resetIdrIndexListBufferAPS();
    }

    AlfSliceParam alfSliceParam;
    alfSliceParam.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
    iAlfSliceParam->alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
    memset(alfSliceParam.alfCtuEnableFlag, 0, N_C * ctx->f_lcu * sizeof(u8));
    memset(iAlfSliceParam->alfCtuEnableFlag, 0, N_C * ctx->f_lcu * sizeof(u8));
    p->Enc_ALFProcess(cs, lambdas, &alfSliceParam);

    if (alfSliceParam.enabledFlag[0] && m_store2ALFBufferFlag)
    {
        const unsigned tidxMAX = MAX_NUM_TLAYER - 1u;
        const unsigned tidx = ctx->nalu.nuh_temporal_id;
        assert(tidx <= tidxMAX);
        storeEncALFParamLineAPS(&alfSliceParam, tidx);
        alfSliceParam.store2ALFBufferFlag = m_store2ALFBufferFlag;
    }
    if (ctx->sh->slice_type == SLICE_I)
    {
        if (alfSliceParam.enabledFlag[0] && m_store2ALFBufferFlag)
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

    iAlfSliceParam->isCtbAlfOn = BOOL(alfSliceParam.isCtbAlfOn ? 1 : 0);
    memcpy(iAlfSliceParam->alfCtuEnableFlag, alfSliceParam.alfCtuEnableFlag, N_C * ctx->f_lcu * sizeof(u8));
    iAlfSliceParam->enabledFlag[0] = BOOL(alfSliceParam.enabledFlag[COMPONENT_Y]);
    iAlfSliceParam->enabledFlag[1] = BOOL(alfSliceParam.enabledFlag[COMPONENT_Cb]);
    iAlfSliceParam->enabledFlag[2] = BOOL(alfSliceParam.enabledFlag[COMPONENT_Cr]);

    iAlfSliceParam->numLumaFilters = alfSliceParam.numLumaFilters;
    iAlfSliceParam->lumaFilterType = int(alfSliceParam.lumaFilterType);

    memcpy(iAlfSliceParam->filterCoeffDeltaIdx, alfSliceParam.filterCoeffDeltaIdx, MAX_NUM_ALF_CLASSES * sizeof(short));
    memcpy(iAlfSliceParam->lumaCoeff, alfSliceParam.lumaCoeff, sizeof(short)*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_LUMA_COEFF);
    memcpy(iAlfSliceParam->chromaCoeff, alfSliceParam.chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
    memcpy(iAlfSliceParam->fixedFilterIdx, alfSliceParam.fixedFilterIdx, MAX_NUM_ALF_CLASSES * sizeof(int));
#if M53608_ALF_7
    memcpy(iAlfSliceParam->fixedFilterUsageFlag, alfSliceParam.fixedFilterUsageFlag, MAX_NUM_ALF_CLASSES * sizeof(u8));
#endif
    iAlfSliceParam->fixedFilterPattern = alfSliceParam.fixedFilterPattern;

    iAlfSliceParam->coeffDeltaFlag = BOOL(alfSliceParam.coeffDeltaFlag);
    iAlfSliceParam->coeffDeltaPredModeFlag = BOOL(alfSliceParam.coeffDeltaPredModeFlag);

    //bool is not a BOOL
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
        iAlfSliceParam->filterCoeffFlag[i] = BOOL(alfSliceParam.filterCoeffFlag[i]);
    }

    iAlfSliceParam->prevIdx = alfSliceParam.prevIdx;
    iAlfSliceParam->prevIdxComp[0] = alfSliceParam.prevIdxComp[0];
    iAlfSliceParam->prevIdxComp[1] = alfSliceParam.prevIdxComp[1];
    iAlfSliceParam->tLayer = alfSliceParam.tLayer;
    iAlfSliceParam->temporalAlfFlag = BOOL(alfSliceParam.temporalAlfFlag);
    iAlfSliceParam->resetALFBufferFlag = BOOL(alfSliceParam.resetALFBufferFlag);
    iAlfSliceParam->store2ALFBufferFlag = BOOL(alfSliceParam.store2ALFBufferFlag);
}

u8 alf_aps_get_current_alf_idx()
{
    return (m_nextFreeAlfIdxInBuffer - 1) < 0 ? APS_MAX_NUM-1 : (m_nextFreeAlfIdxInBuffer - 1);
}

} //<-- extern "C"

#define AlfCtx(c) SubCtx( Ctx::ctbAlfFlag, c )

void AlfSliceParam_reset(AlfSliceParam* p)
{
  p->isCtbAlfOn = false;
  memset(p->alfCtuEnableFlag, 1, (size_t) m_numCTUsInPic * sizeof(u8));
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
#if M53608_ALF_7
  memset(p->fixedFilterUsageFlag, 0, sizeof(p->fixedFilterUsageFlag));
#endif
  p->temporalAlfFlag = false;
  p->prevIdx = 0;
  p->prevIdxComp[0] = 0;
  p->prevIdxComp[1] = 0;
  p->tLayer = 0;
  p->resetALFBufferFlag = false;
  p->store2ALFBufferFlag = false;
  p->m_filterPoc = INT_MAX;  // store POC value for which filter was produced
  p->m_minIdrPoc = INT_MAX;  // Minimal of 2 IDR POC available for current coded nalu  (to identify availability of this filter for temp prediction)
  p->m_maxIdrPoc = INT_MAX;  // Max of 2 IDR POC available for current coded nalu  (to identify availability of this filter for temp prediction)
}

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
{

  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    m_alfCovariance[i] = nullptr;
  }
#if M53608_ALF_9
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
      m_alfCovarianceFrame[i] = nullptr;
  }
#else
  for (int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
      m_alfCovarianceFrame[i] = nullptr;
  }
#endif

  m_filterCoeffQuant = nullptr;
  m_filterCoeffSet = nullptr;
  m_diffFilterCoeff = nullptr;
}

void EncAdaptiveLoopFilter::create(const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth
#if BD_CF_EXT
                                   , int idc
#endif
)
{
#if BD_CF_EXT
   const ChromaFormat chromaFormatIDC = ChromaFormat(idc);
#else
  const ChromaFormat chromaFormatIDC = CHROMA_420;
#endif

  AdaptiveLoopFilter_create( picWidth, picHeight, maxCUWidth, maxCUHeight, maxCUDepth 
#if BD_CF_EXT
                            , idc
#endif
  );
#if M53608_ALF_9
  for (int channelIdx = 0; channelIdx < MAX_NUM_COMPONENT; channelIdx++)
#else
  for (int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++)
#endif
  {
#if M53608_ALF_9
      ChannelType chType = toChannelType(ComponentID(channelIdx));
#else
      ChannelType chType = (ChannelType)channelIdx;
#endif
    const int size = channelIdx ? 1 : 2;
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
#if M53608_ALF_9
    m_alfCovarianceFrame[channelIdx] = new AlfCovariance*[size];
#else
    m_alfCovarianceFrame[chType] = new AlfCovariance*[size];
#endif
    for (int i = 0; i != size; i++)
    {
#if M53608_ALF_9
        m_alfCovarianceFrame[channelIdx][i] = new AlfCovariance[numClasses];
#else
        m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];
#endif
      for (int k = 0; k < numClasses; k++)
      {
#if M53608_ALF_9
          m_alfCovarianceFrame[channelIdx][i][k].create(m_filterShapes[chType][i].numCoeff);
#else
          m_alfCovarianceFrame[chType][i][k].create(m_filterShapes[chType][i].numCoeff);
#endif
      }
    }
  }
#if M53608_ALF_9
  m_alfCovarianceFrame[MAX_NUM_COMPONENT] = new AlfCovariance*[1];
  m_alfCovarianceFrame[MAX_NUM_COMPONENT][0] = new AlfCovariance[1];
  for (int k = 0; k < 1; k++)
  {
      m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][k].create(m_filterShapes[1][0].numCoeff);
  }
#endif
  m_alfSliceParamTemp.alfCtuEnableFlag = (u8 *)malloc(N_C * m_numCTUsInPic * sizeof(u8));
  memset(m_alfSliceParamTemp.alfCtuEnableFlag, 0, N_C * m_numCTUsInPic * sizeof(u8));

  m_ctuEnableFlagTmpLuma = (u8 *)malloc(N_C * m_numCTUsInPic * sizeof(u8));
  memset(m_ctuEnableFlagTmpLuma, 0, N_C * m_numCTUsInPic * sizeof(u8));

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
#if M53608_ALF_9
    for (int channelIdx = 0; channelIdx < MAX_NUM_COMPONENT; channelIdx++)
#else
    for (int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++)
#endif
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
#if M53608_ALF_9
    m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][0].destroy();
    delete[] m_alfCovarianceFrame[MAX_NUM_COMPONENT][0];
    m_alfCovarianceFrame[MAX_NUM_COMPONENT][0] = nullptr;
    delete[] m_alfCovarianceFrame[MAX_NUM_COMPONENT];
    m_alfCovarianceFrame[MAX_NUM_COMPONENT] = nullptr;
#endif
  free(m_alfSliceParamTemp.alfCtuEnableFlag);
  free(m_ctuEnableFlagTmpLuma);

  m_ctuEnableFlagTmpLuma = nullptr;

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

void EncAdaptiveLoopFilter::Enc_ALFProcess(CodingStructure& cs, const double *lambdas, AlfSliceParam* alfSliceParam)
{
  EVCE_CTX* ctx = (EVCE_CTX*)(cs.pCtx);
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_ctuEnableFlag[compIdx] = alfSliceParam->alfCtuEnableFlag + ctx->f_lcu * compIdx;
  }

  // reset ALF parameters
  AlfSliceParam_reset(alfSliceParam);

  // set available filter shapes
  alfSliceParam->filterShapes = m_filterShapes;

  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA] - 8);
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA] - 8);
  m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double(1 << shiftLuma);
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double(1 << shiftChroma);
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double(1 << shiftChroma);

  const int h = cs.pPic->h_l;
  const int w = cs.pPic->w_l;
  const int m = MAX_ALF_FILTER_LENGTH >> 1;
  const int s = w + m + m;

  EVC_PIC* picOrig = PIC_ORIG(ctx);
  EVC_PIC* picReco = PIC_MODE(ctx);

  pel * orgYuv0 = picOrig->y;
  pel * recoYuv0 = picReco->y;

  int oStride = picOrig->s_l;
  int rStride = picReco->s_l;
  pel * recLuma0 = m_tempBuf + s*m + m;

#if BD_CF_EXT
  const int s1 = (w >> (GET_CHROMA_W_SHIFT(cs.idc))) + m + m;
#else
  //chroma (for 4:2:0 only)
  const int s1 = (w >> 1) + m + m;
#endif
  pel * recLuma1 = m_tempBuf1 + s1*m + m;
  pel * recLuma2 = m_tempBuf2 + s1*m + m;
  pel * recoYuv1 = picReco->u;
  pel * recoYuv2 = picReco->v;
  const int rStride1 = picReco->s_c;
  pel * orgYuv1 = picOrig->u;
  pel * orgYuv2 = picOrig->v;
  const int oStride1 = picOrig->s_c;

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

  int  x_l, x_r, y_l, y_r, w_tile, h_tile;
  int  col_bd = 0;
  int  total_tiles_in_slice = ctx->sh->num_tiles_in_slice;
  u8 * tiles_in_slice;
  u32 k = 0;
  int ii=0;

  for (int slice_num = 0; slice_num < ctx->param.num_slice_in_pic; slice_num++)
  {
      ctx->sh = &ctx->sh_array[slice_num];
      tiles_in_slice = ctx->sh->tile_order;
      total_tiles_in_slice = ctx->sh->num_tiles_in_slice;
      k = 0;
      ii = 0;
      while (total_tiles_in_slice)
      {
          ii = tiles_in_slice[k++];
          int x_loc = ((ctx->tile[ii].ctba_rs_first) % ctx->w_lcu);
          int y_loc = ((ctx->tile[ii].ctba_rs_first) / ctx->w_lcu);
          x_l = x_loc << ctx->log2_max_cuwh; //entry point CTB's x location
          y_l = y_loc << ctx->log2_max_cuwh; //entry point CTB's y location
          x_r = x_l + ((int)(ctx->tile[ii].w_ctb) << ctx->log2_max_cuwh);
          y_r = y_l + ((int)(ctx->tile[ii].h_ctb) << ctx->log2_max_cuwh);
          w_tile = x_r > ((int)ctx->w_scu << MIN_CU_LOG2) ? ((int)ctx->w_scu << MIN_CU_LOG2) - x_l : x_r - x_l;
          h_tile = y_r > ((int)ctx->h_scu << MIN_CU_LOG2) ? ((int)ctx->h_scu << MIN_CU_LOG2) - y_l : y_r - y_l;
          Pel * recLuma0_tile = recLuma0 + x_l + y_l * s;
          Pel * recoYuv0_tile = recoYuv0 + x_l + y_l * rStride;
          copy_and_extend_tile(recLuma0_tile, s, recoYuv0_tile, rStride, w_tile, h_tile, m);
          Area blk = { x_l, y_l, w_tile, h_tile };
          deriveClassification(m_classifier, recLuma0, s, &blk);
          total_tiles_in_slice--;
      }
  }
  copy_and_extend(recLuma0, s, recoYuv0, rStride, w, h, m);
#if BD_CF_EXT
  if(cs.idc)
  {
      copy_and_extend(recLuma1, s1, recoYuv1, picReco->s_c, (w >> (GET_CHROMA_W_SHIFT(cs.idc))), (h >> (GET_CHROMA_H_SHIFT(cs.idc))), m);
      copy_and_extend(recLuma2, s1, recoYuv2, picReco->s_c, (w >> (GET_CHROMA_W_SHIFT(cs.idc))), (h >> (GET_CHROMA_H_SHIFT(cs.idc))), m);
  }
#else
  copy_and_extend(recLuma1, s1, recoYuv1, picReco->s_c, (w >> 1), (h >> 1), m);
  copy_and_extend(recLuma2, s1, recoYuv2, picReco->s_c, (w >> 1), (h >> 1), m);
#endif

  // get CTB stats for filtering
  deriveStatsForFiltering(&orgYuv, &recLuma
#if BD_CF_EXT
                          , cs.idc
#endif
  );

  // derive filter (luma)
  alfEncoder(cs, alfSliceParam, CHANNEL_TYPE_LUMA);
  // derive filter (chroma)

  if ( alfSliceParam->enabledFlag[COMPONENT_Y] 
#if BD_CF_EXT
      && cs.idc
#endif
      )
  {
    alfEncoder(cs, alfSliceParam, CHANNEL_TYPE_CHROMA);
  }

  // temporal prediction

  if (ctx->slice_type != SLICE_I)
  {
    deriveStatsForFiltering(&orgYuv, &recLuma
#if BD_CF_EXT
                            , cs.idc
#endif
    );
    alfTemporalEncoderAPSComponent(cs, alfSliceParam);

    m_resetALFBufferFlag = false; 
    alfSliceParam->resetALFBufferFlag = false;
    if( alfSliceParam->temporalAlfFlag ) {
        m_store2ALFBufferFlag = false;
        alfSliceParam->store2ALFBufferFlag = false;
    }
    else 
    {
        m_store2ALFBufferFlag = true;
        alfSliceParam->store2ALFBufferFlag = true;
    }
  }
  else {
      alfSliceParam->store2ALFBufferFlag = true;
      m_store2ALFBufferFlag = true;
      alfSliceParam->resetALFBufferFlag = true;
      m_resetALFBufferFlag = true;

  }

  for (int slice_num = 0; slice_num < ctx->param.num_slice_in_pic; slice_num++)
  {
      ctx->sh = &ctx->sh_array[slice_num];
      tiles_in_slice = ctx->sh->tile_order;
      total_tiles_in_slice = ctx->sh->num_tiles_in_slice;
      k = 0;
      ii = 0;
      while (total_tiles_in_slice)
      {
          int ii = tiles_in_slice[k++];
          int x_loc = ((ctx->tile[ii].ctba_rs_first) % ctx->w_lcu);
          int y_loc = ((ctx->tile[ii].ctba_rs_first) / ctx->w_lcu);

          col_bd = 0;
          if (ii% ctx->param.tile_columns)
          {
              int temp = ii - 1;
              while (temp >= 0)
              {
                  col_bd += ctx->tile[temp].w_ctb;
                  if (!(temp%ctx->param.tile_columns)) break;
                  temp--;
              }
          }
          else
          {
              col_bd = 0;
          }

          x_l = x_loc << ctx->log2_max_cuwh; //entry point CTB's x location
          y_l = y_loc << ctx->log2_max_cuwh; //entry point CTB's y location
          x_r = x_l + ((int)(ctx->tile[ii].w_ctb) << ctx->log2_max_cuwh);
          y_r = y_l + ((int)(ctx->tile[ii].h_ctb) << ctx->log2_max_cuwh);
          w_tile = x_r > ((int)ctx->w_scu << MIN_CU_LOG2) ? ((int)ctx->w_scu << MIN_CU_LOG2) - x_l : x_r - x_l;
          h_tile = y_r > ((int)ctx->h_scu << MIN_CU_LOG2) ? ((int)ctx->h_scu << MIN_CU_LOG2) - y_l : y_r - y_l;
          //This is for YUV420 only 
          Pel * recLuma0_tile = recLuma0 + x_l + y_l * s;
          Pel * recoYuv0_tile = recoYuv0 + x_l + y_l * rStride;
          copy_and_extend_tile(recLuma0_tile, s, recoYuv0_tile, rStride, w_tile, h_tile, m);
#if BD_CF_EXT
          Pel * recLuma1_tile = recLuma1 + (x_l >> (GET_CHROMA_W_SHIFT(cs.idc))) + (y_l >> (GET_CHROMA_H_SHIFT(cs.idc))) * (s1);
          Pel * recLuma2_tile = recLuma2 + (x_l >> (GET_CHROMA_W_SHIFT(cs.idc))) + (y_l >> (GET_CHROMA_H_SHIFT(cs.idc))) * (s1);
          Pel * recoYuv2_tile = recoYuv2 + (x_l >> (GET_CHROMA_W_SHIFT(cs.idc))) + (y_l >> (GET_CHROMA_H_SHIFT(cs.idc))) * picReco->s_c;
          Pel * recoYuv1_tile = recoYuv1 + (x_l >> (GET_CHROMA_W_SHIFT(cs.idc))) + (y_l >> (GET_CHROMA_H_SHIFT(cs.idc))) * picReco->s_c;
          if (cs.idc)
          {
              copy_and_extend_tile(recLuma1_tile, s1, recoYuv1_tile, picReco->s_c, (w_tile >> (GET_CHROMA_W_SHIFT(cs.idc))), (h_tile >> (GET_CHROMA_H_SHIFT(cs.idc))), m);
              copy_and_extend_tile(recLuma2_tile, s1, recoYuv2_tile, picReco->s_c, (w_tile >> (GET_CHROMA_W_SHIFT(cs.idc))), (h_tile >> (GET_CHROMA_H_SHIFT(cs.idc))), m);
          }
#else
          Pel * recLuma1_tile = recLuma1 + (x_l >> 1) + (y_l >> 1) * (s1);
          Pel * recLuma2_tile = recLuma2 + (x_l >> 1) + (y_l >> 1) * (s1);
          Pel * recoYuv1_tile = recoYuv1 + (x_l >> 1) + (y_l >> 1) * picReco->s_c;
          Pel * recoYuv2_tile = recoYuv2 + (x_l >> 1) + (y_l >> 1) * picReco->s_c;

          copy_and_extend_tile(recLuma1_tile, s1, recoYuv1_tile, picReco->s_c, (w_tile >> 1), (h_tile >> 1), m);
          copy_and_extend_tile(recLuma2_tile, s1, recoYuv2_tile, picReco->s_c, (w_tile >> 1), (h_tile >> 1), m);
#endif

          // reconstruct 
#if ALF_CONFORMANCE_CHECK
          if (alfSliceParam->enabledFlag[COMPONENT_Y])
          {
              alfReconstructor(cs, alfSliceParam, orgYuv.yuv[0], orgYuv.s[0], recLuma.yuv[0], recLuma.s[0], COMPONENT_Y, ii, col_bd);
          }
          if (alfSliceParam->enabledFlag[COMPONENT_Cb])
          {
              alfReconstructor(cs, alfSliceParam, orgYuv.yuv[1], orgYuv.s[1], recLuma.yuv[1], recLuma.s[1], COMPONENT_Cb, ii, col_bd);
          }
          if (alfSliceParam->enabledFlag[COMPONENT_Cr])
          {
              alfReconstructor(cs, alfSliceParam, orgYuv.yuv[2], orgYuv.s[2], recLuma.yuv[2], recLuma.s[2], COMPONENT_Cr, ii, col_bd);
          }
#else
          alfReconstructor(cs, alfSliceParam, orgYuv.yuv[0], orgYuv.s[0], recLuma.yuv[0], recLuma.s[0], COMPONENT_Y, ii, col_bd);
          alfReconstructor(cs, alfSliceParam, orgYuv.yuv[1], orgYuv.s[1], recLuma.yuv[1], recLuma.s[1], COMPONENT_Cb, ii, col_bd);
          alfReconstructor(cs, alfSliceParam, orgYuv.yuv[2], orgYuv.s[2], recLuma.yuv[2], recLuma.s[2], COMPONENT_Cr, ii, col_bd);
#endif
          total_tiles_in_slice--;
      }
  }
  for( int i = 0; i < ctx->f_lcu; i++ )
  {
      if (*(alfSliceParam->alfCtuEnableFlag + i) == 0)
     {
          alfSliceParam->isCtbAlfOn = true;
          break;
      } else
          alfSliceParam->isCtbAlfOn = false;
  }
}
#if M53608_ALF_9
double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags(CodingStructure& cs, const int iShapeIdx, ComponentID compID, const int numClasses, const int numCoeff, double& distUnfilter
    , bool recCoeff
)
#else
double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags(CodingStructure& cs, const int iShapeIdx, ChannelType channel, const int numClasses, const int numCoeff, double& distUnfilter
    , bool recCoeff
)
#endif
{
#if M53608_ALF_9
    ChannelType channel = toChannelType(compID);
#endif
#if !M53608_ALF_9
    const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
    const ComponentID compIDLast = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;
#endif
#if !M53608_ALF_8
    if (isLuma(channel))
        CHECK(iShapeIdx != ALF_FILTER_7, "wrong ishapeIdx for luma");
#endif
  double cost = 0;
  distUnfilter = 0;
#if M53608_ALF_9
  setEnableFlag(&m_alfSliceParamTemp, compID, true);
#else
  setEnableFlag(&m_alfSliceParamTemp, channel, true);
#endif
  if (isChroma(channel))
  {
    m_alfSliceParamTemp.chromaCtbPresentFlag = false;
  }
  if (recCoeff)
  {
    reconstructCoeff(&m_alfSliceParamTemp, channel, true, isLuma(channel));
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
      {
        m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfSliceParamTemp.chromaCoeff[i];
      }
    }
  }

  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
#if !M53608_ALF_9
    for (int compID = compIDFirst; compID <= compIDLast; compID++)
#endif
    {
      double distUnfilterCtu = getUnfilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses);

      double costOn = 0;
      costOn = distUnfilterCtu + getFilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfSliceParamTemp.numLumaFilters - 1, numCoeff);
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
#if M53608_ALF_9
      setEnableFlag(&m_alfSliceParamTemp, compID, m_ctuEnableFlag);
#else
      setEnableFlag(&m_alfSliceParamTemp, channel, m_ctuEnableFlag);
#endif
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
#if M53608_ALF_9
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[compID];
#else
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[channel];
#endif
  }


  return cost;
}


void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfSliceParam* alfSliceParam, const ChannelType channel )
{
#if ALF_CONFORMANCE_CHECK
  u8 filter_conformance_flag = 0;
#endif
  double costMin = DBL_MAX;
#if M53608_ALF_9
  double costMin_cb = DBL_MAX;
  double costMin_cr = DBL_MAX;
#endif
  EVCE_CTX* ctx = (EVCE_CTX*)cs.pCtx;

  AlfFilterShape* alfFilterShape = alfSliceParam->filterShapes[channel];

  const int numClasses = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  const int size = channel == CHANNEL_TYPE_LUMA ? 2 : 1;

  int covLrgIdx = size - 1;
  for (int iShapeIdx = 0; iShapeIdx < size; iShapeIdx++)
  {
    copyAlfParam( &m_alfSliceParamTemp, alfSliceParam );
    if (isLuma(channel))
    {
      m_alfSliceParamTemp.lumaFilterType = (AlfFilterType)(alfFilterShape[iShapeIdx].filterType);
    }
    double cost = costMin;
#if M53608_ALF_9
    double cost_cb = costMin_cb;
    double cost_cr = costMin_cr;
#endif
    //1. get unfiltered distortion
    if (isLuma(channel))
    {
#if M53608_ALF_9
        setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Y, 1);
#if M53608_ALF_8
        getFrameStats(COMPONENT_Y, iShapeIdx);
#else
        getFrameStats(COMPONENT_Y, covLrgIdx);
#endif
#else
        setCtuEnableFlag(m_ctuEnableFlag, channel, 1);
#if M53608_ALF_8
        getFrameStats(channel, iShapeIdx);
#else
        getFrameStats(channel, covLrgIdx);
#endif
#endif
    }
#if M53608_ALF_9
    if (isChroma(channel))
    {
        cost_cb = getUnfilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cb][covLrgIdx], channel);
        cost_cb = cost_cb / 1.001;
        cost_cr = getUnfilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cr][covLrgIdx], channel);
        cost_cr = cost_cr / 1.001;
        if (cost_cb < costMin_cb)
        {
            costMin_cb = cost_cb;
            setEnableFlag(alfSliceParam, COMPONENT_Cb, false);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cb, 0);
            alfSliceParam->chromaCtbPresentFlag = false;
        }
        if (cost_cr < costMin_cr)
        {
            costMin_cr = cost_cr;
            setEnableFlag(alfSliceParam, COMPONENT_Cr, false);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cr, 0);
            alfSliceParam->chromaCtbPresentFlag = false;
        }
    }
    else
    {
#endif
#if M53608_ALF_8
        cost = getUnfilteredDistortion(m_alfCovarianceFrame[channel][iShapeIdx], channel);
#else
        cost = getUnfilteredDistortion(m_alfCovarianceFrame[channel][covLrgIdx], channel);
#endif
    cost /= 1.001; // slight preference for unfiltered choice
    if (cost < costMin)
    {
      costMin = cost;
#if M53608_ALF_9
      setEnableFlag(alfSliceParam, COMPONENT_Y, false);
      setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Y, 0);
#else
      setEnableFlag(alfSliceParam, channel, false);
      setCtuEnableFlag(m_ctuEnableFlagTmp, channel, 0);
      if (isChroma(channel))
      {
          alfSliceParam->chromaCtbPresentFlag = false;
      }
#endif
    }
#if M53608_ALF_9
    }
#endif

    //2. all CTUs are on
    if (isChroma(channel))
    {
      m_alfSliceParamTemp.chromaCtbPresentFlag = true;
    }
#if M53608_ALF_9
    if (isLuma(channel))
    {
        setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Y, true);
        setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Y, 1);
        cost = getFilterCoeffAndCost(cs, 0, COMPONENT_Y, isLuma(channel), iShapeIdx, uiCoeffBits
#if ALF_CONFORMANCE_CHECK
        , &filter_conformance_flag
#endif
        );
#if ALF_CONFORMANCE_CHECK
        if (filter_conformance_flag)
        {
           setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Y, false);
        }
#endif
        cost += m_lambda[channel];
        if (cost < costMin)
        {
            costMin = cost;
            copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Y, 1);
        }
    }
    else
    {
        setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cb, true);
        setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cb, 1);
        setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cr, true);
        setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cr, 1);
        double filter_cost[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
        getFilterCoeffAndCost_Chroma(cs, 0, COMPONENT_Cb, isLuma(channel), iShapeIdx, uiCoeffBits, filter_cost);

        filter_cost[0] += m_lambda[COMPONENT_Cb];
        filter_cost[1] += m_lambda[COMPONENT_Cb];
        filter_cost[2] += m_lambda[COMPONENT_Cb];

        if (filter_cost[2] < cost_cb + cost_cr)
        {
            costMin_cb = filter_cost[2] / 2;
            costMin_cr = filter_cost[2] / 2;
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cb, 1);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cr, 1);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cb, true);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cr, true);
            copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
        }
        else if (filter_cost[0] < cost_cb)
        {
            costMin_cb = filter_cost[0];
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cb, 1);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cr, 0);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cb, true);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cr, false);
            copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
        }
        else if (filter_cost[1] < cost_cr)
        {
            costMin_cr = filter_cost[1];
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cb, 0);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cr, 1);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cb, false);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cr, true);
            copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
        }
        else
        {
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cb, 0);
            setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cr, 0);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cb, false);
            setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Cr, false);
            copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
        }
    }
#else
    setEnableFlag(&m_alfSliceParamTemp, channel, true);
    setCtuEnableFlag(m_ctuEnableFlag, channel, 1);
    cost = getFilterCoeffAndCost(cs, 0, channel, isLuma(channel), iShapeIdx, uiCoeffBits);

    cost += m_lambda[channel];
    if (cost < costMin)
    {
      costMin = cost;
      copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
      setCtuEnableFlag(m_ctuEnableFlagTmp, channel, 1);
    }
#endif
    //3. CTU decision
#if ALF_CONFORMANCE_CHECK
    if (channel != CHANNEL_TYPE_CHROMA && !filter_conformance_flag)
#else
    if( channel != CHANNEL_TYPE_CHROMA ) 
#endif
    {

    double distUnfilter = 0;
    const int iterNum = 2 * 2 + 1;
    for (int iter = 0; iter < iterNum; iter++)
    {
      if ((iter & 0x01) == 0)
      {
        if (!filter_conformance_flag)
        {
          cost = m_lambda[channel] * uiCoeffBits;
#if M53608_ALF_9
#if M53608_ALF_8
          cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, COMPONENT_Y, numClasses, (iShapeIdx ? 13 : 7), distUnfilter, true);
#else
          cost += deriveCtbAlfEnableFlags(cs, covLrgIdx, COMPONENT_Y, numClasses, isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF, distUnfilter, true);
#endif
#else
#if M53608_ALF_8
          cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel, numClasses, (iShapeIdx ? 13 : 7), distUnfilter, true);
#else
          cost += deriveCtbAlfEnableFlags(cs, covLrgIdx, channel, numClasses, isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF, distUnfilter, true);
#endif
#endif
          cost += m_lambda[channel]*(m_numCTUsInPic); 
        
          if (cost < costMin)
          {
            costMin = cost;
#if M53608_ALF_9
            copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, COMPONENT_Y);
#else
            copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
#endif
            copyAlfSliceParam(alfSliceParam, &m_alfSliceParamTemp, channel);
            alfSliceParam->isCtbAlfOn = true;
          }
        }
      }
      else
      {
        setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Y, true);
#if M53608_ALF_9
        cost = getFilterCoeffAndCost(cs, distUnfilter, COMPONENT_Y, true, iShapeIdx, uiCoeffBits
#if ALF_CONFORMANCE_CHECK
            , &filter_conformance_flag
#endif
        );
#if ALF_CONFORMANCE_CHECK
        if (filter_conformance_flag)
        {
          setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Y, false);
        }
        else
        {
          setEnableFlag(&m_alfSliceParamTemp, COMPONENT_Y, true);
        }

#endif
#else
          cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits);
#endif
      }
    }//for iter

    }

  }//for shapeIdx
#if M53608_ALF_9
  if (isLuma(channel))
  {
      m_costAlfEncoder[channel] = costMin;
      copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, COMPONENT_Y);
  }
  else
  {
      m_costAlfEncoder[COMPONENT_Cb] = costMin_cb;
      m_costAlfEncoder[COMPONENT_Cr] = costMin_cr;
      copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, COMPONENT_Cb);
      copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, COMPONENT_Cr);
  }
#else
  m_costAlfEncoder[channel] = costMin;
  copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, channel);
#endif
}

void EncAdaptiveLoopFilter::tile_boundary_check(int* availableL, int* availableR, int* availableT, int* availableB, const int width, const int height, int xPos, int yPos, int x_l, int x_r, int y_l, int y_r)
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

void EncAdaptiveLoopFilter::alfReconstructor(CodingStructure& cs, AlfSliceParam* alfSliceParam, const pel * orgUnitBuf, const int oStride, pel * recExtBuf, int recStride, const ComponentID compID, int tile_idx, int col_bd)
{
    int x_l, x_r, y_l, y_r;
    const ChannelType channel = compID == COMPONENT_Y ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;

    reconstructCoeff(alfSliceParam, channel, false, isLuma(channel));
    EVCE_CTX* ctx = (EVCE_CTX*)(cs.pCtx);
    EVC_PIC* recPic = PIC_MODE(ctx);
    pel * recBuf = NULL;

    int x_loc = ((ctx->tile[tile_idx].ctba_rs_first) % ctx->w_lcu);
    int y_loc = ((ctx->tile[tile_idx].ctba_rs_first) / ctx->w_lcu);
    x_l = x_loc << ctx->log2_max_cuwh; //entry point lcu's x location
    y_l = y_loc << ctx->log2_max_cuwh; // entry point lcu's y location
    x_r = x_l + ((int)(ctx->tile[tile_idx].w_ctb) << ctx->log2_max_cuwh);
    y_r = y_l + ((int)(ctx->tile[tile_idx].h_ctb) << ctx->log2_max_cuwh);
    x_r = x_r > ((int)ctx->w_scu << MIN_CU_LOG2) ? ((int)ctx->w_scu << MIN_CU_LOG2) : x_r;
    y_r = y_r > ((int)ctx->h_scu << MIN_CU_LOG2) ? ((int)ctx->h_scu << MIN_CU_LOG2) : y_r;

    switch (compID)
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

    const int m = MAX_ALF_FILTER_LENGTH >> 1;
    int l_zero_offset = (MAX_CU_SIZE + m + m) * m + m;
    int l_stride = MAX_CU_SIZE + 2 * m;
    pel l_buffer[(MAX_CU_SIZE + 2 * m) * (MAX_CU_SIZE + 2 * m)];
    pel *p_buffer = l_buffer + l_zero_offset;
#if BD_CF_EXT
    int l_zero_offset_chroma = ((MAX_CU_SIZE >> (GET_CHROMA_W_SHIFT(cs.idc))) + m + m) * m + m;
    int l_stride_chroma = (MAX_CU_SIZE >> (GET_CHROMA_W_SHIFT(cs.idc))) + m + m;
    pel l_buffer_cr[((MAX_CU_SIZE) + 2 * m) *((MAX_CU_SIZE) + 2 * m)];
    pel l_buffer_cb[((MAX_CU_SIZE) + 2 * m) *((MAX_CU_SIZE) + 2 * m)];
#else
    int l_zero_offset_chroma = ((MAX_CU_SIZE >> 1) + m + m) * m + m;
    int l_stride_chroma = (MAX_CU_SIZE >> 1) + m + m;
    pel l_buffer_cb[((MAX_CU_SIZE >> 1) + 2 * m) *((MAX_CU_SIZE >> 1) + 2 * m)];
    pel l_buffer_cr[((MAX_CU_SIZE >> 1) + 2 * m) *((MAX_CU_SIZE >> 1) + 2 * m)];
#endif
    pel *p_buffer_cr = l_buffer_cr + l_zero_offset_chroma;
    pel *p_buffer_cb = l_buffer_cb + l_zero_offset_chroma;


    if (alfSliceParam->enabledFlag[compID])
    {
#if BD_CF_EXT
        const int chromaScaleX = isLuma(channel) ? 0 : (GET_CHROMA_W_SHIFT(cs.idc));
        const int chromaScaleY = isLuma(channel) ? 0 : (GET_CHROMA_H_SHIFT(cs.idc)); //getComponentScaleY(compID, recBuf.chromaFormat);
#else
        const int chromaScaleX = isLuma(channel) ? 0 : 1;
        const int chromaScaleY = isLuma(channel) ? 0 : 1; //getComponentScaleY(compID, recBuf.chromaFormat);
#endif
        int ctuIdx = (x_loc)+(y_loc)* ctx->w_lcu;

        AlfFilterType filterType = compID == COMPONENT_Y ? ALF_FILTER_7 : ALF_FILTER_5;
        short* coeff = compID == COMPONENT_Y ? m_coeffFinal : alfSliceParam->chromaCoeff;
        for (int yPos = y_l; yPos < y_r; yPos += ctx->max_cuwh)
        {
            for (int xPos = x_l; xPos < x_r; xPos += ctx->max_cuwh)
            {
                const int width = (xPos + ctx->max_cuwh > recPic->w_l) ? (recPic->w_l - xPos) : ctx->max_cuwh;
                const int height = (yPos + ctx->max_cuwh > recPic->h_l) ? (recPic->h_l - yPos) : ctx->max_cuwh;

                int availableL, availableR, availableT, availableB;
                availableL = availableR = availableT = availableB = 1;
                if (!(ctx->pps->loop_filter_across_tiles_enabled_flag))
                {
                    tile_boundary_check(&availableL, &availableR, &availableT, &availableB, width, height, xPos, yPos, x_l, x_r, y_l, y_r);
                }
                else
                {
                    tile_boundary_check(&availableL, &availableR, &availableT, &availableB, width, height, xPos, yPos,
                                        0, ctx->sps.pic_width_in_luma_samples - 1, 0, ctx->sps.pic_height_in_luma_samples - 1);
                }
                if (compID == COMPONENT_Y)
                {
                    for (int i = m; i < height + m; i++)
                    {
                        int dstPos = i * l_stride - l_zero_offset;
                        int srcPos_offset = xPos + yPos * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride : width + m + m);
                        memcpy(p_buffer + dstPos + m, recExtBuf + srcPos_offset + (i - m) * recStride, sizeof(pel) * (stride - 2 * m));
                        for (int j = 0; j < m; j++)
                        {
                            if (availableL)
                                p_buffer[dstPos + j] = recExtBuf[srcPos_offset + (i - m) * recStride - m + j];
                            else
                                p_buffer[dstPos + j] = recExtBuf[srcPos_offset + (i - m) * recStride + m - j];
                            if (availableR)
                                p_buffer[dstPos + j + width + m] = recExtBuf[srcPos_offset + (i - m) * recStride + width + j];
                            else
                                p_buffer[dstPos + j + width + m] = recExtBuf[srcPos_offset + (i - m) * recStride + width - j - 2];
                        }
                    }
                    for (int i = 0; i < m; i++)
                    {
                        int dstPos = i * l_stride - l_zero_offset;
                        int srcPos_offset = xPos + yPos * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride : width + m + m);
                        if (availableT)
                            memcpy(p_buffer + dstPos, recExtBuf + srcPos_offset - (m - i) * recStride - m, sizeof(pel) * stride);
                        else
                            memcpy(p_buffer + dstPos, p_buffer + dstPos + (2 * m - 2 * i) * l_stride, sizeof(pel) * stride);
                    }
                    for (int i = height + m; i < height + m + m; i++)
                    {
                        int dstPos = i * l_stride - l_zero_offset;
                        int srcPos_offset = xPos + yPos * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride : width + m + m);
                        if (availableB)
                            memcpy(p_buffer + dstPos, recExtBuf + srcPos_offset + (i - m) * recStride - m, sizeof(pel) * stride);
                        else
                            memcpy(p_buffer + dstPos, p_buffer + dstPos - (2 * (i - height - m) + 2) * l_stride, sizeof(pel) * stride);
                    }
                }
                else if (compID == COMPONENT_Cb)
                {
                    for (int i = m; i < ((height >> chromaScaleY) + m); i++)
                    {
                        int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                        int srcPos_offset = (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> chromaScaleX) + m + m);
                        memcpy(p_buffer_cb + dstPos + m, recExtBuf + srcPos_offset + (i - m) * recStride, sizeof(pel) * (stride - 2 * m));
                        for (int j = 0; j < m; j++)
                        {
                            if (availableL)
                                p_buffer_cb[dstPos + j] = recExtBuf[srcPos_offset + (i - m) * recStride - m + j];
                            else
                                p_buffer_cb[dstPos + j] = recExtBuf[srcPos_offset + (i - m) * recStride + m - j];
                            if (availableR)
                                p_buffer_cb[dstPos + j + (width >> chromaScaleX) + m] = recExtBuf[srcPos_offset + (i - m) * recStride + (width >> chromaScaleX) + j];
                            else
                                p_buffer_cb[dstPos + j + (width >> chromaScaleX) + m] = recExtBuf[srcPos_offset + (i - m) * recStride + (width >> chromaScaleX) - j - 2];
                        }
                    }

                    for (int i = 0; i < m; i++)
                    {
                        int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                        int srcPos_offset = (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> chromaScaleX) + m + m);
                        if (availableT)
                            memcpy(p_buffer_cb + dstPos, recExtBuf + srcPos_offset - (m - i) * recStride - m, sizeof(pel) * stride);
                        else
                            memcpy(p_buffer_cb + dstPos, p_buffer_cb + dstPos + (2 * m - 2 * i) * l_stride_chroma, sizeof(pel) * stride);
                    }

                    for (int i = ((height >> chromaScaleY) + m); i < ((height >> chromaScaleY) + m + m); i++)
                    {
                        int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                        int srcPos_offset = (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> chromaScaleX) + m + m);
                        if (availableB)
                            memcpy(p_buffer_cb + dstPos, recExtBuf + srcPos_offset + (i - m) * recStride - m, sizeof(pel) * stride);
                        else
                            memcpy(p_buffer_cb + dstPos, p_buffer_cb + dstPos - (2 * (i - (height >> chromaScaleY) - m) + 2) * l_stride_chroma, sizeof(pel) * stride);
                    }
                }
                else
                {
                    for (int i = m; i < ((height >> chromaScaleY) + m); i++)
                    {
                        int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                        int srcPos_offset = (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> chromaScaleX) + m + m);
                        memcpy(p_buffer_cr + dstPos + m, recExtBuf + srcPos_offset + (i - m) * recStride, sizeof(pel) * (stride - 2 * m));
                        for (int j = 0; j < m; j++)
                        {
                            if (availableL)
                                p_buffer_cr[dstPos + j] = recExtBuf[srcPos_offset + (i - m) * recStride - m + j];
                            else
                                p_buffer_cr[dstPos + j] = recExtBuf[srcPos_offset + (i - m) * recStride + m - j];
                            if (availableR)
                                p_buffer_cr[dstPos + j + (width >> chromaScaleX) + m] = recExtBuf[srcPos_offset + (i - m) * recStride + (width >> chromaScaleX) + j];
                            else
                                p_buffer_cr[dstPos + j + (width >> chromaScaleX) + m] = recExtBuf[srcPos_offset + (i - m) * recStride + (width >> chromaScaleX) - j - 2];
                        }
                    }

                    for (int i = 0; i < m; i++)
                    {
                        int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                        int srcPos_offset = (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> chromaScaleX) + m + m);
                        if (availableT)
                            memcpy(p_buffer_cr + dstPos, recExtBuf + srcPos_offset - (m - i) * recStride - m, sizeof(pel) * stride);
                        else
                            memcpy(p_buffer_cr + dstPos, p_buffer_cr + dstPos + (2 * m - 2 * i) * l_stride_chroma, sizeof(pel) * stride);
                    }

                    for (int i = ((height >> chromaScaleY) + m); i < ((height >> chromaScaleY) + m + m); i++)
                    {
                        int dstPos = i * l_stride_chroma - l_zero_offset_chroma;
                        int srcPos_offset = (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recStride;
                        int stride = (width == ctx->max_cuwh ? l_stride_chroma : (width >> chromaScaleX) + m + m);
                        if (availableB)
                            memcpy(p_buffer_cr + dstPos, recExtBuf + srcPos_offset + (i - m) * recStride - m, sizeof(pel) * stride);
                        else
                            memcpy(p_buffer_cr + dstPos, p_buffer_cr + dstPos - (2 * (i - (height >> chromaScaleY) - m) + 2) * l_stride_chroma, sizeof(pel) * stride);
                    }
                }
                Area blk = { 0, 0, width >> chromaScaleX, height >> chromaScaleY };

                if (m_ctuEnableFlag[compID][ctuIdx])
                {
                    {
                        int stride = isLuma(channel) ? recPic->s_l : recPic->s_c;

                        if (filterType == ALF_FILTER_5)
                        {
                            if (compID == COMPONENT_Cb)
                            {
                                m_AdaptiveLoopFilter.m_filter5x5Blk(m_classifier, recBuf + (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recPic->s_c, recPic->s_c, p_buffer_cb, l_stride_chroma, &blk, compID, coeff, &(m_clpRngs.comp[(int)compID]));
                            }
                            else
                            {
                                m_AdaptiveLoopFilter.m_filter5x5Blk(m_classifier, recBuf + (xPos >> chromaScaleX) + (yPos >> chromaScaleY) * recPic->s_c, recPic->s_c, p_buffer_cr, l_stride_chroma, &blk, compID, coeff, &(m_clpRngs.comp[(int)compID]));
                            }
                        }
                        else if (filterType == ALF_FILTER_7)
                        {
                            deriveClassification(m_classifier, p_buffer, l_stride, &blk);
                            m_AdaptiveLoopFilter.m_filter7x7Blk(m_classifier, recBuf + xPos + yPos * (recPic->s_l), recPic->s_l, p_buffer, l_stride, &blk, compID, coeff, &(m_clpRngs.comp[(int)compID]));
                        }
                        else
                        {
                            CHECK(0, "Wrong ALF filter type");
                        }
                    }
                }

                x_loc++;

                if (x_loc >= ctx->tile[tile_idx].w_ctb + col_bd)
                {
                    x_loc = ((ctx->tile[tile_idx].ctba_rs_first) % ctx->w_lcu);
                    y_loc++;

                }
                ctuIdx = x_loc + y_loc * ctx->w_lcu;
            }
        }
    }    
}

void EncAdaptiveLoopFilter::alfTemporalEncoderAPSComponent(CodingStructure& cs, AlfSliceParam* alfSliceParam)
{
    EVCE_CTX* ctx = (EVCE_CTX*)cs.pCtx;
    const int tempLayerId = ctx->nalu.nuh_temporal_id;
    int prevIdxComp[MAX_NUM_CHANNEL_TYPE] = { -1, -1 };
#if M53608_ALF_9
    int talfCompEnable[MAX_NUM_COMPONENT] = { 0, 0, 0 };
#else
    int talfCompEnable[MAX_NUM_CHANNEL_TYPE] = { 0, 0 };
#endif
#if M53608_ALF_9
    double unfilterd_cost_cb = DBL_MAX;
    double unfilterd_cost_cr = DBL_MAX;
    double unfilterd_cost_joint = DBL_MAX;
#endif
    AlfSliceParam *pcStoredAlfPara = ctx->slice_type == SLICE_I ? NULL : m_acAlfLineBuffer;

    copyAlfParam(&m_alfSliceParamTemp, alfSliceParam);
#if M53608_ALF_9
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, COMPONENT_Y);
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, COMPONENT_Cb);
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, COMPONENT_Cr);
#else
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
    copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);
#endif

    ChannelType channel;
    if (pcStoredAlfPara != NULL && m_acAlfLineBufferCurrentSize > 0)
    {
#if M53608_ALF_9
        double costBest[MAX_NUM_COMPONENT] = { DBL_MAX, DBL_MAX, DBL_MAX };
#else
        double costBest[MAX_NUM_CHANNEL_TYPE] = { DBL_MAX, DBL_MAX };
#endif
        for (int bufIdx2 = 0; bufIdx2 < m_acAlfLineBufferCurrentSize && bufIdx2 < APS_MAX_NUM; bufIdx2++)
        {
#if M53608_ALF_9
            double cost[MAX_NUM_COMPONENT + 1] = { DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX };
#else
            double cost[MAX_NUM_CHANNEL_TYPE] = { DBL_MAX, DBL_MAX };
#endif
            int bufIdx = bufIdx2;
            bufIdx = m_alfIndxInScanOrder[bufIdx2];
            {
                if ((pcStoredAlfPara[bufIdx].tLayer > tempLayerId) && (ctx->param.i_period != 0))
                {
                    continue;
                }
                if ((m_currentPoc > pcStoredAlfPara[bufIdx].m_maxIdrPoc + ctx->param.i_period) && (ctx->param.i_period != 0))
                {
                    continue;
                }

                if ((m_currentPoc > m_lastIdrPoc)
                    && (pcStoredAlfPara[bufIdx].m_filterPoc < m_lastIdrPoc)
                    )

                {
                    continue;
                }

                if ((m_currentPoc > pcStoredAlfPara[bufIdx].m_maxIdrPoc)
                    && (pcStoredAlfPara[bufIdx].m_filterPoc < pcStoredAlfPara[bufIdx].m_maxIdrPoc)
                    )

                {
                    continue;
                }
            }

            copyAlfParam(&m_alfSliceParamTemp, &(pcStoredAlfPara[bufIdx]));
#if M53608_ALF_9
            for (int ch = 0; ch < MAX_NUM_COMPONENT; ch++)
#else
            for (int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
#endif
            {
#if M53608_ALF_9
                channel = toChannelType(ComponentID(ch));
#else
                channel = (ChannelType)ch;
#endif
                {
#if M53608_ALF_9
                    int isFilterAvailable = (ch == COMPONENT_Y) ? m_alfSliceParamTemp.enabledFlag[COMPONENT_Y] : (ch == COMPONENT_Cb ? m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] : m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr]);
#else
                    int isFilterAvailable = (ch == CHANNEL_TYPE_LUMA) ? m_alfSliceParamTemp.enabledFlag[COMPONENT_Y] : (m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] || m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr]);
#endif
                    if (isFilterAvailable)
                    {
#if M53608_ALF_8
                        int iShapeIdx = m_alfSliceParamTemp.lumaFilterType;
#else
                        int iShapeIdx = isLuma(channel) ? ALF_FILTER_7 : ALF_FILTER_5;
#endif
#if M53608_ALF_9
                        if (ch == COMPONENT_Y)
#else
                        if (channel == CHANNEL_TYPE_LUMA)
#endif
                        {
                            double distUnfilter;
#if M53608_ALF_9
                            cost[ch] = deriveCtbAlfEnableFlags(cs, iShapeIdx, COMPONENT_Y, isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1, m_filterShapes[channel][iShapeIdx].numCoeff, distUnfilter, true);
                            cost[ch] += m_lambda[COMPONENT_Y] * APS_MAX_NUM_IN_BITS;
#else
                            cost[channel] = deriveCtbAlfEnableFlags(cs, iShapeIdx, channel, isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1, m_filterShapes[channel][iShapeIdx].numCoeff, distUnfilter, true);
                            cost[channel] += m_lambda[channel] * APS_MAX_NUM_IN_BITS;
#endif

                            for (int i = 0; i < ctx->f_lcu; i++)
                            {
#if M53608_ALF_9
                                if (m_ctuEnableFlag[COMPONENT_Y][i] == 0)
#else
                                if (m_ctuEnableFlag[CHANNEL_TYPE_LUMA][i] == 0)
#endif
                                {
                                    m_alfSliceParamTemp.isCtbAlfOn = true;
                                    break;
                                }
                                else
                                    m_alfSliceParamTemp.isCtbAlfOn = false;
                            }
                            if (m_alfSliceParamTemp.isCtbAlfOn)
#if M53608_ALF_9
                                cost[ch] += m_lambda[ch] * (ctx->f_lcu);
#else
                                cost[channel] += m_lambda[channel] * (ctx->f_lcu);
#endif
                        }
#if M53608_ALF_9
                        else if (ch == COMPONENT_Cb)
                        {
                            double costCtbEnable = DBL_MAX;
                            setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cb, true);
                            getFrameStats(COMPONENT_Cb, 0);
                            costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cb][0], channel);
                            unfilterd_cost_cb = costCtbEnable;
                            reconstructCoeff(&m_alfSliceParamTemp, channel, true, isLuma(channel));
                            for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
                            {
                                for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
                                {
                                    m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfSliceParamTemp.chromaCoeff[i];
                                }
                            }
#if M53608_ALF_9
                            costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cb][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                            cost[ch] = costCtbEnable;
                            cost[ch] += m_lambda[ch] * APS_MAX_NUM_IN_BITS;
#else
                            costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                            cost[channel] = costCtbEnable;
                            cost[channel] += m_lambda[channel] * APS_MAX_NUM_IN_BITS;
#endif
                        }
                        else if (ch == COMPONENT_Cr)
                        {
                            double costCtbEnable = DBL_MAX;
                            setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cr, true);
                            getFrameStats(COMPONENT_Cr, 0);
                            costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cr][0], channel);
                            unfilterd_cost_cr = costCtbEnable;
                            reconstructCoeff(&m_alfSliceParamTemp, channel, true, isLuma(channel));
                            for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
                            {
                                for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
                                {
                                    m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfSliceParamTemp.chromaCoeff[i];
                                }
                            }
#if M53608_ALF_9
                            costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cr][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                            cost[ch] = costCtbEnable;
                            cost[ch] += m_lambda[ch] * APS_MAX_NUM_IN_BITS;
#else
                            costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                            cost[channel] = costCtbEnable;
                            cost[channel] += m_lambda[channel] * APS_MAX_NUM_IN_BITS;
#endif
#if M53608_ALF_9
                            if (m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] && m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr])
                            {
                                costCtbEnable = 0;
                                setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cr, true);
                                setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cb, true);
                                getFrameStats(COMPONENT_Cr, 0);
                                getFrameStats(COMPONENT_Cb, 0);
                                m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][0].reset();
                                m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][0] += m_alfCovarianceFrame[COMPONENT_Cb][0][0];
                                m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][0] += m_alfCovarianceFrame[COMPONENT_Cr][0][0];
                                costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[MAX_NUM_COMPONENT][0], channel);
                                unfilterd_cost_joint = costCtbEnable;
                                reconstructCoeff(&m_alfSliceParamTemp, channel, true, isLuma(channel));
                                for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
                                {
                                    for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
                                    {
                                        m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfSliceParamTemp.chromaCoeff[i];
                                    }
                                }
                                costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[MAX_NUM_COMPONENT][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                                cost[MAX_NUM_COMPONENT] = costCtbEnable;
                                cost[MAX_NUM_COMPONENT] += m_lambda[ch] * APS_MAX_NUM_IN_BITS;
                            }
#endif
                        }
#else
                        else
                        {
                            double costCtbEnable = DBL_MAX;
                            setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                            getFrameStats(CHANNEL_TYPE_CHROMA, 0);
                            costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA);

                            reconstructCoeff(&m_alfSliceParamTemp, channel, true, isLuma(channel));
                            for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
                            {
                                for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
                                {
                                    m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfSliceParamTemp.chromaCoeff[i];
                                }
                            }
                            costCtbEnable += getFilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], 1, 0, MAX_NUM_ALF_CHROMA_COEFF);
                            cost[channel] = costCtbEnable;
                            cost[channel] += m_lambda[channel] * APS_MAX_NUM_IN_BITS;
                         }
#endif
                    }
                    else {
#if M53608_ALF_9
                    if (channel == CHANNEL_TYPE_CHROMA)
                    {
                        if (ch == COMPONENT_Cb)
                        {
                            setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cb, true);
                            getFrameStats(COMPONENT_Cb, 0);
                            cost[ch] = getUnfilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cb][0], channel);
                            unfilterd_cost_cb = cost[ch];
                        }
                        else if (ch == COMPONENT_Cr)
                        {
                            setCtuEnableFlag(m_ctuEnableFlag, COMPONENT_Cr, true);
                            getFrameStats(COMPONENT_Cr, 0);
                            cost[ch] = getUnfilteredDistortion(m_alfCovarianceFrame[COMPONENT_Cr][0], channel);
                            unfilterd_cost_cr = cost[ch];
                        }
                    }
#else
                    if (channel == CHANNEL_TYPE_CHROMA)
                    {
                        setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                        getFrameStats(CHANNEL_TYPE_CHROMA, 0);
                        cost[channel] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA);
                    }
#endif
                        else
                        {
                            printf("Error: temporal ALF checked, but enableFlag for luma is OFF\n");
                        }
                    }
                }
#if M53608_ALF_9
                if (ch == COMPONENT_Y)
                {
                    bool isCurrentBetterLocal = cost[ch] < costBest[ch];
                    if (isCurrentBetterLocal)
                    {
                        talfCompEnable[ch] = 1;
                        costBest[ch] = cost[ch];
                        prevIdxComp[ch] = bufIdx;
                        copyCtuEnableFlag(&m_ctuEnableFlagTmpLuma, m_ctuEnableFlag, COMPONENT_Y);
                    }
                }
                else if (ch == COMPONENT_Cr)
                {
                    if (m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] && m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr])
                    {
                        if (cost[MAX_NUM_COMPONENT] < costBest[COMPONENT_Cb] + costBest[COMPONENT_Cr])
                        {
                            costBest[COMPONENT_Cb] = cost[MAX_NUM_COMPONENT] / 2;
                            costBest[COMPONENT_Cr] = cost[MAX_NUM_COMPONENT] / 2;
                            prevIdxComp[channel] = bufIdx;
                            talfCompEnable[COMPONENT_Cb] = 1;
                            talfCompEnable[COMPONENT_Cr] = 1;
                        }
                    }
                    else if (m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb])
                    {
                        if (cost[COMPONENT_Cb] + cost[COMPONENT_Cr] < costBest[COMPONENT_Cb] + costBest[COMPONENT_Cr])
                        {
                            costBest[COMPONENT_Cb] = cost[COMPONENT_Cb];
                            costBest[COMPONENT_Cr] = cost[COMPONENT_Cr];
                            prevIdxComp[channel] = bufIdx;
                            talfCompEnable[COMPONENT_Cb] = 1;
                            talfCompEnable[COMPONENT_Cr] = 0;
                        }
                    }
                    else if (m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr])
                    {
                        if (cost[COMPONENT_Cb] + cost[COMPONENT_Cr] < costBest[COMPONENT_Cb] + costBest[COMPONENT_Cr])
                        {
                            costBest[COMPONENT_Cb] = cost[COMPONENT_Cb];
                            costBest[COMPONENT_Cr] = cost[COMPONENT_Cr];
                            prevIdxComp[channel] = bufIdx;
                            talfCompEnable[COMPONENT_Cb] = 0;
                            talfCompEnable[COMPONENT_Cr] = 1;
                        }
                    }
                    else
                    {
                        if (cost[COMPONENT_Cb] + cost[COMPONENT_Cr] < costBest[COMPONENT_Cb] + costBest[COMPONENT_Cr])
                        {
                            costBest[COMPONENT_Cb] = cost[COMPONENT_Cb];
                            costBest[COMPONENT_Cr] = cost[COMPONENT_Cr];
                            prevIdxComp[channel] = bufIdx;
                            talfCompEnable[COMPONENT_Cb] = 0;
                            talfCompEnable[COMPONENT_Cr] = 1;
                        }
                    }
                }

#else
                bool isCurrentBetterLocal = cost[channel] < costBest[channel];
                if (isCurrentBetterLocal)
                {
                    talfCompEnable[channel] = 1;
                    costBest[channel] = cost[channel];
                    prevIdxComp[channel] = bufIdx;
                    if (channel == CHANNEL_TYPE_LUMA)
                    {
                        copyCtuEnableFlag(&m_ctuEnableFlagTmpLuma, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
                    }
                }
#endif
            }
        }
#if M53608_ALF_9
        bool isCurrentBetterGlobal = (costBest[COMPONENT_Y] + costBest[COMPONENT_Cb] + costBest[COMPONENT_Cr]) < (m_costAlfEncoder[COMPONENT_Y] + m_costAlfEncoder[COMPONENT_Cb] + m_costAlfEncoder[COMPONENT_Cr]);
#else
        bool isCurrentBetterGlobal = (costBest[CHANNEL_TYPE_LUMA] + costBest[CHANNEL_TYPE_CHROMA]) < (m_costAlfEncoder[CHANNEL_TYPE_LUMA] + m_costAlfEncoder[CHANNEL_TYPE_CHROMA]);
#endif
        if (isCurrentBetterGlobal)
        {
#if M53608_ALF_9
            if (talfCompEnable[COMPONENT_Y])
#else
            if (talfCompEnable[CHANNEL_TYPE_LUMA])
#endif
            {
                m_costAlfEncoder[CHANNEL_TYPE_LUMA] = costBest[CHANNEL_TYPE_LUMA];
                copyAlfParam(alfSliceParam, &(pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_LUMA]]));
                alfSliceParam->prevIdx = prevIdxComp[CHANNEL_TYPE_LUMA];
                alfSliceParam->prevIdxComp[CHANNEL_TYPE_LUMA] = prevIdxComp[CHANNEL_TYPE_LUMA];
#if M53608_ALF_9
                copyCtuEnableFlag(m_ctuEnableFlagTmp, &m_ctuEnableFlagTmpLuma, COMPONENT_Y);
#else
                copyCtuEnableFlag(m_ctuEnableFlagTmp, &m_ctuEnableFlagTmpLuma, CHANNEL_TYPE_LUMA);
#endif
                alfSliceParam->enabledFlag[0] = 1;
            }
            else
            {
                alfSliceParam->enabledFlag[0] = 0;
                alfSliceParam->prevIdxComp[0] = -1;
            }
#if M53608_ALF_9
            if (talfCompEnable[COMPONENT_Cb] || talfCompEnable[COMPONENT_Cr])
            {
                m_costAlfEncoder[COMPONENT_Cb] = costBest[COMPONENT_Cb];
                m_costAlfEncoder[COMPONENT_Cr] = costBest[COMPONENT_Cr];
                copyAlfParamChroma(alfSliceParam, &(pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]]));
                alfSliceParam->prevIdxComp[CHANNEL_TYPE_CHROMA] = prevIdxComp[CHANNEL_TYPE_CHROMA];
                setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cb, pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]].enabledFlag[1]);
                setCtuEnableFlag(m_ctuEnableFlagTmp, COMPONENT_Cr, pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]].enabledFlag[2]);
                alfSliceParam->enabledFlag[1] = (pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]].enabledFlag[1]);
                alfSliceParam->enabledFlag[2] = (pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]].enabledFlag[2]);
            }
            else
            {
                alfSliceParam->enabledFlag[1] = 0;
                alfSliceParam->enabledFlag[2] = 0;
                alfSliceParam->prevIdxComp[1] = -1;
            }
#else
            if (talfCompEnable[CHANNEL_TYPE_CHROMA])
            {
                m_costAlfEncoder[CHANNEL_TYPE_CHROMA] = costBest[CHANNEL_TYPE_CHROMA];
                copyAlfParamChroma(alfSliceParam, &(pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]]));
                alfSliceParam->prevIdxComp[CHANNEL_TYPE_CHROMA] = prevIdxComp[CHANNEL_TYPE_CHROMA];
                setCtuEnableFlag(m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA, true);
                alfSliceParam->enabledFlag[1] = (pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]].enabledFlag[1]);
                alfSliceParam->enabledFlag[2] = (pcStoredAlfPara[prevIdxComp[CHANNEL_TYPE_CHROMA]].enabledFlag[2]);
            }
            else
            {
                alfSliceParam->enabledFlag[1] = 0;
                alfSliceParam->enabledFlag[2] = 0;
                alfSliceParam->prevIdxComp[1] = -1;
            }
#endif
            alfSliceParam->temporalAlfFlag = true;
            alfSliceParam->chromaCtbPresentFlag = false;
        }
        else {
            alfSliceParam->temporalAlfFlag = false;
            alfSliceParam->prevIdxComp[0] = -1;
            alfSliceParam->prevIdxComp[1] = -1;
        }

    }
#if M53608_ALF_9
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, COMPONENT_Y);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, COMPONENT_Cb);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, COMPONENT_Cr);
#else
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA);
#endif
}
#if !M53608_ALF_9
void EncAdaptiveLoopFilter::alfTemporalEncoderAPS(CodingStructure& cs, AlfSliceParam* alfSliceParam)
{
    if (!alfSliceParam->enabledFlag[COMPONENT_Y])
    {
        setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, true);
        getFrameStats(CHANNEL_TYPE_CHROMA, 0);
        m_costAlfEncoder[CHANNEL_TYPE_CHROMA] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + lengthTruncatedUnary(0, 3) * m_lambda[CHANNEL_TYPE_CHROMA];
        setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, false);
    }

    EVCE_CTX* ctx = (EVCE_CTX*)cs.pCtx;

    double cost[MAX_NUM_CHANNEL_TYPE] = { DBL_MAX, DBL_MAX };
    const int tempLayerId = ctx->nalu.nuh_temporal_id; //cs.slice->getTLayer();

    AlfSliceParam *pcStoredAlfPara = ctx->slice_type == SLICE_I ? NULL : m_acAlfLineBuffer;

    copyAlfParam(&m_alfSliceParamTemp, alfSliceParam);
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

            copyAlfParam(&m_alfSliceParamTemp, &(pcStoredAlfPara[bufIdx]));

            for (int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
            {
                ChannelType channel = (ChannelType)ch;
                if ((channel == CHANNEL_TYPE_CHROMA && !m_alfSliceParamTemp.enabledFlag[COMPONENT_Y]) || (channel == CHANNEL_TYPE_CHROMA && !m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] && !m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr]))
                {
                    setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                    getFrameStats(channel, 0);
                    cost[channel] = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA) + lengthTruncatedUnary(0, 3) * m_lambda[channel];
                    setEnableFlag(&m_alfSliceParamTemp, channel, false);
                    setCtuEnableFlag(m_ctuEnableFlag, channel, false);
                }
                else
                {
                    double costCtbEnable = DBL_MAX;
                    if (channel == CHANNEL_TYPE_CHROMA && m_alfSliceParamTemp.enabledFlag[COMPONENT_Y])
                    {
                        setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                        getFrameStats(CHANNEL_TYPE_CHROMA, 0);
                        costCtbEnable = getUnfilteredDistortion(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][0], CHANNEL_TYPE_CHROMA);
                        // reconstruct chroma coeffs
                        reconstructCoeff(&m_alfSliceParamTemp, channel, true, isLuma(channel));
                        for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
                        {
                            for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
                            {
                                m_filterCoeffSet[classIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] : m_alfSliceParamTemp.chromaCoeff[i];
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

                    if (channel == CHANNEL_TYPE_CHROMA && m_alfSliceParamTemp.enabledFlag[COMPONENT_Y])
                    {
                        if (costCtbEnable < cost[channel])
                        {
                            setEnableFlag(&m_alfSliceParamTemp, channel, true);
                            setCtuEnableFlag(m_ctuEnableFlag, channel, true);
                            m_alfSliceParamTemp.chromaCtbPresentFlag = true;
                            cost[channel] = costCtbEnable;
                        }
                    }
                }
                if (channel == CHANNEL_TYPE_LUMA)
                {
                    cost[channel] += m_lambda[channel] * APS_MAX_NUM_IN_BITS;

                    for (int i = 0; i < ctx->f_lcu; i++)
                    {
                        if (*(m_alfSliceParamTemp.alfCtuEnableFlag + i) == 0)
                        {
                            m_alfSliceParamTemp.isCtbAlfOn = true;
                            break;
                        }
                        else
                            m_alfSliceParamTemp.isCtbAlfOn = false;
                    }
                    if (m_alfSliceParamTemp.isCtbAlfOn)
                        cost[channel] += m_lambda[channel] * (ctx->f_lcu);
                }
            } // channel loop

            bool isCurrentBetter = (cost[CHANNEL_TYPE_LUMA] + cost[CHANNEL_TYPE_CHROMA]) < (m_costAlfEncoder[CHANNEL_TYPE_LUMA] + m_costAlfEncoder[CHANNEL_TYPE_CHROMA]);
            //      isCurrentBetter = false;
            if (isCurrentBetter)
            {
                m_costAlfEncoder[CHANNEL_TYPE_LUMA] = cost[CHANNEL_TYPE_LUMA];

                copyAlfParam(alfSliceParam, &m_alfSliceParamTemp);
                alfSliceParam->prevIdx = bufIdx;
                alfSliceParam->temporalAlfFlag = true;
                alfSliceParam->chromaCtbPresentFlag = false;
                copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
                copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA);
            }
        } // prevIdx search
        copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
        copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA);
    } // end if 
}
#endif
void EncAdaptiveLoopFilter::xDeriveCovFromLgrTapFilter(AlfCovariance& covLgr, AlfCovariance& covSml, int* patternSml
#if M53608_ALF_8
    , AlfFilterType lumafiltertype
#endif
)
{
  covSml.pixAcc = covLgr.pixAcc;
#if M53608_ALF_8
  for (int i = 0; i < (lumafiltertype ? 13 : 7); i++)
#else
  for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
#endif
  {
    if (patternSml[i] > 0)
    {
      covSml.y[patternSml[i] - 1] = covLgr.y[i];
#if M53608_ALF_8
      for (int j = 0; j < (lumafiltertype ? 13 : 7); j++)
#else
      for (int j = 0; j < MAX_NUM_ALF_LUMA_COEFF; j++)
#endif
      {
        if (patternSml[j] > 0)
        {
          covSml.E[patternSml[i] - 1][patternSml[j] - 1] = covLgr.E[i][j];
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyAlfSliceParam(AlfSliceParam* alfSliceParamDst, AlfSliceParam* alfSliceParamSrc, ChannelType channel)
{
  if (isLuma(channel))
  {
    u8* temp = alfSliceParamDst->alfCtuEnableFlag;
    memcpy(alfSliceParamDst, alfSliceParamSrc, sizeof(AlfSliceParam));
    alfSliceParamDst->alfCtuEnableFlag = temp;
    memcpy(alfSliceParamDst->alfCtuEnableFlag, alfSliceParamSrc->alfCtuEnableFlag, m_numCTUsInPic * sizeof(u8));
  }
  else
  {
    alfSliceParamDst->enabledFlag[COMPONENT_Cb] = alfSliceParamSrc->enabledFlag[COMPONENT_Cb];
    alfSliceParamDst->enabledFlag[COMPONENT_Cr] = alfSliceParamSrc->enabledFlag[COMPONENT_Cr];
    alfSliceParamDst->chromaCtbPresentFlag = alfSliceParamSrc->chromaCtbPresentFlag;
    memcpy(alfSliceParamDst->chromaCoeff, alfSliceParamSrc->chromaCoeff, sizeof(short)*MAX_NUM_ALF_CHROMA_COEFF);
  }
}
#if M53608_ALF_9
double EncAdaptiveLoopFilter::getFilterCoeffAndCost(CodingStructure& cs, double distUnfilter, ComponentID compID, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits
#if ALF_CONFORMANCE_CHECK
    , u8* filter_conformance_flag
#endif
)
#else
double EncAdaptiveLoopFilter::getFilterCoeffAndCost(CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits)
#endif
{
#if M53608_ALF_9
  ChannelType channel = toChannelType(compID);
#endif
  const int size = channel == CHANNEL_TYPE_LUMA ? 2 : 1;

  if (bReCollectStat)
  {
#if M53608_ALF_9
#if M53608_ALF_8
      getFrameStats(compID, iShapeIdx);
#else
      getFrameStats(compID, (int)size - 1);
#endif
#else
#if M53608_ALF_8
      getFrameStats(channel, iShapeIdx);
#else
      getFrameStats(channel, (int)size - 1);
#endif
#endif
  }

  double dist = distUnfilter;
  uiCoeffBits = 0;
  int uiSliceFlag = 0;
  AlfFilterShape alfFilterShape = m_alfSliceParamTemp.filterShapes[channel][iShapeIdx];
#if M53608_ALF_9
  if (compID == COMPONENT_Y)
  {
#if M53608_ALF_8
      dist += mergeFiltersAndCost(&m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits
#if ALF_CONFORMANCE_CHECK
          , filter_conformance_flag
#endif
      );
#else
      dist += mergeFiltersAndCost(&m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][ALF_FILTER_7], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits);
#endif
  }
  else if (compID == COMPONENT_Cb || compID == COMPONENT_Cr)
  {
      dist += m_alfCovarianceFrame[compID][iShapeIdx][0].pixAcc + deriveCoeffQuant(m_filterCoeffQuant, m_alfCovarianceFrame[compID][iShapeIdx][0].E, m_alfCovarianceFrame[compID][iShapeIdx][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true);
      memcpy(m_filterCoeffSet[0], m_filterCoeffQuant, sizeof(*m_filterCoeffQuant) * alfFilterShape.numCoeff);
      const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
          m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
      }
      uiCoeffBits += getCoeffRate(&m_alfSliceParamTemp, true);
      uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }
#else
  if (isLuma(channel))
  {
#if M53608_ALF_8
      dist += mergeFiltersAndCost(&m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits);
#else
      dist += mergeFiltersAndCost(&m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][ALF_FILTER_7], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits);
#endif
  }
  else
  {
      dist = deriveCoeffQuant(m_filterCoeffQuant, m_alfCovarianceFrame[channel][iShapeIdx][0].E, m_alfCovarianceFrame[channel][iShapeIdx][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true);
      dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc;
      memcpy(m_filterCoeffSet[0], m_filterCoeffQuant, sizeof(*m_filterCoeffQuant) * alfFilterShape.numCoeff);
      const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
          m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
      }
      uiCoeffBits += getCoeffRate(&m_alfSliceParamTemp, true);
      uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }
#endif

  double rate = uiCoeffBits + uiSliceFlag;
  if (isLuma(channel) || (!m_alfSliceParamTemp.chromaCtbPresentFlag))
  {
    if (isChroma(channel))
    {
      CHECK(m_alfSliceParamTemp.chromaCtbPresentFlag, "chromaCTB is on");
    }
    else
    {
      CHECK(!m_alfSliceParamTemp.enabledFlag[COMPONENT_Y], "Slice Y is off");
    }
  }
#if M53608_ALF_9
  if (compID == COMPONENT_Y)
  {
      return dist + m_lambda[COMPONENT_Y] * rate;
  }
  else if (compID == COMPONENT_Cb)
  {
      return dist + m_lambda[COMPONENT_Cb] * rate;
  }
  else
  {
      return dist + m_lambda[COMPONENT_Cr] * rate;
  }
#else
  return dist + m_lambda[channel] * rate;
#endif
}
#if M53608_ALF_9
void EncAdaptiveLoopFilter::getFilterCoeffAndCost_Chroma(CodingStructure& cs, double distUnfilter, ComponentID compID, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, double* filter_cost)
{
  ChannelType channel = toChannelType(compID);
  AlfFilterShape alfFilterShape = m_alfSliceParamTemp.filterShapes[channel][iShapeIdx];
  double dist = 0;
  int uiSliceFlag = 0;
  uiCoeffBits = 0;
  double rate = 0;
  int alfChromaIdc = 0;
  m_alfCovarianceFrame[MAX_NUM_COMPONENT][iShapeIdx][0].reset();
  m_alfCovarianceFrame[MAX_NUM_COMPONENT][iShapeIdx][0] += m_alfCovarianceFrame[1][iShapeIdx][0];
  m_alfCovarianceFrame[MAX_NUM_COMPONENT][iShapeIdx][0] += m_alfCovarianceFrame[2][iShapeIdx][0];
  dist += m_alfCovarianceFrame[MAX_NUM_COMPONENT][iShapeIdx][0].pixAcc + deriveCoeffQuant(m_filterCoeffQuant, m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][0].E, m_alfCovarianceFrame[MAX_NUM_COMPONENT][0][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true);
  memcpy(m_filterCoeffSet[0], m_filterCoeffQuant, sizeof(*m_filterCoeffQuant) * alfFilterShape.numCoeff);
#if ALF_CONFORMANCE_CHECK
  u8 filter_conformance_flag = 0;
  int sum = 0;
  int factor = (1 << (m_NUM_BITS - 1));
#endif
  for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF - 1; i++)
  {
    m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
#if ALF_CONFORMANCE_CHECK
    if (m_filterCoeffQuant[i] < -(1 << 9) || m_filterCoeffQuant[i] > (1 << 9) - 1)
        filter_conformance_flag = 1;
    sum += m_alfSliceParamTemp.chromaCoeff[i] << 1;
#endif
  }
#if ALF_CONFORMANCE_CHECK
  int last_coeff = factor - sum;
  if (last_coeff < -(1 << 10) || last_coeff > (1 << 10) - 1)
      filter_conformance_flag = 1;
#endif
  uiCoeffBits += getCoeffRate(&m_alfSliceParamTemp, true);
  alfChromaIdc = 3;
  uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  rate = uiCoeffBits + uiSliceFlag;
  filter_cost[2] = dist + m_lambda[COMPONENT_Cb] * rate;
  dist = m_alfCovarianceFrame[COMPONENT_Cb][iShapeIdx][0].pixAcc + calcErrorForCoeffs(m_alfCovarianceFrame[COMPONENT_Cb][iShapeIdx][0].E, m_alfCovarianceFrame[COMPONENT_Cb][iShapeIdx][0].y, m_filterCoeffQuant, 7, 10);
  alfChromaIdc = 2;
  uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  rate = uiCoeffBits + uiSliceFlag;
  filter_cost[0] = dist + m_lambda[COMPONENT_Cb] * rate;
  dist = m_alfCovarianceFrame[COMPONENT_Cr][iShapeIdx][0].pixAcc + calcErrorForCoeffs(m_alfCovarianceFrame[COMPONENT_Cr][iShapeIdx][0].E, m_alfCovarianceFrame[COMPONENT_Cr][iShapeIdx][0].y, m_filterCoeffQuant, 7, 10);
  alfChromaIdc = 1;
  uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  rate = uiCoeffBits + uiSliceFlag;
  filter_cost[1] = dist + m_lambda[COMPONENT_Cr] * rate;
#if ALF_CONFORMANCE_CHECK
  if (filter_conformance_flag)
  {
      filter_cost[0] = filter_cost[1] = filter_cost[2] = MAX_COST;
  }
#endif
}
#endif
int EncAdaptiveLoopFilter::getCoeffRate(AlfSliceParam* alfSliceParam, bool isChroma)
{
  int iBits = 0;
  if (!isChroma)
  {
    iBits++;                                               // alf_coefficients_delta_flag
    if (!alfSliceParam->coeffDeltaFlag)
    {
      if (alfSliceParam->numLumaFilters > 1)
      {
        iBits++;                                           // coeff_delta_pred_mode_flag
      }
    }
  }

  memset(m_bitsCoeffScan, 0, sizeof(m_bitsCoeffScan));
  AlfFilterShape alfShape;
  init_AlfFilterShape(&alfShape, isChroma ? 5 : (alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7));
  const int maxGolombIdx = getMaxGolombIdx((AlfFilterType)alfShape.filterType);
  const short* coeff = isChroma ? alfSliceParam->chromaCoeff : alfSliceParam->lumaCoeff;
  const int numFilters = isChroma ? 1 : alfSliceParam->numLumaFilters;

  // vlc for all
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (isChroma || !alfSliceParam->coeffDeltaFlag || alfSliceParam->filterCoeffFlag[ind])
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        int coeffVal = abs(coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i]);

        for (int k = 1; k < 15; k++)
        {
#if ETM70_GOLOMB_FIX
            m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k, TRUE);
#else
            m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
#endif
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
    if (alfSliceParam->coeffDeltaFlag)
    {
      iBits += numFilters;             //filter_coefficient_flag[i]
    }
  }

  // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (!isChroma && !alfSliceParam->filterCoeffFlag[ind] && alfSliceParam->coeffDeltaFlag)
    {
      continue;
    }

    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
#if ETM70_GOLOMB_FIX
        iBits += lengthGolomb(coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], m_kMinTab[alfShape.golombIdx[i]], TRUE);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#else
        iBits += lengthGolomb(coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], m_kMinTab[alfShape.golombIdx[i]]);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
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
#if ALF_CONFORMANCE_CHECK
void EncAdaptiveLoopFilter::conformaceCheck(AlfSliceParam* alfSliceParam, u8* filter_conformance_flag)
{
    int factor = (1 << (m_NUM_BITS - 1));
    int numCoeff = alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 7 : 13;
    short lumaCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF] = { 0, };
    short coeffRec[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF] = { 0, };
    int numFilters = alfSliceParam->numLumaFilters;
    if (alfSliceParam->coeffDeltaPredModeFlag)
    {
        for (int j = 0; j < numCoeff - 1; j++)
        {
            lumaCoeff[j] = alfSliceParam->lumaCoeff[j];
        }
        for (int i = 1; i < numFilters; i++)
        {
            for (int j = 0; j < numCoeff - 1; j++)
            {
                lumaCoeff[i * MAX_NUM_ALF_LUMA_COEFF + j] = alfSliceParam->lumaCoeff[i * MAX_NUM_ALF_LUMA_COEFF + j] + alfSliceParam->lumaCoeff[(i - 1) * MAX_NUM_ALF_LUMA_COEFF + j];
            }
        }
    }
    else
    {
        for (int j = 0; j < MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF; j++)
        {
            lumaCoeff[j] = alfSliceParam->lumaCoeff[j];
        }
    }
    int numCoeffLargeMinus1 = MAX_NUM_ALF_LUMA_COEFF - 1;
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
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
        int sum = 0;
        for (int i = 0; i < numCoeffLargeMinus1; i++)
        {
            int curCoeff = 0;
            //fixed filter
            if (fixedFilterUsageFlag > 0)
            {
                curCoeff = m_fixedFilterCoeff[fixedFilterIdx][i];
            }
            //add coded coeff
            if (m_filterShapes[CHANNEL_TYPE_LUMA][alfSliceParam->lumaFilterType].patternToLargeFilter[i] > 0)
            {
                int coeffIdx = m_filterShapes[CHANNEL_TYPE_LUMA][alfSliceParam->lumaFilterType].patternToLargeFilter[i] - 1;
                curCoeff += lumaCoeff[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx];
            }
            coeffRec[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] = curCoeff;
            if (coeffRec[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] < -(1 << 9) || coeffRec[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] > (1 << 9) - 1)
            {
                *filter_conformance_flag = 1;
                break;
            }
            sum += (coeffRec[classIdx* MAX_NUM_ALF_LUMA_COEFF + i] << 1);
        }
        if (*filter_conformance_flag)
            break;
        int last_coeff = factor - sum;
        if (last_coeff < -(1 << 10) || last_coeff >(1 << 10) - 1)
        {
            *filter_conformance_flag = 1;
            break;
        }
    }
}
#endif

double EncAdaptiveLoopFilter::mergeFiltersAndCost(AlfSliceParam* alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits
#if ALF_CONFORMANCE_CHECK
    , u8* filter_conformance_flag
#endif
)
{

  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = DBL_MAX;
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;

  findBestFixedFilter(alfSliceParam, covFrame);

  if (alfShape.filterType == ALF_FILTER_5)
  {
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      xDeriveCovFromLgrTapFilter(covFrame[classIdx], m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][ALF_FILTER_5][classIdx], alfShape.patternToLargeFilter
#if M53608_ALF_8
        , alfSliceParam->lumaFilterType
#endif
      );
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

  alfSliceParam->numLumaFilters = numFiltersBest;

  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfSliceParam->coeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
    alfSliceParam->coeffDeltaPredModeFlag = bestPredMode;
  }
  else
  {
    distReturn = distForce0;
    alfSliceParam->coeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    for(int i=0; i<MAX_NUM_ALF_CLASSES; i++)
        alfSliceParam->filterCoeffFlag[i] = BOOL(codedVarBins[i]);
    alfSliceParam->coeffDeltaPredModeFlag = 0;

    for (int varInd = 0; varInd < numFiltersBest; varInd++)
    {
      if (codedVarBins[varInd] == 0)
      {
        memset(m_filterCoeffSet[varInd], 0, sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
      }
    }
  }
  for (int ind = 0; ind < alfSliceParam->numLumaFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff; i++)
    {
      if (alfSliceParam->coeffDeltaPredModeFlag)
      {
        alfSliceParam->lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_diffFilterCoeff[ind][i];
      }
      else
      {
        alfSliceParam->lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      }
    }
  }

  memcpy(alfSliceParam->filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof(short) * MAX_NUM_ALF_CLASSES);
  const int iNumFixedFilterPerClass = ALF_FIXED_FILTER_NUM;
  if (iNumFixedFilterPerClass > 0)
  {
#if M53608_ALF_7
      int fixedFilterPattern = alfSliceParam->fixedFilterUsageFlag[0] ? 1 : 0;
#else
      int fixedFilterPattern = alfSliceParam->fixedFilterIdx[0] > 0 ? 1 : 0;
#endif
    for (int classIdx = 1; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
#if M53608_ALF_7
        int iCurrFixedFilterPattern = alfSliceParam->fixedFilterUsageFlag[classIdx] ? 1 : 0;
#else
        int iCurrFixedFilterPattern = alfSliceParam->fixedFilterIdx[classIdx] > 0 ? 1 : 0;
#endif
      if (iCurrFixedFilterPattern != fixedFilterPattern)
      {
        fixedFilterPattern = 2;
        break;
      }
    }
    alfSliceParam->fixedFilterPattern = fixedFilterPattern;
  }
#if ALF_CONFORMANCE_CHECK
  *filter_conformance_flag = 0;
  conformaceCheck(alfSliceParam, filter_conformance_flag);
#endif
  uiCoeffBits += getNonFilterCoeffRate(alfSliceParam);
#if ALF_CONFORMANCE_CHECK
  if (*filter_conformance_flag)
      distReturn = MAX_COST;
#endif
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate(AlfSliceParam* alfSliceParam)
{
#if M53608_ALF_3
    int len = 1   // filter_type
        + 1   // alf_coefficients_delta_flag
        + lengthTruncatedUnary(0, 3)    // chroma_idc = 0, it is signalled when ALF is enabled for luma
        + 5;   //numLumaFilters
#else
  int len = 1   // filter_type
    + 1   // alf_coefficients_delta_flag
    + lengthTruncatedUnary(0, 3)    // chroma_idc = 0, it is signalled when ALF is enabled for luma
    + getTBlength(alfSliceParam->numLumaFilters - 1, MAX_NUM_ALF_CLASSES);   //numLumaFilters
#endif
#if !M53608_ALF_6                                                                          //add bits of fixed filter
  char codetab_pred[3] = { 1, 0, 2 };
#endif
  const int iNumFixedFilterPerClass = ALF_FIXED_FILTER_NUM;
  if (iNumFixedFilterPerClass > 0)
  {
#if M53608_ALF_6  
#if ETM70_GOLOMB_FIX
      len += lengthGolomb(alfSliceParam->fixedFilterPattern, 0, FALSE);
#else
      len += lengthGolomb(alfSliceParam->fixedFilterPattern, 0);
#endif
#else
    len += lengthGolomb(codetab_pred[alfSliceParam->fixedFilterPattern], 0);
#endif
    if (alfSliceParam->fixedFilterPattern == 2)
    {
      len += MAX_NUM_ALF_CLASSES;  //"fixed_filter_flag" for each class
    }
    if (alfSliceParam->fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
    {
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
      {
#if M53608_ALF_7
          if (alfSliceParam->fixedFilterUsageFlag[classIdx] > 0)
#else
          if (alfSliceParam->fixedFilterIdx[classIdx] > 0)
#endif
        {
#if M53608_ALF_3
              len += evc_tbl_log2[iNumFixedFilterPerClass - 1] + 1;
#else
#if M53608_ALF_7
              len += getTBlength(alfSliceParam->fixedFilterIdx[classIdx], iNumFixedFilterPerClass);
#else
              len += getTBlength(alfSliceParam->fixedFilterIdx[classIdx] - 1, iNumFixedFilterPerClass);
#endif
#endif
        }
      }
    }
  }

  if (alfSliceParam->numLumaFilters > 1)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
#if M53608_ALF_3
        len += evc_tbl_log2[alfSliceParam->numLumaFilters - 1] + 1;
#else
        len += getTBlength((int)alfSliceParam->filterCoeffDeltaIdx[i], alfSliceParam->numLumaFilters);  //filter_coeff_delta[i]
#endif
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
#if ETM70_GOLOMB_FIX
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k, TRUE);
#else
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
#endif
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
#if ETM70_GOLOMB_FIX
            len += lengthGolomb(abs(pDiffQFilterCoeffIntPP[ind][i]), m_kMinTab[alfShape.golombIdx[i]], TRUE); // alf_coeff_luma_delta[i][j]
#else
            len += lengthGolomb(abs(pDiffQFilterCoeffIntPP[ind][i]), m_kMinTab[alfShape.golombIdx[i]]); // alf_coeff_luma_delta[i][j]
#endif
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
#if ETM70_GOLOMB_FIX
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k, TRUE);
#else
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
#endif
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
#if ETM70_GOLOMB_FIX
          bitCnt += lengthGolomb(abs(FilterCoeff[ind][i]), kMinTab[alfShape.golombIdx[i]], TRUE);
#else
          bitCnt += lengthGolomb(abs(FilterCoeff[ind][i]), kMinTab[alfShape.golombIdx[i]]);
#endif
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
#if ETM70_GOLOMB_FIX
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k, TRUE);
#else
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb(coeffVal, k);
#endif
      }
    }
  }

  getGolombKMin(alfShape, numFilters, m_kMinTab, m_bitsCoeffScan);

  for (int ind = 0; ind < numFilters; ++ind)
  {
    bitsVarBin[ind] = 0;
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
#if ETM70_GOLOMB_FIX
        bitsVarBin[ind] += lengthGolomb(abs(m_filterCoeffSet[ind][i]), m_kMinTab[alfShape.golombIdx[i]], TRUE);
#else
        bitsVarBin[ind] += lengthGolomb(abs(m_filterCoeffSet[ind][i]), m_kMinTab[alfShape.golombIdx[i]]);
#endif
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

#if ETM70_GOLOMB_FIX
int EncAdaptiveLoopFilter::lengthGolomb(int coeffVal, int k, BOOL signed_coeff)
{
    int numBins = 0;
    unsigned int symbol = abs(coeffVal);
    while (symbol >= (unsigned int)(1 << k))
    {
        numBins++;
        symbol -= 1 << k;
        k++;
    }
    numBins += (k + 1);
    if (signed_coeff && coeffVal != 0)
    {
        numBins++;
    }
    return numBins;
}
#else
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
#endif
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

void EncAdaptiveLoopFilter::findBestFixedFilter(AlfSliceParam* alfSliceParam, AlfCovariance* cov)
{
  double factor = 1 << (m_NUM_BITS - 1);
  for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
  {
    double errorMin = cov[classIdx].pixAcc;
    alfSliceParam->fixedFilterIdx[classIdx] = 0;
    for (int filterIdx = 0; filterIdx < ALF_FIXED_FILTER_NUM; filterIdx++)
    {
      int fixedFilterIdx = m_classToFilterMapping[classIdx][filterIdx];
#if M53608_ALF_8
      double errorFilter = cov[classIdx].pixAcc + calcErrorForCoeffs(cov[classIdx].E, cov[classIdx].y, m_fixedFilterCoeff[fixedFilterIdx], (alfSliceParam->lumaFilterType ? 13 : 7), m_NUM_BITS);
#else
      double errorFilter = cov[classIdx].pixAcc + calcErrorForCoeffs(cov[classIdx].E, cov[classIdx].y, m_fixedFilterCoeff[fixedFilterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
#endif
      if (errorFilter < errorMin)
      {
        errorMin = errorFilter;
#if M53608_ALF_7
        alfSliceParam->fixedFilterIdx[classIdx] = filterIdx;
        alfSliceParam->fixedFilterUsageFlag[classIdx] = 1;
#else
        alfSliceParam->fixedFilterIdx[classIdx] = filterIdx + 1;
#endif
      }
    }
    //update stat
    int finalFilterIdx = alfSliceParam->fixedFilterIdx[classIdx];
#if M53608_ALF_7
    u8 finalFilterUsageFlag = alfSliceParam->fixedFilterUsageFlag[classIdx];
    if (finalFilterUsageFlag > 0)
#else
    if (finalFilterIdx > 0)
#endif
    {
#if M53608_ALF_7
        int fixedFilterIdx = m_classToFilterMapping[classIdx][finalFilterIdx];
#else
        int fixedFilterIdx = m_classToFilterMapping[classIdx][finalFilterIdx - 1];
#endif
      cov[classIdx].pixAcc = errorMin;
      //update y
#if M53608_ALF_8
      for (int i = 0; i < (alfSliceParam->lumaFilterType ? 13 : 7); i++)
#else
      for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
#endif
      {
        double sum = 0;
#if M53608_ALF_8
        for (int j = 0; j < (alfSliceParam->lumaFilterType ? 13 : 7); j++)
#else
        for (int j = 0; j < MAX_NUM_ALF_LUMA_COEFF; j++)
#endif
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
#if M53608_ALF_9
void EncAdaptiveLoopFilter::getFrameStats(ComponentID compID, int iShapeIdx)
#else
void EncAdaptiveLoopFilter::getFrameStats(ChannelType channel, int iShapeIdx)
#endif
{
#if M53608_ALF_9
    ChannelType channel = toChannelType(compID);
#endif
  int numClasses = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  for (int i = 0; i < numClasses; i++)
  {
#if M53608_ALF_9
      m_alfCovarianceFrame[compID][iShapeIdx][i].reset();
#else
      m_alfCovarianceFrame[channel][iShapeIdx][i].reset();
#endif
  }
#if M53608_ALF_9
  if (compID == COMPONENT_Y)
  {
      getFrameStat(m_alfCovarianceFrame[COMPONENT_Y][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], numClasses);
  }
  else if (compID == COMPONENT_Cb)
  {
      getFrameStat(m_alfCovarianceFrame[COMPONENT_Cb][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], numClasses);
  }
  else if (compID == COMPONENT_Cr)
  {
      getFrameStat(m_alfCovarianceFrame[COMPONENT_Cr][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], numClasses);
  }
#else
  if (isLuma(channel))
  {
      getFrameStat(m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], numClasses);
  }
  else
  {
      getFrameStat(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], numClasses);
      getFrameStat(m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], numClasses);
  }
#endif
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

void EncAdaptiveLoopFilter::deriveStatsForFiltering(YUV * orgYuv, YUV * recYuv
#if BD_CF_EXT
                                                    , int idc
#endif
)
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
#if M53608_ALF_9
  const int numberOfChannels = getNumberValidComponents(m_chromaFormat);
#else
  const int numberOfChannels = getNumberValidChannels(m_chromaFormat);
#endif
  for (int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++)
  {
#if M53608_ALF_9
      const ComponentID channelID = ComponentID(channelIdx);

      const int numClasses = isLuma(toChannelType(channelID)) ? MAX_NUM_ALF_CLASSES : 1;
      const int size = isLuma(toChannelType(channelID)) ? 2 : 1;
#else
      const ChannelType channelID = ChannelType(channelIdx);

      const int numClasses = isLuma(channelID) ? MAX_NUM_ALF_CLASSES : 1;
      const int size = isLuma(channelID) ? 2 : 1;
#endif

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
#if BD_CF_EXT
            width2 = width >> (GET_CHROMA_W_SHIFT(idc));
            height2 = height >> (GET_CHROMA_H_SHIFT(idc));
            xPos2 = xPos >> (GET_CHROMA_W_SHIFT(idc));
            yPos2 = yPos >> (GET_CHROMA_H_SHIFT(idc));
#else
            width2 = width >> 1;
            height2 = height >> 1;
            xPos2 = xPos >> 1;
            yPos2 = yPos >> 1;
#endif
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
            getBlkStats((int)chType, m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, xPos2, yPos2, width2, height2
#if BD_CF_EXT
                        , idc
#endif
            );

          const int numClasses = compID == COMPONENT_Y ? MAX_NUM_ALF_CLASSES : 1;

          for (int classIdx = 0; classIdx < numClasses; classIdx++)
          {
#if M53608_ALF_9
              m_alfCovarianceFrame[compIdx][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#else
              m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#endif
          }
        }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStats(int ch, AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org0, const int orgStride, Pel* rec0, const int recStride,
                                        int x, int y, int width, int height
#if BD_CF_EXT
                                        , int idc
#endif
)
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
#if BD_CF_EXT
        int x2 = ch ? (x << (GET_CHROMA_W_SHIFT(idc))) : x;
        int y2 = ch ? (y << (GET_CHROMA_H_SHIFT(idc))) : y;
#else
        int x2 = ch ? (x << 1) : x;
        int y2 = ch ? (y << 1) : y;
#endif
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
#if M53608_ALF_9
void EncAdaptiveLoopFilter::setEnableFlag(AlfSliceParam* alfSlicePara, ComponentID channel, bool val)
{
  if (channel == COMPONENT_Y)
  {
    alfSlicePara->enabledFlag[COMPONENT_Y] = val;
  }
  else if (channel == COMPONENT_Cb)
  {
    alfSlicePara->enabledFlag[COMPONENT_Cb] = val;
  }
  else if (channel == COMPONENT_Cr)
  {
    alfSlicePara->enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag(AlfSliceParam* alfSlicePara, ComponentID channel, uint8_t** ctuFlags)
{
  alfSlicePara->enabledFlag[channel] = false;
  for (int i = 0; i < m_numCTUsInPic; i++)
  {
    if (ctuFlags[channel][i])
    {
      alfSlicePara->enabledFlag[channel] = true;
      break;
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ComponentID channel)
{
  if (channel == COMPONENT_Y)
  {
    memcpy(ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof(uint8_t) * m_numCTUsInPic);
  }
  else if (channel == COMPONENT_Cb)
  {
    memcpy(ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof(uint8_t) * m_numCTUsInPic);
  }
  else if (channel == COMPONENT_Cr)
  {
    memcpy(ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof(uint8_t) * m_numCTUsInPic);
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag(uint8_t** ctuFlags, ComponentID channel, uint8_t val)
{
  if (channel == COMPONENT_Y)
  {
    memset(ctuFlags[COMPONENT_Y], val, sizeof(uint8_t) * m_numCTUsInPic);
  }
  else if (channel == COMPONENT_Cb)
  {
    memset(ctuFlags[COMPONENT_Cb], val, sizeof(uint8_t) * m_numCTUsInPic);
  }
  else if (channel == COMPONENT_Cr)
  {
    memset(ctuFlags[COMPONENT_Cr], val, sizeof(uint8_t) * m_numCTUsInPic);
  }
}
#else
void EncAdaptiveLoopFilter::setEnableFlag(AlfSliceParam* alfSlicePara, ChannelType channel, bool val)
{
  if (channel == CHANNEL_TYPE_LUMA)
  {
    alfSlicePara->enabledFlag[COMPONENT_Y] = val;
  }
  else
  {
    alfSlicePara->enabledFlag[COMPONENT_Cb] = alfSlicePara->enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag(AlfSliceParam* alfSlicePara, ChannelType channel, uint8_t** ctuFlags)
{
  const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;
  for (int compId = compIDFirst; compId <= compIDLast; compId++)
  {
    alfSlicePara->enabledFlag[compId] = false;
    for (int i = 0; i < m_numCTUsInPic; i++)
    {
      if (ctuFlags[compId][i])
      {
        alfSlicePara->enabledFlag[compId] = true;
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
