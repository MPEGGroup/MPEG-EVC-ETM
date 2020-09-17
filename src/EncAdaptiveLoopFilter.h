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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include <float.h>
#include <math.h>
#include <vector>
#include <stdint.h>

#include "enc_alf_wrapper.h"

#include "evc_alf.h"

#define m_MAX_SCAN_VAL 11
#define m_MAX_EXP_GOLOMB 16

#pragma warning(disable:4099)    //@Chernyak: why?
#pragma warning(disable:4800)    //@Chernyak: why?

struct AlfCovariance
{
  int numCoeff;
  double *y;
  double **E;
  double pixAcc;

  AlfCovariance() {}
  ~AlfCovariance() {}

  void create(int size)
  {
    numCoeff = size;

    y = new double[numCoeff];
    E = new double*[numCoeff];

    for (int i = 0; i < numCoeff; i++)
    {
      E[i] = new double[numCoeff];
    }
  }

  void destroy()
  {
    for (int i = 0; i < numCoeff; i++)
    {
      delete[] E[i];
      E[i] = nullptr;
    }

    delete[] E;
    E = nullptr;

    delete[] y;
    y = nullptr;
  }

  void reset()
  {
    pixAcc = 0;
    memset(y, 0, sizeof(*y) * numCoeff);
    for (int i = 0; i < numCoeff; i++)
    {
      memset(E[i], 0, sizeof(*E[i]) * numCoeff);
    }
  }

  const AlfCovariance& operator=(const AlfCovariance& src)
  {
    for (int i = 0; i < numCoeff; i++)
    {
      memcpy(E[i], src.E[i], sizeof(*E[i]) * numCoeff);
    }
    memcpy(y, src.y, sizeof(*y) * numCoeff);
    pixAcc = src.pixAcc;

    return *this;
  }

  void add(const AlfCovariance& lhs, const AlfCovariance& rhs)
  {
    for (int j = 0; j < numCoeff; j++)
    {
      for (int i = 0; i < numCoeff; i++)
      {
        E[j][i] = lhs.E[j][i] + rhs.E[j][i];
      }
      y[j] = lhs.y[j] + rhs.y[j];
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }
#if M53608_ALF_9
  const AlfCovariance& operator+ (const AlfCovariance& src)
  {
    for (int j = 0; j < numCoeff; j++)
    {
      for (int i = 0; i < numCoeff; i++)
      {
        E[j][i] += src.E[j][i];
      }
      y[j] += src.y[j];
    }
    pixAcc += src.pixAcc;

    return *this;
  }
#endif
  const AlfCovariance& operator+= (const AlfCovariance& src)
  {
    for (int j = 0; j < numCoeff; j++)
    {
      for (int i = 0; i < numCoeff; i++)
      {
        E[j][i] += src.E[j][i];
      }
      y[j] += src.y[j];
    }
    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance& operator-= (const AlfCovariance& src)
  {
    for (int j = 0; j < numCoeff; j++)
    {
      for (int i = 0; i < numCoeff; i++)
      {
        E[j][i] -= src.E[j][i];
      }
      y[j] -= src.y[j];
    }
    pixAcc -= src.pixAcc;

    return *this;
  }
};

//for 4:2:0 only
typedef struct _YUV {
    pel* yuv[3];
    int s[3];
} YUV;

class EncAdaptiveLoopFilter
{
public:
  AdaptiveLoopFilter    m_AdaptiveLoopFilter;

private:
  AlfCovariance***       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][classIdx]
#if M53608_ALF_9
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_COMPONENT + 1];   // [CHANNEL][shapeIdx][classIdx]
#else
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_CHANNEL_TYPE];   // [CHANNEL][shapeIdx][classIdx]
#endif
  uint8_t*               m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];
  uint8_t*               m_ctuEnableFlagTmpLuma;
  //for RDO
  AlfSliceParam      m_alfSliceParamTemp;
  AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 1];

  EVCE_CORE *           m_core;
/*
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
*/

  double                 m_lambda[MAX_NUM_COMPONENT];
  const double           FracBitsScale = 1.0 / double(1 << SCALE_BITS);
#if M53608_ALF_9
  double                 m_costAlfEncoder[MAX_NUM_COMPONENT];
#else
  double                 m_costAlfEncoder[MAX_NUM_CHANNEL_TYPE];
#endif

  int*                   m_filterCoeffQuant;
  int**                  m_filterCoeffSet;
  int**                  m_diffFilterCoeff;
  int                    m_kMinTab[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
public:
  EncAdaptiveLoopFilter();
  virtual ~EncAdaptiveLoopFilter() {}

  void Enc_ALFProcess(CodingStructure& cs, const double *lambdas, AlfSliceParam* alfSliceParam);
  void initCABACEstimator(EVCE_CORE * core);
  void create(const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth );
  void destroy();
#if ETM70_GOLOMB_FIX
  static int lengthGolomb(int coeffVal, int k, BOOL signed_coeff);
#else
  static int lengthGolomb(int coeffVal, int k);
#endif
  static int getGolombKMin(AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB]);

private:
  void   alfEncoder(CodingStructure& cs, AlfSliceParam* alfSliceParam, const ChannelType channel);
  void   copyAlfSliceParam(AlfSliceParam* alfSliceParamDst, AlfSliceParam* alfSliceParamSrc, ChannelType channel);
  double mergeFiltersAndCost(AlfSliceParam* alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits
#if ALF_CONFORMANCE_CHECK
      , u8* filter_conformance_flag
#endif
  );
#if ALF_CONFORMANCE_CHECK
  void conformaceCheck(AlfSliceParam* alfSliceParam, u8* filter_conformance_flag);
#endif
#if M53608_ALF_9
  void   getFrameStats(ComponentID channel, int iShapeIdx);
#else
  void   getFrameStats(ChannelType channel, int iShapeIdx);
#endif 
  void   getFrameStat(AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses);
  void   getBlkStats(int ch, AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const int x, const int y, const int width, const int height);

  void   deriveStatsForFiltering(YUV * orgYuv, YUV * recYuv);

  void   calcCovariance(int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx);
  void   mergeClasses(AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]);
  void   alfReconstructor(CodingStructure& cs, AlfSliceParam* alfSliceParam, const pel * orgUnitBuf, const int oStride, pel * recExtBuf, const int recStride, const ComponentID compID, int tile_idx, int col_bd2);
#if !M53608_ALF_9
  void   alfTemporalEncoderAPS(CodingStructure& cs, AlfSliceParam* alfSliceParam);
#endif
  void   alfTemporalEncoderAPSComponent(CodingStructure& cs, AlfSliceParam* alfSliceParam);
  void   findBestFixedFilter(AlfSliceParam* alfSliceParam, AlfCovariance* cov);
  void   xDeriveCovFromLgrTapFilter(AlfCovariance& covLgr, AlfCovariance& covSml, int* patternSml
#if M53608_ALF_8
         , AlfFilterType lumafiltertype
#endif
  );

  double calculateError(AlfCovariance& cov);

  double calcErrorForCoeffs(double **E, double *y,
  const int *coeff, const int numCoeff, const int bitDepth);

#if M53608_ALF_9
  double getFilterCoeffAndCost(CodingStructure& cs, double distUnfilter, ComponentID channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits
#if ALF_CONFORMANCE_CHECK
      , u8* filter_conformance_flag
#endif
  );
#else
  double getFilterCoeffAndCost(CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits);
#endif
#if M53608_ALF_9
  void getFilterCoeffAndCost_Chroma(CodingStructure& cs, double distUnfilter, ComponentID channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, double* filter_cost);
#endif
  double deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2]);
  int    deriveFilterCoefficientsPredictionMode(AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode);
  double deriveCoeffQuant(int *filterCoeffQuant, double **E, double *y, const int numCoeff, int* weights, const int bitDepth, const bool bChroma = false);
#if M53608_ALF_9
  double deriveCtbAlfEnableFlags(CodingStructure& cs, const int iShapeIdx, ComponentID channel, const int numClasses, const int numCoeff, double& distUnfilter, bool recCoeff);
#else
  double deriveCtbAlfEnableFlags(CodingStructure& cs, const int iShapeIdx, ChannelType channel, const int numClasses, const int numCoeff, double& distUnfilter, bool recCoeff);
#endif
  void   roundFiltCoeff(int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor);

  double getDistCoeffForce0(bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters);
  int    lengthTruncatedUnary(int symbol, int maxSymbol);
  int    lengthUvlc(int uiCode);
  int    getNonFilterCoeffRate(AlfSliceParam* alfSliceParam);
  int    getTBlength(int uiSymbol, const int uiMaxSymbol);

  int    getCostFilterCoeffForce0(AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins);
  int    getCostFilterCoeff(AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters);
  int    lengthFilterCoeffs(AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab);
  double getDistForce0(AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins);
  int    getCoeffRate(AlfSliceParam* alfSliceParam, bool isChroma);

  double getUnfilteredDistortion(AlfCovariance* cov, ChannelType channel);
  double getUnfilteredDistortion(AlfCovariance* cov, const int numClasses);
  double getFilteredDistortion(AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff);

  // Cholesky decomposition
  int  gnsSolveByChol(double **LHS, double *rhs, double *x, int numEq);
  void gnsBacksubstitution(double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A);
  void gnsTransposeBacksubstitution(double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order);
  int  gnsCholeskyDec(double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq);

#if M53608_ALF_9
  void setEnableFlag(AlfSliceParam* alfSlicePara, ComponentID channel, bool val);
  void setEnableFlag(AlfSliceParam* alfSlicePara, ComponentID channel, uint8_t** ctuFlags);
  void setCtuEnableFlag(uint8_t** ctuFlags, ComponentID channel, uint8_t val);
  void copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ComponentID channel);
#else
  void setEnableFlag(AlfSliceParam* alfSlicePara, ChannelType channel, bool val);
  void setEnableFlag(AlfSliceParam* alfSlicePara, ChannelType channel, uint8_t** ctuFlags);
  void setCtuEnableFlag(uint8_t** ctuFlags, ChannelType channel, uint8_t val);
  void copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel);
#endif
  void tile_boundary_check(int* availableL, int* availableR, int* availableT, int* availableB, const int width, const int height, int xPos, int yPos, int x_l, int x_r, int y_l, int y_r);
};
#endif
