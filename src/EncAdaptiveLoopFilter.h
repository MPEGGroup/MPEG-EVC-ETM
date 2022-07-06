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
        evc_mset(y, 0, sizeof(*y) * numCoeff);
        for (int i = 0; i < numCoeff; i++)
        {
            evc_mset(E[i], 0, sizeof(*E[i]) * numCoeff);
        }
    }

    const AlfCovariance& operator=(const AlfCovariance& src)
    {
        for (int i = 0; i < numCoeff; i++)
        {
            evc_mcpy(E[i], src.E[i], sizeof(*E[i]) * numCoeff);
        }
        evc_mcpy(y, src.y, sizeof(*y) * numCoeff);
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
    AdaptiveLoopFilter     m_AdaptiveLoopFilter;

private:
    AlfCovariance***       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][classIdx]
    AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_COMPONENT + 1];   // [CHANNEL][shapeIdx][classIdx]
    uint8_t*               m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];
    uint8_t*               m_ctuEnableFlagTmpLuma;
    //for RDO
    AlfSliceParam          m_alfSliceParamTemp;
    AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 1];

    EVCE_CORE *            m_core;

    double                 m_lambda[MAX_NUM_COMPONENT];
    const double           FracBitsScale = 1.0 / double(1 << SCALE_BITS);
    double                 m_costAlfEncoder[MAX_NUM_COMPONENT];

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
    void create(const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, int idc);
    void destroy();
    static int lengthGolomb(int coeffVal, int k, BOOL signed_coeff);
    static int getGolombKMin(AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB]);

private:
    void   alfEncoder(CodingStructure& cs, AlfSliceParam* alfSliceParam, const ChannelType channel);
    void   copyAlfSliceParam(AlfSliceParam* alfSliceParamDst, AlfSliceParam* alfSliceParamSrc, ChannelType channel);
    double mergeFiltersAndCost(AlfSliceParam* alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits, u8* filter_conformance_flag);
    void   conformaceCheck(AlfSliceParam* alfSliceParam, u8* filter_conformance_flag);
    void   getFrameStats(ComponentID channel, int iShapeIdx);
    void   getFrameStat(AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses);
    void   getBlkStats(int ch, AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const int x, const int y, const int width, const int height, int idc);

    void   deriveStatsForFiltering(YUV * orgYuv, YUV * recYuv, int idc);

    void   calcCovariance(int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx);
    void   mergeClasses(AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]);
    void   alfReconstructor(CodingStructure& cs, AlfSliceParam* alfSliceParam, const pel * orgUnitBuf, const int oStride, pel * recExtBuf, const int recStride, const ComponentID compID, int tile_idx, int col_bd2);
    void   alfTemporalEncoderAPSComponent(CodingStructure& cs, AlfSliceParam* alfSliceParam);
    void   findBestFixedFilter(AlfSliceParam* alfSliceParam, AlfCovariance* cov);
    void   xDeriveCovFromLgrTapFilter(AlfCovariance& covLgr, AlfCovariance& covSml, int* patternSml, AlfFilterType lumafiltertype);
    double calculateError(AlfCovariance& cov);
    double calcErrorForCoeffs(double **E, double *y, const int *coeff, const int numCoeff, const int bitDepth);
    double getFilterCoeffAndCost(CodingStructure& cs, double distUnfilter, ComponentID channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, u8* filter_conformance_flag);
    void   getFilterCoeffAndCost_Chroma(CodingStructure& cs, double distUnfilter, ComponentID channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, double* filter_cost);
    double deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2]);
    int    deriveFilterCoefficientsPredictionMode(AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode);
    double deriveCoeffQuant(int *filterCoeffQuant, double **E, double *y, const int numCoeff, int* weights, const int bitDepth, const bool bChroma = false);
    double deriveCtbAlfEnableFlags(CodingStructure& cs, const int iShapeIdx, ComponentID channel, const int numClasses, const int numCoeff, double& distUnfilter, bool recCoeff);
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
    void setEnableFlag(AlfSliceParam* alfSlicePara, ComponentID channel, bool val);
    void setEnableFlag(AlfSliceParam* alfSlicePara, ComponentID channel, uint8_t** ctuFlags);
    void setCtuEnableFlag(uint8_t** ctuFlags, ComponentID channel, uint8_t val);
    void copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ComponentID channel);
    void tile_boundary_check(int* availableL, int* availableR, int* availableT, int* availableB, const int width, const int height, int xPos, int yPos, int x_l, int x_r, int y_l, int y_r);
};
#endif
