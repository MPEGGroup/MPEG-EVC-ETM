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

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#ifdef __cplusplus
extern "C" {
#endif

#include "wrapper.h"

#include "evc_def.h"
#include "evcd_def.h"

#define CHECK(a,b) assert((!(a)) && (b));
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0

#define ALF_TEMPORAL_WITH_LINE_BUFFER                     6 // temporal buffer size

#pragma warning(disable:4018)
#pragma warning(disable:4800)

static __inline int Clip3 (const int minVal, const int maxVal, const int a) { return min(max(minVal, a), maxVal); }  ///< general min/max clip

typedef u8 AlfClassifier;
typedef pel Pel;

typedef int PosType;
typedef u32 SizeType;

#define m_NUM_BITS 10
#define m_CLASSIFICATION_BLK_SIZE 32  //non-normative, local buffer size
#define m_fixedFilterNum 64

#if EVC_TILE_SUPPORT
void copy_and_extend_tile(pel* tmpYuv, const int s, const pel* recYuv, const int s2, const int w, const int h, const int m);
#endif
void copy_and_extend( pel* tmpYuv, const int s, const pel* recYuv, const int s2, const int w, const int h, const int m );

typedef struct Area
{
    int x;
    int y;
    int width;
    int height;
} Area;

/// chroma formats (according to semantics of chroma_format_idc)
typedef enum _ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
} ChromaFormat;

typedef enum _ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
} ChannelType;

typedef enum _ComponentID
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT   = 3,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
} ComponentID;

__inline ChannelType toChannelType             (const ComponentID id)                         { return (id==COMPONENT_Y)? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA; }
__inline BOOL        isLuma                    (const ChannelType id)                         { return (id==CHANNEL_TYPE_LUMA);                                    }
__inline BOOL        isChroma                  (const ChannelType id)                         { return (id!=CHANNEL_TYPE_LUMA);   }
__inline u32        getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMPONENT;                  }
__inline u32        getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CHANNEL_TYPE;               }

typedef struct ClpRng
{
  int min;
  int max;
  int bd;
  int n;
} ClpRng;

typedef struct ClpRngs
{
  ClpRng comp[MAX_NUM_COMPONENT]; ///< the bit depth as indicated in the SPS
  BOOL used;
  BOOL chroma;
} ClpRngs;

static __inline int ClipPel (const int a, const ClpRng clpRng)         { return min(max(clpRng.min, a) , clpRng.max); }  ///< clip reconstruction

typedef struct CodingStructure
{
    void * pCtx;
    EVC_PIC * pPic;

    int tempStride; //to pass strides easily
    int picStride;
} CodingStructure;

#if 1
#define MAX_NUM_ALF_CLASSES             25
#define MAX_NUM_ALF_LUMA_COEFF          13
#define MAX_NUM_ALF_CHROMA_COEFF        7
#define MAX_ALF_FILTER_LENGTH           7
#define MAX_NUM_ALF_COEFF               (MAX_ALF_FILTER_LENGTH * MAX_ALF_FILTER_LENGTH / 2 + 1)
#define ALF_FIXED_FILTER_NUM                              16

static const int pattern5[25] =
{
          0,
          1,  2,  3,
          4,  5,  6,  5,  4,
          3,  2,  1,
          0
};

static const int pattern7[25] =
{
      0,
      1,  2,  3,
      4,  5,  6,  7,  8,
      9, 10, 11, 12, 11, 10, 9,
      8,  7,  6,  5,  4,
      3,  2,  1,
      0
};

static const int weights5[14] =
{
      2,
      2, 2, 2,
      2, 2, 1, 1
};

static const int weights7[14] =
{
  2,
  2,  2,  2,
  2,  2,  2,  2,  2,
  2,  2,  2,  1,  1
};

static const int golombIdx5[14] =
{
  0,
  0, 1, 0,
  0, 1
};

static const int golombIdx7[14] =
{
  0,
  0, 1, 0,
  0, 1, 2, 1, 0,
  0, 1, 2
};

static const int patternToLargeFilter5[13] =
{
  0,
  0, 1, 0,
  0, 2, 3, 4, 0,
  0, 5, 6, 7
};

static const int patternToLargeFilter7[13] =
{
  1,
  2, 3, 4,
  5, 6, 7, 8, 9,
  10,11,12,13
};

// The structure below must be aligned to identical structure in evc_def.h!
typedef struct AlfFilterShape
{
  int filterType;
  int filterLength;
  int numCoeff;      
  int filterSize;
  int pattern[25];
  int weights[14];
  int golombIdx[14];
  int patternToLargeFilter[13];
} AlfFilterShape;

extern void init_AlfFilterShape(void* _th, int size);

struct AlfSliceParam
{
  BOOL                         isCtbAlfOn;
  u8                           *alfCtuEnableFlag;
  BOOL                         enabledFlag[MAX_NUM_COMPONENT];                          // alf_slice_enable_flag, alf_chroma_idc
  AlfFilterType                lumaFilterType;                                          // filter_type_flag
  BOOL                         chromaCtbPresentFlag;                                    // alf_chroma_ctb_present_flag
  short                        lumaCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
  short                        chromaCoeff[MAX_NUM_ALF_CHROMA_COEFF];                   // alf_coeff_chroma[i]
  short                        filterCoeffDeltaIdx[MAX_NUM_ALF_CLASSES];                // filter_coeff_delta[i]
  BOOL                         filterCoeffFlag[MAX_NUM_ALF_CLASSES];                    // filter_coefficient_flag[i]
  int                          numLumaFilters;                                          // number_of_filters_minus1 + 1
  BOOL                         coeffDeltaFlag;                                          // alf_coefficients_delta_flag
  BOOL                         coeffDeltaPredModeFlag;                                  // coeff_delta_pred_mode_flag
  AlfFilterShape               (*filterShapes)[2];

  int                          fixedFilterPattern;                                     //0: no pred from pre-defined filters; 1: all are predicted but could be different values; 2: some predicted and some not
                                                                                       //when ALF_LOWDELAY is 1, fixedFilterPattern 0: all are predected, fixedFilterPattern 1: some predicted and some not
  int                          fixedFilterIdx[MAX_NUM_ALF_CLASSES];
#if M53608_ALF_7
  u8                           fixedFilterUsageFlag[MAX_NUM_ALF_CLASSES];
#endif
  int                          tLayer;
  BOOL                         temporalAlfFlag;         //indicate whether reuse previous ALF coefficients
  int                          prevIdx;                 //index of the reused ALF coefficients
  int                          prevIdxComp[MAX_NUM_CHANNEL_TYPE];
  BOOL resetALFBufferFlag;
  BOOL store2ALFBufferFlag;

  // encoder side variables
  u32 m_filterPoc;  // store POC value for which filter was produced
  u32 m_minIdrPoc;  // Minimal of 2 IDR POC available for current coded nalu  (to identify availability of this filter for temp prediction)
  u32 m_maxIdrPoc;  // Max of 2 IDR POC available for current coded nalu  (to identify availability of this filter for temp prediction)
#if M53608_ALF_14
  BOOL chromaFilterPresent;
#endif
};
#endif

enum Direction
{
  HOR,
  VER,
  DIAG0,
  DIAG1,
  NUM_DIRECTIONS
};

extern pel* m_tempBuf, *m_tempBuf1, *m_tempBuf2;
extern int  m_picWidth;
extern int  m_picHeight;
extern int  m_maxCUWidth;
extern int  m_maxCUHeight;
extern int  m_maxCUDepth;
extern int  m_numCTUsInWidth;
extern int  m_numCTUsInHeight;
extern int  m_numCTUsInPic;
extern AlfClassifier** m_classifier;
extern ChromaFormat    m_chromaFormat;

extern BOOL m_store2ALFBufferFlag;
extern BOOL m_resetALFBufferFlag;

extern u32 m_firstIdrPoc;
extern u32 m_lastIdrPoc;
extern u32 m_currentPoc;
extern u32  m_currentTempLayer;
// variables to handle buffer reset for IDR
extern int m_alf_present_idr;
extern int m_alf_idx_idr;
extern u32 m_i_period;

extern int m_lastRasPoc;
extern BOOL m_pendingRasInit;
extern u8* m_ctuEnableFlag[MAX_NUM_COMPONENT];
extern ClpRngs m_clpRngs;

extern AlfFilterShape m_filterShapes[MAX_NUM_CHANNEL_TYPE][2];

extern const int m_fixedFilterCoeff[m_fixedFilterNum][13];
extern const int m_classToFilterMapping[MAX_NUM_ALF_CLASSES][ALF_FIXED_FILTER_NUM];

extern short m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
extern int   m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];

extern u8 m_acAlfLineBufferCurrentSize;

extern u8 m_alfIndxInScanOrder[APS_MAX_NUM];
extern AlfSliceParam m_acAlfLineBuffer[APS_MAX_NUM];

extern AlfSliceParam m_IRAPFilter;

extern void init_AlfFilterShape(void* _th, int size);
extern int  getMaxGolombIdx( AlfFilterType filterType );
extern void reconstructCoeff( AlfSliceParam* alfSliceParam, ChannelType channel, const BOOL bRdo, const BOOL bRedo );
extern void deriveClassification( AlfClassifier** classifier, const pel * srcLuma, const int srcLumaStride, const Area * blk );

extern void copyAlfParam(AlfSliceParam* dst, AlfSliceParam* src);
extern void copyAlfParamChroma(AlfSliceParam* dst, AlfSliceParam* src);
void store_alf_paramline_from_aps(AlfSliceParam* pAlfParam, u8 idx);
void load_alf_paramline_from_aps_buffer(AlfSliceParam* pAlfParam, u8 idx);
void load_alf_paramline_from_aps_buffer2(AlfSliceParam* pAlfParam, u8 idxY, u8 idxUV
#if M53608_ALF_14
    , u8 alfChromaIdc
#endif
);
extern u8 m_nextFreeAlfIdxInBuffer;
extern void resetAlfParam(AlfSliceParam* dst);
extern void resetIdrIndexListBufferAPS();
extern int  getProtectIdxFromList(int idx);
extern void storeEncALFParamLineAPS(AlfSliceParam* pAlfParam, unsigned tLayer);
void AdaptiveLoopFilter_create( const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth);
void AdaptiveLoopFilter_destroy();
void deriveClassificationBlk( AlfClassifier** classifier, const pel * srcLuma, const int srcStride, const Area * blk, const int shift );
void filterBlk_7( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area* blk, const ComponentID compId, short* filterSet, const ClpRng* clpRng );
void filterBlk_5( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area* blk, const ComponentID compId, short* filterSet, const ClpRng* clpRng );

void init_AdaptiveLoopFilter(AdaptiveLoopFilter* p);

struct AdaptiveLoopFilter
{
  void( *m_deriveClassificationBlk )( AlfClassifier** classifier, const pel * srcLuma, const int srcStride, const Area * blk, const int shift );
  void( *m_filter5x5Blk )( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area* blk, const ComponentID compId, short* filterSet, const ClpRng* clpRng );
  void( *m_filter7x7Blk )( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area* blk, const ComponentID compId, short* filterSet, const ClpRng* clpRng );
};

#ifdef __cplusplus
}
#endif

#endif
