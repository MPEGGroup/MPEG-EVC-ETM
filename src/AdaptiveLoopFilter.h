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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "wrapper.h"

#include "evc_def.h"
#include "evcd_def.h"

#ifdef min
#undef min

#ifdef max
#undef max

#include <stdint.h>
#include <vector>
#include <algorithm>
#include <cstring>


#if ALF

#define CHECK //disabled tests
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0

#define ALF_TEMPORAL_WITH_LINE_BUFFER                     6 // temporal buffer size

#pragma warning(disable:4018)
#pragma warning(disable:4800)

template <typename T> INLINE T Clip3 (const T minVal, const T maxVal, const T a) { return std::min<T> (std::max<T> (minVal, a) , maxVal); }  ///< general min/max clip

typedef u8 AlfClassifier;
typedef pel Pel;

typedef int PosType;
typedef uint32_t SizeType;

struct Position
{
  PosType x;
  PosType y;

  Position()                                   : x(0),  y(0)  { }
  Position(const PosType _x, const PosType _y) : x(_x), y(_y) { }

  bool operator!=(const Position &other)  const { return x != other.x || y != other.y; }
  bool operator==(const Position &other)  const { return x == other.x && y == other.y; }

  Position offset(const Position pos)                 const { return Position(x + pos.x, y + pos.y); }
  Position offset(const PosType _x, const PosType _y) const { return Position(x + _x   , y + _y   ); }
  void     repositionTo(const Position newPos)              { x  = newPos.x; y  = newPos.y; }
  void     relativeTo  (const Position origin)              { x -= origin.x; y -= origin.y; }

  Position operator-( const Position &other )         const { return{ x - other.x, y - other.y }; }
};

struct Size
{
  SizeType width;
  SizeType height;

  Size()                                              : width(0),      height(0)       { }
  Size(const SizeType _width, const SizeType _height) : width(_width), height(_height) { }

  bool operator!=(const Size &other)      const { return (width != other.width) || (height != other.height); }
  bool operator==(const Size &other)      const { return (width == other.width) && (height == other.height); }
  uint32_t area()                             const { return (uint32_t) width * (uint32_t) height; }
};

struct Area : public Position, public Size
{
  Area()                                                                         : Position(),       Size()       { }
  Area(const Position &_pos, const Size &_size)                                  : Position(_pos),   Size(_size)  { }
  Area(const PosType _x, const PosType _y, const SizeType _w, const SizeType _h) : Position(_x, _y), Size(_w, _h) { }

        Position& pos()                           { return *this; }
  const Position& pos()                     const { return *this; }
        Size&     size()                          { return *this; }
  const Size&     size()                    const { return *this; }

  const Position& topLeft()                 const { return *this; }
        Position  topRight()                const { return { (PosType) (x + width - 1), y                          }; }
        Position  bottomLeft()              const { return { x                        , (PosType) (y + height - 1) }; }
        Position  bottomRight()             const { return { (PosType) (x + width - 1), (PosType) (y + height - 1) }; }
        Position  center()                  const { return { (PosType) (x + width / 2), (PosType) (y + height / 2) }; }

  bool contains(const Position &_pos)       const { return (_pos.x >= x) && (_pos.x < (x + width)) && (_pos.y >= y) && (_pos.y < (y + height)); }
  bool contains(const Area &_area)          const { return contains(_area.pos()) && contains(_area.bottomRight()); }

  bool operator!=(const Area &other)        const { return (Size::operator!=(other)) || (Position::operator!=(other)); }
  bool operator==(const Area &other)        const { return (Size::operator==(other)) && (Position::operator==(other)); }
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};

enum ComponentID
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT   = 3,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

static INLINE ChannelType toChannelType             (const ComponentID id)                         { return (id==COMPONENT_Y)? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA; }
static INLINE bool        isLuma                    (const ComponentID id)                         { return (id==COMPONENT_Y);                                          }
static INLINE bool        isLuma                    (const ChannelType id)                         { return (id==CHANNEL_TYPE_LUMA);                                    }
static INLINE bool        isChroma                  (const ComponentID id)                         { return (id!=COMPONENT_Y);                                          }
static INLINE bool        isChroma                  (const ChannelType id)                         { return (id!=CHANNEL_TYPE_LUMA);   }

static INLINE uint32_t        getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMPONENT;                  }
static INLINE uint32_t        getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CHANNEL_TYPE;               }

struct ClpRng
{
  int min;
  int max;
  int bd;
  int n;

  ClpRng() { min = 0; max = 1023; bd = 10; n = 0; }
//  const ClpRng() { min = 0; max = 1023; bd = 10; n = 0; }

};

struct ClpRngs
{
  ClpRng comp[MAX_NUM_COMPONENT]; ///< the bit depth as indicated in the SPS
  bool used;
  bool chroma;

  ClpRngs() { used = false; chroma = false; }
//  const ClpRngs() { used = false; chroma = false; }
};

template <typename T> INLINE T ClipPel (const T a, const ClpRng& clpRng)         { return std::min<T> (std::max<T> (clpRng.min, a) , clpRng.max); }  ///< clip reconstruction

struct CodingStructure
{
    void * pCtx;
    EVC_PIC * pPic;

    int tempStride; //to pass strides easily
    int picStride;
};

#if 1
#define MAX_NUM_ALF_CLASSES             25
#define MAX_NUM_ALF_LUMA_COEFF          13
#define MAX_NUM_ALF_CHROMA_COEFF        7
#define MAX_ALF_FILTER_LENGTH           7
#define MAX_NUM_ALF_COEFF               (MAX_ALF_FILTER_LENGTH * MAX_ALF_FILTER_LENGTH / 2 + 1)
#define ALF_FIXED_FILTER_NUM                              16


struct AlfFilterShape
{
  AlfFilterShape(int size)
    : filterLength(size),
    numCoeff(size * size / 4 + 1),
    filterSize(size * size / 2 + 1)
  {
    if (size == 5)
    {
      pattern = {
        0,
        1,  2,  3,
        4,  5,  6,  5,  4,
        3,  2,  1,
        0
      };

      weights = {
        2,
        2, 2, 2,
        2, 2, 1, 1
      };

      golombIdx = {
        0,
        0, 1, 0,
        0, 1, 2, 2
      };

      patternToLargeFilter = {
        0,
        0, 1, 0,
        0, 2, 3, 4, 0,
        0, 5, 6, 7
      };
      filterType = ALF_FILTER_5;
    }
    else if (size == 7)
    {
      pattern = {
        0,
        1,  2,  3,
        4,  5,  6,  7,  8,
        9, 10, 11, 12, 11, 10, 9,
        8,  7,  6,  5,  4,
        3,  2,  1,
        0
      };

      weights = {
        2,
        2,  2,  2,
        2,  2,  2,  2,  2,
        2,  2,  2,  1,  1
      };

      golombIdx = {
        0,
        0, 1, 0,
        0, 1, 2, 1, 0,
        0, 1, 2, 3, 3
      };

      patternToLargeFilter = {
        1,
        2, 3, 4,
        5, 6, 7, 8, 9,
        10,11,12,13
      };

      filterType = ALF_FILTER_7;
    }
    else
    {
      filterType = ALF_NUM_OF_FILTER_TYPES;
      CHECK(0, "Wrong ALF filter shape");
    }
  }

  AlfFilterType filterType;
  int filterLength;
  int numCoeff;      //TO DO: check whether we need both numCoeff and filterSize
  int filterSize;
  std::vector<int> pattern;
  std::vector<int> weights;

  std::vector<int> patternToLargeFilter;
  std::vector<int> golombIdx;
};


struct AlfTileGroupParam
{
  bool                         isCtbAlfOn;
  u8                           alfCtuEnableFlag[3][512];                                // put into tile_group header
  bool                         enabledFlag[MAX_NUM_COMPONENT];                          // alf_tile_group_enable_flag, alf_chroma_idc
  AlfFilterType                lumaFilterType;                                          // filter_type_flag
  bool                         chromaCtbPresentFlag;                                    // alf_chroma_ctb_present_flag
  short                        lumaCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
  short                        chromaCoeff[MAX_NUM_ALF_CHROMA_COEFF];                   // alf_coeff_chroma[i]
  short                        filterCoeffDeltaIdx[MAX_NUM_ALF_CLASSES];                // filter_coeff_delta[i]
  bool                         filterCoeffFlag[MAX_NUM_ALF_CLASSES];                    // filter_coefficient_flag[i]
  int                          numLumaFilters;                                          // number_of_filters_minus1 + 1
  bool                         coeffDeltaFlag;                                          // alf_coefficients_delta_flag
  bool                         coeffDeltaPredModeFlag;                                  // coeff_delta_pred_mode_flag
  std::vector<AlfFilterShape>* filterShapes;

  int                          fixedFilterPattern;                                     //0: no pred from pre-defined filters; 1: all are predicted but could be different values; 2: some predicted and some not
                                                                                       //when ALF_LOWDELAY is 1, fixedFilterPattern 0: all are predected, fixedFilterPattern 1: some predicted and some not
  int                          fixedFilterIdx[MAX_NUM_ALF_CLASSES];
  int                          tLayer;
  bool                         temporalAlfFlag;         //indicate whether reuse previous ALF coefficients
  int                          prevIdx;                 //index of the reused ALF coefficients

    bool resetALFBufferFlag;
    bool store2ALFBufferFlag;
  void reset()
  {
    isCtbAlfOn = false;
    std::memset( alfCtuEnableFlag, 1, sizeof(alfCtuEnableFlag) );
    std::memset(enabledFlag, false, sizeof(enabledFlag));
    lumaFilterType = ALF_FILTER_5;
    std::memset(lumaCoeff, 0, sizeof(lumaCoeff));
    std::memset(chromaCoeff, 0, sizeof(chromaCoeff));
    std::memset(filterCoeffDeltaIdx, 0, sizeof(filterCoeffDeltaIdx));
    std::memset(filterCoeffFlag, true, sizeof(filterCoeffFlag));
    numLumaFilters = 1;
    coeffDeltaFlag = false;
    coeffDeltaPredModeFlag = false;
    chromaCtbPresentFlag = false;
    fixedFilterPattern = 0;
    std::memset(fixedFilterIdx, 0, sizeof(fixedFilterIdx));
    temporalAlfFlag = false;
    prevIdx = 0;
    tLayer = 0;
    resetALFBufferFlag = false;
    store2ALFBufferFlag = false;
  }

  const AlfTileGroupParam& operator = (const AlfTileGroupParam& src)
  {
    std::memcpy(enabledFlag, src.enabledFlag, sizeof(enabledFlag));
    lumaFilterType = src.lumaFilterType;
    std::memcpy(lumaCoeff, src.lumaCoeff, sizeof(lumaCoeff));
    std::memcpy(chromaCoeff, src.chromaCoeff, sizeof(chromaCoeff));
    std::memcpy(filterCoeffDeltaIdx, src.filterCoeffDeltaIdx, sizeof(filterCoeffDeltaIdx));
    std::memcpy(filterCoeffFlag, src.filterCoeffFlag, sizeof(filterCoeffFlag));
    numLumaFilters = src.numLumaFilters;
    coeffDeltaFlag = src.coeffDeltaFlag;
    coeffDeltaPredModeFlag = src.coeffDeltaPredModeFlag;
    filterShapes = src.filterShapes;
    chromaCtbPresentFlag = src.chromaCtbPresentFlag;
    fixedFilterPattern = src.fixedFilterPattern;
    std::memcpy(fixedFilterIdx, src.fixedFilterIdx, sizeof(fixedFilterIdx));
    temporalAlfFlag = src.temporalAlfFlag;
    prevIdx = src.prevIdx;
    tLayer = src.tLayer;
    return *this;
  }
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

struct AdaptiveLoopFilter
{
public:
  static const int m_NUM_BITS = 10;
  static const int m_CLASSIFICATION_BLK_SIZE = 32;  //non-normative, local buffer size

  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() {}
  void ALFProcess( CodingStructure& cs, AlfTileGroupParam& alfTileGroupParam );
  void reconstructCoeff( AlfTileGroupParam& alfTileGroupParam, ChannelType channel, 
  const bool bRdo = false,
  const bool bRedo = false );
  void create( const int picWidth, const int picHeight, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth);
  void destroy();
  static void deriveClassificationBlk( AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const pel * srcLuma, const int srcStride, const Area& blk, const int shift );
  void deriveClassification( AlfClassifier** classifier, const pel * srcLuma, const int srcLumaStride, const Area& blk );

  template<AlfFilterType filtType>
  static void filterBlk( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng );

  INLINE static int getMaxGolombIdx( AlfFilterType filterType )
  {
    return filterType == ALF_FILTER_5 ? 2 : 3;
  }

  void( *m_deriveClassificationBlk )( AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const pel * srcLuma, const int srcStride, const Area& blk, const int shift );
  void( *m_filter5x5Blk )( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng );
  void( *m_filter7x7Blk )( AlfClassifier** classifier, pel * recDst, const int dstStride, const pel * recSrc, const int srcStride, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng );

#ifdef TARGET_SIMD_X86
  void initAdaptiveLoopFilterX86();
  template <X86_VEXT vext>
  void _initAdaptiveLoopFilterX86();
#endif

  INLINE static int getShapeIdx(AlfTileGroupParam& alfParam, bool isLuma)
  {
    if (isLuma)
    {
      return alfParam.lumaFilterType == ALF_FILTER_5 ? 0 : 1;
    }
    else
    {
      return 0;
    }
  }
  void storeALFParam(AlfTileGroupParam* pAlfParam, unsigned tLayer, unsigned tLayerMax);
  void loadALFParam(AlfTileGroupParam* pAlfParam, unsigned idx, unsigned tLayer);
  void resetTemporalAlfBuffer();
  void storeALFParamLine(AlfTileGroupParam* pAlfParam, unsigned tLayer);
  void loadALFParamLine(AlfTileGroupParam* pAlfParam, unsigned idx, unsigned tLayer);
  void resetTemporalAlfBufferLine();
  void resetTemporalAlfBufferLine2First();

protected:
  static const int             m_fixedFilterNum = 64;
  static const int             m_fixedFilterCoeff[m_fixedFilterNum][MAX_NUM_ALF_LUMA_COEFF];
  static const int             m_classToFilterMapping[MAX_NUM_ALF_CLASSES][ALF_FIXED_FILTER_NUM];
  AlfTileGroupParam                m_acStoredAlfPara[MAX_NUM_TLAYER][MAX_NUM_ALFS_PER_TLAYER];
  unsigned                     m_storedAlfParaNum[MAX_NUM_TLAYER];
public:
  bool m_store2ALFBufferFlag;
  bool m_resetALFBufferFlag;
protected:
  AlfTileGroupParam                m_acAlfLineBuffer[ALF_TEMPORAL_WITH_LINE_BUFFER];
  AlfTileGroupParam                m_IRAPFilter;
public:
  int                          m_lastRasPoc;
  bool                         m_pendingRasInit;
  unsigned                     m_acAlfLineBufferCurrentSize;

  int**                        m_laplacian[NUM_DIRECTIONS];
  uint8_t*                     m_ctuEnableFlag[MAX_NUM_COMPONENT];
  pel *                        m_tempBuf;
  int                          m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
  int                          m_picWidth;
  int                          m_picHeight;
  int                          m_maxCUWidth;
  int                          m_maxCUHeight;
  int                          m_maxCUDepth;
  int                          m_numCTUsInWidth;
  int                          m_numCTUsInHeight;
  int                          m_numCTUsInPic;
  ChromaFormat                 m_chromaFormat;

public:
  const ClpRngs                m_clpRngs;
  AlfClassifier**              m_classifier;
  short                        m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  std::vector<AlfFilterShape>  m_filterShapes[MAX_NUM_CHANNEL_TYPE];
};

#endif // <-- min
#endif // <-- max
#endif
#endif