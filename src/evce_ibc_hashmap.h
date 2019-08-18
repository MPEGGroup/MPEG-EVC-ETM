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

/** \file     evce_ibc_hashmap.h
    \brief    IBC hash map encoder class (header)
*/

#ifndef __EVCE_IBC_HASHMAP__
#define __EVCE_IBC_HASHMAP__

#include "evc_def.h"
#include "evce_def.h"

#ifdef min
#undef min

#ifdef max
#undef max

#if IBC

#include <unordered_map>
#include <vector>

// ====================================================================================================================
// Class definition
// ====================================================================================================================
typedef std::pair<int, int> Position;

class EVCE_IBC_HashMap
{
private:
  int     m_picWidth;
  int     m_picHeight;

  unsigned int xxCalcBlockHash(const pel* pel, const int stride, const int width, const int height, unsigned int crc);

  void    xxBuildPicHashMap(const EVC_PIC* pic);

  static  uint32_t xxComputeCrc32c16bit(uint32_t crc, const pel pel);

public:
    unsigned int**  m_pos2Hash;
    std::unordered_map<unsigned int, std::vector<Position>> m_hash2Pos;

public:
  uint32_t (*m_computeCrc32c) (uint32_t crc, const pel pel);

  EVCE_IBC_HashMap();
  virtual ~EVCE_IBC_HashMap();

  void    init(const int picWidth, const int picHeight);
  void    destroy();
  void    rebuildPicHashMap(const EVC_PIC* pic);
  bool    ibcHashMatch(EVCE_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
      std::vector<Position>& cand, const int maxCand, const int searchRange4SmallBlk);
  int     getHashHitRatio(EVCE_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh);
};

#endif // <-- min
#endif // <-- max
#endif
#endif // __EVCE_IBC_HASHMAP__
