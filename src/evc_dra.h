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

#ifndef _EVC_DRA_H_
#define _EVC_DRA_H_

#include "evc_def.h"
#include <stdlib.h>

#if M52291_HDR_DRA
typedef struct _quantParamDRA {

    int m_value;  // Currently 32 bit is considered sufficient
    int m_numFracBits;
    int m_numTotBits;
}quantParamDRA;

typedef struct _DRAChromaOffControl
{
    BOOL   enabled;         ///< Enabled flag (0:default)
    double chromaCbQpScale; ///< Chroma Cb QP Scale (1.0:default)
    double chromaCrQpScale; ///< Chroma Cr QP Scale (1.0:default)
    double chromaQpScale;   ///< Chroma QP Scale (0.0:default)
    double chromaQpOffset;  ///< Chroma QP Offset (0.0:default)

    int m_baseLumaQP;
    int m_draChromaCbQpOffset;
    int m_draChromaCrQpOffset;

}DRAChromaOffControl;

typedef struct _DRAScaleMapping
{
    double m_draScaleMapY[256][2];          ///< first=luma level, second=delta QP.
} DRAScaleMapping;

typedef struct _SignalledParamsDRA
{
    int m_signal_dra_flag;
    int m_baseLumaQP;
    BOOL m_equalRangesFlag;
    int  m_deltaVal;
    int m_numRanges;
    int m_inRanges[33];
    int m_numFracBitsScale;
    int m_numIntBitsScale;
    int m_intScaleCbDRA;
    int m_intScaleCrDRA;
    int m_intDraScales[33 - 1];
}SignalledParamsDRA;

typedef struct _WCGDDRAControl
{
    BOOL m_flagEnabled;
    DRAScaleMapping m_draScaleMap;
    DRAChromaOffControl m_chromaQPModel;
    
    //------ Signalled DRA Params ------//
    int m_numFracBitsScale;
    int m_numIntBitsScale;
    SignalledParamsDRA m_signalledDRA;

    //------ DRA Model ------//
    int m_atfNumRanges;
    int m_atfInRanges[33];
    double m_atfOutRanges[33];
    double m_atfDraScales[33 - 1];
    double m_atfDraOffsets[33 - 1];

    int m_intScaleCbDRA;
    int m_intScaleCrDRA;
    int m_atfIntOutRanges[33];
    int m_atfIntDraScales[33 - 1];
    int m_atfIntInvDraScales[33 - 1];
    int m_atfIntInvDraOffsets[33 - 1];
    int m_atfIntChromaDraScales[2][33 - 1];
    int m_atfIntChromaInvDraScales[2][33 - 1];
       
    //------ DRA LUT ------//
    int m_lumaScaleLUT[DRA_LUT_MAXSIZE];               // LUT for luma and correspionding QP offset
    int m_lumaInvScaleLUT[DRA_LUT_MAXSIZE];               // LUT for luma and correspionding QP offset
    int m_intChromaScaleLUT[2][DRA_LUT_MAXSIZE];               // LUT for chroma scales 
    int m_intChromaInvScaleLUT[2][DRA_LUT_MAXSIZE];               // LUT for chroma scales 
                                                                  //------ Gammut mapping ------//
    //------ Adaptive mapping ------//
    double m_draHistNorm;
    int    m_globalOffset;
    int    m_globalEnd;

} WCGDDRAControl;

/* Encoder side functions are listed below: */
void evce_initDRA(WCGDDRAControl *p_DRAMapping, int totalChangePoints, int *lumaChangePoints, int* qps);
BOOL evce_analyzeInputPic(WCGDDRAControl *p_DRAMapping);

/* Decoder side functions are listed below: */
void evcd_initDRA(WCGDDRAControl *p_DRAMapping);

/* DRA applicaton (sample processing) functions are listed below: */
void evc_apply_dra_luma_plane(EVC_IMGB * dst, EVC_IMGB * src, WCGDDRAControl *p_DRAMapping, int planeId, int backwardMap);
void evc_apply_dra_chroma_plane(EVC_IMGB * dst, EVC_IMGB * src, WCGDDRAControl *p_DRAMapping, int planeId, int backwardMap);

/* DRA APS buffer functions are listed below: */
void evc_addDraApsToBuffer(SignalledParamsDRA* p_g_dra_control_array, EVC_APS_GEN *p_aps_gen_array);
void evc_resetApsGenReadBuffer(EVC_APS_GEN *p_aps_gen_array);
#endif

#endif /* _EVC_DRA_H_ */