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

#include "evc_def.h"
#include <math.h>

#if QC_DRA
double evce_getQP2ScaleDRA(int cbQP) {
    double scaleDRA = 1.0;
    scaleDRA = exp(((double)(cbQP) / 6)*log(2.0));
    return scaleDRA;
}

double evce_getCbScaleDRA(DRAChromaOffControl *p_dra_chroma_control, int l_iQP) {
    double scaleCbDRA = 1.0;
    double chromaQp = p_dra_chroma_control->chromaQpScale * l_iQP + p_dra_chroma_control->chromaQpOffset;
    chromaQp = chromaQp * p_dra_chroma_control->chromaCbQpScale;
    int cbQP = (int)(chromaQp + (chromaQp < 0 ? -0.5 : 0.5));
    cbQP = EVC_CLIP3(-12, 12, min(0, cbQP) + p_dra_chroma_control->m_draChromaCbQpOffset);
    cbQP = cbQP - p_dra_chroma_control->m_draChromaCbQpOffset;
    scaleCbDRA = evce_getQP2ScaleDRA(cbQP);
    scaleCbDRA = 1 / scaleCbDRA;  // chroma QP Offset is added to luma, which equialent of reduced scale factor 1/x
    return    scaleCbDRA;
}

double evce_getCrScaleDRA(DRAChromaOffControl *p_dra_chroma_control, int l_iQP) {
    double scaleCrDRA = 1.0;
    double chromaQp = p_dra_chroma_control->chromaQpScale * l_iQP + p_dra_chroma_control->chromaQpOffset;

    chromaQp = chromaQp * p_dra_chroma_control->chromaCrQpScale;
    int crQP = (int)(chromaQp + (chromaQp < 0 ? -0.5 : 0.5));
    crQP = EVC_CLIP3(-12, 12, min(0, crQP) + p_dra_chroma_control->m_draChromaCrQpOffset);
    crQP = crQP - p_dra_chroma_control->m_draChromaCrQpOffset;
    scaleCrDRA = evce_getQP2ScaleDRA(crQP);
    scaleCrDRA = 1 / scaleCrDRA;  
    return    scaleCrDRA;
}
void evce_zoomInRangeLUT(WCGDDRAControl *p_DRAMapping, int sdrFlag) {
    double m_lumRenorm = 1.0;
    if (sdrFlag == 1)
    {
        p_DRAMapping->m_globalOffset = 0;
        p_DRAMapping->m_globalEnd = DRA_LUT_MAXSIZE - 1;
        m_lumRenorm = 1.0;
    }
    if ((p_DRAMapping->m_globalOffset == 0) && (p_DRAMapping->m_globalEnd == 0))
    {
        return;
    }

    int deltas[33];
    double SCALE_MAX = 1.7;
    m_lumRenorm = (double)(DRA_LUT_MAXSIZE) / (double)(DRA_LUT_MAXSIZE - (p_DRAMapping->m_globalOffset + DRA_LUT_MAXSIZE - p_DRAMapping->m_globalEnd));

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        deltas[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfInRanges[i];
    }
    m_lumRenorm = (m_lumRenorm > SCALE_MAX) ? SCALE_MAX : m_lumRenorm;

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        deltas[i] = (int)(deltas[i] / m_lumRenorm + 0.5);
    }
    p_DRAMapping->m_atfInRanges[0] = p_DRAMapping->m_globalOffset;
    p_DRAMapping->m_atfDraScales[0] = p_DRAMapping->m_atfDraScales[0] * m_lumRenorm;
    for (int i = 1; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfInRanges[i] = p_DRAMapping->m_atfInRanges[i - 1] + deltas[i - 1];
        p_DRAMapping->m_atfDraScales[i] = p_DRAMapping->m_atfDraScales[i] * m_lumRenorm;
    }
    p_DRAMapping->m_atfInRanges[p_DRAMapping->m_atfNumRanges] = p_DRAMapping->m_atfInRanges[p_DRAMapping->m_atfNumRanges - 1] + deltas[p_DRAMapping->m_atfNumRanges - 1];

    p_DRAMapping->m_atfOutRanges[0] = 0;
    for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
    {
        p_DRAMapping->m_atfOutRanges[i] = (int)(p_DRAMapping->m_atfOutRanges[i - 1] + p_DRAMapping->m_atfDraScales[i - 1] * deltas[i - 1] + 0.5);
    }
    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfDraOffsets[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfOutRanges[i + 1] / p_DRAMapping->m_atfDraScales[i];
    }
    return;
}

int evce_getScaleDRA2QP(double scaleDra) {
    int QP = 0;
    double value = 0;
    value = log(scaleDra)*6.0 / log(2.0);
    QP = value > 0 ? (int)(value + 0.5) : (int)(-1 * value + 0.5) * (-1);
    return QP;
}

double evce_getScaleDRA2QPd(double scaleDra) {
    double QP = 0;
    double value = 0;
    value = log(scaleDra)*6.0 / log(2.0);
    QP = value;
    return QP;
}

double evce_getQP2ScaleDRAd(double cbQP) {
    double scaleDRA = 1.0;
    scaleDRA = exp(((double)(cbQP) / 6)*log(2.0));
    return scaleDRA;
}

int getScaledChromaQP2(int compId, int unscaledChromaQP)
{
    int qpBdOffsetC = 6 * (BIT_DEPTH - 8);
    int qp_value = EVC_CLIP3(-qpBdOffsetC, MAX_QP_TABLE_SIZE - 1, unscaledChromaQP);
    qp_value = *(p_evc_tbl_qp_chroma_dynamic[compId - 1] + qp_value);
    return qp_value;
}

void     prec_quantize_entry_i(quantParamDRA *p_quantParamEntry, int const value, int const intBits)
{
    int temp = (int)floor(value + 0.5);
    p_quantParamEntry->m_value = temp;
    p_quantParamEntry->m_numFracBits = 0;
    if (temp == 0)
    {
        p_quantParamEntry->m_numFracBits = 0;
        p_quantParamEntry->m_numTotBits = 1;
    }
    else
    {
        int estBits = (int)ceil(log(abs(temp) + 0.0) / log(2.0));
        p_quantParamEntry->m_numTotBits = min(estBits, intBits);
    }
}
void     prec_quantize_entry_d(quantParamDRA *p_quantParamEntry, double const value, int const fracBits, int const intBits)
{
    int temp = (int)floor(value*(1 << fracBits) + 0.5);
    p_quantParamEntry->m_value = temp;
    p_quantParamEntry->m_numFracBits = fracBits;
    if (temp == 0)
    {
        p_quantParamEntry->m_numFracBits = 0;
        p_quantParamEntry->m_numTotBits = 1;
    }
    else
    {
        int estBits = (int)ceil(log(abs(temp) + 0.0) / log(2.0));
        p_quantParamEntry->m_numTotBits = min(estBits, intBits + fracBits);
    }
}
int getIntVal(quantParamDRA *p_quantParamEntry)
{
    int shiftVal = 1 << (p_quantParamEntry->m_numFracBits - 1);
    return (p_quantParamEntry->m_value + shiftVal) >> p_quantParamEntry->m_numFracBits;
}
int getMValue(quantParamDRA *p_quantParamEntry)
{
    evc_assert(p_quantParamEntry->m_value >= 0);
    return p_quantParamEntry->m_value;
}
float getVal(quantParamDRA *p_quantParamEntry)
{
    return (float)(p_quantParamEntry->m_value + 0.0) / (1 << p_quantParamEntry->m_numFracBits);
}
int getFracBits(quantParamDRA *p_quantParamEntry) { return p_quantParamEntry->m_numFracBits; }
int getTotBits(quantParamDRA *p_quantParamEntry) { return p_quantParamEntry->m_numTotBits; }
void rshiftNoRound(quantParamDRA *valueThis, int const val) 
{
    valueThis->m_value >>= val;
    valueThis->m_numFracBits -= val;
}
void rshift(quantParamDRA *valueThis, int const val)
{
    int shiftVal = 1 << (val - 1);
    valueThis->m_value = (valueThis->m_value + shiftVal) >> val;
    valueThis->m_numFracBits -= val;
}
void lshift(quantParamDRA *valueThis, int const val)
{
    valueThis->m_value <<= val;
    valueThis->m_numFracBits += val;
    valueThis->m_numTotBits += val;
}

quantParamDRA prec_plus_entry(quantParamDRA valueThis, const quantParamDRA rhs)
{
    quantParamDRA thisPrec;
    quantParamDRA temp = rhs;
    quantParamDRA tempL = valueThis;
    if (valueThis.m_numFracBits != rhs.m_numFracBits)
    {
        int numFracBitsFinal = max(tempL.m_numFracBits, temp.m_numFracBits);
        lshift(&tempL, numFracBitsFinal - tempL.m_numFracBits);
        lshift(&temp, numFracBitsFinal - temp.m_numFracBits);

        thisPrec.m_numFracBits = numFracBitsFinal;
    }
    else
    {
        thisPrec.m_numFracBits = rhs.m_numFracBits;
    }
    thisPrec.m_value = tempL.m_value + temp.m_value;
    thisPrec.m_numTotBits = max(tempL.m_numTotBits, rhs.m_numTotBits) + 1;
    return thisPrec;
}
quantParamDRA prec_minus_entry(quantParamDRA valueThis, const quantParamDRA rhs)
{
    quantParamDRA thisPrec;
    quantParamDRA temp = rhs;
    quantParamDRA tempL = valueThis;
    if (valueThis.m_numFracBits != rhs.m_numFracBits)
    {
        int numFracBitsFinal = max(tempL.m_numFracBits, temp.m_numFracBits);
        lshift(&tempL, numFracBitsFinal - valueThis.m_numFracBits);
        lshift(&temp, numFracBitsFinal - temp.m_numFracBits);

        thisPrec.m_numFracBits = numFracBitsFinal;
    }
    else
    {
        thisPrec.m_numFracBits = rhs.m_numFracBits;
    }
    thisPrec.m_value = tempL.m_value - temp.m_value;
    thisPrec.m_numTotBits = max(tempL.m_numTotBits, rhs.m_numTotBits) + 1;
    return thisPrec;
}
quantParamDRA prec_mult_entry(quantParamDRA valueThis, const quantParamDRA rhs)
{
    quantParamDRA thisPrec;
    thisPrec.m_value = valueThis.m_value * rhs.m_value;
    if (thisPrec.m_value == 0)
    {
        thisPrec.m_numTotBits = 1;
        thisPrec.m_numFracBits = 0;
    }
    else
    {
        thisPrec.m_numTotBits = valueThis.m_numTotBits + rhs.m_numTotBits;
        thisPrec.m_numFracBits = valueThis.m_numFracBits + rhs.m_numFracBits;
    }
    return thisPrec;
}
quantParamDRA prec_divide_entry(quantParamDRA valueThis, quantParamDRA const rhs)
{
    quantParamDRA thisPrec;
    thisPrec.m_value = (valueThis.m_value + (rhs.m_value / 2)) / rhs.m_value;
    if (thisPrec.m_value == 0)
    {
        thisPrec.m_numTotBits = 1;
        thisPrec.m_numFracBits = 0;
    }
    else
    {
        thisPrec.m_numTotBits = valueThis.m_numTotBits - rhs.m_numTotBits;
        thisPrec.m_numFracBits = valueThis.m_numFracBits - rhs.m_numFracBits;
    }
    return thisPrec;
}
void setFracBits(quantParamDRA *valueThis, int const nBits)
{
    if (valueThis->m_numFracBits < nBits)
    {
        lshift(valueThis, nBits - valueThis->m_numFracBits);
    }
    else if (valueThis->m_numFracBits > nBits)
    {
        rshift(valueThis, valueThis->m_numFracBits - nBits);
    }
    if (valueThis->m_value == 0)
    {
        valueThis->m_numTotBits = 0;
    }
    else
    {
        int estBits = (int)ceil(log(abs(valueThis->m_value) + 0.0) / log(2.0));
        valueThis->m_numTotBits = estBits;
    }
}
int getDraRangeIdx_gen(WCGDDRAControl *p_DRAMapping, int sample, int *chromaRanges, int numRanges) {
    int rangeIdx = -1;
    for (int i = 0; i < numRanges; i++)
    {
        if (sample < chromaRanges[i + 1])
        {
            rangeIdx = i;
            break;
        }
    }
    if (rangeIdx == -1)
        rangeIdx = numRanges - 1;
    return EVC_CLIP(rangeIdx, 0, numRanges - 1);
}

int correctLocalChromaScale_lut2(WCGDDRAControl *p_DRAMapping, int intScaleLumaDra, int chId)
{
    int l_array[NUM_CHROMA_QP_OFFSET_LOG];
    memcpy(l_array, g_dra_chroma_qp_offset_tbl, NUM_CHROMA_QP_OFFSET_LOG * sizeof(int));
    int SCALE_OFFSET = 1 << QC_SCALE_NUMFBITS;
    int TABLE0_SHIFT = NUM_CHROMA_QP_SCALE_EXP >> 1;
    int outChromaScale = 1;

    int localQPi;
    int Qp0, Qp1;
    int scaleDraInt = 1;
    int qpDraFrac = 0;
    scaleDraInt = (chId == 1) ? p_DRAMapping->m_intScaleCbDRA * intScaleLumaDra : p_DRAMapping->m_intScaleCrDRA * intScaleLumaDra;

    int localChromaQPShift1 = p_DRAMapping->m_chromaQPModel.m_baseLumaQP - (getScaledChromaQP2(chId, p_DRAMapping->m_chromaQPModel.m_baseLumaQP));

    int qpDraInt = 0;
    int OutofRange = -1;
    int scaleDraInt9 = (scaleDraInt + (1 << 8)) >> 9;
    int i = getDraRangeIdx_gen(p_DRAMapping, scaleDraInt9, l_array, NUM_CHROMA_QP_OFFSET_LOG - 1 ) + 1;
    qpDraInt = i - 81;
    localQPi = p_DRAMapping->m_chromaQPModel.m_baseLumaQP - qpDraInt;
    Qp0 = getScaledChromaQP2(chId, localQPi);
    Qp1 = getScaledChromaQP2(chId, (localQPi + 1));

    int interpolationNum = scaleDraInt9 - g_dra_chroma_qp_offset_tbl[i - 1];
    int interpolationDenom = g_dra_chroma_qp_offset_tbl[i] - g_dra_chroma_qp_offset_tbl[i - 1];
    qpDraFrac = SCALE_OFFSET * (interpolationDenom - interpolationNum) / interpolationDenom;

    int qpChDec = (Qp1 - Qp0) * qpDraFrac;
    int    qpDraFracAdj = qpChDec % (1 << 9);
    int qpDraIntAdj = (qpChDec >> 9);

    qpDraFracAdj = qpDraFrac - qpDraFracAdj;
    int localChromaQPShift2 = localQPi - Qp0 - qpDraIntAdj;

    int draChromaQpShift = localChromaQPShift2 - localChromaQPShift1;
    if (qpDraFracAdj < 0)
    {
        draChromaQpShift -= 1;
        qpDraFracAdj = (1 << 9) + qpDraFracAdj;
    }
    assert(abs(draChromaQpShift) < ((NUM_CHROMA_QP_SCALE_EXP + 1) >> 1));

    int draChromaScaleShift = g_dra_exp_nom_v2[draChromaQpShift + TABLE0_SHIFT];
    int draChromaScaleShiftFrac;
    if (draChromaQpShift >= 0)
        draChromaScaleShiftFrac = g_dra_exp_nom_v2[draChromaQpShift + 1 + TABLE0_SHIFT] - g_dra_exp_nom_v2[draChromaQpShift + TABLE0_SHIFT];
    else
        draChromaScaleShiftFrac = g_dra_exp_nom_v2[draChromaQpShift + TABLE0_SHIFT] - g_dra_exp_nom_v2[draChromaQpShift - 1 + TABLE0_SHIFT];

    outChromaScale = draChromaScaleShift + ((draChromaScaleShiftFrac * qpDraFracAdj + (1 << (QC_SCALE_NUMFBITS - 1))) >> QC_SCALE_NUMFBITS);
    return (scaleDraInt * outChromaScale + (1 << 17)) >> 18;
}

int correctLocalChromaScale_lut(WCGDDRAControl *p_DRAMapping, double scaleLumaDra, int chId)
{
    int outChromaScale = 1;
    double chromaQPScale;

    int useFixedPt = TRUE;
    if (useFixedPt)
    {
        quantParamDRA temp;
        prec_quantize_entry_d(&temp, scaleLumaDra, QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
        scaleLumaDra = getVal(&temp);
    }

    int chromaQPOffset;
    int localQPi;
    int qpChroma1Output, qpChroma2Output;
    int l_genChromaOffset = 0;
    double scaleDra = 0.0;
    int ChromaQPoffsetD = 0;
    if (chId == 1)
    {
        scaleDra = p_DRAMapping->m_scaleCbDRA * scaleLumaDra; 
        l_genChromaOffset = p_DRAMapping->m_chromaQPModel.m_draChromaCbQpOffset;
    }
    else if (chId == 2)
    {
        scaleDra = p_DRAMapping->m_scaleCrDRA * scaleLumaDra;
        l_genChromaOffset = p_DRAMapping->m_chromaQPModel.m_draChromaCrQpOffset;
    }
    else { chromaQPOffset = 0; }

    localQPi = p_DRAMapping->m_chromaQPModel.m_baseLumaQP + l_genChromaOffset;
    int localChromaQPShift1 = localQPi - (getScaledChromaQP2(chId, localQPi));

    int scaleDraInt = (int)floor(scaleDra * (1 << QC_SCALE_NUMFBITS) + 0.5);
    int ChromaQPoffset = 0;
    int OutofRange = -1;
    for (int i = 0; i < NUM_CHROMA_QP_OFFSET_LOG; i++)
    {
        if (scaleDraInt < g_dra_chroma_qp_offset_tbl[i])
        {
#if QC_DRA_LUT_SUBSAMPLE_TWO
            ChromaQPoffset = 2 * i - 81;
#else
            ChromaQPoffset = i - 81;
#endif
            int interpolationNum = scaleDraInt - g_dra_chroma_qp_offset_tbl[i - 1];
            int interpolationDenom = g_dra_chroma_qp_offset_tbl[i] - g_dra_chroma_qp_offset_tbl[i - 1];
#if QC_DRA_LUT_SUBSAMPLE_TWO
            ChromaQPoffsetD = (interpolationNum << (QC_SCALE_NUMFBITS + 1)) / interpolationDenom -
                (((interpolationNum << QC_SCALE_NUMFBITS) % (interpolationDenom + 1) >> 1) >= ((interpolationDenom + 1) >> 1) ? 1 : 0);
#else
            ChromaQPoffsetD = (1 << QC_SCALE_NUMFBITS) - (interpolationNum << QC_SCALE_NUMFBITS) / interpolationDenom -
                (((interpolationNum << QC_SCALE_NUMFBITS) % interpolationDenom) >= (interpolationDenom >> 1) ? 1 : 0);
#endif
#if QC_DRA_LUT_SUBSAMPLE_TWO
            if (scaleDraInt < (1 << QC_SCALE_NUMFBITS))
            {
                ChromaQPoffset = ChromaQPoffset == 1 ? ChromaQPoffset - 1 : ChromaQPoffset - ((2 << QC_SCALE_NUMFBITS) - ChromaQPoffsetD) / (1 << QC_SCALE_NUMFBITS);
                ChromaQPoffsetD = ((2 << QC_SCALE_NUMFBITS) - ChromaQPoffsetD) % (1 << QC_SCALE_NUMFBITS);
            }
            else
            {
                ChromaQPoffset = ChromaQPoffset - 1 + ChromaQPoffsetD / (1 << QC_SCALE_NUMFBITS);
                ChromaQPoffsetD = ChromaQPoffsetD % (1 << QC_SCALE_NUMFBITS);
                ChromaQPoffsetD = (1 << QC_SCALE_NUMFBITS) - ChromaQPoffsetD;
            }
#endif
            OutofRange = 0;
            break;
        }
    }
    if (OutofRange)
    {
#if QC_DRA_LUT_SUBSAMPLE_TWO
        ChromaQPoffset = (NUM_CHROMA_QP_OFFSET_LOG << 1) - 82;
#else
        ChromaQPoffset = NUM_CHROMA_QP_OFFSET_LOG - 82;
#endif
        ChromaQPoffsetD = 0;
    }
    localQPi = p_DRAMapping->m_chromaQPModel.m_baseLumaQP + l_genChromaOffset - ChromaQPoffset;
    qpChroma1Output = getScaledChromaQP2(chId, localQPi);
    qpChroma2Output = getScaledChromaQP2(chId, (localQPi + 1));
    int qpChDec = (qpChroma2Output - qpChroma1Output) * ChromaQPoffsetD;
    int    qpChDecDec = qpChDec % (1 << 9);
    int qpChDecInt = (qpChDec >> 9) + (ChromaQPoffsetD >= qpChDecDec ? 0 : 1);
    qpChDecDec = ChromaQPoffsetD >= qpChDecDec ? (ChromaQPoffsetD - qpChDecDec) : ((1 << QC_SCALE_NUMFBITS) + ChromaQPoffsetD - qpChDecDec);
    int localChromaQPShift2 = localQPi - qpChroma1Output - qpChDecInt;
    assert(abs(localChromaQPShift2 - localChromaQPShift1) < ((NUM_CHROMA_QP_SCALE_EXP + 1) >> 1)); 
    int outChromaScaleInt = g_dra_exp_nom_v2[localChromaQPShift2 - localChromaQPShift1 + (NUM_CHROMA_QP_SCALE_EXP >> 1)];
    outChromaScale = outChromaScaleInt + ((localChromaQPShift2 - localChromaQPShift1 >= 0)
        ? (((g_dra_exp_nom_v2[localChromaQPShift2 - localChromaQPShift1 + (NUM_CHROMA_QP_SCALE_EXP >> 1) + 1]
            - g_dra_exp_nom_v2[localChromaQPShift2 - localChromaQPShift1 + (NUM_CHROMA_QP_SCALE_EXP >> 1)]) * qpChDecDec + (1 << (QC_SCALE_NUMFBITS - 1))) >> QC_SCALE_NUMFBITS)
        : (((g_dra_exp_nom_v2[localChromaQPShift2 - localChromaQPShift1 + (NUM_CHROMA_QP_SCALE_EXP >> 1)]
            - g_dra_exp_nom_v2[localChromaQPShift2 - localChromaQPShift1 + (NUM_CHROMA_QP_SCALE_EXP >> 1) - 1]) * qpChDecDec) + (1 << (QC_SCALE_NUMFBITS - 1))) >> QC_SCALE_NUMFBITS);

    if (chId == 1)
    {
        chromaQPScale = p_DRAMapping->m_scaleCbDRA;
    }
    else if (chId == 2)
    {
        chromaQPScale = p_DRAMapping->m_scaleCrDRA;
    }
    else {
        chromaQPScale = 0.0;
    }

    if (useFixedPt)
    {
        quantParamDRA temp;
        prec_quantize_entry_i(&temp, outChromaScale, QC_SCALE_NUMFBITS);
        (&temp)->m_numFracBits += QC_SCALE_NUMFBITS;
        quantParamDRA temp1;
        prec_quantize_entry_d(&temp1, chromaQPScale, QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
        temp1 = prec_mult_entry(temp1, temp);
        quantParamDRA temp2;
        prec_quantize_entry_d(&temp2, scaleLumaDra, QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
        temp1 = prec_mult_entry(temp1, temp2);
        setFracBits(&temp1, QC_SCALE_NUMFBITS);
        outChromaScale = (&temp1)->m_value;
    }
    else
    {
        outChromaScale = (int)(outChromaScale * scaleLumaDra);
    }
    return outChromaScale;
}

void evce_compensateChromaShiftTable(WCGDDRAControl *p_DRAMapping) {
    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++) {
        p_DRAMapping->m_atfChromaDraScales[0][i] = correctLocalChromaScale_lut(p_DRAMapping, p_DRAMapping->m_atfDraScales[i], 1);
        p_DRAMapping->m_atfChromaDraScales[1][i] = correctLocalChromaScale_lut(p_DRAMapping, p_DRAMapping->m_atfDraScales[i], 2);

        p_DRAMapping->m_atfChromaDraScales[0][i] /= (1 << 9);
        p_DRAMapping->m_atfChromaDraScales[1][i] /= (1 << 9);
    }
}

void evc_compensateChromaShiftTableInt(WCGDDRAControl *p_DRAMapping) {
    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++) {
        p_DRAMapping->m_atfIntChromaDraScales[0][i] = correctLocalChromaScale_lut2(p_DRAMapping, p_DRAMapping->m_atfIntDraScales[i], 1);
        p_DRAMapping->m_atfIntChromaDraScales[1][i] = correctLocalChromaScale_lut2(p_DRAMapping, p_DRAMapping->m_atfIntDraScales[i], 2);
        p_DRAMapping->m_atfIntChromaInvDraScales[0][i] = (1 << 18) / p_DRAMapping->m_atfIntChromaDraScales[0][i];
        p_DRAMapping->m_atfIntChromaInvDraScales[1][i] = (1 << 18) / p_DRAMapping->m_atfIntChromaDraScales[1][i];
        p_DRAMapping->m_atfChromaDraScales[0][i] = p_DRAMapping->m_atfIntChromaDraScales[0][i] / (1 << 9);
        p_DRAMapping->m_atfChromaDraScales[1][i] = p_DRAMapping->m_atfIntChromaDraScales[1][i] / (1 << 9);
    }
}



void evc_buildDraLumaLut2(WCGDDRAControl *p_DRAMapping)
{
    for (int i = 0; i < DRA_LUT_MAXSIZE; i++)
    {
        int rangeIdxY = getDraRangeIdx_gen(p_DRAMapping, i, p_DRAMapping->m_atfIntOutRanges, p_DRAMapping->m_atfNumRanges);
        int value = (int)((i * p_DRAMapping->m_atfIntInvDraScales[rangeIdxY] + (1 << 9)) >> 10);
        value = ((p_DRAMapping->m_atfIntInvDraOffsets[rangeIdxY] << 2) + value + (1 << 8)) >> 9;
        value = EVC_CLIP(value, 0, DRA_LUT_MAXSIZE - 1);
        p_DRAMapping->m_lumaInvScaleLUT[i] = value;
    }
}
void evc_buildDraChromaLut(WCGDDRAControl *p_DRAMapping)
{
    for (int i = 0; i < DRA_LUT_MAXSIZE; i++)
    {
        p_DRAMapping->m_intChromaScaleLUT[0][i] = p_DRAMapping->m_intChromaScaleLUT[1][i] = 1;
        p_DRAMapping->m_intChromaInvScaleLUT[0][i] = p_DRAMapping->m_intChromaScaleLUT[1][i] = 1;
    }
    {

        int chromaMultRanges2[33 + 1];
        int chromaMultScale[33 + 1];
        int chromaMultOffset[33 + 1];
        for (int ch = 0; ch < 2; ch++)
        {

            chromaMultRanges2[0] = p_DRAMapping->m_atfIntOutRanges[0];
            chromaMultScale[0] = 0;
            chromaMultOffset[0] = (int)(p_DRAMapping->m_atfIntChromaInvDraScales[ch][0]);
            for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
            {
                chromaMultRanges2[i] = (int)((p_DRAMapping->m_atfIntOutRanges[i - 1] + p_DRAMapping->m_atfIntOutRanges[i]) / 2);
            }

            for (int i = 1; i < p_DRAMapping->m_atfNumRanges; i++)
            {
                int deltaRange = chromaMultRanges2[i + 1] - chromaMultRanges2[i];
                chromaMultOffset[i] = p_DRAMapping->m_atfIntChromaInvDraScales[ch][i - 1];
                int deltaScale = p_DRAMapping->m_atfIntChromaInvDraScales[ch][i] - chromaMultOffset[i];
                chromaMultScale[i] = (int)(((deltaScale << QC_IN_RANGE_NUM_BITS) + (deltaRange >> 1)) / deltaRange);
            }
            chromaMultScale[p_DRAMapping->m_atfNumRanges] = 0;
            chromaMultOffset[p_DRAMapping->m_atfNumRanges] = p_DRAMapping->m_atfIntChromaInvDraScales[ch][p_DRAMapping->m_atfNumRanges - 1];

            for (int i = 0; i < DRA_LUT_MAXSIZE; i++)
            {
                int rangeIdx = getDraRangeIdx_gen(p_DRAMapping, i, chromaMultRanges2, p_DRAMapping->m_atfNumRanges + 1);
                int runI = i - chromaMultRanges2[rangeIdx];
                int runS = (chromaMultScale[rangeIdx] * runI + (1 << (QC_IN_RANGE_NUM_BITS - 1))) >> QC_IN_RANGE_NUM_BITS;
                p_DRAMapping->m_intChromaInvScaleLUT[ch][i] = chromaMultOffset[rangeIdx] + runS;
            }
        }
    }
}

void evce_buildDraLut(WCGDDRAControl *p_DRAMapping, BOOL useFixedPt)
{
    quantParamDRA accum, temp1, temp2, temp3;

    int l_maxInLumaCodeword = DRA_LUT_MAXSIZE - 1;
    int l_maxOutLumaCodeword = DRA_LUT_MAXSIZE - 1;

    for (int i = 0; i < DRA_LUT_MAXSIZE; i++)
    {
        p_DRAMapping->m_lumaScaleLUT[i] = 0;
        p_DRAMapping->m_lumaInvScaleLUT[i] = 0;
        p_DRAMapping->m_chromaScaleLUT[0][i] = p_DRAMapping->m_chromaScaleLUT[1][i] = 1.0;
        p_DRAMapping->m_chromaInvScaleLUT[0][i] = p_DRAMapping->m_chromaScaleLUT[1][i] = 1.0;
    }

    if (useFixedPt == TRUE) {
        quantParamDRA lumaScaleLUT[DRA_LUT_MAXSIZE];

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            int x = p_DRAMapping->m_atfInRanges[i];
            int y = p_DRAMapping->m_atfInRanges[i + 1];
            for (int j = x; j < y; j++)
            {
                prec_quantize_entry_i(&temp1, j, QC_IN_RANGE_NUM_BITS);
                prec_quantize_entry_d(&temp2, p_DRAMapping->m_atfDraOffsets[i], QC_OFFSET_NUMFBITS, 15);
                prec_quantize_entry_d(&temp3, p_DRAMapping->m_atfDraScales[i], QC_SCALE_NUMFBITS, 10);
                accum = prec_minus_entry(temp1, temp2);
                lumaScaleLUT[j] = prec_mult_entry(accum, temp3);
                setFracBits(&(lumaScaleLUT[j]), 0);
                p_DRAMapping->m_lumaScaleLUT[j] = (int)(getVal(&(lumaScaleLUT[j])));
                if (p_DRAMapping->m_lumaScaleLUT[j] > l_maxOutLumaCodeword)
                {
                    p_DRAMapping->m_lumaScaleLUT[j] = l_maxOutLumaCodeword;
                }

                prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfChromaDraScales[0][i], QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
                p_DRAMapping->m_chromaScaleLUT[0][j] = getVal(&temp1);
                prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfChromaDraScales[1][i], QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
                p_DRAMapping->m_chromaScaleLUT[1][j] = getVal(&temp1);
            }
        }

        for (int j = 0; j < p_DRAMapping->m_atfInRanges[0]; j++)
        {
            p_DRAMapping->m_chromaScaleLUT[0][j] = p_DRAMapping->m_chromaScaleLUT[0][p_DRAMapping->m_atfInRanges[0]];
            p_DRAMapping->m_chromaScaleLUT[1][j] = p_DRAMapping->m_chromaScaleLUT[1][p_DRAMapping->m_atfInRanges[0]];
        }

        for (int j = p_DRAMapping->m_atfInRanges[p_DRAMapping->m_atfNumRanges]; j < DRA_LUT_MAXSIZE; j++)
        {
            prec_quantize_entry_i(&temp1, j, QC_IN_RANGE_NUM_BITS);
            prec_quantize_entry_d(&temp2, p_DRAMapping->m_atfDraOffsets[p_DRAMapping->m_atfNumRanges - 1], QC_OFFSET_NUMFBITS, 15);
            prec_quantize_entry_d(&temp3, p_DRAMapping->m_atfDraScales[p_DRAMapping->m_atfNumRanges - 1], QC_SCALE_NUMFBITS, 10);
            accum = prec_minus_entry(temp1, temp2);
            lumaScaleLUT[j] = prec_mult_entry(accum, temp3);
            setFracBits(&(lumaScaleLUT[j]), 0);
            p_DRAMapping->m_lumaScaleLUT[j] = (int)getVal(&(lumaScaleLUT[j]));

            if (p_DRAMapping->m_lumaScaleLUT[j] > l_maxOutLumaCodeword)
            {
                p_DRAMapping->m_lumaScaleLUT[j] = l_maxOutLumaCodeword; 
            }

            prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfChromaDraScales[0][p_DRAMapping->m_atfNumRanges - 1], QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
            p_DRAMapping->m_chromaScaleLUT[0][j] = getVal(&temp1);
            prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfChromaDraScales[1][p_DRAMapping->m_atfNumRanges - 1], QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
            p_DRAMapping->m_chromaScaleLUT[1][j] = getVal(&temp1);
        }


        const int maxNumLutIndex = DRA_LUT_MAXSIZE;
        double tempChromaScaleLut[2][DRA_LUT_MAXSIZE];
        int *rangeUsedForScale = p_DRAMapping->m_atfInRanges;

        for (int i = 0; i < maxNumLutIndex; i++)
        {
            tempChromaScaleLut[0][i] = p_DRAMapping->m_chromaScaleLUT[0][i];
            tempChromaScaleLut[1][i] = p_DRAMapping->m_chromaScaleLUT[1][i];
        }
        for (int i = 1; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            int rangeMin = (int)((rangeUsedForScale[i - 1] + rangeUsedForScale[i - 1 + 1]) / 2 + 0.5);
            int rangeMax = (int)((rangeUsedForScale[i] + rangeUsedForScale[i + 1]) / 2 + 0.5);

            for (int j = rangeMin; j < rangeMax; j++)
            {
                for (int ch = 0; ch < 2; ch++)
                {
                    prec_quantize_entry_d(&temp1, p_DRAMapping->m_chromaScaleLUT[ch][rangeMin], QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
                    lshift(&temp1, QC_IN_RANGE_NUM_BITS);
                    prec_quantize_entry_d(&temp2, p_DRAMapping->m_chromaScaleLUT[ch][rangeMax], QC_SCALE_NUMFBITS, QC_SCALE_NUMFBITS);
                    lshift(&temp2, QC_IN_RANGE_NUM_BITS);

                    quantParamDRA accum1, accum2, accum3, accum4, accum5, accum6, accum7, accum8;
                    prec_quantize_entry_i(&accum1, j, 0);
                    prec_quantize_entry_i(&accum2, rangeMin, 0);
                    prec_quantize_entry_i(&accum3, rangeMax, 0);
                    accum4 = prec_minus_entry(accum1, accum2);
                    accum5 = prec_minus_entry(accum3, accum2);

                    accum6 = prec_minus_entry(temp2, temp1);
                    accum7 = prec_mult_entry(accum6, accum4);
                    accum8 = prec_divide_entry(accum7, accum5);
                    temp3 = prec_plus_entry(temp1, accum8);
                    setFracBits(&temp3, QC_SCALE_NUMFBITS);
                    tempChromaScaleLut[ch][j] = getVal(&temp3);
                }
            }
        }
        for (int i = 0; i < maxNumLutIndex; i++)
        {
            p_DRAMapping->m_chromaScaleLUT[0][i] = tempChromaScaleLut[0][i];
            p_DRAMapping->m_chromaScaleLUT[1][i] = tempChromaScaleLut[1][i];
        }

        //        Inverse DRA LUT        
        quantParamDRA invlumaScaleLUT[DRA_LUT_MAXSIZE];
        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            int x = (int)(p_DRAMapping->m_atfOutRanges[i] + 0.5);
            int y = (int)(p_DRAMapping->m_atfOutRanges[i + 1] + 0.5);

            x = x > DRA_LUT_MAXSIZE - 1 ? DRA_LUT_MAXSIZE - 1 : x;
            y = y > DRA_LUT_MAXSIZE ? DRA_LUT_MAXSIZE : y;

            for (int j = x; j < y; j++)
            {
                prec_quantize_entry_d(&temp1, j, QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS, 10);
                prec_quantize_entry_d(&temp2, p_DRAMapping->m_atfDraOffsets[i], QC_OFFSET_NUMFBITS, 15);
                prec_quantize_entry_d(&temp3, p_DRAMapping->m_atfDraScales[i], QC_SCALE_NUMFBITS, 10);

                accum = prec_divide_entry(temp1, temp3);
                invlumaScaleLUT[j] = prec_plus_entry(accum, temp2);
                setFracBits(&(invlumaScaleLUT[j]), 0);
                p_DRAMapping->m_lumaInvScaleLUT[j] = (int)getVal(&(invlumaScaleLUT[j]));

                // Clip LUT value to maximum allowed codeword
                if (p_DRAMapping->m_lumaInvScaleLUT[j] < 0)
                    p_DRAMapping->m_lumaInvScaleLUT[j] = 0;
                if (p_DRAMapping->m_lumaInvScaleLUT[j] > l_maxInLumaCodeword)
                {
                    p_DRAMapping->m_lumaInvScaleLUT[j] = l_maxInLumaCodeword; // Clip LUT to maximum allowed codeword
                }
            }
        }
        for (int j = (int)(p_DRAMapping->m_atfOutRanges[p_DRAMapping->m_atfNumRanges] + 0.5); j < DRA_LUT_MAXSIZE; j++)
        {
            // Extrapolate Luma LUT curve for pixel outside of the defined ranges with the  DRA param specified for last known range
            prec_quantize_entry_d(&temp1, j, QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS, 10);
            prec_quantize_entry_d(&temp2, p_DRAMapping->m_atfDraOffsets[p_DRAMapping->m_atfNumRanges - 1], QC_OFFSET_NUMFBITS, 15);
            prec_quantize_entry_d(&temp3, p_DRAMapping->m_atfDraScales[p_DRAMapping->m_atfNumRanges - 1], QC_SCALE_NUMFBITS, 10);

            accum = prec_divide_entry(temp1, temp3);
            invlumaScaleLUT[j] = prec_plus_entry(accum, temp2);
            setFracBits(&(invlumaScaleLUT[j]), 0);
            p_DRAMapping->m_lumaInvScaleLUT[j] = (int)getVal(&(invlumaScaleLUT[j]));

            // Clip LUT value to maximum allowed codeword
            if (p_DRAMapping->m_lumaInvScaleLUT[j] < 0)
                p_DRAMapping->m_lumaInvScaleLUT[j] = 0;
            if (p_DRAMapping->m_lumaInvScaleLUT[j] > l_maxInLumaCodeword)
            {
                p_DRAMapping->m_lumaInvScaleLUT[j] = l_maxInLumaCodeword; // Clip LUT to maximum allowed codeword
            }
        }
        for (int i = 0; i < maxNumLutIndex; i++)
        {
            for (int ch = 0; ch < 2; ch++)
            {
                prec_quantize_entry_d(&temp1, 1, QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS, 14);
                prec_quantize_entry_d(&temp2, p_DRAMapping->m_chromaScaleLUT[ch][p_DRAMapping->m_lumaInvScaleLUT[i]], QC_SCALE_NUMFBITS, 10);
                temp3 = prec_divide_entry(temp1, temp2);
                setFracBits(&temp3, QC_INVSCALE_NUMFBITS);
                p_DRAMapping->m_chromaInvScaleLUT[ch][i] = getVal(&temp3);
            }
        }
    }
    else
    {
        /*        Forward DRA LUT        */
        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            int x = p_DRAMapping->m_atfInRanges[i];
            int y = p_DRAMapping->m_atfInRanges[i + 1];
            for (int j = x; j < y; j++)
            {
                p_DRAMapping->m_lumaScaleLUT[j] = (int)((j - p_DRAMapping->m_atfDraOffsets[i]) * p_DRAMapping->m_atfDraScales[i] + 0.5);
                if (p_DRAMapping->m_lumaScaleLUT[j] > l_maxOutLumaCodeword)
                {
                    p_DRAMapping->m_lumaScaleLUT[j] = l_maxOutLumaCodeword;
                }
                p_DRAMapping->m_chromaScaleLUT[0][j] = p_DRAMapping->m_atfChromaDraScales[0][i];
                p_DRAMapping->m_chromaScaleLUT[1][j] = p_DRAMapping->m_atfChromaDraScales[1][i];
            }
        }
        for (int j = p_DRAMapping->m_atfInRanges[p_DRAMapping->m_atfNumRanges]; j < DRA_LUT_MAXSIZE; j++)
        {
            // Extrapolate Luma LUT curve for pixel outside of the defined ranges with the  DRA param specified for last known range
            p_DRAMapping->m_lumaScaleLUT[j] = (int)((j - p_DRAMapping->m_atfDraOffsets[p_DRAMapping->m_atfNumRanges - 1]) * p_DRAMapping->m_atfDraScales[p_DRAMapping->m_atfNumRanges - 1] + 0.5);
            // Clip LUT value to maximum allowed codeword
            if (p_DRAMapping->m_lumaScaleLUT[j] > l_maxOutLumaCodeword)
            {
                p_DRAMapping->m_lumaScaleLUT[j] = l_maxOutLumaCodeword; // Clip LUT to maximum allowed codeword
            }
            // Define DRA scale parameter for chroma sample whose associated Luma sample value is outside of the defined ranges with the  DRA scale param specified for last known luma range
            p_DRAMapping->m_chromaScaleLUT[0][j] = p_DRAMapping->m_atfChromaDraScales[0][p_DRAMapping->m_atfNumRanges - 1];
            p_DRAMapping->m_chromaScaleLUT[1][j] = p_DRAMapping->m_atfChromaDraScales[1][p_DRAMapping->m_atfNumRanges - 1];
        }

        const int maxNumLutIndex = DRA_LUT_MAXSIZE;
        int *rangeUsedForScale = p_DRAMapping->m_atfInRanges;
        double tempChromaScaleLut[2][DRA_LUT_MAXSIZE];
        for (int i = 0; i < maxNumLutIndex; i++)
        {
            tempChromaScaleLut[0][i] = p_DRAMapping->m_chromaScaleLUT[0][i];
            tempChromaScaleLut[1][i] = p_DRAMapping->m_chromaScaleLUT[1][i];
        }
        for (int i = 1; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            int rangeMin = (int)((rangeUsedForScale[i - 1] + rangeUsedForScale[i - 1 + 1]) / 2 + 0.5);
            int rangeMax = (int)((rangeUsedForScale[i] + rangeUsedForScale[i + 1]) / 2 + 0.5);

            for (int j = rangeMin; j < rangeMax; j++)
            {
                tempChromaScaleLut[0][j] = p_DRAMapping->m_chromaScaleLUT[0][rangeMin] +
                    (p_DRAMapping->m_chromaScaleLUT[0][rangeMax] - p_DRAMapping->m_chromaScaleLUT[0][rangeMin]) / (rangeMax - rangeMin) *  (j - rangeMin);
                tempChromaScaleLut[1][j] = p_DRAMapping->m_chromaScaleLUT[1][rangeMin] +
                    (p_DRAMapping->m_chromaScaleLUT[1][rangeMax] - p_DRAMapping->m_chromaScaleLUT[1][rangeMin]) / (rangeMax - rangeMin) *  (j - rangeMin);
            }
        }
        for (int i = 0; i < maxNumLutIndex; i++)
        {
            p_DRAMapping->m_chromaScaleLUT[0][i] = tempChromaScaleLut[0][i];
            p_DRAMapping->m_chromaScaleLUT[1][i] = tempChromaScaleLut[1][i];
        }

        /*        Inverse DRA LUT        */
        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            int x = (int)(p_DRAMapping->m_atfOutRanges[i] + 0.5);
            int y = (int)(p_DRAMapping->m_atfOutRanges[i + 1] + 0.5);

            x = x > DRA_LUT_MAXSIZE - 1 ? DRA_LUT_MAXSIZE - 1 : x;
            y = y > DRA_LUT_MAXSIZE ? DRA_LUT_MAXSIZE : y;

            for (int j = x; j < y; j++)
            {
                p_DRAMapping->m_lumaInvScaleLUT[j] = (int)(j / p_DRAMapping->m_atfDraScales[i] + p_DRAMapping->m_atfDraOffsets[i] + 0.5);

                // Clip LUT value to maximum allowed codeword
                if (p_DRAMapping->m_lumaInvScaleLUT[j] > l_maxInLumaCodeword)
                {
                    p_DRAMapping->m_lumaInvScaleLUT[j] = l_maxInLumaCodeword; // Clip LUT to maximum allowed codeword
                }

            }
        }
        for (int j = (int)(p_DRAMapping->m_atfOutRanges[p_DRAMapping->m_atfNumRanges] + 0.5); j < DRA_LUT_MAXSIZE; j++)
        {
            // Extrapolate Luma LUT curve for pixel outside of the defined ranges with the  DRA param specified for last known range
            p_DRAMapping->m_lumaInvScaleLUT[j] = (int)(j / p_DRAMapping->m_atfDraScales[p_DRAMapping->m_atfNumRanges - 1] + p_DRAMapping->m_atfDraOffsets[p_DRAMapping->m_atfNumRanges - 1] + 0.5);
            // Clip LUT value to maximum allowed codeword
            if (p_DRAMapping->m_lumaInvScaleLUT[j] > l_maxInLumaCodeword)
            {
                p_DRAMapping->m_lumaInvScaleLUT[j] = l_maxInLumaCodeword; // Clip LUT to maximum allowed codeword
            }
        }
        for (int i = 0; i < maxNumLutIndex; i++)
        {
            p_DRAMapping->m_chromaInvScaleLUT[0][i] = 1.0 / p_DRAMapping->m_chromaScaleLUT[0][p_DRAMapping->m_lumaInvScaleLUT[i]];
            p_DRAMapping->m_chromaInvScaleLUT[1][i] = 1.0 / p_DRAMapping->m_chromaScaleLUT[1][p_DRAMapping->m_lumaInvScaleLUT[i]];
        }

    }
}

void normalizeHistogramLUT(WCGDDRAControl *p_DRAMapping, int sdrFlag, int fullRangeFlag, BOOL useFixedPt) {

    if (sdrFlag == 1)
        return;

    int deltas[33];
    //-------- Normilize the scale to full range 0..1 --------//
    double SCALE_NORM = ((int)(100.0*p_DRAMapping->m_draHistNorm + 0.5)) / 100.0;


    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfDraScales[i] = p_DRAMapping->m_atfDraScales[i] / SCALE_NORM;
    }

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        deltas[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfInRanges[i];
    }

    if (useFixedPt == TRUE) {
        quantParamDRA accum, temp1, temp2, temp3;
        quantParamDRA outRanges[33];
        quantParamDRA draOffsets[32];
        prec_quantize_entry_i(&(outRanges[0]), 0, 1);
        for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
        {
            prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfDraScales[i - 1], QC_SCALE_NUMFBITS, 10);
            prec_quantize_entry_i(&temp2, deltas[i - 1], QC_IN_RANGE_NUM_BITS + 1);
            temp3 = prec_mult_entry(temp1, temp2);
            outRanges[i] = prec_plus_entry(outRanges[i - 1], temp3);
        }

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            prec_quantize_entry_d(&temp1, 1, QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS, 11);
            prec_quantize_entry_d(&temp2, p_DRAMapping->m_atfDraScales[i], QC_SCALE_NUMFBITS, 10);
            accum = prec_divide_entry(temp1, temp2);
            temp3 = prec_mult_entry(outRanges[i + 1], accum);

            prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfInRanges[i + 1], 0, QC_IN_RANGE_NUM_BITS);
            draOffsets[i] = prec_minus_entry(temp1, temp3);
            setFracBits(&(draOffsets[i]), QC_OFFSET_NUMFBITS);
        }

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges + 1; i++)
        {
            setFracBits(&(outRanges[i]), 0);
            p_DRAMapping->m_atfOutRanges[i] = getVal(&(outRanges[i]));
        }
        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            p_DRAMapping->m_atfDraOffsets[i] = getVal(&(draOffsets[i]));
        }
    }
    else
    {
        p_DRAMapping->m_atfOutRanges[0] = 0;
        for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
        {
            p_DRAMapping->m_atfOutRanges[i] = (int)(p_DRAMapping->m_atfOutRanges[i - 1] + p_DRAMapping->m_atfDraScales[i - 1] * deltas[i - 1] + 0.5);
            p_DRAMapping->m_atfOutRanges[i] = EVC_CLIP3(0.0, (double)DRA_LUT_MAXSIZE, p_DRAMapping->m_atfOutRanges[i]);
        }

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            p_DRAMapping->m_atfDraOffsets[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfOutRanges[i + 1] / p_DRAMapping->m_atfDraScales[i];
        }
    }
    return;
}

void evce_constructDra(WCGDDRAControl *p_DRAMapping, int sdrFlag, int fullRangeFlag, BOOL useFixedPt) {

    if (sdrFlag == 1)
        return;

    int deltas[33];
    //-------- Normilize the scale to full range 0..1 --------//
    double SCALE_NORM = (p_DRAMapping->m_atfOutRanges[p_DRAMapping->m_atfNumRanges] - p_DRAMapping->m_atfOutRanges[0]) / (p_DRAMapping->m_atfInRanges[p_DRAMapping->m_atfNumRanges] - p_DRAMapping->m_atfInRanges[0]);
    // Handle special case of SDR content in PQ2100 format, stretch LUT to full budeget of codewords
    if (sdrFlag == 1) // SDR config
    {
        SCALE_NORM /= 2;
    }


    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfDraScales[i] = p_DRAMapping->m_atfDraScales[i] / SCALE_NORM;
    }

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        deltas[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfInRanges[i];
    }
    if (useFixedPt == TRUE) {
        quantParamDRA accum, temp1, temp2, temp3;
        quantParamDRA outRanges[33];
        quantParamDRA draOffsets[32];
        prec_quantize_entry_i(&(outRanges[0]), 0, 1);
        for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
        {
            prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfDraScales[i - 1], QC_SCALE_NUMFBITS, 10);
            prec_quantize_entry_i(&temp2, deltas[i - 1], QC_IN_RANGE_NUM_BITS + 1);
            temp3 = prec_mult_entry(temp1, temp2);
            outRanges[i] = prec_plus_entry(outRanges[i - 1], temp3);
        }

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            prec_quantize_entry_d(&temp1, 1, QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS, 11);
            prec_quantize_entry_d(&temp2, p_DRAMapping->m_atfDraScales[i], QC_SCALE_NUMFBITS, 10);
            accum = prec_divide_entry(temp1, temp2);
            temp3 = prec_mult_entry(outRanges[i + 1], accum);

            prec_quantize_entry_d(&temp1, p_DRAMapping->m_atfInRanges[i + 1], 0, QC_IN_RANGE_NUM_BITS);
            draOffsets[i] = prec_minus_entry(temp1, temp3);
            setFracBits(&(draOffsets[i]), QC_OFFSET_NUMFBITS);
        }

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges + 1; i++)
        {
            setFracBits(&(outRanges[i]), 0);
            p_DRAMapping->m_atfOutRanges[i] = getVal(&(outRanges[i]));
        }
        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            p_DRAMapping->m_atfDraOffsets[i] = getVal(&(draOffsets[i]));
        }
    }
    else
    {
        p_DRAMapping->m_atfOutRanges[0] = 0;
        for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
        {
            p_DRAMapping->m_atfOutRanges[i] = (int)(p_DRAMapping->m_atfOutRanges[i - 1] + p_DRAMapping->m_atfDraScales[i - 1] * deltas[i - 1] + 0.5);
            p_DRAMapping->m_atfOutRanges[i] = EVC_CLIP3(0.0, (double)DRA_LUT_MAXSIZE, p_DRAMapping->m_atfOutRanges[i]);
        }

        for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
        {
            p_DRAMapping->m_atfDraOffsets[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfOutRanges[i + 1] / p_DRAMapping->m_atfDraScales[i];
        }
    }
    return;
}

void checkEqualRangeFlag(WCGDDRAControl *p_DRAMapping)
{
    SignalledParamsDRA * l_signalledDRA = &(p_DRAMapping->m_signalledDRA);
    BOOL retValFlag = TRUE;
    for (int i = 1; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        if (p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfInRanges[i] != p_DRAMapping->m_atfInRanges[i] - p_DRAMapping->m_atfInRanges[i - 1])
        { // If one 
            retValFlag = FALSE;
            break;
        }
    }
    if (retValFlag == TRUE)
    {
        l_signalledDRA->m_equalRangesFlag = TRUE;
        l_signalledDRA->m_inRanges[0] = p_DRAMapping->m_atfInRanges[0];

        int deltaVal = (int)floor((1024 - p_DRAMapping->m_atfInRanges[0] + 0.0) / p_DRAMapping->m_atfNumRanges + 0.5);
        l_signalledDRA->m_deltaVal = deltaVal - (p_DRAMapping->m_atfInRanges[1] - p_DRAMapping->m_atfInRanges[0]);
    }
    else
    {
        l_signalledDRA->m_equalRangesFlag = FALSE;
        for (int i = 0; i <= p_DRAMapping->m_atfNumRanges; i++)
        {
            l_signalledDRA->m_inRanges[i] = p_DRAMapping->m_atfInRanges[i];
        }
    }
}

void quatnizeParamsDRA(WCGDDRAControl *p_DRAMapping)
{

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfDraScales[i] = EVC_CLIP3(0, (1 << p_DRAMapping->m_numIntBitsScale), p_DRAMapping->m_atfDraScales[i]);
        p_DRAMapping->m_atfIntDraScales[i] = (int)(p_DRAMapping->m_atfDraScales[i] * (1 << p_DRAMapping->m_numFracBitsScale) + 0.5);
        p_DRAMapping->m_atfDraScales[i] = (double)p_DRAMapping->m_atfIntDraScales[i] / (1 << p_DRAMapping->m_numFracBitsScale);
    }
    p_DRAMapping->m_scaleCbDRA = EVC_CLIP3(0, 1 << p_DRAMapping->m_numIntBitsScale, p_DRAMapping->m_scaleCbDRA);
    p_DRAMapping->m_intScaleCbDRA = (int)(p_DRAMapping->m_scaleCbDRA * (1 << p_DRAMapping->m_numFracBitsScale) + 0.5);
    p_DRAMapping->m_scaleCbDRA = (double)p_DRAMapping->m_intScaleCbDRA / (1 << p_DRAMapping->m_numFracBitsScale);

    p_DRAMapping->m_scaleCrDRA = EVC_CLIP3(0, 1 << p_DRAMapping->m_numIntBitsScale, p_DRAMapping->m_scaleCrDRA);
    p_DRAMapping->m_intScaleCrDRA = (int)(p_DRAMapping->m_scaleCrDRA * (1 << p_DRAMapping->m_numFracBitsScale) + 0.5);
    p_DRAMapping->m_scaleCrDRA = (double)p_DRAMapping->m_intScaleCrDRA / (1 << p_DRAMapping->m_numFracBitsScale);

}

void setSignalledParamsDRA(WCGDDRAControl *p_DRAMapping)
{
    p_DRAMapping->m_signalledDRA.m_signal_dra_flag = p_DRAMapping->m_flagEnabled;
    int numPivotPoints = p_DRAMapping->m_atfNumRanges + 1;
    p_DRAMapping->m_signalledDRA.m_numFracBitsScale = p_DRAMapping->m_numFracBitsScale;
    p_DRAMapping->m_signalledDRA.m_numIntBitsScale = p_DRAMapping->m_numIntBitsScale;

    p_DRAMapping->m_signalledDRA.m_baseLumaQP = p_DRAMapping->m_chromaQPModel.m_baseLumaQP;
    p_DRAMapping->m_signalledDRA.m_numRanges = p_DRAMapping->m_atfNumRanges;
    for (int i = 0; i < numPivotPoints; i++)
    {
        p_DRAMapping->m_signalledDRA.m_inRanges[i] = p_DRAMapping->m_atfInRanges[i];
    }
    for (int i = 0; i < numPivotPoints; i++)
    {
        p_DRAMapping->m_signalledDRA.m_intDraScales[i] = p_DRAMapping->m_atfIntDraScales[i];
    }
    assert(QC_SCALE_NUMFBITS >= p_DRAMapping->m_numFracBitsScale); 
    p_DRAMapping->m_signalledDRA.m_intScaleCbDRA = p_DRAMapping->m_intScaleCbDRA >> (QC_SCALE_NUMFBITS - p_DRAMapping->m_numFracBitsScale);
    p_DRAMapping->m_signalledDRA.m_intScaleCrDRA = p_DRAMapping->m_intScaleCrDRA >> (QC_SCALE_NUMFBITS - p_DRAMapping->m_numFracBitsScale);

    checkEqualRangeFlag(p_DRAMapping);
}

void evce_updateDRA(WCGDDRAControl *p_DRAMapping, int sdrFlag) {

    evce_constructDra(p_DRAMapping, sdrFlag, 0, FALSE); 
    evce_zoomInRangeLUT(p_DRAMapping, FALSE);
    normalizeHistogramLUT(p_DRAMapping, 0, 1, TRUE);
    quatnizeParamsDRA(p_DRAMapping);
    setSignalledParamsDRA(p_DRAMapping);

    // Produce forward DRA 
    evce_compensateChromaShiftTable(p_DRAMapping);
    evce_buildDraLut(p_DRAMapping, TRUE);

    // Produce inverse DRA from signalled parameters
    evcd_initDRA(p_DRAMapping);
    for (int ch = 0; ch < 2; ch++)
    {
        for (int i = 0; i < DRA_LUT_MAXSIZE; i++)
        {
            int value1 = 1 << (QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS);
            int value3 = p_DRAMapping->m_intChromaInvScaleLUT[ch][p_DRAMapping->m_lumaScaleLUT[i]];
            int temp = (int)(value1 + (value3 / 2)) / value3;
            p_DRAMapping->m_intChromaScaleLUT[ch][i] = temp;
        }
    }
    return;
}

BOOL evce_analyzeInputPic(WCGDDRAControl *p_DRAMapping) {

    p_DRAMapping->m_globalOffset = 64;
    p_DRAMapping->m_globalEnd = 940;
    evce_updateDRA(p_DRAMapping, 0);

    return TRUE;
}

void evce_initDRA(WCGDDRAControl *p_DRAMapping, int totalChangePoints, int *lumaChangePoints, int* qps) {

    p_DRAMapping->m_flagEnabled = TRUE;
    p_DRAMapping->m_scaleCbDRA = evce_getCbScaleDRA(&(p_DRAMapping->m_chromaQPModel), p_DRAMapping->m_chromaQPModel.m_baseLumaQP);
    p_DRAMapping->m_scaleCrDRA = evce_getCrScaleDRA(&(p_DRAMapping->m_chromaQPModel), p_DRAMapping->m_chromaQPModel.m_baseLumaQP);

    int configID = 0; // 0: HDR, 1: SDR

    p_DRAMapping->m_globalOffset = 0;
    totalChangePoints = p_DRAMapping->m_atfNumRanges + 1;
    int totalNumberRanges = p_DRAMapping->m_atfNumRanges;
    double scales[32];
    int inRanges[33];
    double outRanges[33];
    int deltas[33];

    for (int i = 0; i < DRA_LUT_MAXSIZE; i++)
    {
        p_DRAMapping->m_lumaInvScaleLUT[i] = i;
        p_DRAMapping->m_lumaScaleLUT[i] = i;
    };

    for (int i = 0; i < totalChangePoints; i++)
    {
        scales[i] = (p_DRAMapping->m_draScaleMap.m_draScaleMapY[i][1]);
        inRanges[i] = (int)(p_DRAMapping->m_draScaleMap.m_draScaleMapY[i][0]);
    }


    for (int i = 0; i < totalNumberRanges; i++)
    {
        deltas[i] = inRanges[i + 1] - inRanges[i];
    }

    outRanges[0] = 0;
    for (int i = 1; i < totalChangePoints; i++)
    {
        outRanges[i] = (int)(outRanges[i - 1] + scales[i - 1] * deltas[i - 1] + 0.5);
    }

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfDraScales[i] = scales[i];
        p_DRAMapping->m_atfInRanges[i] = inRanges[i];
        p_DRAMapping->m_atfOutRanges[i] = outRanges[i];
    }
    p_DRAMapping->m_atfInRanges[p_DRAMapping->m_atfNumRanges] = inRanges[p_DRAMapping->m_atfNumRanges];
    p_DRAMapping->m_atfOutRanges[p_DRAMapping->m_atfNumRanges] = outRanges[p_DRAMapping->m_atfNumRanges];

    evce_constructDra(p_DRAMapping, configID, 1, TRUE); 

    return;
}

 /* Decoder side functions are listed below: */
void evcd_getSignalledParamsDRA(WCGDDRAControl *p_DRAMapping)
{
    p_DRAMapping->m_flagEnabled = p_DRAMapping->m_signalledDRA.m_signal_dra_flag;
    p_DRAMapping->m_chromaQPModel.m_baseLumaQP = p_DRAMapping->m_signalledDRA.m_baseLumaQP;
    p_DRAMapping->m_atfNumRanges = p_DRAMapping->m_signalledDRA.m_numRanges;
    p_DRAMapping->m_numFracBitsScale = p_DRAMapping->m_signalledDRA.m_numFracBitsScale;
    p_DRAMapping->m_numIntBitsScale = p_DRAMapping->m_signalledDRA.m_numIntBitsScale;

    p_DRAMapping->m_intScaleCbDRA = p_DRAMapping->m_signalledDRA.m_intScaleCbDRA;
    p_DRAMapping->m_intScaleCrDRA = p_DRAMapping->m_signalledDRA.m_intScaleCrDRA;
    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfIntDraScales[i] = p_DRAMapping->m_signalledDRA.m_intDraScales[i];
    }
    for (int i = 0; i <= p_DRAMapping->m_atfNumRanges; i++)
    {
        p_DRAMapping->m_atfInRanges[i] = p_DRAMapping->m_signalledDRA.m_inRanges[i];
    }
}

void evcd_constructDra(WCGDDRAControl *p_DRAMapping)
{
    int numFracBits = p_DRAMapping->m_numFracBitsScale;
    int NUM_MULT_BITS = QC_SCALE_NUMFBITS + QC_INVSCALE_NUMFBITS;
    int deltas[33];

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        deltas[i] = p_DRAMapping->m_atfInRanges[i + 1] - p_DRAMapping->m_atfInRanges[i];
    }

    p_DRAMapping->m_atfIntOutRanges[0] = 0;
    for (int i = 1; i < p_DRAMapping->m_atfNumRanges + 1; i++)
    {
        p_DRAMapping->m_atfIntOutRanges[i] = p_DRAMapping->m_atfIntOutRanges[i - 1] + deltas[i - 1] * p_DRAMapping->m_atfIntDraScales[i - 1];
    }

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges; i++)
    {
        int invScale2;
        invScale2 = (int)(((1 << (NUM_MULT_BITS + 1))) / p_DRAMapping->m_atfIntDraScales[i] + 1) >> 1;
        p_DRAMapping->m_atfIntInvDraScales[i] = invScale2;
        int invScale3 = (int)(((1 << (10 + NUM_MULT_BITS))) / p_DRAMapping->m_atfIntDraScales[i]); 
        p_DRAMapping->m_atfIntInvDraScales[i] = invScale3;

        int diffVal2 = p_DRAMapping->m_atfIntOutRanges[i + 1] * invScale2; 
        p_DRAMapping->m_atfIntInvDraOffsets[i] = ((p_DRAMapping->m_atfInRanges[i + 1] << NUM_MULT_BITS) - diffVal2 + (1 << 10)) >> 11;
    }

    for (int i = 0; i < p_DRAMapping->m_atfNumRanges + 1; i++)
    {
        p_DRAMapping->m_atfIntOutRanges[i] = (p_DRAMapping->m_atfIntOutRanges[i] + (1 << (numFracBits - 1))) >> numFracBits;
    }
    return;
}
void evcd_initDRA(WCGDDRAControl *p_DRAMapping) {
    evcd_getSignalledParamsDRA(p_DRAMapping);
    evcd_constructDra(p_DRAMapping); 
    evc_compensateChromaShiftTableInt(p_DRAMapping); 
    evc_buildDraLumaLut2(p_DRAMapping); 
    evc_buildDraChromaLut(p_DRAMapping); 
}

void evc_apply_dra_luma_plane(EVC_IMGB * dst, EVC_IMGB * src, WCGDDRAControl *p_DRAMapping, int planeId, int backwardMap)
{
    short* srcPlane;
    short* dstPlane;
    short srcValue, dstValue;
    int i, k, j;

    for (i = planeId; i <= planeId; i++)
    {
        srcPlane = (short*)src->a[i];
        dstPlane = (short*)dst->a[i];
        for (j = 0; j < src->h[i]; j++)
        {
            for (k = 0; k < src->w[i]; k++)
            {
                srcValue = srcPlane[k];

                dstValue = dstPlane[k];
                if (backwardMap == TRUE)
                    dstValue = p_DRAMapping->m_lumaInvScaleLUT[srcValue];
                else
                    dstValue = p_DRAMapping->m_lumaScaleLUT[srcValue];
                dstPlane[k] = dstValue;
            }
            srcPlane = (short*)((unsigned char *)srcPlane + src->s[i]);
            dstPlane = (short*)((unsigned char *)dstPlane + dst->s[i]);
        }
    }
}

void evc_apply_dra_chroma_plane(EVC_IMGB * dst, EVC_IMGB * src, WCGDDRAControl *p_DRAMapping, int planeId, int backwardMap)
{
    int roundOffset = 1 << (QC_INVSCALE_NUMFBITS - 1);
    int offsetValue = 0;
    int intScale = 1;
    double scale = 0;
    int width = 0;
    int height = 0;

    short* refPlane;
    short* srcPlane;
    short* dstPlane;
    short refValue, srcValue, dstValue;
    int i, k, j;
    int cShift = (planeId == 0) ? 0 : 1;

    for (i = planeId; i <= planeId; i++)
    {
        refPlane = (short*)src->a[0]; //luma reference
        srcPlane = (short*)src->a[i];
        dstPlane = (short*)dst->a[i];
        width = src->w[i];
        height = src->h[i];

        for (j = 0; j < src->h[i]; j++)
        {
            for (k = 0; k < src->w[i]; k++)
            {
                refValue = refPlane[k << cShift];
                refValue = (refValue < 0) ? 0 : refValue;
                srcValue = srcPlane[k];
                dstValue = dstPlane[k];
                srcValue = srcValue - 512;
                offsetValue = srcValue;
                if (backwardMap == TRUE)
                    intScale = (p_DRAMapping->m_intChromaInvScaleLUT[i - 1][refValue]);
                else
                    intScale = (p_DRAMapping->m_intChromaScaleLUT[i - 1][refValue]);
                if (srcValue < 0)
                {
                    offsetValue *= -1;
                }
                offsetValue = (offsetValue * intScale + roundOffset) >> QC_INVSCALE_NUMFBITS;
                if (srcValue < 0)
                {
                    offsetValue *= -1;
                }
                dstValue = 512 + offsetValue;

                dstPlane[k] = dstValue;
            }
            refPlane = (short*)((unsigned char *)refPlane + (dst->s[0] << cShift));
            srcPlane = (short*)((unsigned char *)srcPlane + src->s[i]);
            dstPlane = (short*)((unsigned char *)dstPlane + dst->s[i]);
        }
    }
}

void evc_resetApsGenReadBuffer(EVC_APS_GEN *p_aps_gen_array) {
    p_aps_gen_array->aps_type_id = 0; // ALF
    p_aps_gen_array->aps_id = -1;
    p_aps_gen_array->signal_flag = 0;
    (p_aps_gen_array + 1)->aps_type_id = 1; // DRA
    (p_aps_gen_array + 1)->aps_id = -1;
    (p_aps_gen_array + 1)->signal_flag = 0;
}

void evc_addDraApsToBuffer(SignalledParamsDRA* p_g_dra_control_array, EVC_APS_GEN *p_aps_gen_array) {
    int dra_id = (p_aps_gen_array + 1)->aps_id;
    assert((dra_id >-2) && (dra_id < 31));
    if (dra_id != -1)
    {
        SignalledParamsDRA* pDraBuffer = p_g_dra_control_array + dra_id;
        SignalledParamsDRA* pDraSrc = (SignalledParamsDRA*)((p_aps_gen_array + 1)->aps_data);
        if (pDraBuffer->m_signal_dra_flag == -1)
        {
            memcpy(pDraBuffer, pDraSrc, sizeof(SignalledParamsDRA));
            (p_aps_gen_array + 1)->aps_id = -1;
        }
        else {
            printf("New DRA APS information ignored. APS ID was used earlier, new APS entity must contain identical content.\n");
        }
    }
}

#endif