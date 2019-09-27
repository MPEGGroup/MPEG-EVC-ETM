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

#ifndef _EVCE_ECO_H_
#define _EVCE_ECO_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "evce_def.h"

#define GET_SBAC_ENC(bs)   ((EVCE_SBAC *)(bs)->pdata[1])

int evce_eco_nalu(EVC_BSW * bs, EVC_NALU * nalu);
int evce_eco_sps(EVC_BSW * bs, EVC_SPS * sps);
int evce_eco_pps(EVC_BSW * bs, EVC_SPS * sps, EVC_PPS * pps);
#if ALF_PARAMETER_APS
int evce_eco_aps(EVC_BSW * bs, EVC_APS * aps);
#endif
int evce_eco_sh(EVC_BSW * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_SH * sh);
int evce_eco_udata(EVCE_CTX * ctx, EVC_BSW * bs);
int evce_eco_pred_mode(EVC_BSW * bs, u8 pred_mode, int ctx);
#if IBC
int evce_eco_ibc(EVC_BSW * bs, u8 pred_mode_ibc_flag, int ctx);
#endif
int evce_eco_mvd(EVC_BSW * bs, s16 mvd[MV_D]);
void evce_sbac_reset(EVCE_SBAC * sbac, u8 slice_type, u8 slice_qp, int sps_cm_init_flag);
void evce_sbac_finish(EVC_BSW *bs);
void evce_sbac_encode_bin(u32 bin, EVCE_SBAC *sbac, SBAC_CTX_MODEL *ctx_model, EVC_BSW *bs);
void evce_sbac_encode_bin_trm(u32 bin, EVCE_SBAC *sbac, EVC_BSW *bs);
#if DQP
int evce_eco_dqp(EVC_BSW * bs, int ref_qp, int cur_qp);
#endif
int evce_eco_coef(EVC_BSW * bs, s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 pred_mode, int nnz_sub[N_C][MAX_SUB_TB_NUM], int b_no_cbf, int run_stats
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
                  , int tool_ats
#endif
#if ATS_INTRA_PROCESS
                  , u8 ats_intra_cu, u8 ats_tu
#endif
#if ATS_INTER_PROCESS
                  , u8 ats_inter_info
#endif
#if ADCC || DQP
                  , EVCE_CTX * ctx
#endif
#if DQP
                  , EVCE_CORE * core, int enc_dqp, u8 cur_qp
#endif
);
int evce_eco_unit(EVCE_CTX * ctx, EVCE_CORE * core, int x, int y, int cup, int cuw, int cuh);
int evce_eco_split_mode(EVC_BSW *bs, EVCE_CTX *c, EVCE_CORE *core, int cud, int cup, int cuw, int cuh, int lcu_s
                        , const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int* curr_split_allow, int qt_depth, int btt_depth, int x, int y);
int evce_eco_suco_flag(EVC_BSW *bs, EVCE_CTX *c, EVCE_CORE *core, int cud, int cup, int cuw, int cuh, int lcu_s, s8 split_mode, int boundary, u8 log2_max_cuwh);
void evce_eco_mmvd_flag(EVC_BSW * bs, int flag);
int evce_eco_mmvd_info(EVC_BSW *bs, int mvp_idx, int type);
void evce_eco_slice_end_flag(EVC_BSW * bs, int flag);
int evce_eco_mvp_idx(EVC_BSW *bs, int mvp_idx, int sps_amis_flag);
int evce_eco_affine_mvp_idx(EVC_BSW *bs, int mvp_idx);
int evce_eco_mvd(EVC_BSW *bs, s16 mvd[MV_D]);
int evce_eco_refi(EVC_BSW * bs, int num_refp, int refi);
void evce_eco_inter_dir(EVC_BSW * bs, s8 refi[REFP_NUM]);
void evce_eco_inter_t_direct(EVC_BSW *bs, int t_direct_flag);
//! \todo Change list of arguments
void evce_eco_xcoef(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type
#if ADCC  
                    , int tool_adcc
#endif
);
//! \todo Change list of arguments
int evce_eco_intra_dir_b(EVC_BSW *bs, u8 ipm, u8 * mpm, u8 mpm_ext[8], u8 pims[IPD_CNT]);
int evce_eco_intra_dir(EVC_BSW *bs, u8 ipm, u8 mpm[2], u8 mpm_ext[8], u8 pims[IPD_CNT]);
int evce_eco_intra_dir_c(EVC_BSW *bs, u8 ipm, u8 ipm_l);
int evce_eco_mvr_idx(EVC_BSW *bs, u8 mvr_idx);
int evce_eco_bi_idx(EVC_BSW * bs, u8 bi_idx);
#if AFFINE
void evce_eco_affine_flag(EVC_BSW * bs, int flag, int ctx);
void evce_eco_affine_mode(EVC_BSW * bs, int flag);
int evce_eco_affine_mrg_idx(EVC_BSW *bs, s16 affine_mrg_idx);
void evce_eco_affine_mvd_flag(EVC_BSW *bs, int flag, int refi);
#endif
#if ALF
void setAlfFilterShape(evc_AlfFilterShape *  alfShape, int shapeSize);
int evc_lengthGolomb(int coeffVal, int k);
int evc_getGolombKMin(evc_AlfFilterShape *  alfShape, int numFilters, int *kMinTab, int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB]);
void evc_alfGolombEncode(EVC_BSW * bs, int coeff, int kMinTab);
#if ALF_PARAMETER_APS
int evce_eco_alf_aps_param(EVC_BSW * bs, EVC_APS * aps);
#endif
int evce_eco_alf_sh_param(EVC_BSW * bs, EVC_SH * sh);
#endif
#ifdef __cplusplus
}
#endif
#endif /* _EVCE_ECO_H_ */
