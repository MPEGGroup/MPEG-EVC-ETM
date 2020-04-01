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

#ifndef _EVC_TBL_H_
#define _EVC_TBL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "evc_def.h"
extern const u8 evc_tbl_split_flag_ctx[6][6];
extern const u8 evc_tbl_log2[257];
extern const s8 evc_tbl_tm2[2][2];
extern const s8 evc_tbl_tm4[4][4];
extern const s8 evc_tbl_tm8[8][8];
extern const s8 evc_tbl_tm16[16][16];
extern const s8 evc_tbl_tm32[32][32];
extern const s8 evc_tbl_tm64[64][64];
extern const s8 *evc_tbl_tm[MAX_CU_DEPTH];
extern int evc_scan_sr[MAX_TR_SIZE*MAX_TR_SIZE];
extern int evc_inv_scan_sr[MAX_TR_SIZE*MAX_TR_SIZE];
extern u16 * evc_inv_scan_tbl[COEF_SCAN_TYPE_NUM][MAX_CU_LOG2 - 1][MAX_CU_LOG2 - 1];
extern u16 * evc_scan_tbl[COEF_SCAN_TYPE_NUM][MAX_CU_LOG2 - 1][MAX_CU_LOG2 - 1];
extern const u8 evey_tbl_mpm[6][6][5];

extern const int evc_tbl_dq_scale[6];
extern const int evc_tbl_dq_scale_b[6];
extern const u8 evc_tbl_df_st[4][52];

extern const int evc_tbl_ipred_adi[32][4];
extern const int evc_tbl_ipred_dxdy[IPD_CNT][2];

extern u16 evc_split_tbl[SPLIT_CHECK_NUM][2];
extern const u8  evc_split_order[2][SPLIT_CHECK_NUM];

extern int evc_tbl_qp_chroma_ajudst_main[MAX_QP_TABLE_SIZE];
extern int evc_tbl_qp_chroma_ajudst_base[MAX_QP_TABLE_SIZE];
extern int* evc_tbl_qp_chroma_ajudst;
extern int evc_tbl_qp_chroma_dynamic_ext[2][MAX_QP_TABLE_SIZE_EXT];
extern int* p_evc_tbl_qp_chroma_dynamic_ext[2];// pointer to [0th position in evc_tbl_qp_chroma_dynamic_ext]
extern int* p_evc_tbl_qp_chroma_dynamic[2];    // pointer to [12th position in evc_tbl_qp_chroma_dynamic_ext]
void evc_derived_chroma_qp_mapping_tables(EVC_CHROMA_TABLE *structChromaQP);

extern const s16 init_cbf_luma[2][NUM_CTX_CBF_LUMA];
extern const s16 init_cbf_cb[2][NUM_CTX_CBF_CR];
extern const s16 init_cbf_cr[2][NUM_CTX_CBF_CB];
extern const s16 init_cbf_all[2][NUM_CTX_CBF_ALL];
#if DQP
extern const s16 init_dqp[2][NUM_CTX_DELTA_QP];
#endif
extern const s16 init_pred_mode[2][NUM_CTX_PRED_MODE];
#if M50761_CHROMA_NOT_SPLIT
extern const s16 init_mode_cons[2][NUM_CTX_MODE_CONS];
#endif
extern const s16 init_direct_mode_flag[2][NUM_CTX_DIRECT_MODE_FLAG];
extern const s16 init_merge_mode_flag[2][NUM_CTX_MERGE_MODE_FLAG];
extern const s16 init_inter_dir[2][NUM_CTX_INTER_PRED_IDC];
extern const s16 init_intra_dir[2][NUM_CTX_INTRA_PRED_MODE];
extern const s16 init_intra_luma_pred_mpm_flag[2][NUM_CTX_INTRA_LUMA_PRED_MPM_FLAG];
extern const s16 init_intra_luma_pred_mpm_idx[2][NUM_CTX_INTRA_LUMA_PRED_MPM_IDX];
extern const s16 init_intra_chroma_pred_mode[2][NUM_CTX_INTRA_CHROMA_PRED_MODE];
extern const s16 init_mmvd_flag[2][NUM_CTX_MMVD_FLAG];
extern const s16 init_mmvd_merge_idx[2][NUM_CTX_MMVD_MERGE_IDX];
extern const s16 init_mmvd_distance_idx[2][NUM_CTX_MMVD_DIST_IDX];
extern const s16 init_mmvd_direction_idx[2][NUM_CTX_MMVD_DIRECTION_IDX];
extern const s16 init_mmvd_group_idx[2][NUM_CTX_MMVD_GROUP_IDX];
extern const s16 init_merge_idx[2][NUM_CTX_MERGE_IDX];
extern const s16 init_mvp_idx[2][NUM_CTX_MVP_IDX];
extern const s16 init_affine_mvp_idx[2][NUM_CTX_AFFINE_MVP_IDX];
extern const s16 init_mvr_idx[2][NUM_CTX_AMVR_IDX];
extern const s16 init_bi_idx[2][NUM_CTX_BI_PRED_IDX];
extern const s16 init_mvd[2][NUM_CTX_MVD];
extern const s16 init_refi[2][NUM_CTX_REF_IDX];
extern const s16 init_btt_split_flag[2][NUM_CTX_BTT_SPLIT_FLAG];
extern const s16 init_btt_split_dir[2][NUM_CTX_BTT_SPLIT_DIR];
extern const s16 init_btt_split_type[2][NUM_CTX_BTT_SPLIT_TYPE];
extern const s16 init_run[2][NUM_CTX_CC_RUN];
extern const s16 init_last[2][NUM_CTX_CC_LAST];
extern const s16 init_level[2][NUM_CTX_CC_LEVEL];
extern const s16 init_suco_flag[2][NUM_CTX_SUCO_FLAG];
extern const s16 init_alf_ctb_flag[2][NUM_CTX_ALF_CTB_FLAG];
extern const s16 init_split_cu_flag[2][NUM_CTX_SPLIT_CU_FLAG];
extern const s16 init_sig_coeff_flag[2][NUM_CTX_SIG_COEFF_FLAG];
extern const s16 init_coeff_abs_level_greaterAB_flag[2][NUM_CTX_GTX];
extern const s16 init_last_sig_coeff_x_prefix[2][NUM_CTX_LAST_SIG_COEFF];
extern const s16 init_last_sig_coeff_y_prefix[2][NUM_CTX_LAST_SIG_COEFF];
extern const s16 init_affine_flag[2][NUM_CTX_AFFINE_FLAG];
extern const s16 init_affine_mode[2][NUM_CTX_AFFINE_MODE];
extern const s16 init_affine_mrg[2][NUM_CTX_AFFINE_MRG];
extern const s16 init_affine_mvd_flag[2][NUM_CTX_AFFINE_MVD_FLAG];
extern const s16 init_skip_flag[2][NUM_CTX_SKIP_FLAG];
extern const s16 init_ats_intra_cu[2][NUM_CTX_ATS_INTRA_CU_FLAG];
extern const s16 init_ibc_flag[2][NUM_CTX_IBC_FLAG];
extern const s16 init_ats_mode[2][NUM_CTX_ATS_MODE_FLAG];
extern const s16 init_ats_cu_inter_flag[2][NUM_CTX_ATS_INTER_FLAG];
extern const s16 init_ats_cu_inter_quad_flag[2][NUM_CTX_ATS_INTER_QUAD_FLAG];
extern const s16 init_ats_cu_inter_hor_flag[2][NUM_CTX_ATS_INTER_HOR_FLAG];
extern const s16 init_ats_cu_inter_pos_flag[2][NUM_CTX_ATS_INTER_POS_FLAG];

extern s16 evc_tbl_tr2[NUM_TRANS_TYPE][2][2];
extern s16 evc_tbl_tr4[NUM_TRANS_TYPE][4][4];
extern s16 evc_tbl_tr8[NUM_TRANS_TYPE][8][8];
extern s16 evc_tbl_tr16[NUM_TRANS_TYPE][16][16];
extern s16 evc_tbl_tr32[NUM_TRANS_TYPE][32][32];
extern s16 evc_tbl_tr64[NUM_TRANS_TYPE][64][64];
extern s16 evc_tbl_tr128[NUM_TRANS_TYPE][128][128];

extern int evc_tbl_tr_subset_intra[4];

extern s16 evc_tbl_inv_tr2[NUM_TRANS_TYPE][2][2];
extern s16 evc_tbl_inv_tr4[NUM_TRANS_TYPE][4][4];
extern s16 evc_tbl_inv_tr8[NUM_TRANS_TYPE][8][8];
extern s16 evc_tbl_inv_tr16[NUM_TRANS_TYPE][16][16];
extern s16 evc_tbl_inv_tr32[NUM_TRANS_TYPE][32][32];
extern s16 evc_tbl_inv_tr64[NUM_TRANS_TYPE][64][64];
extern s16 evc_tbl_inv_tr128[NUM_TRANS_TYPE][128][128];

#if ADDB_FLAG_FIX
extern const u8 ALPHA_TABLE[52];
extern const u8 BETA_TABLE[52];
extern const u8 CLIP_TAB[52][5];
#endif

extern const int g_min_in_group[LAST_SIGNIFICANT_GROUPS];
extern const int g_group_idx[MAX_TR_SIZE];
extern const int g_go_rice_range[MAX_GR_ORDER_RESIDUAL];
extern const int g_go_rice_para_coeff[32];

#if HDR_MD5_CHECK
extern int g_lumaInvScaleLUT[DRA_LUT_MAXSIZE];               // LUT for luma and correspionding QP offset
extern double g_chromaInvScaleLUT[2][DRA_LUT_MAXSIZE];               // LUT for chroma scales 
extern int g_intChromaInvScaleLUT[2][DRA_LUT_MAXSIZE];               // LUT for chroma scales 
#endif
#if HDR_METRIC
extern const double g_UCF_Filters[8][4][15];
extern const double g_color_trans[3][3];
extern const float  g_RGB2XYZ_REC[5][9];
#endif
#if M52291_HDR_DRA
extern const int g_dra_chroma_qp_offset_tbl[NUM_CHROMA_QP_OFFSET_LOG];
extern const int g_dra_exp_nom_v2[NUM_CHROMA_QP_SCALE_EXP];
#endif
#ifdef __cplusplus
}
#endif

#endif /* _EVC_TBL_H_ */
