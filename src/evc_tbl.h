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
extern u16 * evc_scan_tbl[COEF_SCAN_TYPE_NUM][MAX_CU_LOG2 - 1][MAX_CU_LOG2 - 1];
extern const u8 evey_tbl_mpm[6][6][5];

extern const int evc_tbl_dq_scale[6];
extern const int evc_tbl_dq_scale_b[6];
extern const u8 evc_tbl_df_st[4][52];

extern const int evc_tbl_ipred_adi[32][4];
extern const int evc_tbl_ipred_dxdy[IPD_CNT][2];

extern u16 evc_split_tbl[6][SPLIT_CHECK_NUM][2];
extern const u8  evc_split_order[2][SPLIT_CHECK_NUM];

extern int evc_tbl_qp_chroma_ajudst_main[58];
extern int evc_tbl_qp_chroma_ajudst_base[58];
extern int* evc_tbl_qp_chroma_ajudst;

#if CTX_REPRESENTATION_IMPROVEMENT
extern const s16 init_cbf[2][NUM_QT_CBF_CTX][1];
extern const s16 init_all_cbf[2][NUM_QT_ROOT_CBF_CTX][1];
extern const s16 init_pred_mode[2][NUM_PRED_MODE_CTX][1];
extern const s16 init_inter_dir[2][NUM_INTER_DIR_CTX][1];
extern const s16 init_intra_dir[2][NUM_INTRA_DIR_CTX][1];
extern const s16 init_mmvd_flag[2][NUM_SBAC_CTX_MMVD_FLAG][1];
extern const s16 init_mmvd_merge_idx[2][NUM_SBAC_CTX_MMVD_MERGE_IDX][1];
extern const s16 init_mmvd_distance_idx[2][NUM_SBAC_CTX_MMVD_DIST_IDX][1];
extern const s16 init_mmvd_direction_idx[2][NUM_SBAC_CTX_DIRECTION_IDX][1];
extern const s16 init_mmvd_group_idx[2][NUM_SBAC_CTX_MMVD_GRP_IDX][1];
extern const s16 init_mvp_idx[2][NUM_MVP_IDX_CTX][1];
extern const s16 init_affine_mvp_idx[2][NUM_AFFINE_MVP_IDX_CTX][1];
extern const s16 init_mvr_idx[2][NUM_MVR_IDX_CTX][1];
extern const s16 init_bi_idx[2][NUM_BI_IDX_CTX][1];
extern const s16 init_mvd[2][NUM_MV_RES_CTX][1];
extern const s16 init_refi[2][NUM_REFI_CTX][1];
extern const s16 init_btt_split_flag[2][NUM_SBAC_CTX_BTT_SPLIT_FLAG][1];
extern const s16 init_btt_split_dir[2][NUM_SBAC_CTX_BTT_SPLIT_DIR][1];
extern const s16 init_btt_split_type[2][NUM_SBAC_CTX_BTT_SPLIT_TYPE][1];
extern const s16 init_run[2][NUM_SBAC_CTX_RUN][1];
extern const s16 init_last[2][NUM_SBAC_CTX_LAST][1];
extern const s16 init_level[2][NUM_SBAC_CTX_LEVEL][1];
extern const s16 init_suco_flag[2][NUM_SBAC_CTX_SUCO_FLAG][1];
#if ALF
extern const s16 init_ctb_alf_flag[2][NUM_SBAC_CTX_ALF_FLAG][1];
#endif
#if AFFINE
extern const s16 init_affine_flag[2][NUM_SBAC_CTX_AFFINE_FLAG][1];
extern const s16 init_affine_mode[2][NUM_SBAC_CTX_AFFINE_MODE][1];
extern const s16 init_affine_mrg[2][AFF_MAX_CAND][1];
extern const s16 init_affine_mvd_flag[2][NUM_SBAC_CTX_AFFINE_MVD_FLAG][1];
#endif
extern const s16 init_skip_flag[2][NUM_SBAC_CTX_SKIP_FLAG][1];
#else

extern const s16 init_cbf[2][NUM_QT_CBF_CTX][2];
extern const s16 init_all_cbf[2][NUM_QT_ROOT_CBF_CTX][2];
extern const s16 init_pred_mode[2][NUM_PRED_MODE_CTX][2];
extern const s16 init_inter_dir[2][NUM_INTER_DIR_CTX][2];
extern const s16 init_intra_dir[2][NUM_INTRA_DIR_CTX][2];
extern const s16 init_mmvd_flag[2][NUM_SBAC_CTX_MMVD_FLAG][2];
extern const s16 init_mmvd_merge_idx[2][NUM_SBAC_CTX_MMVD_MERGE_IDX][2];
extern const s16 init_mmvd_distance_idx[2][NUM_SBAC_CTX_MMVD_DIST_IDX][2];
extern const s16 init_mmvd_direction_idx[2][NUM_SBAC_CTX_DIRECTION_IDX][2];
extern const s16 init_mmvd_group_idx[2][NUM_SBAC_CTX_MMVD_GRP_IDX][2];
extern const s16 init_mvp_idx[2][NUM_MVP_IDX_CTX][2];
extern const s16 init_affine_mvp_idx[2][NUM_AFFINE_MVP_IDX_CTX][2];
extern const s16 init_mvr_idx[2][NUM_MVR_IDX_CTX][2];
extern const s16 init_bi_idx[2][NUM_BI_IDX_CTX][2];
extern const s16 init_mvd[2][NUM_MV_RES_CTX][2];
extern const s16 init_refi[2][NUM_REFI_CTX][2];
extern const s16 init_btt_split_flag[2][NUM_SBAC_CTX_BTT_SPLIT_FLAG][2];
extern const s16 init_btt_split_dir[2][NUM_SBAC_CTX_BTT_SPLIT_DIR][2];
extern const s16 init_btt_split_type[2][NUM_SBAC_CTX_BTT_SPLIT_TYPE][2];
extern const s16 init_run[2][NUM_SBAC_CTX_RUN][2];
extern const s16 init_last[2][NUM_SBAC_CTX_LAST][2];
extern const s16 init_level[2][NUM_SBAC_CTX_LEVEL][2];
extern const s16 init_suco_flag[2][NUM_SBAC_CTX_SUCO_FLAG][2];
#if ALF
extern const s16 init_ctb_alf_flag[2][NUM_SBAC_CTX_ALF_FLAG][2];
#endif
#if AFFINE
extern const s16 init_affine_flag[2][NUM_SBAC_CTX_AFFINE_FLAG][2];
extern const s16 init_affine_mode[2][NUM_SBAC_CTX_AFFINE_MODE][2];
extern const s16 init_affine_mrg[2][AFF_MAX_CAND][2];
extern const s16 init_affine_mvd_flag[2][NUM_SBAC_CTX_AFFINE_MVD_FLAG][2];
#endif
extern const s16 init_skip_flag[2][NUM_SBAC_CTX_SKIP_FLAG][2];

#endif
#if DBF == DBF_AVC
extern const u8 ALPHA_TABLE[52];
extern const u8 BETA_TABLE[52];
extern const u8 CLIP_TAB[52][5];
#endif

#ifdef __cplusplus
}
#endif

#endif /* _EVC_TBL_H_ */
