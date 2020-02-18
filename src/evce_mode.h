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

#ifndef _EVCE_MODE_H_
#define _EVCE_MODE_H_

#ifdef __cplusplus
extern "C"
{
#endif

int evce_mode_create(EVCE_CTX * ctx, int complexity);

void evce_rdo_bit_cnt_cu_intra(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s16 coef[N_C][MAX_CU_DIM]);
void evce_rdo_bit_cnt_cu_intra_luma(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s16 coef[N_C][MAX_CU_DIM]);
void evce_rdo_bit_cnt_cu_intra_chroma(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s16 coef[N_C][MAX_CU_DIM]);
void evce_rdo_bit_cnt_cu_inter(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][MV_D], s16 coef[N_C][MAX_CU_DIM], int pidx, u8 * mvp_idx, u8 mvr_idx, u8 bi_idx, s16 affine_mvd[REFP_NUM][VER_NUM][MV_D]);
#if M50761_CHROMA_NOT_SPLIT
void evce_rdo_bit_cnt_cu_inter_chroma(EVCE_CTX * ctx, EVCE_CORE * core, s16 coef[N_C][MAX_CU_DIM], int pidx);

BOOL evc_signal_mode_cons( TREE_CONS* parent, TREE_CONS* cur_split );
#endif

void evce_rdo_bit_cnt_cu_ibc(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, s16 mvd[MV_D],
  s16 coef[N_C][MAX_CU_DIM], u8 mvp_idx, u8 pred_mode);

void evce_rdo_bit_cnt_cu_inter_comp(EVCE_CORE * core, s16 coef[N_C][MAX_CU_DIM], int ch_type, int pidx, EVCE_CTX * ctx
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
);
void evce_rdo_bit_cnt_cu_skip(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s32 cup, int mvp_idx0, int mvp_idx1, int c_num, int tool_mmvd);
void evce_rdo_bit_cnt_mvp(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][MV_D], int pidx, int mvp_idx);
void evce_rdo_bit_cnt_affine_mvp(EVCE_CTX * ctx, EVCE_CORE * core, s32 slice_type, s8 refi[REFP_NUM], s16 mvd[REFP_NUM][VER_NUM][MV_D], int pidx, int mvp_idx, int vertex_num);
void evce_sbac_bit_reset(EVCE_SBAC * sbac);
u32  evce_get_bit_number(EVCE_SBAC * sbac);
void evce_init_bits_est();
u16  evc_get_lr(u16 avail_lr);
u16  evc_get_bef_data_idx(EVCE_CORE * core);
void evce_init_bef_data(EVCE_CORE * core, EVCE_CTX * ctx);

#if RDO_DBK
void calc_delta_dist_filter_boundary(EVCE_CTX* ctx, EVC_PIC *pic_rec, EVC_PIC *pic_org, int cuw, int cuh, pel(*src)[MAX_CU_DIM], int s_src, int x, int y, u16 avail_lr, u8 intra_flag, u8 cbf_l, s8 *refi, s16(*mv)[MV_D], u8 is_mv_from_mvf, u8 ats_inter_info);
#endif

int evce_hmvp_init(EVC_HISTORY_BUFFER *history_buffer);

#ifdef __cplusplus
}
#endif

#endif /* _EVCE_MODE_H_ */
