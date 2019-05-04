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

#ifndef _EVCD_ECO_H_
#define _EVCD_ECO_H_

#include "evcd_def.h"


#define GET_SBAC_DEC(bs)   ((EVCD_SBAC *)((bs)->pdata[1]))
#define SET_SBAC_DEC(bs, sbac) ((bs)->pdata[1] = (sbac))

u32 evcd_sbac_decode_bin(EVC_BSR *bs, EVCD_SBAC *sbac, SBAC_CTX_MODEL *model);
u32 evcd_sbac_decode_bin_trm(EVC_BSR *bs, EVCD_SBAC *sbac);

int evcd_eco_cnkh(EVC_BSR * bs, EVC_CNKH * cnkh);
int evcd_eco_sps(EVC_BSR * bs, EVC_SPS * sps);
int evcd_eco_pps(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps);
#if ALF_PARAMETER_APS
int evcd_eco_aps(EVC_BSR * bs, EVC_APS * aps);
int evcd_eco_alf_aps_param(EVC_BSR * bs, EVC_APS * aps);
#endif
int evcd_eco_tgh(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_TGH * tgh);

int evcd_eco_udata(EVCD_CTX * ctx, EVC_BSR * bs);

void evcd_eco_sbac_reset(EVC_BSR * bs, u8 tile_group_type, u8 tile_group_qp, int sps_cm_init_flag);

int evcd_eco_inter_dir(EVC_BSR * bs, EVCD_SBAC * sbac, int *direct_idx, int type, u8 mvr_idx, u16 avail_lr);
int evcd_eco_intra_dir(EVC_BSR * bs, EVCD_SBAC * sbac, u8 mpm[2], u8 mpm_ext[8], u8 pims[IPD_CNT]);
int evcd_eco_intra_dir_b(EVC_BSR * bs, EVCD_SBAC * sbac, u8 * mpm, u8 mpm_ext[8], u8 pims[IPD_CNT]);
int evcd_eco_intra_dir_c(EVC_BSR * bs, EVCD_SBAC * sbac, u8 ipm_l);

int evcd_eco_coef(EVCD_CTX * ctx, EVCD_CORE * core);

int evcd_eco_cu(EVCD_CTX * ctx, EVCD_CORE * core);

s8 evcd_eco_split_mode(EVCD_CTX * ctx, EVC_BSR *bs, EVCD_SBAC *sbac, int cuw, int cuh, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int* curr_split_allow, int qt_depth, int btt_depth, int x, int y);
s8 evcd_eco_suco_flag(EVC_BSR *bs, EVCD_SBAC *sbac, EVCD_CTX *c, EVCD_CORE *core, int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, int parent_suco);
#define evcd_eco_tile_group_end_flag(bs, sbac) \
    ((int)evcd_sbac_decode_bin_trm((bs), (sbac)))

#if AFFINE
#define evcd_eco_affine_flag(bs, sbac) \
    (evcd_sbac_decode_bin((bs), (sbac), (sbac)->ctx.affine_flag))
#define evcd_eco_affine_mode(bs, sbac) \
    (evcd_sbac_decode_bin((bs), (sbac), (sbac)->ctx.affine_mode))
int evcd_eco_affine_mrg_idx(EVC_BSR * bs, EVCD_SBAC * sbac);
#define evcd_eco_affine_mvd_flag(bs, sbac, refi) \
    (evcd_sbac_decode_bin((bs), (sbac), &(sbac)->ctx.affine_mvd_flag[refi]))
#endif

#endif /* _EVCD_ECO_H_ */
