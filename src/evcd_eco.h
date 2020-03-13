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

#ifndef _EVCD_ECO_H_
#define _EVCD_ECO_H_

#include "evcd_def.h"


#define GET_SBAC_DEC(bs)   ((EVCD_SBAC *)((bs)->pdata[1]))
#define SET_SBAC_DEC(bs, sbac) ((bs)->pdata[1] = (sbac))

u32 evcd_sbac_decode_bin(EVC_BSR *bs, EVCD_SBAC *sbac, SBAC_CTX_MODEL *model);
u32 evcd_sbac_decode_bin_trm(EVC_BSR *bs, EVCD_SBAC *sbac);

int evcd_eco_nalu(EVC_BSR * bs, EVC_NALU * nalu);
int evcd_eco_sps(EVC_BSR * bs, EVC_SPS * sps);
int evcd_eco_pps(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps);
#if M52291_HDR_DRA
int evcd_eco_aps_gen(EVC_BSR * bs, EVC_APS_GEN * aps);
int evcd_eco_dra_aps_param(EVC_BSR * bs, EVC_APS_GEN * aps);
#endif
int evcd_eco_aps(EVC_BSR * bs, EVC_APS * aps);
int evcd_eco_alf_aps_param(EVC_BSR * bs, EVC_APS * aps);
int evcd_eco_sh(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_SH * sh, int nut);
int evcd_eco_sei(EVCD_CTX * ctx, EVC_BSR * bs);
void evcd_eco_sbac_reset(EVC_BSR * bs, u8 slice_type, u8 slice_qp, int sps_cm_init_flag);
int evcd_eco_cu(EVCD_CTX * ctx, EVCD_CORE * core);

s8 evcd_eco_split_mode(EVCD_CTX * ctx, EVC_BSR *bs, EVCD_SBAC *sbac, int cuw, int cuh, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int* curr_split_allow, int qt_depth, int btt_depth, int x, int y
#if M50761_CHROMA_NOT_SPLIT
    , MODE_CONS mode_cons
#endif
);
s8 evcd_eco_suco_flag(EVC_BSR *bs, EVCD_SBAC *sbac, EVCD_CTX *c, EVCD_CORE *core, int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, int parent_suco);
#define evcd_eco_slice_end_flag(bs, sbac) \
    ((int)evcd_sbac_decode_bin_trm((bs), (sbac)))

#if EVC_TILE_SUPPORT
#define evcd_eco_tile_end_flag(bs, sbac) \
    ((int)evcd_sbac_decode_bin_trm((bs), (sbac)))
#endif

int evcd_eco_affine_mrg_idx(EVC_BSR * bs, EVCD_SBAC * sbac);
#if M50761_CHROMA_NOT_SPLIT
MODE_CONS evcd_eco_mode_constr( EVC_BSR *bs, u8 ctx_num );
#endif

#if GRAB_STAT
void encd_stat_cu(int x, int y, int cuw, int cuh, int cup, void *ctx, void *core
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
);
#endif
#endif /* _EVCD_ECO_H_ */
