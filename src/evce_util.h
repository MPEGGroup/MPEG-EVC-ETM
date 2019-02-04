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

#ifndef _EVCE_UTIL_H_
#define _EVCE_UTIL_H_

#define PIC_CURR(ctx)             ((ctx)->pic[PIC_IDX_CURR])
#define PIC_ORIG(ctx)             ((ctx)->pic[PIC_IDX_ORIG])
#define PIC_MODE(ctx)             ((ctx)->pic[PIC_IDX_MODE])

void evce_picbuf_expand(EVCE_CTX *ctx, EVC_PIC *pic);

EVC_PIC * evce_pic_alloc(PICBUF_ALLOCATOR *pa, int *ret);
void evce_pic_free(PICBUF_ALLOCATOR *pa, EVC_PIC *pic);

void evce_bsw_skip_tile_group_size(EVC_BSW *bs);
void evce_bsw_write_tile_group_size(EVC_BSW *bs);

void evce_diff_pred(int x, int y, int log2_cuw, int log2_cuh, EVC_PIC *org, pel pred[N_C][MAX_CU_DIM], s16 diff[N_C][MAX_CU_DIM]);

#if AFFINE && RDO_DBK
void evc_set_affine_mvf(EVCE_CTX * ctx, EVCE_CORE * core, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][VER_NUM][MV_D], int vertex_num);
#endif

#define SBAC_STORE(dst, src) evc_mcpy(&dst, &src, sizeof(EVCE_SBAC))
#define SBAC_LOAD(dst, src)  evc_mcpy(&dst, &src, sizeof(EVCE_SBAC))

int evce_create_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh);
int evce_delete_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh);

void evce_split_tbl_init(EVCE_CTX *ctx);

#endif /* _EVCE_UTIL_H_ */
