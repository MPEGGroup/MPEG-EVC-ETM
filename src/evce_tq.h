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

#ifndef _EVCE_TQ_H_
#define _EVCE_TQ_H_

#include "evce_def.h"

int evce_tq_nnz(u8 qp, double lambda, s16 * coef, int log2_cuw, int log2_cuh, u16 scale, int slice_type, int ch_type, int is_intra, int sps_cm_init_flag, int iqt_flag, u8 ats_intra_cu, u8 ats_tu, int tool_adcc);

//! \todo Change list of arguments
int evce_sub_block_tq(s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 qp_y, u8 qp_u, u8 qp_v, int slice_type, int nnz[N_C]
                      , int nnz_sub[N_C][MAX_SUB_TB_NUM], int is_intra, double lambda_y, double lambda_u, double lambda_v, int run_stats, int sps_cm_init_flag, int iqt_flag
                      , u8 ats_intra_cu, u8 ats_tu, u8 ats_inter_info, int tool_adcc
#if M50761_CHROMA_NOT_SPLIT
                      , TREE_CONS tree_cons
#endif

);
void evce_init_err_scale();

#endif /* _EVCE_TQ_H_ */
