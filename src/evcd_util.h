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

#ifndef _EVCD_UTIL_H_
#define _EVCD_UTIL_H_

void evcd_picbuf_expand(EVCD_CTX * ctx, EVC_PIC * pic);
EVC_PIC * evcd_picbuf_alloc(PICBUF_ALLOCATOR * pa, int * ret);
void evcd_picbuf_free(PICBUF_ALLOCATOR * pa, EVC_PIC * pic);

#if HDR_MD5_CHECK
int evcd_picbuf_check_signature(EVC_PIC * pic, u8 signature[N_C][16], int tool_dra, void* pps_draParams, u16 width, u16 height, int doCompare
#if BD_CF_EXT
                                , int bit_depth
#endif
);
#else
int evcd_picbuf_check_signature(EVC_PIC * pic, u8 signature[16]);
#endif
void evcd_get_mmvd_motion(EVCD_CTX * ctx, EVCD_CORE * core);

static int evcd_hmvp_init(EVCD_CORE * core);

/* set decoded information, such as MVs, inter_dir, etc. */
void evcd_set_dec_info(EVCD_CTX * ctx, EVCD_CORE * core
#if ENC_DEC_TRACE
                       , u8 write_trace
#endif
);

void evcd_split_tbl_init(EVCD_CTX *ctx);

#if USE_DRAW_PARTITION_DEC
void evcd_draw_partition(EVCD_CTX * ctx, EVC_PIC * pic);
#endif

u8 evcd_check_luma(EVCD_CTX *ctx, EVCD_CORE * core);
u8 evcd_check_chroma(EVCD_CTX *ctx, EVCD_CORE * core);
u8 evcd_check_all(EVCD_CTX *ctx, EVCD_CORE * core);
u8 evcd_check_only_intra(EVCD_CTX *ctx, EVCD_CORE * core);
u8 evcd_check_only_inter(EVCD_CTX *ctx, EVCD_CORE * core);
u8 evcd_check_all_preds(EVCD_CTX *ctx, EVCD_CORE * core);
MODE_CONS evcd_derive_mode_cons(EVCD_CTX *ctx, int scup);
#endif /* _EVCD_UTIL_H_ */
