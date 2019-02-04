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

#ifndef _EVC_PICMAN_H_
#define _EVC_PICMAN_H_

 /*Declaration for ref pic marking and ref pic list construction functions */
int evc_picman_refp_rpl_based_init(EVC_PM *pm, EVC_TGH *tgh, EVC_REFP(*refp)[REFP_NUM]);
int evc_picman_refpic_marking(EVC_PM *pm, EVC_TGH *tgh);

int evc_picman_refp_init(EVC_PM *pm, int num_ref_pics_act, int tile_group_type, u32 ptr, u8 layer_id, int last_intra, EVC_REFP (*refp)[REFP_NUM]);

EVC_PIC * evc_picman_get_empty_pic(EVC_PM *pm, int *err);

int evc_picman_put_pic(EVC_PM *pm, EVC_PIC *pic, int tile_group_type, u32 ptr, u32 dtr, u8 layer_id, int need_for_output, EVC_REFP (*refp)[REFP_NUM], EVC_MMCO *mmco, int pnpf);
EVC_PIC * evc_picman_out_pic(EVC_PM *pm, int *err);
int evc_picman_deinit(EVC_PM *pm);
int evc_picman_refp_reorder(EVC_PM *pm, int num_ref_pics_act, u8 tile_group_type, u32 ptr, EVC_REFP (*refp)[REFP_NUM], int last_intra, EVC_RMPNI *rmpni);
int evc_picman_init(EVC_PM *pm, int max_pb_size, int max_num_ref_pics, int use_closed_gop, PICBUF_ALLOCATOR *pa);

#endif /* _EVC_PICMAN_H_ */
