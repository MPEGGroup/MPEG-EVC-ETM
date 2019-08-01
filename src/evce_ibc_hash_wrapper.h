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

#ifndef __EVCE_IBC_HASH_WRAPPER_H
#define __EVCE_IBC_HASH_WRAPPER_H

#include <stdint.h>
#include "evce_def.h"

#if USE_IBC

#ifdef __cplusplus
extern "C" {
#endif

typedef void *ibc_hash_handle;

ibc_hash_handle* create_enc_IBC(int picWidth, int picHeight);
void destroy_enc_IBC(ibc_hash_handle* p);

void rebuild_hashmap(ibc_hash_handle* p, const EVC_PIC* pic);

u32 search_ibc_hash_match(EVCE_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y,
    int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D]);

int get_hash_hit_ratio(EVCE_CTX* ctx, ibc_hash_handle* p, int cu_x, int cu_y, int log2_cuw, int log2_cuh);

#ifdef __cplusplus
}
#endif

#endif

#endif /* end of __EVCE_IBC_HASH_WRAPPER_H */
