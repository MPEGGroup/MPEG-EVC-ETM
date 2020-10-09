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

#ifndef _EVC_IPRED_H_
#define _EVC_IPRED_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "evc_def.h"

#define EVC_IPRED_CHK_CONV(mode)\
    ((mode) == IPD_VER || (mode) == IPD_HOR || (mode) == IPD_DC || (mode) == IPD_BI)
    
#define EVC_IPRED_CONV_L2C(mode)\
    ((mode) == IPD_VER) ? IPD_VER_C : \
    ((mode) == IPD_HOR ? IPD_HOR_C : ((mode) == IPD_DC ? IPD_DC_C : IPD_BI_C))

#define EVC_IPRED_CONV_L2C_CHK(mode, chk) \
    if(EVC_IPRED_CHK_CONV(mode)) \
    {\
        (mode) = ((mode) == IPD_VER) ? IPD_VER_C : ((mode) == IPD_HOR ? IPD_HOR_C:\
        ((mode) == IPD_DC ? IPD_DC_C : IPD_BI_C)); \
        (chk) = 1; \
    }\
    else \
        (chk) = 0;

void evc_get_nbr_b(int x, int y, int cuw, int cuh, pel *src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 *map_scu
                   , int w_scu, int h_scu, int ch_type, int constrained_intra_pred, u8* map_tidx
#if BD_CF_EXT
                   , int bit_depth
                   , int chroma_format_idc
#endif
);
void evc_get_nbr(int x, int y, int cuw, int cuh, pel *src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 *map_scu
                 , int w_scu, int h_scu, int ch_type, int constrained_intra_pred, u8 * map_tidx
#if BD_CF_EXT
                 , int bit_depth
                 , int chroma_format_idc
#endif
);

void evc_ipred(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm, int w, int h
#if BD_CF_EXT
               , int bit_depth
#endif
);
void evc_ipred_uv(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm_c, int ipm, int w, int h
#if BD_CF_EXT
                  , int bit_depth
#endif
);
void evc_ipred_b(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm, int w, int h);
void evc_ipred_uv_b(pel *src_le, pel *src_up, pel *src_ri, u16 avail_lr, pel *dst, int ipm_c, int ipm, int w, int h);

void evc_get_mpm_b(int x_scu, int y_scu, int cuw, int cuh, u32 * map_scu, s8 * map_ipm, int scup, int w_scu, u8 ** mpm, u16 avail_lr, u8 mpm_ext[8], u8 pms[IPD_CNT], u8 * map_tidx);
void evc_get_mpm(int x_scu, int y_scu, int cuw, int cuh, u32 * map_scu, s8 * map_ipm, int scup, int w_scu, u8 mpm[2], u16 avail_lr, u8 mpm_ext[8], u8 pms[IPD_CNT], u8 * map_tidx);

#ifdef __cplusplus
}
#endif

#endif /* _EVC_IPRED_H_ */
