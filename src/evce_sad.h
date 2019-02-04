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

#ifndef _EVCE_SAD_H_
#define _EVCE_SAD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "evc_port.h"

typedef int  (*EVCE_FN_SAD) (int w, int h, void *src1, void *src2, int s_src1, int s_src2);
typedef int  (*EVCE_FN_SATD)(int w, int h, void *src1, void *src2, int s_src1, int s_src2);
typedef s64  (*EVCE_FN_SSD) (int w, int h, void *src1, void *src2, int s_src1, int s_src2);
typedef void (*EVCE_FN_DIFF)(int w, int h, void *src1, void *src2, int s_src1, int s_src2, int s_diff, s16 *diff);

extern const EVCE_FN_SAD evce_tbl_sad_16b[8][8];
#define evce_sad_16b(log2w, log2h, src1, src2, s_src1, s_src2)\
    evce_tbl_sad_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2)

#define evce_sad_bi_16b(log2w, log2h, src1, src2, s_src1, s_src2)\
    (evce_tbl_sad_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2) >> 1)



extern const EVCE_FN_SATD evce_tbl_satd_16b[8][8];
#define evce_satd_16b(log2w, log2h, src1, src2, s_src1, s_src2)\
    evce_tbl_satd_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2)
#define evce_satd_bi_16b(log2w, log2h, src1, src2, s_src1, s_src2)\
    (evce_tbl_satd_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2) >> 1)

extern const EVCE_FN_SSD evce_tbl_ssd_16b[8][8];
#define evce_ssd_16b(log2w, log2h, src1, src2, s_src1, s_src2)\
    evce_tbl_ssd_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2)

extern const EVCE_FN_DIFF evce_tbl_diff_16b[8][8];
#define evce_diff_16b(log2w, log2h, src1, src2, s_src1, s_src2, s_diff, diff) \
    evce_tbl_diff_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2, s_diff, diff)


#ifdef __cplusplus
}
#endif

#endif /* _EVCE_SAD_H_ */
