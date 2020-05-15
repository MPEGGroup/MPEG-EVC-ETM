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

#ifndef _EVCD_BSR_H_
#define _EVCD_BSR_H_

#include "evc_port.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _EVC_BSR EVC_BSR;

/*! Function pointer for */
typedef int (*EVC_BSR_FN_FLUSH)(EVC_BSR *bs, int byte);

/*!
 * bitstream structure for decoder.
 *
 * NOTE: Don't change location of variable because this variable is used
 *       for assembly coding!
 */
struct _EVC_BSR
{
    /* temporary read code buffer */
    u32                code;
    /* left bits count in code */
    int                leftbits;
    /*! address of current bitstream position */
    u8               * cur;
    /*! address of bitstream end */
    u8               * end;
    /*! address of bitstream begin */
    u8               * beg;
    /*! size of original bitstream in byte */
    int                size;
    /*! Function pointer for bs_flush */
    EVC_BSR_FN_FLUSH  fn_flush;
    /*! arbitrary data, if needs */
    int                ndata[4];
    /*! arbitrary address, if needs */
    void             * pdata[4];
};

#define EVC_BSR_SKIP_CODE(bs, size) \
    evc_assert((bs)->leftbits >= (size)); \
    if((size) == 32) {(bs)->code = 0; (bs)->leftbits = 0;} \
    else           {(bs)->code <<= (size); (bs)->leftbits -= (size);}

/*! Is bitstream byte aligned? */
#define EVC_BSR_IS_BYTE_ALIGN(bs) ((((bs)->leftbits & 0x7) == 0) ? 1: 0)

/* get number of byte consumed */
#define EVC_BSR_GET_READ_BYTE(bs) ((int)((bs)->cur - (bs)->beg) - ((bs)->leftbits >> 3))

void evc_bsr_init(EVC_BSR * bs, u8 * buf, int size, EVC_BSR_FN_FLUSH fn_flush);
int evc_bsr_flush(EVC_BSR * bs, int byte);
int evc_bsr_clz_in_code(u32 code);
#if TRACE_HLS
#define evc_bsr_read(A, B, C) evc_bsr_read_trace(A, B, #B, C)
void evc_bsr_read_trace(EVC_BSR * bs, u32 * val, char * name, int size);

#define evc_bsr_read1(A, B) evc_bsr_read1_trace(A, B, #B)
void evc_bsr_read1_trace(EVC_BSR * bs, u32 * val, char * name);

#define evc_bsr_read_ue(A, B) evc_bsr_read_ue_trace(A, B, #B)
void evc_bsr_read_ue_trace(EVC_BSR * bs, u32 * val, char * name);

#define evc_bsr_read_se(A, B) evc_bsr_read_se_trace(A, B, #B)
void evc_bsr_read_se_trace(EVC_BSR * bs, s32 * val, char * name);
#else
void evc_bsr_read(EVC_BSR * bs, u32 * val, int size);
void evc_bsr_read1(EVC_BSR * bs, u32 * val);
void evc_bsr_read_ue(EVC_BSR * bs, u32 * val);
void evc_bsr_read_se(EVC_BSR * bs, s32 * val);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _EVCD_BSR_H_ */
