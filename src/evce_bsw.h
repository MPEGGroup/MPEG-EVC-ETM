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

#ifndef _EVCE_BSW_H_
#define _EVCE_BSW_H_

#include "evc_port.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _EVC_BSW EVC_BSW;

/*! Function pointer for */
typedef int (*EVC_BSW_FN_FLUSH)(EVC_BSW *bs);

/*! Bitstream structure for encoder */
struct _EVC_BSW
{
    /* buffer */
    u32                code;
    /* bits left in buffer */
    int                leftbits;
    /*! address of current writing position */
    u8               * cur;
    /*! address of bitstream buffer end */
    u8               * end;
    /*! address of bitstream buffer begin */
    u8               * beg;
    /*! size of bitstream buffer in byte */
    int                size;
    /*! address of function for flush */
    EVC_BSW_FN_FLUSH  fn_flush;
    /*! arbitrary data, if needs */
    int                 ndata[4];
    /*! arbitrary address, if needs */
    void              * pdata[4];
};

/* is bitstream byte aligned? */
#define EVC_BSW_IS_BYTE_ALIGN(bs)     !((bs)->leftbits & 0x7)

/* get number of byte written */
#define EVC_BSW_GET_WRITE_BYTE(bs)    (int)((bs)->cur - (bs)->beg)

/* number of bytes to be sunk */
#define EVC_BSW_GET_SINK_BYTE(bs)     ((32 - (bs)->leftbits + 7) >> 3)

void evc_bsw_init(EVC_BSW * bs, u8 * buf, int size, EVC_BSW_FN_FLUSH fn_flush);
#if EVC_TILE_SUPPORT
void evc_bsw_init_slice(EVC_BSW * bs, u8 * buf, int size, EVC_BSW_FN_FLUSH fn_flush);
#endif
void evc_bsw_deinit(EVC_BSW * bs);
#if TRACE_HLS
#define evc_bsw_write1(A, B) evc_bsw_write1_trace(A, B, #B)
int evc_bsw_write1_trace(EVC_BSW * bs, int val, char* name);

#define evc_bsw_write(A, B, C) evc_bsw_write_trace(A, B, #B, C)
int evc_bsw_write_trace(EVC_BSW * bs, u32 val, char* name, int len);

#define evc_bsw_write_ue(A, B) evc_bsw_write_ue_trace(A, B, #B)
void evc_bsw_write_ue_trace(EVC_BSW * bs, u32 val, char* name);

#define evc_bsw_write_se(A, B) evc_bsw_write_se_trace(A, B, #B)
void evc_bsw_write_se_trace(EVC_BSW * bs, int val, char* name);
#else
int evc_bsw_write1(EVC_BSW * bs, int val);
int evc_bsw_write(EVC_BSW * bs, u32 val, int len);
void evc_bsw_write_ue(EVC_BSW * bs, u32 val);
void evc_bsw_write_se(EVC_BSW * bs, int val);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _EVCE_BSW_H_ */
