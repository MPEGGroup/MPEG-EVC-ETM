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

#include "evcd_def.h"


/* Table of count of leading zero for 4 bit value */
static const u8 tbl_zero_count4[16] =
{
    4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0
};

void evc_bsr_init(EVC_BSR * bs, u8 * buf, int size, EVC_BSR_FN_FLUSH fn_flush)
{
    bs->size     = size;
    bs->cur      = buf;
    bs->beg      = buf;
    bs->end      = buf + size - 1;
    bs->code     = 0;
    bs->leftbits = 0;
    bs->fn_flush = (fn_flush == NULL)? evc_bsr_flush : fn_flush;
}

int evc_bsr_flush(EVC_BSR * bs, int byte)
{
    int    shift = 24, remained;
    u32 code = 0;

    evc_assert(byte);

    remained = (int)(bs->end - bs->cur) + 1;
    if(byte > remained) byte = remained;

    if(byte <= 0)
    {
        bs->code = 0;
        bs->leftbits = 0;
        return -1;
    }

    bs->leftbits = byte << 3;

    bs->cur += byte;
    while(byte)
    {
        code |= *(bs->cur - byte) << shift;
        byte--;
        shift -= 8;
    }
    bs->code = code;
    return 0;
}

int evc_bsr_clz_in_code(u32 code)
{
    int clz, bits4, shift;

    if(code == 0) return 32; /* to protect infinite loop */

    bits4 = 0;
    clz = 0;
    shift = 28;

    while(bits4 == 0 && shift >= 0)
    {
        bits4 = (code >> shift) & 0xf;
        clz += tbl_zero_count4[bits4];
        shift -= 4;
    }
    return clz;
}

#if TRACE_HLS
void evc_bsr_read_trace(EVC_BSR * bs, u32 * val, char * name, int size)
{
    u32 code = 0;

    evc_assert(size > 0);

    if (bs->leftbits < size)
    {
        code = bs->code >> (32 - size);
        size -= bs->leftbits;

        if (bs->fn_flush(bs, 4))
        {
            evc_trace("already reached the end of bitstream\n");
            *val = (u32)-1;
            return;
        }
    }
    code |= bs->code >> (32 - size);

    EVC_BSR_SKIP_CODE(bs, size);

    *val = code;

    if (name)
    {
        EVC_TRACE_STR(name + 1);
        EVC_TRACE_STR(" ");
        EVC_TRACE_INT(*val);
        EVC_TRACE_STR("\n");
    }
}

void evc_bsr_read1_trace(EVC_BSR * bs, u32 * val, char * name)
{
    int code;
    if (bs->leftbits == 0)
    {
        if (bs->fn_flush(bs, 4))
        {
            evc_trace("already reached the end of bitstream\n");
            return;
        }
    }
    code = (int)(bs->code >> 31);

    bs->code <<= 1;
    bs->leftbits -= 1;

    *val = code;
    if (name)
    {
        EVC_TRACE_STR(name + 1);
        EVC_TRACE_STR(" ");
        EVC_TRACE_INT(*val);
        EVC_TRACE_STR("\n");
    }
}
void evc_bsr_read_ue_trace(EVC_BSR * bs, u32 * val, char * name)
{
    int clz, len;

    if ((bs->code >> 31) == 1)
    {
        /* early termination.
        we don't have to worry about leftbits == 0 case, because if the bs->code
        is not equal to zero, that means leftbits is not zero */
        bs->code <<= 1;
        bs->leftbits -= 1;
        *val = 0;
        if (name)
        {
            EVC_TRACE_STR(name+1);
            EVC_TRACE_STR(" ");
            EVC_TRACE_INT(*val);
            EVC_TRACE_STR("\n");
        }
        return;
    }

    clz = 0;
    if (bs->code == 0)
    {
        clz = bs->leftbits;

        bs->fn_flush(bs, 4);
    }

    len = evc_bsr_clz_in_code(bs->code);

    clz += len;

    if (clz == 0)
    {
        /* early termination */
        bs->code <<= 1;
        bs->leftbits--;
        *val = 0;
        if (name)
        {
            EVC_TRACE_STR(name+1);
            EVC_TRACE_STR(" ");
            EVC_TRACE_INT(*val);
            EVC_TRACE_STR("\n");
        }
        return;
    }

    evc_assert(bs->leftbits >= 0);
    evc_bsr_read_trace(bs, val, 0, len + clz + 1);
    *val -= 1;

    if (name)
    {
        EVC_TRACE_STR(name+1);
        EVC_TRACE_STR(" ");
        EVC_TRACE_INT(*val);
        EVC_TRACE_STR("\n");
    }
}

void evc_bsr_read_se_trace(EVC_BSR * bs, s32 * val, char * name)
{
    evc_assert(bs != NULL);

    evc_bsr_read_ue_trace(bs, val, 0);

    *val = ((*val & 0x01) ? ((*val + 1) >> 1) : -(*val >> 1));

    if (name)
    {
        EVC_TRACE_STR(name + 1);
        EVC_TRACE_STR(" ");
        EVC_TRACE_INT(*val);
        EVC_TRACE_STR("\n");
    }
}
#else
void evc_bsr_read(EVC_BSR * bs, u32 * val, int size)
{
    u32 code = 0;

    evc_assert(size > 0);

    if (bs->leftbits < size)
    {
        code = bs->code >> (32 - size);
        size -= bs->leftbits;

        if (bs->fn_flush(bs, 4))
        {
            evc_trace("already reached the end of bitstream\n");
            *val = (u32)-1;
            return;
        }
    }
    code |= bs->code >> (32 - size);

    EVC_BSR_SKIP_CODE(bs, size);

    *val = code;
}

void evc_bsr_read1(EVC_BSR * bs, u32 * val)
{
    int code;
    if (bs->leftbits == 0)
    {
        if (bs->fn_flush(bs, 4))
        {
            evc_trace("already reached the end of bitstream\n");
            return;
        }
    }
    code = (int)(bs->code >> 31);

    bs->code <<= 1;
    bs->leftbits -= 1;

    *val = code;
}

void evc_bsr_read_ue(EVC_BSR * bs, u32 * val)
{
    int clz, len;

    if((bs->code >> 31) == 1)
    {
        /* early termination.
        we don't have to worry about leftbits == 0 case, because if the bs->code
        is not equal to zero, that means leftbits is not zero */
        bs->code <<= 1;
        bs->leftbits -= 1;
        *val = 0;
        return;
    }

    clz = 0;
    if(bs->code == 0)
    {
        clz = bs->leftbits;

        bs->fn_flush(bs, 4);
    }

    len = evc_bsr_clz_in_code(bs->code);

    clz += len;

    if(clz == 0)
    {
        /* early termination */
        bs->code <<= 1;
        bs->leftbits--;
        *val = 0;
        return;
    }

    evc_assert(bs->leftbits >= 0);
    evc_bsr_read(bs, val, len + clz + 1);
    *val -= 1;
}

void evc_bsr_read_se(EVC_BSR * bs, s32 * val)
{
    evc_assert(bs != NULL);

    evc_bsr_read_ue(bs, val);

    *val = ((*val & 0x01) ? ((*val + 1) >> 1) : -(*val >> 1));
}
#endif
