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

#include "evce_def.h"


static int evc_bsw_flush(EVC_BSW * bs)
{
    int bytes = EVC_BSW_GET_SINK_BYTE(bs);

    while(bytes--)
    {
        *bs->cur++ = (bs->code >> 24) & 0xFF;
        bs->code <<= 8;
    }

    bs->leftbits = 32;

    return 0;
}

void evc_bsw_init(EVC_BSW * bs, u8 * buf, int size,
                   EVC_BSW_FN_FLUSH fn_flush)
{
    bs->size = size;
    bs->beg = buf;
    bs->cur = buf;
    bs->end = buf + size - 1;
    bs->code = 0;
    bs->leftbits = 32;
    bs->fn_flush = (fn_flush == NULL ? evc_bsw_flush : fn_flush);
}

void evc_bsw_deinit(EVC_BSW * bs)
{
    bs->fn_flush(bs);
}

int evc_bsw_write1(EVC_BSW * bs, int val)
{
    evc_assert(bs);

    bs->leftbits--;
    bs->code |= ((val & 0x1) << bs->leftbits);

    if(bs->leftbits == 0)
    {
        evc_assert_rv(bs->cur <= bs->end, -1);
        bs->fn_flush(bs);

        bs->code = 0;
        bs->leftbits = 32;
    }

    return 0;
}

int evc_bsw_write(EVC_BSW * bs, u32 val, int len) /* len(1 ~ 32) */
{
    int leftbits;

    evc_assert(bs);

    leftbits = bs->leftbits;
    val <<= (32 - len);
    bs->code |= (val >> (32 - leftbits));

    if(len < leftbits)
    {
        bs->leftbits -= len;
    }
    else
    {
        evc_assert_rv(bs->cur + 4 <= bs->end, -1);

        bs->leftbits = 0;
        bs->fn_flush(bs);

#if defined(X86F)
        /* on X86 machine, shift operation works properly when the value of the
           right operand is less than the number of bits of the left operand. */
        bs->code = (leftbits < 32 ? val << leftbits : 0);
#else
        bs->code = (val << leftbits);
#endif
        bs->leftbits = 32 - (len - leftbits);
    }

    return 0;
}

void evc_bsw_write_ue(EVC_BSW * bs, u32 val)
{
    int   len_i, len_c, info, nn;
    u32  code;

    nn = ((val + 1) >> 1);
    for(len_i = 0; len_i < 16 && nn != 0; len_i++)
    {
        nn >>= 1;
    }

    info = val + 1 - (1 << len_i);
    code = (1 << len_i) | ((info)& ((1 << len_i) - 1));

    len_c = (len_i << 1) + 1;

    evc_bsw_write(bs, code, len_c);
}

void evc_bsw_write_se(EVC_BSW * bs, int val)
{
    evc_bsw_write_ue(bs, val <= 0 ? (-val * 2) : (val * 2 - 1));
}
