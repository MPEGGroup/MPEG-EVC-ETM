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

#include "evc.h"
#include "evc_img.h"

static void imgb_delete(EVC_IMGB * imgb)
{
    int i;
    evc_assert_r(imgb);

    for(i=0; i<EVC_IMGB_MAX_PLANE; i++)
    {
        if (imgb->baddr[i]) evc_mfree(imgb->baddr[i]);
    }
    evc_mfree(imgb);
}

static int imgb_addref(EVC_IMGB * imgb)
{
    evc_assert_rv(imgb, EVC_ERR_INVALID_ARGUMENT);
    return evc_atomic_inc(&imgb->refcnt);
}

static int imgb_getref(EVC_IMGB * imgb)
{
    evc_assert_rv(imgb, EVC_ERR_INVALID_ARGUMENT);
    return imgb->refcnt;
}

static int imgb_release(EVC_IMGB * imgb)
{
    int refcnt;
    evc_assert_rv(imgb, EVC_ERR_INVALID_ARGUMENT);
    refcnt = evc_atomic_dec(&imgb->refcnt);
    if(refcnt == 0)
    {
        imgb_delete(imgb);
    }
    return refcnt;
}

EVC_IMGB * evc_imgb_create(int w, int h, int cs, int opt, int pad[EVC_IMGB_MAX_PLANE], int align[EVC_IMGB_MAX_PLANE])
{
    int i, p_size, a_size, bd;
    EVC_IMGB * imgb;

    imgb = (EVC_IMGB *)evc_malloc(sizeof(EVC_IMGB));
    evc_assert_rv(imgb, NULL);
    evc_mset(imgb, 0, sizeof(EVC_IMGB));
#if MULTIPLE_NAL
    imgb->imgb_active_pps_id = -1;
    imgb->imgb_active_aps_id = -1;
#endif
#if BD_CF_EXT
    int bit_depth = (BD_FROM_CS(cs));
    int idc = CF_FROM_CS(cs);
    int np = idc == 0 ? 1 : 3;

    if(bit_depth >= 8 && bit_depth <= 14)
    {
        if(bit_depth == 8) bd = 1;
        else /*if(cs == EVC_COLORSPACE_YUV420_10LE)*/ bd = 2;
        for(i=0;i<np;i++)
#else
    if(cs == EVC_COLORSPACE_YUV420 || cs == EVC_COLORSPACE_YUV420_10LE)
    {
        if(cs == EVC_COLORSPACE_YUV420) bd = 1;
        else /*if(cs == EVC_COLORSPACE_YUV420_10LE)*/ bd = 2;

        for(i=0; i<3; i++)
#endif
        {
            imgb->w[i] = w;
            imgb->h[i] = h;
            imgb->x[i] = 0;
            imgb->y[i] = 0;

            a_size = (align != NULL)? align[i] : 0;
            p_size = (pad != NULL)? pad[i] : 0;

            imgb->aw[i] = EVC_ALIGN(w, a_size);
            imgb->ah[i] = EVC_ALIGN(h, a_size);

            imgb->padl[i] = imgb->padr[i]=imgb->padu[i]=imgb->padb[i]=p_size;

            imgb->s[i] = (imgb->aw[i] + imgb->padl[i] + imgb->padr[i]) * bd;
            imgb->e[i] = imgb->ah[i] + imgb->padu[i] + imgb->padb[i];

            imgb->bsize[i] = imgb->s[i]*imgb->e[i];
            imgb->baddr[i] = evc_malloc(imgb->bsize[i]);

            imgb->a[i] = ((u8*)imgb->baddr[i]) + imgb->padu[i]*imgb->s[i] +
                imgb->padl[i]*bd;

            if(i == 0) 
            { 
#if BD_CF_EXT
                if((GET_CHROMA_W_SHIFT(idc)))
                    w = (w + 1) >> (GET_CHROMA_W_SHIFT(idc));
                if((GET_CHROMA_H_SHIFT(idc)))
                    h = (h + 1) >> (GET_CHROMA_H_SHIFT(idc));
#else
                w = (w+1)>>1; h = (h+1)>>1; 
#endif
            }
        }
#if BD_CF_EXT
        imgb->np = np;
#else
        imgb->np = 3;
#endif
    }
    else
    {
        evc_trace("unsupported color space\n");
        evc_mfree(imgb);
        return NULL;
    }
    imgb->addref = imgb_addref;
    imgb->getref = imgb_getref;
    imgb->release = imgb_release;
    imgb->cs = cs;
    imgb->addref(imgb);

    return imgb;
}
