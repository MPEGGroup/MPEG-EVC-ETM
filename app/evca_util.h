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

#ifndef _EVCA_UTIL_H_
#define _EVCA_UTIL_H_
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* for support  16bit aligned input source */
#define APP_PEL_16BIT
#ifdef APP_PEL_16BIT
#define CONV8_16BIT
#endif

#ifndef RECON_POC_ORDER
/* 0: let reconstructed pictures order to decoding order*/
/* 1: let reconstructed pictures order to display order*/
#define RECON_POC_ORDER            1
#endif

#define USE_CPU_TIME_CLOCK         1

#include <stdio.h>
#include <string.h>
#include <time.h>

/* print function ************************************************************/
#if _WIN32
#define print(args, ...) printf(args, __VA_ARGS__)
#else
#define print(args...) printf(args)
#endif

#define VERBOSE_0                  0
#define VERBOSE_1                  1
#define VERBOSE_2                  2

static int op_verbose = VERBOSE_2;

#if _WIN32
#define v0print(args,...) {if(op_verbose >= VERBOSE_0){print(args,__VA_ARGS__);}}
#define v1print(args,...) {if(op_verbose >= VERBOSE_1){print(args,__VA_ARGS__);}}
#define v2print(args,...) {if(op_verbose >= VERBOSE_2){print(args,__VA_ARGS__);}}
#else
#define v0print(args...) {if(op_verbose >= VERBOSE_0) {print(args);}}
#define v1print(args...) {if(op_verbose >= VERBOSE_1) {print(args);}}
#define v2print(args...) {if(op_verbose >= VERBOSE_2) {print(args);}}
#endif
/****************************************************************************/
#if _WIN32
#include <windows.h>

#define EVC_CLK             DWORD
#define EVC_CLK_PER_SEC     (1000)
#define EVC_CLK_PER_MSEC    (1)
#define EVC_CLK_MAX         ((EVC_CLK)(-1))
#if USE_CPU_TIME_CLOCK
#define evc_clk_get()       clock()
#else
#define evc_clk_get()       GetTickCount()
#endif
/****************************************************************************/
#elif __linux__ || __CYGWIN__
#include <time.h>
#include <sys/time.h>
#define EVC_CLK             unsigned long
#define EVC_CLK_MAX         ((EVC_CLK)(-1))
#define EVC_CLK_PER_SEC     (10000)
#define EVC_CLK_PER_MSEC    (10)
static EVC_CLK evc_clk_get(void)
{
    EVC_CLK clk;
#if USE_CPU_TIME_CLOCK
    clk = (EVC_CLK)(clock()) * 10000L / CLOCKS_PER_SEC;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    clk = t.tv_sec*10000L + t.tv_usec/100L;
#endif
    return clk;
}
/* The others ***************************************************************/
#else
#error THIS PLATFORM CANNOT SUPPORT CLOCK
#endif

#define evc_clk_diff(t1, t2) \
    (((t2) >= (t1)) ? ((t2) - (t1)) : ((EVC_CLK_MAX - (t1)) + (t2)))
#if REMOVE_WARNING
EVC_CLK evc_clk_from(EVC_CLK from) \
{
  EVC_CLK now = evc_clk_get(); \
    return evc_clk_diff(from, now); \
}
#else
static EVC_CLK evc_clk_from(EVC_CLK from) \
{
  EVC_CLK now = evc_clk_get(); \
    return evc_clk_diff(from, now); \
}
#endif


#define evc_clk_msec(clk) \
    ((int)((clk + (EVC_CLK_PER_MSEC/2))/EVC_CLK_PER_MSEC))
#define evc_clk_sec(clk)  \
    ((int)((clk + (EVC_CLK_PER_SEC/2))/EVC_CLK_PER_SEC))

#define IFVCA_CLIP(n,min,max) (((n)>(max))? (max) : (((n)<(min))? (min) : (n)))
#if REMOVE_WARNING
int imgb_read(FILE * fp, EVC_IMGB * img)
#else
static int imgb_read(FILE * fp, EVC_IMGB * img)
#endif
{
    int f_w, f_h;
    int y_size, u_size, v_size;

    f_w = img->w[0];
    f_h = img->h[0];

    if(img->cs == EVC_COLORSPACE_YUV420)
    {
        y_size = f_w * f_h;
        u_size = v_size = (f_w >> 1) * (f_h >> 1);

        if(fread(img->a[0], 1, y_size, fp) != (unsigned)y_size)
        {
            return -1;
        }
        if(fread(img->a[1], 1, u_size, fp) != (unsigned)u_size)
        {
            return -1;
        }
        if(fread(img->a[2], 1, v_size, fp) != (unsigned)v_size)
        {
            return -1;
        }
    }
    else if(img->cs == EVC_COLORSPACE_YUV420_10LE)
    {

        y_size = f_w * f_h * sizeof(short);
        u_size = v_size = (f_w >> 1) * (f_h >> 1) * sizeof(short);
        if(fread(img->a[0], 1, y_size, fp) != (unsigned)y_size)
        {
            return -1;
        }
        if(fread(img->a[1], 1, u_size, fp) != (unsigned)u_size)
        {
            return -1;
        }
        if(fread(img->a[2], 1, v_size, fp) != (unsigned)v_size)
        {
            return -1;
        }
    }
    else
    {
        print("not supported color space\n");
        return -1;
    }

    return 0;
}
#if REMOVE_WARNING
int imgb_write(char * fname, EVC_IMGB * img)
#else
static int imgb_write(char * fname, EVC_IMGB * img)
#endif
{
    unsigned char * p8;
    int             i, j, bd;
    FILE          * fp;

    fp = fopen(fname, "ab");
    if(fp == NULL)
    {
        print("cannot open file = %s\n", fname);
        return -1;
    }
    if(img->cs == EVC_COLORSPACE_YUV420_10LE)
    {
        bd = 2;
    }
    else if(img->cs == EVC_COLORSPACE_YUV420)
    {
        bd = 1;
    }
    else
    {
        print("cannot support the color space\n");
        return -1;
    }

    for(i = 0; i < 3; i++)
    {
        p8 = (unsigned char *)img->a[i] + (img->s[i] * img->y[i]) + (img->x[i] * bd);

        for(j = 0; j < img->h[i]; j++)
        {
            fwrite(p8, img->w[i] * bd, 1, fp);
            p8 += img->s[i];
        }
    }
    fclose(fp);
    return 0;
}

static void __imgb_cpy_plane(void *src, void *dst, int bw, int h, int s_src,
                             int s_dst)
{
    int i;
    unsigned char *s, *d;

    s = (unsigned char*)src;
    d = (unsigned char*)dst;

    for(i = 0; i < h; i++)
    {
        memcpy(d, s, bw);
        s += s_src;
        d += s_dst;
    }
}
#if REMOVE_WARNING
int write_data(char * fname, unsigned char * data, int size)
#else
static int write_data(char * fname, unsigned char * data, int size)
#endif
{
    FILE * fp;

    fp = fopen(fname, "ab");
    if(fp == NULL)
    {
        v0print("cannot open an writing file=%s\n", fname);
        return -1;
    }
    fwrite(data, 1, size, fp);
    fclose(fp);
    return 0;
}

static void imgb_conv_8b_to_16b(EVC_IMGB * imgb_dst, EVC_IMGB * imgb_src,
                                int shift)
{
    int i, j, k;

    unsigned char * s;
    short         * d;

    for(i = 0; i < 3; i++)
    {
        s = imgb_src->a[i];
        d = imgb_dst->a[i];

        for(j = 0; j < imgb_src->h[i]; j++)
        {
            for(k = 0; k < imgb_src->w[i]; k++)
            {
                d[k] = (short)(s[k] << shift);
            }
            s = s + imgb_src->s[i];
            d = (short*)(((unsigned char *)d) + imgb_dst->s[i]);
        }
    }
}

static void imgb_conv_16b_to_8b(EVC_IMGB * imgb_dst, EVC_IMGB * imgb_src,
                                int shift)
{

    int i, j, k, t0, add;

    short         * s;
    unsigned char * d;

    add = 1 << (shift - 1);

    for(i = 0; i < 3; i++)
    {
        s = imgb_src->a[i];
        d = imgb_dst->a[i];

        for(j = 0; j < imgb_src->h[i]; j++)
        {
            for(k = 0; k < imgb_src->w[i]; k++)
            {
                t0 = ((s[k] + add) >> shift);
                d[k] = (unsigned char)(IFVCA_CLIP(t0, 0, 255));

            }
            s = (short*)(((unsigned char *)s) + imgb_src->s[i]);
            d = d + imgb_dst->s[i];
        }
    }
}
#if REMOVE_WARNING
void imgb_cpy(EVC_IMGB * dst, EVC_IMGB * src)
#else
static void imgb_cpy(EVC_IMGB * dst, EVC_IMGB * src)
#endif
{
    int i, bd;

    if(src->cs == dst->cs)
    {
        if(src->cs == EVC_COLORSPACE_YUV420_10LE) bd = 2;
        else bd = 1;

        for(i = 0; i < src->np; i++)
        {
            __imgb_cpy_plane(src->a[i], dst->a[i], bd*src->w[i], src->h[i],
                             src->s[i], dst->s[i]);

        }
    }
    else if(src->cs == EVC_COLORSPACE_YUV420 &&
            dst->cs == EVC_COLORSPACE_YUV420_10LE)
    {
        imgb_conv_8b_to_16b(dst, src, 2);
    }
    else if(src->cs == EVC_COLORSPACE_YUV420_10LE &&
            dst->cs == EVC_COLORSPACE_YUV420)
    {
        imgb_conv_16b_to_8b(dst, src, 2);
    }
    else
    {
        v0print("ERROR: unsupported image copy\n");
        return;
    }
    for(i = 0; i < 4; i++)
    {
        dst->ts[i] = src->ts[i];
    }
}

static void imgb_free(EVC_IMGB * imgb)
{
    int i;
    for(i = 0; i < EVC_IMGB_MAX_PLANE; i++)
    {
        if(imgb->baddr[i]) free(imgb->baddr[i]);
    }
    free(imgb);
}

EVC_IMGB * imgb_alloc(int w, int h, int cs)
{
    int i;
    EVC_IMGB * imgb;

    imgb = (EVC_IMGB *)malloc(sizeof(EVC_IMGB));
    if(imgb == NULL)
    {
        v0print("cannot create image buffer\n");
        return NULL;
    }
    memset(imgb, 0, sizeof(EVC_IMGB));

    if(cs == EVC_COLORSPACE_YUV420)
    {
        for(i = 0; i < 3; i++)
        {
            imgb->w[i] = imgb->aw[i] = imgb->s[i] = w;
            imgb->h[i] = imgb->ah[i] = imgb->e[i] = h;
            imgb->bsize[i] = imgb->s[i] * imgb->e[i];

            imgb->a[i] = imgb->baddr[i] = malloc(imgb->bsize[i]);
            if(imgb->a[i] == NULL)
            {
                v0print("cannot allocate picture buffer\n");
                return NULL;
            }

            if(i == 0)
            {
                w = (w + 1) >> 1; h = (h + 1) >> 1;
            }
        }
        imgb->np = 3;
    }
    else if(cs == EVC_COLORSPACE_YUV420_10LE)
    {
        for(i = 0; i < 3; i++)
        {
            imgb->w[i] = imgb->aw[i] = w;
            imgb->s[i] = w * sizeof(short);
            imgb->h[i] = imgb->ah[i] = imgb->e[i] = h;
            imgb->bsize[i] = imgb->s[i] * imgb->e[i];

            imgb->a[i] = imgb->baddr[i] = malloc(imgb->bsize[i]);
            if(imgb->a[i] == NULL)
            {
                v0print("cannot allocate picture buffer\n");
                return NULL;
            }

            if(i == 0)
            {
                w = (w + 1) >> 1; h = (h + 1) >> 1;
            }
        }
        imgb->np = 3;
    }
    else
    {
        v0print("unsupported color space\n");
        if(imgb)free(imgb);
        return NULL;
    }

    imgb->cs = cs;
    return imgb;
}

#ifdef __cplusplus
}
#endif

#endif /* _EVCA_UTIL_H_ */
