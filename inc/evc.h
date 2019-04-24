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


#ifndef _EVC_H_
#define _EVC_H_


#ifdef __cplusplus
extern "C"
{
#endif

#define USE_TILE_GROUP_DQP              1

/*****************************************************************************
 * return values and error code
 *****************************************************************************/
/* no more frames, but it is OK */
#define EVC_OK_NO_MORE_FRM              (205)
/* progress success, but output is not available temporarily */
#define EVC_OK_OUT_NOT_AVAILABLE        (204)
/* frame dimension (width or height) has been changed */
#define EVC_OK_DIM_CHANGED              (203)
/* decoding success, but output frame has been delayed */
#define EVC_OK_FRM_DELAYED              (202)
#define EVC_OK                          (0)

#define EVC_ERR                         (-1) /* generic error */
#define EVC_ERR_INVALID_ARGUMENT        (-101)
#define EVC_ERR_OUT_OF_MEMORY           (-102)
#define EVC_ERR_REACHED_MAX             (-103)
#define EVC_ERR_UNSUPPORTED             (-104)
#define EVC_ERR_UNEXPECTED              (-105)
#define EVC_ERR_BAD_CRC                 (-130) /* not matched CRC value */

#define EVC_ERR_UNSUPPORTED_COLORSPACE  (-201)
#define EVC_ERR_MALFORMED_BITSTREAM     (-202)

#define EVC_ERR_UNKNOWN                 (-32767) /* unknown error */


/* return value checking *****************************************************/
#define EVC_SUCCEEDED(ret)              ((ret) >= 0)
#define EVC_FAILED(ret)                 ((ret) < 0)

/*****************************************************************************
 * color spaces
 *****************************************************************************/
#define EVC_COLORSPACE_UNKNOWN          0 /* unknown color space */

/* YUV planar ****************************************************************/
#define EVC_COLORSPACE_YUV_PLANAR_START 100

/* YUV planar 8bit */
#define EVC_COLORSPACE_YUV400          300 /* Y 8bit */
#define EVC_COLORSPACE_YUV420          301 /* YUV420 8bit */
#define EVC_COLORSPACE_YUV422          302 /* YUV422 8bit narrow chroma*/
#define EVC_COLORSPACE_YUV444          303 /* YUV444 8bit */
#define EVC_COLORSPACE_YUV422N         EVC_COLORSPACE_YUV422
#define EVC_COLORSPACE_YUV422W         310 /* YUV422 8bit wide chroma */

#define EVC_COLORSPACE_YUV400A8        400 /* Y+alpha 8bit */
#define EVC_COLORSPACE_YUV420A8        401 /* YUV420+alpha 8bit */
#define EVC_COLORSPACE_YUV422A8        402 /* YUV422+alpha 8bit narrow chroma*/
#define EVC_COLORSPACE_YUV444A8        403 /* YUV444+alpha 8bit */
#define EVC_COLORSPACE_YUV422NA8       EVC_COLORSPACE_YUV422A8
#define EVC_COLORSPACE_YUV422WA8       414 /* YUV422+alpha 8bit wide chroma*/

/* YUV planar 10bit */
#define EVC_COLORSPACE_YUV400_10LE     500 /* Y 10bit little-endian */
#define EVC_COLORSPACE_YUV400_10BE     501 /* Y 10bit big-endian */
#define EVC_COLORSPACE_YUV420_10LE     502 /* YUV420 10bit little-endian */
#define EVC_COLORSPACE_YUV420_10BE     503 /* YUV420 10bit big-endian */
#define EVC_COLORSPACE_YUV422_10LE     504 /* YUV422 10bit little-endian */
#define EVC_COLORSPACE_YUV422_10BE     505 /* YUV422 10bit big-endian */
#define EVC_COLORSPACE_YUV444_10LE     506 /* YUV444 10bit little-endian */
#define EVC_COLORSPACE_YUV444_10BE     507 /* YUV444 10bit big-endian */

#define EVC_COLORSPACE_YUV_PLANAR_END  999

/* RGB pack ******************************************************************/
#define EVC_COLORSPACE_RGB_PACK_START  2000

/* RGB pack 8bit */
#define EVC_COLORSPACE_RGB888          2200
#define EVC_COLORSPACE_BGR888          2201

#define EVC_COLORSPACE_RGBA8888        2220
#define EVC_COLORSPACE_BGRA8888        2221
#define EVC_COLORSPACE_ARGB8888        2222
#define EVC_COLORSPACE_ABGR8888        2223

#define EVC_COLORSPACE_RGB_PACK_END    2999

/* macro for colorspace checking *********************************************/
#define EVC_COLORSPACE_IS_YUV_PLANAR(cs)   \
    ((cs)>=EVC_COLORSPACE_YUV_PLANAR_START && (cs)<=EVC_COLORSPACE_YUV_PLANAR_END)

#define EVC_COLORSPACE_IS_RGB_PACK(cs)   \
    ((cs)>=EVC_COLORSPACE_RGB_PACK_START &&  (cs)<=EVC_COLORSPACE_RGB_PACK_END)

/*****************************************************************************
 * config types for decoder
 *****************************************************************************/
#define EVCD_CFG_SET_USE_PIC_SIGNATURE  (301)

/*****************************************************************************
 * config types for encoder
 *****************************************************************************/
#define EVCE_CFG_SET_COMPLEXITY         (100)
#define EVCE_CFG_SET_SPEED              (101)
#define EVCE_CFG_SET_FORCE_OUT          (102)

#define EVCE_CFG_SET_FINTRA             (200)
#define EVCE_CFG_SET_QP                 (201)
#define EVCE_CFG_SET_BPS                (202)
#define EVCE_CFG_SET_VBV_SIZE           (203)
#define EVCE_CFG_SET_FPS                (204)
#define EVCE_CFG_SET_I_PERIOD           (207)
#define EVCE_CFG_SET_QP_MIN             (208)
#define EVCE_CFG_SET_QP_MAX             (209)
#define EVCE_CFG_SET_BU_SIZE            (210)
#define EVCE_CFG_SET_USE_DEBLOCK        (211)

#define EVCE_CFG_SET_USE_PIC_SIGNATURE  (301)

#define EVCE_CFG_GET_COMPLEXITY         (500)
#define EVCE_CFG_GET_SPEED              (501)

#define EVCE_CFG_GET_QP_MIN             (600)
#define EVCE_CFG_GET_QP_MAX             (601)
#define EVCE_CFG_GET_QP                 (602)
#define EVCE_CFG_GET_RCT                (603)
#define EVCE_CFG_GET_BPS                (604)
#define EVCE_CFG_GET_FPS                (605)
#define EVCE_CFG_GET_I_PERIOD           (608)
#define EVCE_CFG_GET_BU_SIZE            (609)
#define EVCE_CFG_GET_USE_DEBLOCK        (610)
#define EVCE_CFG_GET_CLOSED_GOP         (611)
#define EVCE_CFG_GET_HIERARCHICAL_GOP   (612)

#define EVCE_CFG_GET_WIDTH              (701)
#define EVCE_CFG_GET_HEIGHT             (702)
#define EVCE_CFG_GET_RECON              (703)


/*****************************************************************************
 * chunk type
 *****************************************************************************/
#define EVC_CT_UNKNOWN                  (0)
#define EVC_CT_TILE_GROUP                    (1) /* tile_group */
#define EVC_CT_SPS                      (2) /* sequence parameter set */
#define EVC_CT_SIGN                     (6) /* picture signature */

/*****************************************************************************
 * tile_group type
 *****************************************************************************/
#define EVC_ST_UNKNOWN                  (0)
#define EVC_ST_I                        (1)
#define EVC_ST_P                        (2)
#define EVC_ST_B                        (3)

/*****************************************************************************
 * evc version
 *****************************************************************************/
#define EVC_VER_UNKNOWN                 (0)
#define EVC_VER_1                       (1)

/*****************************************************************************
 * type and macro for media time
 *****************************************************************************/
/* media time in 100-nanosec unit */
typedef long long                    EVC_MTIME;

/*****************************************************************************
 * image buffer format
 *****************************************************************************
 baddr
    +---------------------------------------------------+ ---
    |                                                   |  ^
    |                                              |    |  |
    |    a                                         v    |  |
    |   --- +-----------------------------------+ ---   |  |
    |    ^  |  (x, y)                           |  y    |  |
    |    |  |   +---------------------------+   + ---   |  |
    |    |  |   |                           |   |  ^    |  |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  |    |  |
    |       |   |                           |   |       |
    |    ah |   |                           |   |  h    |  e
    |       |   |                           |   |       |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  v    |  |
    |    |  |   +---------------------------+   | ---   |  |
    |    v  |                                   |       |  |
    |   --- +---+-------------------------------+       |  |
    |     ->| x |<----------- w ----------->|           |  |
    |       |<--------------- aw -------------->|       |  |
    |                                                   |  v
    +---------------------------------------------------+ ---

    |<---------------------- s ------------------------>|

 *****************************************************************************/

#define EVC_IMGB_MAX_PLANE              (4)

typedef struct _EVC_IMGB EVC_IMGB;
struct _EVC_IMGB
{
    int                 cs; /* color space */
    int                 np; /* number of plane */
    /* width (in unit of pixel) */
    int                 w[EVC_IMGB_MAX_PLANE];
    /* height (in unit of pixel) */
    int                 h[EVC_IMGB_MAX_PLANE];
    /* X position of left top (in unit of pixel) */
    int                 x[EVC_IMGB_MAX_PLANE];
    /* Y postion of left top (in unit of pixel) */
    int                 y[EVC_IMGB_MAX_PLANE];
    /* buffer stride (in unit of byte) */
    int                 s[EVC_IMGB_MAX_PLANE];
    /* buffer elevation (in unit of byte) */
    int                 e[EVC_IMGB_MAX_PLANE];
    /* address of each plane */
    void              * a[EVC_IMGB_MAX_PLANE];

    /* time-stamps */
    EVC_MTIME          ts[4];

    int                 ndata[4]; /* arbitrary data, if needs */
    void              * pdata[4]; /* arbitrary adedress if needs */

    /* aligned width (in unit of pixel) */
    int                 aw[EVC_IMGB_MAX_PLANE];
    /* aligned height (in unit of pixel) */
    int                 ah[EVC_IMGB_MAX_PLANE];

    /* left padding size (in unit of pixel) */
    int                 padl[EVC_IMGB_MAX_PLANE];
    /* right padding size (in unit of pixel) */
    int                 padr[EVC_IMGB_MAX_PLANE];
    /* up padding size (in unit of pixel) */
    int                 padu[EVC_IMGB_MAX_PLANE];
    /* bottom padding size (in unit of pixel) */
    int                 padb[EVC_IMGB_MAX_PLANE];

    /* address of actual allocated buffer */
    void              * baddr[EVC_IMGB_MAX_PLANE];
    /* actual allocated buffer size */
    int                 bsize[EVC_IMGB_MAX_PLANE];

    /* life cycle management */
    int                 refcnt;
    int                 (*addref)(EVC_IMGB * imgb);
    int                 (*getref)(EVC_IMGB * imgb);
    int                 (*release)(EVC_IMGB * imgb);
};


/*****************************************************************************
 * Bitstream buffer
 *****************************************************************************/
typedef struct _EVC_BITB
{
    /* user space address indicating buffer */
    void              * addr;
    /* physical address indicating buffer, if any */
    void              * pddr;
    /* byte size of buffer memory */
    int                 bsize;
    /* byte size of bitstream in buffer */
    int                 ssize;
    /* bitstream has an error? */
    int                 err;
    /* arbitrary data, if needs */
    int                 ndata[4];
    /* arbitrary address, if needs */
    void              * pdata[4];
    /* time-stamps */
    EVC_MTIME          ts[4];

} EVC_BITB;

/*****************************************************************************
 * description for creating of decoder
 *****************************************************************************/
typedef struct _EVCD_CDSC
{
    int            __na; /* nothing */
} EVCD_CDSC;

/*****************************************************************************
 * status after decoder operation
 *****************************************************************************/
typedef struct _EVCD_STAT
{
    /* byte size of decoded bitstream (read size of bitstream) */
    int            read;
    /* chunk type */
    int            ctype;
    /* tile_group type */
    int            stype;
    /* frame number monotonically increased whenever decoding a frame.
    note that it has negative value if the decoded data is not frame */
    int            fnum;
    /* picture order count */
    int            poc;
    /* layer id */
    int            tid;

    /* number of reference pictures */
    unsigned char  refpic_num[2];
    /* list of reference pictures */
    int            refpic[2][16];
} EVCD_STAT;

#define MAX_NUM_REF_PICS                   21
#define MAX_NUM_ACTIVE_REF_FRAME           5
#define MAX_NUM_RPLS                       32

/* rpl structure */
typedef struct _EVC_RPL
{
    int poc;
    int tid;
    int ref_pic_num;
    int ref_pic_active_num;
    int ref_pics[MAX_NUM_REF_PICS];
} EVC_RPL;

/*****************************************************************************
 * description for creating of encoder
 *****************************************************************************/
typedef struct _EVCE_CDSC
{
    /* width of input frame */
    int            w;
    /* height of input frame */
    int            h;
    /* frame rate (Hz) */
    int            fps;
    /* period of I-frame.
    - 0: only one I-frame at the first time.
    - 1: every frame is coded in I-frame
    */
    int            iperiod;
    /* quantization parameter.
    if the rate control is enabled, the value would be ignored */
    int            qp;
    /* color space of input image */
    int            cs;
    /* The maximum number of consecutive B frames (up to 7)
       - Default: Off(0)                                                      */
    int            max_b_frames;
    /* Has meaning only when max_b_frames is more than 0
       - enable(0) means use hierarchy GOP structure for B frmeas
               is valid only when max_b_frames is equal to 1, 3 and 7
       - disable (1) means frame type will be decided automatically
       - Default: enable(0)                                                       */
    int            disable_hgop;
    int            ref_pic_gap_length;
    /* use closed GOP sturcture
       - 0 : use open GOP (default)
       - 1 : use closed GOP */
    int            closed_gop;
    /* bit depth of input video */
    int            in_bit_depth;
    /* bit depth of output video */
    int            out_bit_depth;
#if USE_TILE_GROUP_DQP
    int            add_qp_frame;
#endif
    int            framework_ctu_size;
    int            framework_cu11_max;
    int            framework_cu11_min;
    int            framework_cu12_max;
    int            framework_cu12_min;
    int            framework_cu14_max;
    int            framework_cu14_min;
    int            framework_tris_max;
    int            framework_tris_min;
    int            framework_suco_max;
    int            framework_suco_min;
    int            tool_amvr;
    int            tool_mmvd;
    int            tool_affine;
    int            tool_dmvr;
    int            tool_alf;
    int            tool_htdf;
    int            tool_admvp;
    int            tool_amis;

    EVC_RPL rpls_l0[MAX_NUM_RPLS];
    EVC_RPL rpls_l1[MAX_NUM_RPLS];
    int rpls_l0_cfg_num;
    int rpls_l1_cfg_num;

} EVCE_CDSC;

/*****************************************************************************
 * status after encoder operation
 *****************************************************************************/
typedef struct _EVCE_STAT
{
    /* encoded bitstream byte size */
    int            write;
    /* picture number increased whenever encoding a frame */
    unsigned long  fnum;
    /* chunk type */
    int            ctype;
    /* tile_group type */
    int            stype;
    /* quantization parameter used for encoding */
    int            qp;
    /* picture order count */
    int            poc;
    /* layer id */
    int            tid;
    /* number of reference pictures */
    int            refpic_num[2];
    /* list of reference pictures */
    int            refpic[2][16];

} EVCE_STAT;

/*****************************************************************************
 * API for decoder only
 *****************************************************************************/
/* EVC instance identifier for decoder */
typedef void  * EVCD;

EVCD evcd_create(EVCD_CDSC * cdsc, int * err);
void evcd_delete(EVCD id);
int evcd_decode(EVCD id, EVC_BITB * bitb, EVCD_STAT * stat);
int evcd_pull(EVCD id, EVC_IMGB ** img);
int evcd_config(EVCD id, int cfg, void * buf, int * size);

/*****************************************************************************
 * API for encoder only
 *****************************************************************************/
/* EVC instance identifier for encoder */
typedef void  * EVCE;

EVCE evce_create(EVCE_CDSC * cdsc, int * err);
void evce_delete(EVCE id);
int evce_push(EVCE id, EVC_IMGB * imgb);
int evce_encode(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat);
int evce_encode_header(EVCE id, EVC_BITB * bitb,  EVCE_STAT * stat);
int evce_get_inbuf(EVCE id, EVC_IMGB ** imgb);
int evce_config(EVCE id, int cfg, void * buf, int * size);

#ifdef __cplusplus
}
#endif

#endif /* _EVC_H_ */
