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

#ifndef _EVCE_DEF_H_
#define _EVCE_DEF_H_

#include "evc_def.h"
#include "evce_bsw.h"
#include "evce_sad.h"


/* support RDOQ */
#define SCALE_BITS               15    /* Inherited from TMuC, pressumably for fractional bit estimates in RDOQ */
#if USE_RDOQ
#define ERR_SCALE_PRECISION_BITS 20
#endif
/* EVC encoder magic code */
#define EVCE_MAGIC_CODE         0x45565945 /* EVYE */

/* Max. and min. Quantization parameter */
#define MAX_QUANT                51
#define MIN_QUANT                0

#define GOP_P                    4

/* count of picture including encoding and reference pictures
0: encoding picture buffer
1: forward reference picture buffer
2: backward reference picture buffer, if exists
3: original (input) picture buffer
4: mode decision picture buffer, if exists
*/
#define PIC_D                    5
/* current encoding picture buffer index */
#define PIC_IDX_CURR             0
/* list0 reference picture buffer index */
#define PIC_IDX_FORW             1
/* list1 reference picture buffer index */
#define PIC_IDX_BACK             2
/* original (input) picture buffer index */
#define PIC_IDX_ORIG             3
/* mode decision picture buffer index */
#define PIC_IDX_MODE             4

/* check whether bumping is progress or not */
#define FORCE_OUT(ctx)          (ctx->param.force_output == 1)

/* motion vector accuracy level for inter-mode decision */
#define ME_LEV_IPEL              1
#define ME_LEV_HPEL              2
#define ME_LEV_QPEL              3

/* maximum inbuf count */
#if HLS_M47668
#define EVCE_MAX_INBUF_CNT      33
#else
#define EVCE_MAX_INBUF_CNT      32

#endif

/* maximum cost value */
#define MAX_COST                (1.7e+308)


/*****************************************************************************
 * mode decision structure
 *****************************************************************************/
typedef struct _EVCE_MODE
{
    void *pdata[4];
    int  *ndata[4];
    pel  *rec[N_C];
    int   s_rec[N_C];

    /* CU count in a CU row in a LCU (== log2_max_cuwh - MIN_CU_LOG2) */
    u8    log2_culine;
    /* reference indices */
    s8    refi[REFP_NUM];
    /* MVP indices */
    u8    mvp_idx[REFP_NUM];
    /* MVR indices */
    u8    mvr_idx;
    u8    bi_idx;
    s16   mmvd_idx;
    /* mv difference */
    s16   mvd[REFP_NUM][MV_D];    
#if DMVR_LAG
    /* mv */
    s16   dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
#endif
    /* mv */
    s16   mv[REFP_NUM][MV_D];

    pel  *pred_y_best;

#if AFFINE
    s16   affine_mv[REFP_NUM][VER_NUM][MV_D];
    s16   affine_mvd[REFP_NUM][VER_NUM][MV_D];
#endif

#if ADMVP
    int   cu_mode;
    u8    affine_flag;
#endif

#if AFFINE_UPDATE && AFFINE
    // spatial neighboring MV of affine block
    s8    refi_sp[REFP_NUM];
    s16   mv_sp[REFP_NUM][MV_D];
#endif
#if TRACE_ENC_CU_DATA
    u64   trace_cu_idx;
#endif
#if TRACE_ENC_HISTORIC
    EVC_HISTORY_BUFFER     history_buf;
#endif
} EVCE_MODE;

/* virtual frame depth B picture */
#define FRM_DEPTH_0                   0
#define FRM_DEPTH_1                   1
#define FRM_DEPTH_2                   2
#define FRM_DEPTH_3                   3
#define FRM_DEPTH_4                   4
#define FRM_DEPTH_5                   5
#if HLS_M47668
#define FRM_DEPTH_6                   6
#define FRM_DEPTH_MAX                 7
#else
#define FRM_DEPTH_MAX                 6
#endif
/* I-tile_group, P-tile_group, B-tile_group + depth + 1 (max for GOP 8 size)*/
#define LIST_NUM                      1

/*****************************************************************************
 * original picture buffer structure
 *****************************************************************************/
typedef struct _EVCE_PICO
{
    /* original picture store */
    EVC_PIC                pic;
    /* input picture count */
    u32                     pic_icnt;
    /* be used for encoding input */
    u8                      is_used;

    /* address of sub-picture */
    EVC_PIC              * spic;
} EVCE_PICO;

/*****************************************************************************
 * intra prediction structure
 *****************************************************************************/
typedef struct _EVCE_PINTRA
{
    /* temporary prediction buffer */
    pel                 pred[N_C][MAX_CU_DIM];
    pel                 pred_cache[IPD_CNT][MAX_CU_DIM]; // only for luma

    /* reconstruction buffer */
    pel                 rec[N_C][MAX_CU_DIM];

    s16                 coef_tmp[N_C][MAX_CU_DIM];
    s16                 coef_best[N_C][MAX_CU_DIM];
    int                 nnz_best[N_C];
    int                 nnz_sub_best[N_C][MAX_SUB_TB_NUM];
    pel                 rec_best[N_C][MAX_CU_DIM];

    /* original (input) picture buffer */
    EVC_PIC          * pic_o;
    /* address of original (input) picture buffer */
    pel               * o[N_C];
    /* stride of original (input) picture buffer */
    int                 s_o[N_C];
    /* mode picture buffer */
    EVC_PIC          * pic_m;
    /* address of mode picture buffer */

    pel               * m[N_C];
    /* stride of mode picture buffer */
    int                 s_m[N_C];

    /* QP for luma */
    u8                  qp_y;
    /* QP for chroma */
    u8                  qp_u;
    u8                  qp_v;

    int                 tile_group_type;

    int                 complexity;
    void              * pdata[4];
    int               * ndata[4];
} EVCE_PINTRA;

/*****************************************************************************
 * inter prediction structure
 *****************************************************************************/
#define MV_RANGE_MIN           0
#define MV_RANGE_MAX           1
#define MV_RANGE_DIM           2

typedef struct _EVCE_PINTER EVCE_PINTER;
struct _EVCE_PINTER
{
    /* temporary prediction buffer (only used for ME)*/
    pel  pred_buf[MAX_CU_DIM];
    /* reconstruction buffer */
    pel  rec_buf[N_C][MAX_CU_DIM];
    /* temporary buffer for analyze_cu */
    s8   refi[PRED_NUM][REFP_NUM];
    /* Ref idx predictor */
    s8   refi_pred[REFP_NUM][MAX_NUM_MVP]; 
    u8   mvp_idx[PRED_NUM][REFP_NUM];

    s16  mvp_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MAX_NUM_MVP][MV_D];
    s16  mv_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
    u8   mvp_idx_temp_for_bi[PRED_NUM][REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME];
    int  best_index[PRED_NUM][4];
    s16  mmvd_idx[PRED_NUM];
    u8   mvr_idx[PRED_NUM];
    u8   curr_mvr;
    int  max_imv[MV_D];
    s8   first_refi[PRED_NUM][REFP_NUM];
    u8   bi_idx[PRED_NUM];
    u8   curr_bi;
    int max_search_range;
#if AFFINE
    s16  affine_mvp_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MAX_NUM_MVP][VER_NUM][MV_D];
    s16  affine_mv_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][VER_NUM][MV_D];
    u8   mvp_idx_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME];

    s16  affine_mvp[REFP_NUM][MAX_NUM_MVP][VER_NUM][MV_D];
    s16  affine_mv[PRED_NUM][REFP_NUM][VER_NUM][MV_D];
    s16  affine_mvd[PRED_NUM][REFP_NUM][VER_NUM][MV_D];

    pel  p_error[MAX_CU_DIM];
    int  i_gradient[2][MAX_CU_DIM];
#endif

    /* MV predictor */
    s16  mvp[REFP_NUM][MAX_NUM_MVP][MV_D]; 
#if DMVR_LAG
    s16  dmvr_mv[PRED_NUM][MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
#endif
    s16  mv[PRED_NUM][REFP_NUM][MV_D];
    s16  mvd[PRED_NUM][REFP_NUM][MV_D];

    s16  org_bi[MAX_CU_DIM];
    s32  mot_bits[REFP_NUM];
    /* temporary prediction buffer (only used for ME)*/
    pel  pred[PRED_NUM+1][2][N_C][MAX_CU_DIM];

#if DMVR
    pel  dmvr_template[MAX_CU_DIM];
    pel dmvr_half_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
#if DMVR_PADDING
    pel  dmvr_padding_buf[PRED_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE];
#endif
    pel  dmvr_ref_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))];
#endif

    /* reconstruction buffer */
    pel  rec[PRED_NUM][N_C][MAX_CU_DIM];
    /* last one buffer used for RDO */
    s16  coef[PRED_NUM+1][N_C][MAX_CU_DIM];

    s16  residue[N_C][MAX_CU_DIM];
    int  nnz_best[PRED_NUM][N_C];
    int  nnz_sub_best[PRED_NUM][N_C][MAX_SUB_TB_NUM];

    u8   num_refp;
    /* minimum clip value */
    s16  min_clip[MV_D];
    /* maximum clip value */
    s16  max_clip[MV_D]; 
    /* search range for int-pel */
    s16  search_range_ipel[MV_D]; 
    /* search range for sub-pel */
    s16  search_range_spel[MV_D]; 
    s8  (*search_pattern_hpel)[2];
    u8   search_pattern_hpel_cnt;
    s8  (*search_pattern_qpel)[2];
    u8   search_pattern_qpel_cnt;

    /* original (input) picture buffer */
    EVC_PIC        *pic_o;
    /* address of original (input) picture buffer */
    pel             *o[N_C];
    /* stride of original (input) picture buffer */
    int              s_o[N_C];
    /* mode picture buffer */
    EVC_PIC        *pic_m;
    /* address of mode picture buffer */
    pel             *m[N_C];
    /* stride of mode picture buffer */
    int              s_m[N_C];
    /* motion vector map */
    s16            (*map_mv)[REFP_NUM][MV_D];
#if DMVR_LAG
    /* unrefined motion vector map */
    s16(*map_unrefined_mv)[REFP_NUM][MV_D];
#endif
    /* picture width in SCU unit */
    u16              w_scu;
    /* QP for luma of current encoding CU */
    u8               qp_y;
    /* QP for chroma of current encoding CU */
    u8               qp_u;
    u8               qp_v;
    u32              lambda_mv;
    /* reference pictures */
    EVC_REFP      (*refp)[REFP_NUM];
    int              tile_group_type;
    /* search level for motion estimation */
    int              me_level;
    int              complexity;
    void            *pdata[4];
    int             *ndata[4];
    /* current frame numbser */
    int              ptr;
    /* gop size */
    int              gop_size;
    int              sps_amvr_flag;
    /* ME function (Full-ME or Fast-ME) */
    u32            (*fn_me)(EVCE_PINTER *pi, int x, int y, int log2_cuw, int log2_cuh, s8 *refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi);
#if AFFINE
    /* AFFINE ME function (Gradient-ME) */
    u32            (*fn_affine_me)(EVCE_PINTER *pi, int x, int y, int log2_cuw, int log2_cuh, s8 *refi, int lidx, s16 mvp[VER_NUM][MV_D], s16 mv[VER_NUM][MV_D], int bi, int vertex_num
#if EIF
                                   , pel *tmp
#endif
                                   );
#endif
};

/* EVC encoder parameter */
typedef struct _EVCE_PARAM
{
    /* picture size of input sequence (width) */
    int                 w;
    /* picture size of input sequence (height) */
    int                 h;
    /* picture bit depth*/
    int                 bit_depth;
    /* qp value for I- and P- tile_group */
    int                 qp;
    /* frame per second */
    int                 fps;
    /* Enable deblocking filter or not
       - 0: Disable deblocking filter
       - 1: Enable deblocking filter
    */
    int                 use_deblock;
#if ALF
    int                 use_alf;
#endif
    /* I-frame period */
    int                 i_period;
    /* force I-frame */
    int                 f_ifrm;
    /* Maximum qp value */
    int                 qp_max;
    /* Minimum qp value */
    int                 qp_min;
    /* use picture signature embedding */
    int                 use_pic_sign;
    int                 max_b_frames;
#if HLS_M47668
    int                 ref_pic_gap_length;
#endif
    /* start bumping process if force_output is on */
    int                 force_output;
    int                 gop_size;
    int                 use_dqp;
    int                 use_closed_gop;
    int                 use_hgop;
#if USE_TILE_GROUP_DQP
    int                 qp_incread_frame;           /* 10 bits*/
#endif

} EVCE_PARAM;

typedef struct _EVCE_SBAC
{
    u32            range;
    u32            code;
    u32            code_bits;
    u32            stacked_ff;
    u32            stacked_zero;
    u32            pending_byte;
    u32            is_pending_byte;
    EVC_SBAC_CTX   ctx;
    u32            bitcounter;
    u8             is_bitcount;
} EVCE_SBAC;

typedef struct _EVCE_CU_DATA
{
    s8  split_mode[MAX_CU_DEPTH][NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU];
    s8  suco_flag[MAX_CU_DEPTH][NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU];
    u8  *qp_y;
    u8  *qp_u;
    u8  *qp_v;
    u8  *pred_mode;
    u8  **mpm;
    u8  **mpm_ext;
    s8  **ipm;
    u8  *skip_flag;
#if DMVR_FLAG
    u8  *dmvr_flag;
#endif
    s8  **refi;    
    u8  **mvp_idx; 
    u8  *mvr_idx;
    u8  *bi_idx;
    s16 *mmvd_idx;
    u8  *mmvd_flag;
    s16 mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
#if DMVR_LAG
    s16 unrefined_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
#endif
    s16 mvd[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
    int *nnz[N_C];
    int *nnz_sub[N_C][4];
    u32 *map_scu;
#if AFFINE
    u8  *affine_flag;
    u32 *map_affine;
#endif
    u32 *map_cu_mode;
    s16 **block_size;
    s8  *depth;

    s16 *coef[N_C]; 
    pel *reco[N_C]; 
#if TRACE_ENC_CU_DATA
    u64  trace_idx[MAX_CU_CNT_IN_LCU];
#endif
#if TRACE_ENC_HISTORIC
    EVC_HISTORY_BUFFER     history_buf[MAX_CU_CNT_IN_LCU];
#endif
} EVCE_CU_DATA;

typedef struct _EVCE_BEF_DATA
{
    int    visit;
    int    nosplit;
    int    split;
    int    ipm[2];
    int    split_visit;
    double split_cost[MAX_SPLIT_NUM];
    /* splits which are not tried in the first visit (each bit corresponds to one split mode)*/
    u8     remaining_split;
    int    suco[3];
    int    mvr_idx;
    int    bi_idx;
    s16    mmvd_idx;
#if AFFINE
    int    affine_flag;
#endif

} EVCE_BEF_DATA;

/*****************************************************************************
 * CORE information used for encoding process.
 *
 * The variables in this structure are very often used in encoding process.
 *****************************************************************************/
typedef struct _EVCE_CORE
{
    /* coefficient buffer of current CU */
    s16            coef[N_C][MAX_CU_DIM];
    /* CU data for RDO */
    EVCE_CU_DATA  cu_data_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    EVCE_CU_DATA  cu_data_temp[MAX_CU_DEPTH][MAX_CU_DEPTH];
    /* temporary coefficient buffer */
    s16            ctmp[N_C][MAX_CU_DIM];
    /* pred buffer of current CU. [1][x][x] is used for bi-pred */
    pel            pred[2][N_C][MAX_CU_DIM];
    /* neighbor pixel buffer for intra prediction */
    pel            nb[N_C][N_REF][MAX_CU_SIZE * 3];
    /* current encoding LCU number */
    int            lcu_num;
    /* QP for luma of current encoding CU */
    u8             qp_y;
    /* QP for chroma of current encoding CU */
    u8             qp_u;
    u8             qp_v;
    /* X address of current LCU */
    u16            x_lcu;
    /* Y address of current LCU */
    u16            y_lcu;
    /* X address of current CU in SCU unit */
    u16            x_scu;
    /* Y address of current CU in SCU unit */
    u16            y_scu;
    /* left pel position of current LCU */
    u16            x_pel;
    /* top pel position of current LCU */
    u16            y_pel;
    /* CU position in current frame in SCU unit */
    u32            scup;
    /* CU position in current LCU in SCU unit */
    u32            cup;
    /* CU depth */
    int            cud;
    /* neighbor CUs availability of current CU */
    u16            avail_cu;
    /* Left, right availability of current CU */
    u16            avail_lr; 
    /* CU mode */
    int            cu_mode;
    /* intra prediction mode */
    u8             mpm[2]; /* mpm table pointer*/
    u8            *mpm_b_list;
    u8             mpm_ext[8];
    u8             pims[IPD_CNT]; /* probable intra mode set*/
    s8             ipm[2];
    /* skip flag for MODE_INTER */
    u8             skip_flag;
#if ADMVP
    /* history-based prediction buffer */
    EVC_HISTORY_BUFFER  m_pTempMotLUTs[MAX_CU_DEPTH][MAX_CU_DEPTH];
    EVC_HISTORY_BUFFER  m_pBestMotLUTs[MAX_CU_DEPTH][MAX_CU_DEPTH];
    EVC_HISTORY_BUFFER  history_buffer;
#endif
    /* mmvd_flag for MODE_INTER */
    u8             mmvd_flag;
#if AFFINE
    /* affine flag for MODE_INTER */
    u8             affine_flag;
#endif
    /* width of current CU */
    u16            cuw;
    /* height of current CU */
    u16            cuh;
    /* log2 of cuw */
    u8             log2_cuw;
    /* log2 of cuh */
    u8             log2_cuh;
    /* number of non-zero coefficient */
    int            nnz[N_C]; 
    int            nnz_sub[N_C][MAX_SUB_TB_NUM];
    /* platform specific data, if needed */
    void          *pf;
    /* bitstream structure for RDO */
    EVC_BSW       bs_temp;
    /* SBAC structure for full RDO */
    EVCE_SBAC     s_curr_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    EVCE_SBAC     s_next_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    EVCE_SBAC     s_temp_best;
#if MERGE
    EVCE_SBAC     s_temp_best_merge;
#endif
    EVCE_SBAC     s_temp_run;
    EVCE_SBAC     s_temp_prev_comp_best;
    EVCE_SBAC     s_temp_prev_comp_run;
    EVCE_SBAC     s_curr_before_split[MAX_CU_DEPTH][MAX_CU_DEPTH];
    EVCE_BEF_DATA bef_data[MAX_CU_DEPTH][MAX_CU_DEPTH][MAX_CU_CNT_IN_LCU][NUM_NEIB];

    double         cost_best;
    u32            inter_satd;
    s32            dist_cu;
    s32            dist_cu_best; //dist of the best intra mode (note: only updated in intra coding now)

#if EIF
    /* temporal pixel buffer for inter prediction */
#if !HW_EIF
#if EIF_SIMD
    ALIGNED_(EIF_NUM_BYTES_IN_SSE_REG)
#endif
#if EIF_3TAP
    pel            eif_tmp_buffer[EIF_NUM_LINES_IN_PREP_DATA * (EIF_PREP_DATA_STRIDE + 8) + EIF_NUM_LINES_IN_UPSCALED_DATA * EIF_UPSCALED_DATA_STRIDE];
#else
    pel            eif_tmp_buffer[EIF_NUM_LINES_IN_PREP_DATA * EIF_PREP_DATA_STRIDE + EIF_NUM_LINES_IN_UPSCALED_DATA * EIF_UPSCALED_DATA_STRIDE];
#endif
#else
    pel            eif_tmp_buffer[ (MAX_CU_SIZE + 2) * (MAX_CU_SIZE + 2) ];
#endif
#endif
#if MERGE
    u8             au8_eval_mvp_idx[MAX_NUM_MVP];
#endif
#if DMVR_FLAG
    u8            dmvr_flag;
#endif
#if TRACE_ENC_CU_DATA
    u64  trace_idx;
#endif
} EVCE_CORE;

/******************************************************************************
 * CONTEXT used for encoding process.
 *
 * All have to be stored are in this structure.
 *****************************************************************************/
typedef struct _EVCE_CTX EVCE_CTX;
struct _EVCE_CTX
{
    /* address of current input picture, ref_picture  buffer structure */
    EVCE_PICO            *pico_buf[EVCE_MAX_INBUF_CNT];
    /* address of current input picture buffer structure */
    EVCE_PICO            *pico;
    /* index of current input picture buffer in pico_buf[] */
    u8                     pico_idx;
    int                    pico_max_cnt;
    /* magic code */
    u32                    magic;
    /* EVCE identifier */
    EVCE                  id;
    /* address of core structure */
    EVCE_CORE            *core;
    /* current input (original) image */
    EVC_PIC               pic_o;
    /* address indicating current encoding, list0, list1 and original pictures */
    EVC_PIC              *pic[PIC_D + 1]; /* the last one is for original */
    /* picture address for mode decision */
    EVC_PIC              *pic_m;
    /* reference picture (0: foward, 1: backward) */
    EVC_REFP              refp[MAX_NUM_REF_PICS][REFP_NUM];
    /* encoding parameter */
    EVCE_PARAM            param;
    /* bitstream structure */
    EVC_BSW               bs;
    /* bitstream structure for RDO */
    EVC_BSW               bs_temp;
    /* sequnce parameter set */
    EVC_SPS               sps;
    /* picture parameter set */
    EVC_PPS               pps;
#if ALF_PARAMETER_APS
    /* adaptation parameter set */
    EVC_APS               aps;
    u8                    aps_counter;
    u8                    aps_temp;
#endif
    /* tile_group header */
    EVC_TGH                tgh;
    /* reference picture manager */
    EVC_PM                rpm;
    /* create descriptor */
    EVCE_CDSC             cdsc;
    /* quantization value of current encoding tile_group */
    u8                     qp;
    /* encoding picture width */
    u16                    w;
    /* encoding picture height */
    u16                    h;
    /* encoding picture width * height */
    u16                    f;
#if HLS_M47668
    /* the picture order count of the previous Tid0 picture */
    u32                     prev_pic_order_cnt_val;
    /* the decoding order count of the previous picture */
    u32                     prev_doc_offset;
#endif
    /* current encoding picture count(This is not PicNum or FrameNum.
    Just count of encoded picture correctly) */
    u32                    pic_cnt;
    /* current picture input count (only update when CTX0) */
    u32                    pic_icnt;
    /* total input picture count (only used for bumping process) */
    u32                    pic_ticnt;
    /* remaining pictures is encoded to p or b tile_group (only used for bumping process) */
    u8                     force_tile_group;
    /* ignored pictures for force tile_group count (unavailable pictures cnt in gop,\
    only used for bumping process) */
    u8                     force_ignored_cnt;
    /* initial frame return number(delayed input count) due to B picture or Forecast */
    u32                    frm_rnum;
    /* current encoding tile_group number in one picture */
    int                    tile_group_num;
    /* first mb number of current encoding tile_group in one picture */
    int                    sl_first_mb;
    /* current tile_group type */
    u8                     tile_group_type;
    /* tile_group depth for current picture */
    u8                     tile_group_depth;
#if !HLS_M47668
    /* whether current picture is referred or not */
    u8                     ref_depth;
#endif
    /* flag whether current picture is refecened picture or not */
    u8                     tile_group_ref_flag;
#if HLS_M47668
    /* distance between ref pics in addition to closest ref ref pic in LD*/
    int                    ref_pic_gap_length;
#endif
    /* current picture POC number */
    int                    poc;
    /* maximum CU depth */
    u8                     max_cud;
    EVCE_SBAC             sbac_enc;
    /* address of inbufs */
    EVC_IMGB             *inbuf[EVCE_MAX_INBUF_CNT];
    /* last coded intra picture's presentation temporal reference */
    int                    last_intra_ptr;
    /* maximum CU width and height */
    u16                    max_cuwh;
    /* log2 of maximum CU width and height */
    u8                     log2_max_cuwh;
    /* total count of remained LCU for encoding one picture. if a picture is
    encoded properly, this value should reach to zero */
    int                    lcu_cnt;
    /* picture width in LCU unit */
    u16                    w_lcu;
    /* picture height in LCU unit */
    u16                    h_lcu;
    /* picture size in LCU unit (= w_lcu * h_lcu) */
    u32                    f_lcu;
    /* picture width in SCU unit */
    u16                    w_scu;
    /* picture height in SCU unit */
    u16                    h_scu;
    /* picture size in SCU unit (= w_scu * h_scu) */
    u32                    f_scu;
    /* log2 of SCU count in a LCU row */
    u8                     log2_culine;
    /* log2 of SCU count in a LCU (== log2_culine * 2) */
    u8                     log2_cudim;
    /* mode decision structure */
    EVCE_MODE             mode;
    /* intra prediction analysis */
    EVCE_PINTRA           pintra;
    /* inter prediction analysis */
    EVCE_PINTER           pinter;
    /* picture buffer allocator */
    PICBUF_ALLOCATOR       pa;
    /* current picture's decoding temporal reference */
    u32                    dtr;
    /* current picture's presentation temporal reference */
    u32                    ptr;
    /* picture coding structure unit for reorder*/
    EVC_REORDER_ARG       reorder[32];
    /*current picutre's layer id for hierachical structure */
    u8                     layer_id;
    /* MAPS *******************************************************************/
    /* CU map (width in SCU x height in SCU) of raster scan order in a frame */
    u32                   *map_scu;
    /* cu data for current LCU */
    EVCE_CU_DATA         *map_cu_data;
    /* map for encoded motion vectors in SCU */
    s16                  (*map_mv)[REFP_NUM][MV_D];
#if DMVR_LAG
    /* map for encoded motion vectors in SCU */
    s16                  (*map_unrefined_mv)[REFP_NUM][MV_D];
#endif
    /* map for reference indices */
    s8                   (*map_refi)[REFP_NUM];
    /* map for intra pred mode */
    s8                    *map_ipm;
#if AQS 
    /* adaptive quantization step size derivation*/
    EVC_AQS               aqs;
#endif

    s16                  (*map_block_size)[2];
    s8                    *map_depth;

#if RDO_DBK
    EVC_PIC              *pic_dbk;          //one picture that arranges cu pixels and neighboring pixels for deblocking (just to match the interface of deblocking functions)
    s64                    delta_dist[N_C];  //delta distortion from filtering (negative values mean distortion reduced)
    s64                    dist_nofilt[N_C]; //distortion of not filtered samples
    s64                    dist_filter[N_C]; //distortion of filtered samples
#endif
#if AFFINE
    /* affine map (width in SCU x height in SCU) of raster scan order in a frame */
    u32                   *map_affine;
#endif
    u32                   *map_cu_mode;
    u8                     ctx_flags[NUM_CNID];
    double                 lambda[3];
    double                 sqrt_lambda[3];
    double                 dist_chroma_weight[2];

    int (*fn_ready)(EVCE_CTX * ctx);
    void (*fn_flush)(EVCE_CTX * ctx);
    int (*fn_enc)(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
    int (*fn_enc_header)(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);

    int (*fn_enc_pic_prepare)(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
    int (*fn_enc_pic)(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
    int (*fn_enc_pic_finish)(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);

    int (*fn_push)(EVCE_CTX * ctx, EVC_IMGB * img);
    int (*fn_deblock)(EVCE_CTX * ctx, EVC_PIC * pic);

#if ALF
    void* enc_alf;
#if ALF_PARAMETER_APS
    int(*fn_alf)(EVCE_CTX * ctx, EVC_PIC * pic, EVC_TGH* tgh, EVC_APS* aps);
#else
    int (*fn_alf)(EVCE_CTX * ctx, EVC_PIC * pic, EVC_TGH* tgh);
#endif
#endif

    void (*fn_picbuf_expand)(EVCE_CTX * ctx, EVC_PIC * pic);

    int  (*fn_get_inbuf)(EVCE_CTX * ctx, EVC_IMGB ** img);

    /* mode decision functions */
    int (*fn_mode_init_frame)(EVCE_CTX * ctx);
    int (*fn_mode_init_lcu)(EVCE_CTX * ctx, EVCE_CORE * core);

    int (*fn_mode_analyze_frame)(EVCE_CTX * ctx);
    int (*fn_mode_analyze_lcu)(EVCE_CTX * ctx, EVCE_CORE * core);

    int (*fn_mode_set_complexity)(EVCE_CTX * ctx, int complexity);

    /* intra prediction functions */
    int (*fn_pintra_init_frame)(EVCE_CTX * ctx);
    int (*fn_pintra_init_lcu)(EVCE_CTX * ctx, EVCE_CORE * core);

    double(*fn_pintra_analyze_cu)(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, \
                                  int log2_cuw, int log2_cuh, EVCE_MODE *mi, \
                                  s16 coef[N_C][MAX_CU_DIM], pel *rec[N_C], int s_rec[N_C]

                                  );
    int (*fn_pintra_set_complexity)(EVCE_CTX * ctx, int complexity);

    /* inter prediction functions */
    int (*fn_pinter_init_frame)(EVCE_CTX * ctx);
    int (*fn_pinter_init_lcu)(EVCE_CTX * ctx, EVCE_CORE * core);

    double (*fn_pinter_analyze_cu)(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, \
                                   int log2_cuw, int log2_cuh, EVCE_MODE *mi, \
                                   s16 coef[N_C][MAX_CU_DIM], pel *rec[N_C], int s_rec[N_C]
                                   );

    int (*fn_pinter_set_complexity)(EVCE_CTX * ctx, int complexity);

    /* platform specific data, if needed */
    void                  * pf;
};

int evce_platform_init(EVCE_CTX * ctx);
void evce_platform_deinit(EVCE_CTX * ctx);
int evce_enc_pic_prepare(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
int evce_enc_pic_finish(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
int evce_enc_pic(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
int evce_deblock_h263(EVCE_CTX * ctx, EVC_PIC * pic);
int evce_enc(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat);
int evce_push_frm(EVCE_CTX * ctx, EVC_IMGB * img);
int evce_ready(EVCE_CTX * ctx);
void evce_flush(EVCE_CTX * ctx);
int evce_picbuf_get_inbuf(EVCE_CTX * ctx, EVC_IMGB ** img);

#include "evce_util.h"
#include "evce_eco.h"
#include "evce_mode.h"
#include "evce_tq.h"
#include "evce_pintra.h"
#include "evce_pinter.h"
#include "evce_tbl.h"

#endif /* _EVCE_DEF_H_ */
