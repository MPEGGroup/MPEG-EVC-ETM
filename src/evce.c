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

#include <math.h>
#include "evce_def.h"
#include "evce_eco.h"
#include "evc_df.h"
#include "evce_mode.h"
#include "evc_util.h"
#if ALF
#include "enc_alf_wrapper.h"
#endif
/* Convert EVCE into EVCE_CTX */
#define EVCE_ID_TO_CTX_R(id, ctx) \
    evc_assert_r((id)); \
    (ctx) = (EVCE_CTX *)id; \
    evc_assert_r((ctx)->magic == EVCE_MAGIC_CODE);

/* Convert EVCE into EVCE_CTX with return value if assert on */
#define EVCE_ID_TO_CTX_RV(id, ctx, ret) \
    evc_assert_rv((id), (ret)); \
    (ctx) = (EVCE_CTX *)id; \
    evc_assert_rv((ctx)->magic == EVCE_MAGIC_CODE, (ret));

#define RMPNI_GOP_NUM                   16
static const EVC_REORDER_ARG reorder_arg[RMPNI_GOP_NUM] =
{
/* poc, layer_id, #of refpic, {refpics..} */
    {
        16, 1, 3, { -16, -24, -32,},
               3, { -16, -24, -32,}
    },
    {
        8, 2, 3, { -8, -16, 8,},
              3, {8, -8, -16, },
    },
    {
        4, 3, 4, { -4, -12, 4, 12,},
              4, { 4, 12, -4, -12,}
    },
    {
        2, 4, 5, { -2, -10, 2, 6, 14, },
              5, { 2, 6, 14, -2, -10, }
    },
    {
        1, 5, 5, { -1, 1, 3, 7, 15, },
              5, { 1, 3, 7, 15, -1 },
    },
    {
        3, 5, 5, { -1, -3, 1, 5, 13, },
              5, {  1, 5, 13, -1, -3,}
    },
    {
        6, 4, 4, { -2, -6, 2, 10,   },
              4, { 2, 10, -2, -6,   },
    },
    {
        5, 5, 5, { -1, -5, 1, 3, 11, },
              5, { 1, 3, 11, -1, -5,  },
    },
    {
        7, 5, 5, { -1, -3, -7, 1, 9, },
              5, { 1, 9, -1, -3, -7, },
    },
    {
        12, 3, 3, { -4, -12, 4, },
               3, { 4, -4, -12, },
    },
    {
        10, 4, 4, { -2, -10, 2, 6, },
                  4, { 2, 6, -2, -10, },
    },
    {
        9, 5, 5, { -1, -9, 1, 3, 7, },
                5, { 1, 3, 7, -1, -9, },
    },
    {
        11, 5, 5, { -1, -3, -11, 1, 5, },
               5, { 1, 5, -1, -3, -11, },
    },
    {
        14, 4, 4, { -2, -6, -14, 2, },
               4, { 2, -2, -6, -14, },
    },
    {
        13, 5, 5, { -1, -5, -13, 1, 3, },
               5, {  1, 3, -1, -5, -13, },
    },
    {
        15, 5, 5, { -1, -3, -7, -15, 1, },
               5, { 1, -1, -3, -7, -15, },
    }
};

#define RMPNI_GOP_NUM_LDB                  4 
static const EVC_REORDER_ARG reorder_arg_ldb[RMPNI_GOP_NUM_LDB] =
{
    /* poc, layer_id, #of refpic, {refpics..} */
    {
        1, 3, 4,{-1,-5,-9,-13},
        4,{-1,-5,-9,-13}
    },
    {
        2, 2, 4,{-1,-2,-6,-10},
        4,{-1,-2,-6,-10}
    },
    {
        3, 3, 4,{-1,-3,-7,-11},
        4,{-1,-3,-7,-11}
    },
    {
        4, 1, 4,{-1,-4,-8,-12},
        4,{-1,-4,-8,-12}
    }
};

static const s8 tbl_poc_gop_offset[5][15] =
{
    { -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { -4,   -6,   -7,   -5,   -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { -8,   -12, -14,  -15,  -13,  -10,  -11,   -9,   -4,   -6,   -7,   -5,   -2,   -3,   -1}   /* gop_size = 16 */
};

static const s8 tbl_ref_depth[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 4 */
    { FRM_DEPTH_1, FRM_DEPTH_2, FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 8 */
    { FRM_DEPTH_1, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    /* gop_size = 16 */
    { FRM_DEPTH_1, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, \
     FRM_DEPTH_4,  FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4}
};

static const s8 tbl_tile_group_ref[5][15] =
{
    /* gop_size = 2 */
    { 1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 4 */
    { 1, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 8 */
    { 1, 1, 0, 0, 1, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 16 */
    {1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0}
};

static const s8 tbl_tile_group_depth_P[GOP_P] = { FRM_DEPTH_3,  FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_1};

static const s8 tbl_tile_group_depth[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 4 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 8 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 16 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, \
      FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5}
};

static EVCE_CTX * ctx_alloc(void)
{
    EVCE_CTX * ctx;

    ctx = (EVCE_CTX*)evc_malloc_fast(sizeof(EVCE_CTX));
    evc_assert_rv(ctx, NULL);
    evc_mset_x64a(ctx, 0, sizeof(EVCE_CTX));
    return ctx;
}

static void ctx_free(EVCE_CTX * ctx)
{
    evc_mfree_fast(ctx);
}

static EVCE_CORE * core_alloc(void)
{
    EVCE_CORE * core;
    int i, j;

    core = (EVCE_CORE *)evc_malloc_fast(sizeof(EVCE_CORE));

    evc_assert_rv(core, NULL);
    evc_mset_x64a(core, 0, sizeof(EVCE_CORE));

    for(i = 0; i < MAX_CU_DEPTH; i++)
    {
        for(j = 0; j < MAX_CU_DEPTH; j++)
        {
            evce_create_cu_data(&core->cu_data_best[i][j], i, j);
            evce_create_cu_data(&core->cu_data_temp[i][j], i, j);
        }
    }

    return core;
}

static void core_free(EVCE_CORE * core)
{
    int i, j;

    for(i = 0; i < MAX_CU_DEPTH; i++)
    {
        for(j = 0; j < MAX_CU_DEPTH; j++)
        {
            evce_delete_cu_data(&core->cu_data_best[i][j], i, j);
            evce_delete_cu_data(&core->cu_data_temp[i][j], i, j);
        }
    }
    evc_mfree_fast(core);
}

static int set_init_param(EVCE_CDSC * cdsc, EVCE_PARAM * param)
{
    /* check input parameters */
    evc_assert_rv(cdsc->w > 0 && cdsc->h > 0, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv((cdsc->w & (MIN_CU_SIZE-1)) == 0,EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv((cdsc->h & (MIN_CU_SIZE-1)) == 0,EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(cdsc->qp >= MIN_QUANT && cdsc->qp <= MAX_QUANT, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(cdsc->iperiod >= 0 ,EVC_ERR_INVALID_ARGUMENT);

    if(cdsc->disable_hgop == 0)
    {
        evc_assert_rv(cdsc->max_b_frames == 0 || cdsc->max_b_frames == 1 || \
                       cdsc->max_b_frames == 3 || cdsc->max_b_frames == 7 || \
                       cdsc->max_b_frames == 15, EVC_ERR_INVALID_ARGUMENT);

        if(cdsc->max_b_frames != 0)
        {
            if(cdsc->iperiod % (cdsc->max_b_frames + 1) != 0)
            {
                evc_assert_rv(0, EVC_ERR_INVALID_ARGUMENT);
            }
        }
    }

    /* set default encoding parameter */
    param->w              = cdsc->w;
    param->h              = cdsc->h;
    param->bit_depth      = cdsc->out_bit_depth;
    param->qp             = cdsc->qp;
    param->fps            = cdsc->fps;
    param->i_period       = cdsc->iperiod;
    param->f_ifrm         = 0;
    param->use_deblock    = 1;
    param->qp_max         = MAX_QUANT;
    param->qp_min         = MIN_QUANT;
    param->use_pic_sign   = 0;
    param->max_b_frames   = cdsc->max_b_frames;
    param->gop_size       = param->max_b_frames +1;
    param->use_closed_gop = (cdsc->closed_gop)? 1: 0;
    param->use_hgop       = (cdsc->disable_hgop)? 0: 1;
#if USE_TILE_GROUP_DQP
    param->qp_incread_frame = cdsc->add_qp_frame;
#endif

    if(cdsc->tool_iqt == 0)
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_base;
    }
    else
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_main;
    }

    return EVC_OK;
}

static int set_enc_param(EVCE_CTX * ctx, EVCE_PARAM * param)
{
    int ret = EVC_OK;
    ctx->qp = (u8)param->qp;
    return ret;
}

static void set_cnkh(EVCE_CTX * ctx, EVC_CNKH * cnkh, int ver, int ctype)
{
    cnkh->ver = ver;
    cnkh->ctype = ctype;
    cnkh->broken = 0;
}

static void set_sps(EVCE_CTX * ctx, EVC_SPS * sps)
{
    sps->profile_idc = ctx->cdsc.profile;
    sps->level_idc = ctx->cdsc.level;
    sps->pic_width_in_luma_samples = ctx->param.w;
    sps->pic_height_in_luma_samples = ctx->param.h;
    sps->closed_gop = (ctx->param.use_closed_gop) ? 1 : 0;
    if(ctx->param.max_b_frames > 0)
    {
        sps->num_ref_pics_act = MAX_NUM_ACTIVE_REF_FRAME_B;
    }
    else
    {
        sps->num_ref_pics_act = MAX_NUM_ACTIVE_REF_FRAME_LDB;
    }

    if(sps->profile_idc == PROFILE_MAIN)
    {
        sps->sps_btt_flag = ctx->cdsc.btt;
        sps->sps_suco_flag = ctx->cdsc.suco;
    }
    else
    {
        sps->sps_btt_flag = 0;
        sps->sps_suco_flag = 0;
    }

    if(sps->profile_idc == PROFILE_MAIN)
    {
        sps->log2_diff_ctu_max_11_cb_size = ctx->log2_max_cuwh - ctx->cdsc.framework_cu11_max;
        sps->log2_diff_max_11_min_11_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_cu11_min;
        sps->log2_diff_max_11_max_12_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_cu12_max;
        sps->log2_diff_min_11_min_12_cb_size_minus1 = ctx->cdsc.framework_cu12_min - ctx->cdsc.framework_cu11_min - 1;
        sps->log2_diff_max_12_max_14_cb_size = ctx->cdsc.framework_cu12_max - ctx->cdsc.framework_cu14_max;
        sps->log2_diff_min_12_min_14_cb_size_minus1 = ctx->cdsc.framework_cu14_min - ctx->cdsc.framework_cu12_min - 1;
        sps->log2_diff_max_11_max_tt_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_tris_max;
        sps->log2_diff_min_11_min_tt_cb_size_minus2 = ctx->cdsc.framework_tris_min - ctx->cdsc.framework_cu11_min - 2;
        sps->log2_diff_ctu_size_max_suco_cb_size = ctx->log2_max_cuwh - ctx->cdsc.framework_suco_max;
        sps->log2_diff_max_suco_min_suco_cb_size = ctx->cdsc.framework_suco_max - ctx->cdsc.framework_suco_min;
    }

    sps->tool_amvr = ctx->cdsc.tool_amvr;
    sps->tool_mmvd = ctx->cdsc.tool_mmvd;
#if AFFINE
    sps->tool_affine = ctx->cdsc.tool_affine;
#endif
#if DMVR
    sps->tool_dmvr = ctx->cdsc.tool_dmvr;
#endif
#if ALF
    sps->tool_alf = ctx->cdsc.tool_alf;
#endif
#if HTDF
    sps->tool_htdf = ctx->cdsc.tool_htdf;
#endif
#if ADMVP
    sps->tool_admvp = ctx->cdsc.tool_admvp;
#endif
    sps->tool_amis = ctx->cdsc.tool_amis;
    sps->tool_eipd = ctx->cdsc.tool_eipd;
    sps->tool_iqt = ctx->cdsc.tool_iqt;
    sps->tool_cm_init = ctx->cdsc.tool_cm_init;

    if(sps->profile_idc == PROFILE_MAIN)
    {
        sps->picture_num_present_flag = 0;
        sps->log2_ctu_size_minus2 = ctx->log2_max_cuwh - 2;
    }
    else
    {
        sps->picture_num_present_flag = 1;
    }

    sps->long_term_ref_pics_flag = 0;

    if (sps->picture_num_present_flag)
    {
        sps->rpls_l0_num = 0;
        sps->rpls_l1_num = 0;
        sps->rpl_candidates_present_flag = 0;
        sps->rpl1_same_as_rpl0_flag = 0;
    }
    else
    {
        sps->rpls_l0_num = ctx->cdsc.rpls_l0_cfg_num;
        sps->rpls_l1_num = ctx->cdsc.rpls_l1_cfg_num;
        sps->rpl_candidates_present_flag = 1;
        sps->rpl1_same_as_rpl0_flag = 0;
    }

    memcpy(sps->rpls_l0, ctx->cdsc.rpls_l0, ctx->cdsc.rpls_l0_cfg_num * sizeof(sps->rpls_l0[0]));
    memcpy(sps->rpls_l1, ctx->cdsc.rpls_l1, ctx->cdsc.rpls_l1_cfg_num * sizeof(sps->rpls_l1[0]));
}

static void set_pps(EVCE_CTX * ctx, EVC_PPS * pps)
{
    pps->single_tile_in_pic_flag = 1;
}

typedef struct _QP_ADAPT_PARAM
{
    int qp_offset_layer;
    double qp_offset_model_offset;
    double qp_offset_model_scale;
} QP_ADAPT_PARAM;

QP_ADAPT_PARAM qp_adapt_param_ra[8] = 
{
    {-3,  0.0000, 0.0000},
    { 1,  0.0000, 0.0000},
    { 1, -4.8848, 0.2061},
    { 4, -5.7476, 0.2286},
    { 5, -5.9000, 0.2333},
    { 6, -7.1444, 0.3000},
    { 7, -7.1444, 0.3000},
    { 8, -7.1444, 0.3000},
};

QP_ADAPT_PARAM qp_adapt_param_ld[8] =
{
    {-1,  0.0000, 0.0000},
    { 1,  0.0000, 0.0000},
    { 4, -6.5000, 0.2590},
    { 5, -6.5000, 0.2590},
    { 6, -6.5000, 0.2590},
    { 7, -6.5000, 0.2590},
    { 8, -6.5000, 0.2590},
    { 9, -6.5000, 0.2590},
};

  //Implementation for selecting and assigning RPL0 & RPL1 candidates in the SPS to TGH
static void select_assign_rpl_for_tgh(EVCE_CTX *ctx, EVC_TGH *tgh)
{
    //TBD: when NALU types are implemented; if the current picture is an IDR, simply return without doing the rest of the codes for this function

    /* introduce this variable for LD reason. The predefined RPL in the cfg file is made assuming GOP size is 4 for LD configuration*/
    int gopSize = (ctx->param.gop_size == 1) ? 4 : ctx->param.gop_size;

    //Assume it the pic is in the normal GOP first. Normal GOP here means it is not the first (few) GOP in the beginning of the bitstream
    tgh->rpl_l0_idx = tgh->rpl_l1_idx = -1;
    tgh->ref_pic_list_sps_flag[0] = tgh->ref_pic_list_sps_flag[1] = 0;

    int availableRPLs = (ctx->cdsc.rpls_l0_cfg_num < gopSize) ? ctx->cdsc.rpls_l0_cfg_num : gopSize;
    for (int i = 0; i < availableRPLs; i++)
    {
        int pocIdx = (tgh->poc % gopSize == 0) ? gopSize : tgh->poc % gopSize;
        if (pocIdx == ctx->cdsc.rpls_l0[i].poc)
        {
            tgh->rpl_l0_idx = i;
            tgh->rpl_l1_idx = tgh->rpl_l0_idx;
            break;
        }
    }

    //For special case when the pic is in the first (few) GOP in the beginning of the bitstream.
    if (ctx->param.i_period == 0)                          //For low delay configuration
    {
        if (tgh->poc <= (ctx->cdsc.rpls_l0_cfg_num - gopSize))
        {
            tgh->rpl_l0_idx = tgh->poc + gopSize - 1;
            tgh->rpl_l1_idx = tgh->rpl_l0_idx;
        }
    }
    else                                                 //For random access configuration
    {
        //for (int i = ctx->param.gop_size; i < ctx->cdsc.rpls_l0_cfg_num; i++)
        for (int i = gopSize; i < ctx->cdsc.rpls_l0_cfg_num; i++)
        {
            int pocIdx = (tgh->poc % ctx->param.i_period == 0) ? ctx->param.i_period : tgh->poc % ctx->param.i_period;
            if (pocIdx == ctx->cdsc.rpls_l0[i].poc)
            {
                tgh->rpl_l0_idx = i;
                tgh->rpl_l1_idx = i;
                break;
            }
        }
    }
    //Copy RPL0 from the candidate in SPS to this SH
    tgh->rpl_l0.poc = tgh->poc;
    tgh->rpl_l0.tid = ctx->cdsc.rpls_l0[tgh->rpl_l0_idx].tid;
    tgh->rpl_l0.ref_pic_num = ctx->cdsc.rpls_l0[tgh->rpl_l0_idx].ref_pic_num;
    tgh->rpl_l0.ref_pic_active_num = ctx->cdsc.rpls_l0[tgh->rpl_l0_idx].ref_pic_active_num;
    for (int i = 0; i < tgh->rpl_l0.ref_pic_num; i++)
        tgh->rpl_l0.ref_pics[i] = ctx->cdsc.rpls_l0[tgh->rpl_l0_idx].ref_pics[i];

    //Copy RPL0 from the candidate in SPS to this SH
    tgh->rpl_l1.poc = tgh->poc;
    tgh->rpl_l1.tid = ctx->cdsc.rpls_l1[tgh->rpl_l1_idx].tid;
    tgh->rpl_l1.ref_pic_num = ctx->cdsc.rpls_l1[tgh->rpl_l1_idx].ref_pic_num;
    tgh->rpl_l1.ref_pic_active_num = ctx->cdsc.rpls_l1[tgh->rpl_l1_idx].ref_pic_active_num;
    for (int i = 0; i < tgh->rpl_l1.ref_pic_num; i++)
        tgh->rpl_l1.ref_pics[i] = ctx->cdsc.rpls_l1[tgh->rpl_l1_idx].ref_pics[i];

    if (tgh->rpl_l0_idx != -1)
    {
        tgh->ref_pic_list_sps_flag[0] = 1;
    }

    if (tgh->rpl_l1_idx != -1)
    {
        tgh->ref_pic_list_sps_flag[1] = 1;
    }
}

//Return value 0 means all ref pic listed in the given rpl are available in the DPB
//Return value 1 means there is at least one ref pic listed in the given rpl not available in the DPB
static int check_refpic_available(int currentPOC, EVC_PM *pm, EVC_RPL *rpl)
{
    for (int i = 0; i < rpl->ref_pic_num; i++)
    {
        int isExistInDPB = 0;
        for (int j = 0; !isExistInDPB && j < MAX_PB_SIZE; j++)
        {
            if (pm->pic[j] && pm->pic[j]->is_ref && pm->pic[j]->ptr == (currentPOC - rpl->ref_pics[i]))
                isExistInDPB = 1;
        }
        if (!isExistInDPB) //Found one ref pic missing return 1
            return 1;
    }
    return 0; 
}

//Return value 0 means no explicit RPL is created. The given input parameters rpl0 and rpl1 are not modified
//Return value 1 means the given input parameters rpl0 and rpl1 are modified
static int create_explicit_rpl(EVC_PM *pm, EVC_TGH *tgh)

{
    int currentPOC = tgh->poc;
    EVC_RPL *rpl0 = &tgh->rpl_l0;
    EVC_RPL *rpl1 = &tgh->rpl_l1;
    if (!check_refpic_available(currentPOC, pm, rpl0) && !check_refpic_available(currentPOC, pm, rpl1))
    {
        return 0;
    }
    
    EVC_PIC * pic = NULL;

    int isRPLChanged = 0;
    //Remove ref pic in RPL0 that is not available in the DPB
    for (int ii = 0; ii < rpl0->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl0->ref_pics[ii]))
                isAvailable = 1;
            pic = NULL;
        }
        if (!isAvailable)
        {
            for (int jj = ii; jj < rpl0->ref_pic_num - 1; jj++)
                rpl0->ref_pics[jj] = rpl0->ref_pics[jj + 1];
            ii--;
            rpl0->ref_pic_num--;
            isRPLChanged = 1;
        }
    }
    if (isRPLChanged)
        tgh->rpl_l0_idx = -1;

    //Remove ref pic in RPL1 that is not available in the DPB
    isRPLChanged = 0;
    for (int ii = 0; ii < rpl1->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl1->ref_pics[ii]))
                isAvailable = 1;
            pic = NULL;
        }
        if (!isAvailable)
        {
            for (int jj = ii; jj < rpl1->ref_pic_num - 1; jj++)
                rpl1->ref_pics[jj] = rpl1->ref_pics[jj + 1];
            ii--;
            rpl1->ref_pic_num--;
            isRPLChanged = 1;
        }
    }
    if (isRPLChanged)
        tgh->rpl_l1_idx = -1;

    /*if number of ref pic in RPL0 is less than its number of active ref pic, try to copy from RPL1*/
    if (rpl0->ref_pic_num < rpl0->ref_pic_active_num)
    {
        for (int ii = rpl0->ref_pic_num; ii < rpl0->ref_pic_active_num; ii++)
        {
            //First we need to find ref pic in RPL1 that is not already in RPL0
            int isAlreadyIncluded = 1;
            int idx = -1;
            int status = 0;
            do {
                status = 0;
                idx++;
                for (int mm = 0; mm < rpl0->ref_pic_num && idx < rpl1->ref_pic_num; mm++)
                {
                    if (rpl1->ref_pics[idx] == rpl0->ref_pics[mm])
                        status = 1;
                }
                if (!status) isAlreadyIncluded = 0;
            } while (isAlreadyIncluded && idx < rpl1->ref_pic_num);

            if (idx < rpl1->ref_pic_num)
            {
                rpl0->ref_pics[ii] = rpl1->ref_pics[idx];
                rpl0->ref_pic_num++;
            }
        }
        if (rpl0->ref_pic_num < rpl0->ref_pic_active_num) rpl0->ref_pic_active_num = rpl0->ref_pic_num;
    }

    /*same logic as above, just apply to RPL1*/
    if (rpl1->ref_pic_num < rpl1->ref_pic_active_num)
    {
        for (int ii = rpl1->ref_pic_num; ii < rpl1->ref_pic_active_num; ii++)
        {
            int isAlreadyIncluded = 1;
            int idx = -1;
            int status = 0;
            do {
                status = 0;
                idx++;
                for (int mm = 0; mm < rpl1->ref_pic_num && idx < rpl0->ref_pic_num; mm++)
                {
                    if (rpl0->ref_pics[idx] == rpl1->ref_pics[mm])
                        status = 1;
                }
                if (!status) isAlreadyIncluded = 0;
            } while (isAlreadyIncluded && idx < rpl0->ref_pic_num);

            if (idx < rpl0->ref_pic_num)
            {
                rpl1->ref_pics[ii] = rpl0->ref_pics[idx];
                rpl1->ref_pic_num++;
            }
        }
        if (rpl1->ref_pic_num < rpl1->ref_pic_active_num) rpl1->ref_pic_active_num = rpl1->ref_pic_num;
    }
    return 1;
}

static void set_tgh(EVCE_CTX *ctx, EVC_TGH *tgh)
{
    double qp;
    int qp_l_i;
    int qp_c_i;
    QP_ADAPT_PARAM *qp_adapt_param = ctx->param.max_b_frames == 0 ? qp_adapt_param_ld : qp_adapt_param_ra;

    tgh->dtr = ctx->dtr & DTR_BIT_MSK;
    tgh->tile_group_type = ctx->tile_group_type;
    tgh->keyframe = (tgh->tile_group_type == TILE_GROUP_I) ? 1 : 0;
    tgh->deblocking_filter_on = (ctx->param.use_deblock) ? 1 : 0;
    tgh->udata_exist = (ctx->param.use_pic_sign) ? 1 : 0;
    tgh->dptr = ctx->ptr - ctx->dtr;
    tgh->layer_id = ctx->layer_id;
    tgh->single_tile_in_tile_group_flag = 1;
    
    if (!ctx->sps.picture_num_present_flag)
    {
        tgh->poc = ctx->ptr;
        select_assign_rpl_for_tgh(ctx, tgh);
        tgh->num_ref_idx_active_override_flag = 1;
    }

    /* set lambda */
#if USE_TILE_GROUP_DQP
    qp = EVC_CLIP3(0, MAX_QUANT, (ctx->param.qp_incread_frame != 0 && (int)(ctx->ptr) >= ctx->param.qp_incread_frame) ? ctx->qp + 1.0 : ctx->qp);
#else
    qp = ctx->qp;
#endif

    if(ctx->param.use_hgop)
    {
        double dqp_offset;
        int qp_offset;
        qp += qp_adapt_param[ctx->layer_id].qp_offset_layer;
        dqp_offset = qp * qp_adapt_param[ctx->layer_id].qp_offset_model_scale + qp_adapt_param[ctx->layer_id].qp_offset_model_offset + 0.5;
        qp_offset = (int)floor(EVC_CLIP3(0.0, 3.0, dqp_offset));
        qp += qp_offset;
    }

    tgh->qp = (u8)EVC_CLIP3(-(6 * (BIT_DEPTH - 8)), MAX_QUANT, qp);
    tgh->qp_u = (u8)(tgh->qp + ctx->cdsc.cb_qp_offset); 
    tgh->qp_v = (u8)(tgh->qp + ctx->cdsc.cr_qp_offset);

    qp_l_i = tgh->qp;
    ctx->lambda[0] = 0.57 * pow(2.0, (qp_l_i - 12.0) / 3.0);
    qp_c_i = evc_tbl_qp_chroma_ajudst[tgh->qp_u];
    ctx->dist_chroma_weight[0] = pow(2.0, (qp_l_i - qp_c_i) / 3.0);
    qp_c_i = evc_tbl_qp_chroma_ajudst[tgh->qp_v];
    ctx->dist_chroma_weight[1] = pow(2.0, (qp_l_i - qp_c_i) / 3.0);
    ctx->lambda[1] = ctx->lambda[0] / ctx->dist_chroma_weight[0];
    ctx->lambda[2] = ctx->lambda[0] / ctx->dist_chroma_weight[1];
    ctx->sqrt_lambda[0] = sqrt(ctx->lambda[0]);
    ctx->sqrt_lambda[1] = sqrt(ctx->lambda[1]);
    ctx->sqrt_lambda[2] = sqrt(ctx->lambda[2]);

    if (ctx->sps.picture_num_present_flag)
    {
        /* set MMCO command */
        if (ctx->tile_group_ref_flag == 0)
        {
            tgh->mmco_on = 1;
            tgh->mmco.cnt = 1;
            tgh->mmco.type[0] = MMCO_UNUSED;
            tgh->mmco.data[0] = 0; /* current picture */
        }
        else
        {
            tgh->mmco_on = 0;
            tgh->mmco.cnt = 0;
        }
    }
}
static int evce_eco_tree(EVCE_CTX * ctx, EVCE_CORE * core, int x0, int y0, int cup, int cuw, int cuh, int cud, int next_split, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth)
{
    int ret;
    EVC_BSW * bs;
    s8  split_mode;
    s8  suco_flag = 0;
    int bound;
    int split_mode_child[4] = {NO_SPLIT, NO_SPLIT, NO_SPLIT, NO_SPLIT};
    int split_allow[6];

    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[core->lcu_num].split_mode);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[core->lcu_num].suco_flag);

    same_layer_split[node_idx] = split_mode;

    bs = &ctx->bs;

    if(split_mode != NO_SPLIT)
    {
        if(!ctx->sps.sps_btt_flag || ((x0 + cuw <= ctx->w) && (y0 + cuh <= ctx->h)))
        {
            evce_eco_split_mode(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh
                                , parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
        }

        bound = !((x0 + cuw <= ctx->w) && (y0 + cuh <= ctx->h));
        evce_eco_suco_flag(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh, split_mode, bound, ctx->log2_max_cuwh);
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x0, y0, cuw, cuh, cup, cud, ctx->log2_culine, &split_struct);
        evc_split_get_suco_order(suco_flag, split_mode, suco_order);

        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                ret = evce_eco_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], sub_cuw, sub_cuh, split_struct.cud[cur_part_num], 1, split_mode, split_mode_child, part_num, split_allow, INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, bound));
                evc_assert_g(EVC_SUCCEEDED(ret), ERR);
            }
        }
    }
    else
    {
        evc_assert(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);

        if((cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE) && next_split)
        {
            evce_eco_split_mode(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh
                                , parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
        }

        ret = evce_eco_unit(ctx, core, x0, y0, cup, cuw, cuh);
        evc_assert_g(EVC_SUCCEEDED(ret), ERR);
    }

    return EVC_OK;
ERR:
    return ret;
}

int evce_ready(EVCE_CTX * ctx)
{
    EVCE_CORE * core = NULL;
    int          w, h, ret, i;
    s64          size;

    evc_assert(ctx);

    core = core_alloc();
    evc_assert_gv(core != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

    /* set various value */
    ctx->core = core;
    w = ctx->w = ctx->param.w;
    h = ctx->h = ctx->param.h;
    ctx->f = w *h;

    evce_init_bits_est();

    if (ctx->cdsc.btt)
    {
        ctx->max_cuwh = 1 << ctx->cdsc.framework_ctu_size;
        if (w < ctx->max_cuwh * 2 && h < ctx->max_cuwh * 2)
        {
            ctx->max_cuwh = ctx->max_cuwh >> 1;
        }
        else
        {
            ctx->max_cuwh = ctx->max_cuwh;
        }
    }
    else
    {
        ctx->max_cuwh = 64; 
    }

    ctx->log2_max_cuwh = CONV_LOG2(ctx->max_cuwh);
    ctx->max_cud = ctx->log2_max_cuwh - MIN_CU_LOG2;
    ctx->w_lcu = (w + ctx->max_cuwh - 1) >> ctx->log2_max_cuwh;
    ctx->h_lcu = (h + ctx->max_cuwh - 1) >> ctx->log2_max_cuwh;
    ctx->f_lcu = ctx->w_lcu * ctx->h_lcu;
    ctx->w_scu = (w + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->h_scu = (h + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->f_scu = ctx->w_scu * ctx->h_scu;
    ctx->log2_culine = ctx->log2_max_cuwh - MIN_CU_LOG2;
    ctx->log2_cudim = ctx->log2_culine << 1;

    ctx->cdsc.framework_cu11_max = min(ctx->log2_max_cuwh, ctx->cdsc.framework_cu11_max);
    ctx->cdsc.framework_cu12_max = min(ctx->cdsc.framework_cu11_max, ctx->cdsc.framework_cu12_max);
    ctx->cdsc.framework_cu14_max = min(ctx->cdsc.framework_cu12_max, ctx->cdsc.framework_cu14_max);
    ctx->cdsc.framework_suco_max = min(ctx->log2_max_cuwh, ctx->cdsc.framework_suco_max);
#if ALF
    ctx->enc_alf = new_enc_ALF();
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);
    call_create_enc_ALF(p, ctx->w, ctx->h, ctx->max_cuwh, ctx->max_cuwh, 5);
#endif

    /*  allocate CU data map*/
    if(ctx->map_cu_data == NULL)
    {
        size = sizeof(EVCE_CU_DATA) * ctx->f_lcu;
        ctx->map_cu_data = (EVCE_CU_DATA*)evc_malloc_fast(size);
        evc_assert_gv(ctx->map_cu_data, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_cu_data, 0, size);

        for(i = 0; i < (int)ctx->f_lcu; i++)
        {
            evce_create_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
        }
    }

    /* allocate maps */
    if(ctx->map_scu == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_scu = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_scu, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_scu, 0, size);
    }

    if(ctx->map_ipm == NULL)
    {
        size = sizeof(s8) * ctx->f_scu;
        ctx->map_ipm = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ipm, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ipm, -1, size);
    }

    size = sizeof(s16) * ctx->f_scu * 2;
    ctx->map_block_size = evc_malloc_fast(size);
    evc_assert_gv(ctx->map_block_size, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset(ctx->map_block_size, -1, size);

    size = sizeof(s8) * ctx->f_scu;
    ctx->map_depth = evc_malloc_fast(size);
    evc_assert_gv(ctx->map_depth, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset(ctx->map_depth, -1, size);

#if AFFINE
    if (ctx->map_affine == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_affine = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_affine, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_affine, 0, size);
    }
#endif

    if(ctx->map_cu_mode == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_cu_mode = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_cu_mode, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_cu_mode, 0, size);
    }

    /* initialize reference picture manager */
    ctx->pa.fn_alloc = evce_pic_alloc;
    ctx->pa.fn_free  = evce_pic_free;
    ctx->pa.w        = ctx->w;
    ctx->pa.h        = ctx->h;
    ctx->pa.pad_l    = PIC_PAD_SIZE_L;
    ctx->pa.pad_c    = PIC_PAD_SIZE_C;
    ctx->pic_cnt     = 0;
    ctx->pic_icnt    = -1;
    ctx->dtr         = 0;
    ctx->ptr         = 0;

    ret = evc_picman_init(&ctx->rpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, ctx->param.use_closed_gop, &ctx->pa);
    evc_assert_g(EVC_SUCCEEDED(ret), ERR);

    ctx->pico_max_cnt = 1 + (ctx->param.max_b_frames << 1) ;
    ctx->frm_rnum = ctx->param.max_b_frames;
    ctx->qp = ctx->param.qp;

    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        ctx->pico_buf[i] = (EVCE_PICO*)evc_malloc(sizeof(EVCE_PICO));
        evc_assert_gv(ctx->pico_buf[i], ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->pico_buf[i], 0, sizeof(EVCE_PICO));
    }

    return EVC_OK;
ERR:
    for (i = 0; i < (int)ctx->f_lcu; i++)
    {
        evce_delete_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
    }
    evc_mfree_fast(ctx->map_cu_data);
    evc_mfree_fast(ctx->map_ipm);
    evc_mfree_fast(ctx->map_block_size);
    evc_mfree_fast(ctx->map_depth);
#if AQS 
    evc_mfree(ctx->aqs.es_map_buf);
#endif
#if AFFINE
    evc_mfree_fast(ctx->map_affine);
#endif
    evc_mfree_fast(ctx->map_cu_mode);

    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        evc_mfree_fast(ctx->pico_buf[i]);
    }

    if(core)
    {
        core_free(core);
    }
    return ret;
}

void evce_flush(EVCE_CTX * ctx)
{
    int i;
    evc_assert(ctx);

    evc_mfree_fast(ctx->map_scu);
    for(i = 0; i < (int)ctx->f_lcu; i++)
    {
        evce_delete_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
    }
    evc_mfree_fast(ctx->map_cu_data);
    evc_mfree_fast(ctx->map_ipm);
    evc_mfree_fast(ctx->map_block_size);
    evc_mfree_fast(ctx->map_depth);
#if AQS 
    evc_mfree(ctx->aqs.es_map_buf);
#endif
#if AFFINE
    evc_mfree_fast(ctx->map_affine);
#endif
    evc_mfree_fast(ctx->map_cu_mode);
#if RDO_DBK
    evc_picbuf_free(ctx->pic_dbk);
#endif
    evc_picman_deinit(&ctx->rpm);
    core_free(ctx->core);
    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        evc_mfree_fast(ctx->pico_buf[i]);
    }
    for(i = 0; i < EVCE_MAX_INBUF_CNT; i++)
    {
        if(ctx->inbuf[i]) ctx->inbuf[i]->release(ctx->inbuf[i]);
    }
}

static void deblock_tree(EVCE_CTX * ctx, EVC_PIC * pic, int x, int y, int cuw, int cuh, int cud, int cup, int is_hor)
{
    s8  split_mode;
    int lcu_num;
    s8  suco_flag = 0;

    lcu_num = (x >> ctx->log2_max_cuwh) + (y >> ctx->log2_max_cuwh) * ctx->w_lcu;
    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[lcu_num].split_mode);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[lcu_num].suco_flag);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure(split_mode, x, y, cuw, cuh, cup, cud, ctx->log2_culine, &split_struct);
        evc_split_get_suco_order(suco_flag, split_mode, suco_order);
        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                deblock_tree(ctx, pic, x_pos, y_pos, sub_cuw, sub_cuh, split_struct.cud[cur_part_num], split_struct.cup[cur_part_num], is_hor);
            }
        }
    }
    else
    {
        if(is_hor)
        {
            if (cuh > MAX_TR_SIZE)
            {
                evc_deblock_cu_hor(pic, x, y              , cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
                evc_deblock_cu_hor(pic, x, y + MAX_TR_SIZE, cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
            }
            else
            {
                evc_deblock_cu_hor(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp);
            }
        }
        else
        {
            if (cuw > MAX_TR_SIZE)
            {
                evc_deblock_cu_ver(pic, x              , y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                    , ctx->map_cu_mode
#endif
                    , ctx->refp
                );
                evc_deblock_cu_ver(pic, x + MAX_TR_SIZE, y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                    , ctx->map_cu_mode
#endif                            
                    , ctx->refp
                );
            }
            else
            {
                evc_deblock_cu_ver(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                                   , ctx->map_cu_mode
#endif
                                   , ctx->refp
                );
            }
        }
    }
}

int evce_deblock_h263(EVCE_CTX * ctx, EVC_PIC * pic)
{
    int i, j;
    u32 k;

    for(k = 0; k < ctx->f_scu; k++)
    {
        MCU_CLR_COD(ctx->map_scu[k]);
    }

    /* horizontal filtering */
    for(j = 0; j < ctx->h_lcu; j++)
    {
        for(i = 0; i < ctx->w_lcu; i++)
        {
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1);
        }
    }

    for(k = 0; k < ctx->f_scu; k++)
    {
        MCU_CLR_COD(ctx->map_scu[k]);
    }

    /* vertical filtering */
    for(j = 0; j < ctx->h_lcu; j++)
    {
        for(i = 0; i < ctx->w_lcu; i++)
        {
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0);
        }
    }

    return EVC_OK;
}
#if ALF
int evce_alf(EVCE_CTX * ctx, EVC_PIC * pic, EVC_TGH* tgh)
{
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);

    double lambdas[3];
    for(int i = 0; i < 3; i++)
        lambdas[i] = (ctx->lambda[i]) * ALF_LAMBDA_SCALE; //this is for appr match of different lambda sets


    set_resetALFBufferFlag(p, tgh->tile_group_type == TILE_GROUP_I ? 1 : 0);
    call_enc_ALFProcess(p, lambdas, ctx, pic, &(tgh->alf_tgh_param));
    return EVC_OK;
}
#endif

int evce_picbuf_get_inbuf(EVCE_CTX * ctx, EVC_IMGB ** imgb)
{
    int i, opt, align[EVC_IMGB_MAX_PLANE], pad[EVC_IMGB_MAX_PLANE];

    for(i = 0; i < EVCE_MAX_INBUF_CNT; i++)
    {
        if(ctx->inbuf[i] == NULL)
        {
            opt = EVC_IMGB_OPT_NONE;

            /* set align value*/
            align[0] = MIN_CU_SIZE;
            align[1] = MIN_CU_SIZE >> 1;
            align[2] = MIN_CU_SIZE >> 1;

            /* no padding */
            pad[0] = 0;
            pad[1] = 0;
            pad[2] = 0;

            *imgb = evc_imgb_create(ctx->param.w, ctx->param.h, EVC_COLORSPACE_YUV420_10LE, opt, pad, align);
            evc_assert_rv(*imgb != NULL, EVC_ERR_OUT_OF_MEMORY);

            ctx->inbuf[i] = *imgb;

            (*imgb)->addref(*imgb);
            return EVC_OK;
        }
        else if(ctx->inbuf[i]->getref(ctx->inbuf[i]) == 1)
        {
            *imgb = ctx->inbuf[i];

            (*imgb)->addref(*imgb);
            return EVC_OK;
        }
    }

    return EVC_ERR_UNEXPECTED;
}

int evce_enc_header(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;
    EVC_CNKH  cnkh;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* encode sequence parameter set */
    /* skip first four byte to write the bitstream size */
    evce_bsw_skip_tile_group_size(bs);

    /* chunk header */
    set_cnkh(ctx, &cnkh, EVC_VER_1, EVC_CT_SPS);
    evce_eco_cnkh(bs, &cnkh);

    /* sequence parameter set*/
    set_sps(ctx, sps);
    evc_assert_rv(evce_eco_sps(bs, sps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* picture parameter set*/
    set_pps(ctx, pps);
    evc_assert_rv(evce_eco_pps(bs, sps, pps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_tile_group_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->ctype = EVC_CT_SPS;

    return EVC_OK;
}

static void decide_normal_gop(EVCE_CTX * ctx, u32 pic_imcnt)
{
    int i_period, gop_size, pos;
    u32        pic_icnt_b;

    i_period = ctx->param.i_period;
    gop_size = ctx->param.gop_size;

    if(i_period == 0 && pic_imcnt == 0)
    {
        ctx->tile_group_type = TILE_GROUP_I;
        ctx->tile_group_depth = FRM_DEPTH_0;
        ctx->poc = pic_imcnt;
        ctx->tile_group_ref_flag = 1;
    }
    else if((i_period != 0) && pic_imcnt % i_period == 0)
    {
        ctx->tile_group_type = TILE_GROUP_I;
        ctx->tile_group_depth = FRM_DEPTH_0;
        ctx->poc = pic_imcnt;
        ctx->tile_group_ref_flag = 1;
    }
    else if(pic_imcnt % gop_size == 0)
    {
        ctx->tile_group_type = TILE_GROUP_B;
        ctx->tile_group_ref_flag = 1;
        ctx->tile_group_depth = FRM_DEPTH_1;
        ctx->poc = pic_imcnt;
        ctx->tile_group_ref_flag = 1;
    }
    else
    {
        ctx->tile_group_type = TILE_GROUP_B;
        if(ctx->param.use_hgop)
        {
            pos = (pic_imcnt % gop_size) - 1;
            ctx->tile_group_depth = tbl_tile_group_depth[gop_size >> 2][pos];
            ctx->ref_depth = tbl_ref_depth[gop_size >> 2][pos];
            ctx->poc = ((pic_imcnt / gop_size) * gop_size) +
                tbl_poc_gop_offset[gop_size >> 2][pos];
            ctx->tile_group_ref_flag = tbl_tile_group_ref[gop_size >> 2][pos];
        }
        else
        {
            pos = (pic_imcnt % gop_size) - 1;
            ctx->tile_group_depth = FRM_DEPTH_2;
            ctx->ref_depth = FRM_DEPTH_1;
            ctx->poc = ((pic_imcnt / gop_size) * gop_size) - gop_size + pos + 1;
            ctx->tile_group_ref_flag = 0;
        }
        /* find current encoding picture's(B picture) pic_icnt */
        pic_icnt_b = ctx->poc;

        /* find pico again here */
        ctx->pico_idx = (u8)(pic_icnt_b % ctx->pico_max_cnt);
        ctx->pico = ctx->pico_buf[ctx->pico_idx];

        PIC_ORIG(ctx) = &ctx->pico->pic;
    }
}

/* tile_group_type / tile_group_depth / poc / PIC_ORIG setting */
static void decide_tile_group_type(EVCE_CTX * ctx)
{
    u32 pic_imcnt, pic_icnt;
    int i_period, gop_size;
    int force_cnt = 0;

    i_period = ctx->param.i_period;
    gop_size = ctx->param.gop_size;
    pic_icnt = (ctx->pic_cnt + ctx->param.max_b_frames);
    pic_imcnt = pic_icnt;
    ctx->pico_idx = pic_icnt % ctx->pico_max_cnt;
    ctx->pico = ctx->pico_buf[ctx->pico_idx];
    PIC_ORIG(ctx) = &ctx->pico->pic;

    if(gop_size == 1) /* IPPP... */
    {
        pic_imcnt = (i_period > 0) ? pic_icnt % i_period : pic_icnt;
        if(pic_imcnt == 0)
        {
            ctx->tile_group_type = TILE_GROUP_I;
            ctx->tile_group_depth = FRM_DEPTH_0;
            ctx->poc = 0;
            ctx->tile_group_ref_flag = 1;
        }
        else
        {
            ctx->tile_group_type = TILE_GROUP_B;
            if(ctx->param.use_hgop)
            {
                ctx->tile_group_depth = tbl_tile_group_depth_P[(pic_imcnt - 1) % GOP_P];
            }
            else
            {
                ctx->tile_group_depth = FRM_DEPTH_1;
            }
            ctx->poc = (i_period > 0) ? ctx->pic_cnt % i_period : ctx->pic_cnt;
            ctx->tile_group_ref_flag = 1;
        }
    }
    else /* include B Picture (gop_size = 2 or 4 or 8 or 16) */
    {
        if(pic_icnt == gop_size - 1) /* special case when sequence start */
        {
            ctx->tile_group_type = TILE_GROUP_I;
            ctx->tile_group_depth = FRM_DEPTH_0;
            ctx->poc = 0;
            ctx->tile_group_ref_flag = 1;

            /* flush the first IDR picture */
            PIC_ORIG(ctx) = &ctx->pico_buf[0]->pic;
            ctx->pico = ctx->pico_buf[0];
        }
        else if(ctx->force_tile_group)
        {
            for(force_cnt = ctx->force_ignored_cnt; force_cnt < gop_size; force_cnt++)
            {
                pic_icnt = (ctx->pic_cnt + ctx->param.max_b_frames + force_cnt);
                pic_imcnt = pic_icnt;

                decide_normal_gop(ctx, pic_imcnt);

                if(ctx->poc <= (int)ctx->pic_ticnt)
                {
                    break;
                }
            }
            ctx->force_ignored_cnt = force_cnt;
        }
        else /* normal GOP case */
        {
            decide_normal_gop(ctx, pic_imcnt);
        }
    }

    if(ctx->param.use_hgop)
    {
        ctx->layer_id = ctx->tile_group_depth;
    }
    else
    {
        ctx->layer_id = 0;
    }

    ctx->ptr = ctx->poc;
    ctx->dtr = ctx->ptr;
}

int check_reorder(EVCE_PARAM * param, EVC_PM * pm, u8 num_ref_pics_act, u8 tile_group_type, u32 ptr, EVC_REFP(*refp)[REFP_NUM],
                  const EVC_REORDER_ARG * reorder, u32 last_intra, EVC_RMPNI * rmpni)
{
    int i, j, pos, poc, idx;
    const EVC_REORDER_ARG * cur_reorder = NULL;
    u8 rmpni_on[REFP_NUM] = {0, 0};
    int gop_num;
    int refp_idx, reorder_num_refpic;
    const s8 *reorder_refpic;

    rmpni[REFP_0].cnt = 0;
    rmpni[REFP_1].cnt = 0;

    if(param->max_b_frames + 1 == RMPNI_GOP_NUM)
    {
        gop_num = RMPNI_GOP_NUM;/*ptr(0~3), pos(1~4)*/
    }
    else if(param->max_b_frames == 0)
    {
        gop_num = RMPNI_GOP_NUM_LDB; /*ptr(0~15), pos(1~16)*/
    }
    else
    {
        return EVC_OK;
    }

    /* TILE_GROUP_P: current reordering argument is setup for TILE_GROUP_B only, TILE_GROUP_P case are not considered*/
    if(tile_group_type == TILE_GROUP_I || tile_group_type == TILE_GROUP_P)
    {
        return EVC_OK;
    }

    /* find current test set */
    pos = ((ptr - 1) % gop_num) + 1;
    for(i = 0; i < gop_num; i++)
    {
        if(reorder[i].poc == pos)
        {
            cur_reorder = &reorder[i];
            break;
        }
    }

    for(refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
    {
        if(refp_idx == REFP_0)
        {
            reorder_num_refpic = cur_reorder->num_ref_pics_l0;
            reorder_refpic = (s8 *)cur_reorder->ref_pic_l0;
        }
        else
        {
            reorder_num_refpic = cur_reorder->num_ref_pics_l1;
            reorder_refpic = (s8 *)cur_reorder->ref_pic_l1;
        }

        /* check whether use reordering or not */
        for(i = 0, idx = 0; i < reorder_num_refpic; i++)
        {
            poc = reorder_refpic[i] + ptr;
            for(j = 0; j < pm->cur_num_ref_pics && idx < num_ref_pics_act; j++)
            {
                if(ptr >= last_intra && pm->pic_ref[j]->ptr < last_intra)
                {
                    continue;
                }

                if(pm->pic_ref[j]->ptr == poc)
                {
                    if(refp[idx][refp_idx].ptr != poc)
                    {
                        rmpni_on[refp_idx] = 1;
                        break;
                    }
                    idx++;
                }
            }
        }

        /* if rmpni_on, reorder all reference picutres in list */
        if(rmpni_on[refp_idx])
        {
            for(i = 0, idx = 0; i < reorder_num_refpic && idx < num_ref_pics_act; i++)
            {
                poc = reorder_refpic[i] + ptr;
                for(j = 0; j < pm->cur_num_ref_pics; j++)
                {
                    if(ptr >= last_intra && pm->pic_ref[j]->ptr < last_intra)
                    {
                        continue;
                    }
                    if(pm->pic_ref[j]->ptr == poc)
                    {
                        rmpni[refp_idx].delta_poc[idx] = reorder_refpic[i];
                        rmpni[refp_idx].cnt++;
                        idx++;
                    }
                }
            }
        }
    }

    return (rmpni_on[REFP_0] || rmpni_on[REFP_1]);
}

int evce_enc_pic_prepare(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_PARAM   * param;
    int             ret;
    int             size;

    evc_assert_rv(PIC_ORIG(ctx) != NULL, EVC_ERR_UNEXPECTED);

    param = &ctx->param;
    ret = set_enc_param(ctx, param);
    evc_assert_rv(ret == EVC_OK, ret);

    PIC_CURR(ctx) = evc_picman_get_empty_pic(&ctx->rpm, &ret);
    evc_assert_rv(PIC_CURR(ctx) != NULL, ret);
    ctx->map_refi = PIC_CURR(ctx)->map_refi;
    ctx->map_mv = PIC_CURR(ctx)->map_mv;
#if DMVR_LAG
    ctx->map_unrefined_mv = PIC_CURR(ctx)->map_unrefined_mv;
#endif
    PIC_MODE(ctx) = PIC_CURR(ctx);
#if RDO_DBK
    if(ctx->pic_dbk == NULL)
    {
        ctx->pic_dbk = evce_pic_alloc(&ctx->rpm.pa, &ret);
        evc_assert_rv(ctx->pic_dbk != NULL, ret);
    }
#endif

    decide_tile_group_type(ctx);

    ctx->lcu_cnt = ctx->f_lcu;
    ctx->tile_group_num = 0;

    if(ctx->tile_group_type == TILE_GROUP_I) ctx->last_intra_ptr = ctx->ptr;

    size = sizeof(s8) * ctx->f_scu * REFP_NUM;
    evc_mset_x64a(ctx->map_refi, -1, size);

    size = sizeof(s16) * ctx->f_scu * REFP_NUM * MV_D;
    evc_mset_x64a(ctx->map_mv, 0, size);
#if DMVR_LAG
    size = sizeof(s16) * ctx->f_scu * REFP_NUM * MV_D;
    evc_mset_x64a(ctx->map_unrefined_mv, 0, size);
#endif
    /* initialize bitstream container */
    evc_bsw_init(&ctx->bs, bitb->addr, bitb->bsize, NULL);

    /* clear map */
    evc_mset_x64a(ctx->map_scu, 0, sizeof(u32) * ctx->f_scu);

#if AFFINE
    evc_mset_x64a(ctx->map_affine, 0, sizeof(u32) * ctx->f_scu);
#endif
    evc_mset_x64a(ctx->map_cu_mode, 0, sizeof(u32) * ctx->f_scu);

    return EVC_OK;
}

int evce_enc_pic_finish(EVCE_CTX *ctx, EVC_BITB *bitb, EVCE_STAT *stat)
{
    EVC_IMGB *imgb_o, *imgb_c;
    EVC_BSW  *bs;
    int        ret;
    int        i, j;

    bs = &ctx->bs;

    /* adding user data */
    if(ctx->tgh.udata_exist)
    {
        ret = evce_eco_udata(ctx, bs);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* ending */
    evce_bsw_write_tile_group_size(bs);

    /* expand current encoding picture, if needs */
    ctx->fn_picbuf_expand(ctx, PIC_CURR(ctx));

    /* picture buffer management */
    ret = evc_picman_put_pic(&ctx->rpm, PIC_CURR(ctx), ctx->tile_group_type,
                              ctx->ptr, ctx->dtr, ctx->layer_id, 0, ctx->refp,
                              (ctx->tgh.mmco_on ? &ctx->tgh.mmco : NULL), ctx->sps.picture_num_present_flag);

    evc_assert_rv(ret == EVC_OK, ret);

    imgb_o = PIC_ORIG(ctx)->imgb;
    evc_assert(imgb_o != NULL);

    imgb_c = PIC_CURR(ctx)->imgb;
    evc_assert(imgb_c != NULL);

    /* set stat */
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->ctype = EVC_CT_TILE_GROUP;
    stat->stype = ctx->tile_group_type;
    stat->fnum = ctx->pic_cnt;
    stat->qp = ctx->tgh.qp;
    stat->poc = ctx->ptr;
#if ALF
    stat->tid = ctx->tgh.layer_id;
#endif
    for(i = 0; i < 2; i++)
    {
        stat->refpic_num[i] = ctx->rpm.num_refp[i];
        for(j = 0; j < stat->refpic_num[i]; j++)
        {
            stat->refpic[i][j] = ctx->refp[j][i].ptr;
        }
    }

    ctx->pic_cnt++; /* increase picture count */
    ctx->param.f_ifrm = 0; /* clear force-IDR flag */
    ctx->pico->is_used = 0;

    imgb_c->ts[0] = bitb->ts[0] = imgb_o->ts[0];
    imgb_c->ts[1] = bitb->ts[1] = imgb_o->ts[1];
    imgb_c->ts[2] = bitb->ts[2] = imgb_o->ts[2];
    imgb_c->ts[3] = bitb->ts[3] = imgb_o->ts[3];

    if(imgb_o) imgb_o->release(imgb_o);

    return EVC_OK;
}

#if AQS 
//initial the look-up table for error sensitivity functions (shall be the same as evcd_init_es_factor)
void evce_init_es_factor(EVCE_CTX * ctx)
{
    int i = 0;
    double factor;

    //luminance masking with Wei's function
    //a larger value means lower error sensitivity
    for(i = 0; i < 60; i++)
    {
        factor = (1.0 + (60.0 - i) / 150.0);
        ctx->aqs.luminance_factor[i] = (u16)(factor * ESM_DEFAULT + 0.5);
    }
    for(i = 60; i < 170; i++)
    {
        factor = 1.0;
        ctx->aqs.luminance_factor[i] = (u16)(factor * ESM_DEFAULT + 0.5);
    }
    for(i = 170; i <= 256; i++)
    {
        factor = (1.0 + (i - 170.0) / 425.0);
        ctx->aqs.luminance_factor[i] = (u16)(factor * ESM_DEFAULT + 0.5);
    }

    //contrast masking
    //a larger value means lower error sensitivity
    for(i = 0; i < 256; i++)
    {
        int k = 5; //threshold of the zero-slope line

        //with pedestal effect
        if(i < k)
            factor = 1.0;
        else
            factor = (double)(i - k + 10.0) / 10.0;

        ctx->aqs.contrast_factor[i] = (u16)(factor * ESM_DEFAULT + 0.5);
    }
}

//fetch a block of pixels (in one plane) from a picture, given the up-left corner of the block (x,y) in the picture and the block size
//pel* pic,   picture buffer of one plane
//pel* block, block buffer
//int pic_w,  image width without padding
//int pic_h,  image height --
//int x,      the x coordinate of the upper-left corner of the block
//int y,      the y --
//int block_w, width of the block
//int block_h, height of --
//int pic_s   stride of the picture
void evce_fetch_block(pel* pic, pel* block, int pic_w, int pic_h, int x, int y, int block_w, int block_h, int pic_s)
{
    int i, j, ind_p, ind_b, k, kk, posx, posy;

    for(j = 0; j < block_h; j++)
    {
        posy = y + j;
        posy = posy >= pic_h ? (pic_h - 1) : posy;
        k = posy * pic_s;
        kk = j * block_w;
        for(i = 0; i < block_w; i++)
        {
            posx = x + i;
            posx = posx >= pic_w ? (pic_w - 1) : posx;
            ind_p = k + posx;
            ind_b = kk + i;
            block[ind_b] = pic[ind_p];
            assert(block[ind_b] >= 0 && block[ind_b] <= 1023);
        }
    }
}

void evce_assign_block(u16* pic, u16 val, int pic_w, int pic_h, int x, int y, int block_w, int block_h)
{
    int i, j, ind_p, k, posx, posy;

    for(j = 0; j < block_h; j++)
    {
        posy = y + j;
        if(posy >= pic_h)
            break;
        k = posy * pic_w;
        for(i = 0; i < block_w; i++)
        {
            posx = x + i;
            if(posx >= pic_w)
                break;
            ind_p = k + posx;
            pic[ind_p] = val;
        }
    }
}

void evce_assign_block_u8(u8* pic, u16 val, int pic_w, int pic_h, int x, int y, int block_w, int block_h)
{
    int i, j, ind_p, k, posx, posy;

    for(j = 0; j < block_h; j++)
    {
        posy = y + j;
        if(posy >= pic_h)
            break;
        k = posy * pic_w;
        for(i = 0; i < block_w; i++)
        {
            posx = x + i;
            if(posx >= pic_w)
                break;
            ind_p = k + posx;
            pic[ind_p] = (u8)val;
        }
    }
}

//get the average luminance of a block
int evce_get_block_avg(pel *buff, int block_w, int block_h)
{
    int sum = 0;
    int i;
    int block_size = block_w * block_h;

    for(i = 0; i < block_size; i++)
        sum += (int)buff[i];

    return (sum + (block_size >> 1)) / block_size;
}

//get simplified "standard deviation" of a block
int evce_get_block_std_simp(pel *buff, int block_w, int block_h, int average)
{
    int sum = 0;
    int i;
    int block_size = block_w*block_h;

    for(i = 0; i < block_size; i++)
        sum += abs(buff[i] - average);

    return (sum + (block_size >> 1)) / block_size;
}

void evce_es_map_est(EVCE_CTX * ctx)
{
    int i, j, k;
    pel* pic_luma = ctx->pico->pic.y;
    int pic_w = ctx->pico->pic.w_l;
    int pic_h = ctx->pico->pic.h_l;
    int pic_s = ctx->pico->pic.s_l;
    int fac_lumi = 1;
    int fac_cont = 1;

    int blk_w = 8;
    int blk_h = 8;
    int bg_w = 8;
    int bg_h = 8;

    pel* bg_pxl = NULL;
    int bg_size;
    int i_offset = (bg_w - blk_w) / 2;
    int j_offset = (bg_h - blk_h) / 2;
    int a, b;
    int avg_bg, std_bg;
    u16 es_val_blk;
    int scu_w = 1 << MIN_CU_LOG2;
    int scu_h = 1 << MIN_CU_LOG2;

    //for deriving the geometric mean of es map
    u16 esm_avg = 0;
    s16 esm_offset = 0;
    int Num = ((pic_h + (blk_h - 1)) / blk_h) * ((pic_w + (blk_w - 1)) / blk_w);
    u16 val = 0;
    double esm_geo_mean = 0;

    //for aqs normalizer derivation
    u16 tabel_th_pos[11] = {261, 273, 285, 298, 311, 325, 340, 354, 370, 387, 403};
    u16 tabel_th_neg[11] = {251, 240, 230, 220, 211, 202, 193, 185, 177, 170, 163};
    s16 table_factor_pos[12] = {256, 267, 279, 291, 305, 318, 333, 347, 362, 379, 395, 412}; //also 8-bit 
    s16 table_factor_neg[12] = {256, 245, 235, 225, 215, 206, 197, 189, 181, 173, 166, 159};
    int clipVal = 11;

    //normalizing the ES value into [0.5, 2.0]
    int up = ESM_DEFAULT << 1; //MAX_ESM
    int low = ESM_DEFAULT >> 1; //MIN_ESM
    int norm_inv;

    //allocate memory
    bg_size = bg_w * bg_h;
    bg_pxl = (pel*)malloc(bg_size * sizeof(pel));

    for(j = 0; j < pic_h; j = j + blk_h)
    {
        k = j*pic_w;
        for(i = 0; i < pic_w; i = i + blk_w)
        {

            //prepare background for a block
            if(blk_w == bg_w)
                evce_fetch_block(pic_luma, bg_pxl, pic_w, pic_h, i, j, bg_w, bg_h, pic_s);
            else if(bg_w > blk_w)
            {
                a = i - i_offset;
                b = j - j_offset;
                if(a >= 0 && b >= 0)
                    evce_fetch_block(pic_luma, bg_pxl, pic_w, pic_h, a, b, bg_w, bg_h, pic_s);
                else //approximation for blk at image margin
                    evce_fetch_block(pic_luma, bg_pxl, pic_w, pic_h, (i / bg_w)*bg_w, (j / bg_h)*bg_h, bg_w, bg_h, pic_s);
            }

            //get average luminance and simplified "standard deviation" of the background
            avg_bg = evce_get_block_avg(bg_pxl, bg_w, bg_h);
            std_bg = evce_get_block_std_simp(bg_pxl, bg_w, bg_h, avg_bg);
            avg_bg = (avg_bg + 2) >> 2; //[0, 1023] normalize to [0, 256] space, since internal bit-depth is 10
            std_bg = (std_bg + 2) >> 2;

            //luminance
            fac_lumi = ctx->aqs.luminance_factor[avg_bg];
            //contrast
            fac_cont = ctx->aqs.contrast_factor[std_bg];
            //perceptual ESM weight of the blk
            fac_lumi = fac_lumi * fac_cont;
            es_val_blk = EVC_CLIP3(low, up, ((1 << (ESM_SHIFT * 3 + 1)) + (fac_lumi >> 1)) / fac_lumi);

            evce_assign_block(ctx->aqs.es_map_buf, es_val_blk, ctx->aqs.es_map_width, ctx->aqs.es_map_height, i >> MIN_CU_LOG2, j >> MIN_CU_LOG2, blk_w >> MIN_CU_LOG2, blk_h >> MIN_CU_LOG2);
        }
    }

    // modify ESMap by considering overall image feature for bitrate alignment
    for(j = 0; j < pic_h; j = j + blk_h)
    {
        k = (j >> MIN_CU_LOG2)*ctx->aqs.es_map_stride;
        for(i = 0; i < pic_w; i = i + blk_w)
        {
            esm_geo_mean += log((double)ctx->aqs.es_map_buf[k + (i >> MIN_CU_LOG2)]);
        }
    }
    esm_geo_mean = exp(esm_geo_mean / (double)Num);
    esm_avg = (u16)esm_geo_mean;

    if(ctx->tgh.tile_group_type == TILE_GROUP_I)
        esm_avg = (esm_avg * 5 + ESM_DEFAULT * 3) >> 3;
    else
        esm_avg = (esm_avg * 3 + ESM_DEFAULT * 5) >> 3;
    //qp related change
    if(ctx->tgh.qp >= 30)
        esm_avg = esm_avg - (ctx->tgh.qp - 30) * 3;
    else
        esm_avg = esm_avg - (ctx->tgh.qp - 30) * 4;
    esm_avg = (u16)EVC_CLIP3((u16)(ESM_DEFAULT / 1.4), (u16)(ESM_DEFAULT*1.4), esm_avg);

    esm_offset = esm_avg - ESM_DEFAULT;

    //***************** quantize, since this value will be transmitted but it does not need high precision ***********
    // get factor index
    if(esm_offset >= 0)
    {
        ctx->aqs.es_map_norm_idx = clipVal;
        for(i = 0; i <= clipVal; i++)
        {
            if(esm_avg < tabel_th_pos[i])
            {
                ctx->aqs.es_map_norm_idx = i;
                break;
            }
        }
    }
    else
    {
        ctx->aqs.es_map_norm_idx = -clipVal;
        for(i = 0; i <= clipVal; i++)
        {
            if(esm_avg > tabel_th_neg[i])
            {
                ctx->aqs.es_map_norm_idx = -i;
                break;
            }
        }
    }

    //update normalization factor
    if(ctx->aqs.es_map_norm_idx >= 0)
    {
        ctx->aqs.es_map_norm = table_factor_pos[ctx->aqs.es_map_norm_idx];
        norm_inv = table_factor_neg[ctx->aqs.es_map_norm_idx];
    }
    else
    {
        ctx->aqs.es_map_norm = table_factor_neg[-ctx->aqs.es_map_norm_idx];
        norm_inv = table_factor_pos[-ctx->aqs.es_map_norm_idx];
    }
    //print es_map_norm
    if(ctx->ptr == 0)
        printf("\nEs_geomean %3d\tES_map_norm %3d\n", (u16)esm_geo_mean, ctx->aqs.es_map_norm);

    // modify ESM values
    for(j = 0; j < pic_h; j = j + scu_h)
    {
        k = (j >> MIN_CU_LOG2)*ctx->aqs.es_map_stride;
        for(i = 0; i < pic_w; i = i + scu_w)
        {
            val = ctx->aqs.es_map_buf[k + (i >> MIN_CU_LOG2)];
            ctx->aqs.es_map_buf[k + (i >> MIN_CU_LOG2)] = EVC_CLIP3(low, up, (val * norm_inv + 128) >> 8);
        }
    }

    if(bg_pxl)
        free(bg_pxl);
}
#endif

int evce_enc_pic(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CORE * core;
    EVC_BSW   * bs;
    EVC_TGH    * tgh;
    EVC_CNKH    cnkh;
    int          ret;
    u32          i;
    int split_mode_child[4];
    int split_allow[6] = { 0, 0, 0, 0, 0, 1 };

    bs = &ctx->bs;
    core = ctx->core;
    tgh = &ctx->tgh;

    if (ctx->sps.picture_num_present_flag)
    {
        /* initialize reference pictures */
        ret = evc_picman_refp_init(&ctx->rpm, ctx->sps.num_ref_pics_act, ctx->tile_group_type, ctx->ptr, ctx->layer_id, ctx->last_intra_ptr, ctx->refp);
    }
    else
    {
        /* Set tile_group header */
        //This needs to be done before reference picture marking and reference picture list construction are invoked
        set_tgh(ctx, tgh);

        if (tgh->tile_group_type != TILE_GROUP_I && tgh->poc != 0) //TBD: change this condition to say that if this tile_group is not a tile_group in IDR picture
        {
            ret = create_explicit_rpl(&ctx->rpm, tgh);
            if (ret == 1)
            {
                if (tgh->rpl_l0_idx == -1)
                    tgh->ref_pic_list_sps_flag[0] = 0;
                if (tgh->rpl_l1_idx == -1)
                    tgh->ref_pic_list_sps_flag[1] = 0;
            }
        }

        /* reference picture marking */
        ret = evc_picman_refpic_marking(&ctx->rpm, tgh);
        evc_assert_rv(ret == EVC_OK, ret);

        /* reference picture lists construction */
        ret = evc_picman_refp_rpl_based_init(&ctx->rpm, tgh, ctx->refp);
    }
    evc_assert_rv(ret == EVC_OK, ret);

    if (ctx->sps.picture_num_present_flag)
    {
        if (ctx->tile_group_type != TILE_GROUP_I)
        {
            if (ctx->param.max_b_frames == 0)
            {
                tgh->rmpni_on = check_reorder(&ctx->param, &ctx->rpm, ctx->sps.num_ref_pics_act, ctx->tile_group_type, ctx->ptr, ctx->refp, reorder_arg_ldb, ctx->last_intra_ptr, tgh->rmpni);
            }
            else
            {
                tgh->rmpni_on = check_reorder(&ctx->param, &ctx->rpm, ctx->sps.num_ref_pics_act, ctx->tile_group_type, ctx->ptr, ctx->refp, reorder_arg, ctx->last_intra_ptr, tgh->rmpni);
            }
        }

        if (tgh->rmpni_on && ctx->tile_group_type != TILE_GROUP_I)
        {
            ret = evc_picman_refp_reorder(&ctx->rpm, ctx->sps.num_ref_pics_act, tgh->tile_group_type, ctx->ptr, ctx->refp, ctx->last_intra_ptr, tgh->rmpni);
            evc_assert_rv(ret == EVC_OK, ret);
        }
    }

    /* initialize mode decision for frame encoding */
    ret = ctx->fn_mode_init_frame(ctx);
    evc_assert_rv(ret == EVC_OK, ret);

    ctx->fn_mode_analyze_frame(ctx);

    /* tile_group layer encoding loop */
    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->f_lcu;

    /* Set chunk header */
    set_cnkh(ctx, &cnkh, EVC_VER_1, EVC_CT_TILE_GROUP);

    if (ctx->sps.picture_num_present_flag)
    {
        /* Set tile_group header */
        set_tgh(ctx, tgh);
    }

    core->qp_y = ctx->tgh.qp + 6 * (BIT_DEPTH - 8);
    core->qp_u = evc_tbl_qp_chroma_ajudst[tgh->qp_u] + 6 * (BIT_DEPTH - 8);
    core->qp_v = evc_tbl_qp_chroma_ajudst[tgh->qp_v] + 6 * (BIT_DEPTH - 8);

#if ADMVP
    evc_mset(core->history_buffer.history_mv_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * MV_D * sizeof(s16));
    
    //evc_mset(core->history_buffer.history_refi_table, 0, ALLOWED_CHECKED_NUM * REFP_NUM * sizeof(s8));
    for (int i = 0; i < ALLOWED_CHECKED_NUM; i++)
    {
        core->history_buffer.history_refi_table[i][REFP_0] = REFI_INVALID;
        core->history_buffer.history_refi_table[i][REFP_1] = REFI_INVALID;
    }

    core->history_buffer.currCnt = 0;
    core->history_buffer.m_maxCnt = ALLOWED_CHECKED_NUM;
#endif

    /* initialize entropy coder */
    evce_sbac_reset(GET_SBAC_ENC(bs), ctx->tgh.tile_group_type, ctx->tgh.qp, ctx->sps.tool_cm_init);
    evce_sbac_reset(&core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], ctx->tgh.tile_group_type, ctx->tgh.qp, ctx->sps.tool_cm_init);

    core->bs_temp.pdata[1] = &core->s_temp_run;
#if AQS
    ctx->aqs.es_map_norm_idx = 0;
    ctx->aqs.es_map_norm = ESM_DEFAULT;
#endif
#if AQS_ENC 
    evce_es_map_est(ctx);
#endif
    /* LCU encoding */
#if TRACE_RDO_EXCLUDE_I
    if(ctx->tile_group_type != TILE_GROUP_I)
    {
#endif
        EVC_TRACE_SET(0);
#if TRACE_RDO_EXCLUDE_I
    }
#endif

    while(1)
    {
        /* initialize structures *****************************************/
        ret = ctx->fn_mode_init_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);

        /* mode decision *************************************************/
        SBAC_LOAD(core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], *GET_SBAC_ENC(bs));
        core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].is_bitcount = 1;

        evce_init_bef_data(core, ctx);

        ret = ctx->fn_mode_analyze_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);

        /* entropy coding ************************************************/

        ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1
                             , NO_SPLIT, split_mode_child, 0, split_allow, 0, 0);
        evc_assert_rv(ret == EVC_OK, ret);

        /* prepare next step *********************************************/
        core->x_lcu++;
        if(core->x_lcu >= ctx->w_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }
        core->x_pel = core->x_lcu << ctx->log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->log2_max_cuwh;
        core->lcu_num++;
        ctx->lcu_cnt--;

#if HISTORY_LCU_COPY_BUG_FIX
        evc_mcpy(&core->history_buffer, &core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], sizeof(core->history_buffer));
#endif

        /* end_of_picture_flag */
        if(ctx->lcu_cnt > 0)
        {
            evce_eco_tile_group_end_flag(bs, 0);
        }
        else
        {
            evce_eco_tile_group_end_flag(bs, 1);
            evce_sbac_finish(bs);
            break;
        }
    }

    /* deblocking filter */
    if(ctx->param.use_deblock)
    {
        ret = ctx->fn_deblock(ctx, PIC_MODE(ctx));
        evc_assert_rv(ret == EVC_OK, ret);
    }

#if ALF
    /* adaptive loop filter */
    tgh->alf_on = ctx->sps.tool_alf;
    if(tgh->alf_on)
    {
        ret = ctx->fn_alf(ctx, PIC_MODE(ctx), tgh);
        evc_assert_rv(ret == EVC_OK, ret);
    }
#endif

#if AQS_SYNTAX
    tgh->es_map_norm_idx = 0;
    tgh->es_map_norm = ESM_DEFAULT;
#endif
#if AQS_ENC
    tgh->es_map_norm_idx = ctx->aqs.es_map_norm_idx;
    tgh->es_map_norm = ctx->aqs.es_map_norm;
#endif


    /* Bit-stream re-writing (START) */
    evc_bsw_init(&ctx->bs, (u8*)bitb->addr, bitb->bsize, NULL);
#if TRACE_RDO_EXCLUDE_I
    if(ctx->tile_group_type != TILE_GROUP_I)
    {
#endif
        EVC_TRACE_SET(1);
#if TRACE_RDO_EXCLUDE_I
    }
#endif

    /* Encode skip tile_group_size field */
    evce_bsw_skip_tile_group_size(bs);

    /* Encode chunk header */
    ret = evce_eco_cnkh(bs, &cnkh);
    evc_assert_rv(ret == EVC_OK, ret);

    /* Encode tile_group header */
#if ALF
    tgh->num_ctb = ctx->f_lcu;
#endif
    ret = evce_eco_tgh(bs, &ctx->sps, &ctx->pps, tgh);
    evc_assert_rv(ret == EVC_OK, ret);

    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->f_lcu;
    for(i = 0; i < ctx->f_scu; i++)
    {
        MCU_CLR_COD(ctx->map_scu[i]);
    }

    evce_sbac_reset(GET_SBAC_ENC(bs), ctx->tgh.tile_group_type, ctx->tgh.qp, ctx->sps.tool_cm_init);

    /* Encode tile_group data */
    while(1)
    {
        ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1, NO_SPLIT, split_mode_child, 0, split_allow, 0, 0);
        evc_assert_rv(ret == EVC_OK, ret);

        core->x_lcu++;
        if(core->x_lcu >= ctx->w_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }
        core->x_pel = core->x_lcu << ctx->log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->log2_max_cuwh;
        core->lcu_num++;
        ctx->lcu_cnt--;
        /* end_of_picture_flag */
        if(ctx->lcu_cnt > 0)
        {
            evce_eco_tile_group_end_flag(bs, 0);
        }
        else
        {
            evce_eco_tile_group_end_flag(bs, 1);
            evce_sbac_finish(bs);
            break;
        }
    }

    /* Bit-stream re-writing (END) */

    return EVC_OK;
}

int evce_enc(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    int            ret;
    int            gop_size, pic_cnt;

    pic_cnt = ctx->pic_icnt - ctx->frm_rnum;
    gop_size = ctx->param.gop_size;

    ctx->force_tile_group = ((ctx->pic_ticnt % gop_size >= ctx->pic_ticnt - pic_cnt + 1) && FORCE_OUT(ctx)) ? 1 : 0;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* initialize variables for a picture encoding */
    ret = ctx->fn_enc_pic_prepare(ctx, bitb, stat);
    evc_assert_rv(ret == EVC_OK, ret);

    /* encode one picture */
    ret = ctx->fn_enc_pic(ctx, bitb, stat);
    evc_assert_rv(ret == EVC_OK, ret);

    /* finishing of encoding a picture */
    ctx->fn_enc_pic_finish(ctx, bitb, stat);
    evc_assert_rv(ret == EVC_OK, ret);

    return EVC_OK;
}

int evce_push_frm(EVCE_CTX * ctx, EVC_IMGB * imgb)
{
    EVC_PIC    * pic;

    ctx->pic_icnt++;

    ctx->pico_idx = ctx->pic_icnt % ctx->pico_max_cnt;
    ctx->pico = ctx->pico_buf[ctx->pico_idx];
    ctx->pico->pic_icnt = ctx->pic_icnt;
    ctx->pico->is_used = 1;
    pic = &ctx->pico->pic;
    PIC_ORIG(ctx) = pic;

    /* set pushed image to current input (original) image */
    evc_mset(pic, 0, sizeof(EVC_PIC));

    pic->buf_y = imgb->baddr[0];
    pic->buf_u = imgb->baddr[1];
    pic->buf_v = imgb->baddr[2];

    pic->y = imgb->a[0];
    pic->u = imgb->a[1];
    pic->v = imgb->a[2];

    pic->w_l = imgb->w[0];
    pic->h_l = imgb->h[0];
    pic->w_c = imgb->w[1];
    pic->h_c = imgb->h[1];

    pic->s_l = STRIDE_IMGB2PIC(imgb->s[0]);
    pic->s_c = STRIDE_IMGB2PIC(imgb->s[1]);

    pic->imgb = imgb;

    imgb->addref(imgb);

    return EVC_OK;
}

int evce_platform_init(EVCE_CTX * ctx)
{
    int ret = EVC_ERR_UNKNOWN;

    /* create mode decision */
    ret = evce_mode_create(ctx, 0);
    evc_assert_rv(EVC_OK == ret, ret);

    /* create intra prediction analyzer */
    ret = evce_pintra_create(ctx, 0);
    evc_assert_rv(EVC_OK == ret, ret);

    /* create inter prediction analyzer */
    ret = evce_pinter_create(ctx, 0);
    evc_assert_rv(EVC_OK == ret, ret);
#if RDO_DBK
    ctx->pic_dbk = NULL;
#endif

#if ALF
    ctx->fn_alf = evce_alf;
#endif
    ctx->fn_ready = evce_ready;
    ctx->fn_flush = evce_flush;
    ctx->fn_enc = evce_enc;
    ctx->fn_enc_header = evce_enc_header;
    ctx->fn_enc_pic = evce_enc_pic;
    ctx->fn_enc_pic_prepare = evce_enc_pic_prepare;
    ctx->fn_enc_pic_finish = evce_enc_pic_finish;
    ctx->fn_push = evce_push_frm;
    ctx->fn_deblock = evce_deblock_h263;
    ctx->fn_picbuf_expand = evce_picbuf_expand;
    ctx->fn_get_inbuf = evce_picbuf_get_inbuf;
    ctx->pf = NULL;

    return EVC_OK;
}

void evce_platform_deinit(EVCE_CTX * ctx)
{
    evc_assert(ctx->pf == NULL);

    ctx->fn_ready = NULL;
    ctx->fn_flush = NULL;
    ctx->fn_enc = NULL;
    ctx->fn_enc_pic = NULL;
    ctx->fn_enc_pic_prepare = NULL;
    ctx->fn_enc_pic_finish = NULL;
    ctx->fn_push = NULL;
    ctx->fn_deblock = NULL;
#if ALF
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);
    call_destroy_enc_ALF(p);
    delete_enc_ALF(ctx->enc_alf);
    ctx->fn_alf = NULL;
#endif
    ctx->fn_picbuf_expand = NULL;
    ctx->fn_get_inbuf = NULL;
}

EVCE evce_create(EVCE_CDSC * cdsc, int * err)
{
    EVCE_CTX  * ctx;
    int          ret;

#if ENC_DEC_TRACE
    fp_trace = fopen("enc_trace.txt", "w+");
#endif
    ctx = NULL;

    /* memory allocation for ctx and core structure */
    ctx = ctx_alloc();
    evc_assert_gv(ctx != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mcpy(&ctx->cdsc, cdsc, sizeof(EVCE_CDSC));

    /* set default value for encoding parameter */
    ret = set_init_param(cdsc, &ctx->param);
    evc_assert_g(ret == EVC_OK, ERR);

    ret = evce_platform_init(ctx);
    evc_assert_g(ret == EVC_OK, ERR);

    ret = evc_scan_tbl_init();
    evc_assert_g(ret == EVC_OK, ERR);

    evce_init_err_scale();
    evce_split_tbl_init(ctx);

#if AQS 
    //initial look-up tables for factors and the es map
    evce_init_es_factor(ctx);
    ctx->aqs.es_map_width = ctx->param.w >> MIN_CU_LOG2;
    ctx->aqs.es_map_height = ctx->param.h >> MIN_CU_LOG2;
    ctx->aqs.es_map_stride = ctx->aqs.es_map_width;
    ctx->aqs.es_map_buf = (u16*)malloc(ctx->aqs.es_map_stride * ctx->aqs.es_map_height * sizeof(u16));
#endif

    if(ctx->fn_ready != NULL)
    {
        ret = ctx->fn_ready(ctx);
        evc_assert_g(ret == EVC_OK, ERR);
    }

    /* set default value for ctx */
    ctx->magic = EVCE_MAGIC_CODE;
    ctx->id = (EVCE)ctx;

    return (ctx->id);
ERR:
    if(ctx)
    {
        evce_platform_deinit(ctx);
        ctx_free(ctx);
    }
    if(err) *err = ret;
    return NULL;
}

void evce_delete(EVCE id)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_R(id, ctx);
#if ENC_DEC_TRACE
    fclose(fp_trace);
#endif

    if(ctx->fn_flush != NULL)
    {
        ctx->fn_flush(ctx);
    }
    evce_platform_deinit(ctx);

    ctx_free(ctx);

    evc_scan_tbl_delete();
}

int evce_encode_header(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc_header, EVC_ERR_UNEXPECTED);

    /* update BSB */
    bitb->err = 0;

    return ctx->fn_enc_header(ctx, bitb, stat);
}

static int check_frame_delay(EVCE_CTX * ctx)
{
    if(ctx->pic_icnt < ctx->frm_rnum)
    {
        return EVC_OK_OUT_NOT_AVAILABLE;
    }
    return EVC_OK;
}

static int check_more_frames(EVCE_CTX * ctx)
{
    EVCE_PICO  * pico;
    int           i;

    if(FORCE_OUT(ctx))
    {
        /* pseudo evce_push() for bumping process ****************/
        ctx->pic_icnt++;
        /**********************************************************/

        for(i=0; i<ctx->pico_max_cnt; i++)
        {
            pico = ctx->pico_buf[i];
            if(pico != NULL)
            {
                if(pico->is_used == 1)
                {
                    return EVC_OK;
                }
            }
        }

        return EVC_OK_NO_MORE_FRM;
    }

    return EVC_OK;
}

int evce_encode(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc, EVC_ERR_UNEXPECTED);

    /* bumping - check whether input pictures are remaining or not in pico_buf[] */
    if(EVC_OK_NO_MORE_FRM == check_more_frames(ctx))
    {
        return EVC_OK_NO_MORE_FRM;
    }

    /* store input picture and return if needed */
    if(EVC_OK_OUT_NOT_AVAILABLE == check_frame_delay(ctx))
    {
        return EVC_OK_OUT_NOT_AVAILABLE;
    }

    /* update BSB */
    bitb->err = 0;

    return ctx->fn_enc(ctx, bitb, stat);
}

int evce_push(EVCE id, EVC_IMGB * img)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_push, EVC_ERR_UNEXPECTED);

    return ctx->fn_push(ctx, img);
}

int evce_config(EVCE id, int cfg, void * buf, int * size)
{
    EVCE_CTX      * ctx;
    int               t0;
    EVC_IMGB      * imgb;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);

    switch(cfg)
    {
        /* set config **********************************************************/
        case EVCE_CFG_SET_FORCE_OUT:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.force_output = (t0) ? 1 : 0;
            /* store total input picture count at this time */
            ctx->pic_ticnt = ctx->pic_icnt;
            break;

        case EVCE_CFG_SET_FINTRA:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.f_ifrm = t0;
            break;
        case EVCE_CFG_SET_QP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 >= MIN_QUANT && t0 <= MAX_QUANT, \
                           EVC_ERR_INVALID_ARGUMENT);
            ctx->param.qp = t0;
            break;
        case EVCE_CFG_SET_FPS:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 > 0, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.fps = t0;
            break;
        case EVCE_CFG_SET_I_PERIOD:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 >= 0, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.i_period = t0;
            break;
        case EVCE_CFG_SET_QP_MIN:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 >= MIN_QUANT, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.qp_min = t0;
            break;
        case EVCE_CFG_SET_QP_MAX:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            evc_assert_rv(t0 <= MAX_QUANT, EVC_ERR_INVALID_ARGUMENT);
            ctx->param.qp_max = t0;
            break;
        case EVCE_CFG_SET_USE_DEBLOCK:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.use_deblock = t0;
            break;
        case EVCE_CFG_SET_USE_PIC_SIGNATURE:
            ctx->param.use_pic_sign = (*((int *)buf)) ? 1 : 0;
            break;

            /* get config *******************************************************/
        case EVCE_CFG_GET_QP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.qp;
            break;
        case EVCE_CFG_GET_WIDTH:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.w;
            break;
        case EVCE_CFG_GET_HEIGHT:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.h;
            break;
        case EVCE_CFG_GET_FPS:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.fps;
            break;
        case EVCE_CFG_GET_I_PERIOD:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.i_period;
            break;
        case EVCE_CFG_GET_RECON:
            evc_assert_rv(*size == sizeof(EVC_IMGB**), EVC_ERR_INVALID_ARGUMENT);
            imgb = PIC_CURR(ctx)->imgb;
            *((EVC_IMGB **)buf) = imgb;
            imgb->addref(imgb);
            break;
        case EVCE_CFG_GET_USE_DEBLOCK:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.use_deblock;
            break;
        case EVCE_CFG_GET_CLOSED_GOP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.use_closed_gop;
            break;
        case EVCE_CFG_GET_HIERARCHICAL_GOP:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.use_hgop;
            break;
        default:
            evc_trace("unknown config value (%d)\n", cfg);
            evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
    }

    return EVC_OK;
}

int evce_get_inbuf(EVCE id, EVC_IMGB ** img)
{
    EVCE_CTX *ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_get_inbuf, EVC_ERR_UNEXPECTED);
    evc_assert_rv(img != NULL, EVC_ERR_INVALID_ARGUMENT);

    return ctx->fn_get_inbuf(ctx, img);
}

void evce_malloc_1d(void** dst, int size)
{
    if(*dst == NULL)
    {
        *dst = evc_malloc_fast(size);
        evc_mset(*dst, 0, size);
    }
}

void evce_malloc_2d(s8*** dst, int size_1d, int size_2d, int type_size)
{
    int i;

    if(*dst == NULL)
    {
        *dst = evc_malloc_fast(size_1d * sizeof(s8*));
        evc_mset(*dst, 0, size_1d * sizeof(s8*));


        (*dst)[0] = evc_malloc_fast(size_1d * size_2d * type_size);
        evc_mset((*dst)[0], 0, size_1d * size_2d * type_size);

        for(i = 1; i < size_1d; i++)
        {
            (*dst)[i] = (*dst)[i - 1] + size_2d * type_size;
        }
    }
}

int evce_create_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh)
{
    int i, j;
    int cuw_scu, cuh_scu;
    int size_8b, size_16b, size_32b, cu_cnt, pixel_cnt;

    cuw_scu = 1 << log2_cuw;
    cuh_scu = 1 << log2_cuh;

    size_8b = cuw_scu * cuh_scu * sizeof(s8);
    size_16b = cuw_scu * cuh_scu * sizeof(s16);
    size_32b = cuw_scu * cuh_scu * sizeof(s32);
    cu_cnt = cuw_scu * cuh_scu;
    pixel_cnt = cu_cnt << 4;

    evce_malloc_1d((void**)&cu_data->qp_y, size_8b);
    evce_malloc_1d((void**)&cu_data->qp_u, size_8b);
    evce_malloc_1d((void**)&cu_data->qp_v, size_8b);
    evce_malloc_1d((void**)&cu_data->pred_mode, size_8b);
    evce_malloc_2d((s8***)&cu_data->mpm, 2, cu_cnt, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->ipm, 2, cu_cnt, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->mpm_ext, 8, cu_cnt, sizeof(u8));
    evce_malloc_1d((void**)&cu_data->skip_flag, size_8b);
#if DMVR_FLAG
    evce_malloc_1d((void**)&cu_data->dmvr_flag, size_8b);
#endif
    evce_malloc_2d((s8***)&cu_data->refi, cu_cnt, REFP_NUM, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->mvp_idx, cu_cnt, REFP_NUM, sizeof(u8));
    evce_malloc_1d((void**)&cu_data->mvr_idx, size_8b);
    evce_malloc_1d((void**)&cu_data->bi_idx, size_8b);
    evce_malloc_1d((void**)&cu_data->mmvd_idx, size_16b);
    evce_malloc_1d((void**)&cu_data->mmvd_flag, size_8b);

    for(i = 0; i < N_C; i++)
    {
        evce_malloc_1d((void**)&cu_data->nnz[i], size_32b);
    }
    for (i = 0; i < N_C; i++)
    {
        for (j = 0; j < 4; j++)
        {
            evce_malloc_1d((void**)&cu_data->nnz_sub[i][j], size_32b);
        }
    }
    evce_malloc_1d((void**)&cu_data->map_scu, size_32b);
#if AFFINE
    evce_malloc_1d((void**)&cu_data->affine_flag, size_8b);
    evce_malloc_1d((void**)&cu_data->map_affine, size_32b);
#endif    
    evce_malloc_1d((void**)&cu_data->map_cu_mode, size_32b);
    evce_malloc_2d((s8***)&cu_data->block_size, cu_cnt, 2, sizeof(s16));
    evce_malloc_1d((void**)&cu_data->depth, size_8b);

    for(i = 0; i < N_C; i++)
    {
        evce_malloc_1d((void**)&cu_data->coef[i], (pixel_cnt >> (!!(i)* 2)) * sizeof(s16));
        evce_malloc_1d((void**)&cu_data->reco[i], (pixel_cnt >> (!!(i)* 2)) * sizeof(pel));
    }

    return EVC_OK;
}

void evce_free_1d(void* dst)
{
    if(dst != NULL)
    {
        evc_mfree_fast(dst);
    }
}

void evce_free_2d(void** dst)
{
    if (dst)
    {
        if (dst[0])
        {
            evc_mfree_fast(dst[0]);
        }
        evc_mfree_fast(dst);
    }
}

int evce_delete_cu_data(EVCE_CU_DATA *cu_data, int log2_cuw, int log2_cuh)
{
    int i, j;

    evce_free_1d((void*)cu_data->qp_y);
    evce_free_1d((void*)cu_data->qp_u);
    evce_free_1d((void*)cu_data->qp_v);
    evce_free_1d((void*)cu_data->pred_mode);
    evce_free_2d((void**)cu_data->mpm);
    evce_free_2d((void**)cu_data->ipm);
    evce_free_2d((void**)cu_data->mpm_ext);
    evce_free_1d((void*)cu_data->skip_flag);
#if DMVR_FLAG
    evce_free_1d((void*)cu_data->dmvr_flag);
#endif
    evce_free_2d((void**)cu_data->refi);
    evce_free_2d((void**)cu_data->mvp_idx);
    evce_free_1d(cu_data->mvr_idx);
    evce_free_1d(cu_data->bi_idx);
    evce_free_1d((void*)cu_data->mmvd_idx);
    evce_free_1d((void*)cu_data->mmvd_flag);

    for (i = 0; i < N_C; i++)
    {
        evce_free_1d((void*)cu_data->nnz[i]);
    }
    for (i = 0; i < N_C; i++)
    {
        for (j = 0; j < 4; j++)
        {
            evce_free_1d((void*)cu_data->nnz_sub[i][j]);
        }
    }
    evce_free_1d((void*)cu_data->map_scu);
#if AFFINE
    evce_free_1d((void*)cu_data->affine_flag);
    evce_free_1d((void*)cu_data->map_affine);
#endif    
    evce_free_1d((void*)cu_data->map_cu_mode);
    evce_free_2d((void**)cu_data->block_size);
    evce_free_1d((void*)cu_data->depth);
    for (i = 0; i < N_C; i++)
    {
        evce_free_1d((void*)cu_data->coef[i]);
        evce_free_1d((void*)cu_data->reco[i]);
    }

    return EVC_OK;
}

#if ALF
void codeAlfCtuEnableFlag(EVC_BSW *bs, EVCE_CTX * ctx, int refId, int compIdx)
{
// TO DO
}
#endif
