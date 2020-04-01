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

#include <math.h>
#include "evce_def.h"
#include "evce_eco.h"
#include "evc_df.h"
#include "evce_mode.h"
#include "evc_util.h"
#include "enc_alf_wrapper.h"
int last_intra_poc = INT_MAX;
BOOL aps_counter_reset = FALSE;
#include "evce_ibc_hash_wrapper.h"

#if GRAB_STAT
#include "evc_debug.h"
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

static const s8 tbl_poc_gop_offset[5][15] =
{
    { -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { -4,   -6,   -7,   -5,   -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { -8,   -12, -14,  -15,  -13,  -10,  -11,   -9,   -4,   -6,   -7,   -5,   -2,   -3,   -1}   /* gop_size = 16 */
};

#if LD_CONFIG_CHANGE
static const s8 tbl_slice_depth_P_orig[GOP_P] = { FRM_DEPTH_4,  FRM_DEPTH_3,  FRM_DEPTH_4,  FRM_DEPTH_2, FRM_DEPTH_4,  FRM_DEPTH_3, FRM_DEPTH_4,  FRM_DEPTH_1 };
#else
static const s8 tbl_slice_depth_P_orig[GOP_P] = { FRM_DEPTH_3,  FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_1 };
#endif

static const s8 tbl_slice_depth_P[5][16] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, FRM_DEPTH_1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 4 */
    { FRM_DEPTH_3, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_1, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 8 */
    { FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_2, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_1,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 16 */
    { FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_2, \
      FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_1 }
};

static const s8 tbl_slice_depth[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 4 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 8 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 16 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_5, \
      FRM_DEPTH_5,  FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_5 }
};

static const s8 tbl_slice_depth_orig[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 4 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 8 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4,\
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    /* gop_size = 16 */
    { FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, \
      FRM_DEPTH_3,  FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5 }
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

void evce_copy_chroma_qp_mapping_params(EVC_CHROMA_TABLE *dst, EVC_CHROMA_TABLE *src)
{
    dst->chroma_qp_table_present_flag = src->chroma_qp_table_present_flag;
    dst->same_qp_table_for_chroma = src->same_qp_table_for_chroma;
    dst->global_offset_flag = src->global_offset_flag;
    dst->num_points_in_qp_table_minus1[0] = src->num_points_in_qp_table_minus1[0];
    dst->num_points_in_qp_table_minus1[1] = src->num_points_in_qp_table_minus1[1];
    memcpy(&(dst->delta_qp_in_val_minus1), &(src->delta_qp_in_val_minus1), sizeof(int) * 2 * MAX_QP_TABLE_SIZE);
    memcpy(&(dst->delta_qp_out_val), &(src->delta_qp_out_val), sizeof(int) * 2 * MAX_QP_TABLE_SIZE);
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

    if (cdsc->ref_pic_gap_length != 0)
    {
        evc_assert_rv(cdsc->max_b_frames == 0, EVC_ERR_INVALID_ARGUMENT);
    }


    if (cdsc->max_b_frames == 0)
    {
        if (cdsc->ref_pic_gap_length == 0)
        {
            cdsc->ref_pic_gap_length = 1;
        }
        evc_assert_rv(cdsc->ref_pic_gap_length == 1 || cdsc->ref_pic_gap_length == 2 || \
                      cdsc->ref_pic_gap_length == 4 || cdsc->ref_pic_gap_length == 8 || \
                      cdsc->ref_pic_gap_length == 16, EVC_ERR_INVALID_ARGUMENT);
    }

    /* set default encoding parameter */
    param->w              = cdsc->w;
    param->h              = cdsc->h;
    param->bit_depth      = cdsc->out_bit_depth;
    param->qp             = cdsc->qp;
    param->fps            = cdsc->fps;
    param->i_period       = cdsc->iperiod;
    param->f_ifrm         = 0;
#if ENC_DBF_CONTROL
    param->use_deblock    = cdsc->use_deblock;
#else
    param->use_deblock    = 1;  
#endif
    param->deblock_alpha_offset = cdsc->deblock_aplha_offset;
    param->deblock_beta_offset = cdsc->deblock_beta_offset;
    param->qp_max         = MAX_QUANT;
    param->qp_min         = MIN_QUANT;
    param->use_pic_sign   = 0;
    param->max_b_frames   = cdsc->max_b_frames;
    param->ref_pic_gap_length = cdsc->ref_pic_gap_length;
    param->gop_size       = param->max_b_frames +1;
    param->use_closed_gop = (cdsc->closed_gop)? 1: 0;
    param->use_ibc_flag = (cdsc->ibc_flag) ? 1 : 0;
    param->ibc_search_range_x = cdsc->ibc_search_range_x;
    param->ibc_search_range_y = cdsc->ibc_search_range_y;
    param->ibc_hash_search_flag = cdsc->ibc_hash_search_flag;
    param->ibc_hash_search_max_cand = cdsc->ibc_hash_search_max_cand;
    param->ibc_hash_search_range_4smallblk = cdsc->ibc_hash_search_range_4smallblk;
    param->ibc_fast_method = cdsc->ibc_fast_method;
    param->use_hgop       = (cdsc->disable_hgop)? 0: 1;
    param->qp_incread_frame = cdsc->add_qp_frame;
#if DQP_CFG
    param->use_dqp = cdsc->use_dqp;
    param->cu_qp_delta_area = cdsc->cu_qp_delta_area;
#endif

    if(cdsc->tool_iqt == 0)
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_base;
    }
    else
    {
        evc_tbl_qp_chroma_ajudst = evc_tbl_qp_chroma_ajudst_main;
    }

    if (cdsc->chroma_qp_table_struct.chroma_qp_table_present_flag)
    {
        evc_derived_chroma_qp_mapping_tables(&(cdsc->chroma_qp_table_struct));
    }
    else 
    {
        memcpy(&(evc_tbl_qp_chroma_dynamic_ext[0][6 * (BIT_DEPTH - 8)]), evc_tbl_qp_chroma_ajudst, MAX_QP_TABLE_SIZE * sizeof(int));
        memcpy(&(evc_tbl_qp_chroma_dynamic_ext[1][6 * (BIT_DEPTH - 8)]), evc_tbl_qp_chroma_ajudst, MAX_QP_TABLE_SIZE * sizeof(int));
    }

#if EVC_TILE_SUPPORT    
    param->tile_columns = cdsc->tile_columns;
    param->tile_rows = cdsc->tile_rows;
    param->uniform_spacing_tiles = cdsc->tile_uniform_spacing_flag; //To be udpated when non-uniform tiles is implemeneted
    param->num_slice_in_pic = cdsc->num_slice_in_pic;
    param->arbitrary_slice_flag = cdsc->arbitrary_slice_flag;
    param->num_remaining_tiles_in_slice_minus1 = cdsc->num_remaining_tiles_in_slice_minus1;
    if (param->arbitrary_slice_flag)
    {
        for (u32 i = 0; i < (param->num_slice_in_pic *(param->num_remaining_tiles_in_slice_minus1 + 2)); i++)
        {
            param->slice_boundary_array[i] = cdsc->slice_boundary_array[i];
        }
    }
    else
    {
        for (int i = 0; i < (2 * param->num_slice_in_pic); i++)
        {
            param->slice_boundary_array[i] = cdsc->slice_boundary_array[i];
        }
    }
#endif    
    return EVC_OK;
}

static int set_enc_param(EVCE_CTX * ctx, EVCE_PARAM * param)
{
    int ret = EVC_OK;
    ctx->qp = (u8)param->qp;
    return ret;
}

#if FIX_TEMPORAL_ID_SET
static void set_nalu(EVC_NALU * nalu, int nalu_type, int nuh_temporal_id)
#else
static void set_nalu(EVC_NALU * nalu, int nalu_type)
#endif
{
    nalu->nal_unit_size = 0;
    nalu->forbidden_zero_bit = 0;
    nalu->nal_unit_type_plus1 = nalu_type + 1;
#if FIX_TEMPORAL_ID_SET
    nalu->nuh_temporal_id = nuh_temporal_id;
#endif
    nalu->nuh_reserved_zero_5bits = 0;
    nalu->nuh_extension_flag = 0;
}
#if EVC_VUI_FIX
// Dummy VUI initialization 
static void set_vui(EVCE_CTX * ctx, EVC_VUI * vui) 
{
    vui->aspect_ratio_info_present_flag = 1;
    vui->aspect_ratio_idc = 1;
    vui->sar_width = 1;
    vui->sar_height = 1;
    vui->overscan_info_present_flag = 1; 
    vui->overscan_appropriate_flag = 1;
    vui->video_signal_type_present_flag = 1;
    vui->video_format = 1;
    vui->video_full_range_flag = 1;
    vui->colour_description_present_flag = 1;
    vui->colour_primaries = 1;
    vui->transfer_characteristics = 1;
    vui->matrix_coefficients = 1;
    vui->chroma_loc_info_present_flag = 1;
    vui->chroma_sample_loc_type_top_field = 1;
    vui->chroma_sample_loc_type_bottom_field = 1;
    vui->neutral_chroma_indication_flag = 1;
    vui->timing_info_present_flag = 1;
    vui->num_units_in_tick = 1;
    vui->time_scale = 1;
    vui->fixed_pic_rate_flag = 1;
    vui->nal_hrd_parameters_present_flag = 1;
    vui->vcl_hrd_parameters_present_flag = 1;
    vui->low_delay_hrd_flag = 1;
    vui->pic_struct_present_flag = 1;
    vui->bitstream_restriction_flag = 1;
    vui->motion_vectors_over_pic_boundaries_flag = 1;
    vui->max_bytes_per_pic_denom = 1;
    vui->max_bits_per_mb_denom = 1;
    vui->log2_max_mv_length_horizontal = 1;
    vui->log2_max_mv_length_vertical = 1;
    vui->num_reorder_pics = 1;
    vui->max_dec_pic_buffering = 1;
    vui->hrd_parameters.cpb_cnt_minus1 = 1;
    vui->hrd_parameters.bit_rate_scale = 1;
    vui->hrd_parameters.cpb_size_scale = 1;
    memset(&(vui->hrd_parameters.bit_rate_value_minus1), 0, sizeof(int)*NUM_CPB);
    memset(&(vui->hrd_parameters.cpb_size_value_minus1), 0, sizeof(int)*NUM_CPB);
    memset(&(vui->hrd_parameters.cbr_flag), 0, sizeof(int)*NUM_CPB);
    vui->hrd_parameters.initial_cpb_removal_delay_length_minus1 = 1;
    vui->hrd_parameters.cpb_removal_delay_length_minus1 = 1;
    vui->hrd_parameters.dpb_output_delay_length_minus1 = 1;
    vui->hrd_parameters.time_offset_length = 1;
}
#endif

static void set_sps(EVCE_CTX * ctx, EVC_SPS * sps)
{
    sps->profile_idc = ctx->cdsc.profile;
    sps->level_idc = ctx->cdsc.level;
    sps->pic_width_in_luma_samples = ctx->param.w;
    sps->pic_height_in_luma_samples = ctx->param.h;
    sps->toolset_idc_h = 0x7FFFF;
    sps->toolset_idc_l = 0;
    sps->bit_depth_luma_minus8 = ctx->cdsc.out_bit_depth - 8;
    sps->bit_depth_chroma_minus8 = ctx->cdsc.out_bit_depth - 8;
    sps->chroma_format_idc = 1; // YCbCr 4:2:0
    sps->ibc_flag = (ctx->param.use_ibc_flag) ? 1 : 0;
    sps->ibc_log_max_size = IBC_MAX_CU_LOG2;
    sps->log2_max_pic_order_cnt_lsb_minus4 = POC_LSB_BIT - 4;
    sps->max_dec_pic_buffering_minus1 = 0; //[TBF]

    if(ctx->param.max_b_frames > 0)
    {
        sps->max_num_ref_pics = MAX_NUM_ACTIVE_REF_FRAME_B;
    }
    else
    {
        sps->max_num_ref_pics = MAX_NUM_ACTIVE_REF_FRAME_LDB;
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
#if M52166_PARTITION
        sps->log2_min_cb_size_minus2 = ctx->cdsc.framework_cb_min - 2;
        sps->log2_diff_ctu_max_14_cb_size = min(ctx->log2_max_cuwh - ctx->cdsc.framework_cu14_max, 6);
        sps->log2_diff_ctu_max_tt_cb_size = min(ctx->log2_max_cuwh - ctx->cdsc.framework_tris_max, 6);
        sps->log2_diff_min_cb_min_tt_cb_size_minus2 = ctx->cdsc.framework_tris_min - ctx->cdsc.framework_cb_min - 2;
#else
        sps->log2_diff_ctu_max_11_cb_size = ctx->log2_max_cuwh - ctx->cdsc.framework_cu11_max;
        sps->log2_diff_max_11_min_11_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_cu11_min;
        sps->log2_diff_max_11_max_12_cb_size = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_cu12_max;
        sps->log2_diff_min_11_min_12_cb_size_minus1 = ctx->cdsc.framework_cu12_min - ctx->cdsc.framework_cu11_min - 1;
        sps->log2_diff_max_12_max_14_cb_size_minus1 = ctx->cdsc.framework_cu12_max - ctx->cdsc.framework_cu14_max - 1;
        sps->log2_diff_min_12_min_14_cb_size_minus1 = ctx->cdsc.framework_cu14_min - ctx->cdsc.framework_cu12_min - 1;
        sps->log2_diff_max_11_max_tt_cb_size_minus1 = ctx->cdsc.framework_cu11_max - ctx->cdsc.framework_tris_max - 1;
        sps->log2_diff_min_11_min_tt_cb_size_minus2 = ctx->cdsc.framework_tris_min - ctx->cdsc.framework_cu11_min - 2;
#endif
#if M52166_SUCO
        sps->log2_diff_ctu_size_max_suco_cb_size = ctx->log2_max_cuwh - min(ctx->cdsc.framework_suco_max, min(6, ctx->log2_max_cuwh));
        sps->log2_diff_max_suco_min_suco_cb_size = max(ctx->log2_max_cuwh - sps->log2_diff_ctu_size_max_suco_cb_size - max(ctx->cdsc.framework_suco_min, max(4, ctx->cdsc.framework_cb_min)), 0);
#else
        sps->log2_diff_ctu_size_max_suco_cb_size = ctx->log2_max_cuwh - ctx->cdsc.framework_suco_max;
        sps->log2_diff_max_suco_min_suco_cb_size = ctx->cdsc.framework_suco_max - ctx->cdsc.framework_suco_min;
#endif
    }

    sps->tool_amvr = ctx->cdsc.tool_amvr;
    sps->tool_mmvd = ctx->cdsc.tool_mmvd;
    sps->tool_affine = ctx->cdsc.tool_affine;
    sps->tool_dmvr = ctx->cdsc.tool_dmvr;
#if ADDB_FLAG_FIX
    sps->tool_addb = ctx->cdsc.tool_addb;
#endif
#if M52291_HDR_DRA
    sps->tool_dra = ctx->cdsc.tool_dra;
#endif
    sps->tool_alf = ctx->cdsc.tool_alf;
    sps->tool_htdf = ctx->cdsc.tool_htdf;
    sps->tool_admvp = ctx->cdsc.tool_admvp;
    sps->tool_eipd = ctx->cdsc.tool_eipd;
    sps->tool_iqt = ctx->cdsc.tool_iqt;
    sps->tool_adcc = ctx->cdsc.tool_adcc;
    sps->tool_cm_init = ctx->cdsc.tool_cm_init;
    sps->tool_ats = ctx->cdsc.tool_ats;
    sps->tool_rpl = ctx->cdsc.tool_rpl;
    sps->tool_pocs = ctx->cdsc.tool_pocs;

    if(sps->profile_idc == PROFILE_MAIN)
    {
#if M52166_PARTITION
        sps->log2_ctu_size_minus5 = ctx->log2_max_cuwh - 5;
#else
        sps->log2_ctu_size_minus2 = ctx->log2_max_cuwh - 2;
#endif
    }
    else
    {
        sps->log2_sub_gop_length = (int)(log2(ctx->param.gop_size) + .5);
        ctx->ref_pic_gap_length = ctx->param.ref_pic_gap_length;
        sps->log2_ref_pic_gap_length = (int)(log2(ctx->param.ref_pic_gap_length) + .5);
    }

    sps->long_term_ref_pics_flag = 0;

    if (!sps->tool_rpl)
    {
        sps->num_ref_pic_lists_in_sps0 = 0;
        sps->num_ref_pic_lists_in_sps1 = 0;
        sps->rpl1_same_as_rpl0_flag = 0;
    }
    else
    {
        sps->num_ref_pic_lists_in_sps0 = ctx->cdsc.rpls_l0_cfg_num;
        sps->num_ref_pic_lists_in_sps1 = ctx->cdsc.rpls_l1_cfg_num;
        sps->rpl1_same_as_rpl0_flag = 0;
    }

    memcpy(sps->rpls_l0, ctx->cdsc.rpls_l0, ctx->cdsc.rpls_l0_cfg_num * sizeof(sps->rpls_l0[0]));
    memcpy(sps->rpls_l1, ctx->cdsc.rpls_l1, ctx->cdsc.rpls_l1_cfg_num * sizeof(sps->rpls_l1[0]));

    sps->vui_parameters_present_flag = 0;
#if EVC_VUI_FIX
    set_vui(ctx, &(sps->vui_parameters));
#endif

#if DQP
    sps->dquant_flag = ctx->cdsc.profile == 0 ? 0 : 1;                 /*Baseline : Active SPSs shall have sps_dquant_flag equal to 0 only*/
#endif

    if (ctx->cdsc.chroma_qp_table_struct.chroma_qp_table_present_flag)
    {
        evce_copy_chroma_qp_mapping_params(&(sps->chroma_qp_table_struct), &(ctx->cdsc.chroma_qp_table_struct));
    }
}

static void set_pps(EVCE_CTX * ctx, EVC_PPS * pps)
{
#if EVC_TILE_SUPPORT
    int tile_columns, tile_rows, num_tiles;
#endif
    pps->single_tile_in_pic_flag = 1;
    pps->constrained_intra_pred_flag = ctx->cdsc.constrained_intra_pred;
#if DQP
    pps->cu_qp_delta_enabled_flag = EVC_ABS(ctx->cdsc.use_dqp);
    pps->cu_qp_delta_area         = ctx->cdsc.cu_qp_delta_area;
#endif
#if EVC_TILE_SUPPORT
    tile_rows = ctx->cdsc.tile_rows;
    tile_columns = ctx->cdsc.tile_columns;

    if (tile_rows > 1 || tile_columns > 1)
    {
        pps->single_tile_in_pic_flag = 0;
    }
    pps->num_tile_rows_minus1 = tile_rows - 1;
    pps->num_tile_columns_minus1 = tile_columns - 1;
    pps->uniform_tile_spacing_flag = ctx->cdsc.tile_uniform_spacing_flag;
    pps->tile_offset_lens_minus1 = 31;
    pps->arbitrary_slice_present_flag = ctx->cdsc.arbitrary_slice_flag;
    num_tiles = tile_rows * tile_columns;
    pps->tile_id_len_minus1 = 0;
    while (num_tiles > (1 << pps->tile_id_len_minus1))
    {
        pps->tile_id_len_minus1++; //Ceil(log2(MAX_NUM_TILES_ROW * MAX_NUM_TILES_COLUMN)) - 1
    }

    if (!pps->uniform_tile_spacing_flag)
    {
        pps->tile_column_width_minus1[pps->num_tile_columns_minus1] = ctx->w_lcu - 1;
        pps->tile_row_height_minus1[pps->num_tile_rows_minus1] = ctx->h_lcu - 1;

        for (int i = 0; i < pps->num_tile_columns_minus1; i++)
        {
            pps->tile_column_width_minus1[i] = ctx->cdsc.tile_column_width_array[i] - 1;
            pps->tile_column_width_minus1[pps->num_tile_columns_minus1] -= (pps->tile_column_width_minus1[i] + 1);

        }
        for (int i = 0; i < pps->num_tile_rows_minus1; i++)
        {
            pps->tile_row_height_minus1[i] = ctx->cdsc.tile_row_height_array[i] - 1;
            pps->tile_row_height_minus1[pps->num_tile_rows_minus1] -= (pps->tile_row_height_minus1[i] + 1);
        }
    }

    pps->loop_filter_across_tiles_enabled_flag = 0;

#if M52291_HDR_DRA
    if (ctx->sps.tool_dra)
    {
        EVC_APS_GEN                *p_aps = ctx->aps_gen_array[0];
        if ((p_aps + 1)->signal_flag == 1)
        {
            assert(((p_aps + 1)->aps_id) < 31 && ((p_aps + 1)->aps_id > -1));
            pps->pic_dra_enabled_present_flag = 1;
            pps->pic_dra_enabled_flag = 1;
            pps->pic_dra_aps_id = (p_aps + 1)->aps_id;
        }
    }
    else
    {
        pps->pic_dra_enabled_present_flag = 0;
        pps->pic_dra_enabled_flag = 0;
    }
#endif
#endif
}

#if !M52291_HDR_DRA
static void set_aps(EVCE_CTX * ctx, EVC_APS * aps)
{
}
#endif

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

#if LD_CONFIG_CHANGE
QP_ADAPT_PARAM qp_adapt_param_ld[8] =
{
    {-1,  0.0000, 0.0000 },
    { 1,  0.0000, 0.0000 },
    { 4, -6.5000, 0.2590 },
    { 4, -6.5000, 0.2590 },
    { 5, -6.5000, 0.2590 },
    { 5, -6.5000, 0.2590 },
    { 5, -6.5000, 0.2590 },
    { 5, -6.5000, 0.2590 },
};
#else
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
#endif

QP_ADAPT_PARAM qp_adapt_param_ai[8] =
{
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
    { 0,  0.0000, 0.0000},
};

  //Implementation for selecting and assigning RPL0 & RPL1 candidates in the SPS to SH
static void select_assign_rpl_for_sh(EVCE_CTX *ctx, EVC_SH *sh)
{
    //TBD: when NALU types are implemented; if the current picture is an IDR, simply return without doing the rest of the codes for this function

#if LD_CONFIG_CHANGE
    /* introduce this variable for LD reason. The predefined RPL in the cfg file is made assuming GOP size is 4 for LD configuration*/
    int gopSize = (ctx->param.gop_size == 1) ? GOP_P : ctx->param.gop_size;
#else
    /* introduce this variable for LD reason. The predefined RPL in the cfg file is made assuming GOP size is 4 for LD configuration*/
    int gopSize = (ctx->param.gop_size == 1) ? 4 : ctx->param.gop_size;
#endif

    //Assume it the pic is in the normal GOP first. Normal GOP here means it is not the first (few) GOP in the beginning of the bitstream
    sh->rpl_l0_idx = sh->rpl_l1_idx = -1;
    sh->ref_pic_list_sps_flag[0] = sh->ref_pic_list_sps_flag[1] = 0;

    int availableRPLs = (ctx->cdsc.rpls_l0_cfg_num < gopSize) ? ctx->cdsc.rpls_l0_cfg_num : gopSize;
    for (int i = 0; i < availableRPLs; i++)
    {
        int pocIdx = (ctx->poc.poc_val % gopSize == 0) ? gopSize : ctx->poc.poc_val % gopSize;
        if (pocIdx == ctx->cdsc.rpls_l0[i].poc)
        {
            sh->rpl_l0_idx = i;
            sh->rpl_l1_idx = sh->rpl_l0_idx;
            break;
        }
    }

    //For special case when the pic is in the first (few) GOP in the beginning of the bitstream.
    if(ctx->param.gop_size == 1)                          //For low delay configuration
    {
        if (ctx->poc.poc_val <= (ctx->cdsc.rpls_l0_cfg_num - gopSize))
        {
            sh->rpl_l0_idx = ctx->poc.poc_val + gopSize - 1;
            sh->rpl_l1_idx = sh->rpl_l0_idx;
        }
    }
    else                                                 //For random access configuration
    {
        //for (int i = ctx->param.gop_size; i < ctx->cdsc.rpls_l0_cfg_num; i++)
        for (int i = gopSize; i < ctx->cdsc.rpls_l0_cfg_num; i++)
        {
            int pocIdx = ctx->param.i_period == 0 ? ctx->poc.poc_val : (ctx->poc.poc_val % ctx->param.i_period == 0) ? ctx->param.i_period : ctx->poc.poc_val % ctx->param.i_period;
            if (pocIdx == ctx->cdsc.rpls_l0[i].poc)
            {
                sh->rpl_l0_idx = i;
                sh->rpl_l1_idx = i;
                break;
            }
        }
    }
    if (ctx->slice_type != SLICE_I)
    {
        ctx->slice_type = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].pic_type == 'P' ? SLICE_P : SLICE_B;
    }
    //Copy RPL0 from the candidate in SPS to this SH
    sh->rpl_l0.poc = ctx->poc.poc_val;
    sh->rpl_l0.tid = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].tid;
    sh->rpl_l0.ref_pic_num = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].ref_pic_num;
    sh->rpl_l0.ref_pic_active_num = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].ref_pic_active_num;
    for (int i = 0; i < sh->rpl_l0.ref_pic_num; i++)
        sh->rpl_l0.ref_pics[i] = ctx->cdsc.rpls_l0[sh->rpl_l0_idx].ref_pics[i];

    //Copy RPL0 from the candidate in SPS to this SH
    sh->rpl_l1.poc = ctx->poc.poc_val;
    sh->rpl_l1.tid = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].tid;
    sh->rpl_l1.ref_pic_num = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].ref_pic_num;
    sh->rpl_l1.ref_pic_active_num = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].ref_pic_active_num;
    for (int i = 0; i < sh->rpl_l1.ref_pic_num; i++)
        sh->rpl_l1.ref_pics[i] = ctx->cdsc.rpls_l1[sh->rpl_l1_idx].ref_pics[i];

    if (sh->rpl_l0_idx != -1)
    {
        sh->ref_pic_list_sps_flag[0] = 1;
    }

    if (sh->rpl_l1_idx != -1)
    {
        sh->ref_pic_list_sps_flag[1] = 1;
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
            if (pm->pic[j] && pm->pic[j]->is_ref && pm->pic[j]->poc == (currentPOC - rpl->ref_pics[i]))
                isExistInDPB = 1;
        }
        if (!isExistInDPB) //Found one ref pic missing return 1
            return 1;
    }
    return 0; 
}

//Return value 0 means no explicit RPL is created. The given input parameters rpl0 and rpl1 are not modified
//Return value 1 means the given input parameters rpl0 and rpl1 are modified
static int create_explicit_rpl(EVC_PM *pm, EVC_SH *sh, int poc_val)
{
    EVC_RPL *rpl0 = &sh->rpl_l0;
    EVC_RPL *rpl1 = &sh->rpl_l1;
    if (!check_refpic_available(poc_val, pm, rpl0) && !check_refpic_available(poc_val, pm, rpl1))
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
            if (pic && pic->is_ref && pic->poc == (poc_val - rpl0->ref_pics[ii]))
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
        sh->rpl_l0_idx = -1;

    //Remove ref pic in RPL1 that is not available in the DPB
    isRPLChanged = 0;
    for (int ii = 0; ii < rpl1->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
            if (pic && pic->is_ref && pic->poc == (poc_val - rpl1->ref_pics[ii]))
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
        sh->rpl_l1_idx = -1;

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

static void set_sh(EVCE_CTX *ctx, EVC_SH *sh)
{
    double qp;
    int qp_l_i;
    int qp_c_i;

    QP_ADAPT_PARAM *qp_adapt_param = ctx->param.max_b_frames == 0 ?
        (ctx->param.i_period == 1 ? qp_adapt_param_ai : qp_adapt_param_ld) : qp_adapt_param_ra;

    if (ctx->sps.tool_rpl)
    {
        sh->poc_lsb = ctx->poc.poc_val & ((1 << (ctx->sps.log2_max_pic_order_cnt_lsb_minus4 + 4))-1);
        select_assign_rpl_for_sh(ctx, sh);
        sh->num_ref_idx_active_override_flag = 1;
    }

    sh->slice_type = ctx->slice_type;
    sh->no_output_of_prior_pics_flag = 0;
    sh->deblocking_filter_on = (ctx->param.use_deblock) ? 1 : 0;
    sh->sh_deblock_alpha_offset = ctx->param.deblock_alpha_offset;
    sh->sh_deblock_beta_offset = ctx->param.deblock_beta_offset;
    sh->single_tile_in_slice_flag = 1;
    sh->collocated_from_list_idx = (sh->slice_type == SLICE_P) ? REFP_0 : REFP_1;  // Specifies source (List ID) of the collocated picture, equialent of the collocated_from_l0_flag
    sh->collocated_from_ref_idx = 0;        // Specifies source (RefID_ of the collocated picture, equialent of the collocated_ref_idx
    sh->collocated_mvp_source_list_idx = REFP_0;  // Specifies source (List ID) in collocated pic that provides MV information (Applicability is function of NoBackwardPredFlag)

    /* set lambda */
    qp = EVC_CLIP3(0, MAX_QUANT, (ctx->param.qp_incread_frame != 0 && (int)(ctx->poc.poc_val) >= ctx->param.qp_incread_frame) ? ctx->qp + 1.0 : ctx->qp);

#if EVC_TILE_SUPPORT
    if (ctx->param.arbitrary_slice_flag == 1)
    {
        ctx->sh.arbitrary_slice_flag = 1;
        sh->num_remaining_tiles_in_slice_minus1 = ctx->param.num_remaining_tiles_in_slice_minus1;
    if (ctx->tile_cnt > 1)
    {
        sh->single_tile_in_slice_flag = 0;
            sh->first_tile_id = ctx->param.slice_boundary_array[ctx->slice_num * (sh->num_remaining_tiles_in_slice_minus1 + 2)];
            for (int i = 0; i < sh->num_remaining_tiles_in_slice_minus1 + 1; ++i)
            {
                sh->delta_tile_id_minus1[i] = ctx->param.slice_boundary_array[ctx->slice_num * (sh->num_remaining_tiles_in_slice_minus1 + 2) + i + 1] -
                    ctx->param.slice_boundary_array[ctx->slice_num * (sh->num_remaining_tiles_in_slice_minus1 + 2) + i];
            }
        }
    }
    else
    {
        if (ctx->tile_cnt > 1)
        {
            sh->single_tile_in_slice_flag = 0;
            sh->first_tile_id = ctx->param.slice_boundary_array[2 * ctx->slice_num];
            sh->last_tile_id = ctx->param.slice_boundary_array[2 * ctx->slice_num + 1];
        }
    }
#endif    
#if DQP
    sh->dqp = EVC_ABS(ctx->param.use_dqp);
#endif
    if(ctx->param.use_hgop)
    {
        double dqp_offset;
        int qp_offset;

        if (ctx->sps.tool_rpl)
        {
            qp += qp_adapt_param[ctx->nalu.nuh_temporal_id].qp_offset_layer;
            dqp_offset = qp * qp_adapt_param[ctx->nalu.nuh_temporal_id].qp_offset_model_scale + qp_adapt_param[ctx->nalu.nuh_temporal_id].qp_offset_model_offset + 0.5;
        }
        else
        {
            qp += qp_adapt_param[ctx->slice_depth].qp_offset_layer;
            dqp_offset = qp * qp_adapt_param[ctx->slice_depth].qp_offset_model_scale + qp_adapt_param[ctx->slice_depth].qp_offset_model_offset + 0.5;
        }

        qp_offset = (int)floor(EVC_CLIP3(0.0, 3.0, dqp_offset));
        qp += qp_offset;
    }

    sh->qp = (u8)EVC_CLIP3(-(6 * (BIT_DEPTH - 8)), MAX_QUANT, qp);
    sh->qp_u = (u8)(sh->qp + ctx->cdsc.cb_qp_offset); 
    sh->qp_v = (u8)(sh->qp + ctx->cdsc.cr_qp_offset);
    sh->sh_deblock_alpha_offset = ctx->cdsc.deblock_aplha_offset;
    sh->sh_deblock_beta_offset = ctx->cdsc.deblock_beta_offset;

    qp_l_i = sh->qp;
    ctx->lambda[0] = 0.57 * pow(2.0, (qp_l_i - 12.0) / 3.0);
    qp_c_i = p_evc_tbl_qp_chroma_dynamic[0][sh->qp_u];
    ctx->dist_chroma_weight[0] = pow(2.0, (qp_l_i - qp_c_i) / 3.0);
    qp_c_i = p_evc_tbl_qp_chroma_dynamic[1][sh->qp_v];
    ctx->dist_chroma_weight[1] = pow(2.0, (qp_l_i - qp_c_i) / 3.0);
    ctx->lambda[1] = ctx->lambda[0] / ctx->dist_chroma_weight[0];
    ctx->lambda[2] = ctx->lambda[0] / ctx->dist_chroma_weight[1];
    ctx->sqrt_lambda[0] = sqrt(ctx->lambda[0]);
    ctx->sqrt_lambda[1] = sqrt(ctx->lambda[1]);
    ctx->sqrt_lambda[2] = sqrt(ctx->lambda[2]);
}
#if EVC_TILE_SUPPORT
static int set_tile_info(EVCE_CTX * ctx, int is_ctx0)
{
    EVC_TILE     * tile;
    int          i, j, size, x, y, w, h, w_tile, h_tile, w_lcu, h_lcu, tidx, t0;
    int          col_w[MAX_NUM_TILES_COL], row_h[MAX_NUM_TILES_ROW], f_tile;
    u8           * map_tidx;
    u8          * tile_to_slice_map = ctx->tile_to_slice_map;
    int          num_slice_in_pic;
    int          first_tile_in_slice, last_tile_in_slice, w_tile_slice, h_tile_slice, first_row_slice, first_col_slice;
    int          slice_num = 0;
    int          tmp1, tmp2;

    ctx->tile_cnt = ctx->param.tile_columns * ctx->param.tile_rows;

    w_tile = ctx->param.tile_columns;
    h_tile = ctx->param.tile_rows;
    f_tile = w_tile * h_tile;
    w_lcu = ctx->w_lcu;
    h_lcu = ctx->h_lcu;
    num_slice_in_pic = ctx->param.num_slice_in_pic;

    for (i = 0; i < (2 * num_slice_in_pic); i = i + 2)
    {
        first_tile_in_slice = ctx->param.slice_boundary_array[i];
        last_tile_in_slice = ctx->param.slice_boundary_array[i + 1];
        first_row_slice = first_tile_in_slice / w_tile;
        first_col_slice = first_tile_in_slice % w_tile;
        w_tile_slice = (last_tile_in_slice % w_tile) - first_col_slice; //Number of tiles in slice width
        h_tile_slice = (last_tile_in_slice / w_tile) - first_row_slice; //Number of tiles in slice height
        tmp1 = 0;
        tmp2 = 0;
        while (tmp1 <= h_tile_slice)
        {
            while (tmp2 <= w_tile_slice)
            {
                tile_to_slice_map[first_tile_in_slice + tmp2 + (first_row_slice + tmp1) *w_tile] = slice_num;
                tmp2++;
            }
            tmp1++;
            tmp2 = 0;
        }
        slice_num++;
    }
#if 0
    /* alloc temporary bitstream buffer for tiles dependent on picture size  */
    size = ctx->f / f_tile; /* !! CHECK-ME LATER !! */
    ctx->bs_tbuf_size = size;
    for (i = 0; i<f_tile; i++)
    {
        ctx->bs_tbuf[i] = evc_malloc(size);
        evc_assert_rv(ctx->bs_tbuf[i], EVC_ERR_OUT_OF_MEMORY);
        evc_mset(ctx->bs_tbuf[i], 0, size);
    }
#endif
    if (is_ctx0)
    {
        /* alloc tile information */
        size = sizeof(EVC_TILE) * f_tile;
        ctx->tile = evc_malloc(size);
        evc_assert_rv(ctx->tile, EVC_ERR_OUT_OF_MEMORY);
        evc_mset(ctx->tile, 0, size);

        /* set tile information */
        if (ctx->param.uniform_spacing_tiles)
        {
            for (i = 0; i<w_tile; i++)
            {
                col_w[i] = ((i + 1) * w_lcu) / w_tile - (i * w_lcu) / w_tile;
                if (col_w[i] < 1 )
                    evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
            }
            for (j = 0; j<h_tile; j++)
            {
                row_h[j] = ((j + 1) * h_lcu) / h_tile - (j * h_lcu) / h_tile;
                if (row_h[j] < 1 )
                    evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
            }
        }
        else
        {
            //Non-uniform tile case

            for (i = 0, t0 = 0; i<(w_tile - 1); i++)
            {
                col_w[i] = ctx->cdsc.tile_column_width_array[i];
                t0 += col_w[i];
                if (col_w[i] < 1 )
                    evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
            }
            col_w[i] = w_lcu - t0;
            if (col_w[i] < 1)
                evc_assert_rv(0, EVC_ERR_UNSUPPORTED);

            for (j = 0, t0 = 0; j<(h_tile - 1); j++)
            {
                row_h[j] = ctx->cdsc.tile_row_height_array[j];
                if (row_h[j] < 1 )
                    evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
                t0 += row_h[j];
            }
            row_h[j] = h_lcu - t0;
            if (row_h[j] < 1)
                evc_assert_rv(0, EVC_ERR_UNSUPPORTED);
        }

        /* update tile information - Tile width, height, First ctb address */
        tidx = 0;
        for (y = 0; y<h_tile; y++)
        {
            for (x = 0; x<w_tile; x++)
            {
                tile = &ctx->tile[tidx];
                tile->w_ctb = col_w[x];
                tile->h_ctb = row_h[y];
                tile->f_ctb = tile->w_ctb * tile->h_ctb;
                tile->ctba_rs_first = 0;

                for (i = 0; i<x; i++)
                {
                    tile->ctba_rs_first += col_w[i];
                }
                for (j = 0; j<y; j++)
                {
                    tile->ctba_rs_first += w_lcu * row_h[j];
                }
                tidx++;
            }
        }

        /* set tile map - SCU level mapping to tile index */
        for (tidx = 0; tidx<(w_tile * h_tile); tidx++)
        {
            slice_num = tile_to_slice_map[tidx];
            tile = ctx->tile + tidx;
            x = PEL2SCU((tile->ctba_rs_first % w_lcu) << ctx->log2_max_cuwh);
            y = PEL2SCU((tile->ctba_rs_first / w_lcu) << ctx->log2_max_cuwh);
            t0 = PEL2SCU(tile->w_ctb << ctx->log2_max_cuwh);
            w = min((ctx->w_scu - x), t0);
            t0 = PEL2SCU(tile->h_ctb << ctx->log2_max_cuwh);
            h = min((ctx->h_scu - y), t0);

            map_tidx = ctx->map_tidx + x + y * ctx->w_scu;
            for (j = 0; j<h; j++)
            {
                for (i = 0; i<w; i++)
                {
                    map_tidx[i] = tidx;
                    MCU_SET_SN(ctx->map_scu[i], slice_num);  //Mapping CUs to the slices
                }
                map_tidx += ctx->w_scu;
            }
        }
    }
    else
    {
        //    ctx->tile = ctx->ctx0->tile;
        //evc_mcpy(ctx->map_tidx, ctx->ctx0->map_tidx, sizeof(uint8) * ctx->f_scu);
    }
    return EVC_OK;
}
#endif

static int evce_eco_tree(EVCE_CTX * ctx, EVCE_CORE * core, int x0, int y0, int cup, int cuw, int cuh, int cud, int next_split, const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth
#if DQP
    , int cu_qp_delta_code
#endif
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int ret;
    
    EVC_BSW * bs;
    s8  split_mode;
    s8  suco_flag = 0;
    int bound;
    int split_mode_child[4] = {NO_SPLIT, NO_SPLIT, NO_SPLIT, NO_SPLIT};
    int split_allow[6];
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
    core->tree_cons = tree_cons;
#else
    ctx->tree_cons = tree_cons;
#endif
#endif

    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[core->lcu_num].split_mode);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[core->lcu_num].suco_flag);

    same_layer_split[node_idx] = split_mode;


    bs = &ctx->bs;


#if DQP
    if(ctx->pps.cu_qp_delta_enabled_flag && ctx->sps.dquant_flag)
    {
        if (split_mode == NO_SPLIT && (CONV_LOG2(cuw) + CONV_LOG2(cuh) >= ctx->pps.cu_qp_delta_area) && cu_qp_delta_code != 2)
        {
            if (CONV_LOG2(cuw) == 7 || CONV_LOG2(cuh) == 7)
            {
                cu_qp_delta_code = 2;
            }
            else
            {
                cu_qp_delta_code = 1;
            }
            core->cu_qp_delta_is_coded = 0;
        }
        else if ((((CONV_LOG2(cuw) + CONV_LOG2(cuh) == ctx->pps.cu_qp_delta_area + 1) && (split_mode == SPLIT_TRI_VER || split_mode == SPLIT_TRI_HOR)) ||
            (CONV_LOG2(cuh) + CONV_LOG2(cuw) == ctx->pps.cu_qp_delta_area && cu_qp_delta_code != 2)))
        {
            cu_qp_delta_code = 2;
            core->cu_qp_delta_is_coded = 0;
        }
    }
#endif

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
#if M50761_CHROMA_NOT_SPLIT
        split_struct.tree_cons = tree_cons;

        BOOL mode_cons_changed = FALSE;

        if ( ctx->sps.sps_btt_flag && ctx->sps.tool_admvp )       // TODO: Tim create the specific variable for local dual tree ON/OFF
        {
            split_struct.tree_cons.changed = tree_cons.mode_cons == eAll && !evc_is_chroma_split_allowed( cuw, cuh, split_mode );

            mode_cons_changed = evc_signal_mode_cons(
                                                    #if EVC_CONCURENCY
                                                                    &core->tree_cons 
                                                    #else
                                                                    &ctx->tree_cons 
                                                    #endif
                                                    ,               &split_struct.tree_cons);

            BOOL mode_cons_signal = mode_cons_changed && (ctx->sh.slice_type != SLICE_I) && (evc_get_mode_cons_by_split(split_mode, cuw, cuh) == eAll);
            if (mode_cons_changed)
            {
                MODE_CONS mode = evce_derive_mode_cons(ctx, core->lcu_num, cup);
                evc_set_tree_mode(&split_struct.tree_cons, mode);
            }

            if (mode_cons_signal)
            {

                evc_get_ctx_some_flags(PEL2SCU(x0), PEL2SCU(y0), cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode 
#if EVC_CONCURENCY
                    , core->ctx_flags
#else
                    , ctx->ctx_flags
#endif
                    
                    , ctx->sh.slice_type, ctx->sps.tool_cm_init
                    , ctx->param.use_ibc_flag, ctx->sps.ibc_log_max_size
#if EVC_TILE_SUPPORT
                    , ctx->map_tidx
#endif
                );
#if EVC_CONCURENCY
                evce_eco_mode_constr(bs, split_struct.tree_cons.mode_cons, core->ctx_flags[CNID_MODE_CONS]);

#else
                evce_eco_mode_constr(bs, split_struct.tree_cons.mode_cons, ctx->ctx_flags[CNID_MODE_CONS]);
#endif
            }
        }
        else
        {
            split_struct.tree_cons = evc_get_default_tree_cons();
        }
#endif


        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                ret = evce_eco_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], sub_cuw, sub_cuh, split_struct.cud[cur_part_num], 1, split_mode, split_mode_child, part_num, split_allow, INC_QT_DEPTH(qt_depth, split_mode), INC_BTT_DEPTH(btt_depth, split_mode, bound)
#if DQP
                    , cu_qp_delta_code
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , split_struct.tree_cons
#endif
                );
                evc_assert_g(EVC_SUCCEEDED(ret), ERR);
            }
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
            core->tree_cons = tree_cons;
#else
            ctx->tree_cons = tree_cons;
#endif
#endif
        }
#if M50761_CHROMA_NOT_SPLIT
        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
        {
            evc_assert(x0 + cuw <= PIC_ORIG(ctx)->w_l && y0 + cuh <= PIC_ORIG(ctx)->h_l);
            TREE_CONS local_tree_cons = split_struct.tree_cons;
            local_tree_cons.tree_type = TREE_C;
            ret = evce_eco_unit(ctx, core, x0, y0, cup, cuw, cuh, local_tree_cons
            );
#if EVC_CONCURENCY
            core->tree_cons = tree_cons;
#else
            ctx->tree_cons = tree_cons;
#endif
        }
#endif
    }
    else
    {
        evc_assert(x0 + cuw <= ctx->w && y0 + cuh <= ctx->h);

        if((cuw > MIN_CU_SIZE || cuh > MIN_CU_SIZE) && next_split
#if M50761_CHROMA_NOT_SPLIT
            && evce_check_luma(ctx 
#if EVC_CONCURENCY
            , core
#endif
            )
#endif
            )
        {
            evce_eco_split_mode(bs, ctx, core, cud, cup, cuw, cuh, ctx->max_cuwh
                                , parent_split, same_layer_split, node_idx, parent_split_allow, split_allow, qt_depth, btt_depth, x0, y0);
        }

#if DQP
        core->cu_qp_delta_code = cu_qp_delta_code;
#endif
        ret = evce_eco_unit(ctx, core, x0, y0, cup, cuw, cuh
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
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

    ctx->map_ats_inter = NULL;


    ctx->ats_inter_info_pred = NULL;
    ctx->ats_inter_pred_dist = NULL;
    ctx->ats_inter_num_pred = NULL;


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
#if M52166_PARTITION
        ctx->max_cuwh = 1 << ctx->cdsc.framework_cb_max;
#else
        ctx->max_cuwh = 1 << ctx->cdsc.framework_ctu_size;
#endif
        if (w < ctx->max_cuwh * 2 && h < ctx->max_cuwh * 2)
        {
            ctx->max_cuwh = ctx->max_cuwh >> 1;
        }
        else
        {
            ctx->max_cuwh = ctx->max_cuwh;
        }
#if M52166_PARTITION
        ctx->min_cuwh = 1 << ctx->cdsc.framework_cb_min;
        ctx->log2_min_cuwh = ctx->cdsc.framework_cb_min;
#endif
    }
    else
    {
        ctx->max_cuwh = 64; 
#if M52166_PARTITION
        ctx->min_cuwh = 1 << 2;
        ctx->log2_min_cuwh = 2;
#endif
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
#if !M52166_PARTITION   
    ctx->cdsc.framework_cu11_max = min(ctx->log2_max_cuwh, ctx->cdsc.framework_cu11_max);
    ctx->cdsc.framework_cu12_max = min(ctx->cdsc.framework_cu11_max, ctx->cdsc.framework_cu12_max);
    ctx->cdsc.framework_cu14_max = min(ctx->cdsc.framework_cu12_max, ctx->cdsc.framework_cu14_max);
#endif
    ctx->cdsc.framework_suco_max = min(ctx->log2_max_cuwh, ctx->cdsc.framework_suco_max);
    ctx->enc_alf = new_enc_ALF();
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);
    call_create_enc_ALF(p, ctx->w, ctx->h, ctx->max_cuwh, ctx->max_cuwh, 5);

    if (ctx->param.use_ibc_flag)
    {
      ctx->ibc_hash_handle = create_enc_IBC(ctx->w, ctx->h);
    }

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

    size = sizeof(s8) * ctx->f_scu;
    ctx->map_depth = evc_malloc_fast(size);
    evc_assert_gv(ctx->map_depth, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
    evc_mset(ctx->map_depth, -1, size);

    if (ctx->map_affine == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_affine = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_affine, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_affine, 0, size);
    }

    if(ctx->map_cu_mode == NULL)
    {
        size = sizeof(u32) * ctx->f_scu;
        ctx->map_cu_mode = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_cu_mode, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_cu_mode, 0, size);
    }

    if (ctx->map_ats_intra_cu == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_intra_cu = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_intra_cu, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_intra_cu, 0, size);
    }
    if (ctx->map_ats_mode_h == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_mode_h = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_mode_h, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_mode_h, 0, size);
    }
    if (ctx->map_ats_mode_v == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_mode_v = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_mode_v, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_mode_v, 0, size);
    }

    if (ctx->map_ats_inter == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_ats_inter = evc_malloc_fast(size);
        evc_assert_gv(ctx->map_ats_inter, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->map_ats_inter, -1, size);
    }

    if (ctx->ats_inter_info_pred == NULL)
    {
        int num_route = ATS_INTER_SL_NUM;
        int num_size_idx = MAX_TR_LOG2 - MIN_CU_LOG2 + 1;
        size = sizeof(u32) * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2) * num_route; //only correct when the largest cu is <=128
        ctx->ats_inter_pred_dist = evc_malloc_fast(size);
        evc_assert_gv(ctx->ats_inter_pred_dist, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

        size = sizeof(u8)  * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2) * num_route;
        ctx->ats_inter_info_pred = evc_malloc_fast(size);
        evc_assert_gv(ctx->ats_inter_info_pred, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

        size = sizeof(u8)  * num_size_idx * num_size_idx * (ctx->max_cuwh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2);
        ctx->ats_inter_num_pred = evc_malloc_fast(size);
        evc_assert_gv(ctx->ats_inter_num_pred, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
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
    ctx->poc.poc_val         = 0;

    ret = evc_picman_init(&ctx->rpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, &ctx->pa);
    evc_assert_g(EVC_SUCCEEDED(ret), ERR);

    ctx->pico_max_cnt = 1 + (ctx->param.max_b_frames << 1) ;
    ctx->frm_rnum = ctx->param.max_b_frames;
    ctx->qp = ctx->param.qp;
    ctx->deblock_alpha_offset = ctx->param.deblock_alpha_offset;
    ctx->deblock_beta_offset = ctx->param.deblock_beta_offset;

    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        ctx->pico_buf[i] = (EVCE_PICO*)evc_malloc(sizeof(EVCE_PICO));
        evc_assert_gv(ctx->pico_buf[i], ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset(ctx->pico_buf[i], 0, sizeof(EVCE_PICO));
    }
#if EVC_TILE_SUPPORT
    /* alloc tile index map in SCU unit */
    if (ctx->map_tidx == NULL)
    {
        size = sizeof(u8) * ctx->f_scu;
        ctx->map_tidx = (u8*)evc_malloc_fast(size);
        evc_assert_gv(ctx->map_tidx, ret, EVC_ERR_OUT_OF_MEMORY, ERR);
        evc_mset_x64a(ctx->map_tidx, 0, size);
    }

    /*curently is_ctx0 is set to 1 by default */
    int is_ctx0 = 1;
    ret = set_tile_info(ctx, is_ctx0);
    if (ret != EVC_OK)
        goto ERR;
#endif

    return EVC_OK;
ERR:
    for (i = 0; i < (int)ctx->f_lcu; i++)
    {
        evce_delete_cu_data(ctx->map_cu_data + i, ctx->log2_max_cuwh - MIN_CU_LOG2, ctx->log2_max_cuwh - MIN_CU_LOG2);
    }
    evc_mfree_fast(ctx->map_cu_data);
    evc_mfree_fast(ctx->map_ipm);
    evc_mfree_fast(ctx->map_depth);
    evc_mfree_fast(ctx->map_affine);
    evc_mfree_fast(ctx->map_ats_intra_cu);
    evc_mfree_fast(ctx->map_ats_mode_h);
    evc_mfree_fast(ctx->map_ats_mode_v);
    evc_mfree_fast(ctx->map_ats_inter);

    evc_mfree_fast(ctx->ats_inter_pred_dist);
    evc_mfree_fast(ctx->ats_inter_info_pred);
    evc_mfree_fast(ctx->ats_inter_num_pred);

    evc_mfree_fast(ctx->map_cu_mode);
#if EVC_TILE_SUPPORT
    evc_mfree_fast(ctx->map_tidx);
#endif
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
    evc_mfree_fast(ctx->map_depth);
    evc_mfree_fast(ctx->map_affine);
    evc_mfree_fast(ctx->map_ats_intra_cu);
    evc_mfree_fast(ctx->map_ats_mode_h);
    evc_mfree_fast(ctx->map_ats_mode_v);
    evc_mfree_fast(ctx->map_ats_inter);
    evc_mfree_fast(ctx->ats_inter_pred_dist);
    evc_mfree_fast(ctx->ats_inter_info_pred);
    evc_mfree_fast(ctx->ats_inter_num_pred);
    evc_mfree_fast(ctx->map_cu_mode);
#if EVC_TILE_SUPPORT
    evc_mfree_fast(ctx->map_tidx);
#endif
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

static void deblock_tree(EVCE_CTX * ctx, EVC_PIC * pic, int x, int y, int cuw, int cuh, int cud, int cup, int is_hor_edge
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
#if EVC_CONCURENCY
    , EVCE_CORE * core
#endif
)
{
    s8  split_mode;
    int lcu_num;
    s8  suco_flag = 0;
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
    core->tree_cons = tree_cons;
#else
    ctx->tree_cons = tree_cons;
#endif
#endif

    pic->pic_deblock_alpha_offset = ctx->sh.sh_deblock_alpha_offset;
    pic->pic_deblock_beta_offset = ctx->sh.sh_deblock_beta_offset;

    lcu_num = (x >> ctx->log2_max_cuwh) + (y >> ctx->log2_max_cuwh) * ctx->w_lcu;
    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[lcu_num].split_mode);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_cu_data[lcu_num].suco_flag);

    if(split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];
        evc_split_get_part_structure( split_mode, x, y, cuw, cuh, cup, cud, ctx->log2_culine, &split_struct );

        evc_split_get_suco_order(suco_flag, split_mode, suco_order);
        
#if M50761_CHROMA_NOT_SPLIT
        split_struct.tree_cons = tree_cons;

        BOOL mode_cons_changed = FALSE;

        if ( ctx->sps.tool_admvp && ctx->sps.sps_btt_flag )       // TODO: Tim create the specific variable for local dual tree ON/OFF
        {
            split_struct.tree_cons.changed = tree_cons.mode_cons == eAll && !evc_is_chroma_split_allowed( cuw, cuh, split_mode );
            mode_cons_changed = evc_signal_mode_cons(
                                                        #if EVC_CONCURENCY
                                                                        &core->tree_cons 
                                                        #else
                                                                        &ctx->tree_cons 
                                                        #endif
                                                                        , &split_struct.tree_cons);

            if (mode_cons_changed)
            {
                MODE_CONS mode = evce_derive_mode_cons(ctx, lcu_num, cup);
                evc_set_tree_mode(&split_struct.tree_cons, mode);
            }
        }
        else
        {
            split_struct.tree_cons = evc_get_default_tree_cons();
        }
#endif
        for(int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->w && y_pos < ctx->h)
            {
                deblock_tree(ctx, pic, x_pos, y_pos, sub_cuw, sub_cuh, split_struct.cud[cur_part_num], split_struct.cup[cur_part_num], is_hor_edge
#if M50761_CHROMA_NOT_SPLIT
                    , split_struct.tree_cons
#endif
#if EVC_CONCURENCY
                    , core
#endif
                );
            }
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
            core->tree_cons = tree_cons;
#else
            ctx->tree_cons = tree_cons;
#endif
#endif
        }
#if M50761_CHROMA_NOT_SPLIT
        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
        {
                        
#if EVC_CONCURENCY
            core->tree_cons = split_struct.tree_cons;
            core->tree_cons.tree_type = TREE_C;
#else
            ctx->tree_cons = split_struct.tree_cons;
            ctx->tree_cons.tree_type = TREE_C;
#endif
            split_mode = NO_SPLIT;
        }
#endif
    }
#if M50761_CHROMA_NOT_SPLIT
    if (split_mode == NO_SPLIT)
#else
    else
#endif
    {
        // deblock
        int t = (x >> MIN_CU_LOG2) + (y >> MIN_CU_LOG2) * ctx->w_scu;
        u8 ats_inter_info = ctx->map_ats_inter[t];
        u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
        u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
        if(is_hor_edge)
        {
            if (cuh > MAX_TR_SIZE)
            {
              
                evc_deblock_cu_hor(pic, x, y, cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, ctx->map_unrefined_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
                  
#if EVC_CONCURENCY
                    , core->tree_cons 
#else
                    , ctx->tree_cons 
#endif
#endif
#if EVC_TILE_SUPPORT
                    , ctx->map_tidx
#endif
#if ADDB_FLAG_FIX
                    , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
                    , ctx->map_ats_inter
#endif
                );
                evc_deblock_cu_hor(pic, x, y + MAX_TR_SIZE, cuw, cuh >> 1, ctx->map_scu, ctx->map_refi, ctx->map_unrefined_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
                    , core->tree_cons
#else
                    , ctx->tree_cons
#endif
#endif
#if EVC_TILE_SUPPORT
                    , ctx->map_tidx
#endif
#if ADDB_FLAG_FIX
                    , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
                    , ctx->map_ats_inter
#endif
                );
            }
            else
            {
                evc_deblock_cu_hor(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_unrefined_mv, ctx->w_scu, ctx->log2_max_cuwh, ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
                    , core->tree_cons
#else
                    , ctx->tree_cons
#endif
#endif
#if EVC_TILE_SUPPORT
                    , ctx->map_tidx
#endif
#if ADDB_FLAG_FIX
                    , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
                    , ctx->map_ats_inter
#endif
                );
            }
        }
        else
        {
            if (cuw > MAX_TR_SIZE)
            {
                evc_deblock_cu_ver(pic, x              , y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, ctx->map_unrefined_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                  , ctx->map_cu_mode
#endif
                  , ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
                  , core->tree_cons
#else
                  , ctx->tree_cons
#endif
#endif
#if EVC_TILE_SUPPORT
                  , ctx->map_tidx
#endif
#if ADDB_FLAG_FIX
                  , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
                  , ctx->map_ats_inter
#endif
                );
                evc_deblock_cu_ver(pic, x + MAX_TR_SIZE, y, cuw >> 1, cuh, ctx->map_scu, ctx->map_refi, ctx->map_unrefined_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                  , ctx->map_cu_mode
#endif                            
                  , ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
                  
#if EVC_CONCURENCY
                  , core->tree_cons 
#else
                  , ctx->tree_cons 
#endif
#endif
#if EVC_TILE_SUPPORT
                  , ctx->map_tidx
#endif
#if ADDB_FLAG_FIX
                  , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
                  , ctx->map_ats_inter
#endif
                );
            }
            else
            {
                evc_deblock_cu_ver(pic, x, y, cuw, cuh, ctx->map_scu, ctx->map_refi, ctx->map_unrefined_mv, ctx->w_scu, ctx->log2_max_cuwh
#if FIX_PARALLEL_DBF
                  , ctx->map_cu_mode
#endif
                  , ctx->refp, 0
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
                  , core->tree_cons
#else
                  , ctx->tree_cons
#endif
#endif
#if EVC_TILE_SUPPORT
                  , ctx->map_tidx
#endif
#if ADDB_FLAG_FIX
                  , ctx->sps.tool_addb
#endif
#if DEBLOCKING_FIX
                  , ctx->map_ats_inter
#endif
                );
            }
        }
    }
#if M50761_CHROMA_NOT_SPLIT
#if EVC_CONCURENCY
    core->tree_cons = tree_cons;
#else
    ctx->tree_cons = tree_cons;
#endif
#endif
}

int evce_deblock_h263(EVCE_CTX * ctx, EVC_PIC * pic
#if EVC_TILE_SUPPORT
    , int tile_idx
#endif
#if EVC_CONCURENCY
    , EVCE_CORE * core
#endif
)
{
    int i, j;
#if EVC_TILE_SUPPORT
    int x_l, x_r, y_l, y_r, l_scu, r_scu, t_scu, b_scu;
    u32 k1;
    int scu_in_lcu_wh = 1 << (ctx->log2_max_cuwh - MIN_CU_LOG2);
    
    x_l = (ctx->tile[tile_idx].ctba_rs_first) % ctx->w_lcu; //entry point lcu's x location
    y_l = (ctx->tile[tile_idx].ctba_rs_first) / ctx->w_lcu; // entry point lcu's y location
    x_r = x_l + ctx->tile[tile_idx].w_ctb;
    y_r = y_l + ctx->tile[tile_idx].h_ctb;
    l_scu = x_l * scu_in_lcu_wh;
    r_scu = EVC_CLIP3(0, ctx->w_scu, x_r*scu_in_lcu_wh);
    t_scu = y_l * scu_in_lcu_wh;
    b_scu = EVC_CLIP3(0, ctx->h_scu, y_r*scu_in_lcu_wh);

    for (j = t_scu; j < b_scu; j++)
    {
        for (i = l_scu; i < r_scu; i++)
        {
            k1 = i + j * ctx->w_scu;
            MCU_CLR_COD(ctx->map_scu[k1]);

            if (!MCU_GET_DMVRF(ctx->map_scu[k1]))
            {
                ctx->map_unrefined_mv[k1][REFP_0][MV_X] = ctx->map_mv[k1][REFP_0][MV_X];
                ctx->map_unrefined_mv[k1][REFP_0][MV_Y] = ctx->map_mv[k1][REFP_0][MV_Y];
                ctx->map_unrefined_mv[k1][REFP_1][MV_X] = ctx->map_mv[k1][REFP_1][MV_X];
                ctx->map_unrefined_mv[k1][REFP_1][MV_Y] = ctx->map_mv[k1][REFP_1][MV_Y];
            }
        }
    }

    /* horizontal filtering */
    for (j = y_l; j < y_r; j++)
    {
        for (i = x_l; i < x_r; i++)
        {
#if DB_SPEC_ALIGNMENT1
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0/*0 - horizontal filtering of vertical edge*/
#else
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1
#endif
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
#if EVC_CONCURENCY
                , core
#endif
            );
        }
    }

    for (j = t_scu; j < b_scu; j++)
    {
        for (i = l_scu; i < r_scu; i++)
        {
            MCU_CLR_COD(ctx->map_scu[i + j * ctx->w_scu]);
        }
    }

    /* vertical filtering */
    for (j = y_l; j < y_r; j++)
    {
        for (i = x_l; i < x_r; i++)
        {
#if DB_SPEC_ALIGNMENT1
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1/*1 - vertical filtering of horizontal edge*/
#else
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
#if EVC_CONCURENCY
                , core
#endif
            );
        }
    }
#else
    u32 k;
    for(k = 0; k < ctx->f_scu; k++)
    {
        MCU_CLR_COD(ctx->map_scu[k]);

        if (!MCU_GET_DMVRF(ctx->map_scu[k])) {
          ctx->map_unrefined_mv[k][REFP_0][MV_X] = ctx->map_mv[k][REFP_0][MV_X];
          ctx->map_unrefined_mv[k][REFP_0][MV_Y] = ctx->map_mv[k][REFP_0][MV_Y];
          ctx->map_unrefined_mv[k][REFP_1][MV_X] = ctx->map_mv[k][REFP_1][MV_X];
          ctx->map_unrefined_mv[k][REFP_1][MV_Y] = ctx->map_mv[k][REFP_1][MV_Y];
        }
    }

    /* horizontal filtering */
    for(j = 0; j < ctx->h_lcu; j++)
    {
        for(i = 0; i < ctx->w_lcu; i++)
        {
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 1
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
#if EVC_CONCURENCY
                , core
#endif
            );
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
            deblock_tree(ctx, pic, (i << ctx->log2_max_cuwh), (j << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 0
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
#if EVC_CONCURENCY
                , core
#endif
            );
        }
    }
#endif
    return EVC_OK;
}

int evce_alf_aps(EVCE_CTX * ctx, EVC_PIC * pic, EVC_SH* sh, EVC_APS* aps)
{
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);

    double lambdas[3];
    for (int i = 0; i < 3; i++)
        lambdas[i] = (ctx->lambda[i]) * ALF_LAMBDA_SCALE; //this is for appr match of different lambda sets


    set_resetALFBufferFlag(p, sh->slice_type == SLICE_I ? 1 : 0);
    alf_aps_enc_opt_process(p, lambdas, ctx, pic, &(sh->alf_sh_param));

    aps->alf_aps_param = sh->alf_sh_param;
    if (sh->alf_sh_param.resetALFBufferFlag) // reset aps index counter (buffer) if ALF flag reset is present
    {
        ctx->aps_counter = -1;
    }
    sh->alf_on = sh->alf_sh_param.enabledFlag[0];
    if (sh->alf_on == 0)
    {
        sh->alf_sh_param.isCtbAlfOn = 0;
    }
    if (sh->alf_on)
    {
        if (aps->alf_aps_param.temporalAlfFlag)
        {
            aps->aps_id = sh->alf_sh_param.prevIdx;
            sh->aps_id_y = sh->alf_sh_param.prevIdxComp[0];
            sh->aps_id_ch = sh->alf_sh_param.prevIdxComp[1];
            sh->aps_signaled = aps->aps_id;
        }
        else
        {
            aps->aps_id = alf_aps_get_current_alf_idx();
            sh->aps_id_y = aps->aps_id;
            sh->aps_id_ch = aps->aps_id;
            sh->aps_signaled = aps->aps_id;
        }
    }
    return EVC_OK;
}

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

#if M52291_HDR_DRA
int evce_aps_header(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat, EVC_APS_GEN * aps)
#else
int evce_aps_header(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat, EVC_APS * aps)
#endif
{
    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* encode sequence parameter set */
    /* skip first four byte to write the bitstream size */
    evce_bsw_skip_slice_size(bs);

    /* Encode APS nalu header */
    EVC_NALU aps_nalu;
#if FIX_TEMPORAL_ID_SET
    set_nalu(&aps_nalu, EVC_APS_NUT, ctx->nalu.nuh_temporal_id);
#else
    set_nalu(&aps_nalu, EVC_APS_NUT);
#endif

    /* Write APS */
#if M52291_HDR_DRA
    evc_assert_rv(evce_eco_aps_gen(bs, aps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);
#else
    set_aps(ctx, aps); // TBD: empty function call
    evc_assert_rv(evce_eco_aps(bs, aps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);
#endif

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_APS_NUT;

    return EVC_OK;
}

int evce_enc_header(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;
    EVC_NALU  nalu;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* encode sequence parameter set */
    /* skip first four byte to write the bitstream size */
    evce_bsw_skip_slice_size(bs);

    /* nalu header */
#if FIX_TEMPORAL_ID_SET
    set_nalu(&nalu, EVC_SPS_NUT, 0);
#else
    set_nalu(&nalu, EVC_SPS_NUT);
#endif
    evce_eco_nalu(bs, nalu);

    /* sequence parameter set*/
    set_sps(ctx, sps);
    evc_assert_rv(evce_eco_sps(bs, sps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* picture parameter set*/
    set_pps(ctx, pps);
    evc_assert_rv(evce_eco_pps(bs, sps, pps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_SPS_NUT;

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
        ctx->slice_type = SLICE_I;
        ctx->slice_depth = FRM_DEPTH_0;
        ctx->poc.poc_val = pic_imcnt;
        ctx->poc.prev_doc_offset = 0;
        ctx->poc.prev_poc_val = ctx->poc.poc_val;
        ctx->slice_ref_flag = 1;
    }
    else if((i_period != 0) && pic_imcnt % i_period == 0)
    {
        ctx->slice_type = SLICE_I;
        ctx->slice_depth = FRM_DEPTH_0;
        ctx->poc.poc_val = pic_imcnt;
        ctx->poc.prev_doc_offset = 0;
        ctx->poc.prev_poc_val = ctx->poc.poc_val;
        ctx->slice_ref_flag = 1;
    }
    else if(pic_imcnt % gop_size == 0)
    {
        ctx->slice_type = ctx->cdsc.inter_slice_type;
        ctx->slice_ref_flag = 1;
        ctx->slice_depth = FRM_DEPTH_1;
        ctx->poc.poc_val = pic_imcnt;
        ctx->poc.prev_doc_offset = 0;
        ctx->poc.prev_poc_val = ctx->poc.poc_val;
        ctx->slice_ref_flag = 1;
    }
    else
    {
        ctx->slice_type = ctx->cdsc.inter_slice_type;
        if(ctx->param.use_hgop)
        {
            pos = (pic_imcnt % gop_size) - 1;

            if (ctx->sps.tool_pocs)
            {
                ctx->slice_depth = tbl_slice_depth_orig[gop_size >> 2][pos];
                ctx->poc.poc_val = ((pic_imcnt / gop_size) * gop_size) +
                    tbl_poc_gop_offset[gop_size >> 2][pos];
            }
            else
            {
                ctx->slice_depth = tbl_slice_depth[gop_size >> 2][pos];
                int tid = ctx->slice_depth - (ctx->slice_depth > 0);
                evc_poc_derivation(ctx->sps, tid, &ctx->poc);
                ctx->poc.poc_val = ctx->poc.poc_val;
            }
            if (!ctx->sps.tool_pocs && gop_size >= 2)
            {
                ctx->slice_ref_flag = (ctx->slice_depth == tbl_slice_depth[gop_size >> 2][gop_size - 2] ? 0 : 1);
            }
            else
            {
                ctx->slice_ref_flag = 1;
            }
        }
        else
        {
            pos = (pic_imcnt % gop_size) - 1;
            ctx->slice_depth = FRM_DEPTH_2;
            ctx->poc.poc_val = ((pic_imcnt / gop_size) * gop_size) - gop_size + pos + 1;
            ctx->slice_ref_flag = 0;
        }
        /* find current encoding picture's(B picture) pic_icnt */
        pic_icnt_b = ctx->poc.poc_val;

        /* find pico again here */
        ctx->pico_idx = (u8)(pic_icnt_b % ctx->pico_max_cnt);
        ctx->pico = ctx->pico_buf[ctx->pico_idx];

        PIC_ORIG(ctx) = &ctx->pico->pic;
    }
}

/* slice_type / slice_depth / poc / PIC_ORIG setting */
static void decide_slice_type(EVCE_CTX * ctx)
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

    if(gop_size == 1) 
    {
        if (i_period == 1) /* IIII... */
        {
            ctx->slice_type = SLICE_I;
            ctx->slice_depth = FRM_DEPTH_0;
            ctx->poc.poc_val = pic_icnt;
            ctx->slice_ref_flag = 0;
        }
        else /* IPPP... */
        {
            pic_imcnt = (i_period > 0) ? pic_icnt % i_period : pic_icnt;
            if (pic_imcnt == 0)
            {
                ctx->slice_type = SLICE_I;
                ctx->slice_depth = FRM_DEPTH_0;
                ctx->poc.poc_val = 0;
                ctx->slice_ref_flag = 1;
            }
            else
            {
                ctx->slice_type = ctx->cdsc.inter_slice_type;

                if (ctx->param.use_hgop)
                {
                    if (ctx->sps.tool_rpl)
                    {
                        ctx->slice_depth = tbl_slice_depth_P_orig[(pic_imcnt - 1) % GOP_P];
                    }
                    else
                    {
                        ctx->slice_depth = tbl_slice_depth_P[ctx->param.ref_pic_gap_length >> 2][(pic_imcnt - 1) % ctx->param.ref_pic_gap_length];
                    }
                }
                else
                {
                    ctx->slice_depth = FRM_DEPTH_1;
                }
                ctx->poc.poc_val = (i_period > 0) ? ctx->pic_cnt % i_period : ctx->pic_cnt;
                ctx->slice_ref_flag = 1;
            }
        }
    }
    else /* include B Picture (gop_size = 2 or 4 or 8 or 16) */
    {
        if(pic_icnt == gop_size - 1) /* special case when sequence start */
        {
            ctx->slice_type = SLICE_I;
            ctx->slice_depth = FRM_DEPTH_0;
            ctx->poc.poc_val = 0;
            ctx->poc.prev_doc_offset = 0;
            ctx->poc.prev_poc_val = ctx->poc.poc_val;
            ctx->slice_ref_flag = 1;

            /* flush the first IDR picture */
            PIC_ORIG(ctx) = &ctx->pico_buf[0]->pic;
            ctx->pico = ctx->pico_buf[0];
        }
        else if(ctx->force_slice)
        {
            for(force_cnt = ctx->force_ignored_cnt; force_cnt < gop_size; force_cnt++)
            {
                pic_icnt = (ctx->pic_cnt + ctx->param.max_b_frames + force_cnt);
                pic_imcnt = pic_icnt;

                decide_normal_gop(ctx, pic_imcnt);

                if(ctx->poc.poc_val <= (int)ctx->pic_ticnt)
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
    if (ctx->param.use_hgop && (gop_size > 1 || ctx->sps.tool_rpl))
    {
        if (ctx->sps.tool_rpl)
        {
            ctx->nalu.nuh_temporal_id = ctx->slice_depth;
        } else
        {
            ctx->nalu.nuh_temporal_id = ctx->slice_depth - (ctx->slice_depth > 0);
        }
    }
    else
    {
        ctx->nalu.nuh_temporal_id = 0;
    }
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

    decide_slice_type(ctx);

    ctx->lcu_cnt = ctx->f_lcu;
    ctx->slice_num = 0;

    if(ctx->slice_type == SLICE_I) ctx->last_intra_poc = ctx->poc.poc_val;

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
    evc_mset_x64a(ctx->map_affine, 0, sizeof(u32) * ctx->f_scu);
    evc_mset_x64a(ctx->map_ats_inter, 0, sizeof(u8) * ctx->f_scu);
    evc_mset_x64a(ctx->map_cu_mode, 0, sizeof(u32) * ctx->f_scu);

    return EVC_OK;
}

int evce_enc_pic_finish(EVCE_CTX *ctx, EVC_BITB *bitb, EVCE_STAT *stat)
{
    EVC_IMGB *imgb_o, *imgb_c;
    int        ret;
    int        i, j;

    evc_mset(stat, 0, sizeof(EVCE_STAT));

    /* adding picture sign */
    if (ctx->param.use_pic_sign)
    {
        EVC_BSW  *bs = &ctx->bs;
        EVC_NALU sei_nalu;
#if FIX_TEMPORAL_ID_SET
        set_nalu(&sei_nalu, EVC_SEI_NUT, ctx->nalu.nuh_temporal_id);
#else
        set_nalu(&sei_nalu, EVC_SEI_NUT);
#endif

        int* size_field = (int*)(*(&bs->cur));
        u8* cur_tmp = bs->cur;

        evce_eco_nalu(bs, sei_nalu);

        ret = evce_eco_sei(ctx, bs);
        evc_assert_rv(ret == EVC_OK, ret);

        evc_bsw_deinit(bs);
        stat->sei_size = (int)(bs->cur - cur_tmp);
        *size_field = stat->sei_size - 4;
    }

    /* expand current encoding picture, if needs */
    ctx->fn_picbuf_expand(ctx, PIC_CURR(ctx));

    /* picture buffer management */
    ret = evc_picman_put_pic(&ctx->rpm, PIC_CURR(ctx), ctx->nalu.nal_unit_type_plus1 - 1 == EVC_IDR_NUT,
                              ctx->poc.poc_val, ctx->nalu.nuh_temporal_id, 0, ctx->refp,
                              ctx->slice_ref_flag, ctx->sps.tool_rpl, ctx->ref_pic_gap_length);

    evc_assert_rv(ret == EVC_OK, ret);

    imgb_o = PIC_ORIG(ctx)->imgb;
    evc_assert(imgb_o != NULL);

    imgb_c = PIC_CURR(ctx)->imgb;
    evc_assert(imgb_c != NULL);

    /* set stat */
    stat->write = EVC_BSW_GET_WRITE_BYTE(&ctx->bs);
    stat->nalu_type = ctx->slice_type == SLICE_I ? EVC_IDR_NUT : EVC_NONIDR_NUT;
    stat->stype = ctx->slice_type;
    stat->fnum = ctx->pic_cnt;
    stat->qp = ctx->sh.qp;
    stat->poc = ctx->poc.poc_val;
    stat->tid = ctx->nalu.nuh_temporal_id;

    for(i = 0; i < 2; i++)
    {
        stat->refpic_num[i] = ctx->rpm.num_refp[i];
        for (j = 0; j < stat->refpic_num[i]; j++)
        {
            stat->refpic[i][j] = ctx->refp[j][i].poc;
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
#if EVC_TILE_SUPPORT

static void update_core_loc_param(EVCE_CTX * ctx, EVCE_CORE * core)
{
    core->x_pel = core->x_lcu << ctx->log2_max_cuwh;  // entry point's x location in pixel
    core->y_pel = core->y_lcu << ctx->log2_max_cuwh;  // entry point's y location in pixel
    core->x_scu = core->x_lcu << (MAX_CU_LOG2 - MIN_CU_LOG2); // set x_scu location 
    core->y_scu = core->y_lcu << (MAX_CU_LOG2 - MIN_CU_LOG2); // set y_scu location 
    core->lcu_num = core->x_lcu + core->y_lcu*ctx->w_lcu; // Init the first lcu_num in tile
}
#endif


int evce_enc_pic(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CORE * core;
    EVC_BSW   * bs;
    EVC_SH    * sh;
    EVC_APS   * aps;  // ALF aps
#if M52291_HDR_DRA
    EVC_APS_GEN   *aps_alf = ctx->aps_gen_array[0];
    EVC_APS_GEN   *aps_dra = ctx->aps_gen_array[1];
#endif

    int         ret;
    u32         i;
    int         split_mode_child[4];
    int         split_allow[6] = { 0, 0, 0, 0, 0, 1 };
#if EVC_TILE_SUPPORT
    int         ctb_cnt_in_tile = 0;
    int         col_bd = 0;
    int num_slice_in_pic = ctx->param.num_slice_in_pic;
    u8  * tiles_in_slice, total_tiles_in_slice, total_tiles_in_slice_copy;
    u8* curr_temp = NULL;
    for (ctx->slice_num = 0; ctx->slice_num < num_slice_in_pic; ctx->slice_num++)
    {
        if (num_slice_in_pic > 1)
        {
            if (!ctx->param.arbitrary_slice_flag)
            {
                total_tiles_in_slice = 0;
                for (u32 k = 0; k < ctx->tile_cnt; k++)
                {
                    if (ctx->tile_to_slice_map[k] == ctx->slice_num)
                    {
                        ctx->tiles_in_slice[total_tiles_in_slice] = k;
                        total_tiles_in_slice++;
                    }
                }
                total_tiles_in_slice_copy = total_tiles_in_slice;
            }
            else
            {
                total_tiles_in_slice = ctx->param.num_remaining_tiles_in_slice_minus1 + 2;
                for (u32 k = 0; k < (ctx->param.num_remaining_tiles_in_slice_minus1 + 2); k++)
                {
                    ctx->tiles_in_slice[k] = ctx->param.slice_boundary_array[ctx->slice_num * total_tiles_in_slice + k];
                }
                total_tiles_in_slice_copy = total_tiles_in_slice;
            }
        }
        else
        {
            if (ctx->param.arbitrary_slice_flag)
            {
                total_tiles_in_slice = ctx->param.num_remaining_tiles_in_slice_minus1 + 2;
                for (u32 k = 0; k < (ctx->param.num_remaining_tiles_in_slice_minus1 + 2); k++)
                {
                    ctx->tiles_in_slice[k] = ctx->param.slice_boundary_array[ctx->slice_num * total_tiles_in_slice + k];
                }
                total_tiles_in_slice_copy = total_tiles_in_slice;
            }
            else
            {
                total_tiles_in_slice = 0;
                for (u32 k = 0; k < ctx->tile_cnt; k++)
                {
                    ctx->tiles_in_slice[total_tiles_in_slice] = k;
                    total_tiles_in_slice++;
                }
                total_tiles_in_slice_copy = total_tiles_in_slice;
            }
        }
        tiles_in_slice = ctx->tiles_in_slice;
#endif

    bs = &ctx->bs;
    core = ctx->core;
    sh = &ctx->sh;
#if EVC_TILE_SUPPORT    
        sh->num_tiles_in_slice = total_tiles_in_slice;
#endif    
    aps = &ctx->aps;
    aps_counter_reset = FALSE;

    if ((int)ctx->poc.poc_val > last_intra_poc)
    {
        last_intra_poc = INT_MAX;
        aps_counter_reset = TRUE;
    }
    if (ctx->slice_type == SLICE_I)
        last_intra_poc = ctx->poc.poc_val;
    if (aps_counter_reset)
        ctx->aps_counter = 0;
    if (ctx->slice_type == SLICE_I)
    {
        ctx->aps_counter = -1;

        aps->aps_id = -1;
#if M52291_HDR_DRA
        aps_alf->aps_id = -1;
#endif
        ctx->sh.aps_signaled = -1; // reset stored aps id in tile group header
        ctx->aps_temp = 0;
    }

    if (!ctx->sps.tool_rpl)
    {
        /* initialize reference pictures */
        ret = evc_picman_refp_init(&ctx->rpm, ctx->sps.max_num_ref_pics, ctx->slice_type, ctx->poc.poc_val, ctx->nalu.nuh_temporal_id, ctx->last_intra_poc, ctx->refp);
    }
    else
    {
        /* Set slice header */
        //This needs to be done before reference picture marking and reference picture list construction are invoked
        set_sh(ctx, sh);
#if GRAB_STAT
        evc_stat_set_poc(ctx->poc.poc_val);
#endif

        if (sh->slice_type != SLICE_I && ctx->poc.poc_val != 0) //TBD: change this condition to say that if this slice is not a slice in IDR picture
        {
            ret = create_explicit_rpl(&ctx->rpm, sh, ctx->poc.poc_val);
            if (ret == 1)
            {
                if (ctx->pps.rpl1_idx_present_flag)
                {
                    if (sh->rpl_l0_idx == -1)
                    {
                        sh->ref_pic_list_sps_flag[0] = 0;
                    }
                    if (sh->rpl_l1_idx == -1)
                    {
                        sh->ref_pic_list_sps_flag[1] = 0;
                    }
                }
                else
                {
                    sh->ref_pic_list_sps_flag[0] = 0;
                    sh->ref_pic_list_sps_flag[1] = 0;
                }
            }
        }

        /* reference picture marking */
        ret = evc_picman_refpic_marking(&ctx->rpm, sh, ctx->poc.poc_val);
        evc_assert_rv(ret == EVC_OK, ret);

        /* reference picture lists construction */
        ret = evc_picman_refp_rpl_based_init(&ctx->rpm, sh, ctx->poc.poc_val, ctx->refp);
        if (sh->slice_type != SLICE_I)
        {
            int delta_poc0 = (int)(ctx->poc.poc_val) - (int)(ctx->refp[0][REFP_0].poc);
            int delta_poc1 = (int)(ctx->poc.poc_val) - (int)(ctx->refp[0][REFP_1].poc);
            sh->temporal_mvp_asigned_flag = !(((delta_poc0 > 0) && (delta_poc1 > 0)) || ((delta_poc0 < 0) && (delta_poc1 < 0)));
            //printf("tmvp: %d %d %d %d\n", ctx->poc, ctx->refp[0][REFP_0].poc, ctx->refp[0][REFP_1].poc, sh->temporal_mvp_asigned_flag);
        }
    }
    evc_assert_rv(ret == EVC_OK, ret);

    /* initialize mode decision for frame encoding */
    ret = ctx->fn_mode_init_frame(ctx);
    evc_assert_rv(ret == EVC_OK, ret);

    ctx->fn_mode_analyze_frame(ctx);

    /* slice layer encoding loop */
    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->f_lcu;

    /* Set nalu header */
#if FIX_TEMPORAL_ID_SET
    set_nalu(&ctx->nalu, ctx->pic_cnt == 0 || (ctx->slice_type == SLICE_I && ctx->param.use_closed_gop) ? EVC_IDR_NUT : EVC_NONIDR_NUT, ctx->nalu.nuh_temporal_id);
#else
    set_nalu(&ctx->nalu, ctx->pic_cnt == 0 || (ctx->slice_type == SLICE_I && ctx->param.use_closed_gop) ? EVC_IDR_NUT : EVC_NONIDR_NUT);
#endif

    if (!ctx->sps.tool_rpl)
    {
        /* Set slice header */
        set_sh(ctx, sh);
    }

    core->qp_y = ctx->sh.qp + 6 * (BIT_DEPTH - 8);
    core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][sh->qp_u] + 6 * (BIT_DEPTH - 8);
    core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][sh->qp_v] + 6 * (BIT_DEPTH - 8);
#if HISTORY_UNDER_ADMVP_FIX
    if (ctx->sps.tool_admvp)
    {
#endif
    ret = evce_hmvp_init(&(core->history_buffer));
    evc_assert_rv(ret == EVC_OK, ret);
#if HISTORY_UNDER_ADMVP_FIX
    }
#endif
#if !EVC_TILE_SUPPORT    
    /* initialize entropy coder */
    evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
    evce_sbac_reset(&core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
#endif

    core->bs_temp.pdata[1] = &core->s_temp_run;

    /* LCU encoding */
#if TRACE_RDO_EXCLUDE_I
    if(ctx->slice_type != SLICE_I)
    {
#endif
        EVC_TRACE_SET(0);
#if TRACE_RDO_EXCLUDE_I
    }
#endif
    if (ctx->sps.tool_mmvd && (ctx->slice_type == SLICE_B))
    {
        sh->mmvd_group_enable_flag = !(ctx->refp[0][0].poc == ctx->refp[0][1].poc);
    }
    else if (ctx->sps.tool_mmvd && (ctx->slice_type == SLICE_P))
    {
        sh->mmvd_group_enable_flag = 0;
    }
    else
    {
        sh->mmvd_group_enable_flag = 0;
    }
#if DQP
    ctx->sh.qp_prev_eco = ctx->sh.qp;
    ctx->sh.qp_prev_mode = ctx->sh.qp;
    core->dqp_data[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].prev_QP = ctx->sh.qp_prev_mode;
    core->dqp_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].curr_QP = ctx->sh.qp;
    core->dqp_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].prev_QP = ctx->sh.qp;
#endif
#if EVC_TILE_SUPPORT
    /* Tile wise encoding with in a slice */
        u32 k = 0;
        total_tiles_in_slice = total_tiles_in_slice_copy;

        while (total_tiles_in_slice)
        {
            int i = tiles_in_slice[k++];
            core->tile_num = i;
#if EVC_TILE_DQP
            ctx->tile[i].qp = ctx->sh.qp;
            ctx->tile[i].qp_prev_eco = ctx->sh.qp;
            core->tile_idx = i;
#endif
        /* CABAC Initialize for each Tile */
        evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
        evce_sbac_reset(&core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);

        /*Set entry point for each Tile in the tile Slice*/
        core->x_lcu = (ctx->tile[i].ctba_rs_first) % ctx->w_lcu; //entry point lcu's x location
        core->y_lcu = (ctx->tile[i].ctba_rs_first) / ctx->w_lcu; // entry point lcu's y location
        ctb_cnt_in_tile = ctx->tile[i].f_ctb; //Total LCUs in the current tile
        update_core_loc_param(ctx, core);

#if FIX_DQP_ON
#if EVC_TILE_DQP
        int bef_cu_qp = ctx->tile[i].qp_prev_eco;
#else
        int bef_cu_qp = ctx->sh.qp;
#endif
#endif
      
            col_bd = 0;
            if (i% ctx->param.tile_columns)
            {
                int temp = i - 1;
                while (temp >= 0)
                {
                    col_bd += ctx->tile[temp].w_ctb;
                    if (!(temp%ctx->param.tile_columns)) break;
                    temp--;
                }
            }
            else
            {
                col_bd = 0;
            }

        /* LCU decoding loop */
        while (1)
        {
            /* initialize structures *****************************************/
            ret = ctx->fn_mode_init_lcu(ctx, core);
            evc_assert_rv(ret == EVC_OK, ret);
#if HISTORY_UNDER_ADMVP_FIX
            if (ctx->sps.tool_admvp && (core->x_lcu == (ctx->tile[i].ctba_rs_first) % ctx->w_lcu))
#else
            if (core->x_lcu == (ctx->tile[i].ctba_rs_first) % ctx->w_lcu) //This condition will reset history buffer
#endif
            {
                ret = evce_hmvp_init(&(core->history_buffer));
                evc_assert_rv(ret == EVC_OK, ret);
            }

            /* mode decision *************************************************/
            SBAC_LOAD(core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], *GET_SBAC_ENC(bs));
            core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].is_bitcount = 1;
            evce_init_bef_data(core, ctx);
#if GRAB_STAT
            evc_stat_set_enc_state(TRUE);
#endif
            ret = ctx->fn_mode_analyze_lcu(ctx, core);
            evc_assert_rv(ret == EVC_OK, ret);
            if (ctx->param.use_ibc_flag && (ctx->param.ibc_fast_method & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE) && ctx->param.ibc_hash_search_flag)
            {
                reset_ibc_search_range(ctx, core->x_pel, core->y_pel, ctx->max_cuwh, ctx->max_cuwh);
            }
#if FIX_DQP_ON
#if EVC_TILE_DQP
            ctx->tile[i].qp_prev_eco = bef_cu_qp;
#else
            ctx->sh.qp_prev_eco = bef_cu_qp;
#endif
#endif
            /* entropy coding ************************************************/
            ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1
                , NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
                , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif
            );
#if FIX_DQP_ON
#if EVC_TILE_DQP
            bef_cu_qp = ctx->tile[i].qp_prev_eco;
#else
            bef_cu_qp = ctx->sh.qp_prev_eco;
#endif
#endif
#if GRAB_STAT
            evc_stat_set_enc_state(FALSE);
            evc_stat_write_lcu(core->x_pel, core->y_pel, ctx->w, ctx->h, ctx->max_cuwh, ctx->log2_culine, ctx, core, ctx->map_cu_data[core->lcu_num].split_mode, ctx->map_cu_data[core->lcu_num].suco_flag);
#endif
            evc_assert_rv(ret == EVC_OK, ret);
            /* prepare next step *********************************************/
            core->x_lcu++;
            if (core->x_lcu >= ctx->tile[i].w_ctb + col_bd)
            {
                core->x_lcu = (ctx->tile[i].ctba_rs_first) % ctx->w_lcu;
                core->y_lcu++;
            }
#if 0
            printf("processing lcu  X_LCU = [%d]  Y_LCU = [%d]\n", core->x_lcu, core->y_lcu);
#endif
            update_core_loc_param(ctx, core);
            ctb_cnt_in_tile--;
            ctx->lcu_cnt--; //To be updated properly in case of multicore

#if HISTORY_LCU_COPY_BUG_FIX
#if HISTORY_UNDER_ADMVP_FIX
            if (ctx->sps.tool_admvp)
            {
#endif
                evc_mcpy(&core->history_buffer, &core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], sizeof(core->history_buffer));
#if HISTORY_UNDER_ADMVP_FIX
            }
#endif
#endif
#if 0
            if (ctb_cnt_in_tile > 0)
            {
                evce_eco_tile_end_flag(bs, 0);
            }
            else
#else
            if (ctb_cnt_in_tile == 0)
#endif
            {
                evce_eco_tile_end_flag(bs, 1);
                evce_sbac_finish(bs);
                break;
            }
        } //End of LCU processing loop for a tile
            total_tiles_in_slice--;
    } //End of Slice encoding loop (All the tiles in a slice)
#else    
    while(1)
    {
        /* initialize structures *****************************************/
        ret = ctx->fn_mode_init_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
#if HISTORY_UNDER_ADMVP_FIX
        if (core->x_pel == 0 && ctx->sps.tool_admvp)
#else
        if (core->x_pel == 0)
#endif
        {
            ret = evce_hmvp_init(&(core->history_buffer));
            evc_assert_rv(ret == EVC_OK, ret);
        }

        /* mode decision *************************************************/
        SBAC_LOAD(core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], *GET_SBAC_ENC(bs));
        core->s_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].is_bitcount = 1;
        evce_init_bef_data(core, ctx);

#if DQP_RDO
        if(ctx->pps.cu_qp_delta_enabled_flag)
        {
            core->dqp_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].prev_QP = ctx->sh.qp_prev_eco;
            core->dqp_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].cu_qp_delta_is_coded = 0;
            core->dqp_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].cu_qp_delta_code = 0;
        }
#endif
      
#if GRAB_STAT
        evc_stat_set_enc_state(TRUE);
#endif
        ret = ctx->fn_mode_analyze_lcu(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
        if (ctx->param.use_ibc_flag && (ctx->param.ibc_fast_method & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE) && ctx->param.ibc_hash_search_flag)
        {
                reset_ibc_search_range(ctx, core->x_pel, core->y_pel, ctx->max_cuwh, ctx->max_cuwh);
        }
        /* entropy coding ************************************************/
#if DQP_RDO
        if(ctx->pps.cu_qp_delta_enabled_flag)
        {
            ctx->sh.qp_prev_eco = core->dqp_curr_best[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2].prev_QP;
        }
#endif
        ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1
                             , NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
            , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
            , evc_get_default_tree_cons()
#endif
        );
#if GRAB_STAT
        evc_stat_set_enc_state(FALSE);
        evc_stat_write_lcu(core->x_pel, core->y_pel, ctx->w, ctx->h, ctx->max_cuwh, ctx->log2_culine, ctx, core, ctx->map_cu_data[core->lcu_num].split_mode, ctx->map_cu_data[core->lcu_num].suco_flag);
#endif
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
#if HISTORY_UNDER_ADMVP_FIX
        if (ctx->sps.tool_admvp)
        {
#endif
            evc_mcpy(&core->history_buffer, &core->m_pBestMotLUTs[ctx->log2_max_cuwh - 2][ctx->log2_max_cuwh - 2], sizeof(core->history_buffer));
#if HISTORY_UNDER_ADMVP_FIX
        }
#endif
#endif

        /* end_of_picture_flag */
        if(ctx->lcu_cnt > 0)
        {
            evce_eco_slice_end_flag(bs, 0);
        }
        else
        {
            evce_eco_slice_end_flag(bs, 1);
            evce_sbac_finish(bs);
            break;
        }
    }
#endif
         
    /* deblocking filter */
#if ENC_DBF_CONTROL
    if (sh->deblocking_filter_on)
#else
    if (ctx->param.use_deblock)
#endif
    {
#if TRACE_DBF
        EVC_TRACE_SET(1);
#endif
#if EVC_TILE_SUPPORT
        u32 k = 0;
        total_tiles_in_slice = total_tiles_in_slice_copy;
        while (total_tiles_in_slice)
        {
            int i = tiles_in_slice[k++];
            ret = ctx->fn_deblock(ctx, PIC_MODE(ctx), i
#if EVC_CONCURENCY
                , core
#endif
            );
            evc_assert_rv(ret == EVC_OK, ret);
            total_tiles_in_slice--;
        }
#else 
        ret = ctx->fn_deblock(ctx, PIC_MODE(ctx)
#if EVC_CONCURENCY
            , core
#endif
        );
        evc_assert_rv(ret == EVC_OK, ret);
#endif
#if TRACE_DBF
        EVC_TRACE_SET(0);
#endif
    }

    /* adaptive loop filter */
    sh->alf_on = ctx->sps.tool_alf;
    if(sh->alf_on)
    {
        ret = ctx->fn_alf(ctx, PIC_MODE(ctx), sh, aps);
        evc_assert_rv(ret == EVC_OK, ret);
    }

#if EVC_TILE_SUPPORT
#if TEST_ALF_BIT_CALC 
        core->x_lcu = core->y_lcu = 0;
        core->x_pel = core->y_pel = 0;
        core->lcu_num = 0;
        ctx->lcu_cnt = ctx->f_lcu;
        for (i = 0; i < ctx->f_scu; i++)
        {
            MCU_CLR_COD(ctx->map_scu[i]);
        }

#if DQP
        ctx->sh.qp_prev_eco = ctx->sh.qp;
#endif
#if GRAB_STAT
        evc_stat_set_enc_state(FALSE);
#endif
        k = 0;
        total_tiles_in_slice = total_tiles_in_slice_copy;
        while (total_tiles_in_slice)
        {
            int i = tiles_in_slice[k++];
#if EVC_TILE_DQP
            ctx->tile[i].qp = ctx->sh.qp;
            ctx->tile[i].qp_prev_eco = ctx->sh.qp;
            core->tile_idx = i;
#endif
            evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
            core->x_lcu = (ctx->tile[i].ctba_rs_first) % ctx->w_lcu; //entry point lcu's x location
            core->y_lcu = (ctx->tile[i].ctba_rs_first) / ctx->w_lcu; // entry point lcu's y location
            ctb_cnt_in_tile = ctx->tile[i].f_ctb; //Total LCUs in the current tile
            update_core_loc_param(ctx, core);
#if EVC_TILE_SUPPORT
            EVC_BSW bs_beg;
            bs_beg.cur = bs->cur;
            bs_beg.leftbits = bs->leftbits;
#endif 
            col_bd = 0;
            if (i% ctx->param.tile_columns)
            {
                int temp = i - 1;
                while (temp >= 0)
                {
                    col_bd += ctx->tile[temp].w_ctb;
                    if (!(temp%ctx->param.tile_columns)) break;
                    temp--;
                }
            }
            else
            {
                col_bd = 0;
            }
            while (1) // LCU level CABAC loop
            {
                evc_AlfSliceParam* alfSliceParam = &(ctx->sh.alf_sh_param);
                if ((alfSliceParam->isCtbAlfOn) && (sh->alf_on))
                {
                    EVCE_SBAC *sbac;
                    sbac = GET_SBAC_ENC(bs);
                    EVC_TRACE_COUNTER;
                    EVC_TRACE_STR("Usage of ALF: ");
                    evce_sbac_encode_bin((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)), sbac, sbac->ctx.alf_ctb_flag, bs);
                    EVC_TRACE_INT((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)));
                    EVC_TRACE_STR("\n");
                }
                ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1, NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
                    , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                    , evc_get_default_tree_cons()
#endif

                );
                evc_assert_rv(ret == EVC_OK, ret);
                core->x_lcu++;
                if (core->x_lcu >= ctx->tile[i].w_ctb + col_bd)
                {
                    core->x_lcu = (ctx->tile[i].ctba_rs_first) % ctx->w_lcu;
                    core->y_lcu++;
                }
                update_core_loc_param(ctx, core);
                ctb_cnt_in_tile--;
                ctx->lcu_cnt--; //To be updated properly in case of multicore
                if (ctb_cnt_in_tile == 0)
                {
                    evce_eco_tile_end_flag(bs, 1);
                    evce_sbac_finish(bs);
                    break;
                }
            } //End of LCU encoding loop in a tile
#if EVC_TILE_SUPPORT
            total_tiles_in_slice--;
#endif
#if EVC_TILE_SUPPORT
            sh->entry_point_offset_minus1[k - 1] = (u32)((bs)->cur - bs_beg.cur - 4 + (4 - (bs->leftbits >> 3)) + (bs_beg.leftbits >> 3) - 1);
#endif
        } // End to tile encoding loop in a slice
#endif
#endif

    /* Bit-stream re-writing (START) */
#if EVC_TILE_SUPPORT
        u8* tmp_ptr1;
        if (ctx->slice_num == 0)
        {
            evc_bsw_init(&ctx->bs, (u8*)bitb->addr, bitb->bsize, NULL);
            tmp_ptr1 = bs->beg;
        }
        else
        {
            evc_bsw_init_slice(&ctx->bs, (u8*)curr_temp, bitb->bsize, NULL);
            tmp_ptr1 = curr_temp;
        }
#else
        evc_bsw_init(&ctx->bs, (u8*)bitb->addr, bitb->bsize, NULL);
#endif    
#if TRACE_START_POC
    if (fp_trace_started == 1)
    {
        EVC_TRACE_SET(1);
    }
    else
    {
        EVC_TRACE_SET(0);
    }
#else
#if TRACE_RDO_EXCLUDE_I
    if(ctx->slice_type != SLICE_I)
    {
#endif
#if !TRACE_DBF
        EVC_TRACE_SET(1);
#endif
#if TRACE_RDO_EXCLUDE_I
    }
#endif
#endif

    /* Send available APSs */
    int aps_nalu_size = 0;
    {
        /* Encode ALF in APS */
        if ((ctx->sps.tool_alf) && (ctx->sh.alf_on)) // User defined params
        {
            if ((aps->alf_aps_param.enabledFlag[0]) && (aps->alf_aps_param.temporalAlfFlag == 0))    // Encoder defined parameters (RDO): ALF is selected, and new ALF was derived for TG
            {
                aps_nalu_size = 0;
                EVC_NALU aps_nalu;
#if FIX_TEMPORAL_ID_SET
                set_nalu(&aps_nalu, EVC_APS_NUT, ctx->nalu.nuh_temporal_id);
#else
                set_nalu(&aps_nalu, EVC_APS_NUT);
#endif

                /* Encode APS nalu header */
                int* size_field = (int*)(*(&bs->cur));
                u8* cur_tmp = bs->cur;
                ret = evce_eco_nalu(bs, aps_nalu);
                evc_assert_rv(ret == EVC_OK, ret);

                /* Write ALF-APS */
#if M52291_HDR_DRA
                evc_AlfSliceParam* p_aps_data = (evc_AlfSliceParam*)aps_alf->aps_data;
                aps_alf->aps_id = aps->aps_id;
                memcpy(p_aps_data, &(aps->alf_aps_param), sizeof(evc_AlfSliceParam));
                evc_assert_rv(evce_eco_aps_gen(bs, aps_alf) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);
#else
                set_aps(ctx, aps); // TBD: empty function call
                evc_assert_rv(evce_eco_aps(bs, aps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);
#endif

                evc_bsw_deinit(bs);
                *size_field = (int)(bs->cur - cur_tmp) - 4;
            }
        }

#if M52291_HDR_DRA
        /* Encode DRA in APS */
        if ((ctx->sps.tool_dra) && aps_dra->signal_flag) // User defined params
        {
            aps_nalu_size = 0;
            EVC_NALU aps_nalu;
#if FIX_TEMPORAL_ID_SET
            set_nalu(&aps_nalu, EVC_APS_NUT, ctx->nalu.nuh_temporal_id);
#else
            set_nalu(&aps_nalu, EVC_APS_NUT);
#endif

            int* size_field = (int*)(*(&bs->cur));
            u8* cur_tmp = bs->cur;
    
            /* Encode APS nalu header */
            ret = evce_eco_nalu(bs, aps_nalu);
            evc_assert_rv(ret == EVC_OK, ret);

            /* Write DRA-APS */
            evc_assert_rv(evce_eco_aps_gen(bs, aps_dra) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);
            evc_bsw_deinit(bs);
            *size_field = (int)(bs->cur - cur_tmp) - 4;
            aps_dra->signal_flag = 0;

        }
#endif
    }


    int* size_field = (int*)(*(&bs->cur));
    u8* cur_tmp = bs->cur;

    /* Encode nalu header */
    ret = evce_eco_nalu(bs, ctx->nalu);
    evc_assert_rv(ret == EVC_OK, ret);

    /* Encode slice header */
    sh->num_ctb = ctx->f_lcu;
#if EVC_TILE_SUPPORT
        EVC_BSW bs_sh;
        evc_mcpy(&bs_sh, bs, sizeof(EVC_BSW));
#endif
    ret = evce_eco_sh(bs, &ctx->sps, &ctx->pps, sh, ctx->nalu.nal_unit_type_plus1 - 1);
    evc_assert_rv(ret == EVC_OK, ret);

    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->f_lcu;
    for(i = 0; i < ctx->f_scu; i++)
    {
        MCU_CLR_COD(ctx->map_scu[i]);
    }

#if !EVC_TILE_SUPPORT
    evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);
#endif
#if DQP
    ctx->sh.qp_prev_eco = ctx->sh.qp;
#endif

#if GRAB_STAT
    evc_stat_set_enc_state(FALSE);
#endif
#if EVC_TILE_SUPPORT
    /* Tile level encoding for a slice */
    /* Tile wise encoding with in a slice */
    k = 0;
    total_tiles_in_slice = total_tiles_in_slice_copy;
    while (total_tiles_in_slice)
    {
        int i = tiles_in_slice[k++];
#if EVC_TILE_DQP
        ctx->tile[i].qp = ctx->sh.qp;
        ctx->tile[i].qp_prev_eco = ctx->sh.qp;
        core->tile_idx = i;
#endif
        /* CABAC Initialize for each Tile */
        evce_sbac_reset(GET_SBAC_ENC(bs), ctx->sh.slice_type, ctx->sh.qp, ctx->sps.tool_cm_init);

        /*Set entry point for each Tile in the tile Slice*/
        core->x_lcu = (ctx->tile[i].ctba_rs_first) % ctx->w_lcu; //entry point lcu's x location
        core->y_lcu = (ctx->tile[i].ctba_rs_first) / ctx->w_lcu; // entry point lcu's y location
        ctb_cnt_in_tile = ctx->tile[i].f_ctb; //Total LCUs in the current tile
        update_core_loc_param(ctx, core);


        EVC_BSW bs_beg;
        bs_beg.cur = bs->cur;
        bs_beg.leftbits = bs->leftbits;

        col_bd = 0;
        if (i% ctx->param.tile_columns)
        {
            int temp = i - 1;
            while (temp >= 0)
            {
                col_bd += ctx->tile[temp].w_ctb;
                if (!(temp%ctx->param.tile_columns)) break;
                temp--;
            }
        }
        else
        {
            col_bd = 0;
        }

        while (1) // LCU level CABAC loop
        {
            evc_AlfSliceParam* alfSliceParam = &(ctx->sh.alf_sh_param);
            if ((alfSliceParam->isCtbAlfOn) && (sh->alf_on))
            {
                EVCE_SBAC *sbac;
                sbac = GET_SBAC_ENC(bs);
                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("Usage of ALF: ");
                evce_sbac_encode_bin((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)), sbac, sbac->ctx.alf_ctb_flag, bs);
                EVC_TRACE_INT((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)));
                EVC_TRACE_STR("\n");
            }
            ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1, NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
                , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
                , evc_get_default_tree_cons()
#endif

            );
            evc_assert_rv(ret == EVC_OK, ret);
            /* prepare next step *********************************************/
            core->x_lcu++;
            //if (core->x_lcu >= ctx->tile[i].w_ctb + col_bd[i % ctx->param.tile_columns])
            if (core->x_lcu >= ctx->tile[i].w_ctb + col_bd)
            {
                core->x_lcu = (ctx->tile[i].ctba_rs_first) % ctx->w_lcu;
                core->y_lcu++;
            }

            update_core_loc_param(ctx, core);
            ctb_cnt_in_tile--;
            ctx->lcu_cnt--; //To be updated properly in case of multicore

            /* end_of_picture_flag */
            if (ctb_cnt_in_tile == 0)
            {
                evce_eco_tile_end_flag(bs, 1);
                evce_sbac_finish(bs);
                break;
            }
        } //End of LCU encoding loop in a tile

        total_tiles_in_slice--;

        sh->entry_point_offset_minus1[k - 1] = (u32)((bs)->cur - bs_beg.cur - 4 + (4 - (bs->leftbits >> 3)) + (bs_beg.leftbits >> 3) - 1);
    } // End to tile encoding loop in a slice

#else 
    /* Encode slice data */
    while(1)
    {
        evc_AlfSliceParam* alfSliceParam = &(ctx->sh.alf_sh_param);
        if ((alfSliceParam->isCtbAlfOn) && (sh->alf_on))
        {
            EVCE_SBAC *sbac;
            sbac = GET_SBAC_ENC(bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("Usage of ALF: ");
            evce_sbac_encode_bin((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)), sbac, sbac->ctx.alf_ctb_flag, bs);
            EVC_TRACE_INT((int)(*(alfSliceParam->alfCtuEnableFlag + core->lcu_num)));
            EVC_TRACE_STR("\n");
        }
        ret = evce_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->max_cuwh, ctx->max_cuwh, 0, 1, NO_SPLIT, split_mode_child, 0, split_allow, 0, 0
#if DQP
            , 0
#endif
#if M50761_CHROMA_NOT_SPLIT
            , evc_get_default_tree_cons()
#endif
        );
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
            evce_eco_slice_end_flag(bs, 0);
        }
        else
        {
            evce_eco_slice_end_flag(bs, 1);
            evce_sbac_finish(bs);
            break;
        }
    }
#endif

    /* Bit-stream re-writing (END) */
#if EVC_TILE_SUPPORT
    ret = evce_eco_sh(&bs_sh, &ctx->sps, &ctx->pps, sh, ctx->nalu.nal_unit_type_plus1 - 1);
    evc_assert_rv(ret == EVC_OK, ret);
#endif
#if EVC_TILE_SUPPORT    
    evc_bsw_deinit(bs);
    *size_field = (int)(bs->cur - cur_tmp) - 4;
    curr_temp = bs->cur;
#else
    evc_bsw_deinit(bs);
    *size_field = (int)(bs->cur - cur_tmp) - 4;
#endif
#if EVC_TILE_SUPPORT
    }  // End of slice loop
#endif

    return EVC_OK;
}

int evce_enc(EVCE_CTX * ctx, EVC_BITB * bitb, EVCE_STAT * stat)
{
    int            ret;
    int            gop_size, pic_cnt;

    pic_cnt = ctx->pic_icnt - ctx->frm_rnum;
    gop_size = ctx->param.gop_size;

    ctx->force_slice = ((ctx->pic_ticnt % gop_size >= ctx->pic_ticnt - pic_cnt + 1) && FORCE_OUT(ctx)) ? 1 : 0;

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
    if (ctx->param.use_ibc_flag)
    {
      /* create ibc prediction analyzer */
      ret = evce_pibc_create(ctx, 0);
      evc_assert_rv(EVC_OK == ret, ret);
    }
#if RDO_DBK
    ctx->pic_dbk = NULL;
#endif
    ctx->fn_alf = evce_alf_aps;
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

#if M52291_HDR_DRA
    ctx->aps_gen_array[0] = NULL;
    ctx->aps_gen_array[1] = NULL;
#endif

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
    EncAdaptiveLoopFilter* p = (EncAdaptiveLoopFilter*)(ctx->enc_alf);
    call_destroy_enc_ALF(p);
    delete_enc_ALF(ctx->enc_alf);
    ctx->fn_alf = NULL;
    if (ctx->param.use_ibc_flag)
    {
      destroy_enc_IBC(ctx->ibc_hash_handle);
      ctx->ibc_hash_handle = NULL;
    }
    ctx->fn_picbuf_expand = NULL;
    ctx->fn_get_inbuf = NULL;
}

EVCE evce_create(EVCE_CDSC * cdsc, int * err)
{
    EVCE_CTX  * ctx;
    int          ret;
    
#if ENC_DEC_TRACE
#if TRACE_DBF
    fp_trace = fopen("enc_trace_dbf.txt", "w+");
#else
    fp_trace = fopen("enc_trace.txt", "w+");
#endif
#endif
#if GRAB_STAT
    evc_stat_init("enc_stat.vtmbmsstats", esu_only_enc, 0, -1, ence_stat_cu);
    enc_stat_header(cdsc->w, cdsc->h);
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

    if(ctx->fn_ready != NULL)
    {
        ret = ctx->fn_ready(ctx);
        evc_assert_g(ret == EVC_OK, ERR);
    }

    /* set default value for ctx */
    ctx->magic = EVCE_MAGIC_CODE;
    ctx->id = (EVCE)ctx;
    ctx->sh.aps_signaled = -1;
    evc_init_multi_tbl();
    evc_init_multi_inv_tbl();

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
#if GRAB_STAT
    evc_stat_finish();
#endif

    if(ctx->fn_flush != NULL)
    {
        ctx->fn_flush(ctx);
    }
    evce_platform_deinit(ctx);

    ctx_free(ctx);

    evc_scan_tbl_delete();
}

#if M52291_HDR_DRA
int evce_get_pps_dra_flag(EVCE id)
{
    EVCE_CTX * ctx;
    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    return ctx->pps.pic_dra_enabled_flag;
}
#endif
#if M52291_HDR_DRA
int evce_encode_sps(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat, void *p_signalledAPS)
#else
int evce_encode_sps(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
#endif
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc_header, EVC_ERR_UNEXPECTED);
#if M52291_HDR_DRA
    ctx->aps_gen_array[0] = (EVC_APS_GEN*)p_signalledAPS;
    ctx->aps_gen_array[1] = (EVC_APS_GEN*)(p_signalledAPS) + 1;
#endif
    /* update BSB */
    bitb->err = 0;

    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_NALU  nalu;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* nalu header */
#if FIX_TEMPORAL_ID_SET
    set_nalu(&nalu, EVC_SPS_NUT, 0);
#else
    set_nalu(&nalu, EVC_SPS_NUT);
#endif
    evce_eco_nalu(bs, nalu);

    /* sequence parameter set*/
    set_sps(ctx, sps);
    evc_assert_rv(evce_eco_sps(bs, sps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_SPS_NUT;

    return EVC_OK;
}

int evce_encode_pps(EVCE id, EVC_BITB * bitb, EVCE_STAT * stat)
{
    EVCE_CTX * ctx;

    EVCE_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);
    evc_assert_rv(ctx->fn_enc_header, EVC_ERR_UNEXPECTED);

    /* update BSB */
    bitb->err = 0;

    EVC_BSW * bs = &ctx->bs;
    EVC_SPS * sps = &ctx->sps;
    EVC_PPS * pps = &ctx->pps;
    EVC_NALU  nalu;

    evc_assert_rv(bitb->addr && bitb->bsize > 0, EVC_ERR_INVALID_ARGUMENT);

    /* bitsteam initialize for sequence */
    evc_bsw_init(bs, bitb->addr, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;

    /* nalu header */
#if FIX_TEMPORAL_ID_SET
    set_nalu(&nalu, EVC_PPS_NUT, ctx->nalu.nuh_temporal_id);
#else
    set_nalu(&nalu, EVC_PPS_NUT);
#endif
    evce_eco_nalu(bs, nalu);

    /* sequence parameter set*/
    set_pps(ctx, pps);
    evc_assert_rv(evce_eco_pps(bs, sps, pps) == EVC_OK, EVC_ERR_INVALID_ARGUMENT);

    /* de-init BSW */
    evc_bsw_deinit(bs);

    /* write the bitstream size */
    evce_bsw_write_nalu_size(bs);

    /* set stat ***************************************************************/
    evc_mset(stat, 0, sizeof(EVCE_STAT));
    stat->write = EVC_BSW_GET_WRITE_BYTE(bs);
    stat->nalu_type = EVC_PPS_NUT;

    return EVC_OK;
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
        case EVCE_CFG_SET_DEBLOCK_A_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.deblock_alpha_offset = t0;
            break;
        case EVCE_CFG_SET_DEBLOCK_B_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            t0 = *((int *)buf);
            ctx->param.deblock_beta_offset = t0;
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
        case EVCE_CFG_GET_DEBLOCK_A_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.deblock_alpha_offset;
            break;
        case EVCE_CFG_GET_DEBLOCK_B_OFFSET:
            evc_assert_rv(*size == sizeof(int), EVC_ERR_INVALID_ARGUMENT);
            *((int *)buf) = ctx->param.deblock_beta_offset;
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
#if M50761_CHROMA_NOT_SPLIT
    evce_malloc_1d((void**)&cu_data->pred_mode_chroma, size_8b);
#endif
    evce_malloc_2d((s8***)&cu_data->mpm, 2, cu_cnt, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->ipm, 2, cu_cnt, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->mpm_ext, 8, cu_cnt, sizeof(u8));
    evce_malloc_1d((void**)&cu_data->skip_flag, size_8b);
    evce_malloc_1d((void**)&cu_data->ibc_flag, size_8b);
#if DMVR_FLAG
    evce_malloc_1d((void**)&cu_data->dmvr_flag, size_8b);
#endif
    evce_malloc_2d((s8***)&cu_data->refi, cu_cnt, REFP_NUM, sizeof(u8));
    evce_malloc_2d((s8***)&cu_data->mvp_idx, cu_cnt, REFP_NUM, sizeof(u8));
    evce_malloc_1d((void**)&cu_data->mvr_idx, size_8b);
    evce_malloc_1d((void**)&cu_data->bi_idx, size_8b);
    evce_malloc_1d((void**)&cu_data->mmvd_idx, size_16b);
    evce_malloc_1d((void**)&cu_data->mmvd_flag, size_8b);

    evce_malloc_1d((void**)& cu_data->ats_intra_cu, size_8b);
    evce_malloc_1d((void**)& cu_data->ats_mode_h, size_8b);
    evce_malloc_1d((void**)& cu_data->ats_mode_v, size_8b);

    evce_malloc_1d((void**)&cu_data->ats_inter_info, size_8b);

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
    evce_malloc_1d((void**)&cu_data->affine_flag, size_8b);
    evce_malloc_1d((void**)&cu_data->map_affine, size_32b);
    evce_malloc_1d((void**)&cu_data->map_cu_mode, size_32b);
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
#if M50761_CHROMA_NOT_SPLIT
    evce_free_1d((void*)cu_data->pred_mode_chroma);
#endif
    evce_free_2d((void**)cu_data->mpm);
    evce_free_2d((void**)cu_data->ipm);
    evce_free_2d((void**)cu_data->mpm_ext);
    evce_free_1d((void*)cu_data->skip_flag);
    evce_free_1d((void*)cu_data->ibc_flag);
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
    evce_free_1d((void*)cu_data->affine_flag);
    evce_free_1d((void*)cu_data->map_affine);
    evce_free_1d((void*)cu_data->ats_intra_cu);
    evce_free_1d((void*)cu_data->ats_mode_h);
    evce_free_1d((void*)cu_data->ats_mode_v);
    evce_free_1d((void*)cu_data->ats_inter_info);
    evce_free_1d((void*)cu_data->map_cu_mode);
    evce_free_1d((void*)cu_data->depth);

    for (i = 0; i < N_C; i++)
    {
        evce_free_1d((void*)cu_data->coef[i]);
        evce_free_1d((void*)cu_data->reco[i]);
    }

    return EVC_OK;
}

void codeAlfCtuEnableFlag(EVC_BSW *bs, EVCE_CTX * ctx, int refId, int compIdx)
{
// TO DO
}

