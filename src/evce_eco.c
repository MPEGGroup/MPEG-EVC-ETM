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

#include "evce_def.h"
#include <limits.h>

#include <math.h>
#include "wrapper.h"
#include "enc_alf_wrapper.h"

#if M52291_HDR_DRA
#include "evc_util.h"
#endif

#if GRAB_STAT
#include "evc_debug.h"
#endif
#pragma warning(disable:4018)

int evce_eco_nalu(EVC_BSW * bs, EVC_NALU * nalu)
{
#if TRACE_HLS
    evc_bsw_write_trace(bs, nalu->nal_unit_size, 0, 32);
#else
    evc_bsw_write(bs, nalu->nal_unit_size, 32);
#endif
    evc_bsw_write(bs, nalu->forbidden_zero_bit, 1);
    evc_bsw_write(bs, nalu->nal_unit_type_plus1, 6);
    evc_bsw_write(bs, nalu->nuh_temporal_id, 3);
    evc_bsw_write(bs, nalu->nuh_reserved_zero_5bits, 5);
    evc_bsw_write(bs, nalu->nuh_extension_flag, 1);
    return EVC_OK;
}

int evce_eco_rlp(EVC_BSW * bs, EVC_RPL * rpl)
{
    u32 delta_poc_st, strp_entry_sign_flag;
    evc_bsw_write_ue(bs, rpl->ref_pic_num);
    if (rpl->ref_pic_num > 0)
    {
        delta_poc_st = (u32)abs(rpl->ref_pics[0]);
        evc_bsw_write_ue(bs, delta_poc_st);
        if (rpl->ref_pics[0] != 0)
        {
            strp_entry_sign_flag = rpl->ref_pics[0] < 0;
            evc_bsw_write1(bs, strp_entry_sign_flag);
        }

        for (int i = 1; i < rpl->ref_pic_num; ++i)
        {
            delta_poc_st = (u32)abs(rpl->ref_pics[i] - rpl->ref_pics[i - 1]);
            strp_entry_sign_flag = rpl->ref_pics[i - 1] > rpl->ref_pics[i];

            evc_bsw_write_ue(bs, delta_poc_st);
            if (delta_poc_st != 0)
            {                
                evc_bsw_write1(bs, strp_entry_sign_flag);
            }
        }
    }

    return EVC_OK;
}

int evce_eco_ref_pic_list_mod(EVC_BSW * bs)
{
    return EVC_OK;
}

#if EVC_VUI_FIX
int evce_eco_hrd_parameters(EVC_BSW * bs, EVC_HRD * hrd) {
    evc_bsw_write_ue(bs, hrd->cpb_cnt_minus1);
    evc_bsw_write(bs, hrd->bit_rate_scale, 4);
    evc_bsw_write(bs, hrd->cpb_size_scale, 4);
    for (int SchedSelIdx = 0; SchedSelIdx <= hrd->cpb_cnt_minus1; SchedSelIdx++) {
        evc_bsw_write_ue(bs, hrd->bit_rate_value_minus1[SchedSelIdx]);
        evc_bsw_write_ue(bs, hrd->cpb_size_value_minus1[SchedSelIdx]);
        evc_bsw_write1(bs, hrd->cbr_flag[SchedSelIdx]);
    }
    evc_bsw_write(bs, hrd->initial_cpb_removal_delay_length_minus1, 5);
    evc_bsw_write(bs, hrd->cpb_removal_delay_length_minus1, 5);
    evc_bsw_write(bs, hrd->dpb_output_delay_length_minus1, 5);
    evc_bsw_write(bs, hrd->time_offset_length, 5);

    return EVC_OK;
}

int evce_eco_vui(EVC_BSW * bs, EVC_VUI * vui)
{
    evc_bsw_write1(bs, vui->aspect_ratio_info_present_flag);
    if (vui->aspect_ratio_info_present_flag) {
        evc_bsw_write(bs, vui->aspect_ratio_idc, 8);
        if (vui->aspect_ratio_idc == EXTENDED_SAR) {
            evc_bsw_write(bs, vui->sar_width, 16);
            evc_bsw_write(bs, vui->sar_height, 16);
        }
    }
    evc_bsw_write1(bs, vui->overscan_info_present_flag);
    if (vui->overscan_info_present_flag)
        evc_bsw_write1(bs, vui->overscan_appropriate_flag);
    evc_bsw_write1(bs, vui->video_signal_type_present_flag);
    if (vui->video_signal_type_present_flag) {
        evc_bsw_write(bs, vui->video_format, 3);
        evc_bsw_write1(bs, vui->video_full_range_flag);
        evc_bsw_write1(bs, vui->colour_description_present_flag);
        if (vui->colour_description_present_flag) {
            evc_bsw_write(bs, vui->colour_primaries, 8);
            evc_bsw_write(bs, vui->transfer_characteristics, 8);
            evc_bsw_write(bs, vui->matrix_coefficients, 8);
        }
    }
    evc_bsw_write1(bs, vui->chroma_loc_info_present_flag);
    if (vui->chroma_loc_info_present_flag) {
        evc_bsw_write_ue(bs, vui->chroma_sample_loc_type_top_field);
        evc_bsw_write_ue(bs, vui->chroma_sample_loc_type_bottom_field);
    }
    evc_bsw_write1(bs, vui->neutral_chroma_indication_flag);
#if ETM60_HLS_FIX
    evc_bsw_write1(bs, vui->field_seq_flag);
#endif
    evc_bsw_write1(bs, vui->timing_info_present_flag);
    if (vui->timing_info_present_flag) {
        evc_bsw_write(bs, vui->num_units_in_tick, 32);
        evc_bsw_write(bs, vui->time_scale, 32);
        evc_bsw_write1(bs, vui->fixed_pic_rate_flag);
    }
    evc_bsw_write1(bs, vui->nal_hrd_parameters_present_flag);
    if (vui->nal_hrd_parameters_present_flag)
        evce_eco_hrd_parameters(bs, &(vui->hrd_parameters));
    evc_bsw_write1(bs, vui->vcl_hrd_parameters_present_flag);
    if (vui->vcl_hrd_parameters_present_flag)
        evce_eco_hrd_parameters(bs, &(vui->hrd_parameters));
    if (vui->nal_hrd_parameters_present_flag || vui->vcl_hrd_parameters_present_flag)
        evc_bsw_write1(bs, vui->low_delay_hrd_flag);
    evc_bsw_write1(bs, vui->pic_struct_present_flag);
    evc_bsw_write1(bs, vui->bitstream_restriction_flag);
    if (vui->bitstream_restriction_flag) {
        evc_bsw_write1(bs, vui->motion_vectors_over_pic_boundaries_flag);
        evc_bsw_write_ue(bs, vui->max_bytes_per_pic_denom);
        evc_bsw_write_ue(bs, vui->max_bits_per_mb_denom);
        evc_bsw_write_ue(bs, vui->log2_max_mv_length_horizontal);
        evc_bsw_write_ue(bs, vui->log2_max_mv_length_vertical);
        evc_bsw_write_ue(bs, vui->num_reorder_pics);
        evc_bsw_write_ue(bs, vui->max_dec_pic_buffering);
    }

    return EVC_OK;
}
#else
int evce_eco_vui(EVC_BSW * bs)
{
    return EVC_OK;
}
#endif


int evce_eco_sps(EVC_BSW * bs, EVC_SPS * sps)
{
#if TRACE_HLS
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ SPS Start ************\n");
#endif
    evc_bsw_write_ue(bs, sps->sps_seq_parameter_set_id);
    evc_bsw_write(bs, sps->profile_idc, 8);
    evc_bsw_write(bs, sps->level_idc, 8);
    evc_bsw_write(bs, sps->toolset_idc_h, 32);
    evc_bsw_write(bs, sps->toolset_idc_l, 32);
    evc_bsw_write_ue(bs, sps->chroma_format_idc);
    evc_bsw_write_ue(bs, sps->pic_width_in_luma_samples);
    evc_bsw_write_ue(bs, sps->pic_height_in_luma_samples);
    evc_bsw_write_ue(bs, sps->bit_depth_luma_minus8);
    evc_bsw_write_ue(bs, sps->bit_depth_chroma_minus8);
    evc_bsw_write1(bs, sps->sps_btt_flag);
    if (sps->sps_btt_flag)
    {
        evc_bsw_write_ue(bs, sps->log2_ctu_size_minus5);
        evc_bsw_write_ue(bs, sps->log2_min_cb_size_minus2);
        evc_bsw_write_ue(bs, sps->log2_diff_ctu_max_14_cb_size);
        evc_bsw_write_ue(bs, sps->log2_diff_ctu_max_tt_cb_size);
        evc_bsw_write_ue(bs, sps->log2_diff_min_cb_min_tt_cb_size_minus2);
    }
    evc_bsw_write1(bs, sps->sps_suco_flag);
    if (sps->sps_suco_flag)
    {
        evc_bsw_write_ue(bs, sps->log2_diff_ctu_size_max_suco_cb_size);
        evc_bsw_write_ue(bs, sps->log2_diff_max_suco_min_suco_cb_size);
    }

    evc_bsw_write1(bs, sps->tool_admvp);
    if (sps->tool_admvp)
    {
        evc_bsw_write1(bs, sps->tool_affine);
        evc_bsw_write1(bs, sps->tool_amvr);
        evc_bsw_write1(bs, sps->tool_dmvr);
        evc_bsw_write1(bs, sps->tool_mmvd);
#if M53737
        evc_bsw_write1(bs, sps->tool_hmvp);
#endif
    }

    evc_bsw_write1(bs, sps->tool_eipd);
    if (sps->tool_eipd)
    {
        evc_bsw_write1(bs, sps->ibc_flag);
        if (sps->ibc_flag)
        {
            evc_bsw_write_ue(bs, (sps->ibc_log_max_size - 2));
        }
    }

    evc_bsw_write1(bs, sps->tool_cm_init);
    if (sps->tool_cm_init)
    {
        evc_bsw_write1(bs, sps->tool_adcc);
    }

    evc_bsw_write1(bs, sps->tool_iqt);
    if (sps->tool_iqt)
    {
        evc_bsw_write1(bs, sps->tool_ats);
    }
    evc_bsw_write1(bs, sps->tool_addb);
#if !DB_SPEC_ALIGNMENT2
#if M52291_HDR_DRA
    evc_bsw_write1(bs, sps->tool_dra);
#endif
#endif
    evc_bsw_write1(bs, sps->tool_alf);
    evc_bsw_write1(bs, sps->tool_htdf);
    evc_bsw_write1(bs, sps->tool_rpl);
    evc_bsw_write1(bs, sps->tool_pocs);
#if DQP
    evc_bsw_write1(bs, sps->dquant_flag);
#endif
#if DB_SPEC_ALIGNMENT2
#if M52291_HDR_DRA
    evc_bsw_write1(bs, sps->tool_dra);
#endif
#endif
    if (sps->tool_pocs)
    {
        evc_bsw_write_ue(bs, sps->log2_max_pic_order_cnt_lsb_minus4);
    }
    if (!sps->tool_rpl || !sps->tool_pocs)
    {
        evc_bsw_write_ue(bs, sps->log2_sub_gop_length);
        if (sps->log2_sub_gop_length == 0)
        {
            evc_bsw_write_ue(bs, sps->log2_ref_pic_gap_length);
        }
    }
#if !M53744
    evc_bsw_write_ue(bs, sps->sps_max_dec_pic_buffering_minus1);
#endif
    if (!sps->tool_rpl)
    {
        evc_bsw_write_ue(bs, sps->max_num_ref_pics);
    }
    else
    {
#if M53744
        evc_bsw_write_ue(bs, sps->sps_max_dec_pic_buffering_minus1);
#endif
        evc_bsw_write1(bs, sps->long_term_ref_pics_flag);
        evc_bsw_write1(bs, sps->rpl1_same_as_rpl0_flag);

        evc_bsw_write_ue(bs, sps->num_ref_pic_lists_in_sps0);
        for (int i = 0; i < sps->num_ref_pic_lists_in_sps0; ++i)
            evce_eco_rlp(bs, &sps->rpls_l0[i]);

        if (!sps->rpl1_same_as_rpl0_flag)
        {
                evc_bsw_write_ue(bs, sps->num_ref_pic_lists_in_sps1);
                for (int i = 0; i < sps->num_ref_pic_lists_in_sps1; ++i)
                evce_eco_rlp(bs, &sps->rpls_l1[i]);
        }
    }

    evc_bsw_write1(bs, sps->picture_cropping_flag);
    if (sps->picture_cropping_flag)
    {
        evc_bsw_write_ue(bs, sps->picture_crop_left_offset);
        evc_bsw_write_ue(bs, sps->picture_crop_right_offset);
        evc_bsw_write_ue(bs, sps->picture_crop_top_offset);
        evc_bsw_write_ue(bs, sps->picture_crop_bottom_offset);
    }

#if M53744
    if (sps->chroma_format_idc != 0)
#endif
    {
        evc_bsw_write1(bs, sps->chroma_qp_table_struct.chroma_qp_table_present_flag);
        if (sps->chroma_qp_table_struct.chroma_qp_table_present_flag)
        {
            evc_bsw_write1(bs, sps->chroma_qp_table_struct.same_qp_table_for_chroma);
            evc_bsw_write1(bs, sps->chroma_qp_table_struct.global_offset_flag);
            for (int i = 0; i < (sps->chroma_qp_table_struct.same_qp_table_for_chroma ? 1 : 2); i++)
            {
                evc_bsw_write_ue(bs, (u32)sps->chroma_qp_table_struct.num_points_in_qp_table_minus1[i]);
                for (int j = 0; j <= sps->chroma_qp_table_struct.num_points_in_qp_table_minus1[i]; j++)
                {
                    evc_bsw_write(bs, sps->chroma_qp_table_struct.delta_qp_in_val_minus1[i][j], 6);
                    evc_bsw_write_se(bs, (u32)sps->chroma_qp_table_struct.delta_qp_out_val[i][j]);
                }
            }
        }
    }

    evc_bsw_write1(bs, sps->vui_parameters_present_flag);
    if (sps->vui_parameters_present_flag)
#if EVC_VUI_FIX
        evce_eco_vui(bs, &(sps->vui_parameters));
#else
        evce_eco_vui(bs); //To be implemented
#endif
    u32 t0 = 0;
    while(!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, t0);
    }
#if TRACE_HLS
    EVC_TRACE_STR("************ SPS End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}

int evce_eco_pps(EVC_BSW * bs, EVC_SPS * sps, EVC_PPS * pps)
{
#if TRACE_HLS
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ PPS Start ************\n");
#endif
    evc_bsw_write_ue(bs, pps->pps_pic_parameter_set_id);
    evc_bsw_write_ue(bs, pps->pps_seq_parameter_set_id);
    evc_bsw_write_ue(bs, pps->num_ref_idx_default_active_minus1[0]);
    evc_bsw_write_ue(bs, pps->num_ref_idx_default_active_minus1[1]);
    evc_bsw_write_ue(bs, pps->additional_lt_poc_lsb_len);
    evc_bsw_write1(bs, pps->rpl1_idx_present_flag);
    evc_bsw_write1(bs, pps->single_tile_in_pic_flag);

    if (!pps->single_tile_in_pic_flag)
    {
        evc_bsw_write_ue(bs, pps->num_tile_columns_minus1);
        evc_bsw_write_ue(bs, pps->num_tile_rows_minus1);
        evc_bsw_write1(bs, pps->uniform_tile_spacing_flag);
        if (!pps->uniform_tile_spacing_flag)
        {
            for (int i = 0; i < pps->num_tile_columns_minus1; ++i)
            {
                evc_bsw_write_ue(bs, pps->tile_column_width_minus1[i]);
            }
            for (int i = 0; i < pps->num_tile_rows_minus1; ++i)
            {
                evc_bsw_write_ue(bs, pps->tile_row_height_minus1[i]);
            }
        }
        evc_bsw_write1(bs, pps->loop_filter_across_tiles_enabled_flag);
        evc_bsw_write_ue(bs, pps->tile_offset_lens_minus1);
    }

    evc_bsw_write_ue(bs, pps->tile_id_len_minus1);
    evc_bsw_write1(bs, pps->explicit_tile_id_flag);
    if (pps->explicit_tile_id_flag)
    {
        for (int i = 0; i <= pps->num_tile_rows_minus1; ++i)
        {
            for (int j = 0; j <= pps->num_tile_columns_minus1; ++j)
            {
                evc_bsw_write(bs, pps->tile_id_val[i][j], pps->tile_id_len_minus1 + 1);
            }
        }
    }

#if M52291_HDR_DRA
    evc_bsw_write1(bs, pps->pic_dra_enabled_flag);
#if ETM60_HLS_FIX
    if (pps->pic_dra_enabled_flag)
    {
        evc_bsw_write(bs, pps->pic_dra_aps_id, APS_MAX_NUM_IN_BITS);
    }
#else
    if (sps->tool_dra)
    {
        evc_bsw_write1(bs, pps->pic_dra_enabled_present_flag);
#if DB_SPEC_ALIGNMENT2
        if (pps->pic_dra_enabled_present_flag) {
            evc_bsw_write1(bs, pps->pic_dra_enabled_flag);
            if (pps->pic_dra_enabled_flag)
                evc_bsw_write(bs, pps->pic_dra_aps_id, APS_TYPE_ID_BITS);
        }
#else
        evc_bsw_write1(bs, pps->pic_dra_enabled_flag);
        evc_bsw_write(bs, pps->pic_dra_aps_id, APS_TYPE_ID_BITS);
#endif
    }
#endif
#endif
    evc_bsw_write1(bs, pps->arbitrary_slice_present_flag);
    evc_bsw_write1(bs, pps->constrained_intra_pred_flag); 

#if DQP

    evc_bsw_write1(bs, pps->cu_qp_delta_enabled_flag);
    if (pps->cu_qp_delta_enabled_flag)
    {
        evc_bsw_write_ue(bs, pps->cu_qp_delta_area - 6);
    }
#endif
    u32 t0 = 0;
    while (!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, t0);
    }
#if TRACE_HLS    
    EVC_TRACE_STR("************ PPS End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}

#if M52291_HDR_DRA
int evce_eco_aps_gen(EVC_BSW * bs, EVC_APS_GEN * aps
#if BD_CF_EXT
                     , int bit_depth
#endif
)
{
#if TRACE_HLS        
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ APS Start ************\n");
    u32 aps_id = aps->aps_id;
    u32 aps_type_id = aps->aps_type_id;
    evc_bsw_write(bs, aps_id, APS_MAX_NUM_IN_BITS); // signal APS ID
    evc_bsw_write(bs, aps_type_id, APS_TYPE_ID_BITS); // signal APS TYPE ID
#else
    evc_bsw_write(bs, aps->aps_id, APS_MAX_NUM_IN_BITS); // signal APS ID
    evc_bsw_write(bs, aps->aps_type_id, APS_TYPE_ID_BITS); // signal APS TYPE ID
#endif
    if (aps->aps_type_id == 0)
    {
        EVC_APS local_aps;
        evc_AlfSliceParam * p_aps_dataDst = (evc_AlfSliceParam *)aps->aps_data;
        memcpy(&(local_aps.alf_aps_param), p_aps_dataDst, sizeof(evc_AlfSliceParam));
        evce_eco_alf_aps_param(bs, aps); // signal ALF filter parameter except ALF map
    }
    else if (aps->aps_type_id == 1)
        evce_eco_dra_aps_param(bs, aps
#if BD_CF_EXT
                               , bit_depth
#endif
        );
    else
        printf("This version of ETM doesnot support this APS type: %d\n", aps->aps_type_id);

    u8 aps_extension_flag = 0;
    evc_bsw_write1(bs, aps_extension_flag);

    assert(aps_extension_flag == 0);
    if (aps_extension_flag)
    {
        while (0/*more_rbsp_data()*/)
        {
            u8 aps_extension_data_flag;
            evc_bsw_write1(bs, aps_extension_data_flag);
        }
    }

    u32 t0 = 0;
    while (!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, t0);
    }
#if TRACE_HLS    
    EVC_TRACE_STR("************ APS End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}
#else
int evce_eco_aps(EVC_BSW * bs, EVC_APS * aps)
{
    evc_bsw_write(bs, aps->aps_id, APS_MAX_NUM_IN_BITS); // signal APS ID
    evce_eco_alf_aps_param(bs, aps); // signal ALF filter parameter except ALF map

    u8 aps_extension_flag = 0;
    evc_bsw_write1(bs, aps_extension_flag);

    // Dmytro: This is a temporal solution for: Decoder confirming EVC specification version 1, shall not depend on the value of aps_extension_data_flag. 
    // Assert shall be removed after implementing more_rbsp_data function
    assert(aps_extension_flag == 0);
    if (aps_extension_flag)
    {
        while (0/*more_rbsp_data()*/)
        {
            u8 aps_extension_data_flag;
            evc_bsw_write1(bs, aps_extension_data_flag);
        }
    }

    while (!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, 0);
    }

    return EVC_OK;
}
#endif

int evce_eco_sh(EVC_BSW * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_SH * sh, int nut)
{
#if TRACE_HLS    
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ SH  Start ************\n");    
#endif
    int num_tiles_in_slice;
    if (!sh->arbitrary_slice_flag)
    {
        num_tiles_in_slice = sh->num_tiles_in_slice; 
    }
    else
    {
        num_tiles_in_slice = sh->num_remaining_tiles_in_slice_minus1 + 2;
    }

    evc_bsw_write_ue(bs, sh->slice_pic_parameter_set_id);
#if M53744
    if (!pps->single_tile_in_pic_flag)
#endif
    {
        evc_bsw_write1(bs, sh->single_tile_in_slice_flag);
        evc_bsw_write(bs, sh->first_tile_id, pps->tile_id_len_minus1 + 1);
    }

    if (!sh->single_tile_in_slice_flag)
    {
        if (pps->arbitrary_slice_present_flag)
        {
            evc_bsw_write1(bs, sh->arbitrary_slice_flag);
        }
        if (!sh->arbitrary_slice_flag)
        {
            evc_bsw_write(bs, sh->last_tile_id, pps->tile_id_len_minus1 + 1);
        }
        else
        {
            evc_bsw_write_ue(bs, sh->num_remaining_tiles_in_slice_minus1);
            for (int i = 0; i < num_tiles_in_slice - 1; ++i)
            {
                evc_bsw_write_ue(bs, sh->delta_tile_id_minus1[i]);
            }
        }
    }

    evc_bsw_write_ue(bs, sh->slice_type);

    if (nut == EVC_IDR_NUT)
    {
        evc_bsw_write1(bs, sh->no_output_of_prior_pics_flag);
    }

    if (sps->tool_mmvd && (sh->slice_type == SLICE_B))
    {
        evc_bsw_write1(bs, sh->mmvd_group_enable_flag);
    }
    else if (sps->tool_mmvd && (sh->slice_type == SLICE_P))
    {
        evc_bsw_write1(bs, sh->mmvd_group_enable_flag);
    }

    if (sps->tool_alf)
    {
        evc_bsw_write1(bs, sh->alf_on);
        if (sh->alf_on)
        {
            evc_bsw_write(bs, sh->aps_id_y, APS_MAX_NUM_IN_BITS);
            evce_eco_alf_sh_param(bs, sh); // signaling ALF map
#if M53608_ALF_14
            sh->alfChromaIdc = ((sh->alf_sh_param.enabledFlag[2]) << 1) + sh->alf_sh_param.enabledFlag[1];
            evc_bsw_write(bs, sh->alfChromaIdc, 2);
            if (sh->alfChromaIdc == 1)
            {
                sh->ChromaAlfEnabledFlag = 1;
                sh->ChromaAlfEnabled2Flag = 0;
            }
            else if (sh->alfChromaIdc == 2)
            {
                sh->ChromaAlfEnabledFlag = 0;
                sh->ChromaAlfEnabled2Flag = 1;
            }
            else if (sh->alfChromaIdc == 3)
            {
                sh->ChromaAlfEnabledFlag = 1;
                sh->ChromaAlfEnabled2Flag = 1;
            }
            else
            {
                sh->ChromaAlfEnabledFlag = 0;
                sh->ChromaAlfEnabled2Flag = 0;
            }
#endif
#if M53608_ALF_14
            if (sh->alfChromaIdc && (sps->chroma_format_idc == 1 || sps->chroma_format_idc == 2))
            {
#endif
                evc_bsw_write(bs, sh->aps_id_ch, APS_MAX_NUM_IN_BITS);
#if M53608_ALF_14
            }
#endif
        }
#if M53608_ALF_14
        if (sps->chroma_format_idc == 3 && sh->ChromaAlfEnabledFlag)
        {
            evc_bsw_write(bs, sh->aps_id_ch, APS_MAX_NUM_IN_BITS);
            evc_bsw_write1(bs, sh->alfChromaMapSignalled);
        }
        if (sps->chroma_format_idc == 3 && sh->ChromaAlfEnabled2Flag)
        {
            evc_bsw_write(bs, sh->aps_id_ch2, APS_MAX_NUM_IN_BITS);
            evc_bsw_write1(bs, sh->alfChroma2MapSignalled);
        }
#endif
    }

    if (nut != EVC_IDR_NUT)
    {
        if (sps->tool_pocs)
        {
            evc_bsw_write(bs, sh->poc_lsb, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
        }
        if (sps->tool_rpl)
        {
            //L0 candidates signaling
            if (sps->num_ref_pic_lists_in_sps0 > 0)
            {
                evc_bsw_write1(bs, sh->ref_pic_list_sps_flag[0]);
            }
            if (sh->ref_pic_list_sps_flag[0])
            {
                if (sps->num_ref_pic_lists_in_sps0 > 1)
                {
                    evc_bsw_write_ue(bs, sh->rpl_l0_idx);
                }
            }
            else
            {
                evce_eco_rlp(bs, &sh->rpl_l0);
            }

            //L1 candidates signaling
            if (sps->num_ref_pic_lists_in_sps1 > 0 && pps->rpl1_idx_present_flag)
            {
                evc_bsw_write1(bs, sh->ref_pic_list_sps_flag[1]);
            }

            if (sh->ref_pic_list_sps_flag[1])
            {
                if (sps->num_ref_pic_lists_in_sps1 > 1 && pps->rpl1_idx_present_flag)
                {
                    evc_bsw_write_ue(bs, sh->rpl_l1_idx);
                }
            }
            else
            {
                evce_eco_rlp(bs, &sh->rpl_l1);
            }
        }
    }

    if (sh->slice_type != SLICE_I)
    {
        evc_bsw_write1(bs, sh->num_ref_idx_active_override_flag);
        if (sh->num_ref_idx_active_override_flag)
        {
            u32 num_ref_idx_active_minus1 = sh->rpl_l0.ref_pic_active_num - 1;
            evc_bsw_write_ue(bs, num_ref_idx_active_minus1);
            if (sh->slice_type == SLICE_B)
            {
                num_ref_idx_active_minus1 = sh->rpl_l1.ref_pic_active_num - 1;
                evc_bsw_write_ue(bs, num_ref_idx_active_minus1);
            }
        }

        if (sps->tool_admvp)
        {
            evc_bsw_write1(bs, sh->temporal_mvp_asigned_flag);
            if (sh->temporal_mvp_asigned_flag)
            {
                if (sh->slice_type == SLICE_B)
                {
                    evc_bsw_write1(bs, sh->collocated_from_list_idx);
                    evc_bsw_write1(bs, sh->collocated_mvp_source_list_idx);
                }
                evc_bsw_write1(bs, sh->collocated_from_ref_idx);
            }
        }
    }
    evc_bsw_write1(bs, sh->deblocking_filter_on);

    if(sh->deblocking_filter_on && sps->tool_addb)
    {
        evc_bsw_write_se(bs, sh->sh_deblock_alpha_offset);
        evc_bsw_write_se(bs, sh->sh_deblock_beta_offset);
    }

    evc_bsw_write(bs, sh->qp, 6);
    evc_bsw_write_se(bs, sh->qp_u_offset);
    evc_bsw_write_se(bs, sh->qp_v_offset);

    if (!sh->single_tile_in_slice_flag)
    {
        for (int i = 0; i < num_tiles_in_slice - 1; ++i)
        {
            evc_bsw_write(bs, sh->entry_point_offset_minus1[i], pps->tile_offset_lens_minus1 + 1);
        }
    }

    /* byte align */
    u32 t0 = 0;
    while(!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, t0);
    }
#if TRACE_HLS    
    EVC_TRACE_STR("************ SH  End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}

int evce_eco_signature(EVCE_CTX * ctx, EVC_BSW * bs)
{
    if (ctx->param.use_pic_sign)
    {
        int ret;
        u8 pic_sign[N_C][16] = { {0} };

        /* get picture signature */
#if HDR_MD5_CHECK
        if (ctx->pps.pic_dra_enabled_flag == 0)
        {
#endif
            ret = evc_picbuf_signature(PIC_CURR(ctx), pic_sign);
            evc_assert_rv(ret == EVC_OK, ret);
#if HDR_MD5_CHECK
        }
#endif
#if HDR_MD5_CHECK
        else
        {
            ret = evce_eco_udata_hdr(ctx, bs, pic_sign);
            evc_assert_rv(ret == EVC_OK, ret);
        }
#endif
        u32 payload_type = EVC_UD_PIC_SIGNATURE;
        u32 payload_size = 16;

        evc_bsw_write(bs, payload_type, 8);
        evc_bsw_write(bs, payload_size, 8);

        for (int i = 0; i < ctx->pic[0]->imgb->np; ++i)
        {
            for (int j = 0; j < payload_size; j++)
            {
                evc_bsw_write(bs, pic_sign[i][j], 8);
            }
        }
    }

    return EVC_OK;
}

int evce_eco_sei(EVCE_CTX * ctx, EVC_BSW * bs)
{
    evc_assert_rv(EVC_BSW_IS_BYTE_ALIGN(bs), EVC_ERR_UNKNOWN);

    evce_eco_signature(ctx, bs);

    while (!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, 0);
    }

    return EVC_OK;
}
#if HDR_MD5_CHECK
static void __imgb_cpy_plane(void *src, void *dst, int bw, int h, int s_src,
    int s_dst)
{
    int i;
    unsigned char *s, *d;

    s = (unsigned char*)src;
    d = (unsigned char*)dst;

    for (i = 0; i < h; i++)
    {
        memcpy(d, s, bw);
        s += s_src;
        d += s_dst;
    }
}
#define EVCA_CLIP(n,min,max) (((n)>(max))? (max) : (((n)<(min))? (min) : (n)))
static void imgb_conv_8b_to_16b(EVC_IMGB * imgb_dst, EVC_IMGB * imgb_src,
    int shift)
{
    int i, j, k;

    unsigned char * s;
    short         * d;

    for (i = 0; i < 3; i++)
    {
        s = imgb_src->a[i];
        d = imgb_dst->a[i];

        for (j = 0; j < imgb_src->h[i]; j++)
        {
            for (k = 0; k < imgb_src->w[i]; k++)
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

    for (i = 0; i < 3; i++)
    {
        s = imgb_src->a[i];
        d = imgb_dst->a[i];

        for (j = 0; j < imgb_src->h[i]; j++)
        {
            for (k = 0; k < imgb_src->w[i]; k++)
            {
                t0 = ((s[k] + add) >> shift);
                d[k] = (unsigned char)(EVCA_CLIP(t0, 0, 255));

            }
            s = (short*)(((unsigned char *)s) + imgb_src->s[i]);
            d = d + imgb_dst->s[i];
        }
    }
}
static void imgb_cpy(EVC_IMGB * dst, EVC_IMGB * src)
{
    int i, bd;

    if (src->cs == dst->cs)
    {
        if (src->cs == EVC_COLORSPACE_YUV420_10LE) bd = 2;
        else bd = 1;

        for (i = 0; i < src->np; i++)
        {
            __imgb_cpy_plane(src->a[i], dst->a[i], bd*src->w[i], src->h[i],
                src->s[i], dst->s[i]);
        }
    }
    else if (src->cs == EVC_COLORSPACE_YUV420 &&
        dst->cs == EVC_COLORSPACE_YUV420_10LE)
    {
        imgb_conv_8b_to_16b(dst, src, 2);
    }
    else if (src->cs == EVC_COLORSPACE_YUV420_10LE &&
        dst->cs == EVC_COLORSPACE_YUV420)
    {
        imgb_conv_16b_to_8b(dst, src, 2);
    }
    else
    {
        printf("ERROR: unsupported image copy\n");
        return;
    }
    for (i = 0; i < 4; i++)
    {
        dst->ts[i] = src->ts[i];
    }
}
static void imgb_free1(EVC_IMGB * imgb)
{
    int i;
    for (i = 0; i < EVC_IMGB_MAX_PLANE; i++)
    {
        if (imgb->baddr[i]) free(imgb->baddr[i]);
    }
    free(imgb);
}

EVC_IMGB * imgb_alloc1(int w, int h, int cs)
{
    int i;
    EVC_IMGB * imgb;

    imgb = (EVC_IMGB *)malloc(sizeof(EVC_IMGB));
    if (imgb == NULL)
    {
        printf("cannot create image buffer\n");
        return NULL;
    }
    memset(imgb, 0, sizeof(EVC_IMGB));

    if (cs == EVC_COLORSPACE_YUV420)
    {
        for (i = 0; i < 3; i++)
        {
            imgb->w[i] = imgb->aw[i] = imgb->s[i] = w;
            imgb->h[i] = imgb->ah[i] = imgb->e[i] = h;
            imgb->bsize[i] = imgb->s[i] * imgb->e[i];

            imgb->a[i] = imgb->baddr[i] = malloc(imgb->bsize[i]);
            if (imgb->a[i] == NULL)
            {
                printf("cannot allocate picture buffer\n");
                return NULL;
            }

            if (i == 0)
            {
                w = (w + 1) >> 1; h = (h + 1) >> 1;
            }
        }
        imgb->np = 3;
    }
    else if (cs == EVC_COLORSPACE_YUV420_10LE)
    {
        for (i = 0; i < 3; i++)
        {
            imgb->w[i] = imgb->aw[i] = w;
            imgb->s[i] = w * sizeof(short);
            imgb->h[i] = imgb->ah[i] = imgb->e[i] = h;
            imgb->bsize[i] = imgb->s[i] * imgb->e[i];

            imgb->a[i] = imgb->baddr[i] = malloc(imgb->bsize[i]);
            if (imgb->a[i] == NULL)
            {
                printf("cannot allocate picture buffer\n");
                return NULL;
            }

            if (i == 0)
            {
                w = (w + 1) >> 1; h = (h + 1) >> 1;
            }
        }
        imgb->np = 3;
    }
    else if (cs == EVC_COLORSPACE_YUV444_10LE)
    {
        for (i = 0; i < 3; i++)
        {
            imgb->w[i] = imgb->aw[i] = w;
            imgb->s[i] = w * sizeof(float);
            imgb->h[i] = imgb->ah[i] = imgb->e[i] = h;
            imgb->bsize[i] = imgb->s[i] * imgb->e[i];

            imgb->a[i] = imgb->baddr[i] = malloc(imgb->bsize[i]);
            if (imgb->a[i] == NULL)
            {
                printf("cannot allocate picture buffer\n");
                return NULL;
            }
        }
        imgb->np = 3;
    }
    else
    {
        printf("unsupported color space\n");
        if (imgb)free(imgb);
        return NULL;
    }

    imgb->cs = cs;
    return imgb;
}
int evce_eco_udata_hdr(EVCE_CTX * ctx, EVC_BSW * bs, u8 pic_sign[N_C][16])
{
    int ret;
    EVC_IMGB *imgb_hdr_md5 = NULL;
    imgb_hdr_md5 = imgb_alloc1(PIC_CURR(ctx)->imgb->w[0], PIC_CURR(ctx)->imgb->h[0],
        EVC_COLORSPACE_YUV420_10LE);

    imgb_cpy(imgb_hdr_md5, PIC_CURR(ctx)->imgb);  // store copy of the reconstructed picture in DPB

    int effective_aps_id = ctx->pico->pic.imgb->imgb_active_aps_id;
    assert(effective_aps_id == ctx->pps.pic_dra_aps_id);
    assert(effective_aps_id >= 0 && effective_aps_id < APS_MAX_NUM);
    SignalledParamsDRA *p_pps_draParams = (SignalledParamsDRA *)ctx->g_void_dra_array;
    evc_apply_dra_from_array(imgb_hdr_md5, imgb_hdr_md5, &(p_pps_draParams[0]), effective_aps_id, TRUE);

    /* should be aligned before adding user data */
    evc_assert_rv(EVC_BSW_IS_BYTE_ALIGN(bs), EVC_ERR_UNKNOWN);

    /* picture signature */
    if (ctx->param.use_pic_sign)
    {
        /* get picture signature */
        ret = evc_md5_imgb(imgb_hdr_md5, pic_sign);
        evc_assert_rv(ret == EVC_OK, ret);
    }
    imgb_free1(imgb_hdr_md5);
    return EVC_OK;
}
#endif
static void evc_bsw_write_est(EVCE_SBAC *sbac, u32 byte, int len)
{
    sbac->bitcounter += len;
}

static void sbac_put_byte(u8 writing_byte, EVCE_SBAC *sbac, EVC_BSW *bs)
{
    if(sbac->is_pending_byte)
    {
        if(sbac->pending_byte == 0)
        {
            sbac->stacked_zero++;
        }
        else
        {
            while(sbac->stacked_zero > 0)
            {
                if(sbac->is_bitcount)
                    evc_bsw_write_est(sbac, 0x00, 8);
                else
#if TRACE_HLS
                    evc_bsw_write_trace(bs, 0x00, 0, 8);
#else
                    evc_bsw_write(bs, 0x00, 8);
#endif
                sbac->stacked_zero--;
            }
            if(sbac->is_bitcount)
                evc_bsw_write_est(sbac, sbac->pending_byte, 8);
            else
#if TRACE_HLS
                evc_bsw_write_trace(bs, sbac->pending_byte, 0, 8);
#else
                evc_bsw_write(bs, sbac->pending_byte, 8);
#endif
        }
    }
    sbac->pending_byte = writing_byte;
    sbac->is_pending_byte = 1;
}

static void sbac_carry_propagate(EVCE_SBAC *sbac, EVC_BSW *bs)
{
    u32 out_bits = sbac->code >> 17;
    
    sbac->code &= (1 << 17) - 1;

    if(out_bits < 0xFF)
    {
        while(sbac->stacked_ff != 0)
        {
            sbac_put_byte(0xFF, sbac, bs);
            sbac->stacked_ff--;
        }
        sbac_put_byte(out_bits, sbac, bs);
    }
    else if(out_bits > 0xFF)
    {
        sbac->pending_byte++;
        while(sbac->stacked_ff != 0)
        {
            sbac_put_byte(0x00, sbac, bs);
            sbac->stacked_ff--;
        }
        sbac_put_byte(out_bits & 0xFF, sbac, bs);
    }
    else
    {
        sbac->stacked_ff++;
    }
}

static void sbac_encode_bin_ep(u32 bin, EVCE_SBAC *sbac, EVC_BSW *bs)
{
#if CABAC_ZERO_WORD // increase bin counter
    sbac->bin_counter++;
#endif 

    (sbac->range) >>= 1;

    if(bin != 0)
    {
        (sbac->code) += (sbac->range);
    }

    (sbac->range) <<= 1;
    (sbac->code) <<= 1;

    if(--(sbac->code_bits) == 0)
    {
        sbac_carry_propagate(sbac, bs);
        sbac->code_bits = 8;
    }
}

static void sbac_write_unary_sym_ep(u32 sym, EVCE_SBAC *sbac, EVC_BSW *bs, u32 max_val)
{
    u32 icounter = 0;

    sbac_encode_bin_ep(sym ? 1 : 0, sbac, bs); icounter++;

    if(sym == 0)
    {
        return;
    }

    while(sym--)
    {
        if(icounter < max_val)
        {
            sbac_encode_bin_ep(sym ? 1 : 0, sbac, bs); icounter++;
        }
    }
}

static void sbac_write_unary_sym(u32 sym, u32 num_ctx, EVCE_SBAC *sbac, SBAC_CTX_MODEL *model, EVC_BSW *bs)
{
    u32 ctx_idx = 0;

    evce_sbac_encode_bin(sym ? 1 : 0, sbac, model, bs);

    if(sym == 0)
    {
        return;
    }

    while(sym--)
    {
        if(ctx_idx < num_ctx - 1)
        {
            ctx_idx++;
        }
        evce_sbac_encode_bin(sym ? 1 : 0, sbac, &model[ctx_idx], bs);
    }
}

static void sbac_write_truncate_unary_sym(u32 sym, u32 num_ctx, u32 max_num, EVCE_SBAC *sbac, SBAC_CTX_MODEL *model, EVC_BSW *bs)
{
    u32 ctx_idx = 0;
    int symbol = 0;

    if(max_num > 1)
    {
        for(ctx_idx = 0; ctx_idx < max_num - 1; ++ctx_idx)
        {
            symbol = (ctx_idx == sym) ? 0 : 1;
            evce_sbac_encode_bin(symbol, sbac, model + (ctx_idx > max_num - 1 ? max_num - 1 : ctx_idx), bs);

            if(symbol == 0)
                break;
        }
    }
}

static void sbac_encode_bins_ep(u32 value, int num_bin, EVCE_SBAC *sbac, EVC_BSW *bs)
{
    int bin = 0;
    for(bin = num_bin - 1; bin >= 0; bin--)
    {
        sbac_encode_bin_ep(value & (1 << (u32)bin), sbac, bs);
    }
}

void evce_sbac_encode_bin(u32 bin, EVCE_SBAC *sbac, SBAC_CTX_MODEL *model, EVC_BSW *bs)
{
    u32 lps;
    u16 mps, state;

#if CABAC_ZERO_WORD // increase bin counter
    sbac->bin_counter++;
#endif 

    state = (*model) >> 1;
    mps = (*model) & 1;

    lps = (state * (sbac->range)) >> 9;
    lps = lps < 437 ? 437 : lps;    

    sbac->range -= lps;

#if TRACE_BIN
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("model ");
    EVC_TRACE_INT(*model);
    EVC_TRACE_STR("range ");
    EVC_TRACE_INT(sbac->range);
    EVC_TRACE_STR("lps ");
    EVC_TRACE_INT(lps);
    EVC_TRACE_STR("\n");
#endif

    if(bin != mps)
    {
        if(sbac->range >= lps)
        {
            sbac->code += sbac->range;
            sbac->range = lps;
        }

        state = state + ((512 - state + 16) >> 5);
        if(state > 256)
        {
            mps = 1 - mps; 
            state = 512 - state;
        }
        *model = (state << 1) + mps;
    }
    else
    {
        state = state - ((state + 16) >> 5);
        *model = (state << 1) + mps;
    }

    while(sbac->range < 8192)
    {
        sbac->range <<= 1;
        sbac->code <<= 1;
        sbac->code_bits--;

        if(sbac->code_bits == 0)
        {
            sbac_carry_propagate(sbac, bs);
            sbac->code_bits = 8;
        }
    }
}

void evce_sbac_encode_bin_trm(u32 bin, EVCE_SBAC *sbac, EVC_BSW *bs)
{
#if CABAC_ZERO_WORD // increase bin counter
    sbac->bin_counter++;
#endif 

    sbac->range--;

    if(bin)
    {
        sbac->code += sbac->range;
        sbac->range = 1;
    }
   
    while(sbac->range < 8192)
    {
        sbac->range <<= 1;
        sbac->code <<= 1;
        if(--(sbac->code_bits) == 0)
        {
            sbac_carry_propagate(sbac, bs);
            sbac->code_bits = 8;
        }
    }
}

void evce_sbac_reset(EVCE_SBAC *sbac, u8 slice_type, u8 slice_qp, int sps_cm_init_flag)
{
    EVC_SBAC_CTX *sbac_ctx;
    sbac_ctx = &sbac->ctx;

    /* Initialization of the internal variables */
    sbac->range = 16384;
    sbac->code = 0;
    sbac->code_bits = 11;
    sbac->pending_byte = 0;
    sbac->is_pending_byte = 0;
    sbac->stacked_ff = 0;
    sbac->stacked_zero = 0;
#if CABAC_ZERO_WORD // init Bin counter
    sbac->bin_counter = 0;
#endif 

    evc_mset(sbac_ctx, 0x00, sizeof(*sbac_ctx));

    sbac_ctx->sps_cm_init_flag = sps_cm_init_flag;

    /* Initialization of the context models */
    if(sps_cm_init_flag == 1)
    {
        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf_luma, (s16*)init_cbf_luma, NUM_CTX_CBF_LUMA, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf_cb, (s16*)init_cbf_cb, NUM_CTX_CBF_CB, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf_cr, (s16*)init_cbf_cr, NUM_CTX_CBF_CR, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf_all, (s16*)init_cbf_all, NUM_CTX_CBF_ALL, slice_type, slice_qp);
#if DQP
        evc_eco_sbac_ctx_initialize(sbac_ctx->delta_qp, (s16*)init_dqp, NUM_CTX_DELTA_QP, slice_type, slice_qp);
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->sig_coeff_flag, (s16*)init_sig_coeff_flag, NUM_CTX_SIG_COEFF_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->coeff_abs_level_greaterAB_flag, (s16*)init_coeff_abs_level_greaterAB_flag, NUM_CTX_GTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->last_sig_coeff_x_prefix, (s16*)init_last_sig_coeff_x_prefix, NUM_CTX_LAST_SIG_COEFF, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->last_sig_coeff_y_prefix, (s16*)init_last_sig_coeff_y_prefix, NUM_CTX_LAST_SIG_COEFF, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->pred_mode, (s16*)init_pred_mode, NUM_CTX_PRED_MODE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mode_cons, (s16*)init_mode_cons, NUM_CTX_MODE_CONS, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->direct_mode_flag, (s16*)init_direct_mode_flag, NUM_CTX_DIRECT_MODE_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->merge_mode_flag, (s16*)init_merge_mode_flag, NUM_CTX_MERGE_MODE_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->inter_dir, (s16*)init_inter_dir, NUM_CTX_INTER_PRED_IDC, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->intra_dir, (s16*)init_intra_dir, NUM_CTX_INTRA_PRED_MODE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->intra_luma_pred_mpm_flag, (s16*)init_intra_luma_pred_mpm_flag, NUM_CTX_INTRA_LUMA_PRED_MPM_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->intra_luma_pred_mpm_idx, (s16*)init_intra_luma_pred_mpm_idx, NUM_CTX_INTRA_LUMA_PRED_MPM_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->intra_chroma_pred_mode, (s16*)init_intra_chroma_pred_mode, NUM_CTX_INTRA_CHROMA_PRED_MODE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->run, (s16*)init_run, NUM_CTX_CC_RUN, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->last, (s16*)init_last, NUM_CTX_CC_LAST, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->level, (s16*)init_level, NUM_CTX_CC_LEVEL, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_flag, (s16*)init_mmvd_flag, NUM_CTX_MMVD_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_merge_idx, (s16*)init_mmvd_merge_idx, NUM_CTX_MMVD_MERGE_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_distance_idx, (s16*)init_mmvd_distance_idx, NUM_CTX_MMVD_DIST_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_direction_idx, (s16*)init_mmvd_direction_idx, NUM_CTX_MMVD_DIRECTION_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_group_idx, (s16*)init_mmvd_group_idx, NUM_CTX_MMVD_GROUP_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->merge_idx, (s16*)init_merge_idx, NUM_CTX_MERGE_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvp_idx, (s16*)init_mvp_idx, NUM_CTX_MVP_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvp_idx, (s16*)init_affine_mvp_idx, NUM_CTX_AFFINE_MVP_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvr_idx, (s16*)init_mvr_idx, NUM_CTX_AMVR_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->bi_idx, (s16*)init_bi_idx, NUM_CTX_BI_PRED_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvd, (s16*)init_mvd, NUM_CTX_MVD, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->refi, (s16*)init_refi, NUM_CTX_REF_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_flag, (s16*)init_btt_split_flag, NUM_CTX_BTT_SPLIT_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_dir, (s16*)init_btt_split_dir, NUM_CTX_BTT_SPLIT_DIR, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_type, (s16*)init_btt_split_type, NUM_CTX_BTT_SPLIT_TYPE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->suco_flag, (s16*)init_suco_flag, NUM_CTX_SUCO_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->alf_ctb_flag, (s16*)init_alf_ctb_flag, NUM_CTX_ALF_CTB_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->split_cu_flag, (s16*)init_split_cu_flag, NUM_CTX_SPLIT_CU_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_flag, (s16*)init_affine_flag, NUM_CTX_AFFINE_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mode, (s16*)init_affine_mode, NUM_CTX_AFFINE_MODE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mrg, (s16*)init_affine_mrg, NUM_CTX_AFFINE_MRG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvd_flag, (s16*)init_affine_mvd_flag, NUM_CTX_AFFINE_MVD_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->skip_flag, (s16*)init_skip_flag, NUM_CTX_SKIP_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ibc_flag, (s16*)init_ibc_flag, NUM_CTX_IBC_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_mode, (s16*)init_ats_mode, NUM_CTX_ATS_MODE_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_cu_inter_flag, (s16*)init_ats_cu_inter_flag, NUM_CTX_ATS_INTER_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_cu_inter_quad_flag, (s16*)init_ats_cu_inter_quad_flag, NUM_CTX_ATS_INTER_QUAD_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_cu_inter_hor_flag, (s16*)init_ats_cu_inter_hor_flag, NUM_CTX_ATS_INTER_HOR_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_cu_inter_pos_flag, (s16*)init_ats_cu_inter_pos_flag, NUM_CTX_ATS_INTER_POS_FLAG, slice_type, slice_qp);
    }
    else // (sps_cm_init_flag == 0)
    {
        int i;

        for(i = 0; i < NUM_CTX_ALF_CTB_FLAG; i++) sbac_ctx->alf_ctb_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_SPLIT_CU_FLAG; i++) sbac_ctx->split_cu_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CC_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CC_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CC_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CBF_LUMA; i++) sbac_ctx->cbf_luma[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CBF_CB; i++) sbac_ctx->cbf_cb[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CBF_CR; i++) sbac_ctx->cbf_cr[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_CBF_ALL; i++) sbac_ctx->cbf_all[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_SIG_COEFF_FLAG; i++) sbac_ctx->sig_coeff_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_GTX; i++) sbac_ctx->coeff_abs_level_greaterAB_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_LAST_SIG_COEFF; i++) sbac_ctx->last_sig_coeff_x_prefix[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_LAST_SIG_COEFF; i++) sbac_ctx->last_sig_coeff_y_prefix[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_PRED_MODE; i++) sbac_ctx->pred_mode[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MODE_CONS; i++) sbac_ctx->mode_cons[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_DIRECT_MODE_FLAG; i++) sbac_ctx->direct_mode_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MERGE_MODE_FLAG; i++) sbac_ctx->merge_mode_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_INTER_PRED_IDC; i++) sbac_ctx->inter_dir[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_INTRA_PRED_MODE; i++) sbac_ctx->intra_dir[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_INTRA_LUMA_PRED_MPM_FLAG; i++) sbac_ctx->intra_luma_pred_mpm_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_INTRA_LUMA_PRED_MPM_IDX; i++) sbac_ctx->intra_luma_pred_mpm_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_INTRA_CHROMA_PRED_MODE; i++) sbac_ctx->intra_chroma_pred_mode[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MMVD_FLAG; i++) sbac_ctx->mmvd_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MMVD_MERGE_IDX; i++) sbac_ctx->mmvd_merge_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MMVD_DIST_IDX; i++) sbac_ctx->mmvd_distance_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MMVD_DIRECTION_IDX; i++) sbac_ctx->mmvd_direction_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MMVD_GROUP_IDX; i++) sbac_ctx->mmvd_group_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MERGE_IDX; i++) sbac_ctx->merge_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MVP_IDX; i++) sbac_ctx->mvp_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_AMVR_IDX; i++) sbac_ctx->mvr_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_BI_PRED_IDX; i++) sbac_ctx->bi_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_MVD; i++)  sbac_ctx->mvd[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_REF_IDX; i++)   sbac_ctx->refi[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_BTT_SPLIT_FLAG; i++) sbac_ctx->btt_split_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_BTT_SPLIT_DIR; i++) sbac_ctx->btt_split_dir[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_BTT_SPLIT_TYPE; i++) sbac_ctx->btt_split_type[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_SUCO_FLAG; i++) sbac_ctx->suco_flag[i] = PROB_INIT;
#if DQP
        for(i = 0; i < NUM_CTX_DELTA_QP; i++) sbac_ctx->delta_qp[i] = PROB_INIT;
#endif
        for(i = 0; i < NUM_CTX_AFFINE_FLAG; i++) sbac_ctx->affine_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_AFFINE_MODE; i++) sbac_ctx->affine_mode[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_AFFINE_MRG; i++) sbac_ctx->affine_mrg[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_AFFINE_MVP_IDX; i++) sbac_ctx->affine_mvp_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_AFFINE_MVD_FLAG; i++) sbac_ctx->affine_mvd_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_SKIP_FLAG; i++) sbac_ctx->skip_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_IBC_FLAG; i++) sbac_ctx->ibc_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_MODE_FLAG; i++) sbac_ctx->ats_mode[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_FLAG; i++) sbac_ctx->ats_cu_inter_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_QUAD_FLAG; i++) sbac_ctx->ats_cu_inter_quad_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_HOR_FLAG; i++) sbac_ctx->ats_cu_inter_hor_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_POS_FLAG; i++) sbac_ctx->ats_cu_inter_pos_flag[i] = PROB_INIT;
    }
}

void evce_sbac_finish(EVC_BSW *bs)
{
    EVCE_SBAC *sbac;
    u32 tmp;

    sbac = GET_SBAC_ENC(bs);

    tmp = (sbac->code + sbac->range - 1) & (0xFFFFFFFF << 14);
    if(tmp < sbac->code)
    {
        tmp += 8192;
    }

    sbac->code = tmp << sbac->code_bits;
    sbac_carry_propagate(sbac, bs);

    sbac->code <<= 8;
    sbac_carry_propagate(sbac, bs);

    while(sbac->stacked_zero > 0)
    {
#if TRACE_HLS
        evc_bsw_write_trace(bs, 0x00, 0, 8);
#else
        evc_bsw_write(bs, 0x00, 8);
#endif
        sbac->stacked_zero--;
    }
    if(sbac->pending_byte != 0)
    {
#if TRACE_HLS
        evc_bsw_write_trace(bs, sbac->pending_byte, 0, 8);
#else
        evc_bsw_write(bs, sbac->pending_byte, 8);
#endif
    }
    else
    {
        if(sbac->code_bits < 4)
        {
#if TRACE_HLS
            evc_bsw_write_trace(bs, 0, 0, 4 - sbac->code_bits);
#else
            evc_bsw_write(bs, 0, 4 - sbac->code_bits);
#endif

            while(!EVC_BSW_IS_BYTE_ALIGN(bs))
            {
#if TRACE_HLS
                evc_bsw_write1_trace(bs, 0, 0);
#else
                evc_bsw_write1(bs, 0);
#endif
            }
        }
    }
}

void evce_eco_mmvd_flag(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);

    evce_sbac_encode_bin(flag, sbac, sbac->ctx.mmvd_flag, bs); /* mmvd_flag */

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mmvd_flag ");
    EVC_TRACE_INT(flag);
    EVC_TRACE_STR("\n");
}

void evce_eco_affine_flag(EVC_BSW * bs, int flag, int ctx)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(flag, sbac, sbac->ctx.affine_flag + ctx, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("affine flag ");
    EVC_TRACE_INT(flag);
    EVC_TRACE_STR("\n");
}

void evce_eco_affine_mode(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(flag, sbac, sbac->ctx.affine_mode, bs);
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("affine mode ");
    EVC_TRACE_INT(flag);
    EVC_TRACE_STR("\n");
#endif
}

int evce_eco_affine_mrg_idx(EVC_BSW * bs, s16 affine_mrg)
{
    EVCE_SBAC * sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX * sbac_ctx = &sbac->ctx;

    sbac_write_truncate_unary_sym(affine_mrg, AFF_MAX_CAND, AFF_MAX_CAND, sbac, sbac_ctx->affine_mrg, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("merge affine idx ");
    EVC_TRACE_INT(affine_mrg);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

void evce_eco_affine_mvd_flag(EVC_BSW * bs, int flag, int refi)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(flag, sbac, &sbac->ctx.affine_mvd_flag[refi], bs);
}

static void evce_eco_skip_flag(EVC_BSW * bs, int flag, int ctx)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(flag, sbac, sbac->ctx.skip_flag + ctx, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("skip flag ");
    EVC_TRACE_INT(flag);
    EVC_TRACE_STR("ctx ");
    EVC_TRACE_INT(ctx);
    EVC_TRACE_STR("\n");
}

void evce_eco_ibc_flag(EVC_BSW * bs, int flag, int ctx)
{
  EVCE_SBAC *sbac;
  sbac = GET_SBAC_ENC(bs);
  evce_sbac_encode_bin(flag, sbac, sbac->ctx.ibc_flag + ctx, bs);
#if TRACE_ADDITIONAL_FLAGS
  EVC_TRACE_COUNTER;
  EVC_TRACE_STR("ibc pred mode ");
  EVC_TRACE_INT(!!flag);
  EVC_TRACE_STR("ctx ");
  EVC_TRACE_INT(ctx);
  EVC_TRACE_STR("\n");
#endif
}

void evce_eco_merge_mode_flag(EVC_BSW *bs, int merge_mode_flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(merge_mode_flag, sbac, sbac->ctx.merge_mode_flag, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("merge_mode_flag ");
    EVC_TRACE_INT(merge_mode_flag ? PRED_DIR : 0);
    EVC_TRACE_STR("\n");
}

void evce_eco_direct_mode_flag(EVC_BSW *bs, int direct_mode_flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(direct_mode_flag, sbac, sbac->ctx.direct_mode_flag, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("direct_mode_flag ");
    EVC_TRACE_INT(direct_mode_flag ? PRED_DIR : 0);
    EVC_TRACE_STR("\n");
}

void evce_eco_slice_end_flag(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin_trm(flag, sbac, bs);
}

void evce_eco_tile_end_flag(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin_trm(flag, sbac, bs);
}

void evce_eco_run_length_cc(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type)
{
    EVCE_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;
    u32            num_coeff, scan_pos;
    u32            sign, level, prev_level, run, last_flag;
    s32            t0;
    const u16     *scanp;
    s16            coef_cur;
    int ctx_last = 0;

    sbac = GET_SBAC_ENC(bs);
    sbac_ctx = &sbac->ctx;
    scanp = evc_scan_tbl[COEF_SCAN_ZIGZAG][log2_w - 1][log2_h - 1];
    num_coeff = 1 << (log2_w + log2_h);
    run = 0;
    prev_level = 6;

    for(scan_pos = 0; scan_pos < num_coeff; scan_pos++)
    {
        coef_cur = coef[scanp[scan_pos]];
        if(coef_cur)
        {
            level = EVC_ABS16(coef_cur);
            sign = (coef_cur > 0) ? 0 : 1;
            t0 = sbac->ctx.sps_cm_init_flag == 1 ? ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12) : (ch_type == Y_C ? 0 : 2);

            /* Run coding */
            sbac_write_unary_sym(run, 2, sbac, sbac_ctx->run + t0, bs);

            /* Level coding */
            sbac_write_unary_sym(level - 1, 2, sbac, sbac_ctx->level + t0, bs);

            /* Sign coding */
            sbac_encode_bin_ep(sign, sbac, bs);

            if(scan_pos == num_coeff - 1)
            {
                break;
            }

            run = 0;
            prev_level = level;
            num_sig--;

            /* Last flag coding */
            last_flag = (num_sig == 0) ? 1 : 0;
            ctx_last = (ch_type == Y_C) ? 0 : 1;
            evce_sbac_encode_bin(last_flag, sbac, sbac_ctx->last + ctx_last, bs);

            if(last_flag)
            {
                break;
            }
        }
        else
        {
            run++;
        }
    }

#if ENC_DEC_TRACE
    EVC_TRACE_STR("coef luma ");
    for (scan_pos = 0; scan_pos < num_coeff; scan_pos++)
    {
        EVC_TRACE_INT(coef[scan_pos]);
    }
    EVC_TRACE_STR("\n");
#endif
}

static void code_positionLastXY(EVC_BSW *bs, int last_x, int last_y, int width, int height, int ch_type)
{
    EVCE_SBAC *sbac = GET_SBAC_ENC(bs);
    SBAC_CTX_MODEL* cm_x = sbac->ctx.last_sig_coeff_x_prefix + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_LAST_SIG_COEFF_LUMA : 11));
    SBAC_CTX_MODEL* cm_y = sbac->ctx.last_sig_coeff_y_prefix + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_LAST_SIG_COEFF_LUMA : 11));

    int bin;
    int group_idx_x;
    int group_idx_y;
    int blk_offset_x, blk_offset_y, shift_x, shift_y;
    int i, cnt;

    group_idx_x = g_group_idx[last_x];
    group_idx_y = g_group_idx[last_y];
    if (sbac->ctx.sps_cm_init_flag == 1)
    {
        evc_get_ctx_last_pos_xy_para(ch_type, width, height, &blk_offset_x, &blk_offset_y, &shift_x, &shift_y);
    }
    else
    {
        blk_offset_x = 0;
        blk_offset_y = 0;
        shift_x = 0;
        shift_y = 0;
    }
    //------------------

    // last_sig_coeff_x_prefix
    for (bin = 0; bin < group_idx_x; bin++)
    {
        evce_sbac_encode_bin(1, sbac, &cm_x[blk_offset_x + (bin >> shift_x)], bs);
    }
    if (group_idx_x < g_group_idx[width - 1])
    {
        evce_sbac_encode_bin(0, sbac, &cm_x[blk_offset_x + (bin >> shift_x)], bs);
    }

    // last_sig_coeff_y_prefix
    for (bin = 0; bin < group_idx_y; bin++)
    {
        evce_sbac_encode_bin(1, sbac, &cm_y[blk_offset_y + (bin >> shift_y)], bs);
    }
    if (group_idx_y < g_group_idx[height - 1])
    {
        evce_sbac_encode_bin(0, sbac, &cm_y[blk_offset_y + (bin >> shift_y)], bs);
    }

    // last_sig_coeff_x_suffix
    if (group_idx_x > 3)
    {
        cnt = (group_idx_x - 2) >> 1;
        last_x = last_x - g_min_in_group[group_idx_x];
        for (i = cnt - 1; i >= 0; i--)
        {
            sbac_encode_bin_ep((last_x >> i) & 1, sbac, bs);
        }
    }
    // last_sig_coeff_y_suffix
    if (group_idx_y > 3)
    {
        cnt = (group_idx_y - 2) >> 1;
        last_y = last_y - g_min_in_group[group_idx_y];
        for (i = cnt - 1; i >= 0; i--)
        {
            sbac_encode_bin_ep((last_y >> i) & 1, sbac, bs);
        }
    }
}

static void code_coef_remain_exgolomb(EVC_BSW *bs, int symbol, int rparam)
{
    EVCE_SBAC    * sbac = GET_SBAC_ENC(bs);
    int code_number = symbol;
    int length;
    if (code_number < (g_go_rice_range[rparam] << rparam))
    {
        length = code_number >> rparam;
        sbac_encode_bins_ep((1 << (length + 1)) - 2, length + 1, sbac, bs);
        sbac_encode_bins_ep((code_number % (1 << rparam)), rparam, sbac, bs);
    }
    else
    {
        length = rparam;
        code_number = code_number - (g_go_rice_range[rparam] << rparam);
        while (code_number >= (1 << length))
        {
            code_number -= (1 << (length++));
        }
        sbac_encode_bins_ep((1 << (g_go_rice_range[rparam] + length + 1 - rparam)) - 2, g_go_rice_range[rparam] + length + 1 - rparam, sbac, bs);
        sbac_encode_bins_ep(code_number, length, sbac, bs);
    }
}

int countNonZeroCoeffs(s16 *pcCoef, int *scan, int uiSize)
{
    int count = 0;

    for (int i = 0; i < uiSize; i++)
    {
        int rastrer_pos = scan[i];
        count += pcCoef[rastrer_pos] != 0;
    }

    return count;
}

static void evce_eco_adcc(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type)
{
    int width = 1 << log2_w;
    int height = 1 << log2_h;
    int offset0;
    EVCE_SBAC    * sbac = GET_SBAC_ENC(bs);
    SBAC_CTX_MODEL* cm_sig_coeff;
    SBAC_CTX_MODEL* cm_gtx;
    int scan_type = COEF_SCAN_ZIGZAG;
    int log2_block_size = min(log2_w, log2_h);
    u16 *scan;
    int scan_pos_last = -1;
    int last_x = 0, last_y = 0;
    int ipos;
    int last_scan_set;
    int rice_param;
    int sub_set;

    int ctx_sig_coeff = 0;
    int cg_log2_size = LOG2_CG_SIZE;
    int is_last_x = 0;
    int is_last_y = 0;
    int is_last_nz = 0;
    int pos_last = 0;
    int ctx_gtA = 0;
    int ctx_gtB = 0;
    int escape_data_present_ingroup = 0;
    int cnt_nz = 0;
    int blkpos, sx, sy;
    int sig_coeff_flag;


    int max_num_coef = width * height;
    scan = evc_scan_tbl[scan_type][log2_w - 1][log2_h - 1];

    int last_pos_in_scan = 0;
    int numNonZeroCoefs = 0;

    last_pos_in_scan = -1;
    int last_pos_in_raster_from_scan = -1;

    for (int blk_pos = 0; blk_pos < max_num_coef; blk_pos++)
    {
        int scan_pos = scan[blk_pos];

        if (coef[scan_pos] != 0)
        {
            last_y = scan_pos >> log2_w;
            last_x = scan_pos - (last_y << log2_w);

            numNonZeroCoefs++;
            last_pos_in_scan = blk_pos;
            last_pos_in_raster_from_scan = scan_pos;
        }
    }
    code_positionLastXY(bs, last_x, last_y, width, height, ch_type);

    //===== code significance flag =====
    last_scan_set = last_pos_in_scan >> cg_log2_size;
    if (sbac->ctx.sps_cm_init_flag == 1)
    {
        offset0 = log2_block_size <= 2 ? 0 : NUM_CTX_SIG_COEFF_LUMA_TU << (EVC_MIN(1, (log2_block_size - 3)));
        cm_sig_coeff = (ch_type == Y_C) ? sbac->ctx.sig_coeff_flag + offset0 : sbac->ctx.sig_coeff_flag + NUM_CTX_SIG_COEFF_LUMA;
        cm_gtx = (ch_type == Y_C) ? sbac->ctx.coeff_abs_level_greaterAB_flag : sbac->ctx.coeff_abs_level_greaterAB_flag + NUM_CTX_GTX_LUMA;
    }
    else
    {
        cm_sig_coeff = (ch_type == Y_C) ? sbac->ctx.sig_coeff_flag : sbac->ctx.sig_coeff_flag + 1;
        cm_gtx = (ch_type == Y_C) ? sbac->ctx.coeff_abs_level_greaterAB_flag : sbac->ctx.coeff_abs_level_greaterAB_flag + 1;
    }
    rice_param = 0;
    ipos = last_pos_in_scan;

    for (sub_set = last_scan_set; sub_set >= 0; sub_set--)
    {
        int num_nz = 0;
        int sub_pos = sub_set << cg_log2_size;
        int coef_signs_group = 0;
        int abs_coef[1 << LOG2_CG_SIZE];  // array size of CG
        int pos[1 << LOG2_CG_SIZE];  // array size of CG
        int last_nz_pos_in_cg = -1;
        int first_nz_pos_in_cg = 1 << cg_log2_size;

        {
            for (; ipos >= sub_pos; ipos--)
            {
                blkpos = scan[ipos];
                sy = blkpos >> log2_w;
                sx = blkpos - (sy << log2_w);

                // sigmap
                sig_coeff_flag = (coef[blkpos] != 0 ? 1 : 0);
                if (ipos == last_pos_in_scan)
                {
                    ctx_sig_coeff = 0;
                }
                else
                {
                    ctx_sig_coeff = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_sig_coeff_inc(coef, blkpos, width, height, ch_type) : 0;
                }

                if (!(ipos == last_pos_in_scan))
                {
                    evce_sbac_encode_bin((u32)sig_coeff_flag, sbac, &cm_sig_coeff[ctx_sig_coeff], bs);
                }

                if (sig_coeff_flag)
                {
                    pos[num_nz] = blkpos;
                    abs_coef[num_nz] = (int)(EVC_ABS(coef[blkpos]));
                    coef_signs_group = 2 * coef_signs_group + (coef[blkpos] < 0 ? 1 : 0);
                    num_nz++;

                    if (last_nz_pos_in_cg == -1)
                    {
                        last_nz_pos_in_cg = ipos;
                    }
                    first_nz_pos_in_cg = ipos;
                    if (is_last_nz == 0)
                    {
                        pos_last = blkpos;
                        is_last_nz = 1;
                    }
                }
            }

            if (num_nz > 0)
            {
                int numC1Flag = min(num_nz, CAFLAG_NUMBER);

                int firstC2FlagIdx = -1;
                escape_data_present_ingroup = 0;

                for (int idx = 0; idx < numC1Flag; idx++)  // 
                {
                    u32 coeff_abs_level_greaterA_flag = abs_coef[idx] > 1 ? 1 : 0;
                    if (pos[idx] != pos_last)
                    {
                        ctx_gtA = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtA_inc(coef, pos[idx], width, height, ch_type) : 0;
                    }
                    evce_sbac_encode_bin(coeff_abs_level_greaterA_flag, sbac, &cm_gtx[ctx_gtA], bs);
                    if (coeff_abs_level_greaterA_flag)
                    {
                        if (firstC2FlagIdx == -1)
                        {
                            firstC2FlagIdx = idx;
                        }
                        else
                        {
                            escape_data_present_ingroup = TRUE;
                        }
                    }
                }
                if (firstC2FlagIdx != -1)
                {
                    u32 coeff_abs_level_greaterB_flag = abs_coef[firstC2FlagIdx] > 2 ? 1 : 0;
                    if (pos[firstC2FlagIdx] != pos_last)
                    {
                        ctx_gtB = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtB_inc(coef, pos[firstC2FlagIdx], width, height, ch_type) : 0;
                    }
                    evce_sbac_encode_bin(coeff_abs_level_greaterB_flag, sbac, &cm_gtx[ctx_gtB], bs);

                    if (coeff_abs_level_greaterB_flag != 0)
                    {
                        escape_data_present_ingroup = 1;
                    }
                }
                escape_data_present_ingroup = escape_data_present_ingroup || (num_nz > CAFLAG_NUMBER);

                int iFirstCoeff2 = 1;
                if (escape_data_present_ingroup)
                {
                    for (int idx = 0; idx < num_nz; idx++)
                    {
                        int base_level = (idx < CAFLAG_NUMBER) ? (2 + iFirstCoeff2) : 1;
                        if (abs_coef[idx] >= base_level)
                        {
                            int coeff_abs_level_remaining = abs_coef[idx] - base_level;
                            rice_param = get_rice_para(coef, pos[idx], width, height, base_level);
                            code_coef_remain_exgolomb(bs, coeff_abs_level_remaining, rice_param);
                        }
                        if (abs_coef[idx] >= 2)
                        {
                            iFirstCoeff2 = 0;
                        }
                    }
                }
                sbac_encode_bins_ep(coef_signs_group, num_nz, sbac, bs);
            }
        }
    }
}

static int evce_eco_ats_intra_cu(EVC_BSW *bs, u8 ats_intra_cu)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    sbac_encode_bin_ep(ats_intra_cu, sbac, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra CU ");
    EVC_TRACE_INT(ats_intra_cu);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

static int evce_eco_ats_mode_h(EVC_BSW *bs, u8 ats_mode_h)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(ats_mode_h, sbac, sbac->ctx.ats_mode, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuH ");
    EVC_TRACE_INT(ats_mode_h);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

static int evce_eco_ats_mode_v(EVC_BSW *bs, u8 ats_mode_v)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(ats_mode_v, sbac, sbac->ctx.ats_mode, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuV ");
    EVC_TRACE_INT(ats_mode_v);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

void evce_eco_xcoef(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type, int tool_adcc)
{
    if (tool_adcc)
    {
        evce_eco_adcc(bs, coef, log2_w, log2_h, num_sig, (ch_type == Y_C ? 0 : 1));
    }
    else
    {
        evce_eco_run_length_cc(bs, coef, log2_w, log2_h, num_sig, (ch_type == Y_C ? 0 : 1));
    }

#if TRACE_COEFFS
    int cuw = 1 << log2_w;
    int cuh = 1 << log2_h;
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("Coeff for ");
    EVC_TRACE_INT(ch_type);
    EVC_TRACE_STR(": ");
    for (int i = 0; i < (cuw * cuh); ++i)
    {
        if (i != 0)
            EVC_TRACE_STR(", ");
        EVC_TRACE_INT(coef[i]);
    }
    EVC_TRACE_STR("\n");
#endif
}

int evce_eco_ats_inter_info(EVC_BSW * bs, int log2_cuw, int log2_cuh, int ats_inter_info, u8 ats_inter_avail)
{
    u8 mode_vert = (ats_inter_avail >> 0) & 0x1;
    u8 mode_hori = (ats_inter_avail >> 1) & 0x1;
    u8 mode_vert_quad = (ats_inter_avail >> 2) & 0x1;
    u8 mode_hori_quad = (ats_inter_avail >> 3) & 0x1;
    u8 num_ats_inter_mode_avail = mode_vert + mode_hori + mode_vert_quad + mode_hori_quad;

    if (num_ats_inter_mode_avail == 0)
    {
        assert(ats_inter_info == 0);
        return EVC_OK;
    }
    else
    {
        u8 ats_inter_idx = get_ats_inter_idx(ats_inter_info);
        u8 ats_inter_flag = ats_inter_idx != 0;
        u8 ats_inter_hor = is_ats_inter_horizontal(ats_inter_idx);
        u8 ats_inter_quad = is_ats_inter_quad_size(ats_inter_idx);
        u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
        int size = 1 << (log2_cuw + log2_cuh);

        EVCE_SBAC    *sbac;
        EVC_SBAC_CTX *sbac_ctx;
        sbac = GET_SBAC_ENC(bs);
        sbac_ctx = &sbac->ctx;

        u8 ctx_ats_inter = sbac->ctx.sps_cm_init_flag == 1 ? ((log2_cuw + log2_cuh >= 8) ? 0 : 1) : 0;
        u8 ctx_ats_inter_hor = sbac->ctx.sps_cm_init_flag == 1 ? ((log2_cuw == log2_cuh) ? 0 : (log2_cuw < log2_cuh ? 1 : 2)) : 0;

        if (ats_inter_idx == 0)
            assert(ats_inter_pos == 0);

        evce_sbac_encode_bin(ats_inter_flag, sbac, sbac_ctx->ats_cu_inter_flag + ctx_ats_inter, bs);
        EVC_TRACE_STR("ats_inter_flag ");
        EVC_TRACE_INT(ats_inter_flag);
        EVC_TRACE_STR("\n");

        if (ats_inter_flag)
        {
            if ((mode_vert_quad || mode_hori_quad) && (mode_vert || mode_hori))
            {
                evce_sbac_encode_bin(ats_inter_quad, sbac, sbac_ctx->ats_cu_inter_quad_flag, bs);
                EVC_TRACE_STR("ats_inter_quad ");
                EVC_TRACE_INT(ats_inter_quad);
                EVC_TRACE_STR("\n");
            }
            else
            {
                assert(ats_inter_quad == 0);
            }

            if ((ats_inter_quad && mode_vert_quad && mode_hori_quad) || (!ats_inter_quad && mode_vert && mode_hori))
            {
                evce_sbac_encode_bin(ats_inter_hor, sbac, sbac_ctx->ats_cu_inter_hor_flag + ctx_ats_inter_hor, bs);
                EVC_TRACE_STR("ats_inter_hor ");
                EVC_TRACE_INT(ats_inter_hor);
                EVC_TRACE_STR("\n");
            }
            else
            {
                assert(ats_inter_hor == ((ats_inter_quad && mode_hori_quad) || (!ats_inter_quad && mode_hori)));
            }

            evce_sbac_encode_bin(ats_inter_pos, sbac, sbac_ctx->ats_cu_inter_pos_flag, bs);
            EVC_TRACE_STR("ats_inter_pos ");
            EVC_TRACE_INT(ats_inter_pos);
            EVC_TRACE_STR("\n");
        }

        return EVC_OK;
    }
}

int evce_eco_cbf(EVC_BSW * bs, int cbf_y, int cbf_u, int cbf_v, u8 pred_mode, int b_no_cbf, int is_sub, int sub_pos, int cbf_all, int run[N_C], TREE_CONS tree_cons
#if BD_CF_EXT
                 , int chroma_format_idc
#endif
)
{
    EVCE_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;

    sbac = GET_SBAC_ENC(bs);
    sbac_ctx = &sbac->ctx;

    /* code allcbf */
    if(pred_mode != MODE_INTRA && !evc_check_only_intra(tree_cons) )
    {
        if (!cbf_all && sub_pos)
        {
            return EVC_OK;
        }            
        if(b_no_cbf == 1)
        {
            evc_assert(cbf_all != 0);
        }
        else if(sub_pos == 0 && (run[Y_C] + run[U_C] + run[V_C]) == 3) // not count bits of root_cbf when checking each component
        {
            if(cbf_all == 0)
            {
                evce_sbac_encode_bin(0, sbac, sbac_ctx->cbf_all, bs);

                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("all_cbf ");
                EVC_TRACE_INT(0);
                EVC_TRACE_STR("\n");

                return EVC_OK;
            }
            else
            {
                evce_sbac_encode_bin(1, sbac, sbac_ctx->cbf_all, bs);

                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("all_cbf ");
                EVC_TRACE_INT(1);
                EVC_TRACE_STR("\n");
            }
        }

#if BD_CF_EXT
        if(run[U_C] && chroma_format_idc != 0)
#else
        if (run[U_C])
#endif
        {
            evce_sbac_encode_bin(cbf_u, sbac, sbac_ctx->cbf_cb, bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf U ");
            EVC_TRACE_INT(cbf_u);
            EVC_TRACE_STR("\n");
        }

#if BD_CF_EXT
        if(run[V_C] && chroma_format_idc != 0)
#else
        if (run[V_C])
#endif
        {
            evce_sbac_encode_bin(cbf_v, sbac, sbac_ctx->cbf_cr, bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf V ");
            EVC_TRACE_INT(cbf_v);
            EVC_TRACE_STR("\n");
        }

        if (run[Y_C] && (cbf_u + cbf_v != 0 || is_sub))
        {
            evce_sbac_encode_bin(cbf_y, sbac, sbac_ctx->cbf_luma, bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf Y ");
            EVC_TRACE_INT(cbf_y);
            EVC_TRACE_STR("\n");
        }       
    }
    else
    {
#if BD_CF_EXT
        if(run[U_C] && chroma_format_idc != 0)
#else
        if (run[U_C])
#endif
        {
            evc_assert(evc_check_chroma(tree_cons));
            evce_sbac_encode_bin(cbf_u, sbac, sbac_ctx->cbf_cb, bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf U ");
            EVC_TRACE_INT(cbf_u);
            EVC_TRACE_STR("\n");
        }

#if BD_CF_EXT
        if(run[V_C] && chroma_format_idc != 0)
#else
        if (run[V_C])
#endif
        {
            evc_assert(evc_check_chroma(tree_cons));
            evce_sbac_encode_bin(cbf_v, sbac, sbac_ctx->cbf_cr, bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf V ");
            EVC_TRACE_INT(cbf_v);
            EVC_TRACE_STR("\n");
        }

        if (run[Y_C])
        {
            evc_assert(evc_check_luma(tree_cons));
            evce_sbac_encode_bin(cbf_y, sbac, sbac_ctx->cbf_luma, bs);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf Y ");
            EVC_TRACE_INT(cbf_y);
            EVC_TRACE_STR("\n");
        }
    }

    return EVC_OK;
}

#if DQP
int evce_eco_dqp(EVC_BSW * bs, int ref_qp, int cur_qp)
{
    int abs_dqp, dqp, t0;
    u32 sign;
    EVCE_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;

    sbac = GET_SBAC_ENC(bs);
    sbac_ctx = &sbac->ctx;

    dqp = cur_qp - ref_qp;
    abs_dqp = EVC_ABS(dqp);
    t0 = abs_dqp;

    sbac_write_unary_sym(t0, NUM_CTX_DELTA_QP, sbac, sbac_ctx->delta_qp, bs);

    if(abs_dqp > 0)
    {
        sign = dqp > 0 ? 0 : 1;
        sbac_encode_bin_ep(sign, sbac, bs);
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("dqp ");
    EVC_TRACE_INT(dqp);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}
#endif

int evce_eco_coef(EVC_BSW * bs, s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 pred_mode, int nnz_sub[N_C][MAX_SUB_TB_NUM], int b_no_cbf, int run_stats
                  , int tool_ats, u8 ats_intra_cu, u8 ats_mode, u8 ats_inter_info, EVCE_CTX * ctx
#if DQP
                  , EVCE_CORE * core, int enc_dqp, u8 cur_qp
#endif
                  , TREE_CONS tree_cons
#if BD_CF_EXT
                  , int chroma_format_idc
#endif
)
{
    run_stats = evc_get_run(run_stats, tree_cons);
    int run[N_C] = { run_stats & 1, (run_stats >> 1) & 1, (run_stats >> 2) & 1 };
    s16 *coef_temp[N_C];
    s16 coef_temp_buf[N_C][MAX_TR_DIM];
    int i, j, c;
    int log2_w_sub = (log2_cuw > MAX_TR_LOG2) ? MAX_TR_LOG2 : log2_cuw;
    int log2_h_sub = (log2_cuh > MAX_TR_LOG2) ? MAX_TR_LOG2 : log2_cuh;
    int loop_w = (log2_cuw > MAX_TR_LOG2) ? (1 << (log2_cuw - MAX_TR_LOG2)) : 1;
    int loop_h = (log2_cuh > MAX_TR_LOG2) ? (1 << (log2_cuh - MAX_TR_LOG2)) : 1;
    int stride = (1 << log2_cuw);
    int sub_stride = (1 << log2_w_sub);
    int is_sub = loop_h + loop_w > 2 ? 1 : 0;

    if (!evc_check_luma(tree_cons))
    {
        evc_assert(run[0] == 0);
    }
    if (!evc_check_chroma(tree_cons))
    {
        evc_assert((run[1] == 0) && (run[2] == 0));
    }
    evc_assert(run_stats != 0);

    int cbf_all = 0;
    u8 is_intra = (pred_mode == MODE_INTRA) ? 1 : 0;
    EVCE_SBAC    * sbac = GET_SBAC_ENC(bs);

    u8 ats_inter_avail = check_ats_inter_info_coded(1 << log2_cuw, 1 << log2_cuh, pred_mode, tool_ats);
    if( ats_inter_avail )
    {
        get_tu_size( ats_inter_info, log2_cuw, log2_cuh, &log2_w_sub, &log2_h_sub );
        sub_stride = (1 << log2_w_sub);
    }

    for (j = 0; j < loop_h; j++)
    {
        for (i = 0; i < loop_w; i++)
        {
            for (c = 0; c < N_C; c++)
            {
                if (run[c])
                {
                    cbf_all += !!nnz_sub[c][(j << 1) | i];
                }
            }
        }
    }

    for (j = 0; j < loop_h; j++)
    {
        for (i = 0; i < loop_w; i++)
        {
            evce_eco_cbf(bs, !!nnz_sub[Y_C][(j << 1) | i], !!nnz_sub[U_C][(j << 1) | i], !!nnz_sub[V_C][(j << 1) | i], pred_mode, b_no_cbf, is_sub, j + i, cbf_all, run, tree_cons
#if BD_CF_EXT
                         , chroma_format_idc
#endif
            );
#if DQP
            if(ctx->pps.cu_qp_delta_enabled_flag)
            {
                if(enc_dqp == 1)
                {
                    int cbf_for_dqp = (!!nnz_sub[Y_C][(j << 1) | i]) || (!!nnz_sub[U_C][(j << 1) | i]) || (!!nnz_sub[V_C][(j << 1) | i]);
                    if((((!(ctx->sps.dquant_flag) || (core->cu_qp_delta_code == 1 && !core->cu_qp_delta_is_coded)) && (cbf_for_dqp))
                        || (core->cu_qp_delta_code == 2 && !core->cu_qp_delta_is_coded)))
                    {
                        evce_eco_dqp(bs, ctx->tile[core->tile_idx].qp_prev_eco, cur_qp);
                        core->cu_qp_delta_is_coded = 1;
                        ctx->tile[core->tile_idx].qp_prev_eco = cur_qp;
                    }
                }
            }
#endif

            if (tool_ats && (!!nnz_sub[Y_C][(j << 1) | i]) && (log2_cuw <= 5 && log2_cuh <= 5) && is_intra && run[Y_C])
            {
                evce_eco_ats_intra_cu(bs, ats_intra_cu);

                if (ats_intra_cu)
                {
                    evce_eco_ats_mode_h(bs, (ats_mode >> 1));
                    evce_eco_ats_mode_v(bs, (ats_mode & 1));
                }
            }

            if (pred_mode != MODE_INTRA && pred_mode != MODE_IBC && run[Y_C] && run[U_C] && run[V_C])
            {
                if (ats_inter_avail && cbf_all)
                {
                    assert(loop_w == 1 && loop_h == 1);
                    evce_eco_ats_inter_info(bs, log2_cuw, log2_cuh, ats_inter_info, ats_inter_avail);
                }
                else
                {
                    assert(ats_inter_info == 0);
                }
            }

            for (c = 0; c < N_C; c++)
            {
                if (nnz_sub[c][(j << 1) | i] && run[c])
                {
#if BD_CF_EXT
                    int pos_sub_x = c == 0 ? (i * (1 << (log2_w_sub))) : (i * (1 << (log2_w_sub - (GET_CHROMA_W_SHIFT(chroma_format_idc)))));
                    int pos_sub_y = c == 0 ? j * (1 << (log2_h_sub)) * (stride) : j * (1 << (log2_h_sub - (GET_CHROMA_H_SHIFT(chroma_format_idc)))) * (stride >> (GET_CHROMA_W_SHIFT(chroma_format_idc)));
#else
                    int pos_sub_x = i * (1 << (log2_w_sub - !!c));
                    int pos_sub_y = j * (1 << (log2_h_sub - !!c)) * (stride >> (!!c));
#endif

                    if (is_sub)
                    {
#if BD_CF_EXT
                        if(c == 0)
                            evc_block_copy(coef[c] + pos_sub_x + pos_sub_y, stride, coef_temp_buf[c], sub_stride, log2_w_sub, log2_h_sub);
                        else
                            evc_block_copy(coef[c] + pos_sub_x + pos_sub_y, stride >> (GET_CHROMA_W_SHIFT(chroma_format_idc)), coef_temp_buf[c], sub_stride >> (GET_CHROMA_W_SHIFT(chroma_format_idc)), log2_w_sub - (GET_CHROMA_W_SHIFT(chroma_format_idc)), log2_h_sub - (GET_CHROMA_H_SHIFT(chroma_format_idc)));
#else
                        evc_block_copy(coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), coef_temp_buf[c], sub_stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
#endif
                        coef_temp[c] = coef_temp_buf[c];
                    }
                    else
                    {
                        coef_temp[c] = coef[c];
                    }

#if BD_CF_EXT
                    if(c==0)
                        evce_eco_xcoef(bs, coef_temp[c], log2_w_sub, log2_h_sub, nnz_sub[c][(j << 1) | i], c, ctx->sps.tool_adcc);
                    else
                        evce_eco_xcoef(bs, coef_temp[c], log2_w_sub - (GET_CHROMA_W_SHIFT(chroma_format_idc)), log2_h_sub - (GET_CHROMA_H_SHIFT(chroma_format_idc)), nnz_sub[c][(j << 1) | i], c, ctx->sps.tool_adcc);
#else
                    evce_eco_xcoef(bs, coef_temp[c], log2_w_sub - (!!c), log2_h_sub - (!!c), nnz_sub[c][(j << 1) | i], c, ctx->sps.tool_adcc);
#endif

                    if (is_sub)
                    {
#if BD_CF_EXT
                        if(c == 0)
                            evc_block_copy(coef_temp_buf[c], sub_stride, coef[c] + pos_sub_x + pos_sub_y, stride, log2_w_sub, log2_h_sub);
                        else
                            evc_block_copy(coef_temp_buf[c], sub_stride >> (GET_CHROMA_W_SHIFT(chroma_format_idc)), coef[c] + pos_sub_x + pos_sub_y
                                , stride >> (GET_CHROMA_W_SHIFT(chroma_format_idc)), log2_w_sub - (GET_CHROMA_W_SHIFT(chroma_format_idc)), log2_h_sub - (GET_CHROMA_H_SHIFT(chroma_format_idc)));
#else
                        evc_block_copy(coef_temp_buf[c], sub_stride >> (!!c), coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
#endif
                    }
                }
            }
        }
    }
    return EVC_OK;
}

int evce_eco_pred_mode(EVC_BSW * bs, u8 pred_mode, int ctx)
{
    EVCE_SBAC * sbac = GET_SBAC_ENC(bs);
       
    evce_sbac_encode_bin(pred_mode == MODE_INTRA, sbac, sbac->ctx.pred_mode + ctx, bs);
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("pred mode ");
    EVC_TRACE_INT(pred_mode == MODE_INTRA ? MODE_INTRA : MODE_INTER);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

int evce_eco_ibc(EVC_BSW * bs, u8 pred_mode_ibc_flag, int ctx)
{
  EVCE_SBAC * sbac = GET_SBAC_ENC(bs);

  EVC_TRACE_COUNTER;
  EVC_TRACE_STR("ibc pred mode ");
  EVC_TRACE_INT(!!pred_mode_ibc_flag);
  EVC_TRACE_STR("\n");

  evce_sbac_encode_bin(pred_mode_ibc_flag, sbac, sbac->ctx.ibc_flag + ctx, bs);

  return EVC_OK;
}

static void intra_mode_write_trunc_binary(int symbol, int max_symbol, EVCE_SBAC *sbac, EVC_BSW *bs)
{
    int threshold = 4; /* we use 5 bits to signal the default mode */
    int val = 1 << threshold;
    int b;

    if(val > max_symbol)
    {
        printf("val =%d max_symbol= %d", val, max_symbol);
    }
    assert(val <= max_symbol);
    assert((val << 1) > max_symbol);
    assert(symbol < max_symbol);

    b = max_symbol - val;
    assert(b < val);

    if(symbol < val - b)
    {
        sbac_encode_bins_ep(symbol, threshold, sbac, bs);
    }
    else
    {
        symbol += val - b;
        assert(symbol < (val << 1));
        assert((symbol >> 1) >= val - b);
        sbac_encode_bins_ep(symbol, threshold + 1, sbac, bs);
    }
}

int evce_eco_intra_dir_b(EVC_BSW *bs, u8 ipm, u8  * mpm, u8 mpm_ext[8], u8 pims[IPD_CNT])
{
    EVCE_SBAC *sbac;

    sbac = GET_SBAC_ENC(bs);
    sbac_write_unary_sym(mpm[ipm], 2, sbac, sbac->ctx.intra_dir, bs);
    EVC_TRACE_COUNTER;
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_STR("mpm list: ");
    for (int i = 0; i < IPD_CNT_B; i++)
    {
        EVC_TRACE_INT(mpm[i]);
    }
#endif
    EVC_TRACE_STR("ipm Y ");
    EVC_TRACE_INT(ipm);
    EVC_TRACE_STR("\n");
    return EVC_OK;
}

int evce_eco_intra_dir(EVC_BSW *bs, u8 ipm, u8 mpm[2], u8 mpm_ext[8], u8 pims[IPD_CNT])
{
    EVCE_SBAC *sbac;

    int t0;
    sbac = GET_SBAC_ENC(bs);
    if(ipm == mpm[0] || ipm == mpm[1])
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.intra_luma_pred_mpm_flag, bs);
        t0 = ipm == mpm[0] ? 0 : 1;
        evce_sbac_encode_bin(t0, sbac, sbac->ctx.intra_luma_pred_mpm_idx, bs);
    }
    else
    {
        int i;
        int pms_cnt = -1;
        int flag = 0;
        int check = 8;

        evce_sbac_encode_bin(0, sbac, sbac->ctx.intra_luma_pred_mpm_flag, bs);

        for(i = 0; i < check; i++)
        {
            if(ipm == mpm_ext[i])
            {
                flag = i + 1;
                break;
            }
        }

        if(flag)
        {
            sbac_encode_bin_ep(1, sbac, bs);
            flag = flag - 1;
            {
                sbac_encode_bin_ep((flag >> 2) & 1, sbac, bs);
                sbac_encode_bin_ep((flag >> 1) & 1, sbac, bs);
                sbac_encode_bin_ep(flag & 1, sbac, bs);
            }
        }
        else
        {
            sbac_encode_bin_ep(0, sbac, bs);

            for(pms_cnt = 0; pms_cnt < IPD_CNT; pms_cnt++)
            {
                if(ipm == pims[pms_cnt])
                {
                    break;
                }
            }
            pms_cnt -= check + 2;
            intra_mode_write_trunc_binary(pms_cnt, IPD_CNT - (check + 2), sbac, bs);
        }
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ipm Y ");
    EVC_TRACE_INT(ipm);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

int evce_eco_intra_dir_c(EVC_BSW *bs, u8 ipm, u8 ipm_l)
{
    EVCE_SBAC *sbac;
    u8         chk_bypass;
    int        remain;
#if TRACE_ADDITIONAL_FLAGS
    u8 ipm_l_saved = ipm_l;
#endif
    sbac = GET_SBAC_ENC(bs);
       
    EVC_IPRED_CONV_L2C_CHK(ipm_l, chk_bypass);

    if(ipm == 0)
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.intra_chroma_pred_mode, bs);
    }
    else
    {
        evce_sbac_encode_bin(0, sbac, sbac->ctx.intra_chroma_pred_mode, bs);
        remain = (chk_bypass && ipm > ipm_l) ? ipm - 2 : ipm - 1;
        sbac_write_unary_sym_ep(remain, sbac, bs, IPD_CHROMA_CNT - 1);
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ipm UV ");
    EVC_TRACE_INT(ipm);
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_STR("ipm L ");
    EVC_TRACE_INT(ipm_l_saved);
#endif
    EVC_TRACE_STR("\n");
       
    return EVC_OK;
}

int evce_eco_mmvd_info(EVC_BSW *bs, int mvp_idx, int type)
{
    EVCE_SBAC *sbac = GET_SBAC_ENC(bs);
    int var0, var1, var2;
    int dev0 = 0;
    int var;
    int t_idx = mvp_idx;

    if(type == 1)
    {
        if(t_idx >= (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM))
        {
            t_idx = t_idx - (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            dev0 = t_idx / (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            t_idx = t_idx - dev0 * (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            var = 1;
        }
        else
        {
            var = 0;
        }

        /* mmvd_group_idx */
        evce_sbac_encode_bin(var, sbac, sbac->ctx.mmvd_group_idx + 0, bs);
        if(var == 1)
        {
            evce_sbac_encode_bin(dev0, sbac, sbac->ctx.mmvd_group_idx + 1, bs);
        }
    }
    else
    {
        var = 0;
        dev0 = 0;
    }

    var0 = t_idx / MMVD_MAX_REFINE_NUM;
    var1 = (t_idx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
    var2 = t_idx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;

    sbac_write_truncate_unary_sym(var0, NUM_CTX_MMVD_MERGE_IDX, MMVD_BASE_MV_NUM, sbac, sbac->ctx.mmvd_merge_idx, bs); /* mmvd_merge_idx */
    sbac_write_truncate_unary_sym(var1, NUM_CTX_MMVD_DIST_IDX, MMVD_DIST_NUM, sbac, sbac->ctx.mmvd_distance_idx, bs); /* mmvd_distance_idx */

    /* mmvd_direction_idx */
    if(var2 == 0)
    {
        evce_sbac_encode_bin(0, sbac, sbac->ctx.mmvd_direction_idx, bs);
        evce_sbac_encode_bin(0, sbac, sbac->ctx.mmvd_direction_idx + 1, bs);
    }
    else if(var2 == 1)
    {
        evce_sbac_encode_bin(0, sbac, sbac->ctx.mmvd_direction_idx, bs);
        evce_sbac_encode_bin(1, sbac, sbac->ctx.mmvd_direction_idx + 1, bs);
    }
    else if(var2 == 2)
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.mmvd_direction_idx, bs);
        evce_sbac_encode_bin(0, sbac, sbac->ctx.mmvd_direction_idx + 1, bs);
    }
    else if(var2 == 3)
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.mmvd_direction_idx, bs);
        evce_sbac_encode_bin(1, sbac, sbac->ctx.mmvd_direction_idx + 1, bs);
    }
    
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mmvd_idx ");
    EVC_TRACE_INT(mvp_idx);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

void evce_eco_inter_pred_idc(EVC_BSW *bs, s8 refi[REFP_NUM], int slice_type, int cuw, int cuh, int is_sps_admvp)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);

    if(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1])) /* PRED_BI */
    {
        assert(check_bi_applicability(slice_type, cuw, cuh, is_sps_admvp));
        evce_sbac_encode_bin(0, sbac, sbac->ctx.inter_dir, bs);       
    }
    else
    {
        if (check_bi_applicability(slice_type, cuw, cuh, is_sps_admvp))
        {
            evce_sbac_encode_bin(1, sbac, sbac->ctx.inter_dir, bs);
        }

        if(REFI_IS_VALID(refi[REFP_0])) /* PRED_L0 */
        {
            evce_sbac_encode_bin(0, sbac, sbac->ctx.inter_dir + 1, bs);
        }
        else /* PRED_L1 */
        {
            evce_sbac_encode_bin(1, sbac, sbac->ctx.inter_dir + 1, bs);
        }
    }
#if ENC_DEC_TRACE
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("inter dir ");
    EVC_TRACE_INT(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1])? PRED_BI : (REFI_IS_VALID(refi[REFP_0]) ? PRED_L0 : PRED_L1));
    EVC_TRACE_STR("\n");
#endif
    return;
}

int evce_eco_refi(EVC_BSW *bs, int num_refp, int refi)
{
    EVCE_SBAC    *sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;
    int            i, bin;

    if(num_refp > 1)
    {
        if(refi == 0)
        {
            evce_sbac_encode_bin(0, sbac, sbac_ctx->refi, bs);
        }
        else
        {
            evce_sbac_encode_bin(1, sbac, sbac_ctx->refi, bs);
            if(num_refp > 2)
            {
                for(i = 2; i < num_refp; i++)
                {
                    bin = (i == refi + 1) ? 0 : 1;
                    if(i == 2)
                    {
                        evce_sbac_encode_bin(bin, sbac, sbac_ctx->refi + 1, bs);
                    }
                    else
                    {
                        sbac_encode_bin_ep(bin, sbac, bs);
                    }
                    if(bin == 0)
                    {
                        break;
                    }
                }
            }
        }
    }

    return EVC_OK;
}

int evce_eco_merge_idx(EVC_BSW *bs, int merge_idx)
{
    EVCE_SBAC *sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;

    sbac_write_truncate_unary_sym(merge_idx, NUM_CTX_MERGE_IDX, MAX_NUM_MVP, sbac, sbac_ctx->merge_idx, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("merge idx ");
    EVC_TRACE_INT(merge_idx);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

int evce_eco_mvp_idx(EVC_BSW *bs, int mvp_idx)
{
    EVCE_SBAC *sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;

    sbac_write_truncate_unary_sym(mvp_idx, 3, 4, sbac, sbac_ctx->mvp_idx, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvp idx ");
    EVC_TRACE_INT(mvp_idx);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

int evce_eco_affine_mvp_idx( EVC_BSW *bs, int mvp_idx )
{
#if AFF_MAX_NUM_MVP > 1
    EVCE_SBAC *sbac = GET_SBAC_ENC( bs );
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;

    sbac_write_truncate_unary_sym( mvp_idx, NUM_CTX_AFFINE_MVP_IDX, AFF_MAX_NUM_MVP, sbac, sbac_ctx->affine_mvp_idx, bs );

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR( "affine mvp idx " );
    EVC_TRACE_INT( mvp_idx );
    EVC_TRACE_STR( "\n" );
#endif

    return EVC_OK;
}

int evce_eco_mvr_idx(EVC_BSW * bs, u8 mvr_idx)
{
    EVCE_SBAC * sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX * sbac_ctx = &sbac->ctx;

    sbac_write_truncate_unary_sym(mvr_idx, MAX_NUM_MVR, MAX_NUM_MVR, sbac, sbac_ctx->mvr_idx, bs);
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvr_idx ");
    EVC_TRACE_INT(mvr_idx);
    EVC_TRACE_STR("\n");
#endif

    return EVC_OK;
}

int evce_eco_bi_idx(EVC_BSW *bs, u8 bi_idx)
{
    EVCE_SBAC * sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX * sbac_ctx = &sbac->ctx;

    if(bi_idx == 0)
    {
        evce_sbac_encode_bin(1, sbac, sbac_ctx->bi_idx, bs);
    }
    else
    {
        evce_sbac_encode_bin(0, sbac, sbac_ctx->bi_idx, bs);
        if(bi_idx == 1)
        {
            evce_sbac_encode_bin(1, sbac, sbac_ctx->bi_idx + 1, bs);
        }
        else
        {
            evce_sbac_encode_bin(0, sbac, sbac_ctx->bi_idx + 1, bs);
        }
    }
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("bi_idx ");
    EVC_TRACE_INT(bi_idx);
    EVC_TRACE_STR("\n");
#endif
    return EVC_OK;
}

static int evce_eco_abs_mvd(u32 sym, EVCE_SBAC *sbac, SBAC_CTX_MODEL *model, EVC_BSW *bs)
{
    u32 val = sym;
    s32 len_i, len_c, info, nn;
    u32  code;
    s32 i;

    nn = ((val + 1) >> 1);
    for(len_i = 0; len_i < 16 && nn != 0; len_i++)
    {
        nn >>= 1;
    }

    info = val + 1 - (1 << len_i);
    code = (1 << len_i) | ((info)& ((1 << len_i) - 1));

    len_c = (len_i << 1) + 1;

    for(i = 0; i < len_c; i++)
    {
        int bin = (code >> (len_c - 1 - i)) & 0x01;
        if(i <= 1)
        {
            evce_sbac_encode_bin(bin, sbac, model, bs); /* use one context model for two bins */
        }
        else
        {
            sbac_encode_bin_ep(bin, sbac, bs);
        }
    }

    return EVC_OK;
}

int evce_eco_mvd(EVC_BSW *bs, s16 mvd[MV_D])
{
    EVCE_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;
    int            t0;
    u32            mv;

    sbac     = GET_SBAC_ENC(bs);
    sbac_ctx = &sbac->ctx;

    t0 = 0;

    mv = mvd[MV_X];
    if(mvd[MV_X] < 0)
    {
        t0 = 1;
        mv = -mvd[MV_X];
    }
    evce_eco_abs_mvd(mv, sbac, sbac_ctx->mvd, bs);

    if(mv)
    {
        sbac_encode_bin_ep(t0, sbac, bs);
    }

    t0 = 0;
    mv = mvd[MV_Y];
    if(mvd[MV_Y] < 0)
    {
        t0 = 1;
        mv = -mvd[MV_Y];
    }

    evce_eco_abs_mvd(mv, sbac, sbac_ctx->mvd, bs);

    if(mv)
    {
        sbac_encode_bin_ep(t0, sbac, bs);
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvd x ");
    EVC_TRACE_INT(mvd[MV_X]);
    EVC_TRACE_STR("mvd y ");
    EVC_TRACE_INT(mvd[MV_Y]);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

static int cu_init(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int cup, int cuw, int cuh)
{
    EVCE_CU_DATA *cu_data = &ctx->map_cu_data[core->lcu_num];

    core->cuw = cuw;
    core->cuh = cuh;
    core->log2_cuw = CONV_LOG2(cuw);
    core->log2_cuh = CONV_LOG2(cuh);
    core->x_scu = PEL2SCU(x);
    core->y_scu = PEL2SCU(y);
    core->scup = ((u32)core->y_scu * ctx->w_scu) + core->x_scu;
    core->avail_cu = 0;
    core->skip_flag = 0;
    core->ibc_flag = 0;
    core->mmvd_flag = 0;
    core->affine_flag = cu_data->affine_flag[cup];
    core->nnz[Y_C] = core->nnz[U_C] = core->nnz[V_C] = 0;
    core->ats_inter_info = cu_data->ats_inter_info[cup];
    core->cu_mode = evce_check_luma(ctx, core) ? cu_data->pred_mode[cup] : cu_data->pred_mode_chroma[cup];

    if(core->cu_mode == MODE_INTRA)
    {
        core->avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->log2_cuw, core->log2_cuh, ctx->map_scu, ctx->map_tidx);
    }
    else if (core->cu_mode == MODE_IBC)
    {
      core->ibc_flag = 1;

      if (!evce_check_luma(ctx, core))
      {
          evc_assert(0);
      }
      core->mmvd_flag = 0; // core->new_skip_flag = 0;
      core->affine_flag = 0;
      core->avail_cu = evc_get_avail_ibc(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu, ctx->map_tidx);
    }
    else
    {
        evc_assert(evce_check_luma(ctx, core));

        if((cu_data->pred_mode[cup] == MODE_SKIP) || (cu_data->pred_mode[cup] == MODE_SKIP_MMVD))
        {
            core->skip_flag = 1;
        }

        if(cu_data->pred_mode[cup] == MODE_SKIP_MMVD)
        {
            core->mmvd_flag = 1;
        }

        core->avail_cu = evc_get_avail_inter(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu, ctx->map_tidx);
    }

    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, ctx->map_tidx);
#if DMVR_FLAG
    core->dmvr_flag = 0;
#endif
    return EVC_OK;
}

static void coef_rect_to_series(EVCE_CTX * ctx, s16 *coef_src[N_C], int x, int y, int cuw, int cuh, s16 coef_dst[N_C][MAX_CU_DIM], EVCE_CORE * core)
{
    int i, j, sidx, didx;

    sidx = (x&(ctx->max_cuwh - 1)) + ((y&(ctx->max_cuwh - 1)) << ctx->log2_max_cuwh);
    didx = 0;

    if(evce_check_luma(ctx, core))
    {
        for(j = 0; j < cuh; j++)
        {
            for(i = 0; i < cuw; i++)
            {
                coef_dst[Y_C][didx++] = coef_src[Y_C][sidx + i];
            }
            sidx += ctx->max_cuwh;
        }

    }
    if(evce_check_chroma(ctx, core)
#if BD_CF_EXT
       && ctx->sps.chroma_format_idc
#endif
       )
    {
#if BD_CF_EXT
        x >>= (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc));
        y >>= (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc));
        cuw >>= (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc));
        cuh >>= (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc));

        sidx = (x&((ctx->max_cuwh >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc))) - 1)) + ((y&((ctx->max_cuwh >> (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc))) - 1)) << (ctx->log2_max_cuwh - (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc))));
#else
        x >>= 1;
        y >>= 1;
        cuw >>= 1;
        cuh >>= 1;

        sidx = (x&((ctx->max_cuwh >> 1) - 1)) + ((y&((ctx->max_cuwh >> 1) - 1)) << (ctx->log2_max_cuwh - 1));
#endif
        didx = 0;

        for(j = 0; j < cuh; j++)
        {
            for(i = 0; i < cuw; i++)
            {
                coef_dst[U_C][didx] = coef_src[U_C][sidx + i];
                coef_dst[V_C][didx] = coef_src[V_C][sidx + i];
                didx++;
            }
#if BD_CF_EXT
            sidx += (ctx->max_cuwh >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)));
#else
            sidx += (ctx->max_cuwh >> 1);
#endif
        }
    }
}

int evce_eco_split_mode(EVC_BSW *bs, EVCE_CTX *c, EVCE_CORE *core, int cud, int cup, int cuw, int cuh, int lcu_s,
                        const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow,
                        int* curr_split_allow, int qt_depth, int btt_depth, int x, int y)
{
    EVCE_SBAC *sbac;
    int sps_cm_init_flag;
    int ret = EVC_OK;
    s8 split_mode;
    int ctx = 0;
    int order_idx = cuw >= cuh ? 0 : 1;
    
    int i, curr_cnt, split_mode_sum;
    int split_allow[SPLIT_CHECK_NUM];

    if(cuw < 8 && cuh < 8)
    {
        return ret;
    }

    evc_assert(evce_check_luma(c, core));

    sbac = GET_SBAC_ENC(bs);
    sps_cm_init_flag = sbac->ctx.sps_cm_init_flag;
       
    if(sbac->is_bitcount)
    {
        evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, lcu_s, core->cu_data_temp[CONV_LOG2(cuw) - 2][CONV_LOG2(cuh) - 2].split_mode);
    }
    else
    {
        evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, lcu_s, c->map_cu_data[core->lcu_num].split_mode);
    }

    if (!c->sps.sps_btt_flag)
    {
        evce_sbac_encode_bin(split_mode != NO_SPLIT, sbac, sbac->ctx.split_cu_flag, bs); /* split_cu_flag */

        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("x pos ");
        EVC_TRACE_INT(core->x_pel + ((cup % (c->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
        EVC_TRACE_STR("y pos ");
        EVC_TRACE_INT(core->y_pel + ((cup / (c->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
        EVC_TRACE_STR("width ");
        EVC_TRACE_INT(cuw);
        EVC_TRACE_STR("height ");
        EVC_TRACE_INT(cuh);
        EVC_TRACE_STR("depth ");
        EVC_TRACE_INT(cud);
        EVC_TRACE_STR("split mode ");
        EVC_TRACE_INT(split_mode);
        EVC_TRACE_STR("\n");

        return ret;
    }

    evc_check_split_mode(split_allow, CONV_LOG2(cuw), CONV_LOG2(cuh), 0, 0, 0, c->log2_max_cuwh
                         , parent_split, same_layer_split, node_idx, parent_split_allow, qt_depth, btt_depth
                         , x, y, c->w, c->h
                         , NULL, c->sps.sps_btt_flag, core->tree_cons.mode_cons);

    split_mode_sum = 1;
    curr_cnt = 1;

    for(i = 1; i < SPLIT_CHECK_NUM; i++)
    {
        split_mode_sum += split_allow[evc_split_order[order_idx][i]];
        if(i <= evc_split_order[order_idx][split_mode])
        {
            curr_cnt += split_allow[evc_split_order[order_idx][i]];
        }
        curr_split_allow[i] = split_allow[i];
    }
  
    if (split_mode_sum == 1)
    {
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("x pos ");
        EVC_TRACE_INT(core->x_pel + ((cup % (c->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
        EVC_TRACE_STR("y pos ");
        EVC_TRACE_INT(core->y_pel + ((cup / (c->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
        EVC_TRACE_STR("width ");
        EVC_TRACE_INT(cuw);
        EVC_TRACE_STR("height ");
        EVC_TRACE_INT(cuh);
        EVC_TRACE_STR("depth ");
        EVC_TRACE_INT(cud);
        EVC_TRACE_STR("split mode ");
        EVC_TRACE_INT(split_mode);
        EVC_TRACE_STR("\n");

        return ret;
    }

    {
        int log2_cuw = CONV_LOG2(cuw);
        int log2_cuh = CONV_LOG2(cuh);

        if(sps_cm_init_flag == 1)
        {
            int i;
            u16 x_scu = x >> MIN_CU_LOG2;
            u16 y_scu = y >> MIN_CU_LOG2;
            u16 scuw = cuw >> MIN_CU_LOG2;
            u16 w_scu = c->w >> MIN_CU_LOG2;
            u8  smaller[3] = {0, 0, 0};
            u8  avail[3] = {0, 0, 0};
            int scun[3];
            int w[3], h[3];
            int scup = x_scu + y_scu * w_scu;

            avail[0] = y_scu > 0 && (c->map_tidx[scup] == c->map_tidx[scup - w_scu]);    //up
            if (x_scu > 0)
            {
                avail[1] = MCU_GET_COD(c->map_scu[scup - 1]) && (c->map_tidx[scup] == c->map_tidx[scup - 1]); //left
            }
            if (x_scu + scuw < w_scu)
            {
                avail[2] = MCU_GET_COD(c->map_scu[scup + scuw]) && (c->map_tidx[scup] == c->map_tidx[scup + scuw]); //right
            }
            scun[0] = scup - w_scu;
            scun[1] = scup - 1;
            scun[2] = scup + scuw;
            for(i = 0; i < 3; i++)
            {
                if(avail[i])
                {
                    w[i] = 1 << MCU_GET_LOGW(c->map_cu_mode[scun[i]]);
                    h[i] = 1 << MCU_GET_LOGH(c->map_cu_mode[scun[i]]);
                    if(i == 0)
                        smaller[i] = w[i] < cuw;
                    else
                        smaller[i] = h[i] < cuh;
                }
            }
            ctx = min(smaller[0] + smaller[1] + smaller[2], 2);
            ctx = ctx + 3 * evc_tbl_split_flag_ctx[log2_cuw - 2][log2_cuh - 2];
        }
        else
        {
            ctx = 0;
        }

        evce_sbac_encode_bin(split_mode != NO_SPLIT, sbac, sbac->ctx.btt_split_flag + ctx, bs); /* btt_split_flag */
        if(split_mode != NO_SPLIT)
        {
            u8 HBT = split_allow[SPLIT_BI_HOR];
            u8 VBT = split_allow[SPLIT_BI_VER];
            u8 HTT = split_allow[SPLIT_TRI_HOR];
            u8 VTT = split_allow[SPLIT_TRI_VER];
            u8 sum = HBT + VBT + HTT + VTT;
            u8 ctx_dir = sps_cm_init_flag == 1 ? (log2_cuw - log2_cuh + 2) : 0;
            u8 ctx_typ = 0;
            u8 split_dir = (split_mode == SPLIT_BI_VER) || (split_mode == SPLIT_TRI_VER);
            u8 split_typ = (split_mode == SPLIT_TRI_HOR) || (split_mode == SPLIT_TRI_VER);

            if(sum == 4)
            {
                evce_sbac_encode_bin(split_dir, sbac, sbac->ctx.btt_split_dir + ctx_dir, bs); /* btt_split_dir */
                evce_sbac_encode_bin(split_typ, sbac, sbac->ctx.btt_split_type + ctx_typ, bs); /* btt_split_type */
            }
            else if(sum == 3)
            {
                evce_sbac_encode_bin(split_dir, sbac, sbac->ctx.btt_split_dir + ctx_dir, bs); /* btt_split_dir */
                if(!HBT || !HTT)
                {
                    if(split_dir)
                        evce_sbac_encode_bin(split_typ, sbac, sbac->ctx.btt_split_type + ctx_typ, bs); /* btt_split_type */
                    else
                        assert(split_typ == !HBT);
                }
                else// if(!VBT || !VTT)
                {
                    if(!split_dir)
                        evce_sbac_encode_bin(split_typ, sbac, sbac->ctx.btt_split_type + ctx_typ, bs); /* btt_split_type */
                    else
                        assert(split_typ == !VBT);
                }
            }
            else if(sum == 2)
            {
                if((HBT && HTT) || (VBT && VTT))
                {
                    assert(split_dir == !HBT);
                    evce_sbac_encode_bin(split_typ, sbac, sbac->ctx.btt_split_type + ctx_typ, bs); /* btt_split_type */
                }
                else
                {
                    evce_sbac_encode_bin(split_dir, sbac, sbac->ctx.btt_split_dir + ctx_dir, bs); /* btt_split_dir */

                    if(!HTT && !VTT)
                        assert(split_typ == 0);
                    else if(HBT && VTT)
                        assert(split_typ == split_dir);
                    else if(VBT && HTT)
                        assert(split_typ == !split_dir);
                    else
                        assert(0);
                }
            }
            else // if(sum==1)
            {
                assert(split_dir == (VBT || VTT));
                assert(split_typ == (HTT || VTT));
            }
        }
    }
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("x pos ");
    EVC_TRACE_INT(core->x_pel + ((cup % (c->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
    EVC_TRACE_STR("y pos ");
    EVC_TRACE_INT(core->y_pel + ((cup / (c->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
    EVC_TRACE_STR("width ");
    EVC_TRACE_INT(cuw);
    EVC_TRACE_STR("height ");
    EVC_TRACE_INT(cuh);
    EVC_TRACE_STR("depth ");
    EVC_TRACE_INT(cud);
    EVC_TRACE_STR("split mode ");
    EVC_TRACE_INT(split_mode);
    EVC_TRACE_STR("\n");
    return ret;
}

int evce_eco_suco_flag(EVC_BSW *bs, EVCE_CTX *c, EVCE_CORE *core, int cud, int cup, int cuw, int cuh, int lcu_s, s8 split_mode, int boundary, u8 log2_max_cuwh)
{
    EVCE_SBAC *sbac;
    int ret = EVC_OK;
    s8 suco_flag;
    int ctx;
    u8 allow_suco = c->sps.sps_suco_flag ? evc_check_suco_cond(cuw, cuh, split_mode, boundary, log2_max_cuwh, c->sps.log2_diff_ctu_size_max_suco_cb_size, c->sps.log2_diff_max_suco_min_suco_cb_size, c->sps.log2_min_cb_size_minus2 + 2) : 0;

    if(!allow_suco)
    {
        return ret;
    }

    sbac = GET_SBAC_ENC(bs);

    if(sbac->is_bitcount)
        evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, lcu_s, core->cu_data_temp[CONV_LOG2(cuw) - 2][CONV_LOG2(cuh) - 2].suco_flag);
    else
        evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, lcu_s, c->map_cu_data[core->lcu_num].suco_flag);

    if(sbac->ctx.sps_cm_init_flag == 1)
    {
        ctx = CONV_LOG2(EVC_MAX(cuw, cuh)) - 2;
        ctx = (cuw == cuh) ? ctx * 2 : ctx * 2 + 1;
    }
    else
    {
        ctx = 0;
    }
    evce_sbac_encode_bin(suco_flag, sbac, sbac->ctx.suco_flag + ctx, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("suco flag ");
    EVC_TRACE_INT(suco_flag);
    EVC_TRACE_STR("\n");

    return ret;
}

int evce_eco_mode_constr(EVC_BSW * bs, MODE_CONS mode_cons, int ctx)
{
    EVCE_SBAC * sbac = GET_SBAC_ENC(bs);
    u8 bit = mode_cons == eOnlyIntra;
    evce_sbac_encode_bin(bit, sbac, sbac->ctx.mode_cons + ctx, bs);
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mode_constr ");
    EVC_TRACE_INT(bit);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

int evce_eco_unit(EVCE_CTX * ctx, EVCE_CORE * core, int x, int y, int cup, int cuw, int cuh, TREE_CONS tree_cons)
{
    core->tree_cons = tree_cons;
    s16(*coef)[MAX_CU_DIM] = core->ctmp;

    EVC_BSW *bs; 

    u32 *map_scu;
    int slice_type, refi0, refi1;
    int i, j, w, h;
    EVCE_CU_DATA *cu_data = &ctx->map_cu_data[core->lcu_num];
    u32 *map_cu_mode;
    u32 *map_affine;
#if TRACE_ENC_CU_DATA
    core->trace_idx = cu_data->trace_idx[cup];
#endif
#if TRACE_ENC_HISTORIC
    evc_mcpy(&core->history_buffer, &(cu_data->history_buf[cup]), sizeof(core->history_buffer));
#endif
#if TRACE_ENC_CU_DATA_CHECK
    evc_assert(core->trace_idx != 0);
#endif
#if DQP
    int enc_dqp;
#endif
    slice_type = ctx->slice_type;
     
    bs = &ctx->bs;

    cu_init(ctx, core, x, y, cup, cuw, cuh);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("poc: ");
    EVC_TRACE_INT(ctx->poc.poc_val);
    EVC_TRACE_STR("x pos ");
    EVC_TRACE_INT(core->x_pel + ((cup % (ctx->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
    EVC_TRACE_STR("y pos ");
    EVC_TRACE_INT(core->y_pel + ((cup / (ctx->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
    EVC_TRACE_STR("width ");
    EVC_TRACE_INT(cuw);
    EVC_TRACE_STR("height ");
    EVC_TRACE_INT(cuh);

#if ENC_DEC_TRACE
    if (ctx->sh.slice_type != SLICE_I && ctx->sps.sps_btt_flag)
    {
        EVC_TRACE_STR("tree status ");
        EVC_TRACE_INT(core->tree_cons.tree_type);
        EVC_TRACE_STR("mode status ");
        EVC_TRACE_INT(core->tree_cons.mode_cons);
    }
#endif

    EVC_TRACE_STR("\n");

    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, core->ctx_flags
                         , ctx->sh.slice_type, ctx->sps.tool_cm_init , ctx->param.use_ibc_flag, ctx->sps.ibc_log_max_size, ctx->map_tidx);    

    if (ctx->sps.tool_admvp && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2)
    {
        evc_assert(cu_data->pred_mode[cup] == MODE_INTRA || cu_data->pred_mode[cup] == MODE_IBC);
    }
    
    if (core->skip_flag == 0)
    {
        /* get coefficients and tq */
        coef_rect_to_series(ctx, cu_data->coef, x, y, cuw, cuh, coef, core);

        for(i = 0; i < N_C; i++)
        {
            core->nnz[i] = cu_data->nnz[i][cup];

            for (j = 0; j < MAX_SUB_TB_NUM; j++)
            {
                core->nnz_sub[i][j] = cu_data->nnz_sub[i][j][cup];
            }
        }
    }
    else
    {
        evc_mset(core->nnz, 0, sizeof(int) * N_C);
        evc_mset(core->nnz_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);
    }

    /* entropy coding a CU */
    if(slice_type != SLICE_I && (ctx->sps.tool_admvp == 0 || !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2) || ctx->param.use_ibc_flag) && !evce_check_only_intra(ctx, core) )
    {
        if(!(ctx->sps.tool_admvp && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2))
        {
            evce_eco_skip_flag(bs, core->skip_flag,core->ctx_flags[CNID_SKIP_FLAG]);
        }

        if(core->skip_flag)
        {
            if(ctx->sps.tool_mmvd)
            {
                evce_eco_mmvd_flag(bs, core->mmvd_flag); 
            }

            if(core->mmvd_flag)
            {
                evce_eco_mmvd_info(bs, cu_data->mmvd_idx[cup], ctx->sh.mmvd_group_enable_flag && !(cuw*cuh <= NUM_SAMPLES_BLOCK));
            }
            else
            {
                if(cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine)
                {
                    evce_eco_affine_flag(bs, core->affine_flag != 0, core->ctx_flags[CNID_AFFN_FLAG]); /* skip affine_flag */
                }

                if(core->affine_flag)
                {
                    evce_eco_affine_mrg_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                }
                else
                {
                    if(!ctx->sps.tool_admvp)
                    {
                        evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_0]);

                        if(slice_type == SLICE_B)
                        {
                            evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_1]);
                        }
                    }
                    else
                    {
                        evce_eco_merge_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                    }
                }
            }
        }
        else
        {
            if (evce_check_all_preds(ctx, core))
                if (!(ctx->sps.tool_admvp && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2))
                {
                    evce_eco_pred_mode(bs, core->cu_mode, core->ctx_flags[CNID_PRED_MODE]);
                }

            if (((( core->cu_mode
                != MODE_INTRA) || (ctx->sps.tool_admvp && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2))
                && !evce_check_only_inter(ctx, core) ) 
                && evce_check_luma(ctx, core)
                && ctx->param.use_ibc_flag && core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
            {
                evce_eco_ibc_flag(bs, core->ibc_flag, core->ctx_flags[CNID_IBC_FLAG]);
            }

            if(core->cu_mode != MODE_INTRA && core->cu_mode != MODE_IBC)
            {
                if(ctx->sps.tool_amvr)
                {
                    evce_eco_mvr_idx(bs, cu_data->mvr_idx[cup]);
                }

                {
                    if(ctx->sps.tool_admvp == 0)
                    {
                        evce_eco_direct_mode_flag(bs, cu_data->pred_mode[cup] == MODE_DIR);
                    }
                    else
                    {
                        if(cu_data->mvr_idx[cup] == 0)
                        {
                            evce_eco_merge_mode_flag(bs, cu_data->pred_mode[cup] == MODE_DIR || cu_data->pred_mode[cup] == MODE_DIR_MMVD);
                        }
                    }

                    if(ctx->sps.tool_mmvd)
                    {
                        if((cu_data->pred_mode[cup] == MODE_DIR) || (cu_data->pred_mode[cup] == MODE_DIR_MMVD))
                        {
                            evce_eco_mmvd_flag(bs, cu_data->pred_mode[cup] == MODE_DIR_MMVD);
                        }

                        if((cu_data->pred_mode[cup] == MODE_DIR_MMVD))
                        {
                            evce_eco_mmvd_info(bs, cu_data->mmvd_idx[cup], ctx->sh.mmvd_group_enable_flag && !(cuw*cuh <= NUM_SAMPLES_BLOCK));
                        }
                    }

                    if(cu_data->pred_mode[cup] == MODE_DIR && cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine)
                    {
                        evce_eco_affine_flag(bs, core->affine_flag != 0, core->ctx_flags[CNID_AFFN_FLAG]); /* direct affine_flag */
                        if(core->affine_flag)
                        {
                            evce_eco_affine_mrg_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                        }
                    }
                    if(ctx->sps.tool_admvp == 1 && cu_data->pred_mode[cup] == MODE_DIR && !core->affine_flag && cu_data->mvr_idx[cup] == 0 )
                    {
                        evce_eco_merge_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                    }
                }

                if(((cu_data->pred_mode[cup] % ORG_PRED_NUM) != MODE_DIR) && ((cu_data->pred_mode[cup] % ORG_PRED_NUM) != MODE_DIR_MMVD))
                {
                    evce_eco_inter_pred_idc(bs, cu_data->refi[cup], slice_type, cuw, cuh, ctx->sps.tool_admvp);

                    // affine inter mode
                    if(cuw >= 16 && cuh >= 16 && cu_data->mvr_idx[cup] == 0 && ctx->sps.tool_affine)
                    {
                        evce_eco_affine_flag(bs, core->affine_flag != 0, core->ctx_flags[CNID_AFFN_FLAG]); /* inter affine_flag */
                    }

                    if(core->affine_flag)
                    {
                        evce_eco_affine_mode(bs, core->affine_flag - 1); /* inter affine_mode */
                    }

                    if(core->affine_flag)
                    {
                        int vertex;
                        int vertex_num = core->affine_flag + 1;
                        int aff_scup[VER_NUM] = { 0 };

                        aff_scup[0] = cup;
                        aff_scup[1] = cup + ((cuw >> MIN_CU_LOG2) - 1);
                        aff_scup[2] = cup + (((cuh >> MIN_CU_LOG2) - 1) << ctx->log2_culine);

                        refi0 = cu_data->refi[cup][REFP_0];
                        refi1 = cu_data->refi[cup][REFP_1];

                        if(IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
                        {
                            int b_zero = 1;

                            evce_eco_refi(bs, ctx->rpm.num_refp[REFP_0], refi0);
                            evce_eco_affine_mvp_idx( bs, cu_data->mvp_idx[cup][REFP_0] );
  
                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                int mvd_x = cu_data->mvd[aff_scup[vertex]][REFP_0][MV_X];
                                int mvd_y = cu_data->mvd[aff_scup[vertex]][REFP_0][MV_Y];
                                if(mvd_x != 0 || mvd_y != 0)
                                {
                                    b_zero = 0;
                                    break;
                                }
                            }
                            evce_eco_affine_mvd_flag(bs, b_zero, REFP_0);

                            if(b_zero == 0)
                            {
                                for(vertex = 0; vertex < vertex_num; vertex++)
                                {
                                    evce_eco_mvd(bs, cu_data->mvd[aff_scup[vertex]][REFP_0]);
                                }
                            }
                        }

                        if(slice_type == SLICE_B && REFI_IS_VALID(refi1))
                        {
                            int b_zero = 1;

                            evce_eco_refi(bs, ctx->rpm.num_refp[REFP_1], refi1);
                            evce_eco_affine_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_1]);

                            for(vertex = 0; vertex < vertex_num; vertex++)
                            {
                                int mvd_x = cu_data->mvd[aff_scup[vertex]][REFP_1][MV_X];
                                int mvd_y = cu_data->mvd[aff_scup[vertex]][REFP_1][MV_Y];
                                if(mvd_x != 0 || mvd_y != 0)
                                {
                                    b_zero = 0;
                                    break;
                                }
                            }
                            evce_eco_affine_mvd_flag(bs, b_zero, REFP_1);

                            if(b_zero == 0)
                                for(vertex = 0; vertex < vertex_num; vertex++)
                                {
                                    evce_eco_mvd(bs, cu_data->mvd[aff_scup[vertex]][REFP_1]);
                                }
                        }
                    }
                    else
                    {
                        if(ctx->sps.tool_admvp == 1 && REFI_IS_VALID(cu_data->refi[cup][REFP_0]) && REFI_IS_VALID(cu_data->refi[cup][REFP_1]))
                        {
                            evce_eco_bi_idx(bs, cu_data->bi_idx[cup] - 1);
                        }

                        refi0 = cu_data->refi[cup][REFP_0];
                        refi1 = cu_data->refi[cup][REFP_1];
                        if(IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
                        {
                            if(ctx->sps.tool_admvp == 0)
                            {
                                evce_eco_refi(bs, ctx->rpm.num_refp[REFP_0], refi0);
                                evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                                evce_eco_mvd(bs, cu_data->mvd[cup][REFP_0]);
                            }
                            else
                            {
                                if(cu_data->bi_idx[cup] != BI_FL0 && cu_data->bi_idx[cup] != BI_FL1)
                                {
                                    evce_eco_refi(bs, ctx->rpm.num_refp[REFP_0], refi0);
                                }

                                cu_data->mvd[cup][REFP_0][MV_Y] >>= cu_data->mvr_idx[cup];
                                cu_data->mvd[cup][REFP_0][MV_X] >>= cu_data->mvr_idx[cup];

                                if(cu_data->bi_idx[cup] != BI_FL0)
                                {
                                    evce_eco_mvd(bs, cu_data->mvd[cup][REFP_0]);
                                }

                                cu_data->mvd[cup][REFP_0][MV_Y] <<= cu_data->mvr_idx[cup];
                                cu_data->mvd[cup][REFP_0][MV_X] <<= cu_data->mvr_idx[cup];
                            }
                        }

                        if(slice_type == SLICE_B && REFI_IS_VALID(refi1))
                        {
                            if(ctx->sps.tool_admvp == 0)
                            {
                                evce_eco_refi(bs, ctx->rpm.num_refp[REFP_1], refi1);
                                evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_1]);
                                evce_eco_mvd(bs, cu_data->mvd[cup][REFP_1]);
                            }
                            else
                            {
                                if(cu_data->bi_idx[cup] != BI_FL0 && cu_data->bi_idx[cup] != BI_FL1)
                                {
                                    evce_eco_refi(bs, ctx->rpm.num_refp[REFP_1], refi1);
                                }

                                cu_data->mvd[cup][REFP_1][MV_Y] >>= cu_data->mvr_idx[cup];
                                cu_data->mvd[cup][REFP_1][MV_X] >>= cu_data->mvr_idx[cup];

                                if(cu_data->bi_idx[cup] != BI_FL1)
                                {
                                    evce_eco_mvd(bs, cu_data->mvd[cup][REFP_1]);
                                }

                                cu_data->mvd[cup][REFP_1][MV_Y] <<= cu_data->mvr_idx[cup];
                                cu_data->mvd[cup][REFP_1][MV_X] <<= cu_data->mvr_idx[cup];
                            }
                        }
                    }
                }
            }
        }
    }
    else if (((ctx->sh.slice_type == SLICE_I || evce_check_only_intra(ctx, core)) && ctx->param.use_ibc_flag))
    {
        if (core->skip_flag == 0 && evce_check_luma(ctx, core))
        {
            if (core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
            {
                evce_eco_ibc_flag(bs, core->ibc_flag, core->ctx_flags[CNID_IBC_FLAG]);
            }
        }
    }

    if(core->cu_mode == MODE_INTRA)
    {
        evc_assert(cu_data->ipm[0][cup] != IPD_INVALID);
        evc_assert(cu_data->ipm[1][cup] != IPD_INVALID);

        if(ctx->sps.tool_eipd)
        {
            evc_get_mpm(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                        core->mpm, core->avail_lr, core->mpm_ext, core->pims, ctx->map_tidx);
            if (evce_check_luma(ctx, core))
            {
            evce_eco_intra_dir(bs, cu_data->ipm[0][cup], core->mpm, core->mpm_ext, core->pims);
            }
            if (evce_check_chroma(ctx, core))
            {
                int luma_ipm = cu_data->ipm[0][cup];
                if (!evce_check_luma(ctx, core))
                {
                    int luma_cup = evc_get_luma_cup(core->x_scu - PEL2SCU(core->x_pel), core->y_scu - PEL2SCU(core->y_pel), PEL2SCU(cuw), PEL2SCU(cuh), 1 << ctx->log2_culine);
                    if (cu_data->pred_mode[luma_cup] == MODE_INTRA)
                    {
                        luma_ipm = cu_data->ipm[0][luma_cup];
                    }
                    else
                    {
                        luma_ipm = IPD_DC;
                    }
                }
                evc_assert(cu_data->ipm[1][cup] != IPD_INVALID);
#if BD_CF_EXT
                if(ctx->sps.chroma_format_idc != 0)
#endif
                    evce_eco_intra_dir_c(bs, cu_data->ipm[1][cup], luma_ipm);
            }
        }
        else
        {
            evc_get_mpm_b(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                          &core->mpm_b_list, core->avail_lr, core->mpm_ext, core->pims, ctx->map_tidx);

#if FIX_EIPD_OFF 
            if (evce_check_luma(ctx, core))
            {
#endif
                evce_eco_intra_dir_b(bs, cu_data->ipm[0][cup], core->mpm_b_list, core->mpm_ext, core->pims);
#if FIX_EIPD_OFF 
            }
#endif
        }
    }
    else if (core->ibc_flag)
    {
      if (core->skip_flag == 0)
      {
        if (core->cu_mode == MODE_IBC)        // Does this condition required?
        {
            if (!evce_check_luma(ctx, core))
            {
                evc_assert(0);
            }
            else
            {
                evce_eco_mvd(bs, cu_data->mvd[cup][REFP_0]);
            }
        }
      }
    }

    if((core->skip_flag == 0) && (core->mmvd_flag == 0))
    {
        int b_no_cbf = 0;
        b_no_cbf |= cu_data->affine_flag[cup] && core->cu_mode == MODE_DIR;
        b_no_cbf |= core->cu_mode == MODE_DIR_MMVD;
        b_no_cbf |= core->cu_mode == MODE_DIR;
        if (ctx->sps.tool_admvp == 0)
        {
            b_no_cbf = 0;
        }
#if DQP
        enc_dqp = 1;
#endif
        evce_eco_coef(bs, coef, core->log2_cuw, core->log2_cuh, core->cu_mode
                      , core->nnz_sub, b_no_cbf, RUN_L | RUN_CB | RUN_CR, ctx->sps.tool_ats, cu_data->ats_intra_cu[cup], (cu_data->ats_mode_h[cup] << 1 | cu_data->ats_mode_v[cup]), core->ats_inter_info
#if DQP
                      , ctx
#if BD_CF_EXT
                      , core, enc_dqp, cu_data->qp_y[cup] - 6 * ctx->sps.bit_depth_luma_minus8
#else
                      , core, enc_dqp, cu_data->qp_y[cup] - 6 * (BIT_DEPTH - 8)
#endif
#endif
                      , core->tree_cons
#if BD_CF_EXT
                      , ctx->sps.chroma_format_idc
#endif
        );
    }

    map_scu = ctx->map_scu + core->scup;
    w = (core->cuw >> MIN_CU_LOG2);
    h = (core->cuh >> MIN_CU_LOG2);
    map_affine = ctx->map_affine + core->scup;
    map_cu_mode = ctx->map_cu_mode + core->scup;

    if (evce_check_luma(ctx, core))
    {
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            if((core->skip_flag) || (core->mmvd_flag))
            {
                MCU_SET_SF(map_scu[j]);
            }
            else
            {
                MCU_CLR_SF(map_scu[j]);
            }
            int sub_idx = ((!!(i & 32)) << 1) | (!!(j & 32));
            if (core->nnz_sub[Y_C][sub_idx] > 0)
            {
                MCU_SET_CBFL(map_scu[j]);
            }
            else
            {
                MCU_CLR_CBFL(map_scu[j]);
            }

            MCU_SET_COD(map_scu[j]);
#if DQP
            if(ctx->pps.cu_qp_delta_enabled_flag)
            {
                MCU_RESET_QP(map_scu[j]);
                MCU_SET_QP(map_scu[j], ctx->tile[core->tile_idx].qp_prev_eco);
            }
#endif

            if(core->affine_flag)
            {
                MCU_SET_AFF(map_scu[j], core->affine_flag);

                MCU_SET_AFF_LOGW(map_affine[j], core->log2_cuw);
                MCU_SET_AFF_LOGH(map_affine[j], core->log2_cuh);
                MCU_SET_AFF_XOFF(map_affine[j], j);
                MCU_SET_AFF_YOFF(map_affine[j], i);
            }
            else
            {
                MCU_CLR_AFF(map_scu[j]);
            }

            if (core->ibc_flag)
            {
              MCU_SET_IBC(map_scu[j]);
            }
            else
            {
              MCU_CLR_IBC(map_scu[j]);
            }
            
            MCU_SET_LOGW(map_cu_mode[j], core->log2_cuw);
            MCU_SET_LOGH(map_cu_mode[j], core->log2_cuh);

            if (core->mmvd_flag)
            {
                MCU_SET_MMVDS(map_cu_mode[j]);
            }
            else
            {
                MCU_CLR_MMVDS(map_cu_mode[j]);
            }
        }
        map_scu += ctx->w_scu;
        map_affine += ctx->w_scu;
        map_cu_mode += ctx->w_scu;
    }
    if (core->ats_inter_info)
    {
        assert(core->nnz_sub[Y_C][0] == core->nnz[Y_C]);
        assert(core->nnz_sub[U_C][0] == core->nnz[U_C]);
        assert(core->nnz_sub[V_C][0] == core->nnz[V_C]);
        set_cu_cbf_flags(core->nnz[Y_C], core->ats_inter_info, core->log2_cuw, core->log2_cuh, ctx->map_scu + core->scup, ctx->w_scu);
    }

    }
    if (evce_check_chroma(ctx, core))
    {
        if (!evce_check_luma(ctx, core))
        {
            evc_assert((core->cu_mode == MODE_INTRA) || (core->cu_mode == MODE_IBC));
        }
    }

#if TRACE_ENC_CU_DATA
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("RDO check id ");
    EVC_TRACE_INT((int)core->trace_idx);
    EVC_TRACE_STR("\n");
    evc_assert(core->trace_idx != 0);
#endif
#if TRACE_ENC_HISTORIC
    //if (core->cu_mode != MODE_INTRA)
    {
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("Historic (");
        EVC_TRACE_INT((int)core->history_buffer.currCnt);
        EVC_TRACE_STR("): ");
        for (int i = 0; i < core->history_buffer.currCnt; ++i)
        {
            EVC_TRACE_STR("(");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_0][MV_X]);
            EVC_TRACE_STR(", ");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_0][MV_Y]);
            EVC_TRACE_STR("; ");
            EVC_TRACE_INT((int)core->history_buffer.history_refi_table[i][REFP_0]);
            EVC_TRACE_STR("), (");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_1][MV_X]);
            EVC_TRACE_STR(", ");
            EVC_TRACE_INT((int)core->history_buffer.history_mv_table[i][REFP_1][MV_Y]);
            EVC_TRACE_STR("; ");
            EVC_TRACE_INT((int)core->history_buffer.history_refi_table[i][REFP_1]);
            EVC_TRACE_STR("); ");
        }
        EVC_TRACE_STR("\n");
    }
#endif

#if MVF_TRACE
    // Trace MVF in encoder
    {
        s8(*map_refi)[REFP_NUM];
        s16(*map_mv)[REFP_NUM][MV_D];
#if DMVR_LAG
        s16(*map_unrefined_mv)[REFP_NUM][MV_D];
#endif
        u32  *map_scu;

        map_affine = ctx->map_affine + core->scup;

        map_refi = ctx->map_refi + core->scup;
        map_scu = ctx->map_scu + core->scup;
        map_mv = ctx->map_mv + core->scup;
#if DMVR_LAG
        map_unrefined_mv = ctx->map_unrefined_mv + core->scup;
#endif
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                EVC_TRACE_COUNTER;
                EVC_TRACE_STR(" x: ");
                EVC_TRACE_INT(j);
                EVC_TRACE_STR(" y: ");
                EVC_TRACE_INT(i);

                EVC_TRACE_STR(" ref0: ");
                EVC_TRACE_INT(map_refi[j][REFP_0]);
                EVC_TRACE_STR(" mv: ");
                EVC_TRACE_MV(map_mv[j][REFP_0][MV_X], map_mv[j][REFP_0][MV_Y]);

                EVC_TRACE_STR(" ref1: ");
                EVC_TRACE_INT(map_refi[j][REFP_1]);
                EVC_TRACE_STR(" mv: ");
                EVC_TRACE_MV(map_mv[j][REFP_1][MV_X], map_mv[j][REFP_1][MV_Y]);

                EVC_TRACE_STR(" affine: ");
                EVC_TRACE_INT(MCU_GET_AFF(map_scu[j]));
                if(MCU_GET_AFF(map_scu[j]))
                {
                    EVC_TRACE_STR(" logw: ");
                    EVC_TRACE_INT(MCU_GET_AFF_LOGW(map_affine[j]));
                    EVC_TRACE_STR(" logh: ");
                    EVC_TRACE_INT(MCU_GET_AFF_LOGH(map_affine[j]));
                    EVC_TRACE_STR(" xoff: ");
                    EVC_TRACE_INT(MCU_GET_AFF_XOFF(map_affine[j]));
                    EVC_TRACE_STR(" yoff: ");
                    EVC_TRACE_INT(MCU_GET_AFF_YOFF(map_affine[j]));
                }
#if DMVR_LAG
                if (MCU_GET_DMVRF(map_scu[j]))
                {
                    //map_unrefined_mv += ctx->w_scu;
                    EVC_TRACE_STR("; DMVR: ref0: ");
                    EVC_TRACE_INT(map_refi[j][REFP_0]);
                    EVC_TRACE_STR(" mv: ");
                    EVC_TRACE_MV(map_unrefined_mv[j][REFP_0][MV_X], map_unrefined_mv[j][REFP_0][MV_Y]);

                    EVC_TRACE_STR(" ref1: ");
                    EVC_TRACE_INT(map_refi[j][REFP_1]);
                    EVC_TRACE_STR(" mv: ");
                    EVC_TRACE_MV(map_unrefined_mv[j][REFP_1][MV_X], map_unrefined_mv[j][REFP_1][MV_Y]);
                }
#endif

                EVC_TRACE_STR("\n");
            }

            map_refi += ctx->w_scu;
            map_mv += ctx->w_scu;
            map_scu += ctx->w_scu;
            map_affine += ctx->w_scu;
#if DMVR_LAG
            map_unrefined_mv += ctx->w_scu;
#endif
        }
    }
#endif

    return EVC_OK;
}

#if ETM70_GOLOMB_FIX
int evc_lengthGolomb(int coeffVal, int k, BOOL signed_coeff)
{
    int numBins = 0;
    unsigned int symbol = abs(coeffVal);
    while (symbol >= (unsigned int)(1 << k))
    {
        numBins++;
        symbol -= 1 << k;
        k++;
    }
    numBins += (k + 1);
    if (signed_coeff && coeffVal != 0)
    {
        numBins++;
    }
    return numBins;
}
#else 
int evc_lengthGolomb(int coeffVal, int k)
{
    int m = k == 0 ? 1 : (2 << (k - 1));
    int q = coeffVal / m;
    if(coeffVal != 0)
    {
        return q + 2 + k;
    }
    else
    {
        return q + 1 + k;
    }
};
#endif

int evc_getGolombKMin(evc_AlfFilterShape *  alfShape, int numFilters, int *kMinTab, int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB])
{
    int kStart;
  const int maxGolombIdx = alfShape->filterType == 0 ? 2 : 3;

    int minBitsKStart = INT_MAX;
    int minKStart = -1;

    for(int k = 1; k < 8; k++)
    {
        int bitsKStart = 0; kStart = k;
        for(int scanPos = 0; scanPos < maxGolombIdx; scanPos++)
        {
            int kMin = kStart;
            int minBits = bitsCoeffScan[scanPos][kMin];

            if(bitsCoeffScan[scanPos][kStart + 1] < minBits)
            {
                kMin = kStart + 1;
                minBits = bitsCoeffScan[scanPos][kMin];
            }
            kStart = kMin;
            bitsKStart += minBits;
        }
        if(bitsKStart < minBitsKStart)
        {
            minBitsKStart = bitsKStart;
            minKStart = k;
        }
    }

    kStart = minKStart;
    for(int scanPos = 0; scanPos < maxGolombIdx; scanPos++)
    {
        int kMin = kStart;
        int minBits = bitsCoeffScan[scanPos][kMin];

        if(bitsCoeffScan[scanPos][kStart + 1] < minBits)
        {
            kMin = kStart + 1;
            minBits = bitsCoeffScan[scanPos][kMin];
        }

        kMinTab[scanPos] = kMin;
        kStart = kMin;
    }

    return minKStart;
};

#if ETM70_GOLOMB_FIX
void evc_alfGolombEncode(EVC_BSW * bs, int coeff, int k, const BOOL signed_coeff)
{

    unsigned int symbol = abs(coeff);
    while (symbol >= (unsigned int)(1 << k))
    {
        symbol -= 1 << k;
        k++;
#if TRACE_HLS
        evc_bsw_write1_trace(bs, 0, 0);
#else
        evc_bsw_write1(bs, 0);
#endif
    }
#if TRACE_HLS
    evc_bsw_write1_trace(bs, 1, 0);
#else
    evc_bsw_write1(bs, 1);
#endif
    // write one zero

    if (k > 0)
    {
#if TRACE_HLS
        evc_bsw_write1_trace(bs, symbol & 0x01, 0);
#else
        evc_bsw_write(bs, symbol, k);
#endif
    }

    if (signed_coeff && coeff != 0)
    {
#if TRACE_HLS
        evc_bsw_write1_trace(bs, sign, (coeff < 0) ? 0 : 1);
#else
        evc_bsw_write1(bs, (coeff < 0) ? 0 : 1);

#endif
    }
};
#else
void evc_alfGolombEncode(EVC_BSW * bs, int coeff, int k)
{
    int symbol = abs(coeff);

    int m = (int)pow(2.0, k);
    int q = symbol / m;

    for(int i = 0; i < q; i++)
    {
#if TRACE_HLS
        evc_bsw_write1_trace(bs, 1, 0);
#else
        evc_bsw_write1(bs, 1);
#endif
    }
#if TRACE_HLS
    evc_bsw_write1_trace(bs, 0, 0);
#else
    evc_bsw_write1(bs, 0);
#endif
    // write one zero

    for(int i = 0; i < k; i++)
    {
#if TRACE_HLS
        evc_bsw_write1_trace(bs, symbol & 0x01, 0);
#else
        evc_bsw_write1(bs, symbol & 0x01);
#endif
        symbol >>= 1;
    }

    if(coeff != 0)
    {
        int sign = (coeff > 0) ? 1 : 0;
#if TRACE_HLS
        evc_bsw_write1_trace(bs, sign, 0);
#else
        evc_bsw_write1(bs, sign);
#endif
    }
};
#endif

void evce_truncatedUnaryEqProb(EVC_BSW * bs, int symbol, const int maxSymbol)
{
    if(maxSymbol == 0)
    {
        return;
    }

    BOOL codeLast = (maxSymbol > symbol);
    int bins = 0;
    int numBins = 0;

    while(symbol--)
    {
        bins <<= 1;
        bins++;
        numBins++;
    }
    if(codeLast)
    {
        bins <<= 1;
        numBins++;
    }
    evc_bsw_write(bs, bins, numBins);
}

void evce_xWriteTruncBinCode(EVC_BSW * bs, u32 uiSymbol, const int uiMaxSymbol)
{
    int uiThresh;
    if(uiMaxSymbol > 256)
    {
        int uiThreshVal = 1 << 8;
        uiThresh = 8;
        while(uiThreshVal <= uiMaxSymbol)
        {
            uiThresh++;
            uiThreshVal <<= 1;
        }
        uiThresh--;
    }
    else
    {
        uiThresh = g_tbMax[uiMaxSymbol];
    }

    int uiVal = 1 << uiThresh;
    assert(uiVal <= uiMaxSymbol);
    assert((uiVal << 1) > uiMaxSymbol);
    assert(uiSymbol < uiMaxSymbol);
    int b = uiMaxSymbol - uiVal;
    assert(b < uiVal);
    if(uiSymbol < uiVal - b)
    {
        evc_bsw_write(bs, uiSymbol, uiThresh); //xWriteCode( uiSymbol, uiThresh );
    }
    else
    {
        uiSymbol += uiVal - b;
        assert(uiSymbol < (uiVal << 1));
        assert((uiSymbol >> 1) >= uiVal - b);
        evc_bsw_write(bs, uiSymbol, uiThresh + 1); //xWriteCode( uiSymbol, uiThresh + 1 );
    }
}

void evce_eco_alf_filter(EVC_BSW * bs, evc_AlfSliceParam asp, const BOOL isChroma)
{
    const evc_AlfSliceParam * alfSliceParam = &asp;
    if (!isChroma)
    {
        evc_bsw_write1(bs, alfSliceParam->coeffDeltaFlag); // "alf_coefficients_delta_flag"
        if (!alfSliceParam->coeffDeltaFlag)
        {
            if (alfSliceParam->numLumaFilters > 1)
            {
                evc_bsw_write1(bs, alfSliceParam->coeffDeltaPredModeFlag); // "coeff_delta_pred_mode_flag"
            }
        }
    }

    // this logic need to be moved to ALF files
    evc_AlfFilterShape alfShape;
    init_AlfFilterShape( &alfShape, isChroma ? 5 : ( alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );

    int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
    memset(bitsCoeffScan, 0, m_MAX_SCAN_VAL*m_MAX_EXP_GOLOMB * sizeof(int));

    const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;
    const short* coeff = isChroma ? alfSliceParam->chromaCoeff : alfSliceParam->lumaCoeff;
    const int numFilters = isChroma ? 1 : alfSliceParam->numLumaFilters;

    // vlc for all
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (isChroma || !alfSliceParam->coeffDeltaFlag || alfSliceParam->filterCoeffFlag[ind])
        {
            for (int i = 0; i < alfShape.numCoeff - 1; i++)
            {
                int coeffVal = abs(coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i]);

                for (int k = 1; k < 15; k++)
                {
#if ETM70_GOLOMB_FIX
                    bitsCoeffScan[alfShape.golombIdx[i]][k] += evc_lengthGolomb(coeffVal, k, TRUE);
#else
                    bitsCoeffScan[alfShape.golombIdx[i]][k] += evc_lengthGolomb(coeffVal, k);
#endif
                }
            }
        }
    }

    int kMinTab[MAX_NUM_ALF_COEFF];
    int kMin = evc_getGolombKMin(&alfShape, numFilters, kMinTab, bitsCoeffScan);

    // Golomb parameters
    u32 alf_luma_min_eg_order_minus1 = kMin - 1;
    evc_bsw_write_ue(bs, alf_luma_min_eg_order_minus1);

    for (int idx = 0; idx < maxGolombIdx; idx++)
    {
        BOOL alf_eg_order_increase_flag = (kMinTab[idx] != kMin) ? TRUE : FALSE;
        evc_bsw_write1(bs, alf_eg_order_increase_flag);
        kMin = kMinTab[idx];
    }

    if (!isChroma)
    {
        if (alfSliceParam->coeffDeltaFlag)
        {
            for (int ind = 0; ind < numFilters; ++ind)
            {
                evc_bsw_write1(bs, alfSliceParam->filterCoeffFlag[ind]);  // WRITE_FLAG(alfSliceParam.filterCoeffFlag[ind], "filter_coefficient_flag[i]");
            }
        }
    }

    // Filter coefficients
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (!isChroma && !alfSliceParam->filterCoeffFlag[ind] && alfSliceParam->coeffDeltaFlag)
        {
            continue;
        }

        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
#if ETM70_GOLOMB_FIX
            evc_alfGolombEncode(bs, coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]], TRUE);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#else
            evc_alfGolombEncode(bs, coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]]);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
#endif
        }
    }
}
#if M52291_HDR_DRA
int evce_eco_dra_aps_param(EVC_BSW * bs, EVC_APS_GEN * aps
#if BD_CF_EXT
                           , int bit_depth
#endif
)
{
    SignalledParamsDRA *p_dra_param = (SignalledParamsDRA *)aps->aps_data;
    evc_bsw_write(bs, (u32)p_dra_param->m_dra_descriptor1, 4);
    evc_bsw_write(bs, (u32)p_dra_param->m_dra_descriptor2, 4);
    evc_bsw_write_ue(bs, (u32)p_dra_param->m_numRanges - 1);
    evc_bsw_write1(bs, p_dra_param->m_equalRangesFlag);
#if BD_CF_EXT
    evc_bsw_write(bs, (u32)p_dra_param->m_inRanges[0], bit_depth); // delta_luma_dqp_change_point
#else
    evc_bsw_write(bs, (u32)p_dra_param->m_inRanges[0], QC_IN_RANGE_NUM_BITS); // delta_luma_dqp_change_point
#endif
    if (p_dra_param->m_equalRangesFlag == TRUE)
    {
#if BD_CF_EXT
        evc_bsw_write(bs, (u32)p_dra_param->m_deltaVal, bit_depth);
#else
        evc_bsw_write(bs, (u32)p_dra_param->m_deltaVal, QC_IN_RANGE_NUM_BITS);
#endif
    }
    else
    {
        for (int i = 1; i <= p_dra_param->m_numRanges; i++)
        {
#if BD_CF_EXT
            evc_bsw_write(bs, (u32)(p_dra_param->m_inRanges[i] - p_dra_param->m_inRanges[i - 1]), bit_depth);
#else
            evc_bsw_write(bs, (u32)(p_dra_param->m_inRanges[i] - p_dra_param->m_inRanges[i - 1]), QC_IN_RANGE_NUM_BITS);
#endif
        }
    }

    int numBits = p_dra_param->m_dra_descriptor1 + p_dra_param->m_dra_descriptor2;
    for (int i = 0; i < p_dra_param->m_numRanges; i++)
    {
        evc_bsw_write(bs, p_dra_param->m_dra_scale_value[i], numBits);
    }

    evc_bsw_write(bs, p_dra_param->m_dra_cb_scale_value, numBits);
    evc_bsw_write(bs, p_dra_param->m_dra_cr_scale_value, numBits);
    evc_bsw_write_ue(bs, (u32)p_dra_param->dra_table_idx);
    p_dra_param->m_signal_dra_flag = 0; // dra was sent
    return EVC_OK;
}
#endif

#if INTEGR_M53608
int evce_eco_alf_aps_param(EVC_BSW * bs, EVC_APS_GEN * aps)
{
    evc_AlfSliceParam *p_alfSliceParam = (evc_AlfSliceParam *)aps->aps_data;
    evc_AlfSliceParam alfSliceParam = *p_alfSliceParam;

    u32 alf_luma_filter_signal_flag = alfSliceParam.enabledFlag[0];
    u8  alf_chroma_filter_signal_flag = (alfSliceParam.enabledFlag[1] || alfSliceParam.enabledFlag[2]);

    evc_bsw_write1(bs, alf_luma_filter_signal_flag);
    evc_bsw_write1(bs, alf_chroma_filter_signal_flag);

    if (alfSliceParam.enabledFlag[0])
    {
        u32 alf_luma_num_filters_signalled_minus1 = alfSliceParam.numLumaFilters - 1;
        u32 alf_luma_type_flag = alfSliceParam.lumaFilterType;
        evc_bsw_write_ue(bs, alf_luma_num_filters_signalled_minus1);
        evc_bsw_write1(bs, alf_luma_type_flag); //  "filter_type_flag"

        if (alfSliceParam.numLumaFilters > 1)
        {
            s16 * alf_luma_coeff_delta_idx = alfSliceParam.filterCoeffDeltaIdx;
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
                evc_bsw_write(bs, alf_luma_coeff_delta_idx[i], evc_tbl_log2[alfSliceParam.numLumaFilters - 1] + 1);
            }
        }
        const int iNumFixedFilterPerClass = 16;
        {
#if ETM70_GOLOMB_FIX
            evc_alfGolombEncode(bs, alfSliceParam.fixedFilterPattern, 0, FALSE);
#else
            evc_alfGolombEncode(bs, alfSliceParam.fixedFilterPattern, 0);
#endif

            if (alfSliceParam.fixedFilterPattern == 2)
            {
                u8 * alf_luma_fixed_filter_usage_flag = alfSliceParam.fixedFilterUsageFlag;
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    evc_bsw_write1(bs, alf_luma_fixed_filter_usage_flag[classIdx]); 
                }
            }
            if (alfSliceParam.fixedFilterPattern > 0)
            {
                s32 * alf_luma_fixed_filter_set_idx = alfSliceParam.fixedFilterIdx;
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    if (alfSliceParam.fixedFilterUsageFlag[classIdx] > 0)
                    {
                        evc_bsw_write(bs, alf_luma_fixed_filter_set_idx[classIdx], evc_tbl_log2[iNumFixedFilterPerClass - 1] + 1);
                    }
                }
            }
        }

        evce_eco_alf_filter(bs, alfSliceParam, FALSE);
    }
    if (alf_chroma_filter_signal_flag)
    {
        {
            evce_eco_alf_filter(bs, alfSliceParam, TRUE);
        }
    }
    return EVC_OK;
}

#else
#if M52291_HDR_DRA
int evce_eco_alf_aps_param(EVC_BSW * bs, EVC_APS_GEN * aps)
{
    evc_AlfSliceParam *p_alfSliceParam = (evc_AlfSliceParam *)aps->aps_data;
    evc_AlfSliceParam alfSliceParam = *p_alfSliceParam;

#else
int evce_eco_alf_aps_param(EVC_BSW * bs, EVC_APS * aps)
{
    evc_AlfSliceParam alfSliceParam = aps->alf_aps_param;
#endif

    evc_bsw_write1(bs, alfSliceParam.enabledFlag[0]); //"alf_slice_enable_flag"
#if !M53608_ALF_5
    if (!alfSliceParam.enabledFlag[0])
    {
        return 0;
    }
#endif
#if M53608_ALF_14
    u8 alfChromaSignalled = (alfSliceParam.enabledFlag[1] || alfSliceParam.enabledFlag[2]);
    evc_bsw_write1(bs, alfChromaSignalled); //"alf_chroma_enable_flag"
#else
    const int alfChromaIdc = alfSliceParam.enabledFlag[1] * 2 + alfSliceParam.enabledFlag[2];
    evce_truncatedUnaryEqProb(bs, alfChromaIdc, 3);
#endif
#if M53608_ALF_5
    if (alfSliceParam.enabledFlag[0])
#endif
    {
#if M53608_ALF_3
        evc_bsw_write_ue(bs, alfSliceParam.numLumaFilters - 1);
#else
        evce_xWriteTruncBinCode(bs, alfSliceParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES);
#endif
#if M53608_ALF_10
        evc_bsw_write1(bs, alfSliceParam.lumaFilterType); //  "filter_type_flag"
#else
        evc_bsw_write1(bs, !alfSliceParam.lumaFilterType); //  "filter_type_flag"
#endif

        if (alfSliceParam.numLumaFilters > 1)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
#if M53608_ALF_3
                evc_bsw_write(bs, (u32)(alfSliceParam.filterCoeffDeltaIdx[i]), evc_tbl_log2[alfSliceParam.numLumaFilters - 1] + 1);
#else
                evce_xWriteTruncBinCode(bs, (u32)(alfSliceParam.filterCoeffDeltaIdx[i]), alfSliceParam.numLumaFilters);  //filter_coeff_delta[i]
#endif
            }
        }
#if !M53608_ALF_6 
        char codetab_pred[3] = { 1, 0, 2 };
#endif
        const int iNumFixedFilterPerClass = 16;
        {
#if M53608_ALF_6 
            evc_alfGolombEncode(bs, alfSliceParam.fixedFilterPattern, 0);
#else
            evc_alfGolombEncode(bs, codetab_pred[alfSliceParam.fixedFilterPattern], 0);
#endif

            if (alfSliceParam.fixedFilterPattern == 2)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
#if M53608_ALF_7
                    evc_bsw_write1(bs, alfSliceParam.fixedFilterUsageFlag[classIdx] ? 1 : 0); // "fixed_filter_flag"
#else
                    evc_bsw_write1(bs, alfSliceParam.fixedFilterIdx[classIdx] > 0 ? 1 : 0); // "fixed_filter_flag"
#endif
                }
            }
            if (alfSliceParam.fixedFilterPattern > 0)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
#if M53608_ALF_7
                    if (alfSliceParam.fixedFilterUsageFlag[classIdx] > 0)
#else
                    if (alfSliceParam.fixedFilterIdx[classIdx] > 0)
#endif
                    {
#if M53608_ALF_3
#if M53608_ALF_7
                        evc_bsw_write(bs, alfSliceParam.fixedFilterIdx[classIdx], evc_tbl_log2[iNumFixedFilterPerClass - 1] + 1);
#else
                        evc_bsw_write(bs, alfSliceParam.fixedFilterIdx[classIdx] - 1, evc_tbl_log2[iNumFixedFilterPerClass - 1] + 1);
#endif
#else
#if M53608_ALF_7
                        evce_xWriteTruncBinCode(bs, alfSliceParam.fixedFilterIdx[classIdx], iNumFixedFilterPerClass);
#else
                        evce_xWriteTruncBinCode(bs, alfSliceParam.fixedFilterIdx[classIdx] - 1, iNumFixedFilterPerClass);
#endif
#endif
                    }
                }
            }
        }

        evce_eco_alf_filter(bs, alfSliceParam, FALSE);
    }
#if M53608_ALF_14
    if (alfChromaSignalled)
#else
    if (alfChromaIdc)
#endif
    {
        {
            evce_eco_alf_filter(bs, alfSliceParam, TRUE);
        }
    }

    return EVC_OK;
}
#endif
int evce_eco_alf_sh_param(EVC_BSW * bs, EVC_SH * sh)
{
    evc_AlfSliceParam * alfSliceParam = &sh->alf_sh_param;

    evc_bsw_write1(bs, alfSliceParam->isCtbAlfOn);
    return EVC_OK;
}

#if GRAB_STAT
void ence_stat_cu(int x, int y, int cuw, int cuh, int cup, void *ctx, void *core, TREE_CONS tree_cons)
{
    EVCE_CTX *enc_ctx = (EVCE_CTX *)ctx;
    EVCE_CORE *enc_core = (EVCE_CORE *)core;
    EVCE_CU_DATA *cu_data = &enc_ctx->map_cu_data[enc_core->lcu_num];
    int scup = PEL2SCU(y) * enc_ctx->w_scu + PEL2SCU(x);

    int pred_mode = cu_data->pred_mode[cup];
    int mmvd_flag = 0;

    if (pred_mode > MODE_DIR && pred_mode < MODE_IBC)
    {
        pred_mode -= 2;
        mmvd_flag = 1;
    }

    if (evc_check_only_inter(tree_cons))
    {
        evc_assert(pred_mode == MODE_INTER);
    }
    if (evc_check_only_intra(tree_cons))
    {
        evc_assert( (pred_mode == MODE_IBC) || (pred_mode == MODE_INTRA) );
    }

    //cu_init(enc_ctx, enc_core, x, y, cup, cuw, cuh);
    evc_stat_write_cu_str(x, y, cuw, cuh, "PredMode", pred_mode);
    evc_stat_write_cu_str(x, y, cuw, cuh, "AffineFlag", cu_data->affine_flag[cup]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "MMVDFlag", mmvd_flag);
    evc_stat_write_cu_vec(x, y, cuw, cuh, "MV0", cu_data->mv[cup][0][0], cu_data->mv[cup][0][1]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "REF0", cu_data->refi[cup][0]);
    evc_stat_write_cu_vec(x, y, cuw, cuh, "MV1", cu_data->mv[cup][1][0], cu_data->mv[cup][1][1]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "REF1", cu_data->refi[cup][1]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "ats_intra_cu", cu_data->ats_intra_cu[cup]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "ats_inter_info", cu_data->ats_inter_info[cup]);
    if (evc_check_luma(tree_cons))
    {
    //evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma_enc", MCU_GET_CBFL(enc_ctx->map_scu[scup]));
    //evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma_nnz_sub", cu_data->nnz_sub[0][0][cup] > 0);
    evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma", cu_data->nnz[Y_C][cup] > 0);
    evc_stat_write_cu_str(x, y, cuw, cuh, "Tile_ID", enc_core->tile_num);
    evc_stat_write_cu_str(x, y, cuw, cuh, "Slice_IDX", enc_ctx->slice_num);
    }
}
#endif
