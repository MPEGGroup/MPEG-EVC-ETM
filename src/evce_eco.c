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
#include <limits.h>

#if ALF
#include <math.h>
#include "wrapper.h"
#include "enc_alf_wrapper.h"
#endif
#if GRAB_STAT
#include "evc_debug.h"
#endif
#pragma warning(disable:4018)

int evce_eco_nalu(EVC_BSW * bs, EVC_NALU nalu)
{
    evc_bsw_write(bs, nalu.nal_unit_size, 32);
    evc_bsw_write(bs, nalu.forbidden_zero_bit, 1);
    evc_bsw_write(bs, nalu.nal_unit_type_plus1, 6);
    evc_bsw_write(bs, nalu.nuh_temporal_id, 3);
    evc_bsw_write(bs, nalu.nuh_reserved_zero_5bits, 5);
    evc_bsw_write(bs, nalu.nuh_extension_flag, 1);

    return EVC_OK;
}

int evce_eco_rlp(EVC_BSW * bs, EVC_RPL * rpl)
{
    evc_bsw_write_ue(bs, (u32) rpl->ref_pic_num);
    if (rpl->ref_pic_num > 0)
    {
        evc_bsw_write_ue(bs, (u32)abs(rpl->ref_pics[0]));
        if (rpl->ref_pics[0] != 0)
        {
            evc_bsw_write1(bs, rpl->ref_pics[0] < 0);
        }
    }
    for (int i = 1; i < rpl->ref_pic_num; ++i)
    {
        int deltaRefPic = rpl->ref_pics[i] - rpl->ref_pics[i - 1];
        evc_bsw_write_ue(bs, (u32)abs(deltaRefPic));
        if (deltaRefPic != 0)
        {
            evc_bsw_write1(bs, deltaRefPic < 0);
        }
    }

    return EVC_OK;
}

int evce_eco_ref_pic_list_mod(EVC_BSW * bs)
{
    return EVC_OK;
}

int evce_eco_vui(EVC_BSW * bs)
{
    return EVC_OK;
}

#if CHROMA_QP_TABLE_SUPPORT_M50663
void derivedChromaQPMappingTables(EVC_SPS * sps)
{
    int MAX_QP = MAX_QP_TABLE_SIZE - 1;
    int qpInVal[MAX_QP_TABLE_SIZE_EXT] = { 0 };
    int qpOutVal[MAX_QP_TABLE_SIZE_EXT] = { 0 };

    int qpBdOffsetC = 6 * sps->bit_depth_chroma_minus8;
    int startQp = (sps->global_offset_flag == 1) ? 16 : -qpBdOffsetC;

    for (int i = 0; i < (sps->same_qp_table_for_chroma ? 1 : 2); i++)
    {
        qpInVal[0] = startQp + sps->delta_qp_in_val_minus1[i][0];
        qpOutVal[0] = startQp + sps->delta_qp_in_val_minus1[i][0] + sps->delta_qp_out_val[i][0];
        for (int j = 1; j <= sps->num_points_in_qp_table[i]; j++)
        {
            qpInVal[j] = qpInVal[j - 1] + sps->delta_qp_in_val_minus1[i][j] + 1;
            qpOutVal[j] = qpOutVal[j - 1] + (sps->delta_qp_in_val_minus1[i][j] + 1 + sps->delta_qp_out_val[i][j]);
        }

        for (int j = 0; j <= sps->num_points_in_qp_table[i]; j++)
        {
        assert(qpInVal[j]  >= -qpBdOffsetC && qpInVal[j]  < MAX_QP);// , "qpInVal out of range");
        assert(qpOutVal[j] >= -qpBdOffsetC && qpOutVal[j] < MAX_QP);// , "qpOutVal out of range");
        }

        p_evc_tbl_qp_chroma_dynamic[i][qpInVal[0]] = qpOutVal[0];
        for (int k = qpInVal[0] - 1; k >= -qpBdOffsetC; k--)
        {
            p_evc_tbl_qp_chroma_dynamic[i][k] = EVC_CLIP3(-qpBdOffsetC, MAX_QP, p_evc_tbl_qp_chroma_dynamic[i][k + 1] - 1);
        }
        for (int j = 0; j < sps->num_points_in_qp_table[i]; j++)
        {
            int sh = (sps->delta_qp_in_val_minus1[i][j + 1] + 1) >> 1;
            for (int k = qpInVal[j] + 1, m = 1; k <= qpInVal[j + 1]; k++, m++)
            {
                p_evc_tbl_qp_chroma_dynamic[i][k] = p_evc_tbl_qp_chroma_dynamic[i][qpInVal[j]]
                    + ((qpOutVal[j + 1] - qpOutVal[j]) * m + sh) / (sps->delta_qp_in_val_minus1[i][j + 1] + 1);
            }
        }
        for (int k = qpInVal[sps->num_points_in_qp_table[i]] + 1; k <= MAX_QP; k++)
        {
            p_evc_tbl_qp_chroma_dynamic[i][k] = EVC_CLIP3(-qpBdOffsetC, MAX_QP, p_evc_tbl_qp_chroma_dynamic[i][k - 1] + 1);
        }
    }
    if (sps->same_qp_table_for_chroma)
    {
        memcpy(&(p_evc_tbl_qp_chroma_dynamic[1][-qpBdOffsetC]), &(p_evc_tbl_qp_chroma_dynamic[0][-qpBdOffsetC]), MAX_QP_TABLE_SIZE_EXT * sizeof(int));
    }
}
#endif

int evce_eco_sps(EVC_BSW * bs, EVC_SPS * sps)
{
    evc_bsw_write_ue(bs, (u32)sps->sps_seq_parameter_set_id);
#if CHROMA_QP_TABLE_SUPPORT_M50663
    evc_bsw_write(bs, (u32)sps->profile_idc, 8);
    evc_bsw_write(bs, (u32)sps->level_idc, 8);
    evc_bsw_write(bs, (u32)sps->toolset_idc, 32);
#else
    evc_bsw_write(bs, (u32)sps->profile_idc, 7);
    evc_bsw_write(bs, (u32)sps->level_idc, 8);
#endif
    evc_bsw_write_ue(bs, (u32)sps->chroma_format_idc);
    evc_bsw_write_ue(bs, (u32)sps->pic_width_in_luma_samples);
    evc_bsw_write_ue(bs, (u32)sps->pic_height_in_luma_samples);
    evc_bsw_write_ue(bs, (u32)sps->bit_depth_luma_minus8);
    evc_bsw_write_ue(bs, (u32)sps->bit_depth_chroma_minus8);
    evc_bsw_write1(bs, (u32)sps->sps_btt_flag);
    if (sps->sps_btt_flag)
    {
        evc_bsw_write_ue(bs, (u32)sps->log2_ctu_size_minus2);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_ctu_max_11_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_11_min_11_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_11_max_12_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_min_11_min_12_cb_size_minus1);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_12_max_14_cb_size_minus1);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_min_12_min_14_cb_size_minus1);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_11_max_tt_cb_size_minus1);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_min_11_min_tt_cb_size_minus2);
    }
    evc_bsw_write1(bs, (u32)sps->sps_suco_flag);
    if (sps->sps_suco_flag)
    {
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_ctu_size_max_suco_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_suco_min_suco_cb_size);
    }
#if M50632_IMPROVEMENT_SPS
    evc_bsw_write1(bs, sps->tool_amis);
    if(sps->tool_amis)
    {
#if ADMVP
        evc_bsw_write1(bs, sps->tool_admvp);
#endif
#if AFFINE
        evc_bsw_write1(bs, sps->tool_affine);
#endif
        evc_bsw_write1(bs, sps->tool_amvr);
#if DMVR
        evc_bsw_write1(bs, sps->tool_dmvr);
#endif
        evc_bsw_write1(bs, sps->tool_mmvd);
    }

    evc_bsw_write1(bs, sps->tool_eipd);
    if(sps->tool_eipd)
    {
#if IBC
        evc_bsw_write1(bs, sps->ibc_flag);
        if (sps->ibc_flag)
           evc_bsw_write_ue(bs, (u32)(sps->ibc_log_max_size - 2));
#endif
    }

    evc_bsw_write1(bs, sps->tool_cm_init);
    if(sps->tool_cm_init)
    {
#if ADCC
        evc_bsw_write1(bs, sps->tool_adcc);
#endif
    }

    evc_bsw_write1(bs, sps->tool_iqt);
    if(sps->tool_iqt)
    {
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
    evc_bsw_write1(bs, sps->tool_ats);
#endif
    }

#if ALF
    evc_bsw_write1(bs, sps->tool_alf);
#endif
#if HTDF
    evc_bsw_write1(bs, sps->tool_htdf);
#endif
#else

    evc_bsw_write1(bs, sps->tool_amvr);
    evc_bsw_write1(bs, sps->tool_mmvd);
#if AFFINE
    evc_bsw_write1(bs, sps->tool_affine);
#endif
#if DMVR
    evc_bsw_write1(bs, sps->tool_dmvr);
#endif
#if ALF
    evc_bsw_write1(bs, sps->tool_alf);
#endif
#if ADMVP
    evc_bsw_write1(bs, sps->tool_admvp);
#endif
    evc_bsw_write1(bs, sps->tool_eipd);
    evc_bsw_write1(bs, sps->tool_amis);
    evc_bsw_write1(bs, sps->tool_iqt);
#if HTDF
    evc_bsw_write1(bs, sps->tool_htdf);
#endif
#if ADCC
    evc_bsw_write1(bs, sps->tool_adcc);
#endif
    evc_bsw_write1(bs, sps->tool_cm_init);
#if IBC
    evc_bsw_write1(bs, sps->ibc_flag);
    if (sps->ibc_flag)
        evc_bsw_write_ue(bs, (u32)(sps->ibc_log_max_size - 2));
#endif
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
    evc_bsw_write1(bs, sps->tool_ats);
#endif
#endif
    evc_bsw_write1(bs, sps->tool_rpl);
    evc_bsw_write1(bs, sps->tool_pocs);
#if DQP
    evc_bsw_write1(bs, sps->dquant_flag);
#endif
    if (sps->tool_pocs)
    {
        evc_bsw_write_ue(bs, (u32)sps->log2_max_pic_order_cnt_lsb_minus4);
    }
    if (!sps->tool_rpl || !sps->tool_pocs)
    {
        evc_bsw_write_ue(bs, (u32)sps->log2_sub_gop_length);
        if (sps->log2_sub_gop_length == 0)
        {
            evc_bsw_write_ue(bs, sps->log2_ref_pic_gap_length);
        }
    }
    evc_bsw_write_ue(bs, (u32)sps->max_dec_pic_buffering_minus1);
    if (!sps->tool_rpl)
    {
        evc_bsw_write_ue(bs, (u32)sps->max_num_ref_pics);
    }
    else
    {
        evc_bsw_write1(bs, sps->long_term_ref_pics_flag);
        evc_bsw_write1(bs, sps->rpl_candidates_present_flag);

        if (sps->rpl_candidates_present_flag)
        {
            evc_bsw_write1(bs, sps->rpl1_same_as_rpl0_flag);

            evc_bsw_write_ue(bs, (u32)sps->rpls_l0_num);
            for (int i = 0; i < sps->rpls_l0_num; ++i)
                evce_eco_rlp(bs, &sps->rpls_l0[i]);

            if (!sps->rpl1_same_as_rpl0_flag)
            {
                evc_bsw_write_ue(bs, (u32)sps->rpls_l1_num);
                for (int i = 0; i < sps->rpls_l1_num; ++i)
                    evce_eco_rlp(bs, &sps->rpls_l1[i]);
            }
        }
    }

    evc_bsw_write1(bs, sps->picture_cropping_flag);
    if (sps->picture_cropping_flag)
    {
        evc_bsw_write_ue(bs, (u32)sps->picture_crop_left_offset);
        evc_bsw_write_ue(bs, (u32)sps->picture_crop_right_offset);
        evc_bsw_write_ue(bs, (u32)sps->picture_crop_top_offset);
        evc_bsw_write_ue(bs, (u32)sps->picture_crop_bottom_offset);
    }
#if CHROMA_QP_TABLE_SUPPORT_M50663
    int qpBdOffsetC = 6 * sps->bit_depth_chroma_minus8;
    int inArray[] = { 22,37,40,45 };
    int outArray[] = { 22,36,38,42 };
    sps->chroma_qp_table_present_flag = 0;
    evc_bsw_write1(bs, sps->chroma_qp_table_present_flag);
    if (sps->chroma_qp_table_present_flag)
    {
        sps->same_qp_table_for_chroma = 1;
        sps->global_offset_flag = (inArray[0] > 15 && inArray[0] > 15) ? 1 : 0;
        int startQp = (sps->global_offset_flag == 1) ? 16 : -qpBdOffsetC;
        for (int i = 0; i < (sps->same_qp_table_for_chroma ? 1 : 2); i++) {
            sps->num_points_in_qp_table[i] = 3;
            sps->delta_qp_in_val_minus1[i][0] = inArray[0]- startQp;
            sps->delta_qp_out_val[i][0] = outArray[0] - startQp - sps->delta_qp_in_val_minus1[i][0];

            for (int k = 1; k <= sps->num_points_in_qp_table[i]; k++)
            {
                sps->delta_qp_in_val_minus1[i][k] = (inArray[k] - inArray[k - 1]) - 1;
                sps->delta_qp_out_val[i][k] = (outArray[k] - outArray[k - 1]) - (sps->delta_qp_in_val_minus1[i][k] + 1);
            }
        }

        derivedChromaQPMappingTables(sps);

        evc_bsw_write1(bs, sps->same_qp_table_for_chroma);
        evc_bsw_write1(bs, sps->global_offset_flag);
        for (int i = 0; i < (sps->same_qp_table_for_chroma ? 1 : 2); i++) {
            evc_bsw_write_ue(bs, (u32)sps->num_points_in_qp_table[i]);
            for (int j = 0; j <= sps->num_points_in_qp_table[i]; j++) {
                evc_bsw_write(bs, sps->delta_qp_in_val_minus1[i][j], 6);
                evc_bsw_write_se(bs, (u32)sps->delta_qp_out_val[i][j]);
            }
        }
    }
#endif
    evc_bsw_write1(bs, sps->vui_parameters_present_flag);
    if (sps->vui_parameters_present_flag)
        evce_eco_vui(bs); //To be implemented

    while(!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, 0);
    }

    return EVC_OK;
}

int evce_eco_pps(EVC_BSW * bs, EVC_SPS * sps, EVC_PPS * pps)
{
    evc_bsw_write_ue(bs, (u32)pps->pps_pic_parameter_set_id);
    evc_bsw_write_ue(bs, (u32)pps->pps_seq_parameter_set_id);
    evc_bsw_write_ue(bs, (u32)pps->num_ref_idx_default_active_minus1[0]);
    evc_bsw_write_ue(bs, (u32)pps->num_ref_idx_default_active_minus1[1]);

    if (sps->tool_rpl)
    {
        if (sps->long_term_ref_pics_flag)
        {
            evc_bsw_write_ue(bs, (u32)pps->additional_lt_poc_lsb_len);
        }

        if (sps->rpl_candidates_present_flag)
        {
            evc_bsw_write1(bs, pps->rpl1_idx_present_flag);
        }
    }
    
    evc_bsw_write1(bs, pps->single_tile_in_pic_flag);
    if (!pps->single_tile_in_pic_flag)
    {
        evc_bsw_write_ue(bs, (u32)pps->num_tile_columns_minus1);
        evc_bsw_write_ue(bs, (u32)pps->num_tile_rows_minus1);
        evc_bsw_write1(bs, pps->uniform_tile_spacing_flag);
        if (!pps->uniform_tile_spacing_flag)
        {
            for (int i = 0; i < pps->num_tile_columns_minus1; ++i)
            {
                evc_bsw_write_ue(bs, (u32)pps->tile_column_width_minus1[i]);
            }
            for (int i = 0; i < pps->num_tile_rows_minus1; ++i)
            {
                evc_bsw_write_ue(bs, (u32)pps->tile_row_height_minus1[i]);
            }
        }
        evc_bsw_write1(bs, pps->loop_filter_across_tiles_enabled_flag);
        evc_bsw_write_ue(bs, (u32)pps->tile_offset_lens_minus1);
    }

    evc_bsw_write_ue(bs, (u32)pps->tile_id_len_minus1);
    evc_bsw_write1(bs, pps->explicit_tile_id_flag);
    if (pps->explicit_tile_id_flag)
    {
        for (int i = 0; i <= pps->num_tile_rows_minus1; ++i)
        {
            for (int j = 0; j <= pps->num_tile_columns_minus1; ++j)
            {
                evc_bsw_write(bs, (u32)pps->tile_id_val[i][j], pps->tile_id_len_minus1 + 1);
            }
        }
    }

    evc_bsw_write1(bs, pps->arbitrary_slice_present_flag);
    evc_bsw_write1(bs, pps->constrained_intra_pred_flag); 

#if DQP
    evc_bsw_write1(bs, pps->cu_qp_delta_enabled_flag != 0 ? 1 : 0);
    if (pps->cu_qp_delta_enabled_flag)
    {
        evc_bsw_write_ue(bs, pps->cu_qp_delta_area - 6);
    }
#endif

    while (!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, 0);
    }

    return EVC_OK;
}

#if ALF
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
#if EVC_TILE_SUPPORT
    int NumTilesInSlice = (pps->num_tile_columns_minus1 + 1) * (pps->num_tile_rows_minus1 + 1);
#else
    int NumTilesInSlice = 0; //TBD according to the spec
#endif

    evc_bsw_write1(bs, sh->temporal_mvp_asigned_flag);
    if (sh->temporal_mvp_asigned_flag)
    {
        evc_bsw_write1(bs, sh->collocated_from_list_idx);
        evc_bsw_write1(bs, sh->collocated_from_ref_idx);
        evc_bsw_write1(bs, sh->collocated_mvp_source_list_idx);
    }

    evc_bsw_write_ue(bs, sh->slice_pic_parameter_set_id);
    evc_bsw_write1(bs, sh->single_tile_in_slice_flag);
    evc_bsw_write(bs, sh->first_tile_id, pps->tile_id_len_minus1 + 1);

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
            for (int i = 0; i < NumTilesInSlice - 1; ++i)
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
#if M50632_IMPROVEMENT_MMVD
    else if (sps->tool_mmvd && (sh->slice_type == SLICE_P))
    {
        evc_bsw_write1(bs, sh->mmvd_group_enable_flag);
    }
#endif
#if ALF
    if (sps->tool_alf)
    {
        evc_bsw_write1(bs, sh->alf_on);
        if (sh->alf_on)
        {
#if M50662_LUMA_CHROMA_SEPARATE_APS
            evc_bsw_write(bs, sh->aps_id_y, APS_MAX_NUM_IN_BITS);
            evc_bsw_write(bs, sh->aps_id_ch, APS_MAX_NUM_IN_BITS);
#else
            evc_bsw_write(bs, sh->aps_signaled, APS_MAX_NUM_IN_BITS); //encode tile group aps id
#endif
            evce_eco_alf_sh_param(bs, sh); // signaling ALF map
        }
    }
#endif

    if (nut != EVC_IDR_NUT)
    {
        if (sps->tool_pocs)
        {
            evc_bsw_write(bs, sh->poc_lsb, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
        }
        if (sps->tool_rpl)
        {
            //L0 candidates signaling
            if (sps->rpl_candidates_present_flag)
            {
                evc_bsw_write1(bs, sh->ref_pic_list_sps_flag[0]);
            }
            if (sh->ref_pic_list_sps_flag[0])
            {
                if (sps->rpls_l0_num)
                {
                    evc_bsw_write_ue(bs, sh->rpl_l0_idx);
                }
            }
            else
            {
                evce_eco_rlp(bs, &sh->rpl_l0);
            }

            //L1 candidates signaling
            if (sps->rpl_candidates_present_flag)
            {
                evc_bsw_write1(bs, sh->ref_pic_list_sps_flag[1]);
            }
            if (sh->ref_pic_list_sps_flag[1])
            {
                if (sps->rpls_l1_num)
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
            evc_bsw_write_ue(bs, (u32)(sh->rpl_l0).ref_pic_active_num - 1);
            if (sh->slice_type == SLICE_B)
            {
                evc_bsw_write_ue(bs, (u32)(sh->rpl_l1).ref_pic_active_num - 1);
            }
        }
    }

    evc_bsw_write1(bs, sh->deblocking_filter_on);
    evc_bsw_write_se(bs, sh->sh_deblock_alpha_offset);
    evc_bsw_write_se(bs, sh->sh_deblock_beta_offset);
    evc_bsw_write(bs, sh->qp, 6);
    evc_bsw_write_se(bs, (int)sh->qp - (int)sh->qp_u);
    evc_bsw_write_se(bs, (int)sh->qp - (int)sh->qp_v);

    if (!sh->single_tile_in_slice_flag)
    {
        for (int i = 0; i < NumTilesInSlice - 1; ++i)
        {
            evc_bsw_write(bs, sh->entry_point_offset_minus1[i], pps->tile_offset_lens_minus1 + 1);
        }
    }

    /* byte align */
    while(!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, 0);
    }

    return EVC_OK;
}

int evce_eco_udata(EVCE_CTX * ctx, EVC_BSW * bs)
{
    int ret, i;

    /* should be aligned before adding user data */
    evc_assert_rv(EVC_BSW_IS_BYTE_ALIGN(bs), EVC_ERR_UNKNOWN);

    /* picture signature */
    if (ctx->param.use_pic_sign)
    {
        u8 pic_sign[16];

        /* should be aligned before adding user data */
        evc_assert_rv(EVC_BSW_IS_BYTE_ALIGN(bs), EVC_ERR_UNKNOWN);

        /* get picture signature */
        ret = evc_picbuf_signature(PIC_CURR(ctx), pic_sign);
        evc_assert_rv(ret == EVC_OK, ret);

        /* write user data type */
        evc_bsw_write(bs, EVC_UD_PIC_SIGNATURE, 8);

        /* embed signature (HASH) to bitstream */
        for (i = 0; i < 16; i++)
        {
            evc_bsw_write(bs, pic_sign[i], 8);
        }
    }
    /* write end of user data syntax */
    evc_bsw_write(bs, EVC_UD_END, 8);

    return EVC_OK;
}


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
                    evc_bsw_write(bs, 0x00, 8);
                sbac->stacked_zero--;
            }
            if(sbac->is_bitcount)
                evc_bsw_write_est(sbac, sbac->pending_byte, 8);
            else
                evc_bsw_write(bs, sbac->pending_byte, 8);
        }
    }
    sbac->pending_byte = writing_byte;
    sbac->is_pending_byte = 1;
}

static void sbac_carry_propagate(EVCE_SBAC *sbac, EVC_BSW *bs)
{
#if VARIABLE_RANGE
    u32 out_bits = (sbac->code) >> (RANGE_BITS + 3);
    (sbac->code) &= (1 << (RANGE_BITS + 3)) - 1;
#else
    u32 out_bits = (sbac->code) >> 19;
    (sbac->code) &= 0x7FFFF;
#endif

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

static void sbac_encode_bins_ep_msb(u32 value, int num_bin, EVCE_SBAC *sbac, EVC_BSW *bs)
{
    int bin = 0;
    for(bin = num_bin - 1; bin >= 0; bin--)
    {
        sbac_encode_bin_ep(value & (1 << (u32)bin), sbac, bs);
    }
}

void evce_sbac_encode_bin(u32 bin, EVCE_SBAC *sbac, SBAC_CTX_MODEL *model, EVC_BSW *bs)
{
    u32  lps, range, code, code_bits;
    u16  cmps, p0, p0_lps, p0_mps;

    p0 = ((*model) >> 1) & PROB_MASK;
    lps = (p0*(sbac->range)) >> MCABAC_PROB_BITS;

    lps = lps < 437 ? 437 : lps;
    cmps = (*model) & 1;

    range = sbac->range;
    code = sbac->code;
    code_bits = sbac->code_bits;

    range -= lps;

#if TRACE_BIN
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("model ");
    EVC_TRACE_INT(*model);
    EVC_TRACE_STR("range ");
    EVC_TRACE_INT(range);
    EVC_TRACE_STR("lps ");
    EVC_TRACE_INT(lps);
    EVC_TRACE_STR("\n");
#endif

    if(bin != (cmps))
    {
        if((range) >= lps)
        {
            (code) += (range);
            (range) = lps;
        }
        p0_lps = p0 + ((MAX_PROB - p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0);

        //to do
        if(p0_lps > (MAX_PROB>>1))
        {
            cmps = 1 - cmps;
            p0_lps = MAX_PROB - p0_lps;
        }
        *model = (p0_lps << 1) + cmps;
    }
    else
    {
        p0_mps = p0 - ((p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0);
        *model = (p0_mps << 1) + cmps;
#if VARIABLE_RANGE
        if((range) >= HALF_RANGE)
#else
        if((range) >= 0x8000)
#endif
        {
            goto END;
        }
        if((range) < lps)
        {
            (code) += (range);
            (range) = lps;
        }
    }

    do
    {
        range <<= 1;
        code <<= 1;
        code_bits--;

        if(code_bits == 0)
        {
            sbac->code = code;

            sbac_carry_propagate(sbac, bs);

            code = sbac->code;
            code_bits = 8;
        }
#if VARIABLE_RANGE
    } while(range < HALF_RANGE);
#else
    } while(range < 0x8000);
#endif

END:
    sbac->range = range;
    sbac->code = code;
    sbac->code_bits = code_bits;
}

void evce_sbac_encode_bin_trm(u32 bin, EVCE_SBAC *sbac, EVC_BSW *bs)
{
    (sbac->range)--;

    if(bin)
    {
        (sbac->code) += (sbac->range);
        (sbac->range) = 1;
    }
    else
    {
#if VARIABLE_RANGE
        if((sbac->range) >= HALF_RANGE)
#else
        if((sbac->range) >= 0x8000)
#endif
        {
            return;
        }
    }
    do
    {
        (sbac->range) <<= 1;
        (sbac->code) <<= 1;
        if(--(sbac->code_bits) == 0)
        {
            sbac_carry_propagate(sbac, bs);
            sbac->code_bits = 8;
        }
#if VARIABLE_RANGE
    } while((sbac->range) < HALF_RANGE);
#else
    } while((sbac->range) < 0x8000);
#endif
}

void evce_sbac_reset(EVCE_SBAC *sbac, u8 slice_type, u8 slice_qp, int sps_cm_init_flag)
{
    EVC_SBAC_CTX *sbac_ctx;
    sbac_ctx = &sbac->ctx;

    /* Initialization of the internal variables */
#if VARIABLE_RANGE
    sbac->range = MAX_RANGE;
#else
    sbac->range = 0x10000;
#endif
    sbac->code = 0;
    sbac->code_bits = 11;
    sbac->pending_byte = 0;
    sbac->is_pending_byte = 0;
    sbac->stacked_ff = 0;
    sbac->stacked_zero = 0;

    evc_mset(sbac_ctx, 0x00, sizeof(*sbac_ctx));

    sbac_ctx->sps_cm_init_flag = sps_cm_init_flag;

    /* Initialization of the context models */
    if(sps_cm_init_flag == 1)
    {
#if !CTX_REPRESENTATION_IMPROVEMENT
        int i; 
#endif

        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf, (s16*)init_cbf, NUM_QT_CBF_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->all_cbf, (s16*)init_all_cbf, NUM_QT_ROOT_CBF_CTX, slice_type, slice_qp);
#if DQP
        evc_eco_sbac_ctx_initialize(sbac_ctx->delta_qp, (s16*)init_dqp, NUM_DELTA_QP_CTX, slice_type, slice_qp);
#endif
#if ADCC 
#if COEFF_CODE_ADCC2
#if M50631_IMPROVEMENT_ADCC_CTXINIT
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_gt0, (s16*)init_cc_gt0_4, NUM_CTX_GT0, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_gtA, (s16*)init_cc_gtA_4, NUM_CTX_GTA, slice_type, slice_qp);
#else
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_gt0, (s16*)init_cc_gt0_3, NUM_CTX_GT0, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_gtA, (s16*)init_cc_gtA_3, NUM_CTX_GTA, slice_type, slice_qp);
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_scanr_x, (s16*)init_cc_scanr_x_3, NUM_CTX_SCANR, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_scanr_y, (s16*)init_cc_scanr_y_3, NUM_CTX_SCANR, slice_type, slice_qp);
#else
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_gt0, (s16*)init_cc_gt0, NUM_CTX_GT0, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_gtA, (s16*)init_cc_gtA, NUM_CTX_GTA, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_scanr_x, (s16*)init_cc_scanr_x, NUM_CTX_SCANR, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->cc_scanr_y, (s16*)init_cc_scanr_y, NUM_CTX_SCANR, slice_type, slice_qp);
#endif
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->pred_mode, (s16*)init_pred_mode, NUM_PRED_MODE_CTX, slice_type, slice_qp);
#if M50761_CHROMA_NOT_SPLIT
        evc_eco_sbac_ctx_initialize(sbac_ctx->mode_cons, (s16*)init_mode_cons, NUM_MODE_CONS_CTX, slice_type, slice_qp);
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->inter_dir, (s16*)init_inter_dir, NUM_INTER_DIR_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->intra_dir, (s16*)init_intra_dir, NUM_INTRA_DIR_CTX, slice_type, slice_qp);
#if CTX_REPRESENTATION_IMPROVEMENT
        evc_eco_sbac_ctx_initialize(sbac_ctx->run, (s16*)init_run, NUM_SBAC_CTX_RUN, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->last, (s16*)init_last, NUM_SBAC_CTX_LAST, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->level, (s16*)init_level, NUM_SBAC_CTX_LEVEL, slice_type, slice_qp);
#else
        for(i = 0; i < NUM_SBAC_CTX_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_flag, (s16*)init_mmvd_flag, NUM_SBAC_CTX_MMVD_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_merge_idx, (s16*)init_mmvd_merge_idx, NUM_SBAC_CTX_MMVD_MERGE_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_distance_idx, (s16*)init_mmvd_distance_idx, NUM_SBAC_CTX_MMVD_DIST_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_direction_idx, (s16*)init_mmvd_direction_idx, NUM_SBAC_CTX_DIRECTION_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_group_idx, (s16*)init_mmvd_group_idx, NUM_SBAC_CTX_MMVD_GRP_IDX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvp_idx, (s16*)init_mvp_idx, NUM_MVP_IDX_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvp_idx, (s16*)init_affine_mvp_idx, NUM_AFFINE_MVP_IDX_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvr_idx, (s16*)init_mvr_idx, NUM_MVR_IDX_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->bi_idx, (s16*)init_bi_idx, NUM_BI_IDX_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvd, (s16*)init_mvd, NUM_MV_RES_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->refi, (s16*)init_refi, NUM_REFI_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_flag, (s16*)init_btt_split_flag, NUM_SBAC_CTX_BTT_SPLIT_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_dir, (s16*)init_btt_split_dir, NUM_SBAC_CTX_BTT_SPLIT_DIR, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_type, (s16*)init_btt_split_type, NUM_SBAC_CTX_BTT_SPLIT_TYPE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->suco_flag, (s16*)init_suco_flag, NUM_SBAC_CTX_SUCO_FLAG, slice_type, slice_qp);
#if ALF
        evc_eco_sbac_ctx_initialize(sbac_ctx->ctb_alf_flag, (s16*)init_ctb_alf_flag, NUM_SBAC_CTX_ALF_FLAG, slice_type, slice_qp);
#endif
#if AFFINE
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_flag, (s16*)init_affine_flag, NUM_SBAC_CTX_AFFINE_FLAG, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mode, (s16*)init_affine_mode, NUM_SBAC_CTX_AFFINE_MODE, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mrg, (s16*)init_affine_mrg, AFF_MAX_CAND, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvd_flag, (s16*)init_affine_mvd_flag, NUM_SBAC_CTX_AFFINE_MVD_FLAG, slice_type, slice_qp);
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->skip_flag, (s16*)init_skip_flag, NUM_SBAC_CTX_SKIP_FLAG, slice_type, slice_qp);
#if IBC
        evc_eco_sbac_ctx_initialize(sbac_ctx->ibc_flag, (s16*)init_ibc_flag, NUM_SBAC_CTX_IBC_FLAG, slice_type, slice_qp);
#endif
#if ATS_INTRA_PROCESS
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_intra_cu, (s16*)init_ats_intra_cu, NUM_ATS_INTRA_CU_FLAG_CTX, slice_type, slice_qp);
#if M50632_SIMPLIFICATION_ATS
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_tu, (s16*)init_ats_tu, NUM_ATS_INTRA_TU_FLAG_CTX, slice_type, slice_qp);
#else
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_tu_h, (s16*)init_ats_tu_h, NUM_ATS_INTRA_TU_FLAG_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_tu_v, (s16*)init_ats_tu_v, NUM_ATS_INTRA_TU_FLAG_CTX, slice_type, slice_qp);
#endif
#endif
#if ATS_INTER_PROCESS
        evc_eco_sbac_ctx_initialize(sbac_ctx->ats_inter_info, (s16*)init_ats_inter_info, NUM_SBAC_CTX_ATS_INTER_INFO, slice_type, slice_qp);
#endif
    }
    else // (sps_cm_init_flag == 0)
    {
        int i;

        for(i = 0; i < NUM_SBAC_CTX_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;
        for(i = 0; i < NUM_QT_CBF_CTX; i++) sbac_ctx->cbf[i] = PROB_INIT;
        sbac_ctx->all_cbf[0] = PROB_INIT;
#if ADCC 
        for (i = 0; i < NUM_CTX_GT0; i++) sbac_ctx->cc_gt0[i] = PROB_INIT;
        for (i = 0; i < NUM_CTX_GTA; i++) sbac_ctx->cc_gtA[i] = PROB_INIT;
        for (i = 0; i < NUM_CTX_SCANR; i++) sbac_ctx->cc_scanr_x[i] = PROB_INIT;
        for (i = 0; i < NUM_CTX_SCANR; i++) sbac_ctx->cc_scanr_y[i] = PROB_INIT;
#endif
        for(i = 0; i < NUM_PRED_MODE_CTX; i++) sbac_ctx->pred_mode[i] = PROB_INIT;
#if M50761_CHROMA_NOT_SPLIT
        for (i = 0; i < NUM_MODE_CONS_CTX; i++) sbac_ctx->mode_cons[i] = PROB_INIT;
#endif
        for(i = 0; i < NUM_INTER_DIR_CTX; i++) sbac_ctx->inter_dir[i] = PROB_INIT;
        for(i = 0; i < NUM_INTRA_DIR_CTX; i++) sbac_ctx->intra_dir[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_MMVD_FLAG; i++) sbac_ctx->mmvd_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_MMVD_MERGE_IDX; i++) sbac_ctx->mmvd_merge_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_MMVD_DIST_IDX; i++) sbac_ctx->mmvd_distance_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_DIRECTION_IDX; i++) sbac_ctx->mmvd_direction_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_MMVD_GRP_IDX; i++) sbac_ctx->mmvd_group_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_MVP_IDX_CTX; i++) sbac_ctx->mvp_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_MVR_IDX_CTX; i++) sbac_ctx->mvr_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_BI_IDX_CTX; i++) sbac_ctx->bi_idx[i] = PROB_INIT;
        for(i = 0; i < NUM_MV_RES_CTX; i++)  sbac_ctx->mvd[i] = PROB_INIT;
        for(i = 0; i < NUM_REFI_CTX; i++)   sbac_ctx->refi[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_BTT_SPLIT_FLAG; i++) sbac_ctx->btt_split_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_BTT_SPLIT_DIR; i++) sbac_ctx->btt_split_dir[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_BTT_SPLIT_TYPE; i++) sbac_ctx->btt_split_type[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_SUCO_FLAG; i++) sbac_ctx->suco_flag[i] = PROB_INIT;
#if DQP
        for (i = 0; i < NUM_DELTA_QP_CTX; i++) sbac_ctx->delta_qp[i] = PROB_INIT;
#endif
#if AFFINE
        for (i = 0; i < NUM_SBAC_CTX_AFFINE_FLAG; i++)
        {
            sbac_ctx->affine_flag[i] = PROB_INIT;
        }

        sbac_ctx->affine_mode[0] = PROB_INIT;

        for (i = 0; i < AFF_MAX_CAND; i++) 
        {
            sbac_ctx->affine_mrg[i] = PROB_INIT;
        }

        sbac_ctx->affine_mvd_flag[0] = PROB_INIT;
        sbac_ctx->affine_mvd_flag[1] = PROB_INIT;
#endif
        for (i = 0; i < NUM_SBAC_CTX_SKIP_FLAG; i++) sbac_ctx->skip_flag[i] = PROB_INIT;
#if IBC
        for (i = 0; i < NUM_SBAC_CTX_IBC_FLAG; i++) sbac_ctx->ibc_flag[i] = PROB_INIT;
#endif
#if ATS_INTRA_PROCESS
        for (i = 0; i < NUM_ATS_INTRA_CU_FLAG_CTX; i++) sbac_ctx->ats_intra_cu[i] = PROB_INIT;
#if M50632_SIMPLIFICATION_ATS
        for (i = 0; i < NUM_ATS_INTRA_TU_FLAG_CTX; i++) sbac_ctx->ats_tu[i] = PROB_INIT;
#else
        for (i = 0; i < NUM_ATS_INTRA_TU_FLAG_CTX; i++) sbac_ctx->ats_tu_h[i] = PROB_INIT;
        for (i = 0; i < NUM_ATS_INTRA_TU_FLAG_CTX; i++) sbac_ctx->ats_tu_v[i] = PROB_INIT;
#endif
#endif
#if ATS_INTER_PROCESS
        for (i = 0; i < NUM_SBAC_CTX_ATS_INTER_INFO; i++) sbac_ctx->ats_inter_info[i] = PROB_INIT;
#endif

    }
}

void evce_sbac_finish(EVC_BSW *bs)
{
    EVCE_SBAC *sbac;
    u32 tmp;

    sbac = GET_SBAC_ENC(bs);

#if VARIABLE_RANGE
    tmp = (sbac->code + sbac->range - 1) & (0xFFFFFFFF << RANGE_BITS);
#else
    tmp = (sbac->code + sbac->range - 1) & 0xFFFF0000;
#endif
    if(tmp < sbac->code)
    {
#if VARIABLE_RANGE
        tmp += HALF_RANGE;
#else
        tmp += 0x8000;
#endif
    }

    sbac->code = tmp << sbac->code_bits;
    sbac_carry_propagate(sbac, bs);

    sbac->code <<= 8;
    sbac_carry_propagate(sbac, bs);

    while(sbac->stacked_zero > 0)
    {
        evc_bsw_write(bs, 0x00, 8);
        sbac->stacked_zero--;
    }
    if(sbac->pending_byte != 0)
    {
        evc_bsw_write(bs, sbac->pending_byte, 8);
    }
    else
    {
        if(sbac->code_bits < 4)
        {
            evc_bsw_write(bs, 0, 4 - sbac->code_bits);

            while(!EVC_BSW_IS_BYTE_ALIGN(bs))
            {
                evc_bsw_write1(bs, 0);
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

#if AFFINE
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
#endif

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
#if IBC
#if !M50761_BUGFIX_ENCSIDE_IBC
static 
#endif
void evce_eco_ibc_flag(EVC_BSW * bs, int flag, int ctx)
{
  EVCE_SBAC *sbac;
  sbac = GET_SBAC_ENC(bs);
  evce_sbac_encode_bin(flag, sbac, sbac->ctx.ibc_flag + ctx, bs);
#if TRACE_ADDITIONAL_FLAGS
  EVC_TRACE_COUNTER;
  EVC_TRACE_STR("IBC pred mode ");
  EVC_TRACE_INT(!!flag);
  EVC_TRACE_STR("ctx ");
  EVC_TRACE_INT(ctx);
  EVC_TRACE_STR("\n");
#endif
}
#endif
void evce_eco_inter_t_direct(EVC_BSW *bs, int t_direct_flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(t_direct_flag, sbac, sbac->ctx.inter_dir, bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("direct_merge ");
    EVC_TRACE_INT(t_direct_flag ? PRED_DIR : 0);
    EVC_TRACE_STR("\n");
}

void evce_eco_slice_end_flag(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin_trm(flag, sbac, bs);
}
#if EVC_TILE_SUPPORT
void evce_eco_tile_end_flag(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin_trm(flag, sbac, bs);
}
#endif
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
#if CTX_MODEL_FOR_RESIDUAL_IN_BASE
            t0 = ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);
#else
            t0 = sbac->ctx.sps_cm_init_flag == 1 ? ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12) : (ch_type == Y_C ? 0 : 2);
#endif

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

#if ADCC
static void code_positionLastXY(EVC_BSW *bs, int sr_x, int sr_y, int width, int height, int ch_type)
{
    EVCE_SBAC *sbac = GET_SBAC_ENC(bs);
    SBAC_CTX_MODEL* cm_x = sbac->ctx.cc_scanr_x + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_SCANR_LUMA : 11));
    SBAC_CTX_MODEL* cm_y = sbac->ctx.cc_scanr_y + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_SCANR_LUMA : 11));

    int bin;
    int group_idx_x;
    int group_idx_y;
    int blk_offset_x, blk_offset_y, shift_x, shift_y;
    int i, cnt;

    group_idx_x = g_group_idx[sr_x];
    group_idx_y = g_group_idx[sr_y];
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

    // posX

    for (bin = 0; bin < group_idx_x; bin++)
    {
        evce_sbac_encode_bin(1, sbac, &cm_x[blk_offset_x + (bin >> shift_x)], bs);
    }
    if (group_idx_x < g_group_idx[width - 1])
    {
        evce_sbac_encode_bin(0, sbac, &cm_x[blk_offset_x + (bin >> shift_x)], bs);
    }

    // posY

    for (bin = 0; bin < group_idx_y; bin++)
    {
        evce_sbac_encode_bin(1, sbac, &cm_y[blk_offset_y + (bin >> shift_y)], bs);
    }
    if (group_idx_y < g_group_idx[height - 1])
    {
        evce_sbac_encode_bin(0, sbac, &cm_y[blk_offset_y + (bin >> shift_y)], bs);
    }

    // EP-coded part

    if (group_idx_x > 3)
    {
        cnt = (group_idx_x - 2) >> 1;
        sr_x = sr_x - g_min_in_group[group_idx_x];
        for (i = cnt - 1; i >= 0; i--)
        {
            sbac_encode_bin_ep((sr_x >> i) & 1, sbac, bs);
        }
    }
    if (group_idx_y > 3)
    {
        cnt = (group_idx_y - 2) >> 1;
        sr_y = sr_y - g_min_in_group[group_idx_y];
        for (i = cnt - 1; i >= 0; i--)
        {
            sbac_encode_bin_ep((sr_y >> i) & 1, sbac, bs);
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
        sbac_encode_bins_ep_msb((1 << (length + 1)) - 2, length + 1, sbac, bs);
        sbac_encode_bins_ep_msb((code_number % (1 << rparam)), rparam, sbac, bs);
    }
    else
    {
        length = rparam;
        code_number = code_number - (g_go_rice_range[rparam] << rparam);
        while (code_number >= (1 << length))
        {
            code_number -= (1 << (length++));
        }
        sbac_encode_bins_ep_msb((1 << (g_go_rice_range[rparam] + length + 1 - rparam)) - 2, g_go_rice_range[rparam] + length + 1 - rparam, sbac, bs);
        sbac_encode_bins_ep_msb(code_number, length, sbac, bs);
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

static void evce_eco_ccA(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type)
{
        int width = 1 << log2_w;
        int height = 1 << log2_h;
        int offset0;
        EVCE_SBAC    * sbac = GET_SBAC_ENC(bs);
        SBAC_CTX_MODEL* cm_gt0;
        SBAC_CTX_MODEL* cm_gtx;
        int scan_type = COEF_SCAN_ZIGZAG;
        int log2_block_size = min(log2_w, log2_h);
        u16 *scan;
        int scan_pos_last = -1;
        int sr_x = 0, sr_y = 0;
        int ipos;
        int last_scan_set;
        int rice_param;
        int sub_set;

        int ctx_gt0 = 0;
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
        int sig;


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
                sr_y = scan_pos >> log2_w;
                sr_x = scan_pos - (sr_y << log2_w);

                numNonZeroCoefs++;
                last_pos_in_scan = blk_pos;
                last_pos_in_raster_from_scan = scan_pos;
            }
        }
        code_positionLastXY(bs, sr_x, sr_y, width, height, ch_type);

        //===== code significance flag =====
        last_scan_set = last_pos_in_scan >> cg_log2_size;  
        if (sbac->ctx.sps_cm_init_flag == 1)
        {
            offset0 = log2_block_size <= 2 ? 0 : NUM_CTX_GT0_LUMA_TU << (EVC_MIN(1, (log2_block_size - 3)));
            cm_gt0 = (ch_type == Y_C) ? sbac->ctx.cc_gt0 + offset0 : sbac->ctx.cc_gt0 + NUM_CTX_GT0_LUMA;
            cm_gtx = (ch_type == Y_C) ? sbac->ctx.cc_gtA : sbac->ctx.cc_gtA + NUM_CTX_GTA_LUMA;
        }
        else
        {
            cm_gt0 = (ch_type == Y_C) ? sbac->ctx.cc_gt0 : sbac->ctx.cc_gt0 + 1;
            cm_gtx = (ch_type == Y_C) ? sbac->ctx.cc_gtA : sbac->ctx.cc_gtA + 1;
        }
        rice_param = 0;
        ipos = last_pos_in_scan;

        for (sub_set = last_scan_set; sub_set >= 0; sub_set--)
        {
            int num_nz = 0;
            int sub_pos = sub_set << cg_log2_size;
            int coef_signs = 0;
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
                    sig = (coef[blkpos] != 0 ? 1 : 0);
                    if (ipos == last_pos_in_scan)
                    {
                        ctx_gt0 = 0;
                    }
                    else
                    {
#if M50631_IMPROVEMENT_ADCC_CTXGT12
                        ctx_gt0 = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gt0_inc(coef, blkpos, width, height, ch_type) : 0;
#else
                        ctx_gt0 = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gt0_inc(coef, blkpos, width, height, ch_type, sr_x, sr_y) : 0;
#endif
                    }

                    if (!(ipos == last_pos_in_scan)) 
                    {
                        evce_sbac_encode_bin((u32)sig, sbac, &cm_gt0[ctx_gt0], bs);
                    }

                    if (sig)
                    {
                        pos[num_nz] = blkpos;  
                        abs_coef[num_nz] = (int)(EVC_ABS(coef[blkpos]));
                        coef_signs = 2 * coef_signs + (coef[blkpos] < 0 ? 1 : 0);
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
                        u32 symbol = abs_coef[idx] > 1 ? 1 : 0;
                        if (pos[idx] != pos_last)  
                        {
#if M50631_IMPROVEMENT_ADCC_CTXGT12
                            ctx_gtA = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtA_inc(coef, pos[idx], width, height, ch_type) : 0;
#else
                            ctx_gtA = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtA_inc(coef, pos[idx], width, height, ch_type, sr_x, sr_y) : 0;
#endif
                        }
                        evce_sbac_encode_bin(symbol, sbac, &cm_gtx[ctx_gtA], bs);
                        if (symbol)
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
                    if (firstC2FlagIdx !=-1)
                    {
                        u32 symbol2 = abs_coef[firstC2FlagIdx] > 2 ? 1 : 0;
                        if (pos[firstC2FlagIdx] != pos_last)  
                        {
#if M50631_IMPROVEMENT_ADCC_CTXGT12
                            ctx_gtB = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtB_inc(coef, pos[firstC2FlagIdx], width, height, ch_type) : 0;
#else
                            ctx_gtB = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtB_inc(coef, pos[firstC2FlagIdx], width, height, ch_type, sr_x, sr_y) : 0;
#endif
                        }
                        evce_sbac_encode_bin(symbol2, sbac, &cm_gtx[ctx_gtB], bs);

                        if (symbol2 != 0)
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
                                int escape_code_value = abs_coef[idx] - base_level;
                                rice_param = get_rice_para(coef, pos[idx], width, height, base_level);
                                code_coef_remain_exgolomb(bs, escape_code_value, rice_param);
                            }
                            if (abs_coef[idx] >= 2)
                            {
                                iFirstCoeff2 = 0;
                            }
                        }
                    }
                    sbac_encode_bins_ep_msb(coef_signs, num_nz, sbac, bs);
                }
            }
        }
    }
#endif

#if ATS_INTRA_PROCESS   
static int evce_eco_ats_intra_cu(EVC_BSW *bs, u8 ats_intra_cu, u8 ctx)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
#if M50632_SIMPLIFICATION_ATS
    sbac_encode_bin_ep(ats_intra_cu, sbac, bs);
#else
    evce_sbac_encode_bin(ats_intra_cu, sbac, sbac->ctx.ats_intra_cu + ctx, bs);
#endif
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra CU ");
    EVC_TRACE_INT(ats_intra_cu);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

static int evce_eco_ats_tu_h(EVC_BSW *bs, u8 ats_tu_h, u8 ctx)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
#if M50632_SIMPLIFICATION_ATS
    evce_sbac_encode_bin(ats_tu_h, sbac, sbac->ctx.ats_tu + ctx, bs);
#else
    evce_sbac_encode_bin(ats_tu_h, sbac, sbac->ctx.ats_tu_h + ctx, bs);
#endif
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuH ");
    EVC_TRACE_INT(ats_tu_h);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

static int evce_eco_ats_tu_v(EVC_BSW *bs, u8 ats_tu_v, u8 ctx)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
#if M50632_SIMPLIFICATION_ATS
    evce_sbac_encode_bin(ats_tu_v, sbac, sbac->ctx.ats_tu + ctx, bs);
#else
    evce_sbac_encode_bin(ats_tu_v, sbac, sbac->ctx.ats_tu_v + ctx, bs);
#endif
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuV ");
    EVC_TRACE_INT(ats_tu_v);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}
#endif

void evce_eco_xcoef(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type
#if ADCC  
                     , int tool_adcc
#endif
)
{
#if ADCC
    if (tool_adcc)
        evce_eco_ccA(bs, coef, log2_w, log2_h, num_sig, (ch_type == Y_C ? 0 : 1));
    else
#endif
    evce_eco_run_length_cc(bs, coef, log2_w, log2_h, num_sig, (ch_type == Y_C ? 0 : 1));

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

#if ATS_INTER_PROCESS
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
        u8 ats_inter_dir = is_ats_inter_horizontal(ats_inter_idx);
        u8 ats_inter_quad = is_ats_inter_quad_size(ats_inter_idx);
        u8 ats_inter_pos = get_ats_inter_pos(ats_inter_info);
        int size = 1 << (log2_cuw + log2_cuh);

        EVCE_SBAC    *sbac;
        EVC_SBAC_CTX *sbac_ctx;
        sbac = GET_SBAC_ENC(bs);
        sbac_ctx = &sbac->ctx;

        u8 ctx_ats_inter_flag = sbac->ctx.sps_cm_init_flag == 1 ? (size >= 256 ? 0 : 1) : 0;
        u8 ctx_ats_inter_quad = sbac->ctx.sps_cm_init_flag == 1 ? 2 : 1;
        u8 ctx_ats_inter_dir = sbac->ctx.sps_cm_init_flag == 1 ? (((log2_cuw == log2_cuh) ? 0 : (log2_cuw < log2_cuh ? 1 : 2)) + 3) : 2;
        u8 ctx_ats_inter_pos = sbac->ctx.sps_cm_init_flag == 1 ? 6 : 3;

        if (ats_inter_idx == 0)
            assert(ats_inter_pos == 0);

        evce_sbac_encode_bin(ats_inter_flag, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_flag, bs);
        EVC_TRACE_STR("ats_inter_flag ");
        EVC_TRACE_INT(ats_inter_flag);
        EVC_TRACE_STR("\n");

        if (ats_inter_flag)
        {
            if ((mode_vert_quad || mode_hori_quad) && (mode_vert || mode_hori))
            {
                evce_sbac_encode_bin(ats_inter_quad, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_quad, bs);
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
                evce_sbac_encode_bin(ats_inter_dir, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_dir, bs);
                EVC_TRACE_STR("ats_inter_dir ");
                EVC_TRACE_INT(ats_inter_dir);
                EVC_TRACE_STR("\n");
            }
            else
            {
                assert(ats_inter_dir == ((ats_inter_quad && mode_hori_quad) || (!ats_inter_quad && mode_hori)));
            }

            evce_sbac_encode_bin(ats_inter_pos, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_pos, bs);
            EVC_TRACE_STR("ats_inter_pos ");
            EVC_TRACE_INT(ats_inter_pos);
            EVC_TRACE_STR("\n");
        }

        return EVC_OK;
    }
}
#endif

int evce_eco_cbf(EVC_BSW * bs, int cbf_y, int cbf_u, int cbf_v, u8 pred_mode, int b_no_cbf, int is_sub,int sub_pos, int cbf_all, int run[N_C]
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    EVCE_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;

    sbac = GET_SBAC_ENC(bs);
    sbac_ctx = &sbac->ctx;
       
    /* code allcbf */
    if(pred_mode != MODE_INTRA
#if M50761_CHROMA_NOT_SPLIT 
        && !evc_check_only_intra(tree_cons)
#endif
        )
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
                evce_sbac_encode_bin(0, sbac, sbac_ctx->all_cbf, bs);

                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("all_cbf ");
                EVC_TRACE_INT(0);
                EVC_TRACE_STR("\n");

                return EVC_OK;
            }
            else
            {
                evce_sbac_encode_bin(1, sbac, sbac_ctx->all_cbf, bs);

                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("all_cbf ");
                EVC_TRACE_INT(1);
                EVC_TRACE_STR("\n");
            }
        }
 
        if (run[U_C])
        {
            evce_sbac_encode_bin(cbf_u, sbac, sbac_ctx->cbf + 1, bs);
        }
        if (run[V_C])
        {
            evce_sbac_encode_bin(cbf_v, sbac, sbac_ctx->cbf + 2, bs);
        }

        if (run[Y_C] && (cbf_u + cbf_v != 0 || is_sub))
        {
            evce_sbac_encode_bin(cbf_y, sbac, sbac_ctx->cbf + 0, bs);
        }
    }
    else
    {
        if (run[U_C])
        {
#if M50761_CHROMA_NOT_SPLIT 
            evc_assert(evc_check_chroma(tree_cons));
#endif
            evce_sbac_encode_bin(cbf_u, sbac, sbac_ctx->cbf + 1, bs);
        }
        if (run[V_C])
        {
#if M50761_CHROMA_NOT_SPLIT 
            evc_assert(evc_check_chroma(tree_cons));
#endif
            evce_sbac_encode_bin(cbf_v, sbac, sbac_ctx->cbf + 2, bs);
        }
        if (run[Y_C])
        {
#if M50761_CHROMA_NOT_SPLIT 
            evc_assert(evc_check_luma(tree_cons));
#endif
            evce_sbac_encode_bin(cbf_y, sbac, sbac_ctx->cbf + 0, bs);
        }
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("cbf Y ");
    EVC_TRACE_INT(cbf_y);
    EVC_TRACE_STR("cbf U ");
    EVC_TRACE_INT(cbf_u);
    EVC_TRACE_STR("cbf V ");
    EVC_TRACE_INT(cbf_v);
    EVC_TRACE_STR("\n");

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

    sbac_write_unary_sym(t0, NUM_DELTA_QP_CTX, sbac, sbac_ctx->delta_qp, bs);

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
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
                  , int tool_ats
#endif
#if ATS_INTRA_PROCESS    
                  , u8 ats_intra_cu, u8 ats_tu
#endif
#if ATS_INTER_PROCESS
                  , u8 ats_inter_info
#endif
#if ADCC  
    , EVCE_CTX * ctx
#endif
#if DQP
    , EVCE_CORE * core, int enc_dqp, u8 cur_qp
#endif
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{    
#if M50761_CHROMA_NOT_SPLIT
    run_stats = evc_get_run(run_stats, tree_cons);
#endif
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
#if M50761_CHROMA_NOT_SPLIT
    if (!evc_check_luma(tree_cons))
    {
        evc_assert(run[0] == 0);
    }
    if (!evc_check_chroma(tree_cons))
    {
        evc_assert((run[1] == 0) && (run[2] == 0));
    }
    evc_assert(run_stats != 0);
#endif

    int cbf_all = 0;
#if ATS_INTRA_PROCESS
    u8 is_intra = (pred_mode == MODE_INTRA) ? 1 : 0;
    EVCE_SBAC    * sbac = GET_SBAC_ENC(bs);
#endif
#if ATS_INTER_PROCESS
    u8 ats_inter_avail = check_ats_inter_info_coded(1 << log2_cuw, 1 << log2_cuh, pred_mode, tool_ats);
    if( ats_inter_avail )
    {
        get_tu_size( ats_inter_info, log2_cuw, log2_cuh, &log2_w_sub, &log2_h_sub );
        sub_stride = (1 << log2_w_sub);
    }
#endif
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
            evce_eco_cbf(bs, !!nnz_sub[Y_C][(j << 1) | i], !!nnz_sub[U_C][(j << 1) | i], !!nnz_sub[V_C][(j << 1) | i], pred_mode, b_no_cbf, is_sub, j + i, cbf_all, run
#if M50761_CHROMA_NOT_SPLIT
                , tree_cons
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
                        evce_eco_dqp(bs, ctx->sh.qp_prev_eco, cur_qp);

                        //evce_eco_dqp(bs, ctx->sh.qp, cur_qp - 6 * (BIT_DEPTH - 8));
                        //printf("eco (%4d,%4d) [%4d %4d] qp=(%d,%d)\n", core->x_scu * 4, core->y_scu*4, 1 << log2_cuw, 1 << log2_cuh, cur_qp, ctx->sh.qp_prev_eco);
                            core->cu_qp_delta_is_coded = 1;
                        ctx->sh.qp_prev_eco = cur_qp;
                    }
                }
            }
#endif

#if ATS_INTRA_PROCESS
            if (tool_ats && (!!nnz_sub[Y_C][(j << 1) | i]) && (log2_cuw <= 5 && log2_cuh <= 5) && is_intra
#if M50761_CHROMA_NOT_SPLIT
                && run[Y_C]
#endif
            )
            {
#if M50632_SIMPLIFICATION_ATS
                evce_eco_ats_intra_cu(bs, ats_intra_cu, 0);
#else
                evce_eco_ats_intra_cu(bs, ats_intra_cu, sbac->ctx.sps_cm_init_flag == 1 ? ((log2_cuw > log2_cuh) ? log2_cuw : log2_cuh) - MIN_CU_LOG2 : 0);
#endif
                if (ats_intra_cu)
                {
#if M50632_SIMPLIFICATION_ATS
                    evce_eco_ats_tu_h(bs, (ats_tu >> 1), 0);
                    evce_eco_ats_tu_v(bs, (ats_tu & 1), 0);
#else
                    evce_eco_ats_tu_h(bs, (ats_tu >> 1), is_intra);
                    evce_eco_ats_tu_v(bs, (ats_tu & 1), is_intra);
#endif
                }
        }
#endif

#if ATS_INTER_PROCESS
#if IBC
            if (pred_mode != MODE_INTRA && pred_mode != MODE_IBC && run[Y_C] && run[U_C] && run[V_C])
#else
            if (pred_mode != MODE_INTRA && run[Y_C] && run[U_C] && run[V_C])
#endif
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
#endif
            
            for (c = 0; c < N_C; c++)
            {
                if (nnz_sub[c][(j << 1) | i] && run[c])
                {
                    int pos_sub_x = i * (1 << (log2_w_sub - !!c));
                    int pos_sub_y = j * (1 << (log2_h_sub - !!c)) * (stride >> (!!c));

                    if (is_sub)
                    {
                        evc_block_copy(coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), coef_temp_buf[c], sub_stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
                        coef_temp[c] = coef_temp_buf[c];
                    }
                    else
                    {
                        coef_temp[c] = coef[c];
                    }

                    evce_eco_xcoef(bs, coef_temp[c], log2_w_sub - (!!c), log2_h_sub - (!!c), nnz_sub[c][(j << 1) | i], c
#if ADCC  
                                    , ctx->sps.tool_adcc
#endif   
                    );

                    if (is_sub)
                    {
                        evc_block_copy(coef_temp_buf[c], sub_stride >> (!!c), coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
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

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("pred mode ");
    EVC_TRACE_INT(!!pred_mode);
    EVC_TRACE_STR("\n");

    evce_sbac_encode_bin(pred_mode == MODE_INTRA, sbac, sbac->ctx.pred_mode + ctx, bs);

    return EVC_OK;
}
#if IBC
int evce_eco_ibc(EVC_BSW * bs, u8 pred_mode_ibc_flag, int ctx)
{
  EVCE_SBAC * sbac = GET_SBAC_ENC(bs);

  EVC_TRACE_COUNTER;
  EVC_TRACE_STR("IBC pred mode ");
  EVC_TRACE_INT(!!pred_mode_ibc_flag);
  EVC_TRACE_STR("\n");

  evce_sbac_encode_bin(pred_mode_ibc_flag, sbac, sbac->ctx.ibc_flag + ctx, bs);

  return EVC_OK;
}
#endif
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
        sbac_encode_bins_ep_msb(symbol, threshold, sbac, bs);
    }
    else
    {
        symbol += val - b;
        assert(symbol < (val << 1));
        assert((symbol >> 1) >= val - b);
        sbac_encode_bins_ep_msb(symbol, threshold + 1, sbac, bs);
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
        evce_sbac_encode_bin(1, sbac, sbac->ctx.intra_dir, bs);
        t0 = ipm == mpm[0] ? 0 : 1;
        evce_sbac_encode_bin(t0, sbac, sbac->ctx.intra_dir + 1, bs);
    }
    else
    {
        int i;
        int pms_cnt = -1;
        int flag = 0;
        int check = 8;

        evce_sbac_encode_bin(0, sbac, sbac->ctx.intra_dir, bs);

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

    sbac = GET_SBAC_ENC(bs);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ipm UV ");
    EVC_TRACE_INT(ipm);
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_STR("ipm L ");
    EVC_TRACE_INT(ipm_l);
#endif
    EVC_TRACE_STR("\n");

    EVC_IPRED_CONV_L2C_CHK(ipm_l, chk_bypass);

    if(ipm == 0)
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.intra_dir + 2, bs);
    }
    else
    {
        evce_sbac_encode_bin(0, sbac, sbac->ctx.intra_dir + 2, bs);
        ipm = (chk_bypass && ipm > ipm_l) ? ipm - 2 : ipm - 1;
        sbac_write_unary_sym_ep(ipm, sbac, bs, IPD_CHROMA_CNT - 1);
    }
       
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

    sbac_write_truncate_unary_sym(var0, NUM_SBAC_CTX_MMVD_MERGE_IDX, MMVD_BASE_MV_NUM, sbac, sbac->ctx.mmvd_merge_idx, bs); /* mmvd_merge_idx */
    sbac_write_truncate_unary_sym(var1, NUM_SBAC_CTX_MMVD_DIST_IDX, MMVD_DIST_NUM, sbac, sbac->ctx.mmvd_distance_idx, bs); /* mmvd_distance_idx */

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

void evce_eco_inter_dir(EVC_BSW *bs, s8 refi[REFP_NUM]
#if REMOVE_BI_INTERDIR
    , int slice_type, int cuw, int cuh
#endif
)
{
    EVCE_SBAC *sbac;

    sbac = GET_SBAC_ENC(bs);
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("inter dir ");
    if(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1])) /* PRED_BI */
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.inter_dir + 1, bs);
        EVC_TRACE_INT(PRED_BI);
#if REMOVE_BI_INTERDIR
        evc_assert(check_bi_applicability_rdo(slice_type, cuw, cuh));
#endif
    }
    else
    {
#if REMOVE_BI_INTERDIR
        if (check_bi_applicability_rdo(slice_type, cuw, cuh))
#endif
        evce_sbac_encode_bin(0, sbac, sbac->ctx.inter_dir + 1, bs);
        if(REFI_IS_VALID(refi[REFP_0])) /* PRED_L0 */
        {
            evce_sbac_encode_bin(0, sbac, sbac->ctx.inter_dir + 2, bs);
            EVC_TRACE_INT(PRED_L0);
        }
        else /* PRED_L1 */
        {
            evce_sbac_encode_bin(1, sbac, sbac->ctx.inter_dir + 2, bs);
            EVC_TRACE_INT(PRED_L1);
        }
    }

    EVC_TRACE_STR("\n");

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

int evce_eco_mvp_idx(EVC_BSW *bs, int mvp_idx, int sps_amis_flag)
{
    EVCE_SBAC *sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;

    if(sps_amis_flag == 1)
    {
        sbac_write_truncate_unary_sym(mvp_idx, NUM_MVP_IDX_CTX, MAX_NUM_MVP, sbac, sbac_ctx->mvp_idx, bs);
    }
    else
    {
        sbac_write_truncate_unary_sym(mvp_idx, 3, 4, sbac, sbac_ctx->mvp_idx, bs);
    }

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

    sbac_write_truncate_unary_sym( mvp_idx, NUM_AFFINE_MVP_IDX_CTX, AFF_MAX_NUM_MVP, sbac, sbac_ctx->affine_mvp_idx, bs );

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
#if IBC
    core->ibc_flag = 0;
#endif
    core->mmvd_flag = 0;
#if AFFINE
    core->affine_flag = cu_data->affine_flag[cup];
#endif
    core->nnz[Y_C] = core->nnz[U_C] = core->nnz[V_C] = 0;
#if ATS_INTER_PROCESS //assign ctu data to cu
    core->ats_inter_info = cu_data->ats_inter_info[cup];
#endif
#if M50761_CHROMA_NOT_SPLIT
    core->cu_mode = evce_check_luma(ctx) ? cu_data->pred_mode[cup] : cu_data->pred_mode_chroma[cup];
#endif

    if(
#if M50761_CHROMA_NOT_SPLIT
        core->cu_mode
#else
        cu_data->pred_mode[cup] 
#endif
        == MODE_INTRA)
    {
        core->avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->log2_cuw, core->log2_cuh, ctx->map_scu
#if EVC_TILE_SUPPORT
            , ctx->map_tidx
#endif 
        );
    }
#if IBC
    else if (
#if M50761_CHROMA_NOT_SPLIT
        core->cu_mode
#else
        cu_data->pred_mode[cup]
#endif
        == MODE_IBC)
    {
      core->ibc_flag = 1;
#if M50761_CHROMA_NOT_SPLIT
      if (!evce_check_luma(ctx))
      {
          evc_assert(0);
      }
#endif
      core->mmvd_flag = 0; // core->new_skip_flag = 0;
      core->affine_flag = 0;
      core->avail_cu = evc_get_avail_ibc(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu);
    }
#endif
    else
    {
#if M50761_CHROMA_NOT_SPLIT
        evc_assert(!evce_check_only_intra(ctx));
        evc_assert(evce_check_luma(ctx));
#endif
        if((cu_data->pred_mode[cup] == MODE_SKIP) || (cu_data->pred_mode[cup] == MODE_SKIP_MMVD))
        {
            core->skip_flag = 1;
        }

        if(cu_data->pred_mode[cup] == MODE_SKIP_MMVD)
        {
            core->mmvd_flag = 1;
        }

        core->avail_cu = evc_get_avail_inter(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu
#if EVC_TILE_SUPPORT
            , ctx->map_tidx
#endif 
        );
    }

    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu
#if EVC_TILE_SUPPORT
        , ctx->map_tidx
#endif
    );
#if DMVR_FLAG
    core->dmvr_flag = 0;
#endif
    return EVC_OK;
}

static void coef_rect_to_series(EVCE_CTX * ctx, s16 *coef_src[N_C], int x, int y, int cuw, int cuh, s16 coef_dst[N_C][MAX_CU_DIM])
{
    int i, j, sidx, didx;

    sidx = (x&(ctx->max_cuwh - 1)) + ((y&(ctx->max_cuwh - 1)) << ctx->log2_max_cuwh);
    didx = 0;

#if M50761_CHROMA_NOT_SPLIT
    if (evce_check_luma(ctx))
    {
#endif
    for(j = 0; j < cuh; j++)
    {
        for(i = 0; i < cuw; i++)
        {
            coef_dst[Y_C][didx++] = coef_src[Y_C][sidx + i];
        }
        sidx += ctx->max_cuwh;
    }
#if M50761_CHROMA_NOT_SPLIT
    }
    if (evce_check_chroma(ctx))
    {
#endif

    x >>= 1;
    y >>= 1;
    cuw >>= 1;
    cuh >>= 1;

    sidx = (x&((ctx->max_cuwh >> 1) - 1)) + ((y&((ctx->max_cuwh >> 1) - 1)) << (ctx->log2_max_cuwh - 1));

    didx = 0;

    for(j = 0; j < cuh; j++)
    {
        for(i = 0; i < cuw; i++)
        {
            coef_dst[U_C][didx] = coef_src[U_C][sidx + i];
            coef_dst[V_C][didx] = coef_src[V_C][sidx + i];
            didx++;
        }
        sidx += (ctx->max_cuwh >> 1);
    }
#if M50761_CHROMA_NOT_SPLIT
    }
#endif
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
#if M50761_CHROMA_NOT_SPLIT
    evc_assert(evce_check_luma(c));
#endif
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
        evce_sbac_encode_bin(split_mode != NO_SPLIT, sbac, sbac->ctx.btt_split_flag, bs);

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
                         , NULL, c->sps.sps_btt_flag
#if M50761_CHROMA_NOT_SPLIT
        , c->tree_cons
#endif
    );

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
  
    if (split_mode_sum == 1)
    {
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

            avail[0] = y_scu > 0;  //up
            if(x_scu > 0)
                avail[1] = MCU_GET_COD(c->map_scu[scup - 1]); //left
            if(x_scu + scuw < w_scu)
                avail[2] = MCU_GET_COD(c->map_scu[scup + scuw]); //right
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
    u8 allow_suco = c->sps.sps_suco_flag ? evc_check_suco_cond(cuw, cuh, split_mode, boundary, log2_max_cuwh, c->sps.log2_diff_ctu_size_max_suco_cb_size, c->sps.log2_diff_max_suco_min_suco_cb_size) : 0;

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

#if M50761_CHROMA_NOT_SPLIT
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
#endif

int evce_eco_unit(EVCE_CTX * ctx, EVCE_CORE * core, int x, int y, int cup, int cuw, int cuh
#if M50761_CHROMA_NOT_SPLIT 
    , TREE_CONS tree_cons
#endif
)
{
#if M50761_CHROMA_NOT_SPLIT 
    ctx->tree_cons = tree_cons;
#endif
    s16(*coef)[MAX_CU_DIM] = core->ctmp;
    EVC_BSW *bs;
    u32 *map_scu;
    int slice_type, refi0, refi1;
    int i, j, w, h;
    EVCE_CU_DATA *cu_data = &ctx->map_cu_data[core->lcu_num];
    u32 *map_cu_mode;
#if AFFINE
    u32 *map_affine;
#endif
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
#if M50761_CHROMA_NOT_SPLIT
    EVC_TRACE_STR("tree status ");
    EVC_TRACE_INT(ctx->tree_cons.tree_type);
    EVC_TRACE_STR("mode status ");
    EVC_TRACE_INT(ctx->tree_cons.mode_cons);
#endif
    EVC_TRACE_STR("\n");

    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
#if IBC
      , ctx->param.use_ibc_flag, ctx->sps.ibc_log_max_size
#endif
    );

#if FIX_IBC_PRED_MODE_4x4
    if (ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2) 
    {
#if IBC
        evc_assert(cu_data->pred_mode[cup] == MODE_INTRA || cu_data->pred_mode[cup] == MODE_IBC);
#else
        evc_assert(cu_data->pred_mode[cup] == MODE_INTRA);
#endif
    }
#endif
    
    if (core->skip_flag == 0)
    {
        /* get coefficients and tq */
        coef_rect_to_series(ctx, cu_data->coef, x, y, cuw, cuh, coef);

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
    if(slice_type != SLICE_I && 
#if IBC
    (ctx->sps.tool_amis == 0 || !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2) || ctx->param.use_ibc_flag)
#else  
      (ctx->sps.tool_amis == 0 || !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2))
#endif
#if M50761_CHROMA_NOT_SPLIT
        && !evce_check_only_intra(ctx)
#endif
      )
    {
#if M50761_REMOVE_BIBLOCKS_8x4 
        if (!check_bi_applicability_rdo(ctx->sh.slice_type, cuw, cuh) && REFI_IS_VALID(cu_data->refi[cup][REFP_0]) && REFI_IS_VALID(cu_data->refi[cup][REFP_1]))
        {
            evc_assert(0);
        }
#endif
#if IBC
        if(!(ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2))
        {
#endif
        evce_eco_skip_flag(bs, core->skip_flag, ctx->ctx_flags[CNID_SKIP_FLAG]);
#if IBC
        }
#endif
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
#if AFFINE
                if(cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine)
                {
                    evce_eco_affine_flag(bs, core->affine_flag != 0, ctx->ctx_flags[CNID_AFFN_FLAG]); /* skip affine_flag */
                }

                if(core->affine_flag)
                {
                    evce_eco_affine_mrg_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                }
                else
#endif
                {
                    evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_0], ctx->sps.tool_amis);

                    if(ctx->sps.tool_amis == 0 && slice_type == SLICE_B)
                    {
                        evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_1], ctx->sps.tool_amis);
                    }
                }
            }
        }
        else
        {
#if M50761_CHROMA_NOT_SPLIT
            if (evce_check_all_preds(ctx))
#endif
#if FIX_IBC_PRED_MODE_4x4
            if (!(ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2))
#endif
            evce_eco_pred_mode(bs, 
#if M50761_CHROMA_NOT_SPLIT
                core->cu_mode
#else
                cu_data->pred_mode[cup]
#endif
                , ctx->ctx_flags[CNID_PRED_MODE]);
#if IBC
            if ((((
#if M50761_CHROMA_NOT_SPLIT
                core->cu_mode
#else
                cu_data->pred_mode[cup]
#endif
                != MODE_INTRA)
#if FIX_IBC_PRED_MODE_4x4
                || (ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2)
#endif
                )
#if M50761_CHROMA_NOT_SPLIT
                && !evce_check_only_inter(ctx)
#endif
                ) 
#if M50761_CHROMA_NOT_SPLIT
                && evce_check_luma(ctx)
#endif
                && ctx->param.use_ibc_flag && core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
            {

              evce_eco_ibc_flag(bs, core->ibc_flag, ctx->ctx_flags[CNID_IBC_FLAG]);
            }
#endif
            if(
#if M50761_CHROMA_NOT_SPLIT
                core->cu_mode
#else
                cu_data->pred_mode[cup]
#endif
                != MODE_INTRA
#if IBC
              && 
#if M50761_CHROMA_NOT_SPLIT
                core->cu_mode
#else
                cu_data->pred_mode[cup] 
#endif
                != MODE_IBC
#endif   

              )
            {
                if(ctx->sps.tool_amvr)
                {
                    evce_eco_mvr_idx(bs, cu_data->mvr_idx[cup]);
                }

                {
                    if(ctx->sps.tool_mmvd)
                    {
                        if(cu_data->mvr_idx[cup] == 0)
                        {
                            evce_eco_inter_t_direct(bs, cu_data->pred_mode[cup] == MODE_DIR || cu_data->pred_mode[cup] == MODE_DIR_MMVD);
                        }
                    }
                    else
                    {
                        if(cu_data->mvr_idx[cup] == 0)
                        {
                            evce_eco_inter_t_direct(bs, cu_data->pred_mode[cup] == MODE_DIR);
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

#if AFFINE
                    if(cu_data->pred_mode[cup] == MODE_DIR && cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine)
                    {
                        evce_eco_affine_flag(bs, core->affine_flag != 0, ctx->ctx_flags[CNID_AFFN_FLAG]); /* direct affine_flag */
                        if(core->affine_flag)
                        {
                            evce_eco_affine_mrg_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                        }
                    }
#endif
#if MERGE
                    if(ctx->sps.tool_amis == 1 && cu_data->pred_mode[cup] == MODE_DIR
#if AFFINE
                       && !core->affine_flag
#endif
                       && cu_data->mvr_idx[cup] == 0
                       )
                    {
                        evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_0], ctx->sps.tool_amis);
                    }
#endif
                }

                if(((cu_data->pred_mode[cup] % ORG_PRED_NUM) != MODE_DIR) && ((cu_data->pred_mode[cup] % ORG_PRED_NUM) != MODE_DIR_MMVD))
                {
                    evce_eco_inter_dir(bs, cu_data->refi[cup]
#if REMOVE_BI_INTERDIR
                        , slice_type, cuw, cuh
#endif
                    );

#if AFFINE // affine inter mode
                    if(cuw >= 16 && cuh >= 16 && cu_data->mvr_idx[cup] == 0 && ctx->sps.tool_affine)
                    {
                        evce_eco_affine_flag(bs, core->affine_flag != 0, ctx->ctx_flags[CNID_AFFN_FLAG]); /* inter affine_flag */
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
#endif
                        if(ctx->sps.tool_amis == 1 && REFI_IS_VALID(cu_data->refi[cup][REFP_0]) && REFI_IS_VALID(cu_data->refi[cup][REFP_1]))
                        {
                            evce_eco_bi_idx(bs, cu_data->bi_idx[cup] - 1);
                        }

                        refi0 = cu_data->refi[cup][REFP_0];
                        refi1 = cu_data->refi[cup][REFP_1];
                        if(IS_INTER_SLICE(slice_type) && REFI_IS_VALID(refi0))
                        {
                            if(ctx->sps.tool_amis == 0)
                            {
                                evce_eco_refi(bs, ctx->rpm.num_refp[REFP_0], refi0);
                                evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_0], ctx->sps.tool_amis);
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
                            if(ctx->sps.tool_amis == 0)
                            {
                                evce_eco_refi(bs, ctx->rpm.num_refp[REFP_1], refi1);
                                evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_1], ctx->sps.tool_amis);
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
#if AFFINE
                    }
#endif
                }
            }
        }
    }
#if IBC
    else if (((ctx->sh.slice_type == SLICE_I
#if M50761_CHROMA_NOT_SPLIT
        || evce_check_only_intra(ctx)
#endif
        ) && ctx->param.use_ibc_flag))
    {
      if (core->skip_flag == 0
#if M50761_CHROMA_NOT_SPLIT
          && evce_check_luma(ctx)
#endif
          )
      {
        if (core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
        {
          evce_eco_ibc_flag(bs, core->ibc_flag, ctx->ctx_flags[CNID_IBC_FLAG]);
        }
      }
    }
#endif
    if(
#if M50761_CHROMA_NOT_SPLIT
        core->cu_mode
#else
        cu_data->pred_mode[cup]
#endif
        == MODE_INTRA)
    {
        evc_assert(cu_data->ipm[0][cup] != IPD_INVALID);
        evc_assert(cu_data->ipm[1][cup] != IPD_INVALID);

        if(ctx->sps.tool_eipd)
        {
            evc_get_mpm(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                        core->mpm, core->avail_lr, core->mpm_ext, core->pims);
#if M50761_CHROMA_NOT_SPLIT
            if (evce_check_luma(ctx))
            {
#endif
            evce_eco_intra_dir(bs, cu_data->ipm[0][cup], core->mpm, core->mpm_ext, core->pims);
#if M50761_CHROMA_NOT_SPLIT 
            }
            if (evce_check_chroma(ctx))
            {
                int luma_ipm = cu_data->ipm[0][cup];
                if (!evce_check_luma(ctx))
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
#endif
                evc_assert(cu_data->ipm[1][cup] != IPD_INVALID);
            evce_eco_intra_dir_c(bs, cu_data->ipm[1][cup], 
#if M50761_CHROMA_NOT_SPLIT 
                luma_ipm
#else
                cu_data->ipm[0][cup]
#endif
            );
#if M50761_CHROMA_NOT_SPLIT
            }
#endif
        }
        else
        {
            evc_get_mpm_b(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                          &core->mpm_b_list, core->avail_lr, core->mpm_ext, core->pims);
            evce_eco_intra_dir_b(bs, cu_data->ipm[0][cup], core->mpm_b_list, core->mpm_ext, core->pims);
        }
    }
#if IBC
    else if (core->ibc_flag)
    {
      if (core->skip_flag == 0)
      {
        if (
#if M50761_CHROMA_NOT_SPLIT
            core->cu_mode
#else
            cu_data->pred_mode[cup]
#endif
            == MODE_IBC)        // Does this condition required?
        {
#if M50761_CHROMA_NOT_SPLIT
            if (!evce_check_luma(ctx))
            {
#if M50761_CHROMA_NOT_SPLIT
                evc_assert(0);
#else
                s16 mvd_chroma[MV_D];
                mvd_chroma[0] = cu_data->bv_chroma[cup][0] >> 1;
                mvd_chroma[1] = cu_data->bv_chroma[cup][1] >> 1;
                evce_eco_mvd(bs, mvd_chroma);
#endif
            }
            else
#endif
            {
          evce_eco_mvd(bs, cu_data->mvd[cup][REFP_0]);
            }
        }
      }
    }
#endif

    if((core->skip_flag == 0) && (core->mmvd_flag == 0))
    {
        int b_no_cbf = 0;
#if AFFINE
        b_no_cbf |= cu_data->affine_flag[cup] && 
#if M50761_CHROMA_NOT_SPLIT
            core->cu_mode
#else
            cu_data->pred_mode[cup] 
#endif
            == MODE_DIR;
#endif
        b_no_cbf |= 
#if M50761_CHROMA_NOT_SPLIT
            core->cu_mode
#else
            cu_data->pred_mode[cup] 
#endif
            == MODE_DIR_MMVD;
#if MERGE
        b_no_cbf |= 
#if M50761_CHROMA_NOT_SPLIT
            core->cu_mode
#else
            cu_data->pred_mode[cup]
#endif
            == MODE_DIR;
#endif
        if(ctx->sps.tool_amis == 0)
            b_no_cbf = 0;
#if DQP
        enc_dqp = 1;
#endif
        evce_eco_coef(bs, coef, core->log2_cuw, core->log2_cuh, 
#if M50761_CHROMA_NOT_SPLIT
            core->cu_mode
#else
            cu_data->pred_mode[cup]
#endif
            , core->nnz_sub, b_no_cbf, RUN_L | RUN_CB | RUN_CR
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
                      , ctx->sps.tool_ats
#endif
#if ATS_INTRA_PROCESS            
                      , cu_data->ats_intra_cu[cup], (cu_data->ats_tu_h[cup] << 1 | cu_data->ats_tu_v[cup])
#endif
#if ATS_INTER_PROCESS
                      , core->ats_inter_info
#endif
#if ADCC || DQP
            , ctx
#endif
#if DQP
            , core, enc_dqp, cu_data->qp_y[cup] - 6 * (BIT_DEPTH - 8)
#endif
#if M50761_CHROMA_NOT_SPLIT
            , ctx->tree_cons
#endif
        );
    }

    map_scu = ctx->map_scu + core->scup;
    w = (core->cuw >> MIN_CU_LOG2);
    h = (core->cuh >> MIN_CU_LOG2);

#if AFFINE
    map_affine = ctx->map_affine + core->scup;
#endif
    map_cu_mode = ctx->map_cu_mode + core->scup;
#if M50761_CHROMA_NOT_SPLIT
    if (evce_check_luma(ctx))
    {
#endif
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
                MCU_SET_QP(map_scu[j], ctx->sh.qp_prev_eco);
            }
#endif

#if AFFINE
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
#endif
#if IBC
            if (core->ibc_flag)
            {
              MCU_SET_IBC(map_scu[j]);
            }
            else
            {
              MCU_CLR_IBC(map_scu[j]);
            }
#endif
            MCU_SET_LOGW(map_cu_mode[j], core->log2_cuw);
            MCU_SET_LOGH(map_cu_mode[j], core->log2_cuh);

            if(core->mmvd_flag)
                MCU_SET_MMVDS(map_cu_mode[j]);
            else
                MCU_CLR_MMVDS(map_cu_mode[j]);
        }
        map_scu += ctx->w_scu;
#if AFFINE
        map_affine += ctx->w_scu;
#endif
        map_cu_mode += ctx->w_scu;
    }
#if ATS_INTER_PROCESS //set cbf
    if (core->ats_inter_info)
    {
        assert(core->nnz_sub[Y_C][0] == core->nnz[Y_C]);
        assert(core->nnz_sub[U_C][0] == core->nnz[U_C]);
        assert(core->nnz_sub[V_C][0] == core->nnz[V_C]);
        set_cu_cbf_flags(core->nnz[Y_C], core->ats_inter_info, core->log2_cuw, core->log2_cuh, ctx->map_scu + core->scup, ctx->w_scu);
    }
#endif
#if M50761_CHROMA_NOT_SPLIT
    }
    if (evce_check_chroma(ctx))
    {
        if (!evce_check_luma(ctx))
        {
            evc_assert((core->cu_mode == MODE_INTRA) || (core->cu_mode == MODE_IBC));
        }
    }
#endif
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

#if AFFINE
        map_affine = ctx->map_affine + core->scup;
#endif

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

#if AFFINE
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
#endif
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
#if AFFINE
            map_affine += ctx->w_scu;
#endif
#if DMVR_LAG
            map_unrefined_mv += ctx->w_scu;
#endif
        }
    }
#endif

    return EVC_OK;
}

#if ALF

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

void evc_alfGolombEncode(EVC_BSW * bs, int coeff, int k)
{
    int symbol = abs(coeff);

    int m = (int)pow(2.0, k);
    int q = symbol / m;

    for(int i = 0; i < q; i++)
    {
        evc_bsw_write1(bs, 1);
    }
    evc_bsw_write1(bs, 0);
    // write one zero

    for(int i = 0; i < k; i++)
    {
        evc_bsw_write1(bs, symbol & 0x01);
        symbol >>= 1;
    }

    if(coeff != 0)
    {
        int sign = (coeff > 0) ? 1 : 0;
        evc_bsw_write1(bs, sign);
    }
};

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
    const evc_AlfSliceParam alfSliceParam = asp;
    if (!isChroma)
    {
        evc_bsw_write1(bs, alfSliceParam.coeffDeltaFlag); // "alf_coefficients_delta_flag"
        if (!alfSliceParam.coeffDeltaFlag)
        {
            if (alfSliceParam.numLumaFilters > 1)
            {
                evc_bsw_write1(bs, alfSliceParam.coeffDeltaPredModeFlag); // "coeff_delta_pred_mode_flag"
            }
        }
    }

    // this logic need to be moved to ALF files
    evc_AlfFilterShape alfShape;
    init_AlfFilterShape( &alfShape, isChroma ? 5 : ( alfSliceParam.lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );

    int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
    memset(bitsCoeffScan, 0, m_MAX_SCAN_VAL*m_MAX_EXP_GOLOMB * sizeof(int));

    const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;
    const short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
    const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;

    // vlc for all
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (isChroma || !alfSliceParam.coeffDeltaFlag || alfSliceParam.filterCoeffFlag[ind])
        {
            for (int i = 0; i < alfShape.numCoeff - 1; i++)
            {
                int coeffVal = abs(coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i]);

                for (int k = 1; k < 15; k++)
                {
                    bitsCoeffScan[alfShape.golombIdx[i]][k] += evc_lengthGolomb(coeffVal, k);
                }
            }
        }
    }

    int kMinTab[MAX_NUM_ALF_COEFF];
    int kMin = evc_getGolombKMin(&alfShape, numFilters, kMinTab, bitsCoeffScan);

    // Golomb parameters
    evc_bsw_write_ue(bs, kMin - 1); //"min_golomb_order" 

    for (int idx = 0; idx < maxGolombIdx; idx++)
    {
        BOOL golombOrderIncreaseFlag = (kMinTab[idx] != kMin) ? TRUE : FALSE;
        evc_bsw_write1(bs, golombOrderIncreaseFlag);
        kMin = kMinTab[idx];
    }

    if (!isChroma)
    {
        if (alfSliceParam.coeffDeltaFlag)
        {
            for (int ind = 0; ind < numFilters; ++ind)
            {
                evc_bsw_write1(bs, alfSliceParam.filterCoeffFlag[ind]);  // WRITE_FLAG(alfSliceParam.filterCoeffFlag[ind], "filter_coefficient_flag[i]");
            }
        }
    }

    // Filter coefficients
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (!isChroma && !alfSliceParam.filterCoeffFlag[ind] && alfSliceParam.coeffDeltaFlag)
        {
            continue;
        }

        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
            evc_alfGolombEncode(bs, coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]]);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
        }
    }
}
int evce_eco_alf_aps_param(EVC_BSW * bs, EVC_APS * aps)
{
    evc_AlfSliceParam alfSliceParam = aps->alf_aps_param;

    evc_bsw_write1(bs, alfSliceParam.enabledFlag[0]); //"alf_slice_enable_flag"
    if (!alfSliceParam.enabledFlag[0])
    {
        return 0;
    }

    const int alfChromaIdc = alfSliceParam.enabledFlag[1] * 2 + alfSliceParam.enabledFlag[2];
    evce_truncatedUnaryEqProb(bs, alfChromaIdc, 3);
    {
        evce_xWriteTruncBinCode(bs, alfSliceParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES);
        evc_bsw_write1(bs, !alfSliceParam.lumaFilterType); //  "filter_type_flag"

        if (alfSliceParam.numLumaFilters > 1)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
                evce_xWriteTruncBinCode(bs, (u32)(alfSliceParam.filterCoeffDeltaIdx[i]), alfSliceParam.numLumaFilters);  //filter_coeff_delta[i]
            }
        }

        char codetab_pred[3] = { 1, 0, 2 };
        const int iNumFixedFilterPerClass = 16;
        {
            evc_alfGolombEncode(bs, codetab_pred[alfSliceParam.fixedFilterPattern], 0);

            if (alfSliceParam.fixedFilterPattern == 2)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    evc_bsw_write1(bs, alfSliceParam.fixedFilterIdx[classIdx] > 0 ? 1 : 0); // "fixed_filter_flag"
                }
            }
            if (alfSliceParam.fixedFilterPattern > 0)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    if (alfSliceParam.fixedFilterIdx[classIdx] > 0)
                    {
                        evce_xWriteTruncBinCode(bs, alfSliceParam.fixedFilterIdx[classIdx] - 1, iNumFixedFilterPerClass);
                    }
                }
            }
        }

        evce_eco_alf_filter(bs, aps->alf_aps_param, FALSE);
    }

    if (alfChromaIdc)
    {
        {
            evce_eco_alf_filter(bs, aps->alf_aps_param, TRUE);
        }
    }

    return EVC_OK;
}

int evce_eco_alf_sh_param(EVC_BSW * bs, EVC_SH * sh)
{
    evc_AlfSliceParam alfSliceParam = sh->alf_sh_param;

    evc_bsw_write1(bs, alfSliceParam.isCtbAlfOn);
    return EVC_OK;
}
#endif

#if GRAB_STAT
void ence_stat_cu(int x, int y, int cuw, int cuh, int cup, void *ctx, void *core
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
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
#if M50761_CHROMA_NOT_SPLIT
    if (evc_check_only_inter(tree_cons))
    {
        evc_assert(pred_mode == MODE_INTER);
    }
    if (evc_check_only_intra(tree_cons))
    {
        evc_assert( (pred_mode == MODE_IBC) || (pred_mode == MODE_INTRA) );
    }
#endif

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
#if M50761_CHROMA_NOT_SPLIT
    if (evc_check_luma(tree_cons))
    {
#endif
    //evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma_enc", MCU_GET_CBFL(enc_ctx->map_scu[scup]));
    //evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma_nnz_sub", cu_data->nnz_sub[0][0][cup] > 0);
    evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma", cu_data->nnz[Y_C][cup] > 0);
    
#if M50761_CHROMA_NOT_SPLIT
    }
#endif
}
#endif
