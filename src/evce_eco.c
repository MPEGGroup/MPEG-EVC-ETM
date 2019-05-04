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

#pragma warning(disable:4018)

int evce_eco_cnkh(EVC_BSW * bs, EVC_CNKH * cnkh)
{
    evc_bsw_write(bs, cnkh->ver, 3);
    evc_bsw_write(bs, cnkh->ctype, 4);
    evc_bsw_write1(bs, cnkh->broken);
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

int evce_eco_sps(EVC_BSW * bs, EVC_SPS * sps)
{
    evc_bsw_write_ue(bs, (u32)sps->sps_seq_parameter_set_id);
    evc_bsw_write(bs, (u32)sps->profile_idc, 7);
    evc_bsw_write1(bs, sps->tier_flag);
    evc_bsw_write(bs, (u32)sps->level_idc, 8);
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
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_12_max_14_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_min_12_min_14_cb_size_minus1);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_11_max_tt_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_min_11_min_tt_cb_size_minus2);
    }
    evc_bsw_write1(bs, (u32)sps->sps_suco_flag);
    if (sps->sps_suco_flag)
    {
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_ctu_size_max_suco_cb_size);
        evc_bsw_write_ue(bs, (u32)sps->log2_diff_max_suco_min_suco_cb_size);
    }

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
    evc_bsw_write1(bs, sps->tool_cm_init);
#if HLS_M47668
    evc_bsw_write1(bs, sps->tool_rpl);
    evc_bsw_write1(bs, sps->tool_pocs);
    if (!sps->tool_rpl)
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
#else
    evc_bsw_write_ue(bs, (u32)sps->log2_max_pic_order_cnt_lsb_minus4);
#endif
    evc_bsw_write_ue(bs, (u32)sps->sps_max_dec_pic_buffering_minus1);
    evc_bsw_write1(bs, sps->picture_num_present_flag);
    if (sps->picture_num_present_flag)
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

    evc_bsw_write1(bs, sps->closed_gop);
    evc_bsw_write(bs, (sps->num_ref_pics_act - 1), 4);

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

    if (sps->long_term_ref_pics_flag)
    {
        evc_bsw_write_ue(bs, (u32)pps->additional_lt_poc_lsb_len);
    }

    if (sps->rpl_candidates_present_flag)
    {
        evc_bsw_write1(bs, pps->rpl1_idx_present_flag);
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

    evc_bsw_write1(bs, pps->arbitrary_tile_group_present_flag);

    while (!EVC_BSW_IS_BYTE_ALIGN(bs))
    {
        evc_bsw_write1(bs, 0);
    }

    return EVC_OK;
}

#if ALF_PARAMETER_APS
int evce_eco_aps(EVC_BSW * bs, EVC_APS * aps)
{
    evc_bsw_write(bs, aps->aps_id, 5); // signal APS ID
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

int evce_eco_tgh(EVC_BSW * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_TGH * tgh)
{
    int NumTilesInTileGroup = 0; //TBD according to the spec

#if HLS_M47668
    evc_bsw_write(bs, tgh->dtr, DTR_BIT_CNT);
    evc_bsw_write(bs, tgh->layer_id, 3);
#endif
    evc_bsw_write_ue(bs, tgh->tile_group_pic_parameter_set_id);
    evc_bsw_write1(bs, tgh->single_tile_in_tile_group_flag);
    evc_bsw_write(bs, tgh->first_tile_id, pps->tile_id_len_minus1 + 1);

    if (!tgh->single_tile_in_tile_group_flag)
    {
        if (pps->arbitrary_tile_group_present_flag)
        {
            evc_bsw_write1(bs, tgh->arbitrary_tile_group_flag);
        }
        if (!tgh->arbitrary_tile_group_flag)
        {
            evc_bsw_write(bs, tgh->last_tile_id, pps->tile_id_len_minus1 + 1);
        }
        else
        {
            evc_bsw_write_ue(bs, tgh->num_remaining_tiles_in_tile_group_minus1);
            for (int i = 0; i < NumTilesInTileGroup - 1; ++i)
            {
                evc_bsw_write_ue(bs, tgh->delta_tile_id_minus1[i]);
            }
        }
    }

    evc_bsw_write_ue(bs, tgh->tile_group_type);

#if ALF
    if (sps->tool_alf)
    {
        evc_bsw_write1(bs, tgh->alf_on);
#if ALF_PARAMETER_APS
        if (tgh->alf_on)
        {
            evc_bsw_write(bs, tgh->aps_signaled, 5); //encode tile group aps id
            evce_eco_alf_tgh_param(bs, tgh); // signaling ALF map
        }
#else
        if (tgh->alf_on)
        {
            evce_eco_alf_tgh_param(bs, tgh);
        }
#endif
    }
#endif

    // if (NalUnitType != IDR_NUT)  TBD: NALU types to be implemented
    {
#if HLS_M47668
        if (sps->tool_pocs)
        {
            evc_bsw_write(bs, tgh->poc, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
        }
#else
        evc_bsw_write(bs, tgh->poc, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
#endif
        if (sps->picture_num_present_flag)
        {
            evc_bsw_write1(bs, tgh->ref_pic_flag);
            evc_bsw_write(bs, tgh->picture_num, 8);
        }
    }
    // else
    {
        if (!sps->picture_num_present_flag)
        {
            //L0 candidates signaling
            if (sps->rpl_candidates_present_flag)
            {
                evc_bsw_write1(bs, tgh->ref_pic_list_sps_flag[0]);
            }
            if (tgh->ref_pic_list_sps_flag[0])
            {
                if (sps->rpls_l0_num)
                {
                    evc_bsw_write_ue(bs, tgh->rpl_l0_idx);
                }
            }
            else
            {
                evce_eco_rlp(bs, &tgh->rpl_l0);
            }

            //L1 candidates signaling
            if (sps->rpl_candidates_present_flag)
            {
                evc_bsw_write1(bs, tgh->ref_pic_list_sps_flag[1]);
            }
            if (tgh->ref_pic_list_sps_flag[1])
            {
                if (sps->rpls_l1_num)
                {
                    evc_bsw_write_ue(bs, tgh->rpl_l1_idx);
                }
            }
            else
            {
                evce_eco_rlp(bs, &tgh->rpl_l1);
            }
        }
    }

    if (!sps->picture_num_present_flag)
    {
        if (tgh->tile_group_type != TILE_GROUP_I)
        {
            evc_bsw_write1(bs, tgh->num_ref_idx_active_override_flag);
            if (tgh->num_ref_idx_active_override_flag)
            {
                evc_bsw_write_ue(bs, (u32)(tgh->rpl_l0).ref_pic_active_num - 1);
                if (tgh->tile_group_type == TILE_GROUP_B)
                {
                    evc_bsw_write_ue(bs, (u32)(tgh->rpl_l1).ref_pic_active_num - 1);
                }
            }
            if (sps->picture_num_present_flag)
            {
                evce_eco_ref_pic_list_mod(bs);
            }
        }
    }

    evc_bsw_write1(bs, tgh->deblocking_filter_on);
    evc_bsw_write(bs, tgh->qp, 6);
    evc_bsw_write_se(bs, (int)tgh->qp - (int)tgh->qp_u);
    evc_bsw_write_se(bs, (int)tgh->qp - (int)tgh->qp_v);

    if (!tgh->single_tile_in_tile_group_flag)
    {
        for (int i = 0; i < NumTilesInTileGroup - 1; ++i)
        {
            evc_bsw_write(bs, tgh->entry_point_offset_minus1[i], pps->tile_offset_lens_minus1 + 1);
        }
    }
#if !HLS_M47668
    evc_bsw_write(bs, tgh->dtr, DTR_BIT_CNT);
#endif
    evc_bsw_write1(bs, tgh->keyframe);
    evc_bsw_write1(bs, tgh->udata_exist);

#if AQS_SYNTAX
    evc_bsw_write_se(bs, tgh->es_map_norm_idx);
#endif

    if(tgh->tile_group_type != TILE_GROUP_I)
    {
        evc_bsw_write_se(bs, tgh->dptr);
    }

#if !HLS_M47668
    evc_bsw_write(bs, tgh->layer_id, 3);
#endif

    /* write MMCO */
    evc_bsw_write1(bs, tgh->mmco_on);

    if(tgh->mmco_on)
    {
        int cnt = tgh->mmco.cnt;
        while(cnt-- > 0)
        {
            evc_bsw_write_ue(bs, tgh->mmco.type[cnt]);
            evc_bsw_write_ue(bs, tgh->mmco.data[cnt]);
        }
        evc_bsw_write_ue(bs, MMCO_END);
    }

    /* write RMPNI */
    evc_bsw_write1(bs, tgh->rmpni_on);
    if(tgh->rmpni_on)
    {
        int i;
        for(i = 0; i < tgh->rmpni[REFP_0].cnt; i++)
        {
            if(tgh->rmpni[REFP_0].delta_poc[i] < 0)
            {
                evc_bsw_write_ue(bs, RMPNI_ADPN_NEG);
                evc_bsw_write_ue(bs, -(tgh->rmpni[REFP_0].delta_poc[i] + 1));
            }
            else
            {
                evc_bsw_write_ue(bs, RMPNI_ADPN_POS);
                evc_bsw_write_ue(bs, tgh->rmpni[REFP_0].delta_poc[i] - 1);
            }
        }
        evc_bsw_write_ue(bs, RMPNI_END);

        for(i = 0; i < tgh->rmpni[REFP_1].cnt; i++)
        {
            if(tgh->rmpni[REFP_1].delta_poc[i] < 0)
            {
                evc_bsw_write_ue(bs, RMPNI_ADPN_NEG);
                evc_bsw_write_ue(bs, -(tgh->rmpni[REFP_1].delta_poc[i] + 1));
            }
            else
            {
                evc_bsw_write_ue(bs, RMPNI_ADPN_POS);
                evc_bsw_write_ue(bs, tgh->rmpni[REFP_1].delta_poc[i] - 1);
            }
        }
        evc_bsw_write_ue(bs, RMPNI_END);
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
    if(ctx->param.use_pic_sign)
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
        for(i = 0; i < 16; i++)
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

void evce_sbac_reset(EVCE_SBAC *sbac, u8 tile_group_type, u8 tile_group_qp, int sps_cm_init_flag)
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

        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf, (s16*)init_cbf, NUM_QT_CBF_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->all_cbf, (s16*)init_all_cbf, NUM_QT_ROOT_CBF_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->pred_mode, (s16*)init_pred_mode, NUM_PRED_MODE_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->inter_dir, (s16*)init_inter_dir, NUM_INTER_DIR_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->intra_dir, (s16*)init_intra_dir, NUM_INTRA_DIR_CTX, tile_group_type, tile_group_qp);
#if CTX_REPRESENTATION_IMPROVEMENT
        evc_eco_sbac_ctx_initialize(sbac_ctx->run, (s16*)init_run, NUM_SBAC_CTX_RUN, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->last, (s16*)init_last, NUM_SBAC_CTX_LAST, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->level, (s16*)init_level, NUM_SBAC_CTX_LEVEL, tile_group_type, tile_group_qp);
#else
        for(i = 0; i < NUM_SBAC_CTX_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_flag, (s16*)init_mmvd_flag, NUM_SBAC_CTX_MMVD_FLAG, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_merge_idx, (s16*)init_mmvd_merge_idx, NUM_SBAC_CTX_MMVD_MERGE_IDX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_distance_idx, (s16*)init_mmvd_distance_idx, NUM_SBAC_CTX_MMVD_DIST_IDX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_direction_idx, (s16*)init_mmvd_direction_idx, NUM_SBAC_CTX_DIRECTION_IDX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mmvd_group_idx, (s16*)init_mmvd_group_idx, NUM_SBAC_CTX_MMVD_GRP_IDX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvp_idx, (s16*)init_mvp_idx, NUM_MVP_IDX_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvp_idx, (s16*)init_affine_mvp_idx, NUM_AFFINE_MVP_IDX_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvr_idx, (s16*)init_mvr_idx, NUM_MVR_IDX_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->bi_idx, (s16*)init_bi_idx, NUM_BI_IDX_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->mvd, (s16*)init_mvd, NUM_MV_RES_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->refi, (s16*)init_refi, NUM_REFI_CTX, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_flag, (s16*)init_btt_split_flag, NUM_SBAC_CTX_BTT_SPLIT_FLAG, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_dir, (s16*)init_btt_split_dir, NUM_SBAC_CTX_BTT_SPLIT_DIR, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->btt_split_type, (s16*)init_btt_split_type, NUM_SBAC_CTX_BTT_SPLIT_TYPE, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->suco_flag, (s16*)init_suco_flag, NUM_SBAC_CTX_SUCO_FLAG, tile_group_type, tile_group_qp);
#if ALF
        evc_eco_sbac_ctx_initialize(sbac_ctx->ctb_alf_flag, (s16*)init_ctb_alf_flag, NUM_SBAC_CTX_ALF_FLAG, tile_group_type, tile_group_qp);
#endif
#if AFFINE
#if CTX_NEV_AFFINE_FLAG
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_flag, (s16*)init_affine_flag, NUM_SBAC_CTX_AFFINE_FLAG, tile_group_type, tile_group_qp);
#else
        sbac_ctx->affine_flag[0] = PROB_INIT;
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mode, (s16*)init_affine_mode, NUM_SBAC_CTX_AFFINE_MODE, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mrg, (s16*)init_affine_mrg, AFF_MAX_CAND, tile_group_type, tile_group_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvd_flag, (s16*)init_affine_mvd_flag, NUM_SBAC_CTX_AFFINE_MVD_FLAG, tile_group_type, tile_group_qp);
#endif
        evc_eco_sbac_ctx_initialize(sbac_ctx->skip_flag, (s16*)init_skip_flag, NUM_SBAC_CTX_SKIP_FLAG, tile_group_type, tile_group_qp);
    }
    else // (sps_cm_init_flag == 0)
    {
        int i;

        for(i = 0; i < NUM_SBAC_CTX_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
        for(i = 0; i < NUM_SBAC_CTX_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;
        for(i = 0; i < NUM_QT_CBF_CTX; i++) sbac_ctx->cbf[i] = PROB_INIT;
        sbac_ctx->all_cbf[0] = PROB_INIT;
        for(i = 0; i < NUM_PRED_MODE_CTX; i++) sbac_ctx->pred_mode[i] = PROB_INIT;
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
#if AFFINE
#if CTX_NEV_AFFINE_FLAG
        for(i = 0; i < NUM_SBAC_CTX_AFFINE_FLAG; i++) sbac_ctx->affine_flag[i] = PROB_INIT;
#else
        sbac_ctx->affine_flag[0] = PROB_INIT;
#endif
        sbac_ctx->affine_mode[0] = PROB_INIT;
        for(i = 0; i < AFF_MAX_CAND; i++) sbac_ctx->affine_mrg[i] = PROB_INIT;
        sbac_ctx->affine_mvd_flag[0] = PROB_INIT;
        sbac_ctx->affine_mvd_flag[1] = PROB_INIT;
#endif
        for(i = 0; i < NUM_SBAC_CTX_SKIP_FLAG; i++) sbac_ctx->skip_flag[i] = PROB_INIT;
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
}

#if AFFINE
void evce_eco_affine_flag(EVC_BSW * bs, int flag
#if CTX_NEV_AFFINE_FLAG
                          , int ctx
#endif
)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
#if CTX_NEV_AFFINE_FLAG
    evce_sbac_encode_bin(flag, sbac, sbac->ctx.affine_flag + ctx, bs);
#else
    evce_sbac_encode_bin(flag, sbac, sbac->ctx.affine_flag, bs);
#endif
}

void evce_eco_affine_mode(EVC_BSW * bs, int flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(flag, sbac, sbac->ctx.affine_mode, bs);
}

int evce_eco_affine_mrg_idx(EVC_BSW * bs, s16 affine_mrg)
{
    EVCE_SBAC * sbac = GET_SBAC_ENC(bs);
    EVC_SBAC_CTX * sbac_ctx = &sbac->ctx;

    sbac_write_truncate_unary_sym(affine_mrg, AFF_MAX_CAND, AFF_MAX_CAND, sbac, sbac_ctx->affine_mrg, bs);

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
}

void evce_eco_inter_t_direct(EVC_BSW *bs, int t_direct_flag)
{
    EVCE_SBAC *sbac;
    sbac = GET_SBAC_ENC(bs);
    evce_sbac_encode_bin(t_direct_flag, sbac, sbac->ctx.inter_dir, bs);
}

void evce_eco_tile_group_end_flag(EVC_BSW * bs, int flag)
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

void evce_eco_xcoef(EVC_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type)
{
    evce_eco_run_length_cc(bs, coef, log2_w, log2_h, num_sig, (ch_type == Y_C ? 0 : 1));
}

int evce_eco_cbf(EVC_BSW * bs, int cbf_y, int cbf_u, int cbf_v, u8 pred_mode, int b_no_cbf, int is_sub,int sub_pos, int cbf_all, int run[N_C])
{
    EVCE_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;

    sbac = GET_SBAC_ENC(bs);
    sbac_ctx = &sbac->ctx;
       
    /* code allcbf */
    if(pred_mode != MODE_INTRA)
    {
        if (!cbf_all && sub_pos)
        {
            return EVC_OK;
        }            
        if(b_no_cbf == 1)
        {
            evc_assert(cbf_all != 0);
        }
        else if(sub_pos == 0)
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
            evce_sbac_encode_bin(cbf_u, sbac, sbac_ctx->cbf + 1, bs);
        }
        if (run[V_C])
        {
            evce_sbac_encode_bin(cbf_v, sbac, sbac_ctx->cbf + 2, bs);
        }
        if (run[Y_C])
        {
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

int evce_eco_coef(EVC_BSW * bs, s16 coef[N_C][MAX_CU_DIM], int log2_cuw, int log2_cuh, u8 pred_mode, int nnz_sub[N_C][MAX_SUB_TB_NUM], int b_no_cbf, int run_stats)
{    
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

    int cbf_all = 0;
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
            evce_eco_cbf(bs, !!nnz_sub[Y_C][(j << 1) | i], !!nnz_sub[U_C][(j << 1) | i], !!nnz_sub[V_C][(j << 1) | i], pred_mode, b_no_cbf, is_sub, j + i, cbf_all, run);
            
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

                    evce_eco_xcoef(bs, coef_temp[c], log2_w_sub - (!!c), log2_h_sub - (!!c), nnz_sub[c][(j << 1) | i], c);

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

    if(type == 1)
    {
        if(mvp_idx >= (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM))
        {
            mvp_idx = mvp_idx - (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            dev0 = mvp_idx / (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
            mvp_idx = mvp_idx - dev0 * (MMVD_MAX_REFINE_NUM*MMVD_BASE_MV_NUM);
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

    var0 = mvp_idx / MMVD_MAX_REFINE_NUM;
    var1 = (mvp_idx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
    var2 = mvp_idx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;

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

    return EVC_OK;
}

void evce_eco_inter_dir(EVC_BSW *bs, s8 refi[REFP_NUM])
{
    EVCE_SBAC *sbac;

    sbac = GET_SBAC_ENC(bs);
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("inter dir ");
    if(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1])) /* PRED_BI */
    {
        evce_sbac_encode_bin(1, sbac, sbac->ctx.inter_dir + 1, bs);
        EVC_TRACE_INT(PRED_BI);
    }
    else
    {
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
    core->mmvd_flag = 0;
#if AFFINE
    core->affine_flag = cu_data->affine_flag[cup];
#endif
    core->nnz[Y_C] = core->nnz[U_C] = core->nnz[V_C] = 0;

    if(cu_data->pred_mode[cup] == MODE_INTRA)
    {
        core->avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->log2_cuw, core->log2_cuh, ctx->map_scu);
    }
    else
    {
        if((cu_data->pred_mode[cup] == MODE_SKIP) || (cu_data->pred_mode[cup] == MODE_SKIP_MMVD))
        {
            core->skip_flag = 1;
        }

        if(cu_data->pred_mode[cup] == MODE_SKIP_MMVD)
        {
            core->mmvd_flag = 1;
        }

        core->avail_cu = evc_get_avail_inter(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->cuw, core->cuh, ctx->map_scu);
    }

    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu);
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

    for(j = 0; j < cuh; j++)
    {
        for(i = 0; i < cuw; i++)
        {
            coef_dst[Y_C][didx++] = coef_src[Y_C][sidx + i];
        }
        sidx += ctx->max_cuwh;
    }

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

    evc_check_split_mode(split_allow, CONV_LOG2(cuw), CONV_LOG2(cuh), 0, 0, 0, c->log2_max_cuwh, c->layer_id
                         , parent_split, same_layer_split, node_idx, parent_split_allow, qt_depth, btt_depth
                         , x, y, c->w, c->h
                         , NULL, c->sps.sps_btt_flag);

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

int evce_eco_unit(EVCE_CTX * ctx, EVCE_CORE * core, int x, int y, int cup, int cuw, int cuh)
{
    s16(*coef)[MAX_CU_DIM] = core->ctmp;
    EVC_BSW *bs;
    u32 *map_scu;
    int tile_group_type, refi0, refi1;
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
    tile_group_type = ctx->tile_group_type;
    bs = &ctx->bs;

    cu_init(ctx, core, x, y, cup, cuw, cuh);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ptr: ");
    EVC_TRACE_INT(ctx->ptr);
    EVC_TRACE_STR("x pos ");
    EVC_TRACE_INT(core->x_pel + ((cup % (ctx->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
    EVC_TRACE_STR("y pos ");
    EVC_TRACE_INT(core->y_pel + ((cup / (ctx->max_cuwh >> MIN_CU_LOG2)) << MIN_CU_LOG2));
    EVC_TRACE_STR("width ");
    EVC_TRACE_INT(cuw);
    EVC_TRACE_STR("height ");
    EVC_TRACE_INT(cuh);
    EVC_TRACE_STR("\n");

    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->tgh.tile_group_type, ctx->sps.tool_cm_init);
    
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
    if(tile_group_type != TILE_GROUP_I && (ctx->sps.tool_amis == 0 || !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2)))
    {
        evce_eco_skip_flag(bs, core->skip_flag, ctx->ctx_flags[CNID_SKIP_FLAG]);

        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("skip flag ");
        EVC_TRACE_INT(core->skip_flag);
        EVC_TRACE_STR("ctx ");
        EVC_TRACE_INT(ctx->ctx_flags[CNID_SKIP_FLAG]);
        EVC_TRACE_STR("\n");

        if(core->skip_flag)
        {
            if(ctx->sps.tool_mmvd)
            {
                evce_eco_mmvd_flag(bs, core->mmvd_flag);
                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("mmvd_flag ");
                EVC_TRACE_INT(core->mmvd_flag);
                EVC_TRACE_STR("\n");
            }

            if(core->mmvd_flag)
            {
                evce_eco_mmvd_info(bs, cu_data->mmvd_idx[cup], !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));
            }
            else
            {
#if AFFINE
                if(cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine)
                {
                    evce_eco_affine_flag(bs, core->affine_flag != 0
#if CTX_NEV_AFFINE_FLAG
                                          , ctx->ctx_flags[CNID_AFFN_FLAG]
#endif
                                          ); /* skip affine_flag */
                }

                if(core->affine_flag)
                {
                    evce_eco_affine_mrg_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                    EVC_TRACE_COUNTER;
                    EVC_TRACE_STR("merge affine idx ");
                    EVC_TRACE_INT(cu_data->mvp_idx[cup][REFP_0]);
                    EVC_TRACE_STR("\n");

                    EVC_TRACE_COUNTER;
                    EVC_TRACE_STR("merge affine flag ");
                    EVC_TRACE_INT(core->affine_flag);
                    EVC_TRACE_STR("\n");
                }
                else
#endif
                {
                    evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_0], ctx->sps.tool_amis);

                    if(ctx->sps.tool_amis == 0 && tile_group_type == TILE_GROUP_B)
                    {
                        evce_eco_mvp_idx(bs, cu_data->mvp_idx[cup][REFP_1], ctx->sps.tool_amis);
                    }
                }
            }
        }
        else
        {
            evce_eco_pred_mode(bs, cu_data->pred_mode[cup], ctx->ctx_flags[CNID_PRED_MODE]);

            if(cu_data->pred_mode[cup] != MODE_INTRA)
            {
                if(ctx->sps.tool_amvr)
                {
                    evce_eco_mvr_idx(bs, cu_data->mvr_idx[cup]);
                }

                if(ctx->sps.tool_mmvd)
                {
                    if(tile_group_type == TILE_GROUP_P)
                    {
                        if(cu_data->mvr_idx[cup] == 0)
                        {
                            evce_eco_mmvd_flag(bs, cu_data->pred_mode[cup] == MODE_DIR_MMVD);
                        }

                        if((cu_data->pred_mode[cup] == MODE_DIR_MMVD))
                        {

                            evce_eco_mmvd_info(bs, cu_data->mmvd_idx[cup], !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));
                        }
                    }
                }

                if(tile_group_type == TILE_GROUP_B)
                {
                    if(ctx->sps.tool_mmvd)
                    {
                        if(cu_data->mvr_idx[cup] == 0)
                        {
                            evce_eco_inter_t_direct(bs, cu_data->pred_mode[cup] == MODE_DIR || cu_data->pred_mode[cup] == MODE_DIR_MMVD);
                        }

#if ENC_DEC_TRACE
                        if(cu_data->pred_mode[cup] == MODE_DIR)
                        {
                            EVC_TRACE_COUNTER;
                            EVC_TRACE_STR("inter dir ");
                            EVC_TRACE_INT(PRED_DIR);
                            EVC_TRACE_STR("\n");
                        }
                        else if(cu_data->pred_mode[cup] == MODE_DIR_MMVD)
                        {
                            EVC_TRACE_COUNTER;
                            EVC_TRACE_STR("inter dir ");
                            EVC_TRACE_INT(PRED_DIR_MMVD);
                            EVC_TRACE_STR("\n");
                        }
#endif
                    }
                    else
                    {
                        if(cu_data->mvr_idx[cup] == 0)
                        {
                            evce_eco_inter_t_direct(bs, cu_data->pred_mode[cup] == MODE_DIR);
                        }
#if ENC_DEC_TRACE 
                        if(cu_data->pred_mode[cup] == MODE_DIR)
                        {
                            EVC_TRACE_COUNTER;
                            EVC_TRACE_STR("inter dir ");
                            EVC_TRACE_INT(PRED_DIR);
                            EVC_TRACE_STR("\n");
                        }
#endif
                    }

                    if(ctx->sps.tool_mmvd)
                    {
                        if((cu_data->pred_mode[cup] == MODE_DIR) || (cu_data->pred_mode[cup] == MODE_DIR_MMVD))
                        {
                            evce_eco_mmvd_flag(bs, cu_data->pred_mode[cup] == MODE_DIR_MMVD);
                        }

                        if((cu_data->pred_mode[cup] == MODE_DIR_MMVD))
                        {
                            evce_eco_mmvd_info(bs, cu_data->mmvd_idx[cup], !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));
                        }
                    }

#if AFFINE
                    if(cu_data->pred_mode[cup] == MODE_DIR && cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine)
                    {
                        evce_eco_affine_flag(bs, core->affine_flag != 0
#if CTX_NEV_AFFINE_FLAG
                                             , ctx->ctx_flags[CNID_AFFN_FLAG]
#endif
                        ); /* direct affine_flag */
                        if(core->affine_flag)
                        {
                            evce_eco_affine_mrg_idx(bs, cu_data->mvp_idx[cup][REFP_0]);
                            EVC_TRACE_COUNTER;
                            EVC_TRACE_STR("merge affine idx ");
                            EVC_TRACE_INT(cu_data->mvp_idx[cup][REFP_0]);
                            EVC_TRACE_STR("\n");
                            EVC_TRACE_COUNTER;
                            EVC_TRACE_STR("merge affine flag ");
                            EVC_TRACE_INT(core->affine_flag);
                            EVC_TRACE_STR("\n");
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
                        EVC_TRACE_COUNTER;
                        EVC_TRACE_STR("merge mvp index ");
                        EVC_TRACE_INT(cu_data->mvp_idx[cup][REFP_0]);
                        EVC_TRACE_STR("\n");
                    }
#endif
                }

                if(((cu_data->pred_mode[cup] % ORG_PRED_NUM) != MODE_DIR) && ((cu_data->pred_mode[cup] % ORG_PRED_NUM) != MODE_DIR_MMVD))
                {
                    evce_eco_inter_dir(bs, cu_data->refi[cup]);

#if AFFINE // affine inter mode
                    if(cuw >= 16 && cuh >= 16 && cu_data->mvr_idx[cup] == 0 && ctx->sps.tool_affine)
                    {
                        evce_eco_affine_flag(bs, core->affine_flag != 0
#if CTX_NEV_AFFINE_FLAG
                                             , ctx->ctx_flags[CNID_AFFN_FLAG]
#endif
                        ); /* inter affine_flag */
                    }

                    if(core->affine_flag)
                    {
                        evce_eco_affine_mode(bs, core->affine_flag - 1); /* inter affine_mode */
                    }

                    if(ctx->sps.tool_affine)
                    {
                        EVC_TRACE_COUNTER;
                        EVC_TRACE_STR("inter affine flag ");
                        EVC_TRACE_INT(core->affine_flag);
                        EVC_TRACE_STR("\n");
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

                        if(IS_INTER_TILE_GROUP(tile_group_type) && REFI_IS_VALID(refi0))
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

                        if(tile_group_type == TILE_GROUP_B && REFI_IS_VALID(refi1))
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
                        if(IS_INTER_TILE_GROUP(tile_group_type) && REFI_IS_VALID(refi0))
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

                        if(tile_group_type == TILE_GROUP_B && REFI_IS_VALID(refi1))
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

    if(cu_data->pred_mode[cup] == MODE_INTRA)
    {
        evc_assert(cu_data->ipm[0][cup] != IPD_INVALID);
        evc_assert(cu_data->ipm[1][cup] != IPD_INVALID);

        if(ctx->sps.tool_eipd)
        {
            evc_get_mpm(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                        core->mpm, core->avail_lr, core->mpm_ext, core->pims);
            evce_eco_intra_dir(bs, cu_data->ipm[0][cup], core->mpm, core->mpm_ext, core->pims);
            evce_eco_intra_dir_c(bs, cu_data->ipm[1][cup], cu_data->ipm[0][cup]);
        }
        else
        {
            evc_get_mpm_b(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                          &core->mpm_b_list, core->avail_lr, core->mpm_ext, core->pims);
            evce_eco_intra_dir_b(bs, cu_data->ipm[0][cup], core->mpm_b_list, core->mpm_ext, core->pims);
        }
    }

    if((core->skip_flag == 0) && (core->mmvd_flag == 0))
    {
        int b_no_cbf = 0;
#if AFFINE
        b_no_cbf |= cu_data->affine_flag[cup] && cu_data->pred_mode[cup] == MODE_DIR;
#endif
        b_no_cbf |= cu_data->pred_mode[cup] == MODE_DIR_MMVD;
#if MERGE
        b_no_cbf |= cu_data->pred_mode[cup] == MODE_DIR;
#endif
        if(ctx->sps.tool_amis == 0)
            b_no_cbf = 0;
        evce_eco_coef(bs, coef, core->log2_cuw, core->log2_cuh, cu_data->pred_mode[cup], core->nnz_sub, b_no_cbf, RUN_L | RUN_CB | RUN_CR);
    }

    map_scu = ctx->map_scu + core->scup;
    w = (core->cuw >> MIN_CU_LOG2);
    h = (core->cuh >> MIN_CU_LOG2);

#if AFFINE
    map_affine = ctx->map_affine + core->scup;
#endif
    map_cu_mode = ctx->map_cu_mode + core->scup;
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

void evce_eco_alf_filter(EVC_BSW * bs, evc_AlfTileGroupParam asp, const BOOL isChroma)
{
    const evc_AlfTileGroupParam alfTileGroupParam = asp;
    if (!isChroma)
    {
        evc_bsw_write1(bs, alfTileGroupParam.coeffDeltaFlag); // "alf_coefficients_delta_flag"
        if (!alfTileGroupParam.coeffDeltaFlag)
        {
            if (alfTileGroupParam.numLumaFilters > 1)
            {
                evc_bsw_write1(bs, alfTileGroupParam.coeffDeltaPredModeFlag); // "coeff_delta_pred_mode_flag"
            }
        }
    }

    // this logic need to be moved to ALF files
    evc_AlfFilterShape alfShape;
    init_AlfFilterShape( &alfShape, isChroma ? 5 : ( alfTileGroupParam.lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );

    int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
    memset(bitsCoeffScan, 0, m_MAX_SCAN_VAL*m_MAX_EXP_GOLOMB * sizeof(int));

    const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;
    const short* coeff = isChroma ? alfTileGroupParam.chromaCoeff : alfTileGroupParam.lumaCoeff;
    const int numFilters = isChroma ? 1 : alfTileGroupParam.numLumaFilters;

    // vlc for all
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (isChroma || !alfTileGroupParam.coeffDeltaFlag || alfTileGroupParam.filterCoeffFlag[ind])
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
        if (alfTileGroupParam.coeffDeltaFlag)
        {
            for (int ind = 0; ind < numFilters; ++ind)
            {
                evc_bsw_write1(bs, alfTileGroupParam.filterCoeffFlag[ind]);  // WRITE_FLAG(alfTileGroupParam.filterCoeffFlag[ind], "filter_coefficient_flag[i]");
            }
        }
    }

    // Filter coefficients
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (!isChroma && !alfTileGroupParam.filterCoeffFlag[ind] && alfTileGroupParam.coeffDeltaFlag)
        {
            continue;
        }

        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
            evc_alfGolombEncode(bs, coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], kMinTab[alfShape.golombIdx[i]]);  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
        }
    }
}
#if ALF_PARAMETER_APS
int evce_eco_alf_aps_param(EVC_BSW * bs, EVC_APS * aps)
{
    evc_AlfTileGroupParam alfTileGroupParam = aps->alf_aps_param;

    evc_bsw_write1(bs, alfTileGroupParam.enabledFlag[0]); //"alf_tile_group_enable_flag"
    if (!alfTileGroupParam.enabledFlag[0])
    {
        return 0;
    }

    const int alfChromaIdc = alfTileGroupParam.enabledFlag[1] * 2 + alfTileGroupParam.enabledFlag[2];
    evce_truncatedUnaryEqProb(bs, alfChromaIdc, 3);
    {
        evce_xWriteTruncBinCode(bs, alfTileGroupParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES);
        evc_bsw_write1(bs, !alfTileGroupParam.lumaFilterType); //  "filter_type_flag"

        if (alfTileGroupParam.numLumaFilters > 1)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
                evce_xWriteTruncBinCode(bs, (u32)(alfTileGroupParam.filterCoeffDeltaIdx[i]), alfTileGroupParam.numLumaFilters);  //filter_coeff_delta[i]
            }
        }

        char codetab_pred[3] = { 1, 0, 2 };
        const int iNumFixedFilterPerClass = 16;
        if (iNumFixedFilterPerClass > 0)
        {
            evc_alfGolombEncode(bs, codetab_pred[alfTileGroupParam.fixedFilterPattern], 0);

            if (alfTileGroupParam.fixedFilterPattern == 2)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    evc_bsw_write1(bs, alfTileGroupParam.fixedFilterIdx[classIdx] > 0 ? 1 : 0); // "fixed_filter_flag"
                }
            }

            if (alfTileGroupParam.fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    if (alfTileGroupParam.fixedFilterIdx[classIdx] > 0)
                    {
                        evce_xWriteTruncBinCode(bs, alfTileGroupParam.fixedFilterIdx[classIdx] - 1, iNumFixedFilterPerClass);
                    }
                }
            }
        }

        evce_eco_alf_filter(bs, aps->alf_aps_param, FALSE);
    }

    if (alfChromaIdc)
    {
        evc_bsw_write1(bs, alfTileGroupParam.chromaCtbPresentFlag);
        if (!(alfTileGroupParam.temporalAlfFlag))
        {
            evce_eco_alf_filter(bs, aps->alf_aps_param, TRUE);
        }
    }

    return EVC_OK;
}

int evce_eco_alf_tgh_param(EVC_BSW * bs, EVC_TGH * tgh)
{
    evc_AlfTileGroupParam alfTileGroupParam = tgh->alf_tgh_param;

    evc_bsw_write1(bs, alfTileGroupParam.isCtbAlfOn);
    if (alfTileGroupParam.isCtbAlfOn)
    {
        for (int i = 0; i < tgh->num_ctb; i++)
            evc_bsw_write1(bs, (int)(alfTileGroupParam.alfCtuEnableFlag[0][i]));
    }

    return EVC_OK;
}
#else
int evce_eco_alf_tgh_param(EVC_BSW * bs, EVC_TGH * tgh)
{
    evc_AlfTileGroupParam alfTileGroupParam = tgh->alf_tgh_param;
    evc_bsw_write1(bs, alfTileGroupParam.enabledFlag[0]); //"alf_tile_group_enable_flag"
    if (!alfTileGroupParam.enabledFlag[0])
    {
        return 0;
    }

    const int alfChromaIdc = alfTileGroupParam.enabledFlag[1] * 2 + alfTileGroupParam.enabledFlag[2];
    evce_truncatedUnaryEqProb(bs, alfChromaIdc, 3);

    {
        evc_bsw_write1( bs, alfTileGroupParam.temporalAlfFlag ); // "alf_temporal_enable_flag"
        if( alfTileGroupParam.temporalAlfFlag )
        {
            evc_bsw_write_ue( bs, alfTileGroupParam.prevIdx );   // "alf_temporal_index"
        }
        else
        {
            evc_bsw_write1( bs, alfTileGroupParam.resetALFBufferFlag );
            evc_bsw_write1( bs, alfTileGroupParam.store2ALFBufferFlag );
        }
    }

    if (!alfTileGroupParam.temporalAlfFlag)
    {
        evce_xWriteTruncBinCode(bs, alfTileGroupParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES);
        evc_bsw_write1(bs, !alfTileGroupParam.lumaFilterType); //  "filter_type_flag"

        if (alfTileGroupParam.numLumaFilters > 1)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
                evce_xWriteTruncBinCode(bs, (u32)(alfTileGroupParam.filterCoeffDeltaIdx[i]), alfTileGroupParam.numLumaFilters);  //filter_coeff_delta[i]
            }
        }

        char codetab_pred[3] = { 1, 0, 2 };
        const int iNumFixedFilterPerClass = 16;
        if (iNumFixedFilterPerClass > 0)
        {
            evc_alfGolombEncode(bs, codetab_pred[alfTileGroupParam.fixedFilterPattern], 0);

            if (alfTileGroupParam.fixedFilterPattern == 2)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    evc_bsw_write1(bs, alfTileGroupParam.fixedFilterIdx[classIdx] > 0 ? 1 : 0); // "fixed_filter_flag"
                }
            }

            if (alfTileGroupParam.fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    if (alfTileGroupParam.fixedFilterIdx[classIdx] > 0)
                    {
                        evce_xWriteTruncBinCode(bs, alfTileGroupParam.fixedFilterIdx[classIdx] - 1, iNumFixedFilterPerClass);
                    }
                }
            }
        }

        evce_eco_alf_filter(bs, tgh->alf_tgh_param, FALSE);
    }

    if (alfChromaIdc) 
    {
        evc_bsw_write1(bs, alfTileGroupParam.chromaCtbPresentFlag);
        if (!(alfTileGroupParam.temporalAlfFlag))
        {
            evce_eco_alf_filter(bs, tgh->alf_tgh_param, TRUE);
        }
    }

    evc_bsw_write1(bs, alfTileGroupParam.isCtbAlfOn);
    if( alfTileGroupParam.isCtbAlfOn )
    {
        for(int i = 0; i < tgh->num_ctb; i++)
            evc_bsw_write1(bs, (int)(alfTileGroupParam.alfCtuEnableFlag[0][i]));
    }

    return EVC_OK;
}
#endif
#endif