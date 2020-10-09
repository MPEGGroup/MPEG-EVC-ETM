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

#include "evcd_def.h"
#include "evc_tbl.h"

#include <math.h>
#include "wrapper.h"
#include "enc_alf_wrapper.h"
#pragma warning(disable:4018)

#if GRAB_STAT
#include "evc_debug.h"
#endif

u32 evcd_sbac_decode_bin(EVC_BSR * bs, EVCD_SBAC * sbac, SBAC_CTX_MODEL * model)
{
    u32 bin, lps, t0;
    u16 mps, state;

    state = (*model) >> 1;
    mps = (*model) & 1;

    lps = (state * (sbac->range)) >> 9;
    lps = lps < 437 ? 437 : lps;    

    bin = mps;

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

    if(sbac->value >= sbac->range)
    {
        bin = 1 - mps;
        sbac->value -= sbac->range;
        sbac->range = lps;

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
        bin = mps;
        state = state - ((state + 16) >> 5);
        *model = (state << 1) + mps;
    }

    while(sbac->range < 8192)
    {
        sbac->range <<= 1;
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &t0, 0);
#else
        evc_bsr_read1(bs, &t0);
#endif
        sbac->value = ((sbac->value << 1) | t0) & 0xFFFF;
    }

    return bin;
}

static u32 sbac_decode_bin_ep(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    u32 bin, t0;

    sbac->range >>= 1;

    if(sbac->value >= sbac->range)
    {
        bin = 1;
        sbac->value -= sbac->range;
    }
    else
    {        
        bin = 0;
    }

    sbac->range <<= 1;
#if TRACE_HLS
    evc_bsr_read1_trace(bs, &t0, 0);
#else
    evc_bsr_read1(bs, &t0);
#endif
    sbac->value = ((sbac->value << 1) | t0) & 0xFFFF;

    return bin;
}

u32 evcd_sbac_decode_bin_trm(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    u32 bin, t0;

    sbac->range--;

    if(sbac->value >= sbac->range)
    {
        bin = 1;

        /*
        sbac->value -= sbac->range;
        sbac->range = 1;
        */

        while(!EVC_BSR_IS_BYTE_ALIGN(bs))
        {
#if TRACE_HLS
            evc_bsr_read1_trace(bs, &t0, 0);
#else
            evc_bsr_read1(bs, &t0);
#endif
            evc_assert_rv(t0 == 0, EVC_ERR_MALFORMED_BITSTREAM);
        }
    }
    else
    {
        bin = 0;
        while(sbac->range < 8192)
        {
            sbac->range <<= 1;
#if TRACE_HLS
            evc_bsr_read1_trace(bs, &t0, 0);
#else
            evc_bsr_read1(bs, &t0);
#endif
            sbac->value = ((sbac->value << 1) | t0) & 0xFFFF;
        }
    }

    return bin;
}

static u32 sbac_read_unary_sym_ep(EVC_BSR * bs, EVCD_SBAC * sbac, u32 max_val)
{
    u32 t32u;
    u32 symbol;
    int counter = 0;

    symbol = sbac_decode_bin_ep(bs, sbac); counter++;

    if(symbol == 0)
    {
        return symbol;
    }

    symbol = 0;
    do
    {
        if(counter == max_val) t32u = 0;
        else t32u = sbac_decode_bin_ep(bs, sbac);
        counter++;
        symbol++;
    } while(t32u);

    return symbol;
}

static u32 sbac_decode_bins_ep(EVC_BSR *bs, EVCD_SBAC *sbac, int num_bin)
{
    int bin = 0;
    u32 value = 0;

    for(bin = num_bin - 1; bin >= 0; bin--)
    {
        if(sbac_decode_bin_ep(bs, sbac))
        {
            value += (1 << bin);
        }
    }

    return value;
}

static u32 sbac_read_unary_sym(EVC_BSR * bs, EVCD_SBAC * sbac, SBAC_CTX_MODEL * model, u32 num_ctx)
{
    u32 ctx_idx = 0;
    u32 t32u;
    u32 symbol;

    symbol = evcd_sbac_decode_bin(bs, sbac, model);

    if(symbol == 0)
    {
        return symbol;
    }

    symbol = 0;
    do
    {
        if(ctx_idx < num_ctx - 1)
        {
            ctx_idx++;
        }
        t32u = evcd_sbac_decode_bin(bs, sbac, &model[ctx_idx]);
        symbol++;
    } while(t32u);

    return symbol;
}

static u32 sbac_read_truncate_unary_sym(EVC_BSR * bs, EVCD_SBAC * sbac, SBAC_CTX_MODEL * model, u32 num_ctx, u32 max_num)
{
    u32 ctx_idx = 0;
    u32 t32u;
    u32 symbol;

    if(max_num > 1)
    {
        for(; ctx_idx < max_num - 1; ++ctx_idx)
        {
            symbol = evcd_sbac_decode_bin(bs, sbac, model + (ctx_idx > num_ctx - 1 ? num_ctx - 1 : ctx_idx));
            if(symbol == 0)
            {
                break;
            }
        }
    }
    t32u = ctx_idx;

    return t32u;
}

static int eco_ats_inter_info(EVC_BSR * bs, EVCD_SBAC * sbac, int log2_cuw, int log2_cuh, u8* ats_inter_info, u8 ats_inter_avail)
{
    u8 mode_vert = (ats_inter_avail >> 0) & 0x1;
    u8 mode_hori = (ats_inter_avail >> 1) & 0x1;
    u8 mode_vert_quad = (ats_inter_avail >> 2) & 0x1;
    u8 mode_hori_quad = (ats_inter_avail >> 3) & 0x1;
    u8 num_ats_inter_mode_avail = mode_vert + mode_hori + mode_vert_quad + mode_hori_quad;

    if (num_ats_inter_mode_avail == 0)
    {
        *ats_inter_info = 0;
        return EVC_OK;
    }
    else
    {
        u8 ats_inter_flag = 0;
        u8 ats_inter_hor = 0;
        u8 ats_inter_quad = 0;
        u8 ats_inter_pos = 0;
        int size = 1 << (log2_cuw + log2_cuh);
        u8 ctx_ats_inter = sbac->ctx.sps_cm_init_flag == 1 ? ((log2_cuw + log2_cuh >= 8) ? 0 : 1) : 0;
        u8 ctx_ats_inter_hor = sbac->ctx.sps_cm_init_flag == 1 ? ((log2_cuw == log2_cuh) ? 0 : (log2_cuw < log2_cuh ? 1 : 2)) : 0;

        EVC_SBAC_CTX * sbac_ctx;
        sbac_ctx = &sbac->ctx;

        ats_inter_flag = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_cu_inter_flag + ctx_ats_inter);
        EVC_TRACE_STR("ats_inter_flag ");
        EVC_TRACE_INT(ats_inter_flag);
        EVC_TRACE_STR("\n");

        if (ats_inter_flag)
        {
            if ((mode_vert_quad || mode_hori_quad) && (mode_vert || mode_hori))
            {
                ats_inter_quad = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_cu_inter_quad_flag);
                EVC_TRACE_STR("ats_inter_quad ");
                EVC_TRACE_INT(ats_inter_quad);
                EVC_TRACE_STR("\n");
            }
            else
            {
                ats_inter_quad = 0;
            }

            if ((ats_inter_quad && mode_vert_quad && mode_hori_quad) || (!ats_inter_quad && mode_vert && mode_hori))
            {
                ats_inter_hor = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_cu_inter_hor_flag + ctx_ats_inter_hor);
                EVC_TRACE_STR("ats_inter_hor ");
                EVC_TRACE_INT(ats_inter_hor);
                EVC_TRACE_STR("\n");
            }
            else
            {
                ats_inter_hor = (ats_inter_quad && mode_hori_quad) || (!ats_inter_quad && mode_hori);
            }

            ats_inter_pos = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_cu_inter_pos_flag);
            EVC_TRACE_STR("ats_inter_pos ");
            EVC_TRACE_INT(ats_inter_pos);
            EVC_TRACE_STR("\n");
        }
        *ats_inter_info = get_ats_inter_info((ats_inter_quad ? 2 : 0) + (ats_inter_hor ? 1 : 0) + ats_inter_flag, ats_inter_pos);

        return EVC_OK;
    }
}

static int eco_cbf(EVC_BSR * bs, EVCD_SBAC * sbac, u8 pred_mode, u8 cbf[N_C], int b_no_cbf, int is_sub, int sub_pos, int *cbf_all, TREE_CONS tree_cons
#if BD_CF_EXT
                   , int chroma_format_idc
#endif
)
{
    EVC_SBAC_CTX * sbac_ctx;
    sbac_ctx = &sbac->ctx;

    /* decode allcbf */
    if (pred_mode != MODE_INTRA && tree_cons.tree_type == TREE_LC)
    {
        if (b_no_cbf == 0 && sub_pos == 0)
        {
            if (evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_all) == 0)
            {
                *cbf_all = 0;
                cbf[Y_C] = cbf[U_C] = cbf[V_C] = 0;

                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("all_cbf ");
                EVC_TRACE_INT(0);
                EVC_TRACE_STR("\n");

                return EVC_OK;
            }
            else
            {
                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("all_cbf ");
                EVC_TRACE_INT(1);
                EVC_TRACE_STR("\n");
            }
        }
#if  BD_CF_EXT
        if(chroma_format_idc != 0)
        {
#endif
            cbf[U_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_cb);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf U ");
            EVC_TRACE_INT(cbf[U_C]);
            EVC_TRACE_STR("\n");

            cbf[V_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_cr);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf V ");
            EVC_TRACE_INT(cbf[V_C]);
            EVC_TRACE_STR("\n");
#if  BD_CF_EXT
        }
        else
        {
            cbf[U_C] = 0;
            cbf[V_C] = 0;
        }
#endif

        if (cbf[U_C] + cbf[V_C] == 0 && !is_sub)
        {
            cbf[Y_C] = 1;
        }
        else
        {
            cbf[Y_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_luma);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf Y ");
            EVC_TRACE_INT(cbf[Y_C]);
            EVC_TRACE_STR("\n");
        }
    }
    else
    {
#if BD_CF_EXT
        if(evc_check_chroma(tree_cons) && chroma_format_idc != 0)
#else
        if (evc_check_chroma(tree_cons))
#endif
        {
            cbf[U_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_cb);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf U ");
            EVC_TRACE_INT(cbf[U_C]);
            EVC_TRACE_STR("\n");

            cbf[V_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_cr);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf V ");
            EVC_TRACE_INT(cbf[V_C]);
            EVC_TRACE_STR("\n");
        }
        else
        {
            cbf[U_C] = cbf[V_C] = 0;
        }

        if (evc_check_luma(tree_cons))
        {
            cbf[Y_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf_luma);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("cbf Y ");
            EVC_TRACE_INT(cbf[Y_C]);
            EVC_TRACE_STR("\n");
        }
        else
        {
            cbf[Y_C] = 0;
        }
    }

    return EVC_OK;
}

static int evcd_eco_run_length_cc(EVC_BSR *bs, EVCD_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type)
{
    EVC_SBAC_CTX *sbac_ctx;
    int            sign, level, prev_level, run, last_flag;
    int            t0, scan_pos_offset, num_coeff, i, coef_cnt = 0;
    const u16     *scanp;
    int            ctx_last = 0;

    sbac_ctx = &sbac->ctx;
    scanp = evc_scan_tbl[COEF_SCAN_ZIGZAG][log2_w - 1][log2_h - 1];
    num_coeff = 1 << (log2_w + log2_h);
    scan_pos_offset = 0;
    prev_level = 6;

    do
    {
        t0 = sbac->ctx.sps_cm_init_flag == 1 ? ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12) : (ch_type == Y_C ? 0 : 2);

        /* Run parsing */
        run = sbac_read_unary_sym(bs, sbac, sbac_ctx->run + t0, 2);
        for(i = scan_pos_offset; i < scan_pos_offset + run; i++)
        {
            coef[scanp[i]] = 0;
        }
        scan_pos_offset += run;

        /* Level parsing */
        level = sbac_read_unary_sym(bs, sbac, sbac_ctx->level + t0, 2);
        level++;
        prev_level = level;

        /* Sign parsing */
        sign = sbac_decode_bin_ep(bs, sbac);
        coef[scanp[scan_pos_offset]] = sign ? -(s16)level : (s16)level;

        coef_cnt++;

        if(scan_pos_offset >= num_coeff - 1)
        {
            break;
        }
        scan_pos_offset++;

        /* Last flag parsing */
        ctx_last = (ch_type == Y_C) ? 0 : 1;
        last_flag = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->last + ctx_last);
    } while(!last_flag);

#if ENC_DEC_TRACE
    EVC_TRACE_STR("coef luma ");
    for(scan_pos_offset = 0; scan_pos_offset < num_coeff; scan_pos_offset++)
    {
        EVC_TRACE_INT(coef[scan_pos_offset]);
    }
    EVC_TRACE_STR("\n");
#endif

    return EVC_OK;
}
 
static int evcd_eco_ats_intra_cu(EVC_BSR* bs, EVCD_SBAC* sbac)
{
    u32 t0;
    t0 = sbac_decode_bin_ep(bs, sbac);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra CU ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return t0;
}

static int evcd_eco_ats_mode_h(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    u32 t0;

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_mode);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuH ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return t0;
}

static int evcd_eco_ats_mode_v(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    u32 t0;

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_mode);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuV ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return t0;
}

static void parse_positionLastXY(EVC_BSR *bs, EVCD_SBAC *sbac, int* last_x, int* last_y, int width, int height, int ch_type)
{
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;
    SBAC_CTX_MODEL* cm_x = sbac_ctx->last_sig_coeff_x_prefix + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_LAST_SIG_COEFF_LUMA : 11));
    SBAC_CTX_MODEL* cm_y = sbac_ctx->last_sig_coeff_y_prefix + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_LAST_SIG_COEFF_LUMA : 11));
    int last;
    int blk_offset_x, blk_offset_y, shift_x, shift_y;
    int pos_x, pos_y;
    int i, cnt, tmp;
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
    // last_sig_coeff_x_prefix
    for (pos_x = 0; pos_x < g_group_idx[width - 1]; pos_x++)
    {
        last = evcd_sbac_decode_bin(bs, sbac, cm_x + blk_offset_x + (pos_x >> shift_x));
        if (!last)
        {
            break;
        }
    }

    // last_sig_coeff_y_prefix
    for (pos_y = 0; pos_y < g_group_idx[height - 1]; pos_y++)
    {
        last = evcd_sbac_decode_bin(bs, sbac, cm_y + blk_offset_y + (pos_y >> shift_y));
        if (!last)
        {
            break;
        }
    }

    // last_sig_coeff_x_suffix
    if (pos_x > 3)
    {
        tmp = 0;
        cnt = (pos_x - 2) >> 1;
        for (i = cnt - 1; i >= 0; i--)
        {
            last = sbac_decode_bin_ep(bs, sbac);
            tmp += last << i;
        }

        pos_x = g_min_in_group[pos_x] + tmp;
    }
    // last_sig_coeff_y_suffix
    if (pos_y > 3)
    {
        tmp = 0;
        cnt = (pos_y - 2) >> 1;
        for (i = cnt - 1; i >= 0; i--)
        {
            last = sbac_decode_bin_ep(bs, sbac);
            tmp += last << i;
        }
        pos_y = g_min_in_group[pos_y] + tmp;
    }

    *last_x = pos_x;
    *last_y = pos_y;
}
static int parse_coef_remain_exgolomb(EVC_BSR *bs, EVCD_SBAC *sbac, int rparam)
{
    int symbol = 0;
    int prefix = 0;
    int code_word = 0;

    do
    {
        prefix++;
        code_word = sbac_decode_bin_ep(bs, sbac);
    } while (code_word);

    code_word = 1 - code_word;
    prefix -= code_word;
    code_word = 0;
    if (prefix < g_go_rice_range[rparam])
    {
        code_word = sbac_decode_bins_ep(bs, sbac, rparam);
        symbol = (prefix << rparam) + code_word;
    }
    else
    {
        code_word = sbac_decode_bins_ep(bs, sbac, prefix - g_go_rice_range[rparam] + rparam);
        symbol = (((1 << (prefix - g_go_rice_range[rparam])) + g_go_rice_range[rparam] - 1) << rparam) + code_word;
    }

    return symbol;
}
static int evcd_eco_adcc(EVC_BSR *bs, EVCD_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type)
{
    int width = 1 << log2_w;
    int height = 1 << log2_h;
    int offset0;
    SBAC_CTX_MODEL* cm_sig_coeff;
    SBAC_CTX_MODEL* cm_gtAB;
    int scan_type = COEF_SCAN_ZIGZAG;

    int log2_block_size = min(log2_w, log2_h);
    u16 *scan;
    u16 *scan_inv;
    int scan_pos_last = -1;
    int last_x = 0, last_y = 0;
    int num_coeff;
    int ipos;
    int last_scan_set;
    int rice_param;
    int sub_set;
    int ctx_sig_coeff = 0;
    int cg_log2_size = LOG2_CG_SIZE;
    int is_last_nz = 0;
    int pos_last = 0;
    int cnt_gtA = 0;
    int cnt_gtB = 0;
    int ctx_gtA = 0;
    int ctx_gtB = 0;
    int escape_data_present_ingroup = 0;
    int blkpos, sx, sy;
    u32 sig_coeff_flag;

    // decode last position
    parse_positionLastXY(bs, sbac, &last_x, &last_y, width, height, ch_type);
    int max_num_coef = width * height;

    scan = evc_scan_tbl[scan_type][log2_w - 1][log2_h - 1];
    scan_inv = evc_inv_scan_tbl[scan_type][log2_w - 1][log2_h - 1];

    int last_pos_in_raster = last_x + last_y * width;
    int last_pos_in_scan = scan_inv[last_pos_in_raster];
    num_coeff = last_pos_in_scan + 1;
    //===== code significance flag =====
    offset0 = log2_block_size <= 2 ? 0 : NUM_CTX_SIG_COEFF_LUMA_TU << (EVC_MIN(1, (log2_block_size - 3)));
    if (sbac->ctx.sps_cm_init_flag == 1)
    {
        cm_sig_coeff = (ch_type == Y_C) ? sbac->ctx.sig_coeff_flag + offset0 : sbac->ctx.sig_coeff_flag + NUM_CTX_SIG_COEFF_LUMA;
        cm_gtAB = (ch_type == Y_C) ? sbac->ctx.coeff_abs_level_greaterAB_flag : sbac->ctx.coeff_abs_level_greaterAB_flag + NUM_CTX_GTX_LUMA;
    }
    else
    {
        cm_sig_coeff = (ch_type == Y_C) ? sbac->ctx.sig_coeff_flag : sbac->ctx.sig_coeff_flag + 1;
        cm_gtAB = (ch_type == Y_C) ? sbac->ctx.coeff_abs_level_greaterAB_flag : sbac->ctx.coeff_abs_level_greaterAB_flag + 1;
    }
    last_scan_set = (num_coeff - 1) >> cg_log2_size;
    scan_pos_last = num_coeff - 1;

    rice_param = 0;
    ipos = scan_pos_last;

    for (sub_set = last_scan_set; sub_set >= 0; sub_set--)
    {
        int num_nz = 0;
        int sub_pos = sub_set << cg_log2_size;
        int abs_coef[1 << LOG2_CG_SIZE ];
        int pos[1 << LOG2_CG_SIZE];
        int last_nz_pos_in_cg = -1;
        int first_nz_pos_in_cg = 1 << cg_log2_size;

        {
            for (; ipos >= sub_pos; ipos--)
            {
                blkpos = scan[ipos];
                sx = blkpos & (width - 1);
                sy = blkpos >> log2_w;

                // sigmap         
                if (ipos == scan_pos_last)
                {
                    ctx_sig_coeff = 0;
                }
                else
                {
                    ctx_sig_coeff = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_sig_coeff_inc(coef, blkpos, width, height, ch_type) : 0;
                }

                if (!(ipos == scan_pos_last)) // skipping signaling flag for last, we know it is non-zero
                {
                    sig_coeff_flag = evcd_sbac_decode_bin(bs, sbac, cm_sig_coeff + ctx_sig_coeff);
                }
                else
                {
                    sig_coeff_flag = 1;
                }
                coef[blkpos] = sig_coeff_flag;

                if (sig_coeff_flag)
                {
                    pos[num_nz] = blkpos;
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
                int i, idx;
                u32 coef_signs_group;
                int c2_idx = 0;
                escape_data_present_ingroup = 0;

                for (i = 0; i < num_nz; i++)
                {
                    abs_coef[i] = 1;
                }
                int numC1Flag = min(num_nz, CAFLAG_NUMBER);
                int firstC2FlagIdx = -1;

                for (int idx = 0; idx < numC1Flag; idx++)
                {
                    if (pos[idx] != pos_last)
                    {
                        ctx_gtA = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtA_inc(coef, pos[idx], width, height, ch_type) : 0;
                    }
                    u32 coeff_abs_level_greaterA_flag = evcd_sbac_decode_bin(bs, sbac, cm_gtAB + ctx_gtA);
                    coef[pos[idx]] += coeff_abs_level_greaterA_flag;
                    abs_coef[idx] = coeff_abs_level_greaterA_flag + 1;
                    if (coeff_abs_level_greaterA_flag == 1)
                    {
                        if (firstC2FlagIdx == -1)
                        {
                            firstC2FlagIdx = idx;
                        }
                        else //if a greater-than-one has been encountered already this group
                        {
                            escape_data_present_ingroup = TRUE;
                        }
                    }
                }
                if (firstC2FlagIdx != -1)
                {
                    if (pos[firstC2FlagIdx] != pos_last)
                    {
                        ctx_gtB = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtB_inc(coef, pos[firstC2FlagIdx], width, height, ch_type) : 0;
                    }
                    u32 coeff_abs_level_greaterB_flag = evcd_sbac_decode_bin(bs, sbac, cm_gtAB + ctx_gtB);
                    coef[pos[firstC2FlagIdx]] += coeff_abs_level_greaterB_flag;
                    abs_coef[firstC2FlagIdx] = coeff_abs_level_greaterB_flag + 2;
                    if (coeff_abs_level_greaterB_flag != 0)
                    {
                        escape_data_present_ingroup = 1;
                    }
                }
                escape_data_present_ingroup = escape_data_present_ingroup || (num_nz > CAFLAG_NUMBER);

                int iFirstCoeff2 = 1;
                if (escape_data_present_ingroup)
                {
                    for (idx = 0; idx < num_nz; idx++)
                    {
                        int base_level = (idx < CAFLAG_NUMBER) ? (2 + iFirstCoeff2) : 1;

                        if (abs_coef[idx] >= base_level)
                        {
                            int coeff_abs_level_remaining;

                            rice_param = get_rice_para(coef, pos[idx], width, height, base_level);
                            coeff_abs_level_remaining = parse_coef_remain_exgolomb(bs, sbac, rice_param);
                            coef[pos[idx]] = coeff_abs_level_remaining + base_level;
                            abs_coef[idx]  = coeff_abs_level_remaining + base_level;
                        }
                        if (abs_coef[idx] >= 2)
                        {
                            iFirstCoeff2 = 0;
                        }
                    }
                }
                coef_signs_group = sbac_decode_bins_ep(bs, sbac, num_nz);
                coef_signs_group <<= 32 - num_nz;

                for (idx = 0; idx < num_nz; idx++)
                {
                    blkpos = pos[idx];
                    coef[blkpos] = abs_coef[idx];

                    int sign = (int)((coef_signs_group) >> 31);
                    coef[blkpos] = sign > 0 ? -coef[blkpos] : coef[blkpos];
                    coef_signs_group <<= 1;
                } // for non-zero coefs within cg
            } // if nnz
        }
    } // for (cg)
    return EVC_OK;
}


static int evcd_eco_xcoef(EVC_BSR *bs, EVCD_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type, u8 ats_inter_info, int is_intra, int tool_adcc)
{
    if (is_intra)
    {
        assert(ats_inter_info == 0);
    }
    get_tu_size(ats_inter_info, log2_w, log2_h, &log2_w, &log2_h);

    if (tool_adcc)
    {
        evcd_eco_adcc(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1));
    }
    else
    {
        evcd_eco_run_length_cc(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1));
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
    return EVC_OK;
}

static int evcd_eco_refi(EVC_BSR * bs, EVCD_SBAC * sbac, int num_refp)
{
    EVC_SBAC_CTX * c = &sbac->ctx;
    int ref_num = 0;

    if(num_refp > 1)
    {
        if(evcd_sbac_decode_bin(bs, sbac, c->refi))
        {
            ref_num++;
            if(num_refp > 2 && evcd_sbac_decode_bin(bs, sbac, c->refi + 1))
            {
                ref_num++;
                for(; ref_num < num_refp - 1; ref_num++)
                {
                    if(!sbac_decode_bin_ep(bs, sbac))
                    {
                        break;
                    }
                }
                return ref_num;
            }
        }
    }
    return ref_num;
}

static int evcd_eco_merge_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    int idx;
    idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.merge_idx, NUM_CTX_MERGE_IDX, MAX_NUM_MVP);

#if ENC_DEC_TRACE
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("merge idx ");
    EVC_TRACE_INT(idx);
    EVC_TRACE_STR("\n");
#endif

    return idx;
}

static int evcd_eco_mvp_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    int idx;
    idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, 3, 4);

#if ENC_DEC_TRACE
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvp idx ");
    EVC_TRACE_INT(idx);
    EVC_TRACE_STR("\n");
#endif

    return idx;
}

static int evcd_eco_affine_mvp_idx( EVC_BSR * bs, EVCD_SBAC * sbac )
{
#if AFF_MAX_NUM_MVP == 1
    return 0;
#else
#if ENC_DEC_TRACE
    int idx;
    idx = sbac_read_truncate_unary_sym( bs, sbac, sbac->ctx.affine_mvp_idx, NUM_CTX_AFFINE_MVP_IDX, AFF_MAX_NUM_MVP );

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR( "affine mvp idx " );
    EVC_TRACE_INT( idx );
    EVC_TRACE_STR( "\n" );

    return idx;
#else
    return sbac_read_truncate_unary_sym( bs, sbac, sbac->ctx.affine_mvp_idx, NUM_CTX_AFFINE_MVP_IDX, AFF_MAX_NUM_MVP );
#endif
#endif
}

void evcd_eco_mmvd_data(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;
    int        type = ctx->sh.mmvd_group_enable_flag && !(1 << (core->log2_cuw + core->log2_cuh) <= NUM_SAMPLES_BLOCK);
    int        parse_idx = 0;
    int        temp = 0;
    int        temp_t;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    if (type == 1)
    {
        /* mmvd_group_idx */
        temp_t = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.mmvd_group_idx + 0);
        if (temp_t == 1)
        {
            temp_t += evcd_sbac_decode_bin(bs, sbac, sbac->ctx.mmvd_group_idx + 1);
        }
    }
    else
    {
        temp_t = 0;
    }

    temp = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mmvd_merge_idx, NUM_CTX_MMVD_MERGE_IDX, MMVD_BASE_MV_NUM); /* mmvd_merge_idx */
    parse_idx = temp * MMVD_MAX_REFINE_NUM + temp_t * (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);

    temp = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mmvd_distance_idx, NUM_CTX_MMVD_DIST_IDX, MMVD_DIST_NUM); /* mmvd_distance_idx */
    parse_idx += (temp * 4);

    /* mmvd_direction_idx */
    temp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.mmvd_direction_idx);
    parse_idx += (temp * 2);
    temp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.mmvd_direction_idx + 1);
    parse_idx += (temp);

    core->mmvd_idx = parse_idx;

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mmvd_idx ");
    EVC_TRACE_INT(core->mmvd_idx);
    EVC_TRACE_STR("\n");
}

static int evcd_eco_mvr_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    return sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvr_idx, MAX_NUM_MVR, MAX_NUM_MVR);
}

static int evcd_eco_bi_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    u32 t0;

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.bi_idx);
    if(t0)
    {
        return 0;
    }
    else
    {
        t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.bi_idx + 1);
        if(t0)
        {
            return 1;
        }
        else
        {
            return 2;
        }
    }
}
#if DQP
static int evcd_eco_dqp(EVC_BSR * bs)
{
    EVCD_SBAC    *sbac;
    EVC_SBAC_CTX *sbac_ctx;
    int             dqp, sign;
    sbac = GET_SBAC_DEC(bs);
    sbac_ctx = &sbac->ctx;

    dqp = sbac_read_unary_sym(bs, sbac, sbac_ctx->delta_qp, NUM_CTX_DELTA_QP);

    if (dqp > 0)
    {
        sign = sbac_decode_bin_ep(bs, sbac);
        dqp = EVC_SIGN_SET(dqp, sign);
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("dqp ");
    EVC_TRACE_INT(dqp);
    EVC_TRACE_STR("\n");

    return dqp;
}
#endif
static u32 evcd_eco_abs_mvd(EVC_BSR *bs, EVCD_SBAC *sbac, SBAC_CTX_MODEL *model)
{
    u32 val = 0;
    u32 code = 0;
    u32 len;

    code = evcd_sbac_decode_bin(bs, sbac, model); /* use one model */

    if(code == 0)
    {
        len = 0;

        while(!(code & 1))
        {
            if(len == 0)
                code = evcd_sbac_decode_bin(bs, sbac, model);
            else
                code = sbac_decode_bin_ep(bs, sbac);
            len++;
        }
        val = (1 << len) - 1;

        while(len != 0)
        {
            if(len == 0)
                code = evcd_sbac_decode_bin(bs, sbac, model);
            else
                code = sbac_decode_bin_ep(bs, sbac);
            val += (code << (--len));
        }
    }

    return val;
}

static int evcd_eco_get_mvd(EVC_BSR * bs, EVCD_SBAC * sbac, s16 mvd[MV_D])
{
    u32 sign;
    s16 t16;

    /* MV_X */
    t16 = (s16)evcd_eco_abs_mvd(bs, sbac, sbac->ctx.mvd);

    if(t16 == 0) mvd[MV_X] = 0;
    else
    {
        /* sign */
        sign = sbac_decode_bin_ep(bs, sbac);

        if(sign) mvd[MV_X] = -t16;
        else mvd[MV_X] = t16;
    }

    /* MV_Y */
    t16 = (s16)evcd_eco_abs_mvd(bs, sbac, sbac->ctx.mvd);

    if(t16 == 0)
    {
        mvd[MV_Y] = 0;
    }
    else
    {
        /* sign */
        sign = sbac_decode_bin_ep(bs, sbac);

        if(sign) mvd[MV_Y] = -t16;
        else mvd[MV_Y] = t16;
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvd x ");
    EVC_TRACE_INT(mvd[MV_X]);
    EVC_TRACE_STR("mvd y ");
    EVC_TRACE_INT(mvd[MV_Y]);
    EVC_TRACE_STR("\n");

    return EVC_OK;
}

int evcd_eco_coef(EVCD_CTX * ctx, EVCD_CORE * core)
{
    u8          cbf[N_C];
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;
    int         ret;
    int         b_no_cbf = 0;

    u8          ats_inter_avail = check_ats_inter_info_coded(1 << core->log2_cuw, 1 << core->log2_cuh, core->pred_mode, ctx->sps.tool_ats);
    int         log2_tuw = core->log2_cuw;
    int         log2_tuh = core->log2_cuh;

    b_no_cbf |= core->pred_mode == MODE_DIR && core->affine_flag;
    b_no_cbf |= core->pred_mode == MODE_DIR_MMVD;
    b_no_cbf |= core->pred_mode == MODE_DIR;

    if(ctx->sps.tool_admvp == 0)
    {
        b_no_cbf = 0;
    }
    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    s16 *coef_temp[N_C];
    s16 coef_temp_buf[N_C][MAX_TR_DIM];
    int i, j, c;
    int log2_w_sub = (core->log2_cuw > MAX_TR_LOG2) ? MAX_TR_LOG2 : core->log2_cuw;
    int log2_h_sub = (core->log2_cuh > MAX_TR_LOG2) ? MAX_TR_LOG2 : core->log2_cuh;
    int loop_w = (core->log2_cuw > MAX_TR_LOG2) ? (1 << (core->log2_cuw - MAX_TR_LOG2)) : 1;
    int loop_h = (core->log2_cuh > MAX_TR_LOG2) ? (1 << (core->log2_cuh - MAX_TR_LOG2)) : 1;
    int stride = (1 << core->log2_cuw);
    int sub_stride = (1 << log2_w_sub);
    int tmp_coef[N_C] = { 0 };
    int is_sub = loop_h + loop_w > 2 ? 1 : 0;
    int cbf_all = 1;

    u8 ats_intra_cu_on = 0;
    u8 ats_mode_idx = 0;
    u8 is_intra = (core->pred_mode == MODE_INTRA) ? 1 : 0;

    evc_mset(core->is_coef_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);

    for (j = 0; j < loop_h; j++)
    {
        for (i = 0; i < loop_w; i++)
        {
            if(cbf_all)
            {
                ret = eco_cbf(bs, sbac, core->pred_mode, cbf, b_no_cbf, is_sub, j + i, &cbf_all, core->tree_cons
#if BD_CF_EXT
                              , ctx->sps.chroma_format_idc
#endif
                );
                evc_assert_rv(ret == EVC_OK, ret);
            }
            else
            {
                cbf[Y_C] = cbf[U_C] = cbf[V_C] = 0;
            }
            
#if DQP            
            int dqp;
            int qp_i_cb, qp_i_cr;
            if(ctx->pps.cu_qp_delta_enabled_flag && (((!(ctx->sps.dquant_flag) || (core->cu_qp_delta_code == 1 && !core->cu_qp_delta_is_coded))
                && (cbf[Y_C] || cbf[U_C] || cbf[V_C])) || (core->cu_qp_delta_code == 2 && !core->cu_qp_delta_is_coded)))
            {
                dqp = evcd_eco_dqp(bs);
                core->qp = GET_QP(ctx->tile[core->tile_num].qp_prev_eco, dqp);
#if BD_CF_EXT
                core->qp_y = GET_LUMA_QP(core->qp, ctx->sps.bit_depth_luma_minus8);
#else
                core->qp_y = GET_LUMA_QP(core->qp);
#endif
                core->cu_qp_delta_is_coded = 1;
                ctx->tile[core->tile_num].qp_prev_eco = core->qp;
            }
            else
            {
                dqp = 0;
                core->qp = GET_QP(ctx->tile[core->tile_num].qp_prev_eco, dqp);
#if BD_CF_EXT
                core->qp_y = GET_LUMA_QP(core->qp, ctx->sps.bit_depth_luma_minus8);
#else
                core->qp_y = GET_LUMA_QP(core->qp);
#endif
            }
#if BD_CF_EXT
            qp_i_cb = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh.qp_u_offset);
            qp_i_cr = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh.qp_v_offset);
            core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * ctx->sps.bit_depth_chroma_minus8;
            core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * ctx->sps.bit_depth_chroma_minus8;
#else
            qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_u_offset);
            qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_v_offset);
            core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * (BIT_DEPTH - 8);
            core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * (BIT_DEPTH - 8);
#endif
            
#endif
            if (ctx->sps.tool_ats && cbf[Y_C] && (core->log2_cuw <= 5 && core->log2_cuh <= 5) && is_intra)
            {
                evc_assert(!evcd_check_only_inter(ctx, core));

                ats_intra_cu_on = evcd_eco_ats_intra_cu(bs, sbac);

                ats_mode_idx = 0;
                if (ats_intra_cu_on)
                {
                    u8 ats_intra_mode_h = evcd_eco_ats_mode_h(bs, sbac);
                    u8 ats_intra_mode_v = evcd_eco_ats_mode_v(bs, sbac);

                    ats_mode_idx = ((ats_intra_mode_h << 1) | ats_intra_mode_v);
                }
            }
            else
            {
                ats_intra_cu_on = 0;
                ats_mode_idx = 0;
            }
            core->ats_intra_cu = ats_intra_cu_on;
            core->ats_intra_mode_h = (ats_mode_idx >> 1);
            core->ats_intra_mode_v = (ats_mode_idx & 1);

            if (ats_inter_avail && (cbf[Y_C] || cbf[U_C] || cbf[V_C]))
            {
                evc_assert(!evcd_check_only_intra(ctx, core));
                eco_ats_inter_info(bs, sbac, core->log2_cuw, core->log2_cuh, &core->ats_inter_info, ats_inter_avail);
            }
            else
            {
                assert(core->ats_inter_info == 0);
            }

            for (c = 0; c < N_C; c++)
            {
                if (cbf[c])
                {
#if BD_CF_EXT
                    int pos_sub_x = c == 0 ? (i * (1 << (log2_w_sub))) : (i * (1 << (log2_w_sub - (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)))));
                    int pos_sub_y = c == 0 ? j * (1 << (log2_h_sub)) * (stride) : j * (1 << (log2_h_sub - (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc)))) * (stride >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)));
#else
                    int pos_sub_x = i * (1 << (log2_w_sub - !!c));
                    int pos_sub_y = j * (1 << (log2_h_sub - !!c)) * (stride >> (!!c));
#endif

                    if (is_sub)
                    {
#if BD_CF_EXT
                        if(c == 0)
                            evc_block_copy(core->coef[c] + pos_sub_x + pos_sub_y, stride, coef_temp_buf[c], sub_stride, log2_w_sub, log2_h_sub);
                        else
                            evc_block_copy(core->coef[c] + pos_sub_x + pos_sub_y, stride >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)), coef_temp_buf[c], sub_stride >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc))
                                           , log2_w_sub - (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)), log2_h_sub - (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc)));
#else
                        evc_block_copy(core->coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), coef_temp_buf[c], sub_stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
#endif
                        coef_temp[c] = coef_temp_buf[c];
                    }
                    else
                    {
                        coef_temp[c] = core->coef[c];
                    }
#if BD_CF_EXT
                    if(c == 0)
                        evcd_eco_xcoef(bs, sbac, coef_temp[c], log2_w_sub, log2_h_sub, c, core->ats_inter_info, core->pred_mode == MODE_INTRA, ctx->sps.tool_adcc);
                    else
                        evcd_eco_xcoef(bs, sbac, coef_temp[c], log2_w_sub - (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)), log2_h_sub - (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc))
                                       , c, core->ats_inter_info, core->pred_mode == MODE_INTRA, ctx->sps.tool_adcc);
#else
                    evcd_eco_xcoef(bs, sbac, coef_temp[c], log2_w_sub - (!!c), log2_h_sub - (!!c), c, core->ats_inter_info, core->pred_mode == MODE_INTRA, ctx->sps.tool_adcc);
#endif

                    evc_assert_rv(ret == EVC_OK, ret);

                    core->is_coef_sub[c][(j << 1) | i] = 1;
                    tmp_coef[c] += 1;

                    if (is_sub)
                    {
#if BD_CF_EXT
                        if(c == 0)
                            evc_block_copy(coef_temp_buf[c], sub_stride, core->coef[c] + pos_sub_x + pos_sub_y, stride, log2_w_sub, log2_h_sub);
                        else
                            evc_block_copy(coef_temp_buf[c], sub_stride >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)), core->coef[c] + pos_sub_x + pos_sub_y, stride >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc))
                                           , log2_w_sub - (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc)), log2_h_sub - (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc)));
#else
                        evc_block_copy(coef_temp_buf[c], sub_stride >> (!!c), core->coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
#endif
                    }
                }
                else
                {
                    core->is_coef_sub[c][(j << 1) | i] = 0;
                    tmp_coef[c] += 0;
                }
            }
        }
    }
    for (c = 0; c < N_C; c++)
    {
        core->is_coef[c] = tmp_coef[c] ? 1 : 0;
    }
    return EVC_OK;
}

void evcd_eco_sbac_reset(EVC_BSR * bs, u8 slice_type, u8 slice_qp, int sps_cm_init_flag)
{
    int i;
    u32 t0;
    EVCD_SBAC    * sbac;
    EVC_SBAC_CTX * sbac_ctx;

    sbac = GET_SBAC_DEC(bs);
    sbac_ctx = &sbac->ctx;

    /* Initialization of the internal variables */
    sbac->range = 16384;
    sbac->value = 0;
    for(i = 0; i < 14; i++)
    {
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &t0, 0);
#else
        evc_bsr_read1(bs, &t0);
#endif
        sbac->value = ((sbac->value << 1) | t0) & 0xFFFF;
    }

    evc_mset(sbac_ctx, 0x00, sizeof(EVC_SBAC_CTX));

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
    else
    {
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
        for(i = 0; i < NUM_CTX_MVD; i++) sbac_ctx->mvd[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_REF_IDX; i++) sbac_ctx->refi[i] = PROB_INIT;
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
        for(i = 0; i < NUM_CTX_SKIP_FLAG; i++)  sbac_ctx->skip_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_IBC_FLAG; i++) sbac_ctx->ibc_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_MODE_FLAG; i++) sbac_ctx->ats_mode[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_FLAG; i++) sbac_ctx->ats_cu_inter_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_QUAD_FLAG; i++) sbac_ctx->ats_cu_inter_quad_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_HOR_FLAG; i++) sbac_ctx->ats_cu_inter_hor_flag[i] = PROB_INIT;
        for(i = 0; i < NUM_CTX_ATS_INTER_POS_FLAG; i++) sbac_ctx->ats_cu_inter_pos_flag[i] = PROB_INIT;
    }
}

static int intra_mode_read_trunc_binary(int max_symbol, EVCD_SBAC * sbac, EVC_BSR *bs)
{
    int threshold = 4; /* we use 6 bits to signal the default mode */
    int val = 1 << threshold;
    int b;
    int ipm = 0;
    int t0 = 0;

    b = max_symbol - val;
    assert(b < val);
    ipm = sbac_decode_bins_ep(bs, sbac, threshold);
    if(ipm >= val - b)
    {
        t0 = sbac_decode_bins_ep(bs, sbac, 1);
        ipm <<= 1;
        ipm += t0;
        ipm -= (val - b);
    }
    return ipm;
}

int evcd_eco_intra_dir_b(EVC_BSR * bs, EVCD_SBAC * sbac,   u8  * mpm, u8 mpm_ext[8], u8 pims[IPD_CNT])
{
    u32 t0;
    int ipm = 0;
    int i;
    t0 = sbac_read_unary_sym(bs, sbac, sbac->ctx.intra_dir, 2);
    EVC_TRACE_COUNTER;
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_STR("mpm list: ");
#endif
    for (i = 0; i< IPD_CNT_B; i++)
    {
        if (t0== mpm[i])
        {
            ipm = i;
        }
#if TRACE_ADDITIONAL_FLAGS
        EVC_TRACE_INT(mpm[i]);
#endif
    }
    EVC_TRACE_STR("ipm Y ");
    EVC_TRACE_INT(ipm);
    EVC_TRACE_STR("\n");
    return ipm;
}

int evcd_eco_intra_dir(EVC_BSR * bs, EVCD_SBAC * sbac, u8 mpm[2], u8 mpm_ext[8], u8 pims[IPD_CNT])
{
    int ipm = 0;
    int mpm_flag;
    
    mpm_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_luma_pred_mpm_flag); /* intra_luma_pred_mpm_flag */

    if(mpm_flag)
    {
        int mpm_idx;
        mpm_idx = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_luma_pred_mpm_idx); /* intra_luma_pred_mpm_idx */
        ipm = mpm[mpm_idx];
    }
    else
    {
        int pims_flag;
        pims_flag = sbac_decode_bin_ep(bs, sbac); /* intra_luma_pred_pims_flag */
        if(pims_flag)
        {
            int pims_idx;
            pims_idx = sbac_decode_bins_ep(bs, sbac, 3); /* intra_luma_pred_pims_idx */
            ipm = mpm_ext[pims_idx];
        }
        else
        {
            int rem_mode;
            rem_mode = intra_mode_read_trunc_binary(IPD_CNT - (INTRA_MPM_NUM + INTRA_PIMS_NUM), sbac, bs); /* intra_luma_pred_rem_mode */
            ipm = pims[(INTRA_MPM_NUM + INTRA_PIMS_NUM) + rem_mode];
        }
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ipm Y ");
    EVC_TRACE_INT(ipm);
    EVC_TRACE_STR("\n");

    return ipm;
}

int evcd_eco_intra_dir_c(EVC_BSR * bs, EVCD_SBAC * sbac, u8 ipm_l)
{
    u32 t0;
    int ipm = 0;
    u8 chk_bypass;
#if TRACE_ADDITIONAL_FLAGS
    u8 ipm_l_saved = ipm_l;
#endif

    EVC_IPRED_CONV_L2C_CHK(ipm_l, chk_bypass);

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_chroma_pred_mode);
    if(t0 == 0)
    {
        ipm = sbac_read_unary_sym_ep(bs, sbac, IPD_CHROMA_CNT - 1);
        ipm++;
        if(chk_bypass &&  ipm >= ipm_l) ipm++;
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ipm UV ");
    EVC_TRACE_INT(ipm);
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_STR("ipm L ");
    EVC_TRACE_INT(ipm_l_saved);
#endif
    EVC_TRACE_STR("\n");

    return ipm;
}

void evcd_eco_direct_mode_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;
    int        direct_mode_flag = 0;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    direct_mode_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.direct_mode_flag);

    if(direct_mode_flag)
    {
        core->inter_dir = PRED_DIR;
    }
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("direct_mode_flag ");
    EVC_TRACE_INT(core->inter_dir);
    EVC_TRACE_STR("\n");
}

void evcd_eco_merge_mode_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    if(core->mvr_idx == 0)
    {
        EVCD_SBAC *sbac;
        EVC_BSR   *bs;
        int        merge_mode_flag = 0;

        bs = &ctx->bs;
        sbac = GET_SBAC_DEC(bs);

        merge_mode_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.merge_mode_flag);

        if(merge_mode_flag)
        {
            core->inter_dir = PRED_DIR;
        }
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("merge_mode_flag ");
        EVC_TRACE_INT(core->inter_dir == PRED_DIR ? PRED_DIR : 0);
        EVC_TRACE_STR("\n");
    }
}

void evcd_eco_inter_pred_idc(EVCD_CTX * ctx, EVCD_CORE * core)
{
    int tmp = 1;
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    if (check_bi_applicability(ctx->sh.slice_type, 1 << core->log2_cuw, 1 << core->log2_cuh, ctx->sps.tool_admvp))
    {
        tmp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);
    }

    if (!tmp)
    {
        core->inter_dir = PRED_BI;
    }
    else
    {
        tmp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 1);
        core->inter_dir = tmp ? PRED_L1 : core->ibc_flag ? PRED_IBC : PRED_L0;
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("inter dir ");
    EVC_TRACE_INT(core->inter_dir);
    EVC_TRACE_STR("\n");
}

s8 evcd_eco_split_mode(EVCD_CTX * c, EVC_BSR *bs, EVCD_SBAC *sbac, int cuw, int cuh, const int parent_split, int* same_layer_split,
                        const int node_idx, const int* parent_split_allow, int* curr_split_allow, int qt_depth, int btt_depth, int x, int y
                       , MODE_CONS mode_cons)
{
    int sps_cm_init_flag = sbac->ctx.sps_cm_init_flag;
    s8 split_mode = NO_SPLIT;
    int ctx = 0;
    int split_allow[SPLIT_CHECK_NUM];
    int i;

    if(cuw < 8 && cuh < 8)
    {
        split_mode = NO_SPLIT;
        return split_mode;
    }

    if (!c->sps.sps_btt_flag)
    {
        split_mode = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_cu_flag); /* split_cu_flag */
        split_mode = split_mode ? SPLIT_QUAD : NO_SPLIT;
        return split_mode;
    }

    evc_check_split_mode(split_allow, CONV_LOG2(cuw), CONV_LOG2(cuh), 0, 0, 0, c->log2_max_cuwh
                          , parent_split, same_layer_split, node_idx, parent_split_allow, qt_depth, btt_depth
                          , x, y, c->w, c->h
                          , NULL, c->sps.sps_btt_flag
                          , mode_cons);

    for(i = 1; i < SPLIT_CHECK_NUM; i++)
    {
        curr_split_allow[i] = split_allow[i];
    }

    if(split_allow[SPLIT_BI_VER] || split_allow[SPLIT_BI_HOR] || split_allow[SPLIT_TRI_VER] || split_allow[SPLIT_TRI_HOR])
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

            avail[0] = y_scu > 0  && (c->map_tidx[scup] == c->map_tidx[scup - w_scu]);  //up
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
                    if (i == 0)
                    {
                        smaller[i] = w[i] < cuw;
                    }
                    else
                    {
                        smaller[i] = h[i] < cuh;
                    }
                }
            }
            ctx = min(smaller[0] + smaller[1] + smaller[2], 2);
            ctx = ctx + 3 * evc_tbl_split_flag_ctx[log2_cuw - 2][log2_cuh - 2];
        }
        else
        {
            ctx = 0;
        }
                
        int btt_split_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.btt_split_flag + ctx); /* btt_split_flag */
        if(btt_split_flag)
        {
            u8 ctx_dir = sps_cm_init_flag == 1 ? (log2_cuw - log2_cuh + 2) : 0;
            u8 ctx_typ = 0;
            u8 btt_split_dir, btt_split_type;

            if((split_allow[SPLIT_BI_VER] || split_allow[SPLIT_TRI_VER]) &&
               (split_allow[SPLIT_BI_HOR] || split_allow[SPLIT_TRI_HOR]))
            {
                btt_split_dir = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.btt_split_dir + ctx_dir); /* btt_split_dir */
            }
            else
            {
                if(split_allow[SPLIT_BI_VER] || split_allow[SPLIT_TRI_VER])
                    btt_split_dir = 1;
                else
                    btt_split_dir = 0;
            }

            if((btt_split_dir && split_allow[SPLIT_BI_VER] && split_allow[SPLIT_TRI_VER]) ||
              (!btt_split_dir && split_allow[SPLIT_BI_HOR] && split_allow[SPLIT_TRI_HOR]))
            {
                btt_split_type = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.btt_split_type + ctx_typ); /* btt_split_type */
            }
            else
            {
                if((btt_split_dir && split_allow[SPLIT_TRI_VER]) ||
                  (!btt_split_dir && split_allow[SPLIT_TRI_HOR]))
                    btt_split_type = 1;
                else
                    btt_split_type = 0;
            }

            if(btt_split_type == 0) // BT
                split_mode = btt_split_dir ? SPLIT_BI_VER : SPLIT_BI_HOR;
            else
                split_mode = btt_split_dir ? SPLIT_TRI_VER : SPLIT_TRI_HOR;
        }
    }
    return split_mode;
}

s8 evcd_eco_suco_flag(EVC_BSR *bs, EVCD_SBAC *sbac, EVCD_CTX *c, EVCD_CORE *core, int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, int parent_suco)
{
    s8 suco_flag = parent_suco;
    int ctx = 0;
    u8 allow_suco = c->sps.sps_suco_flag ? evc_check_suco_cond(cuw, cuh, split_mode, boundary, log2_max_cuwh, c->sps.log2_diff_ctu_size_max_suco_cb_size, c->sps.log2_diff_max_suco_min_suco_cb_size, c->sps.log2_min_cb_size_minus2 + 2) : 0;

    if(!allow_suco)
    {
        return suco_flag;
    }

    if(sbac->ctx.sps_cm_init_flag == 1)
    {
        ctx = CONV_LOG2(EVC_MAX(cuw, cuh)) - 2;
        ctx = (cuw == cuh) ? ctx * 2 : ctx * 2 + 1;
    }
    else
    {
        ctx = 0;
    }

    suco_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.suco_flag + ctx);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("suco flag ");
    EVC_TRACE_INT(suco_flag);
    EVC_TRACE_STR("\n");

    return suco_flag;
}


void evcd_eco_mmvd_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    core->mmvd_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.mmvd_flag); /* mmvd_flag */

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mmvd_flag ");
    EVC_TRACE_INT(core->mmvd_flag);
    EVC_TRACE_STR("\n");
}

void evcd_eco_affine_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    core->affine_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_flag + core->ctx_flags[CNID_AFFN_FLAG]);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("affine flag ");
    EVC_TRACE_INT(core->affine_flag);
    EVC_TRACE_STR("\n");
}

int evcd_eco_affine_mode(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;
    u32          t0;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_mode);
#if TRACE_ADDITIONAL_FLAGS
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("affine mode ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");
#endif
    return t0;
}


int evcd_eco_affine_mvd_flag(EVCD_CTX * ctx, EVCD_CORE * core, int refi)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;
    u32          t0;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    t0 = evcd_sbac_decode_bin(bs, sbac, &sbac->ctx.affine_mvd_flag[refi]);
    return t0;
}

void evcd_eco_pred_mode( EVCD_CTX * ctx, EVCD_CORE * core )
{
    EVC_BSR     *bs = &ctx->bs;
    EVCD_SBAC   *sbac = GET_SBAC_DEC( bs );
    BOOL        pred_mode_flag = FALSE;
    TREE_CONS   tree_cons = core->tree_cons;
    u8*         ctx_flags = core->ctx_flags;

    MODE_CONS   pred_mode_constraint = tree_cons.mode_cons; //TODO: Tim changed place

    if (pred_mode_constraint == eAll)
    {
        pred_mode_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.pred_mode + ctx_flags[CNID_PRED_MODE]);
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("pred mode ");
        EVC_TRACE_INT(pred_mode_flag ? MODE_INTRA : MODE_INTER);
        EVC_TRACE_STR("\n");
    }

    BOOL isIbcAllowed = ctx->sps.ibc_flag &&
        core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size &&
        tree_cons.tree_type != TREE_C &&
        pred_mode_constraint != eOnlyInter &&
        !( pred_mode_constraint == eAll && pred_mode_flag );

    core->ibc_flag = FALSE;

    if ( isIbcAllowed )
        core->ibc_flag = evcd_sbac_decode_bin( bs, sbac, sbac->ctx.ibc_flag + ctx_flags[CNID_IBC_FLAG] );

    if ( core->ibc_flag )
        core->pred_mode = MODE_IBC;
    else if ( pred_mode_constraint == eOnlyInter )
        core->pred_mode = MODE_INTER;
    else if( pred_mode_constraint == eOnlyIntra )
        core->pred_mode = MODE_INTRA;
    else 
        core->pred_mode = pred_mode_flag ? MODE_INTRA : MODE_INTER;

#if TRACE_ADDITIONAL_FLAGS
    if ( isIbcAllowed )
    {
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR( "ibc pred mode " );
        EVC_TRACE_INT( !!core->ibc_flag );
        EVC_TRACE_STR( "ctx " );
        EVC_TRACE_INT( ctx_flags[CNID_IBC_FLAG] );
        EVC_TRACE_STR( "\n" );
    }
#endif
}

void evcd_eco_cu_skip_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;
    int        cu_skip_flag = 0;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    cu_skip_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.skip_flag + core->ctx_flags[CNID_SKIP_FLAG]); /* cu_skip_flag */

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("skip flag ");
    EVC_TRACE_INT(cu_skip_flag);
    EVC_TRACE_STR("ctx ");
    EVC_TRACE_INT(core->ctx_flags[CNID_SKIP_FLAG]);
    EVC_TRACE_STR("\n");

    if (cu_skip_flag)
    {
        core->pred_mode = MODE_SKIP;
    }
}

MODE_CONS evcd_eco_mode_constr( EVC_BSR *bs, u8 ctx_num )
{
    EVCD_SBAC   *sbac;
    u32          t0;

    sbac = GET_SBAC_DEC(bs);
    t0 = evcd_sbac_decode_bin( bs, sbac, sbac->ctx.mode_cons + ctx_num );
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mode_constr ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");
    return t0 ? eOnlyIntra : eOnlyInter;
}

int evcd_eco_cu(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;
    int          ret, cuw, cuh, mvp_idx[REFP_NUM] = { 0, 0 };
    int          mmvd_flag = 0;
    int          direct_idx = -1;
    int          REF_SET[3][MAX_NUM_ACTIVE_REF_FRAME] = { {0,0,}, };
    u8           bi_idx = BI_NON;
    
    core->pred_mode = MODE_INTRA;
    core->affine_flag = 0;
    evc_mset(core->affine_bzero, 0, sizeof(int) * REFP_NUM);
    evc_mset(core->affine_mvd, 0, sizeof(s16) * REFP_NUM * 3 * MV_D);
    core->ibc_flag = 0;
    core->mmvd_idx = 0;
    core->mvr_idx = 0;
    core->mmvd_flag = 0;
#if DMVR_FLAG
    core->dmvr_flag = 0;
#endif
    core->ats_inter_info = 0;
    core->mvp_idx[REFP_0] = 0;
    core->mvp_idx[REFP_1] = 0;
    core->inter_dir = 0;
    core->bi_idx = 0;
    evc_mset(core->mvd, 0, sizeof(s16) * REFP_NUM * MV_D);

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);
    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, ctx->map_tidx);
  
    if (!evcd_check_all(ctx, core))
    {
        evc_assert(evcd_check_only_intra(ctx, core));
    }

    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, core->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
                         , ctx->sps.ibc_flag, ctx->sps.ibc_log_max_size, ctx->map_tidx);    

    if ( !evcd_check_only_intra(ctx, core) )
    {
        /* CU skip flag */
        evcd_eco_cu_skip_flag(ctx, core); /* cu_skip_flag */
    }
       
    /* parse prediction info */
    if (core->pred_mode == MODE_SKIP)
    {
        if (ctx->sps.tool_mmvd)
        {
            evcd_eco_mmvd_flag(ctx, core); /* mmvd_flag */
        }

        if (core->mmvd_flag)
        {
            evcd_eco_mmvd_data(ctx, core);
        }
        else
        {
            if (ctx->sps.tool_affine && cuw >= 8 && cuh >= 8)
            {
                evcd_eco_affine_flag(ctx, core); /* affine_flag */
            }

            if (core->affine_flag)
            {
                core->mvp_idx[0] = evcd_eco_affine_mrg_idx(bs, sbac);
            }
            else
            {
                if(!ctx->sps.tool_admvp)
                {
                    core->mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac);
                    if(ctx->sh.slice_type == SLICE_B)
                    {
                        core->mvp_idx[REFP_1] = evcd_eco_mvp_idx(bs, sbac);
                    }
                }
                else
                {
                    core->mvp_idx[REFP_0] = evcd_eco_merge_idx(bs, sbac);
                    core->mvp_idx[REFP_1] = core->mvp_idx[REFP_0];
                }
            }
        }

        core->is_coef[Y_C] = core->is_coef[U_C] = core->is_coef[V_C] = 0;   //TODO: Tim why we need to duplicate code here?
        evc_mset(core->is_coef_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);
#if DQP //need to check
        if(ctx->pps.cu_qp_delta_enabled_flag)
        {
            int qp_i_cb, qp_i_cr;
            core->qp = ctx->tile[core->tile_num].qp_prev_eco;
#if BD_CF_EXT
            core->qp_y = GET_LUMA_QP(core->qp, ctx->sps.bit_depth_luma_minus8);
            qp_i_cb = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh.qp_u_offset);
            qp_i_cr = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh.qp_v_offset);
            core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * ctx->sps.bit_depth_chroma_minus8;
            core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * ctx->sps.bit_depth_chroma_minus8;
#else
            core->qp_y = GET_LUMA_QP(core->qp);

            qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_u_offset);
            qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_v_offset);

            core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * (BIT_DEPTH - 8);
            core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * (BIT_DEPTH - 8);
#endif
        }
        else
        {

            int qp_i_cb, qp_i_cr;
            core->qp = ctx->sh.qp;
#if BD_CF_EXT
            core->qp_y = GET_LUMA_QP(core->qp, ctx->sps.bit_depth_luma_minus8);
            qp_i_cb = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh.qp_u_offset);
            qp_i_cr = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh.qp_v_offset);
            core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * ctx->sps.bit_depth_chroma_minus8;
            core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * ctx->sps.bit_depth_chroma_minus8;
#else
            core->qp_y = GET_LUMA_QP(core->qp);

            qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_u_offset);
            qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh.qp_v_offset);

            core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * (BIT_DEPTH - 8);
            core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * (BIT_DEPTH - 8);
#endif
        }
#endif
    }
    else
    {
        evcd_eco_pred_mode(ctx, core);

        if (core->pred_mode == MODE_INTER)
        {
            if (ctx->sps.tool_amvr)
            {
                core->mvr_idx = evcd_eco_mvr_idx(bs, sbac);
#if TRACE_ADDITIONAL_FLAGS
                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("mvr_idx ");
                EVC_TRACE_INT(core->mvr_idx);
                EVC_TRACE_STR("\n");
#endif
            }

            if (ctx->sps.tool_admvp == 0)
            {
                evcd_eco_direct_mode_flag(ctx, core);
            }
            else
            {
                evcd_eco_merge_mode_flag(ctx, core);
            }
            
            if (core->inter_dir == PRED_DIR)
            {
                if (ctx->sps.tool_admvp != 0)
                {
                    if (ctx->sps.tool_mmvd)
                    {
                        evcd_eco_mmvd_flag(ctx, core); /* mmvd_flag */
                    }

                    if (core->mmvd_flag)
                    {
                        evcd_eco_mmvd_data(ctx, core);
                        core->inter_dir = PRED_DIR_MMVD;
                    }
                    else
                    {
                        if (ctx->sps.tool_affine && cuw >= 8 && cuh >= 8)
                        {
                            evcd_eco_affine_flag(ctx, core); /* affine_flag */
                        }

                        if (core->affine_flag)
                        {
                            core->inter_dir = AFF_DIR;
                            core->mvp_idx[0] = evcd_eco_affine_mrg_idx(bs, sbac);
                        }
                        else
                        {
                            core->mvp_idx[REFP_0] = evcd_eco_merge_idx(bs, sbac);
                            core->mvp_idx[REFP_1] = core->mvp_idx[REFP_0];
                        }
                    }
                    core->pred_mode = MODE_DIR;
                }
            }
            else
            {
                evcd_eco_inter_pred_idc(ctx, core); /* inter_pred_idc */

                if (cuw >= 16 && cuh >= 16 && ctx->sps.tool_affine && core->mvr_idx == 0)
                {
                    evcd_eco_affine_flag(ctx, core);
                }

                if (core->affine_flag)
                {
                    core->affine_flag += evcd_eco_affine_mode(ctx, core);

                    for (int inter_dir_idx = 0; inter_dir_idx < 2; inter_dir_idx++)
                    {
                        if (((core->inter_dir + 1) >> inter_dir_idx) & 1)
                        {
                            core->refi[inter_dir_idx] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[inter_dir_idx]);
                            core->mvp_idx[inter_dir_idx] = evcd_eco_affine_mvp_idx(bs, sbac);
                            core->affine_bzero[inter_dir_idx] = evcd_eco_affine_mvd_flag(ctx, core, inter_dir_idx);

                            for (int vertex = 0; vertex < core->affine_flag + 1; vertex++)
                            {
                                if (core->affine_bzero[inter_dir_idx])
                                {
                                    core->affine_mvd[inter_dir_idx][vertex][MV_X] = 0;
                                    core->affine_mvd[inter_dir_idx][vertex][MV_Y] = 0;
                                }
                                else
                                {
                                    evcd_eco_get_mvd(bs, sbac, core->affine_mvd[inter_dir_idx][vertex]);
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (ctx->sps.tool_admvp == 1 && core->inter_dir == PRED_BI)
                    {
                        core->bi_idx = evcd_eco_bi_idx(bs, sbac) + 1;
#if TRACE_ADDITIONAL_FLAGS
                        EVC_TRACE_COUNTER;
                        EVC_TRACE_STR("bi_idx ");
                        EVC_TRACE_INT(core->bi_idx-1);
                        EVC_TRACE_STR("\n");
#endif
                    }
                    for (int inter_dir_idx = 0; inter_dir_idx < 2; inter_dir_idx++)
                    {
                        /* 0: forward, 1: backward */
                        if (((core->inter_dir + 1) >> inter_dir_idx) & 1)
                        {
                            if (ctx->sps.tool_admvp == 0)
                            {
                                core->refi[inter_dir_idx] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[inter_dir_idx]);
                                core->mvp_idx[inter_dir_idx] = evcd_eco_mvp_idx(bs, sbac);
                                evcd_eco_get_mvd(bs, sbac, core->mvd[inter_dir_idx]);
                            }
                            else
                            {
                                if (core->bi_idx != BI_FL0 && core->bi_idx != BI_FL1)
                                {
                                    core->refi[inter_dir_idx] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[inter_dir_idx]);
                                }
                                if (core->bi_idx != BI_FL0 + inter_dir_idx)
                                {
                                    evcd_eco_get_mvd(bs, sbac, core->mvd[inter_dir_idx]);
                                }
                            }
                        }
                    }
                }
            }
        }
        else if (core->pred_mode == MODE_INTRA)
        {
            if (ctx->sps.tool_eipd)
            {
                evc_get_mpm(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                    core->mpm, core->avail_lr, core->mpm_ext, core->pims, ctx->map_tidx);
            }
            else
            {
                evc_get_mpm_b(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                    &core->mpm_b_list, core->avail_lr, core->mpm_ext, core->pims, ctx->map_tidx);
            }

            if (ctx->sps.tool_eipd)
            {
                if (evcd_check_luma(ctx, core))
                {
                core->ipm[0] = evcd_eco_intra_dir(bs, sbac, core->mpm, core->mpm_ext, core->pims);
                }
                else
                {
                    int luma_cup = evc_get_luma_cup(core->x_scu, core->y_scu, PEL2SCU(cuw), PEL2SCU(cuh), ctx->w_scu);
                    if (MCU_GET_IF(ctx->map_scu[luma_cup]))
                    {
                        core->ipm[0] = ctx->map_ipm[luma_cup];
                    }
                    else
                    {
                        core->ipm[0] = IPD_DC;
                    }
                }
                if (evcd_check_chroma(ctx, core)
#if BD_CF_EXT
                    && (ctx->sps.chroma_format_idc != 0)
#endif
                    )
                {
                core->ipm[1] = evcd_eco_intra_dir_c(bs, sbac, core->ipm[0]);
                }
            }
            else
            {
                int luma_ipm = IPD_DC_B;
                if (evcd_check_luma(ctx, core))
                {
                    core->ipm[0] = evcd_eco_intra_dir_b(bs, sbac, core->mpm_b_list, core->mpm_ext, core->pims);
                    luma_ipm = core->ipm[0];
                }
                else
                {
                    int luma_cup = evc_get_luma_cup(core->x_scu, core->y_scu, PEL2SCU(cuw), PEL2SCU(cuh), ctx->w_scu);
                    assert( MCU_GET_IF( ctx->map_scu[luma_cup] ) && "IBC is forbidden for this case (EIPD off)");
  
                    luma_ipm = ctx->map_ipm[luma_cup];
                }
                if(evcd_check_chroma(ctx, core))
                {
                    core->ipm[1] = luma_ipm;
                }
            }

            SET_REFI(core->refi, REFI_INVALID, REFI_INVALID);

            core->mv[REFP_0][MV_X] = core->mv[REFP_0][MV_Y] = 0;
            core->mv[REFP_1][MV_X] = core->mv[REFP_1][MV_Y] = 0;
        }
        else if (core->pred_mode == MODE_IBC)
        {
            core->affine_flag = 0;
            SET_REFI(core->refi, REFI_INVALID, REFI_INVALID);

            mvp_idx[REFP_0] = 0;

            evcd_eco_get_mvd(bs, sbac, core->mv[REFP_0]);
            
            core->mv[REFP_1][MV_X] = 0;
            core->mv[REFP_1][MV_Y] = 0;
        }
        else
        {
            evc_assert_rv(0, EVC_ERR_MALFORMED_BITSTREAM);
        }

        /* clear coefficient buffer */
        evc_mset(core->coef[Y_C], 0, cuw * cuh * sizeof(s16));
#if BD_CF_EXT
        evc_mset(core->coef[U_C], 0, (cuw >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc))) * (cuh >> (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc))) * sizeof(s16));
        evc_mset(core->coef[V_C], 0, (cuw >> (GET_CHROMA_W_SHIFT(ctx->sps.chroma_format_idc))) * (cuh >> (GET_CHROMA_H_SHIFT(ctx->sps.chroma_format_idc))) * sizeof(s16));
#else
        evc_mset(core->coef[U_C], 0, (cuw >> 1) * (cuh >> 1) * sizeof(s16));
        evc_mset(core->coef[V_C], 0, (cuw >> 1) * (cuh >> 1) * sizeof(s16));
#endif

        /* parse coefficients */
        ret = evcd_eco_coef(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    return EVC_OK;
}

int evcd_eco_nalu(EVC_BSR * bs, EVC_NALU * nalu)
{
    //nalu->nal_unit_size = evc_bsr_read(bs, 32);
    evc_bsr_read(bs, &nalu->forbidden_zero_bit, 1);

    if (nalu->forbidden_zero_bit != 0)
    {
        printf("malformed bitstream: forbidden_zero_bit != 0\n");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    evc_bsr_read(bs, &nalu->nal_unit_type_plus1, 6);
    evc_bsr_read(bs, &nalu->nuh_temporal_id, 3);
    evc_bsr_read(bs, &nalu->nuh_reserved_zero_5bits, 5);

    if (nalu->nuh_reserved_zero_5bits != 0)
    {
        printf("malformed bitstream: nuh_reserved_zero_5bits != 0");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    evc_bsr_read(bs, &nalu->nuh_extension_flag, 1);

    if (nalu->nuh_extension_flag != 0)
    {
        printf("malformed bitstream: nuh_extension_flag != 0");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    return EVC_OK;
}

int evcd_eco_rlp(EVC_BSR * bs, EVC_RPL * rpl)
{
    u32 delta_poc_st, strp_entry_sign_flag = 0;
    evc_bsr_read_ue(bs, &rpl->ref_pic_num);
    if (rpl->ref_pic_num > 0)
    {
        evc_bsr_read_ue(bs, &delta_poc_st);
        rpl->ref_pics[0] = delta_poc_st;
        if (rpl->ref_pics[0] != 0)
        {
            evc_bsr_read1(bs, &strp_entry_sign_flag);
            rpl->ref_pics[0] *= 1 - (strp_entry_sign_flag << 1);
        }
    }
    for (int i = 1; i < rpl->ref_pic_num; ++i)
    {
        evc_bsr_read_ue(bs, &delta_poc_st);        
        if (delta_poc_st != 0) {
            evc_bsr_read1(bs, &strp_entry_sign_flag);
        }
        rpl->ref_pics[i] = rpl->ref_pics[i - 1] + delta_poc_st * (1 - (strp_entry_sign_flag << 1));
    }

    return EVC_OK;
}

#if EVC_VUI_FIX
int evcd_eco_hrd_parameters(EVC_BSR * bs, EVC_HRD * hrd) {
    evc_bsr_read_ue(bs, &hrd->cpb_cnt_minus1);
    evc_bsr_read(bs, &hrd->bit_rate_scale, 4);
    evc_bsr_read(bs, &hrd->cpb_size_scale, 4);
    for (int SchedSelIdx = 0; SchedSelIdx <= hrd->cpb_cnt_minus1; SchedSelIdx++) {
        evc_bsr_read_ue(bs, &hrd->bit_rate_value_minus1[SchedSelIdx]);
        evc_bsr_read_ue(bs, &hrd->cpb_size_value_minus1[SchedSelIdx]);
        evc_bsr_read1(bs, &hrd->cbr_flag[SchedSelIdx]);
    }
    evc_bsr_read(bs, &hrd->initial_cpb_removal_delay_length_minus1, 5);
    evc_bsr_read(bs, &hrd->cpb_removal_delay_length_minus1, 5);
    evc_bsr_read(bs, &hrd->cpb_removal_delay_length_minus1, 5);
    evc_bsr_read(bs, &hrd->time_offset_length, 5);

    return EVC_OK;
}

int evcd_eco_vui(EVC_BSR * bs, EVC_VUI * vui)
{
    evc_bsr_read1(bs, &vui->aspect_ratio_info_present_flag);
    if (vui->aspect_ratio_info_present_flag) 
    {
        evc_bsr_read(bs, &vui->aspect_ratio_idc, 8);
        if (vui->aspect_ratio_idc == EXTENDED_SAR)
        {
            evc_bsr_read(bs, &vui->sar_width, 16);
            evc_bsr_read(bs, &vui->sar_height, 16);
        }
    }
    evc_bsr_read1(bs, &vui->overscan_info_present_flag);
    if (vui->overscan_info_present_flag)
    {
        evc_bsr_read1(bs, &vui->overscan_appropriate_flag);
    }
    evc_bsr_read1(bs, &vui->video_signal_type_present_flag);
    if (vui->video_signal_type_present_flag) 
    {
        evc_bsr_read(bs, &vui->video_format, 3);
        evc_bsr_read1(bs, &vui->video_full_range_flag);
        evc_bsr_read1(bs, &vui->colour_description_present_flag);
        if (vui->colour_description_present_flag) 
        {
            evc_bsr_read(bs, &vui->colour_primaries, 8);
            evc_bsr_read(bs, &vui->transfer_characteristics, 8);
            evc_bsr_read(bs, &vui->matrix_coefficients, 8);
        }
    }
    evc_bsr_read1(bs, &vui->chroma_loc_info_present_flag);
    if (vui->chroma_loc_info_present_flag) 
    {
        evc_bsr_read_ue(bs, &vui->chroma_sample_loc_type_top_field);
        evc_bsr_read_ue(bs, &vui->chroma_sample_loc_type_bottom_field);
    }
    evc_bsr_read1(bs, &vui->neutral_chroma_indication_flag);
#if ETM60_HLS_FIX
    evc_bsr_read1(bs, &vui->field_seq_flag);
#endif
    evc_bsr_read1(bs, &vui->timing_info_present_flag);
    if (vui->timing_info_present_flag)
    {
        evc_bsr_read(bs, &vui->num_units_in_tick, 32);
        evc_bsr_read(bs, &vui->time_scale, 32);
        evc_bsr_read1(bs, &vui->fixed_pic_rate_flag);
    }
    evc_bsr_read1(bs, &vui->nal_hrd_parameters_present_flag);
    if (vui->nal_hrd_parameters_present_flag)
    {
        evcd_eco_hrd_parameters(bs, &vui->hrd_parameters);
    }
    evc_bsr_read1(bs, &vui->vcl_hrd_parameters_present_flag);
    if (vui->vcl_hrd_parameters_present_flag)
    {
        evcd_eco_hrd_parameters(bs, &vui->hrd_parameters);
    }
    if (vui->nal_hrd_parameters_present_flag || vui->vcl_hrd_parameters_present_flag)
    {
        evc_bsr_read1(bs, &vui->low_delay_hrd_flag);
    }
    evc_bsr_read1(bs, &vui->pic_struct_present_flag);
    evc_bsr_read1(bs, &vui->bitstream_restriction_flag);
    if (vui->bitstream_restriction_flag) {
        evc_bsr_read1(bs, &vui->motion_vectors_over_pic_boundaries_flag);
        evc_bsr_read_ue(bs, &vui->max_bytes_per_pic_denom);
        evc_bsr_read_ue(bs, &vui->max_bits_per_mb_denom);
        evc_bsr_read_ue(bs, &vui->log2_max_mv_length_horizontal);
        evc_bsr_read_ue(bs, &vui->log2_max_mv_length_vertical);
        evc_bsr_read_ue(bs, &vui->num_reorder_pics);
        evc_bsr_read_ue(bs, &vui->max_dec_pic_buffering);
    }

    return EVC_OK;
}

#else
int evcd_eco_vui(EVC_BSR * bs)
{
    return EVC_OK;
}
#endif


int evcd_eco_sps(EVC_BSR * bs, EVC_SPS * sps)
{
#if TRACE_HLS
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ SPS Start ************\n");
#endif
    evc_bsr_read_ue(bs, &sps->sps_seq_parameter_set_id);
    evc_bsr_read(bs, &sps->profile_idc, 8);
    evc_bsr_read(bs, &sps->level_idc, 8);
    evc_bsr_read(bs, &sps->toolset_idc_h, 32);
    evc_bsr_read(bs, &sps->toolset_idc_l, 32);
    evc_bsr_read_ue(bs, &sps->chroma_format_idc);
    evc_bsr_read_ue(bs, &sps->pic_width_in_luma_samples);
    evc_bsr_read_ue(bs, &sps->pic_height_in_luma_samples);
    evc_bsr_read_ue(bs, &sps->bit_depth_luma_minus8);
    evc_bsr_read_ue(bs, &sps->bit_depth_chroma_minus8);
    evc_bsr_read1(bs, &sps->sps_btt_flag);
    if (sps->sps_btt_flag)
    {
        evc_bsr_read_ue(bs, &sps->log2_ctu_size_minus5);
        evc_bsr_read_ue(bs, &sps->log2_min_cb_size_minus2);
        evc_bsr_read_ue(bs, &sps->log2_diff_ctu_max_14_cb_size);
        evc_bsr_read_ue(bs, &sps->log2_diff_ctu_max_tt_cb_size);
        evc_bsr_read_ue(bs, &sps->log2_diff_min_cb_min_tt_cb_size_minus2);
    }
    evc_bsr_read1(bs, &sps->sps_suco_flag);
    if (sps->sps_suco_flag)
    {
        evc_bsr_read_ue(bs, &sps->log2_diff_ctu_size_max_suco_cb_size);
        evc_bsr_read_ue(bs, &sps->log2_diff_max_suco_min_suco_cb_size);
    }

    evc_bsr_read1(bs, &sps->tool_admvp);
    if (sps->tool_admvp)
    {
        evc_bsr_read1(bs, &sps->tool_affine);
        evc_bsr_read1(bs, &sps->tool_amvr);
        evc_bsr_read1(bs, &sps->tool_dmvr);
        evc_bsr_read1(bs, &sps->tool_mmvd);
#if M53737
        evc_bsr_read1(bs, &sps->tool_hmvp);
#endif
    }

    evc_bsr_read1(bs, &sps->tool_eipd);
    if(sps->tool_eipd)
    {
        evc_bsr_read1(bs, &sps->ibc_flag);
        if (sps->ibc_flag)
        {
            evc_bsr_read_ue(bs, &sps->ibc_log_max_size);
            sps->ibc_log_max_size += 2;
        }
    }

    evc_bsr_read1(bs, &sps->tool_cm_init);
    if(sps->tool_cm_init)
    {
        evc_bsr_read1(bs, &sps->tool_adcc);
    }

    evc_bsr_read1(bs, &sps->tool_iqt);
    if(sps->tool_iqt)
    {
        evc_bsr_read1(bs, &sps->tool_ats);
    }
    evc_bsr_read1(bs, &sps->tool_addb);
#if !DB_SPEC_ALIGNMENT2
#if M52291_HDR_DRA
    sps->tool_dra = evc_bsr_read1(bs);
#endif
#endif
    evc_bsr_read1(bs, &sps->tool_alf);
    evc_bsr_read1(bs, &sps->tool_htdf);
    evc_bsr_read1(bs, &sps->tool_rpl);
    evc_bsr_read1(bs, &sps->tool_pocs);
#if DQP
    evc_bsr_read1(bs, &sps->dquant_flag);
#endif
#if DB_SPEC_ALIGNMENT2
#if M52291_HDR_DRA
    evc_bsr_read1(bs, &sps->tool_dra);
#endif
#endif
    if (sps->tool_pocs)
    {
        evc_bsr_read_ue(bs, &sps->log2_max_pic_order_cnt_lsb_minus4);
    }
    if (!sps->tool_rpl || !sps->tool_pocs)
    {
        evc_bsr_read_ue(bs, &sps->log2_sub_gop_length);
        if (sps->log2_sub_gop_length == 0)
        {
            evc_bsr_read_ue(bs, &sps->log2_ref_pic_gap_length);
        }
    }

#if !M53744
    evc_bsr_read_ue(bs, &sps->sps_max_dec_pic_buffering_minus1);
#endif

    if (!sps->tool_rpl)
    {
         evc_bsr_read_ue(bs, &sps->max_num_ref_pics);
    }
    else
    {

#if M53744
        evc_bsr_read_ue(bs, &sps->sps_max_dec_pic_buffering_minus1);
#endif
        evc_bsr_read1(bs, &sps->long_term_ref_pics_flag);
        evc_bsr_read1(bs, &sps->rpl1_same_as_rpl0_flag);
        evc_bsr_read_ue(bs, &sps->num_ref_pic_lists_in_sps0);

        for (int i = 0; i < sps->num_ref_pic_lists_in_sps0; ++i)
            evcd_eco_rlp(bs, &sps->rpls_l0[i]);

        if (!sps->rpl1_same_as_rpl0_flag)
        {
            evc_bsr_read_ue(bs, &sps->num_ref_pic_lists_in_sps1);
            for (int i = 0; i < sps->num_ref_pic_lists_in_sps1; ++i)
                evcd_eco_rlp(bs, &sps->rpls_l1[i]);
        }
        else
        {
            assert(!"hasn't been implemented yet");
            //TBD: Basically copy everything from sps->rpls_l0 to sps->rpls_l1
        }
    }

    evc_bsr_read1(bs, &sps->picture_cropping_flag);
    if (sps->picture_cropping_flag)
    {
        evc_bsr_read_ue(bs, &sps->picture_crop_left_offset);
        evc_bsr_read_ue(bs, &sps->picture_crop_right_offset);
        evc_bsr_read_ue(bs, &sps->picture_crop_top_offset);
        evc_bsr_read_ue(bs, &sps->picture_crop_bottom_offset);
    }


#if M53744
    if (sps->chroma_format_idc != 0)
#endif
    {
        evc_bsr_read1(bs, &sps->chroma_qp_table_struct.chroma_qp_table_present_flag);
        if (sps->chroma_qp_table_struct.chroma_qp_table_present_flag)
        {
            evc_bsr_read1(bs, &sps->chroma_qp_table_struct.same_qp_table_for_chroma);
            evc_bsr_read1(bs, &sps->chroma_qp_table_struct.global_offset_flag);
            for (int i = 0; i < (sps->chroma_qp_table_struct.same_qp_table_for_chroma ? 1 : 2); i++)
            {
                evc_bsr_read_ue(bs, &sps->chroma_qp_table_struct.num_points_in_qp_table_minus1[i]);
                for (int j = 0; j <= sps->chroma_qp_table_struct.num_points_in_qp_table_minus1[i]; j++)
                {
                    evc_bsr_read(bs, &sps->chroma_qp_table_struct.delta_qp_in_val_minus1[i][j], 6);
                    evc_bsr_read_se(bs, &sps->chroma_qp_table_struct.delta_qp_out_val[i][j]);
                }
            }
        }
    }

    evc_bsr_read1(bs, &sps->vui_parameters_present_flag);
    if (sps->vui_parameters_present_flag)
#if EVC_VUI_FIX
        evcd_eco_vui(bs, &(sps->vui_parameters));
#else
        evcd_eco_vui(bs); //To be implemented
#endif
    u32 t0;
    while (!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs, &t0);
    }
#if TRACE_HLS
    EVC_TRACE_STR("************ SPS End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}

int evcd_eco_pps(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps)
{
#if TRACE_HLS
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ PPS Start ************\n");
#endif
    evc_bsr_read_ue(bs, &pps->pps_pic_parameter_set_id);
    assert(pps->pps_pic_parameter_set_id >= 0 && pps->pps_pic_parameter_set_id < MAX_NUM_PPS);
    evc_bsr_read_ue(bs, &pps->pps_seq_parameter_set_id);
    evc_bsr_read_ue(bs, &pps->num_ref_idx_default_active_minus1[0]);
    evc_bsr_read_ue(bs, &pps->num_ref_idx_default_active_minus1[1]);
    evc_bsr_read_ue(bs, &pps->additional_lt_poc_lsb_len);
    evc_bsr_read1(bs, &pps->rpl1_idx_present_flag);
    evc_bsr_read1(bs, &pps->single_tile_in_pic_flag);

    if(!pps->single_tile_in_pic_flag)
    {
        evc_bsr_read_ue(bs, &pps->num_tile_columns_minus1);
        evc_bsr_read_ue(bs, &pps->num_tile_rows_minus1);
        evc_bsr_read1(bs, &pps->uniform_tile_spacing_flag);
        if(!pps->uniform_tile_spacing_flag)
        {
            for(int i = 0; i < pps->num_tile_columns_minus1; ++i)
            {
                evc_bsr_read_ue(bs, &pps->tile_column_width_minus1[i]);
            }
            for(int i = 0; i < pps->num_tile_rows_minus1; ++i)
            {
                evc_bsr_read_ue(bs, &pps->tile_row_height_minus1[i]);
            }
        }
        evc_bsr_read1(bs, &pps->loop_filter_across_tiles_enabled_flag);
        evc_bsr_read_ue(bs, &pps->tile_offset_lens_minus1);
    }

    evc_bsr_read_ue(bs, &pps->tile_id_len_minus1);
    evc_bsr_read1(bs, &pps->explicit_tile_id_flag);
    if(pps->explicit_tile_id_flag)
    {
        for(int i = 0; i <= pps->num_tile_rows_minus1; ++i)
        {
            for(int j = 0; j <= pps->num_tile_columns_minus1; ++j)
            {
                evc_bsr_read(bs, &pps->tile_id_val[i][j], pps->tile_id_len_minus1 + 1);
            }
        }
    }

#if M52291_HDR_DRA
#if !ETM60_HLS_FIX
    pps->pic_dra_enabled_present_flag = 0;
#endif
    pps->pic_dra_enabled_flag = 0;
#if ETM60_HLS_FIX
    evc_bsr_read1(bs, &pps->pic_dra_enabled_flag);
    if (pps->pic_dra_enabled_flag)
    {
        evc_assert( sps->tool_dra == 1 ); 
        evc_bsr_read(bs, &pps->pic_dra_aps_id, APS_MAX_NUM_IN_BITS);
    }
#else
    if (sps->tool_dra)
    {
        evc_bsr_read1(bs, &pps->pic_dra_enabled_present_flag);
#if DB_SPEC_ALIGNMENT2
        if (pps->pic_dra_enabled_present_flag) 
        {
            evc_bsr_read1(bs, &pps->pic_dra_enabled_flag);
            if (pps->pic_dra_enabled_flag)
            {
                evc_bsr_read(bs, &pps->pic_dra_aps_id, APS_TYPE_ID_BITS);
            }
        }
#else
        pps->pic_dra_enabled_flag = evc_bsr_read1(bs);
        pps->pic_dra_aps_id = evc_bsr_read(bs, APS_TYPE_ID_BITS);
    }
    else
    {
        pps->pic_dra_enabled_present_flag = 0;
        pps->pic_dra_enabled_flag = 0;
#endif
    }
#endif
#endif

    evc_bsr_read1(bs, &pps->arbitrary_slice_present_flag);
    evc_bsr_read1(bs, &pps->constrained_intra_pred_flag);

#if DQP
    evc_bsr_read1(bs, &pps->cu_qp_delta_enabled_flag);
    if(pps->cu_qp_delta_enabled_flag)
    {
        evc_bsr_read_ue(bs, &pps->cu_qp_delta_area);
        pps->cu_qp_delta_area += 6;
    }
#endif
    u32 t0;
    while(!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs, &t0);
    }
#if TRACE_HLS    
    EVC_TRACE_STR("************ PPS End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}

#if M52291_HDR_DRA
int evcd_eco_aps_gen(EVC_BSR * bs, EVC_APS_GEN * aps
#if BD_CF_EXT
                     , int  bit_depth
#endif
)
{
#if TRACE_HLS        
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ APS Start ************\n");
#endif
    u32 aps_id, aps_type_id;

    evc_bsr_read(bs, &aps_id, APS_MAX_NUM_IN_BITS); // parse APS ID
    evc_bsr_read(bs, &aps_type_id, APS_TYPE_ID_BITS); // parse APS Type ID

    if (aps_type_id == 0)
    {
        EVC_APS_GEN *g_aps = aps ;
        g_aps->aps_id = aps_id;
        g_aps->aps_type_id = aps_type_id;

        EVC_APS local_alf_aps;
        evcd_eco_alf_aps_param(bs, &local_alf_aps); // parse ALF filter parameter (except ALF map)
        evc_AlfSliceParam * p_alf_data = &(local_alf_aps.alf_aps_param);
        evc_AlfSliceParam * p_aps_data = (evc_AlfSliceParam *)(aps->aps_data);
        memcpy(p_aps_data, p_alf_data, sizeof(evc_AlfSliceParam));
    }
    else if (aps_type_id == 1)
    {
        EVC_APS_GEN *g_aps = aps + 1;
        g_aps->aps_id = aps_id;
        g_aps->aps_type_id = aps_type_id;

        evcd_eco_dra_aps_param(bs, g_aps
#if BD_CF_EXT
                               , bit_depth
#endif
        );
    }
    else
        printf("This version of ETM doesn't support APS Type %d\n", aps->aps_type_id);

    u32 aps_extension_flag, aps_extension_data_flag, t0;
    evc_bsr_read1(bs, &aps_extension_flag);
    assert(aps_extension_flag == 0);
    if (aps_extension_flag)
    {
        while (0/*more_rbsp_data()*/)
        {
            evc_bsr_read1(bs, &aps_extension_data_flag);
        }
    }

    while (!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs, &t0);
    }
#if TRACE_HLS    
    EVC_TRACE_STR("************ APS End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}
#else
int evcd_eco_aps(EVC_BSR * bs, EVC_APS * aps)
{
    aps->aps_id = evc_bsr_read(bs, APS_MAX_NUM_IN_BITS); // parse APS ID
    evcd_eco_alf_aps_param(bs, aps); // parse ALF filter parameter (except ALF map)

    u8 aps_extension_flag = evc_bsr_read1(bs);
    // Dmytro: This is a temporal solution for: Decoder confirming EVC specification version 1, shall not depend on the value of aps_extension_data_flag. 
    // Assert shall be removed after implementing more_rbsp_data function
    assert(aps_extension_flag==0);  
    if (aps_extension_flag) 
    {
        while (0/*more_rbsp_data()*/)
        {
            u8 aps_extension_data_flag = evc_bsr_read1(bs);
        }
    }

    while (!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs);
    }


    return EVC_OK;
}
#endif

int evcd_truncatedUnaryEqProb(EVC_BSR * bs, const int maxSymbol)
{
    for(int k = 0; k < maxSymbol; k++)
    {
        u32 symbol;
        evc_bsr_read1(bs, &symbol);
        if(!symbol)
        {
            return k;
        }
    }
    return maxSymbol;
}

#if ETM70_GOLOMB_FIX
int alfGolombDecode(EVC_BSR * bs, const int k, const BOOL signed_val)
{
    int numLeadingBits = -1;
    uint32_t uiSymbol = 0;
    for (; !uiSymbol; numLeadingBits++)

    {
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &uiSymbol, 0);
#else
        evc_bsr_read1(bs, &uiSymbol); //alf_coeff_abs_prefix
#endif
    }


    int symbol = ((1 << numLeadingBits) - 1) << k;
    if (numLeadingBits + k > 0)
    {
        uint32_t bins;
        evc_bsr_read(bs, &bins, numLeadingBits + k);
        symbol += bins;
    }

    if (signed_val && symbol != 0)
    {
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &uiSymbol, 0);
#else
        evc_bsr_read1(bs, &uiSymbol);
#endif
        symbol = (uiSymbol) ? symbol : -symbol;
    }
    return symbol;
}
#else
int alfGolombDecode(EVC_BSR * bs, const int k)
{
    u32 uiSymbol;
    int q = -1;
    int nr = 0;
    int m = (int)pow(2.0, k);
    int a;

    uiSymbol = 1;
    while(uiSymbol)
    {
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &uiSymbol, 0);
#else
        evc_bsr_read1(bs, &uiSymbol);
#endif
        q++;
    }

    for(a = 0; a < k; ++a)          // read out the sequential log2(M) bits
    {
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &uiSymbol, 0);
#else
        evc_bsr_read1(bs, &uiSymbol);
#endif
        if(uiSymbol)
        {
            nr += 1 << a;
        }
    }
    nr += q * m;                    // add the bits and the multiple of M
    if(nr != 0)
    {
#if TRACE_HLS
        evc_bsr_read1_trace(bs, &uiSymbol, 0);
#else
        evc_bsr_read1(bs, &uiSymbol);
#endif
        nr = (uiSymbol) ? nr : -nr;
    }
    return nr;
}
#endif

u32 evcd_xReadTruncBinCode(EVC_BSR * bs, const int uiMaxSymbol)
{
    u32 ruiSymbol;
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
    int b = uiMaxSymbol - uiVal;
    evc_bsr_read(bs, &ruiSymbol, uiThresh); //xReadCode( uiThresh, ruiSymbol );
    if(ruiSymbol >= uiVal - b)
    {
        u32 uiSymbol;
        evc_bsr_read1(bs, &uiSymbol); //xReadFlag( uiSymbol );
        ruiSymbol <<= 1;
        ruiSymbol += uiSymbol;
        ruiSymbol -= (uiVal - b);
    }

    return ruiSymbol;
}

int evcd_eco_alf_filter(EVC_BSR * bs, evc_AlfSliceParam* alfSliceParam, const BOOL isChroma)
{
#if M53608_ALF_12
    if (!isChroma)
    {
        evc_bsr_read1(bs, &alfSliceParam->coeffDeltaFlag); // "alf_coefficients_delta_flag"
        if (!alfSliceParam->coeffDeltaFlag && alfSliceParam->numLumaFilters > 1)
        {
            evc_bsr_read1(bs, &alfSliceParam->coeffDeltaPredModeFlag); // "coeff_delta_pred_mode_flag"
        }
        else
        {
            alfSliceParam->coeffDeltaPredModeFlag = 0;
        }
        evc_AlfFilterShape alfShape;
        init_AlfFilterShape(&alfShape, isChroma ? 5 : (alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7));

        const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;

        int alf_luma_min_eg_order_minus1;
        int kMin;
        int kMinTab[MAX_NUM_ALF_COEFF];

        evc_bsr_read_ue(bs, &alf_luma_min_eg_order_minus1);
#if ALF_CONFORMANCE_CHECK
        evc_assert(alf_luma_min_eg_order_minus1 >= 0 && alf_luma_min_eg_order_minus1 <= 6);
#endif
        kMin = alf_luma_min_eg_order_minus1 + 1;

        const int numFilters = isChroma ? 1 : alfSliceParam->numLumaFilters;
        short* coeff = isChroma ? alfSliceParam->chromaCoeff : alfSliceParam->lumaCoeff;

        for (int idx = 0; idx < maxGolombIdx; idx++)
        {
            u32 alf_eg_order_increase_flag;
            evc_bsr_read1(bs, &alf_eg_order_increase_flag);
            kMinTab[idx] = kMin + alf_eg_order_increase_flag;
            kMin = kMinTab[idx];
        }
        if (alfSliceParam->coeffDeltaFlag)
        {
            for (int ind = 0; ind < alfSliceParam->numLumaFilters; ++ind)
            {
                evc_bsr_read1(bs, &alfSliceParam->filterCoeffFlag[ind]); // "filter_coefficient_flag[i]"
            }
        }
        for (int ind = 0; ind < numFilters; ++ind)
        {
            if (alfSliceParam->filterCoeffFlag[ind])
            {
                for (int i = 0; i < alfShape.numCoeff - 1; i++)
                {
#if ETM70_GOLOMB_FIX
                    coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode(bs, kMinTab[alfShape.golombIdx[i]], TRUE);
#else
                    coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode(bs, kMinTab[alfShape.golombIdx[i]]);
#endif
                }
            }
            else
            {
                memset(coeff + ind * MAX_NUM_ALF_LUMA_COEFF, 0, sizeof(*coeff) * alfShape.numCoeff);
                continue;
            }
        }
    }
    else
    {
        evc_AlfFilterShape alfShape;
        init_AlfFilterShape(&alfShape, isChroma ? 5 : (alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7));

        const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;

        int alf_luma_min_eg_order_minus1;
        int kMin;
        int kMinTab[MAX_NUM_ALF_COEFF];

        evc_bsr_read_ue(bs, &alf_luma_min_eg_order_minus1);
#if ALF_CONFORMANCE_CHECK
        evc_assert(alf_luma_min_eg_order_minus1 >= 0 && alf_luma_min_eg_order_minus1 <= 6);
#endif
        kMin = alf_luma_min_eg_order_minus1 + 1;

        const int numFilters = isChroma ? 1 : alfSliceParam->numLumaFilters;
        short* coeff = isChroma ? alfSliceParam->chromaCoeff : alfSliceParam->lumaCoeff;

        for (int idx = 0; idx < maxGolombIdx; idx++)
        {
            u32 alf_eg_order_increase_flag;
            evc_bsr_read1(bs, &alf_eg_order_increase_flag);
            kMinTab[idx] = kMin + alf_eg_order_increase_flag;
            kMin = kMinTab[idx];
        }
        for (int ind = 0; ind < numFilters; ++ind)
        {
            for (int i = 0; i < alfShape.numCoeff - 1; i++)
            {
#if ETM70_GOLOMB_FIX
                coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode(bs, kMinTab[alfShape.golombIdx[i]], TRUE);
#else
                coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode(bs, kMinTab[alfShape.golombIdx[i]]);
#endif
            }
        }
    }
#else
    if (!isChroma)
    {
        alfSliceParam->coeffDeltaFlag = evc_bsr_read1(bs); // "alf_coefficients_delta_flag"
        if (!alfSliceParam->coeffDeltaFlag && alfSliceParam->numLumaFilters > 1)
        {
            alfSliceParam->coeffDeltaPredModeFlag = evc_bsr_read1(bs); // "coeff_delta_pred_mode_flag"
        }
        else
        {
            alfSliceParam->coeffDeltaPredModeFlag = 0;
        }

    }

    evc_AlfFilterShape alfShape;
    init_AlfFilterShape(&alfShape, isChroma ? 5 : (alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7));

    const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;

    int min_golomb_order = evc_bsr_read_ue(bs); // "alf_luma_min_eg_order_minus1"
    int kMin = min_golomb_order + 1;
    int kMinTab[MAX_NUM_ALF_COEFF];

    const int numFilters = isChroma ? 1 : alfSliceParam->numLumaFilters;
    short* coeff = isChroma ? alfSliceParam->chromaCoeff : alfSliceParam->lumaCoeff;

    for (int idx = 0; idx < maxGolombIdx; idx++)
    {
        int code = evc_bsr_read1(bs); //"alf_eg_order_increase_flag"
        kMinTab[idx] = kMin + code;
        kMin = kMinTab[idx];
    }

    if (!isChroma)
    {
        if (alfSliceParam->coeffDeltaFlag)
        {
            for (int ind = 0; ind < alfSliceParam->numLumaFilters; ++ind)
            {
                int code = evc_bsr_read1(bs); // "filter_coefficient_flag[i]"
                alfSliceParam->filterCoeffFlag[ind] = code;
            }
        }
    }

    // Filter coefficients
    for (int ind = 0; ind < numFilters; ++ind)
    {
        if (!isChroma && !alfSliceParam->filterCoeffFlag[ind] && alfSliceParam->coeffDeltaFlag)
        {
            memset(coeff + ind * MAX_NUM_ALF_LUMA_COEFF, 0, sizeof(*coeff) * alfShape.numCoeff);
            continue;
        }

        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
            coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode(bs, kMinTab[alfShape.golombIdx[i]]);
        }
    }
#endif
    return EVC_OK;
}

#if M52291_HDR_DRA
int evcd_eco_dra_aps_param(EVC_BSR * bs, EVC_APS_GEN * aps
#if BD_CF_EXT
                           , int bit_depth
#endif
)
{
    int DRA_RANGE_10 = 10;
    int dra_number_ranges_minus1;
    int dra_equal_ranges_flag;
    int dra_global_offset;
    int dra_delta_range[32];
    SignalledParamsDRA *p_dra_param = (SignalledParamsDRA *)aps->aps_data;
    p_dra_param->m_signal_dra_flag = 1;
    evc_bsr_read(bs, &p_dra_param->m_dra_descriptor1, 4);
    evc_bsr_read(bs, &p_dra_param->m_dra_descriptor2, 4);
#if DRA_CONFORMANCE_CHECK
    evc_assert(p_dra_param->m_dra_descriptor1 == 4);
    evc_assert(p_dra_param->m_dra_descriptor2 == 9);
#endif
    int numBits = p_dra_param->m_dra_descriptor1 + p_dra_param->m_dra_descriptor2;
#if DRA_CONFORMANCE_CHECK
    evc_assert(numBits > 0);
#endif
    evc_bsr_read_ue(bs, &dra_number_ranges_minus1);
#if DRA_CONFORMANCE_CHECK
    evc_assert(dra_number_ranges_minus1 >= 0 && dra_number_ranges_minus1 <= 31);
#endif
    evc_bsr_read1(bs, &dra_equal_ranges_flag);
    evc_bsr_read(bs, &dra_global_offset, DRA_RANGE_10);
#if DRA_CONFORMANCE_CHECK
#if BD_CF_EXT
    evc_assert(dra_global_offset >= 1 && dra_global_offset <= EVC_MIN(1023, (1 << bit_depth) - 1));
#else
    evc_assert(dra_global_offset >= 1 && dra_global_offset <= EVC_MIN(1023, ( 1<<BIT_DEPTH) - 1) );
#endif
#endif
    if (dra_equal_ranges_flag)
    {
        evc_bsr_read(bs, &dra_delta_range[0], DRA_RANGE_10);
    }
    else
    {
        for (int i = 0; i <= dra_number_ranges_minus1; i++)
        {
            evc_bsr_read(bs, &dra_delta_range[i], DRA_RANGE_10);
#if DRA_CONFORMANCE_CHECK
#if BD_CF_EXT
            evc_assert(dra_delta_range[i] >= 1 && dra_delta_range[i] <= EVC_MIN(1023, (1 << bit_depth) - 1));
#else
            evc_assert(dra_delta_range[i] >= 1 && dra_delta_range[i] <= EVC_MIN(1023, (1 << BIT_DEPTH) - 1));
#endif
#endif
        }
    }

    for (int i = 0; i <= dra_number_ranges_minus1; i++)
    {
        evc_bsr_read(bs, &p_dra_param->m_dra_scale_value[i], numBits);
#if DRA_CONFORMANCE_CHECK
        evc_assert(p_dra_param->m_dra_scale_value[i] < (4 << p_dra_param->m_dra_descriptor2) );
#endif
    }

    evc_bsr_read(bs, &p_dra_param->m_dra_cb_scale_value, numBits);
    evc_bsr_read(bs, &p_dra_param->m_dra_cr_scale_value, numBits);
#if DRA_CONFORMANCE_CHECK
    evc_assert(p_dra_param->m_dra_cb_scale_value < (4 << p_dra_param->m_dra_descriptor2));
    evc_assert(p_dra_param->m_dra_cr_scale_value < (4 << p_dra_param->m_dra_descriptor2));
#endif
    evc_bsr_read_ue(bs, &p_dra_param->dra_table_idx);
#if DRA_CONFORMANCE_CHECK
    evc_assert(p_dra_param->dra_table_idx >= 0 && p_dra_param->dra_table_idx <= 58);
#endif
    p_dra_param->m_numRanges = dra_number_ranges_minus1 + 1;
    p_dra_param->m_equalRangesFlag = dra_equal_ranges_flag;
#if BD_CF_EXT
    p_dra_param->m_inRanges[0] = dra_global_offset << EVC_MAX(0, bit_depth - DRA_RANGE_10);
#else
    p_dra_param->m_inRanges[0] = dra_global_offset << EVC_MAX(0, BIT_DEPTH - DRA_RANGE_10) ;
#endif

    for (int i = 1; i <= p_dra_param->m_numRanges; i++)
    {
        int deltaRange = ((p_dra_param->m_equalRangesFlag == 1) ? dra_delta_range[0] : dra_delta_range[i - 1]);
#if BD_CF_EXT
        p_dra_param->m_inRanges[i] = p_dra_param->m_inRanges[i - 1] + (deltaRange << EVC_MAX(0, bit_depth - DRA_RANGE_10));
#else
        p_dra_param->m_inRanges[i] = p_dra_param->m_inRanges[i - 1] + ( deltaRange << EVC_MAX(0, BIT_DEPTH - DRA_RANGE_10) );
#endif
    }

    return EVC_OK;
}
#endif

#if INTEGR_M53608
int evcd_eco_alf_aps_param(EVC_BSR * bs, EVC_APS * aps)
{
    evc_AlfSliceParam* alfSliceParam = &(aps->alf_aps_param);
    //AlfSliceParam reset
    alfSliceParam->resetALFBufferFlag = 0;
    alfSliceParam->store2ALFBufferFlag = 0;
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->prevIdxComp[0] = 0;
    alfSliceParam->prevIdxComp[1] = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->isCtbAlfOn = 0;
    memset(alfSliceParam->enabledFlag, 0, 3 * sizeof(BOOL));
    alfSliceParam->lumaFilterType = ALF_FILTER_5;
    memset(alfSliceParam->lumaCoeff, 0, sizeof(short) * 325);
    memset(alfSliceParam->chromaCoeff, 0, sizeof(short) * 7);
    memset(alfSliceParam->filterCoeffDeltaIdx, 0, sizeof(short)*MAX_NUM_ALF_CLASSES);
    memset(alfSliceParam->filterCoeffFlag, 1, sizeof(BOOL) * 25);
    alfSliceParam->numLumaFilters = 1;
    alfSliceParam->coeffDeltaFlag = 0;
    alfSliceParam->coeffDeltaPredModeFlag = 0;
    alfSliceParam->chromaCtbPresentFlag = 0;
    alfSliceParam->fixedFilterPattern = 0;
    memset(alfSliceParam->fixedFilterIdx, 0, sizeof(int) * 25);
    memset(alfSliceParam->fixedFilterUsageFlag, 0, sizeof(u8) * 25);

    const int iNumFixedFilterPerClass = 16;
    int alf_luma_filter_signal_flag;
    int alf_chroma_filter_signal_flag;
    int alf_luma_num_filters_signalled_minus1;
    int alf_luma_type_flag;
    u32 alf_luma_coeff_delta_idx[MAX_NUM_ALF_CLASSES];
    int alf_luma_fixed_filter_usage_pattern;
    u32 alf_luma_fixed_filter_usage_flag[MAX_NUM_ALF_CLASSES];
    int alf_luma_fixed_filter_set_idx[MAX_NUM_ALF_CLASSES];

    memset(alf_luma_fixed_filter_set_idx, 0, sizeof(alf_luma_fixed_filter_set_idx));
    memset(alf_luma_fixed_filter_usage_flag, 0, sizeof(alf_luma_fixed_filter_usage_flag));

    evc_bsr_read1(bs, &alf_luma_filter_signal_flag);
    alfSliceParam->enabledFlag[0] = alf_luma_filter_signal_flag;
    evc_bsr_read1(bs, &alf_chroma_filter_signal_flag);
    alfSliceParam->chromaFilterPresent = alf_chroma_filter_signal_flag;

    if (alf_luma_filter_signal_flag)
    {
        evc_bsr_read_ue(bs, &alf_luma_num_filters_signalled_minus1);
#if ALF_CONFORMANCE_CHECK
        evc_assert( (alf_luma_num_filters_signalled_minus1 >= 0 ) && ( alf_luma_num_filters_signalled_minus1 <= MAX_NUM_ALF_CLASSES - 1) );
#endif
        evc_bsr_read1(bs, &alf_luma_type_flag);
        alfSliceParam->numLumaFilters = alf_luma_num_filters_signalled_minus1 + 1;
        alfSliceParam->lumaFilterType = alf_luma_type_flag;

        if (alf_luma_num_filters_signalled_minus1 > 0)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
                evc_bsr_read(bs, &alf_luma_coeff_delta_idx[i], evc_tbl_log2[alf_luma_num_filters_signalled_minus1] + 1);
                alfSliceParam->filterCoeffDeltaIdx[i] = alf_luma_coeff_delta_idx[i];
            }
        }
#if ETM70_GOLOMB_FIX
        alf_luma_fixed_filter_usage_pattern = alfGolombDecode(bs, 0, FALSE);
#else
        alf_luma_fixed_filter_usage_pattern = alfGolombDecode(bs, 0);
#endif
        alfSliceParam->fixedFilterPattern = alf_luma_fixed_filter_usage_pattern;
        if (alf_luma_fixed_filter_usage_pattern == 2)
        {
            for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
                evc_bsr_read1(bs, &alf_luma_fixed_filter_usage_flag[classIdx]); // "fixed_filter_flag"
                alfSliceParam->fixedFilterUsageFlag[classIdx] = alf_luma_fixed_filter_usage_flag[classIdx];
            }
        }
        else if (alf_luma_fixed_filter_usage_pattern == 1)
        {
            for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
                //on
                alf_luma_fixed_filter_usage_flag[classIdx] = 1;
                alfSliceParam->fixedFilterUsageFlag[classIdx] = 1;
            }
        }
        if (alf_luma_fixed_filter_usage_pattern > 0)
        {
            for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
                if (alf_luma_fixed_filter_usage_flag[classIdx] > 0)
                {
                    evc_bsr_read(bs, &alf_luma_fixed_filter_set_idx[classIdx], 4 );
#if ALF_CONFORMANCE_CHECK
                    evc_assert(alf_luma_fixed_filter_set_idx[classIdx] >= 0 && alf_luma_fixed_filter_set_idx[classIdx] <= 15);
#endif
                    alfSliceParam->fixedFilterIdx[classIdx] = alf_luma_fixed_filter_set_idx[classIdx];
                }
            }
        }

        evcd_eco_alf_filter(bs, &(aps->alf_aps_param), FALSE);
    }

    if (alf_chroma_filter_signal_flag)
    {
        evcd_eco_alf_filter(bs, &(aps->alf_aps_param), TRUE);
    }

    return EVC_OK;
}
#else
int evcd_eco_alf_aps_param(EVC_BSR * bs, EVC_APS * aps)
{
    evc_AlfSliceParam* alfSliceParam = &(aps->alf_aps_param);
    //AlfSliceParam reset
    alfSliceParam->resetALFBufferFlag = 0;
    alfSliceParam->store2ALFBufferFlag = 0;
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->prevIdxComp[0] = 0;
    alfSliceParam->prevIdxComp[1] = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->isCtbAlfOn = 0;
    memset(alfSliceParam->enabledFlag, 0, 3 * sizeof(BOOL));
    alfSliceParam->lumaFilterType = ALF_FILTER_5;
    memset(alfSliceParam->lumaCoeff, 0, sizeof(short) * 325);
    memset(alfSliceParam->chromaCoeff, 0, sizeof(short) * 7);
    memset(alfSliceParam->filterCoeffDeltaIdx, 0, sizeof(short)*MAX_NUM_ALF_CLASSES);
    memset(alfSliceParam->filterCoeffFlag, 1, sizeof(BOOL) * 25);
    alfSliceParam->numLumaFilters = 1;
    alfSliceParam->coeffDeltaFlag = 0;
    alfSliceParam->coeffDeltaPredModeFlag = 0;
    alfSliceParam->chromaCtbPresentFlag = 0;
    alfSliceParam->fixedFilterPattern = 0;
    memset(alfSliceParam->fixedFilterIdx, 0, sizeof(int) * 25);
#if M53608_ALF_7
    memset(alfSliceParam->fixedFilterUsageFlag, 0, sizeof(u8) * 25);
#endif
    alfSliceParam->enabledFlag[0] = evc_bsr_read1(bs);

#if !M53608_ALF_5
    if (!alfSliceParam->enabledFlag[0])
    {
        return EVC_OK;
    }
#endif
#if M53608_ALF_14
    u8 alfChromaSignalled = evc_bsr_read1(bs);
#else
    int alfChromaIdc = evcd_truncatedUnaryEqProb(bs, 3);
    alfSliceParam->enabledFlag[2] = alfChromaIdc & 1;
    alfSliceParam->enabledFlag[1] = (alfChromaIdc >> 1) & 1;
#endif
#if M53608_ALF_5
    if (alfSliceParam->enabledFlag[0])
#endif
    {
#if M53608_ALF_3
        alfSliceParam->numLumaFilters = evc_bsr_read_ue(bs) + 1;
#else
        alfSliceParam->numLumaFilters = evcd_xReadTruncBinCode(bs, MAX_NUM_ALF_CLASSES) + 1;
#endif
        alfSliceParam->lumaFilterType = !(evc_bsr_read1(bs));

        if (alfSliceParam->numLumaFilters > 1)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
#if M53608_ALF_3
                alfSliceParam->filterCoeffDeltaIdx[i] = (short)evc_bsr_read(bs, evc_tbl_log2[alfSliceParam->numLumaFilters - 1] + 1);
#else
                alfSliceParam->filterCoeffDeltaIdx[i] = (short)evcd_xReadTruncBinCode(bs, alfSliceParam->numLumaFilters);  //filter_coeff_delta[i]
#endif
            }
        }
#if !M53608_ALF_6
        char codetab_pred[3] = { 1, 0, 2 };
#endif
        const int iNumFixedFilterPerClass = 16;
        memset(alfSliceParam->fixedFilterIdx, 0, sizeof(alfSliceParam->fixedFilterIdx));
#if M53608_ALF_7
        memset(alfSliceParam->fixedFilterUsageFlag, 0, sizeof(alfSliceParam->fixedFilterUsageFlag));
#endif
        if (iNumFixedFilterPerClass > 0)
        {
#if M53608_ALF_6
            alfSliceParam->fixedFilterPattern = alfGolombDecode(bs, 0);
#else
            alfSliceParam->fixedFilterPattern = codetab_pred[alfGolombDecode(bs, 0)];
#endif
            if (alfSliceParam->fixedFilterPattern == 2)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
#if M53608_ALF_7
                    alfSliceParam->fixedFilterUsageFlag[classIdx] = evc_bsr_read1(bs); // "fixed_filter_flag"
#else
                    alfSliceParam->fixedFilterIdx[classIdx] = evc_bsr_read1(bs); // "fixed_filter_flag"
#endif
                }
            }
            else if (alfSliceParam->fixedFilterPattern == 1)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
#if M53608_ALF_7
                    //on
                    alfSliceParam->fixedFilterUsageFlag[classIdx] = 1;
#else
                    //on
                    alfSliceParam->fixedFilterIdx[classIdx] = 1;
#endif
                }
            }

            if (alfSliceParam->fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
#if M53608_ALF_7
                    if (alfSliceParam->fixedFilterUsageFlag[classIdx] > 0)
#else
                    if (alfSliceParam->fixedFilterIdx[classIdx] > 0)
#endif
                    {
#if M53608_ALF_3
#if M53608_ALF_7
                        alfSliceParam->fixedFilterIdx[classIdx] = (int)evc_bsr_read(bs, evc_tbl_log2[iNumFixedFilterPerClass - 1] + 1);
#else
                        alfSliceParam->fixedFilterIdx[classIdx] = (int)evc_bsr_read(bs, evc_tbl_log2[iNumFixedFilterPerClass - 1] + 1) + 1;
#endif
#else
#if M53608_ALF_7
                        alfSliceParam->fixedFilterIdx[classIdx] = (int)evcd_xReadTruncBinCode(bs, iNumFixedFilterPerClass);
#else
                        alfSliceParam->fixedFilterIdx[classIdx] = (int)evcd_xReadTruncBinCode(bs, iNumFixedFilterPerClass) + 1;
#endif
#endif
                    }
                }
            }
        }

        evcd_eco_alf_filter(bs, &(aps->alf_aps_param), FALSE);
    }

#if M53608_ALF_14
    if (alfChromaSignalled)
#else
    if (alfChromaIdc)
#endif
    {
        {
            evcd_eco_alf_filter(bs, &(aps->alf_aps_param), TRUE);
        }
    }

    return EVC_OK;
}
#endif

int evcd_eco_alf_sh_param(EVC_BSR * bs, EVC_SH * sh)
{
    evc_AlfSliceParam* alfSliceParam = &(sh->alf_sh_param);

    //AlfSliceParam reset
    alfSliceParam->prevIdxComp[0] = 0;
    alfSliceParam->prevIdxComp[1] = 0;
    alfSliceParam->resetALFBufferFlag = 0;
    alfSliceParam->store2ALFBufferFlag = 0;
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->isCtbAlfOn = 0;
    memset(alfSliceParam->enabledFlag, 0, 3 * sizeof(BOOL));
    alfSliceParam->lumaFilterType = ALF_FILTER_5;
    memset(alfSliceParam->lumaCoeff, 0, sizeof(short) * 325);
    memset(alfSliceParam->chromaCoeff, 0, sizeof(short) * 7);
    memset(alfSliceParam->filterCoeffDeltaIdx, 0, sizeof(short)*MAX_NUM_ALF_CLASSES);
    memset(alfSliceParam->filterCoeffFlag, 1, sizeof(BOOL) * 25);
    alfSliceParam->numLumaFilters = 1;
    alfSliceParam->coeffDeltaFlag = 0;
    alfSliceParam->coeffDeltaPredModeFlag = 0;
    alfSliceParam->chromaCtbPresentFlag = 0;
    alfSliceParam->fixedFilterPattern = 0;
    memset(alfSliceParam->fixedFilterIdx, 0, sizeof(int) * 25);
#if M53608_ALF_7
    memset(alfSliceParam->fixedFilterUsageFlag, 0, sizeof(u8) * 25);
#endif
    //decode map
    evc_bsr_read1(bs, &alfSliceParam->isCtbAlfOn);
    return EVC_OK;
}

int evcd_eco_sh(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_SH * sh, int nut)
{
#if TRACE_HLS    
    EVC_TRACE_STR("***********************************\n");
    EVC_TRACE_STR("************ SH  Start ************\n");
#endif
    int num_tiles_in_slice = 0;

    evc_bsr_read_ue(bs, &sh->slice_pic_parameter_set_id);
    assert(sh->slice_pic_parameter_set_id >= 0 && sh->slice_pic_parameter_set_id < MAX_NUM_PPS);
  #if M53744
    if (!pps->single_tile_in_pic_flag)
#endif
    {
        evc_bsr_read1(bs, &sh->single_tile_in_slice_flag);
        evc_bsr_read(bs, &sh->first_tile_id, pps->tile_id_len_minus1 + 1);
    }
    else
    {
        sh->single_tile_in_slice_flag = 1;    
    }

    if (!sh->single_tile_in_slice_flag)
    {
        if (pps->arbitrary_slice_present_flag)
        {
            evc_bsr_read1(bs, &sh->arbitrary_slice_flag);
        }
        if (!sh->arbitrary_slice_flag)
        {
            evc_bsr_read(bs, &sh->last_tile_id, pps->tile_id_len_minus1 + 1);
        }
        else
        {
            evc_bsr_read_ue(bs, &sh->num_remaining_tiles_in_slice_minus1);
            num_tiles_in_slice = sh->num_remaining_tiles_in_slice_minus1 + 2;
            for (int i = 0; i < num_tiles_in_slice - 1; ++i)
            {
                evc_bsr_read_ue(bs, &sh->delta_tile_id_minus1[i]);
            }
        }
    }

    evc_bsr_read_ue(bs, &sh->slice_type);

    if (!sh->arbitrary_slice_flag)
    {
        int first_tile_in_slice, last_tile_in_slice, first_tile_col_idx, last_tile_col_idx, delta_tile_idx;
        int w_tile, w_tile_slice, h_tile_slice, tile_cnt;

        w_tile = (pps->num_tile_columns_minus1 + 1);
        tile_cnt = (pps->num_tile_rows_minus1 + 1) * (pps->num_tile_columns_minus1 + 1);

        first_tile_in_slice = sh->first_tile_id;
        last_tile_in_slice = sh->last_tile_id;
        

        first_tile_col_idx = first_tile_in_slice % w_tile;
        last_tile_col_idx = last_tile_in_slice % w_tile;
        delta_tile_idx = last_tile_in_slice - first_tile_in_slice;

        if (last_tile_in_slice < first_tile_in_slice)
        {
            if (first_tile_col_idx > last_tile_col_idx)
            {
                delta_tile_idx += tile_cnt + w_tile;
            }
            else
            {
                delta_tile_idx += tile_cnt;
            }
        }
        else if (first_tile_col_idx > last_tile_col_idx)
        {
            delta_tile_idx += w_tile;
        }

        w_tile_slice = (delta_tile_idx % w_tile) + 1; //Number of tiles in slice width
        h_tile_slice = (delta_tile_idx / w_tile) + 1;
        num_tiles_in_slice = w_tile_slice * h_tile_slice;
    }
    else
    {
        num_tiles_in_slice = sh->num_remaining_tiles_in_slice_minus1 + 2;
    }

    if (nut == EVC_IDR_NUT)
    {
        evc_bsr_read1(bs, &sh->no_output_of_prior_pics_flag);
    }

    if (sps->tool_mmvd && ((sh->slice_type == SLICE_B)||(sh->slice_type == SLICE_P)) )
    {
        evc_bsr_read1(bs, &sh->mmvd_group_enable_flag);
    }
    else
    {
        sh->mmvd_group_enable_flag = 0;
    }

    if (sps->tool_alf)
    {
#if M53608_ALF_14
        sh->alfChromaIdc = 0;
        sh->alf_sh_param.enabledFlag[0] = sh->alf_sh_param.enabledFlag[1] = sh->alf_sh_param.enabledFlag[2] = 0;
#endif
        evc_bsr_read1(bs, &sh->alf_on);
#if M53608_ALF_14
        sh->alf_sh_param.enabledFlag[0] = sh->alf_on;
#endif
        if (sh->alf_on)
        {
            evc_bsr_read(bs, &sh->aps_id_y, 5);
            evcd_eco_alf_sh_param(bs, sh); // parse ALF map
#if M53608_ALF_14
            evc_bsr_read(bs, &sh->alfChromaIdc, 2);
            sh->alf_sh_param.enabledFlag[1] = sh->alfChromaIdc & 1;
            sh->alf_sh_param.enabledFlag[2] = (sh->alfChromaIdc >> 1) & 1;
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
                evc_bsr_read(bs, &sh->aps_id_ch, 5);
#if M53608_ALF_14
            }
#endif
        }
#if M53608_ALF_14
        if (sps->chroma_format_idc == 3 && sh->ChromaAlfEnabledFlag)
        {
            evc_bsr_read(bs, &sh->aps_id_ch, 5);
            evc_bsr_read1(bs, &sh->alfChromaMapSignalled);
        }
        if (sps->chroma_format_idc == 3 && sh->ChromaAlfEnabled2Flag)
        {
            evc_bsr_read(bs, &sh->aps_id_ch2, 5);
            evc_bsr_read1(bs, &sh->alfChroma2MapSignalled);
        }
#endif
    }

    if (nut != EVC_IDR_NUT)
    {
        if (sps->tool_pocs)
        {
            evc_bsr_read(bs, &sh->poc_lsb, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
        }
        if (sps->tool_rpl)
        {
            //L0 candidates signaling
            if (sps->num_ref_pic_lists_in_sps0 > 0)
            {
                evc_bsr_read1(bs, &sh->ref_pic_list_sps_flag[0]);
            }
            else
            {
                sh->ref_pic_list_sps_flag[0] = 0;
            }

            if (sh->ref_pic_list_sps_flag[0])
            {
                if (sps->num_ref_pic_lists_in_sps0 > 1)
                {
                    evc_bsr_read_ue(bs, &sh->rpl_l0_idx);
                    memcpy(&sh->rpl_l0, &sps->rpls_l0[sh->rpl_l0_idx], sizeof(sh->rpl_l0)); //TBD: temporal workaround, consider refactoring
                    sh->rpl_l0.poc = sh->poc_lsb;
                }
            }
            else
            {
                evcd_eco_rlp(bs, &sh->rpl_l0);
                sh->rpl_l0.poc = sh->poc_lsb;
            }

            //L1 candidates signaling
            if (pps->rpl1_idx_present_flag)
            {
                if (sps->num_ref_pic_lists_in_sps1 > 0)
                {
                    evc_bsr_read1(bs, &sh->ref_pic_list_sps_flag[1]);
                }
                else
                {
                    sh->ref_pic_list_sps_flag[1] = 0;
                }
            }
            else
            {
                sh->ref_pic_list_sps_flag[1] = sh->ref_pic_list_sps_flag[0];
            }

            if (sh->ref_pic_list_sps_flag[1])
            {
                if (pps->rpl1_idx_present_flag)
                {
                    if (sps->num_ref_pic_lists_in_sps1 > 1)
                    {
                        evc_bsr_read_ue(bs, &sh->rpl_l1_idx);
                    }
                }
                else
                {
                    sh->rpl_l1_idx = sh->rpl_l0_idx;
                }

                memcpy(&sh->rpl_l1, &sps->rpls_l1[sh->rpl_l1_idx], sizeof(sh->rpl_l1)); //TBD: temporal workaround, consider refactoring
                sh->rpl_l1.poc = sh->poc_lsb;
            }
            else
            {
                evcd_eco_rlp(bs, &sh->rpl_l1);
                sh->rpl_l1.poc = sh->poc_lsb;
            }
        }
    }

    if (sh->slice_type != SLICE_I)
    {
        evc_bsr_read1(bs, &sh->num_ref_idx_active_override_flag);
        if (sh->num_ref_idx_active_override_flag)
        {
            u32 num_ref_idx_active_minus1;
            evc_bsr_read_ue(bs, &num_ref_idx_active_minus1);
            sh->rpl_l0.ref_pic_active_num = num_ref_idx_active_minus1 + 1;
            if (sh->slice_type == SLICE_B)
            {
                evc_bsr_read_ue(bs, &num_ref_idx_active_minus1);
                sh->rpl_l1.ref_pic_active_num = num_ref_idx_active_minus1 + 1;
            }
        }
        else
        {
#if RPL_CLEANUP
            sh->rpl_l0.ref_pic_active_num = pps->num_ref_idx_default_active_minus1[REFP_0] + 1;
            sh->rpl_l1.ref_pic_active_num = pps->num_ref_idx_default_active_minus1[REFP_1] + 1;
#else
            sh->rpl_l0.ref_pic_active_num = 2;  //Temporarily i set it to 2. this should be set equal to the signalled num_ref_idx_default_active_minus1[0] + 1.
            sh->rpl_l1.ref_pic_active_num = 2;  //Temporarily i set it to 2. this should be set equal to the signalled num_ref_idx_default_active_minus1[1] + 1.
#endif
        }

        if (sps->tool_admvp)
        {
            evc_bsr_read1(bs, &sh->temporal_mvp_asigned_flag);
            if (sh->temporal_mvp_asigned_flag)
            {
                if (sh->slice_type == SLICE_B)
                {
                    evc_bsr_read1(bs, &sh->collocated_from_list_idx);
                    evc_bsr_read1(bs, &sh->collocated_mvp_source_list_idx);
                }
                evc_bsr_read1(bs, &sh->collocated_from_ref_idx);
            }
        }
    }
    evc_bsr_read1(bs, &sh->deblocking_filter_on);

    if(sh->deblocking_filter_on && sps->tool_addb)
    {
        evc_bsr_read_se(bs, &sh->sh_deblock_alpha_offset);
        evc_bsr_read_se(bs, &sh->sh_deblock_beta_offset);
    }

    evc_bsr_read(bs, &sh->qp, 6);
    if (sh->qp < 0 || sh->qp > 51)
    {
        printf("malformed bitstream: slice_qp should be in the range of 0 to 51\n");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    evc_bsr_read_se(bs, &sh->qp_u_offset);
    evc_bsr_read_se(bs, &sh->qp_v_offset);
#if BD_CF_EXT
    sh->qp_u = (s8)EVC_CLIP3(-6 * sps->bit_depth_luma_minus8, 57, sh->qp + sh->qp_u_offset);
    sh->qp_v = (s8)EVC_CLIP3(-6 * sps->bit_depth_luma_minus8, 57, sh->qp + sh->qp_v_offset);
#else
    sh->qp_u = (s8)EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, sh->qp + sh->qp_u_offset);
    sh->qp_v = (s8)EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, sh->qp + sh->qp_v_offset);
#endif

    if (!sh->single_tile_in_slice_flag)
    {
        for (int i = 0; i < num_tiles_in_slice - 1; ++i)
        {
            evc_bsr_read(bs, &sh->entry_point_offset_minus1[i], pps->tile_offset_lens_minus1 + 1);
        }
    }
    
    /* byte align */
    u32 t0;
    while(!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs, &t0);
        evc_assert_rv(0 == t0, EVC_ERR_MALFORMED_BITSTREAM);
    }
#if TRACE_HLS    
    EVC_TRACE_STR("************ SH  End   ************\n");
    EVC_TRACE_STR("***********************************\n");
#endif
    return EVC_OK;
}

int evcd_eco_sei(EVCD_CTX * ctx, EVC_BSR * bs)
{
    u32 payload_type, payload_size;
    u32 pic_sign[N_C][16];

    /* should be aligned before adding user data */
    evc_assert_rv(EVC_BSR_IS_BYTE_ALIGN(bs), EVC_ERR_UNKNOWN);

    evc_bsr_read(bs, &payload_type, 8);
    evc_bsr_read(bs, &payload_size, 8);

    switch (payload_type)
    {
    case EVC_UD_PIC_SIGNATURE:
        /* read signature (HASH) from bitstream */
        for (int i = 0; i < ctx->pic[0].imgb->np; ++i)
        {
            for (int j = 0; j < payload_size; ++j)
            {
                evc_bsr_read(bs, &pic_sign[i][j], 8);
                ctx->pic_sign[i][j] = pic_sign[i][j];
            }
        }
        ctx->pic_sign_exist = 1;
        break;

    default:
        evc_assert_rv(0, EVC_ERR_UNEXPECTED);
    }

    return EVC_OK;
}

int evcd_eco_affine_mrg_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    int t0 = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.affine_mrg, AFF_MAX_CAND, AFF_MAX_CAND);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("merge affine idx ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return  t0;
}

#if GRAB_STAT
void encd_stat_cu(int x, int y, int cuw, int cuh, int cup, void *ctx, void *core, TREE_CONS tree_cons)
{
    EVCD_CTX *dec_ctx = (EVCD_CTX *)ctx;
    EVCD_CORE *dec_core = (EVCD_CORE *)core;

    int pred_mode = dec_core->pred_mode;
    int mmvd_flag = dec_core->mmvd_flag;
    if (pred_mode > MODE_DIR && pred_mode < MODE_IBC)
    {
        mmvd_flag = 1;
        pred_mode -= 2;
    }

    evc_stat_write_cu_str(x, y, cuw, cuh, "PredMode", pred_mode);
    evc_stat_write_cu_str(x, y, cuw, cuh, "AffineFlag", dec_core->affine_flag);
    evc_stat_write_cu_str(x, y, cuw, cuh, "MMVDFlag", mmvd_flag);
    evc_stat_write_cu_vec(x, y, cuw, cuh, "MV0", dec_core->mv[0][0], dec_core->mv[0][1]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "REF0", dec_core->refi[0]);
    evc_stat_write_cu_vec(x, y, cuw, cuh, "MV1", dec_core->mv[1][0], dec_core->mv[1][1]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "REF1", dec_core->refi[1]);
    evc_stat_write_cu_str(x, y, cuw, cuh, "ats_intra_cu", dec_core->ats_intra_cu);
    evc_stat_write_cu_str(x, y, cuw, cuh, "ats_inter_info", dec_core->ats_inter_info);
    //evc_stat_write_cu_str(x, y, cuw, cuh, "BiBlock", (REFI_IS_VALID(dec_core->refi[1]) && REFI_IS_VALID(dec_core->refi[0])) ? 1 : 0);
    if (evc_check_luma(tree_cons))
    evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma", dec_core->is_coef[Y_C] > 0);
    evc_stat_write_cu_str(x, y, cuw, cuh, "Tile_ID", dec_core->tile_num);
    evc_stat_write_cu_str(x, y, cuw, cuh, "Slice_IDX", dec_ctx->slice_num);
}
#endif
