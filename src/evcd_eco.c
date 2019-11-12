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

#include "evcd_def.h"
#include "evc_tbl.h"

#if ALF
#include <math.h>
#include "wrapper.h"
#include "enc_alf_wrapper.h"
#pragma warning(disable:4018)
#endif
#if GRAB_STAT
#include "evc_debug.h"
#endif

#define SBAC_READ_BIT(bs, sbac)  \
        (sbac)->value = (((sbac)->value << 1) | evc_bsr_read1(bs)) & 0xFFFF;

u32 evcd_sbac_decode_bin(EVC_BSR * bs, EVCD_SBAC * sbac, SBAC_CTX_MODEL * model)
{
    u32  bin, lps;
    u16  cmps, p0, p0_lps, p0_mps;

    p0 = ((*model) >> 1) & PROB_MASK;
    lps = (p0*(sbac->range)) >> MCABAC_PROB_BITS;

    lps = lps < 437 ? 437 : lps;
    cmps = (*model) & 1;

    bin = cmps;

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

    if(sbac->value < sbac->range)
    {
        p0_mps = p0 - ((p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0);
        *model = (p0_mps << 1) + cmps;
#if VARIABLE_RANGE
        if(sbac->range >= HALF_RANGE)
#else
        if(sbac->range >= 0x8000)
#endif
        {
            return bin;
        }
    }
    else
    {
        sbac->value -= sbac->range;
        if(sbac->range < lps)
        {
            p0_mps = p0 - ((p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0);
            *model = (p0_mps << 1) + cmps;
        }
        else
        {
            bin = 1 - bin;
            p0_lps = p0 + ((MAX_PROB - p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0);
            if(p0_lps  > (MAX_PROB>>1))
            {
                cmps = cmps == 1 ? 0 : 1;
                p0_lps = MAX_PROB - p0_lps;
            }
            *model = (p0_lps << 1) + cmps;
        }
        sbac->range = lps;
    }

    do
    {
        sbac->range <<= 1;
        SBAC_READ_BIT(bs, sbac);
#if VARIABLE_RANGE
    } while(sbac->range < HALF_RANGE);
#else
    } while(sbac->range < 0x8000);
#endif

    return bin;
}

static u32 sbac_decode_bin_ep(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    u32 bin;

    sbac->range >>= 1;

    if(sbac->value < sbac->range)
    {
        bin = 0;
    }
    else
    {
        sbac->value -= sbac->range;
        bin = 1;
    }

    sbac->range <<= 1;
    SBAC_READ_BIT(bs, sbac);

    return bin;
}

u32 evcd_sbac_decode_bin_trm(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    sbac->range--;

    if(sbac->value < sbac->range)
    {
#if VARIABLE_RANGE
        if((sbac->range) < HALF_RANGE)
#else
        if((sbac->range) < 0x8000)
#endif
        {
            do
            {
                sbac->range <<= 1;
                SBAC_READ_BIT(bs, sbac);
#if VARIABLE_RANGE
            } while((sbac->range) < HALF_RANGE);
#else
            } while((sbac->range) < 0x8000);
#endif
        }
        return 0;
    }
    else
    {
        sbac->value -= sbac->range;
        sbac->range = 1;

        while(!EVC_BSR_IS_BYTE_ALIGN(bs))
        {
            evc_assert_rv(evc_bsr_read1(bs) == 0, EVC_ERR_MALFORMED_BITSTREAM);
        }
        return 1; /* end of slice */
    }
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

#if ATS_INTER_PROCESS
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
        u8 ats_inter_dir = 0;
        u8 ats_inter_quad = 0;
        u8 ats_inter_pos = 0;
        int size = 1 << (log2_cuw + log2_cuh);
        u8 ctx_ats_inter_flag = sbac->ctx.sps_cm_init_flag == 1 ? (size >= 256 ? 0 : 1) : 0;
        u8 ctx_ats_inter_quad = sbac->ctx.sps_cm_init_flag == 1 ? 2 : 1;
        u8 ctx_ats_inter_dir = sbac->ctx.sps_cm_init_flag == 1 ? (((log2_cuw == log2_cuh) ? 0 : (log2_cuw < log2_cuh ? 1 : 2)) + 3) : 2;
        u8 ctx_ats_inter_pos = sbac->ctx.sps_cm_init_flag == 1 ? 6 : 3;

        EVC_SBAC_CTX * sbac_ctx;
        sbac_ctx = &sbac->ctx;

        ats_inter_flag = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_flag);
        EVC_TRACE_STR("ats_inter_flag ");
        EVC_TRACE_INT(ats_inter_flag);
        EVC_TRACE_STR("\n");

        if (ats_inter_flag)
        {
            if ((mode_vert_quad || mode_hori_quad) && (mode_vert || mode_hori))
            {
                ats_inter_quad = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_quad);
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
                ats_inter_dir = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_dir);
                EVC_TRACE_STR("ats_inter_dir ");
                EVC_TRACE_INT(ats_inter_dir);
                EVC_TRACE_STR("\n");
            }
            else
            {
                ats_inter_dir = (ats_inter_quad && mode_hori_quad) || (!ats_inter_quad && mode_hori);
            }

            ats_inter_pos = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->ats_inter_info + ctx_ats_inter_pos);
            EVC_TRACE_STR("ats_inter_pos ");
            EVC_TRACE_INT(ats_inter_pos);
            EVC_TRACE_STR("\n");
        }
        *ats_inter_info = get_ats_inter_info((ats_inter_quad ? 2 : 0) + (ats_inter_dir ? 1 : 0) + ats_inter_flag, ats_inter_pos);

        return EVC_OK;
    }
}
#endif

static int eco_cbf(EVC_BSR * bs, EVCD_SBAC * sbac, u8 pred_mode, u8 cbf[N_C], int b_no_cbf, int is_sub, int sub_pos, int *cbf_all
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    EVC_SBAC_CTX * sbac_ctx;
    sbac_ctx = &sbac->ctx;

    /* decode allcbf */
    if(pred_mode != MODE_INTRA
#if M50761_CHROMA_NOT_SPLIT 
        && !evc_check_only_intra(tree_cons)
#endif
        )
    {
        if(b_no_cbf == 0 && sub_pos == 0)
        {
            if(evcd_sbac_decode_bin(bs, sbac, sbac_ctx->all_cbf) == 0)
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

        cbf[U_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
        cbf[V_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);

        if (cbf[U_C] + cbf[V_C] == 0 && !is_sub)
        {
            cbf[Y_C] = 1;
        }
        else
        {
            cbf[Y_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 0);
        }
    }
    else
    {
#if M50761_CHROMA_NOT_SPLIT 
        if (evc_check_chroma(tree_cons))
        {
#endif
        cbf[U_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
        cbf[V_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);
#if M50761_CHROMA_NOT_SPLIT 
        }
        else
        {
            cbf[U_C] = cbf[V_C] = 0;
        }
        if (evc_check_luma(tree_cons))
        {
#endif
        cbf[Y_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 0);
#if M50761_CHROMA_NOT_SPLIT 
        }
        else
        {
            cbf[Y_C] = 0;
        }
#endif
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("cbf Y ");
    EVC_TRACE_INT(cbf[Y_C]);
    EVC_TRACE_STR("cbf U ");
    EVC_TRACE_INT(cbf[U_C]);
    EVC_TRACE_STR("cbf V ");
    EVC_TRACE_INT(cbf[V_C]);
    EVC_TRACE_STR("\n");

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
#if CTX_MODEL_FOR_RESIDUAL_IN_BASE
        t0 = ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);
#else
        t0 = sbac->ctx.sps_cm_init_flag == 1 ? ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12) : (ch_type == Y_C ? 0 : 2);
#endif

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

#if ATS_INTRA_PROCESS   
static int evcd_eco_ats_intra_cu(EVC_BSR* bs, EVCD_SBAC* sbac, u8 ctx)
{
    u32 t0;
#if M50632_SIMPLIFICATION_ATS
    t0 = sbac_decode_bin_ep(bs, sbac);
#else
    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_intra_cu + ctx);
#endif
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra CU ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return t0;
}

static int evcd_eco_ats_tu_h(EVC_BSR * bs, EVCD_SBAC * sbac, u8 ctx)
{
    u32 t0;

#if M50632_SIMPLIFICATION_ATS
	t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_tu + ctx);
#else
    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_tu_h + ctx);
#endif
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuH ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return t0;
}

static int evcd_eco_ats_tu_v(EVC_BSR * bs, EVCD_SBAC * sbac, u8 ctx)
{
    u32 t0;

#if M50632_SIMPLIFICATION_ATS
	t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_tu + ctx);
#else
    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ats_tu_v + ctx);
#endif
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("ats intra tuV ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return t0;
}
#endif
#if ADCC
static void parse_positionLastXY(EVC_BSR *bs, EVCD_SBAC *sbac, int* sr_x, int* sr_y, int width, int height, int ch_type)
{
    EVC_SBAC_CTX *sbac_ctx = &sbac->ctx;
    SBAC_CTX_MODEL* cm_x = sbac_ctx->cc_scanr_x + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_SCANR_LUMA : 11));
    SBAC_CTX_MODEL* cm_y = sbac_ctx->cc_scanr_y + (ch_type == Y_C ? 0 : (sbac->ctx.sps_cm_init_flag == 1 ? NUM_CTX_SCANR_LUMA : 11));
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
    // posX
    for (pos_x = 0; pos_x < g_group_idx[width - 1]; pos_x++)
    {
        last = evcd_sbac_decode_bin(bs, sbac, cm_x + blk_offset_x + (pos_x >> shift_x));
        if (!last)
        {
            break;
        }
    }

    // posY
    for (pos_y = 0; pos_y < g_group_idx[height - 1]; pos_y++)
    {
        last = evcd_sbac_decode_bin(bs, sbac, cm_y + blk_offset_y + (pos_y >> shift_y));
        if (!last)
        {
            break;
        }
    }

    // EP-coded part
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

    *sr_x = pos_x;
    *sr_y = pos_y;
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
#if INTRA_PACKAGE
        code_word = sbac_decode_bins_ep_msb(bs, sbac, rparam);
#else
        code_word = sbac_decode_bins_ep(bs, sbac, rparam);
#endif
        symbol = (prefix << rparam) + code_word;
    }
    else
    {
#if INTRA_PACKAGE
        code_word = sbac_decode_bins_ep_msb(bs, sbac, prefix - g_go_rice_range[rparam] + rparam);
#else
        code_word = sbac_decode_bins_ep(bs, sbac, prefix - g_go_rice_range[rparam] + rparam);
#endif
        symbol = (((1 << (prefix - g_go_rice_range[rparam])) + g_go_rice_range[rparam] - 1) << rparam) + code_word;
    }

    return symbol;
}
static int evcd_eco_ccA(EVC_BSR *bs, EVCD_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type)
{
    int width = 1 << log2_w;
    int height = 1 << log2_h;
    int offset0;
    SBAC_CTX_MODEL* cm_gt0;
    SBAC_CTX_MODEL* cm_gtx;
    int scan_type = COEF_SCAN_ZIGZAG;

    int log2_block_size = min(log2_w, log2_h);
    u16 *scan;
    u16 *scan_inv;
    int scan_pos_last = -1;
    int sr_x = 0, sr_y = 0;
    int num_coeff;
    int ipos;
    int last_scan_set;
    int rice_param;
    int sub_set;
    int ctx_gt0 = 0;
    int cg_log2_size = LOG2_CG_SIZE;
    int abs_sum = 0;
    int is_last_nz = 0;
    int pos_last = 0;
    int cnt_gtA = 0;
    int cnt_gtB = 0;
    int ctx_gtA = 0;
    int ctx_gtB = 0;
    int escape_data_present_ingroup = 0;
    int blkpos, sx, sy;
    u32 sig;

    // decode last position
    parse_positionLastXY(bs, sbac, &sr_x, &sr_y, width, height, ch_type);
    int max_num_coef = width * height;

    scan = evc_scan_tbl[scan_type][log2_w - 1][log2_h - 1];
    scan_inv = evc_inv_scan_tbl[scan_type][log2_w - 1][log2_h - 1];

    int last_pos_in_raster = sr_x + sr_y * width;
    int last_pos_in_scan = scan_inv[last_pos_in_raster];
    num_coeff = last_pos_in_scan + 1;
    //===== code significance flag =====
    offset0 = log2_block_size <= 2 ? 0 : NUM_CTX_GT0_LUMA_TU << (EVC_MIN(1, (log2_block_size - 3)));
    if (sbac->ctx.sps_cm_init_flag == 1)
    {
        cm_gt0 = (ch_type == Y_C) ? sbac->ctx.cc_gt0 + offset0 : sbac->ctx.cc_gt0 + NUM_CTX_GT0_LUMA;
        cm_gtx = (ch_type == Y_C) ? sbac->ctx.cc_gtA : sbac->ctx.cc_gtA + NUM_CTX_GTA_LUMA;
    }
    else
    {
        cm_gt0 = (ch_type == Y_C) ? sbac->ctx.cc_gt0 : sbac->ctx.cc_gt0 + 1;
        cm_gtx = (ch_type == Y_C) ? sbac->ctx.cc_gtA : sbac->ctx.cc_gtA + 1;
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

        abs_sum = 0;

        {
            for (; ipos >= sub_pos; ipos--)
            {
                blkpos = scan[ipos];
                sx = blkpos & (width - 1);
                sy = blkpos >> log2_w;

                // sigmap         
                if (ipos == scan_pos_last)
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

                if (!(ipos == scan_pos_last)) // skipping signaling flag for last, we know it is non-zero
                {
                    sig = evcd_sbac_decode_bin(bs, sbac, cm_gt0 + ctx_gt0);
                }
                else
                {
                    sig = 1;
                }
                coef[blkpos] = sig;

                if (sig)
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
                u32 bin;
                int i, idx;
                u32 coef_signs;
                int c2_idx = 0;
                abs_sum = 0;
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
#if M50631_IMPROVEMENT_ADCC_CTXGT12
                        ctx_gtA = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtA_inc(coef, pos[idx], width, height, ch_type) : 0;
#else
                        ctx_gtA = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtA_inc(coef, pos[idx], width, height, ch_type, sr_x, sr_y) : 0;
#endif
                    }
                    bin = evcd_sbac_decode_bin(bs, sbac, cm_gtx + ctx_gtA);
                    coef[pos[idx]] += bin;
                    abs_coef[idx] = bin + 1;
                    if (bin == 1)
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
#if M50631_IMPROVEMENT_ADCC_CTXGT12
                        ctx_gtB = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtB_inc(coef, pos[firstC2FlagIdx], width, height, ch_type) : 0;
#else
                        ctx_gtB = sbac->ctx.sps_cm_init_flag == 1 ? evc_get_ctx_gtB_inc(coef, pos[firstC2FlagIdx], width, height, ch_type, sr_x, sr_y) : 0;
#endif
                    }
                    bin = evcd_sbac_decode_bin(bs, sbac, cm_gtx + ctx_gtB);
                    coef[pos[firstC2FlagIdx]] += bin;
                    abs_coef[firstC2FlagIdx] = bin + 2;
                    if (bin != 0)
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
                            int level;

                            rice_param = get_rice_para(coef, pos[idx], width, height, base_level);
                            level = parse_coef_remain_exgolomb(bs, sbac, rice_param);
                            coef[pos[idx]] = level + base_level;
                            abs_coef[idx] = level + base_level;
                        }
                        if (abs_coef[idx] >= 2)
                        {
                            iFirstCoeff2 = 0;
                        }
                    }
                }
                coef_signs = sbac_decode_bins_ep(bs, sbac, num_nz);
                coef_signs <<= 32 - num_nz;

                for (idx = 0; idx < num_nz; idx++)
                {
                    blkpos = pos[idx];
                    coef[blkpos] = abs_coef[idx];
                    abs_sum += abs_coef[idx];

                    int sign = (int)((coef_signs) >> 31);
                    coef[blkpos] = sign > 0 ? -coef[blkpos] : coef[blkpos];
                    coef_signs <<= 1;
                } // for non-zero coefs within cg
            } // if nnz
        }
    } // for (cg)
    return EVC_OK;
}

#endif

static int evcd_eco_xcoef(EVC_BSR *bs, EVCD_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type
#if ATS_INTER_PROCESS
    , u8 ats_inter_info, int is_intra
#endif
#if ADCC
    , int tool_adcc
#endif
)
{
#if ATS_INTER_PROCESS
    if (is_intra)
    {
        assert(ats_inter_info == 0);
    }
    get_tu_size(ats_inter_info, log2_w, log2_h, &log2_w, &log2_h);
#endif
#if ADCC
    if(tool_adcc)
        evcd_eco_ccA(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1));
    else
#endif
        evcd_eco_run_length_cc(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1));
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

static int evcd_eco_mvp_idx(EVC_BSR * bs, EVCD_SBAC * sbac, int sps_amis_flag)
{
#if ENC_DEC_TRACE
    int idx;
    if(sps_amis_flag == 1)
    {
        idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, NUM_MVP_IDX_CTX, MAX_NUM_MVP);
    }
    else
    {
        idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, 3, 4);
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvp idx ");
    EVC_TRACE_INT(idx);
    EVC_TRACE_STR("\n");

    return idx;
#else
    if(sps_amis_flag == 1)
    {
        return sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, NUM_MVP_IDX_CTX, MAX_NUM_MVP);
    }
    else
    {
        return sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, 3, 4);
    }
#endif
}
#if AFFINE
static int evcd_eco_affine_mvp_idx( EVC_BSR * bs, EVCD_SBAC * sbac )
{
#if AFF_MAX_NUM_MVP == 1
    return 0;
#else
#if ENC_DEC_TRACE
    int idx;
    idx = sbac_read_truncate_unary_sym( bs, sbac, sbac->ctx.affine_mvp_idx, NUM_AFFINE_MVP_IDX_CTX, AFF_MAX_NUM_MVP );

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR( "affine mvp idx " );
    EVC_TRACE_INT( idx );
    EVC_TRACE_STR( "\n" );

    return idx;
#else
    return sbac_read_truncate_unary_sym( bs, sbac, sbac->ctx.affine_mvp_idx, NUM_AFFINE_MVP_IDX_CTX, AFF_MAX_NUM_MVP );
#endif
#endif
}
#endif


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

    temp = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mmvd_merge_idx, NUM_SBAC_CTX_MMVD_MERGE_IDX, MMVD_BASE_MV_NUM); /* mmvd_merge_idx */
    parse_idx = temp * MMVD_MAX_REFINE_NUM + temp_t * (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);

    temp = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mmvd_distance_idx, NUM_SBAC_CTX_MMVD_DIST_IDX, MMVD_DIST_NUM); /* mmvd_distance_idx */
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

    dqp = sbac_read_unary_sym(bs, sbac, sbac_ctx->delta_qp, NUM_DELTA_QP_CTX);

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
#if ATS_INTER_PROCESS
    u8          ats_inter_avail = check_ats_inter_info_coded(1 << core->log2_cuw, 1 << core->log2_cuh, core->pred_mode, ctx->sps.tool_ats);
    int         log2_tuw = core->log2_cuw;
    int         log2_tuh = core->log2_cuh;
#endif

#if AFFINE
    b_no_cbf |= core->pred_mode == MODE_DIR && core->affine_flag;
#endif
    b_no_cbf |= core->pred_mode == MODE_DIR_MMVD;
#if MERGE
    b_no_cbf |= core->pred_mode == MODE_DIR;
#endif
    if(ctx->sps.tool_amis == 0)
        b_no_cbf = 0;

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
#if ATS_INTRA_PROCESS
    u8 ats_intra_cu_on = 0;
    u8 ats_tu_mode = 0;
    u8 is_intra = (core->pred_mode == MODE_INTRA) ? 1 : 0;
#endif

    evc_mset(core->is_coef_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);

    for (j = 0; j < loop_h; j++)
    {
        for (i = 0; i < loop_w; i++)
        {
            if (cbf_all)
            {
                ret = eco_cbf(bs, sbac, core->pred_mode, cbf, b_no_cbf, is_sub, j + i, &cbf_all
#if M50761_CHROMA_NOT_SPLIT
                             , ctx->tree_cons
#endif
                             );
                evc_assert_rv(ret == EVC_OK, ret);
#if DQP
                {
                    int dqp;
                    int qp_i_cb, qp_i_cr;
                    u8 * dqp_used;
                    dqp_used = evc_get_dqp_used(core->x_scu, core->y_scu, ctx->w_scu, ctx->map_dqp_used, ctx->pps.cu_qp_delta_area);
                    if(*dqp_used == DQP_UNUSED)
                    {
                        if (ctx->pps.cu_qp_delta_enabled_flag && (((!(ctx->sps.dquant_flag) || (core->cu_qp_delta_code == 1 && !core->cu_qp_delta_is_coded)) 
                            && (cbf[Y_C] || cbf[U_C] || cbf[V_C])) || (core->cu_qp_delta_code == 2 && !core->cu_qp_delta_is_coded)))
                        {
                            dqp = evcd_eco_dqp(bs);
                            core->qp = GET_QP(ctx->sh.qp_prev, dqp);
                            core->qp_y = GET_LUMA_QP(core->qp);
                            *dqp_used = core->qp;
                            core->cu_qp_delta_is_coded = 1;
                            ctx->sh.qp_prev = core->qp;
                        }
                        else
                        {
                            dqp = 0;
                            core->qp = GET_QP(ctx->sh.qp_prev, dqp);
                            core->qp_y = GET_LUMA_QP(core->qp);
                        }
    
                        qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_u));
                        qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_v));
    
                        core->qp_u = evc_tbl_qp_chroma_ajudst[qp_i_cb] + 6 * (BIT_DEPTH - 8);
                        core->qp_v = evc_tbl_qp_chroma_ajudst[qp_i_cr] + 6 * (BIT_DEPTH - 8);
                    }
                    else
                    {
                        if(cbf[Y_C] || cbf[U_C] || cbf[V_C])
                        {
                            core->qp = *dqp_used;
                            core->qp_y = GET_LUMA_QP(core->qp);
                        }
                        else
                        {
                            dqp = 0;
                            core->qp = GET_QP(ctx->sh.qp_prev, dqp);
                            core->qp_y = GET_LUMA_QP(core->qp);
                        }
                        qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_u));
                        qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_v));
    
                        core->qp_u = evc_tbl_qp_chroma_ajudst[qp_i_cb] + 6 * (BIT_DEPTH - 8);
                        core->qp_v = evc_tbl_qp_chroma_ajudst[qp_i_cr] + 6 * (BIT_DEPTH - 8);
                    }
                }
#endif
#if ATS_INTRA_PROCESS
                if (ctx->sps.tool_ats && cbf[Y_C] && (core->log2_cuw <= 5 && core->log2_cuh <= 5) && is_intra)
                {
#if M50761_CHROMA_NOT_SPLIT
                    evc_assert(!evcd_check_only_inter(ctx));
#endif
                  
#if M50632_SIMPLIFICATION_ATS 
                    ats_intra_cu_on = evcd_eco_ats_intra_cu(bs, sbac, 0);
#else
                    ats_intra_cu_on = evcd_eco_ats_intra_cu(bs, sbac, sbac->ctx.sps_cm_init_flag == 1 ? ((core->log2_cuw > core->log2_cuh) ? core->log2_cuw : core->log2_cuh) - MIN_CU_LOG2 : 0);
#endif
                    ats_tu_mode = 0;
                    if (ats_intra_cu_on)
                    {
#if M50632_SIMPLIFICATION_ATS
						u8 ats_intra_tu_h = evcd_eco_ats_tu_h(bs, sbac, 0);
						u8 ats_intra_tu_v = evcd_eco_ats_tu_v(bs, sbac, 0);
#else
                        u8 ats_intra_tu_h = evcd_eco_ats_tu_h(bs, sbac, is_intra);
                        u8 ats_intra_tu_v = evcd_eco_ats_tu_v(bs, sbac, is_intra);
#endif
                        ats_tu_mode = ((ats_intra_tu_h << 1) | ats_intra_tu_v);
                    }
                }
                else
                {
                    ats_intra_cu_on = 0;
                    ats_tu_mode = 0;
                }
                core->ats_intra_cu = ats_intra_cu_on;
                core->ats_intra_tu_h = (ats_tu_mode >> 1);
                core->ats_intra_tu_v = (ats_tu_mode & 1);
#endif

#if ATS_INTER_PROCESS
                if (ats_inter_avail && (cbf[Y_C] || cbf[U_C] || cbf[V_C]))
                {
#if M50761_CHROMA_NOT_SPLIT
                    evc_assert(!evcd_check_only_intra(ctx));
#endif
                    eco_ats_inter_info(bs, sbac, core->log2_cuw, core->log2_cuh, &core->ats_inter_info, ats_inter_avail);
                }
                else
                {
                    assert(core->ats_inter_info == 0);
                }
#endif
            }
            else
            {
                cbf[Y_C] = cbf[U_C] = cbf[V_C] = 0;
            }

            for (c = 0; c < N_C; c++)
            {
                if (cbf[c])
                {
                    int pos_sub_x = i * (1 << (log2_w_sub - !!c));
                    int pos_sub_y = j * (1 << (log2_h_sub - !!c)) * (stride >> (!!c));

                    if (is_sub)
                    {
                        evc_block_copy(core->coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), coef_temp_buf[c], sub_stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
                        coef_temp[c] = coef_temp_buf[c];
                    }
                    else
                    {
                        coef_temp[c] = core->coef[c];
                    }

                    evcd_eco_xcoef(bs, sbac, coef_temp[c], log2_w_sub - (!!c), log2_h_sub - (!!c), c
#if ATS_INTER_PROCESS
                                    , core->ats_inter_info, core->pred_mode == MODE_INTRA
#endif
#if ADCC
                                    , ctx->sps.tool_adcc
#endif
                    );

                    evc_assert_rv(ret == EVC_OK, ret);

                    core->is_coef_sub[c][(j << 1) | i] = 1;
                    tmp_coef[c] += 1;

                    if (is_sub)
                    {
                        evc_block_copy(coef_temp_buf[c], sub_stride >> (!!c), core->coef[c] + pos_sub_x + pos_sub_y, stride >> (!!c), log2_w_sub - (!!c), log2_h_sub - (!!c));
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
    EVCD_SBAC    * sbac;
    EVC_SBAC_CTX * sbac_ctx;

    sbac = GET_SBAC_DEC(bs);
    sbac_ctx = &sbac->ctx;

    /* Initialization of the internal variables */
#if VARIABLE_RANGE
    sbac->range = MAX_RANGE;
#else
    sbac->range = 0x10000;
#endif
    sbac->value = 0;
#if VARIABLE_RANGE
    for(i = 0; i < RANGE_BITS; i++)
#else
    for(i = 0; i < 16; i++)
#endif
    {
        SBAC_READ_BIT(bs, sbac);
    }

    evc_mset(sbac_ctx, 0x00, sizeof(*sbac_ctx));

    sbac_ctx->sps_cm_init_flag = sps_cm_init_flag;

    /* Initialization of the context models */
    if(sps_cm_init_flag == 1)
    {
        evc_eco_sbac_ctx_initialize(sbac_ctx->cbf, (s16*)init_cbf, NUM_QT_CBF_CTX, slice_type, slice_qp);
        evc_eco_sbac_ctx_initialize(sbac_ctx->all_cbf, (s16*)init_all_cbf, NUM_QT_ROOT_CBF_CTX, slice_type, slice_qp);
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
    else
    {
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
        for(i = 0; i < NUM_MV_RES_CTX; i++) sbac_ctx->mvd[i] = PROB_INIT;
        for(i = 0; i < NUM_REFI_CTX; i++) sbac_ctx->refi[i] = PROB_INIT;
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
        for (i = 0; i < NUM_SBAC_CTX_SKIP_FLAG; i++)  sbac_ctx->skip_flag[i] = PROB_INIT;
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
    
    mpm_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir); /* intra_luma_pred_mpm_flag */

    if(mpm_flag)
    {
        int mpm_idx;
        mpm_idx = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 1); /* intra_luma_pred_mpm_idx */
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

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 2);
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
    if (core->mvr_idx == 0)
    {
        EVCD_SBAC *sbac;
        EVC_BSR   *bs;
        int        direct_mode_flag = 0;

        bs = &ctx->bs;
        sbac = GET_SBAC_DEC(bs);

        direct_mode_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);

        if (direct_mode_flag)
        {
            core->inter_dir = PRED_DIR; 
        }
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("direct_merge ");
        EVC_TRACE_INT(core->inter_dir);
        EVC_TRACE_STR("\n");
    }
}

void evcd_eco_merge_mode_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    if (core->mvr_idx == 0)
    {
        EVCD_SBAC *sbac;
        EVC_BSR   *bs;
        int        merge_mode_flag = 0;

        bs = &ctx->bs;
        sbac = GET_SBAC_DEC(bs);

        merge_mode_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);

        if (merge_mode_flag)
        {
            core->inter_dir = PRED_DIR;
        }
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("direct_merge ");
        EVC_TRACE_INT(core->inter_dir == PRED_DIR ? PRED_DIR : 0);
        EVC_TRACE_STR("\n");
    }
}

void evcd_eco_inter_pred_idc(EVCD_CTX * ctx, EVCD_CORE * core)
{
    int tmp = 0;
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

#if REMOVE_BI_INTERDIR
    if (check_bi_applicability_rdo(ctx->sh.slice_type, 1 << core->log2_cuw, 1 << core->log2_cuh))
#endif
    tmp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 1);
    if (tmp)
    {
        core->inter_dir = PRED_BI;
    }
    else
    {
        tmp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 2);
#if IBC
        core->inter_dir = tmp ? PRED_L1 : core->ibc_flag ? PRED_IBC : PRED_L0;
#else
        core->inter_dir = tmp ? PRED_L1 : PRED_L0;
#endif
    }

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("inter dir ");
    EVC_TRACE_INT(core->inter_dir);
    EVC_TRACE_STR("\n");
}

s8 evcd_eco_split_mode(EVCD_CTX * c, EVC_BSR *bs, EVCD_SBAC *sbac, int cuw, int cuh, const int parent_split, int* same_layer_split,
                        const int node_idx, const int* parent_split_allow, int* curr_split_allow, int qt_depth, int btt_depth, int x, int y)
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
        split_mode = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.btt_split_flag);
        split_mode = split_mode ? SPLIT_QUAD : NO_SPLIT;

        EVC_TRACE_STR("split mode ");
        EVC_TRACE_INT(split_mode);
        EVC_TRACE_STR("\n");

        return split_mode;
    }

    evc_check_split_mode(split_allow, CONV_LOG2(cuw), CONV_LOG2(cuh), 0, 0, 0, c->log2_max_cuwh
                          , parent_split, same_layer_split, node_idx, parent_split_allow, qt_depth, btt_depth
                          , x, y, c->w, c->h
                          , NULL, c->sps.sps_btt_flag
#if M50761_CHROMA_NOT_SPLIT
        , c->tree_cons
#endif
    );

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
    EVC_TRACE_STR("split mode ");
    EVC_TRACE_INT(split_mode);
    EVC_TRACE_STR("\n");

    return split_mode;
}

s8 evcd_eco_suco_flag(EVC_BSR *bs, EVCD_SBAC *sbac, EVCD_CTX *c, EVCD_CORE *core, int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, int parent_suco)
{
    s8 suco_flag = parent_suco;
    int ctx = 0;
    u8 allow_suco = c->sps.sps_suco_flag ? evc_check_suco_cond(cuw, cuh, split_mode, boundary, log2_max_cuwh, c->sps.log2_diff_ctu_size_max_suco_cb_size, c->sps.log2_diff_max_suco_min_suco_cb_size) : 0;

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

#if AFFINE
void evcd_eco_affine_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    core->affine_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_flag + ctx->ctx_flags[CNID_AFFN_FLAG]);

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

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("affine mode ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

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
#endif

void evcd_eco_pred_mode(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;
    int          cuw, cuh;
    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);
    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    /* get pred_mode */
#if IBC
    if (ctx->sh.slice_type != SLICE_I && !(!ctx->sps.ibc_flag && ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2))
#else
    if (ctx->sh.slice_type != SLICE_I && !(ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2
#if M50761_CHROMA_NOT_SPLIT
        && evcd_check_only_inter(ctx)
#endif
        ))
#endif
    {
#if M50761_CHROMA_NOT_SPLIT
        if (!evcd_check_all_preds(ctx))
            core->pred_mode = evcd_check_only_inter(ctx) ? MODE_INTER : MODE_INTRA;
        else
        {
#endif
#if FIX_IBC_PRED_MODE_4x4
        if (ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2)
        {
            core->pred_mode = MODE_INTRA;
        }
        else
        {
#endif
        core->pred_mode = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.pred_mode + ctx->ctx_flags[CNID_PRED_MODE]) ? MODE_INTRA : MODE_INTER;
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("pred mode ");
        EVC_TRACE_INT(core->pred_mode);
        EVC_TRACE_STR("\n");
#if FIX_IBC_PRED_MODE_4x4
        }
#endif
#if M50761_CHROMA_NOT_SPLIT
        }
#endif
#if IBC
        if ( (core->pred_mode != MODE_INTRA 
#if M50761_CHROMA_NOT_SPLIT
            || evcd_check_only_intra(ctx)
#endif
#if FIX_IBC_PRED_MODE_4x4
            || (ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2)
#endif
            ) 
#if M50761_CHROMA_NOT_SPLIT
            && evcd_check_luma(ctx) && !evcd_check_only_inter(ctx)
#endif

            && ctx->sps.ibc_flag && core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
        {
            if (evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ibc_flag + ctx->ctx_flags[CNID_IBC_FLAG])) /* is ibc mode? */
            {
                core->pred_mode = MODE_IBC;
                core->ibc_flag = 1;
                core->mmvd_flag = 0;
                core->affine_flag = 0;
#if ATS_INTER_PROCESS
                core->ats_inter_info = 0;
#endif
            }
#if TRACE_ADDITIONAL_FLAGS
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("ibc pred mode ");
            EVC_TRACE_INT(!!core->ibc_flag);
            EVC_TRACE_STR("ctx ");
            EVC_TRACE_INT(ctx->ctx_flags[CNID_IBC_FLAG]);
            EVC_TRACE_STR("\n");
#endif
        }
#endif
#if !TRACE_ADDITIONAL_FLAGS
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("pred mode ");
        EVC_TRACE_INT(core->pred_mode);
        EVC_TRACE_STR("\n");
#endif
    }
#if IBC
    else if (ctx->sh.slice_type == SLICE_I && ctx->sps.ibc_flag
#if M50761_CHROMA_NOT_SPLIT
        && evcd_check_luma(ctx)
#endif
        )
    {
        core->pred_mode = MODE_INTRA;
        core->mmvd_flag = 0;
        core->affine_flag = 0;

        if (core->log2_cuw <= ctx->sps.ibc_log_max_size && core->log2_cuh <= ctx->sps.ibc_log_max_size)
        {
            if (evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ibc_flag + ctx->ctx_flags[CNID_IBC_FLAG])) /* is ibc mode? */
            {
                core->pred_mode = MODE_IBC;
                core->ibc_flag = 1;
#if ATS_INTER_PROCESS
                core->ats_inter_info = 0;
#endif
            }
#if TRACE_ADDITIONAL_FLAGS
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("IBC pred mode ");
            EVC_TRACE_INT(!!core->ibc_flag);
            EVC_TRACE_STR("ctx ");
            EVC_TRACE_INT(ctx->ctx_flags[CNID_IBC_FLAG]);
            EVC_TRACE_STR("\n");
#endif
        }
    }
#endif
    else /* SLICE_I */
    {
#if M50761_CHROMA_NOT_SPLIT
        evc_assert(!evcd_check_only_inter(ctx));
#endif
        core->pred_mode = MODE_INTRA;
    }
}

void evcd_eco_cu_skip_flag(EVCD_CTX * ctx, EVCD_CORE * core)
{
    EVCD_SBAC *sbac;
    EVC_BSR   *bs;
    int        cu_skip_flag = 0;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    cu_skip_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.skip_flag + ctx->ctx_flags[CNID_SKIP_FLAG]); /* cu_skip_flag */

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("skip flag ");
    EVC_TRACE_INT(cu_skip_flag);
    EVC_TRACE_STR("ctx ");
    EVC_TRACE_INT(ctx->ctx_flags[CNID_SKIP_FLAG]);
    EVC_TRACE_STR("\n");

    if (cu_skip_flag)
    {
        core->pred_mode = MODE_SKIP;
    }
}

#if M50761_CHROMA_NOT_SPLIT
MODE_CONS evcd_eco_mode_constr(EVCD_CTX * ctx)
{
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;
    u32          t0;

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.mode_cons + ctx->ctx_flags[CNID_MODE_CONS]);
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mode_constr ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");
    return t0 ? eOnlyIntra : eOnlyInter;
}
#endif

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
#if AFFINE
    core->affine_flag = 0;
    evc_mset(core->affine_bzero, 0, sizeof(int) * REFP_NUM);
    evc_mset(core->affine_mvd, 0, sizeof(s16) * REFP_NUM * 3 * MV_D);
#endif
#if IBC
    core->ibc_flag = 0;
#endif
    core->mmvd_idx = 0;
    core->mvr_idx = 0;
    core->mmvd_flag = 0;
#if DMVR_FLAG
    core->dmvr_flag = 0;
#endif
#if ATS_INTER_PROCESS
    core->ats_inter_info = 0;
#endif
    core->mvp_idx[REFP_0] = 0;
    core->mvp_idx[REFP_1] = 0;
    core->inter_dir = 0;
    core->bi_idx = 0;
    evc_mset(core->mvd, 0, sizeof(s16) * REFP_NUM * MV_D);

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);
    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu);
  
#if M50761_CHROMA_NOT_SPLIT
    if (!evcd_check_all(ctx))
    {
        evc_assert(evcd_check_only_intra(ctx));
    }
#endif
    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->sh.slice_type, ctx->sps.tool_cm_init
#if IBC
        , ctx->sps.ibc_flag, ctx->sps.ibc_log_max_size
#endif
    );
  
    if (ctx->sh.slice_type != SLICE_I && !(ctx->sps.tool_amis && core->log2_cuw == MIN_CU_LOG2 && core->log2_cuh == MIN_CU_LOG2)
#if M50761_CHROMA_NOT_SPLIT
        && (evcd_check_only_inter(ctx) || evcd_check_all_preds(ctx))
#endif
        )
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
#if AFFINE 
            if (ctx->sps.tool_affine && cuw >= 8 && cuh >= 8)
            {
                evcd_eco_affine_flag(ctx, core); /* affine_flag */
            }

            if (core->affine_flag)
            {
                core->mvp_idx[0] = evcd_eco_affine_mrg_idx(bs, sbac);
            }
            else
#endif
            {
                core->mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac, ctx->sps.tool_amis);
                if (ctx->sps.tool_amis == 0 && ctx->sh.slice_type == SLICE_B)
                {
                    core->mvp_idx[REFP_1] = evcd_eco_mvp_idx(bs, sbac, ctx->sps.tool_amis);
                }
                else
                {
                    core->mvp_idx[REFP_1] = core->mvp_idx[REFP_0];
                }
            }
        }

        core->is_coef[Y_C] = core->is_coef[U_C] = core->is_coef[V_C] = 0;
        evc_mset(core->is_coef_sub, 0, sizeof(int) * N_C * MAX_SUB_TB_NUM);
#if DQP //need to check
        if(ctx->pps.cu_qp_delta_enabled_flag)
        {
            int qp_i_cb, qp_i_cr;
            core->qp = ctx->sh.qp_prev;
            core->qp_y = GET_LUMA_QP(core->qp);

            qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_u));
            qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_v));

            core->qp_u = evc_tbl_qp_chroma_ajudst[qp_i_cb] + 6 * (BIT_DEPTH - 8);
            core->qp_v = evc_tbl_qp_chroma_ajudst[qp_i_cr] + 6 * (BIT_DEPTH - 8);
        }
        else
        {

            int qp_i_cb, qp_i_cr;
            core->qp = ctx->sh.qp;
            core->qp_y = GET_LUMA_QP(core->qp);

            qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_u));
            qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + (ctx->sh.qp - ctx->sh.qp_v));

            core->qp_u = evc_tbl_qp_chroma_ajudst[qp_i_cb] + 6 * (BIT_DEPTH - 8);
            core->qp_v = evc_tbl_qp_chroma_ajudst[qp_i_cr] + 6 * (BIT_DEPTH - 8);
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
            
            if (ctx->sps.tool_amis == 0)
            {
                evcd_eco_direct_mode_flag(ctx, core);
            }
            else
            {
                evcd_eco_merge_mode_flag(ctx, core);
            }
            
            if (core->inter_dir == PRED_DIR)
            {
                if (ctx->sps.tool_amis != 0)
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
#if AFFINE
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
#endif
                        {
                            core->mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac, ctx->sps.tool_amis);
                            core->mvp_idx[REFP_1] = core->mvp_idx[REFP_0];
                        }
                    }
#if MERGE
                    core->pred_mode = MODE_DIR;
#else
#if ADMVP
                    s8 refidx;
#endif
#if ADMVP                            
                    evc_get_mv_dir(ctx->refp[0], ctx->ptr, core->scup + ((1 << (core->log2_cuw - MIN_CU_LOG2)) - 1) + ((1 << (core->log2_cuh - MIN_CU_LOG2)) - 1) * ctx->w_scu, core->scup, ctx->w_scu, ctx->h_scu, core->mv
#if ADMVP
                        , &refidx
#endif
                    );
#else
                    evc_get_mv_dir(ctx->refp[0], ctx->ptr, core->scup + ((1 << (core->log2_cuw - MIN_CU_LOG2)) - 1) + ((1 << (core->log2_cuh - MIN_CU_LOG2)) - 1) * ctx->w_scu, ctx->w_scu, core->mv
#if ADMVP
                        , &refidx
#endif
                    );
#endif
#if ADMVP
                    if (refidx == REFI_INVALID)
                    {
                        core->refi[REFP_0] = REFI_INVALID;
                        core->refi[REFP_1] = REFI_INVALID;
                    }
                    else
                    {
                        core->refi[REFP_0] = 0;
                        core->refi[REFP_1] = 0;
                    }
#else
                    core->refi[REFP_0] = 0;
                    core->refi[REFP_1] = 0;
#endif
#endif                
                }
            }
            else
            {

                evcd_eco_inter_pred_idc(ctx, core); /* inter_pred_idc */

#if AFFINE // affine inter mode
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
#endif
                    if (ctx->sps.tool_amis == 1 && core->inter_dir == PRED_BI)
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
                            if (ctx->sps.tool_amis == 0)
                            {
                                core->refi[inter_dir_idx] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[inter_dir_idx]);
                                core->mvp_idx[inter_dir_idx] = evcd_eco_mvp_idx(bs, sbac, ctx->sps.tool_amis);
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
                    core->mpm, core->avail_lr, core->mpm_ext, core->pims);
            }
            else
            {
                evc_get_mpm_b(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                    &core->mpm_b_list, core->avail_lr, core->mpm_ext, core->pims);
            }

            if (ctx->sps.tool_eipd)
            {
#if M50761_CHROMA_NOT_SPLIT
                if (evcd_check_luma(ctx))
                {
#endif
                core->ipm[0] = evcd_eco_intra_dir(bs, sbac, core->mpm, core->mpm_ext, core->pims);
#if M50761_CHROMA_NOT_SPLIT
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
                if (evcd_check_chroma(ctx))
                {
#endif
                core->ipm[1] = evcd_eco_intra_dir_c(bs, sbac, core->ipm[0]);
#if M50761_CHROMA_NOT_SPLIT
                }
#endif
            }
            else
            {
                core->ipm[0] = evcd_eco_intra_dir_b(bs, sbac, core->mpm_b_list, core->mpm_ext, core->pims);
                core->ipm[1] = core->ipm[0];
            }

            SET_REFI(core->refi, REFI_INVALID, REFI_INVALID);

            core->mv[REFP_0][MV_X] = core->mv[REFP_0][MV_Y] = 0;
            core->mv[REFP_1][MV_X] = core->mv[REFP_1][MV_Y] = 0;
        }
#if IBC
        else if (core->pred_mode == MODE_IBC)
        {
            core->affine_flag = 0;
            SET_REFI(core->refi, REFI_INVALID, REFI_INVALID);

            mvp_idx[REFP_0] = 0;

            evcd_eco_get_mvd(bs, sbac, core->mv[REFP_0]);
            
            core->mv[REFP_1][MV_X] = 0;
            core->mv[REFP_1][MV_Y] = 0;
        }
#endif
        else
        {
            evc_assert_rv(0, EVC_ERR_MALFORMED_BITSTREAM);
        }

        /* clear coefficient buffer */
        evc_mset(core->coef[Y_C], 0, cuw * cuh * sizeof(s16));
        evc_mset(core->coef[U_C], 0, (cuw >> 1) * (cuh >> 1) * sizeof(s16));
        evc_mset(core->coef[V_C], 0, (cuw >> 1) * (cuh >> 1) * sizeof(s16));

        /* parse coefficients */
        ret = evcd_eco_coef(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
    }

    return EVC_OK;
}

int evcd_eco_nalu(EVC_BSR * bs, EVC_NALU * nalu)
{
    //nalu->nal_unit_size = evc_bsr_read(bs, 32);
    nalu->forbidden_zero_bit = evc_bsr_read(bs, 1);
    if (nalu->forbidden_zero_bit != 0)
    {
        printf("malformed bitstream: forbidden_zero_bit != 0\n");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    nalu->nal_unit_type_plus1 = evc_bsr_read(bs, 6);
    nalu->nuh_temporal_id = evc_bsr_read(bs,3);

    nalu->nuh_reserved_zero_5bits = evc_bsr_read(bs, 5);
    if (nalu->nuh_reserved_zero_5bits != 0)
    {
        printf("malformed bitstream: nuh_reserved_zero_5bits != 0");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    nalu->nuh_extension_flag = evc_bsr_read(bs, 1);
    if (nalu->nuh_extension_flag != 0)
    {
        printf("malformed bitstream: nuh_extension_flag != 0");
        return EVC_ERR_MALFORMED_BITSTREAM;
    }

    return EVC_OK;
}

int evcd_eco_rlp(EVC_BSR * bs, EVC_RPL * rpl)
{
    rpl->ref_pic_num = (u32)evc_bsr_read_ue(bs);
    if (rpl->ref_pic_num > 0)
    {
        rpl->ref_pics[0] = (u32)evc_bsr_read_ue(bs);
        if (rpl->ref_pics[0] != 0) rpl->ref_pics[0] *= 1 - ((u32)evc_bsr_read1(bs) << 1);
    }
    for (int i = 1; i < rpl->ref_pic_num; ++i)
    {
        int deltaRefPic = (u32)evc_bsr_read_ue(bs);
        if (deltaRefPic != 0) deltaRefPic *= 1 - ((u32)evc_bsr_read1(bs) << 1);
        rpl->ref_pics[i] = rpl->ref_pics[i - 1] + deltaRefPic;
    }

    return EVC_OK;
}

int evcd_eco_sps(EVC_BSR * bs, EVC_SPS * sps)
{
    sps->sps_seq_parameter_set_id = (u32)evc_bsr_read_ue(bs);
    sps->profile_idc = evc_bsr_read(bs, 7);
    sps->level_idc = evc_bsr_read(bs, 8);
    sps->chroma_format_idc = (u32)evc_bsr_read_ue(bs);
    sps->pic_width_in_luma_samples = (u32)evc_bsr_read_ue(bs);
    sps->pic_height_in_luma_samples = (u32)evc_bsr_read_ue(bs);
    sps->bit_depth_luma_minus8 = (u32)evc_bsr_read_ue(bs);
    sps->bit_depth_chroma_minus8 = (u32)evc_bsr_read_ue(bs);
    sps->sps_btt_flag = (u32)evc_bsr_read1(bs);
    if (sps->sps_btt_flag)
    {
        sps->log2_ctu_size_minus2 = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_ctu_max_11_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_11_min_11_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_11_max_12_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_min_11_min_12_cb_size_minus1 = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_12_max_14_cb_size_minus1 = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_min_12_min_14_cb_size_minus1 = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_11_max_tt_cb_size_minus1 = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_min_11_min_tt_cb_size_minus2 = (u32)evc_bsr_read_ue(bs);
    }
    sps->sps_suco_flag = (u32)evc_bsr_read1(bs);
    if (sps->sps_suco_flag)
    {
        sps->log2_diff_ctu_size_max_suco_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_suco_min_suco_cb_size = (u32)evc_bsr_read_ue(bs);
    }
#if M50632_IMPROVEMENT_SPS
    sps->tool_amis = evc_bsr_read1(bs);
    if(sps->tool_amis)
    {
#if ADMVP
        sps->tool_admvp = evc_bsr_read1(bs);
#endif
#if AFFINE
        sps->tool_affine = evc_bsr_read1(bs);
#endif
        sps->tool_amvr = evc_bsr_read1(bs);
#if DMVR
        sps->tool_dmvr = evc_bsr_read1(bs);
#endif
        sps->tool_mmvd = evc_bsr_read1(bs);
    }

    sps->tool_eipd = evc_bsr_read1(bs);
    if(sps->tool_eipd)
    {
#if IBC
        sps->ibc_flag = evc_bsr_read1(bs);
        if(sps->ibc_flag)
           sps->ibc_log_max_size = (u32)evc_bsr_read_ue(bs) + 2;
#endif
    }

    sps->tool_cm_init = evc_bsr_read1(bs);
    if(sps->tool_cm_init)
    {
#if ADCC
        sps->tool_adcc = evc_bsr_read1(bs);
#endif
    }

    sps->tool_iqt = evc_bsr_read1(bs);
    if(sps->tool_iqt)
    {
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
    sps->tool_ats = evc_bsr_read1(bs);
#endif
    }
    
#if ALF
    sps->tool_alf = evc_bsr_read1(bs);
#endif
#if HTDF
    sps->tool_htdf = evc_bsr_read1(bs);
#endif
#else
    sps->tool_amvr = evc_bsr_read1(bs);
    sps->tool_mmvd = evc_bsr_read1(bs);
#if AFFINE
    sps->tool_affine = evc_bsr_read1(bs);
#endif
#if DMVR
    sps->tool_dmvr = evc_bsr_read1(bs);
#endif
#if ALF
    sps->tool_alf = evc_bsr_read1(bs);
#endif
#if ADMVP
    sps->tool_admvp = evc_bsr_read1(bs);
#endif
    sps->tool_eipd = evc_bsr_read1(bs);
    sps->tool_amis = evc_bsr_read1(bs);
    sps->tool_iqt = evc_bsr_read1(bs);
#if HTDF
    sps->tool_htdf = evc_bsr_read1(bs);
#endif
#if ADCC
    sps->tool_adcc = evc_bsr_read1(bs);
#endif
    sps->tool_cm_init = evc_bsr_read1(bs);
#if IBC
    sps->ibc_flag = evc_bsr_read1(bs);
    if (sps->ibc_flag)
        sps->ibc_log_max_size = (u32)evc_bsr_read_ue(bs) + 2;
#endif
#if ATS_INTRA_PROCESS || ATS_INTER_PROCESS
    sps->tool_ats = evc_bsr_read1(bs);
#endif
#endif
    sps->tool_rpl = evc_bsr_read1(bs);
    sps->tool_pocs = evc_bsr_read1(bs);
#if DQP
    sps->dquant_flag = evc_bsr_read1(bs);
#endif
    if (sps->tool_pocs)
    {
        sps->log2_max_pic_order_cnt_lsb_minus4 = (u32)evc_bsr_read_ue(bs);
    }
    if (!sps->tool_rpl || !sps->tool_pocs)
    {
        sps->log2_sub_gop_length = (u32)evc_bsr_read_ue(bs);
        if (sps->log2_sub_gop_length == 0)
        {
            sps->log2_ref_pic_gap_length = (u32)evc_bsr_read_ue(bs);
        }

    }
    sps->sps_max_dec_pic_buffering_minus1 = (u32)evc_bsr_read_ue(bs);
    if (!sps->tool_rpl)
    {
        sps->max_num_ref_pics = (u32)evc_bsr_read_ue(bs);
    }
    else
    {
        sps->long_term_ref_pics_flag = evc_bsr_read1(bs);
        sps->rpl_candidates_present_flag = (u32)evc_bsr_read1(bs);

        if (sps->rpl_candidates_present_flag)
        {
            sps->rpl1_same_as_rpl0_flag = (u32)evc_bsr_read1(bs);

            sps->rpls_l0_num = (u32)evc_bsr_read_ue(bs);
            for (int i = 0; i < sps->rpls_l0_num; ++i)
                evcd_eco_rlp(bs, &sps->rpls_l0[i]);

            if (!sps->rpl1_same_as_rpl0_flag)
            {
                sps->rpls_l1_num = (u32)evc_bsr_read_ue(bs);
                for (int i = 0; i < sps->rpls_l1_num; ++i)
                    evcd_eco_rlp(bs, &sps->rpls_l1[i]);
            }
            else
            {
                assert(!"hasn't been implemented yet");
                //TBD: Basically copy everything from sps->rpls_l0 to sps->rpls_l1
            }
        }
    }

    sps->picture_cropping_flag = evc_bsr_read1(bs);
    if (sps->picture_cropping_flag)
    {
        sps->picture_crop_left_offset = (u32)evc_bsr_read_ue(bs);
        sps->picture_crop_right_offset = (u32)evc_bsr_read_ue(bs);
        sps->picture_crop_top_offset = (u32)evc_bsr_read_ue(bs);
        sps->picture_crop_bottom_offset = (u32)evc_bsr_read_ue(bs);
    }

    while (!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs);
    }

    return EVC_OK;
}

int evcd_eco_pps(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps)
{
    pps->pps_pic_parameter_set_id = evc_bsr_read_ue(bs);
    pps->pps_seq_parameter_set_id = evc_bsr_read_ue(bs);
    pps->num_ref_idx_default_active_minus1[0] = evc_bsr_read_ue(bs);
    pps->num_ref_idx_default_active_minus1[1] = evc_bsr_read_ue(bs);

    if(sps->long_term_ref_pics_flag)
    {
        pps->additional_lt_poc_lsb_len = evc_bsr_read_ue(bs);
    }

    if(sps->rpl_candidates_present_flag)
    {
        pps->rpl1_idx_present_flag = evc_bsr_read1(bs);
    }

    pps->single_tile_in_pic_flag = evc_bsr_read1(bs);
    if(!pps->single_tile_in_pic_flag)
    {
        pps->num_tile_columns_minus1 = evc_bsr_read_ue(bs);
        pps->num_tile_rows_minus1 = evc_bsr_read_ue(bs);
        pps->uniform_tile_spacing_flag = evc_bsr_read1(bs);
        if(!pps->uniform_tile_spacing_flag)
        {
            for(int i = 0; i < pps->num_tile_columns_minus1; ++i)
            {
                pps->tile_column_width_minus1[i] = evc_bsr_read_ue(bs);
            }
            for(int i = 0; i < pps->num_tile_rows_minus1; ++i)
            {
                pps->tile_row_height_minus1[i] = evc_bsr_read_ue(bs);
            }
        }
        pps->loop_filter_across_tiles_enabled_flag = evc_bsr_read1(bs);
        pps->tile_offset_lens_minus1 = evc_bsr_read1(bs);
    }

    pps->tile_id_len_minus1 = evc_bsr_read_ue(bs);
    pps->explicit_tile_id_flag = evc_bsr_read1(bs);
    if(pps->explicit_tile_id_flag)
    {
        for(int i = 0; i <= pps->num_tile_rows_minus1; ++i)
        {
            for(int j = 0; j <= pps->num_tile_columns_minus1; ++j)
            {
                pps->tile_id_val[i][j] = evc_bsr_read(bs, pps->tile_id_len_minus1 + 1);
            }
        }
    }

    pps->arbitrary_slice_present_flag = evc_bsr_read1(bs);
    pps->constrained_intra_pred_flag = evc_bsr_read1(bs);  

#if DQP
    pps->cu_qp_delta_enabled_flag = evc_bsr_read1(bs);
    if(pps->cu_qp_delta_enabled_flag)
    {
        pps->cu_qp_delta_area = evc_bsr_read_ue(bs) + 6;
    }
#endif

    while(!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs);
    }

    return EVC_OK;
}

#if ALF_PARAMETER_APS
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

#if ALF
int evcd_truncatedUnaryEqProb(EVC_BSR * bs, const int maxSymbol)
{
    for(int k = 0; k < maxSymbol; k++)
    {
        int symbol = evc_bsr_read1(bs);
        if(!symbol)
        {
            return k;
        }
    }
    return maxSymbol;
}

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
        uiSymbol = evc_bsr_read1(bs);
        q++;
    }

    for(a = 0; a < k; ++a)          // read out the sequential log2(M) bits
    {
        uiSymbol = evc_bsr_read1(bs);
        if(uiSymbol)
        {
            nr += 1 << a;
        }
    }
    nr += q * m;                    // add the bits and the multiple of M
    if(nr != 0)
    {
        uiSymbol = evc_bsr_read1(bs);
        nr = (uiSymbol) ? nr : -nr;
    }
    return nr;
}

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
    ruiSymbol = evc_bsr_read(bs, uiThresh); //xReadCode( uiThresh, ruiSymbol );
    if(ruiSymbol >= uiVal - b)
    {
        u32 uiSymbol = evc_bsr_read1(bs); //xReadFlag( uiSymbol );
        ruiSymbol <<= 1;
        ruiSymbol += uiSymbol;
        ruiSymbol -= (uiVal - b);
    }

    return ruiSymbol;
}

int evcd_eco_alf_filter(EVC_BSR * bs, evc_AlfSliceParam* alfSliceParam, const BOOL isChroma)
{
    if(!isChroma)
    {
        alfSliceParam->coeffDeltaFlag = evc_bsr_read1(bs); // "alf_coefficients_delta_flag"
        if(!alfSliceParam->coeffDeltaFlag)
        {
            if(alfSliceParam->numLumaFilters > 1)
            {
                alfSliceParam->coeffDeltaPredModeFlag = evc_bsr_read1(bs); // "coeff_delta_pred_mode_flag"
            }
            else
            {
                alfSliceParam->coeffDeltaPredModeFlag = 0;
            }
        }
        else
        {
            alfSliceParam->coeffDeltaPredModeFlag = 0;
        }
    }

    evc_AlfFilterShape alfShape;
    init_AlfFilterShape( &alfShape, isChroma ? 5 : ( alfSliceParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );

    const int maxGolombIdx = alfShape.filterType == 0 ? 2 : 3;

    int min_golomb_order = evc_bsr_read_ue(bs); // "min_golomb_order"
    int kMin = min_golomb_order + 1;
    int kMinTab[MAX_NUM_ALF_COEFF];

    const int numFilters = isChroma ? 1 : alfSliceParam->numLumaFilters;
    short* coeff = isChroma ? alfSliceParam->chromaCoeff : alfSliceParam->lumaCoeff;

    for(int idx = 0; idx < maxGolombIdx; idx++)
    {
        int code = evc_bsr_read1(bs); //"golomb_order_increase_flag"
        kMinTab[idx] = kMin + code;
        kMin = kMinTab[idx];
    }

    if(!isChroma)
    {
        if(alfSliceParam->coeffDeltaFlag)
        {
            for(int ind = 0; ind < alfSliceParam->numLumaFilters; ++ind)
            {
                int code = evc_bsr_read1(bs); // "filter_coefficient_flag[i]"
                alfSliceParam->filterCoeffFlag[ind] = code;
            }
        }
    }

    // Filter coefficients
    for(int ind = 0; ind < numFilters; ++ind)
    {
        if(!isChroma && !alfSliceParam->filterCoeffFlag[ind] && alfSliceParam->coeffDeltaFlag)
        {
            memset(coeff + ind * MAX_NUM_ALF_LUMA_COEFF, 0, sizeof(*coeff) * alfShape.numCoeff);
            continue;
        }

        for(int i = 0; i < alfShape.numCoeff - 1; i++)
        {
            coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode(bs, kMinTab[alfShape.golombIdx[i]]);
        }
    }

    return EVC_OK;
}
#if ALF_PARAMETER_APS
int evcd_eco_alf_aps_param(EVC_BSR * bs, EVC_APS * aps)
{
    evc_AlfSliceParam* alfSliceParam = &(aps->alf_aps_param);
    //AlfSliceParam reset
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->resetALFBufferFlag = 0;
    alfSliceParam->store2ALFBufferFlag = 0;
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
#if M50662_LUMA_CHROMA_SEPARATE_APS
    alfSliceParam->prevIdxComp[0] = 0;
    alfSliceParam->prevIdxComp[1] = 0;
#endif
    alfSliceParam->tLayer = 0;
    alfSliceParam->isCtbAlfOn = 0;
#if !ALF_CTU_MAP_DYNAMIC
    memset(alfSliceParam->alfCtuEnableFlag, 1, 512 * 3 * sizeof(u8));
#endif
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

    alfSliceParam->enabledFlag[0] = evc_bsr_read1(bs);

    if (!alfSliceParam->enabledFlag[0])
    {
        return EVC_OK;
    }

    int alfChromaIdc = evcd_truncatedUnaryEqProb(bs, 3);
    alfSliceParam->enabledFlag[2] = alfChromaIdc & 1;
    alfSliceParam->enabledFlag[1] = (alfChromaIdc >> 1) & 1;
    {

        alfSliceParam->numLumaFilters = evcd_xReadTruncBinCode(bs, MAX_NUM_ALF_CLASSES) + 1;
        alfSliceParam->lumaFilterType = !(evc_bsr_read1(bs));

        if (alfSliceParam->numLumaFilters > 1)
        {
            for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
            {
                alfSliceParam->filterCoeffDeltaIdx[i] = (short)evcd_xReadTruncBinCode(bs, alfSliceParam->numLumaFilters);  //filter_coeff_delta[i]
            }
        }

        char codetab_pred[3] = { 1, 0, 2 };
        const int iNumFixedFilterPerClass = 16;
        memset(alfSliceParam->fixedFilterIdx, 0, sizeof(alfSliceParam->fixedFilterIdx));
        if (iNumFixedFilterPerClass > 0)
        {
            alfSliceParam->fixedFilterPattern = codetab_pred[alfGolombDecode(bs, 0)];
            if (alfSliceParam->fixedFilterPattern == 2)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    alfSliceParam->fixedFilterIdx[classIdx] = evc_bsr_read1(bs); // "fixed_filter_flag"
                }
            }
            else if (alfSliceParam->fixedFilterPattern == 1)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    //on
                    alfSliceParam->fixedFilterIdx[classIdx] = 1;
                }
            }

            if (alfSliceParam->fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
            {
                for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
                {
                    if (alfSliceParam->fixedFilterIdx[classIdx] > 0)
                    {
                        alfSliceParam->fixedFilterIdx[classIdx] = (int)evcd_xReadTruncBinCode(bs, iNumFixedFilterPerClass) + 1;
                    }
                }
            }
        }

        evcd_eco_alf_filter(bs, &(aps->alf_aps_param), FALSE);
    }


    if (alfChromaIdc)
    {
        alfSliceParam->chromaCtbPresentFlag = (BOOL)evc_bsr_read1(bs);
        if (!(alfSliceParam->temporalAlfFlag))
        {
            evcd_eco_alf_filter(bs, &(aps->alf_aps_param), TRUE);
        }
    }

    return EVC_OK;
}

int evcd_eco_alf_sh_param(EVC_BSR * bs, EVC_SH * sh)
{
    evc_AlfSliceParam* alfSliceParam = &(sh->alf_sh_param);

    //AlfSliceParam reset
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
#if M50662_LUMA_CHROMA_SEPARATE_APS
    alfSliceParam->prevIdxComp[0] = 0;
    alfSliceParam->prevIdxComp[1] = 0;
#endif
    alfSliceParam->tLayer = 0;
    alfSliceParam->resetALFBufferFlag = 0;
    alfSliceParam->store2ALFBufferFlag = 0;
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->isCtbAlfOn = 0;
#if !ALF_CTU_MAP_DYNAMIC
    memset(alfSliceParam->alfCtuEnableFlag, 1, 512 * 3 * sizeof(u8));
#endif
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

    //decode map
    alfSliceParam->isCtbAlfOn = evc_bsr_read1(bs);
#if !APS_ALF_CTU_FLAG
    if (alfSliceParam->isCtbAlfOn)
    {
        for (int i = 0; i < sh->num_ctb; i++)
#if ALF_CTU_MAP_DYNAMIC
            *(alfSliceParam->alfCtuEnableFlag + i) = evc_bsr_read1(bs);
#else
            alfSliceParam->alfCtuEnableFlag[0][i] = evc_bsr_read1(bs);
#endif
    }
#endif
    return EVC_OK;
}
#else
int evcd_eco_alf_sh_param(EVC_BSR * bs, EVC_SH * sh)
{
    evc_AlfSliceParam* alfSliceParam = &(sh->alf_sh_param);
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->resetALFBufferFlag = 0;
    alfSliceParam->store2ALFBufferFlag = 0;
    alfSliceParam->temporalAlfFlag = 0;
    alfSliceParam->prevIdx = 0;
    alfSliceParam->tLayer = 0;
    alfSliceParam->isCtbAlfOn = 0;
#if !ALF_CTU_MAP_DYNAMIC
    memset(alfSliceParam->alfCtuEnableFlag, 1 , 512*3*sizeof(u8));
#endif
    memset(alfSliceParam->enabledFlag, 0, 3*sizeof(BOOL));
    alfSliceParam->lumaFilterType = ALF_FILTER_5;
    memset(alfSliceParam->lumaCoeff, 0, sizeof(short)*325);
    memset(alfSliceParam->chromaCoeff, 0, sizeof(short)*7);
    memset(alfSliceParam->filterCoeffDeltaIdx, 0, sizeof(short)*MAX_NUM_ALF_CLASSES);
    memset(alfSliceParam->filterCoeffFlag, 1, sizeof(BOOL)*25);
    alfSliceParam->numLumaFilters = 1;
    alfSliceParam->coeffDeltaFlag = 0;
    alfSliceParam->coeffDeltaPredModeFlag = 0;
    alfSliceParam->chromaCtbPresentFlag = 0;
    alfSliceParam->fixedFilterPattern = 0;
    memset(alfSliceParam->fixedFilterIdx, 0, sizeof(int)*25);

    alfSliceParam->enabledFlag[0] = evc_bsr_read1(bs);

    if (!alfSliceParam->enabledFlag[0])
    {
        return EVC_OK;
    }

    int alfChromaIdc = evcd_truncatedUnaryEqProb(bs, 3);
    alfSliceParam->enabledFlag[2] = alfChromaIdc & 1;
    alfSliceParam->enabledFlag[1] = (alfChromaIdc >> 1) & 1;
    {
        alfSliceParam->temporalAlfFlag = evc_bsr_read1(bs); // "alf_temporal_enable_flag"
        if ( alfSliceParam->temporalAlfFlag )
        {
            alfSliceParam->prevIdx = evc_bsr_read_ue(bs);  // "alf_temporal_index"
        }
        else
        {
            alfSliceParam->resetALFBufferFlag  = evc_bsr_read1( bs );
            alfSliceParam->store2ALFBufferFlag = evc_bsr_read1( bs );
        }
    }

    if (!alfSliceParam->temporalAlfFlag)
    {

    alfSliceParam->numLumaFilters = evcd_xReadTruncBinCode( bs, MAX_NUM_ALF_CLASSES) + 1;
    alfSliceParam->lumaFilterType = !(evc_bsr_read1(bs));

    if (alfSliceParam->numLumaFilters > 1)
    {
        for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
        {
            alfSliceParam->filterCoeffDeltaIdx[i] = (short)evcd_xReadTruncBinCode( bs, alfSliceParam->numLumaFilters );  //filter_coeff_delta[i]
        }
    }

    char codetab_pred[3] = {1, 0, 2};
    const int iNumFixedFilterPerClass = 16;
    memset(alfSliceParam->fixedFilterIdx, 0, sizeof(alfSliceParam->fixedFilterIdx));
    if(iNumFixedFilterPerClass > 0)
    {
        alfSliceParam->fixedFilterPattern = codetab_pred[alfGolombDecode(bs, 0)];
        if(alfSliceParam->fixedFilterPattern == 2)
        {
            for(int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
                alfSliceParam->fixedFilterIdx[classIdx] = evc_bsr_read1(bs); // "fixed_filter_flag"
            }
        }
        else if(alfSliceParam->fixedFilterPattern == 1)
        {
            for(int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
                //on
                alfSliceParam->fixedFilterIdx[classIdx] = 1;
            }
        }

        if(alfSliceParam->fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
        {
            for(int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
                if(alfSliceParam->fixedFilterIdx[classIdx] > 0)
                {
                    alfSliceParam->fixedFilterIdx[classIdx] = (int)evcd_xReadTruncBinCode(bs, iNumFixedFilterPerClass) + 1;
                }
            }
        }
    }

    evcd_eco_alf_filter(bs, &(sh->alf_sh_param), FALSE);
    }

    if (alfChromaIdc) 
    {
        alfSliceParam->chromaCtbPresentFlag = (BOOL)evc_bsr_read1(bs); 
        if (!(alfSliceParam->temporalAlfFlag))
        {
            evcd_eco_alf_filter(bs, &(sh->alf_sh_param), TRUE);
        }
    }

    //decode map
    alfSliceParam->isCtbAlfOn = evc_bsr_read1(bs);
    if(alfSliceParam->isCtbAlfOn)
    {
        for(int i = 0; i < sh->num_ctb; i++)
#if ALF_CTU_MAP_DYNAMIC
            *(alfSliceParam->alfCtuEnableFlag + i) = evc_bsr_read1(bs);
#else
            alfSliceParam->alfCtuEnableFlag[0][i] = evc_bsr_read1(bs);
#endif
    }
    return EVC_OK;
}
#endif
#endif

int evcd_eco_sh(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_SH * sh)
{
    int NumTilesInSlice = 0;    //TBD according to the spec
    sh->dtr = evc_bsr_read(bs, DTR_BIT_CNT);
    sh->layer_id = evc_bsr_read(bs, 3);
    sh->temporal_mvp_asigned_flag = evc_bsr_read1(bs);
    
    if (sh->temporal_mvp_asigned_flag)
    {
        sh->collocated_from_list_idx = evc_bsr_read1(bs);
        sh->collocated_from_ref_idx = evc_bsr_read1(bs);
        sh->collocated_mvp_source_list_idx = evc_bsr_read1(bs);
    }

    sh->slice_pic_parameter_set_id = evc_bsr_read_ue(bs);
    sh->single_tile_in_slice_flag = evc_bsr_read1(bs);
    sh->first_tile_id = evc_bsr_read(bs, pps->tile_id_len_minus1 + 1);

    if (!sh->single_tile_in_slice_flag)
    {
        if (pps->arbitrary_slice_present_flag)
        {
            sh->arbitrary_slice_flag = evc_bsr_read1(bs);
        }
        if (!sh->arbitrary_slice_flag)
        {
            sh->last_tile_id = evc_bsr_read(bs, pps->tile_id_len_minus1 + 1);
        }
        else
        {
            sh->num_remaining_tiles_in_slice_minus1 = evc_bsr_read_ue(bs);
            for (int i = 0; i < NumTilesInSlice - 1; ++i)
            {
                sh->delta_tile_id_minus1[i] = evc_bsr_read_ue(bs);
            }
        }
    }

    sh->slice_type = evc_bsr_read_ue(bs);
  #if M50632_IMPROVEMENT_MMVD
    if (sps->tool_mmvd && ((sh->slice_type == SLICE_B)||(sh->slice_type == SLICE_P)) )
#else
    if (sps->tool_mmvd && (sh->slice_type == SLICE_B))
#endif
    {
        sh->mmvd_group_enable_flag = evc_bsr_read1(bs);
    }
    else
    {
        sh->mmvd_group_enable_flag = 0;
    }
#if ALF
    if (sps->tool_alf)
    {
        sh->alf_on = evc_bsr_read1(bs);
#if ALF_PARAMETER_APS
        if (sh->alf_on)
        {
#if M50662_LUMA_CHROMA_SEPARATE_APS
            sh->aps_id_y = evc_bsr_read(bs, 5);
            sh->aps_id_ch = evc_bsr_read(bs, 5);
#else
            sh->aps_signaled = evc_bsr_read(bs, 5); // parse APS ID in tile group header
#endif
            evcd_eco_alf_sh_param(bs, sh); // parse ALF map
        }
#else
        if (sh->alf_on)
        {
            evcd_eco_alf_sh_param(bs, sh);
        }
#endif
    }
#endif

    // if (NalUnitType != IDR_NUT)  TBD: NALU types to be implemented
    {
        if (sps->tool_pocs)
        {
            sh->poc = evc_bsr_read(bs, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
        }
    }
    // else 
    {
        if (sps->tool_rpl)
        {
            //L0 candidates signaling
            sh->ref_pic_list_sps_flag[0] = sps->rpl_candidates_present_flag ? evc_bsr_read1(bs) : 0;

            if (sh->ref_pic_list_sps_flag[0])
            {
                if (sps->rpls_l0_num)
                {
                    sh->rpl_l0_idx = evc_bsr_read_ue(bs);
                    memcpy(&sh->rpl_l0, &sps->rpls_l0[sh->rpl_l0_idx], sizeof(sh->rpl_l0)); //TBD: temporal workaround, consider refactoring
                    sh->rpl_l0.poc = sh->poc;
                }
            }
            else
            {
                evcd_eco_rlp(bs, &sh->rpl_l0);
                sh->rpl_l0.poc = sh->poc;
            }

            //L1 candidates signaling
            sh->ref_pic_list_sps_flag[1] = sps->rpl_candidates_present_flag ? evc_bsr_read1(bs) : 0;

            if (sh->ref_pic_list_sps_flag[1])
            {
                if (sps->rpls_l1_num)
                {
                    sh->rpl_l1_idx = evc_bsr_read_ue(bs);
                    memcpy(&sh->rpl_l1, &sps->rpls_l1[sh->rpl_l1_idx], sizeof(sh->rpl_l1)); //TBD: temporal workaround, consider refactoring
                    sh->rpl_l1.poc = sh->poc;
                }
            }
            else
            {
                evcd_eco_rlp(bs, &sh->rpl_l1);
                sh->rpl_l1.poc = sh->poc;
            }
        }
    }

    if (sh->slice_type != SLICE_I)
    {
        sh->num_ref_idx_active_override_flag = evc_bsr_read1(bs);
        if (sh->num_ref_idx_active_override_flag)
        {
            sh->rpl_l0.ref_pic_active_num = (u32)evc_bsr_read_ue(bs) + 1;
            if (sh->slice_type == SLICE_B)
            {
                sh->rpl_l1.ref_pic_active_num = (u32)evc_bsr_read_ue(bs) + 1;
            }
        }
        else
        {
            sh->rpl_l0.ref_pic_active_num = 2;  //Temporarily i set it to 2. this should be set equal to the signalled num_ref_idx_default_active_minus1[0] + 1.
            sh->rpl_l1.ref_pic_active_num = 2;  //Temporarily i set it to 2. this should be set equal to the signalled num_ref_idx_default_active_minus1[1] + 1.
        }
    }

    sh->deblocking_filter_on = evc_bsr_read1(bs);
#if M49023_DBF_IMPROVE
    sh->sh_deblock_alpha_offset = evc_bsr_read_se(bs);
    sh->sh_deblock_beta_offset = evc_bsr_read_se(bs);
#endif
    sh->qp = evc_bsr_read(bs, 6);
    sh->qp_u = sh->qp - evc_bsr_read_se(bs);
    sh->qp_v = sh->qp - evc_bsr_read_se(bs);

    if (!sh->single_tile_in_slice_flag)
    {
        for (int i = 0; i < NumTilesInSlice - 1; ++i)
        {
            sh->entry_point_offset_minus1[i] = evc_bsr_read(bs, pps->tile_offset_lens_minus1 + 1);
        }
    }

    if(sh->slice_type!= SLICE_I)
    {
        /* dptr: delta of presentation temporal reference */
        sh->dptr = evc_bsr_read_se(bs);
    }

    sh->poc = sh->dtr + sh->dptr;

    /* byte align */
    while(!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_assert_rv(0 == evc_bsr_read1(bs), EVC_ERR_MALFORMED_BITSTREAM);
    }
    return EVC_OK;
}

#if AFFINE
int evcd_eco_affine_mrg_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    int t0 = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.affine_mrg, AFF_MAX_CAND, AFF_MAX_CAND);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("merge affine idx ");
    EVC_TRACE_INT(t0);
    EVC_TRACE_STR("\n");

    return  t0;
}
#endif

#if GRAB_STAT
void encd_stat_cu(int x, int y, int cuw, int cuh, int cup, void *ctx, void *core
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
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
#if M50761_CHROMA_NOT_SPLIT
    if (evc_check_luma(tree_cons))
#endif
    evc_stat_write_cu_str(x, y, cuw, cuh, "CBF_luma", dec_core->is_coef[Y_C] > 0);
}
#endif