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

#define evcd_eco_pred_mode(bs, sbac) \
    ((evcd_sbac_decode_bin((bs), (sbac), (sbac)->ctx.pred_mode))? \
    MODE_INTRA : MODE_INTER)    

#define evcd_eco_skip_flag(bs, sbac) \
    (evcd_sbac_decode_bin((bs), (sbac), (sbac)->ctx.skip_flag))

#if MMVD
#define evcd_eco_new_skip_flag(bs, sbac) \
    (evcd_sbac_decode_bin((bs), (sbac), (sbac)->ctx.new_skip_flag))
#endif

#define SBAC_READ_BIT(bs, sbac)  \
        (sbac)->value = (((sbac)->value << 1) | evc_bsr_read1(bs)) & 0xFFFF;

u32 evcd_sbac_decode_bin(EVC_BSR * bs, EVCD_SBAC * sbac, SBAC_CTX_MODEL * model)
{
    u32  bin, lps;
    u16  cmps, p0, p1, p0_lps, p0_mps, p1_lps, p1_mps;
    u8   counter = (*model) >> PROB_COUNTER_BITS;
    char fast_prob_update = (counter < FAST_PROB_UPDATE);

    fast_prob_update = 1;

    p0 = ((*model) >> MPS_SHIFT)& PROB_MASK;
    p1 = ((*model) >> 1) & PROB_MASK;
    if(fast_prob_update)
    {
        counter++;
    }
    lps = ((p0 + p1)*(sbac->range)) >> MPS_SHIFT;

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
        p0_mps = p0 - ((p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0); p1_mps = fast_prob_update ? p0_mps : p1 - ((p1 + MCABAC_OFFSET_1) >> MCABAC_SHIFT_1);
        *model = (p0_mps << MPS_SHIFT) + (p1_mps << 1) + cmps + (counter << PROB_COUNTER_BITS);
        if(sbac->range >= 0x8000)
        {
            return bin;
        }
    }
    else
    {
        sbac->value -= sbac->range;
        if(sbac->range < lps)
        {
            p0_mps = p0 - ((p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0); p1_mps = fast_prob_update ? p0_mps : p1 - ((p1 + MCABAC_OFFSET_1) >> MCABAC_SHIFT_1);
            *model = (p0_mps << MPS_SHIFT) + (p1_mps << 1) + cmps + (counter << PROB_COUNTER_BITS);
        }
        else
        {
            bin = 1 - bin;
            p0_lps = p0 + ((MAX_PROB - p0 + MCABAC_OFFSET_0) >> MCABAC_SHIFT_0);  p1_lps = fast_prob_update ? p0_lps : p1 + ((MAX_PROB - p1 + MCABAC_OFFSET_1) >> MCABAC_SHIFT_1);
            if(p0_lps + p1_lps > MAX_PROB)
            {
                cmps = cmps == 1 ? 0 : 1;
                p0_lps = MAX_PROB - p0_lps;
                p1_lps = MAX_PROB - p1_lps;
            }
            *model = (p0_lps << MPS_SHIFT) + (p1_lps << 1) + cmps + (counter << PROB_COUNTER_BITS);
        }
        sbac->range = lps;
    }

    do
    {
        sbac->range <<= 1;
        SBAC_READ_BIT(bs, sbac);
    } while(sbac->range < 0x8000);

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
        if((sbac->range) < 0x8000)
        {
            do
            {
                sbac->range <<= 1;
                SBAC_READ_BIT(bs, sbac);
            } while((sbac->range) < 0x8000);
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
        return 1; /* end of tile_group */
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
            symbol = evcd_sbac_decode_bin(bs, sbac, &model[ctx_idx > num_ctx - 1 ? num_ctx - 1 : ctx_idx]);
            if(symbol == 0)
            {
                break;
            }
        }
    }
    t32u = ctx_idx;

    return t32u;
}

static int eco_cbf(EVC_BSR * bs, EVCD_SBAC * sbac, u8 pred_mode, u8 cbf[N_C], int b_no_cbf)
{
    EVC_SBAC_CTX * sbac_ctx;

    sbac_ctx = &sbac->ctx;

    /* decode allcbf */
    if(pred_mode != MODE_INTRA)
    {
        if(b_no_cbf == 0)
        {
            if(evcd_sbac_decode_bin(bs, sbac, sbac_ctx->all_cbf) == 0)
            {
                cbf[Y_C] = cbf[U_C] = cbf[V_C] = 0;
                return EVC_OK;
            }
        }

        cbf[U_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
        cbf[V_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);

        if(cbf[U_C] + cbf[V_C] == 0)
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
        cbf[U_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 1);
        cbf[V_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 2);
        cbf[Y_C] = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + 0);
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
        t0 = ((EVC_MIN(prev_level - 1, 5)) << 1) + (ch_type == Y_C ? 0 : 12);

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

static int evcd_eco_xcoef(EVC_BSR *bs, EVCD_SBAC *sbac, s16 *coef, int log2_w, int log2_h, int ch_type)
{
#if TU_MAX_64X64
    if((log2_w > 6) || (log2_h > 6))
    {
        s16 t[MAX_TR_DIM];
        int m, n;
        int nnz_tmp = 0;
        int log_w2 = (log2_w > 6) ? 6 : log2_w;
        int log_h2 = (log2_h > 6) ? 6 : log2_h;
        int log_w_loop2 = (log2_w > 6) ? (1 << (log2_w - 6)) : 1;
        int log_h_loop2 = (log2_h > 6) ? (1 << (log2_h - 6)) : 1;
        EVC_SBAC_CTX *sbac_ctx;
        int nzz_total = 0;
        int stride = (1 << log2_w);
        int stride1 = (1 << log_w2);

        sbac_ctx = &sbac->ctx;

        for(n = 0; n < log_h_loop2; n++)
        {
            for(m = 0; m < log_w_loop2; m++)
            {
                int l;
                s16 * coef_temp = &coef[n * 64 * (1 << log2_w) + m * 64];
                int nzz = 0;

                if((m == (log_w_loop2 - 1)) && (n == (log_h_loop2 - 1)) && (nzz_total == 0))
                {
                    nzz = 1;
                }
                else
                {
                    nzz = evcd_sbac_decode_bin(bs, sbac, sbac_ctx->cbf + (ch_type == Y_C ? 0 : (ch_type == U_C ? 1 : 2)));
                }
                if(nzz)
                {
                    nzz_total = 1;

                    for(l = 0; l < (1 << log_h2); l++)
                    {
                        memset(&t[l*stride1], 0, sizeof(s16)*stride1);
                    }

                    evcd_eco_run_length_cc(bs, sbac, t, log_w2, log_h2, (ch_type == Y_C ? 0 : 1));

                    /* copy backto coefbuf */
                    coef_temp = &coef[n * 64 * stride + m * 64];
                    for(l = 0; l < (1 << log_h2); l++)
                    {
                        memcpy(coef_temp, &t[l*stride1], sizeof(s16)*stride1);
                        coef_temp += stride;
                    }
                }
            }
        }
    }
    else
#endif
    {
        evcd_eco_run_length_cc(bs, sbac, coef, log2_w, log2_h, (ch_type == Y_C ? 0 : 1));
    }

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

static int evcd_eco_mvp_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
#if MAX_NUM_MVP == 1
    return 0;
#else
#if ENC_DEC_TRACE
    int idx;
    idx = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, NUM_MVP_IDX_CTX, MAX_NUM_MVP);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("mvp idx ");
    EVC_TRACE_INT(idx);
    EVC_TRACE_STR("\n");

    return idx;
#else
    return sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvp_idx, NUM_MVP_IDX_CTX, MAX_NUM_MVP);
#endif
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
#if MMVD
static int evcd_eco_mvp_idx_ext(EVC_BSR * bs, EVCD_SBAC * sbac, int type)
{
    int parse_idx = 0;
    int temp = 0;
    int temp_t;
    int dev0;

    if(type == 1)
    {
        if(MMVD_MODE_STEP > 2)
        {
            temp_t = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.ext_mode_idx, MMVD_MODE_STEP, MMVD_MODE_STEP);
        }
        else
        {
            temp_t = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ext_mode_idx);
        }
    }
    else
    {
        temp_t = 0;
    }

    if(temp_t == 1)
    {
        dev0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.ext_mode_dev0);
        temp_t += dev0;
    }

    if(MMVD_BASE_MV_NUM > 2)
    {
        temp = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.base_mvp_idx, NUM_BASE_MVP_IDX_CTX, MMVD_BASE_MV_NUM);
    }
    else
    {
        temp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.base_mvp_idx);
    }
    parse_idx = temp * MMVD_MAX_REFINE_NUM + temp_t * (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);
    temp = sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.step_mvp_idx, NUM_STEP_MVP_IDX_CTX, MMVD_REFINE_STEP);
    parse_idx += (temp * 4);
    temp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.position);
    parse_idx += (temp * 2);
    temp = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.position + 1);
    parse_idx += (temp);

    return parse_idx;
}
#endif

#if AMVR
static int evcd_eco_mvr_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
#if MAX_NUM_MVR > 1
    return sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.mvr_idx, MAX_NUM_MVR, MAX_NUM_MVR);
#else
    return 0;
#endif
}

#if ABP
static int evcd_eco_bi_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
#if MAX_NUM_BI > 1
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
#else
    return 0;
#endif
}
#endif
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

#if AFFINE
    b_no_cbf |= core->pred_mode == MODE_DIR && core->affine_flag;
#endif
#if MMVD
    b_no_cbf |= core->pred_mode == MODE_DIR_MMVD;
#endif
#if MERGE
    b_no_cbf |= core->pred_mode == MODE_DIR;
#endif

    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    ret = eco_cbf(bs, sbac, core->pred_mode, cbf, b_no_cbf);
    evc_assert_rv(ret == EVC_OK, ret);

    if(cbf[Y_C])
    {
        ret = evcd_eco_xcoef(bs, sbac, core->coef[Y_C], core->log2_cuw, core->log2_cuh, Y_C);
        evc_assert_rv(ret == EVC_OK, ret);
        core->is_coef[Y_C] = 1;
    }
    else
    {
        core->is_coef[Y_C] = 0;
    }

    if(cbf[U_C])
    {
        ret = evcd_eco_xcoef(bs, sbac, core->coef[U_C], core->log2_cuw - 1, core->log2_cuh - 1, U_C);
        evc_assert_rv(ret == EVC_OK, ret);
        core->is_coef[U_C] = 1;
    }
    else
    {
        core->is_coef[U_C] = 0;
    }

    if(cbf[V_C])
    {
        ret = evcd_eco_xcoef(bs, sbac, core->coef[V_C], core->log2_cuw - 1, core->log2_cuh - 1, V_C);
        evc_assert_rv(ret == EVC_OK, ret);
        core->is_coef[V_C] = 1;
    }
    else
    {
        core->is_coef[V_C] = 0;
    }

    return EVC_OK;
}

#if CABAC_INIT
void evcd_eco_sbac_reset(EVC_BSR * bs, u8 tile_group_type, u8 tile_group_qp)
#else
void evcd_eco_sbac_reset(EVC_BSR * bs)
#endif
{
    int i;
    EVCD_SBAC    * sbac;
    EVC_SBAC_CTX * sbac_ctx;

    sbac = GET_SBAC_DEC(bs);
    sbac_ctx = &sbac->ctx;

    /* Initialization of the internal variables */
    sbac->range = 0x10000;
    sbac->value = 0;
    for(i = 0; i < 16; i++)
    {
        SBAC_READ_BIT(bs, sbac);
    }

#if CABAC_MSET_ZERO
    evc_mset(sbac_ctx, 0x00, sizeof(*sbac_ctx));
#endif

    /* Initialization of the context models */
#if CABAC_INIT
    evc_eco_sbac_ctx_initialize(sbac_ctx->cbf, (s16*)init_cbf, NUM_QT_CBF_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->all_cbf, (s16*)init_all_cbf, NUM_QT_ROOT_CBF_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->pred_mode, (s16*)init_pred_mode, NUM_PRED_MODE_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->inter_dir, (s16*)init_inter_dir, NUM_INTER_DIR_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->intra_dir, (s16*)init_intra_dir, NUM_INTRA_DIR_CTX, tile_group_type, tile_group_qp);
    for(i = 0; i < NUM_SBAC_CTX_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
    for(i = 0; i < NUM_SBAC_CTX_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
    for(i = 0; i < NUM_SBAC_CTX_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;
#if MMVD
    evc_eco_sbac_ctx_initialize(sbac_ctx->new_skip_flag, (s16*)init_new_skip_flag, NUM_SBAC_CTX_NEW_SKIP_FLAG, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->base_mvp_idx, (s16*)init_base_mvp_idx, NUM_BASE_MVP_IDX_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->step_mvp_idx, (s16*)init_step_mvp_idx, NUM_STEP_MVP_IDX_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->position, (s16*)init_position, NUM_POSITION_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->ext_mode_idx, (s16*)init_ext_mode_idx, MMVD_MODE_STEP, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->ext_mode_dev0, (s16*)init_ext_mode_dev0, MMVD_MODE_SG0, tile_group_type, tile_group_qp);
#endif
    evc_eco_sbac_ctx_initialize(sbac_ctx->mvp_idx, (s16*)init_mvp_idx, NUM_MVP_IDX_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->affine_mvp_idx, (s16*)init_affine_mvp_idx, NUM_AFFINE_MVP_IDX_CTX, tile_group_type, tile_group_qp );
#if AMVR
    evc_eco_sbac_ctx_initialize(sbac_ctx->mvr_idx, (s16*)init_mvr_idx, NUM_MVR_IDX_CTX, tile_group_type, tile_group_qp);
#if ABP
    evc_eco_sbac_ctx_initialize(sbac_ctx->bi_idx, (s16*)init_bi_idx, NUM_BI_IDX_CTX, tile_group_type, tile_group_qp);
#endif
#endif
    evc_eco_sbac_ctx_initialize(sbac_ctx->mvd, (s16*)init_mvd, NUM_MV_RES_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->refi, (s16*)init_refi, NUM_REFI_CTX, tile_group_type, tile_group_qp);
    evc_eco_sbac_ctx_initialize(sbac_ctx->split_mode, (s16*)init_split_mode, NUM_SBAC_CTX_SPLIT_MODE, tile_group_type, tile_group_qp);
#if SUCO
    evc_eco_sbac_ctx_initialize(sbac_ctx->suco_flag, (s16*)init_suco_flag, NUM_SBAC_CTX_SUCO_FLAG, tile_group_type, tile_group_qp);
#endif
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
#if CTX_NEV_SKIP_FLAG
    evc_eco_sbac_ctx_initialize(sbac_ctx->skip_flag, (s16*)init_skip_flag, NUM_SBAC_CTX_SKIP_FLAG, tile_group_type, tile_group_qp);
#else
    sbac_ctx->skip_flag[0] = PROB_INIT;
#endif
#else
    for (i = 0; i < NUM_SBAC_CTX_RUN; i++) sbac_ctx->run[i] = PROB_INIT;
    for (i = 0; i < NUM_SBAC_CTX_LAST; i++) sbac_ctx->last[i] = PROB_INIT;
    for (i = 0; i < NUM_SBAC_CTX_LEVEL; i++) sbac_ctx->level[i] = PROB_INIT;

    for(i = 0; i < NUM_QT_CBF_CTX; i++) sbac_ctx->cbf[i] = PROB_INIT;
    sbac_ctx->all_cbf[0] = PROB_INIT;
    for(i = 0; i < NUM_PRED_MODE_CTX; i++) sbac_ctx->pred_mode[i] = PROB_INIT;
    for(i = 0; i < NUM_INTER_DIR_CTX; i++) sbac_ctx->inter_dir[i] = PROB_INIT;
    for(i = 0; i < NUM_INTRA_DIR_CTX; i++) sbac_ctx->intra_dir[i] = PROB_INIT;
#if MMVD
    for(i = 0; i < NUM_SBAC_CTX_NEW_SKIP_FLAG; i++) sbac_ctx->new_skip_flag[i] = PROB_INIT;
    for(i = 0; i < NUM_BASE_MVP_IDX_CTX; i++) sbac_ctx->base_mvp_idx[i] = PROB_INIT;
    for(i = 0; i < NUM_STEP_MVP_IDX_CTX; i++) sbac_ctx->step_mvp_idx[i] = PROB_INIT;
    for(i = 0; i < NUM_POSITION_CTX; i++) sbac_ctx->position[i] = PROB_INIT;
    for(i = 0; i < MMVD_MODE_STEP; i++) sbac_ctx->ext_mode_idx[i] = PROB_INIT;
    for(i = 0; i < MMVD_MODE_SG0; i++) sbac_ctx->ext_mode_dev0[i] = PROB_INIT;
#endif
    for(i = 0; i < NUM_MVP_IDX_CTX; i++) sbac_ctx->mvp_idx[i] = PROB_INIT;
#if AMVR
    for(i = 0; i < NUM_MVR_IDX_CTX; i++) sbac_ctx->mvr_idx[i] = PROB_INIT;
#if ABP
    for(i = 0; i < NUM_BI_IDX_CTX; i++) sbac_ctx->bi_idx[i] = PROB_INIT;
#endif
#endif
    for(i = 0; i < NUM_MV_RES_CTX; i++) sbac_ctx->mvd[i] = PROB_INIT;
    for(i = 0; i < NUM_REFI_CTX; i++) sbac_ctx->refi[i] = PROB_INIT;
    for(i = 0; i < NUM_SBAC_CTX_SPLIT_MODE; i++) sbac_ctx->split_mode[i] = PROB_INIT;
#if SUCO
    for(i = 0; i < NUM_SBAC_CTX_SUCO_FLAG; i++) sbac_ctx->suco_flag[i] = PROB_INIT;
#endif
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
#if CTX_NEV_SKIP_FLAG
    for(i = 0; i < NUM_SBAC_CTX_SKIP_FLAG; i++) sbac_ctx->skip_flag[i] = PROB_INIT;
#else
    sbac_ctx->skip_flag[0] = PROB_INIT;
#endif
#endif
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

int evcd_eco_intra_dir(EVC_BSR * bs, EVCD_SBAC * sbac, u8 mpm[2], u8 mpm_ext[8], u8 pims[IPD_CNT])
{
    u32 t0;
    int ipm = 0;
#if INTRA_GR
    int i;
    t0 = sbac_read_unary_sym(bs, sbac, sbac->ctx.intra_dir, 2);
    for (i = 0; i<IPD_CNT; i++)
    {
        if (t0== mpm[i])
        {
            ipm = i;
        }
    }
#else

    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir);

    if(t0 == 1)
    {
        ipm = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.intra_dir + 1);
        ipm = mpm[ipm];
    }
    else
    {
        int check = 8;
        t0 = sbac_decode_bin_ep(bs, sbac);
        if(t0 == 1)
        {
            t0 = sbac_decode_bin_ep(bs, sbac); ipm += t0 << 2;
            t0 = sbac_decode_bin_ep(bs, sbac); ipm += t0 << 1;
            t0 = sbac_decode_bin_ep(bs, sbac); ipm += t0;
            ipm = mpm_ext[ipm];
        }
        else
        {
            ipm = intra_mode_read_trunc_binary(IPD_CNT - (check + 2), sbac, bs);
            ipm = pims[2 + check + ipm];
        }
    }

#endif
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
    EVC_TRACE_STR("\n");

    return ipm;
}

int evcd_eco_inter_dir(EVC_BSR * bs, EVCD_SBAC * sbac
#if MMVD
                        , int *direct_idx, int type
#endif
#if AMVR
                        , u8 mvr_idx
#endif
                        , u16 avail_lr)
{
    u32 t0 = 0;

#if AMVR
    if(mvr_idx == 0)
    {
#endif
        t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);
#if AMVR
    }
#endif

    if(t0)
    {
#if MMVD
#if MMVD
        if(direct_idx != NULL)
        {
#endif
            t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.new_skip_flag);
            if(t0 != 0)
            {
                t0 = evcd_eco_mvp_idx_ext(bs, sbac, type);
                *direct_idx = t0;
                return PRED_DIR_MMVD;
            }
#if MMVD
            else
            {
                return PRED_DIR;
            }
        }
        else
#endif
#endif
            return PRED_DIR;
    }
    else
    {
        t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 1);
        if(t0)
        {
            return PRED_BI;
        }
        else
        {
            t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir + 2);
            return t0 ? PRED_L1 : PRED_L0;
        }
    }
}

s8 evcd_eco_split_mode(EVCD_CTX * c, EVC_BSR *bs, EVCD_SBAC *sbac, int cuw, int cuh, const int parent_split, int* same_layer_split,
                        const int node_idx, const int* parent_split_allow, int* curr_split_allow, int qt_depth, int btt_depth, int x, int y)
{
    s8 split_mode = NO_SPLIT;
    int ctx = 0;
    int order_idx = cuw >= cuh ? 0 : 1;
#if !QT_ON_BTT_OFF
    u32 t0;
    int split_allow[SPLIT_CHECK_NUM];
    int i, split_mode_sum;
    int base_ctx;
#endif
    if(cuw < 8 && cuh < 8)
    {
        split_mode = NO_SPLIT;
        return split_mode;
    }
#if QT_ON_BTT_OFF
    split_mode = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode);
    split_mode = split_mode ? SPLIT_QUAD : NO_SPLIT;
#else
    if(cuw > c->max_cuwh / 2 || cuh > c->max_cuwh / 2)
    {
        split_mode = SPLIT_QUAD;
        return split_mode;
    }

    evc_check_split_mode(split_allow, CONV_LOG2(cuw), CONV_LOG2(cuh), 0, 0, 0, c->log2_max_cuwh, c->tgh.layer_id
                          , parent_split, same_layer_split, node_idx, parent_split_allow, qt_depth, btt_depth
#if FBT_ALL
                          , x, y, c->w, c->h
#endif
                          , NULL);
    split_mode_sum = 1;

    for(i = 1; i < SPLIT_CHECK_NUM; i++)
    {
        split_mode_sum += split_allow[evc_split_order[order_idx][i]];
        curr_split_allow[i] = split_allow[i];
    }

    base_ctx = (CONV_LOG2(cuw) - CONV_LOG2(cuh) + SQUARE) * 20 + (CONV_LOG2(cuw > cuh ? cuw : cuh) - 3);
    ctx = base_ctx;

    if(split_mode_sum == 1)
    {
        t0 = 0;
    }
    else
    {
        int num_ctx_flag = 15;
        int num_ctx_dir = 5;
        int dir_ctx_offset = num_ctx_flag;
        int typ_ctx_offset = num_ctx_flag + num_ctx_dir;
        int log2_cuw = CONV_LOG2(cuw);
        int log2_cuh = CONV_LOG2(cuh);
        //split flag
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

        t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx);
        if(!t0)
        {
            split_mode = NO_SPLIT;
        }
        else
        {
            u8 HBT = split_allow[SPLIT_BI_HOR];
            u8 VBT = split_allow[SPLIT_BI_VER];
            u8 HTT = split_allow[SPLIT_TRI_HOR];
            u8 VTT = split_allow[SPLIT_TRI_VER];
            u8 sum = HBT + VBT + HTT + VTT;
            u8 ctx_dir = (log2_cuw - log2_cuh + 2) + dir_ctx_offset;
            u8 ctx_typ = 0 + typ_ctx_offset;
            u8 split_dir, split_typ;

            if(sum == 4)
            {
                split_dir = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_dir);
                split_typ = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_typ);
            }
            else if(sum == 3)
            {
                split_dir = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_dir);
                if(!HBT || !HTT)
                {
                    if(split_dir)
                        split_typ = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_typ);
                    else
                        split_typ = !HBT;
                }
                else// if(!VBT || !VTT)
                {
                    if(!split_dir)
                        split_typ = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_typ);
                    else
                        split_typ = !VBT;
                }
            }
            else if(sum == 2)
            {
                if((HBT && HTT) || (VBT && VTT))
                {
                    split_dir = !HBT;
                    split_typ = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_typ);
                }
                else
                {
                    split_dir = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.split_mode + ctx_dir);

                    if(!HTT && !VTT)
                        split_typ = 0;
                    else if(HBT && VTT)
                        split_typ = split_dir;
                    else if(VBT && HTT)
                        split_typ = !split_dir;
                    else
                        assert(0);
                }
            }
            else // if(sum==1)
            {
                split_dir = (VBT || VTT);
                split_typ = (HTT || VTT);
            }

            if(split_typ == 0) //BT
                split_mode = split_dir ? SPLIT_BI_VER : SPLIT_BI_HOR;
            else
                split_mode = split_dir ? SPLIT_TRI_VER : SPLIT_TRI_HOR;
        }
    }
#endif
    EVC_TRACE_STR("split mode ");
    EVC_TRACE_INT(split_mode);
    EVC_TRACE_STR("\n");

    return split_mode;
}

#if SUCO
s8 evcd_eco_suco_flag(EVC_BSR *bs, EVCD_SBAC *sbac, EVCD_CORE *core, int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, int parent_suco, u8 suco_max, u8 suco_min)
{
    s8 suco_flag = parent_suco;
    int ctx = 0;
    u8 allow_suco = evc_check_suco_cond(cuw, cuh, split_mode, boundary, log2_max_cuwh, suco_max, suco_min);

    if(!allow_suco)
    {
        return suco_flag;
    }
    ctx = CONV_LOG2(EVC_MAX(cuw, cuh)) - 2;
    ctx = (cuw == cuh) ? ctx * 2 : ctx * 2 + 1;
    suco_flag = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.suco_flag + ctx);

    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("suco flag ");
    EVC_TRACE_INT(suco_flag);
    EVC_TRACE_STR("\n");

    return suco_flag;
}
#endif
#if ADMVP
BOOL check_bi_applicability_dec(int tile_group_type, int cuw, int cuh)
{
    BOOL is_applicable = FALSE;
    if ((tile_group_type == TILE_GROUP_B) &&
        !((max(cuw, cuh) < 8 && min(cuw, cuh) < 8))
        )
    {
        is_applicable = TRUE;
    }
    return is_applicable;
}
#endif


int evcd_eco_cu(EVCD_CTX * ctx, EVCD_CORE * core)
{
    s16           mvp[MAX_NUM_MVP][MV_D], mvd[MV_D];
    s8            srefi[REFP_NUM][MAX_NUM_MVP];
    s16           smvp[REFP_NUM][MAX_NUM_MVP][MV_D];
    s8            refi[MAX_NUM_MVP];
    EVCD_SBAC   *sbac;
    EVC_BSR     *bs;
    int           ret, cuw, cuh, inter_dir = 0, mvp_idx[REFP_NUM] = {0, 0};
#if MMVD || MERGE
    s8            refi1[MAX_NUM_MVP] = {0,0,0,0,0,0};
    s16           mvp1[MAX_NUM_MVP][MV_D];
#endif
#if MMVD 
    int           new_skip_flag = 0;
    int           direct_idx = -1;
    int           k;
    int           REF_SET[3][MAX_NUM_ACTIVE_REF_FRAME] = {{0,0,},};
    int           real_mv[MMVD_MODE_GRP * MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM][2][3];
    int           best_candi_idx;
#endif
#if ABP
    u8            bi_idx = BI_NON;
#endif
#if AFFINE
    s16           affine_mvp[MAX_NUM_MVP][VER_NUM][MV_D];

    core->affine_flag = 0;
#endif
#if MMVD
    core->mmvd_idx = 0;
#endif
#if AMVR
    core->mvr_idx = 0;
#endif
#if MMVD
    core->new_skip_flag = 0;
#endif
#if DMVR_FLAG
    core->dmvr_flag = 0;
#endif
    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);
    core->avail_lr = evc_check_nev_avail(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu);

    evc_get_ctx_some_flags(core->x_scu, core->y_scu, cuw, cuh, ctx->w_scu, ctx->map_scu, ctx->map_cu_mode, ctx->ctx_flags, ctx->tgh.tile_group_type);
    /* get pred_mode */
#if (PROFILE_IS_MAIN(PROFILE)) 
#if AMIS
    if (ctx->tgh.tile_group_type != TILE_GROUP_I &&  !(ctx->sps.tool_amis == 1 && core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2))
#else
    if(ctx->tgh.tile_group_type != TILE_GROUP_I && !(core->log2_cuw <= MIN_CU_LOG2 && core->log2_cuh <= MIN_CU_LOG2))
#endif
#else
    if(ctx->tgh.tile_group_type != TILE_GROUP_I)
#endif
    {
#if CTX_NEV_SKIP_FLAG
        if(evcd_sbac_decode_bin(bs, sbac, sbac->ctx.skip_flag + ctx->ctx_flags[CNID_SKIP_FLAG])) /* is skip mode? */
#else
        if(evcd_eco_skip_flag(bs, sbac)) /* is skip mode? */
#endif
        {
            core->pred_mode = MODE_SKIP;
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("skip flag ");
            EVC_TRACE_INT(1);
#if CTX_NEV_SKIP_FLAG
            EVC_TRACE_STR("ctx ");
            EVC_TRACE_INT(ctx->ctx_flags[CNID_SKIP_FLAG]);
#endif
            EVC_TRACE_STR("\n");

#if MMVD
            if(ctx->sps.tool_mmvd)
            {
                new_skip_flag = evcd_eco_new_skip_flag(bs, sbac);

                EVC_TRACE_COUNTER;
                EVC_TRACE_STR("new skip flag ");
                EVC_TRACE_INT(new_skip_flag);
                EVC_TRACE_STR("\n");
                core->new_skip_flag = new_skip_flag;
            }
#endif

#if AFFINE /* skip affine_flag */
            if (cuw >= 8 && cuh >= 8
#if MMVD
                && new_skip_flag == 0
#endif
                && ctx->sps.tool_affine
#if CTX_NEV_AFFINE_FLAG
                && evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_flag + ctx->ctx_flags[CNID_AFFN_FLAG])
#else
                && evcd_eco_affine_flag(bs, sbac)
#endif
                )
                core->affine_flag = 1;
#endif
        }
        else
        {
#if CTX_NEV_PRED_MODE
            core->pred_mode = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.pred_mode + ctx->ctx_flags[CNID_PRED_MODE]) ? MODE_INTRA : MODE_INTER;
#else
            core->pred_mode = evcd_eco_pred_mode(bs, sbac);
#endif
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("skip flag ");
            EVC_TRACE_INT(0);
#if CTX_NEV_SKIP_FLAG
            EVC_TRACE_STR("ctx ");
            EVC_TRACE_INT(ctx->ctx_flags[CNID_SKIP_FLAG]);
#endif
            EVC_TRACE_STR("\n");
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("pred mode ");
            EVC_TRACE_INT(core->pred_mode);
            EVC_TRACE_STR("\n");
#if AMVR
            if((core->pred_mode != MODE_INTRA) && ctx->sps.tool_amvr)
            {
                core->mvr_idx = evcd_eco_mvr_idx(bs, sbac);
            }
#endif
        }
    }
    else /* TILE_GROUP_I */
    {
        core->pred_mode = MODE_INTRA;
    }

    if(core->pred_mode == MODE_INTRA)
    {
        core->avail_cu = evc_get_avail_intra(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, core->log2_cuw, core->log2_cuh, ctx->map_scu);
    }
    else
    {
        core->avail_cu = evc_get_avail_inter(core->x_scu, core->y_scu, ctx->w_scu, ctx->h_scu, core->scup, cuw, cuh, ctx->map_scu);
    }

    /* parse prediction info */
    if(core->pred_mode == MODE_SKIP)
    {
#if AFFINE
        if(!core->affine_flag)
        {
#endif
#if MMVD
            if(ctx->sps.tool_mmvd)
            {
                if(new_skip_flag == 1)
                {
                    best_candi_idx = -1;

                    for(k = 0; k < MAX_NUM_ACTIVE_REF_FRAME; k++)
                    {
                        REF_SET[0][k] = ctx->refp[k][0].ptr;
                        REF_SET[1][k] = ctx->refp[k][1].ptr;
                    }
                    REF_SET[2][0] = ctx->ptr;

                    evc_get_ext_mvp_list(ctx->map_refi, ctx->refp[0], ctx->map_mv, ctx->w_scu, ctx->h_scu, core->scup, core->avail_cu, core->log2_cuw, core->log2_cuh, ctx->tgh.tile_group_type, real_mv, ctx->map_scu, REF_SET, core->avail_lr
#if ADMVP
                                          , core->history_buffer, ctx->sps.tool_admvp
#endif
                    );
                    mvp_idx[REFP_0] = evcd_eco_mvp_idx_ext(bs, sbac, !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));

                    best_candi_idx = mvp_idx[REFP_0];

                    mvp_idx[REFP_0] = 0;
                    mvp[mvp_idx[REFP_0]][MV_X] = real_mv[best_candi_idx][0][MV_X];
                    mvp[mvp_idx[REFP_0]][MV_Y] = real_mv[best_candi_idx][0][MV_Y];
                    refi[mvp_idx[REFP_0]] = real_mv[best_candi_idx][0][2];
#if ADMVP
                    if (check_bi_applicability_dec(ctx->tgh.tile_group_type, cuw, cuh))
#else
                    if(ctx->tgh.tile_group_type == TILE_GROUP_B)
#endif
                    {
                        mvp_idx[REFP_1] = 0;
                        mvp1[mvp_idx[REFP_1]][MV_X] = real_mv[best_candi_idx][1][MV_X];
                        mvp1[mvp_idx[REFP_1]][MV_Y] = real_mv[best_candi_idx][1][MV_Y];
                        refi1[mvp_idx[REFP_1]] = real_mv[best_candi_idx][1][2];
                    }
                }
                else
                {
#if ADMVP
                    if (ctx->sps.tool_admvp == 0)
                    evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_cu
                            , ctx->map_unrefined_mv
                            , core->history_buffer, ctx->sps.tool_admvp
                        );
                    else
#endif
                    evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                        , ctx->map_unrefined_mv
#endif
#if ADMVP
                        , core->history_buffer, ctx->sps.tool_admvp
#endif
                    );

                    mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac);
                    if(ctx->tgh.tile_group_type == TILE_GROUP_B)
                    {
                        mvp_idx[REFP_1] = mvp_idx[REFP_0];
                    }

                    refi[mvp_idx[REFP_0]] = srefi[REFP_0][mvp_idx[REFP_0]];
                    refi1[mvp_idx[REFP_1]] = srefi[REFP_1][mvp_idx[REFP_1]];
                    mvp[mvp_idx[REFP_0]][MV_X] = smvp[REFP_0][mvp_idx[REFP_0]][MV_X];
                    mvp[mvp_idx[REFP_0]][MV_Y] = smvp[REFP_0][mvp_idx[REFP_0]][MV_Y];
                    if(ctx->tgh.tile_group_type == TILE_GROUP_B)
                    {
                        mvp1[mvp_idx[REFP_1]][MV_X] = smvp[REFP_1][mvp_idx[REFP_1]][MV_X];
                        mvp1[mvp_idx[REFP_1]][MV_Y] = smvp[REFP_1][mvp_idx[REFP_1]][MV_Y];
                    }
                }

                core->refi[REFP_0] = refi[mvp_idx[REFP_0]];
                core->refi[REFP_1] = refi1[mvp_idx[REFP_1]];

                core->mv[REFP_0][MV_X] = mvp[mvp_idx[REFP_0]][MV_X];
                core->mv[REFP_0][MV_Y] = mvp[mvp_idx[REFP_0]][MV_Y];
#if ADMVP
                if ((ctx->tgh.tile_group_type == TILE_GROUP_P) || (!check_bi_applicability_dec(ctx->tgh.tile_group_type, cuw, cuh)))
#else
                if (ctx->tgh.tile_group_type == TILE_GROUP_P)
#endif
                {
                    core->refi[REFP_1] = REFI_INVALID;
                    core->mv[REFP_1][MV_X] = 0;
                    core->mv[REFP_1][MV_Y] = 0;
                }
                else
                {
                    core->mv[REFP_1][MV_X] = mvp1[mvp_idx[REFP_1]][MV_X];
                    core->mv[REFP_1][MV_Y] = mvp1[mvp_idx[REFP_1]][MV_Y];
                }
#if MMVD
            }
            else
            {
#if ADMVP
                if (ctx->sps.tool_admvp == 0)
                evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_cu
                        , ctx->map_unrefined_mv
                        , core->history_buffer, ctx->sps.tool_admvp
                    );
                else
#endif
                evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                  ,ctx->map_unrefined_mv
#endif
#if ADMVP
                  , core->history_buffer, ctx->sps.tool_admvp
#endif
                );

                mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac);
                refi[mvp_idx[REFP_0]] = srefi[REFP_0][mvp_idx[REFP_0]];
                core->refi[REFP_0] = refi[mvp_idx[REFP_0]];
                mvp[mvp_idx[REFP_0]][MV_X] = smvp[REFP_0][mvp_idx[REFP_0]][MV_X];
                mvp[mvp_idx[REFP_0]][MV_Y] = smvp[REFP_0][mvp_idx[REFP_0]][MV_Y];
                core->mv[REFP_0][MV_X] = mvp[mvp_idx[REFP_0]][MV_X];
                core->mv[REFP_0][MV_Y] = mvp[mvp_idx[REFP_0]][MV_Y];
#if ADMVP
                if ((ctx->tgh.tile_group_type == TILE_GROUP_P) || (!check_bi_applicability_dec(ctx->tgh.tile_group_type, cuw, cuh)))
#else
                if (ctx->tgh.tile_group_type == TILE_GROUP_P)
#endif
                {
                    core->refi[REFP_1] = REFI_INVALID;
                    core->mv[REFP_1][MV_X] = 0;
                    core->mv[REFP_1][MV_Y] = 0;
                }
                else
                {
                    mvp_idx[REFP_1] = mvp_idx[REFP_0];
                    refi[mvp_idx[REFP_1]] = srefi[REFP_1][mvp_idx[REFP_1]];
                    core->refi[REFP_1] = refi[mvp_idx[REFP_1]];
                    mvp[mvp_idx[REFP_1]][MV_X] = smvp[REFP_1][mvp_idx[REFP_1]][MV_X];
                    mvp[mvp_idx[REFP_1]][MV_Y] = smvp[REFP_1][mvp_idx[REFP_1]][MV_Y];
                    core->mv[REFP_1][MV_X] = mvp[mvp_idx[REFP_1]][MV_X];
                    core->mv[REFP_1][MV_Y] = mvp[mvp_idx[REFP_1]][MV_Y];
                }

            }
#endif
#else
#if INTER_GR
            evc_get_motion(core->scup, REFP_0, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu,  core->avail_cu, srefi[REFP_0], smvp[REFP_0]);
#else
            evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_lr
#if ADMVP
                                 , core->history_buffer
#endif
            );
#endif
            mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac);
            refi[mvp_idx[REFP_0]] = srefi[REFP_0][mvp_idx[REFP_0]];
            core->refi[REFP_0] = refi[mvp_idx[REFP_0]];
            mvp[mvp_idx[REFP_0]][MV_X] = smvp[REFP_0][mvp_idx[REFP_0]][MV_X];
            mvp[mvp_idx[REFP_0]][MV_Y] = smvp[REFP_0][mvp_idx[REFP_0]][MV_Y];
            core->mv[REFP_0][MV_X] = mvp[mvp_idx[REFP_0]][MV_X];
            core->mv[REFP_0][MV_Y] = mvp[mvp_idx[REFP_0]][MV_Y];
#if ADMVP
            if ( (ctx->tgh.tile_group_type == TILE_GROUP_P) || (! check_bi_applicability_dec(ctx->tgh.tile_group_type, cuw, cuh)) )
#else
            if (ctx->tgh.tile_group_type == TILE_GROUP_P)
#endif
            {
                core->refi[REFP_1] = REFI_INVALID;
                core->mv[REFP_1][MV_X] = 0;
                core->mv[REFP_1][MV_Y] = 0;
            }
            else
            {
#if INTER_GR
                evc_get_motion(core->scup, REFP_1, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, core->avail_cu, srefi[REFP_1], smvp[REFP_1]);
                if (ctx->tgh.tile_group_type == TILE_GROUP_B)
                {
                    mvp_idx[REFP_1] = evcd_eco_mvp_idx(bs, sbac);
                }
#else
                mvp_idx[REFP_1] = mvp_idx[REFP_0];
#endif
                refi[mvp_idx[REFP_1]] = srefi[REFP_1][mvp_idx[REFP_1]];
                core->refi[REFP_1] = refi[mvp_idx[REFP_1]];
                mvp[mvp_idx[REFP_1]][MV_X] = smvp[REFP_1][mvp_idx[REFP_1]][MV_X];
                mvp[mvp_idx[REFP_1]][MV_Y] = smvp[REFP_1][mvp_idx[REFP_1]][MV_Y];
                core->mv[REFP_1][MV_X] = mvp[mvp_idx[REFP_1]][MV_X];
                core->mv[REFP_1][MV_Y] = mvp[mvp_idx[REFP_1]][MV_Y];
            }
#endif
#if AFFINE
        }
#endif
        core->is_coef[Y_C] = core->is_coef[U_C] = core->is_coef[V_C] = 0;
    }
    else if(core->pred_mode == MODE_INTER || core->pred_mode == MODE_DIR)
    {
#if ADMVP
        if ((ctx->tgh.tile_group_type == TILE_GROUP_P) || (!check_bi_applicability_dec(ctx->tgh.tile_group_type, cuw, cuh)))
#else
        if (ctx->tgh.tile_group_type == TILE_GROUP_P)
#endif
        {
#if MMVD
            u32 t0 = 0;
#endif
            inter_dir = PRED_L0;
#if MMVD
            if(ctx->sps.tool_mmvd)
            {
#if AMVR
                if(core->mvr_idx == 0)
                {
#endif
                    t0 = evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir);
#if AMVR
                }
#endif
                if(t0)
                {
#if AFFINE
                    if (cuw >= 8 && cuh >= 8 && ctx->sps.tool_affine
#if CTX_NEV_AFFINE_FLAG
                        && evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_flag + ctx->ctx_flags[CNID_AFFN_FLAG])
#else
                        && evcd_eco_affine_flag(bs, sbac)
#endif
                        )
                    {
                        core->affine_flag = 1;
                        inter_dir = AFF_DIR;
                    }
                    else
                    {
#endif
                        t0 = evcd_eco_mvp_idx_ext(bs, sbac, !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr));
                        direct_idx = t0;
                        inter_dir = PRED_DIR_MMVD;
#if AFFINE
                    }
#endif
                }
            }
#endif

#if AFFINE
            if(cuw >= 8 && cuh >= 8
#if AMVR
               && core->mvr_idx == 0
#endif
#if MMVD
               && ctx->sps.tool_mmvd == 0
#endif
               && ctx->sps.tool_affine
               && evcd_sbac_decode_bin(bs, sbac, sbac->ctx.inter_dir))
            {
                core->affine_flag = 1;
                inter_dir = AFF_DIR;
            }
#endif
        }
        else /* if(ctx->tgh.tile_group_type == TILE_GROUP_B) */
        {
#if MMVD
            direct_idx = -1;
#endif
            inter_dir = evcd_eco_inter_dir(bs, sbac
#if MMVD                
                                            , ctx->sps.tool_mmvd ? &direct_idx : NULL
                                            , !(ctx->refp[0][0].ptr == ctx->refp[0][1].ptr)
#endif
#if AMVR
                                            , core->mvr_idx
#endif
                                            , core->avail_lr);
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("inter dir ");
            EVC_TRACE_INT(inter_dir);
            EVC_TRACE_STR("\n");

#if AFFINE
            if(inter_dir == PRED_DIR && cuw >= 8 && cuh >= 8
               && ctx->sps.tool_affine
#if CTX_NEV_AFFINE_FLAG
               && evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_flag + ctx->ctx_flags[CNID_AFFN_FLAG])
#else
               && evcd_eco_affine_flag(bs, sbac)) /* direct affine_flag */
#endif
               )
            {
                inter_dir = AFF_DIR;
                core->pred_mode = MODE_DIR;
                core->affine_flag = 1;
            }
#endif
        }

        if(inter_dir == PRED_DIR)
        {
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
#if MMVD
        else if(inter_dir == PRED_DIR_MMVD)
        {
            core->pred_mode = MODE_DIR_MMVD;
            best_candi_idx = direct_idx;

            for(k = 0; k < MAX_NUM_ACTIVE_REF_FRAME; k++)
            {
                REF_SET[0][k] = ctx->refp[k][0].ptr;
                REF_SET[1][k] = ctx->refp[k][1].ptr;
            }
            REF_SET[2][0] = ctx->ptr;

            evc_get_ext_mvp_list(ctx->map_refi, ctx->refp[0], ctx->map_mv, ctx->w_scu, ctx->h_scu, core->scup, core->avail_cu, core->log2_cuw, core->log2_cuh, ctx->tgh.tile_group_type, real_mv, ctx->map_scu, REF_SET, core->avail_lr
#if ADMVP
                                  , core->history_buffer, ctx->sps.tool_admvp
#endif
            );

            core->mv[REFP_0][MV_X] = real_mv[best_candi_idx][0][MV_X];
            core->mv[REFP_0][MV_Y] = real_mv[best_candi_idx][0][MV_Y];
            core->mv[REFP_1][MV_X] = real_mv[best_candi_idx][1][MV_X];
            core->mv[REFP_1][MV_Y] = real_mv[best_candi_idx][1][MV_Y];

            mvp_idx[REFP_0] = 0;
            mvp_idx[REFP_1] = 0;
            refi[mvp_idx[REFP_0]] = real_mv[best_candi_idx][0][2];
            core->refi[REFP_0] = refi[mvp_idx[REFP_0]];
            if(ctx->tgh.tile_group_type == TILE_GROUP_B)
            {
                refi1[mvp_idx[REFP_1]] = real_mv[best_candi_idx][1][2];
                core->refi[REFP_1] = refi1[mvp_idx[REFP_1]];
            }
#if ADMVP
            if ((ctx->tgh.tile_group_type == TILE_GROUP_P) || (!check_bi_applicability_dec(ctx->tgh.tile_group_type, cuw, cuh)))
#else
            if (ctx->tgh.tile_group_type == TILE_GROUP_P)
#endif
            {
                core->refi[REFP_1] = -1;
            }
        }
#endif
        else
        {
#if AFFINE
            if(inter_dir != AFF_DIR)
#endif
            {
#if AFFINE // affine inter mode
                if(cuw >= 8 && cuh >= 8
#if AFFINE
                   && ctx->sps.tool_affine
#endif
#if AMVR
                   && core->mvr_idx == 0
#endif
#if CTX_NEV_AFFINE_FLAG
                   && evcd_sbac_decode_bin(bs, sbac, sbac->ctx.affine_flag + ctx->ctx_flags[CNID_AFFN_FLAG])
#else
                   && evcd_eco_affine_flag(bs, sbac)) /* inter affine_flag */
#endif
                   )
                {
                    core->affine_flag = 1;
                    core->affine_flag += evcd_eco_affine_mode(bs, sbac);
                    EVC_TRACE_COUNTER;
                    EVC_TRACE_STR("inter affine flag ");
                    EVC_TRACE_INT(core->affine_flag);
                    EVC_TRACE_STR("\n");
                }
                else
                {
                    core->affine_flag = 0;
                    EVC_TRACE_COUNTER;
                    EVC_TRACE_STR("inter affine flag ");
                    EVC_TRACE_INT(0);
                    EVC_TRACE_STR("\n");
#endif
#if ABP
                    if(inter_dir == PRED_BI)
                    {
                        bi_idx = evcd_eco_bi_idx(bs, sbac) + 1;
                    }
#endif
                    /* forward */
                    if(inter_dir == PRED_L0 || inter_dir == PRED_BI)
                    {
#if AMVR
#if ABP
                   if(bi_idx == BI_FL0 || bi_idx == BI_FL1)
                   {
                       core->refi[REFP_0] = evc_get_first_refi(core->scup, REFP_0, ctx->map_refi, ctx->map_mv, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, core->mvr_idx, core->avail_lr
#if DMVR_LAG
                           , ctx->map_unrefined_mv
#endif
#if ADMVP
                           , core->history_buffer
                           , ctx->sps.tool_admvp
#endif
                                                                    );
                   }
                   else
                   {
                            core->refi[REFP_0] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_0]);
                   }
#else
                        core->refi[REFP_0] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_0]);
#endif
                        evc_get_motion_from_mvr(core->mvr_idx, ctx->ptr, core->scup, REFP_0, core->refi[REFP_0], ctx->dpm.num_refp[REFP_0], ctx->map_mv, ctx->map_refi, ctx->refp, \
                                                 cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mvp, refi, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                       ,ctx->map_unrefined_mv
#endif
#if ADMVP
                            , core->history_buffer
                            , ctx->sps.tool_admvp
#endif
                                                 );
                        mvp_idx[REFP_0] = 0;
#if ABP
                        if(bi_idx == BI_FL0) mvd[MV_X] = mvd[MV_Y] = 0;
                        else
                        {
#endif
                            evcd_eco_get_mvd(bs, sbac, mvd);
#if ABP
                        }
#endif

                        core->mv[REFP_0][MV_X] = mvp[mvp_idx[REFP_0]][MV_X] + (mvd[MV_X] << core->mvr_idx);
                        core->mv[REFP_0][MV_Y] = mvp[mvp_idx[REFP_0]][MV_Y] + (mvd[MV_Y] << core->mvr_idx);
#else
                        core->refi[REFP_0] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_0]);
#if INTER_GR
                        evc_get_motion(core->scup, REFP_0, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, core->avail_cu, refi, mvp);
#else
                        evc_get_motion_scaling(ctx->ptr, core->scup, REFP_0, core->refi[REFP_0], ctx->dpm.num_refp[REFP_0], ctx->map_mv, ctx->map_refi, ctx->refp, \
                                                cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mvp, refi, ctx->map_scu, core->avail_lr);
#endif
                        mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac);
                        evcd_eco_get_mvd(bs, sbac, mvd);
                        core->mv[REFP_0][MV_X] = mvp[mvp_idx[REFP_0]][MV_X] + mvd[MV_X];
                        core->mv[REFP_0][MV_Y] = mvp[mvp_idx[REFP_0]][MV_Y] + mvd[MV_Y];
#endif
                    }
                    else
                    {
                        core->refi[REFP_0] = REFI_INVALID;
                        core->mv[REFP_0][MV_X] = 0;
                        core->mv[REFP_0][MV_Y] = 0;
                    }

                    /* backward */
                    if(inter_dir == PRED_L1 || inter_dir == PRED_BI)
                    {
#if AMVR
#if ABP
                        if(bi_idx == BI_FL0 || bi_idx == BI_FL1)
                        {
                            core->refi[REFP_1] = evc_get_first_refi(core->scup, REFP_1, ctx->map_refi, ctx->map_mv, cuw, cuh, ctx->w_scu, ctx->h_scu, ctx->map_scu, core->mvr_idx, core->avail_lr
#if DMVR_LAG
                           , ctx->map_unrefined_mv
#endif
#if ADMVP
                                , core->history_buffer
                                , ctx->sps.tool_admvp
#endif
                                                                    );
                        }
                        else
                        {
                            core->refi[REFP_1] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_1]);
                        }
#else
                        core->refi[REFP_1] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_1]);
#endif
                        evc_get_motion_from_mvr(core->mvr_idx, ctx->ptr, core->scup, REFP_1, core->refi[REFP_1], ctx->dpm.num_refp[REFP_1], ctx->map_mv, ctx->map_refi, ctx->refp, \
                                                 cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mvp, refi, ctx->map_scu, core->avail_lr
#if DMVR_LAG
                       , ctx->map_unrefined_mv
#endif
#if ADMVP
                            , core->history_buffer
                            , ctx->sps.tool_admvp
#endif
                                                 );
                        mvp_idx[REFP_1] = 0;
#if ABP
                        if(bi_idx == BI_FL1) mvd[MV_X] = mvd[MV_Y] = 0;
                        else
                        {
#endif
                            evcd_eco_get_mvd(bs, sbac, mvd);
#if ABP
                        }
#endif
                        core->mv[REFP_1][MV_X] = mvp[mvp_idx[REFP_1]][MV_X] + (mvd[MV_X] << core->mvr_idx);
                        core->mv[REFP_1][MV_Y] = mvp[mvp_idx[REFP_1]][MV_Y] + (mvd[MV_Y] << core->mvr_idx);
#else
                        core->refi[REFP_1] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_1]);
#if INTER_GR
                        evc_get_motion(core->scup, REFP_1, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, core->avail_cu, refi, mvp);
#else
                        evc_get_motion_scaling(ctx->ptr, core->scup, REFP_1, core->refi[REFP_1], ctx->dpm.num_refp[REFP_1], ctx->map_mv, ctx->map_refi, ctx->refp, \
                                                cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, mvp, refi, ctx->map_scu, core->avail_lr);
#endif
                        mvp_idx[REFP_1] = evcd_eco_mvp_idx(bs, sbac);
                        evcd_eco_get_mvd(bs, sbac, mvd);
                        core->mv[REFP_1][MV_X] = mvp[mvp_idx[REFP_1]][MV_X] + mvd[MV_X];
                        core->mv[REFP_1][MV_Y] = mvp[mvp_idx[REFP_1]][MV_Y] + mvd[MV_Y];
#endif
                    }
                    else
                    {
                        core->refi[REFP_1] = REFI_INVALID;
                        core->mv[REFP_1][MV_X] = 0;
                        core->mv[REFP_1][MV_Y] = 0;
                    }
#if AFFINE
                }
#endif
            }
        }
    }
    else if(core->pred_mode == MODE_INTRA)
    {
        evc_get_mpm(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu, 
#if INTRA_GR
            &core->mpm, 
#else
            core->mpm, 
#endif
            core->avail_lr, core->mpm_ext, core->pims);

        core->ipm[0] = evcd_eco_intra_dir(bs, sbac, core->mpm, core->mpm_ext, core->pims);
#if INTRA_GR
        core->ipm[1] = core->ipm[0];
#else
        core->ipm[1] = evcd_eco_intra_dir_c(bs, sbac, core->ipm[0]);
#endif

        SET_REFI(core->refi, REFI_INVALID, REFI_INVALID);

        core->mv[REFP_0][MV_X] = core->mv[REFP_0][MV_Y] = 0;
        core->mv[REFP_1][MV_X] = core->mv[REFP_1][MV_Y] = 0;
    }
    else
    {
        evc_assert_rv(0, EVC_ERR_MALFORMED_BITSTREAM);
    }

    // AFFINE motion derivation
#if AFFINE
    if(core->affine_flag && (core->pred_mode == MODE_SKIP || core->pred_mode == MODE_DIR)) // affine merge motion vector
    {
        s16 aff_mrg_mvp[AFF_MAX_CAND][REFP_NUM][VER_NUM][MV_D];
        s8  aff_refi[AFF_MAX_CAND][REFP_NUM];
        int vertex_num[AFF_MAX_CAND];
        int vertex, lidx;
        int mrg_idx = 0;

        mrg_idx = evcd_eco_affine_mrg_idx(bs, sbac);
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("merge affine idx ");
        EVC_TRACE_INT(mrg_idx);
        EVC_TRACE_STR("\n");

        evc_get_affine_merge_candidate(ctx->ptr, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp, cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, aff_refi, aff_mrg_mvp, vertex_num, ctx->map_scu, ctx->map_affine
#if DMVR_LAG
            ,ctx->map_unrefined_mv
#endif
        );

        core->affine_flag = vertex_num[mrg_idx] - 1;
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("merge affine flag ");
        EVC_TRACE_INT(core->affine_flag);
        EVC_TRACE_STR("\n");

        for(lidx = 0; lidx < REFP_NUM; lidx++)
        {
            if(REFI_IS_VALID(aff_refi[mrg_idx][lidx]))
            {
                core->refi[lidx] = aff_refi[mrg_idx][lidx];
                for(vertex = 0; vertex < vertex_num[mrg_idx]; vertex++)
                {
                    core->affine_mv[lidx][vertex][MV_X] = aff_mrg_mvp[mrg_idx][lidx][vertex][MV_X];
                    core->affine_mv[lidx][vertex][MV_Y] = aff_mrg_mvp[mrg_idx][lidx][vertex][MV_Y];
                }
            }
            else
            {
                core->refi[lidx] = REFI_INVALID;
                core->mv[lidx][MV_X] = 0;
                core->mv[lidx][MV_Y] = 0;
            }
        }
    }
    else if(core->affine_flag && core->pred_mode == MODE_INTER) // affine inter motion vector
    {
        int vertex;
        int vertex_num = core->affine_flag + 1;

        /* forward */
        if(inter_dir == PRED_L0 || inter_dir == PRED_BI)
        {
            int bzero;
            core->refi[REFP_0] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_0]);
            evc_get_affine_motion_scaling(ctx->ptr, core->scup, REFP_0, core->refi[REFP_0],
                                           ctx->dpm.num_refp[REFP_0], ctx->map_mv, ctx->map_refi, ctx->refp, \
                                           cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, affine_mvp, refi
                                           , ctx->map_scu, ctx->map_affine, vertex_num, core->avail_lr
#if DMVR_LAG
                ,ctx->map_unrefined_mv
#endif
            );
            mvp_idx[REFP_0] = evcd_eco_affine_mvp_idx( bs, sbac );
            bzero = evcd_eco_affine_mvd_flag(bs, sbac, REFP_0);
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                if(bzero)
                {
                    mvd[MV_X] = 0;
                    mvd[MV_Y] = 0;
                }
                else
                {
                    evcd_eco_get_mvd(bs, sbac, mvd);
                }

                core->affine_mv[REFP_0][vertex][MV_X] = affine_mvp[mvp_idx[REFP_0]][vertex][MV_X] + mvd[MV_X];
                core->affine_mv[REFP_0][vertex][MV_Y] = affine_mvp[mvp_idx[REFP_0]][vertex][MV_Y] + mvd[MV_Y];
#if AFFINE_MVD_PREDICTION
                if (vertex == 0)
                {
                    affine_mvp[mvp_idx[REFP_0]][1][MV_X] += mvd[MV_X];
                    affine_mvp[mvp_idx[REFP_0]][1][MV_Y] += mvd[MV_Y];
                    affine_mvp[mvp_idx[REFP_0]][2][MV_X] += mvd[MV_X];
                    affine_mvp[mvp_idx[REFP_0]][2][MV_Y] += mvd[MV_Y];
                }
#endif
            }
        }
        else
        {
            core->refi[REFP_0] = REFI_INVALID;
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                core->affine_mv[REFP_0][vertex][MV_X] = 0;
                core->affine_mv[REFP_0][vertex][MV_Y] = 0;
            }

            core->refi[REFP_0] = REFI_INVALID;
            core->mv[REFP_0][MV_X] = 0;
            core->mv[REFP_0][MV_Y] = 0;
        }

        /* backward */
        if(inter_dir == PRED_L1 || inter_dir == PRED_BI)
        {
            int bzero;
            core->refi[REFP_1] = evcd_eco_refi(bs, sbac, ctx->dpm.num_refp[REFP_1]);
            evc_get_affine_motion_scaling(ctx->ptr, core->scup, REFP_1, core->refi[REFP_1],
                                           ctx->dpm.num_refp[REFP_1], ctx->map_mv, ctx->map_refi, ctx->refp, \
                                           cuw, cuh, ctx->w_scu, ctx->h_scu, core->avail_cu, affine_mvp, refi
                                           , ctx->map_scu, ctx->map_affine, vertex_num, core->avail_lr
#if DMVR_LAG
                , ctx->map_unrefined_mv
#endif
            );
            mvp_idx[REFP_1] = evcd_eco_affine_mvp_idx( bs, sbac );
            bzero = evcd_eco_affine_mvd_flag(bs, sbac, REFP_1);
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                if(bzero)
                {
                    mvd[MV_X] = 0;
                    mvd[MV_Y] = 0;
                }
                else
                {
                    evcd_eco_get_mvd(bs, sbac, mvd);
                }

                core->affine_mv[REFP_1][vertex][MV_X] = affine_mvp[mvp_idx[REFP_1]][vertex][MV_X] + mvd[MV_X];
                core->affine_mv[REFP_1][vertex][MV_Y] = affine_mvp[mvp_idx[REFP_1]][vertex][MV_Y] + mvd[MV_Y];
#if AFFINE_MVD_PREDICTION
                if (vertex == 0)
                {
                    affine_mvp[mvp_idx[REFP_1]][1][MV_X] += mvd[MV_X];
                    affine_mvp[mvp_idx[REFP_1]][1][MV_Y] += mvd[MV_Y];
                    affine_mvp[mvp_idx[REFP_1]][2][MV_X] += mvd[MV_X];
                    affine_mvp[mvp_idx[REFP_1]][2][MV_Y] += mvd[MV_Y];
                }
#endif
            }
        }
        else
        {
            core->refi[REFP_1] = REFI_INVALID;
            for(vertex = 0; vertex < vertex_num; vertex++)
            {
                core->affine_mv[REFP_1][vertex][MV_X] = 0;
                core->affine_mv[REFP_1][vertex][MV_Y] = 0;
            }
            core->refi[REFP_1] = REFI_INVALID;
            core->mv[REFP_1][MV_X] = 0;
            core->mv[REFP_1][MV_Y] = 0;
        }
    }

#if AFFINE_UPDATE
    core->refi_sp[REFP_0] = REFI_INVALID;
    core->refi_sp[REFP_1] = REFI_INVALID;

    core->mv_sp[REFP_0][MV_X] = 0;
    core->mv_sp[REFP_0][MV_Y] = 0;
    core->mv_sp[REFP_1][MV_X] = 0;
    core->mv_sp[REFP_1][MV_Y] = 0;

    if (core->affine_flag)
    {
        int neb_addr[MAX_NUM_POSSIBLE_SCAND], valid_flag[MAX_NUM_POSSIBLE_SCAND];

        for (k = 0; k < MAX_NUM_POSSIBLE_SCAND; k++)
        {
            valid_flag[k] = 0;
        }
#if ADMVP
        evc_check_motion_availability2(core->scup, cuw, cuh, ctx->w_scu, ctx->h_scu, neb_addr, valid_flag, ctx->map_scu, core->avail_lr, 1);
#else
        evc_check_motion_availability(core->scup, cuw, cuh, ctx->w_scu, ctx->h_scu, neb_addr, valid_flag, ctx->map_scu, core->avail_lr, 1);
#endif

        for (k = 0; k < 5; k++)
        {
            if (valid_flag[k])
            {
                core->refi_sp[REFP_0] = REFI_IS_VALID(ctx->map_refi[neb_addr[k]][REFP_0]) ? ctx->map_refi[neb_addr[k]][REFP_0] : REFI_INVALID;
                core->mv_sp[REFP_0][MV_X] = ctx->map_mv[neb_addr[k]][REFP_0][MV_X];
                core->mv_sp[REFP_0][MV_Y] = ctx->map_mv[neb_addr[k]][REFP_0][MV_Y];

                if (ctx->tgh.tile_group_type == TILE_GROUP_B)
                {
                    core->refi_sp[REFP_1] = REFI_IS_VALID(ctx->map_refi[neb_addr[k]][REFP_1]) ? ctx->map_refi[neb_addr[k]][REFP_1] : REFI_INVALID;
                    core->mv_sp[REFP_1][MV_X] = ctx->map_mv[neb_addr[k]][REFP_1][MV_X];
                    core->mv_sp[REFP_1][MV_Y] = ctx->map_mv[neb_addr[k]][REFP_1][MV_Y];
                }

                break;
            }
        }
    }
#endif
#endif

#if MERGE
    if(inter_dir == PRED_DIR
#if AMVR
       && core->mvr_idx == 0
#endif
       )
    {
#if ADMVP
        if (ctx->sps.tool_admvp == 0)
        evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_cu
                , ctx->map_unrefined_mv
                , core->history_buffer, ctx->sps.tool_admvp
            );
        else
#endif
        evc_get_motion_skip(ctx->ptr, ctx->tgh.tile_group_type, core->scup, ctx->map_refi, ctx->map_mv, ctx->refp[0], cuw, cuh, ctx->w_scu, ctx->h_scu, srefi, smvp, ctx->map_scu, core->avail_lr
#if DMVR_LAG
          ,ctx->map_unrefined_mv
#endif
#if ADMVP
          , core->history_buffer, ctx->sps.tool_admvp
#endif
        );

        mvp_idx[REFP_0] = evcd_eco_mvp_idx(bs, sbac);
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("merge mvp index ");
        EVC_TRACE_INT(mvp_idx[REFP_0]);
        EVC_TRACE_STR("\n");

        mvp_idx[REFP_1] = mvp_idx[REFP_0];
        mvp[mvp_idx[REFP_0]][MV_X] = smvp[REFP_0][mvp_idx[REFP_0]][MV_X];
        mvp[mvp_idx[REFP_0]][MV_Y] = smvp[REFP_0][mvp_idx[REFP_0]][MV_Y];
        refi[mvp_idx[REFP_0]] = srefi[REFP_0][mvp_idx[REFP_0]];
        if(ctx->tgh.tile_group_type == TILE_GROUP_B)
        {
            mvp1[mvp_idx[REFP_1]][MV_X] = smvp[REFP_1][mvp_idx[REFP_1]][MV_X];
            mvp1[mvp_idx[REFP_1]][MV_Y] = smvp[REFP_1][mvp_idx[REFP_1]][MV_Y];
            refi1[mvp_idx[REFP_1]] = srefi[REFP_1][mvp_idx[REFP_1]];
        }

        core->refi[REFP_0] = refi[mvp_idx[REFP_0]];
        core->refi[REFP_1] = refi1[mvp_idx[REFP_1]];

        core->mv[REFP_0][MV_X] = mvp[mvp_idx[REFP_0]][MV_X];
        core->mv[REFP_0][MV_Y] = mvp[mvp_idx[REFP_0]][MV_Y];
        if(ctx->tgh.tile_group_type == TILE_GROUP_P)
        {
            core->refi[REFP_1] = REFI_INVALID;
            core->mv[REFP_1][MV_X] = 0;
            core->mv[REFP_1][MV_Y] = 0;
        }
        else
        {
            core->mv[REFP_1][MV_X] = mvp1[mvp_idx[REFP_1]][MV_X];
            core->mv[REFP_1][MV_Y] = mvp1[mvp_idx[REFP_1]][MV_Y];
        }
    }
#endif

    /* parse coefficients */
    if(core->pred_mode != MODE_SKIP)
    {
        /* clear coefficient buffer */
        evc_mset(core->coef[Y_C], 0, cuw * cuh * sizeof(s16));
        evc_mset(core->coef[U_C], 0, (cuw >> 1) * (cuh >> 1) * sizeof(s16));
        evc_mset(core->coef[V_C], 0, (cuw >> 1) * (cuh >> 1) * sizeof(s16));

        ret = evcd_eco_coef(ctx, core);
        evc_assert_rv(ret == EVC_OK, ret);
    }

#if DMVR
    core->DMVRenable = 0;
    if(core->pred_mode == MODE_SKIP
#if MMVD
       && !new_skip_flag
#endif
       )
       core->DMVRenable = 1;

    if(inter_dir == PRED_DIR)

        core->DMVRenable = 1;

#if AFFINE
    if(core->affine_flag)
        core->DMVRenable = 0;
#endif
#endif

#if DMVR
    core->DMVRenable = 0;
    if(core->pred_mode == MODE_SKIP
#if MMVD
       && !new_skip_flag
#endif
       )
       core->DMVRenable = 1;

    if(inter_dir == PRED_DIR)

        core->DMVRenable = 1;

#if AFFINE
    if(core->affine_flag)
        core->DMVRenable = 0;
#endif 
#endif
    return EVC_OK;
}

int evcd_eco_cnkh(EVC_BSR * bs, EVC_CNKH * cnkh)
{
    cnkh->ver = evc_bsr_read(bs, 3);
    cnkh->ctype = evc_bsr_read(bs, 4);
    cnkh->broken = evc_bsr_read1(bs);
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

int evcd_eco_ref_pic_list_mod(EVC_BSR * bs)
{
    return EVC_OK;
}

int evcd_eco_sps(EVC_BSR * bs, EVC_SPS * sps)
{
    sps->sps_seq_parameter_set_id = (u32)evc_bsr_read_ue(bs);
    sps->profile_idc = evc_bsr_read(bs, 7);
    sps->tier_flag = evc_bsr_read1(bs);
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
        sps->log2_diff_max_12_max_14_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_min_12_min_14_cb_size_minus1 = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_11_max_tt_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_min_11_min_tt_cb_size_minus2 = (u32)evc_bsr_read_ue(bs);
    }
    sps->sps_suco_flag = (u32)evc_bsr_read1(bs);
    if (sps->sps_suco_flag)
    {
        sps->log2_diff_ctu_size_max_suco_cb_size = (u32)evc_bsr_read_ue(bs);
        sps->log2_diff_max_suco_min_suco_cb_size = (u32)evc_bsr_read_ue(bs);
    }
#if PROFILE_IS_MAIN(PROFILE)
#if AMVR
    sps->tool_amvr = evc_bsr_read1(bs);
#endif
#if MMVD
    sps->tool_mmvd = evc_bsr_read1(bs);
#endif
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
#if ADMVP
    sps->tool_amis = evc_bsr_read1(bs);
#endif
#if HTDF
    sps->tool_htdf = evc_bsr_read1(bs);
#endif

#endif
    sps->log2_max_pic_order_cnt_lsb_minus4 = (u32)evc_bsr_read_ue(bs);
    sps->sps_max_dec_pic_buffering_minus1 = (u32)evc_bsr_read_ue(bs);
    sps->picture_num_present_flag = evc_bsr_read1(bs);
    if (sps->picture_num_present_flag)
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

    sps->closed_gop = evc_bsr_read1(bs);
    sps->num_ref_pics_act = evc_bsr_read(bs, 4) + 1;

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

    if (sps->long_term_ref_pics_flag)
    {
        pps->additional_lt_poc_lsb_len = evc_bsr_read_ue(bs);
    }

    if (sps->rpl_candidates_present_flag)
    {
        pps->rpl1_idx_present_flag = evc_bsr_read1(bs);
    }

    pps->single_tile_in_pic_flag = evc_bsr_read1(bs);
    if (!pps->single_tile_in_pic_flag)
    {
        pps->num_tile_columns_minus1 = evc_bsr_read_ue(bs);
        pps->num_tile_rows_minus1 = evc_bsr_read_ue(bs);
        pps->uniform_tile_spacing_flag = evc_bsr_read1(bs);
        if (!pps->uniform_tile_spacing_flag)
        {
            for (int i = 0; i < pps->num_tile_columns_minus1; ++i)
            {
                pps->tile_column_width_minus1[i] = evc_bsr_read_ue(bs);
            }
            for (int i = 0; i < pps->num_tile_rows_minus1; ++i)
            {
                pps->tile_row_height_minus1[i] = evc_bsr_read_ue(bs);
            }
        }
        pps->loop_filter_across_tiles_enabled_flag = evc_bsr_read1(bs);
        pps->tile_offset_lens_minus1 = evc_bsr_read1(bs);
    }

    pps->tile_id_len_minus1 = evc_bsr_read_ue(bs);
    pps->explicit_tile_id_flag = evc_bsr_read1(bs);
    if (pps->explicit_tile_id_flag)
    {
        for (int i = 0; i <= pps->num_tile_rows_minus1; ++i)
        {
            for (int j = 0; j <= pps->num_tile_columns_minus1; ++j)
            {
                pps->tile_id_val[i][j] = evc_bsr_read(bs, pps->tile_id_len_minus1 + 1);
            }
        }
    }

    pps->arbitrary_tile_group_present_flag = evc_bsr_read1(bs);

    while (!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_bsr_read1(bs);
    }

    return EVC_OK;
}

#if ALF
int evcd_truncatedUnaryEqProb( EVC_BSR * bs, const int maxSymbol )
{
  for( int k = 0; k < maxSymbol; k++ )
  {
    int symbol = evc_bsr_read1( bs );
    if( !symbol )
    {
      return k;
    }
  }
  return maxSymbol;
}

int alfGolombDecode( EVC_BSR * bs, const int k )
{
  u32 uiSymbol;
  int q = -1;
  int nr = 0;
  int m = (int)pow( 2.0, k );
  int a;

  uiSymbol = 1;
  while( uiSymbol )
  {
    uiSymbol = evc_bsr_read1(bs);
    q++;
  }

  for( a = 0; a < k; ++a )          // read out the sequential log2(M) bits
  {
    uiSymbol = evc_bsr_read1(bs);
    if( uiSymbol )
    {
      nr += 1 << a;
    }
  }
  nr += q * m;                    // add the bits and the multiple of M
  if( nr != 0 )
  {
    uiSymbol = evc_bsr_read1(bs);
    nr = ( uiSymbol ) ? nr : -nr;
  }
  return nr;
}

u32 evcd_xReadTruncBinCode( EVC_BSR * bs, const int uiMaxSymbol )
{
  u32 ruiSymbol;
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
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
  ruiSymbol = evc_bsr_read( bs, uiThresh ); //xReadCode( uiThresh, ruiSymbol );
  if( ruiSymbol >= uiVal - b )
  {
    u32 uiSymbol = evc_bsr_read1( bs ); //xReadFlag( uiSymbol );
    ruiSymbol <<= 1;
    ruiSymbol += uiSymbol;
    ruiSymbol -= ( uiVal - b );
  }

  return ruiSymbol;
}

int evcd_eco_alf_filter(EVC_BSR * bs, evc_AlfTileGroupParam* alfTileGroupParam, const BOOL isChroma)
{
    if (!isChroma)
    {
        alfTileGroupParam->coeffDeltaFlag = evc_bsr_read1(bs); // "alf_coefficients_delta_flag"
        if (!alfTileGroupParam->coeffDeltaFlag)
        {
            if (alfTileGroupParam->numLumaFilters > 1)
            {
                alfTileGroupParam->coeffDeltaPredModeFlag = evc_bsr_read1(bs); // "coeff_delta_pred_mode_flag"
            }
            else
            {
                alfTileGroupParam->coeffDeltaPredModeFlag = 0;
            }
        }
        else
        {
            alfTileGroupParam->coeffDeltaPredModeFlag = 0;
        }
    }

    AlfFilterShape* alfShape1 = new_AlfFilterShape( isChroma ? 5 : ( alfTileGroupParam->lumaFilterType == ALF_FILTER_5 ? 5 : 7 ) );
    evc_AlfFilterShape alfShape;
    portAlfFilterShape(&alfShape, alfShape1);

    const int maxGolombIdx = alfShape.filterType == ALF_FILTER_5 ? 2 : 3;

    int min_golomb_order = evc_bsr_read_ue(bs); // "min_golomb_order"
    int kMin = min_golomb_order + 1;
    int kMinTab[MAX_NUM_ALF_COEFF];

    const int numFilters = isChroma ? 1 : alfTileGroupParam->numLumaFilters;
    short* coeff = isChroma ? alfTileGroupParam->chromaCoeff : alfTileGroupParam->lumaCoeff;

    for( int idx = 0; idx < maxGolombIdx; idx++ )
    {
      int code = evc_bsr_read1( bs ); //"golomb_order_increase_flag"
      kMinTab[idx] = kMin + code;
      kMin = kMinTab[idx];
    }

    if( !isChroma )
    {
      if( alfTileGroupParam->coeffDeltaFlag )
      {
        for( int ind = 0; ind < alfTileGroupParam->numLumaFilters; ++ind )
        {
          int code = evc_bsr_read1(bs); // "filter_coefficient_flag[i]"
          alfTileGroupParam->filterCoeffFlag[ind] = code;
        }
      }
    }

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {
      if( !isChroma && !alfTileGroupParam->filterCoeffFlag[ind] && alfTileGroupParam->coeffDeltaFlag )
      {
        memset( coeff + ind * MAX_NUM_ALF_LUMA_COEFF, 0, sizeof( *coeff ) * alfShape.numCoeff );
        continue;
      }

      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode( bs, kMinTab[alfShape.golombIdx[i]] );
      }
    }

    delete_AlfFilterShape(alfShape1);

    return EVC_OK;
}

int evcd_eco_alf_tgh_param(EVC_BSR * bs, EVC_TGH * tgh)
{
    evc_AlfTileGroupParam* alfTileGroupParam = &(tgh->alf_tgh_param);

    //AlfTileGroupParam reset
    alfTileGroupParam->temporalAlfFlag = 0;
    alfTileGroupParam->prevIdx = 0;
    alfTileGroupParam->tLayer = 0;
    alfTileGroupParam->resetALFBufferFlag = 0;
    alfTileGroupParam->store2ALFBufferFlag = 0;
    alfTileGroupParam->temporalAlfFlag = 0;
    alfTileGroupParam->prevIdx = 0;
    alfTileGroupParam->tLayer = 0;
    alfTileGroupParam->isCtbAlfOn = 0;
    memset(alfTileGroupParam->alfCtuEnableFlag, 1 , 512*3*sizeof(u8));
    memset(alfTileGroupParam->enabledFlag, 0, 3*sizeof(BOOL));
    alfTileGroupParam->lumaFilterType = ALF_FILTER_5;
    memset(alfTileGroupParam->lumaCoeff, 0, sizeof(short)*325);
    memset(alfTileGroupParam->chromaCoeff, 0, sizeof(short)*7);
    memset(alfTileGroupParam->filterCoeffDeltaIdx, 0, sizeof(short)*MAX_NUM_ALF_CLASSES);
    memset(alfTileGroupParam->filterCoeffFlag, 1, sizeof(BOOL)*25);
    alfTileGroupParam->numLumaFilters = 1;
    alfTileGroupParam->coeffDeltaFlag = 0;
    alfTileGroupParam->coeffDeltaPredModeFlag = 0;
    alfTileGroupParam->chromaCtbPresentFlag = 0;
    alfTileGroupParam->fixedFilterPattern = 0;
    memset(alfTileGroupParam->fixedFilterIdx, 0, sizeof(int)*25);

    alfTileGroupParam->enabledFlag[0] = evc_bsr_read1(bs);

    if (!alfTileGroupParam->enabledFlag[0])
    {
        return EVC_OK;
    }

    int alfChromaIdc = evcd_truncatedUnaryEqProb(bs, 3);
    alfTileGroupParam->enabledFlag[2] = alfChromaIdc & 1;
    alfTileGroupParam->enabledFlag[1] = (alfChromaIdc >> 1) & 1;
    {
        alfTileGroupParam->temporalAlfFlag = evc_bsr_read1(bs); // "alf_temporal_enable_flag"
        if ( alfTileGroupParam->temporalAlfFlag )
        {
            alfTileGroupParam->prevIdx = evc_bsr_read_ue(bs);  // "alf_temporal_index"
        }
        else
        {
            alfTileGroupParam->resetALFBufferFlag  = evc_bsr_read1( bs );
            alfTileGroupParam->store2ALFBufferFlag = evc_bsr_read1( bs );
        }
    }

    if (!alfTileGroupParam->temporalAlfFlag)
    {

    alfTileGroupParam->numLumaFilters = evcd_xReadTruncBinCode( bs, MAX_NUM_ALF_CLASSES) + 1;
    alfTileGroupParam->lumaFilterType = !(evc_bsr_read1(bs));

    if (alfTileGroupParam->numLumaFilters > 1)
    {
        for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
        {
            alfTileGroupParam->filterCoeffDeltaIdx[i] = (short)evcd_xReadTruncBinCode( bs, alfTileGroupParam->numLumaFilters );  //filter_coeff_delta[i]
        }
    }

       char codetab_pred[3] = { 1, 0, 2 };
      const int iNumFixedFilterPerClass = 16;
      memset(alfTileGroupParam->fixedFilterIdx, 0, sizeof(alfTileGroupParam->fixedFilterIdx));
      if (iNumFixedFilterPerClass > 0)
      {
        alfTileGroupParam->fixedFilterPattern =  codetab_pred[alfGolombDecode(bs, 0)];
        if (alfTileGroupParam->fixedFilterPattern == 2)
        {
          for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
          {
              alfTileGroupParam->fixedFilterIdx[classIdx] = evc_bsr_read1(bs); // "fixed_filter_flag"
          }
        }
        else if (alfTileGroupParam->fixedFilterPattern == 1)
        {
          for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
          {
            //on
            alfTileGroupParam->fixedFilterIdx[classIdx] = 1;
          }
        }

        if (alfTileGroupParam->fixedFilterPattern > 0 && iNumFixedFilterPerClass > 1)
        {
          for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
          {
            if (alfTileGroupParam->fixedFilterIdx[classIdx] > 0)
            {
                alfTileGroupParam->fixedFilterIdx[classIdx] = (int)evcd_xReadTruncBinCode( bs, iNumFixedFilterPerClass ) + 1;
            }
          }
        }
      }

      evcd_eco_alf_filter(bs, &(tgh->alf_tgh_param), FALSE);
    }


      //decode map
    alfTileGroupParam->isCtbAlfOn = evc_bsr_read1(bs);
      if( alfTileGroupParam->isCtbAlfOn )
      {
          for(int i = 0; i < tgh->num_ctb; i++)
              alfTileGroupParam->alfCtuEnableFlag[0][i] = evc_bsr_read1(bs);
      }
    return EVC_OK;
}
#endif

int evcd_eco_tgh(EVC_BSR * bs, EVC_SPS * sps, EVC_PPS * pps, EVC_TGH * tgh)
{
    int NumTilesInTileGroup = 0;    //TBD according to the spec
    tgh->tile_group_pic_parameter_set_id = evc_bsr_read_ue(bs);
    tgh->single_tile_in_tile_group_flag = evc_bsr_read1(bs);
    tgh->first_tile_id = evc_bsr_read(bs, pps->tile_id_len_minus1 + 1);

    if (!tgh->single_tile_in_tile_group_flag)
    {
        if (pps->arbitrary_tile_group_present_flag)
        {
            tgh->arbitrary_tile_group_flag = evc_bsr_read1(bs);
        }
        if (!tgh->arbitrary_tile_group_flag)
        {
            tgh->last_tile_id = evc_bsr_read(bs, pps->tile_id_len_minus1 + 1);
        }
        else
        {
            tgh->num_remaining_tiles_in_tile_group_minus1 = evc_bsr_read_ue(bs);
            for (int i = 0; i < NumTilesInTileGroup - 1; ++i)
            {
                tgh->delta_tile_id_minus1[i] = evc_bsr_read_ue(bs);
            }
        }
    }

    tgh->tile_group_type = evc_bsr_read_ue(bs);
#if ALF
    if (sps->tool_alf)
    {
        tgh->alf_on = evc_bsr_read1(bs);
        if (tgh->alf_on)
        {
            evcd_eco_alf_tgh_param(bs, tgh);
        }
    }
#endif

    // if (NalUnitType != IDR_NUT)  TBD: NALU types to be implemented
    {
        tgh->poc = evc_bsr_read_ue(bs);
        if (sps->picture_num_present_flag)
        {
            tgh->ref_pic_flag = evc_bsr_read1(bs);
            tgh->picture_num = evc_bsr_read(bs, 8);
        }
    }
    // else 
    {
        if (!sps->picture_num_present_flag)
        {
            //L0 candidates signaling
            tgh->ref_pic_list_sps_flag[0] = sps->rpl_candidates_present_flag ? evc_bsr_read1(bs) : 0;

            if (tgh->ref_pic_list_sps_flag[0])
            {
                if (sps->rpls_l0_num)
                {
                    tgh->rpl_l0_idx = evc_bsr_read_ue(bs);
                    memcpy(&tgh->rpl_l0, &sps->rpls_l0[tgh->rpl_l0_idx], sizeof(tgh->rpl_l0)); //TBD: temporal workaround, consider refactoring
                    tgh->rpl_l0.poc = tgh->poc;
                }
            }
            else
            {
                evcd_eco_rlp(bs, &tgh->rpl_l0);
                tgh->rpl_l0.poc = tgh->poc;
            }

            //L1 candidates signaling
            tgh->ref_pic_list_sps_flag[1] = sps->rpl_candidates_present_flag ? evc_bsr_read1(bs) : 0;

            if (tgh->ref_pic_list_sps_flag[1])
            {
                if (sps->rpls_l1_num)
                {
                    tgh->rpl_l1_idx = evc_bsr_read_ue(bs);
                    memcpy(&tgh->rpl_l1, &sps->rpls_l1[tgh->rpl_l1_idx], sizeof(tgh->rpl_l1)); //TBD: temporal workaround, consider refactoring
                    tgh->rpl_l1.poc = tgh->poc;
                }
            }
            else
            {
                evcd_eco_rlp(bs, &tgh->rpl_l1);
                tgh->rpl_l1.poc = tgh->poc;
            }
        }
    }

    if (!sps->picture_num_present_flag)
    {
        if (tgh->tile_group_type != TILE_GROUP_I)
        {
            tgh->num_ref_idx_active_override_flag = evc_bsr_read1(bs);
            if (tgh->num_ref_idx_active_override_flag)
            {
                tgh->rpl_l0.ref_pic_active_num = (u32)evc_bsr_read_ue(bs) + 1;
                if (tgh->tile_group_type == TILE_GROUP_B)
                {
                    tgh->rpl_l1.ref_pic_active_num = (u32)evc_bsr_read_ue(bs) + 1;
                }
            }
            else
            {
                tgh->rpl_l0.ref_pic_active_num = 2;  //Temporarily i set it to 2. this should be set equal to the signalled num_ref_idx_default_active_minus1[0] + 1.
                tgh->rpl_l1.ref_pic_active_num = 2;  //Temporarily i set it to 2. this should be set equal to the signalled num_ref_idx_default_active_minus1[1] + 1.
            }
            if (sps->picture_num_present_flag)
            {
                evcd_eco_ref_pic_list_mod(bs);
            }
        }
    }

    tgh->deblocking_filter_on = evc_bsr_read1(bs);
    tgh->qp = evc_bsr_read(bs, 6);
    tgh->qp_u = tgh->qp - evc_bsr_read_se(bs);
    tgh->qp_v = tgh->qp - evc_bsr_read_se(bs);

    if (!tgh->single_tile_in_tile_group_flag)
    {
        for (int i = 0; i < NumTilesInTileGroup - 1; ++i)
        {
            tgh->entry_point_offset_minus1[i] = evc_bsr_read(bs, pps->tile_offset_lens_minus1 + 1);
        }
    }

    tgh->dtr                   = evc_bsr_read(bs, DTR_BIT_CNT);
    tgh->keyframe = evc_bsr_read1(bs);
    tgh->udata_exist = evc_bsr_read1(bs);

#if AQS_SYNTAX
    tgh->es_map_norm_idx = evc_bsr_read_se(bs);
    {
        s16 table_factor_pos[12] = { 256, 267, 279, 291, 305, 318, 333, 347, 362, 379, 395, 412 }; //also 8-bit 
        s16 table_factor_neg[12] = { 256, 245, 235, 225, 215, 206, 197, 189, 181, 173, 166, 159 };

        //get multiplication factor
        if (tgh->es_map_norm_idx >= 0)
            tgh->es_map_norm = table_factor_pos[tgh->es_map_norm_idx];
        else
            sps->es_map_norm = table_factor_neg[-sps->es_map_norm_idx];
    }
#endif

    if(tgh->tile_group_type!= TILE_GROUP_I)
    {
        /* dptr: delta of presentation temporal reference */
        tgh->dptr = evc_bsr_read_se(bs);
    }

    tgh->poc = tgh->dtr + tgh->dptr;
    tgh->layer_id = evc_bsr_read(bs, 3);

    /* parse MMCO */
    tgh->mmco_on = evc_bsr_read1(bs);
    if(tgh->mmco_on)
    {
        tgh->mmco.cnt = 0;
        while(tgh->mmco.cnt < MAX_NUM_MMCO)
        {
            evc_assert_rv(tgh->mmco.cnt < MAX_NUM_MMCO, EVC_ERR_MALFORMED_BITSTREAM);

            tgh->mmco.type[tgh->mmco.cnt] = evc_bsr_read_ue(bs);
            if(tgh->mmco.type[tgh->mmco.cnt] == MMCO_END) break; /* END of MMCO */

            tgh->mmco.data[tgh->mmco.cnt] = evc_bsr_read_ue(bs);
            tgh->mmco.cnt++;
        }
    }

    /* parse RMPNI*/
    tgh->rmpni_on = evc_bsr_read1(bs);
    if(tgh->rmpni_on)
    {
        static int aa =0;
        int t0, t1;
        aa++;
        tgh->rmpni[REFP_0].cnt = 0;
        while(tgh->rmpni[REFP_0].cnt < MAX_NUM_RMPNI)
        {
            evc_assert_rv(tgh->rmpni[REFP_0].cnt < MAX_NUM_RMPNI, EVC_ERR_MALFORMED_BITSTREAM);

            t0 = evc_bsr_read_ue(bs);
            if(t0 == RMPNI_END) 
            {
                break; 
            }
            else if(t0 == RMPNI_ADPN_NEG)
            {
                t1 = evc_bsr_read_ue(bs);
                tgh->rmpni[REFP_0].delta_poc[tgh->rmpni[REFP_0].cnt] = -(t1+1);
                tgh->rmpni[REFP_0].cnt++;
            }
            else if(t0 == RMPNI_ADPN_POS)
            {
                t1 = evc_bsr_read_ue(bs);
                tgh->rmpni[REFP_0].delta_poc[tgh->rmpni[REFP_0].cnt] = t1 +1;
                tgh->rmpni[REFP_0].cnt++;
            }
            else
            {
                /* not support */
            }
        }

        tgh->rmpni[REFP_1].cnt = 0;
        while(tgh->rmpni[REFP_1].cnt < MAX_NUM_RMPNI)
        {
            evc_assert_rv(tgh->rmpni[REFP_1].cnt < MAX_NUM_RMPNI,
                EVC_ERR_MALFORMED_BITSTREAM);

            t0 = evc_bsr_read_ue(bs);
            if(t0 == RMPNI_END) 
            {
                break; 
            }
            else if(t0 == RMPNI_ADPN_NEG)
            {
                t1 = evc_bsr_read_ue(bs);
                tgh->rmpni[REFP_1].delta_poc[tgh->rmpni[REFP_1].cnt] = -(t1+1);
                tgh->rmpni[REFP_1].cnt++;
            }
            else if(t0 == RMPNI_ADPN_POS)
            {
                t1 = evc_bsr_read_ue(bs);
                tgh->rmpni[REFP_1].delta_poc[tgh->rmpni[REFP_1].cnt] = t1 +1;
                tgh->rmpni[REFP_1].cnt++;
            }
            else
            {
                /* not support */
            }
        }
    }

    /* byte align */
    while(!EVC_BSR_IS_BYTE_ALIGN(bs))
    {
        evc_assert_rv(0 == evc_bsr_read1(bs), EVC_ERR_MALFORMED_BITSTREAM);
    }
    return EVC_OK;
}

int evcd_eco_udata(EVCD_CTX * ctx, EVC_BSR * bs)
{
    int    i;
    u32 code;

    /* should be aligned before adding user data */
    evc_assert_rv(EVC_BSR_IS_BYTE_ALIGN(bs), EVC_ERR_UNKNOWN);

    code = evc_bsr_read(bs, 8);

    while(code != EVC_UD_END)
    {
        switch(code)
        {
            case EVC_UD_PIC_SIGNATURE:
                /* read signature (HASH) from bitstream */
                for(i = 0; i < 16; i++)
                {
                    ctx->pic_sign[i] = evc_bsr_read(bs, 8);
                }
                ctx->pic_sign_exist = 1;
                break;

            default:
                evc_assert_rv(0, EVC_ERR_UNEXPECTED);
        }
        code = evc_bsr_read(bs, 8);
    }
    return EVC_OK;
}

#if AFFINE
int evcd_eco_affine_mrg_idx(EVC_BSR * bs, EVCD_SBAC * sbac)
{
    return sbac_read_truncate_unary_sym(bs, sbac, sbac->ctx.affine_mrg, AFF_MAX_CAND, AFF_MAX_CAND);
}
#endif
