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

#include "evc_util.h"
#include "evc_df.h"
#include "evce_def.h"
#include "evce_util.h"
#include <math.h>
/******************************************************************************
 * picture buffer alloc/free/expand
 ******************************************************************************/
void evce_picbuf_expand(EVCE_CTX *ctx, EVC_PIC *pic)
{
    evc_picbuf_expand(pic, pic->pad_l, pic->pad_c);
}

EVC_PIC * evce_pic_alloc(PICBUF_ALLOCATOR * pa, int * ret)
{
    return evc_picbuf_alloc(pa->w, pa->h, pa->pad_l, pa->pad_c, ret
#if BD_CF_EXT
                            , pa->idc
#endif
    );
}

void evce_pic_free(PICBUF_ALLOCATOR *pa, EVC_PIC *pic)
{
    evc_picbuf_free(pic);
}

/******************************************************************************
 * implementation of bitstream writer
 ******************************************************************************/
void evce_bsw_skip_slice_size(EVC_BSW *bs)
{
    evc_bsw_write(bs, 0, 32);
}

int evce_bsw_write_nalu_size(EVC_BSW *bs)
{
    u32 size;

    size = EVC_BSW_GET_WRITE_BYTE(bs) - 4;
    
    bs->beg[0] = size & 0x000000ff; //TBC(@Chernyak): is there a better way?
    bs->beg[1] = (size & 0x0000ff00) >> 8;
    bs->beg[2] = (size & 0x00ff0000) >> 16;
    bs->beg[3] = (size & 0xff000000) >> 24;
    return size;
}

void evce_diff_pred(int x, int y, int log2_cuw, int log2_cuh, EVC_PIC *org, pel pred[N_C][MAX_CU_DIM], s16 diff[N_C][MAX_CU_DIM]
#if BD_CF_EXT
                    , int bit_depth_luma, int bit_depth_chroma
                    , int chroma_format_idc
#endif
)
{
    pel * buf;
    int cuw, cuh, stride;

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;
    stride = org->s_l;

    /* Y */
    buf = org->y + (y * stride) + x;

#if BD_CF_EXT
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[Y_C], stride, cuw, cuw, diff[Y_C], bit_depth_luma);
#else
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[Y_C], stride, cuw, cuw, diff[Y_C]);
#endif
#if BD_CF_EXT
    if(!chroma_format_idc)
        return;
#endif
#if BD_CF_EXT
    cuw >>= (GET_CHROMA_W_SHIFT(chroma_format_idc));
    cuh >>= (GET_CHROMA_H_SHIFT(chroma_format_idc));
    x >>= (GET_CHROMA_W_SHIFT(chroma_format_idc));
    y >>= (GET_CHROMA_H_SHIFT(chroma_format_idc));
    log2_cuw -= (GET_CHROMA_W_SHIFT(chroma_format_idc));
    log2_cuh -= (GET_CHROMA_H_SHIFT(chroma_format_idc));
#else
    cuw >>= 1;
    cuh >>= 1;
    x >>= 1;
    y >>= 1;
    log2_cuw--;
    log2_cuh--;
#endif

    stride = org->s_c;

    /* U */
    buf = org->u + (y * stride) + x;
#if BD_CF_EXT
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[U_C], stride, cuw, cuw, diff[U_C], bit_depth_chroma);
#else
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[U_C], stride, cuw, cuw, diff[U_C]);
#endif

    /* V */
    buf = org->v + (y * stride) + x;
#if BD_CF_EXT
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[V_C], stride, cuw, cuw, diff[V_C], bit_depth_chroma);
#else
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[V_C], stride, cuw, cuw, diff[V_C]);
#endif
}

#if RDO_DBK
void evc_set_affine_mvf(EVCE_CTX * ctx, EVCE_CORE * core, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][VER_NUM][MV_D], int vertex_num)
{
    s8 (*map_refi)[REFP_NUM];
    int w_cu;
    int h_cu;
    int scup;
    int w_scu;
    int i, j;
    int lidx;
    int aff_scup[VER_NUM];
    int log2_cuw = CONV_LOG2(w);
    int log2_cuh = CONV_LOG2(h);

    scup = core->scup;
    w_cu = w >> MIN_CU_LOG2;
    h_cu = h >> MIN_CU_LOG2;
    w_scu = ctx->w_scu;

    aff_scup[0] = 0;
    aff_scup[1] = (w_cu - 1);
    aff_scup[2] = (h_cu - 1) * w_scu;
    aff_scup[3] = (w_cu - 1) + (h_cu - 1) * w_scu;

    map_refi = ctx->map_refi + scup;
    for (i = 0; i < h_cu; i++)
    {
        for (j = 0; j < w_cu; j++)
        {
            map_refi[j][REFP_0] = refi[REFP_0];
            map_refi[j][REFP_1] = refi[REFP_1];
        }
        map_refi += w_scu;
    }

    // derive sub-block size
    int sub_w = 4, sub_h = 4;
    derive_affine_subblock_size_bi( mv, refi, core->cuw, core->cuh, &sub_w, &sub_h, vertex_num, NULL);

    int   sub_w_in_scu = PEL2SCU( sub_w );
    int   sub_h_in_scu = PEL2SCU( sub_h );
    int   half_w = sub_w >> 1;
    int   half_h = sub_h >> 1;

    for (lidx = 0; lidx < REFP_NUM; lidx++)
    {
        if (refi[lidx] >= 0)
        {
            s16(*ac_mv)[MV_D] = mv[lidx];

            int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
            int mv_scale_hor = ac_mv[0][MV_X] << 7;
            int mv_scale_ver = ac_mv[0][MV_Y] << 7;
            int mv_scale_tmp_hor, mv_scale_tmp_ver;

            // convert to 2^(storeBit + iBit) precision
            dmv_hor_x = (ac_mv[1][MV_X] - ac_mv[0][MV_X]) << (7 - log2_cuw);     // deltaMvHor
            dmv_hor_y = (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << (7 - log2_cuw);
            if ( vertex_num == 3 )
            {
                dmv_ver_x = (ac_mv[2][MV_X] - ac_mv[0][MV_X]) << (7 - log2_cuh); // deltaMvVer
                dmv_ver_y = (ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << (7 - log2_cuh);
            }
            else
            {
                dmv_ver_x = -dmv_hor_y;                                          // deltaMvVer
                dmv_ver_y = dmv_hor_x;
            }

            for ( int h = 0; h < h_cu; h += sub_h_in_scu )
            {
                for ( int w = 0; w < w_cu; w += sub_w_in_scu )
                {
                    if ( w == 0 && h == 0 )
                    {
                        mv_scale_tmp_hor = ac_mv[0][MV_X];
                        mv_scale_tmp_ver = ac_mv[0][MV_Y];
                    }
                    else if ( w + sub_w_in_scu == w_cu && h == 0 )
                    {
                        mv_scale_tmp_hor = ac_mv[1][MV_X];
                        mv_scale_tmp_ver = ac_mv[1][MV_Y];
                    }
                    else if ( w == 0 && h + sub_h_in_scu == h_cu && vertex_num == 3 )
                    {
                        mv_scale_tmp_hor = ac_mv[2][MV_X];
                        mv_scale_tmp_ver = ac_mv[2][MV_Y];
                    }
                    else
                    {
                        int pos_x = (w << MIN_CU_LOG2) + half_w;
                        int pos_y = (h << MIN_CU_LOG2) + half_h;

                        mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
                        mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

                        // 1/16 precision, 18 bits, same as MC
                        evc_mv_rounding_s32( mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0 );

                        mv_scale_tmp_hor = EVC_CLIP3( -(1 << 17), (1 << 17) - 1, mv_scale_tmp_hor );
                        mv_scale_tmp_ver = EVC_CLIP3( -(1 << 17), (1 << 17) - 1, mv_scale_tmp_ver );

                        // 1/4 precision, 16 bits for storage
                        mv_scale_tmp_hor >>= 2;
                        mv_scale_tmp_ver >>= 2;
                    }

                    // save MV for each 4x4 block
                    for ( int y = h; y < h + sub_h_in_scu; y++ )
                    {
                        for ( int x = w; x < w + sub_w_in_scu; x++ )
                        {
                            int addr_in_scu = scup + x + y * w_scu;
                            ctx->map_mv[addr_in_scu][lidx][MV_X] = (s16)mv_scale_tmp_hor;
                            ctx->map_mv[addr_in_scu][lidx][MV_Y] = (s16)mv_scale_tmp_ver;
                        }
                    }
                }
            }
        }
    }
}
#endif

#if DQP_RDO
void evce_set_qp(EVCE_CTX *ctx, EVCE_CORE *core, u8 qp)
{
    u8 qp_i_cb, qp_i_cr;
    core->qp = qp;
#if BD_CF_EXT
    core->qp_y = GET_LUMA_QP(core->qp, ctx->sps.bit_depth_luma_minus8);
    qp_i_cb = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh->qp_u_offset);
    qp_i_cr = EVC_CLIP3(-6 * ctx->sps.bit_depth_chroma_minus8, 57, core->qp + ctx->sh->qp_v_offset);
    core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * ctx->sps.bit_depth_chroma_minus8;
    core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * ctx->sps.bit_depth_chroma_minus8;
#else
    core->qp_y = GET_LUMA_QP(core->qp);
    qp_i_cb = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh->qp_u_offset);
    qp_i_cr = EVC_CLIP3(-6 * (BIT_DEPTH - 8), 57, core->qp + ctx->sh->qp_v_offset);
    core->qp_u = p_evc_tbl_qp_chroma_dynamic[0][qp_i_cb] + 6 * (BIT_DEPTH - 8);
    core->qp_v = p_evc_tbl_qp_chroma_dynamic[1][qp_i_cr] + 6 * (BIT_DEPTH - 8);
#endif
}
#endif
void evce_split_tbl_init(EVCE_CTX *ctx)
{
    evc_split_tbl[BLOCK_11][IDX_MAX] = ctx->cdsc.framework_cb_max;
    evc_split_tbl[BLOCK_11][IDX_MIN] = ctx->cdsc.framework_cb_min;
    evc_split_tbl[BLOCK_12][IDX_MAX] = ctx->cdsc.framework_cb_max;
    evc_split_tbl[BLOCK_12][IDX_MIN] = evc_split_tbl[BLOCK_11][IDX_MIN] + 1;
    evc_split_tbl[BLOCK_14][IDX_MAX] = ctx->cdsc.framework_cu14_max;
    evc_split_tbl[BLOCK_14][IDX_MIN] = evc_split_tbl[BLOCK_12][IDX_MIN] + 1;
    evc_split_tbl[BLOCK_TT][IDX_MAX] = ctx->cdsc.framework_tris_max;
    evc_split_tbl[BLOCK_TT][IDX_MIN] = ctx->cdsc.framework_tris_min;
}

u8 evce_check_luma(EVCE_CTX *ctx, EVCE_CORE * core)
{
    return evc_check_luma(core->tree_cons);    
}

u8 evce_check_chroma(EVCE_CTX *ctx, EVCE_CORE * core)
{
    return evc_check_chroma(core->tree_cons);    
}
u8 evce_check_all(EVCE_CTX *ctx, EVCE_CORE * core)
{
    return evc_check_all(core->tree_cons);    
}

u8 evce_check_only_intra(EVCE_CTX *ctx, EVCE_CORE * core)
{
    return evc_check_only_intra(core->tree_cons);
}

u8 evce_check_only_inter(EVCE_CTX *ctx, EVCE_CORE * core)
{
    return evc_check_only_inter(core->tree_cons);
}

u8 evce_check_all_preds(EVCE_CTX *ctx, EVCE_CORE * core)
{
    return evc_check_all_preds(core->tree_cons);       
}

MODE_CONS evce_derive_mode_cons(EVCE_CTX *ctx, int lcu_num, int cup)
{
    return ((ctx->map_cu_data[lcu_num].pred_mode[cup] == MODE_INTRA) || (ctx->map_cu_data[lcu_num].pred_mode[cup] == MODE_IBC) ) ? eOnlyIntra : eOnlyInter;
}

static void create_sub_pa(PICBUF_ALLOCATOR * pa_dst, PICBUF_ALLOCATOR * pa_src, int ds_ratio)
{
    evc_mcpy(pa_dst, pa_src, sizeof(PICBUF_ALLOCATOR));
    pa_dst->w = pa_src->w >> ds_ratio;
    pa_dst->h = pa_src->h >> ds_ratio;
}

const int tf_interpolation_coeff[16][8] =
{
    {   0,   0,   0,  64,   0,   0,   0,   0 },
    {   0,   1,  -3,  64,   4,  -2,   0,   0 },
    {   0,   1,  -6,  62,   9,  -3,   1,   0 },
    {   0,   2,  -8,  60,  14,  -5,   1,   0 },
    {   0,   2,  -9,  57,  19,  -7,   2,   0 },
    {   0,   3, -10,  53,  24,  -8,   2,   0 },
    {   0,   3, -11,  50,  29,  -9,   2,   0 },
    {   0,   3, -11,  44,  35, -10,   3,   0 },
    {   0,   1,  -7,  38,  38,  -7,   1,   0 },
    {   0,   3, -10,  35,  44, -11,   3,   0 },
    {   0,   2,  -9,  29,  50, -11,   3,   0 },
    {   0,   2,  -8,  24,  53, -10,   3,   0 },
    {   0,   2,  -7,  19,  57,  -9,   2,   0 },
    {   0,   1,  -5,  14,  60,  -8,   2,   0 },
    {   0,   1,  -3,   9,  62,  -6,   1,   0 },
    {   0,   0,  -2,   4,  64,  -3,   1,   0 } 
};

static int get_motion_cost(EVC_PIC * pic_cur, EVC_PIC * pic_ref, int x, int y, int dx, int dy, int block_size, int best_cost)
{
    const int mv_factor = 16;
    pel * pel_cur = pic_cur->y;
    pel * pel_ref = pic_ref->y;
    int   cur_stride = pic_cur->s_l;
    int   ref_stride = pic_cur->s_l;
    int   cost = 0;

    if (((dx | dy) & 0xF) == 0)
    {
        dx /= mv_factor;
        dy /= mv_factor;
        for (int y1 = 0; y1 < block_size; y1++)
        {
            const pel* cur_row = pel_cur + (y + y1) * cur_stride + x;
            const pel* ref_row = pel_ref + (y + y1 + dy) * ref_stride + (x + dx);
            for (int x1 = 0; x1 < block_size; x1 += 2)
            {
                int diff = cur_row[x1] - ref_row[x1];
                cost += diff * diff;
                diff = cur_row[x1 + 1] - ref_row[x1 + 1];
                cost += diff * diff;
            }
            if (cost > best_cost)
            {
                return cost;
            }
        }
    }
    else
    {
        const int *x_filter = tf_interpolation_coeff[dx & 0xF];
        const int *y_filter = tf_interpolation_coeff[dy & 0xF];
        int temp_arr[64 + 8][64];

        int sum, base;
        for (int y1 = 1; y1 < block_size + 7; y1++)
        {
            const int y_offset = y + y1 + (dy >> 4) - 3;
            const pel * src_row = pel_ref + (y_offset) * ref_stride + 0;
            for (int x1 = 0; x1 < block_size; x1++)
            {
                sum = 0;
                base = x + x1 + (dx >> 4) - 3;
                const pel *row_st = src_row + base;

                sum += x_filter[1] * row_st[1];
                sum += x_filter[2] * row_st[2];
                sum += x_filter[3] * row_st[3];
                sum += x_filter[4] * row_st[4];
                sum += x_filter[5] * row_st[5];
                sum += x_filter[6] * row_st[6];

                temp_arr[y1][x1] = sum;
            }
        }

        const pel max_val = (1 << BD_FROM_CS(pic_cur->imgb->cs) ) - 1;
        for (int y1 = 0; y1 < block_size; y1++)
        {
            const pel *org_row = pel_cur + (y + y1) * cur_stride + 0;
            for (int x1 = 0; x1 < block_size; x1++)
            {
                sum = 0;
                sum += y_filter[1] * temp_arr[y1 + 1][x1];
                sum += y_filter[2] * temp_arr[y1 + 2][x1];
                sum += y_filter[3] * temp_arr[y1 + 3][x1];
                sum += y_filter[4] * temp_arr[y1 + 4][x1];
                sum += y_filter[5] * temp_arr[y1 + 5][x1];
                sum += y_filter[6] * temp_arr[y1 + 6][x1];

                sum = (sum + (1 << 11)) >> 12;
                sum = sum < 0 ? 0 : (sum > max_val ? max_val : sum);

                cost += (sum - org_row[x + x1]) * (sum - org_row[x + x1]);
            }
            if (cost > best_cost)
            {
                return cost;
            }
        }
    }
    return cost;

}

static void get_motion(EVC_PIC * pic_cur, EVC_PIC * pic_ref, int block_size, void * prev_mv, int factor, int d_range)
{
    int range = 5;
    int step  = block_size;
    int pic_w = pic_cur->w_l;
    int pic_h = pic_cur->h_l;
    int pic_w_scu = pic_w >> MIN_CU_LOG2;
    int pic_mv_offset = d_range == 0 ? 1 : 0;
    int best_mv[2];
    int best_cost;
    s16(*map_mv)[REFP_NUM][MV_D] = NULL;
    if (prev_mv != NULL)
    {
        map_mv = prev_mv;
    }
    const int mv_factor = 16;
    int store_mv_idx = d_range == 0 ? 0 : 1;

    static int print_cnt = 0;

    for (int y = 0; y + block_size < pic_h; y += step)
    {
        for (int x = 0; x + block_size < pic_w; x += step)
        {
            best_cost = block_size * block_size * 1024 * 1024;

            if (prev_mv == NULL)
            {
                range = 8;
                evc_mset(best_mv, 0, sizeof(int) * MV_D);
            }
            else
            {
                for (int py = -2; py <= 2; py++)
                {
                    int test_y = y / (2 * block_size) + py;
                    for (int px = -2; px <= 2; px++)
                    {
                        int test_x = x / (2 * block_size) + px;
                        if ((test_x >= 0) && (test_x < pic_w / (2 * block_size)) && (test_y >= 0) && (test_y < pic_h / (2 * block_size)))
                        {
                            int idx = test_y * (pic_w_scu >> pic_mv_offset) + test_x;
                            int old_mv[2];
                            old_mv[MV_X] = map_mv[idx][0][MV_X];
                            old_mv[MV_Y] = map_mv[idx][0][MV_Y];

                            int cost = get_motion_cost(pic_cur, pic_ref, x, y, old_mv[MV_X] * factor, old_mv[MV_Y] * factor, block_size, best_cost);
                            if (cost < best_cost)
                            {
                                best_cost = cost;
                                best_mv[MV_X] = old_mv[MV_X] * factor;
                                best_mv[MV_Y] = old_mv[MV_Y] * factor;
                            }
                        }
                    }
                }
            }

            int prev_b_mv[MV_D];
            evc_mcpy(prev_b_mv, best_mv, sizeof(int) * MV_D);

            for (int y2 = prev_b_mv[MV_Y] / mv_factor - range; y2 <= prev_b_mv[MV_Y] / mv_factor + range; y2++)
            {
                for (int x2 = prev_b_mv[MV_X] / mv_factor - range; x2 <= prev_b_mv[MV_X] / mv_factor + range; x2++)
                {
                    int cost = get_motion_cost(pic_cur, pic_ref, x, y, x2 * mv_factor, y2 * mv_factor, block_size, best_cost);
                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        best_mv[MV_X] = x2 * mv_factor;
                        best_mv[MV_Y] = y2 * mv_factor;
                    }
                }
            }

            if (d_range)
            { 
                evc_mcpy(prev_b_mv, best_mv, sizeof(int) * MV_D);
                int double_range = 3 * 4;
                for (int y2 = prev_b_mv[MV_Y] - double_range; y2 <= prev_b_mv[MV_Y] + double_range; y2 += 4)
                {
                    for (int x2 = prev_b_mv[MV_X] - double_range; x2 <= prev_b_mv[MV_X] + double_range; x2 += 4)
                    {
                        int cost = get_motion_cost(pic_cur, pic_ref, x, y, x2, y2, block_size, best_cost);
                        if (cost < best_cost)
                        {
                            best_cost = cost;
                            best_mv[MV_X] = x2 * factor;
                            best_mv[MV_Y] = y2 * factor;
                        }
                    }
                }

                evc_mcpy(prev_b_mv, best_mv, sizeof(int) * MV_D);
                double_range = 3;
                for (int y2 = prev_b_mv[MV_Y] - double_range; y2 <= prev_b_mv[MV_Y] + double_range; y2++)
                {
                    for (int x2 = prev_b_mv[MV_X] - double_range; x2 <= prev_b_mv[MV_X] + double_range; x2++)
                    {
                        int cost = get_motion_cost(pic_cur, pic_ref, x, y, x2, y2, block_size, best_cost);
                        if (cost < best_cost)
                        {
                            best_cost = cost;
                            best_mv[MV_X] = x2 * factor;
                            best_mv[MV_Y] = y2 * factor;
                        }
                    }
                }
            }

            if (store_mv_idx == 0)
            {
                int idx = (y / step) * pic_w_scu + x / step;
                pic_ref->map_mv[idx][store_mv_idx][MV_X] = best_mv[MV_X];
                pic_ref->map_mv[idx][store_mv_idx][MV_Y] = best_mv[MV_Y];
            }
            else
            {
                int scu = ((y >> MIN_CU_LOG2) * pic_w_scu + (x >> MIN_CU_LOG2));
                pic_ref->map_mv[scu][store_mv_idx][MV_X] = best_mv[MV_X];
                pic_ref->map_mv[scu][store_mv_idx][MV_Y] = best_mv[MV_Y];

                for (int sy = 0; sy < block_size >> MIN_CU_LOG2; sy++)
                {
                    for (int sx = 0; sx < block_size >> MIN_CU_LOG2; sx++)
                    {
                        int scu = ((y >> MIN_CU_LOG2) + sy) * pic_w_scu + (x >> MIN_CU_LOG2) + sx;
                        pic_ref->map_mv[scu][store_mv_idx][MV_X] = best_mv[MV_X];
                        pic_ref->map_mv[scu][store_mv_idx][MV_Y] = best_mv[MV_Y];
                    }
                }
            }
       }
    }
}

static void apply_motion(EVC_PIC * pic_ref)
{
    const int block_size_luma = 8;
    const int chroma_idc = CF_FROM_CS(pic_ref->imgb->cs);
    const int bit_depth = BD_FROM_CS(pic_ref->imgb->cs);
    int cs_num = chroma_idc == 0 ? 1 : 3;

    pel * tmp_img = (pel *)evc_malloc(sizeof(pel) * pic_ref->h_l * pic_ref->w_l);

    for (int c = 0 ; c < cs_num ; c++)
    {
        const int w_shift = c == 0 ? 0 : GET_CHROMA_W_SHIFT(chroma_idc);
        const int h_shift = c == 0 ? 0 : GET_CHROMA_H_SHIFT(chroma_idc);
        const int block_size_x = block_size_luma >> w_shift;
        const int block_size_y = block_size_luma >> h_shift;
        int height, width;
        pel *src_img;
        int src_s, dst_s;

        if (c == Y_C)
        {
            height = pic_ref->h_l;
            width  = pic_ref->w_l;
            src_img = pic_ref->y;
            src_s = pic_ref->s_l;
            
        }
        else
        {
            height = pic_ref->h_c;
            width =  pic_ref->w_c;
            src_s = pic_ref->s_c;
            if (c == U_C)
            {
                src_img = pic_ref->u;
            }
            else // (c == V_C)
            {
                src_img = pic_ref->v;
            }
        }

        dst_s = width;

        const pel max_val = (1 << bit_depth) - 1;

        for (int y = 0; y + block_size_y <= height; y += block_size_y)
        {
            for (int x = 0; x + block_size_x <= width; x += block_size_x)
            {
                int scup_luma = c == Y_C ? ((y >> MIN_CU_LOG2) * (width >> MIN_CU_LOG2) + (x >> MIN_CU_LOG2)) : 
                                           ((y >> (MIN_CU_LOG2 - h_shift)) * (width >> (MIN_CU_LOG2 - w_shift)) + (x >> (MIN_CU_LOG2 - w_shift)));

                const int dx = pic_ref->map_mv[scup_luma][1][MV_X] >> w_shift;
                const int dy = pic_ref->map_mv[scup_luma][1][MV_Y] >> h_shift;
                const int xInt = pic_ref->map_mv[scup_luma][1][MV_X] >> (4 + w_shift);
                const int yInt = pic_ref->map_mv[scup_luma][1][MV_Y] >> (4 + h_shift);

                const int * x_filter = tf_interpolation_coeff[dx & 0xf];
                const int * y_filter = tf_interpolation_coeff[dy & 0xf]; // will add 6 bit.
                const int filter_taps = 7;
                const int centre_tap_offset = 3;

                int temp_arr[15][8];

                for (int by = 1; by < block_size_y + filter_taps; by++)
                {
                    const int y_offset = y + by + yInt - centre_tap_offset;
                    const pel * src_row = src_img + y_offset * src_s;
                    for (int bx = 0; bx < block_size_x; bx++)
                    {
                        int base = x + bx + xInt - centre_tap_offset;
                        const pel *row_st = src_row + base;

                        int sum = 0;
                        sum += x_filter[1] * row_st[1];
                        sum += x_filter[2] * row_st[2];
                        sum += x_filter[3] * row_st[3];
                        sum += x_filter[4] * row_st[4];
                        sum += x_filter[5] * row_st[5];
                        sum += x_filter[6] * row_st[6];

                        temp_arr[by][bx] = sum;
                    }
                }

                pel *dst_row = tmp_img + y * dst_s;
                for (int by = 0; by < block_size_y; by++, dst_row += dst_s)
                {
                    pel *dst_pel = dst_row + x;
                    for (int bx = 0; bx < block_size_x; bx++, dst_pel++)
                    {
                        int sum = 0;

                        sum += y_filter[1] * temp_arr[by + 1][bx];
                        sum += y_filter[2] * temp_arr[by + 2][bx];
                        sum += y_filter[3] * temp_arr[by + 3][bx];
                        sum += y_filter[4] * temp_arr[by + 4][bx];
                        sum += y_filter[5] * temp_arr[by + 5][bx];
                        sum += y_filter[6] * temp_arr[by + 6][bx];

                        sum = (sum + (1 << 11)) >> 12;
                        sum = sum < 0 ? 0 : (sum > max_val ? max_val : sum);
                        *dst_pel = sum;
                    }
                }
            }
        }

        pel * dst = src_img;
        pel * src = tmp_img;
        for (int i = 0; i < height; i++)
        {
            evc_mcpy(dst, src, sizeof(pel) * width);
            dst += src_s;
            src += width;
        }

    }

    evc_mfree(tmp_img);
}

static void apply_filter(EVCE_CTX * ctx, EVC_PIC * tmp_pic[5][3], int pic_cnt, int curr_idx, double overall_strength)
{
    const double sigma_zero_point = 10.0;
    const double sigma_multiplier = 9.0;
    const double chroma_factor = 0.55;
    const double ref_strengths[3][2] =
    { // abs(POC offset)
      //  1,    2
      {0.85, 0.60},  // s_range * 2
      {1.20, 1.00},  // s_range
      {0.30, 0.30}   // otherwise
    };

    const double luma_sigma_sq = (ctx->param.qp - sigma_zero_point) * (ctx->param.qp - sigma_zero_point) * sigma_multiplier;
    const double chroma_sigma_sq = 30 * 30;

    const int chroma_idc = CF_FROM_CS(tmp_pic[curr_idx][0]->imgb->cs);
    const int bit_depth = BD_FROM_CS(tmp_pic[curr_idx][0]->imgb->cs);
    int cs_num = chroma_idc == 0 ? 1 : 3;

    pel * tmp_dst = (pel *)evc_malloc(sizeof(pel) * tmp_pic[curr_idx][0]->h_l * tmp_pic[curr_idx][0]->w_l);


    int ref_strength_row = 2;
    if (pic_cnt - 1 == 4)
    {
        ref_strength_row = 0;
    }
    else if (pic_cnt - 1 == 2)
    {
        ref_strength_row = 1;
    }

    const pel max_val = (1 << bit_depth) - 1;
    const double bd_diff_weighting = 1024.0 / (max_val + 1);

    for (int c = 0; c < cs_num; c++)
    {
        int height, width;
        pel *src_pel_row, *dst_pel_row;
        int src_s, dst_s;
        double sigma_sq, weight_scaling;

        if (c == Y_C)
        {
            height = tmp_pic[curr_idx][0]->h_l;
            width  = tmp_pic[curr_idx][0]->w_l;
            src_pel_row = tmp_pic[curr_idx][0]->y;
            src_s = tmp_pic[curr_idx][0]->s_l;
            dst_pel_row = ctx->pico->pic.y;
            dst_s = ctx->pico->pic.s_l;
            sigma_sq = luma_sigma_sq;
            weight_scaling = overall_strength *  0.4;
        }
        else
        {
            if (c == U_C)
            {
                src_pel_row = tmp_pic[curr_idx][0]->u;
                dst_pel_row = ctx->pico->pic.u;
            }
            else
            {
                src_pel_row = tmp_pic[curr_idx][0]->v;
                dst_pel_row = ctx->pico->pic.v;
            }
            height = tmp_pic[curr_idx][0]->h_c;
            width  = tmp_pic[curr_idx][0]->w_c;
            src_s = tmp_pic[curr_idx][0]->s_c;
            dst_s = ctx->pico->pic.s_c;
            sigma_sq = chroma_sigma_sq;
            weight_scaling = overall_strength * chroma_factor;
        }

        for (int y = 0; y < height; y++, src_pel_row += src_s, dst_pel_row += dst_s)
        {
            const pel *src_pel = src_pel_row;
            pel *dst_pel = dst_pel_row;
            for (int x = 0; x < width; x++, src_pel++, dst_pel++)
            {
                const int org_val = (int)*src_pel;
                double temporal_weight_sum = 1.0;
                double new_val = (double)org_val;
                for (int i = 0; i < pic_cnt; i++)
                {
                    if (i == curr_idx)
                    {
                        continue;
                    }

                    pel *corrected_pel;
                    if (c == Y_C)
                    {
                        corrected_pel = tmp_pic[i][0]->y + (y * tmp_pic[i][0]->s_l + x);
                    }
                    else if(c == U_C)
                    {
                        corrected_pel = tmp_pic[i][0]->u + (y * tmp_pic[i][0]->s_c + x);
                    }
                    else
                    {
                        corrected_pel = tmp_pic[i][0]->v + (y * tmp_pic[i][0]->s_c + x);
                    }

                    const int ref_val = (int)*corrected_pel;
                    double diff = (double)(ref_val - org_val);
                    diff *= bd_diff_weighting;
                    double diff_sq = diff * diff;
                    const int index = EVC_MIN(1, EVC_ABS(curr_idx - i) - 1);
                    const double weight = weight_scaling * ref_strengths[ref_strength_row][index] * exp(-diff_sq / (2 * sigma_sq));
                    new_val += weight * ref_val;
                    temporal_weight_sum += weight;
                }
                new_val /= temporal_weight_sum;
                pel sampleVal = (pel)round(new_val);
                sampleVal = (sampleVal < 0 ? 0 : (sampleVal > max_val ? max_val : sampleVal));
                *dst_pel = sampleVal;
            }
        }
    }
    evc_mfree(tmp_dst);
}

static void evce_gen_subpic(pel * src_y, pel * dst_y, int w, int h, int s_s, int d_s, int bit_depth)
{
    pel * src_b, *src_t;
    pel * dst;
    int   x, k, y, shift;

    src_t = src_y;
    src_b = src_t + s_s;
    dst = dst_y;
    shift = 2; 

    for (y = 0; y < h; y++)
    {
        for (x = 0; x < w; x++)
        {
            k = x << 1;
            dst[x] = (pel)((src_t[k] + src_b[k] + src_t[k + 1] + src_b[k + 1] + (1 << (shift - 1))) >> shift);
        }
        src_t += (s_s << 1);
        src_b += (s_s << 1);
        dst += d_s;
    }
}

void evce_temporal_filter(EVCE_CTX * ctx, EVC_IMGB * img_list[EVCE_TF_FRAME_NUM], int curr_fr)
{
    PICBUF_ALLOCATOR sub_pa[3] = { 0 };
    EVC_PIC * tmp_pic[EVCE_TF_FRAME_NUM][3] = { NULL };
    int first_fr, end_fr, ret;
    int search_range = EVCE_TF_RANGE;

    first_fr = curr_fr - search_range;
    end_fr   = curr_fr + search_range;

    double strength = 0;

    if (curr_fr % ctx->param.gop_size == 0)
    {
        strength = 1.5;
    }
    else if (curr_fr % (ctx->param.gop_size >> 1) == 0)
    {
        strength = 0.95;
    }
    else
    {
        return;
    }

    create_sub_pa(&sub_pa[0], &ctx->pa, 0);
    create_sub_pa(&sub_pa[1], &sub_pa[0], 1);
    create_sub_pa(&sub_pa[2], &sub_pa[1], 1);

    /* get sub pic */
    int curr_idx = EVCE_TF_CR, pic_cnt = 0;
    for (int i = 0; i < EVCE_TF_FRAME_NUM; i++)
    {

        if (img_list[i] != NULL)
        {
            for (int s = 0; s < 3; s++)
            {
                tmp_pic[pic_cnt][s] = ctx->pa.fn_alloc(&sub_pa[s], &ret);
                if (s == 0)
                {
                    evce_imgb_cpy(tmp_pic[pic_cnt][s]->imgb, img_list[i]);
                }
                else
                {
                    evce_gen_subpic(tmp_pic[pic_cnt][s - 1]->y, tmp_pic[pic_cnt][s]->y, tmp_pic[pic_cnt][s]->w_l, tmp_pic[pic_cnt][s]->h_l
                                  , tmp_pic[pic_cnt][s - 1]->s_l, tmp_pic[pic_cnt][s]->s_l, BD_FROM_CS(img_list[i]->cs));

                    evce_gen_subpic(tmp_pic[pic_cnt][s - 1]->u, tmp_pic[pic_cnt][s]->u, tmp_pic[pic_cnt][s]->w_c, tmp_pic[pic_cnt][s]->h_c
                                  , tmp_pic[pic_cnt][s - 1]->s_c, tmp_pic[pic_cnt][s]->s_c, BD_FROM_CS(img_list[i]->cs));

                    evce_gen_subpic(tmp_pic[pic_cnt][s - 1]->v, tmp_pic[pic_cnt][s]->v, tmp_pic[pic_cnt][s]->w_c, tmp_pic[pic_cnt][s]->h_c
                                  , tmp_pic[pic_cnt][s - 1]->s_c, tmp_pic[pic_cnt][s]->s_c, BD_FROM_CS(img_list[i]->cs));
                }
                ctx->fn_picbuf_expand(ctx, tmp_pic[pic_cnt][s]);
            }
            pic_cnt++;
        }
        else
        {
            curr_idx--;
        }

    }

    if (pic_cnt > 1)
    {
        /* get motion */
        for (int i = 0; i < pic_cnt; i++)
        {
            if (i == curr_idx)
            {
                continue;
            }
            get_motion(tmp_pic[curr_idx][2], tmp_pic[i][2], 16, NULL, 1, 0);
            get_motion(tmp_pic[curr_idx][1], tmp_pic[i][1], 16, tmp_pic[i][2]->map_mv, 2, 0);
            get_motion(tmp_pic[curr_idx][0], tmp_pic[i][0], 16, tmp_pic[i][1]->map_mv, 2, 0);
            get_motion(tmp_pic[curr_idx][0], tmp_pic[i][0], 8, tmp_pic[i][0]->map_mv, 1, 1);
        }

        fflush(stdout);

        /* apply motion */
        for (int i = 0; i < pic_cnt; i++)
        {
            if (i == curr_idx)
            {
                continue;
            }
            apply_motion(tmp_pic[i][0]);
        }

        /* bilateral filter */
        apply_filter(ctx, tmp_pic, pic_cnt, curr_idx, strength);
    }

    /* free buffer */
    for (int i = 0; i < pic_cnt; i++)
    {
        for (int s = 0; s < 3; s++)
        {
            if (tmp_pic[i][s] != NULL)
            {
                evce_pic_free(&sub_pa[s], tmp_pic[i][s]);
            }
        }
    }

    return;
}