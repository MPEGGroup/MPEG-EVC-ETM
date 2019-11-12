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

#include "evc_util.h"
#include "evc_df.h"
#include "evce_def.h"
#include "evce_util.h"

/******************************************************************************
 * picture buffer alloc/free/expand
 ******************************************************************************/
void evce_picbuf_expand(EVCE_CTX *ctx, EVC_PIC *pic)
{
    evc_picbuf_expand(pic, pic->pad_l, pic->pad_c);
}

EVC_PIC * evce_pic_alloc(PICBUF_ALLOCATOR * pa, int * ret)
{
    return evc_picbuf_alloc(pa->w, pa->h, pa->pad_l, pa->pad_c, ret);
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

void evce_diff_pred(int x, int y, int log2_cuw, int log2_cuh, EVC_PIC *org, pel pred[N_C][MAX_CU_DIM], s16 diff[N_C][MAX_CU_DIM])
{
    pel * buf;
    int cuw, cuh, stride;

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;
    stride = org->s_l;

    /* Y */
    buf = org->y + (y * stride) + x;

    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[Y_C], stride, cuw, cuw, diff[Y_C]);

    cuw >>= 1;
    cuh >>= 1;
    x >>= 1;
    y >>= 1;
    log2_cuw--;
    log2_cuh--;

    stride = org->s_c;

    /* U */
    buf = org->u + (y * stride) + x;
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[U_C], stride, cuw, cuw, diff[U_C]);

    /* V */
    buf = org->v + (y * stride) + x;
    evce_diff_16b(log2_cuw, log2_cuh, buf, pred[V_C], stride, cuw, cuw, diff[V_C]);
}

#if AFFINE && RDO_DBK
void evc_set_affine_mvf(EVCE_CTX * ctx, EVCE_CORE * core, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][VER_NUM][MV_D], int vertex_num)
{
#if !M48933_AFFINE
    s16 (*map_mv)[REFP_NUM][MV_D];
#endif
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

#if M50761_AFFINE_ADAPT_SUB_SIZE
    // derive sub-block size
    int sub_w = 4, sub_h = 4;
    derive_affine_subblock_size_bi( mv, refi, core->cuw, core->cuh, &sub_w, &sub_h, vertex_num
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV_HW
      , NULL
#endif
    );

    int   sub_w_in_scu = PEL2SCU( sub_w );
    int   sub_h_in_scu = PEL2SCU( sub_h );
    int   half_w = sub_w >> 1;
    int   half_h = sub_h >> 1;
#endif

    for (lidx = 0; lidx < REFP_NUM; lidx++)
    {
        if (refi[lidx] >= 0)
        {
#if !M48933_AFFINE
#endif
            s16(*ac_mv)[MV_D] = mv[lidx];

            int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
#if M48933_AFFINE
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

#if !M50761_AFFINE_ADAPT_SUB_SIZE
            // derive sub-block size
            int sub_w = 4, sub_h = 4;
            derive_affine_subblock_size( mv[lidx], core->cuw, core->cuh, &sub_w, &sub_h, vertex_num );
            int   sub_w_in_scu = PEL2SCU( sub_w );
            int   sub_h_in_scu = PEL2SCU( sub_h );
            int   half_w = sub_w >> 1;
            int   half_h = sub_h >> 1;
#endif

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
                        mv_scale_tmp_hor = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor );
                        mv_scale_tmp_ver = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver );

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
#else
            int mv_scale_tmp_hor, mv_scale_tmp_ver;

            // convert to 2^(storeBit + iBit) precision
            dmv_hor_x = (ac_mv[1][MV_X] - ac_mv[0][MV_X]) << (7 - log2_cuw);     // deltaMvHor
            dmv_hor_y = (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << (7 - log2_cuw);
            if (vertex_num == 3)
            {
                dmv_ver_x = (ac_mv[2][MV_X] - ac_mv[0][MV_X]) << (7 - log2_cuh); // deltaMvVer
                dmv_ver_y = (ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << (7 - log2_cuh);
            }
            else
            {
                dmv_ver_x = -dmv_hor_y;                                          // deltaMvVer
                dmv_ver_y = dmv_hor_x;
            }

            map_mv = ctx->map_mv + scup;
            for (i = 0; i < h_cu; i++)
            {
                for (j = 0; j < w_cu; j++)
                {
                    int pos_x = (j << MIN_CU_LOG2) + 2;
                    int pos_y = (i << MIN_CU_LOG2) + 2;

                    mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
                    mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

                    map_mv[j][lidx][MV_X] = mv_scale_tmp_hor >> 7;
                    map_mv[j][lidx][MV_Y] = mv_scale_tmp_ver >> 7;
                }
                map_mv += w_scu;
            }

            // reset vertex mv
            map_mv = ctx->map_mv + scup;
            for (i = 0; i < vertex_num; i++)
            {
                map_mv[aff_scup[i]][lidx][MV_X] = ac_mv[i][MV_X];
                map_mv[aff_scup[i]][lidx][MV_Y] = ac_mv[i][MV_Y];
            }
            if (vertex_num == 2) // reset lt vertex mv
            {
                s16 vx2 = ac_mv[0][MV_X] - (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) * h_cu / w_cu;
                s16 vy2 = ac_mv[0][MV_Y] + (ac_mv[1][MV_X] - ac_mv[0][MV_X]) * h_cu / w_cu;
                map_mv[aff_scup[2]][lidx][MV_X] = vx2;
                map_mv[aff_scup[2]][lidx][MV_Y] = vy2;
            }
#endif
        }
    }
}
#endif

void evce_split_tbl_init(EVCE_CTX *ctx)
{
    evc_split_tbl[0][0] = ctx->cdsc.framework_cu11_max;
    evc_split_tbl[0][1] = ctx->cdsc.framework_cu11_min;
    evc_split_tbl[1][0] = ctx->cdsc.framework_cu12_max;
    evc_split_tbl[1][1] = ctx->cdsc.framework_cu12_min;
    evc_split_tbl[2][0] = ctx->cdsc.framework_cu14_max;
    evc_split_tbl[2][1] = ctx->cdsc.framework_cu14_min;
    evc_split_tbl[3][0] = 0;
    evc_split_tbl[3][1] = 0;
    evc_split_tbl[4][0] = 0;
    evc_split_tbl[4][1] = 0;
    evc_split_tbl[5][0] = ctx->cdsc.framework_tris_max;
    evc_split_tbl[5][1] = ctx->cdsc.framework_tris_min;
}

#if M50761_CHROMA_NOT_SPLIT
u8 evce_check_luma(EVCE_CTX *ctx)
{
    return evc_check_luma(ctx->tree_cons);
}

u8 evce_check_chroma(EVCE_CTX *ctx)
{
    return evc_check_chroma(ctx->tree_cons);
}
u8 evce_check_all(EVCE_CTX *ctx)
{
    return evc_check_all(ctx->tree_cons);
}

u8 evce_check_only_intra(EVCE_CTX *ctx)
{
    return evc_check_only_intra(ctx->tree_cons);
}

u8 evce_check_only_inter(EVCE_CTX *ctx)
{
    return evc_check_only_inter(ctx->tree_cons);
}

u8 evce_check_all_preds(EVCE_CTX *ctx)
{
    return evc_check_all_preds(ctx->tree_cons);
}

MODE_CONS evce_derive_mode_cons(EVCE_CTX *ctx, int lcu_num, int cup)
{
        return ((ctx->map_cu_data[lcu_num].pred_mode[cup] == MODE_INTRA)
#if IBC
            || (ctx->map_cu_data[lcu_num].pred_mode[cup] == MODE_IBC)
#endif
            ) ? eOnlyIntra : eOnlyInter;
}
#endif