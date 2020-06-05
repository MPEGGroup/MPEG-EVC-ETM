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


void evcd_picbuf_expand(EVCD_CTX * ctx, EVC_PIC * pic)
{
    evc_picbuf_expand(pic, pic->pad_l, pic->pad_c);
}

EVC_PIC * evcd_picbuf_alloc(PICBUF_ALLOCATOR * pa, int * ret)
{
    return evc_picbuf_alloc(pa->w, pa->h, pa->pad_l, pa->pad_c, ret);
}

void evcd_picbuf_free(PICBUF_ALLOCATOR * pa, EVC_PIC * pic)
{
    evc_picbuf_free(pic);
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
int evcd_picbuf_check_signature(EVC_PIC * pic, u8 signature[N_C][16], int tool_dra, void* pps_draParams, u16 width, u16 height)
{
    u8 pic_sign[N_C][16] = { {0} };
    int ret;
    if (tool_dra)
    {

        EVC_IMGB *imgb_hdr_md5 = NULL;
        WCGDDRAControl l_dra_control;
        WCGDDRAControl *local_g_dra_control = &l_dra_control;
        SignalledParamsDRA* p_pps_draParams = (SignalledParamsDRA*)pps_draParams;
        memcpy(&(local_g_dra_control->m_signalledDRA), p_pps_draParams, sizeof(SignalledParamsDRA));
        evcd_initDRA(local_g_dra_control);
        int align[EVC_IMGB_MAX_PLANE] = { MIN_CU_SIZE, MIN_CU_SIZE >> 1, MIN_CU_SIZE >> 1 };
        int pad[EVC_IMGB_MAX_PLANE] = { 0, 0, 0, };
        imgb_hdr_md5 = evc_imgb_create(width, height, EVC_COLORSPACE_YUV420_10LE, 0, pad, align);
        if (imgb_hdr_md5 == NULL)
        {
            printf("Cannot get original image buffer (DRA)\n");
            return -1;
        }
        imgb_cpy(imgb_hdr_md5, pic->imgb);  // store copy of the reconstructed picture in DPB
        evc_apply_dra_chroma_plane(imgb_hdr_md5, imgb_hdr_md5, local_g_dra_control, 1, TRUE);
        evc_apply_dra_chroma_plane(imgb_hdr_md5, imgb_hdr_md5, local_g_dra_control, 2, TRUE);
        evc_apply_dra_luma_plane(imgb_hdr_md5, imgb_hdr_md5, local_g_dra_control, 0, TRUE);
        /* execute MD5 digest here */
        ret = evc_md5_imgb(imgb_hdr_md5, pic_sign);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
        imgb_hdr_md5->release(imgb_hdr_md5);
    }
    else
    {
        /* execute MD5 digest here */
        ret = evc_picbuf_signature(pic, pic_sign);
        evc_assert_rv(EVC_SUCCEEDED(ret), ret);
    }

    if (memcmp(signature, pic_sign[0], N_C * 16) != 0)
    {
        return EVC_ERR_BAD_CRC;
    }

    memcpy(pic->digest, pic_sign, N_C * 16);

    return EVC_OK;
}
#endif
#if !HDR_MD5_CHECK
int evcd_picbuf_check_signature(EVC_PIC * pic, u8 signature[16])
{
    u8 pic_sign[16];
    int ret;

    /* execute MD5 digest here */
    ret = evc_picbuf_signature(pic, pic_sign);
    evc_assert_rv(EVC_SUCCEEDED(ret), ret);
    if (memcmp(signature, pic_sign, 16) != 0)
    {
        return EVC_ERR_BAD_CRC;
    }

    return EVC_OK;
}
#endif
void evcd_set_affine_mvf(EVCD_CTX * ctx, EVCD_CORE * core)
{
    int   w_cu;
    int   h_cu;
    int   scup;
    int   w_scu;
    int   lidx;
    int   vertex_num = core->affine_flag + 1;
    int   aff_scup[VER_NUM];

    scup = core->scup;
    w_cu = (1 << core->log2_cuw) >> MIN_CU_LOG2;
    h_cu = (1 << core->log2_cuh) >> MIN_CU_LOG2;
    w_scu = ctx->w_scu;

    aff_scup[0] = 0;
    aff_scup[1] = (w_cu - 1);
    aff_scup[2] = (h_cu - 1) * w_scu;
    aff_scup[3] = (w_cu - 1) + (h_cu - 1) * w_scu;

    // derive sub-block size
    int sub_w = 4, sub_h = 4;
    derive_affine_subblock_size_bi( core->affine_mv, core->refi, (1 << core->log2_cuw), (1 << core->log2_cuh), &sub_w, &sub_h, vertex_num, NULL);

    int   sub_w_in_scu = PEL2SCU( sub_w );
    int   sub_h_in_scu = PEL2SCU( sub_h );
    int   half_w = sub_w >> 1;
    int   half_h = sub_h >> 1;

    for(lidx = 0; lidx < REFP_NUM; lidx++)
    {
        if(core->refi[lidx] >= 0)
        {
            s16( *ac_mv )[MV_D] = core->affine_mv[lidx];
            int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
            int mv_scale_hor = ac_mv[0][MV_X] << 7;
            int mv_scale_ver = ac_mv[0][MV_Y] << 7;
            int mv_y_hor = mv_scale_hor;
            int mv_y_ver = mv_scale_ver;
            int mv_scale_tmp_hor, mv_scale_tmp_ver;

            // convert to 2^(storeBit + iBit) precision
            dmv_hor_x = (ac_mv[1][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuw);     // deltaMvHor
            dmv_hor_y = (ac_mv[1][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuw);
            if ( vertex_num == 3 )
            {
                dmv_ver_x = (ac_mv[2][MV_X] - ac_mv[0][MV_X]) << (7 - core->log2_cuh); // deltaMvVer
                dmv_ver_y = (ac_mv[2][MV_Y] - ac_mv[0][MV_Y]) << (7 - core->log2_cuh);
            }
            else
            {
                dmv_ver_x = -dmv_hor_y;                                                // deltaMvVer
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
#if AFFINE_CLIPPING_BF
                        mv_scale_tmp_hor = EVC_CLIP3( -(1 << 17), (1 << 17) - 1, mv_scale_tmp_hor );
                        mv_scale_tmp_ver = EVC_CLIP3( -(1 << 17), (1 << 17) - 1, mv_scale_tmp_ver );
#else
                        mv_scale_tmp_hor = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_hor );
                        mv_scale_tmp_ver = EVC_CLIP3( -(2 << 17), (2 << 17) - 1, mv_scale_tmp_ver );
#endif

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

void evcd_set_dec_info(EVCD_CTX * ctx, EVCD_CORE * core
#if ENC_DEC_TRACE
                       , u8 write_trace
#endif
)
{
    s8  (*map_refi)[REFP_NUM];
    s16 (*map_mv)[REFP_NUM][MV_D];
#if DMVR_LAG
    s16(*map_unrefined_mv)[REFP_NUM][MV_D];
    u32 idx;
#endif
    u32  *map_scu;
    s8   *map_ipm;
    u8   *map_ats_inter;
    int   w_cu;
    int   h_cu;
    int   scup;
    int   w_scu;
    int   i, j;
    int   flag;

    u32  *map_affine;
    u32  *map_cu_mode;

    scup = core->scup;
    w_cu = (1 << core->log2_cuw) >> MIN_CU_LOG2;
    h_cu = (1 << core->log2_cuh) >> MIN_CU_LOG2;
    w_scu = ctx->w_scu;
    map_refi = ctx->map_refi + scup;
    map_scu  = ctx->map_scu + scup;
    map_mv   = ctx->map_mv + scup;
#if DMVR_LAG
    map_unrefined_mv = ctx->map_unrefined_mv + scup;
#endif
    map_ipm  = ctx->map_ipm + scup;

    flag = (core->pred_mode == MODE_INTRA) ? 1 : 0;
    map_affine = ctx->map_affine + scup;
    map_cu_mode = ctx->map_cu_mode + scup;
    map_ats_inter = ctx->map_ats_inter + scup;

#if DMVR_LAG
    idx = 0;
#endif
#if M50761_CHROMA_NOT_SPLIT
    if (evcd_check_luma(ctx, core))
    {
#endif
    for(i = 0; i < h_cu; i++)
    {
        for(j = 0; j < w_cu; j++)
        {
            if(core->pred_mode == MODE_SKIP)
            {
                MCU_SET_SF(map_scu[j]);
            }
            else
            {
                MCU_CLR_SF(map_scu[j]);
            }
#if DMVR_FLAG
            if((core->pred_mode == MODE_SKIP) || (core->pred_mode == MODE_DIR))
            {
                if(core->dmvr_flag)
                {
                    MCU_SET_DMVRF(map_scu[j]);
                }
                else
                {
                    MCU_CLR_DMVRF(map_scu[j]);
                }
            }
#endif
            int sub_idx = ((!!(i & 32)) << 1) | (!!(j & 32));
            if (core->is_coef_sub[Y_C][sub_idx])
            {
                MCU_SET_CBFL(map_scu[j]);
            }
            else
            {
                MCU_CLR_CBFL(map_scu[j]);
            }

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

            if(core->mmvd_flag)
            {
                MCU_SET_MMVDS(map_cu_mode[j]);
            }
            else
            {
                MCU_CLR_MMVDS(map_cu_mode[j]);
            }

#if DQP
            if(ctx->pps.cu_qp_delta_enabled_flag)
            {
                MCU_RESET_QP(map_scu[j]);
                MCU_SET_IF_COD_SN_QP(map_scu[j], flag, ctx->slice_num, core->qp);
            }
            else
            {
                MCU_SET_IF_COD_SN_QP(map_scu[j], flag, ctx->slice_num, ctx->tile[core->tile_num].qp);
            }
#else
            MCU_SET_IF_COD_SN_QP(map_scu[j], flag, ctx->slice_num, ctx->tile[core->tile_num].qp);
#endif

            map_refi[j][REFP_0] = core->refi[REFP_0];
            map_refi[j][REFP_1] = core->refi[REFP_1];
            map_ats_inter[j] = core->ats_inter_info;
            if (core->pred_mode == MODE_IBC)
            {
                map_ats_inter[j] = 0;
            }

#if DMVR_LAG
            if(core->dmvr_flag)
            {
                map_mv[j][REFP_0][MV_X] = core->dmvr_mv[idx + j][REFP_0][MV_X];
                map_mv[j][REFP_0][MV_Y] = core->dmvr_mv[idx + j][REFP_0][MV_Y];
                map_mv[j][REFP_1][MV_X] = core->dmvr_mv[idx + j][REFP_1][MV_X];
                map_mv[j][REFP_1][MV_Y] = core->dmvr_mv[idx + j][REFP_1][MV_Y];

                map_unrefined_mv[j][REFP_0][MV_X] = core->mv[REFP_0][MV_X];
                map_unrefined_mv[j][REFP_0][MV_Y] = core->mv[REFP_0][MV_Y];
                map_unrefined_mv[j][REFP_1][MV_X] = core->mv[REFP_1][MV_X];
                map_unrefined_mv[j][REFP_1][MV_Y] = core->mv[REFP_1][MV_Y];
            }
            else
#endif
            {

                map_mv[j][REFP_0][MV_X] = core->mv[REFP_0][MV_X];
                map_mv[j][REFP_0][MV_Y] = core->mv[REFP_0][MV_Y];
                map_mv[j][REFP_1][MV_X] = core->mv[REFP_1][MV_X];
                map_mv[j][REFP_1][MV_Y] = core->mv[REFP_1][MV_Y];
#if DMVR_LAG
                map_unrefined_mv[j][REFP_0][MV_X] = core->mv[REFP_0][MV_X];
                map_unrefined_mv[j][REFP_0][MV_Y] = core->mv[REFP_0][MV_Y];
                map_unrefined_mv[j][REFP_1][MV_X] = core->mv[REFP_1][MV_X];
                map_unrefined_mv[j][REFP_1][MV_Y] = core->mv[REFP_1][MV_Y];
#endif

            }

            map_ipm[j] = core->ipm[0];
        }
        map_refi += w_scu;
        map_mv += w_scu;
#if DMVR_LAG
        map_unrefined_mv += w_scu;
        idx += w_cu;
#endif
        map_scu += w_scu;
        map_ipm += w_scu;

        map_affine += w_scu;
        map_cu_mode += w_scu;
        map_ats_inter += w_scu;
    }

    if (core->ats_inter_info)
    {
        assert(core->is_coef_sub[Y_C][0] == core->is_coef[Y_C]);
        assert(core->is_coef_sub[U_C][0] == core->is_coef[U_C]);
        assert(core->is_coef_sub[V_C][0] == core->is_coef[V_C]);
        set_cu_cbf_flags(core->is_coef[Y_C], core->ats_inter_info, core->log2_cuw, core->log2_cuh, ctx->map_scu + core->scup, ctx->w_scu);
    }

    if(core->affine_flag)
    {
        evcd_set_affine_mvf(ctx, core);
    }

#if HISTORY_LCU_COPY_BUG_FIX
    map_refi = ctx->map_refi + scup;
    map_mv = ctx->map_mv + scup;

    evc_mcpy(core->mv, map_mv, sizeof(core->mv));
    evc_mcpy(core->refi, map_refi, sizeof(core->refi));
#endif
#if M50761_CHROMA_NOT_SPLIT
    }
#endif
#if MVF_TRACE
    // Trace MVF in decoder
#if ENC_DEC_TRACE
    if (write_trace)
#endif
    {
        map_refi = ctx->map_refi + scup;
        map_scu = ctx->map_scu + scup;
        map_mv = ctx->map_mv + scup;
        map_affine = ctx->map_affine + scup;
#if DMVR_LAG
        map_unrefined_mv = ctx->map_unrefined_mv + scup;
#endif

        for(i = 0; i < h_cu; i++)
        {
            for (j = 0; j < w_cu; j++)
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
                if (MCU_GET_AFF(map_scu[j]))
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
            map_refi += w_scu;
            map_mv += w_scu;
            map_scu += w_scu;
            map_affine += w_scu;
#if DMVR_LAG
            map_unrefined_mv += w_scu;
#endif
        }
    }
#endif
}

void evcd_split_tbl_init(EVCD_CTX *ctx)
{
#if M52166_PARTITION
    evc_split_tbl[BLOCK_11][IDX_MAX] = ctx->log2_max_cuwh;
    evc_split_tbl[BLOCK_11][IDX_MIN] = ctx->sps.log2_min_cb_size_minus2 + 2;
    evc_split_tbl[BLOCK_12][IDX_MAX] = ctx->log2_max_cuwh;
    evc_split_tbl[BLOCK_12][IDX_MIN] = evc_split_tbl[BLOCK_11][IDX_MIN] + 1;
    evc_split_tbl[BLOCK_14][IDX_MAX] = min(ctx->log2_max_cuwh - ctx->sps.log2_diff_ctu_max_14_cb_size, 64);
    evc_split_tbl[BLOCK_14][IDX_MIN] = evc_split_tbl[BLOCK_12][IDX_MIN] + 1;
    evc_split_tbl[BLOCK_TT][IDX_MAX] = min(ctx->log2_max_cuwh - ctx->sps.log2_diff_ctu_max_tt_cb_size, 64);
    evc_split_tbl[BLOCK_TT][IDX_MIN] = evc_split_tbl[BLOCK_11][IDX_MIN] + ctx->sps.log2_diff_min_cb_min_tt_cb_size_minus2 + 2;
#else
    evc_split_tbl[0][0] = ctx->log2_max_cuwh - ctx->sps.log2_diff_ctu_max_11_cb_size;
    evc_split_tbl[0][1] = evc_split_tbl[0][0] - ctx->sps.log2_diff_max_11_min_11_cb_size;
    evc_split_tbl[1][0] = evc_split_tbl[0][0] - ctx->sps.log2_diff_max_11_max_12_cb_size;
    evc_split_tbl[1][1] = evc_split_tbl[0][1] + 1 + ctx->sps.log2_diff_min_11_min_12_cb_size_minus1;
    evc_split_tbl[2][0] = evc_split_tbl[1][0] - 1 - ctx->sps.log2_diff_max_12_max_14_cb_size_minus1;
    evc_split_tbl[2][1] = evc_split_tbl[1][1] + 1 + ctx->sps.log2_diff_min_12_min_14_cb_size_minus1;
    evc_split_tbl[3][0] = 0;
    evc_split_tbl[3][1] = 0;
    evc_split_tbl[4][0] = 0;
    evc_split_tbl[4][1] = 0;
    evc_split_tbl[5][0] = evc_split_tbl[0][0] - 1 - ctx->sps.log2_diff_max_11_max_tt_cb_size_minus1;
    evc_split_tbl[5][1] = evc_split_tbl[0][1] + 2 + ctx->sps.log2_diff_min_11_min_tt_cb_size_minus2;
#endif
}

#if USE_DRAW_PARTITION_DEC
void cpy_pic(EVC_PIC * pic_src, EVC_PIC * pic_dst)
{
    int i, aw, ah, s, e, bsize;

    int a_size, p_size;
    for (i = 0; i<3; i++)
    {

        a_size = MIN_CU_SIZE >> (!!i);
        p_size = i ? pic_dst->pad_c : pic_dst->pad_l;

        aw = EVC_ALIGN(pic_dst->w_l >> (!!i), a_size);
        ah = EVC_ALIGN(pic_dst->h_l >> (!!i), a_size);

        s = aw + p_size + p_size;
        e = ah + p_size + p_size;

        bsize = s * ah * sizeof(pel);
        switch (i)
        {
        case 0:
            memcpy(pic_dst->y, pic_src->y, bsize); break;
        case 1:
            memcpy(pic_dst->u, pic_src->u, bsize); break;
        case 2:
            memcpy(pic_dst->v, pic_src->v, bsize); break;
        default:
            break;
        }
    }
}

int write_pic(char * fname, EVC_PIC * pic)
{
    pel    * p;
    int      j;
    FILE   * fp;
    static int cnt = 0;

    if (cnt == 0)
        fp = fopen(fname, "wb");
    else
        fp = fopen(fname, "ab");
    cnt++;
    if (fp == NULL)
    {
        assert(!"cannot open file");
        return -1;
    }

    {
        /* Crop image supported */
        /* luma */
        p = pic->y;
        for (j = 0; j<pic->h_l; j++)
        {
            fwrite(p, pic->w_l, sizeof(pel), fp);
            p += pic->s_l;
        }

        /* chroma */
        p = pic->u;
        for (j = 0; j<pic->h_c; j++)
        {
            fwrite(p, pic->w_c, sizeof(pel), fp);
            p += pic->s_c;
        }

        p = pic->v;
        for (j = 0; j<pic->h_c; j++)
        {
            fwrite(p, pic->w_c, sizeof(pel), fp);
            p += pic->s_c;
        }
    }

    fclose(fp);
    return 0;
}

static int draw_tree(EVCD_CTX * ctx, EVC_PIC * pic, int x, int y,
                     int cuw, int cuh, int cud, int cup, int next_split)
{
    s8      split_mode;
    int     cup_x1, cup_y1;
    int     x1, y1, lcu_num;
    int     dx, dy, cup1, cup2, cup3;

    lcu_num = (x >> ctx->log2_max_cuwh) + (y >> ctx->log2_max_cuwh) * ctx->w_lcu;
    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, ctx->max_cuwh, ctx->map_split[lcu_num]);

    if (split_mode != NO_SPLIT && !(cuw == 4 && cuh == 4))
    {
        if (split_mode == SPLIT_BI_VER)
        {
            int sub_cud = cud + 1;
            int sub_cuw = cuw >> 1;
            int sub_cuh = cuh;

            x1 = x + sub_cuw;
            y1 = y + sub_cuh;

            cup_x1 = cup + (sub_cuw >> MIN_CU_LOG2);
            cup_y1 = (sub_cuh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2);

            draw_tree(ctx, pic, x, y, sub_cuw, sub_cuh, sub_cud, cup, next_split - 1);

            if (x1 < pic->w_l)
            {
                draw_tree(ctx, pic, x1, y, sub_cuw, sub_cuh, sub_cud, cup_x1, next_split - 1);
            }
        }
        else if (split_mode == SPLIT_BI_HOR)
        {
            int sub_cud = cud + 1;
            int sub_cuw = cuw;
            int sub_cuh = cuh >> 1;

            x1 = x + sub_cuw;
            y1 = y + sub_cuh;

            cup_x1 = cup + (sub_cuw >> MIN_CU_LOG2);
            cup_y1 = (sub_cuh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2);

            draw_tree(ctx, pic, x, y, sub_cuw, sub_cuh, sub_cud, cup, next_split - 1);

            if (y1 < pic->h_l)
            {
                draw_tree(ctx, pic, x, y1, sub_cuw, sub_cuh, sub_cud, cup + cup_y1, next_split - 1);
            }
        }
        else if (split_mode == SPLIT_TRI_VER)
        {
            int sub_cud = cud + 2;
            int sub_cuw = cuw >> 2;
            int sub_cuh = cuh;
            int x2, cup_x2;
            
            draw_tree(ctx, pic, x, y, sub_cuw, sub_cuh, sub_cud, cup, next_split - 1);

            x1 = x + sub_cuw;
            cup_x1 = cup + (sub_cuw >> MIN_CU_LOG2);
            sub_cuw = cuw >> 1;
            if (x1 < pic->w_l)
            {
                draw_tree(ctx, pic, x1, y, sub_cuw, sub_cuh, sub_cud - 1, cup_x1, (next_split - 1) == 0 ? 0 : 0);
            }

            x2 = x1 + sub_cuw;
            cup_x2 = cup_x1 + (sub_cuw >> MIN_CU_LOG2);
            sub_cuw = cuw >> 2;
            if (x2 < pic->w_l)
            {
                draw_tree(ctx, pic, x2, y, sub_cuw, sub_cuh, sub_cud, cup_x2, next_split - 1);
            }
        }
        else if (split_mode == SPLIT_TRI_HOR)
        {
            int sub_cud = cud + 2;
            int sub_cuw = cuw;
            int sub_cuh = cuh >> 2;
            int y2, cup_y2;

            draw_tree(ctx, pic, x, y, sub_cuw, sub_cuh, sub_cud, cup, next_split - 1);

            y1 = y + sub_cuh;
            cup_y1 = (sub_cuh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2);
            sub_cuh = cuh >> 1;
            if (y1 < pic->h_l)
            {
                draw_tree(ctx, pic, x, y1, sub_cuw, sub_cuh, sub_cud - 1, cup + cup_y1, (next_split - 1) == 0 ? 0 : 0);
            }

            y2 = y1 + sub_cuh;
            cup_y2 = cup_y1 + (sub_cuh >> MIN_CU_LOG2) * (ctx->max_cuwh >> MIN_CU_LOG2);
            sub_cuh = cuh >> 2;
            if (y2 < pic->h_l)
            {
                draw_tree(ctx, pic, x, y2, sub_cuw, sub_cuh, sub_cud, cup + cup_y2, next_split - 1);
            }
        }
        else if (split_mode == SPLIT_QUAD)
        {
            cud+=2;
            cuw >>= 1;
            cuh >>= 1;
            x1 = x + cuw;
            y1 = y + cuh;
            dx = (cuw >> MIN_CU_LOG2);
            dy = (dx * (ctx->max_cuwh >> MIN_CU_LOG2));

            cup1 = cup + dx;
            cup2 = cup + dy;
            cup3 = cup + dx + dy;

            draw_tree(ctx, pic, x, y, cuw, cuh, cud, cup, next_split - 1);
            if (x1 < pic->w_l)
            {
                draw_tree(ctx, pic, x1, y, cuw, cuh, cud, cup1, next_split - 1);
            }
            if (y1 < pic->h_l)
            {
                draw_tree(ctx, pic, x, y1, cuw, cuh, cud, cup2, next_split - 1);
            }
            if (x1 < pic->w_l && y1 < pic->h_l)
            {
                draw_tree(ctx, pic, x1, y1, cuw, cuh, cud, cup3, next_split - 1);
            }
        }
    }
    else
    {
        int     i, s_l;
        s16 * luma;
        /* draw rectangle */
        s_l = pic->s_l;
        luma = pic->y + (y * s_l) + x;

        for (i = 0; i<cuw; i++) luma[i] = sizeof(pel) << 7;
        for (i = 0; i<cuh; i++) luma[i*s_l] = sizeof(pel) << 7;
    }

    return EVC_OK;
}

void evcd_draw_partition(EVCD_CTX * ctx, EVC_PIC * pic)
{
    int i, j, k, cuw, cuh, s_l;
    s16 * luma;

    EVC_PIC * tmp;
    int * ret = NULL;
    char file_name[256];

    tmp = evc_picbuf_alloc(ctx->w, ctx->h, pic->pad_l, pic->pad_c, ret);

    cpy_pic(pic, tmp);

    /* CU partition line */
    for (i = 0; i<ctx->h_lcu; i++)
    {
        for (j = 0; j<ctx->w_lcu; j++)
        {
            draw_tree(ctx, tmp, (j << ctx->log2_max_cuwh), (i << ctx->log2_max_cuwh), ctx->max_cuwh, ctx->max_cuwh, 0, 0, 255);
        }
    }

    s_l = tmp->s_l;
    luma = tmp->y;

    /* LCU boundary line */
    for (i = 0; i<ctx->h; i += ctx->max_cuwh)
    {
        for (j = 0; j<ctx->w; j += ctx->max_cuwh)
        {
            cuw = j + ctx->max_cuwh > ctx->w ? ctx->w - j : ctx->max_cuwh;
            cuh = i + ctx->max_cuwh > ctx->h ? ctx->h - i : ctx->max_cuwh;
            
            for (k = 0; k<cuw; k++)
            {
                luma[i*s_l + j + k] = 0;
            }
                
            for (k = 0; k < cuh; k++)
            {
                luma[(i + k)*s_l + j] = 0;
            }
        }
    }

    sprintf(file_name, "dec_partition_%dx%d.yuv", pic->w_l, pic->h_l);

    write_pic(file_name, tmp);
    evc_picbuf_free(tmp);
}
#endif


void evcd_get_mmvd_motion(EVCD_CTX * ctx, EVCD_CORE * core)
{
    int real_mv[MMVD_GRP_NUM * MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM][2][3];
#if M52166_MMVD
    int REF_SET[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME] = { {0,0,}, };
#else
    int REF_SET[3][MAX_NUM_ACTIVE_REF_FRAME] = { {0,0,}, };
#endif
    int cuw, cuh;

    for (int k = 0; k < MAX_NUM_ACTIVE_REF_FRAME; k++)
    {
        REF_SET[0][k] = ctx->refp[k][0].poc;
        REF_SET[1][k] = ctx->refp[k][1].poc;
    }
#if !M52166_MMVD
    REF_SET[2][0] = ctx->poc.poc_val;
    REF_SET[2][1] = ctx->dpm.cur_num_ref_pics;
#endif
    cuw = (1 << core->log2_cuw);
    cuh = (1 << core->log2_cuh);

    evc_get_mmvd_mvp_list(ctx->map_refi, ctx->refp[0], ctx->map_mv, ctx->w_scu, ctx->h_scu, core->scup, core->avail_cu, core->log2_cuw, core->log2_cuh, ctx->sh.slice_type, real_mv, ctx->map_scu, REF_SET, core->avail_lr
#if M52166_MMVD
        , ctx->poc.poc_val, ctx->dpm.num_refp
#endif
        , core->history_buffer, ctx->sps.tool_admvp, &ctx->sh, ctx->log2_max_cuwh, ctx->map_tidx, core->mmvd_idx);

    core->mv[REFP_0][MV_X] = real_mv[core->mmvd_idx][0][MV_X];
    core->mv[REFP_0][MV_Y] = real_mv[core->mmvd_idx][0][MV_Y];
    core->refi[REFP_0] = real_mv[core->mmvd_idx][0][2];;

    if (ctx->sh.slice_type == SLICE_B)
    {
        core->refi[REFP_1] = real_mv[core->mmvd_idx][1][2];
        core->mv[REFP_1][MV_X] = real_mv[core->mmvd_idx][1][MV_X];
        core->mv[REFP_1][MV_Y] = real_mv[core->mmvd_idx][1][MV_Y];
    }

#if !MMVD_CLEANUP
    if ((ctx->sh.slice_type == SLICE_P) || (!check_bi_applicability(ctx->sh.slice_type, cuw, cuh, ctx->sps.tool_admvp)))
    {
        core->refi[REFP_1] = -1;
    }
#endif
}

#if M50761_CHROMA_NOT_SPLIT
u8 evcd_check_luma(EVCD_CTX *ctx, EVCD_CORE * core)
{
    return evc_check_luma(core->tree_cons);
}

u8 evcd_check_chroma(EVCD_CTX *ctx, EVCD_CORE * core)
{
    return evc_check_chroma(core->tree_cons);
}
u8 evcd_check_all(EVCD_CTX *ctx, EVCD_CORE * core)
{
    return evc_check_all(core->tree_cons);
}

u8 evcd_check_only_intra(EVCD_CTX *ctx, EVCD_CORE * core)
{
    return evc_check_only_intra(core->tree_cons);
}

u8 evcd_check_only_inter(EVCD_CTX *ctx, EVCD_CORE * core)
{
    return evc_check_only_inter(core->tree_cons);
}

u8 evcd_check_all_preds(EVCD_CTX *ctx, EVCD_CORE * core)
{
    return evc_check_all_preds(core->tree_cons);
}

MODE_CONS evcd_derive_mode_cons(EVCD_CTX *ctx, int scup)
{
    return ( MCU_GET_IF(ctx->map_scu[scup]) || MCU_GET_IBC(ctx->map_scu[scup]) ) ? eOnlyIntra : eOnlyInter;
}
#endif