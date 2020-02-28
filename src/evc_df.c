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

#include "evc_def.h"
#include "evc_df.h"
#include "evc_tbl.h"

const u8 * get_tbl_qp_to_st(u32 mcu0, u32 mcu1, s8 *refi0, s8 *refi1, s16 (*mv0)[MV_D], s16 (*mv1)[MV_D])
{
    int idx = 3;

    if(MCU_GET_IF(mcu0) || MCU_GET_IF(mcu1))
    {
        idx = 0;
    }
    else if(MCU_GET_CBFL(mcu0) == 1 || MCU_GET_CBFL(mcu1) == 1)
    {
        idx = 1;
    }
    else if (MCU_GET_IBC(mcu0) || MCU_GET_IBC(mcu1))
    {
      idx = 2;
    }
    else
    {
        int mv0_l0[2] = {mv0[REFP_0][MV_X], mv0[REFP_0][MV_Y]};
        int mv0_l1[2] = {mv0[REFP_1][MV_X], mv0[REFP_1][MV_Y]};
        int mv1_l0[2] = {mv1[REFP_0][MV_X], mv1[REFP_0][MV_Y]};
        int mv1_l1[2] = {mv1[REFP_1][MV_X], mv1[REFP_1][MV_Y]};

        if(!REFI_IS_VALID(refi0[REFP_0]))
        {
            mv0_l0[0] = mv0_l0[1] = 0;
        }

        if(!REFI_IS_VALID(refi0[REFP_1]))
        {
            mv0_l1[0] = mv0_l1[1] = 0;
        }

        if(!REFI_IS_VALID(refi1[REFP_0]))
        {
            mv1_l0[0] = mv1_l0[1] = 0;
        }

        if(!REFI_IS_VALID(refi1[REFP_1]))
        {
            mv1_l1[0] = mv1_l1[1] = 0;
        }

        if(((refi0[REFP_0] == refi1[REFP_0]) && (refi0[REFP_1] == refi1[REFP_1])))
        {
            idx = (EVC_ABS(mv0_l0[MV_X] - mv1_l0[MV_X]) >= 4 ||
                   EVC_ABS(mv0_l0[MV_Y] - mv1_l0[MV_Y]) >= 4 ||
                   EVC_ABS(mv0_l1[MV_X] - mv1_l1[MV_X]) >= 4 ||
                   EVC_ABS(mv0_l1[MV_Y] - mv1_l1[MV_Y]) >= 4 ) ? 2 : 3;
        }
        else if((refi0[REFP_0] == refi1[REFP_1]) && (refi0[REFP_1] == refi1[REFP_0]))
        {
            idx = (EVC_ABS(mv0_l0[MV_X] - mv1_l1[MV_X]) >= 4 ||
                   EVC_ABS(mv0_l0[MV_Y] - mv1_l1[MV_Y]) >= 4 ||
                   EVC_ABS(mv0_l1[MV_X] - mv1_l0[MV_X]) >= 4 ||
                   EVC_ABS(mv0_l1[MV_Y] - mv1_l0[MV_Y]) >= 4) ? 2 : 3;
        }
        else
        {
            idx = 2;
        }
    }

    return evc_tbl_df_st[idx];
}

static void deblock_scu_hor(pel *buf, int qp, int stride, int is_luma, const u8 *tbl_qp_to_st)
{
    s16 A, B, C, D, d, d1, d2;
    s16 abs, t16, clip, sign, st;
    int i, size;

    st = tbl_qp_to_st[qp] << (BIT_DEPTH - 8);
    size = (is_luma ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1));

    if(st)
    {
        for(i = 0; i < size; i++)
        {
            A = buf[-2 * stride];
            B = buf[-stride];
            C = buf[0];
            D = buf[stride];

            d = (A - (B << 2) + (C << 2) - D) / 8;

            abs = EVC_ABS16(d);
            sign = EVC_SIGN_GET(d);

            t16 = EVC_MAX(0, ((abs - st) << 1));
            clip = EVC_MAX(0, (abs - t16));
            d1 = EVC_SIGN_SET(clip, sign);
            clip >>= 1;
            d2 = EVC_CLIP(((A - D) / 4), -clip, clip);

            A -= d2;
            B += d1;
            C -= d1;
            D += d2;


            buf[-2 * stride] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, A);
            buf[-stride] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, B);
            buf[0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, C);
            buf[stride] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, D);
            buf++;
        }
    }
}

static void deblock_scu_hor_chroma(pel *buf, int qp, int stride, int is_luma, const u8 *tbl_qp_to_st)
{
    s16 A, B, C, D, d, d1;
    s16 abs, t16, clip, sign, st;
    int i, size;

    st = tbl_qp_to_st[qp] << (BIT_DEPTH - 8);
    size = (is_luma ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1));

    if(st)
    {
        for(i = 0; i < size; i++)
        {
            A = buf[-2 * stride];
            B = buf[-stride];
            C = buf[0];
            D = buf[stride];

            d = (A - (B << 2) + (C << 2) - D) / 8;

            abs = EVC_ABS16(d);
            sign = EVC_SIGN_GET(d);

            t16 = EVC_MAX(0, ((abs - st) << 1));
            clip = EVC_MAX(0, (abs - t16));
            d1 = EVC_SIGN_SET(clip, sign);

            B += d1;
            C -= d1;

            buf[-stride] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, B);
            buf[0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, C);

            buf++;
        }
    }
}

static void deblock_scu_ver(pel *buf, int qp, int stride, int is_luma, const u8 *tbl_qp_to_st)
{
    s16 A, B, C, D, d, d1, d2;
    s16 abs, t16, clip, sign, st;
    int i, size;

    st = tbl_qp_to_st[qp] << (BIT_DEPTH - 8);
    size = (is_luma ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1));

    if(st)
    {
        for(i = 0; i < size; i++)
        {
            A = buf[-2];
            B = buf[-1];
            C = buf[0];
            D = buf[1];

            d = (A - (B << 2) + (C << 2) - D) / 8;

            abs = EVC_ABS16(d);
            sign = EVC_SIGN_GET(d);

            t16 = EVC_MAX(0, ((abs - st) << 1));
            clip = EVC_MAX(0, (abs - t16));
            d1 = EVC_SIGN_SET(clip, sign);
            clip >>= 1;
            d2 = EVC_CLIP(((A - D) / 4), -clip, clip);

            A -= d2;
            B += d1;
            C -= d1;
            D += d2;

            buf[-2] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, A);
            buf[-1] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, B);
            buf[0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, C);
            buf[1] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, D);
            buf += stride;
        }
    }
}

static void deblock_scu_ver_chroma(pel *buf, int qp, int stride, int is_luma, const u8 *tbl_qp_to_st)
{
    s16 A, B, C, D, d, d1;
    s16 abs, t16, clip, sign, st;
    int i, size;

    st = tbl_qp_to_st[qp] << (BIT_DEPTH - 8);
    size = (is_luma ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1));

    if(st)
    {
        for(i = 0; i < size; i++)
        {
            A = buf[-2];
            B = buf[-1];
            C = buf[0];
            D = buf[1];

            d = (A - (B << 2) + (C << 2) - D) / 8;

            abs = EVC_ABS16(d);
            sign = EVC_SIGN_GET(d);

            t16 = EVC_MAX(0, ((abs - st) << 1));
            clip = EVC_MAX(0, (abs - t16));
            d1 = EVC_SIGN_SET(clip, sign);

            B += d1;
            C -= d1;

            buf[-1] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, B);
            buf[0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, C);

            buf += stride;
        }
    }
}

static void deblock_h263_cu_hor(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
)
{
    pel       * y, *u, *v;
    const u8  * tbl_qp_to_st;
    int         i, t, qp, s_l, s_c;
    int w = cuw >> MIN_CU_LOG2;
    int h = cuh >> MIN_CU_LOG2;
    u32 *map_scu_tmp;
    int j;
#if EVC_TILE_SUPPORT
    int  t1, t_copy; // Next row scu number
#endif
    t = (x_pel >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
#if EVC_TILE_SUPPORT
    t_copy = t;
    t1 = (x_pel >> MIN_CU_LOG2) + ((y_pel - (1 << MIN_CU_LOG2)) >> MIN_CU_LOG2) * w_scu;
#endif
    map_scu += t;
    map_refi += t;
    map_mv += t;
    map_scu_tmp = map_scu;
    s_l = pic->s_l;
    s_c = pic->s_c;
    y = pic->y + x_pel + y_pel * s_l;
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    u = pic->u + t;
    v = pic->v + t;

    /* horizontal filtering */
    if(y_pel > 0
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif      
        )
    {
        for(i = 0; i < (cuw >> MIN_CU_LOG2); i++)
        {
            tbl_qp_to_st = get_tbl_qp_to_st(map_scu[i], map_scu[i - w_scu], map_refi[i], map_refi[i - w_scu], map_mv[i], map_mv[i - w_scu]);

            qp = MCU_GET_QP(map_scu[i]);
            t = (i << MIN_CU_LOG2);
#if M50761_CHROMA_NOT_SPLIT
            if (evc_check_luma(tree_cons))
#endif
            {
                deblock_scu_hor(y + t, qp, s_l, 1, tbl_qp_to_st);
            }
#if M50761_CHROMA_NOT_SPLIT
            if (evc_check_chroma(tree_cons))
            {
#endif
            deblock_scu_hor_chroma(u + (t >> 1), p_evc_tbl_qp_chroma_dynamic[0][qp], s_c, 0, tbl_qp_to_st);
            deblock_scu_hor_chroma(v + (t >> 1), p_evc_tbl_qp_chroma_dynamic[1][qp], s_c, 0, tbl_qp_to_st);
#if M50761_CHROMA_NOT_SPLIT
            }
#endif
        }
    }

    map_scu = map_scu_tmp;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            MCU_SET_COD(map_scu[j]);
        }
        map_scu += w_scu;
    }
}

static void deblock_h263_cu_ver(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu
#if FIX_PARALLEL_DBF
                              , u32  *map_cu
#endif
#if M50761_CHROMA_NOT_SPLIT
                              , TREE_CONS tree_cons
#endif
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
                              )
{
    pel       * y, *u, *v;
    const u8  * tbl_qp_to_st;
    int         i, t, qp, s_l, s_c;
    int w = cuw >> MIN_CU_LOG2;
    int h = cuh >> MIN_CU_LOG2;
    int j;
    u32 *map_scu_tmp;
    s8 (*map_refi_tmp)[REFP_NUM];
    s16(*map_mv_tmp)[REFP_NUM][MV_D];
#if EVC_TILE_SUPPORT
    int  t1, t2, t_copy; // Next row scu number
#endif
    t = (x_pel >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
#if EVC_TILE_SUPPORT
    t_copy = t;
    t1 = ((x_pel - (1 << MIN_CU_LOG2)) >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
    t2 = ((x_pel + (w << MIN_CU_LOG2)) >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
#endif
    map_scu += t;
    map_refi += t;
    map_mv += t;

    s_l = pic->s_l;
    s_c = pic->s_c;
    y = pic->y + x_pel + y_pel * s_l;
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    u = pic->u + t;
    v = pic->v + t;

    map_scu_tmp = map_scu;
    map_refi_tmp = map_refi;
    map_mv_tmp = map_mv;

    /* vertical filtering */
    if(x_pel > 0 && MCU_GET_COD(map_scu[-1])
#if EVC_TILE_SUPPORT
            && (map_tidx[t_copy] == map_tidx[t1])
#endif
            )
    {
        for(i = 0; i < (cuh >> MIN_CU_LOG2); i++)
        {
            tbl_qp_to_st = get_tbl_qp_to_st(map_scu[0], map_scu[-1], \
                                            map_refi[0], map_refi[-1], map_mv[0], map_mv[-1]);
            qp = MCU_GET_QP(map_scu[0]);
#if M50761_CHROMA_NOT_SPLIT
            if (evc_check_luma(tree_cons))
#endif
            {
                deblock_scu_ver(y, qp, s_l, 1, tbl_qp_to_st);
            }
#if M50761_CHROMA_NOT_SPLIT
            if (evc_check_chroma(tree_cons))
            {
#endif
                deblock_scu_ver_chroma(u, p_evc_tbl_qp_chroma_dynamic[0][qp], s_c, 0, tbl_qp_to_st);
                deblock_scu_ver_chroma(v, p_evc_tbl_qp_chroma_dynamic[1][qp], s_c, 0, tbl_qp_to_st);
#if M50761_CHROMA_NOT_SPLIT
            }
#endif

            y += (s_l << MIN_CU_LOG2);
            u += (s_c << (MIN_CU_LOG2 - 1));
            v += (s_c << (MIN_CU_LOG2 - 1));
            map_scu += w_scu;
            map_refi += w_scu;
            map_mv += w_scu;
        }
    }

    map_scu = map_scu_tmp;
    map_refi = map_refi_tmp;
    map_mv = map_mv_tmp;
    if(x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t2])
#endif
        )
    {
        y = pic->y + x_pel + y_pel * s_l;
        u = pic->u + t;
        v = pic->v + t;

        y += cuw;
        u += (cuw >> 1);
        v += (cuw >> 1);

        for(i = 0; i < (cuh >> MIN_CU_LOG2); i++)
        {
            tbl_qp_to_st = get_tbl_qp_to_st(map_scu[w], map_scu[w - 1], \
                                            map_refi[w], map_refi[w - 1], map_mv[w], map_mv[w - 1]);
#if M52166_DBF
            qp = MCU_GET_QP(map_scu[w]);
#else
            qp = MCU_GET_QP(map_scu[w - 1]);
#endif

#if M50761_CHROMA_NOT_SPLIT
            if (evc_check_luma(tree_cons))
            {
#endif
            deblock_scu_ver(y, qp, s_l, 1, tbl_qp_to_st);
#if M50761_CHROMA_NOT_SPLIT
            }
            if (evc_check_chroma(tree_cons))
            {
#endif
                deblock_scu_ver_chroma(u, p_evc_tbl_qp_chroma_dynamic[0][qp], s_c, 0, tbl_qp_to_st);
                deblock_scu_ver_chroma(v, p_evc_tbl_qp_chroma_dynamic[1][qp], s_c, 0, tbl_qp_to_st);
#if M50761_CHROMA_NOT_SPLIT
            }
#endif

            y += (s_l << MIN_CU_LOG2);
            u += (s_c << (MIN_CU_LOG2 - 1));
            v += (s_c << (MIN_CU_LOG2 - 1));
            map_scu += w_scu;
            map_refi += w_scu;
            map_mv += w_scu;
        }
    }

    map_scu = map_scu_tmp;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            MCU_SET_COD(map_scu[j]);
        }
        map_scu += w_scu;
    }
}



#if DBF 
#define DEFAULT_INTRA_TC_OFFSET             2 
#define MAX_QP                              51

#define TCOFFSETDIV2                        0
#define BETAOFFSETDIV2                      0

#if FIX_PARALLEL_DBF
#define CU_THRESH                           16
#else
#define CU_THRESH                           32
#endif

const u8 sm_tc_table[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,24
};

const u8 sm_beta_table[MAX_QP + 1] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
};
#endif

#if (DBF == DBF_AVC) || ADDB_FLAG_FIX
static const u8 compare_mvs(const int mv0[2], const int mv1[2])
{
    // Return 1 if vetors difference less then 1 pixel
    return (EVC_ABS(mv0[0] - mv1[0]) < 4) && (EVC_ABS(mv0[1] - mv1[1]) < 4);
}

static const u8 get_avc_index(const u8 qp, const u8 offset)
{
    return EVC_CLIP3(0, MAX_QP, qp + offset);
}

static const u8 get_avc_bs(u32 mcu0, u32 x0, u32 y0, u32 mcu1, u32 x1, u32 y1, u32 log2_max_cuwh, s8 *refi0, s8 *refi1, s16(*mv0)[MV_D], s16(*mv1)[MV_D], EVC_REFP(*refp)[REFP_NUM])
{
    u8 bs = DBF_AVC_BS_OTHERS;
    u8 isIntraBlock = MCU_GET_IF(mcu0) || MCU_GET_IF(mcu1);
    int log2_cuwh = log2_max_cuwh;
    u8 sameXLCU = (x0 >> log2_cuwh) == (x1 >> log2_cuwh);
    u8 sameYLCU = (y0 >> log2_cuwh) == (y1 >> log2_cuwh);
#if TRACE_DBF
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("Calculate BS. Input params: mcu0 = ");
    EVC_TRACE_INT_HEX(mcu0);
    EVC_TRACE_STR(", x0 = ");
    EVC_TRACE_INT(x0);
    EVC_TRACE_STR(", y0 = ");
    EVC_TRACE_INT(y0);
    EVC_TRACE_STR(", mcu1 = ");
    EVC_TRACE_INT_HEX(mcu1);
    EVC_TRACE_STR(", x1 = ");
    EVC_TRACE_INT(x1);
    EVC_TRACE_STR(", y1 = ");
    EVC_TRACE_INT(y1);
    EVC_TRACE_STR(", log2_max_cuwh = ");
    EVC_TRACE_INT(log2_max_cuwh);
    EVC_TRACE_STR(". isIntraBlock = ");
    EVC_TRACE_INT(isIntraBlock ? 1 : 0);
    EVC_TRACE_STR(". sameXLCU = ");
    EVC_TRACE_INT(sameXLCU ? 1 : 0);
    EVC_TRACE_STR(". sameYLCU = ");
    EVC_TRACE_INT(sameYLCU ? 1 : 0);
    EVC_TRACE_STR(". MCU_GET_CBFL(mcu0) = ");
    EVC_TRACE_INT(MCU_GET_CBFL(mcu0) ? 1 : 0);
    EVC_TRACE_STR(". MCU_GET_CBFL(mcu1) = ");
    EVC_TRACE_INT(MCU_GET_CBFL(mcu1) ? 1 : 0);
    EVC_TRACE_STR(". MCU_GET_IBC(mcu0) = ");
    EVC_TRACE_INT(MCU_GET_IBC(mcu0) ? 1 : 0);
    EVC_TRACE_STR(". MCU_GET_IBC(mcu1) = ");
    EVC_TRACE_INT(MCU_GET_IBC(mcu1) ? 1 : 0);
#endif

    if (isIntraBlock 
        && (!sameXLCU || !sameYLCU)
        )
    {
        // One of the blocks is Intra and blocks lies in the different LCUs
        bs = DBF_AVC_BS_INTRA_STRONG;
    }
    else 
        if (isIntraBlock)
    {
        // One of the blocks is Intra
        bs = DBF_AVC_BS_INTRA;
    }
#if M52166_DBF
    else if (MCU_GET_IBC(mcu0) || MCU_GET_IBC(mcu1))
    {
        bs = DBF_AVC_BS_INTRA;
    }
#endif
    else 
       if (MCU_GET_CBFL(mcu0) == 1 || MCU_GET_CBFL(mcu1) == 1)
    {
        // One of the blocks has coded residuals
        bs = DBF_AVC_BS_CODED;
    }
#if !M52166_DBF
    else if (MCU_GET_IBC(mcu0) || MCU_GET_IBC(mcu1))
    {
         bs = DBF_AVC_BS_INTRA;
    }
#endif
    else
    {
        EVC_PIC *refPics0[2], *refPics1[2];
        refPics0[REFP_0] = (REFI_IS_VALID(refi0[REFP_0])) ? refp[refi0[REFP_0]][REFP_0].pic : NULL;
        refPics0[REFP_1] = (REFI_IS_VALID(refi0[REFP_1])) ? refp[refi0[REFP_1]][REFP_1].pic : NULL;
        refPics1[REFP_0] = (REFI_IS_VALID(refi1[REFP_0])) ? refp[refi1[REFP_0]][REFP_0].pic : NULL;
        refPics1[REFP_1] = (REFI_IS_VALID(refi1[REFP_1])) ? refp[refi1[REFP_1]][REFP_1].pic : NULL;
        int mv0_l0[2] = { mv0[REFP_0][MV_X], mv0[REFP_0][MV_Y] };
        int mv0_l1[2] = { mv0[REFP_1][MV_X], mv0[REFP_1][MV_Y] };
        int mv1_l0[2] = { mv1[REFP_0][MV_X], mv1[REFP_0][MV_Y] };
        int mv1_l1[2] = { mv1[REFP_1][MV_X], mv1[REFP_1][MV_Y] };
#if TRACE_DBF
        EVC_TRACE_STR(". MV info: refi0[REFP_0] = ");
        EVC_TRACE_INT(refi0[REFP_0]);
        EVC_TRACE_STR(", refi0[REFP_1] = ");
        EVC_TRACE_INT(refi0[REFP_1]);
        EVC_TRACE_STR(", refi1[REFP_0] = ");
        EVC_TRACE_INT(refi1[REFP_0]);
        EVC_TRACE_STR(", refi1[REFP_1] = ");
        EVC_TRACE_INT(refi1[REFP_1]);
        EVC_TRACE_STR("; mv0_l0 = {");
        EVC_TRACE_INT(mv0[REFP_0][MV_X]);
        EVC_TRACE_STR(", ");
        EVC_TRACE_INT(mv0[REFP_0][MV_Y]);
        EVC_TRACE_STR("}, mv0_l1 = {");
        EVC_TRACE_INT(mv0[REFP_1][MV_X]);
        EVC_TRACE_STR(", ");
        EVC_TRACE_INT(mv0[REFP_1][MV_Y]);
        EVC_TRACE_STR("}, mv1_l0 = {");
        EVC_TRACE_INT(mv1[REFP_0][MV_X]);
        EVC_TRACE_STR(", ");
        EVC_TRACE_INT(mv1[REFP_0][MV_Y]);
        EVC_TRACE_STR("}, mv1_l1 = {");
        EVC_TRACE_INT(mv1[REFP_1][MV_X]);
        EVC_TRACE_STR(", ");
        EVC_TRACE_INT(mv1[REFP_1][MV_Y]);
        EVC_TRACE_STR("}");
#endif

        if (!REFI_IS_VALID(refi0[REFP_0]))
        {
            mv0_l0[0] = mv0_l0[1] = 0;
        }

        if (!REFI_IS_VALID(refi0[REFP_1]))
        {
            mv0_l1[0] = mv0_l1[1] = 0;
        }

        if (!REFI_IS_VALID(refi1[REFP_0]))
        {
            mv1_l0[0] = mv1_l0[1] = 0;
        }

        if (!REFI_IS_VALID(refi1[REFP_1]))
        {
            mv1_l1[0] = mv1_l1[1] = 0;
        }


        if ((((refPics0[REFP_0] == refPics1[REFP_0]) && (refPics0[REFP_1] == refPics1[REFP_1])))
            || ((refPics0[REFP_0] == refPics1[REFP_1]) && (refPics0[REFP_1] == refPics1[REFP_0])))
        {
            if (refPics0[REFP_0] == refPics0[REFP_1])
            {
                // Are vectors the same? Yes - 0, otherwise - 1.
                bs = (compare_mvs(mv0_l0, mv1_l0) && compare_mvs(mv0_l1, mv1_l1)
                    && compare_mvs(mv0_l0, mv1_l1) && compare_mvs(mv0_l1, mv1_l0)) ? DBF_AVC_BS_OTHERS : DBF_AVC_BS_DIFF_REFS;
            }
            else
            {
                if (((refPics0[REFP_0] == refPics1[REFP_0]) && (refPics0[REFP_1] == refPics1[REFP_1])))
                {
                    bs = (compare_mvs(mv0_l0, mv1_l0) && compare_mvs(mv0_l1, mv1_l1)) ? DBF_AVC_BS_OTHERS : DBF_AVC_BS_DIFF_REFS;
                }
                else if ((refPics0[REFP_0] == refPics1[REFP_1]) && (refPics0[REFP_1] == refPics1[REFP_0]))
                {
                    bs = (compare_mvs(mv0_l0, mv1_l1) && compare_mvs(mv0_l1, mv1_l0)) ? DBF_AVC_BS_OTHERS : DBF_AVC_BS_DIFF_REFS;
                }
            }
        }
        else
        {
            bs = DBF_AVC_BS_DIFF_REFS;
        }
    }
#if TRACE_DBF
    EVC_TRACE_STR(". Answer, bs = ");
    EVC_TRACE_INT(bs);
    EVC_TRACE_STR(")\n");
#endif

    return bs;
}
#endif
#if DBF == DBF_HEVC
static const u8 get_bs(u32 mcu0, u32 mcu1, s8 *refi0, s8 *refi1, s16(*mv0)[MV_D], s16(*mv1)[MV_D])
{
    u8 bs = 0;

    if(MCU_GET_IF(mcu0) || MCU_GET_IF(mcu1))
    {
        bs = 2;
    }
    else if(MCU_GET_CBFL(mcu0) == 1 || MCU_GET_CBFL(mcu1) == 1)
    {
        bs = 1;
    }
    else
    {
        int mv0_l0[2] = {mv0[REFP_0][MV_X], mv0[REFP_0][MV_Y]};
        int mv0_l1[2] = {mv0[REFP_1][MV_X], mv0[REFP_1][MV_Y]};
        int mv1_l0[2] = {mv1[REFP_0][MV_X], mv1[REFP_0][MV_Y]};
        int mv1_l1[2] = {mv1[REFP_1][MV_X], mv1[REFP_1][MV_Y]};

        if(!REFI_IS_VALID(refi0[REFP_0]))
        {
            mv0_l0[0] = mv0_l0[1] = 0;
        }

        if(!REFI_IS_VALID(refi0[REFP_1]))
        {
            mv0_l1[0] = mv0_l1[1] = 0;
        }

        if(!REFI_IS_VALID(refi1[REFP_0]))
        {
            mv1_l0[0] = mv1_l0[1] = 0;
        }

        if(!REFI_IS_VALID(refi1[REFP_1]))
        {
            mv1_l1[0] = mv1_l1[1] = 0;
        }

        if((((refi0[REFP_0] == refi1[REFP_0]) && (refi0[REFP_1] == refi1[REFP_1])))
           || ((refi0[REFP_0] == refi1[REFP_1]) && (refi0[REFP_1] == refi1[REFP_0])))
        {

            if(refi0[REFP_0] == refi0[REFP_1])
            {
                bs = ((EVC_ABS(mv0_l0[MV_X] - mv1_l0[MV_X]) >= 4 ||
                    EVC_ABS(mv0_l0[MV_Y] - mv1_l0[MV_Y]) >= 4 ||
                    EVC_ABS(mv0_l1[MV_X] - mv1_l1[MV_X]) >= 4 ||
                    EVC_ABS(mv0_l1[MV_Y] - mv1_l1[MV_Y]) >= 4)) &&
                    ((EVC_ABS(mv0_l0[MV_X] - mv1_l1[MV_X]) >= 4 ||
                    EVC_ABS(mv0_l0[MV_Y] - mv1_l1[MV_Y]) >= 4 ||
                    EVC_ABS(mv0_l1[MV_X] - mv1_l0[MV_X]) >= 4 ||
                    EVC_ABS(mv0_l1[MV_Y] - mv1_l0[MV_Y]) >= 4))
                    ? 1 : 0;
            }
            else
            {
                if(((refi0[REFP_0] == refi1[REFP_0]) && (refi0[REFP_1] == refi1[REFP_1])))
                {
                    bs = (EVC_ABS(mv0_l0[MV_X] - mv1_l0[MV_X]) >= 4 ||
                          EVC_ABS(mv0_l0[MV_Y] - mv1_l0[MV_Y]) >= 4 ||
                          EVC_ABS(mv0_l1[MV_X] - mv1_l1[MV_X]) >= 4 ||
                          EVC_ABS(mv0_l1[MV_Y] - mv1_l1[MV_Y]) >= 4) ? 1 : 0;
                }
                else if((refi0[REFP_0] == refi1[REFP_1]) && (refi0[REFP_1] == refi1[REFP_0]))
                {
                    bs = (EVC_ABS(mv0_l0[MV_X] - mv1_l1[MV_X]) >= 4 ||
                          EVC_ABS(mv0_l0[MV_Y] - mv1_l1[MV_Y]) >= 4 ||
                          EVC_ABS(mv0_l1[MV_X] - mv1_l0[MV_X]) >= 4 ||
                          EVC_ABS(mv0_l1[MV_Y] - mv1_l0[MV_Y]) >= 4) ? 1 : 0;
                }
            }
        }
        else
        {
            bs = 1;
        }
    }

    return bs;
}

static int calc_dp(pel* src, int offset)
{
    return abs(src[-offset * 3] - 2 * src[-offset * 2] + src[-offset]);
}

static int calc_dq(pel* src, int offset)
{
    return abs(src[0] - 2 * src[offset] + src[offset * 2]);
}


static u8 use_strong_filtering( int offset, int d, int beta, int tc, pel* src
#if DBF_LONGF
    , u8 * type
#endif
)
{
    pel m4 = src[0];
    pel m3 = src[-offset];
    pel m7 = src[offset * 3];
    pel m0 = src[-offset * 4];
#if DBF_LONGF
    pel m11 = src[offset * 7];
    pel mm4 = src[-offset * 8];
#endif
    int d_strong = abs(m0 - m3) + abs(m7 - m4);
    int sw;

    sw = ((d_strong < (beta >> 3)) && (d < (beta >> 2)) && (abs(m3 - m4) < ((tc * 5 + 1) >> 1)));

#if DBF_LONGF
    if(*type != 0 && sw)
    {
        d_strong = (*type == 2) ? abs(m0 - m3) + abs(m11 - m4) : abs(mm4 - m3) + abs(m11 - m4);
        if(d_strong >= (beta >> 3)) *type = 0;
    }
#endif

    return sw;
}

static void pel_filter_chroma(pel* src, int offset, int tc)
{
    int delta;

    pel m4 = src[0];//C
    pel m3 = src[-offset];//B
    pel m5 = src[offset];//D
    pel m2 = src[-offset * 2];//A

    delta = EVC_CLIP3(-tc, tc, ((((m4 - m3) << 2) + m2 - m5 + 4) >> 3));//(C-B)/4+A-D

    src[-offset] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (m3 + delta));
    src[0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (m4 - delta));
}

static void pel_filter_luma(pel* src, int offset, int tc, u8 sw, int thr_cut, u8 filter_second_p, u8 filter_second_q
#if  DBF_LONGF
                     , u8 stronger_ft, int beta
#endif
                     )
{
    int delta;
    int tc2;
    pel m4 = src[0];
    pel m3 = src[-offset];
    pel m5 = src[offset];
    pel m2 = src[-offset * 2];
    pel m6 = src[offset * 2];
    pel m1 = src[-offset * 3];
    pel m7 = src[offset * 3];
    pel m0 = src[-offset * 4];
#if DBF_LONGF
    pel m8 = src[offset * 4];
    pel m9 = src[offset * 5];
    pel m10 = src[offset * 6];
    pel mm1 = src[-offset * 5];
    pel mm2 = src[-offset * 6];
    pel mm3 = src[-offset * 7];
    pel m11 = src[offset * 7];
    pel mm4 = src[-offset * 8];
    pel t0, t1, t2, t3, t4, t5, t6;
#endif

    if(sw)
    {
#if DBF_LONGF
        if(stronger_ft)
        {
            if(stronger_ft == 1) // filtering vertical block boundary
            {
                t6 = (3 * mm4 + 5 * mm3 + 2 * mm2 + mm1 + m0 + m1 + m2 + m3 + m4 + 8) >> 4;
                t5 = (mm4 + 2 * mm3 + 5 * mm2 + 2 * mm1 + m0 + m1 + m2 + m3 + m4 + m5 + 8) >> 4;
                t4 = (mm3 + 3 * mm2 + 3 * mm1 + 3 * m0 + m1 + m2 + m3 + m4 + m5 + m6 + 8) >> 4;
                t3 = (mm3 + 2 * mm2 + 2 * mm1 + 2 * m0 + 2 * m1 + 2 * m2 + m3 + m4 + m5 + m6 + m7 + 8) >> 4;
                t2 = (mm2 + 2 * mm1 + 2 * m0 + 2 * m1 + 2 * m2 + 2 * m3 + 2 * m4 + m5 + m6 + m7 + 8) >> 4;
                t1 = (mm2 + mm1 + 2 * m0 + 2 * m1 + 2 * m2 + 2 * m3 + 2 * m4 + 2 * m5 + m6 + m7 + 8) >> 4;
                t0 = (mm1 + 2 * m0 + 2 * m1 + 2 * m2 + 2 * m3 + 2 * m4 + 2 * m5 + 2 * m6 + m7 + 8) >> 4;
                src[-offset * 7] = EVC_CLIP3(mm3 - 2 * tc, mm3 + 2 * tc, t6);
                src[-offset * 6] = EVC_CLIP3(mm2 - 2 * tc, mm2 + 2 * tc, t5);
                src[-offset * 5] = EVC_CLIP3(mm1 - 2 * tc, mm1 + 2 * tc, t4);
                src[-offset * 4] = EVC_CLIP3(m0 - 2 * tc, m0 + 2 * tc, t3);
            }
            else // filtering horizontal block boundary
            {
                t2 = (3 * m0 + 5 * m1 + 2 * m2 + 2 * m3 + m4 + m5 + m6 + m7 + 8) >> 4;
                t1 = (2 * m0 + 2 * m1 + 4 * m2 + 2 * m3 + 2 * m4 + 2 * m5 + m6 + m7 + 8) >> 4;
                t0 = (2 * m0 + 2 * m1 + 2 * m2 + 2 * m3 + 2 * m4 + 2 * m5 + 2 * m6 + 2 * m7 + 8) >> 4;
            }

            src[-offset * 3] = EVC_CLIP3(m1 - 2 * tc, m1 + 2 * tc, t2);
            src[-offset * 2] = EVC_CLIP3(m2 - 2 * tc, m2 + 2 * tc, t1);
            src[-offset * 1] = EVC_CLIP3(m3 - 2 * tc, m3 + 2 * tc, t0);

            t6 = (3 * m11 + 5 * m10 + 2 * m9 + m8 + m7 + m6 + m5 + m4 + m3 + 8) >> 4;
            t5 = (m11 + 2 * m10 + 5 * m9 + 2 * m8 + m7 + m6 + m5 + m4 + m3 + m2 + 8) >> 4;
            t4 = (m10 + 3 * m9 + 3 * m8 + 3 * m7 + m6 + m5 + m4 + m3 + m2 + m1 + 8) >> 4;
            t3 = (m10 + 2 * m9 + 2 * m8 + 2 * m7 + 2 * m6 + 2 * m5 + m4 + m3 + m2 + m1 + m0 + 8) >> 4;
            t2 = (m9 + 2 * m8 + 2 * m7 + 2 * m6 + 2 * m5 + 2 * m4 + 2 * m3 + m2 + m1 + m0 + 8) >> 4;
            t1 = (m9 + m8 + 2 * m7 + 2 * m6 + 2 * m5 + 2 * m4 + 2 * m3 + 2 * m2 + m1 + m0 + 8) >> 4;
            t0 = (m8 + 2 * m7 + 2 * m6 + 2 * m5 + 2 * m4 + 2 * m3 + 2 * m2 + 2 * m1 + m0 + 8) >> 4;

            src[0] = EVC_CLIP3(m4 - 2 * tc, m4 + 2 * tc, t0);
            src[offset] = EVC_CLIP3(m5 - 2 * tc, m5 + 2 * tc, t1);
            src[offset * 2] = EVC_CLIP3(m6 - 2 * tc, m6 + 2 * tc, t2);
            src[offset * 3] = EVC_CLIP3(m7 - 2 * tc, m7 + 2 * tc, t3);
            src[offset * 4] = EVC_CLIP3(m8 - 2 * tc, m8 + 2 * tc, t4);
            src[offset * 5] = EVC_CLIP3(m9 - 2 * tc, m9 + 2 * tc, t5);
            src[offset * 6] = EVC_CLIP3(m10 - 2 * tc, m10 + 2 * tc, t6);
        }
        else
        {
#endif
            src[-offset] = EVC_CLIP3(m3 - 2 * tc, m3 + 2 * tc, ((m1 + 2 * m2 + 2 * m3 + 2 * m4 + m5 + 4) >> 3));
            src[0] = EVC_CLIP3(m4 - 2 * tc, m4 + 2 * tc, ((m2 + 2 * m3 + 2 * m4 + 2 * m5 + m6 + 4) >> 3));
            src[-offset * 2] = EVC_CLIP3(m2 - 2 * tc, m2 + 2 * tc, ((m1 + m2 + m3 + m4 + 2) >> 2));
            src[offset] = EVC_CLIP3(m5 - 2 * tc, m5 + 2 * tc, ((m3 + m4 + m5 + m6 + 2) >> 2));
            src[-offset * 3] = EVC_CLIP3(m1 - 2 * tc, m1 + 2 * tc, ((2 * m0 + 3 * m1 + m2 + m3 + m4 + 4) >> 3));
            src[offset * 2] = EVC_CLIP3(m6 - 2 * tc, m6 + 2 * tc, ((m3 + m4 + m5 + 3 * m6 + 2 * m7 + 4) >> 3));
#if DBF_LONGF
        }
#endif
    }
    else
    {
        /* Weak filter */
        delta = (9 * (m4 - m3) - 3 * (m5 - m2) + 8) >> 4;

        if(abs(delta) < thr_cut)
        {
            delta = EVC_CLIP3(-tc, tc, delta);

            src[-offset] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (m3 + delta));
            src[0] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (m4 - delta));

            tc2 = tc >> 1;
            if(filter_second_p)
            {
                int delta1 = EVC_CLIP3(-tc2, tc2, ((((m1 + m3 + 1) >> 1) - m2 + delta) >> 1));
                
                src[-offset * 2] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (m2 + delta1));
            }

            if(filter_second_q)
            {
                int delta2 = EVC_CLIP3(-tc2, tc2, ((((m6 + m4 + 1) >> 1) - m5 - delta) >> 1));
                
                src[offset] = EVC_CLIP3(0, (1 << BIT_DEPTH) - 1, (m5 + delta2));
            }
        }
    }
}

static void deblock_scu_hevc_hor(pel *buf, int qp, int stride, int is_luma, u8 bs
#if DBF_LONGF
                                 , u8 stronger_ft
#endif
                                 )
{
    u8 filter_p, Filter_q, sw0, sw3, sw;
    int dp0, dq0, dp3, dq3, d0, d3, d, dp, dq;
    int i, size;
    int beta_offfset_div2 = 0;
    int tc_offset_div2 = TCOFFSETDIV2;
    int idx_tc = EVC_CLIP((qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + (tc_offset_div2 << 1)), 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
    int idx_b = EVC_CLIP((qp + (beta_offfset_div2 << 1)), 0, MAX_QP);
    const int bit_depth_scale = 1 << (BIT_DEPTH - 8);
    int tc = sm_tc_table[idx_tc] * bit_depth_scale;
    int beta = sm_beta_table[idx_b] * bit_depth_scale;
    int side_thr = (beta + (beta >> 1)) >> 3;
    int thr_cut = tc * 10;
#if DBF_LONGF
    stronger_ft <<= 1;
#endif

    size = (is_luma ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1));

    if(bs > 0)
    {
        dp0 = calc_dp(buf, stride);
        dq0 = calc_dq(buf, stride);
        dp3 = calc_dp(buf + 3, stride);
        dq3 = calc_dq(buf + 3, stride);
        d0 = dp0 + dq0;
        d3 = dp3 + dq3;
        dp = dp0 + dp3;
        dq = dq0 + dq3;
        d = d0 + d3;
        if(d < beta)
        {
            filter_p = (dp < side_thr);
            Filter_q = (dq < side_thr);
#if DBF_LONGF
            sw0 = use_strong_filtering(stride, 2 * d0, beta, tc, buf, &stronger_ft);
            sw3 = use_strong_filtering(stride, 2 * d3, beta, tc, buf + 3, &stronger_ft);
#else
            sw0 = use_strong_filtering(stride, 2 * d0, beta, tc, buf);
            sw3 = use_strong_filtering(stride, 2 * d3, beta, tc, buf + 3);
#endif
            sw = sw0 && sw3;
            for(i = 0; i < size; i++)
            {
                pel_filter_luma(buf + i, stride, tc, sw, thr_cut, filter_p, Filter_q
#if DBF_LONGF
                                , stronger_ft, beta
#endif
                                );
           }
      }
    }
}

static void deblock_scu_hevc_ver(pel *buf, int qp, int stride, int is_luma, u8 bs
#if  DBF_LONGF
                                 , u8 stronger_ftr
#endif
                                 )
{
    u8 filter_p, Filter_q, sw0, sw3, sw;
    int dp0, dq0, dp3, dq3, d0, d3, d, dp, dq;
    int i, size;
    int beta_offfset_div2 = 0;
    int tc_offset_div2 = TCOFFSETDIV2;
    int idx_tc = EVC_CLIP((qp + DEFAULT_INTRA_TC_OFFSET * (bs - 1) + (tc_offset_div2 << 1)), 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET);
    int idx_b = EVC_CLIP((qp + (beta_offfset_div2 << 1)), 0, MAX_QP);
    const int bit_depth_scale = 1 << (BIT_DEPTH - 8);
    int tc = sm_tc_table[idx_tc] * bit_depth_scale;
    int beta = sm_beta_table[idx_b] * bit_depth_scale;
    int side_thr = (beta + (beta >> 1)) >> 3;
    int thr_cut = tc * 10;

    size = (is_luma ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1));

    if(bs > 0)
    {
        dp0 = calc_dp(buf, 1);
        dq0 = calc_dq(buf, 1);
        dp3 = calc_dp(buf + 3 * stride, 1);
        dq3 = calc_dq(buf + 3 * stride, 1);
        d0 = dp0 + dq0;
        d3 = dp3 + dq3;
        dp = dp0 + dp3;
        dq = dq0 + dq3;
        d = d0 + d3;
        if(d < beta)
        {
            filter_p = (dp < side_thr);
            Filter_q = (dq < side_thr);
#if DBF_LONGF
            sw0 = use_strong_filtering(1, 2 * d0, beta, tc, buf, &stronger_ftr);
            sw3 = use_strong_filtering(1, 2 * d3, beta, tc, buf + 3 * stride, &stronger_ftr);
#else
            sw0 = use_strong_filtering(1, 2 * d0, beta, tc, buf);
            sw3 = use_strong_filtering(1, 2 * d3, beta, tc, buf + 3 * stride);
#endif
            sw = sw0 && sw3;
            for(i = 0; i < size; i++)
            {
                pel_filter_luma(buf + i * stride, 1, tc, sw, thr_cut, filter_p, Filter_q
#if DBF_LONGF
                                , stronger_ftr, beta
#endif
                                );
            }
        }
    }
}
#endif

#if (DBF == DBF_AVC) || ADDB_FLAG_FIX
static void deblock_avc_get_pq(pel *buf, int offset, pel* p, pel* q, int size)
{
    // p and q has DBF_LENGTH elements
    u8 i;
    for (i = 0; i < size; ++i)
    {
        q[i] = buf[i * offset];
        p[i] = buf[(i+1) * -offset];
    }
}

static void deblock_avc_set_pq(pel *buf, int offset, pel* p, pel* q, int size)
{
    // p and q has DBF_LENGTH elements
    u8 i;
#if TRACE_DBF
    EVC_TRACE_STR(" Set (P, Q): ");
#endif
    for (i = 0; i < size; ++i)
    {
        buf[i * offset] = q[i];
        buf[(i + 1) * -offset] = p[i];
#if TRACE_DBF
        if (i != 0)
        {
            EVC_TRACE_STR(", ");
        }
        EVC_TRACE_STR("(");
        EVC_TRACE_INT(q[i]);
        EVC_TRACE_STR(", ");
        EVC_TRACE_INT(p[i]);
        EVC_TRACE_STR(")");
#endif
    }
}

static const u8 deblock_line_avc_apply(pel *p, pel* q, u8 alpha, u8 beta)
{
    return (EVC_ABS(p[0] - q[0]) < alpha) && (EVC_ABS(p[1] - p[0]) < beta) && (EVC_ABS(q[1] - q[0]) < beta);
}

static void deblock_line_avc_chroma_strong(pel* x, pel* y, pel* x_out)
{
    x_out[0] = (2 * x[1] + x[0] + y[1] + 2) >> 2;
}

static void deblock_line_avc_luma_strong(pel* x, pel* y, pel* x_out)
{
    x_out[0] = (x[2] + 2 * (x[1] + x[0] + y[0]) + y[1] + 4) >> 3;
    x_out[1] = (x[2] + x[1] + x[0] + y[0] + 2) >> 2;
    x_out[2] = (2 * x[3] + 3 * x[2] + x[1] + x[0] + y[0] + 4) >> 3;
}

static void deblock_line_avc_check(u8 alpha, u8 beta, pel *p, pel* q, u8 *ap, u8 *aq){
    *ap = (EVC_ABS(p[0] - p[2]) < beta) ? 1 : 0;
    *aq = (EVC_ABS(q[0] - q[2]) < beta) ? 1 : 0;
}

static pel deblock_line_avc_normal_delta0(u8 c0, pel* p, pel* q)
{
    // This part of code wrote according to AdaptiveDeblocking Filter by P.List, and etc. IEEE transactions on circuits and ... Vol. 13, No. 7, 2003
    // and inconsists with code in JM 19.0
    return EVC_CLIP3(-(pel)c0, (pel)c0, (4 * (q[0] - p[0]) + p[1] - q[1] + 4) >> 3);
}

static pel deblock_line_avc_normal_delta1(u8 c1, pel* x, pel* y)
{
    return EVC_CLIP3(-(pel)c1, (pel)c1, ((((x[2] + x[0] + y[0]) * 3) - 8 * x[1] - y[1])) >> 4);
}


static void deblock_scu_avc_line_luma(pel *buf, int stride, u8 bs, u8 alpha, u8 beta, u8 c1)
{
#if EVC_CONCURENCY    
    pel p[DBF_LENGTH], q[DBF_LENGTH];
    pel p_out[DBF_LENGTH], q_out[DBF_LENGTH];
#else
    static pel p[DBF_LENGTH], q[DBF_LENGTH];
    static pel p_out[DBF_LENGTH], q_out[DBF_LENGTH];
#endif

    deblock_avc_get_pq(buf, stride, p, q, DBF_LENGTH);
    evc_mcpy(p_out, p, DBF_LENGTH * sizeof(p[0]));
    evc_mcpy(q_out, q, DBF_LENGTH * sizeof(q[0]));
#if TRACE_DBF
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("Process luma line (bs = ");
    EVC_TRACE_INT(bs);
    EVC_TRACE_STR(", alpha = ");
    EVC_TRACE_INT(alpha);
    EVC_TRACE_STR(", beta = ");
    EVC_TRACE_INT(beta);
    EVC_TRACE_STR(", c1 = ");
    EVC_TRACE_INT(c1);
    EVC_TRACE_STR("). P = {");
    for (int i = 0; i < DBF_LENGTH; ++i)
    {
        if (i != 0)
        {
            EVC_TRACE_STR(", ");
        }
        EVC_TRACE_INT(p[i]);
    }
    EVC_TRACE_STR("}. Q = {");
    for (int i = 0; i < DBF_LENGTH; ++i)
    {
        if (i != 0)
        {
            EVC_TRACE_STR(", ");
        }
        EVC_TRACE_INT(q[i]);
    }
    EVC_TRACE_STR("}.");
#endif
    if (deblock_line_avc_apply(p, q, alpha, beta))
    {
        assert(BIT_DEPTH == 10 || BIT_DEPTH == 8);
        int tcAdjShift = ( BIT_DEPTH == 10 ) ? 1 : 0;
        u8 ap, aq;
        deblock_line_avc_check(alpha, beta, p, q, &ap, &aq);
#if TRACE_DBF
        EVC_TRACE_STR(" Ap = ");
        EVC_TRACE_INT(ap);
        EVC_TRACE_STR(" Aq = ");
        EVC_TRACE_INT(aq);
#endif
        if (bs == DBF_AVC_BS_INTRA_STRONG)
        {
            if (EVC_ABS(p[0] - q[0]) < ((alpha >> 2) + 2))
            {
                if (ap)
                {
                    deblock_line_avc_luma_strong(p, q, p_out);
                }
                else
                {
                    deblock_line_avc_chroma_strong(p, q, p_out);
                }

                if (aq)
                {
                    deblock_line_avc_luma_strong(q, p, q_out);
                }
                else
                {
                    deblock_line_avc_chroma_strong(q, p, q_out);
                }
            }
            else
            {
                deblock_line_avc_chroma_strong(p, q, p_out);
                deblock_line_avc_chroma_strong(q, p, q_out);
            }
        }
        else
        {
            u8 c0;
            pel delta0, delta1;
            int pel_max = (1 << BIT_DEPTH) - 1;
            c1 = c1 >> tcAdjShift;
            c0 = c1 + (ap << tcAdjShift) + (aq << tcAdjShift);
#if TRACE_DBF
            EVC_TRACE_STR(" c1 = ");
            EVC_TRACE_INT(c1);
            EVC_TRACE_STR(" c0 = ");
            EVC_TRACE_INT(c0);
#endif

            delta0 = deblock_line_avc_normal_delta0(c0, p, q);
#if TRACE_DBF
            EVC_TRACE_STR(" delta0 = ");
            EVC_TRACE_INT(delta0);
#endif
            p_out[0] = EVC_CLIP3(0, pel_max, p[0] + delta0);
            q_out[0] = EVC_CLIP3(0, pel_max, q[0] - delta0);
            if (ap)
            {
                delta1 = deblock_line_avc_normal_delta1(c1, p, q);
                p_out[1] = p[1] + delta1;
#if TRACE_DBF
                EVC_TRACE_STR(" AP_delta1 = ");
                EVC_TRACE_INT(delta1);
#endif
            }
            if (aq)
            {
                delta1 = deblock_line_avc_normal_delta1(c1, q, p);
                q_out[1] = q[1] + delta1;
#if TRACE_DBF
                EVC_TRACE_STR(" AQ_delta1 = ");
                EVC_TRACE_INT(delta1);
#endif
            }
        }
#if DBF_CLIP_FIX
        int pel_max = (1 << BIT_DEPTH) - 1;
        p_out[0] = EVC_CLIP3(0, pel_max, p_out[0]);
        q_out[0] = EVC_CLIP3(0, pel_max, q_out[0]);
        p_out[1] = EVC_CLIP3(0, pel_max, p_out[1]);
        q_out[1] = EVC_CLIP3(0, pel_max, q_out[1]);
        p_out[2] = EVC_CLIP3(0, pel_max, p_out[2]);
        q_out[2] = EVC_CLIP3(0, pel_max, q_out[2]);
        p_out[3] = EVC_CLIP3(0, pel_max, p_out[3]);
        q_out[3] = EVC_CLIP3(0, pel_max, q_out[3]);
#endif
        deblock_avc_set_pq(buf, stride, p_out, q_out, DBF_LENGTH);
    }
#if TRACE_DBF
    else
    {
        EVC_TRACE_STR("Line won't processed");
    }
    EVC_TRACE_STR("\n");
#endif
}

static void deblock_scu_avc_line_chroma(pel *buf, int stride, u8 bs, u8 alpha, u8 beta, u8 c0)
{
#if EVC_CONCURENCY
    pel p[DBF_LENGTH_CHROMA], q[DBF_LENGTH_CHROMA];
    pel p_out[DBF_LENGTH_CHROMA], q_out[DBF_LENGTH_CHROMA];
#else
    static pel p[DBF_LENGTH_CHROMA], q[DBF_LENGTH_CHROMA];
    static pel p_out[DBF_LENGTH_CHROMA], q_out[DBF_LENGTH_CHROMA];
#endif

    deblock_avc_get_pq(buf, stride, p, q, DBF_LENGTH_CHROMA);
    evc_mcpy(p_out, p, DBF_LENGTH_CHROMA * sizeof(p[0]));
    evc_mcpy(q_out, q, DBF_LENGTH_CHROMA * sizeof(q[0]));
#if TRACE_DBF
    EVC_TRACE_COUNTER;
    EVC_TRACE_STR("Process chroma line (bs = ");
    EVC_TRACE_INT(bs);
    EVC_TRACE_STR(", alpha = ");
    EVC_TRACE_INT(alpha);
    EVC_TRACE_STR(", beta = ");
    EVC_TRACE_INT(beta);
    EVC_TRACE_STR(", c0 = ");
    EVC_TRACE_INT(c0);
    EVC_TRACE_STR("). P = {");
    for (int i = 0; i < DBF_LENGTH_CHROMA; ++i)
    {
        if (i != 0)
        {
            EVC_TRACE_STR(", ");
        }
        EVC_TRACE_INT(p[i]);
    }
    EVC_TRACE_STR("}. Q = {");
    for (int i = 0; i < DBF_LENGTH_CHROMA; ++i)
    {
        if (i != 0)
        {
            EVC_TRACE_STR(", ");
        }
        EVC_TRACE_INT(q[i]);
    }
    EVC_TRACE_STR("}.");
#endif
    if (deblock_line_avc_apply(p, q, alpha, beta))
    {
        if (bs == DBF_AVC_BS_INTRA_STRONG)
        {
            deblock_line_avc_chroma_strong(p, q, p_out);
            deblock_line_avc_chroma_strong(q, p, q_out);
        }
        else
        {
            pel delta0;
            int pel_max = (1 << BIT_DEPTH) - 1;
            delta0 = deblock_line_avc_normal_delta0(c0, p, q);
            p_out[0] = EVC_CLIP3(0, pel_max, p[0] + delta0);
            q_out[0] = EVC_CLIP3(0, pel_max, q[0] - delta0);
#if TRACE_DBF
            EVC_TRACE_STR(" delta0 = ");
            EVC_TRACE_INT(delta0);
#endif
        }
#if DBF_CLIP_FIX
        int pel_max = (1 << BIT_DEPTH) - 1;
        p_out[0] = EVC_CLIP3(0, pel_max, p_out[0]);
        q_out[0] = EVC_CLIP3(0, pel_max, q_out[0]);
        p_out[1] = EVC_CLIP3(0, pel_max, p_out[1]);
        q_out[1] = EVC_CLIP3(0, pel_max, q_out[1]);
#endif
        deblock_avc_set_pq(buf, stride, p_out, q_out, DBF_LENGTH_CHROMA);
    }
#if TRACE_DBF
    else
    {
        EVC_TRACE_STR("Line won't processed");
    }
    EVC_TRACE_STR("\n");
#endif
}

static void deblock_scu_avc_ver_luma(pel *buf,int stride, u8 bs, u8 alpha, u8 beta, u8 c1)
{
    u8 i;
    pel *cur_buf = buf;
    for (i = 0; i < MIN_CU_SIZE; ++i, cur_buf += stride)
    {
        deblock_scu_avc_line_luma(cur_buf, 1, bs, alpha, beta, c1);
    }
}

static void deblock_scu_avc_hor_luma(pel *buf, int stride, u8 bs, u8 alpha, u8 beta, u8 c1)
{
    u8 i;
    pel *cur_buf = buf;
    for (i = 0; i < MIN_CU_SIZE; ++i, ++cur_buf)
    {
        deblock_scu_avc_line_luma(cur_buf, stride, bs, alpha, beta, c1);
    }
}

static void deblock_scu_avc_ver_chroma(pel *buf, int stride, u8 bs, u8 alpha, u8 beta, u8 c0)
{
    u8 i;
    pel *cur_buf = buf;
    for (i = 0; i < (MIN_CU_SIZE >> 1); ++i, cur_buf += stride)
    {
        deblock_scu_avc_line_chroma(cur_buf, 1, bs, alpha, beta, c0);
    }
}

static void deblock_scu_avc_hor_chroma(pel *buf, int stride, u8 bs, u8 alpha, u8 beta, u8 c0)
{
    u8 i;
    pel *cur_buf = buf;
    for (i = 0; i < (MIN_CU_SIZE >> 1); ++i, ++cur_buf)
    {
        deblock_scu_avc_line_chroma(cur_buf, stride, bs, alpha, beta, c0);
    }
}

static u32* deblock_set_coded_block(u32* map_scu, int w, int h, int w_scu)
{
    int i, j;
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            MCU_SET_COD(map_scu[j]);
        }
        map_scu += w_scu;
    }
    return map_scu;
}

static void deblock_avc_cu_hor(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, int log2_max_cuwh, EVC_REFP(*refp)[REFP_NUM], int ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons 
#endif
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
)
{
    pel       * y, *u, *v;
    int         i, t, qp, s_l, s_c;
    int w = cuw >> MIN_CU_LOG2;
    int h = cuh >> MIN_CU_LOG2;

    u8 indexA, indexB;
    u8 alpha, beta;
    u8 c0, c1;
    u32 *map_scu_tmp;
    const int bitdepth_scale = (BIT_DEPTH - 8);

#if DBF_8_8_GRID
    int align_8_8_grid = 0;
    if (y_pel % 8 == 0)
    {
        align_8_8_grid = 1;
    }
#endif
#if EVC_TILE_SUPPORT
    int  t1, t_copy; // Next row scu number
#endif
    t = (x_pel >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
#if EVC_TILE_SUPPORT
    t_copy = t;
#if DBF_8_8_GRID && 0 //need to check
    if (align_8_8_grid)
    {
        t1 = (x_pel >> MIN_CU_LOG2) + ((y_pel - (1 << 3)) >> MIN_CU_LOG2) * w_scu;
    }
    else
#endif
    {
    t1 = (x_pel >> MIN_CU_LOG2) + ((y_pel - (1 << MIN_CU_LOG2)) >> MIN_CU_LOG2) * w_scu;
    }
#endif
   
    map_scu += t;
    map_refi += t;
    map_mv += t;
    map_scu_tmp = map_scu;
    s_l = pic->s_l;
    s_c = pic->s_c;
    y = pic->y + x_pel + y_pel * s_l;
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    u = pic->u + t;
    v = pic->v + t;

#if DBF_8_8_GRID
    if (align_8_8_grid  && y_pel > 0
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif
        )
#else
#if DBF_DISABLE_SCU
    if (cuh >= 8 && y_pel > 0
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif
        )
#else
    if (y_pel > 0
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif
        )
#endif
#endif
    {

        for (i = 0; i < (cuw >> MIN_CU_LOG2); ++i)
        {
#if TRACE_DBF
            EVC_TRACE_COUNTER;
            EVC_TRACE_STR("Start filtering hor boundary of SCU (");
            EVC_TRACE_INT(x_pel);
            EVC_TRACE_STR(", ");
            EVC_TRACE_INT(y_pel);
            EVC_TRACE_STR(") ats_inter_mode = ");
            EVC_TRACE_INT(ats_inter_mode);
#if M50761_CHROMA_NOT_SPLIT
            EVC_TRACE_STR(" tree_type = ");
            EVC_TRACE_INT(tree_cons.tree_type);
            EVC_TRACE_STR(" mode_cons = ");
            EVC_TRACE_INT(tree_cons.mode_cons);
#endif
            EVC_TRACE_STR("\n");
#endif
            {
                t = (i << MIN_CU_LOG2);
                int cur_x_pel = x_pel + t;
                u8 bs_cur = get_avc_bs(map_scu[i], cur_x_pel, y_pel, map_scu[i - w_scu], cur_x_pel, y_pel - 1, log2_max_cuwh, map_refi[i], map_refi[i - w_scu], map_mv[i], map_mv[i - w_scu]
                    , refp
                );

                if ( (bs_cur < DBF_AVC_BS_INTRA_STRONG) && (ats_inter_mode > 0) )
                {
                    bs_cur = DBF_AVC_BS_CODED;
                }

                qp = (MCU_GET_QP(map_scu[i]) + MCU_GET_QP(map_scu[i - w_scu]) + 1) >> 1;

                indexA = get_avc_index(qp, pic->pic_deblock_alpha_offset);            //! \todo Add offset for IndexA
                indexB = get_avc_index(qp, pic->pic_deblock_beta_offset);            //! \todo Add offset for IndexB

                alpha = ALPHA_TABLE[indexA] << bitdepth_scale;
                beta = BETA_TABLE[indexB] << bitdepth_scale;
                c1 = CLIP_TAB[indexA][bs_cur] << bitdepth_scale;

#if M50761_CHROMA_NOT_SPLIT
                if (evc_check_luma(tree_cons))
                {
#endif
                deblock_scu_avc_hor_luma(y + t, s_l, bs_cur, alpha, beta, c1);
#if M50761_CHROMA_NOT_SPLIT
                }
                if (evc_check_chroma(tree_cons))
                {
#endif
                    t >>= 1;
                    indexA = get_avc_index(p_evc_tbl_qp_chroma_dynamic[0][qp], pic->pic_deblock_alpha_offset);
                    indexB = get_avc_index(p_evc_tbl_qp_chroma_dynamic[0][qp], pic->pic_deblock_beta_offset);
                    alpha = ALPHA_TABLE[indexA] << bitdepth_scale;
                    beta = BETA_TABLE[indexB] << bitdepth_scale;
                    c1 = CLIP_TAB[indexA][bs_cur] << bitdepth_scale;
                    c0 = c1 + 1;
                    deblock_scu_avc_hor_chroma(u + t, s_c, bs_cur, alpha, beta, c0);

                    indexA = get_avc_index(p_evc_tbl_qp_chroma_dynamic[1][qp], pic->pic_deblock_alpha_offset);
                    indexB = get_avc_index(p_evc_tbl_qp_chroma_dynamic[1][qp], pic->pic_deblock_beta_offset);
                    alpha = ALPHA_TABLE[indexA] << bitdepth_scale;
                    beta = BETA_TABLE[indexB] << bitdepth_scale;
                    c1 = CLIP_TAB[indexA][bs_cur] << bitdepth_scale;
                    c0 = c1 + 1;
                    deblock_scu_avc_hor_chroma(v + t, s_c, bs_cur, alpha, beta, c0);
#if M50761_CHROMA_NOT_SPLIT
                }
#endif
            }
        }
    }

    map_scu = deblock_set_coded_block(map_scu_tmp, w, h, w_scu);
}

static void deblock_avc_cu_ver_yuv(EVC_PIC *pic, int x_pel, int y_pel, int log2_max_cuwh, pel *y, pel* u, pel *v, int s_l, int s_c, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, EVC_REFP(*refp)[REFP_NUM], int ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    int i, t, qp;
    int h = cuh >> MIN_CU_LOG2;
    u8 indexA, indexB;
    u8 alpha, beta;
    u8 c0, c1;
    const int bitdepth_scale = (BIT_DEPTH - 8);
    for (i = 0; i < h; i++)
    {
#if TRACE_DBF
        EVC_TRACE_COUNTER;
        EVC_TRACE_STR("Start filtering ver boundary of SCU (");
        EVC_TRACE_INT(x_pel);
        EVC_TRACE_STR(", ");
        EVC_TRACE_INT(y_pel);
        EVC_TRACE_STR(") ats_inter_mode = ");
        EVC_TRACE_INT(ats_inter_mode);
#if M50761_CHROMA_NOT_SPLIT
        EVC_TRACE_STR(" tree_type = ");
        EVC_TRACE_INT(tree_cons.tree_type);
        EVC_TRACE_STR(" mode_cons = ");
        EVC_TRACE_INT(tree_cons.mode_cons);
#endif
        EVC_TRACE_STR("\n");
#endif

        {
            int cur_y_pel = y_pel + (i << MIN_CU_LOG2);
            u8 bs_cur = get_avc_bs(map_scu[0], x_pel-1, cur_y_pel, map_scu[-1], x_pel, cur_y_pel, log2_max_cuwh, map_refi[0], map_refi[-1], map_mv[0], map_mv[-1]
                , refp
            );

            if ((bs_cur < DBF_AVC_BS_INTRA_STRONG) && (ats_inter_mode > 0))
            {
                bs_cur = DBF_AVC_BS_CODED;
            }

            qp = (MCU_GET_QP(map_scu[0]) + MCU_GET_QP(map_scu[-1]) + 1) >> 1;

            t = (i << MIN_CU_LOG2);

#if FIX_PARALLEL_DBF
            //neb_w = 1 << MCU_GET_LOGW(map_cu[-1]);
#if DBF_LONGF
            if (stronger_ft)
            {
                stronger_ft = ((neb_w >= 16) ? 1 : 0); /* only if neighbor width is also greater than 16, only then use long tap filtering */
            }
#endif
#endif
#if M50761_CHROMA_NOT_SPLIT
            if (evc_check_luma(tree_cons))
            {
#endif

            indexA = get_avc_index(qp, pic->pic_deblock_alpha_offset);            //! \todo Add offset for IndexA
            indexB = get_avc_index(qp, pic->pic_deblock_beta_offset);            //! \todo Add offset for IndexB

            alpha = ALPHA_TABLE[indexA] << bitdepth_scale;
            beta = BETA_TABLE[indexB] << bitdepth_scale;
            c1 = CLIP_TAB[indexA][bs_cur] << bitdepth_scale;

            deblock_scu_avc_ver_luma(y, s_l, bs_cur, alpha, beta, c1
#if  DBF_LONGF
                , stronger_ft
#endif
            );
#if M50761_CHROMA_NOT_SPLIT
            }
            if (evc_check_chroma(tree_cons))
            {
#endif
                indexA = get_avc_index(p_evc_tbl_qp_chroma_dynamic[0][qp], pic->pic_deblock_alpha_offset);
                indexB = get_avc_index(p_evc_tbl_qp_chroma_dynamic[0][qp], pic->pic_deblock_beta_offset);

                alpha = ALPHA_TABLE[indexA] << bitdepth_scale;
                beta = BETA_TABLE[indexB] << bitdepth_scale;
                c1 = CLIP_TAB[indexA][bs_cur] << bitdepth_scale;
                c0 = c1 + 1;
                deblock_scu_avc_ver_chroma(u, s_c, bs_cur, alpha, beta, c0);

                indexA = get_avc_index(p_evc_tbl_qp_chroma_dynamic[1][qp], pic->pic_deblock_alpha_offset);
                indexB = get_avc_index(p_evc_tbl_qp_chroma_dynamic[1][qp], pic->pic_deblock_beta_offset);

                alpha = ALPHA_TABLE[indexA] << bitdepth_scale;
                beta = BETA_TABLE[indexB] << bitdepth_scale;
                c1 = CLIP_TAB[indexA][bs_cur] << bitdepth_scale;
                c0 = c1 + 1;
                deblock_scu_avc_ver_chroma(v, s_c, bs_cur, alpha, beta, c0);
#if M50761_CHROMA_NOT_SPLIT
            }
#endif
            y += (s_l << MIN_CU_LOG2);
            u += (s_c << (MIN_CU_LOG2 - 1));
            v += (s_c << (MIN_CU_LOG2 - 1));
            map_scu += w_scu;
            map_refi += w_scu;
            map_mv += w_scu;
#if FIX_PARALLEL_DBF
            //map_cu += w_scu;
#endif
        }
    }

}

static void deblock_avc_cu_ver(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, int log2_max_cuwh
#if FIX_PARALLEL_DBF
    , u32  *map_cu
#endif
    , EVC_REFP(*refp)[REFP_NUM]
    , int ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
)
{
    pel       * y, *u, *v;
    int         t, s_l, s_c;
    int w = cuw >> MIN_CU_LOG2;
    int h = cuh >> MIN_CU_LOG2;
    u32 *map_scu_tmp;
    s8(*map_refi_tmp)[REFP_NUM];
    s16(*map_mv_tmp)[REFP_NUM][MV_D];
#if FIX_PARALLEL_DBF
    //int neb_w;
    u32  *map_cu_tmp;
#endif
#if DBF_LONGF
    u8 stronger_ft = ((cuw >= CU_THRESH) ? 1 : 0);
#endif
#if DBF_8_8_GRID
    int align_8_8_grid = 0;

    if (x_pel % 8 == 0)
    {
        align_8_8_grid = 1;
    }
#endif
#if EVC_TILE_SUPPORT
    int  t1, t2, t_copy; // Next row scu number
#endif
    t = (x_pel >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
#if EVC_TILE_SUPPORT
    t_copy = t;
#if DBF_8_8_GRID && 0 //need to check
    if(align_8_8_grid)
    {
        t1 = ((x_pel - (1 << 3)) >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
        t2 = ((x_pel + (w << 3)) >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
    }
    else
#endif
    {
    t1 = ((x_pel - (1 << MIN_CU_LOG2)) >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
    t2 = ((x_pel + (w << MIN_CU_LOG2)) >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
    }
#endif
          
    map_scu += t;
    map_refi += t;
    map_mv += t;

#if FIX_PARALLEL_DBF
    map_cu += t;
#endif

    s_l = pic->s_l;
    s_c = pic->s_c;
    y = pic->y + x_pel + y_pel * s_l;
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    u = pic->u + t;
    v = pic->v + t;

    map_scu_tmp = map_scu;
    map_refi_tmp = map_refi;
    map_mv_tmp = map_mv;

#if FIX_PARALLEL_DBF
    map_cu_tmp = map_cu;
#endif

    /* vertical filtering */
#if DBF_8_8_GRID
#if M52166_DBF
    if ((x_pel + cuw) % 8 == 0)
    {
        align_8_8_grid = 1;
    }
    else
    {
        align_8_8_grid = 0;
    }
#endif
    if (align_8_8_grid && x_pel > 0 && MCU_GET_COD(map_scu[-1])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif
        )
#else
#if DBF_DISABLE_SCU
    if (cuw >= 8 && x_pel > 0 && MCU_GET_COD(map_scu[-1])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif
        )
#else
    if (x_pel > 0 && MCU_GET_COD(map_scu[-1])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t1])
#endif
        )
#endif
#endif
    {
        deblock_avc_cu_ver_yuv(pic, x_pel, y_pel, log2_max_cuwh, y, u, v, s_l, s_c, cuh, map_scu, map_refi, map_mv, w_scu, refp, ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }

    map_scu = map_scu_tmp;
    map_refi = map_refi_tmp;
    map_mv = map_mv_tmp;

#if FIX_PARALLEL_DBF
    map_cu = map_cu_tmp;
#endif

#if DBF_8_8_GRID
    if (align_8_8_grid && x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t2])
#endif
        )
#else
#if DBF_DISABLE_SCU
    if (cuw >= 8 && x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t2])
#endif
        )
#else
    if (x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w])
#if EVC_TILE_SUPPORT
        && (map_tidx[t_copy] == map_tidx[t2])
#endif
        )
#endif
#endif
    {
        y = pic->y + x_pel + y_pel * s_l;
        u = pic->u + t;
        v = pic->v + t;

        y += cuw;
        u += (cuw >> 1);
        v += (cuw >> 1);
        map_scu += w;
        map_refi += w;
        map_mv += w;
        deblock_avc_cu_ver_yuv(pic, x_pel + cuw, y_pel, log2_max_cuwh, y, u, v, s_l, s_c, cuh, map_scu, map_refi, map_mv, w_scu, refp, ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }

    map_scu = deblock_set_coded_block(map_scu_tmp, w, h, w_scu);
}
#endif 
#if DBF == DBF_HEVC    
static void deblock_hevc_cu_hor(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu)
{
    pel       * y, *u, *v;
    u8  bs;
    int         i, t, qp, s_l, s_c;
    int w = cuw >> MIN_CU_LOG2;
    int h = cuh >> MIN_CU_LOG2;
    u32 *map_scu_tmp;
    int j;
    const u8  * tbl_qp_to_st;

#if DBF_8_8_GRID
    int align_8_8_grid = 0;
    if(y_pel % 8 == 0)
    {
        align_8_8_grid = 1;
    }
#endif

#if  DBF_LONGF
    u8 stronger_ft = ((cuh >= CU_THRESH) ? 1 : 0);
#endif
    t = (x_pel >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
    map_scu += t;
    map_refi += t;
    map_mv += t;
    map_scu_tmp = map_scu;
    s_l = pic->s_l;
    s_c = pic->s_c;
    y = pic->y + x_pel + y_pel * s_l;
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    u = pic->u + t;
    v = pic->v + t;

    /* horizontal filtering */
#if DBF_8_8_GRID
    if (align_8_8_grid  && y_pel > 0)
#else
#if DBF_DISABLE_SCU
    if(cuh >= 8 && y_pel > 0)
#else
    if(y_pel > 0)
#endif
#endif
    {
        for(i = 0; i < (cuw >> MIN_CU_LOG2); i++)
        {
            bs = get_bs(map_scu[i], map_scu[i - w_scu], map_refi[i], map_refi[i - w_scu], map_mv[i], map_mv[i - w_scu]);
            tbl_qp_to_st = get_tbl_qp_to_st(map_scu[i], map_scu[i - w_scu], map_refi[i], map_refi[i - w_scu], map_mv[i], map_mv[i - w_scu]);

            qp = MCU_GET_QP(map_scu[i]);
            t = (i << MIN_CU_LOG2);

            deblock_scu_hevc_hor(y + t, qp, s_l, 1, bs
#if DBF_LONGF
                                 , stronger_ft
#endif
                                 );
            deblock_scu_hor_chroma(u + (t >> 1), evc_tbl_qp_chroma_ajudst[qp], s_c, 0, tbl_qp_to_st);
            deblock_scu_hor_chroma(v + (t >> 1), evc_tbl_qp_chroma_ajudst[qp], s_c, 0, tbl_qp_to_st);
        }
    }

    map_scu = map_scu_tmp;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            MCU_SET_COD(map_scu[j]);
        }
        map_scu += w_scu;
    }
}

static void deblock_hevc_cu_ver(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu
#if FIX_PARALLEL_DBF
                              , u32  *map_cu
#endif
                              )
{
    pel       * y, *u, *v;
    u8  bs;
    const u8  * tbl_qp_to_st;
    int         i, t, qp, s_l, s_c;
    int w = cuw >> MIN_CU_LOG2;
    int h = cuh >> MIN_CU_LOG2;
    int j;
    u32 *map_scu_tmp;
    s8 (*map_refi_tmp)[REFP_NUM];
    s16(*map_mv_tmp)[REFP_NUM][MV_D];
#if FIX_PARALLEL_DBF
    int neb_w;
    u32  *map_cu_tmp;
#endif
#if DBF_LONGF
    u8 stronger_ft = ((cuw >= CU_THRESH) ? 1:0);
#endif
#if DBF_8_8_GRID
    int align_8_8_grid = 0;

    if (x_pel % 8 == 0)
    {
      align_8_8_grid = 1;
    }
#endif

    t = (x_pel >> MIN_CU_LOG2) + (y_pel >> MIN_CU_LOG2) * w_scu;
    map_scu += t;
    map_refi += t;
    map_mv += t;

#if FIX_PARALLEL_DBF
    map_cu += t;
#endif

    s_l = pic->s_l;
    s_c = pic->s_c;
    y = pic->y + x_pel + y_pel * s_l;
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    u = pic->u + t;
    v = pic->v + t;

    map_scu_tmp = map_scu;
    map_refi_tmp = map_refi;
    map_mv_tmp = map_mv;

#if FIX_PARALLEL_DBF
    map_cu_tmp = map_cu;
#endif

    /* vertical filtering */
#if DBF_8_8_GRID
    if(align_8_8_grid && x_pel > 0 && MCU_GET_COD(map_scu[-1]))
#else
#if DBF_DISABLE_SCU
    if(cuw >= 8 && x_pel > 0 && MCU_GET_COD(map_scu[-1]))
#else
    if(x_pel > 0 && MCU_GET_COD(map_scu[-1]))
#endif
#endif
    {
        for(i = 0; i < (cuh >> MIN_CU_LOG2); i++)
        {
            tbl_qp_to_st = get_tbl_qp_to_st(map_scu[0], map_scu[-1], \
                                            map_refi[0], map_refi[-1], map_mv[0], map_mv[-1]);
            bs = get_bs(map_scu[0], map_scu[-1], map_refi[0], map_refi[-1], map_mv[0], map_mv[-1]);
            qp = MCU_GET_QP(map_scu[0]);

#if FIX_PARALLEL_DBF
            neb_w = 1 << MCU_GET_LOGW(map_cu[-1]);
#if DBF_LONGF
            if(stronger_ft)
            {
                stronger_ft = ((neb_w >= 16) ? 1 : 0); /* only if neighbor width is also greater than 16, only then use long tap filtering */
            }
#endif
#endif

            deblock_scu_hevc_ver(y, qp, s_l, 1, bs
#if  DBF_LONGF
                                 , stronger_ft
#endif
                                 );
            deblock_scu_ver_chroma(u, evc_tbl_qp_chroma_ajudst[qp], s_c, 0, tbl_qp_to_st);
            deblock_scu_ver_chroma(v, evc_tbl_qp_chroma_ajudst[qp], s_c, 0, tbl_qp_to_st);

            y += (s_l << MIN_CU_LOG2);
            u += (s_c << (MIN_CU_LOG2 - 1));
            v += (s_c << (MIN_CU_LOG2 - 1));
            map_scu += w_scu;
            map_refi += w_scu;
            map_mv += w_scu;
#if FIX_PARALLEL_DBF
            map_cu += w_scu;
#endif
        }
    }

    map_scu = map_scu_tmp;
    map_refi = map_refi_tmp;
    map_mv = map_mv_tmp;

#if FIX_PARALLEL_DBF
    map_cu = map_cu_tmp;
#endif

#if DBF_8_8_GRID
#if M52166_DBF
    if ((x_pel + cuw) % 8 == 0)
    {
        align_8_8_grid = 1;
    }
    else
    {
        align_8_8_grid = 0;
    }
#endif
    if(align_8_8_grid && x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w]))
#else
#if DBF_DISABLE_SCU
    if(cuw >= 8 && x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w]))
#else
    if(x_pel + cuw < pic->w_l && MCU_GET_COD(map_scu[w]))
#endif
#endif
    {
        y = pic->y + x_pel + y_pel * s_l;
        u = pic->u + t;
        v = pic->v + t;

        y += cuw;
        u += (cuw >> 1);
        v += (cuw >> 1);

        for(i = 0; i < (cuh >> MIN_CU_LOG2); i++)
        {
            bs = get_bs(map_scu[w], map_scu[w - 1], map_refi[w], map_refi[w - 1], map_mv[w], map_mv[w - 1]);
            tbl_qp_to_st = get_tbl_qp_to_st(map_scu[w], map_scu[w - 1], \
                                            map_refi[w], map_refi[w - 1], map_mv[w], map_mv[w - 1]);
#if M52166_DBF
            qp = MCU_GET_QP(map_scu[w]);
#else
            qp = MCU_GET_QP(map_scu[w - 1]);
#endif

#if FIX_PARALLEL_DBF
#if M52166_DBF
            neb_w = 1 << MCU_GET_LOGW(map_cu[w]);
#else
            neb_w = 1 << MCU_GET_LOGW(map_cu[w - 1]);
#endif
#if DBF_LONGF
            if(stronger_ft)
            {
                stronger_ft = ((neb_w >= 16) ? 1 : 0); /* only if neighbor width is also greater than 16, only then use long tap filtering */
            }
#endif
#endif

            deblock_scu_hevc_ver(y, qp, s_l, 1, bs
#if DBF_LONGF
                                 , stronger_ft
#endif
                                 );
            deblock_scu_ver_chroma(u, evc_tbl_qp_chroma_ajudst[qp], s_c, 0, tbl_qp_to_st);
            deblock_scu_ver_chroma(v, evc_tbl_qp_chroma_ajudst[qp], s_c, 0, tbl_qp_to_st);

            y += (s_l << MIN_CU_LOG2);
            u += (s_c << (MIN_CU_LOG2 - 1));
            v += (s_c << (MIN_CU_LOG2 - 1));
            map_scu += w_scu;
            map_refi += w_scu;
            map_mv += w_scu;
#if FIX_PARALLEL_DBF
            map_cu += w_scu;
#endif
        }
    }

    map_scu = map_scu_tmp;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
            MCU_SET_COD(map_scu[j]);
        }
        map_scu += w_scu;
    }
}


#endif

void evc_deblock_cu_hor(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, int log2_max_cuwh, EVC_REFP(*refp)[REFP_NUM], int ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
#if ADDB_FLAG_FIX
    , int tool_addb
#endif
)
{
#if ADDB_FLAG_FIX
    if (tool_addb)
    {
        deblock_avc_cu_hor(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu, log2_max_cuwh, refp
            , ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
#if EVC_TILE_SUPPORT
            , map_tidx
#endif
        );
}
    else
    {
        deblock_h263_cu_hor(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
#if EVC_TILE_SUPPORT
            , map_tidx
#endif
        );
    }
#else
#if DBF == DBF_HEVC
    deblock_hevc_cu_hor(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu
#if M50761_CHROMA_NOT_SPLIT
        , tree_cons
#endif
    );
#elif DBF == DBF_AVC
    deblock_avc_cu_hor(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu, log2_max_cuwh, refp 
        , ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
        , tree_cons
#endif
#if EVC_TILE_SUPPORT
        , map_tidx
#endif
    );
#elif DBF == DBF_H263
    deblock_h263_cu_hor(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu
#if M50761_CHROMA_NOT_SPLIT
        , tree_cons
#endif
#if EVC_TILE_SUPPORT
        , map_tidx
#endif
    );
#endif
#endif
}

void evc_deblock_cu_ver(EVC_PIC *pic, int x_pel, int y_pel, int cuw, int cuh, u32 *map_scu, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, int log2_max_cuwh
#if FIX_PARALLEL_DBF
    , u32  *map_cu
#endif
    , EVC_REFP(*refp)[REFP_NUM]
    , int ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
#if ADDB_FLAG_FIX
    , int tool_addb
#endif
)
{
#if ADDB_FLAG_FIX
    if (tool_addb)
    {
        deblock_avc_cu_ver(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu, log2_max_cuwh
#if FIX_PARALLEL_DBF
            , map_cu
#endif
            , refp
            , ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
#if EVC_TILE_SUPPORT
            , map_tidx
#endif
        );
}
    else
    {
        deblock_h263_cu_ver(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu
#if FIX_PARALLEL_DBF
            , map_cu
#endif
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
#if EVC_TILE_SUPPORT
            , map_tidx
#endif
        );
    }
#else
#if DBF == DBF_HEVC
    deblock_hevc_cu_ver(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu
#if FIX_PARALLEL_DBF
        , map_cu
#endif
#if M50761_CHROMA_NOT_SPLIT
        , tree_cons
#endif
    );
#elif DBF == DBF_AVC
    deblock_avc_cu_ver(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu, log2_max_cuwh
#if FIX_PARALLEL_DBF
        , map_cu
#endif
        , refp
        , ats_inter_mode
#if M50761_CHROMA_NOT_SPLIT
        , tree_cons
#endif
#if EVC_TILE_SUPPORT
        , map_tidx
#endif
    );
#elif DBF == DBF_H263
    deblock_h263_cu_ver(pic, x_pel, y_pel, cuw, cuh, map_scu, map_refi, map_mv, w_scu
#if FIX_PARALLEL_DBF
        , map_cu
#endif
#if M50761_CHROMA_NOT_SPLIT
        , tree_cons
#endif
#if EVC_TILE_SUPPORT
        , map_tidx
#endif
    );
#endif
#endif
}

