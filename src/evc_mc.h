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

#ifndef _EVC_MC_H_
#define _EVC_MC_H_

#ifdef __cplusplus

extern "C"
{
#endif

typedef void (*EVC_MC_L) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h);
typedef void (*EVC_MC_C) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h);

extern EVC_MC_L evc_tbl_mc_l[2][2];
extern EVC_MC_C evc_tbl_mc_c[2][2];

#if DMVR_PADDING
typedef void(*EVC_DMVR_MC_L) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h);
typedef void(*EVC_DMVR_MC_C) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h);

extern EVC_DMVR_MC_L evc_tbl_dmvr_mc_l[2][2];
extern EVC_DMVR_MC_C evc_tbl_dmvr_mc_c[2][2];
#endif

extern EVC_MC_C evc_tbl_bl_mc_l[2][2];

#if MC_PRECISION_ADD
#define evc_mc_l(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_mc_l[((ori_mv_x) | ((ori_mv_x)>>1) | ((ori_mv_x)>>2) | ((ori_mv_x)>>3)) & 0x1])\
        [((ori_mv_y) | ((ori_mv_y)>>1) | ((ori_mv_y)>>2) | ((ori_mv_y)>>3)) & 0x1]\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)

#define evc_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_mc_c[((ori_mv_x) | ((ori_mv_x)>>1) | ((ori_mv_x)>>2)| ((ori_mv_x)>>3) | ((ori_mv_x)>>4)) & 0x1]\
        [((ori_mv_y) | ((ori_mv_y)>>1) | ((ori_mv_y)>>2) | ((ori_mv_y)>>3) | ((ori_mv_y)>>4)) & 0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)

#if DMVR_PADDING
#define evc_dmvr_mc_l(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_dmvr_mc_l[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2) | ((gmv_x)>>3)) & 0x1])\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2) | ((gmv_y)>>3)) & 0x1]\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)


#define evc_dmvr_mc_c(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_dmvr_mc_c[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2)| ((gmv_x)>>3) | ((gmv_x)>>4)) & 0x1]\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2) | ((gmv_y)>>3) | ((gmv_y)>>4)) & 0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)
#endif
#else

#define evc_mc_l(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_mc_l[((gmv_x)|((gmv_x)>>1))&0x1][((gmv_y)|((gmv_y)>>1))&0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)


#define evc_mc_c(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_mc_c[((gmv_x)|((gmv_x)>>1)|((gmv_x)>>2))&0x1]\
        [((gmv_y)|((gmv_y)>>1)|((gmv_y)>>2))&0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)
#endif

#if MC_PRECISION_ADD
#define evc_bl_mc_l(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_bl_mc_l[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2) | ((gmv_x)>>3)) & 0x1])\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2) | ((gmv_y)>>3)) & 0x1]\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)
#else
#define evc_bl_mc_l(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h) \
    (evc_tbl_bl_mc_l[((gmv_x)|((gmv_x)>>1))&0x1][((gmv_y)|((gmv_y)>>1))&0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h)
#endif

#if DMVR
void evc_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*mv)[MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[REFP_NUM][N_C][MAX_CU_DIM], int poc_c, pel dmvr_ref_pred_template[MAX_CU_SIZE*MAX_CU_SIZE], pel dmvr_ref_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + ((DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))]
            , pel dmvr_half_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)]
            , BOOL apply_DMVR
#if DMVR_PADDING
            , pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
#endif
#if DMVR_FLAG 
            , u8 *cu_dmvr_flag
#if DMVR_LAG
            , s16 dmvr_mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D]
#endif
#endif
            , int sps_amis_flag
#else
void evc_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[2][N_C][MAX_CU_DIM]
#endif
);
#if IBC
void evc_IBC_mc(int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D], EVC_PIC *ref_pic, pel pred[N_C][MAX_CU_DIM]
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
);
#endif
void mv_clip(int x, int y, int pic_w, int pic_h, int w, int h,
             s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], s16(*mv_t)[MV_D]);
#if AFFINE
void evc_affine_mc(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][VER_NUM][MV_D], EVC_REFP(*refp)[REFP_NUM], pel pred[2][N_C][MAX_CU_DIM], int vertex_num
#if EIF
                   , pel* tmp_buffer
#endif

);
void evc_affine_mc_l(int x, int y, int pic_w, int pic_h, int cuw, int cuh, s16 ac_mv[VER_NUM][MV_D], EVC_PIC* ref_pic, pel pred[MAX_CU_DIM], int vertex_num
#if EIF
                     , pel* tmp_buffer
#endif
);
void evc_affine_mc_lc(int x, int y, int pic_w, int pic_h, int cuw, int cuh, s16 ac_mv[VER_NUM][MV_D], EVC_PIC* ref_pic, pel pred[N_C][MAX_CU_DIM], int vertex_num
#if M50761_AFFINE_ADAPT_SUB_SIZE
                      , int sub_w, int sub_h
#endif
#if EIF
                      , pel* tmp_buffer
#endif
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV_HW
                      , BOOL mem_band_conditions_for_eif_are_satisfied
#endif
);

#if EIF
void evc_eif_mc(int block_width, int block_height, int x, int y, int mv_scale_hor, int mv_scale_ver, int dmv_hor_x, int dmv_hor_y, int dmv_ver_x, int dmv_ver_y,
                int hor_max, int ver_max, int hor_min, int ver_min, pel* p_ref, int ref_stride, pel *p_dst, int dst_stride, pel* p_tmp_buf, char affine_mv_prec, s8 comp);
#endif
#endif

#if OPT_SIMD_MC_L
void average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, int s_src, int s_ref, int s_dst, int wd, int ht);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _EVC_MC_H_ */
