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

#ifndef _EVC_UTIL_H_
#define _EVC_UTIL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "evc_def.h"
#include <stdlib.h>

/*! macro to determine maximum */
#define EVC_MAX(a,b)                   (((a) > (b)) ? (a) : (b))

/*! macro to determine minimum */
#define EVC_MIN(a,b)                   (((a) < (b)) ? (a) : (b))

/*! macro to absolute a value */
#define EVC_ABS(a)                     abs(a)

/*! macro to absolute a 64-bit value */
#define EVC_ABS64(a)                   (((a)^((a)>>63)) - ((a)>>63))

/*! macro to absolute a 32-bit value */
#define EVC_ABS32(a)                   (((a)^((a)>>31)) - ((a)>>31))

/*! macro to absolute a 16-bit value */
#define EVC_ABS16(a)                   (((a)^((a)>>15)) - ((a)>>15))

/*! macro to clipping within min and max */
#define EVC_CLIP3(min_x, max_x, value)   EVC_MAX((min_x), EVC_MIN((max_x), (value)))

/*! macro to clipping within min and max */
#define EVC_CLIP(n,min,max)            (((n)>(max))? (max) : (((n)<(min))? (min) : (n)))

#define EVC_SIGN(x)                    (((x) < 0) ? -1 : 1)

/*! macro to get a sign from a 16-bit value.\n
operation: if(val < 0) return 1, else return 0 */
#define EVC_SIGN_GET(val)              ((val<0)? 1: 0)

/*! macro to set sign into a value.\n
operation: if(sign == 0) return val, else if(sign == 1) return -val */
#define EVC_SIGN_SET(val, sign)        ((sign)? -val : val)

/*! macro to get a sign from a 16-bit value.\n
operation: if(val < 0) return 1, else return 0 */
#define EVC_SIGN_GET16(val)            (((val)>>15) & 1)

/*! macro to set sign into a 16-bit value.\n
operation: if(sign == 0) return val, else if(sign == 1) return -val */
#define EVC_SIGN_SET16(val, sign)      (((val) ^ ((s16)((sign)<<15)>>15)) + (sign))

#define EVC_ALIGN(val, align)          ((((val) + (align) - 1) / (align)) * (align))

#define CONV_LOG2(v)                    (evc_tbl_log2[v])

BOOL is_ptr_aligned(void* ptr, int num_bytes);

u16 evc_get_avail_inter(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int cuw, int cuh, u32 *map_scu
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif 
);
u16 evc_get_avail_intra(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int log2_cuw, int log2_cuh, u32 *map_scu
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif 
);
u16 evc_get_avail_ibc(int x_scu, int y_scu, int w_scu, int h_scu, int scup, int cuw, int cuh, u32 * map_scu);
EVC_PIC* evc_picbuf_alloc(int w, int h, int pad_l, int pad_c, int *err);
void evc_picbuf_free(EVC_PIC *pic);
void evc_picbuf_expand(EVC_PIC *pic, int exp_l, int exp_c);

void evc_poc_derivation(EVC_SPS sps, int tid, EVC_POC *poc);

void evc_get_mmvd_mvp_list(s8(*map_refi)[REFP_NUM], EVC_REFP refp[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int w_scu, int h_scu, int scup, u16 avail, int cuw, int cuh, int slice_t, int real_mv[][2][3], u32 *map_scu, int REF_SET[][MAX_NUM_ACTIVE_REF_FRAME], u16 avail_lr
#if M52166_MMVD
    , u32 curr_ptr, u8 num_refp[REFP_NUM]
#endif
    , EVC_HISTORY_BUFFER history_buffer, int admvp_flag, EVC_SH* sh
#if M50761_TMVP_8X8_GRID
    , int log2_max_cuwh
#endif
);

void evc_check_motion_availability(int scup, int cuw, int cuh, int w_scu, int h_scu, int neb_addr[MAX_NUM_POSSIBLE_SCAND], int valid_flag[MAX_NUM_POSSIBLE_SCAND], u32 *map_scu, u16 avail_lr, int num_mvp, int is_ibc);
void evc_get_default_motion(int neb_addr[MAX_NUM_POSSIBLE_SCAND], int valid_flag[MAX_NUM_POSSIBLE_SCAND], s8 cur_refi, int lidx, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], s8 *refi, s16 mv[MV_D]
#if DMVR_LAG
                            , u32 *map_scu
                            , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
                            , int scup
                            , int w_scu
#endif
                            , EVC_HISTORY_BUFFER history_buffer
                            , int admvp_flag);

s8 evc_get_first_refi(int scup, int lidx, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D], int cuw, int cuh, int w_scu, int h_scu, u32 *map_scu, u8 mvr_idx, u16 avail_lr
#if DMVR_LAG
                      , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
                      , EVC_HISTORY_BUFFER history_buffer
                      , int admvp_flag);

void evc_get_motion(int scup, int lidx, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D],
                    EVC_REFP(*refp)[REFP_NUM], int cuw, int cuh, int w_scu, u16 avail, s8 refi[MAX_NUM_MVP], s16 mvp[MAX_NUM_MVP][MV_D]);
void evc_get_motion_merge_main(int poc, int slice_type, int scup, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D],
    EVC_REFP refp[REFP_NUM], int cuw, int cuh, int w_scu, int h_scu, s8 refi[REFP_NUM][MAX_NUM_MVP], s16 mvp[REFP_NUM][MAX_NUM_MVP][MV_D], u32 *map_scu, u16 avail_lr
#if DMVR_LAG
    , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
    , EVC_HISTORY_BUFFER history_buffer
    , u8 ibc_flag
    , EVC_REFP(*refplx)[REFP_NUM]
    , EVC_SH* sh
#if M50761_TMVP_8X8_GRID
    , int log2_max_cuwh
#endif
);
#if M52165
void evc_get_merge_insert_mv(s8* refi_dst, s16 *mvp_dst_L0, s16 *mvp_dst_L1, s8* map_refi_src, s16* map_mv_src, int slice_type, int cuw, int cuh, int is_sps_admvp);
#else
void evc_get_merge_insert_mv(s8* refi_dst, s16 *mvp_dst_L0, s16 *mvp_dst_L1, s8* map_refi_src, s16* map_mv_src, int slice_type, int cuw, int cuh, int is_sps_amis);
#endif

void evc_get_motion_skip_baseline(int slice_type, int scup, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D],
    EVC_REFP refp[REFP_NUM], int cuw, int cuh, int w_scu, s8 refi[REFP_NUM][MAX_NUM_MVP], s16 mvp[REFP_NUM][MAX_NUM_MVP][MV_D], u16 avail_lr
);
void evc_get_mv_collocated(EVC_REFP(*refp)[REFP_NUM], u32 poc, int scup, int c_scu, u16 w_scu, u16 h_scu, s16 mvp[REFP_NUM][MV_D], s8 *availablePredIdx, EVC_SH* sh);
void evc_get_motion_from_mvr(u8 mvr_idx, int poc, int scup, int lidx, s8 cur_refi, int num_refp, \
                             s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], EVC_REFP(*refp)[REFP_NUM], \
                             int cuw, int cuh, int w_scu, int h_scu, u16 avail, s16 mvp[MAX_NUM_MVP][MV_D], s8 refi_pred[MAX_NUM_MVP], u32* map_scu, u16 avail_lr
#if DMVR_LAG
                             , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
                             , EVC_HISTORY_BUFFER history_buffer
                             , int admvp_flag);

void evc_get_motion_scaling(int poc, int scup, int lidx, s8 cur_refi, int num_refp, \
                            s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], EVC_REFP(*refp)[REFP_NUM], \
                            int cuw, int cuh, int w_scu, int h_scu, u16 avail, s16 mvp[MAX_NUM_MVP][MV_D], s8 refi_pred[MAX_NUM_MVP], u32* map_scu, u16 avail_lr
#if DMVR_LAG
                            , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
);

enum
{
    SPLIT_MAX_PART_COUNT = 4
};

typedef struct _EVC_SPLIT_STRUCT
{
    int       part_count;
    int       cud[SPLIT_MAX_PART_COUNT];
    int       width[SPLIT_MAX_PART_COUNT];
    int       height[SPLIT_MAX_PART_COUNT];
    int       log_cuw[SPLIT_MAX_PART_COUNT];
    int       log_cuh[SPLIT_MAX_PART_COUNT];
    int       x_pos[SPLIT_MAX_PART_COUNT];
    int       y_pos[SPLIT_MAX_PART_COUNT];
    int       cup[SPLIT_MAX_PART_COUNT];
#if M50761_CHROMA_NOT_SPLIT
    TREE_CONS tree_cons;
#endif
} EVC_SPLIT_STRUCT;

//! Count of partitions, correspond to split_mode
int evc_split_part_count(int split_mode);
//! Get partition size
int evc_split_get_part_size(int split_mode, int part_num, int length);
//! Get partition size log
int evc_split_get_part_size_idx(int split_mode, int part_num, int length_idx);
//! Get partition split structure
void evc_split_get_part_structure(int split_mode, int x0, int y0, int cuw, int cuh, int cup, int cud, int log2_culine, EVC_SPLIT_STRUCT* split_struct
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons /*, u8 slice_type */
#endif
);
void evc_split_get_part_structure_d( int split_mode, int x0, int y0, int cuw, int cuh, int cup, int cud, int log2_culine, EVC_SPLIT_STRUCT* split_struct ); //TODO: Tim: remove after full refactoring
//! Get array of split modes tried sequentially in RDO
void evc_split_get_split_rdo_order(int cuw, int cuh, SPLIT_MODE splits[MAX_SPLIT_NUM]);
//! Get split direction. Quad will return vertical direction.
SPLIT_DIR evc_split_get_direction(SPLIT_MODE mode);
//! Get SUCO partition order
void evc_split_get_suco_order(int suco_flag, SPLIT_MODE mode, int suco_order[SPLIT_MAX_PART_COUNT]);
//! Is mode triple tree?
int  evc_split_is_TT(SPLIT_MODE mode);
//! Is mode BT?
int  evc_split_is_BT(SPLIT_MODE mode);
//! Check that mode is vertical
int evc_split_is_vertical(SPLIT_MODE mode);
//! Check that mode is horizontal
int evc_split_is_horizontal(SPLIT_MODE mode);

void evc_get_mv_dir(EVC_REFP refp[REFP_NUM], u32 poc, int scup, int c_scu, u16 w_scu, u16 h_scu, s16 mvp[REFP_NUM][MV_D], int sps_admvp_flag);
int evc_get_avail_cu(int neb_scua[MAX_NEB2], u32 * map_cu);
int evc_scan_tbl_init();
int evc_scan_tbl_delete();
int evc_get_split_mode(s8* split_mode, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
#if !M50761_CHROMA_NOT_SPLIT_CLEANUP
int
#else
void 
#endif
evc_set_split_mode(s8  split_mode, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*split_mode_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
int evc_get_suco_flag(s8* suco_flag, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*suco_flag_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
#if !M50761_CHROMA_NOT_SPLIT_CLEANUP
int
#else
void
#endif
evc_set_suco_flag(s8  suco_flag, int cud, int cup, int cuw, int cuh, int lcu_s, s8(*suco_flag_buf)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);
u8  evc_check_suco_cond(int cuw, int cuh, s8 split_mode, int boundary, u8 log2_max_cuwh, u8 suco_max_depth, u8 suco_depth);
u16 evc_check_nev_avail(int x_scu, int y_scu, int cuw, int cuh, int w_scu, int h_scu, u32 * map_scu
#if EVC_TILE_SUPPORT
    , u8* map_tidx
#endif
);

void evc_get_ctx_some_flags(int x_scu, int y_scu, int cuw, int cuh, int w_scu, u32* map_scu, u32* map_cu_mode, u8* ctx, u8 slice_type, int sps_cm_init_flag, u8 ibc_flag, u8 ibc_log_max_size);

void evc_mv_rounding_s32( s32 hor, int ver, s32 * rounded_hor, s32 * rounded_ver, s32 right_shift, int left_shift );

#if EIF_CLIPPING_REDESIGN
void evc_rounding_s32(s32 comp, s32 *rounded_comp, int right_shift, int left_shift);
#endif

#if M50761_AFFINE_ADAPT_SUB_SIZE
void derive_affine_subblock_size_bi( s16 ac_mv[REFP_NUM][VER_NUM][MV_D], s8 refi[REFP_NUM], int cuw, int cuh, int *sub_w, int *sub_h, int vertex_num
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV_HW
  , BOOL*mem_band_conditions_for_eif_are_satisfied
#endif
);
#endif
void derive_affine_subblock_size( s16 ac_mv[VER_NUM][MV_D], int cuw, int cuh, int *sub_w, int *sub_h, int vertex_num
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV_HW
  , BOOL*mem_band_conditions_for_eif_are_satisfied
#endif
);

#if M50761_EIF_RESTRICTIONS
BOOL check_eif_applicability_bi( s16 ac_mv[REFP_NUM][VER_NUM][MV_D], s8 refi[REFP_NUM], int cuw, int cuh, int vertex_num
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV_HW
  , BOOL* mem_band_conditions_are_satisfied
#endif
);

BOOL check_eif_applicability_uni( s16 ac_mv[VER_NUM][MV_D], int cuw, int cuh, int vertex_num
#if M51449_HARMONIZED_AFFINE_BANDWIDTH_CLIPMV_HW
  , BOOL* mem_band_conditions_are_satisfied
#endif
);
#endif

void evc_get_affine_motion_scaling(int poc, int scup, int lidx, s8 cur_refi, int num_refp, \
                                   s16(*map_mv)[REFP_NUM][MV_D], s8(*map_refi)[REFP_NUM], EVC_REFP(*refp)[REFP_NUM], \
                                   int cuw, int cuh, int w_scu, int h_scu, u16 avail, s16 mvp[MAX_NUM_MVP][VER_NUM][MV_D], s8 refi[MAX_NUM_MVP]
                                   , u32* map_scu, u32* map_affine, int vertex_num, u16 avail_lr
                                   , int log2_max_cuwh
#if DMVR_LAG
                                   , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
);

int evc_get_affine_merge_candidate(int poc, int slice_type, int scup, s8(*map_refi)[REFP_NUM], s16(*map_mv)[REFP_NUM][MV_D],
                                   EVC_REFP(*refp)[REFP_NUM], int cuw, int cuh, int w_scu, int h_scu, u16 avail, s8 mrg_list_refi[AFF_MAX_CAND][REFP_NUM], s16 mrg_list_cp_mv[AFF_MAX_CAND][REFP_NUM][VER_NUM][MV_D], int mrg_list_cp_num[AFF_MAX_CAND], u32* map_scu, u32* map_affine
                                   , int log2_max_cuwh
#if DMVR_LAG
                                   , s16(*map_unrefined_mv)[REFP_NUM][MV_D]
#endif
    , u16 avail_lr
#if M50761_TMVP_ALIGN_SPEC || M50662_AFFINE_IBC_TMVP_SUCO_FIX
    , EVC_SH * sh
#endif
);

#if !EIF_MEMORY_BANDWIDTH_RESTRICTION
int evc_get_affine_memory_access(s16 mv[VER_NUM][MV_D], int cuw, int cuh);
#endif

/* MD5 structure */
typedef struct _EVC_MD5
{
    u32     h[4]; /* hash state ABCD */
    u8      msg[64]; /*input buffer (nalu message) */
    u32     bits[2]; /* number of bits, modulo 2^64 (lsb first)*/
} EVC_MD5;

/* MD5 Functions */
void evc_md5_init(EVC_MD5 * md5);
void evc_md5_update(EVC_MD5 * md5, void * buf, u32 len);
void evc_md5_update_16(EVC_MD5 * md5, void * buf, u32 len);
void evc_md5_finish(EVC_MD5 * md5, u8 digest[16]);
int evc_md5_imgb(EVC_IMGB * imgb, u8 digest[16]);

int evc_picbuf_signature(EVC_PIC * pic, u8 * md5_out);

int evc_atomic_inc(volatile int * pcnt);
int evc_atomic_dec(volatile int * pcnt);
#if M52166_PARTITION
#define ALLOW_SPLIT_RATIO(long_side, block_ratio) (block_ratio <= BLOCK_14 && (long_side <= evc_split_tbl[block_ratio][IDX_MAX] && long_side >= evc_split_tbl[block_ratio][IDX_MIN]) ? 1 : 0)
#define ALLOW_SPLIT_TRI(long_side) ((long_side <= evc_split_tbl[BLOCK_TT][IDX_MAX] && long_side >= evc_split_tbl[BLOCK_TT][IDX_MIN]) ? 1 : 0)
#else
#define ALLOW_SPLIT_RATIO(long_side, block_ratio) (block_ratio < 5 && (long_side <= evc_split_tbl[block_ratio][0] && long_side >= evc_split_tbl[block_ratio][1]) ? 1 : 0)
#define ALLOW_SPLIT_TRI(long_side) ((long_side <= evc_split_tbl[5][0] && long_side >= evc_split_tbl[5][1]) ? 1 : 0)
#endif
void evc_check_split_mode(int *split_allow, int log2_cuw, int log2_cuh, int boundary, int boundary_b, int boundary_r, int log2_max_cuwh
                          , const int parent_split, int* same_layer_split, const int node_idx, const int* parent_split_allow, int qt_depth, int btt_depth
                          , int x, int y, int im_w, int im_h
                          , u8* remaining_split, int sps_btt_flag
#if M50761_CHROMA_NOT_SPLIT
                          , MODE_CONS mode_cons
#endif
);

#if DQP
u8  *evc_get_dqp_used(int x_scu, int y_scu, int w_scu, u8 * map_dqp_input, int dqp_depth);
#endif

void evc_init_scan_sr(int *scan, int size_x, int size_y, int width, int height, int scan_type);
void evc_init_inverse_scan_sr(u16 *scan_inv, u16 *scan_orig, int width, int height, int scan_type);
void evc_get_ctx_last_pos_xy_para(int ch_type, int width, int height, int *result_offset_x, int *result_offset_y, int *result_shift_x, int *result_shift_y);
#if M50631_IMPROVEMENT_ADCC_CTXGT12
int evc_get_ctx_gt0_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type);
int evc_get_ctx_gtA_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type);
int evc_get_ctx_gtB_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type);
#else
int evc_get_ctx_gt0_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y);
int evc_get_ctx_gtA_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y);
int evc_get_ctx_gtB_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y);
#endif
int evc_get_ctx_remain_inc(s16 *pcoeff, int blkpos, int width, int height, int ch_type, int sr_x, int sr_y);
int get_rice_para(s16 *pcoeff, int blkpos, int width, int height, int base_level);
#ifdef __cplusplus
}
#endif

int evc_get_transform_shift(int log2_size, int type);

void evc_eco_sbac_ctx_initialize(SBAC_CTX_MODEL *ctx, s16 *ctx_init_model, u16 num_ctx, u8 slice_type, u8 slice_qp);

#if SIMD_CLIP
void clip_simd(const pel* src, int src_stride, pel *dst, int dst_stride, int width, int height, const int clp_rng_min, const int clp_rng_max);
#endif

u8 check_ats_inter_info_coded(int cuw, int cuh, int pred_mode, int tool_ats);
void get_tu_size(u8 ats_inter_info, int log2_cuw, int log2_cuh, int* log2_tuw, int* log2_tuh);
void get_tu_pos_offset(u8 ats_inter_info, int log2_cuw, int log2_cuh, int* x_offset, int* y_offset);
void get_ats_inter_trs(u8 ats_inter_info, int log2_cuw, int log2_cuh, u8* ats_cu, u8* ats_tu);
void set_cu_cbf_flags(u8 cbf_y, u8 ats_inter_info, int log2_cuw, int log2_cuh, u32 *map_scu, int w_scu);
#if M52165
BOOL check_bi_applicability(int slice_type, int cuw, int cuh, int is_sps_admvp);
#else
BOOL check_bi_applicability(int slice_type, int cuw, int cuh, int is_sps_amis);
#endif
void evc_block_copy(s16 * src, int src_stride, s16 * dst, int dst_stride, int log2_copy_w, int log2_copy_h);

#if M50761_CHROMA_NOT_SPLIT
u8 evc_check_chroma_split_allowed(int luma_width, int luma_height);
u8 evc_is_chroma_split_allowed(int w, int h, SPLIT_MODE split);
int evc_get_luma_cup(int x_scu, int y_scu, int cu_w_scu, int cu_h_scu, int w_scu);
enum TQC_RUN evc_get_run(enum TQC_RUN run_list, TREE_CONS tree_cons);
u8 evc_check_luma(TREE_CONS tree_cons);
u8 evc_check_chroma(TREE_CONS tree_cons);
u8 evc_check_all(TREE_CONS tree_cons);
u8 evc_check_only_intra(TREE_CONS tree_cons);
u8 evc_check_only_inter(TREE_CONS tree_cons);
u8 evc_check_all_preds(TREE_CONS tree_cons);
//u8 evc_get_cur_tree(TREE_CONS tree_cons);       // Return current tree type: 0 - luma (or dual) and 1 - chroma.
TREE_CONS evc_get_default_tree_cons();
void evc_set_tree_mode(TREE_CONS* dest, MODE_CONS mode);
MODE_CONS evc_get_mode_cons_by_split(SPLIT_MODE split_mode, int cuw, int cuh);
#endif




#if GRAB_STAT
void enc_stat_header(int pic_w, int pic_h);
#endif

#endif /* _EVC_UTIL_H_ */
