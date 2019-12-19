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

#include "../src/evc_def.h"
#include "evca_util.h"
#include "evca_args.h"
#include <math.h>

#define SCRIPT_REPORT              1
#define MULT_CONFIG                1
#define VERBOSE_NONE               VERBOSE_0
#define VERBOSE_FRAME              VERBOSE_1
#define VERBOSE_ALL                VERBOSE_2
#define MAX_BUMP_FRM_CNT           (8 <<1)

#define MAX_BS_BUF                 (16*1024*1024)

typedef enum _STATES
{
    STATE_ENCODING,
    STATE_BUMPING,
    STATE_SKIPPING
} STATES;

typedef struct _IMGB_LIST
{
    EVC_IMGB    * imgb;
    int            used;
    EVC_MTIME     ts;
} IMGB_LIST;

#define     MAX_SCALE       5    //number of MS-SSIM scales
#define     WIN_SIZE        11   //window size for MS-SSIM calculation

static double exponent[MAX_SCALE] = { 0.0448, 0.2856, 0.3001, 0.2363, 0.1333 };  //weights for different scales

enum STRUCTURE
{
    LUMA_COMPONENT,
    SIMILARITY_COMPONENT,
    SSIM,
};

static char op_fname_cfg[256]     = "\0"; /* config file path name */
static char op_fname_inp[256]     = "\0"; /* input original video */
static char op_fname_out[256]     = "\0"; /* output bitstream */
static char op_fname_rec[256]     = "\0"; /* reconstructed video */
static int  op_max_frm_num        = 0;
static int  op_use_pic_signature  = 0;
static int  op_w                  = 0;
static int  op_h                  = 0;
static int  op_qp                 = 0;
static int  op_fps                = 0;
static int  op_iperiod            = 0;
static int  op_max_b_frames       = 0;
static int  op_ref_pic_gap_length = 0;
static int  op_closed_gop         = 0;
#if IBC
static int  op_enable_ibc = 0;
static int  op_ibc_search_range_x = IBC_SEARCH_RANGE;
static int  op_ibc_search_range_y = IBC_SEARCH_RANGE;
static int  op_ibc_hash_search_flag = 0;
static int  op_ibc_hash_search_max_cand = IBC_NUM_CANDIDATES;
static int  op_ibc_hash_search_range_4smallblk = IBC_SEARCH_RANGE;
static int  op_ibc_fast_method = IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE;
#endif
static int  op_disable_hgop       = 0;
static int  op_in_bit_depth       = 8;
static int  op_skip_frames        = 0;
static int  op_out_bit_depth      = 0; /* same as input bit depth */
static int  op_profile            = 1;
static int  op_level              = 0;
static int  op_btt                = 1;
static int  op_suco               = 1;
static int  op_add_qp_frames      = 0;
static int  op_framework_ctu_size = 7;
static int  op_framework_cu11_max = 7;
static int  op_framework_cu11_min = 2;
static int  op_framework_cu12_max = 7;
static int  op_framework_cu12_min = 3;
static int  op_framework_cu14_max = 6;
static int  op_framework_cu14_min = 4;
static int  op_framework_tris_max = 6;
static int  op_framework_tris_min = 4;
static int  op_framework_suco_max = 6;
static int  op_framework_suco_min = 4;
static int  op_tool_amvr          = 1; /* default on */
static int  op_tool_mmvd          = 1; /* default on */
static int  op_tool_affine        = 1; /* default on */
static int  op_tool_dmvr          = 1; /* default on */
static int  op_tool_alf           = 1; /* default on */
static int  op_tool_admvp         = 1; /* default on */
static int  op_tool_amis          = 1; /* default on */
static int  op_tool_htdf          = 1; /* default on */
static int  op_tool_eipd          = 1; /* default on */
static int  op_tool_iqt           = 1; /* default on */
static int  op_tool_cm_init       = 1; /* default on */
static int  op_tool_adcc          = 1; /* default on */
static int  op_cb_qp_offset       = 0;
static int  op_cr_qp_offset       = 0;
#if ATS_INTER_PROCESS
static int op_tool_ats            = 1; /* default on */
#endif
static int  op_constrained_intra_pred = 0;
static int  op_deblock_alpha_offset = 0; /* default offset 0*/
static int  op_deblock_beta_offset = 0;  /* default offset 0*/

#if EVC_TILE_SUPPORT
static int  op_tile_uniform_spacing = 1;
static int  op_num_tile_columns_minus1 = 0;     //default 1
static int  op_num_tile_rows_minus1 = 0;        //default 1
static char op_tile_column_width_array[MAX_NUM_TILES_COL];
static char op_tile_row_height_array[MAX_NUM_TILES_ROW];
static int  op_num_slice_in_pic_minus1 = 0;     // default 1 
static char op_slice_boundary_array[2 * 600];   // Max. slices can be 600 for the highest level 6.2
#endif

#if CHROMA_QP_TABLE_SUPPORT_M50663
static int  op_chroma_qp_table_present_flag = 0;
static char op_chroma_qp_num_points_in_table[2] = {0};
static char op_chroma_qp_delta_in_val_cb[MAX_QP_TABLE_SIZE_EXT] = {0};
static char op_chroma_qp_delta_out_val_cb[MAX_QP_TABLE_SIZE_EXT] = { 0 };
static char op_chroma_qp_delta_in_val_cr[MAX_QP_TABLE_SIZE_EXT] = { 0 };
static char op_chroma_qp_delta_out_val_cr[MAX_QP_TABLE_SIZE_EXT] = { 0 };
#endif

static char  op_rpl0[MAX_NUM_RPLS][256];
static char  op_rpl1[MAX_NUM_RPLS][256];

#if DQP_CFG
static int  op_use_dqp             = 0;  /* default cu_delta_qp is off */
static int  op_cu_qp_delta_area    = 10; /* default cu_delta_qp_area is 10 */
#endif

typedef enum _OP_FLAGS
{
    OP_FLAG_FNAME_CFG,
    OP_FLAG_FNAME_INP,
    OP_FLAG_FNAME_OUT,
    OP_FLAG_FNAME_REC,
    OP_FLAG_WIDTH_INP,
    OP_FLAG_HEIGHT_INP,
    OP_FLAG_QP,
#if DQP_CFG
    OP_FLAG_USE_DQP,
    OP_FLAG_CU_QP_DELTA_AREA,
#endif
    OP_FLAG_FPS,
    OP_FLAG_IPERIOD,
    OP_FLAG_MAX_FRM_NUM,
    OP_FLAG_USE_PIC_SIGN,
    OP_FLAG_VERBOSE,
    OP_FLAG_MAX_B_FRAMES,
    OP_FLAG_CLOSED_GOP,
#if IBC
    OP_FLAG_IBC,
    OP_IBC_SEARCH_RNG_X,
    OP_IBC_SEARCH_RND_Y,
    OP_IBC_HASH_FLAG,
    OP_IBC_HASH_SEARCH_MAX_CAND,
    OP_IBC_HASH_SEARCH_RANGE_4SMALLBLK,
    OP_IBC_FAST_METHOD,
#endif
    OP_FLAG_DISABLE_HGOP,
    OP_FLAG_OUT_BIT_DEPTH,
    OP_FLAG_IN_BIT_DEPTH,
    OP_FLAG_SKIP_FRAMES,
    OP_PROFILE,
    OP_LEVEL,
    OP_BTT,
    OP_SUCO,
    OP_FLAG_ADD_QP_FRAME,
    OP_FRAMEWORK_CTU_SIZE,
    OP_FRAMEWORK_CU11_MAX,
    OP_FRAMEWORK_CU11_MIN,
    OP_FRAMEWORK_CU12_MAX,
    OP_FRAMEWORK_CU12_MIN,
    OP_FRAMEWORK_CU14_MAX,
    OP_FRAMEWORK_CU14_MIN,
    OP_FRAMEWORK_TRIS_MAX,
    OP_FRAMEWORK_TRIS_MIN,
    OP_FRAMEWORK_SUCO_MAX,
    OP_FRAMEWORK_SUCO_MIN,
    OP_TOOL_AMVR,
    OP_TOOL_MMVD,
    OP_TOOL_AFFINE,
    OP_TOOL_DMVR,
    OP_TOOL_ALF,
    OP_TOOL_HTDF,
    OP_TOOL_ADMVP,
    OP_TOOL_AMIS,
    OP_TOOL_EIPD,
    OP_TOOL_IQT,
    OP_TOOL_CM_INIT,
    OP_TOOL_ADCC,
    OP_CB_QP_OFFSET,
    OP_CR_QP_OFFSET,
#if ATS_INTER_PROCESS
    OP_TOOL_ATS,
#endif
    OP_CONSTRAINED_INTRA_PRED,
    OP_TOOL_DBFOFFSET,
#if EVC_TILE_SUPPORT
    OP_TILE_UNIFORM_SPACING,
    OP_NUM_TILE_COLUMNS_MINUS1,
    OP_NUM_TILE_ROWS_MINUS1,
    OP_TILE_COLUMN_WIDTH_ARRAY,
    OP_TILE_ROW_HEIGHT_ARRAY,
    OP_NUM_SLICE_IN_PIC_MINUS1,
    OP_SLICE_BOUNDARY_ARRAY,
#endif
#if CHROMA_QP_TABLE_SUPPORT_M50663
    OP_CHROMA_QP_TABLE_PRESENT_FLAG,
    OP_CHROMA_QP_NUM_POINTS_IN_TABLE,
    OP_CHROMA_QP_DELTA_IN_VAL_CB,
    OP_CHROMA_QP_DELTA_OUT_VAL_CB,
    OP_CHROMA_QP_DELTA_IN_VAL_CR,
    OP_CHROMA_QP_DELTA_OUT_VAL_CR,
#endif
    OP_FLAG_RPL0_0,
    OP_FLAG_RPL0_1,
    OP_FLAG_RPL0_2,
    OP_FLAG_RPL0_3,
    OP_FLAG_RPL0_4,
    OP_FLAG_RPL0_5,
    OP_FLAG_RPL0_6,
    OP_FLAG_RPL0_7,
    OP_FLAG_RPL0_8,
    OP_FLAG_RPL0_9,
    OP_FLAG_RPL0_10,
    OP_FLAG_RPL0_11,
    OP_FLAG_RPL0_12,
    OP_FLAG_RPL0_13,
    OP_FLAG_RPL0_14,
    OP_FLAG_RPL0_15,
    OP_FLAG_RPL0_16,
    OP_FLAG_RPL0_17,
    OP_FLAG_RPL0_18,
    OP_FLAG_RPL0_19,
    OP_FLAG_RPL0_20,
    //...
    OP_FLAG_RPL0_31,

    OP_FLAG_RPL1_0,
    OP_FLAG_RPL1_1,
    OP_FLAG_RPL1_2,
    OP_FLAG_RPL1_3,
    OP_FLAG_RPL1_4,
    OP_FLAG_RPL1_5,
    OP_FLAG_RPL1_6,
    OP_FLAG_RPL1_7,
    OP_FLAG_RPL1_8,
    OP_FLAG_RPL1_9,
    OP_FLAG_RPL1_10,
    OP_FLAG_RPL1_11,
    OP_FLAG_RPL1_12,
    OP_FLAG_RPL1_13,
    OP_FLAG_RPL1_14,
    OP_FLAG_RPL1_15,
    OP_FLAG_RPL1_16,
    OP_FLAG_RPL1_17,
    OP_FLAG_RPL1_18,
    OP_FLAG_RPL1_19,
    OP_FLAG_RPL1_20,
    //...
    OP_FLAG_RPL1_31,

    OP_FLAG_MAX
} OP_FLAGS;

static int op_flag[OP_FLAG_MAX] = {0};

static EVC_ARGS_OPTION options[] = \
{
    {
        EVC_ARGS_NO_KEY, EVC_ARGS_KEY_LONG_CONFIG, EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_CFG], op_fname_cfg,
        "file name of configuration"
    },
    {
        'i', "input", EVC_ARGS_VAL_TYPE_STRING|EVC_ARGS_VAL_TYPE_MANDATORY,
        &op_flag[OP_FLAG_FNAME_INP], op_fname_inp,
        "file name of input video"
    },
    {
        'o', "output", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_OUT], op_fname_out,
        "file name of output bitstream"
    },
    {
        'r', "recon", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_REC], op_fname_rec,
        "file name of reconstructed video"
    },
    {
        'w',  "width", EVC_ARGS_VAL_TYPE_INTEGER|EVC_ARGS_VAL_TYPE_MANDATORY,
        &op_flag[OP_FLAG_WIDTH_INP], &op_w,
        "pixel width of input video"
    },
    {
        'h',  "height", EVC_ARGS_VAL_TYPE_INTEGER|EVC_ARGS_VAL_TYPE_MANDATORY,
        &op_flag[OP_FLAG_HEIGHT_INP], &op_h,
        "pixel height of input video"
    },
    {
        'q',  "op_qp", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP], &op_qp,
        "QP value (0~51)"
    },
#if DQP_CFG
    {
     EVC_ARGS_NO_KEY,  "use_dqp", EVC_ARGS_VAL_TYPE_INTEGER,
     &op_flag[OP_FLAG_USE_DQP], &op_use_dqp,
     "use_dqp ({0,..,25})(default: 0) "
    },
    {
        EVC_ARGS_NO_KEY,  "cu_qp_delta_area", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_CU_QP_DELTA_AREA], &op_cu_qp_delta_area,
        "cu_qp_delta_area (>= 6)(default: 6) "
    },
#endif
    {
        'z',  "hz", EVC_ARGS_VAL_TYPE_INTEGER|EVC_ARGS_VAL_TYPE_MANDATORY,
        &op_flag[OP_FLAG_FPS], &op_fps,
        "frame rate (Hz)"
    },
    {
        'p',  "iperiod", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_IPERIOD], &op_iperiod,
        "I-picture period"
    },
    {
        'g',  "max_b_frames", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_B_FRAMES], &op_max_b_frames,
        "Number of maximum B frames (1,3,7,15)\n"
    },
    {
        'f',  "frames", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_FRM_NUM], &op_max_frm_num,
        "maximum number of frames to be encoded"
    },
    {
        's',  "signature", EVC_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_USE_PIC_SIGN], &op_use_pic_signature,
        "embed picture signature (HASH) for conformance checking in decoding"
    },
    {
        'v',  "verbose", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_VERBOSE], &op_verbose,
        "verbose level\n"
        "\t 0: no message\n"
        "\t 1: frame-level messages (default)\n"
        "\t 2: all messages\n"
    },
    {
        'd',  "input_bit_depth", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_IN_BIT_DEPTH], &op_in_bit_depth,
        "input bitdepth (8(default), 10) "
    },
    {
        EVC_ARGS_NO_KEY,  "output_bit_depth", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_OUT_BIT_DEPTH], &op_out_bit_depth,
        "output bitdepth (8, 10)(default: same as input bitdpeth) "
    },
    {
        EVC_ARGS_NO_KEY,  "ref_pic_gap_length", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_OUT_BIT_DEPTH], &op_ref_pic_gap_length,
        "reference picture gap length (1, 2, 4, 8, 16) only available when -g is 0"
    },
    {
        EVC_ARGS_NO_KEY,  "closed_gop", EVC_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_CLOSED_GOP], &op_closed_gop,
        "use closed GOP structure. if not set, open GOP is used"
    },
#if IBC
    {
      EVC_ARGS_NO_KEY,  "ibc", EVC_ARGS_VAL_TYPE_INTEGER,
      &op_flag[OP_FLAG_IBC], &op_enable_ibc,
      "use IBC feature. if not set, IBC feature is disabled"
    },

    {
      EVC_ARGS_NO_KEY,  "ibc_search_range_x", EVC_ARGS_VAL_TYPE_INTEGER,
      &op_flag[OP_IBC_SEARCH_RNG_X], &op_ibc_search_range_x,
      "set ibc search range in horizontal direction (default 64)"
    },

    {
      EVC_ARGS_NO_KEY,  "ibc_search_range_y", EVC_ARGS_VAL_TYPE_INTEGER,
      &op_flag[OP_IBC_SEARCH_RND_Y], &op_ibc_search_range_y,
      "set ibc search range in vertical direction (default 64)"
    },

    {
      EVC_ARGS_NO_KEY,  "ibc_hash_search_flag", EVC_ARGS_VAL_TYPE_NONE,
      &op_flag[OP_IBC_HASH_FLAG], &op_ibc_hash_search_flag,
      "use IBC hash based block matching search feature. if not set, it is disable"
    },

    {
      EVC_ARGS_NO_KEY,  "ibc_hash_search_max_cand", EVC_ARGS_VAL_TYPE_INTEGER,
      &op_flag[OP_IBC_HASH_SEARCH_MAX_CAND], &op_ibc_hash_search_max_cand,
      "Max candidates for hash based IBC search (default 64)"
    },

    {
      EVC_ARGS_NO_KEY,  "ibc_hash_search_range_4smallblk", EVC_ARGS_VAL_TYPE_INTEGER,
      &op_flag[OP_IBC_HASH_SEARCH_RANGE_4SMALLBLK], &op_ibc_hash_search_range_4smallblk,
      "Small block search range in IBC based search. (default 64)"
    },

    {
      EVC_ARGS_NO_KEY,  "ibc_fast_method", EVC_ARGS_VAL_TYPE_INTEGER,
      &op_flag[OP_IBC_FAST_METHOD], &op_ibc_fast_method,
      "Fast methods for IBC\n"
      "\t 1: Buffer IBC block vector (current not support)\n"
          "\t 2: Adaptive search range (default)\n"
    },
#endif
    {
        EVC_ARGS_NO_KEY,  "disable_hgop", EVC_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_DISABLE_HGOP], &op_disable_hgop,
        "disable hierarchical GOP. if not set, hierarchical GOP is used"
    },
    {
        EVC_ARGS_NO_KEY,  "skip_frames", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_SKIP_FRAMES], &op_skip_frames,
        "number of skipped frames before encoding. default 0"
    },
    {
        EVC_ARGS_NO_KEY,  "profile", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_PROFILE], &op_profile,
        "profile setting flag  1: main, 0: baseline (default 1 (main)) "
    },
    {
        EVC_ARGS_NO_KEY,  "level", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_LEVEL], &op_level,
        "level setting "
    },
    {
        EVC_ARGS_NO_KEY,  "btt", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_BTT], &op_btt,
        "binary and ternary splits on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "suco", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_SUCO], &op_suco,
        "split unit coding ordering on/off flag"
    },
    {
        'a',  "qp_add_frm", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_ADD_QP_FRAME], &op_add_qp_frames,
        "one more qp are added after this number of frames, disable:0 (default)"
    },
    {
        EVC_ARGS_NO_KEY,  "ctu", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CTU_SIZE], &op_framework_ctu_size,
        "Max size of CTU (log scale)"
    },    
    {
        EVC_ARGS_NO_KEY,  "cu11_max", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CU11_MAX], &op_framework_cu11_max,
        "Max size of N in NxN block (log scale)"
    },
    {
        EVC_ARGS_NO_KEY,  "cu11_min", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CU11_MIN], &op_framework_cu11_min,
        "Min size of N in NxN block (log scale)"
    },
    {
        EVC_ARGS_NO_KEY,  "cu12_max", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CU12_MAX], &op_framework_cu12_max,
        "Max size of 2N in 2NxN or Nx2N block (log scale)"
    },
    {
        EVC_ARGS_NO_KEY,  "cu12_min", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CU12_MIN], &op_framework_cu12_min,
        "Min size of 2N in 2NxN or Nx2N block (log scale)"
    },
    {
        EVC_ARGS_NO_KEY,  "cu14_max", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CU14_MAX], &op_framework_cu14_max,
        "Max size of 4N in 4NxN or Nx4N block (log scale)"
    },
    {
        EVC_ARGS_NO_KEY,  "cu14_min", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_CU14_MIN], &op_framework_cu14_min,
        "Min size of 4N in 4NxN or Nx4N block (log scale)"
    },
    {
        EVC_ARGS_NO_KEY,  "tris_max", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_TRIS_MAX], &op_framework_tris_max,
        "Max size of Tri-split allowed"
    },
    {
        EVC_ARGS_NO_KEY,  "tris_min", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_TRIS_MIN], &op_framework_tris_min,
        "Min size of Tri-split allowed"
    },
    {
        EVC_ARGS_NO_KEY,  "suco_max", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_SUCO_MAX], &op_framework_suco_max,
        "Max size of suco allowed from top"
    },
    {
        EVC_ARGS_NO_KEY,  "suco_min", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FRAMEWORK_SUCO_MIN], &op_framework_suco_min,
        "Min size of suco allowed from top"
    },
    {
        EVC_ARGS_NO_KEY,  "amvr", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_AMVR], &op_tool_amvr,
        "amvr on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "mmvd", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_MMVD], &op_tool_mmvd,
        "mmvd on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "affine", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_AFFINE], &op_tool_affine,
        "affine on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "dmvr", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_DMVR], &op_tool_dmvr,
        "dmvr on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "alf", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_ALF], &op_tool_alf,
        "alf on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "htdf", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_HTDF], &op_tool_htdf,
        "htdf on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "admvp", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_ADMVP], &op_tool_admvp,
        "admvp on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "amis", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_AMIS], &op_tool_amis,
        "amis on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "eipd", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_EIPD], &op_tool_eipd,
        "eipd on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "iqt", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_IQT], &op_tool_iqt,
        "iqt on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "cm_init", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_CM_INIT], &op_tool_cm_init,
        "cm_init on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "adcc", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_ADCC], &op_tool_adcc,
        "adcc on/off flag"
    },
    {
        EVC_ARGS_NO_KEY,  "cb_qp_offset", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_CB_QP_OFFSET], &op_cb_qp_offset,
        "cb qp offset"
    },
    {
        EVC_ARGS_NO_KEY,  "cr_qp_offset", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_CR_QP_OFFSET], &op_cr_qp_offset,
        "cr qp offset"
    },
#if ATS_INTER_PROCESS
    {
         EVC_ARGS_NO_KEY,  "ats", EVC_ARGS_VAL_TYPE_INTEGER,
         &op_flag[OP_TOOL_ATS], &op_tool_ats,
         "ats on/off flag"
    },
#endif
    {
        EVC_ARGS_NO_KEY,  "constrained_intra_pred", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_CONSTRAINED_INTRA_PRED], &op_constrained_intra_pred,
        "constrained intra pred"
    },
    {
        EVC_ARGS_NO_KEY,  "dbfoffsetA", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_DBFOFFSET], &op_deblock_alpha_offset,
        "AVC Deblocking filter offset for alpha"
    },
    {
        EVC_ARGS_NO_KEY,  "dbfoffsetB", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TOOL_DBFOFFSET], &op_deblock_beta_offset,
        "AVC Deblocking filter offset for beta"
    },
#if EVC_TILE_SUPPORT
    {
        EVC_ARGS_NO_KEY,  "tile_uniform_spacing", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_TILE_UNIFORM_SPACING], &op_tile_uniform_spacing,
        "uniform or non-uniform tile spacing"
    },
    {
        EVC_ARGS_NO_KEY,  "num_tile_columns_minus1", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_NUM_TILE_COLUMNS_MINUS1], &op_num_tile_columns_minus1,
        "Number of tile columns minus 1"
    },
    {
        EVC_ARGS_NO_KEY,  "num_tile_rows_minus1", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_NUM_TILE_ROWS_MINUS1], &op_num_tile_rows_minus1,
        "Number of tile rows minus 1"
    },
    {
        EVC_ARGS_NO_KEY,  "tile_column_width_array", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_TILE_COLUMN_WIDTH_ARRAY], &op_tile_column_width_array,
        "Array of Tile Column Width"
    },
    {
        EVC_ARGS_NO_KEY,  "tile_row_height_array", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_TILE_ROW_HEIGHT_ARRAY], &op_tile_row_height_array,
        "Array of Tile Row Height"
    },
    {
        EVC_ARGS_NO_KEY,  "num_slices_in_pic_minus1", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_NUM_SLICE_IN_PIC_MINUS1], &op_num_slice_in_pic_minus1,
        "Number of slices in the pic minus 1"
    },
    {
        EVC_ARGS_NO_KEY,  "slices_boundary_array", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_SLICE_BOUNDARY_ARRAY], &op_slice_boundary_array,
        "Array of Slice Boundaries"
    },
#endif
#if CHROMA_QP_TABLE_SUPPORT_M50663
    {
        EVC_ARGS_NO_KEY,  "chroma_qp_table_present_flag", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_CHROMA_QP_TABLE_PRESENT_FLAG], &op_chroma_qp_table_present_flag,
        "chroma_qp_table_present_flag"
    },
    {
        EVC_ARGS_NO_KEY,  "chroma_qp_num_points_in_table", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_CHROMA_QP_NUM_POINTS_IN_TABLE], &op_chroma_qp_num_points_in_table,
        "Number of pivot points for Cb and Cr channels"
    },
    {
        EVC_ARGS_NO_KEY,  "chroma_qp_delta_in_val_cb", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_CHROMA_QP_DELTA_IN_VAL_CB], &op_chroma_qp_delta_in_val_cb,
        "Array of input pivot points for Cb"
    },
    {
        EVC_ARGS_NO_KEY,  "chroma_qp_delta_out_val_cb", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_CHROMA_QP_DELTA_OUT_VAL_CB], &op_chroma_qp_delta_out_val_cb,
        "Array of input pivot points for Cb"
    },
    {
        EVC_ARGS_NO_KEY,  "chroma_qp_delta_in_val_cr", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_CHROMA_QP_DELTA_IN_VAL_CR], &op_chroma_qp_delta_in_val_cr,
        "Array of input pivot points for Cr"
    },
    {
        EVC_ARGS_NO_KEY,  "chroma_qp_delta_out_val_cr", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_CHROMA_QP_DELTA_OUT_VAL_CR], &op_chroma_qp_delta_out_val_cr,
        "Array of input pivot points for Cr"
    },
#endif
    {
        EVC_ARGS_NO_KEY,  "RPL0_0", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_0], &op_rpl0[0],
        "RPL0_0"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_1", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_1], &op_rpl0[1],
        "RPL0_1"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_2", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_2], &op_rpl0[2],
        "RPL0_2"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_3", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_3], &op_rpl0[3],
        "RPL0_3"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_4", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_4], &op_rpl0[4],
        "RPL0_4"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_5", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_5], &op_rpl0[5],
        "RPL0_5"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_6", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_6], &op_rpl0[6],
        "RPL0_6"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_7", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_7], &op_rpl0[7],
        "RPL0_7"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_8", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_8], &op_rpl0[8],
        "RPL0_8"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_9", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_9], &op_rpl0[9],
        "RPL0_9"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_10", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_10], &op_rpl0[10],
        "RPL0_10"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_11", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_11], &op_rpl0[11],
        "RPL0_11"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_12", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_12], &op_rpl0[12],
        "RPL0_12"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_13", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_13], &op_rpl0[13],
        "RPL0_13"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_14", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_14], &op_rpl0[14],
        "RPL0_14"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_15", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_15], &op_rpl0[15],
        "RPL0_15"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_16", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_16], &op_rpl0[16],
        "RPL0_16"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_17", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_17], &op_rpl0[17],
        "RPL0_17"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_18", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_18], &op_rpl0[18],
        "RPL0_18"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_19", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_19], &op_rpl0[19],
        "RPL0_19"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL0_20", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL0_20], &op_rpl0[20],
        "RPL0_20"
    },

    {
        EVC_ARGS_NO_KEY,  "RPL1_0", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_0], &op_rpl1[0],
        "RPL1_0"
    },

    {
        EVC_ARGS_NO_KEY,  "RPL1_1", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_1], &op_rpl1[1],
        "RPL1_1"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_2", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_2], &op_rpl1[2],
        "RPL1_2"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_3", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_3], &op_rpl1[3],
        "RPL1_3"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_4", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_4], &op_rpl1[4],
        "RPL1_4"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_5", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_5], &op_rpl1[5],
        "RPL1_5"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_6", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_6], &op_rpl1[6],
        "RPL1_6"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_7", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_7], &op_rpl1[7],
        "RPL1_7"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_8", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_8], &op_rpl1[8],
        "RPL1_8"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_9", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_9], &op_rpl1[9],
        "RPL1_9"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_10", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_10], &op_rpl1[10],
        "RPL1_10"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_11", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_11], &op_rpl1[11],
        "RPL1_11"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_12", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_12], &op_rpl1[12],
        "RPL1_12"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_13", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_13], &op_rpl1[13],
        "RPL1_13"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_14", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_14], &op_rpl1[14],
        "RPL1_14"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_15", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_15], &op_rpl1[15],
        "RPL1_15"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_16", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_16], &op_rpl1[16],
        "RPL1_16"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_17", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_17], &op_rpl1[17],
        "RPL1_17"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_18", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_18], &op_rpl1[18],
        "RPL1_18"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_19", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_19], &op_rpl1[19],
        "RPL1_19"
    },
    {
        EVC_ARGS_NO_KEY,  "RPL1_20", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_RPL1_20], &op_rpl1[20],
        "RPL1_20"
    },


    {0, "", EVC_ARGS_VAL_TYPE_NONE, NULL, NULL, ""} /* termination */
};

#define NUM_ARG_OPTION   ((int)(sizeof(options)/sizeof(options[0]))-1)
static void print_usage(void)
{
    int i;
    char str[1024];

    v0print("< Usage >\n");

    for(i=0; i<NUM_ARG_OPTION; i++)
    {
        if(evc_args_get_help(options, i, str) < 0) return;
        v0print("%s\n", str);
    }
}

static char get_pic_type(char * in)
{
    int len = (int)strlen(in);
    char type = 0;
    for (int i = 0; i < len; i++)
    {
        if (in[i] == 'P')
        {
            type = 'P';
            break;
        }
        else if (in[i] == 'B')
        {
            type = 'B';
            break;
        }
    }

    if (type == 0)
    {
        return 0;
    }

    return type;
}

#if CHROMA_QP_TABLE_SUPPORT_M50663
static void evca_parse_chroma_qp_mapping_params(EVC_CHROMA_TABLE *dst_struct, EVC_CHROMA_TABLE *src_struct)
{
    int qpBdOffsetC = 6 * (BIT_DEPTH - 8);
    EVC_CHROMA_TABLE *p_chroma_qp_table = dst_struct;
    p_chroma_qp_table->chroma_qp_table_present_flag = src_struct->chroma_qp_table_present_flag;
    p_chroma_qp_table->num_points_in_qp_table_minus1[0] = src_struct->num_points_in_qp_table_minus1[0];
    p_chroma_qp_table->num_points_in_qp_table_minus1[1] = src_struct->num_points_in_qp_table_minus1[1];

    if (p_chroma_qp_table->chroma_qp_table_present_flag)
    {
        p_chroma_qp_table->same_qp_table_for_chroma = 1;
        if (src_struct->num_points_in_qp_table_minus1[0] != src_struct->num_points_in_qp_table_minus1[1])
            p_chroma_qp_table->same_qp_table_for_chroma = 0;
        else
        {
            for (int i = 0; i < src_struct->num_points_in_qp_table_minus1[0]; i++)
            {
                if ( (src_struct->delta_qp_in_val_minus1[0][i] != src_struct->delta_qp_in_val_minus1[1][i]) ||
                    (src_struct->delta_qp_out_val[0][i] != src_struct->delta_qp_out_val[1][i]) )
                {
                    p_chroma_qp_table->same_qp_table_for_chroma = 0;
                    break;
                }
            }
        }

        p_chroma_qp_table->global_offset_flag = (src_struct->delta_qp_in_val_minus1[0][0] > 15 && src_struct->delta_qp_out_val[0][0] > 15) ? 1 : 0;
        if (!p_chroma_qp_table->same_qp_table_for_chroma)
        {
            p_chroma_qp_table->global_offset_flag = p_chroma_qp_table->global_offset_flag && ((src_struct->delta_qp_in_val_minus1[1][0] > 15 && src_struct->delta_qp_out_val[1][0] > 15) ? 1 : 0);
        }

        int startQp = (p_chroma_qp_table->global_offset_flag == 1) ? 16 : -qpBdOffsetC;
        for (int ch = 0; ch < (p_chroma_qp_table->same_qp_table_for_chroma ? 1 : 2); ch++) {
            p_chroma_qp_table->delta_qp_in_val_minus1[ch][0] = src_struct->delta_qp_in_val_minus1[ch][0] - startQp;
            p_chroma_qp_table->delta_qp_out_val[ch][0] = src_struct->delta_qp_out_val[ch][0] - startQp - p_chroma_qp_table->delta_qp_in_val_minus1[ch][0];

            for (int k = 1; k <= p_chroma_qp_table->num_points_in_qp_table_minus1[ch]; k++)
            {
                p_chroma_qp_table->delta_qp_in_val_minus1[ch][k] = (src_struct->delta_qp_in_val_minus1[ch][k] - src_struct->delta_qp_in_val_minus1[ch][k - 1]) - 1;
                p_chroma_qp_table->delta_qp_out_val[ch][k] = (src_struct->delta_qp_out_val[ch][k] - src_struct->delta_qp_out_val[ch][k - 1]) - (p_chroma_qp_table->delta_qp_in_val_minus1[ch][k] + 1);
            }
        }
    }
}
#endif

static int get_conf(EVCE_CDSC * cdsc)
{
    memset(cdsc, 0, sizeof(EVCE_CDSC));

    cdsc->w = op_w;
    cdsc->h = op_h;
    cdsc->qp = op_qp;
    cdsc->fps = op_fps;
    cdsc->iperiod = op_iperiod;
    cdsc->max_b_frames = op_max_b_frames;
    cdsc->profile = op_profile;
    cdsc->level = op_level;
    cdsc->btt = op_btt;
    cdsc->suco = op_suco;
#if DQP_CFG
    cdsc->use_dqp = op_use_dqp;
    cdsc->cu_qp_delta_area = op_cu_qp_delta_area;
#endif
    cdsc->ref_pic_gap_length = op_ref_pic_gap_length;
    cdsc->add_qp_frame = op_add_qp_frames;

    if(op_disable_hgop)
    {
        cdsc->disable_hgop = 1;
    }
    if(op_closed_gop)
    {
        cdsc->closed_gop = 1;
    }
#if IBC
    if (op_enable_ibc)
    {
      cdsc->ibc_flag = 1;
    }

    if (cdsc->ibc_flag)
    {
      cdsc->ibc_search_range_x = op_ibc_search_range_x;
      cdsc->ibc_search_range_y = op_ibc_search_range_y;
      cdsc->ibc_hash_search_flag = op_ibc_hash_search_flag;
      cdsc->ibc_hash_search_max_cand = op_ibc_hash_search_max_cand;
      cdsc->ibc_hash_search_range_4smallblk = op_ibc_hash_search_range_4smallblk;
      cdsc->ibc_fast_method = op_ibc_fast_method;
    }
#endif
    cdsc->in_bit_depth = op_in_bit_depth;

    if(op_out_bit_depth == 0)
    {
        op_out_bit_depth = op_in_bit_depth;
    }
    cdsc->out_bit_depth      = op_out_bit_depth;
    cdsc->framework_ctu_size = op_framework_ctu_size;
    cdsc->framework_cu11_max = op_framework_cu11_max;
    cdsc->framework_cu11_min = op_framework_cu11_min;
    cdsc->framework_cu12_max = op_framework_cu12_max;
    cdsc->framework_cu12_min = op_framework_cu12_min;
    cdsc->framework_cu14_max = op_framework_cu14_max;
    cdsc->framework_cu14_min = op_framework_cu14_min;
    cdsc->framework_tris_max = op_framework_tris_max;
    cdsc->framework_tris_min = op_framework_tris_min;
    cdsc->framework_suco_max = op_framework_suco_max;
    cdsc->framework_suco_min = op_framework_suco_min;
    cdsc->tool_amvr          = op_tool_amvr;
    cdsc->tool_mmvd          = op_tool_mmvd;
    cdsc->tool_affine        = op_tool_affine;
    cdsc->tool_dmvr          = op_tool_dmvr;
    cdsc->tool_alf           = op_tool_alf;
    cdsc->tool_admvp         = op_tool_admvp;
    cdsc->tool_amis          = op_tool_amis;
    cdsc->tool_htdf          = op_tool_htdf;
    cdsc->tool_eipd          = op_tool_eipd;
    cdsc->tool_iqt           = op_tool_iqt;
    cdsc->tool_cm_init       = op_tool_cm_init;
    cdsc->tool_adcc          = op_tool_adcc;
    cdsc->cb_qp_offset       = op_cb_qp_offset;
    cdsc->cr_qp_offset       = op_cr_qp_offset;
#if ATS_INTER_PROCESS
    cdsc->tool_ats           = op_tool_ats;
#endif
    cdsc->constrained_intra_pred = op_constrained_intra_pred;
    cdsc->deblock_aplha_offset = op_deblock_alpha_offset;
    cdsc->deblock_beta_offset = op_deblock_beta_offset;
  
#if EVC_TILE_SUPPORT
    cdsc->tile_uniform_spacing_flag = op_tile_uniform_spacing;
    cdsc->tile_columns = op_num_tile_columns_minus1 + 1;
    cdsc->tile_rows = op_num_tile_rows_minus1 + 1;
    cdsc->num_slice_in_pic = op_num_slice_in_pic_minus1 + 1;

    if (!cdsc->tile_uniform_spacing_flag)
    {
        cdsc->tile_column_width_array[0] = atoi(strtok(op_tile_column_width_array, " "));
        int j = 1;
        do
        {
            char* val = strtok(NULL, " \r");
            if (!val)
                break;
            cdsc->tile_column_width_array[j++] = atoi(val);
        } while (1);

        cdsc->tile_row_height_array[0] = atoi(strtok(op_tile_row_height_array, " "));
        j = 1;
        do
        {
            char* val = strtok(NULL, " \r");
            if (!val)
                break;
            cdsc->tile_row_height_array[j++] = atoi(val);
        } while (1);
    }

    if (cdsc->num_slice_in_pic == 1)
    {
        cdsc->slice_boundary_array[0] = 0;
        cdsc->slice_boundary_array[1] = (cdsc->tile_columns * cdsc->tile_rows) - 1;
    }
    else // There are more than one slice in the picture
    {
        cdsc->slice_boundary_array[0] = atoi(strtok(op_slice_boundary_array, " "));
        int j = 1;
        do
        {
            char* val = strtok(NULL, " \r");
            if (!val)
                break;
            cdsc->slice_boundary_array[j++] = atoi(val);
        } while (1);
    }
#endif
#if CHROMA_QP_TABLE_SUPPORT_M50663
    EVC_CHROMA_TABLE l_chroma_qp_table;
    memset(&l_chroma_qp_table, 0, sizeof(EVC_CHROMA_TABLE));

    l_chroma_qp_table.chroma_qp_table_present_flag = op_chroma_qp_table_present_flag;
    if (l_chroma_qp_table.chroma_qp_table_present_flag)
    {
        l_chroma_qp_table.num_points_in_qp_table_minus1[0] = atoi(strtok(op_chroma_qp_num_points_in_table, " ")) - 1;
        l_chroma_qp_table.num_points_in_qp_table_minus1[1] = atoi( strtok(NULL, " \r") ) - 1;

        { // input pivot points
            l_chroma_qp_table.delta_qp_in_val_minus1[0][0] = atoi(strtok(op_chroma_qp_delta_in_val_cb, " "));
            int j = 1;
            do
            {
                char* val = strtok(NULL, " \r");
                if (!val)
                    break;
                l_chroma_qp_table.delta_qp_in_val_minus1[0][j++] = atoi(val);
            } while (1);
            evc_assert(l_chroma_qp_table.num_points_in_qp_table_minus1[0] + 1 == j);

            l_chroma_qp_table.delta_qp_in_val_minus1[1][0] = atoi(strtok(op_chroma_qp_delta_in_val_cr, " "));
            j = 1;
            do
            {
                char* val = strtok(NULL, " \r");
                if (!val)
                    break;
                l_chroma_qp_table.delta_qp_in_val_minus1[1][j++] = atoi(val);
            } while (1);
            evc_assert(l_chroma_qp_table.num_points_in_qp_table_minus1[1] + 1 == j);
        }
        {// output pivot points
            l_chroma_qp_table.delta_qp_out_val[0][0] = atoi(strtok(op_chroma_qp_delta_out_val_cb, " "));
            int j = 1;
            do
            {
                char* val = strtok(NULL, " \r");
                if (!val)
                    break;
                l_chroma_qp_table.delta_qp_out_val[0][j++] = atoi(val);
            } while (1);
            evc_assert(l_chroma_qp_table.num_points_in_qp_table_minus1[0] + 1 == j);

            l_chroma_qp_table.delta_qp_out_val[1][0] = atoi(strtok(op_chroma_qp_delta_out_val_cr, " "));
            j = 1;
            do
            {
                char* val = strtok(NULL, " \r");
                if (!val)
                    break;
                l_chroma_qp_table.delta_qp_out_val[1][j++] = atoi(val);
            } while (1);
            evc_assert(l_chroma_qp_table.num_points_in_qp_table_minus1[1] + 1 == j);
        }
    }
    evca_parse_chroma_qp_mapping_params(&(cdsc->chroma_qp_table_struct), &l_chroma_qp_table);  // parse input params and create chroma_qp_table_struct structure
#endif
    for (int i = 0; i < MAX_NUM_RPLS && op_rpl0[i][0] != 0; ++i)
    {
        cdsc->rpls_l0[i].pic_type = get_pic_type(strtok(op_rpl0[i], " "));

        cdsc->rpls_l0[i].poc = atoi(strtok(NULL, " "));
        cdsc->rpls_l0[i].tid = atoi(strtok(NULL, " "));
        cdsc->rpls_l0[i].ref_pic_active_num = atoi(strtok(NULL, " "));
        
        int j = 0;
        do
        {
            char* val = strtok(NULL, " \r");
            if (!val)
                break;
            cdsc->rpls_l0[i].ref_pics[j++] = atoi(val);
        } while (1);
   
        cdsc->rpls_l0[i].ref_pic_num = j;
        ++cdsc->rpls_l0_cfg_num;
    }

    for (int i = 0; i < MAX_NUM_RPLS && op_rpl1[i][0] != 0; ++i)
    {
        cdsc->rpls_l0[i].pic_type = get_pic_type(strtok(op_rpl1[i], " "));
        cdsc->rpls_l1[i].poc = atoi(strtok(NULL, " "));
        cdsc->rpls_l1[i].tid = atoi(strtok(NULL, " "));
        cdsc->rpls_l1[i].ref_pic_active_num = atoi(strtok(NULL, " "));

        int j = 0;
        do
        {
            char* val = strtok(NULL, " ");
            if (!val)
                break;
            cdsc->rpls_l1[i].ref_pics[j++] = atoi(val);
        } while (1);

        cdsc->rpls_l1[i].ref_pic_num = j;
        ++cdsc->rpls_l1_cfg_num;
    }

    return 0;
}

#if MULT_CONFIG
static void print_enc_conf(EVCE_CDSC * cdsc)
{
    /*
    TBD(@Chernyak)
    This function is to be modified to down all encoding process related information, not only tools list
    */

    printf("AMVR: %d, ",   cdsc->tool_amvr);
    printf("MMVD: %d, ",   cdsc->tool_mmvd);
    printf("AFFINE: %d, ", cdsc->tool_affine);
    printf("DMVR: %d, ",   cdsc->tool_dmvr);
    printf("ALF: %d, ",    cdsc->tool_alf);
    printf("ADMVP: %d, ",  cdsc->tool_admvp);
    printf("AMIS: %d, ",   cdsc->tool_amis);
    printf("HTDF: %d ",    cdsc->tool_htdf);
    printf("EIPD: %d, ",   cdsc->tool_eipd);
    printf("IQT: %d, ",    cdsc->tool_iqt);
    printf("CM_INIT: %d ", cdsc->tool_cm_init);
    printf("ADCC: %d ",    cdsc->tool_adcc);
#if IBC
    printf("IBC: %d, ", cdsc->ibc_flag);
#endif
    printf("ATS: %d, ",    cdsc->tool_ats);
    printf("CONSTRAINED_INTRA_PRED: %d, ", cdsc->constrained_intra_pred);
    printf("DBFOffsetA: %d, ", cdsc->deblock_aplha_offset);
    printf("DBFOffsetB: %d, ", cdsc->deblock_beta_offset);
#if EVC_TILE_SUPPORT
    printf("Uniform Tile Spacing: %d, ", cdsc->tile_uniform_spacing_flag);
    printf("Number of Tile Columns: %d, ", cdsc->tile_columns);
    printf("Number of Tile  Rows: %d ", cdsc->tile_rows);
#endif
#if CHROMA_QP_TABLE_SUPPORT_M50663
    printf("ChromaQPTable: %d ", cdsc->chroma_qp_table_struct.chroma_qp_table_present_flag);
#endif
    printf("\n");
}
#endif

int check_conf(EVCE_CDSC* cdsc)
{
    int success = 1;
    if(cdsc->profile == PROFILE_BASELINE)
    {
        if (cdsc->tool_amvr    == 1) { v0print("AMVR cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_mmvd    == 1) { v0print("MMVD cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_affine  == 1) { v0print("Affine cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_dmvr    == 1) { v0print("DMVR cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_admvp   == 1) { v0print("ADMVP cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_alf     == 1) { v0print("ALF cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_htdf    == 1) { v0print("HTDF cannot be on in base profile\n"); success = 0; }
        if (cdsc->btt          == 1) { v0print("BTT cannot be on in base profile\n"); success = 0; }
        if (cdsc->suco         == 1) { v0print("SUCO cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_amis    == 1) { v0print("AMIS cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_eipd    == 1) { v0print("EIPD cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_iqt     == 1) { v0print("IQT cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_cm_init == 1) { v0print("CM_INIT cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_adcc    == 1) { v0print("ADCC cannot be on in base profile\n"); success = 0; }
        if (cdsc->tool_ats     == 1) { v0print("ATS_INTRA cannot be on in base profile\n"); success = 0; }
        if (cdsc->ibc_flag     == 1) { v0print("IBC cannot be on in base profile\n"); success = 0; }
    }
    else
    {
        if (cdsc->tool_amis    == 0) { v0print("AMIS cannot be off in main profile\n"); success = 0; }
        if (cdsc->tool_eipd    == 0) { v0print("EIPD cannot be off in main profile\n"); success = 0; }
        if (cdsc->tool_iqt     == 0) { v0print("IQT cannot be off in main profile\n"); success = 0; }
        if (cdsc->tool_cm_init == 0) { v0print("CM_INIT cannot be off in main profile\n"); success = 0; }
    }
    return success;
}

static int set_extra_config(EVCE id)
{
    int  ret, size, value;

    if(op_use_pic_signature)
    {
        value = 1;
        size = 4;
        ret = evce_config(id, EVCE_CFG_SET_USE_PIC_SIGNATURE, &value, &size);
        if(EVC_FAILED(ret))
        {
            v0print("failed to set config for picture signature\n");
            return -1;
        }
    }
    return 0;
}

static void print_stat_init(void)
{
    if(op_verbose < VERBOSE_FRAME) return;

    print("---------------------------------------------------------------------------------------\n");
    print("  Input YUV file          : %s \n", op_fname_inp);
    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        print("  Output EVC bitstream    : %s \n", op_fname_out);
    }
    if(op_flag[OP_FLAG_FNAME_REC])
    {
        print("  Output YUV file         : %s \n", op_fname_rec);
    }
    print("---------------------------------------------------------------------------------------\n");
    print("POC   Tid   Ftype   QP   PSNR-Y    PSNR-U    PSNR-V    Bits      EncT(ms)  ");
    print("MS-SSIM     ");
    print("Ref. List\n");

    print("---------------------------------------------------------------------------------------\n");
}

static void print_config(EVCE id)
{
    int s, v;

    if(op_verbose < VERBOSE_ALL) return;

    print("---------------------------------------------------------------------------------------\n");
    print("< Configurations >\n");
    if(op_flag[OP_FLAG_FNAME_CFG])
    {
    print("\tconfig file name         = %s\n", op_fname_cfg);
    }
    s = sizeof(int);
    evce_config(id, EVCE_CFG_GET_WIDTH, (void *)(&v), &s);
    print("\twidth                    = %d\n", v);
    evce_config(id, EVCE_CFG_GET_HEIGHT, (void *)(&v), &s);
    print("\theight                   = %d\n", v);
    evce_config(id, EVCE_CFG_GET_FPS, (void *)(&v), &s);
    print("\tFPS                      = %d\n", v);
    evce_config(id, EVCE_CFG_GET_I_PERIOD, (void *)(&v), &s);
    print("\tintra picture period     = %d\n", v);
    evce_config(id, EVCE_CFG_GET_QP, (void *)(&v), &s);
    print("\tQP                       = %d\n", v);

    print("\tframes                   = %d\n", op_max_frm_num);

    evce_config(id, EVCE_CFG_GET_USE_DEBLOCK, (void *)(&v), &s);
    print("\tdeblocking filter        = %s\n", v? "enabled": "disabled");

    evce_config(id, EVCE_CFG_GET_CLOSED_GOP, (void *)(&v), &s);
    print("\tGOP type                 = %s\n", v? "closed": "open");

    evce_config(id, EVCE_CFG_GET_HIERARCHICAL_GOP, (void *)(&v), &s);
    print("\thierarchical GOP         = %s\n", v? "enabled": "disabled");
}

static void find_psnr_10bit(EVC_IMGB * org, EVC_IMGB * rec, double psnr[3])
{
    double sum[3], mse[3];
    short *o, *r;
    int i, j, k;

    for(i=0; i<org->np; i++)
    {
        o       = (short*)org->a[i];
        r       = (short*)rec->a[i];
        sum[i] = 0;

        for(j=0; j<org->h[i]; j++)
        {
            for(k=0; k<org->w[i]; k++)
            {
                sum[i] += (o[k] - r[k]) * (o[k] - r[k]);
            }

            o = (short*)((unsigned char *)o + org->s[i]);
            r = (short*)((unsigned char *)r + rec->s[i]);
        }
        mse[i] = sum[i] / (org->w[i] * org->h[i]);
        psnr[i] = (mse[i] == 0.0) ? 100. : fabs(10 * log10(((255 * 255 * 16) / mse[i])));
        // psnr[i] = (mse[i]==0.0) ? 100. : fabs( 10*log10(((1023*1023)/mse[i])) );
    }
}

static void find_psnr_8bit(EVC_IMGB * org, EVC_IMGB * rec, double psnr[3])
{
    double sum[3], mse[3];
    unsigned char *o, *r;
    int i, j, k;

    for(i=0; i<org->np; i++)
    {
        o      = (unsigned char*)org->a[i];
        r      = (unsigned char*)rec->a[i];
        sum[i] = 0;

        for(j=0; j<org->h[i]; j++)
        {
            for(k=0; k<org->w[i]; k++)
            {
                sum[i] += (o[k] - r[k]) * (o[k] - r[k]);
            }

            o += org->s[i];
            r += rec->s[i];
        }
        mse[i] = sum[i] / (org->w[i] * org->h[i]);
        psnr[i] = (mse[i]==0.0) ? 100. : fabs( 10*log10(((255*255)/mse[i])) );
    }
}

const double gaussian_filter[11][11] =
{
    {0.000001,0.000008,0.000037,0.000112,0.000219,0.000274,0.000219,0.000112,0.000037,0.000008,0.000001},
    {0.000008,0.000058,0.000274,0.000831,0.001619,0.002021,0.001619,0.000831,0.000274,0.000058,0.000008},
    {0.000037,0.000274,0.001296,0.003937,0.007668,0.009577,0.007668,0.003937,0.001296,0.000274,0.000037},
    {0.000112,0.000831,0.003937,0.011960,0.023294,0.029091,0.023294,0.011960,0.003937,0.000831,0.000112},
    {0.000219,0.001619,0.007668,0.023294,0.045371,0.056662,0.045371,0.023294,0.007668,0.001619,0.000219},
    {0.000274,0.002021,0.009577,0.029091,0.056662,0.070762,0.056662,0.029091,0.009577,0.002021,0.000274},
    {0.000219,0.001619,0.007668,0.023294,0.045371,0.056662,0.045371,0.023294,0.007668,0.001619,0.000219},
    {0.000112,0.000831,0.003937,0.011960,0.023294,0.029091,0.023294,0.011960,0.003937,0.000831,0.000112},
    {0.000037,0.000274,0.001296,0.003937,0.007668,0.009577,0.007668,0.003937,0.001296,0.000274,0.000037},
    {0.000008,0.000058,0.000274,0.000831,0.001619,0.002021,0.001619,0.000831,0.000274,0.000058,0.000008},
    {0.000001,0.000008,0.000037,0.000112,0.000219,0.000274,0.000219,0.000112,0.000037,0.000008,0.000001}
};

void cal_ssim( int factor, double rad_struct[], int width_s1, int height_s1, short* org_curr_scale, short* rec_curr_scale, int bit_depth )
{
    int width, height, stride;
    const double K1 = 0.01;
    const double K2 = 0.03;
    const int    peak = (1<<bit_depth) - 1; //255 for 8-bit, 1023 for 10-bit
    const double C1 = K1 * K1 * peak * peak;
    const double C2 = K2 * K2 * peak * peak;
    double tmp_luma, tmp_simi, loc_mean_ref, loc_mean_rec, loc_var_ref,
        loc_var_rec, loc_covar, num1, num2, den1, den2;
    int num_win;
    int win_width, win_height;
    short *org, *rec, *org_pel, *rec_pel;
    int i, j, x, y;

    width          = stride = width_s1 / factor;
    height         = height_s1 / factor;
    win_width       = WIN_SIZE<width?  WIN_SIZE:width;
    win_height      = WIN_SIZE<height? WIN_SIZE:height;

    org            = org_curr_scale;
    rec            = rec_curr_scale;
    org_pel         = org;
    rec_pel         = rec;

    num_win         = (height - win_height + 1)*(width - win_width + 1);

    for ( j=0; j<=height-win_height; j++ )            
    {
        for ( i=0; i<=width-win_width; i++ )
        {
            loc_mean_ref = 0;
            loc_mean_rec = 0;
            loc_var_ref  = 0;
            loc_var_rec  = 0;
            loc_covar   = 0;
            org_pel     = org + i + width*j;
            rec_pel     = rec + i + width*j;

            for ( y=0; y<win_height; y++ )    
            {
                for ( x=0; x<win_width; x++ )
                {
                    loc_mean_ref    += org_pel[x]*gaussian_filter[y][x];
                    loc_mean_rec    += rec_pel[x]*gaussian_filter[y][x];

                    loc_var_ref     += org_pel[x]*gaussian_filter[y][x] * org_pel[x];
                    loc_var_rec     += rec_pel[x]*gaussian_filter[y][x] * rec_pel[x];
                    loc_covar      += org_pel[x]*gaussian_filter[y][x] * rec_pel[x];

                }
                org_pel += width;
                rec_pel += width;
            }

            loc_var_ref  =  (loc_var_ref  -  loc_mean_ref * loc_mean_ref);
            loc_var_rec  =  (loc_var_rec  -  loc_mean_rec * loc_mean_rec) ;
            loc_covar   =  (loc_covar   -  loc_mean_ref * loc_mean_rec) ;

            num1        =  2.0 * loc_mean_ref * loc_mean_rec + C1;
            num2        =  2.0 * loc_covar                 + C2;
            den1        =  loc_mean_ref * loc_mean_ref + loc_mean_rec * loc_mean_rec + C1;
            den2        =  loc_var_ref                + loc_var_rec                + C2;

            tmp_luma                        =  num1 / den1;
            tmp_simi                        =  num2 / den2;
            rad_struct[LUMA_COMPONENT]       += tmp_luma;
            rad_struct[SIMILARITY_COMPONENT] += tmp_simi;
            rad_struct[SSIM]                 += tmp_luma * tmp_simi;
        }
    }

    rad_struct[LUMA_COMPONENT]           /= (double) num_win;
    rad_struct[SIMILARITY_COMPONENT]     /= (double) num_win;
    rad_struct[SSIM]                     /= (double) num_win;
}

void calc_ssim_scale( double* ms_ssim, int scale, int width_s1, int height_s1, short* org_last_scale, short* rec_last_scale, int bit_depth) //calc ssim of each scale
{
    int width, height;
    int factor = 1 << ( scale-1 );
    int pos[4];
    int x, y;
    short* org_curr_scale = NULL;
    short* rec_curr_scale = NULL;
    short *o, *r;  //pointers to org and rec
    short *od,*rd; //pointers downsampled org and rec
    double struct_y[3] = {0.0, 0.0, 0.0};
    width  = width_s1 >> (scale-1);
    height = height_s1 >> (scale-1);

    if ( scale == 1 )
    {
        org_curr_scale = org_last_scale;
        rec_curr_scale = rec_last_scale;
    }
    else
    {
        org_curr_scale = (short*)malloc(width*height*sizeof(short));
        rec_curr_scale = (short*)malloc(width*height*sizeof(short));

        o = org_last_scale;
        r = rec_last_scale;
        od= org_curr_scale;
        rd= rec_curr_scale;
        /* Downsample */
        // 2x2 low-pass filter in ms-ssim Matlab code by Wang Zhou.
        for ( y=0; y<height; ++y )
        {
            int offset_last = (y<<1)*width_s1;
            int offset     =  y    *width;
            for ( x=0; x<width; ++x )
            {
                pos[0] = offset_last + (x<<1);
                pos[1] = pos[0] + 1;
                pos[2] = pos[0] + width_s1;
                pos[3] = pos[2] + 1;
                od[offset+x] = (o[pos[0]] + o[pos[1]] + o[pos[2]] + o[pos[3]] + 2) >> 2;
                rd[offset+x] = (r[pos[0]] + r[pos[1]] + r[pos[2]] + r[pos[3]] + 2) >> 2;
            }
        }

        //replace PicLastScale with down-sampled version
        memset(org_last_scale, 0, width_s1*height_s1*sizeof(short));
        memset(rec_last_scale, 0, width_s1*height_s1*sizeof(short));
        for( y=0; y<height; ++y )
        {
            int offset_pic_last_scale = y*width_s1;
            int offset_pic_down      = y*width;
            memcpy(org_last_scale + offset_pic_last_scale, od + offset_pic_down, width*sizeof(short));
            memcpy(rec_last_scale + offset_pic_last_scale, rd + offset_pic_down, width*sizeof(short));
        }
    }

    /* Y */
    cal_ssim( factor, struct_y, width_s1, height_s1, org_curr_scale, rec_curr_scale, bit_depth );
    if ( scale < MAX_SCALE )
        *ms_ssim = struct_y[SIMILARITY_COMPONENT];
    else
        *ms_ssim = struct_y[LUMA_COMPONENT]*struct_y[SIMILARITY_COMPONENT];

    if(scale>1)
    {
        if(org_curr_scale)
            free(org_curr_scale);
        if(rec_curr_scale)
            free(rec_curr_scale);
    }
}

static void find_ms_ssim(EVC_IMGB *org, EVC_IMGB *rec, double* ms_ssim, int bit_depth)
{
    int width, height;
    short* org_last_scale = NULL;
    short* rec_last_scale = NULL;
    int i, j;
    *ms_ssim = 1.0;
    width  = org->w[0];
    height = org->h[0];
    org_last_scale = (short*)malloc(width*height*sizeof(short));
    rec_last_scale = (short*)malloc(width*height*sizeof(short));

    if(bit_depth==8)
    {
        unsigned char* o = (unsigned char*)org->a[0];
        unsigned char* r = (unsigned char*)rec->a[0];
        //copy to PicLastScale as short type
        for(j=0; j<height; j++)
        {
            int offset = j * width;
            for(i=0; i<width; i++)
            {
                org_last_scale[offset+i] = o[offset+i];
                rec_last_scale[offset+i] = r[offset+i];
            }
        }
    }
    else
    {
        short* o = (short*)org->a[0];
        short* r = (short*)rec->a[0];
        short* o_dst = org_last_scale;
        short* r_dst = rec_last_scale;
        for(j=0; j<height; j++)
        {
            memcpy(o_dst, o, width*sizeof(short));
            memcpy(r_dst, r, width*sizeof(short));
            o += org->s[0]>>1; //because s[0] is in unit of byte and 1 short = 2 bytes
            r += rec->s[0]>>1;
            o_dst += width;
            r_dst += width;
        }
    }

    for (i = 1; i <= MAX_SCALE; i++)
    {
        double tmp_ms_ssim;
        calc_ssim_scale( &tmp_ms_ssim, i, width, height, org_last_scale, rec_last_scale, bit_depth );
        *ms_ssim *= pow( tmp_ms_ssim, (double)exponent[i-1] );
    }

    if(org_last_scale)
        free(org_last_scale);
    if(rec_last_scale)
        free(rec_last_scale);
}

static int imgb_list_alloc(IMGB_LIST *list, int w, int h, int bit_depth)
{
    int i;

    memset(list, 0, sizeof(IMGB_LIST)*MAX_BUMP_FRM_CNT);

    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(bit_depth == 10)
        {
            list[i].imgb =  imgb_alloc(w, h, EVC_COLORSPACE_YUV420_10LE);
        }
        else
        {
            list[i].imgb =  imgb_alloc(w, h, EVC_COLORSPACE_YUV420);
        }
        if(list[i].imgb == NULL) goto ERR;
    }
    return 0;

ERR:
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].imgb){ imgb_free(list[i].imgb); list[i].imgb = NULL; }
    }
    return -1;
}

static void imgb_list_free(IMGB_LIST *list)
{
    int i;

    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].imgb){ imgb_free(list[i].imgb); list[i].imgb = NULL; }
    }
}

static IMGB_LIST *imgb_list_put(IMGB_LIST *list, EVC_IMGB *imgb, EVC_MTIME ts)
{
    int i;

    /* store original imgb for PSNR */
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].used == 0)
        {
            imgb_cpy(list[i].imgb, imgb);
            list[i].used = 1;
            list[i].ts = ts;
            return &list[i];
        }
    }
    return NULL;
}

static IMGB_LIST *imgb_list_get_empty(IMGB_LIST *list)
{
    int i;

    /* store original imgb for PSNR */
    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].used == 0)
        {
            return &list[i];
        }
    }
    return NULL;
}

static void imgb_list_make_used(IMGB_LIST *list, EVC_MTIME ts)
{
    list->used = 1;
    list->ts = list->imgb->ts[0] = ts;
}

static int cal_psnr(IMGB_LIST * imgblist_inp, EVC_IMGB * imgb_rec, EVC_MTIME ts, double psnr[3], double* ms_ssim)
{
    int            i;
    EVC_IMGB     *imgb_t = NULL;

    /* calculate PSNR */
    psnr[0] = psnr[1] = psnr[2] = 0;

    for(i = 0; i < MAX_BUMP_FRM_CNT; i++)
    {
        if(imgblist_inp[i].ts == ts && imgblist_inp[i].used == 1)
        {
            if(op_out_bit_depth == op_in_bit_depth)
            {
                if(op_out_bit_depth == 10)
                {
                    find_psnr_10bit(imgblist_inp[i].imgb, imgb_rec, psnr);
                    find_ms_ssim(imgblist_inp[i].imgb, imgb_rec, ms_ssim, 10);
                }
                else /* if(op_out_bit_depth == 8) */
                {
                    find_psnr_8bit(imgblist_inp[i].imgb, imgb_rec, psnr);
                    find_ms_ssim(imgblist_inp[i].imgb, imgb_rec, ms_ssim, 8);
                }
            }
            else if(op_out_bit_depth == 10)
            {
                imgb_t = imgb_alloc(imgb_rec->w[0], imgb_rec->h[0],
                    EVC_COLORSPACE_YUV420_10LE);
                imgb_cpy(imgb_t, imgblist_inp[i].imgb);

                find_psnr_10bit(imgb_t, imgb_rec, psnr);
                find_ms_ssim(imgb_t, imgb_rec, ms_ssim, 10);
                imgb_free(imgb_t);
            }
            else /* if(op_out_bit_depth == 8) */
            {
                imgb_t = imgb_alloc(imgb_rec->w[0], imgb_rec->h[0],
                    EVC_COLORSPACE_YUV420);
                imgb_cpy(imgb_t, imgblist_inp[i].imgb);
                find_psnr_8bit(imgb_t, imgb_rec, psnr);
                find_ms_ssim(imgb_t, imgb_rec, ms_ssim, 8);
                imgb_free(imgb_t);
            }
            imgblist_inp[i].used = 0;
            return 0;
        }
    }
    return -1;
}

static int write_rec(IMGB_LIST *list, EVC_MTIME *ts)
{
    int i;

    for(i=0; i<MAX_BUMP_FRM_CNT; i++)
    {
        if(list[i].ts == (*ts) && list[i].used == 1)
        {
            if(op_flag[OP_FLAG_FNAME_REC])
            {
                if(imgb_write(op_fname_rec, list[i].imgb))
                {
                    v0print("cannot write reconstruction image\n");
                    return -1;
                }
            }
            list[i].used = 0;
            (*ts)++;
            break;
        }
    }
    return 0;
}

void print_psnr(EVCE_STAT * stat, double * psnr, double ms_ssim, int bitrate, EVC_CLK clk_end)
{
    char  stype;
    int i, j;
    switch(stat->stype)
    {
    case EVC_ST_I :
        stype = 'I';
        break;

    case EVC_ST_P :
        stype = 'P';
        break;

    case EVC_ST_B :
        stype = 'B';
        break;

    case EVC_ST_UNKNOWN :
    default :
        stype = 'U';
        break;
    }

    v1print("%-7d%-5d(%c)     %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d%-12.7f", \
        stat->poc, stat->tid, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
        bitrate, evc_clk_msec(clk_end), ms_ssim);
    for(i=0; i < 2; i++)
    {
        v1print("[L%d ", i);
        for(j=0; j < stat->refpic_num[i]; j++) v1print("%d ",stat->refpic[i][j]);
        v1print("] ");
    }
    v1print("\n");
}

int setup_bumping(EVCE id)
{
    int val, size;

    v1print("Entering bumping process... \n");
    val  = 1;
    size = sizeof(int);
    if(EVC_FAILED(evce_config(id, EVCE_CFG_SET_FORCE_OUT, (void *)(&val),
        &size)))
    {
        v0print("failed to fource output\n");
        return -1;
    }
    return 0;
}

int main(int argc, const char **argv)
{
    STATES              state = STATE_ENCODING;
    unsigned char      *bs_buf = NULL;
    FILE               *fp_inp = NULL;
    EVCE               id;
    EVCE_CDSC          cdsc;
    EVC_BITB           bitb;
    EVC_IMGB          *imgb_enc = NULL;
    EVC_IMGB          *imgb_rec = NULL;
    EVCE_STAT          stat;
    int                 i, ret, size;
    EVC_CLK            clk_beg, clk_end, clk_tot;
    EVC_MTIME          pic_icnt, pic_ocnt, pic_skip;
    double              bitrate;
    double              psnr[3] = {0,};
    double              psnr_avg[3] = {0,};
    double              ms_ssim = 0;
    double              ms_ssim_avg = 0;
    IMGB_LIST           ilist_org[MAX_BUMP_FRM_CNT];
    IMGB_LIST           ilist_rec[MAX_BUMP_FRM_CNT];
    IMGB_LIST          *ilist_t = NULL;
    static int          is_first_enc = 1;

    /* parse options */
    ret = evc_args_parse_all(argc, argv, options);
    if(ret != 0)
    {
        if(ret > 0) v0print("-%c argument should be set\n", ret);
        if(ret < 0) v0print("config error\n");
        print_usage();
        return -1;
    }

    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        /* bitstream file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_out, "wb");
        if(fp == NULL)
        {
            v0print("cannot open bitstream file (%s)\n", op_fname_out);
            return -1;
        }
        fclose(fp);
    }

    if(op_flag[OP_FLAG_FNAME_REC])
    {
        /* reconstruction file - remove contents and close */
        FILE * fp;
        fp = fopen(op_fname_rec, "wb");
        if(fp == NULL)
        {
            v0print("cannot open reconstruction file (%s)\n", op_fname_rec);
            return -1;
        }
        fclose(fp);
    }

    /* open original file */
    fp_inp = fopen(op_fname_inp, "rb");
    if(fp_inp == NULL)
    {
        v0print("cannot open original file (%s)\n", op_fname_inp);
        print_usage();
        return -1;
    }

    /* allocate bitstream buffer */
    bs_buf = (unsigned char*)malloc(MAX_BS_BUF);
    if(bs_buf == NULL)
    {
        v0print("cannot allocate bitstream buffer, size=%d", MAX_BS_BUF);
        return -1;
    }

    /* read configurations and set values for create descriptor */
    if(get_conf(&cdsc))
    {
        print_usage();
        return -1;
    }

#if MULT_CONFIG
    print_enc_conf(&cdsc);
#endif

    if (!check_conf(&cdsc))
    {
        v0print("invalid configuration\n");
        return -1;
    }

    /* create encoder */
    id = evce_create(&cdsc, NULL);
    if(id == NULL)
    {
        v0print("cannot create EVC encoder\n");
        return -1;
    }

    if(set_extra_config(id))
    {
        v0print("cannot set extra configurations\n");
        return -1;
    }

    /* create image lists */
    if(imgb_list_alloc(ilist_org, cdsc.w, cdsc.h, op_in_bit_depth))
    {
        v0print("cannot allocate image list for original image\n");
        return -1;
    }
    if(imgb_list_alloc(ilist_rec, cdsc.w, cdsc.h, op_out_bit_depth))
    {
        v0print("cannot allocate image list for reconstructed image\n");
        return -1;
    }

    print_config(id);
    print_stat_init();

    bitrate = 0;

    bitb.addr = bs_buf;
    bitb.bsize = MAX_BS_BUF;

    ret = evce_encode_sps(id, &bitb, &stat);
    if(EVC_FAILED(ret))
    {
        v0print("cannot encode SPS\n");
        return -1;
    }

    if (op_flag[OP_FLAG_FNAME_OUT])
    {
        if (write_data(op_fname_out, bs_buf, stat.write))
        {
            v0print("Cannot write header information (SPS)\n");
            return -1;
        }
    }

    bitrate += stat.write;

    ret = evce_encode_pps(id, &bitb, &stat);
    if (EVC_FAILED(ret))
    {
        v0print("cannot encode PPS\n");
        return -1;
    }

    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        if(write_data(op_fname_out, bs_buf, stat.write))
        {
            v0print("Cannot write header information (SPS)\n");
            return -1;
        }
    }

    bitrate += stat.write;

    if(op_flag[OP_FLAG_SKIP_FRAMES] && op_skip_frames > 0)
    {
        state = STATE_SKIPPING;
    }

    clk_tot = 0;
    pic_icnt = 0;
    pic_ocnt = 0;
    pic_skip = 0;

    /* encode pictures *******************************************************/
    while(1)
    {
        if(state == STATE_SKIPPING)
        {
            if(pic_skip < op_skip_frames)
            {
                ilist_t = imgb_list_get_empty(ilist_org);
                if(ilist_t == NULL)
                {
                    v0print("cannot get empty orignal buffer\n");
                    goto ERR;
                }
                if(imgb_read(fp_inp, ilist_t->imgb))
                {
                    v2print("reached end of original file (or reading error)\n");
                    goto ERR;
                }
            }
            else
            {
                state = STATE_ENCODING;
            }

            pic_skip++;
            continue;
        }

        if(state == STATE_ENCODING)
        {
            ilist_t = imgb_list_get_empty(ilist_org);
            if(ilist_t == NULL)
            {
                v0print("cannot get empty orignal buffer\n");
                return -1;
            }

            /* read original image */
            if (pic_icnt >= op_max_frm_num ||imgb_read(fp_inp, ilist_t->imgb))
            {
                v2print("reached end of original file (or reading error)\n");
                state = STATE_BUMPING;
                setup_bumping(id);
                continue;
            }
            imgb_list_make_used(ilist_t, pic_icnt);

            /* get encodng buffer */
            if(EVC_OK != evce_get_inbuf(id, &imgb_enc))
            {
                v0print("Cannot get original image buffer\n");
                return -1;
            }
            /* copy original image to encoding buffer */
            imgb_cpy(imgb_enc, ilist_t->imgb);

            /* push image to encoder */
            ret = evce_push(id, imgb_enc);
            if(EVC_FAILED(ret))
            {
                v0print("evce_push() failed\n");
                return -1;
            }
            /* release encoding buffer */
            imgb_enc->release(imgb_enc);
            pic_icnt++;
        }

        /* encoding */
        clk_beg = evc_clk_get();

        ret = evce_encode(id, &bitb, &stat);
        if(EVC_FAILED(ret))
        {
            v0print("evce_encode() failed\n");
            return -1;
        }

        clk_end = evc_clk_from(clk_beg);
        clk_tot += clk_end;

        /* store bitstream */
        if (ret == EVC_OK_OUT_NOT_AVAILABLE)
        {
            //v1print("--> RETURN OK BUT PICTURE IS NOT AVAILABLE YET\n");
            continue;
        }
        else if(ret == EVC_OK)
        {
            if(op_flag[OP_FLAG_FNAME_OUT] && stat.write > 0)
            {
                if(write_data(op_fname_out, bs_buf, stat.write))
                {
                    v0print("cannot write bitstream\n");
                    return -1;
                }
            }

            /* get reconstructed image */
            size = sizeof(EVC_IMGB**);
            ret = evce_config(id, EVCE_CFG_GET_RECON, (void *)&imgb_rec, &size);
            if(EVC_FAILED(ret))
            {
                v0print("failed to get reconstruction image\n");
                return -1;
            }

            /* store reconstructed image to list */
            ilist_t = imgb_list_put(ilist_rec, imgb_rec, imgb_rec->ts[0]);
            if(ilist_t == NULL)
            {
                v0print("cannot put reconstructed image to list\n");
                return -1;
            }

            /* calculate PSNR */
            if(cal_psnr(ilist_org, ilist_t->imgb, ilist_t->ts, psnr, &ms_ssim))
            {
                v0print("cannot calculate PSNR\n");
                return -1;
            }
            /* store reconstructed image */
            if(write_rec(ilist_rec, &pic_ocnt))
            {
                v0print("cannot write reconstruction image\n");
                return -1;
            }

            if(is_first_enc)
            {
                print_psnr(&stat, psnr, ms_ssim, (stat.write - stat.sei_size + (int)bitrate) << 3, clk_end);
                is_first_enc = 0;
            }
            else
            {
                print_psnr(&stat, psnr, ms_ssim, (stat.write - stat.sei_size) << 3, clk_end);
            }

            bitrate += (stat.write - stat.sei_size);
            for(i=0; i<3; i++) psnr_avg[i] += psnr[i];
            {
                ms_ssim_avg += ms_ssim;
            }
            /* release recon buffer */
            if (imgb_rec)
            {
                imgb_rec->release(imgb_rec);
            }
        }
        else if (ret == EVC_OK_NO_MORE_FRM)
        {
            break;
        }
        else
        {
            v2print("invaild return value (%d)\n", ret);
            return -1;
        }

        if(op_flag[OP_FLAG_MAX_FRM_NUM] && pic_icnt >= op_max_frm_num
            && state == STATE_ENCODING)
        {
            state = STATE_BUMPING;
            setup_bumping(id);
        }
    }

    /* store remained reconstructed pictures in output list */
    while(pic_icnt - pic_ocnt > 0)
    {
        write_rec(ilist_rec, &pic_ocnt);
    }
    if(pic_icnt != pic_ocnt)
    {
        v2print("number of input(=%d) and output(=%d) is not matched\n",
            (int)pic_icnt, (int)pic_ocnt);
    }

    v1print("=======================================================================================\n");
    psnr_avg[0] /= pic_ocnt;
    psnr_avg[1] /= pic_ocnt;
    psnr_avg[2] /= pic_ocnt;
    ms_ssim_avg  /= pic_ocnt;

    v1print("  PSNR Y(dB)       : %-5.4f\n", psnr_avg[0]);
    v1print("  PSNR U(dB)       : %-5.4f\n", psnr_avg[1]);
    v1print("  PSNR V(dB)       : %-5.4f\n", psnr_avg[2]);
    v1print("  MsSSIM_Y         : %-8.7f\n", ms_ssim_avg);
    v1print("  Total bits(bits) : %-.0f\n", bitrate*8);
    bitrate *= (cdsc.fps * 8);
    bitrate /= pic_ocnt;
    bitrate /= 1000;
    v1print("  bitrate(kbps)    : %-5.4f\n", bitrate);

#if SCRIPT_REPORT
    v1print("  Labeles:\t: br,kbps\tPSNR,Y\tPSNR,U\tPSNR,V\t\n");
    v1print("  Summary\t: %-5.4f\t%-5.4f\t%-5.4f\t%-5.4f\n", bitrate, psnr_avg[0], psnr_avg[1], psnr_avg[2]);
#endif
    v1print("=======================================================================================\n");
    v1print("Encoded frame count               = %d\n", (int)pic_ocnt);
    v1print("Total encoding time               = %.3f msec,",
        (float)evc_clk_msec(clk_tot));
    v1print(" %.3f sec\n", (float)(evc_clk_msec(clk_tot)/1000.0));

    v1print("Average encoding time for a frame = %.3f msec\n",
        (float)evc_clk_msec(clk_tot)/pic_ocnt);
    v1print("Average encoding speed            = %.3f frames/sec\n",
        ((float)pic_ocnt * 1000) / ((float)evc_clk_msec(clk_tot)));
    v1print("=======================================================================================\n");

    if (pic_ocnt != op_max_frm_num)
    {
        v2print("Wrong frames count: should be %d was %d\n", op_max_frm_num, (int)pic_ocnt);
    }

ERR:
    evce_delete(id);

    imgb_list_free(ilist_org);
    imgb_list_free(ilist_rec);

    if(fp_inp) fclose(fp_inp);
    if(bs_buf) free(bs_buf); /* release bitstream buffer */
    return 0;
}
