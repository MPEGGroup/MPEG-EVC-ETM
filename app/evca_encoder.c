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

#include "evc.h"
#include "evca_util.h"
#include "evca_args.h"
#include <math.h>

#if LINUX
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>


void handler(int sig)
{
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}
#endif

#define SCRIPT_REPORT              1
#define MULT_CONFIG                1
#define VERBOSE_NONE               VERBOSE_0
#define VERBOSE_FRAME              VERBOSE_1
#define VERBOSE_ALL                VERBOSE_2
#define MAX_BUMP_FRM_CNT           (8 <<1)

#define MAX_BS_BUF                 (6*1024*1024)
#define PRECISE_BS_SIZE            1

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

#define CALC_SSIM           1    //calculate Multi-scale SSIM (ms_ssim)
#if CALC_SSIM
#define     MAX_SCALE       5    //number of scales
#define     WIN_SIZE        11   //window size for SSIM calculation
static double exponent[MAX_SCALE] = { 0.0448, 0.2856, 0.3001, 0.2363, 0.1333 };  //weights for different scales

enum STRUCTURE
{
    LUMA_COMPONENT,
    SIMILARITY_COMPONENT,
    SSIM,
};
#endif

static char op_fname_cfg[256] = "\0"; /* config file path name */
static char op_fname_inp[256] = "\0"; /* input original video */
static char op_fname_out[256] = "\0"; /* output bitstream */
static char op_fname_rec[256] = "\0"; /* reconstructed video */
static int  op_max_frm_num = 0;
static int  op_use_pic_signature = 1;
static int  op_w = 0;
static int  op_h = 0;
static int  op_qp = 0;
static int  op_fps = 0;
static int  op_iperiod = 0;
static int  op_max_b_frames= 0;
static int  op_closed_gop = 0;
static int  op_disable_hgop = 0;
static int  op_in_bit_depth = 8;
static int  op_skip_frames = 0;
static int  op_out_bit_depth = 0; /* same as input bit depth */
#if USE_TILE_GROUP_DQP
static int  op_add_qp_frames = 0;
#endif
static int  op_qp_offset_cb = 0;
static int  op_qp_offset_cr = 0;
static int  op_framework_ctu_size = 8;
static int  op_framework_cu11_max = 8;
static int  op_framework_cu11_min = 2;
static int  op_framework_cu12_max = 7;
static int  op_framework_cu12_min = 3;
static int  op_framework_cu14_max = 6;
static int  op_framework_cu14_min = 4;
static int  op_framework_tris_max = 6;
static int  op_framework_tris_min = 4;
static int  op_framework_suco_max = 7;
static int  op_framework_suco_min = 4;
static int  op_tool_amvr          = 1; /* default on */
static int  op_tool_mmvd          = 1; /* default on */
static int  op_tool_affine        = 1; /* default on */
static int  op_tool_dmvr          = 1; /* default on */
static int  op_tool_alf           = 1; /* default on */
static int  op_tool_admvp         = 1; /* default on */
static int  op_tool_amis          = 1; /* default on */
static int  op_tool_htdf          = 1; /* default on */

static char  op_rpl0[MAX_NUM_RPLS][256];
static char  op_rpl1[MAX_NUM_RPLS][256];

typedef enum _OP_FLAGS
{
    OP_FLAG_FNAME_CFG,
    OP_FLAG_FNAME_INP,
    OP_FLAG_FNAME_OUT,
    OP_FLAG_FNAME_REC,
    OP_FLAG_WIDTH_INP,
    OP_FLAG_HEIGHT_INP,
    OP_FLAG_QP,
    OP_FLAG_FPS,
    OP_FLAG_IPERIOD,
    OP_FLAG_MAX_FRM_NUM,
    OP_FLAG_USE_PIC_SIGN,
    OP_FLAG_VERBOSE,
    OP_FLAG_MAX_B_FRAMES,
    OP_FLAG_CLOSED_GOP,
    OP_FLAG_DISABLE_HGOP,
    OP_FLAG_OUT_BIT_DEPTH,
    OP_FLAG_IN_BIT_DEPTH,
    OP_FLAG_SKIP_FRAMES,
#if USE_TILE_GROUP_DQP
    OP_FLAG_ADD_QP_FRAME,
#endif
    OP_FLAG_QP_OFFSET_CB,
    OP_FLAG_QP_OFFSET_CR,
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
        EVC_ARGS_NO_KEY,  "closed_gop", EVC_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_CLOSED_GOP], &op_closed_gop,
        "use closed GOP structure. if not set, open GOP is used"
    },
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
#if USE_TILE_GROUP_DQP
    {
        'a',  "qp_add_frm", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_ADD_QP_FRAME], &op_add_qp_frames,
        "one more qp are added after this number of frames, disable:0 (default)"
    },
#endif
    {
        EVC_ARGS_NO_KEY,  "qp_offset_cb", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_CB], &op_qp_offset_cb,
        "qp offset for cb, disable:0 (default)"
    },
    {
        EVC_ARGS_NO_KEY,  "qp_offset_cr", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_QP_OFFSET_CR], &op_qp_offset_cr,
        "qp offset for cr, disable:0 (default)"
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

static int get_conf(EVCE_CDSC * cdsc)
{
    memset(cdsc, 0, sizeof(EVCE_CDSC));

    cdsc->w = op_w;
    cdsc->h = op_h;
    cdsc->qp = op_qp;
    cdsc->fps = op_fps;
    cdsc->iperiod = op_iperiod;
    cdsc->max_b_frames = op_max_b_frames;
#if USE_TILE_GROUP_DQP
    cdsc->add_qp_frame = op_add_qp_frames;
#endif
    if(op_disable_hgop)
    {
        cdsc->disable_hgop = 1;
    }
    if(op_closed_gop)
    {
        cdsc->closed_gop = 1;
    }

    cdsc->in_bit_depth = op_in_bit_depth;

    if(op_out_bit_depth == 0)
    {
        op_out_bit_depth = op_in_bit_depth;
    }
    cdsc->out_bit_depth = op_out_bit_depth;
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

    for (int i = 0; i < MAX_NUM_RPLS && op_rpl0[i][0] != 0; ++i)
    {
        strtok(op_rpl0[i], " ");
        cdsc->rpls_l0[i].poc = atoi(strtok(NULL, " "));
        cdsc->rpls_l0[i].tid = atoi(strtok(NULL, " "));
        cdsc->rpls_l0[i].ref_pic_active_num = atoi(strtok(NULL, " "));
        
        int j = 0;
        do
        {
            char* val = strtok(NULL, " ");
            if (!val)
                break;
            cdsc->rpls_l0[i].ref_pics[j++] = atoi(val);
        } while (1);
   
        cdsc->rpls_l0[i].ref_pic_num = j;
        ++cdsc->rpls_l0_cfg_num;
    }

    for (int i = 0; i < MAX_NUM_RPLS && op_rpl1[i][0] != 0; ++i)
    {
        strtok(op_rpl1[i], " ");
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
static int print_enc_conf(EVCE_CDSC * cdsc)
{
    printf("AMVR: %d, ",   cdsc->tool_amvr);
    printf("MMVD: %d, ",   cdsc->tool_mmvd);
    printf("AFFINE: %d, ", cdsc->tool_affine);
    printf("DMVR: %d, ",   cdsc->tool_dmvr);
    printf("ALF: %d, ",    cdsc->tool_alf);
    printf("ADMVP: %d, ",  cdsc->tool_admvp);
    printf("AMIS: %d, ",   cdsc->tool_amis);
    printf("HTDF: %d ",    cdsc->tool_htdf);
    printf("\n");
    return 0;
}
#endif

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
    print(" POC       QP   PSNR-Y    PSNR-U    PSNR-V    Bits      EncT(ms)  ");
#if CALC_SSIM
    print("MS-SSIM     ");
#endif
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

#if CALC_SSIM
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

    for ( i=1; i<=MAX_SCALE; i++ )
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
#endif

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

static int cal_psnr(IMGB_LIST * imgblist_inp, EVC_IMGB * imgb_rec,
                    EVC_MTIME ts, double psnr[3]
#if CALC_SSIM
                    , double* ms_ssim
#endif
)
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
#if CALC_SSIM
                    find_ms_ssim(imgblist_inp[i].imgb, imgb_rec, ms_ssim, 10);
#endif
                }
                else /* if(op_out_bit_depth == 8) */
                {
                    find_psnr_8bit(imgblist_inp[i].imgb, imgb_rec, psnr);
#if CALC_SSIM
                    find_ms_ssim(imgblist_inp[i].imgb, imgb_rec, ms_ssim, 8);
#endif
                }
            }
            else if(op_out_bit_depth == 10)
            {
                imgb_t = imgb_alloc(imgb_rec->w[0], imgb_rec->h[0],
                    EVC_COLORSPACE_YUV420_10LE);
                imgb_cpy(imgb_t, imgblist_inp[i].imgb);

                find_psnr_10bit(imgb_t, imgb_rec, psnr);
#if CALC_SSIM
                find_ms_ssim(imgb_t, imgb_rec, ms_ssim, 10);
#endif

                imgb_free(imgb_t);
            }
            else /* if(op_out_bit_depth == 8) */
            {
                imgb_t = imgb_alloc(imgb_rec->w[0], imgb_rec->h[0],
                    EVC_COLORSPACE_YUV420);
                imgb_cpy(imgb_t, imgblist_inp[i].imgb);

                find_psnr_8bit(imgb_t, imgb_rec, psnr);
#if CALC_SSIM
                find_ms_ssim(imgb_t, imgb_rec, ms_ssim, 8);
#endif

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

#if CALC_SSIM
void print_psnr(EVCE_STAT * stat, double * psnr, double ms_ssim, int bitrate, EVC_CLK clk_end)
#else
void print_psnr(EVCE_STAT * stat, double * psnr, int bitrate, EVC_CLK clk_end)
#endif
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

#if CALC_SSIM
    v1print("%-7d(%c) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d%-12.7f", \
        stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
        bitrate, evc_clk_msec(clk_end), ms_ssim);
#else
    v1print("%-7d(%c) %-5d%-10.4f%-10.4f%-10.4f%-10d%-10d", \
        stat->poc, stype, stat->qp, psnr[0], psnr[1], psnr[2], \
        bitrate, evc_clk_msec(clk_end));
#endif
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
    int                 udata_size;
    int                 i, ret, size;
    EVC_CLK            clk_beg, clk_end, clk_tot;
    EVC_MTIME          pic_icnt, pic_ocnt, pic_skip;
    double              bitrate;
    double              psnr[3] = {0,};
    double              psnr_avg[3] = {0,};
#if CALC_SSIM
    double              ms_ssim = 0;
    double              ms_ssim_avg = 0;
#endif
    IMGB_LIST           ilist_org[MAX_BUMP_FRM_CNT];
    IMGB_LIST           ilist_rec[MAX_BUMP_FRM_CNT];
    IMGB_LIST          *ilist_t = NULL;
    static int          is_first_enc = 1;
#if !CALC_SSIM
    double              seq_header_bit = 0;
#endif
#if LINUX
    signal(SIGSEGV, handler);   // install our handler
#endif

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

    udata_size = (op_use_pic_signature)? 18: 0;
#if !PRECISE_BS_SIZE
    udata_size += 4; /* 4-byte prefix (length field of chunk) */
#endif

    /* encode Sequence Header if needed **************************************/
    ret = evce_encode_header(id, &bitb, &stat);
    if(EVC_FAILED(ret))
    {
        v0print("cannot encode header \n");
        return -1;
    }
    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        /* write Sequence Header bitstream to file */
        if(write_data(op_fname_out, bs_buf, stat.write))
        {
            v0print("Cannot write header information (SPS)\n");
            return -1;
        }
#if PRECISE_BS_SIZE
        bitrate += stat.write;
#if !CALC_SSIM
        seq_header_bit = stat.write;
#endif
#else
    bitrate += (stat.write - 4)/* 4-byte prefix (length field of chunk) */;
#if !CALC_SSIM
    seq_header_bit = (stat.write - 4)/* 4-byte prefix (length field of chunk) */;
#endif
#endif
    }

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
            if(imgb_read(fp_inp, ilist_t->imgb))
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
#if CALC_SSIM
            if(cal_psnr(ilist_org, ilist_t->imgb, ilist_t->ts, psnr, &ms_ssim))
#else
            if(cal_psnr(ilist_org, ilist_t->imgb, ilist_t->ts, psnr))
#endif
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
#if CALC_SSIM
                print_psnr(&stat, psnr, ms_ssim, (stat.write - udata_size + (int)bitrate) << 3, clk_end);
#else
                print_psnr(&stat, psnr, (stat.write - udata_size + (int)seq_header_bit) << 3, clk_end);
#endif
                is_first_enc = 0;
            }
            else
            {
#if CALC_SSIM
                print_psnr(&stat, psnr, ms_ssim, (stat.write - udata_size) << 3, clk_end);
#else
                print_psnr(&stat, psnr, (stat.write - udata_size) << 3, clk_end);
#endif
            }

            bitrate += (stat.write - udata_size);
            for(i=0; i<3; i++) psnr_avg[i] += psnr[i];
#if CALC_SSIM
            ms_ssim_avg += ms_ssim;
#endif
            /* release recon buffer */
            if(imgb_rec) imgb_rec->release(imgb_rec);
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
#if CALC_SSIM
    ms_ssim_avg  /= pic_ocnt;
#endif

    v1print("  PSNR Y(dB)       : %-5.4f\n", psnr_avg[0]);
    v1print("  PSNR U(dB)       : %-5.4f\n", psnr_avg[1]);
    v1print("  PSNR V(dB)       : %-5.4f\n", psnr_avg[2]);
#if CALC_SSIM
    v1print("  MsSSIM_Y         : %-8.7f\n", ms_ssim_avg);
#endif
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
        ((float)pic_ocnt*1000)/((float)evc_clk_msec(clk_tot)));
    v1print("=======================================================================================\n");

ERR:
     evce_delete(id);

    imgb_list_free(ilist_org);
    imgb_list_free(ilist_rec);

    if(fp_inp) fclose(fp_inp);
    if(bs_buf) free(bs_buf); /* release bitstream buffer */
    return 0;
}
