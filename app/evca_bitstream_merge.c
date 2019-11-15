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

#include "../src/evcd_def.h"
#include "evca_util.h"
#include "evca_args.h"

#define VERBOSE_NONE               VERBOSE_0
#define VERBOSE_FRAME              VERBOSE_1
#define VERBOSE_ALL                VERBOSE_2

#define MAX_BS_BUF                 16*1024*1024 /* byte */

static char op_fname_inp[256] = "\0";
static char op_fname_out[256] = "\0";
static int  op_max_frm_num = 0;
static int  op_use_pic_signature = 1;
static int  op_out_bit_depth = 8;

typedef enum _STATES
{
    STATE_DECODING,
    STATE_BUMPING
} STATES;

typedef enum _OP_FLAGS
{
    OP_FLAG_FNAME_INP,
    OP_FLAG_FNAME_OUT,
    OP_FLAG_MAX_FRM_NUM,
    OP_FLAG_USE_PIC_SIGN,
    OP_FLAG_OUT_BIT_DEPTH,
    OP_FLAG_VERBOSE,
    OP_FLAG_MAX
} OP_FLAGS;

static int op_flag[OP_FLAG_MAX] = { 0 };

static EVC_ARGS_OPTION options[] = \
{
    {
        'i', "input", EVC_ARGS_VAL_TYPE_STRING | EVC_ARGS_VAL_TYPE_MANDATORY,
            &op_flag[OP_FLAG_FNAME_INP], op_fname_inp,
            "file name of input bitstream"
    },
    {
        'o', "output", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_OUT], op_fname_out,
        "file name of decoded output"
    },
    {
        'f', "frames", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_FRM_NUM], &op_max_frm_num,
        "maximum number of frames to be decoded"
    },
    {
        's', "signature", EVC_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_USE_PIC_SIGN], &op_use_pic_signature,
        "conformance check using picture signature (HASH)"
    },
    {
        'v', "verbose", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_VERBOSE], &op_verbose,
        "verbose level\n"
        "\t 0: no message\n"
        "\t 1: frame-level messages (default)\n"
        "\t 2: all messages\n"
    },
    {
        EVC_ARGS_NO_KEY, "output_bit_depth", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_OUT_BIT_DEPTH], &op_out_bit_depth,
        "output bitdepth (8(default), 10) "
    },
    { 0, "", EVC_ARGS_VAL_TYPE_NONE, NULL, NULL, "" } /* termination */
};

#define NUM_ARG_OPTION   ((int)(sizeof(options)/sizeof(options[0]))-1)
static void print_usage(void)
{
    int i;
    char str[1024];

    v0print("< Usage >\n");

    for (i = 0; i<NUM_ARG_OPTION; i++)
    {
        if (evc_args_get_help(options, i, str) < 0) return;
        v0print("%s\n", str);
    }
}

static int read_nalu(FILE * fp, int * pos, unsigned char * bs_buf)
{
    int read_size, bs_size;
    unsigned char b = 0;

    bs_size = 0;
    read_size = 0;

    if (!fseek(fp, *pos, SEEK_SET))
    {
        /* read size first */
        if (4 == fread(&bs_size, 1, 4, fp)) //TBC(@Chernyak): is it ok from endianness perspective?
        {
            if (bs_size <= 0)
            {
                v0print("Invalid bitstream size![%d]\n", bs_size);
                return -1;
            }

            while (bs_size)
            {
                /* read byte */
                if (1 != fread(&b, 1, 1, fp))
                {
                    v0print("Cannot read bitstream!\n");
                    return -1;
                }
                bs_buf[read_size] = b;
                read_size++;
                bs_size--;
            }
        }
        else
        {
            if (feof(fp)) { v2print("End of file\n"); }
            else { v0print("Cannot read bitstream size!\n") };

            return -1;
        }
    }
    else
    {
        v0print("Cannot seek bitstream!\n");
        return -1;
    }

    return read_size;
}

int print_stat(EVCD_STAT * stat, int ret)
{
    char stype;
    int i, j;

    if (EVC_SUCCEEDED(ret))
    {
        if (stat->nalu_type < EVC_SPS_NUT)
        {
            switch (stat->stype)
            {
            case EVC_ST_I:
                stype = 'I';
                break;

            case EVC_ST_P:
                stype = 'P';
                break;

            case EVC_ST_B:
                stype = 'B';
                break;

            case EVC_ST_UNKNOWN:
            default:
                stype = 'U';
                break;
            }
            v1print("%c-slice", stype);
        }
        else if (stat->nalu_type == EVC_SPS_NUT)
        {
            v1print("Sequence Parameter Set");
        }
        else
        {
            v0print("Unknown bitstream");
        }
        v1print(" (read=%d, poc=%d) ", stat->read, (int)stat->poc);
        for (i = 0; i < 2; i++)
        {
            v1print("[L%d ", i);
            for (j = 0; j < stat->refpic_num[i]; j++) v1print("%d ", stat->refpic[i][j]);
            v1print("] ");
        }

        if (ret == EVC_OK)
        {
            v1print("\n");
        }
        else if (ret == EVC_OK_FRM_DELAYED)
        {
            v1print("->Frame delayed\n");
        }
        else if (ret == EVC_OK_DIM_CHANGED)
        {
            v1print("->Resolution changed\n");
        }
        else
        {
            v1print("->Unknown OK code = %d\n", ret);
        }
    }
    else
    {
        v0print("Decoding error = %d\n", ret);
    }
    return 0;
}

static int set_extra_config(EVCD id)
{
    int  ret, size, value;

    if (op_use_pic_signature)
    {
        value = 1;
        size = 4;
        ret = evcd_config(id, EVCD_CFG_SET_USE_PIC_SIGNATURE, &value, &size);
        if (EVC_FAILED(ret))
        {
            v0print("failed to set config for picture signature\n");
            return -1;
        }
    }
    return 0;
}

/* convert EVCD into EVCD_CTX with return value if assert on */
#define EVCD_ID_TO_CTX_RV(id, ctx, ret) \
    evc_assert_rv((id), (ret)); \
    (ctx) = (EVCD_CTX *)id; \
    evc_assert_rv((ctx)->magic == EVCD_MAGIC_CODE, (ret));

void make_ue_code(u32 val, u32 * code, int * len_c)
{
    int   len_i, info, nn; 

    nn = ((val + 1) >> 1);
    for (len_i = 0; len_i < 16 && nn != 0; len_i++)
    {
        nn >>= 1;
    }

    info = val + 1 - (1 << len_i);
    *code = (1 << len_i) | ((info)& ((1 << len_i) - 1));

    *len_c = (len_i << 1) + 1;    
}

void write_tmp_bs(EVC_SH * sh, int add_dtr, FILE * merge_fp)
{
    int i, idx;
    u8 tmp_bs;
    u16 dtr = sh->dtr + add_dtr;
    u32 code;
    int len_c;

    tmp_bs = 0;
    idx = 7;

    for (i = 0; i < 8; i++)
    {
        tmp_bs |= (((dtr >> (9 - i)) & 1) << idx);
        idx--;
    }

    fwrite(&tmp_bs, 1, 1, merge_fp);

    tmp_bs = 0;
    idx = 7;

    for (i = 8; i < 10; i++)
    {
        tmp_bs |= (((dtr >> (9 - i)) & 1) << idx);
        idx--;
    }

    for (i = 0; i < 3; i++)
    {
        tmp_bs |= (((sh->layer_id >> (2 - i)) & 1) << idx);
        idx--;
    }

    tmp_bs |= ((sh->temporal_mvp_asigned_flag & 1) << idx);
    idx--;

    if (sh->temporal_mvp_asigned_flag)
    {
        tmp_bs |= ((sh->collocated_from_list_idx & 1) << idx);
        idx--;

        tmp_bs |= ((sh->collocated_from_ref_idx & 1) << idx);
        idx--;
    }

    make_ue_code(sh->slice_pic_parameter_set_id, &code, &len_c);

    for (int i = 0; i < len_c && idx >= 0; i++)
    {
        tmp_bs |= (((code >> (len_c - 1 - i)) & 1) << idx);
        idx--;
    }
    //tmp_bs |= ((sh->slice_pic_parameter_set_id & 1) << idx);
    //idx--;

    if (idx >= 0)
    {
        tmp_bs |= ((sh->single_tile_in_slice_flag & 1) << idx);
        idx--;
    }

    fwrite(&tmp_bs, 1, 1, merge_fp);
}

int main(int argc, const char **argv)
{
    unsigned char    * bs_buf = NULL;
    EVCD              id = NULL;
    EVCD_CDSC         cdsc;
    EVC_BITB          bitb;
    /*temporal buffer for video bit depth less than 10bit */
    EVC_IMGB        * imgb_t = NULL;
    int                ret;
    int                bs_size, bs_read_pos = 0;
    FILE             * fp_bs = NULL;
    FILE             * fp_bs_write = NULL;
    int                bs_num, max_bs_num;
    u8                 tmp_size[4];
    int                bs_end_pos;
    int                intra_dist[2];
    int                intra_dist_idx = 0;
    EVC_BSR         * bs;
    EVC_SPS         * sps;
    EVC_PPS         * pps;
    EVC_SH          * sh;
    EVC_NALU        * nalu;
    EVCD_CTX        * ctx;
#if ALF_PARAMETER_APS
    EVC_APS         * aps;
#endif

    // set line buffering (_IOLBF) for stdout to prevent incomplete logs when app crashed.
    setvbuf(stdout, NULL, _IOLBF, 1024);
    
    max_bs_num = argc - 2;
    fp_bs_write = fopen(argv[max_bs_num + 1], "wb");

    for (bs_num = 0; bs_num < max_bs_num; bs_num++)
    {
        fp_bs = fopen(argv[bs_num + 1], "rb");
        if (fp_bs == NULL)
        {
            v0print("ERROR: cannot open bitstream file = %s\n", op_fname_inp);
            print_usage();
            return -1;
        }

        fseek(fp_bs, 0, SEEK_END);
        bs_end_pos = ftell(fp_bs);
        intra_dist_idx = 0;
        if (bs_num) intra_dist[0] += intra_dist[1];
        
        bs_buf = malloc(MAX_BS_BUF);
        if (bs_buf == NULL)
        {
            v0print("ERROR: cannot allocate bit buffer, size=%d\n", MAX_BS_BUF);
            return -1;
        }
        id = evcd_create(&cdsc, NULL);
        if (id == NULL)
        {
            v0print("ERROR: cannot create EVC decoder\n");
            return -1;
        }
        if (set_extra_config(id))
        {
            v0print("ERROR: cannot set extra configurations\n");
            return -1;
        }

        bs_read_pos = 0;

        while (1)
        {           
            bs_size = read_nalu(fp_bs, &bs_read_pos, bs_buf);

            tmp_size[0] = (bs_size & 0x000000ff) >> 0;  //TBD(@Chernyak): is there a better way?
            tmp_size[1] = (bs_size & 0x0000ff00) >> 8;
            tmp_size[2] = (bs_size & 0x00ff0000) >> 16;
            tmp_size[3] = (bs_size & 0xff000000) >> 24;
                                          
            if (bs_size <= 0)
            {

                v1print("bumping process starting...\n");
                    
                continue;
            }
            bs_read_pos += (4 + bs_size);
            bitb.addr = bs_buf;
            bitb.ssize = bs_size;
            bitb.bsize = MAX_BS_BUF;
            EVCD_ID_TO_CTX_RV(id, ctx, EVC_ERR_INVALID_ARGUMENT);

            bs = &ctx->bs;
            sps = &ctx->sps;
            pps = &ctx->pps;
            sh = &ctx->sh;
            nalu = &ctx->nalu;
#if ALF_PARAMETER_APS
            aps = &ctx->aps;
#endif
            /* set error status */
            ctx->bs_err = bitb.err = 0;
            EVC_TRACE_SET(1);
            /* bitstream reader initialization */
            evc_bsr_init(bs, bitb.addr, bitb.ssize, NULL);
            SET_SBAC_DEC(bs, &ctx->sbac_dec);

            /* parse nalu header */
            ret = evcd_eco_nalu(bs, nalu);
            evc_assert_rv(EVC_SUCCEEDED(ret), ret);

            if (nalu->nal_unit_type_plus1 - 1 == EVC_SPS_NUT)
            {
                ret = evcd_eco_sps(bs, sps);
                evc_assert_rv(EVC_SUCCEEDED(ret), ret);
#if ALF
                sh->alf_on = sps->tool_alf;
#endif
                sh->mmvd_group_enable_flag = sps->tool_mmvd;
                if (!bs_num)
                {
                    fwrite(tmp_size, 1, 4, fp_bs_write);
                    fwrite(bs_buf, 1, bs_size, fp_bs_write);
                }
            }
            else if (nalu->nal_unit_type_plus1 - 1 <  EVC_SPS_NUT)
            {
                /* decode slice header */
#if ALF
                sh->num_ctb = ctx->f_lcu;
#endif
#if ALF_CTU_MAP_DYNAMIC
                sh->alf_sh_param.alfCtuEnableFlag = (u8 *)malloc(N_C * ctx->f_lcu * sizeof(u8));
                memset(sh->alf_sh_param.alfCtuEnableFlag, 1, N_C * ctx->f_lcu * sizeof(u8));
#endif
                ret = evcd_eco_sh(bs, &ctx->sps, &ctx->pps, sh);
                evc_assert_rv(EVC_SUCCEEDED(ret), ret);

                if (bs_num == 0 && sh->slice_type == SLICE_I)
                {
                    intra_dist[intra_dist_idx] = sh->dtr;    
                    intra_dist_idx++;
                }

                if (bs_num == 0)
                {
                    fwrite(tmp_size, 1, 4, fp_bs_write);
                    fwrite(bs_buf, 1, bs_size, fp_bs_write);
                }
                else
                {
                    if (!intra_dist_idx && sh->slice_type == SLICE_I)
                    {
                        intra_dist_idx++;
                    }
                    else
                    {
                        /* chnk size    */
                        fwrite(tmp_size, 1, 4, fp_bs_write);
                        /* write nalu  */
                        fwrite(bs_buf, 1, 2, fp_bs_write);
                        /* re-write dtr */
                        write_tmp_bs(sh, intra_dist[0], fp_bs_write);
                        /* other stream */
                        fwrite(bs_buf + 4, 1, bs_size - 4, fp_bs_write);
                    }
                }
            }
            else if (nalu->nal_unit_type_plus1 - 1 == EVC_PPS_NUT)
            {
                if (!bs_num)
                {
                    fwrite(tmp_size, 1, 4, fp_bs_write);
                    fwrite(bs_buf, 1, bs_size, fp_bs_write);
                }
            }
#if ALF_PARAMETER_APS
            else if (nalu->nal_unit_type_plus1 - 1 == EVC_APS_NUT)
            {
                fwrite(tmp_size, 1, 4, fp_bs_write);
                fwrite(bs_buf, 1, bs_size, fp_bs_write);

            }
#endif
            if (bs_read_pos == bs_end_pos)
            {
                break;
            }
        }
        
        if (id) evcd_delete(id);
        if (imgb_t) imgb_free(imgb_t);
        if (fp_bs) fclose(fp_bs);
        if (bs_buf) free(bs_buf);
    }

    if (fp_bs_write) fclose(fp_bs_write);
    return 0;
}
