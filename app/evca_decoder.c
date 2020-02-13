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

#define DECODING_TIME_TEST 1

#include "../src/evc_def.h"
#include "evca_util.h"
#include "evca_args.h"
#include "../src/evc_dra.h"

#define VERBOSE_NONE               VERBOSE_0
#define VERBOSE_FRAME              VERBOSE_1
#define VERBOSE_ALL                VERBOSE_2

#define MAX_BS_BUF                 16*1024*1024 /* byte */

static char op_fname_inp[256] = "\0";
static char op_fname_out[256] = "\0";
static int  op_max_frm_num = 0;
static int  op_use_pic_signature = 0;
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

static int op_flag[OP_FLAG_MAX] = {0};

static EVC_ARGS_OPTION options[] = \
{
    {
        'i', "input", EVC_ARGS_VAL_TYPE_STRING|EVC_ARGS_VAL_TYPE_MANDATORY,
        &op_flag[OP_FLAG_FNAME_INP], op_fname_inp,
        "file name of input bitstream"
    },
    {
        'o', "output", EVC_ARGS_VAL_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_OUT], op_fname_out,
        "file name of decoded output"
    },
    {
        'f',  "frames", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_FRM_NUM], &op_max_frm_num,
        "maximum number of frames to be decoded"
    },
    {
        's',  "signature", EVC_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_USE_PIC_SIGN], &op_use_pic_signature,
        "conformance check using picture signature (HASH)"
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
        EVC_ARGS_NO_KEY,  "output_bit_depth", EVC_ARGS_VAL_TYPE_INTEGER,
        &op_flag[OP_FLAG_OUT_BIT_DEPTH], &op_out_bit_depth,
        "output bitdepth (8(default), 10) "
    },
    { 0, "", EVC_ARGS_VAL_TYPE_NONE, NULL, NULL, ""} /* termination */
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

static int read_nalu(FILE * fp, int * pos, unsigned char * bs_buf)
{
    int read_size, bs_size;
    unsigned char b = 0;

    bs_size = 0;
    read_size = 0;

    if(!fseek(fp, *pos, SEEK_SET))
    {
        /* read size first */
        if(4 == fread(&bs_size, 1, 4, fp)) //@TBC(Chernyak): is it ok from endianness perspective?
        {
            if(bs_size <= 0)
            {
                v0print("Invalid bitstream size![%d]\n", bs_size);
                return -1;
            }

            while(bs_size)
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
            if(feof(fp)) {v2print("End of file\n");}
            else {v0print("Cannot read bitstream size!\n")};

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

static void print_stat(EVCD_STAT * stat, int ret)
{
    int i, j;

    if(EVC_SUCCEEDED(ret))
    {
        if(stat->nalu_type < EVC_SPS_NUT)
        {
            v1print("%c-slice", stat->stype == EVC_ST_I ? 'I' : stat->stype == EVC_ST_P ? 'P' : 'B');

            v1print(" (%d bytes", stat->read);
            v1print(", poc=%d, tid=%d, ", (int)stat->poc, (int)stat->tid);

            for (i = 0; i < 2; i++)
            {
                v1print("[L%d ", i);
                for (j = 0; j < stat->refpic_num[i]; j++) v1print("%d ", stat->refpic[i][j]);
                v1print("] ");
            }
        }
        else if(stat->nalu_type == EVC_SPS_NUT)
        {
            v1print("Sequence Parameter Set (%d bytes)", stat->read);
        }
        else if (stat->nalu_type == EVC_PPS_NUT)
        {
            v1print("Picture Parameter Set (%d bytes)", stat->read);
        }
        else if (stat->nalu_type == EVC_APS_NUT)
        {
            v1print("Adaptation Parameter Set (%d bytes)", stat->read);
        }
        else if (stat->nalu_type == EVC_SEI_NUT)
        {
            v1print("SEI message: ");
            if (ret == EVC_OK)
            {
                v1print("MD5 check OK");
            }
            else if (ret == EVC_ERR_BAD_CRC)
            {
                v1print("MD5 check mismatch!");
            }
            else if (ret == EVC_WARN_CRC_IGNORED)
            {
                v1print("MD5 check ignored!");
            }
        }
        else
        {
            v0print("Unknown bitstream");
        }

        v1print("\n");

        //TBD the following stages need to be further hadnled in the proper places
        /*if(ret == EVC_OK)
        {
            v1print("\n");
        }
        else if(ret == EVC_OK_FRM_DELAYED)
        {
            v1print("--> Frame delayed\n");
        }
        else if(ret == EVC_OK_DIM_CHANGED)
        {
            v1print("--> Resolution changed\n");
        }
        else
        {
            v1print(" --> Unknown warning code = %d\n", ret);
        }*/
    }
    else
    {
        v0print("Decoding error = %d\n", ret);
    }
}

static int set_extra_config(EVCD id)
{
    int  ret, size, value;

    if(op_use_pic_signature)
    {
        value = 1;
        size = 4;
        ret = evcd_config(id, EVCD_CFG_SET_USE_PIC_SIGNATURE, &value, &size);
        if(EVC_FAILED(ret))
        {
            v0print("failed to set config for picture signature\n");
            return -1;
        }
    }
    return 0;
}

static int write_dec_img(EVCD id, char * fname, EVC_IMGB * img, EVC_IMGB * imgb_t)
{
    if(op_out_bit_depth == 8)
    {
        imgb_conv_16b_to_8b(imgb_t, img, 2);
        if(imgb_write(op_fname_out, imgb_t)) return -1;
    }
    else
    {
        if(imgb_write(op_fname_out, img)) return -1;
    }
    return EVC_OK;
}

int main(int argc, const char **argv)
{
    STATES             state = STATE_DECODING;
    unsigned char    * bs_buf = NULL;
    EVCD              id = NULL;
    EVCD_CDSC         cdsc;
    EVC_BITB          bitb;
    EVC_IMGB        * imgb;
    /*temporal buffer for video bit depth less than 10bit */
    EVC_IMGB        * imgb_t = NULL;
    EVCD_STAT         stat;
    int                ret;
    EVC_CLK           clk_beg, clk_tot;
    int                bs_cnt, pic_cnt;
    int                bs_size, bs_read_pos = 0;
    int                w, h;
    FILE             * fp_bs = NULL;

#if QC_DRA
    EVC_IMGB          *imgb_dra = NULL;
    // global CVS buffer for DRA control
    SignalledParamsDRA g_dra_control_array[32];
    // local PU buffer for 2 types of APS data: ALF and DRA
    evc_AlfSliceParam g_alf_control;
    evc_AlfSliceParam *p_g_alf_control = &g_alf_control;

    WCGDDRAControl g_dra_control_effective;
    WCGDDRAControl g_dra_control_read;
    WCGDDRAControl *p_g_dra_control_read = &g_dra_control_read;

    // Structure to keep 2 types of APS to read at PU
    EVC_APS_GEN aps_gen_array[2];
    EVC_APS_GEN *p_aps_gen_array = &(aps_gen_array[0]);

    for (int i = 0; i < 32; i++)
    {
        g_dra_control_array[i].m_signal_dra_flag = -1;
    }
    g_dra_control_read.m_signalledDRA.m_signal_dra_flag = 0;
    aps_gen_array[0].aps_data = (void*)p_g_alf_control;
    aps_gen_array[1].aps_data = (void*)(&(g_dra_control_read.m_signalledDRA));
    evc_resetApsGenReadBuffer(p_aps_gen_array);
#endif

#if DECODING_TIME_TEST
    clk_beg = evc_clk_get();
#endif
    /* parse options */
    ret = evc_args_parse_all(argc, argv, options);
    if(ret != 0)
    {
        if(ret > 0) v0print("-%c argument should be set\n", ret);
        print_usage();
        return -1;
    }

    /* open input bitstream */
    fp_bs = fopen(op_fname_inp, "rb");
    if(fp_bs == NULL)
    {
        v0print("ERROR: cannot open bitstream file = %s\n", op_fname_inp);
        print_usage();
        return -1;
    }

    if(op_flag[OP_FLAG_FNAME_OUT])
    {
        /* remove decoded file contents if exists */
        FILE * fp;
        fp = fopen(op_fname_out, "wb");
        if(fp == NULL)
        {
            v0print("ERROR: cannot create a decoded file\n");
            print_usage();
            return -1;
        }
        fclose(fp);
    }

    bs_buf = malloc(MAX_BS_BUF);
    if(bs_buf == NULL)
    {
        v0print("ERROR: cannot allocate bit buffer, size=%d\n", MAX_BS_BUF);
        return -1;
    }
    id = evcd_create(&cdsc, NULL);
    if(id == NULL)
    {
        v0print("ERROR: cannot create EVC decoder\n");
        return -1;
    }
    if(set_extra_config(id))
    {
        v0print("ERROR: cannot set extra configurations\n");
        return -1;
    }

    pic_cnt = 0;
    clk_tot = 0;
    bs_cnt  = 0;
    w = h   = 0;

    int process_status = EVC_OK;

    while(1)
    {
        if (state == STATE_DECODING)
        {
            evc_mset(&stat, 0, sizeof(EVCD_STAT));

            bs_size = read_nalu(fp_bs, &bs_read_pos, bs_buf);
            int nalu_size_field_in_bytes = 4;

            if (bs_size <= 0)
            {
                state = STATE_BUMPING;
                v1print("bumping process starting...\n");
                continue;
            }

            bs_read_pos += (nalu_size_field_in_bytes + bs_size);
            stat.read += nalu_size_field_in_bytes;
            bitb.addr = bs_buf;
            bitb.ssize = bs_size;
            bitb.bsize = MAX_BS_BUF;

            v1print("[%4d] NALU --> ", bs_cnt++);
#if !DECODING_TIME_TEST
            clk_beg = evc_clk_get();
#endif
            /* main decoding block */
#if QC_DRA
            ret = evcd_decode(id, &bitb, &stat, (void*)(p_aps_gen_array));

            // check if new DRA APS recieved, update buffer
            if ((p_aps_gen_array + 1)->aps_id != -1)
            {
                evc_addDraApsToBuffer(&(g_dra_control_array[0]), p_aps_gen_array);
            }
            // Assigned effective DRA controls as specified by PPS
            int pps_dra_id = evcd_get_pps_dra_id(id);
            if ((pps_dra_id > -1) && (pps_dra_id < 32))
            {
                memcpy(&(g_dra_control_effective.m_signalledDRA), &(g_dra_control_array[pps_dra_id]), sizeof(SignalledParamsDRA));
                g_dra_control_effective.m_flagEnabled = 1;
                evcd_assign_pps_draParam(id, &(g_dra_control_effective.m_signalledDRA));
            }
            else
            {
                g_dra_control_effective.m_flagEnabled = 0;
                g_dra_control_effective.m_signalledDRA.m_signal_dra_flag = 0;
            }
#else
            ret = evcd_decode(id, &bitb, &stat);
#endif

            if(EVC_FAILED(ret))
            {
                v0print("failed to decode bitstream\n");
                goto END;
            }

#if !DECODING_TIME_TEST
            clk_tot += evc_clk_from(clk_beg);
#endif
            print_stat(&stat, ret);

            if(stat.read - nalu_size_field_in_bytes != bs_size)
            {
                v0print("\t=> different reading of bitstream (in:%d, read:%d)\n",
                    bs_size, stat.read);
            }

            process_status = ret;
        }
        if(stat.fnum >= 0 || state == STATE_BUMPING)
        {
            ret = evcd_pull(id, &imgb);
            if(ret == EVC_ERR_UNEXPECTED)
            {
                v1print("bumping process completed\n");
                goto END;
            }
            else if(EVC_FAILED(ret))
            {
                v0print("failed to pull the decoded image\n");
                return -1;
            }
        }
        else
        {
            imgb = NULL;
        }

        if(imgb)
        {
            w = imgb->w[0];
            h = imgb->h[0];

            if(op_flag[OP_FLAG_FNAME_OUT])
            {
                if(op_out_bit_depth == 8 && imgb_t == NULL)
                {
                    imgb_t = imgb_alloc(w, h, EVC_COLORSPACE_YUV420);
                    if(imgb_t == NULL)
                    {
                        v0print("failed to allocate temporay image buffer\n");
                        return -1;
                    }
                }
#if QC_DRA
                if (g_dra_control_effective.m_flagEnabled)
                {
                    evcd_initDRA(&g_dra_control_effective);
                }
                if (g_dra_control_effective.m_flagEnabled)
                {
                    int align[EVC_IMGB_MAX_PLANE] = { MIN_CU_SIZE, MIN_CU_SIZE >> 1, MIN_CU_SIZE >> 1 };
                    int pad[EVC_IMGB_MAX_PLANE] = { 0, 0, 0, };
                    imgb_dra = evc_imgb_create(w, h, EVC_COLORSPACE_YUV420_10LE, 0, pad, align);
                    if (imgb_dra == NULL)
                    {
                        v0print("Cannot get original image buffer (DRA)\n");
                        return -1;
                    }
                    imgb_cpy(imgb_dra, imgb);
                    evc_apply_dra_chroma_plane(imgb, imgb, &g_dra_control_effective, 1, TRUE/*backwardMapping == true*/);
                    evc_apply_dra_chroma_plane(imgb, imgb, &g_dra_control_effective, 2, TRUE /*backwardMapping == true*/);
                    evc_apply_dra_luma_plane(imgb, imgb, &g_dra_control_effective, 0, TRUE /*backwardMapping == true*/);
                    write_dec_img(id, op_fname_out, imgb, imgb_t);
                    imgb_cpy(imgb, imgb_dra);
                    imgb_dra->release(imgb_dra);
                }
                else
                    write_dec_img(id, op_fname_out, imgb, imgb_t);
#else
                write_dec_img(id, op_fname_out, imgb, imgb_t);
#endif
            }
            imgb->release(imgb);
            pic_cnt++;
        }
    }

END:
#if DECODING_TIME_TEST
    clk_tot += evc_clk_from(clk_beg);
#endif
    v1print("=======================================================================================\n");
    v1print("Resolution                        = %d x %d\n", w, h);
    v1print("Processed NALUs                   = %d\n", bs_cnt);
    v1print("Decoded frame count               = %d\n", pic_cnt);
    if(pic_cnt > 0)
    {
        v1print("total decoding time               = %d msec,",
                (int)evc_clk_msec(clk_tot));
        v1print(" %.3f sec\n",
            (float)(evc_clk_msec(clk_tot) /1000.0));

        v1print("Average decoding time for a frame = %d msec\n",
                (int)evc_clk_msec(clk_tot)/pic_cnt);
        v1print("Average decoding speed            = %.3f frames/sec\n",
                ((float)pic_cnt*1000)/((float)evc_clk_msec(clk_tot)));
    }
    v1print("=======================================================================================\n");

    if(id) evcd_delete(id);
    if(imgb_t) imgb_free(imgb_t);
    if(fp_bs) fclose(fp_bs);
    if(bs_buf) free(bs_buf);

    return process_status;
}
