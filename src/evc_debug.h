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

#pragma once
#ifndef _EVCD_DEBUG_H_
#define _EVCD_DEBUG_H_
#include "evc_def.h"
#if GRAB_STAT

typedef enum _ENUM_STAT_USAGE
{
    esu_only_enc,
    esu_only_rdo,
    esu_rdo_and_enc,
} ENUM_STAT_USAGE;

typedef void(*Stat_Log)(int x, int y, int cuw, int cuh, int cup, void *ctx, void *core
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
    );

void evc_stat_init(const char *fileName, ENUM_STAT_USAGE usage, int start_poc, int end_poc, Stat_Log stat_log);

void evc_stat_set_poc(int poc);

void evc_stat_set_enc_state(BOOL isRDO);

void evc_stat_write_lcu(int x, int y, int pic_w, int pic_h, int lcu_size, int log2_culine, void *ctx, void *core, s8(*map_split)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU], s8(*map_suco)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]);

void evc_stat_write_cu_str(int x, int y, int cuw, int cuh, const char* name, int value);
void evc_stat_write_cu_vec(int x, int y, int cuw, int cuh, const char* name, int vec_x, int vec_y);
void evc_stat_write_comment(const char* format, ...);
void evc_stat_write_type(const char* name, const char* type, const char* range);

void evc_stat_finish();
#endif

#endif