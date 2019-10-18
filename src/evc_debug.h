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