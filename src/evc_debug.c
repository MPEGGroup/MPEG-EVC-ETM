#include "evc_debug.h"
#include <stdarg.h>

#if GRAB_STAT
typedef struct _STAT_DEBUG
{
    FILE*           f;
    ENUM_STAT_USAGE usage;
    int             start_poc;
    int             end_poc;
    Stat_Log        stat_log;

    int             cur_poc;
    BOOL            isRDO;
    BOOL            started;

    BOOL            active;
} STAT_DEBUG;

STAT_DEBUG g_stat;

static void stat_check_conditions()
{
    BOOL poc_state = FALSE;
    if (g_stat.end_poc == -1)
        poc_state = (g_stat.started || (g_stat.cur_poc == g_stat.start_poc));
    else
        poc_state  = (g_stat.cur_poc >= g_stat.start_poc) && (g_stat.cur_poc <= g_stat.end_poc);
    BOOL enc_state = (g_stat.usage == esu_rdo_and_enc) || (g_stat.usage == (g_stat.isRDO ? esu_only_rdo : esu_only_enc));
    g_stat.active = poc_state && enc_state;

    if (g_stat.active && !g_stat.started && (g_stat.cur_poc == g_stat.start_poc))
        g_stat.started = TRUE;
}

void evc_stat_init(const char *fileName, ENUM_STAT_USAGE usage, int start_poc, int end_poc, Stat_Log stat_log)
{
    g_stat.f = fopen(fileName, "w");
    g_stat.usage = usage;
    g_stat.cur_poc = 0;
    g_stat.start_poc = start_poc;
    g_stat.end_poc = end_poc;
    g_stat.active = FALSE;
    g_stat.isRDO = TRUE;
    g_stat.started = FALSE;
    g_stat.stat_log = stat_log;
}

void evc_stat_write_cu_str(int x, int y, int cuw, int cuh, const char* name, int value)
{
    if (g_stat.active)
    {
        fprintf(g_stat.f, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", g_stat.cur_poc, x, y, cuw, cuh, name, value);
    }
}

void evc_stat_write_cu_vec(int x, int y, int cuw, int cuh, const char* name, int vec_x, int vec_y)
{
    if (g_stat.active)
    {
        fprintf(g_stat.f, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", g_stat.cur_poc, x, y, cuw, cuh, name, vec_x, vec_y);
    }
}


void evc_stat_write_comment(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    fprintf(g_stat.f, "# ");
    vfprintf(g_stat.f, format, args);
    fprintf(g_stat.f, "\n");
    va_end(args);
}
void evc_stat_write_type(const char* name, const char* type, const char* range)
{
    fprintf(g_stat.f, "# Block Statistic Type: %s; %s; ", name, type);
    if (range)
        fprintf(g_stat.f, range);
    fprintf(g_stat.f, "\n");
}


static void evc_stat_tree(void *ctx, void *core, int x, int y, int cuw, int cuh, int cup, int cud, int lcu_size, int pic_w, int pic_h, int log2_culine, s8(*map_split)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU], s8(*map_suco)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU]
#if M50761_CHROMA_NOT_SPLIT
    , TREE_CONS tree_cons
#endif
)
{
    s8  split_mode;
    s8  suco_flag = 0;

    evc_get_split_mode(&split_mode, cud, cup, cuw, cuh, lcu_size, map_split);
    evc_get_suco_flag(&suco_flag, cud, cup, cuw, cuh, lcu_size, map_suco);

    if (split_mode != NO_SPLIT)
    {
        EVC_SPLIT_STRUCT split_struct;
        int suco_order[SPLIT_MAX_PART_COUNT];

        evc_split_get_part_structure(split_mode, x, y, cuw, cuh, cup, cud, log2_culine, &split_struct
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
        evc_split_get_suco_order(suco_flag, split_mode, suco_order);
#if M50761_CHROMA_NOT_SPLIT
        BOOL mode_cons_changed = evc_signal_mode_cons(&tree_cons, &split_struct.tree_cons);
#endif


        for (int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = suco_order[part_num];
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if (x_pos < pic_w && y_pos < pic_h)
            {
                evc_stat_tree(ctx, core, x_pos, y_pos, sub_cuw, sub_cuh, split_struct.cup[cur_part_num], split_struct.cud[cur_part_num], lcu_size, pic_w, pic_h, log2_culine, map_split, map_suco
#if M50761_CHROMA_NOT_SPLIT
                    , split_struct.tree_cons
#endif
                );
            }
        }
#if M50761_CHROMA_NOT_SPLIT
        if (mode_cons_changed && !evc_check_all(split_struct.tree_cons))
        {
            TREE_CONS local_cons = split_struct.tree_cons;
            local_cons.tree_type = TREE_C;
            g_stat.stat_log(x, y, cuw, cuh, cup, ctx, core, local_cons);
        }
#endif
    }
    else
    {
        g_stat.stat_log(x, y, cuw, cuh, cup, ctx, core
#if M50761_CHROMA_NOT_SPLIT
            , tree_cons
#endif
        );
    }

}

void evc_stat_write_lcu(int x, int y, int pic_w, int pic_h, int lcu_size, int log2_culine, void *ctx, void *core, s8(*map_split)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU], s8(*map_suco)[NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU])
{
    evc_stat_tree(ctx, core, x, y, lcu_size, lcu_size, 0, 0, lcu_size, pic_w, pic_h, log2_culine, map_split, map_suco
#if M50761_CHROMA_NOT_SPLIT
        , evc_get_default_tree_cons()
#endif
    );
}

void evc_stat_finish()
{
    fclose(g_stat.f);
}

void evc_stat_set_poc(int poc)
{
    g_stat.cur_poc = poc;
    stat_check_conditions();
}

void evc_stat_set_enc_state(BOOL isRDO)
{
    g_stat.isRDO = isRDO;
    stat_check_conditions();
}

#endif