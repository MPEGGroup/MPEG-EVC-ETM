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

#include "evce_def.h"
#include <math.h>


static int pintra_init_frame(EVCE_CTX * ctx)
{
    EVCE_PINTRA * pi;
    EVC_PIC     * pic;

    pi     = &ctx->pintra;

    pic          = pi->pic_o = PIC_ORIG(ctx);
    pi->o[Y_C]   = pic->y;
    pi->o[U_C]   = pic->u;
    pi->o[V_C]   = pic->v;

    pi->s_o[Y_C] = pic->s_l;
    pi->s_o[U_C] = pic->s_c;
    pi->s_o[V_C] = pic->s_c;

    pic          = pi->pic_m = PIC_MODE(ctx);
    pi->m[Y_C]   = pic->y;
    pi->m[U_C]   = pic->u;
    pi->m[V_C]   = pic->v;

    pi->s_m[Y_C] = pic->s_l;
    pi->s_m[U_C] = pic->s_c;
    pi->s_m[V_C] = pic->s_c;

    pi->tile_group_type = ctx->tile_group_type;

    return EVC_OK;
}

static int pintra_analyze_lcu(EVCE_CTX * ctx, EVCE_CORE * core)
{
    return EVC_OK;
}

static double pintra_residue_rdo(EVCE_CTX *ctx, EVCE_CORE *core, pel *org_luma, pel *org_cb, pel *org_cr, int s_org, int s_org_c, int log2_cuw, int log2_cuh, s16 coef[N_C][MAX_CU_DIM], s32 *dist, int mode
#if RDO_DBK
                                 , int x, int y
#endif
)
{
    EVCE_PINTRA *pi = &ctx->pintra;
    int cuw, cuh, bit_cnt;
    double cost = 0;
    int tmp_cbf_l =0;
    int tmp_cbf_sub_l[MAX_SUB_TB_NUM] = {0,};

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;

    if(mode == 0)
    {
        pel * pred = 0;

        pred = pi->pred_cache[core->ipm[0]];
        evce_diff_16b(log2_cuw, log2_cuh, org_luma, pred, s_org, cuw, cuw, pi->coef_tmp[Y_C]);

        evce_sub_block_tq(pi->coef_tmp, log2_cuw, log2_cuh, core->qp_y, core->qp_u, core->qp_v, pi->tile_group_type, core->nnz
                          , core->nnz_sub, 1, ctx->lambda[0], ctx->lambda[1], ctx->lambda[2], RUN_L, ctx->sps.tool_cm_init, ctx->sps.tool_iqt);

        evc_mcpy(coef[Y_C], pi->coef_tmp[Y_C], sizeof(u16) * (cuw * cuh));

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
        evce_sbac_bit_reset(&core->s_temp_run);
        evce_rdo_bit_cnt_cu_intra_luma(ctx, core, ctx->tgh.tile_group_type, core->scup, pi->coef_tmp);
        bit_cnt = evce_get_bit_number(&core->s_temp_run);

        evc_sub_block_itdq(pi->coef_tmp, log2_cuw, log2_cuh, core->qp_y, core->qp_u, core->qp_v, core->nnz, core->nnz_sub, ctx->sps.tool_iqt);

        evc_recon(pi->coef_tmp[Y_C], pred, core->nnz[Y_C], cuw, cuh, cuw, pi->rec[Y_C]);
#if HTDF
        if(ctx->sps.tool_htdf == 1)
        {
#if !HTDF_CBF0_INTRA
            if(core->nnz[Y_C])
#endif
                evc_htdf(pi->rec[Y_C], ctx->tgh.qp, cuw, cuh, cuw, TRUE, core->nb[Y_C][0] + 2, core->nb[Y_C][1] + cuh - 1, core->nb[Y_C][2] + 2, core->avail_cu);
        }
#endif
        cost += evce_ssd_16b(log2_cuw, log2_cuh, pi->rec[Y_C], org_luma, cuw, s_org);

#if RDO_DBK
        {
            calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pi->rec, cuw, x, y, core->avail_lr, 1, core->nnz[Y_C] != 0, NULL, NULL, 0);
            cost += ctx->delta_dist[Y_C];
        }
#endif
        *dist = (s32)cost;
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    }
    else
    {
        if (ctx->sps.tool_eipd)
        {
            evc_ipred_uv(core->nb[1][0] + 2, core->nb[1][1] + (cuh >> 1), core->nb[1][2] + 2, core->avail_lr, pi->pred[U_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
            evc_ipred_uv(core->nb[2][0] + 2, core->nb[2][1] + (cuh >> 1), core->nb[2][2] + 2, core->avail_lr, pi->pred[V_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
        }
        else
        {
            evc_ipred_uv_b(core->nb[1][0] + 2, core->nb[1][1] + (cuh >> 1), core->nb[1][2] + 2, core->avail_lr, pi->pred[U_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
            evc_ipred_uv_b(core->nb[2][0] + 2, core->nb[2][1] + (cuh >> 1), core->nb[2][2] + 2, core->avail_lr, pi->pred[V_C], core->ipm[1], core->ipm[0], cuw >> 1, cuh >> 1, core->avail_cu);
        }

        evce_diff_16b(log2_cuw - 1, log2_cuh - 1, org_cb, pi->pred[U_C], s_org_c, cuw >> 1, cuw >> 1, pi->coef_tmp[U_C]);
        evce_diff_16b(log2_cuw - 1, log2_cuh - 1, org_cr, pi->pred[V_C], s_org_c, cuw >> 1, cuw >> 1, pi->coef_tmp[V_C]);

        if (!ctx->sps.tool_eipd)
        {
            tmp_cbf_l = core->nnz[Y_C];
            evc_mcpy(tmp_cbf_sub_l, core->nnz_sub[Y_C], sizeof(int) * MAX_SUB_TB_NUM);
        }

        evce_sub_block_tq(pi->coef_tmp, log2_cuw, log2_cuh, core->qp_y, core->qp_u, core->qp_v, pi->tile_group_type, core->nnz, core->nnz_sub
                          , 1, ctx->lambda[0], ctx->lambda[1], ctx->lambda[2], RUN_CB | RUN_CR, ctx->sps.tool_cm_init, ctx->sps.tool_iqt);

        evc_mcpy(coef[U_C], pi->coef_tmp[U_C], sizeof(u16) * (cuw * cuh));
        evc_mcpy(coef[V_C], pi->coef_tmp[V_C], sizeof(u16) * (cuw * cuh));

        evc_sub_block_itdq(pi->coef_tmp, log2_cuw, log2_cuh, core->qp_y, core->qp_u, core->qp_v, core->nnz, core->nnz_sub, ctx->sps.tool_iqt);

        if (!ctx->sps.tool_eipd)
        {
            core->nnz[Y_C] = tmp_cbf_l;
            evc_mcpy(core->nnz_sub[Y_C], tmp_cbf_sub_l, sizeof(int) * MAX_SUB_TB_NUM);
        }

        evc_recon(pi->coef_tmp[U_C], pi->pred[U_C], core->nnz[U_C], cuw >> 1, cuh >> 1, cuw >> 1, pi->rec[U_C]);
        evc_recon(pi->coef_tmp[V_C], pi->pred[V_C], core->nnz[V_C], cuw >> 1, cuh >> 1, cuw >> 1, pi->rec[V_C]);

        if (ctx->sps.tool_eipd)
        {
            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_best);
        }

        evce_sbac_bit_reset(&core->s_temp_run);

        evce_rdo_bit_cnt_cu_intra_chroma(ctx, core, ctx->tgh.tile_group_type, core->scup, coef);

        bit_cnt = evce_get_bit_number(&core->s_temp_run);

        cost += ctx->dist_chroma_weight[0] * evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->rec[U_C], org_cb, cuw >> 1, s_org_c);
        cost += ctx->dist_chroma_weight[1] * evce_ssd_16b(log2_cuw - 1, log2_cuh - 1, pi->rec[V_C], org_cr, cuw >> 1, s_org_c);
#if RDO_DBK
        {
            calc_delta_dist_filter_boundary(ctx, PIC_MODE(ctx), PIC_ORIG(ctx), cuw, cuh, pi->rec, cuw, x, y, core->avail_lr, 1, core->nnz[Y_C] != 0, NULL, NULL, 0);
            cost += (ctx->delta_dist[U_C] * ctx->dist_chroma_weight[0]) + (ctx->delta_dist[V_C] * ctx->dist_chroma_weight[1]);
        }
#endif

        *dist = (s32)cost;

        cost += evce_ssd_16b(log2_cuw, log2_cuh, pi->rec[Y_C], org_luma, cuw, s_org);
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    }

    return cost;
}

static int make_ipred_list(EVCE_CTX * ctx, EVCE_CORE * core, int log2_cuw, int log2_cuh, pel * org, int s_org, int * ipred_list)
{
    EVCE_PINTRA * pi = &ctx->pintra;
    int cuw, cuh, pred_cnt, i, j;
    double cost, cand_cost[IPD_RDO_CNT];
    u32 cand_satd_cost[IPD_RDO_CNT];
    u32 cost_satd;
    const int ipd_rdo_cnt = EVC_ABS(log2_cuw - log2_cuh) >= 2 ? IPD_RDO_CNT - 1 : IPD_RDO_CNT;

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;

    for(i = 0; i < ipd_rdo_cnt; i++)
    {
        ipred_list[i] = IPD_DC;
        cand_cost[i] = MAX_COST;
        cand_satd_cost[i] = EVC_UINT32_MAX;
    }

    pred_cnt = (ctx->sps.tool_eipd) ? IPD_CNT : IPD_CNT_B;

    for (i = 0; i < pred_cnt; i++)
    {
        int bit_cnt, shift = 0;
        pel * pred_buf = NULL;

        pred_buf = pi->pred_cache[i];

        if (ctx->sps.tool_eipd)
        {
            evc_ipred(core->nb[0][0] + 2, core->nb[0][1] + cuh, core->nb[0][2] + 2, core->avail_lr, pred_buf, i, cuw, cuh, core->avail_cu);
        }
        else
        {
            evc_ipred_b(core->nb[0][0] + 2, core->nb[0][1] + cuh, core->nb[0][2] + 2, core->avail_lr, pred_buf, i, cuw, cuh, core->avail_cu);
        }

        cost_satd = evce_satd_16b(log2_cuw, log2_cuh, org, pred_buf, s_org, cuw);
        cost = (double)cost_satd;
        SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);
        evce_sbac_bit_reset(&core->s_temp_run);

        if (ctx->sps.tool_eipd)
        {
            evce_eco_intra_dir(&core->bs_temp, i, core->mpm, core->mpm_ext, core->pims);
        }
        else
        {
            evce_eco_intra_dir_b(&core->bs_temp, i, core->mpm_b_list, core->mpm_ext, core->pims);
        }

        bit_cnt = evce_get_bit_number(&core->s_temp_run);
        cost += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);

        while(shift < ipd_rdo_cnt && cost < cand_cost[ipd_rdo_cnt - 1 - shift])
        {
            shift++;
        }

        if(shift != 0)
        {
            for(j = 1; j < shift; j++)
            {
                ipred_list[ipd_rdo_cnt - j] = ipred_list[ipd_rdo_cnt - 1 - j];
                cand_cost[ipd_rdo_cnt - j] = cand_cost[ipd_rdo_cnt - 1 - j];
                cand_satd_cost[ipd_rdo_cnt - j] = cand_satd_cost[ipd_rdo_cnt - 1 - j];
            }
            ipred_list[ipd_rdo_cnt - shift] = i;
            cand_cost[ipd_rdo_cnt - shift] = cost;
            cand_satd_cost[ipd_rdo_cnt - shift] = cost_satd;
        }
    }

    pred_cnt = ipd_rdo_cnt;
    for(i = ipd_rdo_cnt - 1; i >= 0; i--)
    {
        if(cand_satd_cost[i] > core->inter_satd * (1.1))
        {
            pred_cnt--;
        }
        else
        {
            break;
        }
    }

    return EVC_MIN(pred_cnt, ipd_rdo_cnt);
}

//! \todo Change list of arguments
static double pintra_analyze_cu(EVCE_CTX *ctx, EVCE_CORE *core, int x, int y, int log2_cuw, int log2_cuh, EVCE_MODE *mi, s16 coef[N_C][MAX_CU_DIM], pel *rec[N_C], int s_rec[N_C])
{
    EVCE_PINTRA *pi = &ctx->pintra;
    int i, j, s_org, s_org_c, s_mod, s_mod_c, cuw, cuh;
    int best_ipd = IPD_INVALID;
    int best_ipd_c = IPD_INVALID;
    s32 best_dist_y = 0, best_dist_c = 0;
    int ipm_l2c = 0;
    int chk_bypass = 0;
    int bit_cnt = 0;
    int ipred_list[IPD_CNT];
    int pred_cnt = (ctx->sps.tool_eipd) ? IPD_CNT : IPD_CNT_B;;
    pel *org, *mod;
    pel *org_cb, *org_cr;
    pel *mod_cb, *mod_cr;
    double cost_t, cost = MAX_COST;
    int sec_best_ipd = IPD_INVALID;

    cuw = 1 << log2_cuw;
    cuh = 1 << log2_cuh;

    /* Y */
    evc_assert(x + cuw <= pi->pic_o->w_l);
    evc_assert(y + cuh <= pi->pic_o->h_l);

    /* prediction */
    s_mod = pi->s_m[Y_C];
    mod = pi->m[Y_C] + (y * s_mod) + x;

    s_org = pi->s_o[Y_C];
    org = pi->o[Y_C] + (y * s_org) + x;

    s_mod_c = pi->s_m[U_C];
    mod_cb = pi->m[U_C] + ((y >> 1) * s_mod_c) + (x >> 1);
    mod_cr = pi->m[V_C] + ((y >> 1) * s_mod_c) + (x >> 1);

    s_org_c = pi->s_o[U_C];
    org_cb = pi->o[U_C] + ((y >> 1) * s_org_c) + (x >> 1);
    org_cr = pi->o[V_C] + ((y >> 1) * s_org_c) + (x >> 1);

    evc_get_nbr(x, y, cuw, cuh, mod, s_mod, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, Y_C);
    evc_get_nbr(x >> 1, y >> 1, cuw >> 1, cuh >> 1, mod_cb, s_mod_c, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, U_C);
    evc_get_nbr(x >> 1, y >> 1, cuw >> 1, cuh >> 1, mod_cr, s_mod_c, core->avail_cu, core->nb, core->scup, ctx->map_scu, ctx->w_scu, ctx->h_scu, V_C);

    if(ctx->sps.tool_eipd)
    {
        evc_get_mpm(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                    core->mpm, core->avail_lr, core->mpm_ext, core->pims);
    }
    else
    {
        evc_get_mpm_b(core->x_scu, core->y_scu, cuw, cuh, ctx->map_scu, ctx->map_ipm, core->scup, ctx->w_scu,
                      &core->mpm_b_list, core->avail_lr, core->mpm_ext, core->pims);
    }

    pred_cnt = make_ipred_list(ctx, core, log2_cuw, log2_cuh, org, s_org, ipred_list);
    if(pred_cnt == 0)
    {
        return MAX_COST;
    }

    for(j = 0; j < pred_cnt; j++) /* Y */
    {
        s32 dist_t = 0;
        s32 dist_tc = 0;

        i = ipred_list[j];
        core->ipm[0] = i;

        if(ctx->sps.tool_eipd)
        {
            core->ipm[1] = IPD_INVALID;
            cost_t = pintra_residue_rdo(ctx, core, org, NULL, NULL, s_org, s_org_c, log2_cuw, log2_cuh, coef, &dist_t, 0
#if RDO_DBK
                                        , x, y
#endif
            );
        }
        else
        {
            core->ipm[1] = i;
            cost_t = pintra_residue_rdo(ctx, core, org, org_cb, org_cr, s_org, s_org_c, log2_cuw, log2_cuh, coef, &dist_t, 0
#if RDO_DBK
                                        , x, y
#endif
            );
            cost_t += pintra_residue_rdo(ctx, core, org, org_cb, org_cr, s_org, s_org_c, log2_cuw, log2_cuh, coef, &dist_tc, 1
#if RDO_DBK
                                         , x, y
#endif
            );
        }

        if(cost_t < cost)
        {
            cost = cost_t;
            best_dist_y = dist_t;

            if(!ctx->sps.tool_eipd)
            {
                best_dist_c = dist_tc;
            }

            if(sec_best_ipd != best_ipd)
            {
                sec_best_ipd = best_ipd;
            }

            best_ipd = i;

            evc_mcpy(pi->coef_best[Y_C], coef[Y_C], (cuw * cuh) * sizeof(s16));
            evc_mcpy(pi->rec_best[Y_C], pi->rec[Y_C], (cuw * cuh) * sizeof(pel));

            pi->nnz_best[Y_C] = core->nnz[Y_C];
            evc_mcpy(pi->nnz_sub_best[Y_C], core->nnz_sub[Y_C], sizeof(int) * MAX_SUB_TB_NUM);

            if(!ctx->sps.tool_eipd)
            {
                best_ipd_c = i;

                for(j = U_C; j < N_C; j++)
                {
                    int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                    evc_mcpy(pi->coef_best[j], coef[j], size_tmp * sizeof(s16));
                    evc_mcpy(pi->rec_best[j], pi->rec[j], size_tmp * sizeof(pel));

                    pi->nnz_best[j] = core->nnz[j];
                    evc_mcpy(pi->nnz_sub_best[j], core->nnz_sub[j], sizeof(int) * MAX_SUB_TB_NUM);
                }
            }

            SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
        }
    }

    if(ctx->sps.tool_eipd)
    {
        cost = MAX_COST;
        ipm_l2c = best_ipd;
        core->ipm[0] = best_ipd;

        EVC_IPRED_CONV_L2C_CHK(ipm_l2c, chk_bypass);

        for(i = 0; i < IPD_CHROMA_CNT; i++) /* UV */
        {
            s32 dist_t = 0;

            core->ipm[1] = i;

            if(i != IPD_DM_C && chk_bypass && i == ipm_l2c)
            {
                continue;
            }

            cost_t = pintra_residue_rdo(ctx, core, org, org_cb, org_cr, s_org, s_org_c, log2_cuw, log2_cuh, coef, &dist_t, 1
#if RDO_DBK
                                        , x, y
#endif
            );

            if(cost_t < cost)
            {
                cost = cost_t;
                best_dist_c = dist_t;
                best_ipd_c = i;

                for(j = U_C; j < N_C; j++)
                {
                    int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
                    evc_mcpy(pi->coef_best[j], coef[j], size_tmp * sizeof(s16));
                    evc_mcpy(pi->rec_best[j], pi->rec[j], size_tmp * sizeof(pel));

                    pi->nnz_best[j] = core->nnz[j];
                    evc_mcpy(pi->nnz_sub_best[j], core->nnz_sub[j], sizeof(int) * MAX_SUB_TB_NUM);
                }
            }
        }
    }

    core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].ipm[0] = best_ipd;
    core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup][evc_get_lr(core->avail_lr)].ipm[1] = sec_best_ipd;

    for(j = 0; j < N_C; j++)
    {
        int size_tmp = (cuw * cuh) >> (j == 0 ? 0 : 2);
        evc_mcpy(coef[j], pi->coef_best[j], size_tmp * sizeof(u16));
        evc_mcpy(pi->rec[j], pi->rec_best[j], size_tmp * sizeof(pel));
        core->nnz[j] = pi->nnz_best[j];
        evc_mcpy(core->nnz_sub[j], pi->nnz_sub_best[j], sizeof(int) * MAX_SUB_TB_NUM);
    }

    rec[Y_C] = pi->rec[Y_C];
    rec[U_C] = pi->rec[U_C];
    rec[V_C] = pi->rec[V_C];

    s_rec[Y_C] = cuw;
    s_rec[U_C] = cuw >> 1;
    s_rec[V_C] = cuw >> 1;

    core->ipm[0] = best_ipd;
    core->ipm[1] = best_ipd_c;
    evc_assert(best_ipd_c != IPD_INVALID);

    /* cost calculation */
    SBAC_LOAD(core->s_temp_run, core->s_curr_best[log2_cuw - 2][log2_cuh - 2]);

    evce_sbac_bit_reset(&core->s_temp_run);
    evce_rdo_bit_cnt_cu_intra(ctx, core, ctx->tgh.tile_group_type, core->scup, coef);

    bit_cnt = evce_get_bit_number(&core->s_temp_run);
    cost = (best_dist_y + best_dist_c) + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);

    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    core->dist_cu = best_dist_y + best_dist_c;

    return cost;
}

static int pintra_set_complexity(EVCE_CTX * ctx, int complexity)
{
    EVCE_PINTRA * pi;

    pi = &ctx->pintra;
    pi->complexity = complexity;

    return EVC_OK;
}

int evce_pintra_create(EVCE_CTX * ctx, int complexity)
{
    /* set function addresses */
    ctx->fn_pintra_set_complexity = pintra_set_complexity;
    ctx->fn_pintra_init_frame = pintra_init_frame;
    ctx->fn_pintra_init_lcu = pintra_analyze_lcu;
    ctx->fn_pintra_analyze_cu = pintra_analyze_cu;

    return ctx->fn_pintra_set_complexity(ctx, complexity);
}
