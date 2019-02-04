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

#include "evc_def.h"

/* macros for reference picture flag */
#define IS_REF(pic)          (((pic)->is_ref) != 0)
#define SET_REF_UNMARK(pic)  (((pic)->is_ref) = 0)
#define SET_REF_MARK(pic)    (((pic)->is_ref) = 1)

#define PRINT_DPB(pm)\
    printf("%s: current num_ref = %d, dpb_size = %d\n", __FUNCTION__, \
    pm->cur_num_ref_pics, picman_get_num_allocated_pics(pm));

static int picman_get_num_allocated_pics(EVC_PM * pm)
{
    int i, cnt = 0;
    for(i = 0; i < MAX_PB_SIZE; i++) /* this is coding order */
    {
        if(pm->pic[i]) cnt++;
    }
    return cnt;
}

static int picman_move_pic(EVC_PM *pm, int from, int to)
{
    int i;
    EVC_PIC * pic;

    pic = pm->pic[from];

    for(i = from; i < to; i++)
    {
        pm->pic[i] = pm->pic[i + 1];
    }
    pm->pic[to] = pic;

    return 0;
}

static void picman_sliding_window(EVC_PM * pm)
{
    int i;
    EVC_PIC * pic;

    while(pm->cur_num_ref_pics >= pm->max_num_ref_pics)
    {
        for(i = 0; i < MAX_PB_SIZE; i++) /* this is coding order */
        {
            if(pm->pic[i] && IS_REF(pm->pic[i]))
            {
                pic = pm->pic[i];

                /* unmark for reference */
                SET_REF_UNMARK(pic);

                picman_move_pic(pm, i, MAX_PB_SIZE - 1);

                pm->cur_num_ref_pics--;

                break;
            }
        }
    }
}

static void picman_flush_pb(EVC_PM * pm)
{
    int i;

    /* mark all frames unused */
    for(i = 0; i < MAX_PB_SIZE; i++)
    {
        if(pm->pic[i]) SET_REF_UNMARK(pm->pic[i]);
    }
    pm->cur_num_ref_pics = 0;
}

static void picman_update_pic_ref(EVC_PM * pm)
{
    EVC_PIC ** pic;
    EVC_PIC ** pic_ref;
    EVC_PIC  * pic_t;
    int i, j, cnt;

    pic = pm->pic;
    pic_ref = pm->pic_ref;

    for(i = 0, j = 0; i < MAX_PB_SIZE; i++)
    {
        if(pic[i] && IS_REF(pic[i]))
        {
            pic_ref[j++] = pic[i];
        }
    }
    cnt = j;
    while(j < MAX_NUM_REF_PICS) pic_ref[j++] = NULL;

    /* descending order sort based on PTR */
    for(i = 0; i < cnt - 1; i++)
    {
        for(j = i + 1; j < cnt; j++)
        {
            if(pic_ref[i]->ptr < pic_ref[j]->ptr)
            {
                pic_t = pic_ref[i];
                pic_ref[i] = pic_ref[j];
                pic_ref[j] = pic_t;
            }
        }
    }
}

static EVC_PIC * picman_remove_pic_from_pb(EVC_PM * pm, int pos)
{
    int         i;
    EVC_PIC  * pic_rem;

    pic_rem = pm->pic[pos];
    pm->pic[pos] = NULL;

    /* fill empty pic buffer */
    for(i = pos; i < MAX_PB_SIZE - 1; i++)
    {
        pm->pic[i] = pm->pic[i + 1];
    }
    pm->pic[MAX_PB_SIZE - 1] = NULL;

    pm->cur_pb_size--;

    return pic_rem;
}

static void picman_set_pic_to_pb(EVC_PM * pm, EVC_PIC * pic,
                                 EVC_REFP(*refp)[REFP_NUM], int pos)
{
    int i;

    for(i = 0; i < pm->num_refp[REFP_0]; i++)
        pic->list_ptr[i] = refp[i][REFP_0].ptr;

    if(pos >= 0)
    {
        evc_assert(pm->pic[pos] == NULL);
        pm->pic[pos] = pic;
    }
    else /* pos < 0 */
    {
        /* search empty pic buffer position */
        for(i = (MAX_PB_SIZE - 1); i >= 0; i--)
        {
            if(pm->pic[i] == NULL)
            {
                pm->pic[i] = pic;
                break;
            }
        }
        if(i < 0)
        {
            printf("i=%d\n", i);
            evc_assert(i >= 0);
        }
    }
    pm->cur_pb_size++;
}

static int picman_get_empty_pic_from_list(EVC_PM * pm)
{
    EVC_IMGB * imgb;
    EVC_PIC  * pic;
    int i;

    for(i = 0; i < MAX_PB_SIZE; i++)
    {
        pic = pm->pic[i];

        if(pic != NULL && !IS_REF(pic) && pic->need_for_out == 0)
        {
            imgb = pic->imgb;
            evc_assert(imgb != NULL);

            /* check reference count */
            if(1 == imgb->getref(imgb))
            {
                return i; /* this is empty buffer */
            }
        }
    }
    return -1;
}

void set_refp(EVC_REFP * refp, EVC_PIC  * pic_ref)
{
    refp->pic      = pic_ref;
    refp->ptr      = pic_ref->ptr;
    refp->map_mv   = pic_ref->map_mv;
#if DMVR_LAG
    refp->map_unrefined_mv = pic_ref->map_mv;
#endif
    refp->map_refi = pic_ref->map_refi;
    refp->list_ptr = pic_ref->list_ptr;
}

void copy_refp(EVC_REFP * refp_dst, EVC_REFP * refp_src)
{
    refp_dst->pic      = refp_src->pic;
    refp_dst->ptr      = refp_src->ptr;
    refp_dst->map_mv   = refp_src->map_mv;
#if DMVR_LAG
    refp_dst->map_unrefined_mv = refp_src->map_mv;
#endif
    refp_dst->map_refi = refp_src->map_refi;
    refp_dst->list_ptr = refp_src->list_ptr;
}

int check_copy_refp(EVC_REFP(*refp)[REFP_NUM], int cnt, int lidx, EVC_REFP  * refp_src)
{
    int i;

    for(i = 0; i < cnt; i++)
    {
        if(refp[i][lidx].ptr == refp_src->ptr)
        {
            return -1;
        }
    }
    copy_refp(&refp[cnt][lidx], refp_src);

    return EVC_OK;
}

int evc_picman_refp_reorder(EVC_PM *pm, int num_ref_pics_act, u8 tile_group_type, u32 ptr, EVC_REFP(*refp)[REFP_NUM], int last_intra, EVC_RMPNI *rmpni)
{
    int i, j, cnt;
    EVC_REFP refp_temp[MAX_NUM_REF_PICS][REFP_NUM];
    int refp_idx, num_refp;

    if(tile_group_type == TILE_GROUP_I)
    {
        return EVC_OK;
    }
    if(rmpni[REFP_0].cnt == 0 && rmpni[REFP_1].cnt == 0)
    {
        return EVC_OK;
    }

    for(refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
    {
        /* set refp list with rmpni */
        if(rmpni[refp_idx].cnt)
        {
            cnt = 0;
            num_refp = EVC_MAX(rmpni[refp_idx].cnt, pm->num_refp[refp_idx]);
            for(i = 0; i < num_refp; i++)
            {
                if(refp[i][refp_idx].pic != NULL)
                {
                    copy_refp(&refp_temp[i][refp_idx], &refp[i][refp_idx]);
                }
            }

            cnt = 0;
            for(j = 0; j < num_refp && cnt < num_ref_pics_act; j++)
            {
                for(i = 0; i < pm->cur_num_ref_pics; i++)
                {
                    if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;

                    if(pm->pic_ref[i]->ptr == rmpni[refp_idx].delta_poc[j] + ptr)
                    {
                        set_refp(&refp[cnt][refp_idx], pm->pic_ref[i]);
                        cnt++;
                    }
                }
            }
            pm->num_refp[refp_idx] = cnt;
        }
    }

    return EVC_OK;
}

//This is implementation of reference picture list construction based on RPL. This is meant to replace function int evc_picman_refp_init(EVC_PM *pm, int num_ref_pics_act, int tile_group_type, u32 ptr, u8 layer_id, int last_intra, EVC_REFP(*refp)[REFP_NUM])
int evc_picman_refp_rpl_based_init(EVC_PM *pm, EVC_TGH *tgh, EVC_REFP(*refp)[REFP_NUM])
{
    if (tgh->tile_group_type == TILE_GROUP_I)
    {
        return EVC_OK;
    }

    picman_update_pic_ref(pm);
    evc_assert_rv(pm->cur_num_ref_pics > 0, EVC_ERR_UNEXPECTED);

    for (int i = 0; i < MAX_NUM_REF_PICS; i++)
        refp[i][REFP_0].pic = refp[i][REFP_1].pic = NULL;
    pm->num_refp[REFP_0] = pm->num_refp[REFP_1] = 0;

    //Do the L0 first
    for (int i = 0; i < tgh->rpl_l0.ref_pic_active_num; i++)
    {
        int refPicPoc = tgh->poc - tgh->rpl_l0.ref_pics[i];
        //Find the ref pic in the DPB
        int j = 0;
        while (j < pm->cur_num_ref_pics && pm->pic_ref[j]->ptr != refPicPoc) j++;

        //If the ref pic is found, set it to RPL0
        if (j < pm->cur_num_ref_pics && pm->pic_ref[j]->ptr == refPicPoc)
        {
            set_refp(&refp[i][REFP_0], pm->pic_ref[j]);
            pm->num_refp[REFP_0] = pm->num_refp[REFP_0] + 1;
        }
        else
            return EVC_ERR;   //The refence picture must be available in the DPB, if not found then there is problem
    }

    if (tgh->tile_group_type == TILE_GROUP_P) return EVC_OK;

    //Do the L1 first
    for (int i = 0; i < tgh->rpl_l1.ref_pic_active_num; i++)
    {
        int refPicPoc = tgh->poc - tgh->rpl_l1.ref_pics[i];
        //Find the ref pic in the DPB
        int j = 0;
        while (j < pm->cur_num_ref_pics && pm->pic_ref[j]->ptr != refPicPoc) j++;

        //If the ref pic is found, set it to RPL1
        if (j < pm->cur_num_ref_pics && pm->pic_ref[j]->ptr == refPicPoc)
        {
            set_refp(&refp[i][REFP_1], pm->pic_ref[j]);
            pm->num_refp[REFP_1] = pm->num_refp[REFP_1] + 1;
        }
        else
            return EVC_ERR;   //The refence picture must be available in the DPB, if not found then there is problem
    }

    return EVC_OK;  //RPL construction completed
}

int evc_picman_refp_init(EVC_PM *pm, int num_ref_pics_act, int tile_group_type, u32 ptr, u8 layer_id, int last_intra, EVC_REFP(*refp)[REFP_NUM])
{
    int i, cnt;
    if(tile_group_type == TILE_GROUP_I)
    {
        return EVC_OK;
    }

    picman_update_pic_ref(pm);
    evc_assert_rv(pm->cur_num_ref_pics > 0, EVC_ERR_UNEXPECTED);

    for(i = 0; i < MAX_NUM_REF_PICS; i++)
    {
        refp[i][REFP_0].pic = refp[i][REFP_1].pic = NULL;
    }
    pm->num_refp[REFP_0] = pm->num_refp[REFP_1] = 0;

    /* forward */
    if(layer_id > 0)
    {
        if(tile_group_type == TILE_GROUP_P)
        {
            for(i = 0, cnt = 0; i < pm->cur_num_ref_pics && cnt < num_ref_pics_act; i++)
            {
                /* if(ptr >= last_intra && pm->pic_ref[i]->ptr < last_intra) continue; */
                if(layer_id == 1)
                {
                    if(pm->pic_ref[i]->ptr < ptr && pm->pic_ref[i]->layer_id <= layer_id)
                    {
                        set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                        cnt++;
                    }
                }
                else if(pm->pic_ref[i]->ptr < ptr && cnt == 0)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
                else if(cnt != 0 && pm->pic_ref[i]->ptr < ptr && \
                        pm->pic_ref[i]->layer_id <= 1)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
        else /* TILE_GROUP_B */
        {
            for(i = 0, cnt = 0; i < pm->cur_num_ref_pics && cnt < num_ref_pics_act; i++)
            {
                if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                if(pm->pic_ref[i]->ptr < ptr && pm->pic_ref[i]->layer_id <= layer_id)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
    }
    else /* layer_id == 0, non-scalable  */
    {
        for(i = 0, cnt = 0; i < pm->cur_num_ref_pics && cnt < num_ref_pics_act; i++)
        {
            if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
            if(pm->pic_ref[i]->ptr < ptr)
            {
                set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                cnt++;
            }
        }
    }

    if(cnt < num_ref_pics_act && tile_group_type == TILE_GROUP_B)
    {
        if(layer_id > 0)
        {
            for(i = pm->cur_num_ref_pics - 1; i >= 0 && cnt < num_ref_pics_act; i--)
            {
                if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                if(pm->pic_ref[i]->ptr > ptr && pm->pic_ref[i]->layer_id <= layer_id)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
        else
        {
            for(i = pm->cur_num_ref_pics - 1; i >= 0 && cnt < num_ref_pics_act; i--)
            {
                if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                if(pm->pic_ref[i]->ptr > ptr)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
    }

    evc_assert_rv(cnt > 0, EVC_ERR_UNEXPECTED);
    pm->num_refp[REFP_0] = cnt;

    /* backward */
    if(tile_group_type == TILE_GROUP_B)
    {
        if(layer_id > 0)
        {
            for(i = pm->cur_num_ref_pics - 1, cnt = 0; i >= 0 && cnt < num_ref_pics_act; i--)
            {
                if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                if(pm->pic_ref[i]->ptr > ptr && pm->pic_ref[i]->layer_id <= layer_id)
                {
                    set_refp(&refp[cnt][REFP_1], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
        else /* layer_id == 0, non-scalable  */
        {
            for(i = pm->cur_num_ref_pics - 1, cnt = 0; i >= 0 && cnt < num_ref_pics_act; i--)
            {
                if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                if(pm->pic_ref[i]->ptr > ptr)
                {
                    set_refp(&refp[cnt][REFP_1], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }

        if(cnt < num_ref_pics_act)
        {
            if(layer_id > 0)
            {
                for(i = 0; i < pm->cur_num_ref_pics && cnt < num_ref_pics_act; i++)
                {

                    if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                    if(pm->pic_ref[i]->ptr < ptr && pm->pic_ref[i]->layer_id <= layer_id)
                    {
                        set_refp(&refp[cnt][REFP_1], pm->pic_ref[i]);
                        cnt++;
                    }
                }
            }
            else
            {

                for(i = 0; i < pm->cur_num_ref_pics && cnt < num_ref_pics_act; i++)
                {
                    if(ptr >= (u32)last_intra && pm->pic_ref[i]->ptr < (u32)last_intra) continue;
                    if(pm->pic_ref[i]->ptr < ptr)
                    {
                        set_refp(&refp[cnt][REFP_1], pm->pic_ref[i]);
                        cnt++;
                    }
                }
            }
        }

        evc_assert_rv(cnt > 0, EVC_ERR_UNEXPECTED);
        pm->num_refp[REFP_1] = cnt;
    }

    if(tile_group_type == TILE_GROUP_B)
    {
        pm->num_refp[REFP_0] = EVC_MIN(pm->num_refp[REFP_0], num_ref_pics_act);
        pm->num_refp[REFP_1] = EVC_MIN(pm->num_refp[REFP_1], num_ref_pics_act);
    }

    return EVC_OK;
}

EVC_PIC * evc_picman_get_empty_pic(EVC_PM * pm, int * err)
{
    int ret;
    EVC_PIC * pic = NULL;

    /* try to find empty picture buffer in list */
    ret = picman_get_empty_pic_from_list(pm);
    if(ret >= 0)
    {
        pic = picman_remove_pic_from_pb(pm, ret);
        goto END;
    }
    /* else if available, allocate picture buffer */
    pm->cur_pb_size = picman_get_num_allocated_pics(pm);

    if(pm->cur_pb_size < pm->max_pb_size)
    {
        /* create picture buffer */
        pic = pm->pa.fn_alloc(&pm->pa, &ret);
        evc_assert_gv(pic != NULL, ret, EVC_ERR_OUT_OF_MEMORY, ERR);

        goto END;
    }
    evc_assert_gv(0, ret, EVC_ERR_UNKNOWN, ERR);

END:
    pm->pic_lease = pic;
    if(err) *err = EVC_OK;
    return pic;

ERR:
    if(err) *err = ret;
    if(pic) pm->pa.fn_free(&pm->pa, pic);
    return NULL;
}

/*This is the implementation of reference picture marking based on RPL*/
int evc_picman_refpic_marking(EVC_PM *pm, EVC_TGH *tgh)
{
    picman_update_pic_ref(pm);
    if (tgh->tile_group_type != TILE_GROUP_I && tgh->poc != 0)
        evc_assert_rv(pm->cur_num_ref_pics > 0, EVC_ERR_UNEXPECTED);

    EVC_PIC * pic;
    int numberOfPicsToCheck = pm->cur_num_ref_pics;
    for (int i = 0; i < numberOfPicsToCheck; i++)
    {
        pic = pm->pic[i];
        if (pm->pic[i] && IS_REF(pm->pic[i]))
        {
            //If the pic in the DPB is a reference picture, check if this pic is included in RPL0
            int isIncludedInRPL = 0;
            int j = 0;
            while (!isIncludedInRPL && j < tgh->rpl_l0.ref_pic_num)
            {
                if (pic->ptr == (tgh->poc - tgh->rpl_l0.ref_pics[j]))  //NOTE: we need to put POC also in EVC_PIC
                {
                    isIncludedInRPL = 1;
                }
                j++;
            }
            //Check if the pic is included in RPL1. This while loop will be executed only if the ref pic is not included in RPL0
            j = 0;
            while (!isIncludedInRPL && j < tgh->rpl_l1.ref_pic_num)
            {
                if (pic->ptr == (tgh->poc - tgh->rpl_l1.ref_pics[j]))
                {
                    isIncludedInRPL = 1;
                }
                j++;
            }
            //If the ref pic is not included in either RPL0 nor RPL1, then mark it as not used for reference. move it to the end of DPB.
            if (!isIncludedInRPL)
            {
                SET_REF_UNMARK(pic);
                picman_move_pic(pm, i, MAX_PB_SIZE - 1);
                pm->cur_num_ref_pics--;
                i--;                                           //We need to decrement i here because it will be increment by i++ at for loop. We want to keep the same i here because after the move, the current ref pic at i position is the i+1 position which we still need to check.
                numberOfPicsToCheck--;                         //We also need to decrement this variable to avoid checking the moved ref picture twice.
            }
        }
    }
    return EVC_OK;
}

int evc_picman_put_pic(EVC_PM * pm, EVC_PIC * pic, int tile_group_type,
                        u32 ptr, u32 dtr, u8 layer_id, int need_for_output,
                        EVC_REFP(*refp)[REFP_NUM], EVC_MMCO * mmco, int pnpf)
{
    /* manage RPB */
    if(pm->use_closed_gop && tile_group_type == TILE_GROUP_I)
    {
        picman_flush_pb(pm);
    }
//When RPL approach is used, we don't apply sliding window
    else if(pnpf)
    {
        picman_sliding_window(pm);
    }

    SET_REF_MARK(pic);

    if(mmco)
    {
        evc_assert(mmco->cnt == 1);
        evc_assert(mmco->type[0] == MMCO_UNUSED);
        evc_assert(mmco->data[0] == 0);
        SET_REF_UNMARK(pic);
    }

    pic->layer_id = layer_id;
    pic->ptr = ptr;
    pic->dtr = dtr;
    pic->need_for_out = need_for_output;

    /* put picture into listed RPB */
    if(IS_REF(pic))
    {
        picman_set_pic_to_pb(pm, pic, refp, pm->cur_num_ref_pics);
        pm->cur_num_ref_pics++;
    }
    else
    {
        picman_set_pic_to_pb(pm, pic, refp, -1);
    }

    if(pm->pic_lease == pic)
    {
        pm->pic_lease = NULL;
    }

    /*PRINT_DPB(pm);*/

    return EVC_OK;
}

EVC_PIC * evc_picman_out_pic(EVC_PM * pm, int * err)
{
    EVC_PIC ** ps;
    int i, ret, any_need_for_out = 0;

    ps = pm->pic;

    for(i = 0; i < MAX_PB_SIZE; i++)
    {
        if(ps[i] != NULL && ps[i]->need_for_out)
        {
            any_need_for_out = 1;

            if((ps[i]->ptr <= pm->ptr_next_output))
            {
                ps[i]->need_for_out = 0;
                pm->ptr_next_output = ps[i]->ptr + pm->ptr_increase;

                if(err) *err = EVC_OK;
                return ps[i];
            }
        }
    }
    if(any_need_for_out == 0)
    {
        ret = EVC_ERR_UNEXPECTED;
    }
    else
    {
        ret = EVC_OK_FRM_DELAYED;
    }

    if(err) *err = ret;
    return NULL;
}

int evc_picman_deinit(EVC_PM * pm)
{
    int i;

    /* remove allocated picture and picture store buffer */
    for(i = 0; i < MAX_PB_SIZE; i++)
    {
        if(pm->pic[i])
        {
            pm->pa.fn_free(&pm->pa, pm->pic[i]);
            pm->pic[i] = NULL;
        }
    }
    if(pm->pic_lease)
    {
        pm->pa.fn_free(&pm->pa, pm->pic_lease);
        pm->pic_lease = NULL;
    }
    return EVC_OK;
}

int evc_picman_init(EVC_PM * pm, int max_pb_size, int max_num_ref_pics,
                     int use_closed_gop, PICBUF_ALLOCATOR * pa)
{
    if(max_num_ref_pics > MAX_NUM_REF_PICS || max_pb_size > MAX_PB_SIZE)
    {
        return EVC_ERR_UNSUPPORTED;
    }
    pm->max_num_ref_pics = max_num_ref_pics;
    pm->max_pb_size = max_pb_size;
    pm->ptr_increase = 1;
    pm->use_closed_gop = use_closed_gop;
    pm->pic_lease = NULL;

    evc_mcpy(&pm->pa, pa, sizeof(PICBUF_ALLOCATOR));

    return EVC_OK;
}
