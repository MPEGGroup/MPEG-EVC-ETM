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

static void pic_marking_no_rpl(EVC_PM * pm, int ref_pic_gap_length)
{
    int i;
    EVC_PIC * pic;

    // mark all pics with layer id > 0 as unused for reference
    for(i = 0; i < MAX_PB_SIZE; i++) /* this is coding order */
    {
        if(pm->pic[i] && IS_REF(pm->pic[i]) &&
            (pm->pic[i]->temporal_id > 0 || (i > 0 && ref_pic_gap_length > 0 && pm->pic[i]->poc % ref_pic_gap_length != 0)))
        {
            pic = pm->pic[i];

            /* unmark for reference */
            SET_REF_UNMARK(pic);
            picman_move_pic(pm, i, MAX_PB_SIZE - 1);

            if(pm->cur_num_ref_pics > 0)
            {
                pm->cur_num_ref_pics--;
            }
            i--;
        }
    }
    while(pm->cur_num_ref_pics >= MAX_NUM_ACTIVE_REF_FRAME) // TODO: change to signalled num ref pics
    {
        for(i = 0; i < MAX_PB_SIZE; i++) /* this is coding order */
        {
            if(pm->pic[i] && IS_REF(pm->pic[i]) )
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

    /* descending order sort based on POC */
    for(i = 0; i < cnt - 1; i++)
    {
        for(j = i + 1; j < cnt; j++)
        {
            if(pic_ref[i]->poc < pic_ref[j]->poc)
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
        pic->list_poc[i] = refp[i][REFP_0].poc;

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
            if (1 == imgb->getref(imgb))
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
    refp->poc      = pic_ref->poc;
    refp->map_mv   = pic_ref->map_mv;
#if DMVR_LAG
    refp->map_unrefined_mv = pic_ref->map_mv;
#endif
    refp->map_refi = pic_ref->map_refi;
    refp->list_poc = pic_ref->list_poc;
}

void copy_refp(EVC_REFP * refp_dst, EVC_REFP * refp_src)
{
    refp_dst->pic      = refp_src->pic;
    refp_dst->poc      = refp_src->poc;
    refp_dst->map_mv   = refp_src->map_mv;
#if DMVR_LAG
    refp_dst->map_unrefined_mv = refp_src->map_mv;
#endif
    refp_dst->map_refi = refp_src->map_refi;
    refp_dst->list_poc = refp_src->list_poc;
}

int check_copy_refp(EVC_REFP(*refp)[REFP_NUM], int cnt, int lidx, EVC_REFP  * refp_src)
{
    int i;

    for(i = 0; i < cnt; i++)
    {
        if(refp[i][lidx].poc == refp_src->poc)
        {
            return -1;
        }
    }
    copy_refp(&refp[cnt][lidx], refp_src);

    return EVC_OK;
}

//This is implementation of reference picture list construction based on RPL
int evc_picman_refp_rpl_based_init(EVC_PM *pm, EVC_SH *sh, int poc_val, EVC_REFP(*refp)[REFP_NUM])
{
    if (sh->slice_type == SLICE_I)
    {
        return EVC_OK;
    }

    picman_update_pic_ref(pm);
    evc_assert_rv(pm->cur_num_ref_pics > 0, EVC_ERR_UNEXPECTED);

    for (int i = 0; i < MAX_NUM_REF_PICS; i++)
        refp[i][REFP_0].pic = refp[i][REFP_1].pic = NULL;
    pm->num_refp[REFP_0] = pm->num_refp[REFP_1] = 0;

    //Do the L0 first
    for (int i = 0; i < sh->rpl_l0.ref_pic_active_num; i++)
    {
        int refPicPoc = poc_val - sh->rpl_l0.ref_pics[i];
        //Find the ref pic in the DPB
        int j = 0;
        while (j < pm->cur_num_ref_pics && pm->pic_ref[j]->poc != refPicPoc) j++;

        //If the ref pic is found, set it to RPL0
        if (j < pm->cur_num_ref_pics && pm->pic_ref[j]->poc == refPicPoc)
        {
            set_refp(&refp[i][REFP_0], pm->pic_ref[j]);
            pm->num_refp[REFP_0] = pm->num_refp[REFP_0] + 1;
        }
        else
            return EVC_ERR;   //The refence picture must be available in the DPB, if not found then there is problem
    }

    if (sh->slice_type == SLICE_P) return EVC_OK;

    //Do the L1 first
    for (int i = 0; i < sh->rpl_l1.ref_pic_active_num; i++)
    {
        int refPicPoc = poc_val - sh->rpl_l1.ref_pics[i];
        //Find the ref pic in the DPB
        int j = 0;
        while (j < pm->cur_num_ref_pics && pm->pic_ref[j]->poc != refPicPoc) j++;

        //If the ref pic is found, set it to RPL1
        if (j < pm->cur_num_ref_pics && pm->pic_ref[j]->poc == refPicPoc)
        {
            set_refp(&refp[i][REFP_1], pm->pic_ref[j]);
            pm->num_refp[REFP_1] = pm->num_refp[REFP_1] + 1;
        }
        else
            return EVC_ERR;   //The refence picture must be available in the DPB, if not found then there is problem
    }

    return EVC_OK;  //RPL construction completed
}

int evc_picman_refp_init(EVC_PM *pm, int max_num_ref_pics, int slice_type, u32 poc, u8 layer_id, int last_intra, EVC_REFP(*refp)[REFP_NUM])
{
    int i, cnt;
    if(slice_type == SLICE_I)
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
    if(slice_type == SLICE_P)
    {
        if(layer_id > 0)
        {
            for(i = 0, cnt = 0; i < pm->cur_num_ref_pics && cnt < max_num_ref_pics; i++)
            {
                /* if(poc >= last_intra && pm->pic_ref[i]->poc < last_intra) continue; */
                if(layer_id == 1)
                {
                    if(pm->pic_ref[i]->poc < poc && pm->pic_ref[i]->temporal_id <= layer_id)
                    {
                        set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                        cnt++;
                    }
                }
                else if(pm->pic_ref[i]->poc < poc && cnt == 0)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
                else if(cnt != 0 && pm->pic_ref[i]->poc < poc && \
                          pm->pic_ref[i]->temporal_id <= 1)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
        else /* layer_id == 0, non-scalable  */
        {
            for(i = 0, cnt = 0; i < pm->cur_num_ref_pics && cnt < max_num_ref_pics; i++)
            {
                if(poc >= (u32)last_intra && pm->pic_ref[i]->poc < (u32)last_intra) continue;
                if(pm->pic_ref[i]->poc < poc)
                {
                    set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                    cnt++;
                }
            }
        }
    }
    else /* SLICE_B */
    {
        int next_layer_id = EVC_MAX(layer_id - 1, 0);
        for(i = 0, cnt = 0; i < pm->cur_num_ref_pics && cnt < max_num_ref_pics; i++)
        {
            if(poc >= (u32)last_intra && pm->pic_ref[i]->poc < (u32)last_intra) continue;
            if(pm->pic_ref[i]->poc < poc && pm->pic_ref[i]->temporal_id <= next_layer_id)
            {
                set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                cnt++;
                next_layer_id = EVC_MAX(pm->pic_ref[i]->temporal_id - 1, 0);
            }
        }
    }

    if(cnt < max_num_ref_pics && slice_type == SLICE_B)
    {
        int next_layer_id = EVC_MAX(layer_id - 1, 0);
        for(i = pm->cur_num_ref_pics - 1; i >= 0 && cnt < max_num_ref_pics; i--)
        {
            if(poc >= (u32)last_intra && pm->pic_ref[i]->poc < (u32)last_intra) continue;
            if(pm->pic_ref[i]->poc > poc && pm->pic_ref[i]->temporal_id <= next_layer_id)
            {
                set_refp(&refp[cnt][REFP_0], pm->pic_ref[i]);
                cnt++;
                next_layer_id = EVC_MAX(pm->pic_ref[i]->temporal_id - 1, 0);
            }
        }
    }

    evc_assert_rv(cnt > 0, EVC_ERR_UNEXPECTED);
    pm->num_refp[REFP_0] = cnt;

    /* backward */
    if(slice_type == SLICE_B)
    {
        int next_layer_id = EVC_MAX(layer_id - 1, 0);
        for(i = pm->cur_num_ref_pics - 1, cnt = 0; i >= 0 && cnt < max_num_ref_pics; i--)
        {
            if(poc >= (u32)last_intra && pm->pic_ref[i]->poc < (u32)last_intra) continue;
            if(pm->pic_ref[i]->poc > poc && pm->pic_ref[i]->temporal_id <= next_layer_id)
            {
                set_refp(&refp[cnt][REFP_1], pm->pic_ref[i]);
                cnt++;
                next_layer_id = EVC_MAX(pm->pic_ref[i]->temporal_id - 1, 0);
            }
        }

        if(cnt < max_num_ref_pics)
        {
            next_layer_id = EVC_MAX(layer_id - 1, 0);
            for(i = 0; i < pm->cur_num_ref_pics && cnt < max_num_ref_pics; i++)
            {

                if(poc >= (u32)last_intra && pm->pic_ref[i]->poc < (u32)last_intra) continue;
                if(pm->pic_ref[i]->poc < poc && pm->pic_ref[i]->temporal_id <= next_layer_id)
                {
                    set_refp(&refp[cnt][REFP_1], pm->pic_ref[i]);
                    cnt++;
                    next_layer_id = EVC_MAX(pm->pic_ref[i]->temporal_id - 1, 0);
                }
            }
        }

        evc_assert_rv(cnt > 0, EVC_ERR_UNEXPECTED);
        pm->num_refp[REFP_1] = cnt;
    }

    if(slice_type == SLICE_B)
    {
        pm->num_refp[REFP_0] = EVC_MIN(pm->num_refp[REFP_0], max_num_ref_pics);
        pm->num_refp[REFP_1] = EVC_MIN(pm->num_refp[REFP_1], max_num_ref_pics);
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
int evc_picman_refpic_marking(EVC_PM *pm, EVC_SH *sh, int poc_val)
{
    picman_update_pic_ref(pm);
    if (sh->slice_type != SLICE_I && poc_val != 0)
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
            while (!isIncludedInRPL && j < sh->rpl_l0.ref_pic_num)
            {
                if (pic->poc == (poc_val - sh->rpl_l0.ref_pics[j]))  //NOTE: we need to put POC also in EVC_PIC
                {
                    isIncludedInRPL = 1;
                }
                j++;
            }
            //Check if the pic is included in RPL1. This while loop will be executed only if the ref pic is not included in RPL0
            j = 0;
            while (!isIncludedInRPL && j < sh->rpl_l1.ref_pic_num)
            {
                if (pic->poc == (poc_val - sh->rpl_l1.ref_pics[j]))
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

int evc_picman_put_pic(EVC_PM * pm, EVC_PIC * pic, int is_idr,
                        u32 poc, u8 temporal_id, int need_for_output,
                        EVC_REFP(*refp)[REFP_NUM], int ref_pic, int tool_rpl, int ref_pic_gap_length)
{
    /* manage RPB */
    if(is_idr)
    {
        picman_flush_pb(pm);
    }
    //Perform picture marking if RPL approach is not used
    else if(tool_rpl == 0)
    {
        if (temporal_id == 0)
        {
            pic_marking_no_rpl(pm, ref_pic_gap_length);
        }
    }

    SET_REF_MARK(pic);

    if(!ref_pic)
    {
        SET_REF_UNMARK(pic);
    }

    pic->temporal_id = temporal_id;
    pic->poc = poc;
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

            if((ps[i]->poc <= pm->poc_next_output))
            {
                ps[i]->need_for_out = 0;
                pm->poc_next_output = ps[i]->poc + pm->poc_increase;

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
                          PICBUF_ALLOCATOR * pa)
{
    if(max_num_ref_pics > MAX_NUM_REF_PICS || max_pb_size > MAX_PB_SIZE)
    {
        return EVC_ERR_UNSUPPORTED;
    }
    pm->max_num_ref_pics = max_num_ref_pics;
    pm->max_pb_size = max_pb_size;
    pm->poc_increase = 1;
    pm->pic_lease = NULL;

    evc_mcpy(&pm->pa, pa, sizeof(PICBUF_ALLOCATOR));

    return EVC_OK;
}
