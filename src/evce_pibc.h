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

#ifndef _EVCE_PIBC_H_
#define _EVCE_PIBC_H_

#include "evce_def.h"

#if IBC

#if ((defined WIN32) || (defined WIN64))
#define  IBC_INLINE __forceinline
#else
#define  IBC_INLINE __attribute__((always_inline)) inline
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define SHIFT_BITS 0

#define CONSTRAIN_ONE_EXTRA_CTU  1

#define MV_COST(pi, mv_bits) (u32)(((pi)->lambda_mv * mv_bits + (1 << 15)) >> 16)

int evce_pibc_create(EVCE_CTX * ctx, int complexity);

int get_ibc_mv_bits(int mvd_x, int mvd_y);

u32 get_bv_cost_bits(int mv_x, int mv_y);

#define GET_MV_COST(ctx, mv_bits)  ((u32)(ctx->sqrt_lambda[0] * mv_bits / 65536.0))

void reset_ibc_search_range(EVCE_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh);

// for ibc pu validation
IBC_INLINE int is_bv_valid(EVCE_CTX *ctx, int x, int y, int width, int height, int log2_cuw, int log2_cuh,
    int pic_width, int pic_height, int x_bv, int y_bv, int ctu_size)
{
    int x_scu = 0, y_scu = 0;
    int log2_scuw = 0, log2_scuh = 0;
    int scuw = 0, scuh = 0;

    x_scu = PEL2SCU(x);
    y_scu = PEL2SCU(y);

    log2_scuw = log2_cuw - MIN_CU_LOG2;
    log2_scuh = log2_cuh - MIN_CU_LOG2;
    scuw = 1 << log2_scuw;
    scuh = 1 << log2_scuh;

    const int ctu_size_log2 = ctx->pibc.ctu_log2_tbl[ctu_size];

    int ref_right_x = x + x_bv + width - 1;
    int ref_bottom_y = y + y_bv + height - 1;

    int ref_left_x = x + x_bv;
    int ref_top_y = y + y_bv;

    if ((x + x_bv) < 0)
    {
        return 0;
    }
    if (ref_right_x >= pic_width)
    {
        return 0;
    }

    if ((y + y_bv) < 0)
    {
        return 0;
    }
    if (ref_bottom_y >= pic_height)
    {
        return 0;
    }
    if ((x_bv + width) > 0 && (y_bv + height) > 0)
    {
        return 0;
    }

    // cannot be in the above CTU row
    if ((ref_top_y >> ctu_size_log2) < (y >> ctu_size_log2))
        return 0;

    // cannot be in the below CTU row
    if ((ref_bottom_y >> ctu_size_log2) > (y >> ctu_size_log2))
    {
        return 0;
    }

    // in the same CTU line
#if CONSTRAIN_ONE_EXTRA_CTU
    if (((ref_right_x >> ctu_size_log2) <= (x >> ctu_size_log2)) && ((ref_left_x >> ctu_size_log2) >= (x >> ctu_size_log2) - 1))
#else
    if (((ref_right_x >> ctu_size_log2) <= (x >> ctu_size_log2)) && ((ref_left_x >> ctu_size_log2) >= (x >> ctu_size_log2)))
#endif
    {
#if CONSTRAIN_ONE_EXTRA_CTU
        // in the same CTU, or left CTU
        // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
        if ((ref_left_x >> ctu_size_log2) == ((x >> ctu_size_log2) - 1))
        {
            // top left position of ref block's collocated block in current CTU
            int ref_pos_col_x = x + x_bv + ctu_size;
            int ref_pos_col_y = y + y_bv;
            int offset64x = (ref_pos_col_x >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
            int offset64y = (ref_pos_col_y >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
            int offset_x_scu = PEL2SCU(offset64x);
            int offset_y_scu = PEL2SCU(offset64y);
            int offset_scup = (offset_y_scu * ctx->w_scu) + offset_x_scu;

            int avail_cu = MCU_GET_COD(ctx->map_scu[offset_scup]);
            if (avail_cu)
            {
                return 0;
            }

            //corn case: start coding first block in 64x64 CU, then should disable ref 64x64 CU
            if (offset64x == x && offset64y == y)
            {
                return 0;
            }

#if SUCO
            if (ctx->sps.sps_suco_flag)
            {
              // top right position of ref block's collocated block in current CTU
              int offset64_TR_x = offset64x + (1 << (ctu_size_log2 - 1)) - 1;
              if (offset64_TR_x >= pic_width)
              {
                offset64_TR_x = pic_width - 1;
              }
              int offset64_TR_y = offset64y;
              int offset_TR_x_scu = PEL2SCU(offset64_TR_x);
              int offset_TR_y_scu = PEL2SCU(offset64_TR_y);
              int offset_TR_scup = (offset_TR_y_scu * ctx->w_scu) + offset_TR_x_scu;

              int avail_TR_cu = MCU_GET_COD(ctx->map_scu[offset_TR_scup]);
              if (avail_TR_cu)
              {
                return 0;
              }

              if (offset64_TR_x == (x + (1 << log2_cuw) - 1) && offset64_TR_y == y)
              {
                return 0;
              }


              //Check the collocated 64x64 region of the reference block¡¯s top-right corner is valid for reference or not
              int RT_ref_pos_LT_col_x = x + x_bv + ctu_size + width - 1;
              if (RT_ref_pos_LT_col_x < pic_width)
              {
                int RT_ref_pos_LT_offset64x = (RT_ref_pos_LT_col_x >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                int RT_ref_pos_LT_col_y = y + y_bv;
                int RT_ref_pos_LT_offset64y = (RT_ref_pos_LT_col_y >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                int RT_ref_pos_LT_x_scu = PEL2SCU(RT_ref_pos_LT_offset64x);
                int RT_ref_pos_LT_y_scu = PEL2SCU(RT_ref_pos_LT_offset64y);
                int RT_ref_pos_LT_scup = (RT_ref_pos_LT_y_scu * ctx->w_scu) + RT_ref_pos_LT_x_scu;

                int RT_ref_pos_LT_cu = MCU_GET_COD(ctx->map_scu[RT_ref_pos_LT_scup]);
                if (RT_ref_pos_LT_cu)
                {
                  return 0;
                }

                if (RT_ref_pos_LT_offset64x == (x + width - 1) && RT_ref_pos_LT_col_y == y)
                {
                  return 0;
                }

                int RT_ref_pos_RT_offset64x = RT_ref_pos_LT_offset64x + (1 << (ctu_size_log2 - 1)) - 1;
                int RT_ref_pos_RT_col_y = RT_ref_pos_LT_col_y;
                int RT_ref_pos_RT_offset64y = (RT_ref_pos_RT_col_y >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                int RT_ref_pos_RT_x_scu = PEL2SCU(RT_ref_pos_RT_offset64x);
                int RT_ref_pos_RT_y_scu = PEL2SCU(RT_ref_pos_RT_offset64y);
                int RT_ref_pos_RT_scup = (RT_ref_pos_RT_y_scu * ctx->w_scu) + RT_ref_pos_RT_x_scu;

                int RT_ref_pos_RT_cu = MCU_GET_COD(ctx->map_scu[RT_ref_pos_RT_scup]);
                if (RT_ref_pos_RT_cu)
                {
                  return 0;
                }

                if (RT_ref_pos_RT_offset64x == (x + width - 1) && RT_ref_pos_RT_offset64y == y)
                {
                  return 0;
                }
                  }
            }
#endif
        }
#else
        // ref block's collocated block in current CTU
        int ref_pos_LT_x = x + x_bv;
        int ref_pos_LT_y = y + y_bv;

        int offset_x_scu = PEL2SCU(ref_pos_LT_x);
        int offset_y_scu = PEL2SCU(ref_pos_LT_y);
        int offset_LT_scup = (offset_y_scu * ctx->w_scu) + offset_x_scu;

        int avail_cu = MCU_GET_COD(ctx->map_scu[offset_LT_scup]);
        if (!avail_cu)
        {
            return 0;
        }

        int ref_pos_BR_x = x + width - 1 + x_bv;
        int ref_pos_BR_y = y + height - 1 + y_bv;

        offset_x_scu = PEL2SCU(ref_pos_BR_x);
        offset_y_scu = PEL2SCU(ref_pos_BR_y);
        int offset_BR_scup = (offset_y_scu * ctx->w_scu) + offset_x_scu;

        avail_cu = MCU_GET_COD(ctx->map_scu[offset_BR_scup]);
        if (!avail_cu)
        {
            return 0;
        }

        return 1;
#endif
    }
    else
    {
        return 0;
    }

#if CONSTRAIN_ONE_EXTRA_CTU
    // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
    int ref_pos_LT_x = x + x_bv;
    int ref_pos_LT_y = y + y_bv;

    int ref_pos_LT_x_scu = PEL2SCU(ref_pos_LT_x);
    int ref_pos_LT_y_scu = PEL2SCU(ref_pos_LT_y);
    int ref_pos_LT_scup = (ref_pos_LT_y_scu * ctx->w_scu) + ref_pos_LT_x_scu;

    int avail_cu = MCU_GET_COD(ctx->map_scu[ref_pos_LT_scup]);
    if (avail_cu == 0)
    {
        return 0;
    }

    int ref_pos_BR_x = x + width - 1 + x_bv;
    int ref_pos_BR_y = y + height - 1 + y_bv;

    int ref_pos_BR_x_scu = PEL2SCU(ref_pos_BR_x);
    int ref_pos_BR_y_scu = PEL2SCU(ref_pos_BR_y);
    int ref_pos_BR_scup = (ref_pos_BR_y_scu * ctx->w_scu) + ref_pos_BR_x_scu;

    avail_cu = MCU_GET_COD(ctx->map_scu[ref_pos_BR_scup]);
    if (avail_cu == 0)
    {
        return 0;
    }

#if SUCO
    if (ctx->sps.sps_suco_flag)
    {
      // check the availablity of bottom-left corner
      int ref_pos_BL_scup = (ref_pos_BR_y_scu * ctx->w_scu) + ref_pos_LT_x_scu;
      avail_cu = MCU_GET_COD(ctx->map_scu[ref_pos_BL_scup]);
      if (avail_cu == 0)
      {
        return 0;
      }

      // check if the reference block cross the uncoded block
      if (ref_pos_BR_x >= x && ref_pos_BR_y < y)
      {
        int check_point_x = ref_pos_LT_x + width / 2;
        int check_point_y = ref_pos_BR_y;

        int check_point_x_scu = PEL2SCU(check_point_x);
        int check_point_y_scu = PEL2SCU(check_point_y);
        int check_point_scup = (check_point_y_scu * ctx->w_scu) + check_point_x_scu;

        avail_cu = MCU_GET_COD(ctx->map_scu[check_point_scup]);
        if (avail_cu == 0)
        {
          return 0;
        }
      }
    }
#endif

    return 1;
#endif
}

#ifdef __cplusplus
}
#endif

#endif

#endif /* _EVCE_PIBC_H_ */
