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

 /** \file     evce_ibc_hashmap.cpp
     \brief    IBC hash map encoder class
 */


#include "evce_ibc_hashmap.h"
#include "evce_pibc.h"
#include "evce_ibc_hash_wrapper.h"

#define MIN_PU_SIZE 4

extern "C"
{
    ibc_hash_handle* create_enc_IBC(int picWidth, int picHeight)
    {
        EVCE_IBC_HashMap* p_hash = new EVCE_IBC_HashMap;
        p_hash->init(picWidth, picHeight);

        return (ibc_hash_handle *)p_hash;
    }

    void destroy_enc_IBC(ibc_hash_handle* p)
    {
        EVCE_IBC_HashMap* p_hash = (EVCE_IBC_HashMap*)p;
        p_hash->destroy();
        delete p_hash;
    }

    void rebuild_hashmap(ibc_hash_handle* p, const EVC_PIC* pic)
    {
        EVCE_IBC_HashMap* p_hash = (EVCE_IBC_HashMap*)p;

        p_hash->rebuildPicHashMap(pic);
    }

    u32 search_ibc_hash_match(EVCE_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y,
        int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D], EVCE_CORE * core)
    {
        u32 cost = 0;
        u32 min_cost = EVC_UINT32_MAX;
        EVCE_PIBC *pi = &ctx->pibc;
        EVCE_IBC_HashMap* p_hash = (EVCE_IBC_HashMap*)p;

        mvp[MV_X] = 0;
        mvp[MV_Y] = 0;

        mv[MV_X] = 0;
        mv[MV_Y] = 0;

        std::vector<Position> candPos;
        if (p_hash->ibcHashMatch(ctx, cu_x, cu_y, log2_cuw, log2_cuh, candPos,
            ctx->param.ibc_hash_search_max_cand, ctx->param.ibc_hash_search_range_4smallblk))
        {
            const unsigned int  lcuWidth = (1 << ctx->log2_max_cuwh);
            const int pic_width = ctx->w;
            const int pic_height = ctx->h;
            int roi_width = (1 << log2_cuw);
            int roi_height = (1 << log2_cuh);

            for (std::vector<Position>::iterator pos = candPos.begin(); pos != candPos.end(); pos++)
            {
                int ref_pos_LT_x_scu = PEL2SCU(pos->first);
                int ref_pos_LT_y_scu = PEL2SCU(pos->second);
                int ref_pos_LT_scup = (ref_pos_LT_y_scu * ctx->w_scu) + ref_pos_LT_x_scu;

                int avail_LT_cu = MCU_GET_COD(ctx->map_scu[ref_pos_LT_scup]);

                int ref_bottom_right_x = pos->first + roi_width - 1;
                int ref_bottom_right_y = pos->second + roi_height - 1;

                int ref_pos_BR_x_scu = PEL2SCU(ref_bottom_right_x);
                int ref_pos_BR_y_scu = PEL2SCU(ref_bottom_right_y);
                int ref_pos_BR_scup = (ref_pos_BR_y_scu * ctx->w_scu) + ref_pos_BR_x_scu;

                int avail_BR_cu = MCU_GET_COD(ctx->map_scu[ref_pos_BR_scup]);

                if (avail_LT_cu && avail_BR_cu)
                {
                    s16 cand_mv[MV_D];
                    cand_mv[MV_X] = pos->first - cu_x;
                    cand_mv[MV_Y] = pos->second - cu_y;

                    if (!is_bv_valid(ctx, cu_x, cu_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, cand_mv[0], cand_mv[1], lcuWidth, core))
                    {
                        continue;
                    }

                    int mv_bits = get_bv_cost_bits(cand_mv[MV_X], cand_mv[MV_Y]);
                    cost = GET_MV_COST(ctx, mv_bits);

                    if (cost < min_cost)
                    {
                        mv[0] = cand_mv[0];
                        mv[1] = cand_mv[1];
                        min_cost = cost;
                    }
                }
            }
        }

        return min_cost;
    }

    int get_hash_hit_ratio(EVCE_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y, int log2_cuw, int log2_cuh)
    {
        int pic_width = ctx->w;
        int pic_height = ctx->h;
        int roi_width = (1 << log2_cuw);
        int roi_height = (1 << log2_cuh);

        EVCE_IBC_HashMap* p_hash = (EVCE_IBC_HashMap*)p;

        int maxX = EVC_MIN((int)(cu_x + roi_width), pic_width);
        int maxY = EVC_MIN((int)(cu_y + roi_height), pic_height);
        int hit = 0, total = 0;
        for (int y = cu_y; y < maxY; y += MIN_PU_SIZE)
        {
            for (int x = cu_x; x < maxX; x += MIN_PU_SIZE)
            {
                const unsigned int hash = p_hash->m_pos2Hash[y][x];
                hit += (p_hash->m_hash2Pos[hash].size() > 1);
                total++;
            }
        }
        return 100 * hit / total;

        return 0;
    }

} //<-- extern "C"

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EVCE_IBC_HashMap::EVCE_IBC_HashMap()
{
    m_picWidth = 0;
    m_picHeight = 0;
    m_pos2Hash = NULL;
    m_computeCrc32c = xxComputeCrc32c16bit;
}

EVCE_IBC_HashMap::~EVCE_IBC_HashMap()
{
    destroy();
}

void EVCE_IBC_HashMap::init(const int picWidth, const int picHeight)
{
    if (picWidth != m_picWidth || picHeight != m_picHeight)
    {
        destroy();
    }

    m_picWidth = picWidth;
    m_picHeight = picHeight;
    m_pos2Hash = new unsigned int*[m_picHeight];
    m_pos2Hash[0] = new unsigned int[m_picWidth * m_picHeight];
    for (int n = 1; n < m_picHeight; n++)
    {
        m_pos2Hash[n] = m_pos2Hash[n - 1] + m_picWidth;
    }
}

void EVCE_IBC_HashMap::destroy()
{
    if (m_pos2Hash != NULL)
    {
        if (m_pos2Hash[0] != NULL)
        {
            delete[] m_pos2Hash[0];
        }
        delete[] m_pos2Hash;
    }
    m_pos2Hash = NULL;
}
////////////////////////////////////////////////////////
// CRC32C calculation in C code, same results as SSE 4.2's implementation
static const uint32_t crc32Table[256] = {
  0x00000000L, 0xF26B8303L, 0xE13B70F7L, 0x1350F3F4L,
  0xC79A971FL, 0x35F1141CL, 0x26A1E7E8L, 0xD4CA64EBL,
  0x8AD958CFL, 0x78B2DBCCL, 0x6BE22838L, 0x9989AB3BL,
  0x4D43CFD0L, 0xBF284CD3L, 0xAC78BF27L, 0x5E133C24L,
  0x105EC76FL, 0xE235446CL, 0xF165B798L, 0x030E349BL,
  0xD7C45070L, 0x25AFD373L, 0x36FF2087L, 0xC494A384L,
  0x9A879FA0L, 0x68EC1CA3L, 0x7BBCEF57L, 0x89D76C54L,
  0x5D1D08BFL, 0xAF768BBCL, 0xBC267848L, 0x4E4DFB4BL,
  0x20BD8EDEL, 0xD2D60DDDL, 0xC186FE29L, 0x33ED7D2AL,
  0xE72719C1L, 0x154C9AC2L, 0x061C6936L, 0xF477EA35L,
  0xAA64D611L, 0x580F5512L, 0x4B5FA6E6L, 0xB93425E5L,
  0x6DFE410EL, 0x9F95C20DL, 0x8CC531F9L, 0x7EAEB2FAL,
  0x30E349B1L, 0xC288CAB2L, 0xD1D83946L, 0x23B3BA45L,
  0xF779DEAEL, 0x05125DADL, 0x1642AE59L, 0xE4292D5AL,
  0xBA3A117EL, 0x4851927DL, 0x5B016189L, 0xA96AE28AL,
  0x7DA08661L, 0x8FCB0562L, 0x9C9BF696L, 0x6EF07595L,
  0x417B1DBCL, 0xB3109EBFL, 0xA0406D4BL, 0x522BEE48L,
  0x86E18AA3L, 0x748A09A0L, 0x67DAFA54L, 0x95B17957L,
  0xCBA24573L, 0x39C9C670L, 0x2A993584L, 0xD8F2B687L,
  0x0C38D26CL, 0xFE53516FL, 0xED03A29BL, 0x1F682198L,
  0x5125DAD3L, 0xA34E59D0L, 0xB01EAA24L, 0x42752927L,
  0x96BF4DCCL, 0x64D4CECFL, 0x77843D3BL, 0x85EFBE38L,
  0xDBFC821CL, 0x2997011FL, 0x3AC7F2EBL, 0xC8AC71E8L,
  0x1C661503L, 0xEE0D9600L, 0xFD5D65F4L, 0x0F36E6F7L,
  0x61C69362L, 0x93AD1061L, 0x80FDE395L, 0x72966096L,
  0xA65C047DL, 0x5437877EL, 0x4767748AL, 0xB50CF789L,
  0xEB1FCBADL, 0x197448AEL, 0x0A24BB5AL, 0xF84F3859L,
  0x2C855CB2L, 0xDEEEDFB1L, 0xCDBE2C45L, 0x3FD5AF46L,
  0x7198540DL, 0x83F3D70EL, 0x90A324FAL, 0x62C8A7F9L,
  0xB602C312L, 0x44694011L, 0x5739B3E5L, 0xA55230E6L,
  0xFB410CC2L, 0x092A8FC1L, 0x1A7A7C35L, 0xE811FF36L,
  0x3CDB9BDDL, 0xCEB018DEL, 0xDDE0EB2AL, 0x2F8B6829L,
  0x82F63B78L, 0x709DB87BL, 0x63CD4B8FL, 0x91A6C88CL,
  0x456CAC67L, 0xB7072F64L, 0xA457DC90L, 0x563C5F93L,
  0x082F63B7L, 0xFA44E0B4L, 0xE9141340L, 0x1B7F9043L,
  0xCFB5F4A8L, 0x3DDE77ABL, 0x2E8E845FL, 0xDCE5075CL,
  0x92A8FC17L, 0x60C37F14L, 0x73938CE0L, 0x81F80FE3L,
  0x55326B08L, 0xA759E80BL, 0xB4091BFFL, 0x466298FCL,
  0x1871A4D8L, 0xEA1A27DBL, 0xF94AD42FL, 0x0B21572CL,
  0xDFEB33C7L, 0x2D80B0C4L, 0x3ED04330L, 0xCCBBC033L,
  0xA24BB5A6L, 0x502036A5L, 0x4370C551L, 0xB11B4652L,
  0x65D122B9L, 0x97BAA1BAL, 0x84EA524EL, 0x7681D14DL,
  0x2892ED69L, 0xDAF96E6AL, 0xC9A99D9EL, 0x3BC21E9DL,
  0xEF087A76L, 0x1D63F975L, 0x0E330A81L, 0xFC588982L,
  0xB21572C9L, 0x407EF1CAL, 0x532E023EL, 0xA145813DL,
  0x758FE5D6L, 0x87E466D5L, 0x94B49521L, 0x66DF1622L,
  0x38CC2A06L, 0xCAA7A905L, 0xD9F75AF1L, 0x2B9CD9F2L,
  0xFF56BD19L, 0x0D3D3E1AL, 0x1E6DCDEEL, 0xEC064EEDL,
  0xC38D26C4L, 0x31E6A5C7L, 0x22B65633L, 0xD0DDD530L,
  0x0417B1DBL, 0xF67C32D8L, 0xE52CC12CL, 0x1747422FL,
  0x49547E0BL, 0xBB3FFD08L, 0xA86F0EFCL, 0x5A048DFFL,
  0x8ECEE914L, 0x7CA56A17L, 0x6FF599E3L, 0x9D9E1AE0L,
  0xD3D3E1ABL, 0x21B862A8L, 0x32E8915CL, 0xC083125FL,
  0x144976B4L, 0xE622F5B7L, 0xF5720643L, 0x07198540L,
  0x590AB964L, 0xAB613A67L, 0xB831C993L, 0x4A5A4A90L,
  0x9E902E7BL, 0x6CFBAD78L, 0x7FAB5E8CL, 0x8DC0DD8FL,
  0xE330A81AL, 0x115B2B19L, 0x020BD8EDL, 0xF0605BEEL,
  0x24AA3F05L, 0xD6C1BC06L, 0xC5914FF2L, 0x37FACCF1L,
  0x69E9F0D5L, 0x9B8273D6L, 0x88D28022L, 0x7AB90321L,
  0xAE7367CAL, 0x5C18E4C9L, 0x4F48173DL, 0xBD23943EL,
  0xF36E6F75L, 0x0105EC76L, 0x12551F82L, 0xE03E9C81L,
  0x34F4F86AL, 0xC69F7B69L, 0xD5CF889DL, 0x27A40B9EL,
  0x79B737BAL, 0x8BDCB4B9L, 0x988C474DL, 0x6AE7C44EL,
  0xBE2DA0A5L, 0x4C4623A6L, 0x5F16D052L, 0xAD7D5351L
};

uint32_t EVCE_IBC_HashMap::xxComputeCrc32c16bit(uint32_t crc, const pel pel)
{
    const void *buf = &pel;
    const uint8_t *p = (const uint8_t *)buf;
    size_t size = 2;

    while (size--)
    {
        crc = crc32Table[(crc ^ *p++) & 0xff] ^ (crc >> 8);
    }

    return crc;
}
// CRC calculation in C code
////////////////////////////////////////////////////////

unsigned int EVCE_IBC_HashMap::xxCalcBlockHash(const pel* pel, const int stride, const int width, const int height, unsigned int crc)
{
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            crc = m_computeCrc32c(crc, pel[x]);
        }
        pel += stride;
    }
    return crc;
}

void EVCE_IBC_HashMap::xxBuildPicHashMap(const EVC_PIC* pic)
{
    int y_stride = 0;
    int c_stride = 0;
    int width = 0;
    int height = 0;
    const int chromaScalingX = 1;
    const int chromaScalingY = 1;
    const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
    const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
    const pel* pelY = NULL;
    const pel* pelCb = NULL;
    const pel* pelCr = NULL;

    width = pic->w_l;
    height = pic->h_l;

    y_stride = pic->s_l;
    c_stride = pic->s_c;

    Position pos;
    for (pos.second = 0; pos.second + MIN_PU_SIZE <= height; pos.second++)
    {
        // row pointer
        pelY = pic->y + pos.second * y_stride;

        int chromaY = pos.second >> chromaScalingY;
        pelCb = pic->u + chromaY * c_stride;
        pelCr = pic->v + chromaY * c_stride;

        for (pos.first = 0; pos.first + MIN_PU_SIZE <= width; pos.first++)
        {
            // 0x1FF is just an initial value
            unsigned int hashValue = 0x1FF;

            // luma part
            hashValue = xxCalcBlockHash(&pelY[pos.first], y_stride, MIN_PU_SIZE, MIN_PU_SIZE, hashValue);

            // chroma part
            int chromaX = pos.first >> chromaScalingX;
            hashValue = xxCalcBlockHash(&pelCb[chromaX], c_stride, chromaMinBlkWidth, chromaMinBlkHeight, hashValue);
            hashValue = xxCalcBlockHash(&pelCr[chromaX], c_stride, chromaMinBlkWidth, chromaMinBlkHeight, hashValue);

            // hash table
            m_hash2Pos[hashValue].push_back(pos);
            m_pos2Hash[pos.second][pos.first] = hashValue;
        }
    }
}

void EVCE_IBC_HashMap::rebuildPicHashMap(const EVC_PIC* pic)
{
    m_hash2Pos.clear();

    xxBuildPicHashMap(pic);
}

bool EVCE_IBC_HashMap::ibcHashMatch(EVCE_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    std::vector<Position>& cand, const int maxCand, const int searchRange4SmallBlk)
{
    int cuw = (1 << log2_cuw);
    int cuh = (1 << log2_cuh);

    cand.clear();

    // find the block with least candidates
    size_t minSize = EVC_UINT32_MAX;
    unsigned int targetHashOneBlock = 0;
#if 0
    for (int y = 0; y < cuh && minSize > 1; y += MIN_PU_SIZE)
    {
        for (int x = 0; x < cuw && minSize > 1; x += MIN_PU_SIZE)
        {
            unsigned int hash = m_pos2Hash[cu_y + y][cu_x + x];
            if (m_hash2Pos[hash].size() < minSize)
            {
                minSize = m_hash2Pos[hash].size();
                targetHashOneBlock = hash;
            }
        }
    }
#else
    targetHashOneBlock = m_pos2Hash[cu_y][cu_x];
#endif

    if (m_hash2Pos[targetHashOneBlock].size() > 1)
    {
        std::vector<Position>& candOneBlock = m_hash2Pos[targetHashOneBlock];

        // check whether whole block match
        for (std::vector<Position>::iterator refBlockPos = candOneBlock.begin(); refBlockPos != candOneBlock.end(); refBlockPos++)
        {
            int offset_BR_x = refBlockPos->first + cuw - 1;
            int offset_BR_y = refBlockPos->second + cuh - 1;
            int offset_x_scu = PEL2SCU(offset_BR_x);
            int offset_y_scu = PEL2SCU(offset_BR_y);
            int offset_scup = (offset_y_scu * ctx->w_scu) + offset_x_scu;

            int avail_cu = MCU_GET_COD(ctx->map_scu[offset_scup]);

            bool wholeBlockMatch = true;
            if (cuw > MIN_PU_SIZE || cuh > MIN_PU_SIZE)
            {
                if (!avail_cu || offset_BR_x >= m_picWidth || offset_BR_y >= m_picHeight)
                {
                    continue;
                }
                for (int y = 0; y < cuh && wholeBlockMatch; y += MIN_PU_SIZE)
                {
                    for (int x = 0; x < cuw && wholeBlockMatch; x += MIN_PU_SIZE)
                    {
                        // whether the reference block and current block has the same hash
                        wholeBlockMatch &= (m_pos2Hash[cu_y + y][cu_x + x] == m_pos2Hash[refBlockPos->second + y][refBlockPos->first + x]);
                    }
                }
            }
            else
            {
                if (abs(refBlockPos->first - cu_x) > searchRange4SmallBlk || abs(refBlockPos->second - cu_y) > searchRange4SmallBlk || !avail_cu)
                {
                    continue;
                }
            }
            if (wholeBlockMatch)
            {
                cand.push_back(*refBlockPos);
                if (cand.size() > maxCand)
                {
                    break;
                }
            }
        }
    }

    return cand.size() > 0;
}

int EVCE_IBC_HashMap::getHashHitRatio(EVCE_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh)
{
    int cuw = (1 << log2_cuw);
    int cuh = (1 << log2_cuh);

    int maxX = EVC_MIN((int)(cu_x + cuw), m_picWidth);
    int maxY = EVC_MIN((int)(cu_y + cuh), m_picHeight);
    int hit = 0, total = 0;
    for (int y = cu_y; y < maxY; y += MIN_PU_SIZE)
    {
        for (int x = cu_x; x < maxX; x += MIN_PU_SIZE)
        {
            const unsigned int hash = m_pos2Hash[y][x];
            hit += (m_hash2Pos[hash].size() > 1);
            total++;
        }
    }
    return 100 * hit / total;
}
