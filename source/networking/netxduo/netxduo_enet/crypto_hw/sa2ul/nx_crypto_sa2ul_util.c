/*
 *  Copyright (c) Texas Instruments Incorporated 2025
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <tx_port.h>
#include <nx_crypto.h>

#include <security_common/drivers/crypto/crypto.h>
#include <security_common/drivers/crypto/sa2ul/sa2ul.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>

#include "nx_crypto_sa2ul.h"


void stream_to_big_number(const UCHAR t_stream[], size_t stream_length, uint32_t t_big_int[])
{
    size_t cur_stream_ix;
    size_t cur_big_int_ix;
    size_t word_cnt;

    word_cnt = (stream_length + 3u) / 4u;

    t_big_int[0] = word_cnt;

    cur_stream_ix = 0u;
    cur_big_int_ix = word_cnt;
    if (stream_length % 4 == 3) {
        t_big_int[cur_big_int_ix] = ((uint32_t)(t_stream[cur_stream_ix+2]) << 0u) | ((uint32_t)(t_stream[cur_stream_ix+1]) << 8u) | ((uint32_t)(t_stream[cur_stream_ix]) << 16u);
        cur_stream_ix += 3u;
        cur_big_int_ix--;
    } else if (stream_length % 4 == 2) {
        t_big_int[cur_big_int_ix] = ((uint32_t)(t_stream[cur_stream_ix+1]) << 0u) | ((uint32_t)(t_stream[cur_stream_ix]) << 8u);
        cur_stream_ix += 2u;
        cur_big_int_ix--;
    } else if (stream_length % 4 == 1) {
        t_big_int[cur_big_int_ix] = ((uint32_t)(t_stream[cur_stream_ix]) << 0u);
        cur_stream_ix += 1u;
        cur_big_int_ix--;
    }

    while (cur_stream_ix < stream_length) {
        DebugP_assert(cur_big_int_ix >= 1u);
        t_big_int[cur_big_int_ix] = ((uint32_t)(t_stream[cur_stream_ix+3]) << 0u) | ((uint32_t)(t_stream[cur_stream_ix+2]) << 8u) |
                                    ((uint32_t)(t_stream[cur_stream_ix+1]) << 16u) | ((uint32_t)(t_stream[cur_stream_ix]) << 24u);
        cur_stream_ix += 4u;
        cur_big_int_ix--;
    }
}


void big_number_to_stream(uint32_t t_big_num[], UCHAR t_stream[])
{
    size_t cur_big_num_ix;
    size_t cur_stream_ix;
    size_t word_cnt;

    word_cnt = t_big_num[0];
    cur_big_num_ix = word_cnt;
    cur_stream_ix = 0u;
    while (cur_big_num_ix > 0u) {
        for (size_t k = 4u; k > 0u; k--) {
            t_stream[cur_stream_ix] = t_big_num[cur_big_num_ix] >> ((k-1) * 8u);
            cur_stream_ix++;
        }
        cur_big_num_ix--;
    }
}
