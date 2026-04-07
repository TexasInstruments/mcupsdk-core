/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

#define ADDR_DDR_MEM_START  0x80000000UL
#define ADDR_DDR_MEM_MID    0xC0000000UL
#define ADDR_DDR_MEM_END    0xFFFFFFFFUL
/* the given test size below is used to loop over 2MB wrt writing 8 Bytes at a time */
#define DDR_TEST_SIZE       0x40000
#define DDR_END_TEST_ADDR   (ADDR_DDR_MEM_END - (DDR_TEST_SIZE << 3))
static const uint64_t gDdrMemtestPatterns[] = 
{
    0x00ULL, 
    0xffffffffffffffffULL,
    0x5555555555555555ULL,
    0xaaaaaaaaaaaaaaaaULL,
    0x1111111111111111ULL,
    0x2222222222222222ULL,
    0x4444444444444444ULL,
    0x8888888888888888ULL,
    0x3333333333333333ULL,
    0x6666666666666666ULL,
    0x9999999999999999ULL,
    0xccccccccccccccccULL,
    0x7777777777777777ULL,
    0xbbbbbbbbbbbbbbbbULL,
    0xddddddddddddddddULL,
    0xeeeeeeeeeeeeeeeeULL,
    0x7a6c7258554e494cULL,
};

int32_t ddr_read_write(uint64_t pattern, uint64_t *start_addr) 
{

    int32_t status = SystemP_SUCCESS;
    
    for(uint64_t i=0; i<DDR_TEST_SIZE; i++)
    {
        *(start_addr + i) = pattern;
    }
    for(uint64_t i=0; i<DDR_TEST_SIZE; i++) 
    {
        if(*(start_addr + i) != pattern)
        {
            status = SystemP_FAILURE;
            DebugP_logError("Failed at address: 0x%llx\r\n", (uint64_t)(start_addr + i));
            break;
        }
    }

    return status;

}

int32_t ddr_pattern_test(uint64_t *start_addr) 
{

    int32_t status = SystemP_SUCCESS;
    const uint32_t length = sizeof(gDdrMemtestPatterns) / sizeof(gDdrMemtestPatterns[0]);
    
    DebugP_log("\r\n Testing all patterns\r\n");
    for(uint32_t i = 0; i < length; i++)
    {
        DebugP_log("\r\n Testing for pattern: 0x%llx\r\n", gDdrMemtestPatterns[i]);
        status = ddr_read_write(gDdrMemtestPatterns[i], start_addr);
        if(status == SystemP_FAILURE)
        {
            DebugP_logError("\r\n Pattern: 0x%llx failed\r\n", gDdrMemtestPatterns[i]);
            break;
        }
        DebugP_log("\r\n Pattern: 0x%llx passed\r\n", gDdrMemtestPatterns[i]);
    }

    return status;
}

void ddr_memtest_main() {

    int32_t status = SystemP_SUCCESS;

    DebugP_log("\r\n START DDR Memtester\r\n");

    DebugP_log("\r\n Testing from start address: 0x%llx\r\n", (uint64_t)ADDR_DDR_MEM_START);
    status = ddr_pattern_test((uint64_t *)ADDR_DDR_MEM_START);
    if(status == SystemP_FAILURE)
    {
        DebugP_logError("\r\n DDR Memtester failed at start DDR address\r\n");
        return;
    }
    
    DebugP_log("\r\n Testing from mid address: 0x%llx\r\n", (uint64_t)ADDR_DDR_MEM_MID);
    status = ddr_pattern_test((uint64_t *)ADDR_DDR_MEM_MID);
    if(status == SystemP_FAILURE)
    {
        DebugP_logError("\r\n DDR Memtester failed at mid DDR address\r\n");
        return;
    }

    DebugP_log("\r\n Testing from end address: 0x%llx\r\n", (uint64_t)DDR_END_TEST_ADDR);
    status = ddr_pattern_test((uint64_t *)DDR_END_TEST_ADDR);
    if(status == SystemP_FAILURE)
    {
        DebugP_logError("\r\n DDR Memtester failed at end DDR address\r\n");
        return;
    }

    DebugP_log("All tests have passed!!\r\n");

}