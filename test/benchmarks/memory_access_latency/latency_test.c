/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "latency_test.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

benchmark_t latencyCalculate_Write32(uint32_t address, uint32_t address1)
{
    double startCount, stopCount;
    benchmark_t perf;

    uint32_t *writePtrAddr  = (uint32_t *)address;
    uint32_t *writePtrAddr1  = (uint32_t *)address1;

    CycleCounterP_reset();
    startCount = CycleCounterP_getCount32();

    /* Write 32 bytes of data */
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr) );
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr) );
    __asm__ __volatile__( "str r9, %[value]" : [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr) );
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr) );
    __asm__ __volatile__( "str r9, %[value]" : [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r9, %[value]" : [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r9, %[value]" : [value] "=m" (*writePtrAddr1));
    __asm__ __volatile__( "str r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "str r10, %[value]": [value] "=m" (*writePtrAddr1));

    stopCount = CycleCounterP_getCount32();
    perf.cycles = (stopCount - startCount)/(32*4);

    return perf;
}

benchmark_t latencyCalculate_Write64(uint32_t address)
{
    double startCount, stopCount;
    benchmark_t perf;

    uint64_t *writePtrAddr  = (uint64_t *)address;

    CycleCounterP_reset();
    startCount = CycleCounterP_getCount32();

    /* Write 64 bytes of data */
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));
    __asm__ __volatile__( "strd r10,r9, %[value]": [value] "=m" (*writePtrAddr));

    stopCount = CycleCounterP_getCount32();
    perf.cycles = (stopCount - startCount)/(32*8);

    return perf;
}

benchmark_t latencyCalculate_Read32(uint32_t address)
{
    double startCount = 0, stopCount = 0;
    benchmark_t perf;
    uint32_t *writePtrAddr = (uint32_t *)address;

    CycleCounterP_reset();
    startCount = CycleCounterP_getCount32();

    /* Read 32 bytes of data */
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldr r7, %[value]": : [value] "m" (*writePtrAddr) : "r7");

    stopCount = CycleCounterP_getCount32();
    perf.cycles = (stopCount - startCount)/(32*4);

    return perf;
}

benchmark_t latencyCalculate_Read64(uint32_t address)
{
    double startCount, stopCount;
    benchmark_t perf;
    uint64_t *writePtrAddr = (uint64_t *)address;

    CycleCounterP_reset();
    startCount = CycleCounterP_getCount32();

    /* Read 64 bytes of data */
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");
    __asm__ __volatile__( "ldrd r7, r8, %[value]": : [value] "m" (*writePtrAddr) : "r7");

    stopCount = CycleCounterP_getCount32();
    perf.cycles = (stopCount - startCount)/(32*8);

    return perf;
}

/*
 * This is a latency project provided to calculate the latency of
 * read and write access.
 */
void memory_latency_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("BENCHMARK START - ARM R5F - Memory Access latency\r\n\n");
    benchmark_t perf[14];

    /* 32 BIT Write access */
    perf[0] = latencyCalculate_Write32(SELF_TCM_WRITE_ADDRESS, SELF_TCM_WRITE_ADDRESS1);
    perf[1] = latencyCalculate_Write32(SRAM_WRITE_ADDRESS, SRAM_WRITE_ADDRESS1);
    perf[2] = latencyCalculate_Write32(NON_SELF_TCM_ACCESS_ADDRESS, NON_SELF_TCM_ACCESS_ADDRESS1);
    DebugP_log("[32-bit WRITE] Self TCM Access Latency: %f cycles/byte\r\n", perf[0].cycles);
    DebugP_log("[32-bit WRITE] L2OCRAM Access Latency: %f cycles/byte\r\n", perf[1].cycles);
    DebugP_log("[32-bit WRITE] Non Self TCM Access Latency: %f cycles/byte\r\n\n", perf[2].cycles);

    ClockP_sleep(1);

    /* 32 BIT Read access */
    perf[6] = latencyCalculate_Read32(SELF_TCM_READ_ADDRESS);
    perf[7] = latencyCalculate_Read32(SRAM_READ_ADDRESS);
    perf[8] = latencyCalculate_Read32(NON_SELF_TCM_ACCESS_ADDRESS);
    perf[9] = latencyCalculate_Read32(FLASH_ACCESS_ADDRESS);
    DebugP_log("[32-BIT READ] Self TCM Access Latency: %f cycles/byte\r\n", perf[6].cycles);
    DebugP_log("[32-BIT READ] L2OCRAM Access Latency: %f cycles/byte\r\n", perf[7].cycles);
    DebugP_log("[32-BIT READ] Non-Self TCM Access Latency: %f cycles/byte\r\n", perf[8].cycles);
    DebugP_log("[32-BIT READ] Flash (Memory Map Mode) Access Latency: %f cycles/byte\r\n\n", perf[9].cycles);

    ClockP_sleep(1);

    /* 64 BIT Write access */
    perf[3] = latencyCalculate_Write64(SELF_TCM_WRITE_ADDRESS);
    perf[4] = latencyCalculate_Write64(SRAM_WRITE_ADDRESS);
    perf[5] = latencyCalculate_Write64(NON_SELF_TCM_ACCESS_ADDRESS);
    DebugP_log("[64-bit WRITE] Self TCM Access Latency: %f cycles/byte\r\n", perf[3].cycles);
    DebugP_log("[64-bit WRITE] L2OCRAM Access Latency: %f cycles/byte\r\n", perf[4].cycles);
    DebugP_log("[64-bit WRITE] Non-Self TCM Access Latency: %f cycles/byte\r\n\n", perf[5].cycles);

    ClockP_sleep(1);

    /* 64 BIT Read access */
    perf[10] = latencyCalculate_Read64(SELF_TCM_READ_ADDRESS);
    perf[11] = latencyCalculate_Read64(SRAM_READ_ADDRESS);
    perf[12] = latencyCalculate_Read64(NON_SELF_TCM_ACCESS_ADDRESS);
    perf[13] = latencyCalculate_Read64(FLASH_ACCESS_ADDRESS);
    DebugP_log("[64-BIT READ] Self TCM Access Latency: %f cycles/byte\r\n", perf[10].cycles);
    DebugP_log("[64-BIT READ] L2OCRAM Access Read Latency: %f cycles/byte\r\n", perf[11].cycles);
    DebugP_log("[64-BIT READ] Non-Self TCM Access Read Latency: %f cycles/byte\r\n", perf[12].cycles);
    DebugP_log("[64-BIT READ] Flash (Memory Map Mode) Access Read Latency: %f cycles/byte\r\n\n", perf[13].cycles);

    DebugP_log("BENCHMARK END\r\n");

    Board_driversClose();
    Drivers_close();
}