/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/pmu.h>
#include <drivers/optiflash.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

PMU_EventCfg gPmuEventCfg[3] =
{
    {
        .name = "No. Of CPU instructions executed",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_I_X,
    },
    {
        .name = "ICache Miss",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS,
    },
    {
        .name = "ICache Access",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_ACCESS,
    }
};

PMU_Config gPmuConfig =
{
    .bCycleCounter = TRUE,
    .numEventCounters = 3U,
    .eventCounters = gPmuEventCfg,
};

/*

    Aim of this exmaples to show the effect of RL2 or remote layer-2 cache. Remote because SRAM can be configured as L2 cache bank.

    To show how effective RL2 is, external flash is configured to be non-cached which stops load function to not being cached in L1 cache.
    This configuration is done from MPU's syscfg.

    Load function is also placed in external flash from linker.cmd file.
*/

#define NOP1() do{ __asm__ __volatile__("NOP"); }while(0)
#define NOP2() do{ NOP1(); NOP1(); }while(0)
#define NOP4() do{ NOP2(); NOP2(); }while(0)
#define NOP8() do{ NOP4(); NOP4(); }while(0)
#define NOP16() do{ NOP8(); NOP8(); }while(0)
#define NOP32() do{ NOP16(); NOP16(); }while(0)
#define NOP64() do{ NOP32(); NOP32(); }while(0)
#define NOP128() do{ NOP64(); NOP64(); }while(0)
#define NOP256() do{ NOP128(); NOP128(); }while(0)
#define NOP512() do{ NOP256(); NOP256(); }while(0)
#define NOP1024() do{ NOP512(); NOP512(); }while(0)
#define NOP2048() do{ NOP1024(); NOP1024(); }while(0)

void loadFunction ()
{
    NOP2048();
}

void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void rl2_main(void *args)
{
    #define max_iter 10
    uint32_t l2_miss[max_iter] = {0};
    uint32_t l2_hits[max_iter] = {0};

    Drivers_open();
    Board_driversOpen();

    PMU_init(&gPmuConfig);

    PMU_profileStart("With RL2");
    for(unsigned int i = 0; i < max_iter; i++)
    {
        uint32_t hits = 0, miss = 0;
        RL2_getCacheMiss(&(gRL2Config[0]), &(miss));
        RL2_getCacheHits(&(gRL2Config[0]), &(hits));
        loadFunction();
        RL2_getCacheMiss(&(gRL2Config[0]), &(l2_miss[i]));
        RL2_getCacheHits(&(gRL2Config[0]), &(l2_hits[i]));
        l2_miss[i] = l2_miss[i] - miss;
        l2_hits[i] = l2_hits[i] - hits;
    }

    PMU_profileEnd("With RL2");

    for(unsigned int i = 0; i < max_iter; i++)
    {
        DebugP_log("L2 Cache: \ti = %d \tMiss = %d\tHits = %d\r\n", i, l2_miss[i], l2_hits[i]);
    }

    RL2_disable(&(gRL2Config[0]));

    PMU_profileStart("Without RL2");
    for(int i = 0; i < max_iter; i++)
    {
        loadFunction();
    }
    PMU_profileEnd("Without RL2");

    PMU_profilePrint();

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
