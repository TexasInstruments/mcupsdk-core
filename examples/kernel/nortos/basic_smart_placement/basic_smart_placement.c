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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

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


void function_f2(void)
{
    NOP1024();
}

void function_f3(void)
{
    NOP2048();
}

void function_f4(void)
{
    NOP512();
}

void function_f1(void)
{
    function_f2();
    function_f3();
    function_f4();
}

void __attribute__((local(2))) annotated_function_f2(void)
{
    NOP1024();
}

void __attribute__((local(2))) annotated_function_f3(void)
{
    NOP2048();
}

void __attribute__((local(2))) annotated_function_f4(void)
{
    NOP512();
}

void __attribute__((local(1))) annotated_function_f1(void)
{
    annotated_function_f2();
    annotated_function_f3();
    annotated_function_f4();
}

void basic_smart_placement_main(void *args)
{

    Drivers_open();
    Board_driversOpen();

    PMU_init(&gPmuConfig);

    PMU_profileStart("Without Smart Placement");
    function_f1();
    PMU_profileEnd("Without Smart Placement");

    PMU_profileStart("With Smart Placement");
    annotated_function_f1();
    PMU_profileEnd("With Smart Placement");

    PMU_profilePrint();

    DebugP_log("\r\nAll tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
