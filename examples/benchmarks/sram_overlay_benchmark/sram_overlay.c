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
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/QueueP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/optiflash.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "csl_arm_r5_pmu.h"
#include "appprofile.h"
#include "load.h"

typedef void (*pProfFnx_t) (void);

static inline void ovlAccn(uint32_t function_address)
{
    FLC_RegionInfo region;
    region.baseAddress = CSL_RL2_REGS_R5SS0_CORE0_U_BASE;
    region.destinationStartAddress = 0x701D8000;
    region.regionId = 0;
    region.sourceStartAddress = function_address;
    region.sourceEndAddress = function_address + 32*1024;

    FLC_configureRegion(&region);
    FLC_startRegion(&region);
}

static inline void profile_fnx( pProfFnx_t pFnx, const char tag[], int ovlEn)
{
    // initial measurement
    volatile uint32_t cyc = CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
    volatile uint32_t ix = CSL_armR5PmuReadCntr(0);
    volatile uint32_t icm = CSL_armR5PmuReadCntr(1);
    volatile uint32_t ica = CSL_armR5PmuReadCntr(2);

    // test load
    CacheP_wbInvAll(CacheP_TYPE_ALL);
    if(ovlEn)
    {
        ovlAccn((uint32_t)pFnx);
    }
    pFnx();

    // final measurement
    uint32_t endCyc =  CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
    cyc = endCyc > cyc ? endCyc - cyc : (uint64_t)(0xffffffff + endCyc) - cyc;
    ix = CSL_armR5PmuReadCntr(0) - ix;
    icm = CSL_armR5PmuReadCntr(1) - icm;
    ica = CSL_armR5PmuReadCntr(2) - ica;

    DebugP_log("%s: %d, %d, %d, %d\r\n",tag, cyc , ix  , icm , ica);
}

void sram_overlay_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    config_pmu();

    profile_fnx(load_function_t1_sram,  "T1 SRAM", FALSE);
    profile_fnx(load_function_t1_xip,   "T1 XIP ", FALSE);
    profile_fnx(load_function_t1_xip,   "T1 OVL ", TRUE);

    profile_fnx(load_function_t2_sram,  "T2 SRAM", FALSE);
    profile_fnx(load_function_t2_xip,   "T2 XIP ", FALSE);
    profile_fnx(load_function_t2_xip,   "T2 XIP ", TRUE);

    profile_fnx(load_function_t3_sram,  "T3 SRAM", FALSE);
    profile_fnx(load_function_t3_xip,   "T3 XIP ", FALSE);
    profile_fnx(load_function_t3_xip,   "T3 XIP ", TRUE);

    profile_fnx(load_function_t4_sram,  "T4 SRAM", FALSE);
    profile_fnx(load_function_t4_xip,   "T4 XIP ", FALSE);
    profile_fnx(load_function_t4_xip,   "T4 XIP ", TRUE);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void load_function_t1_xip(void)
{
    COMP_BLOCK_256(4, 1, 1, 4);
}

void load_function_t1_sram(void)
{
    COMP_BLOCK_256(4, 1, 1, 4);
}

void load_function_t2_sram(void)
{
    COMP_BLOCK_256(2, 1, 1, 1);
}

void load_function_t2_xip(void)
{
    COMP_BLOCK_256(2, 1, 1, 1);
}

void load_function_t3_sram(void)
{

    COMP_BLOCK_256(1, 1, 1, 0);
}

void load_function_t3_xip(void)
{
    COMP_BLOCK_256(1, 1, 1, 0);
}

void load_function_t4_sram(void)
{
    COMP_BLOCK_256(0, 1, 1, 1);
}

void load_function_t4_xip(void)
{
    COMP_BLOCK_256(0, 1, 1, 1);
}