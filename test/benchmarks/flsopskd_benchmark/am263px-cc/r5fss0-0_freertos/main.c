/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "appprofile.h"
#include "csl_arm_r5_pmu.h"

extern void hopperFnxFlash();
extern void loopwait();

void loopwait()
{
    for(volatile int i = 0; i < 10; i++);
}

uint32_t start_cntr_val = 0;
uint32_t stop_cntr_val = 0;

uint32_t top_vals[10] = {0};
uint32_t bot_vals[10] = {0};

__attribute__((section(".flash_fnx"))) void flashfnx()
{
    stop_cntr_val = CycleCounterP_getCount32();
    uint32_t diff =stop_cntr_val -  start_cntr_val;

    for(int i = 0; i < 10; i++)
    {
        if(diff == top_vals[i])
        {
            break;
        }
        else if(diff > top_vals[i])
        {
            for(int j = 9; j > i; j--)
            {
                top_vals[j] = top_vals[j-1];
            }
            top_vals[i] = diff;
            break;
        }
    }
    
}

int main()
{
    /* init SOC specific modules */
    System_init();
    Board_init();
    Drivers_open();
    Board_driversOpen();

    DebugP_log("\r\nStarting Application.");

    config_pmu();
    {
        // volatile uint64_t time = ClockP_getTimeUsec();
        volatile uint64_t cyc = CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
        volatile uint64_t ix = CSL_armR5PmuReadCntr(0);
        volatile uint64_t icm = CSL_armR5PmuReadCntr(1);
        volatile uint64_t ica = CSL_armR5PmuReadCntr(2);

        for(int i = 0; i < 2000; i++)
        {
            hopperFnxFlash();
        }

        // volatile uint64_t endTime = ClockP_getTimeUsec();
        volatile uint32_t endCyc =  CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
        cyc = endCyc > cyc ? endCyc - cyc : (uint64_t)(0xffffffff + endCyc) - cyc;
        ix = CSL_armR5PmuReadCntr(0) - ix;
        icm = CSL_armR5PmuReadCntr(1) - icm;
        ica = CSL_armR5PmuReadCntr(2) - ica;

        DebugP_log("\r\nwithout FOTA:\tR5F Cyc = %llu\tix = %d\ticm = %d\tica = %d\t", cyc, ix, icm, ica);
    }
    {
        // volatile uint64_t time = ClockP_getTimeUsec();
        volatile uint64_t cyc = CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
        volatile uint64_t ix = CSL_armR5PmuReadCntr(0);
        volatile uint64_t icm = CSL_armR5PmuReadCntr(1);
        volatile uint64_t ica = CSL_armR5PmuReadCntr(2);

        *((volatile uint32_t*)(0x50D08010)) = 1;
        for(int i = 0; i < 2000; i++)
        {
            hopperFnxFlash();
        }

        // volatile uint64_t endTime = ClockP_getTimeUsec();
        volatile uint32_t endCyc =  CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
        cyc = endCyc > cyc ? endCyc - cyc : (uint64_t)((uint64_t)0xffffffff + (uint64_t)endCyc) - (uint64_t)cyc;
        ix = CSL_armR5PmuReadCntr(0) - ix;
        icm = CSL_armR5PmuReadCntr(1) - icm;
        ica = CSL_armR5PmuReadCntr(2) - ica;

        DebugP_log("\r\nwith FOTA:\tR5F Cyc = %llu\tix = %d\ticm = %d\tica = %d\t", cyc, ix, icm, ica);
    }

    Board_driversClose();
    Drivers_close();
    return 0;
}
