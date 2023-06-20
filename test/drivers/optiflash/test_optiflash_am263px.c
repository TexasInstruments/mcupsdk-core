/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/optiflash/v0/rl2/rl2.h>
#include <drivers/optiflash/v0/flc/flc.h>
#include <drivers/hw_include/am263px/cslr_soc_baseaddress.h>
#include "main.h"

#ifdef SOC_AM263PX

/*
 * Test cases
 */
void* test_flc_configuration(void* args)
{
    // MCUSDK-4441 & MCUSDK-4446

    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;
    /*flc should be already configured by syscfg and not be configured else where in the program*/
    /*test case 1: */
    DebugP_assert(rl2_of_reg->FLC[0].LO == (volatile uint32_t)0x88000000);
    DebugP_assert(rl2_of_reg->FLC[0].HI == (volatile uint32_t)(0x88100000));
    DebugP_assert(rl2_of_reg->FLC[0].RA == (volatile uint32_t)(0x70000000));
    // reconfigure branch
    gFLCRegionConfig[0].sourceStartAddress = (uint32_t)sourceBuffer;
    gFLCRegionConfig[0].destinationStartAddress = (uint32_t)destBuffer;
    gFLCRegionConfig[0].sourceEndAddress = (uint32_t)sourceBuffer + TRANSFERSIZE;
    // FLC_startRegion(&gFLCRegionConfig[0]);
    // check if all the required configurations has been written to the correct configurations
    DebugP_assert(rl2_of_reg->FLC[0].LO == (volatile uint32_t)sourceBuffer);
    DebugP_assert(rl2_of_reg->FLC[0].HI == (volatile uint32_t)((uint32_t)sourceBuffer + TRANSFERSIZE));
    DebugP_assert(rl2_of_reg->FLC[0].RA == (volatile uint32_t)(destBuffer));
    return NULL;
}

void* test_flc_interrupt(void* args)
{
    // MCUSDK-4442
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;
    /* see if all interrupts are cleared*/
    DebugP_assert(rl2_of_reg->IRQENABLE_SET == 0);
    /* FLC done interrupt*/
    FLC_setInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_DONE);
    DebugP_assert((rl2_of_reg->IRQENABLE_SET & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_DON_MASK) != 0);
    FLC_clearInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_WRITE_ERROR);
    DebugP_assert(rl2_of_reg->IRQENABLE_SET == 0);
    /* FLC write interrupt*/
    FLC_setInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_WRITE_ERROR);
    DebugP_assert((rl2_of_reg->IRQENABLE_SET & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_WRERR_MASK) != 0);
    FLC_clearInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_WRITE_ERROR);
    DebugP_assert(rl2_of_reg->IRQENABLE_SET == 0);
    /* FLC read interrupt*/
    FLC_setInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_READ_ERROR);
    DebugP_assert((rl2_of_reg->IRQENABLE_SET & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_MSK_FLC_RDERR_MASK) != 0);
    FLC_clearInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_READ_ERROR);
    DebugP_assert(rl2_of_reg->IRQENABLE_SET == 0);

    return NULL;
}

void *test_flc_enable_disable(void* args)
{
    // MCUSDK-4445
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;

    /* enable FLC */
    FLC_startRegion(&gFLCRegionConfig[0]);
    DebugP_assert((rl2_of_reg->FLC[0].CTL & CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK) != 0);
    FLC_disable(&gFLCRegionConfig[0]);
    DebugP_assert((rl2_of_reg->FLC[0].CTL & CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK) == 0);

    return NULL;
}

void *test_rl2_config(void *args)
{
    // MCUSDK-4449
    // MCUSDK-4450
    // MCUSDK-4452
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gRL2Config[0].baseAddress;

    uint32_t RA[10][7] =
    {
        {
            0x70000000, 128*1024,
            0,0,
            0,0,
            RL2_API_STS_SUCCESS
        },
        {
            0x70000000, 32*1024,
            0x70008000, 32*1024,
            0x70010000, 64*1024,
            RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_REGIONS_OVERLAPPING
        },
        {
            0x70000000, 32*1024,
            0x70008000 - 12, 32*1024,
            0x70010000, 64*1024,
            RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_REGIONS_OVERLAPPING
        },
        {
            0x70000000, 32*1024,
            0x70008000, 32*1024,
            0x70010000 - 12, 64*1024,
            RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_REGIONS_OVERLAPPING
        },
        {
            0x70000000, 32*1024,
            0x70008000, 64*1024,
            0x70010000, 32*1024,
            RL2_API_STS_CANNOT_CONFIGURE | RL2_API_STS_REGIONS_OVERLAPPING
        },
        {
            0x70000000, 8*1024,
            0,0,
            0,0,
            RL2_API_STS_SUCCESS
        },
        {
            0x70000000, 16*1024,
            0,0,
            0,0,
            RL2_API_STS_SUCCESS
        },
        {
            0x70000000, 32*1024,
            0,0,
            0,0,
            RL2_API_STS_SUCCESS
        },
        {
            0x70000000, 64*1024,
            0,0,
            0,0,
            RL2_API_STS_SUCCESS
        },
        {
            0x70000000, 256*1024,
            0,0,
            0,0,
            RL2_API_STS_SUCCESS
        }
    };

    /* configure RL2 and check*/
    RL2_API_STS_t res = RL2_configure(&gRL2Config[0]);
    DebugP_assert(RL2_API_STS_SUCCESS == res);

    for(unsigned int i = 0; i < 5; i++)
    {
        gRL2Config[0].l2Sram0Base = RA[i][0];
        gRL2Config[0].l2Sram0Len  = RA[i][1];
        gRL2Config[0].l2Sram1Base = RA[i][2];
        gRL2Config[0].l2Sram1Len  = RA[i][3];
        gRL2Config[0].l2Sram2Base = RA[i][4];
        gRL2Config[0].l2Sram2Len  = RA[i][5];
        res = RL2_configure(&gRL2Config[0]);
        DebugP_assert(RA[i][6] == res);
        if(RL2_API_STS_SUCCESS == res)
        {
            DebugP_assert(rl2_of_reg->REM[0].ADR == RA[i][0]);
            DebugP_assert(rl2_of_reg->REM[0].LEN == RA[i][1]);
            DebugP_assert(rl2_of_reg->REM[1].ADR == RA[i][2]);
            DebugP_assert(rl2_of_reg->REM[1].LEN == RA[i][3]);
            DebugP_assert(rl2_of_reg->REM[2].ADR == RA[i][4]);
            DebugP_assert(rl2_of_reg->REM[2].LEN == RA[i][5]);
        }
    };

    return NULL;
}

void *test_rat_config(void *args)
{
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)CSL_RL2_REGS_R5SS0_CORE0_U_BASE;
    DebugP_assert((rl2_of_reg->RAT[0].CTL & 0x3f) != 0);
    DebugP_assert(rl2_of_reg->RAT[0].RBA == 0x8000);
    DebugP_assert(rl2_of_reg->RAT[0].RTA == 0x200);

    return NULL;
}

#endif
