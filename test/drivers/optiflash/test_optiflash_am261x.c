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
#include <drivers/hw_include/am261x/cslr_soc_baseaddress.h>
#include "main.h"

#ifdef SOC_AM261X

/*
 * Test cases
 */
void* test_flc_configuration(void* args)
{
    // MCUSDK-4441
    int32_t retval = SystemP_SUCCESS;
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;
    /*flc should be already configured by syscfg and not be configured else where in the program*/

    if(SystemP_SUCCESS == retval && (volatile uint32_t)0x88000000 != rl2_of_reg->FLC[0].LO )
    {
        retval = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retval && rl2_of_reg->FLC[0].HI != (volatile uint32_t)(0x88100000))
    {
        retval = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retval && rl2_of_reg->FLC[0].RA != (volatile uint32_t)(0x70000000))
    {
        retval = SystemP_FAILURE;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retval);

    return NULL;
}


void *test_flc_runtimeconfig(void *args)
{
    //MCUSDK-4446
    int32_t retval = SystemP_SUCCESS;
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;
    // reconfigure branch
    gFLCRegionConfig[0].sourceStartAddress = (uint32_t)sourceBuffer;
    gFLCRegionConfig[0].destinationStartAddress = (uint32_t)destBuffer;
    gFLCRegionConfig[0].sourceEndAddress = (uint32_t)sourceBuffer + TRANSFERSIZE;
    FLC_configureRegion(&gFLCRegionConfig[0]);
    FLC_startRegion(&gFLCRegionConfig[0]);
    {
        uint32_t cpy_status = 0;
        do
        {
            FLC_isRegionDone(&gFLCRegionConfig[FLC_REGION_FLC0], &cpy_status);
        } while ((cpy_status & (1<<FLC_REGION_FLC0)) == 0);
    }
    // check if all the required configurations has been written to the correct configurations
    if(SystemP_SUCCESS == retval && rl2_of_reg->FLC[0].LO != (volatile uint32_t)sourceBuffer)
    {
        retval = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retval && rl2_of_reg->FLC[0].HI != (volatile uint32_t)((uint32_t)sourceBuffer + TRANSFERSIZE))
    {
        retval = SystemP_FAILURE;
    }
    if(SystemP_SUCCESS == retval && rl2_of_reg->FLC[0].RA != (volatile uint32_t)(destBuffer))
    {
        retval = SystemP_FAILURE;
    }

    for(uint32_t  i = 0 ; i < TRANSFERSIZE; i++)
    {
        if(SystemP_SUCCESS == retval  && sourceBuffer[i] != destBuffer[i])
        {
            retval = SystemP_FAILURE;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retval);

    return NULL;
}

void* test_flc_interrupt(void* args)
{
    // MCUSDK-4442
    int32_t retval = SystemP_SUCCESS;
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;
    HwiP_disable();
    /* see if all interrupts are cleared*/
    FLC_clearInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_DONE);
    if(SystemP_FAILURE == retval && rl2_of_reg->IRQSTATUS_RAW != 0)
    {
        retval = SystemP_FAILURE;
    }
    /* FLC done interrupt*/
    FLC_enableInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_DONE);
    if(SystemP_FAILURE == retval && (rl2_of_reg->IRQENABLE_SET & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_DON_MASK) == 0)
    {
        retval = SystemP_FAILURE;
    }
    FLC_disableInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_DONE);
    if(SystemP_FAILURE == retval && rl2_of_reg->IRQENABLE_SET != 0)
    {
        retval = SystemP_FAILURE;
    }
    /* FLC write interrupt*/
    FLC_enableInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_WRITE_ERROR);
    if(SystemP_FAILURE == retval && (rl2_of_reg->IRQENABLE_SET & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_WRERR_MASK) == 0)
    {
        retval = SystemP_FAILURE;
    }
    FLC_disableInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_WRITE_ERROR);
    if(SystemP_FAILURE == retval && rl2_of_reg->IRQENABLE_SET != 0)
    {
        retval = SystemP_FAILURE;
    }
    /* FLC read interrupt*/
    FLC_enableInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_READ_ERROR);
    if(SystemP_FAILURE == retval && (rl2_of_reg->IRQENABLE_SET & CSL_RL2_OF_R5FSS0_CORE0_IRQSTATUS_RAW_FLC_RDERR_MASK) == 0)
    {
        retval = SystemP_FAILURE;
    }
    FLC_disableInterrupt(&gFLCRegionConfig[0], FLC_INTERRUPT_READ_ERROR);
    if(SystemP_FAILURE == retval && rl2_of_reg->IRQENABLE_SET != 0)
    {
        retval = SystemP_FAILURE;
    }
    HwiP_enable();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retval);
    return NULL;
}

void *test_flc_enable_disable(void* args)
{
    // MCUSDK-4445
    int32_t retval = SystemP_SUCCESS;
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)gFLCRegionConfig[0].baseAddress;

    /* enable FLC */
    FLC_startRegion(&gFLCRegionConfig[0]);
    if(SystemP_FAILURE == retval && (rl2_of_reg->FLC[0].CTL & CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK) == 0)
    {
        retval = SystemP_FAILURE;
    }
    FLC_disable(&gFLCRegionConfig[0]);
    if(SystemP_FAILURE == retval && (rl2_of_reg->FLC[0].CTL & CSL_RL2_OF_R5FSS0_CORE0_FLC_CTL_FENABLE_MASK) != 0)
    {
        retval = SystemP_FAILURE;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retval);
    return NULL;
}

void *test_rl2_config(void *args)
{
    // MCUSDK-4449
    // MCUSDK-4450
    // MCUSDK-4452
    int32_t retval = SystemP_SUCCESS;
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
    if(SystemP_SUCCESS == retval && RL2_API_STS_SUCCESS != res)
    {
        retval = SystemP_FAILURE;
    }

    for(unsigned int i = 0; i < 5; i++)
    {
        gRL2Config[0].l2Sram0Base = RA[i][0];
        gRL2Config[0].l2Sram0Len  = RA[i][1];
        gRL2Config[0].l2Sram1Base = RA[i][2];
        gRL2Config[0].l2Sram1Len  = RA[i][3];
        gRL2Config[0].l2Sram2Base = RA[i][4];
        gRL2Config[0].l2Sram2Len  = RA[i][5];
        res = RL2_configure(&gRL2Config[0]);
        if(SystemP_SUCCESS == retval && RA[i][6] != res)
        {
            retval = SystemP_FAILURE;
        }
        if(RL2_API_STS_SUCCESS == res)
        {
            if(SystemP_SUCCESS == retval && rl2_of_reg->REM[0].ADR != RA[i][0])
            {
                retval = SystemP_FAILURE;
            }
            if(SystemP_SUCCESS == retval && rl2_of_reg->REM[0].LEN != RA[i][1])
            {
                retval = SystemP_FAILURE;
            }
            if(SystemP_SUCCESS == retval && rl2_of_reg->REM[1].ADR != RA[i][2])
            {
                retval = SystemP_FAILURE;
            }
            if(SystemP_SUCCESS == retval && rl2_of_reg->REM[1].LEN != RA[i][3])
            {
                retval = SystemP_FAILURE;
            }
            if(SystemP_SUCCESS == retval && rl2_of_reg->REM[2].ADR != RA[i][4])
            {
                retval = SystemP_FAILURE;
            }
            if(SystemP_SUCCESS == retval && rl2_of_reg->REM[2].LEN != RA[i][5])
            {
                retval = SystemP_FAILURE;
            }
        }
    };
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retval);

    return NULL;
}

void *test_rat_config(void *args)
{
    int32_t retval = SystemP_SUCCESS;
    CSL_rl2_of_r5fss0_core0Regs * rl2_of_reg;
    rl2_of_reg = (CSL_rl2_of_r5fss0_core0Regs *)CSL_RL2_REGS_R5SS0_CORE0_U_BASE;
    DebugP_assert((rl2_of_reg->RAT[0].CTL & 0x3f) != 0);
    DebugP_assert(rl2_of_reg->RAT[0].RBA == 0x8000);
    DebugP_assert(rl2_of_reg->RAT[0].RTA == 0x2000);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retval);
    return NULL;
}

#endif
