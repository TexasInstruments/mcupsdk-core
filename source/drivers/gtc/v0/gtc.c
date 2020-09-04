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

/**
 *  \file   GTC.c
 *
 *  \brief  Low lever APIs performing hardware register writes and reads for
 *         GTC version 0.
 *
 *   This file contains the hardware register write/read APIs for GTC.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/gtc.h>
#include <drivers/soc.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_gtc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t GTC_setFID(void)
{
    int32_t status = SystemP_FAILURE;
    uint64_t clkRate = 0;

    status = SOC_moduleGetClockFrequency(TISCI_DEV_GTC0, TISCI_DEV_GTC0_GTC_CLK, &clkRate);

    if (status == SystemP_SUCCESS)
    {
        HW_WR_REG32((CSL_GTC0_GTC_CFG1_BASE+CSL_GTC_CFG1_CNTFID0), clkRate);
    }

    return status;
}

void GTC_enable(void)
{
    uint32_t value = 0;

    value = HW_RD_REG32(CSL_GTC0_GTC_CFG1_BASE);
    HW_WR_REG32(CSL_GTC0_GTC_CFG1_BASE, value | CSL_GTC_CFG1_CNTCR_EN_MASK | CSL_GTC_CFG1_CNTCR_HDBG_MASK);

    return;
}

int32_t GTC_init(void)
{
    int32_t status = SystemP_FAILURE;

    status = GTC_setFID();

    if (status == SystemP_SUCCESS)
    {
        GTC_enable();
    }

    return status;
}

uint64_t GTC_getCount64(void)
{
    uint64_t count = 0;
    uint32_t count_low = 0, count_high = 0;

    /* Read and check if count value (high) have changed in between */
    /* Change in the higher 32b would result in invalid values as the read
    was not atomic */
    do
    {
        count_high = HW_RD_REG32(CSL_GTC_CFG1_CNTCV_HI);
        count_low =  HW_RD_REG32(CSL_GTC_CFG1_CNTCV_LO);
    } while(HW_RD_REG32(CSL_GTC_CFG1_CNTCV_HI) != count_high);

    count = (uint64_t)((uint64_t)count_high << 32) | count_low;

    return count;
}