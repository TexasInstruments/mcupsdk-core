/*
 * Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *  \file   firewall.c
 *
 *  \brief  File containing firewall Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/firewall.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr.h>

/**************************************************************************
 * Register Overlay Structure
 **************************************************************************/

typedef struct
{
    /* CTRL REGISTER BITS
     *
     * 9 ==> Cache mode
     * 8 ==> Background region
     * 4 ==> Lock
     */
    volatile uint32_t CTRL;

    /* PERMISSION REGISTER BITS
     * ========================
     * 31:24 reserved
     * 23:16 priv_id
     * 15    nonsec_usr_debug
     * 14    nonsec_usr_cacheable
     * 13    nonsec_usr_read
     * 12    nonsec_usr_write
     * 11    nonsec_priv_debug
     * 10    nonsec_priv_cacheable
     * 9     nonsec_priv_read
     * 8     nonsec_priv_write
     * 7     sec_user_debug
     * 6     sec_user_cacheable
     * 5     sec_usr_read
     * 4     sec_usr_write
     * 3     sec_priv_debug
     * 2     sec_priv_cacheable
     * 1     sec_priv_read
     * 0     sec_priv_write
     */
    volatile uint32_t PERMISSION[FWL_MAX_PRIVID_SLOTS];
    volatile uint32_t START_ADDR_L;
    volatile uint32_t START_ADDR_H;
    volatile uint32_t END_ADDR_L;
    volatile uint32_t END_ADDR_H;
} CSL_firewall_cfgRegs;

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* CSL base Address of Firewall MMR */
#define CSL_FWL_BASE_ADDR (0x45000000)
/* Offset value of Firewall MMR */
#define FWL_ID_ADDR_OFFSET (1024U)
/* Region offset value of Firewall regions */
#define FWL_REGION_OFFSET (0x20)
/* Macro function to calculate firewall Id base address from base and index */
#define FWL_ID_BASE_ADDR(ID) (uint32_t)((uint32_t)CSL_FWL_BASE_ADDR + \
                                        (uint32_t)(FWL_ID_ADDR_OFFSET * ID))
/* Macro function to calculate firewall region address from base and index */
#define FWL_REGION_BASE_ADDR(BASE, INDEX) (uint32_t)((uint32_t)BASE + \
                                                     (uint32_t)(INDEX * FWL_REGION_OFFSET))

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/* Register read functions */
static void Firewall_readRegionAddress(uint32_t baseAddr, uint64_t *startAddr, uint64_t *endAddr);
static void Firewall_readRegionPermission(uint32_t baseAddr,
                                          uint32_t *control,
                                          uint32_t permReg[FWL_MAX_PRIVID_SLOTS]);

/* Register write functions */
static int32_t Firewall_writeRegionAddress(uint32_t baseAddr, uint64_t startAddr, uint64_t endAddr);
static int32_t Firewall_writeRegionPermission(uint32_t baseAddr,
                                              uint32_t control,
                                              uint32_t permReg[FWL_MAX_PRIVID_SLOTS]);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Firewall_init(void)
{
    return;
}

void Firewall_deinit(void)
{
    return;
}

int32_t Firewall_open(Firewall_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Firewall_Config *config = NULL;
    Firewall_Object *object = NULL;

    if (NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = (Firewall_Config *)handle;
    }

    if (SystemP_SUCCESS == status)
    {
        object = config->object;
        DebugP_assert(NULL != object);

        if (TRUE == object->cfgDone)
        {
            /* Handle is already opened */
            status = SystemP_FAILURE;
        }
    }

    if (SystemP_SUCCESS == status)
    {
        object->handle = (Firewall_Handle)config;
    }

    /* Configure Firewall regions */
    if (SystemP_SUCCESS == status)
    {
        if (SOC_isHsDevice())
        {
            status = Firewall_configureRegion(object->handle);
        }
    }

    if (SystemP_SUCCESS == status)
    {
        object->cfgDone = TRUE;
        handle = (Firewall_Handle)config;
    }

    return status;
}

void Firewall_close(Firewall_Handle handle)
{
    Firewall_Config *config = NULL;

    if (handle != NULL)
    {
        config = (Firewall_Config *)handle;

        config->object->cfgDone = FALSE;
    }

    return;
}

/**
 * \brief Read region address from firewall registers
 */
static void Firewall_readRegionAddress(uint32_t baseAddr, uint64_t *startAddr, uint64_t *endAddr)
{
    const CSL_firewall_cfgRegs *pReg = (const CSL_firewall_cfgRegs *)baseAddr;
    *startAddr = (uint64_t)CSL_REG32_RD(&pReg->START_ADDR_L);
    *startAddr = *startAddr | (uint64_t)CSL_REG32_RD(&pReg->START_ADDR_H) << 32;
    *endAddr = (uint64_t)CSL_REG32_RD(&pReg->END_ADDR_L);
    *endAddr = *endAddr | (uint64_t)CSL_REG32_RD(&pReg->END_ADDR_H) << 32;
}

/**
 * \brief Write region address from firewall registers
 */
static int32_t Firewall_writeRegionAddress(uint32_t baseAddr, uint64_t startAddr, uint64_t endAddr)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t readStartAddr, readEndAddr;

    startAddr = startAddr & ~0xFFFU;
    endAddr = endAddr | 0xFFFU;
    const CSL_firewall_cfgRegs *pReg = (const CSL_firewall_cfgRegs *)baseAddr;
    CSL_REG32_WR(&pReg->START_ADDR_L, startAddr);
    CSL_REG32_WR(&pReg->START_ADDR_H, startAddr >> 32);
    CSL_REG32_WR(&pReg->END_ADDR_L, endAddr);
    CSL_REG32_WR(&pReg->END_ADDR_H, endAddr >> 32);

    Firewall_readRegionAddress(baseAddr, &readStartAddr, &readEndAddr);
    if ((startAddr != readStartAddr) || (endAddr != readEndAddr))
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/**
 * \brief Read region permission from firewall registers
 */
static void Firewall_readRegionPermission(uint32_t baseAddr,
                                          uint32_t *control,
                                          uint32_t permReg[FWL_MAX_PRIVID_SLOTS])
{
    uint16_t i;

    const CSL_firewall_cfgRegs *pReg = (const CSL_firewall_cfgRegs *)baseAddr;
    *control = CSL_REG32_RD(&pReg->CTRL);
    for (i = 0U; (i < FWL_MAX_PRIVID_SLOTS); i++)
    {
        permReg[i] = CSL_REG32_RD(&pReg->PERMISSION[i]);
    }
}

/**
 * \brief Write region permission to firewall registers
 */
static int32_t Firewall_writeRegionPermission(uint32_t baseAddr,
                                              uint32_t control,
                                              uint32_t permReg[FWL_MAX_PRIVID_SLOTS])
{
    uint16_t i;
    int32_t status = SystemP_SUCCESS;
    uint32_t readControl, readPermReg[FWL_MAX_PRIVID_SLOTS];

    const CSL_firewall_cfgRegs *pReg = (const CSL_firewall_cfgRegs *)baseAddr;
    for (i = 0U; (i < FWL_MAX_PRIVID_SLOTS); i++)
    {
        CSL_REG32_WR(&pReg->PERMISSION[i], permReg[i]);
    }
    CSL_REG32_WR(&pReg->CTRL, control);

    Firewall_readRegionPermission(baseAddr, &readControl, readPermReg);
    for (i = 0U; (i < FWL_MAX_PRIVID_SLOTS); i++)
    {
        if (permReg[i] != readPermReg[i])
        {
            status = SystemP_FAILURE;
            break;
        }
    }

    if (control != readControl)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/**
 * \brief Configure firewall for single region
 */
int32_t Firewall_configureSingleRegion(uint32_t firewallId, Firewall_RegionCfg *region)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t fwlBaseAddr, regBaseAddr;

    if ((region == NULL) || (firewallId > FWL_MAX_ID) || (region->regionIndex > FWL_MAX_REGION))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        fwlBaseAddr = FWL_ID_BASE_ADDR(firewallId);
        regBaseAddr = FWL_REGION_BASE_ADDR(fwlBaseAddr, region->regionIndex);
    }

    if (SystemP_SUCCESS == status)
    {
        status = Firewall_writeRegionAddress(regBaseAddr, region->startAddr, region->endAddr);
    }

    if (SystemP_SUCCESS == status)
    {
        status = Firewall_writeRegionPermission(regBaseAddr, region->control, region->permissions);
    }

    return status;
}

/**
 * \brief Intialize a firewall with multiple regions
 */
int32_t Firewall_configureRegion(Firewall_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Firewall_Config *config = (Firewall_Config *)handle;
    Firewall_Attrs *attrs = config->attrs;

    if ((handle == NULL) || (attrs == NULL))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {

        if ((attrs->firewallId > FWL_MAX_ID) || (attrs->totalRegions > FWL_MAX_REGION) ||
            (attrs->initRegions > attrs->totalRegions))
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {

        for (uint32_t count = 0; count < attrs->initRegions; count++)
        {
            Firewall_RegionCfg *region = &attrs->regionInfo[count];
            if (SystemP_SUCCESS == status)
            {
                status = Firewall_configureSingleRegion(attrs->firewallId, region);
            }
            else
            {
                break;
            }
        }
    }

    return status;
}