/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/**
 *  \file mpu_firewall.c
 *
 *  \brief File containing MPU Firewall Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/mpu_firewall/v0/mpu_firewall.h>

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/** Gets the start address of the given region in the MPU firewall  */
static uint32_t MPU_FIREWALL_getProgrammableStartAddress (uint32_t baseAddr, uint32_t regionNum);
/** Gets the end address of the given region in the MPU firewall  */
static uint32_t MPU_FIREWALL_getProgrammableEndAddress (uint32_t baseAddr, uint32_t regionNum);
/** Gets the permission attributes of the given region in the MPU firewall  */
static void MPU_FIREWALL_getPermissionAttributes(uint32_t baseAddr, uint32_t regionNum, MPU_FIREWALL_RegionParams* mpuParams);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern uint32_t gMpuFirewallNumRegions;
extern MPU_FIREWALL_RegionParams gMpuFirewallRegionConfig[];
extern MPU_FIREWALL_Config gMpuFirewallConfig[];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t MPU_FIREWALL_getProgrammableStartAddress (uint32_t baseAddr, uint32_t regionNum)
{
    /* Returns the start address of the specified programmable region */
    return (uint32_t)((CSL_MpuRegs*) baseAddr)->PROG_REGION[regionNum].PROG_START_ADDRESS;
}

static uint32_t MPU_FIREWALL_getProgrammableEndAddress (uint32_t baseAddr, uint32_t regionNum)
{
    /* Returns the end address of the specified programmable region */
    return (uint32_t)((CSL_MpuRegs*) baseAddr)->PROG_REGION[regionNum].PROG_END_ADDRESS;
}

static void MPU_FIREWALL_getPermissionAttributes(uint32_t baseAddr, uint32_t regionNum, MPU_FIREWALL_RegionParams* mpuParams)
{
    uint32_t mppa_baseAddress = (uint32_t) &((CSL_MpuRegs*) baseAddr)->PROG_REGION[regionNum].PROG_MPPA;

    /* Populate the members of MPU_FIREWALL_RegionParams */
    mpuParams->aidConfig = CSL_REG32_FEXT_RAW((const volatile uint32_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_AID_15_0_MASK,
                                                        CSL_MPU_PROG_MPPA_AID_15_0_SHIFT);

    mpuParams->aidxConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        (uint8_t) CSL_MPU_PROG_MPPA_AIDX_MASK,
                                                        CSL_MPU_PROG_MPPA_AIDX_SHIFT);

    mpuParams->nonSecureConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_NS_MASK,
                                                        CSL_MPU_PROG_MPPA_NS_SHIFT);

    mpuParams->debugConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_EMU_MASK,
                                                        CSL_MPU_PROG_MPPA_EMU_SHIFT);

    mpuParams->supervisorReadConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_SR_MASK,
                                                        CSL_MPU_PROG_MPPA_SR_SHIFT);

    mpuParams->supervisorWriteConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_SW_MASK,
                                                        CSL_MPU_PROG_MPPA_SW_SHIFT);

    mpuParams->supervisorExecConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_SX_MASK,
                                                        CSL_MPU_PROG_MPPA_SX_SHIFT);

    mpuParams->userReadConfig =  CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_UR_MASK,
                                                        CSL_MPU_PROG_MPPA_UR_SHIFT);

    mpuParams->userWriteConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_UW_MASK,
                                                        CSL_MPU_PROG_MPPA_UW_SHIFT);

    mpuParams->userExecConfig = CSL_REG8_FEXT_RAW( (const volatile uint8_t *) mppa_baseAddress,
                                                        CSL_MPU_PROG_MPPA_UX_MASK,
                                                        CSL_MPU_PROG_MPPA_UX_SHIFT);
}

Fwl_Return_t MPU_FIREWALL_getFirewallConfig(uint32_t firewallId, MPU_FIREWALL_Config** firewallConfig)
{
    Fwl_Return_t status = FWL_DRV_RETURN_FAILURE;

    /* Check if the firewal ID is less then the total number of firewalls in SOC */
    if(firewallId < CSL_FW_CNT)
    {
        status = FWL_DRV_RETURN_SUCCESS;
        *firewallConfig = &gMpuFirewallConfig[firewallId];
    }
    return status;
}

Fwl_Return_t MPU_FIREWALL_getRegion(MPU_FIREWALL_RegionParams* mpuParams)
{
    uint32_t baseAddr;
    uint32_t regionNumber;
    Fwl_Return_t status = FWL_DRV_RETURN_FAILURE;
    MPU_FIREWALL_Config* firewallConfig = NULL;

    /* Get the firewall parameters */
    status = MPU_FIREWALL_getFirewallConfig(mpuParams->id, &firewallConfig);

    /* Get the base address of the firewall to be configured */
    baseAddr = firewallConfig->baseAddr;

    /* Get the region number to be fetched */
    regionNumber = mpuParams->regionNumber;

    /* Check if the region number is less than the total regions in firewall */
    if(status == FWL_DRV_RETURN_SUCCESS && regionNumber < firewallConfig->numRegions)
    {
        /* Start address configuration */
        mpuParams->startAddress = MPU_FIREWALL_getProgrammableStartAddress (baseAddr,  regionNumber);
        /* End Address configuration */
        mpuParams->endAddress = MPU_FIREWALL_getProgrammableEndAddress (baseAddr,  regionNumber);

        MPU_FIREWALL_getPermissionAttributes(baseAddr,  regionNumber, mpuParams);
    }
    return status;
}

Fwl_Return_t MPU_FIREWALL_readFaultAddress (uint32_t firewallId, uint32_t* faultAddress)
{
    uint32_t baseAddr;
    MPU_FIREWALL_Config* firewallConfig = NULL;
    Fwl_Return_t status = FWL_DRV_RETURN_FAILURE;

    /* Get the firewall parameters */
    status = MPU_FIREWALL_getFirewallConfig(firewallId, &firewallConfig);

    if(status == FWL_DRV_RETURN_SUCCESS){
        /* Get the base address of the firewall to be configured */
        baseAddr = firewallConfig->baseAddr;

        /* Read the fault address register */
        *faultAddress = HW_RD_REG32(&((CSL_MpuRegs*) baseAddr)->FAULT_ADDRESS);
    }

    return status;
}

Fwl_Return_t MPU_FIREWALL_readFaultStatus (uint32_t firewallId, uint32_t* faultStatus)
{
    uint32_t baseAddr;
    MPU_FIREWALL_Config* firewallConfig = NULL;
    Fwl_Return_t status = FWL_DRV_RETURN_FAILURE;

    /* Get the firewall parameters */
    status = MPU_FIREWALL_getFirewallConfig(firewallId, &firewallConfig);

    if(status == FWL_DRV_RETURN_SUCCESS){
        /* Get the base address of the firewall to be configured */
        baseAddr = firewallConfig->baseAddr;

        /* Read the fault status register */
        *faultStatus = HW_RD_REG32(&((CSL_MpuRegs*) baseAddr)->FAULT_STATUS);
    }

    return status;
}

Fwl_Return_t MPU_FIREWALL_getInterruptStatus (uint32_t firewallId, uint32_t* interruptStatus)
{
    uint32_t baseAddr;
    MPU_FIREWALL_Config* firewallConfig = NULL;
    Fwl_Return_t status = FWL_DRV_RETURN_FAILURE;

    /* Get the firewall parameters */
    status = MPU_FIREWALL_getFirewallConfig(firewallId, &firewallConfig);

    if(status == FWL_DRV_RETURN_SUCCESS){
        /* Get the base address of the firewall to be configured */
        baseAddr = firewallConfig->baseAddr;

        /* Read the interrupt status register */
        *interruptStatus = HW_RD_REG32(&((CSL_MpuRegs*) baseAddr)->INT_RAW_STATUS_SET);
    }

    return status;
}
