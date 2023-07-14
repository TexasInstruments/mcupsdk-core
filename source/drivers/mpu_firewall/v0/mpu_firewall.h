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
 *  \defgroup DRV_MPU_FIREWALL_MODULE APIs for MPU Firewall
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program the MPU Firewall module.
 *
 *  @{
 */

/**
 *  \file v0/mpu_firewall.h
 *
 *  \brief MPU Firewall Driver API/interface file.
 */

#ifndef MPU_FIREWALL_H_
#define MPU_FIREWALL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/mpu_firewall/v0/cslr_mpu.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief
 *  Firewall Driver Error code
 *
 * \details
 *  The enumeration describes all the possible return and error codes which
 *  the Firewall Driver can return
 */
typedef enum Fwl_Return_e
{
    FWL_DRV_RETURN_SUCCESS                  = 0xBA14D51BU, /*!< Success/pass return code */
    FWL_DRV_RETURN_FAILURE                  = 0xAADF4AFAU, /*!< General or unspecified failure/error */
}Fwl_Return_t;

/**
 *  \brief  MPU Firewall Target
 *
 *  This structure contains parameters associated with an MPU Firewall memory map target.
 */
typedef struct
{
    /** Start address of the target */
    uint32_t  startAddr ;
    /** Size of the region */
    uint32_t regionSize ;

}Firewall_Target;

/**
 *  \brief  MPU Firewall Parameters
 *
 *  This structure contains parameters associated with a MPU Firewall.
 */
typedef struct
{
    /** Base address of the firewall */
    uint32_t  baseAddr ;
    /** Number of regions in the firewall */
    uint32_t  numRegions ;
    /** Number of memory map targets */
    uint32_t targetCount ;
    /** pointer to the target information array*/
    Firewall_Target *target;

}MPU_FIREWALL_Config;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  MPU Region Parameters
 *
 *  This structure contains MPPA configuration prarameters
 *  of a MPU Firewall region.
 */
typedef struct
{
    /** Firewall ID */
    uint32_t  id ;
    /** Region number in a particular firewall */
    uint32_t  regionNumber ;
    /** Start address of a firewall region */
    uint32_t startAddress ;
    /** End address of a firewall region */
    uint32_t endAddress ;
    /** AID configuration - It is a bit mask with each bit denoting an AID
    An AID is allowed if the value in bit position allocated for it is 1  */
    uint32_t aidConfig ;
    /** External AID configuration - 0->not allowed, 1->allowed */
    uint8_t aidxConfig ;
    /** Supervisor read permission - 0->not allowed, 1->allowed */
    uint8_t supervisorReadConfig ;
    /** Supervisor write permission - 0->not allowed, 1->allowed */
    uint8_t supervisorWriteConfig ;
    /** Supervisor Execute permission - 0->not allowed, 1->allowed */
    uint8_t supervisorExecConfig ;
    /** User read permission - 0->not allowed, 1->allowed */
    uint8_t userReadConfig ;
    /** User write permission - 0->not allowed, 1->allowed */
    uint8_t userWriteConfig ;
    /** User Execute permission - 0->not allowed, 1->allowed */
    uint8_t userExecConfig ;
    /** Non secure access permission - 0->not allowed, 1->allowed */
    uint8_t nonSecureConfig ;
    /** Debug permission - 0->not allowed, 1->allowed */
    uint8_t debugConfig ;

}MPU_FIREWALL_RegionParams;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  Retrieves the start and end addresses and the permission attributes of a given region in a given firewall.
 *
 *  \param  mpuParams      Structure containing all region configuration parameters.
 *
 *  \return FWL_DRV_RETURN_SUCCESS if the region configuration is successfully fetched; else FWL_DRV_RETURN_FAILURE
 */
Fwl_Return_t MPU_FIREWALL_getRegion(MPU_FIREWALL_RegionParams* mpuParams);

/**
 *  \brief  Function to get parameters associated to a MPU firewall.
 *
 *  \param  firewallId      Firewall ID.
 *  \param  firewallConfig  Structure to save mpu firewall configuration.
 *
 *  \return FWL_DRV_RETURN_SUCCESS on successful config read; else FWL_DRV_RETURN_FAILURE
 */
Fwl_Return_t MPU_FIREWALL_getFirewallConfig(uint32_t firewallId, MPU_FIREWALL_Config** firewallConfig);

/**
 *  \brief  Function to read the fault address that created the firewall
 *          violation.
 *
 *  \param  firewallId      MPU Firewall Id
 *  \param  faultAddress    pointer to the faultAddress to be populated
 *
 *  \return FWL_DRV_RETURN_SUCCESS on successful fault address read; else FWL_DRV_RETURN_FAILURE
 */
Fwl_Return_t MPU_FIREWALL_readFaultAddress (uint32_t firewallId, uint32_t* faultAddress);

/**
 *  \brief  Function to read the fault status register. It contains information
 *          on the kind of firewall violation that had occurred.
 *
 *  \param  firewallId      MPU Firewall Id
 *  \param  faultStatus     pointer to the faultStatus to be populated
 *
 *  \return FWL_DRV_RETURN_SUCCESS on successful fault status read; else FWL_DRV_RETURN_FAILURE
 */
Fwl_Return_t MPU_FIREWALL_readFaultStatus (uint32_t firewallId, uint32_t* faultStatus);

/**
 *  \brief  Function to read interrupt status.
 *
 *  \param  firewallId          MPU Firewall Id
 *  \param  interruptStatus     pointer to the interruptStatus to be populated
 *
 *  \return FWL_DRV_RETURN_SUCCESS on successful interrupt status read; else FWL_DRV_RETURN_FAILURE
 */
Fwl_Return_t MPU_FIREWALL_getInterruptStatus (uint32_t firewallId, uint32_t* interruptStatus);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MPU_FIREWALL_H_ */

/** @} */
