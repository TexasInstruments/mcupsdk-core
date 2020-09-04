/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
 *  \defgroup DRV_UDMA_MODULE APIs for UDMA
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the UDMA module. The APIs
 *  can be used by other drivers to get access to UDMA resources and also by
 *  application to allocate UDMA channels for memory transfer operation.
 *
 *  @{
 */
/** @} */

/**
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_API_MODULE UDMA Init API
 *            This is UDMA driver init, deinit and common API.
 *
 *  @{
 */

/**
 *  \file udma.h
 *
 *  \brief UDMA Driver API/interface file.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2494)
 */

#ifndef UDMA_H_
#define UDMA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

/* UDMA_SOC_CFG_* macros are defined udma_soc.h.
 * So including this first
 */
#include <drivers/udma/soc/udma_soc.h>
#include <drivers/udma/include/udma_types.h>
#include <drivers/udma/include/udma_ring.h>
#include <drivers/udma/include/udma_flow.h>
#include <drivers/udma/include/udma_event.h>
#include <drivers/udma/include/udma_rm.h>
#include <drivers/udma/include/udma_ch.h>
#include <drivers/udma/include/csl_udmap_tr.h>
#include <drivers/udma/include/csl_udmap_cppi5.h>
#include <drivers/udma/include/csl_pktdma_cppi5.h>
#include <drivers/udma/include/udma_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief UDMA Virtual to Physical address translation callback function.
 *
 *  This function is used by the driver to convert virtual address to physical
 *  address.
 *
 *  \param virtAddr [IN] Virtual address
 *  \param chNum    [IN] Channel number passed during channel open
 *  \param appData  [IN] Callback pointer passed during channel open
 *
 *  \return Corresponding physical address
 */
typedef uint64_t (*Udma_VirtToPhyFxn)(const void *virtAddr,
                                      uint32_t chNum,
                                      void *appData);
/**
 *  \brief UDMA Physical to Virtual address translation callback function.
 *
 *  This function is used by the driver to convert physical address to virtual
 *  address.
 *
 *  \param phyAddr  [IN] Physical address
 *  \param chNum    [IN] Channel number passed during channel open
 *  \param appData  [IN] Callback pointer passed during channel open
 *
 *  \return Corresponding virtual address
 */
typedef void *(*Udma_PhyToVirtFxn)(uint64_t phyAddr,
                                   uint32_t chNum,
                                   void *appData);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA initialization parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2631)
 */
typedef struct
{
    uint32_t                instId;
    /**< [IN] \ref Udma_InstanceIdSoc */
    uint32_t                skipGlobalEventReg;
    /**< Skips the global event registeration for the handle. By default this
     *   is set to FALSE and application can use this common handle to set the
     *   master event to limit the number of IA/IR registration per core
     *   This can be set to TRUE to skip this registration as in the case
     *   of having multiple handles per core in usecases */
    Udma_VirtToPhyFxn       virtToPhyFxn;
    /**< If not NULL, this function will be called to convert virtual address
     *   to physical address to be provided to UDMA.
     *   If NULL, the driver will assume a one-one mapping.
     */
    Udma_PhyToVirtFxn       phyToVirtFxn;
    /**< If not NULL, this function will be called to convert physical address
     *   to virtual address to access the pointer returned by the UDMA.
     *   If NULL, the driver will assume a one-one mapping.
     *
     *   Note: The init fxn will initialize this to the default one-one map
     *   function #Udma_defaultPhyToVirtFxn
     */
} Udma_InitPrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA init function.
 *
 *  Initializes the UDMA drivers.
 *  This function should be called before calling any of driver API's and
 *  should be called only once.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2576)
 *
 *  \param drvHandle    [IN] UDMA driver handle - static memory needs to
 *                           allocated by caller. This is used by the driver to
 *                           maintain the driver states.
 *                           This cannot be NULL.
 *  \param initPrms     [IN] UDMA Initialization parameters.
 *                           If NULL is passed, the default parameters will be
 *                           assumed - address translation disabled.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_init(Udma_DrvHandle drvHandle, const Udma_InitPrms *initPrms);

/**
 *  \brief UDMA deinit function.
 *
 *  Uninitializes the drivers and the hardware and should be called during
 *  system shutdown. Should not be called if Udma_init() is not called.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2577)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_deinit(Udma_DrvHandle drvHandle);

/*
 * Structure Init functions
 *
 * Requirement: DOX_REQ_TAG(PDK-2600)
 */
/**
 *  \brief Udma_InitPrms structure init function.
 *
 *  Note: API returns error when there is a failure in intilaizing
 *  RM parameters. This can be due to the following reasons:
 *  - Wrong entry for resources in Default Board Cfg
 *    (Sciclient_defaultBoardCfg_rm.c)
 *  - Number of resources reserved in Default Board Cfg is less
 *    than the 'minumum requirement' specified as per
 *    UDMA RM Shared resource parameters \ref Udma_RmSharedResPrms.
 *    In this case, user should reserve more resources in Default Board Cfg
 *    OR override the default UDMA RM Shared resource parameters.
 *    (Use \ref Udma_rmGetSharedResPrms API to get default
 *     UDMA RM Shared resource parameters)
 *  - Total number of resources requested for each instance
 *    as per UDMA RM Shared resource parameters
 *    is greater than the number of resorurces reserved in Default
 *    Board Cfg.
 *    In this case, user should reduce the requested share for each
 *    instance in UDMA RM Shared resource parameters.
 *
 *  \param instId       [IN] \ref Udma_InstanceIdSoc
 *  \param initPrms     [IN] Pointer to #Udma_InitPrms structure.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t UdmaInitPrms_init(uint32_t instId, Udma_InitPrms *initPrms);

/**
 *  \brief Default virtual to physical translation function.
 *
 *  \param virtAddr [IN] Virtual address
 *  \param chNum    [IN] Channel number passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to #UDMA_DMA_CH_INVALID.
 *  \param appData  [IN] Callback pointer passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to NULL.
 *
 *  \return Corresponding physical address
 */
uint64_t Udma_defaultVirtToPhyFxn(const void *virtAddr,
                                  uint32_t chNum,
                                  void *appData);

/**
 *  \brief Default physical to virtual translation function.
 *
 *  \param phyAddr  [IN] Physical address
 *  \param chNum    [IN] Channel number passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to #UDMA_DMA_CH_INVALID.
 *  \param appData  [IN] Callback pointer passed during channel open.
 *                       Note: When called for functions which is not channel
 *                       dependent (like ring alloc), this parameter will
 *                       be set to NULL.
 *
 *  \return Corresponding virtual address
 */
void *Udma_defaultPhyToVirtFxn(uint64_t phyAddr,
                               uint32_t chNum,
                               void *appData);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief Opaque UDMA driver object.
 */
typedef struct Udma_DrvObject_t
{
    uintptr_t rsv[635U];
    /**< reserved, should NOT be modified by end users */
} Udma_DrvObject;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_H_ */

/** @} */
