/**********************************************************************
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 13.05.b3ee589
*          Do not edit it manually.
**********************************************************************
* Layer interface for the Cadence DMA controller
**********************************************************************/

#ifndef CUSBDMA_IF_H
#define CUSBDMA_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cdn_stdtypes.h"
#include "sgdma_regs.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** The TRB chain descriptor is unused and free */
#define CUSBDMA_TRB_CHAIN_FREE 0U

/** The TRB chain descriptor is queued for processing */
#define CUSBDMA_TRB_CHAIN_QUEUED 1U

/** The TRB chain descriptor is partially processed */
#define CUSBDMA_TRB_CHAIN_PARTIAL 2U

/** The TRB chain descriptor is completely processed */
#define CUSBDMA_TRB_CHAIN_COMPLETE 3U

/** Number of channel supported by DMA driver. This is used internally by the driver. */
#define CUSBDMA_MAX_DMA_CHANNELS 16U

/** Number of element for holding information about TRB chain. We assume that for every channel for IN and OUT direction we can allocate average one TRBS chain. */
#define CUSBDMA_TRB_SIZE_OF_DMA_CHAIN (CUSBDMA_MAX_DMA_CHANNELS * 16U)

/** Number of IN endpoints */
#define CUSBDMA_NUM_OF_IN_ENDPOINTS 16U

/** OUT endpoint offset */
#define CUSBDMA_OUT_ENDPOINT_OFFSET 16U

/**
 *  @}
 */

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Forward declarations
**********************************************************************/
typedef struct CUSBDMA_DmaTrb_s CUSBDMA_DmaTrb;
typedef struct CUSBDMA_MemResources_s CUSBDMA_MemResources;
typedef struct CUSBDMA_Config_s CUSBDMA_Config;
typedef struct CUSBDMA_SysReq_s CUSBDMA_SysReq;
typedef struct CUSBDMA_ChannelParams_s CUSBDMA_ChannelParams;
typedef struct CUSBDMA_DmaTrbChainDesc_s CUSBDMA_DmaTrbChainDesc;
typedef struct CUSBDMA_DmaChannel_s CUSBDMA_DmaChannel;
typedef struct CUSBDMA_DmaController_s CUSBDMA_DmaController;
typedef struct CUSBDMA_DmaTransferParam_s CUSBDMA_DmaTransferParam;

/**********************************************************************
* Enumerations
**********************************************************************/
/** DMA Channel Status available values */
typedef enum
{
    /** channel unallocated */
    CUSBDMA_STATUS_UNKNOW = 0U,
    /** channel allocated but not busy, no errors */
    CUSBDMA_STATUS_FREE = 1U,
    /** DMA channel is stalled */
    CUSBDMA_STATUS_STALLED = 2U,
    /** channel busy - transfer in progress */
    CUSBDMA_STATUS_BUSY = 3U,
    /** DMA has been armed and Doorbell or DMA BUSY bit is still set.  If those bits are set it indicate that DMA is armed and data has not been transfered yet */
    CUSBDMA_STATUS_ARMED = 4U
} CUSBDMA_Status;

/**
 *  @}
 */

/** @defgroup DriverFunctionAPI Driver Function API
 *  Prototypes for the driver API functions. The user application can link statically to the
 *  necessary API functions and call them directly.
 *  @{
 */

/**********************************************************************
* API methods
**********************************************************************/

/**
 * Obtain the private memory size required by the driver.
 * @param[in] config driver/hardware configuration required
 * @param[out] sysReq sysReq returns the size of memory allocations required
 * @return CDN_EOK on success
 * @return CDN_EINVAL if function parameters are not valid
 * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints
 */
uint32_t CUSBDMA_Probe(const CUSBDMA_Config* config, CUSBDMA_SysReq* sysReq);

/**
 * Initialize the driver instance and state, configure the USB device
 * as specified in the 'config' settings, initialize locks used by the
 * driver.
 * @param[in] pD driver state info specific to this instance
 * @param[in] config specifies driver/hardware configuration
 * @return CDN_EOK on success
 * @return CDN_EINVAL if function parameters are not valid
 * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints
 * @return CDN_EIO if driver encountered an error accessing hardware
 * @return CDN_ENOENT insufficient locks were available (i.e. something allocated locks between probe and init)
 */
uint32_t CUSBDMA_Init(CUSBDMA_DmaController* pD, const CUSBDMA_Config* config);

/**
 * Destroy the driver (automatically performs a stop).
 * @param[in] pD driver state info specific to this instance
 */
uint32_t CUSBDMA_Destroy(CUSBDMA_DmaController* pD);

/**
 * Allocation the DMA channel
 * @param[in] pD driver state info specific to this instance
 * @param[out] channelPtr address of channel pointer
 * @param[in] channelParams Channel parameters
 */
uint32_t CUSBDMA_ChannelAlloc(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel** channelPtr, CUSBDMA_ChannelParams* channelParams);

/**
 * Reset the DMA channel
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel channel pointer to DMA channel
 * @return CDN_EOK on success
 * @return CDN_EINVAL if function parameters are not valid
 * @return CDN_EPERM if the DMA is active and channel cannot be reset
 */
uint32_t CUSBDMA_ChannelReset(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel);

/**
 * Release the DMA channel
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel channel pointer to DMA channel
 * @return CDN_EOK on success
 * @return CDN_EINVAL if function parameters are not valid
 */
uint32_t CUSBDMA_ChannelRelease(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel);

/**
 * Prepares transfer and starts it.
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel channel pointer to DMA channel for which transfer will be started
 * @param[in] params transfer parameters container
 * @return CDN_EOK on success
 * @return CDN_EINVAL if function parameters are not valid
 * @return CDN_ENOMEM if DMA channel has no free memory for TRB
 */
uint32_t CUSBDMA_ChannelProgram(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel, const CUSBDMA_DmaTransferParam* params);

/**
 * Triggers DMA transfer for given DMA channel if TRBs are queued
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel pointer to DMA channel which needs to be triggered
 * @return CDN_EOK on successful trigger or if the DMA is already active
 * @return CDN_EINVAL if function parameters are not valid
 * @return CDN_ENOTSUP if TRBs are not queued
 */
uint32_t CUSBDMA_ChannelTrigger(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel);

/**
 * Updates the data transfer status of a channel
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel pointer to DMA channel whose state needs to be updated
 * @return CDN_EOK on successful trigger or if the DMA is already active
 * @return CDN_EINVAL if function parameters are not valid
 */
uint32_t CUSBDMA_ChannelUpdateState(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel);

/**
 * Updates the max packet size for a channel
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel pointer to DMA channel whose state needs to be updated
 * @param[in] maxPacketSize Value of max packet size
 * @return CDN_EOK on successful successful update of max packet size
 * @return CDN_EINVAL if function parameters are not valid
 */
uint32_t CUSBDMA_ChannelSetMaxPktSz(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel, uint16_t maxPacketSize);

/**
 * Set or Clear channel stall
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel pointer to DMA channel whose stall state is handled
 * @param[in] value Clear stall if 0, else set stall
 * @param[in] timeout Timeout for waiting for flush operation while stalling
 * @return CDN_EOK on successful trigger or if the DMA is already active
 * @return CDN_EINVAL if function parameters are not valid
 */
uint32_t CUSBDMA_ChannelHandleStall(CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel, uint32_t value, uint32_t timeout);

/**
 * Free the head(oldest) TRB chain descriptor for this channel.
 * @param[in] pD driver state info specific to this instance
 * @param[in] channel pointer to DMA channel whose descriptor needs to be freed
 * @return CDN_EOK on success
 * @return CDN_EINVAL if function parameters are not valid
 */
uint32_t CUSBDMA_ChannelFreeHeadTrbChain(const CUSBDMA_DmaController* pD, CUSBDMA_DmaChannel* channel);

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSBDMA_IF_H */
