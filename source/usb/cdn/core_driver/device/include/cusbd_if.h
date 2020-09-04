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
* Layer interface for the Cadence USB device controller family
**********************************************************************/

#ifndef CUSBD_IF_H
#define CUSBD_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cusb_ch9_if.h"
#include "cusbdma_if.h"
#include "list_if.h"
#include "cdn_stdtypes.h"
#include "ss_dev_hw.h"
#include "cusbdma_obj_if.h"
#include "cusbdma_structs_if.h"
#include "list_structs_if.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** DID value */
#define CUSBD_DID_VAL 0x0004024EU

/** RID value */
#define CUSBD_RID_VAL 0x0200U

/** Number of out endpoints */
#define CUSBD_NUM_EP_OUT 15U

/** Number of in endpoints */
#define CUSBD_NUM_EP_IN 15U

/** Inactivity timeout value corresponding to max value of LFPS_POLLING_MAX_TREPEAT */
#define CUSBD_INACTIVITY_TMOUT 0x7FFFU

/** Precise burst length 0 (valid for OCP, AHB, AXI3) */
#define CUSBD_PRECISE_BURST_0 0x00000000U

/** Precise burst length 1 (valid for OCP, AHB, AXI3) */
#define CUSBD_PRECISE_BURST_1 0x01000000U

/** Precise burst length 2 (valid for OCP, AXI3) */
#define CUSBD_PRECISE_BURST_2 0x02000000U

/** Precise burst length 4 (valid for OCP, AHB, AXI3) */
#define CUSBD_PRECISE_BURST_4 0x04000000U

/** Precise burst length 8 (valid for OCP, AHB, AXI3) */
#define CUSBD_PRECISE_BURST_8 0x08000000U

/** Precise burst length 16 (valid for OCP, AHB, AXI3) */
#define CUSBD_PRECISE_BURST_16 0x10000000U

/** Precise burst length 32 (valid for OCP, AXI3) */
#define CUSBD_PRECISE_BURST_32 0x20000000U

/** Precise burst length 64 (valid for OCP, AXI3) */
#define CUSBD_PRECISE_BURST_64 0x40000000U

/** Precise burst length 128 (valid for OCP, AXI3) */
#define CUSBD_PRECISE_BURST_128 0x80000000U

/** Endianess conversion flag. Value is 0 to disable it and 1 to enable it */
#define CUSBD_ENDIANESS_CONV_FLAG 0U

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
typedef struct CUSBD_EpAuxBufferConfig_s CUSBD_EpAuxBufferConfig;
typedef struct CUSBD_EpAuxBuffer_s CUSBD_EpAuxBuffer;
typedef struct CUSBD_EpConfig_s CUSBD_EpConfig;
typedef struct CUSBD_Config_s CUSBD_Config;
typedef struct CUSBD_SysReq_s CUSBD_SysReq;
typedef struct CUSBD_SgList_s CUSBD_SgList;
typedef struct CUSBD_Req_s CUSBD_Req;
typedef struct CUSBD_EpOps_s CUSBD_EpOps;
typedef struct CUSBD_Ep_s CUSBD_Ep;
typedef struct CUSBD_Dev_s CUSBD_Dev;
typedef struct CUSBD_Callbacks_s CUSBD_Callbacks;
typedef struct CUSBD_EpPrivate_s CUSBD_EpPrivate;
typedef struct CUSBD_PrivateData_s CUSBD_PrivateData;

/**********************************************************************
* Enumerations
**********************************************************************/
/** DMA BUS width available values */
typedef enum
{
    /** DMA bus width 64 */
    CUSBD_DMA_64_WIDTH = 8U
} CUSBD_DMAInterfaceWidth;

/** EndPoint state available values */
typedef enum
{
    /** Endpoint disabled */
    CUSBD_EP_DISABLED = 0U,
    /** Endpoint enabled */
    CUSBD_EP_ENABLED = 1U,
    /** Endpoint stalled */
    CUSBD_EP_STALLED = 2U
} CUSBD_epState;

/**********************************************************************
* Callbacks
**********************************************************************/
/**
 * Reports request complete callback.
 * Params:
 * ep - endpoint associated with the request
 * req - request which has been completed
 * Note that request received in completion routine may be used again
 */
typedef void (*CUSBD_CbReqComplete)(CUSBD_Ep* ep, CUSBD_Req* req);

/**
 * Enable Endpoint.  This function should be called within
 * SET_CONFIGURATION(configNum > 0) request context. It configures
 * required hardware controller endpoint with parameters given in desc.
 */
typedef uint32_t (*CUSBD_CbEpEnable)(CUSBD_PrivateData* pD, CUSBD_Ep* ep, const uint8_t* desc);

/**
 * Disable Endpoint. Functions unconfigures hardware endpoint.
 * Endpoint will not respond to any host packets. This function
 * should be called within SET_CONFIGURATION(configNum = 0) request
 * context or on disconnect event. All queued requests on endpoint
 * are completed with CDN_ECANCELED status.
 */
typedef uint32_t (*CUSBD_CbEpDisable)(CUSBD_PrivateData* pD, CUSBD_Ep* ep);

/**
 * Set halt or clear halt state on endpoint. When setting halt, device
 * will respond with STALL packet to each host packet. When clearing
 * halt, endpoint returns to normal operating.
 */
typedef uint32_t (*CUSBD_CbEpSetHalt)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, uint8_t value);

/**
 * Set halt on endpoint. Function sets endpoint to permanent halt
 * state. State can not be changed even with epSetHalt(pD, ep, 0) function.
 * Endpoint returns automatically to its normal state on disconnect
 * event.
 */
typedef uint32_t (*CUSBD_CbEpSetWedge)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

/**
 * Returns number of bytes in hardware endpoint FIFO. Function useful in
 * application where exact number of data bytes is required. In some
 * situation software higher layers must be aware of number of data
 * bytes issued but not transfered by hardware because of aborted
 * transfers, for example on disconnect event.
 */
typedef uint32_t (*CUSBD_CbEpFifoStatus)(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

/** Flush hardware endpoint FIFO. */
typedef uint32_t (*CUSBD_CbEpFifoFlush)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

/** Submits IN/OUT transfer request to an endpoint. */
typedef uint32_t (*CUSBD_CbReqQueue)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, CUSBD_Req* req);

/**
 * Dequeues IN/OUT transfer request from an endpoint. Function completes
 * all queued request with CDN_ECANCELED status.
 */
typedef uint32_t (*CUSBD_CbReqDequeue)(CUSBD_PrivateData* pD, CUSBD_Ep* ep, CUSBD_Req* req);

/**
 * Callback called on disconnect from host event. All queued transfer on all
 * active endpoints are completed by driver with CDN_ECANCELED status.
 */
typedef void (*CUSBD_CbDisconnect)(CUSBD_PrivateData* pD);

/**
 * Callback called on connection to host event. To check speed at which
 * device has connected to host read speed flag of CUSBD object
 */
typedef void (*CUSBD_CbConnect)(CUSBD_PrivateData* pD);

/**
 * Callback called on setup request received event
 * Params: ctrl contains usb setup request in little endian order
 */
typedef uint32_t (*CUSBD_CbSetup)(CUSBD_PrivateData* pD, CH9_UsbSetup* ctrl);

/** Called when device controller goes into suspend state. */
typedef void (*CUSBD_CdSuspend)(CUSBD_PrivateData* pD);

/** Called when device returns from suspend state. */
typedef void (*CUSBD_CbResume)(CUSBD_PrivateData* pD);

/** Called on bus interval event only if any isochronous endpoint used. */
typedef void (*CUSBD_CbbusInterval)(CUSBD_PrivateData* pD);

/** Called when descriptor missing event received */
typedef void (*CUSBD_CbDescMissing)(CUSBD_PrivateData* pD, uint8_t epAddr);

/** Called to perform soft reset of USB 2 PHY */
typedef void (*CUSBD_CbUSB2PhySoftReset)(CUSBD_PrivateData* pD);

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
 * Obtain the private memory size required by the driver
 * @param[in] config driver/hardware configuration required
 * @param[out] sysReqCusbd returns the size of memory allocations required
 * @return 0 on success (requirements structure filled)
 * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints
 */
uint32_t CUSBD_Probe(const CUSBD_Config* config, CUSBD_SysReq* sysReqCusbd);

/**
 * Initialize the driver instance and state, configure the USB device
 * as specified in the 'config' settings, initialize locks used by the
 * driver.
 * @param[out] pD driver state info specific to this instance
 * @param[in,out] config specifies driver/hardware configuration
 * @param[in] callbacks client-supplied callback functions
 * @return CDN_EOK on success
 * @return CDN_ENOTSUP if hardware has an inconsistent configuration or doesn't support feature(s) required by 'config' parameters
 * @return CDN_ENOENT if insufficient locks were available (i.e. something allocated locks between probe and init)
 * @return CDN_EIO if driver encountered an error accessing hardware
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Init(CUSBD_PrivateData* pD, const CUSBD_Config* config, const CUSBD_Callbacks* callbacks);

/**
 * Destroy the driver (automatically performs a stop)
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Destroy(const CUSBD_PrivateData* pD);

/**
 * Start the USB driver.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_Start(CUSBD_PrivateData* pD);

/**
 * Stop the driver. This should disable the hardware, including its
 * interrupt at the source, and make a best-effort to cancel any
 * pending transactions.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EINVAL if any input parameter is NULL
 */
uint32_t CUSBD_Stop(CUSBD_PrivateData* pD);

/**
 * Driver ISR.  Platform-specific code is responsible for ensuring
 * this gets called when the corresponding hardware's interrupt is
 * asserted. Registering the ISR should be done after calling init,
 * and before calling start. The driver's ISR will not attempt to lock
 * any locks, but will perform client callbacks. If the client wishes
 * to defer processing to non-interrupt time, it is responsible for
 * doing so.
 * @param[in] pD driver state info specific to this instance
 */
uint32_t CUSBD_Isr(CUSBD_PrivateData* pD);

/**
 * Enable Endpoint.  This function should be called within
 * SET_CONFIGURATION(configNum > 0) request context. It configures
 * required hardware controller endpoint with parameters given in
 * desc.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being configured
 * @param[in] desc endpoint descriptor
 * @return 0 on success
 * @return CDN_EINVAL if pD, ep or desc is NULL or desc is not a endpoint descriptor
 */
uint32_t CUSBD_EpEnable(CUSBD_PrivateData* pD, CUSBD_Ep* ep, const uint8_t* desc);

/**
 * Disable Endpoint. Functions unconfigures hardware endpoint.
 * Endpoint will not respond to any host packets. This function should
 * be called within SET_CONFIGURATION(configNum = 0) request context
 * or on disconnect event. All queued requests on endpoint are
 * completed with CDN_ECANCELED status.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being unconfigured
 * @return 0 on success
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpDisable(CUSBD_PrivateData* pD, CUSBD_Ep* ep);

/**
 * Set halt or clear halt state on endpoint. When setting halt, device
 * will respond with STALL packet to each host packet. When clearing
 * halt, endpoint returns to normal operating.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being setting or clearing halt state
 * @param[in] value if 1 sets halt, if 0 clears halt on endpoint
 * @return 0 on success
 * @return CDN_EPERM if endpoint is disabled (has not been configured yet)
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpSetHalt(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, uint8_t value);

/**
 * Set halt on endpoint. Function sets endpoint to permanent halt
 * state. State can not be changed even with epSetHalt(pD, ep, 0)
 * function. Endpoint returns automatically to its normal state on
 * disconnect event.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint being setting halt state
 * @return 0 on success
 * @return CDN_EPERM if endpoint is disabled (has not been configured yet)
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpSetWedge(CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

/**
 * Returns number of bytes in hardware endpoint FIFO. Function useful
 * in application where exact number of data bytes is required. In
 * some situation software higher layers must be aware of number of
 * data bytes issued but not transfered by hardware because of aborted
 * transfers, for example on disconnect event. *
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint which status is to be returned
 * @return number of bytes in hardware endpoint FIFO
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_EpFifoStatus(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

/**
 * Flush hardware endpoint FIFO.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint which FIFO is to be flushed
 */
uint32_t CUSBD_EpFifoFlush(CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

/**
 * Submits IN/OUT transfer request to an endpoint.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint associated with the request
 * @param[in] req request being submitted
 * @return 0 on success
 * @return CDN_EPROTO only on default endpoint if endpoint is not in data stage phase
 * @return CDN_EPERM if endpoint is not enabled
 * @return CDN_EINVAL if pD, ep, or req is NULL
 */
uint32_t CUSBD_ReqQueue(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, CUSBD_Req* req);

/**
 * Dequeues IN/OUT transfer request from an endpoint. Function
 * completes all queued request with CDN_ECANCELED status.
 * @param[in] pD driver state info specific to this instance
 * @param[in] ep endpoint associated with the request
 * @param[in] req request being dequeued
 * @return 0 on success
 * @return CDN_EINVAL if pD or ep is NULL
 */
uint32_t CUSBD_ReqDequeue(CUSBD_PrivateData* pD, CUSBD_Ep* ep, CUSBD_Req* req);

/**
 * Returns pointer to CUSBD object. CUSBD object is a logical
 * representation of USB device. CUSBD contains endpoint collection
 * accessed through epList field. Endpoints collection is organized as
 * double linked list.
 * @param[in] pD driver state info specific to this instance
 * @param[out] dev returns pointer to CUSBD instance
 */
void CUSBD_GetDevInstance(CUSBD_PrivateData* pD, CUSBD_Dev** dev);

/**
 * Returns number of frame. Some controllers have counter of SOF
 * packets or ITP packets in the Super Speed case. Function returns
 * value of this counter. This counter can be used for time
 * measurement. Single FS frame is 1 ms measured, for HS and SS is
 * 125us.
 * @param[in] pD driver state info specific to this instance
 * @param[out] numOfFrame returns number of USB frame
 * @return CDN_EOK on success
 * @return CDN_EOPNOTSUPP if feature is not supported
 * @return CDN_EINVAL if pD or numOfFrame is NULL
 */
uint32_t CUSBD_DGetFrame(CUSBD_PrivateData* pD, uint32_t* numOfFrame);

/**
 * Sets the device self powered feature.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EOPNOTSUPP if feature is not supported
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t CUSBD_DSetSelfpowered(const CUSBD_PrivateData* pD);

/**
 * Clear the device self powered feature.
 * @param[in] pD driver state info specific to this instance
 * @return CDN_EOK on success
 * @return CDN_EOPNOTSUPP if feature is not supported
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t CUSBD_DClearSelfpowered(const CUSBD_PrivateData* pD);

/**
 * Returns configuration parameters: U1 exit latency and U2 exit
 * latency Function useful only in Super Speed mode.
 * @param[in] pD driver state info specific to this instance
 * @param[out] configParams pointer to CH9_ConfigParams structure
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in 'config'
 */
uint32_t CUSBD_DGetConfigParams(const CUSBD_PrivateData* pD, CH9_ConfigParams* configParams);

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSBD_IF_H */
