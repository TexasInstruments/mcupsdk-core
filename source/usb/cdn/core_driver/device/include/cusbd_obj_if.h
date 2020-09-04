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
#ifndef CUSBD_OBJ_IF_H
#define CUSBD_OBJ_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cusbd_if.h"

/** @defgroup DriverObject Driver API Object
 *  API listing for the driver. The API is contained in the object as
 *  function pointers in the object structure. As the actual functions
 *  resides in the Driver Object, the client software must first use the
 *  global GetInstance function to obtain the Driver Object Pointer.
 *  The actual APIs then can be invoked using obj->(api_name)() syntax.
 *  These functions are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* API methods
**********************************************************************/

/* parasoft-begin-suppress MISRA2012-DIR-4_8-4 "Consider hiding the implementation of the structure, DRV-4932" */
typedef struct CUSBD_OBJ_s
{
    /**
     * Obtain the private memory size required by the driver
     * @param[in] config driver/hardware configuration required
     * @param[out] sysReqCusbd returns the size of memory allocations required
     * @return 0 on success (requirements structure filled)
     * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints
     */
    uint32_t (*probe)(const CUSBD_Config* config, CUSBD_SysReq* sysReqCusbd);

    /**
     * Initialize the driver instance and state, configure the USB device
     * as specified in the 'config' settings, initialize locks used by
     * the driver.
     * @param[out] pD driver state info specific to this instance
     * @param[in,out] config specifies driver/hardware configuration
     * @param[in] callbacks client-supplied callback functions
     * @return CDN_EOK on success
     * @return CDN_ENOTSUP if hardware has an inconsistent configuration or doesn't support feature(s) required by 'config' parameters
     * @return CDN_ENOENT if insufficient locks were available (i.e. something allocated locks between probe and init)
     * @return CDN_EIO if driver encountered an error accessing hardware
     * @return CDN_EINVAL if illegal/inconsistent values in 'config'
     */
    uint32_t (*init)(CUSBD_PrivateData* pD, const CUSBD_Config* config, const CUSBD_Callbacks* callbacks);

    /**
     * Destroy the driver (automatically performs a stop)
     * @param[in] pD driver state info specific to this instance
     * @return CDN_EOK on success
     * @return CDN_EINVAL if illegal/inconsistent values in 'config'
     */
    uint32_t (*destroy)(const CUSBD_PrivateData* pD);

    /**
     * Start the USB driver.
     * @param[in] pD driver state info specific to this instance
     * @return CDN_EOK on success
     * @return CDN_EINVAL if illegal/inconsistent values in 'config'
     */
    uint32_t (*start)(CUSBD_PrivateData* pD);

    /**
     * Stop the driver. This should disable the hardware, including its
     * interrupt at the source, and make a best-effort to cancel any
     * pending transactions.
     * @param[in] pD driver state info specific to this instance
     * @return CDN_EOK on success
     * @return CDN_EINVAL if any input parameter is NULL
     */
    uint32_t (*stop)(CUSBD_PrivateData* pD);

    /**
     * Driver ISR.  Platform-specific code is responsible for ensuring
     * this gets called when the corresponding hardware's interrupt is
     * asserted. Registering the ISR should be done after calling init,
     * and before calling start. The driver's ISR will not attempt to
     * lock any locks, but will perform client callbacks. If the client
     * wishes to defer processing to non-interrupt time, it is
     * responsible for doing so.
     * @param[in] pD driver state info specific to this instance
     */
    uint32_t (*isr)(CUSBD_PrivateData* pD);

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
    uint32_t (*epEnable)(CUSBD_PrivateData* pD, CUSBD_Ep* ep, const uint8_t* desc);

    /**
     * Disable Endpoint. Functions unconfigures hardware endpoint.
     * Endpoint will not respond to any host packets. This function
     * should be called within SET_CONFIGURATION(configNum = 0) request
     * context or on disconnect event. All queued requests on endpoint
     * are completed with CDN_ECANCELED status.
     * @param[in] pD driver state info specific to this instance
     * @param[in] ep endpoint being unconfigured
     * @return 0 on success
     * @return CDN_EINVAL if pD or ep is NULL
     */
    uint32_t (*epDisable)(CUSBD_PrivateData* pD, CUSBD_Ep* ep);

    /**
     * Set halt or clear halt state on endpoint. When setting halt,
     * device will respond with STALL packet to each host packet. When
     * clearing halt, endpoint returns to normal operating.
     * @param[in] pD driver state info specific to this instance
     * @param[in] ep endpoint being setting or clearing halt state
     * @param[in] value if 1 sets halt, if 0 clears halt on endpoint
     * @return 0 on success
     * @return CDN_EPERM if endpoint is disabled (has not been configured yet)
     * @return CDN_EINVAL if pD or ep is NULL
     */
    uint32_t (*epSetHalt)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, uint8_t value);

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
    uint32_t (*epSetWedge)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

    /**
     * Returns number of bytes in hardware endpoint FIFO. Function useful
     * in application where exact number of data bytes is required. In
     * some situation software higher layers must be aware of number of
     * data bytes issued but not transfered by hardware because of
     * aborted transfers, for example on disconnect event. *
     * @param[in] pD driver state info specific to this instance
     * @param[in] ep endpoint which status is to be returned
     * @return number of bytes in hardware endpoint FIFO
     * @return CDN_EINVAL if pD or ep is NULL
     */
    uint32_t (*epFifoStatus)(const CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

    /**
     * Flush hardware endpoint FIFO.
     * @param[in] pD driver state info specific to this instance
     * @param[in] ep endpoint which FIFO is to be flushed
     */
    uint32_t (*epFifoFlush)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep);

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
    uint32_t (*reqQueue)(CUSBD_PrivateData* pD, const CUSBD_Ep* ep, CUSBD_Req* req);

    /**
     * Dequeues IN/OUT transfer request from an endpoint. Function
     * completes all queued request with CDN_ECANCELED status.
     * @param[in] pD driver state info specific to this instance
     * @param[in] ep endpoint associated with the request
     * @param[in] req request being dequeued
     * @return 0 on success
     * @return CDN_EINVAL if pD or ep is NULL
     */
    uint32_t (*reqDequeue)(CUSBD_PrivateData* pD, CUSBD_Ep* ep, CUSBD_Req* req);

    /**
     * Returns pointer to CUSBD object. CUSBD object is a logical
     * representation of USB device. CUSBD contains endpoint collection
     * accessed through epList field. Endpoints collection is organized
     * as double linked list.
     * @param[in] pD driver state info specific to this instance
     * @param[out] dev returns pointer to CUSBD instance
     */
    void (*getDevInstance)(CUSBD_PrivateData* pD, CUSBD_Dev** dev);

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
    uint32_t (*dGetFrame)(CUSBD_PrivateData* pD, uint32_t* numOfFrame);

    /**
     * Sets the device self powered feature.
     * @param[in] pD driver state info specific to this instance
     * @return CDN_EOK on success
     * @return CDN_EOPNOTSUPP if feature is not supported
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*dSetSelfpowered)(const CUSBD_PrivateData* pD);

    /**
     * Clear the device self powered feature.
     * @param[in] pD driver state info specific to this instance
     * @return CDN_EOK on success
     * @return CDN_EOPNOTSUPP if feature is not supported
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*dClearSelfpowered)(const CUSBD_PrivateData* pD);

    /**
     * Returns configuration parameters: U1 exit latency and U2 exit
     * latency Function useful only in Super Speed mode.
     * @param[in] pD driver state info specific to this instance
     * @param[out] configParams pointer to CH9_ConfigParams structure
     * @return CDN_EOK on success
     * @return CDN_EINVAL if illegal/inconsistent values in 'config'
     */
    uint32_t (*dGetConfigParams)(const CUSBD_PrivateData* pD, CH9_ConfigParams* configParams);

} CUSBD_OBJ;
/* parasoft-end-suppress MISRA2012-DIR-4_8-4 */

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3666" */
/**
 * In order to access the CUSBD APIs, the upper layer software must call
 * this global function to obtain the pointer to the driver object.
 * @return CUSBD_OBJ* Driver Object Pointer
 */
extern CUSBD_OBJ *CUSBD_GetInstance(void);

/**
 *  @}
 */
/* parasoft-end-suppress METRICS-36-3 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSBD_OBJ_IF_H */
