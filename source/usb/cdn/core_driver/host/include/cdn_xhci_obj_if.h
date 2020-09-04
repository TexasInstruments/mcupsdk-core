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
* XHCI driver for both host and device mode header file
**********************************************************************/
#ifndef CDN_XHCI_OBJ_IF_H
#define CDN_XHCI_OBJ_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_xhci_if.h"

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
typedef struct USBSSP_OBJ_s
{
    /**
     * Transfer data on given endpoint. This function is non-blocking
     * type. The XHCI operation result should be checked in complete
     * callback function.
     * @param[in] res Driver resources
     * @param[in] epIndex index of endpoint according to xhci spec e.g for ep1out
     *    epIndex=2, for ep1in epIndex=3, for ep2out epIndex=4 end so on $RANGE $FROM USBSSP_EP_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
     * @param[in] buff Buffer for data to send or to receive
     * @param[in] size Size of data in bytes
     * @param[in] complete pointer to function which will be returned in callback in input parameter,
     *    can be set to NULL when no extra parameter used
     * @return CDN_EINVAL if selected endpoint index is out of available range
     * @return CDN_EOK if selected endpoint is within available endpoint range
     */
    uint32_t (*transferData)(USBSSP_DriverResourcesT* res, uint8_t epIndex, const uintptr_t buff, uint32_t size, USBSSP_Complete complete);

    /**
     * Gather and Transfer Vector data
     * @param[in] res Driver resources
     * @param[in] epIndex index of endpoint according to xhci spec e.g for ep1out
     *    epIndex=2, for ep1in epIndex=3, for ep2out epIndex=4 end so on $RANGE $FROM USBSSP_EP_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
     * @param[in] bufferDesc Pointer to an array of buffer descriptors
     * @param[in] bufferCount Buffer descriptor count
     * @param[in] complete pointer to function which will be returned in callback in input parameter,
     *    can be set to NULL when no extra parameter used
     * @return CDN_EINVAL if selected endpoint index is out of available range
     * @return CDN_EOK if selected endpoint is within available endpoint range
     */
    uint32_t (*transferVectorData)(USBSSP_DriverResourcesT* res, uint8_t epIndex, const USBSSP_XferBufferDesc* bufferDesc, uint32_t bufferCount, USBSSP_Complete complete);

    /**
     * Stop endpoint. Function sends STOP_ENDPOINT_COMMAND command to
     * USBSSP controller
     * @param[in] res Driver resources
     * @param[in] endpoint Index of endpoint to stop $RANGE $FROM USBSSP_EP0_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*stopEndpoint)(USBSSP_DriverResourcesT* res, uint8_t endpoint);

    /**
     * Endpoint reset. Function sends RESET_ENDPOINT_COMMAND to USBSSP
     * controller
     * @param[in] res Driver resources
     * @param[in] endpoint Index of endpoint to reset $RANGE $FROM USBSSP_EP0_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*resetEndpoint)(USBSSP_DriverResourcesT* res, uint8_t endpoint);

    /**
     * Reset of connected device. Function sends RESET_DEVICE_COMMAND to
     * USBSSP controller in order to issue reset state on USB bus.
     * @param[in] res Driver resources
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK selected endpoint is within available endpoint range
     */
    uint32_t (*resetDevice)(USBSSP_DriverResourcesT* res);

    /**
     * Handling of USBSSP controller interrupt. Function is called from
     * USBSSP interrupt context.
     * @param[in] res Driver resources
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*isr)(USBSSP_DriverResourcesT* res);

    /**
     * Function sets memory resources used by driver.
     * @param[in] res Driver resources
     * @param[in] memRes User defined memory resources.
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*SetMemRes)(USBSSP_DriverResourcesT* res, USBSSP_XhciResourcesT* memRes);

    /**
     * Initialization of USBSSP_DriverResourcesT object.
     * USBSSP_DriverResourcesT object keeps all resources required by
     * USBSSP controller. It represents USBSSP hardware controller.
     * @param[in] res Driver resources
     * @param[in] base Physical address of USBSSP controller where is mapped in system
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*init)(USBSSP_DriverResourcesT* res, uintptr_t base);

    /**
     * Get descriptor. Function gets descriptor from connected device,
     * used in host mode and stores it in internal res->ep0Buff buffer.
     * Maximal descriptor length is limited to 255. Function is blocking
     * type and must not be called from interrupt context.
     * @param[in] res Driver resources
     * @param[in] descType Type of descriptor to get (CH9_USB_DT_DEVICE, CH9_USB_DT_CONFIGURATION,...)
     * @param[in] complete Complete callback function
     * @return CDN_EOK on success
     * @return complete_code XHCI transfer complete status code
     */
    uint32_t (*getDescriptor)(USBSSP_DriverResourcesT* res, uint8_t descType, USBSSP_Complete complete);

    /**
     * Set address. Function executes set address request on connected
     * device.
     * @param[in] res Driver resources
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*setAddress)(USBSSP_DriverResourcesT* res);

    /**
     * Reset port.
     * @param[in] res Driver resources
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*resetRootHubPort)(const USBSSP_DriverResourcesT* res);

    /**
     * Issue generic command to SSP controller.
     * @param[in] res Driver resources
     * @param[in] dword0 word 0 of command
     * @param[in] dword1 word 1 of command
     * @param[in] dword2 word 2 of command
     * @param[in] dword3 word 3 of command
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*issueGenericCommand)(USBSSP_DriverResourcesT* res, uint32_t dword0, uint32_t dword1, uint32_t dword2, uint32_t dword3);

    /**
     * Set feature on device's endpoint. Functions sends setup requested
     * to device with set/cleared endpoint feature
     * @param[in] res Driver resources
     * @param[in] epIndex Index of endpoint to set/clear feature on $RANGE $FROM USBSSP_EP_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
     * @param[in] feature When 1 sets stall, when 0 clears stall
     * @return CDN_EOK on success
     * @return complete_code XHCI transfer complete status code
     */
    uint32_t (*endpointSetFeature)(USBSSP_DriverResourcesT* res, uint8_t epIndex, uint8_t feature);

    /**
     * Set configuration. Function configures USBSSP controller as well
     * as device connected to this USBSSP controller. This is a blocking
     * function. This function must not be called from an interrupt
     * context.
     * @param[in] res Driver resources
     * @param[in] configValue USB device's configuration selector
     * @return CDN_EOK on success
     * @return complete_code XHCI transfer complete status code
     */
    uint32_t (*setConfiguration)(USBSSP_DriverResourcesT* res, uint32_t configValue);

    /**
     * No blocking control transfer. Function executes control transfer.
     * Information about transfer like: data direction, data length,
     * wIndex, wValue etc. are passed in 'setup' parameter.
     * @param[in] res Driver resources
     * @param[in] setup Keeps setup packet
     * @param[in] pdata Pointer for data to send/receive $RANGE $NULLABLE $
     * @param[in] complete Complete callback function
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*nBControlTransfer)(USBSSP_DriverResourcesT* res, const CH9_UsbSetup* setup, const uint8_t* pdata, USBSSP_Complete complete);

    /**
     * No Operation test. Function used for testing purposes:
     * NO_OP_COMMAND is send to USBSSP controller. When event ring
     * receives NO_OP_COMMAND complete it calls complete callback
     * @param[in] res Driver resources
     * @param[in] complete Callback
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*noOpTest)(USBSSP_DriverResourcesT* res, USBSSP_Complete complete);

    /**
     * Calculate full/low speed endpoint interval based on bInterval See
     * xHCI spec Section 6.2.3.6 for more details.
     * @param[in] bInterval bInterval
     * @return value valid endpoint contxt interval value
     */
    uint8_t (*calcFsLsEPIntrptInterval)(uint8_t bInterval);

    /**
     * Function enables slot for new connected device.
     * @param[in] res Driver resources
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*enableSlot)(USBSSP_DriverResourcesT* res);

    /**
     * Function disables slot of connected device.
     * @param[in] res Driver resources
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*disableSlot)(USBSSP_DriverResourcesT* res);

    /**
     * Enable endpoint. Function used in device context.
     * @param[in] res Driver resources
     * @param[in] desc pointer to endpoint descriptor
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*enableEndpoint)(USBSSP_DriverResourcesT* res, const uint8_t* desc);

    /**
     * Disable endpoint. Function used in device context.
     * @param[in] res Driver resources
     * @param[in] epAddress address of endpoint to be disabled
     * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
     * @return CDN_EOK if no errors
     */
    uint32_t (*disableEndpoint)(USBSSP_DriverResourcesT* res, uint8_t epAddress);

    /**
     * Get actual frame number. Function returns actual frame number on
     * USB bus. Remember that maximal frame number can be 0x7FF. Next
     * frame increments returned value from 0.
     * @param[in] res driver resources
     * @param[out] index Micro Frame Index returned by function.
     */
    uint32_t (*getMicroFrameIndex)(USBSSP_DriverResourcesT* res, uint32_t* index);

    /**
     * set end point extra flag
     * @param[in] res driver resources
     * @param[in] epIndex endpoint index
     * @param[in] flags Endpoint Extra Flag
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*setEndpointExtraFlag)(USBSSP_DriverResourcesT* res, uint8_t epIndex, USBSSP_ExtraFlagsEnumT flags);

    /**
     * clean endpoint extra flag
     * @param[in] res driver resources
     * @param[in] epIndex end point index
     * @param[in] flags Endpoint Extra Flag
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*cleanEndpointExtraFlag)(USBSSP_DriverResourcesT* res, uint8_t epIndex, USBSSP_ExtraFlagsEnumT flags);

    /**
     * get endpoint extra flag
     * @param[in] res driver resources
     * @param[in] epIndex endpoint index
     * @param[out] flag Endpoint Extra Flag returned by pointer
     */
    uint32_t (*getEndpointExtraFlag)(const USBSSP_DriverResourcesT* res, uint8_t epIndex, uint8_t* flag);

    /**
     * set Frame ID
     * @param[in] res Driver resources
     * @param[in] epIndex endpoint index
     * @param[in] frameID Frame ID
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*setFrameID)(USBSSP_DriverResourcesT* res, uint8_t epIndex, uint32_t frameID);

    /**
     * Add event data TRB
     * @param[in] res Driver resources
     * @param[in] epIndex endpoint index
     * @param[in] eventDataLo event data Low
     * @param[in] eventDataHi event data High
     * @param[in] flags Flags
     * @return value TBC
     */
    uint32_t (*addEventDataTRB)(USBSSP_DriverResourcesT* res, uint8_t epIndex, uint32_t eventDataLo, uint32_t eventDataHi, uint32_t flags);

    /**
     * Force header
     * @param[in] res driver resources
     * @param[in] trbDwords trb words
     * @param[in] complete complete callback
     * @return CDN_EINVAL Invalid Input parameters
     * @return CDN_EOK if no errors
     */
    uint32_t (*forceHeader)(USBSSP_DriverResourcesT* res, const USBSSP_ForceHdrParams* trbDwords, USBSSP_Complete complete);

    /**
     * set port override register
     * @param[in] res driver resources
     * @param[in] regValue Register value
     */
    uint32_t (*setPortOverrideReg)(const USBSSP_DriverResourcesT* res, uint32_t regValue);

    /**
     * Get port override register
     * @param[in] res Driver resources
     * @param[out] regValue Register value
     */
    uint32_t (*getPortOverrideReg)(const USBSSP_DriverResourcesT* res, uint32_t* regValue);

    /**
     * set port control register
     * @param[in] res driver resources
     * @param[in] portId port ID
     * @param[in] portRegIdx port control register ID
     * @param[in] regValue Register value
     */
    uint32_t (*setPortControlReg)(const USBSSP_DriverResourcesT* res, uint8_t portId, USBSSP_PortControlRegIdx portRegIdx, uint32_t regValue);

    /**
     * Get port control register
     * @param[in] res Driver resources
     * @param[in] portId port ID
     * @param[in] portRegIdx port control register ID
     * @param[out] regValue Register value
     */
    uint32_t (*getPortControlReg)(const USBSSP_DriverResourcesT* res, uint8_t portId, USBSSP_PortControlRegIdx portRegIdx, uint32_t* regValue);

    /**
     * Save state and stop xHC
     * @param[in] res Driver resources
     * @param[in] drvContext Pointer to driver context struct
     */
    uint32_t (*SaveState)(USBSSP_DriverResourcesT* res, USBSSP_DriverContextT* drvContext);

    /**
     * Restore state and start xHC
     * @param[in] res Driver resources
     * @param[in] drvContext Pointer to driver context struct
     */
    uint32_t (*RestoreState)(USBSSP_DriverResourcesT* res, const USBSSP_DriverContextT* drvContext);

    /**
     * Get connected status
     * @param[in] res Driver resources
     * @param[out] connected connected status
     */
    uint32_t (*getPortConnected)(const USBSSP_DriverResourcesT* res, uint8_t* connected);

    /**
     * Get dev address state
     * @param[in] res Driver resources
     * @param[out] addrState dev address state
     */
    uint32_t (*getDevAddressState)(const USBSSP_DriverResourcesT* res, uint8_t* addrState);

} USBSSP_OBJ;
/* parasoft-end-suppress MISRA2012-DIR-4_8-4 */

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions, DRV-3666" */
/**
 * In order to access the USBSSP APIs, the upper layer software must call
 * this global function to obtain the pointer to the driver object.
 * @return USBSSP_OBJ* Driver Object Pointer
 */
extern USBSSP_OBJ *USBSSP_GetInstance(void);

/**
 *  @}
 */
/* parasoft-end-suppress METRICS-36-3 */


#ifdef __cplusplus
}
#endif

#endif  /* CDN_XHCI_OBJ_IF_H */
