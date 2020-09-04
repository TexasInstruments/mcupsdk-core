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

#ifndef CDN_XHCI_IF_H
#define CDN_XHCI_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cusb_ch9_if.h"
#include "cps.h"
#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "cdn_xhci_priv.h"

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
#define USBSSP_DID_VAL 0x0004024EU

/** RID value */
#define USBSSP_RID_VAL 0x0200U

/** extended context */
#define USBSSP_EXTENDED_CONTEXT 0U

/** context width */
#define USBSSP_CONTEXT_WIDTH 8U

/** Debug driver */
#define USBSSP_DBG_DRV 0x000000010U

/** host offset */
#define USBSSP_HOST_OFFSET 0x0000U

/** device offset */
#define USBSSP_DEVICE_OFFSET 0x4000U

/** OTG offset */
#define USBSSP_OTG_OFFSET 0x8000U

/** magic number */
#define USBSSP_MAGIC_NUMBER 0x0004034EU

/** ring alignment */
#define USBSSP_RING_ALIGNMENT 64U

/** ring boundary */
#define USBSSP_RING_BOUNDARY 65536U

/** event ring segment table alignment */
#define USBSSP_ERST_ALIGNMENT 64U

/** event ring segment table boundary */
#define USBSSP_ERST_BOUNDARY 0U

/** event ring segment table entry size */
#define USBSSP_ERST_ENTRY_SIZE 8U

/** context alignment */
#define USBSSP_CONTEXT_ALIGNMENT 64U

/** Device context base address array alignment */
#define USBSSP_DCBAA_ALIGNMENT 64U

/** Number of interrupters supported by software driver. */
#define USBSSP_INTERRUPTER_COUNT 4U

/** Max number of interrupters supported by the hardware. */
#define USBSSP_MAX_NUM_INTERRUPTERS 8U

/** Debug test value */
#define USBSSP_DBG_TEST 0x000000040U

/** Debug external stack */
#define USBSSP_DBG_EXTERNAL_STACK 0x000000080U

/** Event queue size */
#define USBSSP_EVENT_QUEUE_SIZE 64U

/** Maximum device slot supported by the hardware. */
#define USBSSP_MAX_DEVICE_SLOT_NUM 64U

/** scratchpad buffer */
#define USBSSP_SCRATCHPAD_BUFF_NUM ((USBSSP_MAX_DEVICE_SLOT_NUM + 1U) >> 1U)

/** page size */
#define USBSSP_PAGE_SIZE 4096U

/** producer queue size */
#define USBSSP_PRODUCER_QUEUE_SIZE 64U

/** Transfer descriptor queue size */
#define USBSSP_XFER_DESC_QUEUE_SIZE (((USBSSP_PRODUCER_QUEUE_SIZE + 7U) >> 3U) + 1U)

/** max speed */
#define USBSSP_MAX_SPEED 6U

/** max string number */
#define USBSSP_MAX_STRING_NUM 5U

/** max endpoint context */
#define USBSSP_MAX_EP_CONTEXT_NUM 30U

/** max endpoint number of streams */
#define USBSSP_MAX_EP_NUM_STRM_EN 30U

/** given according to XHCI register value: 1 = 4 streams, 2 = 8 streams, 3 = 16 streams and so on */
#define USBSSP_MAX_STREMS_PER_EP 2U

/** Should be calculated according to formula: STREAM_ARRAY_SIZE = 2 exp(MAX_STREMS_PER_EP + 1) */
#define USBSSP_STREAM_ARRAY_SIZE 8U

/** Device mode ports settings */
#define USBSSP_DEV_MODE_2_PORT 0U

/** Device mode ports settings */
#define USBSSP_DEV_MODE_3_PORT 1U

/** Endpoint0 container offset value */
#define USBSSP_EP0_CONT_OFFSET 1U

/** Endpoint container offset value */
#define USBSSP_EP_CONT_OFFSET 2U

/** Endpoint container offset max value */
#define USBSSP_EP_CONT_MAX (USBSSP_MAX_EP_CONTEXT_NUM + USBSSP_EP_CONT_OFFSET)

/** Endpoint 0 data buffer size - used in enumeration */
#define USBSSP_EP0_DATA_BUFF_SIZE 1024U

/** Link is in the RxDetect State */
#define USBSSP_RX_DETECT_LINK_STATE 5U

/** Value returned by API functions to report align error. */
#define USBSSP_EALIGN 130U

/** Value returned by API functions to report memory out of page error. */
#define USBSSP_EPAGE 131U

/** Value returned by USBSSP_Isr to report occurrence of unexpectedly large number of consecutive XHCI events. */
#define USBSSP_EEVENT 132U

/** Status code reported by callback when transfer stalls. */
#define USBSSP_ESTALL 133U

/** Delay in ms for delta-T3 debounce interval before reset */
#define USBSSP_DELAY_T3_DEBOUNCE 1U

/** Delay in ms for delta-T6 reset recovery */
#define USBSSP_DELAY_T6_RECOVERY 1U

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
* Type defines
**********************************************************************/

typedef uint32_t USBSSP_DmaAddrT;

/**********************************************************************
* Forward declarations
**********************************************************************/
typedef struct USBSSP_RingElementT_s USBSSP_RingElementT;
typedef struct USBSSP_InputContexT_s USBSSP_InputContexT;
typedef struct USBSSP_OutputContexT_s USBSSP_OutputContexT;
typedef struct USBSSP_XferDescT_s USBSSP_XferDescT;
typedef struct USBSSP_ProducerQueueT_s USBSSP_ProducerQueueT;
typedef struct USBSSP_DescT_s USBSSP_DescT;
typedef struct USBSSP_DcbaaT_s USBSSP_DcbaaT;
typedef struct USBSSP_XhciResourcesT_s USBSSP_XhciResourcesT;
typedef struct USBSSP_IpIdVerRegs_s USBSSP_IpIdVerRegs;
typedef struct USBSSP_DriverResourcesT_s USBSSP_DriverResourcesT;
typedef struct USBSSP_DriverContextT_s USBSSP_DriverContextT;
typedef struct USBSSP_ForceHdrParams_s USBSSP_ForceHdrParams;
typedef struct USBSSP_XferBufferDesc_s USBSSP_XferBufferDesc;

typedef struct USBSSP_QuickAccessRegs_s USBSSP_QuickAccessRegs;
typedef struct USBSSP_SfrT_s USBSSP_SfrT;

/**********************************************************************
* Enumerations
**********************************************************************/
/** Endpoint 0 states */
typedef enum
{
    /** Endpoint 0 not connected */
    USBSSP_EP0_UNCONNECTED = 0U,
    /** Endpoint 0 setup phase */
    USBSSP_EP0_SETUP_PHASE = 1U,
    /** Endpoint 0 data phase */
    USBSSP_EP0_DATA_PHASE = 2U,
    /** Endpoint 0 status phase */
    USBSSP_EP0_STATUS_PHASE = 3U
} USBSSP_Ep0StateEnum;

/** Extra flags types */
typedef enum
{
    /** Extraflags undefined */
    USBSSP_EXTRAFLAGSENUMT_UNDEFINED = 0U,
    /** Extraflags no doorbell */
    USBSSP_EXTRAFLAGSENUMT_NODORBELL = 1U,
    /** Extraflags forcelinkTRB */
    USBSSP_EXTRAFLAGSENUMT_FORCELINKTRB = 2U
} USBSSP_ExtraFlagsEnumT;

/**********************************************************************
* Callbacks
**********************************************************************/
/** Completion callback */
typedef void (*USBSSP_Complete)(USBSSP_DriverResourcesT* arg, uint32_t status, const USBSSP_RingElementT* eventPtr);

/** No-Op completion callback */
typedef void (*USBSSP_NopComplete)(USBSSP_DriverResourcesT* arg, uint32_t status, const USBSSP_RingElementT* eventPtr);

/** Force Header complete callback function */
typedef void (*USBSSP_ForceHeaderComplete)(USBSSP_DriverResourcesT* arg, uint32_t status, const USBSSP_RingElementT* eventPtr);

/** Used for testing purposes */
typedef uint8_t (*USBSSP_GenericCallback)(USBSSP_RingElementT* eventPtr);

/** Used for testing purposes */
typedef void (*USBSSP_PostCallback)(USBSSP_RingElementT* eventPtr);

/** Used for testing purposes */
typedef void (*USBSSP_PreportChangeDetectCallback)(uint32_t portsc_value, uint32_t port_id);

/** Used for virtualization */
typedef void (*USBSSP_InputContextCallback)(USBSSP_DriverResourcesT* arg);

/** Pointer to function to perform soft reset of USB 2.0 PHY */
typedef void (*USBSSP_USB2PhySoftReset)(USBSSP_DriverResourcesT* res);

/** Callback to reset USB-2 interface  */
typedef void (*USBSSP_USB2ResetCallback)(USBSSP_DriverResourcesT* res);

/** Pointer to function that maps logical pointer to physical one */
typedef uintptr_t (*USBSSP_get_phys_from_log_ptr_proc_t)(void* log_ptr, int32_t byte_size);

/** Pointer to function that maps physical pointer to logical one */
typedef void* (*USBSSP_get_log_from_phys_ptr_proc_t)(uintptr_t phys_ptr, int32_t byte_size);

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
uint32_t USBSSP_TransferData(USBSSP_DriverResourcesT* res, uint8_t epIndex, const uintptr_t buff, uint32_t size, USBSSP_Complete complete);

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
uint32_t USBSSP_TransferVectorData(USBSSP_DriverResourcesT* res, uint8_t epIndex, const USBSSP_XferBufferDesc* bufferDesc, uint32_t bufferCount, USBSSP_Complete complete);

/**
 * Stop endpoint. Function sends STOP_ENDPOINT_COMMAND command to
 * USBSSP controller
 * @param[in] res Driver resources
 * @param[in] endpoint Index of endpoint to stop $RANGE $FROM USBSSP_EP0_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_StopEndpoint(USBSSP_DriverResourcesT* res, uint8_t endpoint);

/**
 * Endpoint reset. Function sends RESET_ENDPOINT_COMMAND to USBSSP
 * controller
 * @param[in] res Driver resources
 * @param[in] endpoint Index of endpoint to reset $RANGE $FROM USBSSP_EP0_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_ResetEndpoint(USBSSP_DriverResourcesT* res, uint8_t endpoint);

/**
 * Reset of connected device. Function sends RESET_DEVICE_COMMAND to
 * USBSSP controller in order to issue reset state on USB bus.
 * @param[in] res Driver resources
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK selected endpoint is within available endpoint range
 */
uint32_t USBSSP_ResetDevice(USBSSP_DriverResourcesT* res);

/**
 * Handling of USBSSP controller interrupt. Function is called from
 * USBSSP interrupt context.
 * @param[in] res Driver resources
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_Isr(USBSSP_DriverResourcesT* res);

/**
 * Function sets memory resources used by driver.
 * @param[in] res Driver resources
 * @param[in] memRes User defined memory resources.
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_SetMemRes(USBSSP_DriverResourcesT* res, USBSSP_XhciResourcesT* memRes);

/**
 * Initialization of USBSSP_DriverResourcesT object.
 * USBSSP_DriverResourcesT object keeps all resources required by
 * USBSSP controller. It represents USBSSP hardware controller.
 * @param[in] res Driver resources
 * @param[in] base Physical address of USBSSP controller where is mapped in system
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_Init(USBSSP_DriverResourcesT* res, uintptr_t base);

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
uint32_t USBSSP_GetDescriptor(USBSSP_DriverResourcesT* res, uint8_t descType, USBSSP_Complete complete);

/**
 * Set address. Function executes set address request on connected
 * device.
 * @param[in] res Driver resources
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_SetAddress(USBSSP_DriverResourcesT* res);

/**
 * Reset port.
 * @param[in] res Driver resources
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_ResetRootHubPort(const USBSSP_DriverResourcesT* res);

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
uint32_t USBSSP_IssueGenericCommand(USBSSP_DriverResourcesT* res, uint32_t dword0, uint32_t dword1, uint32_t dword2, uint32_t dword3);

/**
 * Set feature on device's endpoint. Functions sends setup requested
 * to device with set/cleared endpoint feature
 * @param[in] res Driver resources
 * @param[in] epIndex Index of endpoint to set/clear feature on $RANGE $FROM USBSSP_EP_CONT_OFFSET $TO USBSSP_EP_CONT_MAX - (uint8_t)1 $
 * @param[in] feature When 1 sets stall, when 0 clears stall
 * @return CDN_EOK on success
 * @return complete_code XHCI transfer complete status code
 */
uint32_t USBSSP_EndpointSetFeature(USBSSP_DriverResourcesT* res, uint8_t epIndex, uint8_t feature);

/**
 * Set configuration. Function configures USBSSP controller as well as
 * device connected to this USBSSP controller. This is a blocking
 * function. This function must not be called from an interrupt
 * context.
 * @param[in] res Driver resources
 * @param[in] configValue USB device's configuration selector
 * @return CDN_EOK on success
 * @return complete_code XHCI transfer complete status code
 */
uint32_t USBSSP_SetConfiguration(USBSSP_DriverResourcesT* res, uint32_t configValue);

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
uint32_t USBSSP_NBControlTransfer(USBSSP_DriverResourcesT* res, const CH9_UsbSetup* setup, const uint8_t* pdata, USBSSP_Complete complete);

/**
 * No Operation test. Function used for testing purposes:
 * NO_OP_COMMAND is send to USBSSP controller. When event ring
 * receives NO_OP_COMMAND complete it calls complete callback
 * @param[in] res Driver resources
 * @param[in] complete Callback
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_NoOpTest(USBSSP_DriverResourcesT* res, USBSSP_Complete complete);

/**
 * Calculate full/low speed endpoint interval based on bInterval See
 * xHCI spec Section 6.2.3.6 for more details.
 * @param[in] bInterval bInterval
 * @return value valid endpoint contxt interval value
 */
uint8_t USBSSP_CalcFsLsEPIntrptInterval(uint8_t bInterval);

/**
 * Function enables slot for new connected device.
 * @param[in] res Driver resources
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_EnableSlot(USBSSP_DriverResourcesT* res);

/**
 * Function disables slot of connected device.
 * @param[in] res Driver resources
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_DisableSlot(USBSSP_DriverResourcesT* res);

/**
 * Enable endpoint. Function used in device context.
 * @param[in] res Driver resources
 * @param[in] desc pointer to endpoint descriptor
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_EnableEndpoint(USBSSP_DriverResourcesT* res, const uint8_t* desc);

/**
 * Disable endpoint. Function used in device context.
 * @param[in] res Driver resources
 * @param[in] epAddress address of endpoint to be disabled
 * @return CDN_EINVAL when driver's settings doesn't suit to native platform settings
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_DisableEndpoint(USBSSP_DriverResourcesT* res, uint8_t epAddress);

/**
 * Get actual frame number. Function returns actual frame number on
 * USB bus. Remember that maximal frame number can be 0x7FF. Next
 * frame increments returned value from 0.
 * @param[in] res driver resources
 * @param[out] index Micro Frame Index returned by function.
 */
uint32_t USBSSP_GetMicroFrameIndex(USBSSP_DriverResourcesT* res, uint32_t* index);

/**
 * set end point extra flag
 * @param[in] res driver resources
 * @param[in] epIndex endpoint index
 * @param[in] flags Endpoint Extra Flag
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_SetEndpointExtraFlag(USBSSP_DriverResourcesT* res, uint8_t epIndex, USBSSP_ExtraFlagsEnumT flags);

/**
 * clean endpoint extra flag
 * @param[in] res driver resources
 * @param[in] epIndex end point index
 * @param[in] flags Endpoint Extra Flag
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_CleanEndpointExtraFlag(USBSSP_DriverResourcesT* res, uint8_t epIndex, USBSSP_ExtraFlagsEnumT flags);

/**
 * get endpoint extra flag
 * @param[in] res driver resources
 * @param[in] epIndex endpoint index
 * @param[out] flag Endpoint Extra Flag returned by pointer
 */
uint32_t USBSSP_GetEndpointExtraFlag(const USBSSP_DriverResourcesT* res, uint8_t epIndex, uint8_t* flag);

/**
 * set Frame ID
 * @param[in] res Driver resources
 * @param[in] epIndex endpoint index
 * @param[in] frameID Frame ID
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_SetFrameID(USBSSP_DriverResourcesT* res, uint8_t epIndex, uint32_t frameID);

/**
 * Add event data TRB
 * @param[in] res Driver resources
 * @param[in] epIndex endpoint index
 * @param[in] eventDataLo event data Low
 * @param[in] eventDataHi event data High
 * @param[in] flags Flags
 * @return value TBC
 */
uint32_t USBSSP_AddEventDataTRB(USBSSP_DriverResourcesT* res, uint8_t epIndex, uint32_t eventDataLo, uint32_t eventDataHi, uint32_t flags);

/**
 * Force header
 * @param[in] res driver resources
 * @param[in] trbDwords trb words
 * @param[in] complete complete callback
 * @return CDN_EINVAL Invalid Input parameters
 * @return CDN_EOK if no errors
 */
uint32_t USBSSP_ForceHeader(USBSSP_DriverResourcesT* res, const USBSSP_ForceHdrParams* trbDwords, USBSSP_Complete complete);

/**
 * set port override register
 * @param[in] res driver resources
 * @param[in] regValue Register value
 */
uint32_t USBSSP_SetPortOverrideReg(const USBSSP_DriverResourcesT* res, uint32_t regValue);

/**
 * Get port override register
 * @param[in] res Driver resources
 * @param[out] regValue Register value
 */
uint32_t USBSSP_GetPortOverrideReg(const USBSSP_DriverResourcesT* res, uint32_t* regValue);

/**
 * set port control register
 * @param[in] res driver resources
 * @param[in] portId port ID
 * @param[in] portRegIdx port control register ID
 * @param[in] regValue Register value
 */
uint32_t USBSSP_SetPortControlReg(const USBSSP_DriverResourcesT* res, uint8_t portId, USBSSP_PortControlRegIdx portRegIdx, uint32_t regValue);

/**
 * Get port control register
 * @param[in] res Driver resources
 * @param[in] portId port ID
 * @param[in] portRegIdx port control register ID
 * @param[out] regValue Register value
 */
uint32_t USBSSP_GetPortControlReg(const USBSSP_DriverResourcesT* res, uint8_t portId, USBSSP_PortControlRegIdx portRegIdx, uint32_t* regValue);

/**
 * Save state and stop xHC
 * @param[in] res Driver resources
 * @param[in] drvContext Pointer to driver context struct
 */
uint32_t USBSSP_SaveState(USBSSP_DriverResourcesT* res, USBSSP_DriverContextT* drvContext);

/**
 * Restore state and start xHC
 * @param[in] res Driver resources
 * @param[in] drvContext Pointer to driver context struct
 */
uint32_t USBSSP_RestoreState(USBSSP_DriverResourcesT* res, const USBSSP_DriverContextT* drvContext);

/**
 * Get connected status
 * @param[in] res Driver resources
 * @param[out] connected connected status
 */
uint32_t USBSSP_GetPortConnected(const USBSSP_DriverResourcesT* res, uint8_t* connected);

/**
 * Get dev address state
 * @param[in] res Driver resources
 * @param[out] addrState dev address state
 */
uint32_t USBSSP_GetDevAddressState(const USBSSP_DriverResourcesT* res, uint8_t* addrState);

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */


#ifdef __cplusplus
}
#endif

#endif  /* CDN_XHCI_IF_H */
