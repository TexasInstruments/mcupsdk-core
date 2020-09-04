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
* USB 3.0 specification USB 2.0 specification And others Specification
* defined by USB organization  Because of when this file was creating
* USB 3.1 specification was the latest published so it mostly based
* on this specification. USB 3.1 specification does not contains all
* descriptors and definitions included in older specification so
* cusb_ch9_if.h file also refers to other specification defined by
* USB organization.
**********************************************************************/

#ifndef CUSB_CH9_IF_H
#define CUSB_CH9_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cdn_stdtypes.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** Data transfer direction */
#define CH9_USB_DIR_HOST_TO_DEVICE 0U

/** Data transfer direction device to host */
#define CH9_USB_DIR_DEVICE_TO_HOST (1U << 7U)

/** Type of request */
#define CH9_USB_REQ_TYPE_MASK (3U << 5U)

/** USB request type standard */
#define CH9_USB_REQ_TYPE_STANDARD (0U << 5U)

/** USB request type class */
#define CH9_USB_REQ_TYPE_CLASS (1U << 5U)

/** USB request type vendor */
#define CH9_USB_REQ_TYPE_VENDOR (2U << 5U)

/** USB request type other */
#define CH9_USB_REQ_TYPE_OTHER (3U << 5U)

/** Recipient of request */
#define CH9_REQ_RECIPIENT_MASK 0x0fU

/** Request recipient device */
#define CH9_USB_REQ_RECIPIENT_DEVICE 0U

/** Request recipient interface */
#define CH9_USB_REQ_RECIPIENT_INTERFACE 1U

/** Request recipient endpoint */
#define CH9_USB_REQ_RECIPIENT_ENDPOINT 2U

/** Request recipient other */
#define CH9_USB_REQ_RECIPIENT_OTHER 3U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_GET_STATUS 0U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_CLEAR_FEATURE 1U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_SET_FEATURE 3U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_SET_ADDRESS 5U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_GET_DESCRIPTOR 6U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_SET_DESCRIPTOR 7U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_GET_CONFIGURATION 8U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_SET_CONFIGURATION 9U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_GET_INTERFACE 10U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_SET_INTERFACE 11U

/** Standard  Request Code (chapter 9.4 - Table 9-4 of USB Spec) */
#define CH9_USB_REQ_SYNCH_FRAME 12U

/** Request set encryption */
#define CH9_USB_REQ_SET_ENCRYPTION 13U

/** Request get encryption */
#define CH9_USB_REQ_GET_ENCRYPTION 14U

/** Request set handshake */
#define CH9_USB_REQ_SET_HANDSHAKE 15U

/** Request get handshake */
#define CH9_USB_REQ_GET_HANDSHAKE 16U

/** Request set connecion */
#define CH9_USB_REQ_SET_CONNECTION 17U

/** Request set security data */
#define CH9_USB_REQ_SET_SCURITY_DATA 18U

/** Request get security data */
#define CH9_USB_REQ_GET_SCURITY_DATA 19U

/** Request set wusb data */
#define CH9_USB_REQ_SET_WUSB_DATA 20U

/** Request loopback data write */
#define CH9_USB_REQ_LOOPBACK_DATA_WRITE 21U

/** Request loopback data read */
#define CH9_USB_REQ_LOOPBACK_DATA_READ 22U

/** Request set interface ds */
#define CH9_USB_REQ_SET_INTERFACE_DS 23U

/** Request set sel */
#define CH9_USB_REQ_SET_SEL 48U

/** Request isoch delay */
#define CH9_USB_REQ_ISOCH_DELAY 49U

/** Standard Descriptor Types (chapter 9.4 - Table 9-5 of USB Spec) */
#define CH9_USB_DT_DEVICE 1U

/** Standard Descriptor Types (chapter 9.4 - Table 9-5 of USB Spec) */
#define CH9_USB_DT_CONFIGURATION 2U

/** Standard Descriptor Types (chapter 9.4 - Table 9-5 of USB Spec) */
#define CH9_USB_DT_STRING 3U

/** Standard Descriptor Types (chapter 9.4 - Table 9-5 of USB Spec) */
#define CH9_USB_DT_INTERFACE 4U

/** Standard Descriptor Types (chapter 9.4 - Table 9-5 of USB Spec) */
#define CH9_USB_DT_ENDPOINT 5U

/** Standard Descriptor Types (chapter 9.4 - Table 9-5 of USB Spec) */
#define CH9_USB_DT_DEVICE_QUALIFIER 6U

/** USB 2 Hub */
#define CH9_USB_DT_USB2_HUB 41U

/** USB 3 Hub */
#define CH9_USB_DT_USB3_HUB 42U

/** USB 2 */
#define CH9_USB_DT_OTHER_SPEED_CFG 7U

/** USB 2 */
#define CH9_USB_DT_INTERFACE_POWER 8U

/** OTG */
#define CH9_USB_DT_OTG 9U

/** debug */
#define CH9_USB_DT_DEBUG 10U

/** Interface association */
#define CH9_USB_DT_INTERF_ASSOCIATION 11U

/** BOS descriptor */
#define CH9_USB_DT_BOS 15U

/** device capability descriptor */
#define CH9_USB_DT_DEVICE_CAPABILITY 16U

/** super speed descriptor */
#define CH9_USB_DT_SS_USB_EP_COMPANION 48U

/** SSP Isochronous Endpoint Companion Descriptor */
#define CH9_USB_DT_SSP_ISO_EP_COMPANION 49U

/** Descriptor size */
#define CH9_USB_DS_DEVICE 18U

/** BOS descriptor size */
#define CH9_USB_DS_BOS 5U

/** Capability type USB 2.0 EXTENSION */
#define CH9_USB_DS_DEVICE_CAPABILITY_20 7U

/** Capability type SUPERSPEED_USB */
#define CH9_USB_DS_DEVICE_CAPABILITY_30 10U

/** Capability type CONTAINER_ID */
#define CH9_USB_DS_DEV_CAP_CONTAINER_ID 21U

/** Capability type PRECISION_TIME_MEASUREMENT */
#define CH9_USB_DS_DEV_CAP_PR_TIME_MEAS 4U

/** Capability type SUPERSPEED_PLUS. Number of SSID 1 */
#define CH9_USB_DS_DEV_CAP_SSP 20U

/** Configuration descriptor size */
#define CH9_USB_DS_CONFIGURATION 9U

/** USB2 hub DS */
#define CH9_USB_DS_USB2_HUB 7U

/** USB3 hub DS */
#define CH9_USB_DS_USB3_HUB 12U

/** Interface association DS */
#define CH9_USB_DS_INTERF_ASSOCIATION 8U

/** super speed descriptor size */
#define CH9_USB_DS_SS_USB_EP_COMPANION 6U

/** SSP Isochronous Endpoint Companion Descriptor size */
#define CH9_USB_DS_SSP_ISO_EP_COMPANION 8U

/** interface DS */
#define CH9_USB_DS_INTERFACE 9U

/** endpoint DS */
#define CH9_USB_DS_ENDPOINT 7U

/** string DS */
#define CH9_USB_DS_STRING 3U

/** OTG DS */
#define CH9_USB_DS_OTG 5U

/** USB2 */
#define CH9_USB_DS_DEVICE_QUALIFIER 10U

/** USB2 */
#define CH9_USB_DS_OTHER_SPEED_CFG 7U

/** interface power DS */
#define CH9_USB_DS_INTERFACE_POWER 8U

/** Standard Feature Selectors (chapter 9.4 - Table 9-6 of USB Spec) */
#define CH9_USB_FS_ENDPOINT_HALT 0U

/** Function suspend FS */
#define CH9_USB_FS_FUNCTION_SUSPEND 0U

/** Standard Feature Selectors (chapter 9.4 - Table 9-6 of USB Spec) */
#define CH9_USB_FS_DEVICE_REMOTE_WAKEUP 1U

/** Standard Feature Selectors (chapter 9.4 - Table 9-6 of USB Spec) */
#define CH9_USB_FS_TEST_MODE 2U

/** B HNP Enable FS */
#define CH9_USB_FS_B_HNP_ENABLE 3U

/** A HNP Support FS */
#define CH9_USB_FS_A_HNP_SUPPORT 4U

/** A Alt HNP Support FS */
#define CH9_USB_FS_A_ALT_HNP_SUPPORT 5U

/** WUSB device FS */
#define CH9_USB_FS_WUSB_DEVICE 6U

/** U1 Enable FS */
#define CH9_USB_FS_U1_ENABLE 48U

/** U2 enable FS */
#define CH9_USB_FS_U2_ENABLE 49U

/** LTM enable FS */
#define CH9_USB_FS_LTM_ENABLE 50U

/** B3 NTF HOST REL FS */
#define CH9_USB_FS_B3_NTF_HOST_REL 51U

/** B3 RESP Enable FS */
#define CH9_USB_FS_B3_RESP_ENABLE 52U

/** LDM enable FS */
#define CH9_USB_FS_LDM_ENABLE 53U

/** Recipient Device (Figure 9-4 of USB Spec) */
#define CH9_USB_STS_DEV_SELF_POWERED (1U << 0U)

/** Recipient Device (Figure 9-4 of USB Spec) */
#define CH9_USB_STS_DEV_REMOTE_WAKEUP (1U << 1U)

/** U1 Enable status */
#define CH9_USB_STS_DEV_U1_ENABLE (1U << 2U)

/** U2 enable status */
#define CH9_USB_STS_DEV_U2_ENABLE (1U << 3U)

/** LTM enable status */
#define CH9_USB_STS_DEV_LTM_ENABLE (1U << 4U)

/** Recipient Endpoint (Figure 9-6 of USB Spec) */
#define CH9_USB_STS_EP_HALT (1U << 0U)

/** Macros describing information for SetFeauture Request and FUCTION_SUSPEND selector (chapter 9.4.9 - Table 9-9 of USB Spec) */
#define CH9_USB_SF_LOW_PWR_SUSP_STATE 0x1U

/** Remote wake enabled in set feature */
#define CH9_USB_SF_REMOTE_WAKE_ENABLED 0x2U

/** USB class interface */
#define CH9_USB_CLASS_INTERFACE 0x0U

/** USB class audio */
#define CH9_USB_CLASS_AUDIO 0x01U

/** USB class CDC */
#define CH9_USB_CLASS_CDC 0x02U

/** USB class communication */
#define CH9_USB_CLASS_COMMUNICATION 0x01U

/** USB class HID */
#define CH9_USB_CLASS_HID 0x03U

/** USB class physical */
#define CH9_USB_CLASS_PHYSICAL 0x05U

/** USB class image */
#define CH9_USB_CLASS_IMAGE 0x06U

/** USB class printer */
#define CH9_USB_CLASS_PRINTER 0x07U

/** USB class mass storage */
#define CH9_USB_CLASS_MASS_STORAGE 0x08U

/** USB class hub */
#define CH9_USB_CLASS_HUB 0x09U

/** USB class cdc data */
#define CH9_USB_CLASS_CDC_DATA 0x0AU

/** USB class smart card */
#define CH9_USB_CLASS_SMART_CARD 0x0BU

/** USB class content security */
#define CH9_USB_CLASS_CONTENT_SEECURITY 0x0DU

/** USB class video */
#define CH9_USB_CLASS_VIDEO 0x0EU

/** USB class healthcare */
#define CH9_USB_CLASS_HEALTHCARE 0x0FU

/** USB class audio video */
#define CH9_USB_CLASS_AUDIO_VIDEO 0x10U

/** USB class diagnostic */
#define CH9_USB_CLASS_DIAGNOSTIC 0xDCU

/** USB class wireless */
#define CH9_USB_CLASS_WIRELESS 0xE0U

/** USB class miscellaneous */
#define CH9_USB_CLASS_MISCELLANEOUS 0xEFU

/** USB class application */
#define CH9_USB_CLASS_APPLICATION 0xFEU

/** USB class vendor */
#define CH9_USB_CLASS_VENDOR 0xFFU

/** Device Capability Types Codes (see Table 9-14 of USB Spec 3.1) */
#define CH9_USB_DCT_WIRELESS_USB 0x01U

/** Device Capability Types Codes */
#define CH9_USB_DCT_USB20_EXTENSION 0x02U

/** Device Capability Types Codes */
#define CH9_USB_DCT_SS_USB 0x03U

/** Device Capability Types Codes */
#define CH9_USB_DCT_CONTAINER_ID 0x04U

/** Device Capability Types Codes */
#define CH9_USB_DCT_PLATFORM 0x05U

/** Device Capability Types Codes */
#define CH9_USB_DCT_POWER_DELIVERY_CAP 0x06U

/** Device Capability Types Codes */
#define CH9_USB_DCT_BATTERY_INFO_CAP 0x07U

/** Device Capability Types Codes */
#define CH9_USB_DCT_PD_CONS_PORT_CAP 0x08U

/** Device Capability Types Codes */
#define CH9_USB_DCT_PD_PROV_PORT_CAPAB 0x09U

/** Device Capability Types Codes */
#define CH9_USB_DCT_SS_PLUS 0x0AU

/** Device Capability Types Codes */
#define CH9_USB_DCT_PR_TIME_MEAS 0x0BU

/** Device Capability Types Codes */
#define CH9_USB_DCT_WIRELESS_USB_EXT 0x0CU

/** Describe supports LPM defined in bmAttribues field of CUSBD_Usb20ExtensionDescriptor */
#define CH9_USB_USB20_EXT_LPM_SUPPORT (1U << 1U)

/** Describe BESL ALT HIRD defined in bmAttribues field of CUSBD_Usb20ExtensionDescriptor */
#define CH9_USB_USB20_EXT_BESL_ALT_HIRD (1U << 2U)

/** Describe supports LTM defined in bmAttribues field of CUSBD_UsbSuperSpeedDeviceCapabilityDescriptor */
#define CH9_USB_SS_CAP_LTM (1U << 1U)

/** Describe speed supported defined in wSpeedSupported field of CUSBD_UsbSuperSpeedDeviceCapabilityDescriptor */
#define CH9_USB_SS_CAP_SUPPORT_LS (1U << 0U)

/** Describe speed supported defined in wSpeedSupported field of CUSBD_UsbSuperSpeedDeviceCapabilityDescriptor */
#define CH9_USB_SS_CAP_SUPPORT_FS (1U << 1U)

/** Describe speed supported defined in wSpeedSupported field of CUSBD_UsbSuperSpeedDeviceCapabilityDescriptor */
#define CH9_USB_SS_CAP_SUPPORT_HS (1U << 2U)

/** Describe speed supported defined in wSpeedSupported field of CUSBD_UsbSuperSpeedDeviceCapabilityDescriptor */
#define CH9_USB_SS_CAP_SUPPORT_SS (1U << 3U)

/** Describe encoding of bmSublinkSpeedAttr0 filed from CUSBD_UsbSuperSpeedPlusDescriptor */
#define CH9_USB_SSP_SID_OFFSET 0U

/** SID Mask */
#define CH9_USB_SSP_SID_MASK 0x0000000fU

/** LSE offset */
#define CH9_USB_SSP_LSE_OFFSET 4U

/** LSE Mask */
#define CH9_USB_SSP_LSE_MASK (0x00000003U << CH9_USB_SSP_LSE_OFFSET)

/** ST offset */
#define CH9_USB_SSP_ST_OFFSET 6U

/** ST mask */
#define CH9_USB_SSP_ST_MASK (0x00000003U << CH9_USB_SSP_ST_OFFSET)

/** LP offset */
#define CH9_USB_SSP_LP_OFFSET 14U

/** LP mask */
#define CH9_USB_SSP_LP_MASK (0x00000003U << CH9_USB_SSP_LP_OFFSET)

/** LSM offset */
#define CH9_USB_SSP_LSM_OFFSET 16U

/** LSM mask */
#define CH9_USB_SSP_LSM_MASK (0x0000FFFFU << CH9_USB_SSP_LSM_OFFSET)

/** Description of bmAttributes field from  Configuration Description */
#define CH9_USB_CONFIG_RESERVED (1U << 7U)

/** Self Powered */
#define CH9_USB_CONFIG_SELF_POWERED (1U << 6U)

/** Remote Wakeup */
#define CH9_USB_CONFIG_REMOTE_WAKEUP (1U << 5U)

/** Definitions for bEndpointAddress field from  Endpoint descriptor */
#define CH9_USB_EP_DIR_MASK 0x80U

/** endpoint direction */
#define CH9_USB_EP_DIR_IN 0x80U

/** endpoint number mask */
#define CH9_USB_EP_NUMBER_MASK 0x0fU

/** Endpoint attributes from Endpoint descriptor - bmAttributes field */
#define CH9_USB_EP_TRANSFER_MASK 0x03U

/** control endpoint */
#define CH9_USB_EP_CONTROL 0x0U

/** isochronous endpoint */
#define CH9_USB_EP_ISOCHRONOUS 0x01U

/** bulk endpoint */
#define CH9_USB_EP_BULK 0x02U

/** interrupt endpoint */
#define CH9_USB_EP_INTERRUPT 0x03U

/** Synchronization types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_SYNC_MASK 0xCU

/** Synchronization types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_SYNC_NO (0x0U >> 2U)

/** Synchronization types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_SYNC_ASYNCHRONOUS (0x1U >> 2U)

/** Synchronization types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_SYNC_ADAPTIVE (0x02U >> 2U)

/** Synchronization types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_SYNC_SYNCHRONOUS (0x03U >> 2U)

/** Synchronization types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_USAGE_MASK (0x3U >> 4U)

/** Usage types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_USAGE_DATA (0U >> 4U)

/** Usage types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_USAGE_FDBCK (0x01U >> 4U)

/** Usage types for ISOCHRONOUS endpoints */
#define CH9_USB_EP_USAGE_IMPLICIT_FDBCK (0x02U >> 4U)

/** Usage types for INTERRUPTS endpoints */
#define CH9_USB_EP_USAGE_PERIODIC (0U >> 4U)

/** Usage types for INTERRUPTS endpoints */
#define CH9_USB_EP_USAGE_NOTIFICATION (0x01U >> 4U)

/** Description of fields bmAttributes from OTG descriptor */
#define CH9_USB_OTG_ADP_MASK 0x4U

/** Description of fields bmAttributes from OTG descriptor */
#define CH9_USB_OTG_HNP_MASK 0x2U

/** Description of fields bmAttributes from OTG descriptor */
#define CH9_USB_OTG_SRP_MASK 0x1U

/** Test Mode Selectors See USB 2.0 spec Table 9-7 */
#define CH9_TEST_J 1U

/** Test Mode Selectors See USB 2.0 spec Table 9-7 */
#define CH9_TEST_K 2U

/** Test Mode Selectors See USB 2.0 spec Table 9-7 */
#define CH9_TEST_SE0_NAK 3U

/** Test Mode Selectors See USB 2.0 spec Table 9-7 */
#define CH9_TEST_PACKET 4U

/** Test Mode Selectors See USB 2.0 spec Table 9-7 */
#define CH9_TEST_FORCE_EN 5U

/** max packet size mask */
#define CH9_MAX_PACKET_SIZE_MASK 0x7ffU

/** packet per frame shift */
#define CH9_PACKET_PER_FRAME_SHIFT 11U

/** OTG status selector See USB_OTG_AND_EH_2-0 spec Table 6-4 */
#define CH9_OTG_STATUS_SELECTOR 0xF000U

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
typedef struct CH9_UsbSetup_s CH9_UsbSetup;
typedef struct CH9_UsbDeviceDescriptor_s CH9_UsbDeviceDescriptor;
typedef struct CH9_UsbBosDescriptor_s CH9_UsbBosDescriptor;
typedef struct CH9_UsbCapabilityDescriptor_s CH9_UsbCapabilityDescriptor;
typedef struct CH9_Usb20ExtensionDescriptor_s CH9_Usb20ExtensionDescriptor;
typedef struct CH9_UsbSSDeviceCapabilityDescriptor_s CH9_UsbSSDeviceCapabilityDescriptor;
typedef struct CH9_UsbContainerIdDescriptor_s CH9_UsbContainerIdDescriptor;
typedef struct CH9_UsbPlatformDescriptor_s CH9_UsbPlatformDescriptor;
typedef struct CH9_UsbSSPlusDescriptor_s CH9_UsbSSPlusDescriptor;
typedef struct CH9_UsbPTMCapabilityDescriptor_s CH9_UsbPTMCapabilityDescriptor;
typedef struct CH9_UsbConfigurationDescriptor_s CH9_UsbConfigurationDescriptor;
typedef struct CH9_UsbInterfaceAssociationDescriptor_s CH9_UsbInterfaceAssociationDescriptor;
typedef struct CH9_UsbInterfaceDescriptor_s CH9_UsbInterfaceDescriptor;
typedef struct CH9_UsbEndpointDescriptor_s CH9_UsbEndpointDescriptor;
typedef struct CH9_UsbSSEndpointCompanionDescriptor_s CH9_UsbSSEndpointCompanionDescriptor;
typedef struct CH9_UsbSSPlusIsocEndpointCompanionDescriptor_s CH9_UsbSSPlusIsocEndpointCompanionDescriptor;
typedef struct CH9_UsbStringDescriptor_s CH9_UsbStringDescriptor;
typedef struct CH9_UsbDeviceQualifierDescriptor_s CH9_UsbDeviceQualifierDescriptor;
typedef struct CH9_UsbOtherSpeedConfigurationDescriptor_s CH9_UsbOtherSpeedConfigurationDescriptor;
typedef struct CH9_UsbHeaderDescriptor_s CH9_UsbHeaderDescriptor;
typedef struct CH9_UsbOtgDescriptor_s CH9_UsbOtgDescriptor;
typedef struct CH9_ConfigParams_s CH9_ConfigParams;

/**********************************************************************
* Enumerations
**********************************************************************/
/** USB States defined in USB Specification */
typedef enum {
    /** Device not attached yet */
    CH9_USB_STATE_NONE = 0U,
    /** see Figure 9-1 of USB Spec */
    CH9_USB_STATE_ATTACHED = 1U,
    /** Device state powered */
    CH9_USB_STATE_POWERED = 2U,
    /** Device state default */
    CH9_USB_STATE_DEFAULT = 3U,
    /** Device state address */
    CH9_USB_STATE_ADDRESS = 4U,
    /** Device state configured */
    CH9_USB_STATE_CONFIGURED = 5U,
    /** Device state suspended */
    CH9_USB_STATE_SUSPENDED = 6U,
    /** Device state error */
    CH9_USB_STATE_ERROR = 7U
} CH9_UsbState;

/** Speeds defined in USB Specification */
typedef enum {
    /** unknow speed - before enumeration */
    CH9_USB_SPEED_UNKNOWN = 0U,
    /** (1,5Mb/s) */
    CH9_USB_SPEED_LOW = 1U,
    /** usb 1.1 (12Mb/s) */
    CH9_USB_SPEED_FULL = 2U,
    /** usb 2.0 (480Mb/s) */
    CH9_USB_SPEED_HIGH = 3U,
    /** usb 3.0 GEN 1  (5Gb/s) */
    CH9_USB_SPEED_SUPER = 4U,
    /** usb 3.1 GEN2 (10Gb/s) */
    CH9_USB_SPEED_SUPER_PLUS = 5U
} CH9_UsbSpeed;

/** Available states of Endpoint 0. */
typedef enum {
    /** Endpoint disconnected. */
    CH9_EP0_UNCONNECTED = 0U,
    /** Endpoint in setup phase. */
    CH9_EP0_SETUP_PHASE = 1U,
    /** Endpoint in data phase. */
    CH9_EP0_DATA_PHASE = 2U,
    /** Endpoint in status phase. */
    CH9_EP0_STATUS_PHASE = 3U
} CH9_Ep0StateEnum;

/** Max packet 0 size defined in USB Specification */
typedef enum {
    /** unknow speed - before enumeration */
    CH9_USB_EP0_MAX_UNKNOWN = 0U,
    /** (1,5Mb/s) */
    CH9_USB_EP0_MAX_LOW = 8U,
    /** usb 1.1 (12Mb/s) */
    CH9_USB_EP0_MAX_FULL = 64U,
    /** usb 2.0 (480Mb/s) */
    CH9_USB_EP0_MAX_HIGH = 64U,
    /** usb 2.5 wireless */
    CH9_USB_EP0_MAX_WIRELESS = 512U,
    /** usb 3.0 GEN 1  (5Gb/s) */
    CH9_USB_EP0_MAX_SUPER = 512U,
    /** usb 3.1 GEN2 (10Gb/s) */
    CH9_USB_EP0_MAX_SUPER_PLUS = 512U
} CH9_UsbEP0Max;

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSB_CH9_IF_H */
