/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure, DRV-4932" */
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
#ifndef CUSB_CH9_STRUCTS_IF_H
#define CUSB_CH9_STRUCTS_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include "cusb_ch9_if.h"

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Structures and unions
**********************************************************************/
/** Structure describes USB request (SETUP packet). See USB Specification (chapter 9.3) */
struct CH9_UsbSetup_s {
    /** Characteristics of request */
    uint8_t bmRequestType;
    /** Specific request */
    uint8_t bRequest;
    /** Field that varies according to request */
    uint16_t wValue;
    /** typically used to pass an index or offset. */
    uint16_t wIndex;
    /** Number of bytes to transfer if there is a data stage */
    uint16_t wLength;
} __attribute__((packed));

/** Standard Device Descriptor (see Table 9-11 of USB Spec 3.1) */
struct CH9_UsbDeviceDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Device descriptor type */
    uint8_t bDescriptorType;
    /** USB Specification Release Number */
    uint16_t bcdUSB;
    /** Class code (assigned by the USB-IF) */
    uint8_t bDeviceClass;
    /** Subclass code (assigned by the USB-IF */
    uint8_t bDeviceSubClass;
    /** Protocol code (assigned by the USB-IF */
    uint8_t bDeviceProtocol;
    /** Maximum packet size for endpoint zero */
    uint8_t bMaxPacketSize0;
    /** Vendor ID (assigned by the USB-IF */
    uint16_t idVendor;
    /** Product ID (assigned by manufacturer) */
    uint16_t idProduct;
    /** Device release number */
    uint16_t bcdDevice;
    /** Index of string descriptor describing manufacturer */
    uint8_t iManufacturer;
    /** Index of string descriptor describing product */
    uint8_t iProduct;
    /** Index of string descriptor for serial number */
    uint8_t iSerialNumber;
    /** Number of possible configurations */
    uint8_t bNumConfigurations;
} __attribute__((packed));

/** Binary Device Object Store descriptor (see Table 9-12 of USB Spec 3.1) */
struct CH9_UsbBosDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** Descriptor type: BOS */
    uint8_t bDescriptorType;
    /** Length of this descriptor and all of its sub descriptors */
    uint16_t wTotalLength;
    /** The number of separate device capability descriptors in the BOS */
    uint8_t bNumDeviceCaps;
} __attribute__((packed));

/** Device Capability Descriptor (see Table 9-12 of USB Spec 3.1) */
struct CH9_UsbCapabilityDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** Descriptor type: DEVICE CAPABILITY type */
    uint8_t bDescriptorType;
    /** Capability type: USB 2.0 EXTENSION (002h) */
    uint8_t bDevCapabilityType;
    /** Capability specific format */
    uint32_t bmAttributes;
} __attribute__((packed));

/** USB 2.0 Extension Descriptor (see Table 9-15 of USB Spec 3.1) */
struct CH9_Usb20ExtensionDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** Descriptor type: DEVICE CAPABILITY type */
    uint8_t bDescriptorType;
    /** Capability type: USB 2.0 EXTENSION (002h) */
    uint8_t bDevCapabilityType;
    /** Capability specific format */
    uint32_t bmAttributes;
} __attribute__((packed));

/** SuperSpeed USB Device Capability Descriptor (see Table 9-16 of USB Spec 3.1) */
struct CH9_UsbSSDeviceCapabilityDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** DEVICE CAPABILITY Descriptor type */
    uint8_t bDescriptorType;
    /** Capability type: SUPERSPEED_USB */
    uint8_t bDevCapabilityType;
    /** Bitmap encoding of supported device level features */
    uint8_t bmAttributes;
    /** Bitmap encoding of the speed supported by device */
    uint16_t wSpeedSupported;
    /**
     * The lowest speed at which all the functionality
     * supported by the device is available to the user
     */
    uint8_t vFunctionalitySupport;
    /** U1 Device Exit Latency */
    uint8_t bU1DevExitLat;
    /** U2 Device Exit Latency */
    uint16_t bU2DevExitLat;
} __attribute__((packed));

/** Container ID Descriptor (see Table 9-17 of USB Spec 3.1) */
struct CH9_UsbContainerIdDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** DEVICE CAPABILITY Descriptor type */
    uint8_t bDescriptorType;
    /** Capability type: CONTAINER_ID */
    uint8_t bDevCapabilityType;
    /** Field reserved and shall be set to zero */
    uint8_t bReserved;
    /** unique number to device instance */
    uint8_t ContainerId[16];
} __attribute__((packed));

/** Platform descriptor */
struct CH9_UsbPlatformDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** DEVICE CAPABILITY Descriptor type */
    uint8_t bDescriptorType;
    /** Capability type: PLATFORM */
    uint8_t bDevCapabilityType;
    /** Field reserved and shall be set to zero */
    uint8_t bReserved;
    /** unique number to identifies a platform */
    uint8_t PlatformCapabilityUUID[16];
} __attribute__((packed));

/** SuperSpeedPlus USB Device Capability  (see Table 9-19 of USB Spec 3.1) */
struct CH9_UsbSSPlusDescriptor_s {
    /** Size of this descriptor */
    uint8_t bLength;
    /** DEVICE CAPABILITY Descriptor type */
    uint8_t bDescriptorType;
    /** Capability type: SUPERSPEED_PLUS */
    uint8_t bDevCapabilityType;
    /** Field reserved and shall be set to zero */
    uint8_t bReserved;
    /** Bitmap encoding of supported SuperSpeedPlus features */
    uint32_t bmAttributes;
    /** supported functionality */
    uint16_t wFunctionalitySupport;
    /** Reserved. Shall be set to zero */
    uint16_t wReserved;
    /** Sublink Speed Attribute */
    uint32_t bmSublinkSpeedAttr0;
    /** Additional Lane Speed Attributes */
    uint32_t bmSublinkSpeedAttrSSAC;
} __attribute__((packed));

/** SuperSpeedPlus USB Device Capability  (see Table 9-19 of USB Spec 3.1) */
struct CH9_UsbPTMCapabilityDescriptor_s {
    /** Length */
    uint8_t bLength;
    /** Descriptor type */
    uint8_t bDescriptorType;
    /** Device capability type */
    uint8_t bDevCapabilityType;
} __attribute__((packed));

/** Standard Configuration Descriptor (see Table 9-21 of USB Spec 3.1) */
struct CH9_UsbConfigurationDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Configuration descriptor type */
    uint8_t bDescriptorType;
    /** Total length of configuration */
    uint16_t wTotalLength;
    /** Number of interfaces supported by configuration */
    uint8_t bNumInterfaces;
    /** Value use as an argument to SetConfiguration() request */
    uint8_t bConfigurationValue;
    /** Index of string descriptor describing this configuration */
    uint8_t iConfiguration;
    /** Configuration attributes */
    uint8_t bmAttributes;
    /** Maximum power consumption of the USB device */
    uint8_t bMaxPower;
} __attribute__((packed));

/** Standard Interface Association Descriptor  (see Table 9-22 of USB Spec 3.1) */
struct CH9_UsbInterfaceAssociationDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Interface Association Descriptor Type */
    uint8_t bDescriptorType;
    /** interface number of this interface that is associated with this function */
    uint8_t bFirstInterface;
    /** Number of contiguous interfaces that are associated with this function */
    uint8_t bInterfaceCount;
    /** Class code assigned by USB-IF */
    uint8_t bFunctionClass;
    /** Subclass code */
    uint8_t bFunctionSubClass;
    /** Protocol code */
    uint8_t bFunctionProtocol;
    /** Index of string descriptor describing this function */
    uint8_t iFunction;
} __attribute__((packed));

/** Standard Interface Descriptor (see Table 9-23 of USB Spec 3.1) */
struct CH9_UsbInterfaceDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Interface Descriptor Type */
    uint8_t bDescriptorType;
    /** Number of this interface */
    uint8_t bInterfaceNumber;
    /** Value used to select this alternate setting */
    uint8_t bAlternateSetting;
    /** Class code */
    uint8_t bNumEndpoints;
    /** Subclass code */
    uint8_t bInterfaceClass;
    /** Subclass code */
    uint8_t bInterfaceSubClass;
    /** Protocol code */
    uint8_t bInterfaceProtocol;
    /** Index of string */
    uint8_t iInterface;
} __attribute__((packed));

/** Standard Endpoint Descriptor */
struct CH9_UsbEndpointDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Endpoint Descriptor Type */
    uint8_t bDescriptorType;
    /** The address of the endpoint */
    uint8_t bEndpointAddress;
    /** Endpoint attribute */
    uint8_t bmAttributes;
    /** Maximum packet size for this endpoint */
    uint16_t wMaxPacketSize;
    /** interval for polling endpoint data transfer */
    uint8_t bInterval;
} __attribute__((packed));

/** Standard SuperSpeed Endpoint Companion Descriptor (see Table 9-26 of USB Spec 3.1) */
struct CH9_UsbSSEndpointCompanionDescriptor_s {
    /** Size of descriptor in bytes */
    uint8_t bLength;
    /** SUPERSPEED_USB_ENDPOINT_COMPANION Descriptor types */
    uint8_t bDescriptorType;
    /** Number of packets that endpoint can transmit as part of burst */
    uint8_t bMaxBurst;
    /** Attributes */
    uint8_t bmAttributes;
    /** The total number of bytes  for every service interval */
    uint16_t wBytesPerInterval;
} __attribute__((packed));

/**
 * Standard SuperSpeedPlus Isochronous Endpoint
 * Companion Descriptor (see Table 9-27 of USB Spec 3.1)
 */
struct CH9_UsbSSPlusIsocEndpointCompanionDescriptor_s {
    /** Size of descriptor in bytes */
    uint8_t bLength;
    /** SUPERSPEEDPLUS_ISOCHRONOUS_ENDPOINT_COMPANION Descriptor types */
    uint8_t bDescriptorType;
    /** Reserved. Shall be set to zero */
    uint16_t wReserved;
    /** The total number of bytes  for every service interval */
    uint32_t dwBytesPerInterval;
} __attribute__((packed));

/** Standard String Descriptor */
struct CH9_UsbStringDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** STRING Descriptor Type */
    uint8_t bDescriptorType;
    /** UNICODE encoded string */
    uint8_t* bString;
} __attribute__((packed));

/** Standard Device Qualifier Descriptor (see Table 9-9 of USB Spec 2.0) */
struct CH9_UsbDeviceQualifierDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Device Qualifier type */
    uint8_t bDescriptorType;
    /** USB Specification version number */
    uint16_t bcdUSB;
    /** Class code */
    uint8_t bDeviceClass;
    /** Subclass code */
    uint8_t bDeviceSubClass;
    /** Protocol code */
    uint8_t bDeviceProtocol;
    /** Maximum packet size for other speed */
    uint8_t bMaxPacketSize0;
    /** Number of other speed configuration */
    uint8_t bNumConfigurations;
    /** Reserved for future use */
    uint8_t bReserved;
} __attribute__((packed));

/** Standard Other_Speed_Configuration descriptor (see Table 9-11 of USB Spec 2.0) */
struct CH9_UsbOtherSpeedConfigurationDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** Configuration descriptor type */
    uint8_t bDescriptorType;
    /** Total length of configuration */
    uint16_t wTotalLength;
    /** Number of interfaces supported by this speed configuration */
    uint8_t bNumInterfaces;
    /** Value to use to select configuration */
    uint8_t bConfigurationValue;
    /** Index of string descriptor describing this configuration */
    uint8_t iConfiguration;
    /** Configuration attributes */
    uint8_t bmAttributes;
    /** Maximum power consumption of the USB device */
    uint8_t bMaxPower;
} __attribute__((packed));

/**
 * Header descriptor. All descriptor have the same header that
 * consist of bLength and bDescriptorType fields
 */
struct CH9_UsbHeaderDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** descriptor Type */
    uint8_t bDescriptorType;
} __attribute__((packed));

/** OTG descriptor (see OTG spec. Table 6.1) */
struct CH9_UsbOtgDescriptor_s {
    /** Size of descriptor */
    uint8_t bLength;
    /** OTG Descriptor Type */
    uint8_t bDescriptorType;
    /** Attribute field */
    uint8_t bmAttributes;
    /** OTG and EH supplement release number */
    uint16_t bcdOTG;
} __attribute__((packed));

/** Config Params structure */
struct CH9_ConfigParams_s {
    /** U1 Device exit Latency */
    uint8_t bU1devExitLat;
    /** U2 Device exit Latency */
    uint16_t bU2DevExitLat;
} __attribute__((packed));

/**
 *  @}
 */


#ifdef __cplusplus
}
#endif

#endif  /* CUSB_CH9_STRUCTS_IF_H */
