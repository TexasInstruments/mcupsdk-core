/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
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

#include <stddef.h>

/* Define device framework.  */

unsigned char cdc_acm_device_framework_full_speed[] = {
    /* Device descriptor
       0x02 bDeviceClass:    CDC class code
       0x00 bDeviceSubclass: CDC class sub code
       0x00 bDeviceProtocol: CDC Device protocol

       idVendor & idProduct - http://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x00, 0x02,
    0xEF, 0x02, 0x01,
    0x40,
    0x04, 0x51, 0x75, 0x01,
    0x00, 0x01,
    0x01, 0x02, 03,
    0x01,

    /* Device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02,
    0x02, 0x00, 0x00,
    0x40,
    0x01,
    0x00,

    /* Configuration 1 descriptor */
    0x09, 0x02, 0x4b, 0x00,
    0x02, 0x01, 0x00,
    0x40, 0x00,

    /* Interface association descriptor. */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement */
    0x09, 0x04, 0x00,
    0x00,
    0x01,
    0x02, 0x02, 0x01,
    0x00,

    /* Header Functional Descriptor */
    0x05, 0x24, 0x00,
    0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02,
    0x0f,

    /* Union Functional Descriptor */
    0x05, 0x24, 0x06,
    0x00,
    0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01,
    0x00,
    0x01,

    /* Endpoint 1 descriptor */
    0x07, 0x05, 0x81,
    0x03,
    0x08, 0x00,
    0x10,

    /* Data Class Interface Descriptor Requirement */
    0x09, 0x04, 0x01,
    0x00,
    0x02,
    0x0A, 0x00, 0x00,
    0x00,

    /* First alternate setting Endpoint 1 descriptor */
    0x07, 0x05, 0x02,
    0x02,
    0x40, 0x00,
    0x00,

    /* Endpoint 2 descriptor */
    0x07, 0x05, 0x83,
    0x02,
    0x40, 0x00,
    0x00,

};

unsigned char cdc_acm_device_framework_high_speed[] = {
    /* Device descriptor
       0x02 bDeviceClass:    CDC class code
       0x00 bDeviceSubclass: CDC class sub code
       0x00 bDeviceProtocol: CDC Device protocol

       idVendor & idProduct - http://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x00, 0x02,
    0xEF, 0x02, 0x01,
    0x40,
    0x04, 0x51, 0x75, 0x01,
    0x00, 0x01,
    0x01, 0x02, 03,
    0x01,

    /* Device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02,
    0x02, 0x00, 0x00,
    0x40,
    0x01,
    0x00,

    /* Configuration 1 descriptor */
    0x09, 0x02, 0x4b, 0x00,
    0x02, 0x01, 0x00,
    0x40, 0x00,

    /* Interface association descriptor. */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement */
    0x09, 0x04, 0x00,
    0x00,
    0x01,
    0x02, 0x02, 0x01,
    0x00,

    /* Header Functional Descriptor */
    0x05, 0x24, 0x00,
    0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02,
    0x0f,

    /* Union Functional Descriptor */
    0x05, 0x24, 0x06,
    0x00,
    0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01,
    0x00,
    0x01,

    /* Endpoint 1 descriptor */
    0x07, 0x05, 0x81,
    0x03,
    0x08, 0x00,
    0x04,

    /* Data Class Interface Descriptor Requirement */
    0x09, 0x04, 0x01,
    0x00,
    0x02,
    0x0A, 0x00, 0x00,
    0x00,

    /* First alternate setting Endpoint 1 descriptor */
    0x07, 0x05, 0x02,
    0x02,
    0x00, 0x02,
    0x00,

    /* Endpoint 2 descriptor */
    0x07, 0x05, 0x83,
    0x02,
    0x00, 0x02,
    0x00,

};


unsigned char cdc_acm_string_framework[] = {

    /* Manufacturer string descriptor : Index 1 - "Texas Instruments, Inc." */
        0x09, 0x04, /* Language */
        0x01, /* Index */
        0x17, /* Length */
        0x54, 0x65, 0x78, 0x61, 0x73, 0x20, 0x49, 0x6E,
        0x73, 0x74, 0x72, 0x75, 0x6D, 0x65, 0x6E, 0x74,
        0x73, 0x2C, 0x20, 0x49, 0x6E, 0x63, 0x2E,

    /* Product string descriptor : Index 2 - "USBX CDC ACM Example Device" */
        0x09, 0x04, /* Language */
        0x02, /* Index */
        0x1B, /* Length */
        0x55, 0x53, 0x42, 0x58, 0x20, 0x43, 0x44, 0x43,
        0x20, 0x41, 0x43, 0x4D, 0x20, 0x45, 0x78, 0x61,
        0x6D, 0x70, 0x6C, 0x65, 0x20, 0x44, 0x65, 0x76,
        0x69, 0x63, 0x65,

    /* Serial Number string descriptor : Index 3 - "0001" */
        0x09, 0x04, /* Language */
        0x03, /* Index */
        0x04, /* Length */
        0x30, 0x30, 0x30, 0x31
    };

    /* Multiple languages are supported on the device, to add
       a language besides english, the unicode language code must
       be appended to the language_id_framework array and the length
       adjusted accordingly. */
unsigned char cdc_acm_language_id_framework[] = {

    /* English. */
        0x09, 0x04
    };

/* Save the size of each structure to const variable for the usb device framework init function. */
const size_t cdc_acm_device_framework_full_speed_sz = sizeof(cdc_acm_device_framework_full_speed);
const size_t cdc_acm_device_framework_high_speed_sz = sizeof(cdc_acm_device_framework_high_speed);
const size_t cdc_acm_string_framework_sz = sizeof(cdc_acm_string_framework);
const size_t cdc_acm_language_id_framework_sz = sizeof(cdc_acm_language_id_framework);
