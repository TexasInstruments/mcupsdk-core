/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef USB_WRAPPER_H_
#define USB_WRAPPER_H_

#include "device_usb_reg_offset.h"
#include "usb_drv.h"
#include "hardware.h"
#include <kernel/dpl/HwiP.h> 
#include <kernel/dpl/CycleCounterP.h>
#include <cslr_soc.h>

/* Define 60s timeout value for usb connection to happen */
/* Define QT_SIM macro to test timeout in simulation platform */
/* This lowers the timeout value as QT simulation is slow */

#ifndef QT_SIM
#define BOOTPARAM_TIMEOUT_MS 60000U
#else
#define BOOTPARAM_TIMEOUT_MS 20U
#endif

 /**
 *  \defgroup USB_MODULE APIs for USB
 *  \ingroup DRV_MODULE
 *
 *  This module has APIs for USB device driver.
 *  See this page, \ref USB_DEVICE_DRIVER, for using USB using tinyUSB APIs
 *  @{
 */

/** \file device_wrapper.h
 *
 *   \brief This file contains USB device wrapper APIs
 */

/**
 * \brief Structure representing the USB device handle.
 */
typedef struct usb_handle_s {
    /** Base address of the USB DWC3 interface configuration registers. */
    uint32_t cfg_base;  
    /** Pointer to the global USB device structure. */
    dwc_usb3_device_t* dwc_usb3_dev;
    /** Parameters for configuring the USB interrupt. */
    HwiP_Params hwiParamsUsb;
    /** Object representing the USB interrupt handler. */
    HwiP_Object hwiObjUsb; 
} usb_handle_t;

/**
 * \brief This enumeration defines the return status for USB PHY. 
 */
typedef enum UsbPhy_ret_t_
{
    USB_PHY_OK = 0x01U ,
    USB_PHY_ERR = 0x02U

}UsbPhy_ret_t ;


/** 
 * \brief Global buffers for USB IN endpoints. 
 * These buffers are used for data transmission from the device to the host.
 */
extern uint8_t    ep_in_buf[DWC_MAX_EPS - 1U][DWC_MAX_PACKET_SIZE];

/** 
 * \brief Global buffers for USB OUT endpoints. 
 * These buffers are used for data reception from the host to the device.
 */
extern uint8_t    ep_out_buf[DWC_MAX_EPS - 1U][DWC_MAX_PACKET_SIZE];

/**
 * @brief Performs the USB PHY power-on sequence.
 *
 * This function initializes the USB PHY by performing the necessary
 * power-on sequence.
 *
 * @return #UsbPhy_ret_t indicating the status of the operation.
 */
UsbPhy_ret_t usb_phy_power_sequence(void);

/**
 * @brief This function configures the interrupt
 *
 * @param intr Interrupt number or identifier.
 *             Specifies the interrupt to be handled or configured.
 */
void USB_configureInterrupt(uint32_t intr);

/**
 * @brief This function disables the interrupt
 *
 * @param intr Interrupt number or identifier.
 *             Specifies the interrupt to be handled or configured.
 */
void USB_disableInterrupt(uint32_t intr);

/**
 * @brief This function clears the MAIN0 interrupt
 */
void USB_clearInterrupt(void);

#endif /* USB_WRAPPER_H_ */
/**
 *  @}
 */