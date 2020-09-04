/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \defgroup USB_MODULE APIs for USB
 *  \ingroup DRV_MODULE
 *
 *  This module has APIs for USB device driver porting layer.
 *  See this page, \ref USB_DEVICE_DRIVER, for using USB using tinyUSB APIs
 *  @{
 */

 /** \file usb_init.h
 *
 *   \brief This file contains USB initialization APIs
 */

#ifndef INCLUDE_USB_INIT_PARAMS_H_
#define INCLUDE_USB_INIT_PARAMS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor USB_Instance_Number_t
 *  \name USB Instance Number (usb_init_param_s.instanceNum)
 *  @{
 */
/** \brief USB Instance 0 */
#define USB_INSTANCE_0   (0x00U)
/** @} */

/**
 *  \anchor USB_Clock_Out_Selection_Value_t
 *  \name USB Clock Out Selection Value (usb_init_param_s.clkSrcSel)
 *  @{
 */
/** \brief Use HFOSC0 CLKOUT */
#define USB_CLK_SEL_HFOSC0_CLKOUT   (0x00U)
/** \brief Use HFOSC1 CLKOUT */
#define USB_CLK_SEL_HFOSC1_CLKOUT   (0x01U)
/** \brief Use MAIN_PLL3_HSDIV4_CLKOUT */
#define USB_MAIN_PLL3_HSDIV4_CLKOUT (0x02U)
/** \brief Use MAIN_PLL2_HSDIV4_CLKOUT */
#define USB_MAIN_PLL2_HSDIV4_CLKOUT (0x03U)
/** @} */

/**
 *  \anchor USB_PLL_Reference_Clock_Frequency_Selection_Value_t
 *  \name USB PLL Reference Clock Frequency Selection Value (usb_init_param_s.pllRefSel)
 *  @{
 */
/** \brief Use 9.6Mhz */
#define USB_PLL_REF_CLK_9P6MHZ          (0x00)
/** \brief Use 10Mhz */
#define USB_PLL_REF_CLK_10MHZ           (0x01)
/** \brief Use 12Mhz */
#define USB_PLL_REF_CLK_12MHZ           (0x02)
/** \brief Use 19.2Mhz */
#define USB_PLL_REF_CLK_19P2MHZ         (0x03)
/** \brief Use 20Mhz */
#define USB_PLL_REF_CLK_20MHZ           (0x04)
/** \brief Use 24Mhz */
#define USB_PLL_REF_CLK_24MHZ           (0x05)
/** \brief Use 25Mhz */
#define USB_PLL_REF_CLK_25MHZ           (0x06)
/** \brief Use 26Mhz */
#define USB_PLL_REF_CLK_26MHZ           (0x07)
/** \brief Use 38.4Mhz */
#define USB_PLL_REF_CLK_38P4MHZ         (0x08)
/** \brief Use 40Mhz */
#define USB_PLL_REF_CLK_40MHZ           (0x09)
/** \brief Use 48Mhz */
#define USB_PLL_REF_CLK_48MHZ           (0x0A)
/** \brief Use 50Mhz */
#define USB_PLL_REF_CLK_50MHZ           (0x0B)
/** \brief Use 52Mhz */
#define USB_PLL_REF_CLK_52MHZ           (0x0C)
/** @} */

/**
 *  \anchor USB_Mode_Value_t
 *  \name USB Mode Value (usb_init_param_s.usb2Enable)
 *  @{
 */
/** \brief USB 3.0 mode */
#define USB_MODE_3P0                    (0x0)
/** \brief USB 2.0 mode */
#define USB_MODE_2P0                    (0x1)
/** @} */

/**
 *  \anchor USB_Pipe_Receive_Invert_Polarity_Value_t
 *  \name USB Pipe Receive Invert Polarity Value (usb_init_param_s.invertPolarity)
 *  @{
 */
/** \brief USB Pipe Receive Invert Polarity Disabled */
#define USB_PIPE_RX_INVERT_POLARITY_DISABLED        (0x0)
/** \brief USB 2.0 only enable (USB 3.0 disabled) */
#define USB_PIPE_RX_INVERT_POLARITY_ENABLED         (0x1)
/** @} */

/**
 *  \anchor USB_Vbus_External_Divider_Value_t
 *  \name USB VBUS External Divider Value (usb_init_param_s.vbusSel)
 *  @{
 */
/** \brief VBUS External Divider Inactive */
#define USB_VBUS_EXT_DIV3_INACTIVE      (0x0)
/** \brief VBUS External Divider active */
#define USB_VBUS_EXT_DIV3_ACTIVE        (0x1)
/** @} */

/**
 *  \anchor USB_Host_Mode_Value_t
 *  \name USB Host Mode Value (usb_init_param_s.isHostMode)
 *  @{
 */
/** \brief USB Host Mode Disabled */
#define USB_HOST_MODE_DISABLED        (0x0)
/** \brief USB Host Mode Enabled */
#define USB_HOST_MODE_ENABLED         (0x1)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief USB intialization parameter block */
typedef struct usb_init_param_s {
    uint32_t instanceNum;
    /**< \ref USB_Instance_Number_t */
    uint32_t clkSrcSel;
    /**< \ref USB_Clock_Out_Selection_Value_t */
    uint32_t pllRefSel;
    /**< \ref USB_PLL_Reference_Clock_Frequency_Selection_Value_t */
    uint32_t usb2Enable;
    /**< \ref USB_Mode_Value_t */
    uint32_t invertPolarity;
    /**< \ref USB_Pipe_Receive_Invert_Polarity_Value_t */
    uint32_t vbusSel;
    /**< \ref USB_Vbus_External_Divider_Value_t */
    uint32_t isHostMode;
    /**< \ref USB_Host_Mode_Value_t */
} usb_init_param_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief USB Initialization function
 *
 * Set up the USB instance using the USB intialization parameter block:
 *    - USB clock source selection,
 *    - PHY ref clock frequency selection,
 *    - USB 2.0 enable (disable USB 3.0),
 *    - Invert polarity,
 *    - VBUS_SEL for external divider and
 *    - USB host/device mode.
 */
void usb_init(usb_init_param_t *usbInitParamPtr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_USB_INIT_PARAMS_H_ */

/** @} */
