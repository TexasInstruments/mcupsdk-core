/**
 * @file   usb_init.c
 *
 * @brief  usb initilization related functions
 */
/*
 * Copyright (c) 2019 - 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================*/

#include "usb_init.h"
#include "usbss_functions.h"

#include <drivers/hw_include/tistdtypes.h>
#include "hw_usb.h"
#include "usb_wrapper.h"

extern usb_handle_t     usb_handle;      /* global so it can be accessed in ISRs */

usb_init_param_t usbInitParam = {
    USB_INSTANCE_0,
    USB_CLK_SEL_HFOSC0_CLKOUT,
    USB_PLL_REF_CLK_25MHZ,
    USB_MODE_2P0,
    USB_PIPE_RX_INVERT_POLARITY_DISABLED,
    USB_VBUS_EXT_DIV3_ACTIVE,
    USB_HOST_MODE_DISABLED
};

/*
 *  ======== borad_init ========
 */
void usb_init(usb_init_param_t *usbInitParamPtr)
{
    usb_init_param_t *localPtr = usbInitParamPtr;

    if(usbInitParamPtr == NULL)
    {
        /* if input pointre is NULL, use the default one */
        localPtr = &usbInitParam;
    }

    usb_instance_select(&usb_handle, localPtr->instanceNum);    /* Initializes the usb_handle struct. Selects the default instance */

    usbPinmuxConfig(localPtr); /* Pinmux for USB */

    usbPhyOn(localPtr); /* port No and no invert polarity */

    usbClockCfg(localPtr); /* clock set up for USB */

    usbSetupDeviceMode(localPtr); /* set to device mode */

    usbDeviceIntSetup(localPtr); /* set up interrupt for USB */
}
