/**
 * \file  am64x/hw_usb.h
 *
 * \brief Macros for use in accessing the USB registers.
 */

/*
* Copyright (C) 2019 - 2021 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
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
*
*/


#ifndef PDK__HW_USB_H
#define PDK__HW_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/cslr_soc.h>         /* Base addresses */
#include <cslr_usb3p0ss_v5_2.h>

/******************************************************************************
 *
 * The following are defines for the Univeral Serial Bus register offsets.
 *
 *****************************************************************************/

#define USB0_CDN_WRAPPER_BASE_ADDR      (CSL_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_BASE)
                                        /* 0xf400000UL */

#define USB0_XHCI_WRAPPER_BASE_ADDR     USB0_CDN_WRAPPER_BASE_ADDR

#define USB_XHCI_CORE_OFFSET            ((uint32_t)0x10000U)


#define USB0_USB_PHY_BASE_ADDR          ((uint32_t)CSL_USB0_PHY2_BASE)


#define USB_CDN_BASE_ADDR               (USB0_CDN_WRAPPER_BASE_ADDR)

/* interrupt numbers */
#define SYS_INT_USB0                    ((uint32_t)CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_0) /* main MPU IRQ */

#ifdef __cplusplus
}
#endif

#endif /* PDK__HW_USB_H */
