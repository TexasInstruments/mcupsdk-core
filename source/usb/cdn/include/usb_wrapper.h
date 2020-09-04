/**
 *  \file     usb_wrapper.h
 *
 *  \brief    This file contains APIs for manipulating the SOC specific wrapper.
 *
 *  \copyright Copyright (C) 2016-2021 Texas Instruments Incorporated -
 *             http://www.ti.com/
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "stddef.h"
#include "stdint.h"
#include "string.h"

#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/am64x_am243x/cslr_main_pll_mmr.h>

#include "byteorder.h"
#include "cusbd_if.h"
#include "cusbd_structs_if.h"
#include "cusbd_obj_if.h"
#include "cusb_ch9_if.h"
#include "cusb_ch9_structs_if.h"

#include "usbss_functions.h"
#include "usb_init.h"

#include "hw_usb.h"
#include "cps.h"

#include "cdn_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_XFER_BUFFER_COUNT 4

extern usb_handle_t     usb_handle;

extern uint8_t ep0Buff[];
extern CUSBD_Req ep0Req;
extern uint8_t *dataXferBufferOut[];
extern CUSBD_Req DataXferRequestsOut[];
extern uint8_t *dataXferBufferIn[];
extern CUSBD_Req DataXferRequestsIn[];
extern uint8_t buffEp0[];
extern CUSBD_Req ep0DataXferRequest;

/* ========================================================================== */
/*                            Functions                                       */
/* ========================================================================== */

void usbPinmuxConfig(usb_init_param_t *usbInitParams);
void usbPhyOn(usb_init_param_t *usbInitParams);
void usbClockCfg(usb_init_param_t *usbInitParams);
void usbSetupDeviceMode(usb_init_param_t *usbInitParams);
void usbDeviceIntSetup(usb_init_param_t *usbInitParams);
void usbDeviceInit(usb_init_param_t *usbInitParams);

int usbGetDataXferRequestIndexOut(CUSBD_Req * req);

void usbDisplayRequestinfo(CUSBD_Req * req);
void usbDisplayEndpointInfo(CUSBD_Ep * ep);
void usbDisplayDeviceInfo(CUSBD_Dev * dev);

#ifdef __cplusplus
}
#endif
