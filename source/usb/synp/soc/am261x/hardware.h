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


#ifndef __USB_HARDWARE_H__
#define  __USB_HARDWARE_H__

#define MSS_CTRL           0x50D00000U
#define USB2PHY            0x53944000U
#define USB_DWC_3          0x53910000U
#define USB_OCP2SCP_REG    0x53940000U
#define USB_OTGSS_C2       0x53900000U
#define USB_RAM0           0x53908000U
#define DEVICE_TOPRCM_BASE CSL_TOP_RCM_U_BASE

/* USB Interrupt numbers wrt VIM */
#define USB20_MAIN0_INT                  116U
#define USB20_MAIN1_INT                  117U
#define USB20_MAIN2_INT                  118U
#define USB20_MAIN3_INT                  119U
#define USB20_MISC_INT                   120U
#define USB20_PHY_WAKEUP_WUOUT           121U
#define USB20_SLVP_SWAKEUP               122U

/* USB Interrupt numbers wrt USB_OTGSS_C2 */
#define USB_OTG_MAIN0_INT                  0U
#define USB_OTG_MAIN1_INT                  1U
#define USB_OTG_MAIN2_INT                  2U
#define USB_OTG_MAIN3_INT                  3U
#define USB_OTG_MISC_INT                   4U

/* Update the base address of SYNP USB controller as per the design of AM261x
 *
 */
#define USB_CTRL_BASE_ADDR     USB_DWC_3
/* only USB0 on AM261x can be USB DEV */
#define USB_DEV_INSTANCE        (0U)

#endif

