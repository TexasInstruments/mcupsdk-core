/**
 * @file   usbss_functions.c
 *
 * @brief  usb core related functions
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

#ifndef _USBSS_FUNCTIONS_C_
#define _USBSS_FUNCTIONS_C_

#include "hw_usb.h"
#include "usbss_functions.h"
#include "cdn_print.h"

/* #include "tusb_common.h" */
/* #include "device/dcd.h" */

void usbss_reset_assert( usb_handle_t *h ) {
    uint32_t regval;

    regval = CSL_REG32_RD( &(h->p_usbregs_tiwrap->USB3P0SS_W1));
    CSL_FINS( regval, USB3P0SS_CMN_USB3P0SS_W1_PWRUP_RST_N, 0);
    CSL_REG32_WR( &(h->p_usbregs_tiwrap->USB3P0SS_W1), regval);

} /* usbss_reset_assert() */

void usbss_reset_release( usb_handle_t *h ) {
    uint32_t regval;

    regval = CSL_REG32_RD( &(h->p_usbregs_tiwrap->USB3P0SS_W1) );
    CSL_FINS( regval, USB3P0SS_CMN_USB3P0SS_W1_PWRUP_RST_N, 1 );
    CSL_REG32_WR( &(h->p_usbregs_tiwrap->USB3P0SS_W1), regval );

    uint32_t    otgsts, otg_nrdy;
    uint8_t     pollcnt = 0;
    uint32_t    usbss_ready_n = 0;                 /* 0 = ready, 1 = not ready */
    uint32_t    max_pollcnt = 100;

    CSL_REG32_WR(&(h->p_usbregs_ctlr->DRD.SESSVALID_DBNC_CFG), 0x000A0002);

    do {
        ClockP_usleep(10);
        otgsts   = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
        otg_nrdy = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY );
        pollcnt++;
    } while( (otg_nrdy != usbss_ready_n) && (pollcnt <= max_pollcnt) );

    /* Add some delay for reset release to take effect */
    pollcnt = 0;
    do {
        ClockP_usleep(10);
        otgsts   = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
        otg_nrdy = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY );
        pollcnt++;
    } while( (otg_nrdy != usbss_ready_n) && (pollcnt <= max_pollcnt) );

    /* Add some delay for reset release to take effect */
    pollcnt = 0;
    do {
        ClockP_usleep(10);
        otgsts   = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
        otg_nrdy = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY );
        pollcnt++;
    } while( (otg_nrdy != usbss_ready_n) && (pollcnt <= max_pollcnt) );

    if( pollcnt > max_pollcnt ) {
        vDbgMsg(USBSSP_DBG_CUSBD,DBG_WARN,"USB reset release exceeds maximum timeout: pollcnt > %d\n", max_pollcnt);
    }
} /* usbss_reset_release() */

/* ----- */

/* Check USB controller ready */
uint8_t check_usbss_ready ( usb_handle_t *h ) {
    uint32_t    otgsts, otg_nrdy;
    uint8_t     pollcnt = 0;
    uint32_t    usbss_ready_n = 0;                 /* 0 = ready, 1 = not ready */
    uint32_t    max_pollcnt = 100;

    otgsts   = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
    otg_nrdy = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY );

    do {
        ClockP_usleep(10);
        otgsts   = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
        otg_nrdy = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY );
        pollcnt++;
    } while( (otg_nrdy != usbss_ready_n) && (pollcnt <= max_pollcnt) );

    if( pollcnt > max_pollcnt ) {
        vDbgMsg(USBSSP_DBG_CUSBD,DBG_WARN,"USB reset release exceeds maximum timeout: pollcnt > %d\n", max_pollcnt);
        return 1; /* Error */
    } else {
      return 0; /* Ok */
    }
}

/* Set USB to the host mode */
void set_usbss_host_mode( usb_handle_t *h ) {
    uint32_t regval;

    regval = CSL_REG32_RD( &(h->p_usbregs_tiwrap->USB3P0SS_W1) );
    CSL_FINS( regval, USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SEL, 1 );
    CSL_FINS( regval, USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP,     1 ); /* 0=neither, 1=host_mode, 2=device_mode */
    CSL_REG32_WR( &(h->p_usbregs_tiwrap->USB3P0SS_W1), regval );
} /* set_usbss_host_mode() */

/* ----- */

/* Set USB to the device mode */
void set_usbss_device_mode( usb_handle_t *h ) {
    uint32_t regval;

    regval = CSL_REG32_RD( &(h->p_usbregs_tiwrap->USB3P0SS_W1) );
    CSL_FINS( regval, USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SEL, 1 );
    CSL_FINS( regval, USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP,     2 ); /* 0=neither, 1=host_mode, 2=device_mode */
    CSL_REG32_WR( &(h->p_usbregs_tiwrap->USB3P0SS_W1), regval );
} /* set_usbss_device_mode() */

/* ----- */

/* Check host mode */
uint8_t check_usbss_host_mode ( usb_handle_t *h ) {
    uint32_t    otgsts, drd_mode, xhc_ready;
    uint8_t     pollcnt = 0;
    uint32_t    expected_strap_mode = 2;                                 /* 0=neither, 2=host_mode, 4=device_mode */
    uint32_t    expected_xhc_ready  = 1;
    uint32_t    max_pollcnt = 10;

    otgsts    = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
    drd_mode  = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_STRAP );
    xhc_ready = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_HOST_ACTIVE );

    do {
        ClockP_usleep(10);
        otgsts    = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
        drd_mode  = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_STRAP );
        xhc_ready = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_HOST_ACTIVE );
        pollcnt++;
    } while( (drd_mode != expected_strap_mode) && (xhc_ready != expected_xhc_ready) && (pollcnt <= max_pollcnt) );

    if( pollcnt > max_pollcnt ) {
        vDbgMsg(USBSSP_DBG_CUSBD,DBG_WARN,"USB reset release exceeds maximum timeout: pollcnt > %d\n", max_pollcnt);
        return 1; /* Error */
    }
    return 0; /* Ok */
} /* check_usbss_host_mode() */

/* ----- */

/* Check device mode */
uint8_t check_usbss_device_mode( usb_handle_t *h ) {
    uint32_t    otgsts, drd_mode, dev_ready;
    uint8_t     pollcnt = 0;
    uint32_t    expected_strap_mode = 4;                                 /* 0=neither, 2=host_mode, 4=device_mode */
    uint32_t    expected_dev_ready  = 1;
    uint32_t    max_pollcnt = 10;

    otgsts    = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
    drd_mode  = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_STRAP );
    dev_ready = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_DEV_READY );

    do {
        ClockP_usleep(10);
        otgsts    = CSL_REG32_RD( &(h->p_usbregs_ctlr->DRD.OTGSTS) );
        drd_mode  = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_STRAP );
        dev_ready = CSL_FEXT( otgsts, USB3P0SS_CTRL_DRD_OTGSTS_DEV_READY );
        pollcnt++;
    } while( (drd_mode != expected_strap_mode) && (dev_ready != expected_dev_ready) && (pollcnt <= max_pollcnt) );

    if( pollcnt > max_pollcnt ) {
        vDbgMsg(USBSSP_DBG_CUSBD,DBG_WARN,"USB reset release exceeds maximum timeout: pollcnt > %d\n", max_pollcnt);
        return 1; /* Error */
    }
    return 0; /* Ok */
} /* check_usbss_device_mode() */

/* ----- */

uint32_t usb_instance_select( usb_handle_t *h, uint32_t instance ) {
    if(instance != 0) return 1;

    /* base addresses */
    h->cfg_base             = CSL_USB0_MMR_MMRVBP_USBSS_CMN_BASE;
    /* Assign CSL Base addresses for each interface's rsel region: */
    /* -- usb core if -- */
    h->p_usbregs_tiwrap     = (CSL_usb3p0ss_cmnRegs *) CSL_USB0_MMR_MMRVBP_USBSS_CMN_BASE;
    h->p_usbregs_ctlr       = (CSL_usb3p0ss_ctrlRegs *) CSL_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_BASE;
    /* -- usb phy2 if -- */
    h->p_usbregs_phy2       = (CSL_usb3p0ss_phy2Regs *) CSL_USB0_PHY2_BASE;

    h->dinum = instance;

    return 0;
} /* usb_instance_select() */

/* ----- */

void usb_local_disable_module( uint32_t cfg_base ) {
    uint32_t read_val;

    read_val = *mkptr(cfg_base, CSL_USB3P0SS_CMN_STATIC_CONFIG);
    *mkptr(cfg_base, CSL_USB3P0SS_CMN_STATIC_CONFIG) = read_val | 0x000001ffu;
    read_val = *mkptr(cfg_base, CSL_USB3P0SS_CMN_STATIC_CONFIG);
}

#endif
