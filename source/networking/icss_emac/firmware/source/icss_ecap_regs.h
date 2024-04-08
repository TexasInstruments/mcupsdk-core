;************************************************************************************
;**+------------------------------------------------------------------------------+**
;**|                              ******                                          |**
;**|                              ******     o                                    |**
;**|                              *******__////__****                             |**
;**|                              ***** /_ //___/ ***                             |**
;**|                           ********* ////__ ******                            |**
;**|                             *******(_____/ ******                            |**
;**|                                 **********                                   |**
;**|                                   ******                                     |**
;**|                                      ***                                     |**
;**|                                                                              |**
;**|            Copyright (c) 1998-2016 Texas Instruments Incorporated            |**
;**|                           ALL RIGHTS RESERVED                                |**
;**|                                                                              |**
;**|    Permission is hereby granted to licensees of Texas Instruments            |**
;**|    Incorporated (TI) products to use this computer program for the sole      |**
;**|    purpose of implementing a licensee product based on TI products.          |**
;**|    No other rights to reproduce, use, or disseminate this computer           |**
;**|    program, whether in part or in whole, are granted.                        |**
;**|                                                                              |**
;**|    TI makes no representation or warranties with respect to the              |**
;**|    performance of this computer program, and specifically disclaims          |**
;**|    any responsibility for any damages, special or consequential,             |**
;**|    connected with the use of this program.                                   |**
;**|                                                                              |**
;**+------------------------------------------------------------------------------+**
;************************************************************************************
; file:     icss_ecap_regs.h
;
; brief:    ICSS Enhanced Capture Module Registers
;
;   (C) Copyright 2016, Texas Instruments, Inc
;
;   Version     Description                             Author
;
;   0.1         Created                                 Vijaya Sarathy
;
;************************************************************************************

    .if !$defined("____icss_ecap_regs_h")
____icss_ecap_regs_h    .set    1

ICSS_eCAP_TSCNT                 .set    0x0000
ICSS_eCAP_CNTPHS                .set    0x0004
ICSS_eCAP_CAP1                  .set    0x0008
ICSS_eCAP_CAP2                  .set    0x000C
ICSS_eCAP_CAP3                  .set    0x0010
ICSS_eCAP_CAP4                  .set    0x0014
ICSS_eCAP_RSVD0                 .set    0x0018
ICSS_eCAP_ECCTL1                .set    0x0028
ICSS_eCAP_ECCTL2                .set    0x002A
ICSS_eCAP_ECCTL2_TSCTRSTOP_STOP .set    4
ICSS_eCAP_ECEINT                .set    0x002C
ICSS_eCAP_ECFLG                 .set    0x002E
ICSS_eCAP_ECCLR                 .set    0x0030
ICSS_eCAP_ECCLR_PRDEQ_SHIFT     .set    6
ICSS_eCAP_ECFRC                 .set    0x0034
ICSS_eCAP_RSVD1                 .set    0x0038
ICSS_eCAP_PID                   .set    0x005C

    .endif ;____icss_ecap_regs_h
