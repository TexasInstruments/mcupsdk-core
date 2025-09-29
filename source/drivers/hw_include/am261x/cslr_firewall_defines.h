/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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
 *
 */

#ifndef CSLR_FIREWALL_DEFINES_H_
#define CSLR_FIREWALL_DEFINES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** @brief Number of UART instances */
#define CSL_FW_CNT                (18U)


/* SLAVE FIREWALLS */

/***********************************************************************
 * FW R5SS0_CORE0_AXIS_SLV
 ***********************************************************************/
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_ID              (0U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_CFG_ADDR        (0x400A0000U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_NUM_REGION      (8U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_NUM_PROTECTED   (4U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR0     (0x78000000U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE0    (256U*1024U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR1     (0x78100000U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE1    (256U*1024U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR2     (0x74000000U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE2    (8U*1024U*1024U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR3     (0x74800000U)
#define CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE3    (8U*1024U*1024U)

/***********************************************************************
 * FW R5SS0_CORE1_AXIS_SLV
 ***********************************************************************/
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_ID              (1U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_CFG_ADDR        (0x400C0000U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_NUM_REGION      (8U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_NUM_PROTECTED   (4U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR0     (0x78200000U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE0    (128U*1024U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR1     (0x78300000U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE1    (128U*1024U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR2     (0x75000000U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE2    (8U*1024U*1024U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR3     (0x75800000U)
#define CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE3    (8U*1024U*1024U)

/***********************************************************************
 * FW L2OCRAM_BANK0_SLV
 ***********************************************************************/
#define CSL_FW_L2OCRAM_BANK0_SLV_ID              (2U)
#define CSL_FW_L2OCRAM_BANK0_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_L2OCRAM_BANK0_SLV_CFG_ADDR        (0x40020000U)
#define CSL_FW_L2OCRAM_BANK0_SLV_NUM_REGION      (8U)
#define CSL_FW_L2OCRAM_BANK0_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_L2OCRAM_BANK0_SLV_START_ADDR0     (0x70000000U)
#define CSL_FW_L2OCRAM_BANK0_SLV_REGION_SIZE0    (512U*1024U)

/***********************************************************************
 * FW L2OCRAM_BANK1_SLV
 ***********************************************************************/
#define CSL_FW_L2OCRAM_BANK1_SLV_ID              (3U)
#define CSL_FW_L2OCRAM_BANK1_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_L2OCRAM_BANK1_SLV_CFG_ADDR        (0x40040000U)
#define CSL_FW_L2OCRAM_BANK1_SLV_NUM_REGION      (8U)
#define CSL_FW_L2OCRAM_BANK1_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_L2OCRAM_BANK1_SLV_START_ADDR0     (0x70080000U)
#define CSL_FW_L2OCRAM_BANK1_SLV_REGION_SIZE0    (512U*1024U)

/***********************************************************************
 * FW L2OCRAM_BANK2_SLV
 ***********************************************************************/
#define CSL_FW_L2OCRAM_BANK2_SLV_ID              (4U)
#define CSL_FW_L2OCRAM_BANK2_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_L2OCRAM_BANK2_SLV_CFG_ADDR        (0x40060000U)
#define CSL_FW_L2OCRAM_BANK2_SLV_NUM_REGION      (8U)
#define CSL_FW_L2OCRAM_BANK2_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_L2OCRAM_BANK2_SLV_START_ADDR0     (0x70100000U)
#define CSL_FW_L2OCRAM_BANK2_SLV_REGION_SIZE0    (512U*1024U)

/***********************************************************************
 * FW MBOX_RAM_SLV
 ***********************************************************************/
#define CSL_FW_MBOX_RAM_SLV_ID              (5U)
#define CSL_FW_MBOX_RAM_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_MBOX_RAM_SLV_CFG_ADDR        (0x40140000U)
#define CSL_FW_MBOX_RAM_SLV_NUM_REGION      (8U)
#define CSL_FW_MBOX_RAM_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_MBOX_RAM_SLV_START_ADDR0     (0x72000000U)
#define CSL_FW_MBOX_RAM_SLV_REGION_SIZE0    (16U*1024U)

/***********************************************************************
 * FW HSM_SLV
 ***********************************************************************/
#define CSL_FW_HSM_SLV_ID              (6U)
#define CSL_FW_HSM_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_HSM_SLV_CFG_ADDR        (0x40240000U)
#define CSL_FW_HSM_SLV_NUM_REGION      (8U)
#define CSL_FW_HSM_SLV_NUM_PROTECTED   (2U)
#define CSL_FW_HSM_SLV_START_ADDR0     (0x20000000U)
#define CSL_FW_HSM_SLV_REGION_SIZE0    (128U*1024U*1024U)
#define CSL_FW_HSM_SLV_START_ADDR1     (0x40000000U)
#define CSL_FW_HSM_SLV_REGION_SIZE1    (128U*1024U*1024U)

/***********************************************************************
 * FW DTHE_SLV
 ***********************************************************************/
#define CSL_FW_DTHE_SLV_ID              (7U)
#define CSL_FW_DTHE_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_DTHE_SLV_CFG_ADDR        (0x40120000U)
#define CSL_FW_DTHE_SLV_NUM_REGION      (8U)
#define CSL_FW_DTHE_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_DTHE_SLV_START_ADDR0     (0xCE000000U)
#define CSL_FW_DTHE_SLV_REGION_SIZE0    (16U*1024U*1024U)

/***********************************************************************
 * FW OSPI0_CONFIG_SLV
 ***********************************************************************/
#define CSL_FW_OSPI0_CFG_SLV_ID              (8U)
#define CSL_FW_OSPI0_CFG_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_OSPI0_CFG_SLV_CFG_ADDR        (0x40260000U)
#define CSL_FW_OSPI0_CFG_SLV_NUM_REGION      (4U)
#define CSL_FW_OSPI0_CFG_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_OSPI0_CFG_SLV_START_ADDR0     (0x53800000U)
#define CSL_FW_OSPI0_CFG_SLV_REGION_SIZE0    (64U*1024U)

/***********************************************************************
 * FW OSPI1_CONFIG_SLV
 ***********************************************************************/
#define CSL_FW_OSPI1_CFG_SLV_ID              (9U)
#define CSL_FW_OSPI1_CFG_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_OSPI1_CFG_SLV_CFG_ADDR        (0x40320000U)
#define CSL_FW_OSPI1_CFG_SLV_NUM_REGION      (4U)
#define CSL_FW_OSPI1_CFG_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_OSPI1_CFG_SLV_START_ADDR0     (0x53A00000U)
#define CSL_FW_OSPI1_CFG_SLV_REGION_SIZE0    (12U*1024U)
/***********************************************************************
 * FW R5SS0_CONFIG_SLV
 ***********************************************************************/
#define CSL_FW_R5SS0_SLV_ID              (10U)
#define CSL_FW_R5SS0_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_R5SS0_SLV_CFG_ADDR        (0x40280000U)
#define CSL_FW_R5SS0_SLV_NUM_REGION      (8U)
#define CSL_FW_R5SS0_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_R5SS0_SLV_START_ADDR0     (0x50000000U)
#define CSL_FW_R5SS0_SLV_REGION_SIZE0    (256U*1024U*1024U)

/***********************************************************************
 * FW OSPI0_SLV
 ***********************************************************************/
#define CSL_FW_OSPI0_SLV_ID              (11U)
#define CSL_FW_OSPI0_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_OSPI0_SLV_CFG_ADDR        (0x40160000U)
#define CSL_FW_OSPI0_SLV_NUM_REGION      (8U)
#define CSL_FW_OSPI0_SLV_NUM_PROTECTED   (3U)
#define CSL_FW_OSPI0_SLV_START_ADDR0     (0x60000000U)
#define CSL_FW_OSPI0_SLV_REGION_SIZE0    (128U*1024U*1024U)
#define CSL_FW_OSPI0_SLV_START_ADDR1     (0x80000000U)
#define CSL_FW_OSPI0_SLV_REGION_SIZE1    (128U*1024U*1024U)
#define CSL_FW_OSPI0_SLV_START_ADDR2     (0x88000000U)
#define CSL_FW_OSPI0_SLV_REGION_SIZE2    (128U*1024U*1024U)

/***********************************************************************
 * FW OSPI1_SLV
 ***********************************************************************/
#define CSL_FW_OSPI1_SLV_ID              (12U)
#define CSL_FW_OSPI1_SLV_TYPE            (CSL_FW_SLV)
#define CSL_FW_OSPI1_SLV_CFG_ADDR        (0x40300000U)
#define CSL_FW_OSPI1_SLV_NUM_REGION      (8U)
#define CSL_FW_OSPI1_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_OSPI1_SLV_START_ADDR0     (0xA0000000U)
#define CSL_FW_OSPI1_SLV_REGION_SIZE0    (128U*1024U*1024U)


/* MASTER FIREWALLS */

/***********************************************************************
 * FW SCRM2SCRP0
 ***********************************************************************/
#define CSL_FW_SCRM2SCRP0_SLV_ID              (13U)
#define CSL_FW_SCRM2SCRP0_SLV_TYPE            (CSL_FW_MST)
#define CSL_FW_SCRM2SCRP0_SLV_CFG_ADDR        (0x40180000U)
#define CSL_FW_SCRM2SCRP0_SLV_NUM_REGION      (16U)
#define CSL_FW_SCRM2SCRP0_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_SCRM2SCRP0_SLV_START_ADDR0     (0x50000000U)
#define CSL_FW_SCRM2SCRP0_SLV_REGION_SIZE0    (256U*1024U*1024U)

/***********************************************************************
 * FW SCRM2SCRP1
 ***********************************************************************/
#define CSL_FW_SCRM2SCRP1_SLV_ID              (14U)
#define CSL_FW_SCRM2SCRP1_SLV_TYPE            (CSL_FW_MST)
#define CSL_FW_SCRM2SCRP1_SLV_CFG_ADDR        (0x401A0000U)
#define CSL_FW_SCRM2SCRP1_SLV_NUM_REGION      (16U)
#define CSL_FW_SCRM2SCRP1_SLV_NUM_PROTECTED   (1U)
#define CSL_FW_SCRM2SCRP1_SLV_START_ADDR0     (0x50000000U)
#define CSL_FW_SCRM2SCRP1_SLV_REGION_SIZE0    (256U*1024U*1024U)

/***********************************************************************
 * FW  R5SS0_CORE0_AHB_MST
 ***********************************************************************/
#define CSL_FW_R5SS0_CORE0_AHB_MST_ID              (15U)
#define CSL_FW_R5SS0_CORE0_AHB_MST_TYPE            (CSL_FW_MST)
#define CSL_FW_R5SS0_CORE0_AHB_MST_CFG_ADDR        (0x401C0000U)
#define CSL_FW_R5SS0_CORE0_AHB_MST_NUM_REGION      (16U)
#define CSL_FW_R5SS0_CORE0_AHB_MST_NUM_PROTECTED   (1U)
#define CSL_FW_R5SS0_CORE0_AHB_MST_START_ADDR0     (0x50000000U)
#define CSL_FW_R5SS0_CORE0_AHB_MST_REGION_SIZE0    (256U*1024U*1024U)

/***********************************************************************
 * FW  R5SS0_CORE1_MST
 ***********************************************************************/
#define CSL_FW_R5SS0_CORE1_AHB_MST_ID              (16U)
#define CSL_FW_R5SS0_CORE1_AHB_MST_TYPE            (CSL_FW_MST)
#define CSL_FW_R5SS0_CORE1_AHB_MST_CFG_ADDR        (0x401E0000U)
#define CSL_FW_R5SS0_CORE1_AHB_MST_NUM_REGION      (16U)
#define CSL_FW_R5SS0_CORE1_AHB_MST_NUM_PROTECTED   (1U)
#define CSL_FW_R5SS0_CORE1_AHB_MST_START_ADDR0     (0x50000000U)
#define CSL_FW_R5SS0_CORE1_AHB_MST_REGION_SIZE0    (256U*1024U*1024U)

#ifdef __cplusplus
}
#endif

#endif  /* CSLR_FIREWALL_DEFINES_H_ */