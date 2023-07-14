/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_firewall_defines.h
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
#define CSL_FW_CNT                (17U)

/***********************************************************************
 * FW L2_BANKA
 ***********************************************************************/
#define CSL_FW_L2_BANKA_ID                      (0U)
#define CSL_FW_L2_BANKA_TYPE                    (CSL_FW)
#define CSL_FW_L2_BANKA_CFG_ADDR                (0x40020000)
#define CSL_FW_L2_BANKA_NUM_REGION              (8)
#define CSL_FW_L2_BANKA_NUM_PROTECTED           (1)
#define CSL_FW_L2_BANKA_START_ADDR0             (0xC0200000)
#define CSL_FW_L2_BANKA_REGION_SIZE0            (512*1024)

/***********************************************************************
 * FW L2_BANKB
 ***********************************************************************/
#define CSL_FW_L2_BANKB_ID                      (1U)
#define CSL_FW_L2_BANKB_TYPE                    (CSL_FW)
#define CSL_FW_L2_BANKB_CFG_ADDR                (0x40040000)
#define CSL_FW_L2_BANKB_NUM_REGION              (8)
#define CSL_FW_L2_BANKB_NUM_PROTECTED           (1)
#define CSL_FW_L2_BANKB_START_ADDR0             (0xC0280000)
#define CSL_FW_L2_BANKB_REGION_SIZE0            (448*1024)

/***********************************************************************
 * FW HSM_DTHE
 ***********************************************************************/
#define CSL_FW_HSM_DTHE_ID                      (2U)
#define CSL_FW_HSM_DTHE_TYPE                    (CSL_FW)
#define CSL_FW_HSM_DTHE_CFG_ADDR                (0x40060000)
#define CSL_FW_HSM_DTHE_NUM_REGION              (8)
#define CSL_FW_HSM_DTHE_NUM_PROTECTED           (1)
#define CSL_FW_HSM_DTHE_START_ADDR0             (0xCE000000)
#define CSL_FW_HSM_DTHE_REGION_SIZE0            (512*1024)

/***********************************************************************
 * FW MSS_MBOX
 ***********************************************************************/
#define CSL_FW_MSS_MBOX_ID                      (3U)
#define CSL_FW_MSS_MBOX_TYPE                    (CSL_FW)
#define CSL_FW_MSS_MBOX_CFG_ADDR                (0x40080000)
#define CSL_FW_MSS_MBOX_NUM_REGION              (8)
#define CSL_FW_MSS_MBOX_NUM_PROTECTED           (1)
#define CSL_FW_MSS_MBOX_START_ADDR0             (0xC5000000)
#define CSL_FW_MSS_MBOX_REGION_SIZE0            (8*1024)

/***********************************************************************
 * FW MSS_PCRA
 ***********************************************************************/
#define CSL_FW_MSS_PCRA_ID                      (4U)
#define CSL_FW_MSS_PCRA_TYPE                    (CSL_FW)
#define CSL_FW_MSS_PCRA_CFG_ADDR                (0x400A0000)
#define CSL_FW_MSS_PCRA_NUM_REGION              (8)
#define CSL_FW_MSS_PCRA_NUM_PROTECTED           (1)
#define CSL_FW_MSS_PCRA_START_ADDR0             (0x02000000)
#define CSL_FW_MSS_PCRA_REGION_SIZE0            (16*1024*1024)

/***********************************************************************
 * FW QSPI0
 ***********************************************************************/
#define CSL_FW_QSPI0_ID                         (5U)
#define CSL_FW_QSPI0_TYPE                       (CSL_FW)
#define CSL_FW_QSPI0_CFG_ADDR                   (0x400C0000)
#define CSL_FW_QSPI0_NUM_REGION                 (8)
#define CSL_FW_QSPI0_NUM_PROTECTED              (2)
#define CSL_FW_QSPI0_START_ADDR0                (0xC8000000)
#define CSL_FW_QSPI0_REGION_SIZE0               (256*1024)
#define CSL_FW_QSPI0_START_ADDR1                (0xC6000000)
#define CSL_FW_QSPI0_REGION_SIZE1               (32*1024*1024)

/***********************************************************************
 * FW R5SS_COREA_AXIS
 ***********************************************************************/
#define CSL_FW_R5SS_COREA_AXIS_ID               (6U)
#define CSL_FW_R5SS_COREA_AXIS_TYPE             (CSL_FW)
#define CSL_FW_R5SS_COREA_AXIS_CFG_ADDR         (0x400E0000)
#define CSL_FW_R5SS_COREA_AXIS_NUM_REGION       (8)
#define CSL_FW_R5SS_COREA_AXIS_NUM_PROTECTED    (4)
#define CSL_FW_R5SS_COREA_AXIS_START_ADDR0      (0xC1000000)
#define CSL_FW_R5SS_COREA_AXIS_REGION_SIZE0     (64*1024)
#define CSL_FW_R5SS_COREA_AXIS_START_ADDR1      (0xC1800000)
#define CSL_FW_R5SS_COREA_AXIS_REGION_SIZE1     (64*1024)
#define CSL_FW_R5SS_COREA_AXIS_START_ADDR2      (0xC2000000)
#define CSL_FW_R5SS_COREA_AXIS_REGION_SIZE2     (16*1024)
#define CSL_FW_R5SS_COREA_AXIS_START_ADDR3      (0xC2800000)
#define CSL_FW_R5SS_COREA_AXIS_REGION_SIZE3     (16*1024)

/***********************************************************************
 * FW R5SS_COREB_AXIS
 ***********************************************************************/
#define CSL_FW_R5SS_COREB_AXIS_ID               (7U)
#define CSL_FW_R5SS_COREB_AXIS_TYPE             (CSL_FW)
#define CSL_FW_R5SS_COREB_AXIS_CFG_ADDR         (0x40100000)
#define CSL_FW_R5SS_COREB_AXIS_NUM_REGION       (8)
#define CSL_FW_R5SS_COREB_AXIS_NUM_PROTECTED    (4)
#define CSL_FW_R5SS_COREB_AXIS_START_ADDR0      (0xC3000000)
#define CSL_FW_R5SS_COREB_AXIS_REGION_SIZE0     (64*1024)
#define CSL_FW_R5SS_COREB_AXIS_START_ADDR1      (0xC3800000)
#define CSL_FW_R5SS_COREB_AXIS_REGION_SIZE1     (64*1024)
#define CSL_FW_R5SS_COREB_AXIS_START_ADDR2      (0xC4000000)
#define CSL_FW_R5SS_COREB_AXIS_REGION_SIZE2     (16*1024)
#define CSL_FW_R5SS_COREB_AXIS_START_ADDR3      (0xC4800000)
#define CSL_FW_R5SS_COREB_AXIS_REGION_SIZE3     (16*1024)

/***********************************************************************
 * FW L3_BANKA
 ***********************************************************************/
#define CSL_FW_L3_BANKA_ID                      (8U)
#define CSL_FW_L3_BANKA_TYPE                    (CSL_FW)
#define CSL_FW_L3_BANKA_CFG_ADDR                (0x40120000)
#define CSL_FW_L3_BANKA_NUM_REGION              (8)
#define CSL_FW_L3_BANKA_NUM_PROTECTED           (1)
#define CSL_FW_L3_BANKA_START_ADDR0             (0x88000000)
#define CSL_FW_L3_BANKA_REGION_SIZE0            (1*1024*1024)

/***********************************************************************
 * FW L3_BANKB
 ***********************************************************************/
#define CSL_FW_L3_BANKB_ID                      (9U)
#define CSL_FW_L3_BANKB_TYPE                    (CSL_FW)
#define CSL_FW_L3_BANKB_CFG_ADDR                (0x40140000)
#define CSL_FW_L3_BANKB_NUM_REGION              (8)
#define CSL_FW_L3_BANKB_NUM_PROTECTED           (1)
#define CSL_FW_L3_BANKB_START_ADDR0             (0x880E0000)
#define CSL_FW_L3_BANKB_REGION_SIZE0            (1*1024*1024)

/***********************************************************************
 * FW L3_BANKC
 ***********************************************************************/
#define CSL_FW_L3_BANKC_ID                      (10U)
#define CSL_FW_L3_BANKC_TYPE                    (CSL_FW)
#define CSL_FW_L3_BANKC_CFG_ADDR                (0x40160000)
#define CSL_FW_L3_BANKC_NUM_REGION              (8)
#define CSL_FW_L3_BANKC_NUM_PROTECTED           (1)
#define CSL_FW_L3_BANKC_START_ADDR0             (0x881C0000)
#define CSL_FW_L3_BANKC_REGION_SIZE0            (1*1024*1024)

/***********************************************************************
 * FW L3_BANKD
 ***********************************************************************/
#define CSL_FW_L3_BANKD_ID                      (11U)
#define CSL_FW_L3_BANKD_TYPE                    (CSL_FW)
#define CSL_FW_L3_BANKD_CFG_ADDR                (0x40180000)
#define CSL_FW_L3_BANKD_NUM_REGION              (8)
#define CSL_FW_L3_BANKD_NUM_PROTECTED           (1)
#define CSL_FW_L3_BANKD_START_ADDR0             (0x882AC000)
#define CSL_FW_L3_BANKD_REGION_SIZE0            (1*1024*1024)

/***********************************************************************
 * FW HWA_DMA0
 ***********************************************************************/
#define CSL_FW_HWA_DMA0_ID                      (12U)
#define CSL_FW_HWA_DMA0_TYPE                    (CSL_FW)
#define CSL_FW_HWA_DMA0_CFG_ADDR                (0x401A0000)
#define CSL_FW_HWA_DMA0_NUM_REGION              (8)
#define CSL_FW_HWA_DMA0_NUM_PROTECTED           (1)
#define CSL_FW_HWA_DMA0_START_ADDR0             (0x82000000)
#define CSL_FW_HWA_DMA0_REGION_SIZE0            (128*1024)

/***********************************************************************
 * FW HWA_DMA1
 ***********************************************************************/
#define CSL_FW_HWA_DMA1_ID                      (13U)
#define CSL_FW_HWA_DMA1_TYPE                    (CSL_FW)
#define CSL_FW_HWA_DMA1_CFG_ADDR                (0x401C0000)
#define CSL_FW_HWA_DMA1_NUM_REGION              (8)
#define CSL_FW_HWA_DMA1_NUM_PROTECTED           (1)
#define CSL_FW_HWA_DMA1_START_ADDR0             (0x82100000)
#define CSL_FW_HWA_DMA1_REGION_SIZE0            (128*1024)

/***********************************************************************
 * FW DSS_HWA_PROC
 ***********************************************************************/
#define CSL_FW_DSS_HWA_PROC_ID                  (14U)
#define CSL_FW_DSS_HWA_PROC_TYPE                (CSL_FW)
#define CSL_FW_DSS_HWA_PROC_CFG_ADDR            (0x401E0000)
#define CSL_FW_DSS_HWA_PROC_NUM_REGION          (8)
#define CSL_FW_DSS_HWA_PROC_NUM_PROTECTED       (2)
#define CSL_FW_DSS_HWA_PROC_START_ADDR0         (0x48000000)
#define CSL_FW_DSS_HWA_PROC_REGION_SIZE0        (4*1024)
#define CSL_FW_DSS_HWA_PROC_START_ADDR1         (0x48020000)
#define CSL_FW_DSS_HWA_PROC_REGION_SIZE1        (4*1024)


/***********************************************************************
 * FW DSS_MBOX
 ***********************************************************************/
#define CSL_FW_DSS_MBOX_ID                      (15U)
#define CSL_FW_DSS_MBOX_TYPE                    (CSL_FW)
#define CSL_FW_DSS_MBOX_CFG_ADDR                (0x40200000)
#define CSL_FW_DSS_MBOX_NUM_REGION              (8)
#define CSL_FW_DSS_MBOX_NUM_PROTECTED           (1)
#define CSL_FW_DSS_MBOX_START_ADDR0             (0x83100000)
#define CSL_FW_DSS_MBOX_REGION_SIZE0            (4*1024)

/***********************************************************************
 * FW HSM
 ***********************************************************************/
#define CSL_FW_HSM_ID                           (16U)
#define CSL_FW_HSM_TYPE                         (CSL_FW)
#define CSL_FW_HSM_CFG_ADDR                     (0x40220000)
#define CSL_FW_HSM_NUM_REGION                   (8)
#define CSL_FW_HSM_NUM_PROTECTED                (2)
#define CSL_FW_HSM_START_ADDR0                  (0x20008000)
#define CSL_FW_HSM_REGION_SIZE0                 (224*1024)
#define CSL_FW_HSM_START_ADDR1                  (0x40000000)
#define CSL_FW_HSM_REGION_SIZE1                 (96*1024*1024)

#ifdef __cplusplus
}
#endif

#endif  /* CSLR_FIREWALL_DEFINES_H_ */
