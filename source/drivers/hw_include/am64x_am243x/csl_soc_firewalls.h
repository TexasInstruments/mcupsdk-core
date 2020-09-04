/********************************************************************
*
* SOC FIREWALL PROPERTIES. header file
*
* Copyright (C) 2015-2019 Texas Instruments Incorporated.
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
#ifndef CSLR_SOC_FW_H_
#define CSLR_SOC_FW_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* Auto-generated CSL definitions for SoC Firewalls:
*/

#define CSL_FW_SECURITY                                                                            (0U)
#define CSL_FW_CHANNEL                                                                             (1U)
#define CSL_FW_MST                                                                                 (2U)

/* Standard Security Master Firewall Definitions */

/* Standard Security Slave Firewall Definitions */

/* Properties of firewall at slave: DDR16SS0_SDRAM */
#define CSL_STD_FW_DDR16SS0_SDRAM_ID                                                               (1U)
#define CSL_STD_FW_DDR16SS0_SDRAM_TYPE                                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_DDR16SS0_SDRAM_MMR_BASE                                                         (0x00000045000400U)
#define CSL_STD_FW_DDR16SS0_SDRAM_NUM_REGIONS                                                      (8U)
#define CSL_STD_FW_DDR16SS0_SDRAM_NUM_PRIV_IDS_PER_REGION                                          (3U)
#define CSL_STD_FW_DDR16SS0_SDRAM_DRAM_START                                                       (0x00000080000000U)
#define CSL_STD_FW_DDR16SS0_SDRAM_DRAM_END                                                         (0x000000ffffffffU)

/* Properties of firewall at slave: MSRAM_256K0_RAM */
#define CSL_STD_FW_MSRAM_256K0_RAM_ID                                                              (14U)
#define CSL_STD_FW_MSRAM_256K0_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K0_RAM_MMR_BASE                                                        (0x00000045003800U)
#define CSL_STD_FW_MSRAM_256K0_RAM_NUM_REGIONS                                                     (4U)
#define CSL_STD_FW_MSRAM_256K0_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K0_RAM_RAM_START                                                       (0x00000070000000U)
#define CSL_STD_FW_MSRAM_256K0_RAM_RAM_END                                                         (0x0000007003ffffU)

/* Properties of firewall at slave: MSRAM_256K1_RAM */
#define CSL_STD_FW_MSRAM_256K1_RAM_ID                                                              (15U)
#define CSL_STD_FW_MSRAM_256K1_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K1_RAM_MMR_BASE                                                        (0x00000045003c00U)
#define CSL_STD_FW_MSRAM_256K1_RAM_NUM_REGIONS                                                     (4U)
#define CSL_STD_FW_MSRAM_256K1_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K1_RAM_RAM_START                                                       (0x00000070040000U)
#define CSL_STD_FW_MSRAM_256K1_RAM_RAM_END                                                         (0x0000007007ffffU)

/* Properties of firewall at slave: MSRAM_256K2_RAM */
#define CSL_STD_FW_MSRAM_256K2_RAM_ID                                                              (16U)
#define CSL_STD_FW_MSRAM_256K2_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K2_RAM_MMR_BASE                                                        (0x00000045004000U)
#define CSL_STD_FW_MSRAM_256K2_RAM_NUM_REGIONS                                                     (4U)
#define CSL_STD_FW_MSRAM_256K2_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K2_RAM_RAM_START                                                       (0x00000070080000U)
#define CSL_STD_FW_MSRAM_256K2_RAM_RAM_END                                                         (0x000000700bffffU)

/* Properties of firewall at slave: MSRAM_256K5_RAM */
#define CSL_STD_FW_MSRAM_256K5_RAM_ID                                                              (17U)
#define CSL_STD_FW_MSRAM_256K5_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K5_RAM_MMR_BASE                                                        (0x00000045004400U)
#define CSL_STD_FW_MSRAM_256K5_RAM_NUM_REGIONS                                                     (4U)
#define CSL_STD_FW_MSRAM_256K5_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K5_RAM_RAM_START                                                       (0x00000070140000U)
#define CSL_STD_FW_MSRAM_256K5_RAM_RAM_END                                                         (0x0000007017ffffU)

/* Properties of firewall at slave: MSRAM_256K4_RAM */
#define CSL_STD_FW_MSRAM_256K4_RAM_ID                                                              (18U)
#define CSL_STD_FW_MSRAM_256K4_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K4_RAM_MMR_BASE                                                        (0x00000045004800U)
#define CSL_STD_FW_MSRAM_256K4_RAM_NUM_REGIONS                                                     (4U)
#define CSL_STD_FW_MSRAM_256K4_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K4_RAM_RAM_START                                                       (0x00000070100000U)
#define CSL_STD_FW_MSRAM_256K4_RAM_RAM_END                                                         (0x0000007013ffffU)

/* Properties of firewall at slave: MSRAM_256K3_RAM */
#define CSL_STD_FW_MSRAM_256K3_RAM_ID                                                              (19U)
#define CSL_STD_FW_MSRAM_256K3_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K3_RAM_MMR_BASE                                                        (0x00000045004c00U)
#define CSL_STD_FW_MSRAM_256K3_RAM_NUM_REGIONS                                                     (4U)
#define CSL_STD_FW_MSRAM_256K3_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K3_RAM_RAM_START                                                       (0x000000700c0000U)
#define CSL_STD_FW_MSRAM_256K3_RAM_RAM_END                                                         (0x000000700fffffU)

/* Properties of firewall at slave: DMASS0_SEC_PROXY_SRC_TARGET_DATA */
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_ID                                             (20U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_MMR_BASE                                       (0x00000045005000U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_NUM_REGIONS                                    (8U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_SEC_PROXY_SRC_TARGET_DATA_START                (0x0000004d000000U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SRC_TARGET_DATA_SEC_PROXY_SRC_TARGET_DATA_END                  (0x0000004d07ffffU)

/* Properties of firewall at slave: DMASS0_RINGACC_SRC_FIFOS */
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_ID                                                     (20U)
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_MMR_BASE                                               (0x00000045005000U)
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_NUM_REGIONS                                            (8U)
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_RINGACC_SRC_FIFOS_START                                (0x0000004e000000U)
#define CSL_STD_FW_DMASS0_RINGACC_SRC_FIFOS_RINGACC_SRC_FIFOS_END                                  (0x0000004e3fffffU)

/* Properties of firewall at slave: GICSS0_GIC_TRANSLATER_REGS */
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_ID                                                   (22U)
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_TYPE                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_MMR_BASE                                             (0x00000045005800U)
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_NUM_REGIONS                                          (16U)
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_NUM_PRIV_IDS_PER_REGION                              (3U)
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_GIC_TRANSLATER_START                                 (0x00000001000000U)
#define CSL_STD_FW_GICSS0_GIC_TRANSLATER_REGS_GIC_TRANSLATER_END                                   (0x000000013fffffU)

/* Properties of firewall at slave: GICSS0_GIC_REGS */
#define CSL_STD_FW_GICSS0_GIC_REGS_ID                                                              (22U)
#define CSL_STD_FW_GICSS0_GIC_REGS_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_GICSS0_GIC_REGS_MMR_BASE                                                        (0x00000045005800U)
#define CSL_STD_FW_GICSS0_GIC_REGS_NUM_REGIONS                                                     (16U)
#define CSL_STD_FW_GICSS0_GIC_REGS_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_GICSS0_GIC_REGS_GIC_START                                                       (0x00000001800000U)
#define CSL_STD_FW_GICSS0_GIC_REGS_GIC_END                                                         (0x000000018fffffU)

/* Properties of firewall at slave: PRU_ICSSG0_DRAM0_SLV_RAM */
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_ID                                                     (22U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_MMR_BASE                                               (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_NUM_REGIONS                                            (16U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_DRAM0_SLV_RAM_START                                    (0x00000030000000U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM0_SLV_RAM_DRAM0_SLV_RAM_END                                      (0x00000030001fffU)

/* Properties of firewall at slave: PRU_ICSSG0_DRAM1_SLV_RAM */
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_ID                                                     (22U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_MMR_BASE                                               (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_NUM_REGIONS                                            (16U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_DRAM1_SLV_RAM_START                                    (0x00000030002000U)
#define CSL_STD_FW_PRU_ICSSG0_DRAM1_SLV_RAM_DRAM1_SLV_RAM_END                                      (0x00000030003fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_ID                                        (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_TYPE                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_MMR_BASE                                  (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_NUM_REGIONS                               (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                   (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_PR1_RTU0_PR1_RTU0_IRAM_RAM_START          (0x00000030004000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_PR1_RTU0_PR1_RTU0_IRAM_RAM_END            (0x00000030005fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_ID                                        (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_TYPE                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_MMR_BASE                                  (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_NUM_REGIONS                               (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                   (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_PR1_RTU1_PR1_RTU1_IRAM_RAM_START          (0x00000030006000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_PR1_RTU1_PR1_RTU1_IRAM_RAM_END            (0x00000030007fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_RAT_SLICE0_CFG_START                         (0x00000030008000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE0_CFG_MMRS_RAT_SLICE0_CFG_END                           (0x00000030008fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_RAT_SLICE1_CFG_START                         (0x00000030009000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RAT_SLICE1_CFG_MMRS_RAT_SLICE1_CFG_END                           (0x00000030009fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_ID                                             (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_MMR_BASE                                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_NUM_REGIONS                                    (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_PR1_PDSP_TX0_IRAM_RAM_START                    (0x0000003000a000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_PR1_PDSP_TX0_IRAM_RAM_END                      (0x0000003000bfffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_ID                                             (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_MMR_BASE                                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_NUM_REGIONS                                    (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_PR1_PDSP_TX1_IRAM_RAM_START                    (0x0000003000c000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_PR1_PDSP_TX1_IRAM_RAM_END                      (0x0000003000dfffU)

/* Properties of firewall at slave: PRU_ICSSG0_RAM_SLV_RAM */
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_ID                                                       (22U)
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_TYPE                                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_MMR_BASE                                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_NUM_REGIONS                                              (16U)
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_NUM_PRIV_IDS_PER_REGION                                  (3U)
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_RAM_SLV_RAM_START                                        (0x00000030010000U)
#define CSL_STD_FW_PRU_ICSSG0_RAM_SLV_RAM_RAM_SLV_RAM_END                                          (0x0000003001ffffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_PR1_ICSS_INTC_INTC_SLV_START             (0x00000030020000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_REGS_PR1_ICSS_INTC_INTC_SLV_END               (0x00000030021fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP0_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_ID                                               (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_MMR_BASE                                         (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_NUM_REGIONS                                      (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_PR1_PDSP0_IRAM_START                             (0x00000030022000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_REGS_PR1_PDSP0_IRAM_END                               (0x000000300220ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_ID                                              (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_MMR_BASE                                        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_NUM_REGIONS                                     (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_PR1_PDSP0_IRAM_DEBUG_START                      (0x00000030022400U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_DEBUG_PR1_PDSP0_IRAM_DEBUG_END                        (0x000000300224ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_PR1_RTU0_PR1_RTU0_IRAM_START             (0x00000030023000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_REGS_PR1_RTU0_PR1_RTU0_IRAM_END               (0x000000300230ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_ID                                      (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_MMR_BASE                                (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_NUM_REGIONS                             (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_START      (0x00000030023400U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_END        (0x000000300234ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_PR1_RTU1_PR1_RTU1_IRAM_START             (0x00000030023800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_REGS_PR1_RTU1_PR1_RTU1_IRAM_END               (0x000000300238ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_ID                                      (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_MMR_BASE                                (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_NUM_REGIONS                             (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_START      (0x00000030023c00U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_END        (0x00000030023cffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP1_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_ID                                               (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_MMR_BASE                                         (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_NUM_REGIONS                                      (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_PR1_PDSP1_IRAM_START                             (0x00000030024000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_REGS_PR1_PDSP1_IRAM_END                               (0x000000300240ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_ID                                              (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_MMR_BASE                                        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_NUM_REGIONS                                     (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_PR1_PDSP1_IRAM_DEBUG_START                      (0x00000030024400U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_DEBUG_PR1_PDSP1_IRAM_DEBUG_END                        (0x000000300244ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PROTECT_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_ID                                              (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_MMR_BASE                                        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_NUM_REGIONS                                     (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_PR1_PROT_SLV_START                              (0x00000030024c00U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PROTECT_SLV_REGS_PR1_PROT_SLV_END                                (0x00000030024cffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_ID                                            (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_MMR_BASE                                      (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_NUM_REGIONS                                   (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_PR1_PDSP_TX0_IRAM_START                       (0x00000030025000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_REGS_PR1_PDSP_TX0_IRAM_END                         (0x000000300250ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_PR1_PDSP_TX0_IRAM_DEBUG_START                (0x00000030025400U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_DEBUG_PR1_PDSP_TX0_IRAM_DEBUG_END                  (0x000000300254ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_ID                                            (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_MMR_BASE                                      (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_NUM_REGIONS                                   (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_PR1_PDSP_TX1_IRAM_START                       (0x00000030025800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_REGS_PR1_PDSP_TX1_IRAM_END                         (0x000000300258ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_PR1_PDSP_TX1_IRAM_DEBUG_START                (0x00000030025c00U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_DEBUG_PR1_PDSP_TX1_IRAM_DEBUG_END                  (0x00000030025cffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_CFG_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_ID                                                  (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_TYPE                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_MMR_BASE                                            (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_NUM_REGIONS                                         (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_NUM_PRIV_IDS_PER_REGION                             (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_PR1_CFG_SLV_START                                   (0x00000030026000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_CFG_SLV_REGS_PR1_CFG_SLV_END                                     (0x000000300261ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT */
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_ID                                         (22U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_MMR_BASE                                   (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_NUM_REGIONS                                (16U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_PA_STAT_WRAP_PA_SLV_QSTAT_START            (0x00000030027000U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_QSTAT_PA_STAT_WRAP_PA_SLV_QSTAT_END              (0x000000300277ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_PR1_ICSS_UART_UART_SLV_START             (0x00000030028000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_REGS_PR1_ICSS_UART_UART_SLV_END               (0x0000003002803fU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_START (0x0000003002a000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_END (0x0000003002a0ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_START (0x0000003002a100U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_END (0x0000003002a1ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_START (0x0000003002a200U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_END (0x0000003002a2ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_START (0x0000003002a300U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_END (0x0000003002a3ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_ID              (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_TYPE            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_MMR_BASE        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_NUM_REGIONS     (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_START (0x0000003002a400U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_END (0x0000003002a4ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_ID              (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_TYPE            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_MMR_BASE        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_NUM_REGIONS     (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_START (0x0000003002a500U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_END (0x0000003002a5ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT */
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_ID                                         (22U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_MMR_BASE                                   (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_NUM_REGIONS                                (16U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_PA_STAT_WRAP_PA_SLV_CSTAT_START            (0x0000003002c000U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_CSTAT_PA_STAT_WRAP_PA_SLV_CSTAT_END              (0x0000003002c7ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_IEP0_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_ID                                                 (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_MMR_BASE                                           (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_NUM_REGIONS                                        (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_IEP0_START                                         (0x0000003002e000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP0_SLV_REGS_IEP0_END                                           (0x0000003002efffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_IEP1_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_ID                                                 (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_MMR_BASE                                           (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_NUM_REGIONS                                        (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_IEP1_START                                         (0x0000003002f000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_IEP1_SLV_REGS_IEP1_END                                           (0x0000003002ffffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_ID                                      (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_MMR_BASE                                (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_NUM_REGIONS                             (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_PR1_ICSS_ECAP0_ECAP_SLV_START           (0x00000030030000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_REGS_PR1_ICSS_ECAP0_ECAP_SLV_END             (0x000000300300ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_ID                                    (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_TYPE                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_MMR_BASE                              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_NUM_REGIONS                           (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_NUM_PRIV_IDS_PER_REGION               (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_PR1_MII_RT_PR1_MII_RT_CFG_START       (0x00000030032000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_REGS_PR1_MII_RT_PR1_MII_RT_CFG_END         (0x000000300320ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_ID                             (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_TYPE                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_MMR_BASE                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_NUM_REGIONS                    (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_NUM_PRIV_IDS_PER_REGION        (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_START (0x00000030032100U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_END (0x000000300321ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_ID                             (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_TYPE                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_MMR_BASE                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_NUM_REGIONS                    (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_NUM_PRIV_IDS_PER_REGION        (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_START (0x00000030032200U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_END (0x000000300322ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_PR1_MDIO_V1P7_MDIO_START                     (0x00000030032400U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_REGS_PR1_MDIO_V1P7_MDIO_END                       (0x000000300324ffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G */
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ID                                (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_TYPE                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MMR_BASE                          (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_NUM_REGIONS                       (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_NUM_PRIV_IDS_PER_REGION           (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_START (0x00000030033000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_END (0x00000030033fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP0_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_ID                                                (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_PR1_PDSP0_IRAM_RAM_START                          (0x00000030034000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_PR1_PDSP0_IRAM_RAM_END                            (0x00000030037fffU)

/* Properties of firewall at slave: PRU_ICSSG0_PR1_PDSP1_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_ID                                                (22U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_PR1_PDSP1_IRAM_RAM_START                          (0x00000030038000U)
#define CSL_STD_FW_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_PR1_PDSP1_IRAM_RAM_END                            (0x0000003003bfffU)

/* Properties of firewall at slave: PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_ID                                          (22U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_TYPE                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_MMR_BASE                                    (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_NUM_REGIONS                                 (16U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_NUM_PRIV_IDS_PER_REGION                     (3U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_PA_STAT_WRAP_PA_SLV_START                   (0x0000003003c000U)
#define CSL_STD_FW_PRU_ICSSG0_PA_STAT_WRAP_PA_SLV_REGS_PA_STAT_WRAP_PA_SLV_END                     (0x0000003003c0ffU)

/* Properties of firewall at slave: PRU_ICSSG1_DRAM0_SLV_RAM */
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_ID                                                     (22U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_MMR_BASE                                               (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_NUM_REGIONS                                            (16U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_DRAM0_SLV_RAM_START                                    (0x00000030080000U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM0_SLV_RAM_DRAM0_SLV_RAM_END                                      (0x00000030081fffU)

/* Properties of firewall at slave: PRU_ICSSG1_DRAM1_SLV_RAM */
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_ID                                                     (22U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_MMR_BASE                                               (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_NUM_REGIONS                                            (16U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_DRAM1_SLV_RAM_START                                    (0x00000030082000U)
#define CSL_STD_FW_PRU_ICSSG1_DRAM1_SLV_RAM_DRAM1_SLV_RAM_END                                      (0x00000030083fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_ID                                        (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_TYPE                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_MMR_BASE                                  (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_NUM_REGIONS                               (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                   (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_PR1_RTU0_PR1_RTU0_IRAM_RAM_START          (0x00000030084000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_PR1_RTU0_PR1_RTU0_IRAM_RAM_END            (0x00000030085fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_ID                                        (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_TYPE                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_MMR_BASE                                  (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_NUM_REGIONS                               (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                   (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_PR1_RTU1_PR1_RTU1_IRAM_RAM_START          (0x00000030086000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_PR1_RTU1_PR1_RTU1_IRAM_RAM_END            (0x00000030087fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_RAT_SLICE0_CFG_START                         (0x00000030088000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE0_CFG_MMRS_RAT_SLICE0_CFG_END                           (0x00000030088fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_RAT_SLICE1_CFG_START                         (0x00000030089000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RAT_SLICE1_CFG_MMRS_RAT_SLICE1_CFG_END                           (0x00000030089fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_ID                                             (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_MMR_BASE                                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_NUM_REGIONS                                    (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_PR1_PDSP_TX0_IRAM_RAM_START                    (0x0000003008a000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_PR1_PDSP_TX0_IRAM_RAM_END                      (0x0000003008bfffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_ID                                             (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_MMR_BASE                                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_NUM_REGIONS                                    (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_PR1_PDSP_TX1_IRAM_RAM_START                    (0x0000003008c000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_PR1_PDSP_TX1_IRAM_RAM_END                      (0x0000003008dfffU)

/* Properties of firewall at slave: PRU_ICSSG1_RAM_SLV_RAM */
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_ID                                                       (22U)
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_TYPE                                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_MMR_BASE                                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_NUM_REGIONS                                              (16U)
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_NUM_PRIV_IDS_PER_REGION                                  (3U)
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_RAM_SLV_RAM_START                                        (0x00000030090000U)
#define CSL_STD_FW_PRU_ICSSG1_RAM_SLV_RAM_RAM_SLV_RAM_END                                          (0x0000003009ffffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_PR1_ICSS_INTC_INTC_SLV_START             (0x000000300a0000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_REGS_PR1_ICSS_INTC_INTC_SLV_END               (0x000000300a1fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP0_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_ID                                               (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_MMR_BASE                                         (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_NUM_REGIONS                                      (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_PR1_PDSP0_IRAM_START                             (0x000000300a2000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_REGS_PR1_PDSP0_IRAM_END                               (0x000000300a20ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_ID                                              (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_MMR_BASE                                        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_NUM_REGIONS                                     (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_PR1_PDSP0_IRAM_DEBUG_START                      (0x000000300a2400U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_DEBUG_PR1_PDSP0_IRAM_DEBUG_END                        (0x000000300a24ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_PR1_RTU0_PR1_RTU0_IRAM_START             (0x000000300a3000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_REGS_PR1_RTU0_PR1_RTU0_IRAM_END               (0x000000300a30ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_ID                                      (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_MMR_BASE                                (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_NUM_REGIONS                             (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_START      (0x000000300a3400U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_PR1_RTU0_PR1_RTU0_IRAM_DEBUG_END        (0x000000300a34ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_PR1_RTU1_PR1_RTU1_IRAM_START             (0x000000300a3800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_REGS_PR1_RTU1_PR1_RTU1_IRAM_END               (0x000000300a38ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_ID                                      (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_MMR_BASE                                (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_NUM_REGIONS                             (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_START      (0x000000300a3c00U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_PR1_RTU1_PR1_RTU1_IRAM_DEBUG_END        (0x000000300a3cffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP1_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_ID                                               (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_MMR_BASE                                         (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_NUM_REGIONS                                      (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_PR1_PDSP1_IRAM_START                             (0x000000300a4000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_REGS_PR1_PDSP1_IRAM_END                               (0x000000300a40ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_ID                                              (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_MMR_BASE                                        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_NUM_REGIONS                                     (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_PR1_PDSP1_IRAM_DEBUG_START                      (0x000000300a4400U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_DEBUG_PR1_PDSP1_IRAM_DEBUG_END                        (0x000000300a44ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PROTECT_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_ID                                              (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_MMR_BASE                                        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_NUM_REGIONS                                     (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_PR1_PROT_SLV_START                              (0x000000300a4c00U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PROTECT_SLV_REGS_PR1_PROT_SLV_END                                (0x000000300a4cffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_ID                                            (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_MMR_BASE                                      (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_NUM_REGIONS                                   (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_PR1_PDSP_TX0_IRAM_START                       (0x000000300a5000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_REGS_PR1_PDSP_TX0_IRAM_END                         (0x000000300a50ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_PR1_PDSP_TX0_IRAM_DEBUG_START                (0x000000300a5400U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_DEBUG_PR1_PDSP_TX0_IRAM_DEBUG_END                  (0x000000300a54ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_ID                                            (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_MMR_BASE                                      (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_NUM_REGIONS                                   (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_PR1_PDSP_TX1_IRAM_START                       (0x000000300a5800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_REGS_PR1_PDSP_TX1_IRAM_END                         (0x000000300a58ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_PR1_PDSP_TX1_IRAM_DEBUG_START                (0x000000300a5c00U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_DEBUG_PR1_PDSP_TX1_IRAM_DEBUG_END                  (0x000000300a5cffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_CFG_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_ID                                                  (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_TYPE                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_MMR_BASE                                            (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_NUM_REGIONS                                         (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_NUM_PRIV_IDS_PER_REGION                             (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_PR1_CFG_SLV_START                                   (0x000000300a6000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_CFG_SLV_REGS_PR1_CFG_SLV_END                                     (0x000000300a61ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT */
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_ID                                         (22U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_MMR_BASE                                   (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_NUM_REGIONS                                (16U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_PA_STAT_WRAP_PA_SLV_QSTAT_START            (0x000000300a7000U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_QSTAT_PA_STAT_WRAP_PA_SLV_QSTAT_END              (0x000000300a77ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_ID                                       (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_MMR_BASE                                 (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_NUM_REGIONS                              (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_PR1_ICSS_UART_UART_SLV_START             (0x000000300a8000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_REGS_PR1_ICSS_UART_UART_SLV_END               (0x000000300a803fU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_START (0x000000300aa000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_END (0x000000300aa0ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_START (0x000000300aa100U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_END (0x000000300aa1ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_START (0x000000300aa200U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_END (0x000000300aa2ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_ID                    (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_TYPE                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_MMR_BASE              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_NUM_REGIONS           (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_START (0x000000300aa300U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_END (0x000000300aa3ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_ID              (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_TYPE            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_MMR_BASE        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_NUM_REGIONS     (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_START (0x000000300aa400U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_END (0x000000300aa4ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_ID              (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_TYPE            (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_MMR_BASE        (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_NUM_REGIONS     (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_START (0x000000300aa500U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_END (0x000000300aa5ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT */
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_ID                                         (22U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_MMR_BASE                                   (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_NUM_REGIONS                                (16U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_PA_STAT_WRAP_PA_SLV_CSTAT_START            (0x000000300ac000U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_CSTAT_PA_STAT_WRAP_PA_SLV_CSTAT_END              (0x000000300ac7ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_IEP0_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_ID                                                 (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_MMR_BASE                                           (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_NUM_REGIONS                                        (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_IEP0_START                                         (0x000000300ae000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP0_SLV_REGS_IEP0_END                                           (0x000000300aefffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_IEP1_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_ID                                                 (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_MMR_BASE                                           (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_NUM_REGIONS                                        (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_IEP1_START                                         (0x000000300af000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_IEP1_SLV_REGS_IEP1_END                                           (0x000000300affffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_ID                                      (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_MMR_BASE                                (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_NUM_REGIONS                             (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_PR1_ICSS_ECAP0_ECAP_SLV_START           (0x000000300b0000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_REGS_PR1_ICSS_ECAP0_ECAP_SLV_END             (0x000000300b00ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_ID                                    (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_TYPE                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_MMR_BASE                              (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_NUM_REGIONS                           (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_NUM_PRIV_IDS_PER_REGION               (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_PR1_MII_RT_PR1_MII_RT_CFG_START       (0x000000300b2000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_REGS_PR1_MII_RT_PR1_MII_RT_CFG_END         (0x000000300b20ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_ID                             (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_TYPE                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_MMR_BASE                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_NUM_REGIONS                    (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_NUM_PRIV_IDS_PER_REGION        (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_START (0x000000300b2100U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_REGS_PR1_MII_RT_PR1_SGMII0_CFG_SGMII0_END (0x000000300b21ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_ID                             (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_TYPE                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_MMR_BASE                       (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_NUM_REGIONS                    (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_NUM_PRIV_IDS_PER_REGION        (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_START (0x000000300b2200U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_REGS_PR1_MII_RT_PR1_SGMII1_CFG_SGMII1_END (0x000000300b22ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_ID                                           (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_TYPE                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_MMR_BASE                                     (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_NUM_REGIONS                                  (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_NUM_PRIV_IDS_PER_REGION                      (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_PR1_MDIO_V1P7_MDIO_START                     (0x000000300b2400U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_REGS_PR1_MDIO_V1P7_MDIO_END                       (0x000000300b24ffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G */
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ID                                (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_TYPE                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_MMR_BASE                          (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_NUM_REGIONS                       (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_NUM_PRIV_IDS_PER_REGION           (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_START (0x000000300b3000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_END (0x000000300b3fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP0_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_ID                                                (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_PR1_PDSP0_IRAM_RAM_START                          (0x000000300b4000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_PR1_PDSP0_IRAM_RAM_END                            (0x000000300b7fffU)

/* Properties of firewall at slave: PRU_ICSSG1_PR1_PDSP1_IRAM_RAM */
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_ID                                                (22U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_PR1_PDSP1_IRAM_RAM_START                          (0x000000300b8000U)
#define CSL_STD_FW_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_PR1_PDSP1_IRAM_RAM_END                            (0x000000300bbfffU)

/* Properties of firewall at slave: PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS */
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_ID                                          (22U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_TYPE                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_MMR_BASE                                    (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_NUM_REGIONS                                 (16U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_NUM_PRIV_IDS_PER_REGION                     (3U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_PA_STAT_WRAP_PA_SLV_START                   (0x000000300bc000U)
#define CSL_STD_FW_PRU_ICSSG1_PA_STAT_WRAP_PA_SLV_REGS_PA_STAT_WRAP_PA_SLV_END                     (0x000000300bc0ffU)

/* Properties of firewall at slave: TIMERMGR0_TIMERS */
#define CSL_STD_FW_TIMERMGR0_TIMERS_ID                                                             (22U)
#define CSL_STD_FW_TIMERMGR0_TIMERS_TYPE                                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_TIMERMGR0_TIMERS_MMR_BASE                                                       (0x00000045005800U)
#define CSL_STD_FW_TIMERMGR0_TIMERS_NUM_REGIONS                                                    (16U)
#define CSL_STD_FW_TIMERMGR0_TIMERS_NUM_PRIV_IDS_PER_REGION                                        (3U)
#define CSL_STD_FW_TIMERMGR0_TIMERS_TIMERS_START                                                   (0x00000037000000U)
#define CSL_STD_FW_TIMERMGR0_TIMERS_TIMERS_END                                                     (0x0000003703ffffU)

/* Properties of firewall at slave: CPTS0_CPTS_VBUSP */
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_ID                                                             (22U)
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_TYPE                                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_MMR_BASE                                                       (0x00000045005800U)
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_NUM_REGIONS                                                    (16U)
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_NUM_PRIV_IDS_PER_REGION                                        (3U)
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_START                                                          (0x00000039000000U)
#define CSL_STD_FW_CPTS0_CPTS_VBUSP_END                                                            (0x000000390003ffU)

/* Properties of firewall at slave: CBASS0_ERR_REGS */
#define CSL_STD_FW_CBASS0_ERR_REGS_ID                                                              (22U)
#define CSL_STD_FW_CBASS0_ERR_REGS_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_CBASS0_ERR_REGS_MMR_BASE                                                        (0x00000045005800U)
#define CSL_STD_FW_CBASS0_ERR_REGS_NUM_REGIONS                                                     (16U)
#define CSL_STD_FW_CBASS0_ERR_REGS_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_CBASS0_ERR_REGS_ERR_START                                                       (0x0000003a000000U)
#define CSL_STD_FW_CBASS0_ERR_REGS_ERR_END                                                         (0x0000003a0003ffU)

/* Properties of firewall at slave: R5FSS0_REGS */
#define CSL_STD_FW_R5FSS0_REGS_ID                                                                  (22U)
#define CSL_STD_FW_R5FSS0_REGS_TYPE                                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_R5FSS0_REGS_MMR_BASE                                                            (0x00000045005800U)
#define CSL_STD_FW_R5FSS0_REGS_NUM_REGIONS                                                         (16U)
#define CSL_STD_FW_R5FSS0_REGS_NUM_PRIV_IDS_PER_REGION                                             (3U)
#define CSL_STD_FW_R5FSS0_REGS_ECC_AGGR_START                                                      (0x0000003c010000U)
#define CSL_STD_FW_R5FSS0_REGS_ECC_AGGR_END                                                        (0x0000003c0103ffU)

/* Properties of firewall at slave: R5FSS0_EVNT_BUS_VBUSP_MMRS */
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_ID                                                   (22U)
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_TYPE                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_MMR_BASE                                             (0x00000045005800U)
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_NUM_REGIONS                                          (16U)
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_NUM_PRIV_IDS_PER_REGION                              (3U)
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_EVNT_BUS_VBUSP_MMRS_START                            (0x0000003c018000U)
#define CSL_STD_FW_R5FSS0_EVNT_BUS_VBUSP_MMRS_EVNT_BUS_VBUSP_MMRS_END                              (0x0000003c0180ffU)

/* Properties of firewall at slave: R5FSS1_REGS */
#define CSL_STD_FW_R5FSS1_REGS_ID                                                                  (22U)
#define CSL_STD_FW_R5FSS1_REGS_TYPE                                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_R5FSS1_REGS_MMR_BASE                                                            (0x00000045005800U)
#define CSL_STD_FW_R5FSS1_REGS_NUM_REGIONS                                                         (16U)
#define CSL_STD_FW_R5FSS1_REGS_NUM_PRIV_IDS_PER_REGION                                             (3U)
#define CSL_STD_FW_R5FSS1_REGS_ECC_AGGR_START                                                      (0x0000003c030000U)
#define CSL_STD_FW_R5FSS1_REGS_ECC_AGGR_END                                                        (0x0000003c0303ffU)

/* Properties of firewall at slave: R5FSS1_EVNT_BUS_VBUSP_MMRS */
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_ID                                                   (22U)
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_TYPE                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_MMR_BASE                                             (0x00000045005800U)
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_NUM_REGIONS                                          (16U)
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_NUM_PRIV_IDS_PER_REGION                              (3U)
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_EVNT_BUS_VBUSP_MMRS_START                            (0x0000003c038000U)
#define CSL_STD_FW_R5FSS1_EVNT_BUS_VBUSP_MMRS_EVNT_BUS_VBUSP_MMRS_END                              (0x0000003c0380ffU)

/* Properties of firewall at slave: TIMERMGR0_CONFIG */
#define CSL_STD_FW_TIMERMGR0_CONFIG_ID                                                             (22U)
#define CSL_STD_FW_TIMERMGR0_CONFIG_TYPE                                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_TIMERMGR0_CONFIG_MMR_BASE                                                       (0x00000045005800U)
#define CSL_STD_FW_TIMERMGR0_CONFIG_NUM_REGIONS                                                    (16U)
#define CSL_STD_FW_TIMERMGR0_CONFIG_NUM_PRIV_IDS_PER_REGION                                        (3U)
#define CSL_STD_FW_TIMERMGR0_CONFIG_CONFIG_START                                                   (0x0000003cd00000U)
#define CSL_STD_FW_TIMERMGR0_CONFIG_CONFIG_END                                                     (0x0000003cd001ffU)

/* Properties of firewall at slave: MSRAM_256K0_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f001000U)
#define CSL_STD_FW_MSRAM_256K0_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0013ffU)

/* Properties of firewall at slave: MSRAM_256K1_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f002000U)
#define CSL_STD_FW_MSRAM_256K1_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0023ffU)

/* Properties of firewall at slave: MSRAM_256K2_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f003000U)
#define CSL_STD_FW_MSRAM_256K2_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0033ffU)

/* Properties of firewall at slave: GICSS0_REGS */
#define CSL_STD_FW_GICSS0_REGS_ID                                                                  (22U)
#define CSL_STD_FW_GICSS0_REGS_TYPE                                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_GICSS0_REGS_MMR_BASE                                                            (0x00000045005800U)
#define CSL_STD_FW_GICSS0_REGS_NUM_REGIONS                                                         (16U)
#define CSL_STD_FW_GICSS0_REGS_NUM_PRIV_IDS_PER_REGION                                             (3U)
#define CSL_STD_FW_GICSS0_REGS_REGS_START                                                          (0x0000003f004000U)
#define CSL_STD_FW_GICSS0_REGS_REGS_END                                                            (0x0000003f0043ffU)

/* Properties of firewall at slave: DMASS0_ECCAGGR_REGS */
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_ECCAGGR_START                                               (0x0000003f005000U)
#define CSL_STD_FW_DMASS0_ECCAGGR_REGS_ECCAGGR_END                                                 (0x0000003f0053ffU)

/* Properties of firewall at slave: MSRAM_256K5_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f006000U)
#define CSL_STD_FW_MSRAM_256K5_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0063ffU)

/* Properties of firewall at slave: MSRAM_256K4_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f007000U)
#define CSL_STD_FW_MSRAM_256K4_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0073ffU)

/* Properties of firewall at slave: MSRAM_256K3_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f008000U)
#define CSL_STD_FW_MSRAM_256K3_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0083ffU)

/* Properties of firewall at slave: PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS */
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_ID                                            (22U)
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_MMR_BASE                                      (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_NUM_REGIONS                                   (16U)
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_ECC_AGGR_START                                (0x0000003f00a000U)
#define CSL_STD_FW_PRU_ICSSG0_BORG_ECC_AGGR_CFG_REGS_ECC_AGGR_END                                  (0x0000003f00a3ffU)

/* Properties of firewall at slave: PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS */
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_ID                                            (22U)
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_MMR_BASE                                      (0x00000045005800U)
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_NUM_REGIONS                                   (16U)
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_ECC_AGGR_START                                (0x0000003f00b000U)
#define CSL_STD_FW_PRU_ICSSG1_BORG_ECC_AGGR_CFG_REGS_ECC_AGGR_END                                  (0x0000003f00b3ffU)

/* Properties of firewall at slave: R5FSS0_CPU0_ECC_AGGR_CFG_REGS */
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_ID                                                (22U)
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_CORE0_ECC_AGGR_START                              (0x0000003f00d000U)
#define CSL_STD_FW_R5FSS0_CPU0_ECC_AGGR_CFG_REGS_CORE0_ECC_AGGR_END                                (0x0000003f00d3ffU)

/* Properties of firewall at slave: R5FSS1_CPU0_ECC_AGGR_CFG_REGS */
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_ID                                                (22U)
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_CORE0_ECC_AGGR_START                              (0x0000003f00e000U)
#define CSL_STD_FW_R5FSS1_CPU0_ECC_AGGR_CFG_REGS_CORE0_ECC_AGGR_END                                (0x0000003f00e3ffU)

/* Properties of firewall at slave: ECC_AGGR0_REGS */
#define CSL_STD_FW_ECC_AGGR0_REGS_ID                                                               (22U)
#define CSL_STD_FW_ECC_AGGR0_REGS_TYPE                                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_ECC_AGGR0_REGS_MMR_BASE                                                         (0x00000045005800U)
#define CSL_STD_FW_ECC_AGGR0_REGS_NUM_REGIONS                                                      (16U)
#define CSL_STD_FW_ECC_AGGR0_REGS_NUM_PRIV_IDS_PER_REGION                                          (3U)
#define CSL_STD_FW_ECC_AGGR0_REGS_ECC_AGGR_START                                                   (0x0000003f00f000U)
#define CSL_STD_FW_ECC_AGGR0_REGS_ECC_AGGR_END                                                     (0x0000003f00f3ffU)

/* Properties of firewall at slave: MSRAM_256K6_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f010000U)
#define CSL_STD_FW_MSRAM_256K6_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0103ffU)

/* Properties of firewall at slave: MSRAM_256K7_ECC_AGGR_REGSREGS */
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_ID                                                (22U)
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_MMR_BASE                                          (0x00000045005800U)
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_NUM_REGIONS                                       (16U)
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_START                               (0x0000003f011000U)
#define CSL_STD_FW_MSRAM_256K7_ECC_AGGR_REGSREGS_ECC_AGGR_REGS_END                                 (0x0000003f0113ffU)

/* Properties of firewall at slave: PBIST1_MEM */
#define CSL_STD_FW_PBIST1_MEM_ID                                                                   (22U)
#define CSL_STD_FW_PBIST1_MEM_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_PBIST1_MEM_MMR_BASE                                                             (0x00000045005800U)
#define CSL_STD_FW_PBIST1_MEM_NUM_REGIONS                                                          (16U)
#define CSL_STD_FW_PBIST1_MEM_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_PBIST1_MEM_START                                                                (0x0000003f100000U)
#define CSL_STD_FW_PBIST1_MEM_END                                                                  (0x0000003f1003ffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_INTR */
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_INTAGGR_INTR_START                                          (0x00000048000000U)
#define CSL_STD_FW_DMASS0_INTAGGR_INTR_INTAGGR_INTR_END                                            (0x000000480fffffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_IMAP */
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_INTAGGR_IMAP_START                                          (0x00000048100000U)
#define CSL_STD_FW_DMASS0_INTAGGR_IMAP_INTAGGR_IMAP_END                                            (0x00000048103fffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_CFG */
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_INTAGGR_CFG_START                                            (0x00000048110000U)
#define CSL_STD_FW_DMASS0_INTAGGR_CFG_INTAGGR_CFG_END                                              (0x0000004811001fU)

/* Properties of firewall at slave: DMASS0_INTAGGR_L2G */
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_INTAGGR_L2G_START                                            (0x00000048120000U)
#define CSL_STD_FW_DMASS0_INTAGGR_L2G_INTAGGR_L2G_END                                              (0x000000481203ffU)

/* Properties of firewall at slave: DMASS0_PSILCFG_PROXY */
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_PSILCFG_PROXY_START                                        (0x00000048130000U)
#define CSL_STD_FW_DMASS0_PSILCFG_PROXY_PSILCFG_PROXY_END                                          (0x000000481301ffU)

/* Properties of firewall at slave: DMASS0_PSILSS_MMRS */
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_PSILSS_MMRS_START                                            (0x00000048140000U)
#define CSL_STD_FW_DMASS0_PSILSS_MMRS_PSILSS_MMRS_END                                              (0x00000048140fffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_UNMAP */
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_INTAGGR_UNMAP_START                                        (0x00000048180000U)
#define CSL_STD_FW_DMASS0_INTAGGR_UNMAP_INTAGGR_UNMAP_END                                          (0x0000004819ffffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_MCAST */
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_INTAGGR_MCAST_START                                        (0x00000048210000U)
#define CSL_STD_FW_DMASS0_INTAGGR_MCAST_INTAGGR_MCAST_END                                          (0x00000048210fffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_GCNTCFG */
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_ID                                                       (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_TYPE                                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_MMR_BASE                                                 (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_NUM_REGIONS                                              (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_NUM_PRIV_IDS_PER_REGION                                  (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_INTAGGR_GCNTCFG_START                                    (0x00000048220000U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTCFG_INTAGGR_GCNTCFG_END                                      (0x00000048221fffU)

/* Properties of firewall at slave: DMASS0_ETLSW_MMRS */
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_ID                                                            (22U)
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_MMR_BASE                                                      (0x00000045005800U)
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_NUM_REGIONS                                                   (16U)
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_ETLSW_MMRS_START                                              (0x00000048230000U)
#define CSL_STD_FW_DMASS0_ETLSW_MMRS_ETLSW_MMRS_END                                                (0x00000048230fffU)

/* Properties of firewall at slave: DMASS0_RINGACC_GCFG */
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_RINGACC_GCFG_START                                          (0x00000048240000U)
#define CSL_STD_FW_DMASS0_RINGACC_GCFG_RINGACC_GCFG_END                                            (0x000000482403ffU)

/* Properties of firewall at slave: DMASS0_SEC_PROXY_MMRS */
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_ID                                                        (22U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_MMR_BASE                                                  (0x00000045005800U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_NUM_REGIONS                                               (16U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_SEC_PROXY_MMRS_START                                      (0x00000048250000U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_MMRS_SEC_PROXY_MMRS_END                                        (0x000000482500ffU)

/* Properties of firewall at slave: DMASS0_BCDMA_CRED */
#define CSL_STD_FW_DMASS0_BCDMA_CRED_ID                                                            (22U)
#define CSL_STD_FW_DMASS0_BCDMA_CRED_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_CRED_MMR_BASE                                                      (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_CRED_NUM_REGIONS                                                   (16U)
#define CSL_STD_FW_DMASS0_BCDMA_CRED_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_DMASS0_BCDMA_CRED_BCDMA_CRED_START                                              (0x00000048400000U)
#define CSL_STD_FW_DMASS0_BCDMA_CRED_BCDMA_CRED_END                                                (0x000000484007ffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_CRED */
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_PKTDMA_CRED_START                                            (0x00000048410000U)
#define CSL_STD_FW_DMASS0_PKTDMA_CRED_PKTDMA_CRED_END                                              (0x00000048411fffU)

/* Properties of firewall at slave: DMASS0_BCDMA_BCHAN */
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_BCDMA_BCHAN_START                                            (0x00000048420000U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHAN_BCDMA_BCHAN_END                                              (0x00000048421fffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_RFLOW */
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_PKTDMA_RFLOW_START                                          (0x00000048430000U)
#define CSL_STD_FW_DMASS0_PKTDMA_RFLOW_PKTDMA_RFLOW_END                                            (0x00000048433fffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_TCHAN */
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_PKTDMA_TCHAN_START                                          (0x000000484a0000U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHAN_PKTDMA_TCHAN_END                                            (0x000000484a3fffU)

/* Properties of firewall at slave: DMASS0_BCDMA_TCHAN */
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_BCDMA_TCHAN_START                                            (0x000000484a4000U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHAN_BCDMA_TCHAN_END                                              (0x000000484a5fffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_RCHAN */
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_PKTDMA_RCHAN_START                                          (0x000000484c0000U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHAN_PKTDMA_RCHAN_END                                            (0x000000484c1fffU)

/* Properties of firewall at slave: DMASS0_BCDMA_RCHAN */
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_BCDMA_RCHAN_START                                            (0x000000484c2000U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHAN_BCDMA_RCHAN_END                                              (0x000000484c3fffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_GCFG */
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_PKTDMA_GCFG_START                                            (0x000000485c0000U)
#define CSL_STD_FW_DMASS0_PKTDMA_GCFG_PKTDMA_GCFG_END                                              (0x000000485c00ffU)

/* Properties of firewall at slave: DMASS0_BCDMA_GCFG */
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_ID                                                            (22U)
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_MMR_BASE                                                      (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_NUM_REGIONS                                                   (16U)
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_BCDMA_GCFG_START                                              (0x000000485c0100U)
#define CSL_STD_FW_DMASS0_BCDMA_GCFG_BCDMA_GCFG_END                                                (0x000000485c01ffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_RING */
#define CSL_STD_FW_DMASS0_PKTDMA_RING_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_RING_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_RING_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_RING_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_RING_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_RING_PKTDMA_RING_START                                            (0x000000485e0000U)
#define CSL_STD_FW_DMASS0_PKTDMA_RING_PKTDMA_RING_END                                              (0x000000485fffffU)

/* Properties of firewall at slave: DMASS0_BCDMA_RING */
#define CSL_STD_FW_DMASS0_BCDMA_RING_ID                                                            (22U)
#define CSL_STD_FW_DMASS0_BCDMA_RING_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_RING_MMR_BASE                                                      (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_RING_NUM_REGIONS                                                   (16U)
#define CSL_STD_FW_DMASS0_BCDMA_RING_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_DMASS0_BCDMA_RING_BCDMA_RING_START                                              (0x00000048600000U)
#define CSL_STD_FW_DMASS0_BCDMA_RING_BCDMA_RING_END                                                (0x00000048607fffU)

/* Properties of firewall at slave: DMASS0_RINGACC_RT */
#define CSL_STD_FW_DMASS0_RINGACC_RT_ID                                                            (22U)
#define CSL_STD_FW_DMASS0_RINGACC_RT_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_RINGACC_RT_MMR_BASE                                                      (0x00000045005800U)
#define CSL_STD_FW_DMASS0_RINGACC_RT_NUM_REGIONS                                                   (16U)
#define CSL_STD_FW_DMASS0_RINGACC_RT_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_DMASS0_RINGACC_RT_RINGACC_RT_START                                              (0x00000049000000U)
#define CSL_STD_FW_DMASS0_RINGACC_RT_RINGACC_RT_END                                                (0x000000493fffffU)

/* Properties of firewall at slave: DMASS0_RINGACC_CFG */
#define CSL_STD_FW_DMASS0_RINGACC_CFG_ID                                                           (22U)
#define CSL_STD_FW_DMASS0_RINGACC_CFG_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_RINGACC_CFG_MMR_BASE                                                     (0x00000045005800U)
#define CSL_STD_FW_DMASS0_RINGACC_CFG_NUM_REGIONS                                                  (16U)
#define CSL_STD_FW_DMASS0_RINGACC_CFG_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_DMASS0_RINGACC_CFG_RINGACC_CFG_START                                            (0x00000049800000U)
#define CSL_STD_FW_DMASS0_RINGACC_CFG_RINGACC_CFG_END                                              (0x0000004983ffffU)

/* Properties of firewall at slave: DMASS0_INTAGGR_GCNTRTI */
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_ID                                                       (22U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_TYPE                                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_MMR_BASE                                                 (0x00000045005800U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_NUM_REGIONS                                              (16U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_NUM_PRIV_IDS_PER_REGION                                  (3U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_INTAGGR_GCNTRTI_START                                    (0x0000004a000000U)
#define CSL_STD_FW_DMASS0_INTAGGR_GCNTRTI_INTAGGR_GCNTRTI_END                                      (0x0000004a0fffffU)

/* Properties of firewall at slave: DMASS0_SEC_PROXY_SCFG */
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_ID                                                        (22U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_MMR_BASE                                                  (0x00000045005800U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_NUM_REGIONS                                               (16U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_SEC_PROXY_SCFG_START                                      (0x0000004a400000U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_SCFG_SEC_PROXY_SCFG_END                                        (0x0000004a47ffffU)

/* Properties of firewall at slave: DMASS0_SEC_PROXY_RT */
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_SEC_PROXY_RT_START                                          (0x0000004a600000U)
#define CSL_STD_FW_DMASS0_SEC_PROXY_RT_SEC_PROXY_RT_END                                            (0x0000004a67ffffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_RCHANRT */
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_ID                                                        (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_MMR_BASE                                                  (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_NUM_REGIONS                                               (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_PKTDMA_RCHANRT_START                                      (0x0000004a800000U)
#define CSL_STD_FW_DMASS0_PKTDMA_RCHANRT_PKTDMA_RCHANRT_END                                        (0x0000004a81ffffU)

/* Properties of firewall at slave: DMASS0_BCDMA_RCHANRT */
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_BCDMA_RCHANRT_START                                        (0x0000004a820000U)
#define CSL_STD_FW_DMASS0_BCDMA_RCHANRT_BCDMA_RCHANRT_END                                          (0x0000004a83ffffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_TCHANRT */
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_ID                                                        (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_MMR_BASE                                                  (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_NUM_REGIONS                                               (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_PKTDMA_TCHANRT_START                                      (0x0000004aa00000U)
#define CSL_STD_FW_DMASS0_PKTDMA_TCHANRT_PKTDMA_TCHANRT_END                                        (0x0000004aa3ffffU)

/* Properties of firewall at slave: DMASS0_BCDMA_TCHANRT */
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_BCDMA_TCHANRT_START                                        (0x0000004aa40000U)
#define CSL_STD_FW_DMASS0_BCDMA_TCHANRT_BCDMA_TCHANRT_END                                          (0x0000004aa5ffffU)

/* Properties of firewall at slave: DMASS0_PKTDMA_RINGRT */
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_PKTDMA_RINGRT_START                                        (0x0000004b800000U)
#define CSL_STD_FW_DMASS0_PKTDMA_RINGRT_PKTDMA_RINGRT_END                                          (0x0000004bbfffffU)

/* Properties of firewall at slave: DMASS0_BCDMA_RINGRT */
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_ID                                                          (22U)
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_MMR_BASE                                                    (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_NUM_REGIONS                                                 (16U)
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_BCDMA_RINGRT_START                                          (0x0000004bc00000U)
#define CSL_STD_FW_DMASS0_BCDMA_RINGRT_BCDMA_RINGRT_END                                            (0x0000004bcfffffU)

/* Properties of firewall at slave: DMASS0_BCDMA_BCHANRT */
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_ID                                                         (22U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_TYPE                                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_MMR_BASE                                                   (0x00000045005800U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_NUM_REGIONS                                                (16U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_NUM_PRIV_IDS_PER_REGION                                    (3U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_BCDMA_BCHANRT_START                                        (0x0000004c000000U)
#define CSL_STD_FW_DMASS0_BCDMA_BCHANRT_BCDMA_BCHANRT_END                                          (0x0000004c01ffffU)

#if defined(CSL_MODIFICATION)
/* Below macros via CSL scripts are generated with same macro name
 * Disabling this until this is resolved in json/CSL data
 * https://jira.itg.ti.com/browse/PROC_SPECDATA-169
 */

/* Properties of firewall at slave: MSRAM_256K6_RAM */
#define CSL_STD_FW_MSRAM_256K6_RAM_ID                                                              (23U)
#define CSL_STD_FW_MSRAM_256K6_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K6_RAM_MMR_BASE                                                        (0x00000045005c00U)
#define CSL_STD_FW_MSRAM_256K6_RAM_NUM_REGIONS                                                     (8U)
#define CSL_STD_FW_MSRAM_256K6_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K6_RAM_RAM_START                                                       (0x000000440a0000U)
#define CSL_STD_FW_MSRAM_256K6_RAM_RAM_END                                                         (0x000000440bffffU)

/* Properties of firewall at slave: MSRAM_256K6_RAM */
#define CSL_STD_FW_MSRAM_256K6_RAM_ID                                                              (23U)
#define CSL_STD_FW_MSRAM_256K6_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K6_RAM_MMR_BASE                                                        (0x00000045005c00U)
#define CSL_STD_FW_MSRAM_256K6_RAM_NUM_REGIONS                                                     (8U)
#define CSL_STD_FW_MSRAM_256K6_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K6_RAM_RAM_START                                                       (0x000000440c0000U)
#define CSL_STD_FW_MSRAM_256K6_RAM_RAM_END                                                         (0x000000440dffffU)
#endif

/* Properties of firewall at slave: MSRAM_256K6_RAM */
#define CSL_STD_FW_MSRAM_256K6_RAM_ID                                                              (23U)
#define CSL_STD_FW_MSRAM_256K6_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K6_RAM_MMR_BASE                                                        (0x00000045005c00U)
#define CSL_STD_FW_MSRAM_256K6_RAM_NUM_REGIONS                                                     (8U)
#define CSL_STD_FW_MSRAM_256K6_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K6_RAM_RAM_START                                                       (0x00000070180000U)
#define CSL_STD_FW_MSRAM_256K6_RAM_RAM_END                                                         (0x000000701bffffU)

#if defined(CSL_MODIFICATION)
/* Below macros via CSL scripts are generated with same macro name
 * Disabling this until this is resolved in json/CSL data
 * https://jira.itg.ti.com/browse/PROC_SPECDATA-169
 */
/* Properties of firewall at slave: MSRAM_256K7_RAM */
#define CSL_STD_FW_MSRAM_256K7_RAM_ID                                                              (24U)
#define CSL_STD_FW_MSRAM_256K7_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K7_RAM_MMR_BASE                                                        (0x00000045006000U)
#define CSL_STD_FW_MSRAM_256K7_RAM_NUM_REGIONS                                                     (8U)
#define CSL_STD_FW_MSRAM_256K7_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K7_RAM_RAM_START                                                       (0x00000044060000U)
#define CSL_STD_FW_MSRAM_256K7_RAM_RAM_END                                                         (0x0000004407ffffU)

/* Properties of firewall at slave: MSRAM_256K7_RAM */
#define CSL_STD_FW_MSRAM_256K7_RAM_ID                                                              (24U)
#define CSL_STD_FW_MSRAM_256K7_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K7_RAM_MMR_BASE                                                        (0x00000045006000U)
#define CSL_STD_FW_MSRAM_256K7_RAM_NUM_REGIONS                                                     (8U)
#define CSL_STD_FW_MSRAM_256K7_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K7_RAM_RAM_START                                                       (0x00000044080000U)
#define CSL_STD_FW_MSRAM_256K7_RAM_RAM_END                                                         (0x0000004409ffffU)
#endif

/* Properties of firewall at slave: MSRAM_256K7_RAM */
#define CSL_STD_FW_MSRAM_256K7_RAM_ID                                                              (24U)
#define CSL_STD_FW_MSRAM_256K7_RAM_TYPE                                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MSRAM_256K7_RAM_MMR_BASE                                                        (0x00000045006000U)
#define CSL_STD_FW_MSRAM_256K7_RAM_NUM_REGIONS                                                     (8U)
#define CSL_STD_FW_MSRAM_256K7_RAM_NUM_PRIV_IDS_PER_REGION                                         (3U)
#define CSL_STD_FW_MSRAM_256K7_RAM_RAM_START                                                       (0x000000701c0000U)
#define CSL_STD_FW_MSRAM_256K7_RAM_RAM_END                                                         (0x000000701fffffU)

/* Properties of firewall at slave: PSRAMECC0_RAM */
#define CSL_STD_FW_PSRAMECC0_RAM_ID                                                                (34U)
#define CSL_STD_FW_PSRAMECC0_RAM_TYPE                                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_PSRAMECC0_RAM_MMR_BASE                                                          (0x00000045008800U)
#define CSL_STD_FW_PSRAMECC0_RAM_NUM_REGIONS                                                       (8U)
#define CSL_STD_FW_PSRAMECC0_RAM_NUM_PRIV_IDS_PER_REGION                                           (3U)
#define CSL_STD_FW_PSRAMECC0_RAM_RAM_START                                                         (0x00000000000000U)
#define CSL_STD_FW_PSRAMECC0_RAM_RAM_END                                                           (0x000000000003ffU)

/* Properties of firewall at slave: PADCFG_CTRL0_CFG0 */
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_ID                                                            (34U)
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_MMR_BASE                                                      (0x00000045008800U)
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_NUM_REGIONS                                                   (8U)
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_CFG0_START                                                    (0x000000000f0000U)
#define CSL_STD_FW_PADCFG_CTRL0_CFG0_CFG0_END                                                      (0x000000000f7fffU)

/* Properties of firewall at slave: CBASS_DBG0_ERR_REGS */
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_ID                                                          (34U)
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_MMR_BASE                                                    (0x00000045008800U)
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_NUM_REGIONS                                                 (8U)
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_ERR_START                                                   (0x00000000200000U)
#define CSL_STD_FW_CBASS_DBG0_ERR_REGS_ERR_END                                                     (0x000000002003ffU)

/* Properties of firewall at slave: CBASS_FW0_ERR_REGS */
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_ID                                                           (34U)
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_TYPE                                                         (CSL_FW_SECURITY)
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_MMR_BASE                                                     (0x00000045008800U)
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_NUM_REGIONS                                                  (8U)
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_NUM_PRIV_IDS_PER_REGION                                      (3U)
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_ERR_START                                                    (0x00000000220000U)
#define CSL_STD_FW_CBASS_FW0_ERR_REGS_ERR_END                                                      (0x000000002203ffU)

/* Properties of firewall at slave: EFUSE0_MEM */
#define CSL_STD_FW_EFUSE0_MEM_ID                                                                   (34U)
#define CSL_STD_FW_EFUSE0_MEM_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_EFUSE0_MEM_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_EFUSE0_MEM_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_EFUSE0_MEM_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_EFUSE0_MEM_START                                                                (0x00000000300000U)
#define CSL_STD_FW_EFUSE0_MEM_END                                                                  (0x000000003000ffU)

/* Properties of firewall at slave: PBIST0_MEM */
#define CSL_STD_FW_PBIST0_MEM_ID                                                                   (34U)
#define CSL_STD_FW_PBIST0_MEM_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_PBIST0_MEM_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_PBIST0_MEM_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_PBIST0_MEM_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_PBIST0_MEM_START                                                                (0x00000000310000U)
#define CSL_STD_FW_PBIST0_MEM_END                                                                  (0x000000003103ffU)

/* Properties of firewall at slave: COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS */
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_ID                                    (34U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_TYPE                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_MMR_BASE                              (0x00000045008800U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_NUM_REGIONS                           (8U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_NUM_PRIV_IDS_PER_REGION               (3U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_PBIST_START                           (0x00000000330000U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_K3_PBIST_4C28P_WRAP_REGS_PBIST_END                             (0x000000003303ffU)

/* Properties of firewall at slave: PSC0_VBUS */
#define CSL_STD_FW_PSC0_VBUS_ID                                                                    (34U)
#define CSL_STD_FW_PSC0_VBUS_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PSC0_VBUS_MMR_BASE                                                              (0x00000045008800U)
#define CSL_STD_FW_PSC0_VBUS_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_PSC0_VBUS_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_PSC0_VBUS_START                                                                 (0x00000000400000U)
#define CSL_STD_FW_PSC0_VBUS_END                                                                   (0x00000000400fffU)

/* Properties of firewall at slave: PLLCTRL0_MEM */
#define CSL_STD_FW_PLLCTRL0_MEM_ID                                                                 (34U)
#define CSL_STD_FW_PLLCTRL0_MEM_TYPE                                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_PLLCTRL0_MEM_MMR_BASE                                                           (0x00000045008800U)
#define CSL_STD_FW_PLLCTRL0_MEM_NUM_REGIONS                                                        (8U)
#define CSL_STD_FW_PLLCTRL0_MEM_NUM_PRIV_IDS_PER_REGION                                            (3U)
#define CSL_STD_FW_PLLCTRL0_MEM_START                                                              (0x00000000410000U)
#define CSL_STD_FW_PLLCTRL0_MEM_END                                                                (0x000000004101ffU)

/* Properties of firewall at slave: ESM0_CFG */
#define CSL_STD_FW_ESM0_CFG_ID                                                                     (34U)
#define CSL_STD_FW_ESM0_CFG_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_ESM0_CFG_MMR_BASE                                                               (0x00000045008800U)
#define CSL_STD_FW_ESM0_CFG_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_ESM0_CFG_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_ESM0_CFG_CFG_START                                                              (0x00000000420000U)
#define CSL_STD_FW_ESM0_CFG_CFG_END                                                                (0x00000000420fffU)

/* Properties of firewall at slave: DFTSS0_MEM */
#define CSL_STD_FW_DFTSS0_MEM_ID                                                                   (34U)
#define CSL_STD_FW_DFTSS0_MEM_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_DFTSS0_MEM_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_DFTSS0_MEM_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_DFTSS0_MEM_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_DFTSS0_MEM_START                                                                (0x00000000500000U)
#define CSL_STD_FW_DFTSS0_MEM_END                                                                  (0x000000005003ffU)

/* Properties of firewall at slave: DDPA0_DDPA */
#define CSL_STD_FW_DDPA0_DDPA_ID                                                                   (34U)
#define CSL_STD_FW_DDPA0_DDPA_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_DDPA0_DDPA_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_DDPA0_DDPA_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_DDPA0_DDPA_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_DDPA0_DDPA_START                                                                (0x00000000580000U)
#define CSL_STD_FW_DDPA0_DDPA_END                                                                  (0x000000005803ffU)

/* Properties of firewall at slave: GPIO0_MEM */
#define CSL_STD_FW_GPIO0_MEM_ID                                                                    (34U)
#define CSL_STD_FW_GPIO0_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_GPIO0_MEM_MMR_BASE                                                              (0x00000045008800U)
#define CSL_STD_FW_GPIO0_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_GPIO0_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_GPIO0_MEM_START                                                                 (0x00000000600000U)
#define CSL_STD_FW_GPIO0_MEM_END                                                                   (0x000000006000ffU)

/* Properties of firewall at slave: GPIO1_MEM */
#define CSL_STD_FW_GPIO1_MEM_ID                                                                    (34U)
#define CSL_STD_FW_GPIO1_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_GPIO1_MEM_MMR_BASE                                                              (0x00000045008800U)
#define CSL_STD_FW_GPIO1_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_GPIO1_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_GPIO1_MEM_START                                                                 (0x00000000601000U)
#define CSL_STD_FW_GPIO1_MEM_END                                                                   (0x000000006010ffU)

/* Properties of firewall at slave: PLL0_CFG */
#define CSL_STD_FW_PLL0_CFG_ID                                                                     (34U)
#define CSL_STD_FW_PLL0_CFG_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_PLL0_CFG_MMR_BASE                                                               (0x00000045008800U)
#define CSL_STD_FW_PLL0_CFG_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_PLL0_CFG_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_PLL0_CFG_CFG_START                                                              (0x00000000680000U)
#define CSL_STD_FW_PLL0_CFG_CFG_END                                                                (0x0000000068ffffU)

/* Properties of firewall at slave: PSRAMECC0_REGS */
#define CSL_STD_FW_PSRAMECC0_REGS_ID                                                               (34U)
#define CSL_STD_FW_PSRAMECC0_REGS_TYPE                                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_PSRAMECC0_REGS_MMR_BASE                                                         (0x00000045008800U)
#define CSL_STD_FW_PSRAMECC0_REGS_NUM_REGIONS                                                      (8U)
#define CSL_STD_FW_PSRAMECC0_REGS_NUM_PRIV_IDS_PER_REGION                                          (3U)
#define CSL_STD_FW_PSRAMECC0_REGS_ECC_AGGR_START                                                   (0x00000000700000U)
#define CSL_STD_FW_PSRAMECC0_REGS_ECC_AGGR_END                                                     (0x000000007003ffU)

/* Properties of firewall at slave: ECC_AGGR1_REGS */
#define CSL_STD_FW_ECC_AGGR1_REGS_ID                                                               (34U)
#define CSL_STD_FW_ECC_AGGR1_REGS_TYPE                                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_ECC_AGGR1_REGS_MMR_BASE                                                         (0x00000045008800U)
#define CSL_STD_FW_ECC_AGGR1_REGS_NUM_REGIONS                                                      (8U)
#define CSL_STD_FW_ECC_AGGR1_REGS_NUM_PRIV_IDS_PER_REGION                                          (3U)
#define CSL_STD_FW_ECC_AGGR1_REGS_ECC_AGGR_START                                                   (0x00000000701000U)
#define CSL_STD_FW_ECC_AGGR1_REGS_ECC_AGGR_END                                                     (0x000000007013ffU)

/* Properties of firewall at slave: USB0_A_ECC_AGGR_CFG_REGS */
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_ID                                                     (34U)
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_MMR_BASE                                               (0x00000045008800U)
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_NUM_REGIONS                                            (8U)
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_ECC_AGGR_START                                         (0x00000000703000U)
#define CSL_STD_FW_USB0_A_ECC_AGGR_CFG_REGS_ECC_AGGR_END                                           (0x000000007033ffU)

/* Properties of firewall at slave: CPSW0_CPSW_NUSS_VBUSP_ECC */
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_ID                                                    (34U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_TYPE                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_MMR_BASE                                              (0x00000045008800U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_NUM_REGIONS                                           (8U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_NUM_PRIV_IDS_PER_REGION                               (3U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_ECC_START                                             (0x00000000704000U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ECC_ECC_END                                               (0x000000007043ffU)

/* Properties of firewall at slave: MMCSD0_ECC_AGGR_RXMEM_CFG_REGS */
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_ID                                               (34U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_MMR_BASE                                         (0x00000045008800U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_NUM_REGIONS                                      (8U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_ECC_AGGR_RXMEM_START                             (0x00000000706000U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_RXMEM_CFG_REGS_ECC_AGGR_RXMEM_END                               (0x000000007063ffU)

/* Properties of firewall at slave: MMCSD0_ECC_AGGR_TXMEM_CFG_REGS */
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_ID                                               (34U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_MMR_BASE                                         (0x00000045008800U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_NUM_REGIONS                                      (8U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_ECC_AGGR_TXMEM_START                             (0x00000000707000U)
#define CSL_STD_FW_MMCSD0_ECC_AGGR_TXMEM_CFG_REGS_ECC_AGGR_TXMEM_END                               (0x000000007073ffU)

/* Properties of firewall at slave: MMCSD1_ECC_AGGR_RXMEM_CFG_REGS */
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_ID                                               (34U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_MMR_BASE                                         (0x00000045008800U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_NUM_REGIONS                                      (8U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_ECC_AGGR_RXMEM_START                             (0x00000000708000U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_RXMEM_CFG_REGS_ECC_AGGR_RXMEM_END                               (0x000000007083ffU)

/* Properties of firewall at slave: MMCSD1_ECC_AGGR_TXMEM_CFG_REGS */
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_ID                                               (34U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_MMR_BASE                                         (0x00000045008800U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_NUM_REGIONS                                      (8U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_ECC_AGGR_TXMEM_START                             (0x00000000709000U)
#define CSL_STD_FW_MMCSD1_ECC_AGGR_TXMEM_CFG_REGS_ECC_AGGR_TXMEM_END                               (0x000000007093ffU)

/* Properties of firewall at slave: FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS */
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_ID                        (34U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_TYPE                      (CSL_FW_SECURITY)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_MMR_BASE                  (0x00000045008800U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_NUM_REGIONS               (8U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_NUM_PRIV_IDS_PER_REGION   (3U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_OSPI0_ECC_AGGR_START      (0x00000000716000U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGGR_VBP_REGS_OSPI0_ECC_AGGR_END        (0x000000007163ffU)

/* Properties of firewall at slave: COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS */
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_ID                                       (34U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_MMR_BASE                                 (0x00000045008800U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_NUM_REGIONS                              (8U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_SS_ECC_AGGR_START                        (0x00000000717000U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_COREPAC_REGS_SS_ECC_AGGR_END                          (0x000000007173ffU)

/* Properties of firewall at slave: COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS */
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_ID                                         (34U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_MMR_BASE                                   (0x00000045008800U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_NUM_REGIONS                                (8U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_CORE0_ECC_AGGR_START                       (0x00000000717400U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE0_REGS_CORE0_ECC_AGGR_END                         (0x000000007177ffU)

/* Properties of firewall at slave: COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS */
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_ID                                         (34U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_MMR_BASE                                   (0x00000045008800U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_NUM_REGIONS                                (8U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_CORE1_ECC_AGGR_START                       (0x00000000717800U)
#define CSL_STD_FW_COMPUTE_CLUSTER0_ECC_AGGR_CORE1_REGS_CORE1_ECC_AGGR_END                         (0x00000000717bffU)

/* Properties of firewall at slave: PCIE0_CORE_ECC_AGGR0_REGS */
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_ID                                                    (34U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_TYPE                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_MMR_BASE                                              (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_NUM_REGIONS                                           (8U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_NUM_PRIV_IDS_PER_REGION                               (3U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_CORE_ECC_AGGR0_START                                  (0x00000000718000U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR0_REGS_CORE_ECC_AGGR0_END                                    (0x000000007183ffU)

/* Properties of firewall at slave: PCIE0_CORE_ECC_AGGR1_REGS */
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_ID                                                    (34U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_TYPE                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_MMR_BASE                                              (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_NUM_REGIONS                                           (8U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_NUM_PRIV_IDS_PER_REGION                               (3U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_CORE_ECC_AGGR1_START                                  (0x00000000719000U)
#define CSL_STD_FW_PCIE0_CORE_ECC_AGGR1_REGS_CORE_ECC_AGGR1_END                                    (0x000000007193ffU)

/* Properties of firewall at slave: MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG */
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_ID                                      (34U)
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_TYPE                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_MMR_BASE                                (0x00000045008800U)
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_NUM_REGIONS                             (8U)
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_NUM_PRIV_IDS_PER_REGION                 (3U)
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_CFG_START                               (0x00000000a00000U)
#define CSL_STD_FW_MAIN_GPIOMUX_INTROUTER0_INTR_ROUTER_CFG_CFG_END                                 (0x00000000a007ffU)

/* Properties of firewall at slave: CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG */
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_ID                                         (34U)
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_MMR_BASE                                   (0x00000045008800U)
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_NUM_REGIONS                                (8U)
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_CFG_START                                  (0x00000000a30000U)
#define CSL_STD_FW_CMP_EVENT_INTROUTER0_INTR_ROUTER_CFG_CFG_END                                    (0x00000000a307ffU)

/* Properties of firewall at slave: TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG */
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_ID                                    (34U)
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_TYPE                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_MMR_BASE                              (0x00000045008800U)
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_NUM_REGIONS                           (8U)
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_NUM_PRIV_IDS_PER_REGION               (3U)
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_CFG_START                             (0x00000000a40000U)
#define CSL_STD_FW_TIMESYNC_EVENT_INTROUTER0_INTR_ROUTER_CFG_CFG_END                               (0x00000000a407ffU)

/* Properties of firewall at slave: GTC0_GTC_CFG0 */
#define CSL_STD_FW_GTC0_GTC_CFG0_ID                                                                (34U)
#define CSL_STD_FW_GTC0_GTC_CFG0_TYPE                                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_GTC0_GTC_CFG0_MMR_BASE                                                          (0x00000045008800U)
#define CSL_STD_FW_GTC0_GTC_CFG0_NUM_REGIONS                                                       (8U)
#define CSL_STD_FW_GTC0_GTC_CFG0_NUM_PRIV_IDS_PER_REGION                                           (3U)
#define CSL_STD_FW_GTC0_GTC_CFG0_GTC_CFG0_START                                                    (0x00000000a80000U)
#define CSL_STD_FW_GTC0_GTC_CFG0_GTC_CFG0_END                                                      (0x00000000a803ffU)

/* Properties of firewall at slave: GTC0_GTC_CFG1 */
#define CSL_STD_FW_GTC0_GTC_CFG1_ID                                                                (34U)
#define CSL_STD_FW_GTC0_GTC_CFG1_TYPE                                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_GTC0_GTC_CFG1_MMR_BASE                                                          (0x00000045008800U)
#define CSL_STD_FW_GTC0_GTC_CFG1_NUM_REGIONS                                                       (8U)
#define CSL_STD_FW_GTC0_GTC_CFG1_NUM_PRIV_IDS_PER_REGION                                           (3U)
#define CSL_STD_FW_GTC0_GTC_CFG1_GTC_CFG1_START                                                    (0x00000000a90000U)
#define CSL_STD_FW_GTC0_GTC_CFG1_GTC_CFG1_END                                                      (0x00000000a93fffU)

/* Properties of firewall at slave: GTC0_GTC_CFG2 */
#define CSL_STD_FW_GTC0_GTC_CFG2_ID                                                                (34U)
#define CSL_STD_FW_GTC0_GTC_CFG2_TYPE                                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_GTC0_GTC_CFG2_MMR_BASE                                                          (0x00000045008800U)
#define CSL_STD_FW_GTC0_GTC_CFG2_NUM_REGIONS                                                       (8U)
#define CSL_STD_FW_GTC0_GTC_CFG2_NUM_PRIV_IDS_PER_REGION                                           (3U)
#define CSL_STD_FW_GTC0_GTC_CFG2_GTC_CFG2_START                                                    (0x00000000aa0000U)
#define CSL_STD_FW_GTC0_GTC_CFG2_GTC_CFG2_END                                                      (0x00000000aa3fffU)

/* Properties of firewall at slave: GTC0_GTC_CFG3 */
#define CSL_STD_FW_GTC0_GTC_CFG3_ID                                                                (34U)
#define CSL_STD_FW_GTC0_GTC_CFG3_TYPE                                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_GTC0_GTC_CFG3_MMR_BASE                                                          (0x00000045008800U)
#define CSL_STD_FW_GTC0_GTC_CFG3_NUM_REGIONS                                                       (8U)
#define CSL_STD_FW_GTC0_GTC_CFG3_NUM_PRIV_IDS_PER_REGION                                           (3U)
#define CSL_STD_FW_GTC0_GTC_CFG3_GTC_CFG3_START                                                    (0x00000000ab0000U)
#define CSL_STD_FW_GTC0_GTC_CFG3_GTC_CFG3_END                                                      (0x00000000ab3fffU)

/* Properties of firewall at slave: VTM0_MMR_VBUSP_CFG1 */
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_ID                                                          (34U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_MMR_BASE                                                    (0x00000045008800U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_NUM_REGIONS                                                 (8U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_MMR_VBUSP_CFG1_START                                        (0x00000000b00000U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG1_MMR_VBUSP_CFG1_END                                          (0x00000000b003ffU)

/* Properties of firewall at slave: VTM0_MMR_VBUSP_CFG2 */
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_ID                                                          (34U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_MMR_BASE                                                    (0x00000045008800U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_NUM_REGIONS                                                 (8U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_MMR_VBUSP_CFG2_START                                        (0x00000000b01000U)
#define CSL_STD_FW_VTM0_MMR_VBUSP_CFG2_MMR_VBUSP_CFG2_END                                          (0x00000000b013ffU)

/* Properties of firewall at slave: VTM0_ECCAGGR_CFG_REGS */
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_ID                                                       (34U)
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_TYPE                                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_MMR_BASE                                                 (0x00000045008800U)
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_NUM_REGIONS                                              (8U)
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_NUM_PRIV_IDS_PER_REGION                                  (3U)
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_ECCAGGR_CFG_START                                        (0x00000000b02000U)
#define CSL_STD_FW_VTM0__ECCAGGR_CFG_REGS_ECCAGGR_CFG_END                                          (0x00000000b023ffU)

/* Properties of firewall at slave: PDMA0_REGS */
#define CSL_STD_FW_PDMA0_REGS_ID                                                                   (34U)
#define CSL_STD_FW_PDMA0_REGS_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_PDMA0_REGS_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_PDMA0_REGS_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_PDMA0_REGS_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_PDMA0_REGS_REGS_START                                                           (0x00000000c00000U)
#define CSL_STD_FW_PDMA0_REGS_REGS_END                                                             (0x00000000c003ffU)

/* Properties of firewall at slave: PDMA1_REGS */
#define CSL_STD_FW_PDMA1_REGS_ID                                                                   (34U)
#define CSL_STD_FW_PDMA1_REGS_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_PDMA1_REGS_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_PDMA1_REGS_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_PDMA1_REGS_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_PDMA1_REGS_REGS_START                                                           (0x00000000c01000U)
#define CSL_STD_FW_PDMA1_REGS_REGS_END                                                             (0x00000000c013ffU)

/* Properties of firewall at slave: PBIST3_MEM */
#define CSL_STD_FW_PBIST3_MEM_ID                                                                   (34U)
#define CSL_STD_FW_PBIST3_MEM_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_PBIST3_MEM_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_PBIST3_MEM_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_PBIST3_MEM_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_PBIST3_MEM_START                                                                (0x00000003310000U)
#define CSL_STD_FW_PBIST3_MEM_END                                                                  (0x000000033103ffU)

/* Properties of firewall at slave: PBIST2_MEM */
#define CSL_STD_FW_PBIST2_MEM_ID                                                                   (34U)
#define CSL_STD_FW_PBIST2_MEM_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_PBIST2_MEM_MMR_BASE                                                             (0x00000045008800U)
#define CSL_STD_FW_PBIST2_MEM_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_PBIST2_MEM_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_PBIST2_MEM_START                                                                (0x00000003330000U)
#define CSL_STD_FW_PBIST2_MEM_END                                                                  (0x000000033303ffU)

/* Properties of firewall at slave: CPSW0_CPSW_NUSS_VBUSP */
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_ID                                                        (34U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_MMR_BASE                                                  (0x00000045008800U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_NUM_REGIONS                                               (8U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_NUSS_START                                                (0x00000008000000U)
#define CSL_STD_FW_CPSW0_CPSW_NUSS_VBUSP_NUSS_END                                                  (0x000000081fffffU)

/* Properties of firewall at slave: PCIE0_CORE_DBN_CFG_PCIE_CORE_REG */
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_ID                                             (34U)
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_MMR_BASE                                       (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_NUM_REGIONS                                    (8U)
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_CORE_DBN_CFG_PCIE_CORE_START                   (0x0000000d000000U)
#define CSL_STD_FW_PCIE0_CORE_DBN_CFG_PCIE_CORE_REG_CORE_DBN_CFG_PCIE_CORE_END                     (0x0000000d7fffffU)

/* Properties of firewall at slave: SERDES_10G0_WIZ16B2M4CT */
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_ID                                                      (34U)
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_TYPE                                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_MMR_BASE                                                (0x00000045008800U)
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_NUM_REGIONS                                             (8U)
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_NUM_PRIV_IDS_PER_REGION                                 (3U)
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_START                                                   (0x0000000f000000U)
#define CSL_STD_FW_SERDES_10G0_WIZ16B2M4CT_END                                                     (0x0000000f00ffffU)

/* Properties of firewall at slave: PCIE0_CORE_USER_CFG_USER_CFG */
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_ID                                                 (34U)
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_MMR_BASE                                           (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_NUM_REGIONS                                        (8U)
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_CORE_USER_CFG_USER_CFG_START                       (0x0000000f100000U)
#define CSL_STD_FW_PCIE0_CORE_USER_CFG_USER_CFG_CORE_USER_CFG_USER_CFG_END                         (0x0000000f1003ffU)

/* Properties of firewall at slave: PCIE0_CORE_VMAP_OB_MMRS */
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_ID                                                      (34U)
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_TYPE                                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_MMR_BASE                                                (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_NUM_REGIONS                                             (8U)
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_NUM_PRIV_IDS_PER_REGION                                 (3U)
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_CORE_VMAP_OB_MMRS_START                                 (0x0000000f101000U)
#define CSL_STD_FW_PCIE0_CORE_VMAP_OB_MMRS_CORE_VMAP_OB_MMRS_END                                   (0x0000000f101fffU)

/* Properties of firewall at slave: PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG */
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_ID                                            (34U)
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_MMR_BASE                                      (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_NUM_REGIONS                                   (8U)
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_CORE_PCIE_INTD_CFG_INTD_CFG_START             (0x0000000f102000U)
#define CSL_STD_FW_PCIE0_CORE_PCIE_INTD_CFG_INTD_CFG_CORE_PCIE_INTD_CFG_INTD_CFG_END               (0x0000000f102fffU)

/* Properties of firewall at slave: PCIE0_CORE_CPTS_CFG_CPTS_VBUSP */
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_ID                                               (34U)
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_MMR_BASE                                         (0x00000045008800U)
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_NUM_REGIONS                                      (8U)
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_CORE_CPTS_CFG_CPTS_VBUSP_START                   (0x0000000f103000U)
#define CSL_STD_FW_PCIE0_CORE_CPTS_CFG_CPTS_VBUSP_CORE_CPTS_CFG_CPTS_VBUSP_END                     (0x0000000f1033ffU)

/* Properties of firewall at slave: DDR16SS0_REGS_SS_CFG_SSCFG */
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_ID                                                   (34U)
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_TYPE                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_MMR_BASE                                             (0x00000045008800U)
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_NUM_REGIONS                                          (8U)
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_NUM_PRIV_IDS_PER_REGION                              (3U)
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_SS_CFG_START                                         (0x0000000f300000U)
#define CSL_STD_FW_DDR16SS0_REGS_SS_CFG_SSCFG_SS_CFG_END                                           (0x0000000f3001ffU)

/* Properties of firewall at slave: DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG */
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_ID                                          (34U)
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_TYPE                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_MMR_BASE                                    (0x00000045008800U)
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_NUM_REGIONS                                 (8U)
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_NUM_PRIV_IDS_PER_REGION                     (3U)
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_CTL_CFG_START                               (0x0000000f308000U)
#define CSL_STD_FW_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_CTL_CFG_END                                 (0x0000000f30ffffU)

/* Properties of firewall at slave: USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP */
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_ID                               (34U)
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_TYPE                             (CSL_FW_SECURITY)
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_MMR_BASE                         (0x00000045008800U)
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_NUM_REGIONS                      (8U)
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_NUM_PRIV_IDS_PER_REGION          (3U)
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_START (0x0000000f400000U)
#define CSL_STD_FW_USB0_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_VBP2APB_WRAP_CONTROLLER_VBP_CORE_ADDR_MAP_END (0x0000000f43ffffU)

/* Properties of firewall at slave: USB0_MMR_MMRVBP_USBSS_CMN */
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_ID                                                    (34U)
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_TYPE                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_MMR_BASE                                              (0x00000045008800U)
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_NUM_REGIONS                                           (8U)
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_NUM_PRIV_IDS_PER_REGION                               (3U)
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_MMR_MMRVBP_USBSS_CMN_START                            (0x0000000f900000U)
#define CSL_STD_FW_USB0_MMR_MMRVBP_USBSS_CMN_MMR_MMRVBP_USBSS_CMN_END                              (0x0000000f9000ffU)

/* Properties of firewall at slave: USB0_RAMS_INJ_CFG_CFG */
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_ID                                                        (34U)
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_MMR_BASE                                                  (0x00000045008800U)
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_NUM_REGIONS                                               (8U)
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_RAMS_INJ_CFG_START                                        (0x0000000f901000U)
#define CSL_STD_FW_USB0_RAMS_INJ_CFG_CFG_RAMS_INJ_CFG_END                                          (0x0000000f9013ffU)

/* Properties of firewall at slave: USB0_PHY2 */
#define CSL_STD_FW_USB0_PHY2_ID                                                                    (34U)
#define CSL_STD_FW_USB0_PHY2_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_USB0_PHY2_MMR_BASE                                                              (0x00000045008800U)
#define CSL_STD_FW_USB0_PHY2_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_USB0_PHY2_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_USB0_PHY2_PHY2_START                                                            (0x0000000f908000U)
#define CSL_STD_FW_USB0_PHY2_PHY2_END                                                              (0x0000000f9083ffU)

/* Properties of firewall at slave: MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG */
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_ID                                              (34U)
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_MMR_BASE                                        (0x00000045008800U)
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_NUM_REGIONS                                     (8U)
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_CTL_CFG_START                                   (0x0000000fa00000U)
#define CSL_STD_FW_MMCSD1_SDHC_WRAP_CTL_CFG_CTLCFG_CTL_CFG_END                                     (0x0000000fa00fffU)

/* Properties of firewall at slave: MMCSD1_REGS_SS_CFG_SSCFG */
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_ID                                                     (34U)
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_MMR_BASE                                               (0x00000045008800U)
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_NUM_REGIONS                                            (8U)
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_SS_CFG_START                                           (0x0000000fa08000U)
#define CSL_STD_FW_MMCSD1_REGS_SS_CFG_SSCFG_SS_CFG_END                                             (0x0000000fa083ffU)

/* Properties of firewall at slave: MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG */
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_ID                                              (34U)
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_TYPE                                            (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_MMR_BASE                                        (0x00000045008800U)
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_NUM_REGIONS                                     (8U)
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_NUM_PRIV_IDS_PER_REGION                         (3U)
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_CTL_CFG_START                                   (0x0000000fa10000U)
#define CSL_STD_FW_MMCSD0_SDHC_WRAP_CTL_CFG_CTLCFG_CTL_CFG_END                                     (0x0000000fa10fffU)

/* Properties of firewall at slave: MMCSD0_REGS_SS_CFG_SSCFG */
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_ID                                                     (34U)
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_MMR_BASE                                               (0x00000045008800U)
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_NUM_REGIONS                                            (8U)
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_SS_CFG_START                                           (0x0000000fa18000U)
#define CSL_STD_FW_MMCSD0_REGS_SS_CFG_SSCFG_SS_CFG_END                                             (0x0000000fa183ffU)

/* Properties of firewall at slave: FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS */
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_ID                                         (34U)
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_TYPE                                       (CSL_FW_SECURITY)
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_MMR_BASE                                   (0x00000045008800U)
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_NUM_REGIONS                                (8U)
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_NUM_PRIV_IDS_PER_REGION                    (3U)
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_CFG_START                                  (0x0000000fc00000U)
#define CSL_STD_FW_FSS0_FSS_MMR_FSS_MMR_CFG_FSS_GENREGS_CFG_END                                    (0x0000000fc000ffU)

/* Properties of firewall at slave: FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS */
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_ID                                          (34U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_TYPE                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_MMR_BASE                                    (0x00000045008800U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_NUM_REGIONS                                 (8U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_NUM_PRIV_IDS_PER_REGION                     (3U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_FSAS_CFG_START                              (0x0000000fc10000U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_MMR_CFG_FSAS_GENREGS_FSAS_CFG_END                                (0x0000000fc100ffU)

/* Properties of firewall at slave: FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS */
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_ID                                       (34U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_MMR_BASE                                 (0x00000045008800U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_NUM_REGIONS                              (8U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_OTFA_CFG_START                           (0x0000000fc20000U)
#define CSL_STD_FW_FSS0_FSAS_FSAS_OTFA_CFG_FSAS_OTFA_REGS_OTFA_CFG_END                             (0x0000000fc20fffU)

/* Properties of firewall at slave: FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS */
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_ID      (34U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_TYPE    (CSL_FW_SECURITY)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_MMR_BASE (0x00000045008800U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_NUM_REGIONS (8U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_NUM_PRIV_IDS_PER_REGION (3U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_OSPI0_CTRL_START (0x0000000fc40000U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_REGS_OSPI0_CTRL_END (0x0000000fc400ffU)

/* Properties of firewall at slave: FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS */
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_ID                                    (34U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_TYPE                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_MMR_BASE                              (0x00000045008800U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_NUM_REGIONS                           (8U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_NUM_PRIV_IDS_PER_REGION               (3U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_OSPI0_SS_CFG_START                    (0x0000000fc44000U)
#define CSL_STD_FW_FSS0_OSPI0_OSPI_CFG_VBUSP_MMR_MMRVBP_REGS_OSPI0_SS_CFG_END                      (0x0000000fc441ffU)

/* Properties of firewall at slave: ELM0_MEM */
#define CSL_STD_FW_ELM0_MEM_ID                                                                     (34U)
#define CSL_STD_FW_ELM0_MEM_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_ELM0_MEM_MMR_BASE                                                               (0x00000045008800U)
#define CSL_STD_FW_ELM0_MEM_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_ELM0_MEM_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_ELM0_MEM_START                                                                  (0x00000025010000U)
#define CSL_STD_FW_ELM0_MEM_END                                                                    (0x00000025010fffU)

/* Properties of firewall at slave: CTRL_MMR0_CFG0 */
#define CSL_STD_FW_CTRL_MMR0_CFG0_ID                                                               (34U)
#define CSL_STD_FW_CTRL_MMR0_CFG0_TYPE                                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_CTRL_MMR0_CFG0_MMR_BASE                                                         (0x00000045008800U)
#define CSL_STD_FW_CTRL_MMR0_CFG0_NUM_REGIONS                                                      (8U)
#define CSL_STD_FW_CTRL_MMR0_CFG0_NUM_PRIV_IDS_PER_REGION                                          (3U)
#define CSL_STD_FW_CTRL_MMR0_CFG0_CFG0_START                                                       (0x00000043000000U)
#define CSL_STD_FW_CTRL_MMR0_CFG0_CFG0_END                                                         (0x0000004301ffffU)

/* Properties of firewall at slave: SA2_UL0_ECC_REGS */
#define CSL_STD_FW_SA2_UL0_ECC_REGS_ID                                                             (35U)
#define CSL_STD_FW_SA2_UL0_ECC_REGS_TYPE                                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_SA2_UL0_ECC_REGS_MMR_BASE                                                       (0x00000045008c00U)
#define CSL_STD_FW_SA2_UL0_ECC_REGS_NUM_REGIONS                                                    (8U)
#define CSL_STD_FW_SA2_UL0_ECC_REGS_NUM_PRIV_IDS_PER_REGION                                        (3U)
#define CSL_STD_FW_SA2_UL0_ECC_REGS_ECC_AGGR_START                                                 (0x00000000712000U)
#define CSL_STD_FW_SA2_UL0_ECC_REGS_ECC_AGGR_END                                                   (0x000000007123ffU)

/* Properties of firewall at slave: ADC0_ECCREGS */
#define CSL_STD_FW_ADC0_ECCREGS_ID                                                                 (35U)
#define CSL_STD_FW_ADC0_ECCREGS_TYPE                                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_ADC0_ECCREGS_MMR_BASE                                                           (0x00000045008c00U)
#define CSL_STD_FW_ADC0_ECCREGS_NUM_REGIONS                                                        (8U)
#define CSL_STD_FW_ADC0_ECCREGS_NUM_PRIV_IDS_PER_REGION                                            (3U)
#define CSL_STD_FW_ADC0_ECCREGS_ECC_REGS_START                                                     (0x0000000071a000U)
#define CSL_STD_FW_ADC0_ECCREGS_ECC_REGS_END                                                       (0x0000000071a3ffU)

/* Properties of firewall at slave: UART0_MEM */
#define CSL_STD_FW_UART0_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART0_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART0_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART0_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART0_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART0_MEM_START                                                                 (0x00000002800000U)
#define CSL_STD_FW_UART0_MEM_END                                                                   (0x000000028001ffU)

/* Properties of firewall at slave: UART1_MEM */
#define CSL_STD_FW_UART1_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART1_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART1_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART1_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART1_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART1_MEM_START                                                                 (0x00000002810000U)
#define CSL_STD_FW_UART1_MEM_END                                                                   (0x000000028101ffU)

/* Properties of firewall at slave: UART2_MEM */
#define CSL_STD_FW_UART2_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART2_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART2_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART2_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART2_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART2_MEM_START                                                                 (0x00000002820000U)
#define CSL_STD_FW_UART2_MEM_END                                                                   (0x000000028201ffU)

/* Properties of firewall at slave: UART3_MEM */
#define CSL_STD_FW_UART3_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART3_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART3_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART3_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART3_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART3_MEM_START                                                                 (0x00000002830000U)
#define CSL_STD_FW_UART3_MEM_END                                                                   (0x000000028301ffU)

/* Properties of firewall at slave: UART4_MEM */
#define CSL_STD_FW_UART4_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART4_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART4_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART4_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART4_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART4_MEM_START                                                                 (0x00000002840000U)
#define CSL_STD_FW_UART4_MEM_END                                                                   (0x000000028401ffU)

/* Properties of firewall at slave: UART5_MEM */
#define CSL_STD_FW_UART5_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART5_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART5_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART5_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART5_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART5_MEM_START                                                                 (0x00000002850000U)
#define CSL_STD_FW_UART5_MEM_END                                                                   (0x000000028501ffU)

/* Properties of firewall at slave: UART6_MEM */
#define CSL_STD_FW_UART6_MEM_ID                                                                    (35U)
#define CSL_STD_FW_UART6_MEM_TYPE                                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_UART6_MEM_MMR_BASE                                                              (0x00000045008c00U)
#define CSL_STD_FW_UART6_MEM_NUM_REGIONS                                                           (8U)
#define CSL_STD_FW_UART6_MEM_NUM_PRIV_IDS_PER_REGION                                               (3U)
#define CSL_STD_FW_UART6_MEM_START                                                                 (0x00000002860000U)
#define CSL_STD_FW_UART6_MEM_END                                                                   (0x000000028601ffU)

/* Properties of firewall at slave: I2C0_CFG */
#define CSL_STD_FW_I2C0_CFG_ID                                                                     (35U)
#define CSL_STD_FW_I2C0_CFG_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_I2C0_CFG_MMR_BASE                                                               (0x00000045008c00U)
#define CSL_STD_FW_I2C0_CFG_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_I2C0_CFG_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_I2C0_CFG_CFG_START                                                              (0x00000020000000U)
#define CSL_STD_FW_I2C0_CFG_CFG_END                                                                (0x000000200000ffU)

/* Properties of firewall at slave: I2C1_CFG */
#define CSL_STD_FW_I2C1_CFG_ID                                                                     (35U)
#define CSL_STD_FW_I2C1_CFG_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_I2C1_CFG_MMR_BASE                                                               (0x00000045008c00U)
#define CSL_STD_FW_I2C1_CFG_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_I2C1_CFG_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_I2C1_CFG_CFG_START                                                              (0x00000020010000U)
#define CSL_STD_FW_I2C1_CFG_CFG_END                                                                (0x000000200100ffU)

/* Properties of firewall at slave: I2C2_CFG */
#define CSL_STD_FW_I2C2_CFG_ID                                                                     (35U)
#define CSL_STD_FW_I2C2_CFG_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_I2C2_CFG_MMR_BASE                                                               (0x00000045008c00U)
#define CSL_STD_FW_I2C2_CFG_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_I2C2_CFG_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_I2C2_CFG_CFG_START                                                              (0x00000020020000U)
#define CSL_STD_FW_I2C2_CFG_CFG_END                                                                (0x000000200200ffU)

/* Properties of firewall at slave: I2C3_CFG */
#define CSL_STD_FW_I2C3_CFG_ID                                                                     (35U)
#define CSL_STD_FW_I2C3_CFG_TYPE                                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_I2C3_CFG_MMR_BASE                                                               (0x00000045008c00U)
#define CSL_STD_FW_I2C3_CFG_NUM_REGIONS                                                            (8U)
#define CSL_STD_FW_I2C3_CFG_NUM_PRIV_IDS_PER_REGION                                                (3U)
#define CSL_STD_FW_I2C3_CFG_CFG_START                                                              (0x00000020030000U)
#define CSL_STD_FW_I2C3_CFG_CFG_END                                                                (0x000000200300ffU)

/* Properties of firewall at slave: MCSPI0_CFG */
#define CSL_STD_FW_MCSPI0_CFG_ID                                                                   (35U)
#define CSL_STD_FW_MCSPI0_CFG_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_MCSPI0_CFG_MMR_BASE                                                             (0x00000045008c00U)
#define CSL_STD_FW_MCSPI0_CFG_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_MCSPI0_CFG_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_MCSPI0_CFG_CFG_START                                                            (0x00000020100000U)
#define CSL_STD_FW_MCSPI0_CFG_CFG_END                                                              (0x000000201003ffU)

/* Properties of firewall at slave: MCSPI1_CFG */
#define CSL_STD_FW_MCSPI1_CFG_ID                                                                   (35U)
#define CSL_STD_FW_MCSPI1_CFG_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_MCSPI1_CFG_MMR_BASE                                                             (0x00000045008c00U)
#define CSL_STD_FW_MCSPI1_CFG_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_MCSPI1_CFG_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_MCSPI1_CFG_CFG_START                                                            (0x00000020110000U)
#define CSL_STD_FW_MCSPI1_CFG_CFG_END                                                              (0x000000201103ffU)

/* Properties of firewall at slave: MCSPI2_CFG */
#define CSL_STD_FW_MCSPI2_CFG_ID                                                                   (35U)
#define CSL_STD_FW_MCSPI2_CFG_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_MCSPI2_CFG_MMR_BASE                                                             (0x00000045008c00U)
#define CSL_STD_FW_MCSPI2_CFG_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_MCSPI2_CFG_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_MCSPI2_CFG_CFG_START                                                            (0x00000020120000U)
#define CSL_STD_FW_MCSPI2_CFG_CFG_END                                                              (0x000000201203ffU)

/* Properties of firewall at slave: MCSPI3_CFG */
#define CSL_STD_FW_MCSPI3_CFG_ID                                                                   (35U)
#define CSL_STD_FW_MCSPI3_CFG_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_MCSPI3_CFG_MMR_BASE                                                             (0x00000045008c00U)
#define CSL_STD_FW_MCSPI3_CFG_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_MCSPI3_CFG_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_MCSPI3_CFG_CFG_START                                                            (0x00000020130000U)
#define CSL_STD_FW_MCSPI3_CFG_CFG_END                                                              (0x000000201303ffU)

/* Properties of firewall at slave: MCSPI4_CFG */
#define CSL_STD_FW_MCSPI4_CFG_ID                                                                   (35U)
#define CSL_STD_FW_MCSPI4_CFG_TYPE                                                                 (CSL_FW_SECURITY)
#define CSL_STD_FW_MCSPI4_CFG_MMR_BASE                                                             (0x00000045008c00U)
#define CSL_STD_FW_MCSPI4_CFG_NUM_REGIONS                                                          (8U)
#define CSL_STD_FW_MCSPI4_CFG_NUM_PRIV_IDS_PER_REGION                                              (3U)
#define CSL_STD_FW_MCSPI4_CFG_CFG_START                                                            (0x00000020140000U)
#define CSL_STD_FW_MCSPI4_CFG_CFG_END                                                              (0x000000201403ffU)

/* Properties of firewall at slave: MCAN0_MMR_MMRVBP_MCANSS_REGS */
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_ID                                                 (35U)
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_MMR_BASE                                           (0x00000045008c00U)
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_NUM_REGIONS                                        (8U)
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_SS_START                                           (0x00000020700000U)
#define CSL_STD_FW_MCAN0_MMR_MMRVBP_MCANSS_REGS_SS_END                                             (0x000000207000ffU)

/* Properties of firewall at slave: MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS */
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_ID                                       (35U)
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_MMR_BASE                                 (0x00000045008c00U)
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_NUM_REGIONS                              (8U)
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_CFG_START                                (0x00000020701000U)
#define CSL_STD_FW_MCAN0_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_CFG_END                                  (0x000000207011ffU)

/* Properties of firewall at slave: MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM */
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_ID                                             (35U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_MMR_BASE                                       (0x00000045008c00U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_NUM_REGIONS                                    (8U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_MSGMEM_RAM_START                               (0x00000020708000U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_MSGMEM_VBP_RAM_MSGMEM_RAM_END                                 (0x0000002070ffffU)

/* Properties of firewall at slave: MCAN1_MMR_MMRVBP_MCANSS_REGS */
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_ID                                                 (35U)
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_MMR_BASE                                           (0x00000045008c00U)
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_NUM_REGIONS                                        (8U)
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_SS_START                                           (0x00000020710000U)
#define CSL_STD_FW_MCAN1_MMR_MMRVBP_MCANSS_REGS_SS_END                                             (0x000000207100ffU)

/* Properties of firewall at slave: MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS */
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_ID                                       (35U)
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_MMR_BASE                                 (0x00000045008c00U)
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_NUM_REGIONS                              (8U)
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_CFG_START                                (0x00000020711000U)
#define CSL_STD_FW_MCAN1_MCAN_WRAP_MCAN_CFG_VBP_MCAN_REGS_CFG_END                                  (0x000000207111ffU)

/* Properties of firewall at slave: MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM */
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_ID                                             (35U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_TYPE                                           (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_MMR_BASE                                       (0x00000045008c00U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_NUM_REGIONS                                    (8U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_NUM_PRIV_IDS_PER_REGION                        (3U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_MSGMEM_RAM_START                               (0x00000020718000U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_MSGMEM_VBP_RAM_MSGMEM_RAM_END                                 (0x0000002071ffffU)

/* Properties of firewall at slave: MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS */
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_ID                                          (35U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_TYPE                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_MMR_BASE                                    (0x00000045008c00U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_NUM_REGIONS                                 (8U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_NUM_PRIV_IDS_PER_REGION                     (3U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_ECC_AGGR_START                              (0x00000024018000U)
#define CSL_STD_FW_MCAN0_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_ECC_AGGR_END                                (0x000000240183ffU)

/* Properties of firewall at slave: MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS */
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_ID                                          (35U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_TYPE                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_MMR_BASE                                    (0x00000045008c00U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_NUM_REGIONS                                 (8U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_NUM_PRIV_IDS_PER_REGION                     (3U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_ECC_AGGR_START                              (0x00000024019000U)
#define CSL_STD_FW_MCAN1_MSGMEM_WRAP_ECC_AGGR_VBP_REGS_ECC_AGGR_END                                (0x000000240193ffU)

/* Properties of firewall at slave: ADC0_ADC12_FIFO_DMA */
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_ID                                                          (35U)
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_TYPE                                                        (CSL_FW_SECURITY)
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_MMR_BASE                                                    (0x00000045008c00U)
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_NUM_REGIONS                                                 (8U)
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_NUM_PRIV_IDS_PER_REGION                                     (3U)
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_FIFO_START                                                  (0x00000028000000U)
#define CSL_STD_FW_ADC0_ADC12_FIFO_DMA_FIFO_END                                                    (0x000000280003ffU)

/* Properties of firewall at slave: ADC0_ADCREGS */
#define CSL_STD_FW_ADC0_ADCREGS_ID                                                                 (35U)
#define CSL_STD_FW_ADC0_ADCREGS_TYPE                                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_ADC0_ADCREGS_MMR_BASE                                                           (0x00000045008c00U)
#define CSL_STD_FW_ADC0_ADCREGS_NUM_REGIONS                                                        (8U)
#define CSL_STD_FW_ADC0_ADCREGS_NUM_PRIV_IDS_PER_REGION                                            (3U)
#define CSL_STD_FW_ADC0_ADCREGS_START                                                              (0x00000028001000U)
#define CSL_STD_FW_ADC0_ADCREGS_END                                                                (0x000000280013ffU)

/* Properties of firewall at slave: SA2_UL0_MMRS */
#define CSL_STD_FW_SA2_UL0_MMRS_ID                                                                 (35U)
#define CSL_STD_FW_SA2_UL0_MMRS_TYPE                                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_SA2_UL0_MMRS_MMR_BASE                                                           (0x00000045008c00U)
#define CSL_STD_FW_SA2_UL0_MMRS_NUM_REGIONS                                                        (8U)
#define CSL_STD_FW_SA2_UL0_MMRS_NUM_PRIV_IDS_PER_REGION                                            (3U)
#define CSL_STD_FW_SA2_UL0_MMRS_START                                                              (0x00000040900000U)
#define CSL_STD_FW_SA2_UL0_MMRS_END                                                                (0x00000040900fffU)

/* Properties of firewall at slave: SA2_UL0_MMRA_REGS */
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_ID                                                            (35U)
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_TYPE                                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_MMR_BASE                                                      (0x00000045008c00U)
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_NUM_REGIONS                                                   (8U)
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_NUM_PRIV_IDS_PER_REGION                                       (3U)
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_MMRA_START                                                    (0x00000040901000U)
#define CSL_STD_FW_SA2_UL0_MMRA_REGS_MMRA_END                                                      (0x000000409011ffU)

/* Properties of firewall at slave: SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS */
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_ID                                       (35U)
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_TYPE                                     (CSL_FW_SECURITY)
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_MMR_BASE                                 (0x00000045008c00U)
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_NUM_REGIONS                              (8U)
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_NUM_PRIV_IDS_PER_REGION                  (3U)
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_EIP_76_START                             (0x00000040910000U)
#define CSL_STD_FW_SA2_UL0_EIP_76D_8_BCDF_EIP76_REGISTERS_EIP_76_END                               (0x0000004091007fU)

/* Properties of firewall at slave: SA2_UL0_EIP_29T2_REGS */
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_ID                                                        (35U)
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_TYPE                                                      (CSL_FW_SECURITY)
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_MMR_BASE                                                  (0x00000045008c00U)
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_NUM_REGIONS                                               (8U)
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_NUM_PRIV_IDS_PER_REGION                                   (3U)
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_EIP_29T2_START                                            (0x00000040920000U)
#define CSL_STD_FW_SA2_UL0_EIP_29T2_REGS_EIP_29T2_END                                              (0x0000004092ffffU)

/* Properties of firewall at slave: DMSC0_ROM_0_IROM_SLV_ROM */
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_ID                                                     (512U)
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_MMR_BASE                                               (0x00000045080000U)
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_NUM_REGIONS                                            (2U)
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_NUM_PRIV_IDS_PER_REGION                                (1U)
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_IROM_SLV_ROM_START                                     (0x00000000000000U)
#define CSL_STD_FW_DMSC0_ROM_0_IROM_SLV_ROM_IROM_SLV_ROM_END                                       (0x0000000001ffffU)

/* Properties of firewall at slave: DMSC0_SRAM_0_DMSC_SRAM0 */
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_ID                                                      (513U)
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_TYPE                                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_MMR_BASE                                                (0x00000045080400U)
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_NUM_REGIONS                                             (8U)
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_NUM_PRIV_IDS_PER_REGION                                 (3U)
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_SRAM0_START                                             (0x00000044040000U)
#define CSL_STD_FW_DMSC0_SRAM_0_DMSC_SRAM0_SRAM0_END                                               (0x0000004404ffffU)

/* Properties of firewall at slave: DMSC0_SRAM_1_DMSC_SRAM1 */
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_ID                                                      (514U)
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_TYPE                                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_MMR_BASE                                                (0x00000045080800U)
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_NUM_REGIONS                                             (8U)
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_NUM_PRIV_IDS_PER_REGION                                 (3U)
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_SRAM1_START                                             (0x00000044050000U)
#define CSL_STD_FW_DMSC0_SRAM_1_DMSC_SRAM1_SRAM1_END                                               (0x0000004405ffffU)

/* Properties of firewall at slave: DMSC0_PWRCTRL_0_DMSC_PWR_MMR */
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_ID                                                 (528U)
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_MMR_BASE                                           (0x00000045084000U)
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_NUM_REGIONS                                        (1U)
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_PWR_START                                          (0x00000044130000U)
#define CSL_STD_FW_DMSC0_PWRCTRL_0_DMSC_PWR_MMR_PWR_END                                            (0x000000441307ffU)

/* Properties of firewall at slave: DMSC0_DMTIMER_0_DMSC_DMTIMER0 */
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_ID                                                (536U)
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_MMR_BASE                                          (0x00000045086000U)
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_NUM_REGIONS                                       (1U)
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_DMTIMER0_START                                    (0x00000044133000U)
#define CSL_STD_FW_DMSC0_DMTIMER_0_DMSC_DMTIMER0_DMTIMER0_END                                      (0x000000441333ffU)

/* Properties of firewall at slave: DMSC0_DMTIMER_1_DMSC_DMTIMER1 */
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_ID                                                (537U)
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_MMR_BASE                                          (0x00000045086400U)
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_NUM_REGIONS                                       (1U)
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_DMTIMER1_START                                    (0x00000044134000U)
#define CSL_STD_FW_DMSC0_DMTIMER_1_DMSC_DMTIMER1_DMTIMER1_END                                      (0x000000441343ffU)

/* Properties of firewall at slave: DMSC0_RTI_0_DMSC_RTI_MMR */
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_ID                                                     (544U)
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_TYPE                                                   (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_MMR_BASE                                               (0x00000045088000U)
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_NUM_REGIONS                                            (1U)
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_NUM_PRIV_IDS_PER_REGION                                (3U)
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_RTI_START                                              (0x00000044135100U)
#define CSL_STD_FW_DMSC0_RTI_0_DMSC_RTI_MMR_RTI_END                                                (0x000000441351ffU)

/* Properties of firewall at slave: DMSC0_WDTCTRL_0_DMSC_WDT_RTI */
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_ID                                                 (545U)
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_MMR_BASE                                           (0x00000045088400U)
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_NUM_REGIONS                                        (1U)
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_WDT_RTI_START                                      (0x00000044135000U)
#define CSL_STD_FW_DMSC0_WDTCTRL_0_DMSC_WDT_RTI_WDT_RTI_END                                        (0x000000441350ffU)

/* Properties of firewall at slave: DMSC0_RAT_0_DMSC_RAT_MMRS */
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_ID                                                    (552U)
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_TYPE                                                  (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_MMR_BASE                                              (0x0000004508a000U)
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_NUM_REGIONS                                           (1U)
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_NUM_PRIV_IDS_PER_REGION                               (3U)
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_RAT_START                                             (0x00000044200000U)
#define CSL_STD_FW_DMSC0_RAT_0_DMSC_RAT_MMRS_RAT_END                                               (0x00000044200fffU)

/* Properties of firewall at slave: DMSC0_ECC_AGGR_0_DMSC_ECC_REGS */
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_ID                                               (562U)
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_MMR_BASE                                         (0x0000004508c800U)
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_NUM_REGIONS                                      (2U)
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_ECC_AGGR_START                                   (0x0000003f009000U)
#define CSL_STD_FW_DMSC0_ECC_AGGR_0_DMSC_ECC_REGS_ECC_AGGR_END                                     (0x0000003f0093ffU)

/* Properties of firewall at slave: DMSC0_SECCTRL_0_DMSC_SEC_MMR */
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_ID                                                 (576U)
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_MMR_BASE                                           (0x00000045090000U)
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_NUM_REGIONS                                        (1U)
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_SEC_START                                          (0x00000044230000U)
#define CSL_STD_FW_DMSC0_SECCTRL_0_DMSC_SEC_MMR_SEC_END                                            (0x00000044230fffU)

/* Properties of firewall at slave: DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG */
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_ID                                            (578U)
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_TYPE                                          (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_MMR_BASE                                      (0x00000045090800U)
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_NUM_REGIONS                                   (1U)
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_NUM_PRIV_IDS_PER_REGION                       (3U)
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_DMSC_DBGAUTH_REG_START                        (0x00000044232000U)
#define CSL_STD_FW_DMSC0_DBG_AUTH_0_DMSC_DBGAUTH_REG_DMSC_DBGAUTH_REG_END                          (0x000000442320ffU)

/* Properties of firewall at slave: DMSC0_SEC_MGR_0_DMSC_SECMGR */
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_ID                                                  (582U)
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_TYPE                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_MMR_BASE                                            (0x00000045091800U)
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_NUM_REGIONS                                         (2U)
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_NUM_PRIV_IDS_PER_REGION                             (3U)
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_SECMGR_START                                        (0x00000044234000U)
#define CSL_STD_FW_DMSC0_SEC_MGR_0_DMSC_SECMGR_SECMGR_END                                          (0x00000044237fffU)

/* Properties of firewall at slave: DMSC0_DMTIMER_2_DMSC_DMTIMER2 */
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_ID                                                (592U)
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_MMR_BASE                                          (0x00000045094000U)
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_NUM_REGIONS                                       (1U)
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_DMTIMER2_START                                    (0x00000044238000U)
#define CSL_STD_FW_DMSC0_DMTIMER_2_DMSC_DMTIMER2_DMTIMER2_END                                      (0x000000442383ffU)

/* Properties of firewall at slave: DMSC0_DMTIMER_3_DMSC_DMTIMER3 */
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_ID                                                (593U)
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_TYPE                                              (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_MMR_BASE                                          (0x00000045094400U)
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_NUM_REGIONS                                       (1U)
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_NUM_PRIV_IDS_PER_REGION                           (3U)
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_DMTIMER3_START                                    (0x00000044239000U)
#define CSL_STD_FW_DMSC0_DMTIMER_3_DMSC_DMTIMER3_DMTIMER3_END                                      (0x000000442393ffU)

/* Properties of firewall at slave: DMSC0_AES_0_DMSC_AES38T */
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_ID                                                      (602U)
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_TYPE                                                    (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_MMR_BASE                                                (0x00000045096800U)
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_NUM_REGIONS                                             (2U)
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_NUM_PRIV_IDS_PER_REGION                                 (3U)
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_AES_START                                               (0x0000004423c000U)
#define CSL_STD_FW_DMSC0_AES_0_DMSC_AES38T_AES_END                                                 (0x0000004423dfffU)

/* Properties of firewall at slave: DMSC0_INTAGGR_0_DMSC_IA_INTR */
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_ID                                                 (607U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_MMR_BASE                                           (0x00000045097c00U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_NUM_REGIONS                                        (2U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_IA_INTR_START                                      (0x00000044300000U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_INTR_IA_INTR_END                                        (0x0000004437ffffU)

/* Properties of firewall at slave: DMSC0_INTAGGR_0_DMSC_IA_IMAP */
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_ID                                                 (607U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_TYPE                                               (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_MMR_BASE                                           (0x00000045097c00U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_NUM_REGIONS                                        (2U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_NUM_PRIV_IDS_PER_REGION                            (3U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_IA_MAP_START                                       (0x00000044400000U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_IMAP_IA_MAP_END                                         (0x000000444007ffU)

/* Properties of firewall at slave: DMSC0_INTAGGR_0_DMSC_IA_CFG */
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_ID                                                  (607U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_TYPE                                                (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_MMR_BASE                                            (0x00000045097c00U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_NUM_REGIONS                                         (2U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_NUM_PRIV_IDS_PER_REGION                             (3U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_IA_CFG_START                                        (0x00000044410000U)
#define CSL_STD_FW_DMSC0_INTAGGR_0_DMSC_IA_CFG_IA_CFG_END                                          (0x0000004441001fU)

/* Properties of firewall at slave: DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG */
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_ID                                               (639U)
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_TYPE                                             (CSL_FW_SECURITY)
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_MMR_BASE                                         (0x0000004509fc00U)
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_NUM_REGIONS                                      (8U)
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_NUM_PRIV_IDS_PER_REGION                          (3U)
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_FW_START                                         (0x00000045000000U)
#define CSL_STD_FW_DMSC0_FWMGR_0_DMSC_FW_VBUS_CFG_FW_END                                           (0x00000045ffffffU)

/* Channelized Firewall Definitions */

#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_FW_H_ */

