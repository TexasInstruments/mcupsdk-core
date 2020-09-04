/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_xge_cpsw_ss_s.h
*/
#ifndef CSLR_XGE_CPSW_SS_S_H_
#define CSLR_XGE_CPSW_SS_S_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* The following are base address byte offsets to various modules in the
* CPSW subsystem.
**************************************************************************/
#define CPSW_NUSS_OFFSET    (0x00000000U)                   /* CPSW_NUSS offset memory map */
#define CPSW_SGMII0_OFFSET(n)  (0x00000100U+((n)*0x100U))   /* SGMII0 offset memory map */
#define CPSW_MDIO_OFFSET    (0x00000F00U)                   /* MDIO offset memory map */
#define CPSW_REGS_INT_OFFSET    (0x00001800U)                   /* REGS_INT offset memory map */
#define CPSW_NU_OFFSET      (0x00020000U)                   /* CPSW offset memory map */
#define CPSW_CPPI_OFFSET    (CPSW_NU_OFFSET+0x00001000U)    /* CPPI offset memory map */
#define CPSW_ETHX_OFFSET(n) (CPSW_NU_OFFSET+0x00002000U+((n)*0x1000U))    /* Ethernet_X port n offset memory map */
#define CPSW_ETHX0_OFFSET   (CPSW_NU_OFFSET+0x00002000U)    /* Ethernet_X port 0 offset memory map */
#define CPSW_ETHX1_OFFSET   (CPSW_NU_OFFSET+0x00003000U)    /* Ethernet_X port 1 offset memory map */
#define CPSW_EST_OFFSET         (CPSW_NU_OFFSET+0x00012000U)    /* EST offset memory map */
#define CPSW_CPDMA_OFFSET       (CPSW_NU_OFFSET+0x00014000U)    /* CPDMA offset memory map */
#define CPSW_CPDMA_INT_OFFSET   (CPSW_NU_OFFSET+0x00014080U)    /* CPDMA_INT offset memory map */
#define CPSW_CPDMA_SRAM_OFFSET  (CPSW_NU_OFFSET+0x00014200U)    /* CPDMA_SRAM offset memory map */
#define CPSW_STAT0_OFFSET   (CPSW_NU_OFFSET+0x0001A000U)    /* STAT0 offset memory map */
#define CPSW_STAT1_OFFSET   (CPSW_NU_OFFSET+0x0001A200U)    /* STAT1 offset memory map */
#define CPSW_STAT2_OFFSET   (CPSW_NU_OFFSET+0x0001A400U)    /* STAT2 offset memory map */
#define CPSW_CPTS_OFFSET    (CPSW_NU_OFFSET+0x0001D000U)    /* CPTS offset memory map */
#define CPSW_ALE_OFFSET     (CPSW_NU_OFFSET+0x0001E000U)    /* ALE offset memory map */
#define CPSW_ECC_OFFSET     (CPSW_NU_OFFSET+0x0001F000U)    /* CPSW_ECC offset memory map */

/**************************************************************************
* Hardware Region  : CPSW_NUSS Subsystem registers
**************************************************************************/

/**************************************************************************
* Register Overlay Structure for core_int_en
**************************************************************************/
typedef struct {
    volatile uint32_t RX_THRESH_EN;     /* Rx Threshold Pulse Interrupt Enable Register */
    volatile uint32_t RX_EN;	        /* Rx Pulse Interrupt Enable Register */
    volatile uint32_t TX_EN;	        /* Tx Pulse Interrupt Enable Register */
    volatile uint32_t MISC_EN;	        /* Misc Interrupt Enable Register */
} CSL_WrCore_int_enRegs;


/**************************************************************************
* Register Overlay Structure for core_int_stat
**************************************************************************/
typedef struct {
    volatile uint32_t RX_THRESH_STAT;   /* Rx Threshold Pulse Interrupt Status Register */
    volatile uint32_t RX_STAT;	        /* Rx Pulse Interrupt Status Register */
    volatile uint32_t TX_STAT;	        /* Tx Pulse Interrupt Status Register */
    volatile uint32_t MISC_STAT;	    /* Misc Interrupt Status Register */
} CSL_WrCore_int_statRegs;

/**************************************************************************
* Register Overlay Structure for core_ints_per_ms
**************************************************************************/
typedef struct {
    volatile uint32_t RX_IMAX;          /* Rx Interrupt Max Register Register */  	     
    volatile uint32_t TX_IMAX;          /* Tx Interrupt Max Register Register */
} CSL_WrCore_ints_per_msRegs;

/**************************************************************************
* Register Overlay Structure for all interrupts in one core
**************************************************************************/
typedef struct {
    CSL_WrCore_int_enRegs       CORE_INT_EN;      /* Core Interrupt Enable Registers */
    CSL_WrCore_int_statRegs     CORE_INT_STAT;    /* Core Interrupt Status Registers */
    CSL_WrCore_ints_per_msRegs  CORE_INTS_PER_MS; /* Core Interrupt Rate Limit Registers */
	volatile uint8_t Resv[24];
} CSL_WrCore_int_Regs;

typedef struct {
    volatile uint32_t IDVER_REG;              /* ID Version Register */
    volatile uint32_t SYNCE_COUNT_REG;        /* SyncE Count Register */
    volatile uint32_t SYNCE_MUX_REG;          /* SyncE Mux Register */
    volatile uint32_t CONTROL_REG;            /* Control Register */
    volatile uint8_t  Resv_24[8];
    volatile uint32_t INT_CONTROL_REG;        /* Interrupt Control Register */
    volatile uint32_t SUBSYSTEM_STATUS_REG;   /* Subsystem Status Register */
    volatile uint32_t SUBSYSTEM_CONFIG_REG;   /* Subsystem Configuration Register */
    volatile uint8_t  Resv_48[12];
    volatile uint32_t RGMII1_STATUS_REG;      /* RGMII1 Status Register */
    volatile uint32_t RGMII2_STATUS_REG;      /* RGMII2 Status Register */
    volatile uint8_t  Resv_6144[6088];
	CSL_WrCore_int_Regs CORE_INT[4];
} CSL_Xge_cpsw_ss_sRegs;

/**************************************************************************
* Register Macros
**************************************************************************/
#define CSL_XGE_CPSW_SS_S_IDVER_REG                 (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SYNCE_COUNT_REG           (0x00000004U)
#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG             (0x00000008U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG               (0x0000000CU)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG           (0x00000018U)
#define CSL_XGE_CPSW_SS_S_SUBSSYSTEM_STATUS_REG     (0x0000001CU)
#define CSL_XGE_CPSW_SS_S_SUBSSYSTEM_CONFIG_REG     (0x00000020U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG         (0x00000030U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG         (0x00000034U)
#define CSL_XGE_CPSW_SS_S_WR_RX_THRESH_EN(n)        (0x80U + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_RX_EN(n)               (0x84U + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_TX_EN(n)               (0x88U + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_MISC_EN(n)             (0x8CU + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_RX_THRESH_STAT(n)      (0xB0U + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_RX_STAT(n)             (0xB4U + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_TX_STAT(n)             (0xB8U + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_MISC_STAT(n)           (0xBCU + ((n) * (10U)))
#define CSL_XGE_CPSW_SS_S_WR_RX_IMAX(n)             (0xE0U + ((n) * (8U)))
#define CSL_XGE_CPSW_SS_S_WR_TX_IMAX(n)             (0xE4U + ((n) * (8U)))

/**************************************************************************
* Field Definition Macros
**************************************************************************/
/* RX_THRESH_EN */

#define CSL_WR_RX_THRESH_EN_RX_THRESH_EN_SHIFT                  (0x00000000U)
#define CSL_WR_RX_THRESH_EN_RX_THRESH_EN_MASK                   (0x000000FFU)
#define CSL_WR_RX_THRESH_EN_RX_THRESH_EN_RESETVAL               (0x00000000U)
#define CSL_WR_RX_THRESH_EN_RX_THRESH_EN_MAX                    (0x000000FFU)

#define CSL_WR_RX_THRESH_EN_RESETVAL                            (0x00000000U)

/* RX_EN */

#define CSL_WR_RX_EN_RX_EN_SHIFT                                (0x00000000U)
#define CSL_WR_RX_EN_RX_EN_MASK                                 (0x000000FFU)
#define CSL_WR_RX_EN_RX_EN_RESETVAL                             (0x00000000U)
#define CSL_WR_RX_EN_RX_EN_MAX                                  (0x000000FFU)

#define CSL_WR_RX_EN_RESETVAL                                   (0x00000000U)

/* TX_EN */

#define CSL_WR_TX_EN_TX_EN_SHIFT                                (0x00000000U)
#define CSL_WR_TX_EN_TX_EN_MASK                                 (0x000000FFU)
#define CSL_WR_TX_EN_TX_EN_RESETVAL                             (0x00000000U)
#define CSL_WR_TX_EN_TX_EN_MAX                                  (0x000000FFU)

#define CSL_WR_TX_EN_RESETVAL                                   (0x00000000U)

/* MISC_EN */
#define CSL_WR_MISC_EN_MDIO_USERINT_MASK                        (0x00000001U)
#define CSL_WR_MISC_EN_MDIO_USERINT_SHIFT                       (0x00000000U)
#define CSL_WR_MISC_EN_MDIO_USERINT_RESETVAL                    (0x00000000U)
#define CSL_WR_MISC_EN_MDIO_USERINT_MAX                         (0x00000001U)

#define CSL_WR_MISC_EN_MDIO_LINKINT_MASK                        (0x00000002U)
#define CSL_WR_MISC_EN_MDIO_LINKINT_SHIFT                       (0x00000001U)
#define CSL_WR_MISC_EN_MDIO_LINKINT_RESETVAL                    (0x00000000U)
#define CSL_WR_MISC_EN_MDIO_LINKINT_MAX                         (0x00000001U)

#define CSL_WR_MISC_EN_HOST_PEND_MASK                           (0x00000004U)
#define CSL_WR_MISC_EN_HOST_PEND_SHIFT                          (0x00000002U)
#define CSL_WR_MISC_EN_HOST_PEND_RESETVAL                       (0x00000000U)
#define CSL_WR_MISC_EN_HOST_PEND_MAX                            (0x00000001U)

#define CSL_WR_MISC_EN_STAT_PEND_MASK                           (0x00000008U)
#define CSL_WR_MISC_EN_STAT_PEND_SHIFT                          (0x00000003U)
#define CSL_WR_MISC_EN_STAT_PEND_RESETVAL                       (0x00000000U)
#define CSL_WR_MISC_EN_STAT_PEND_MAX                            (0x00000001U)

#define CSL_WR_MISC_EN_EVNT_PEND_MASK                           (0x00000010U)
#define CSL_WR_MISC_EN_EVNT_PEND_SHIFT                          (0x00000004U)
#define CSL_WR_MISC_EN_EVNT_PEND_RESETVAL                       (0x00000000U)
#define CSL_WR_MISC_EN_EVNT_PEND_MAX                            (0x00000001U)

#define CSL_WR_MISC_EN_SEC_PEND_MASK                            (0x00000020U)
#define CSL_WR_MISC_EN_SEC_PEND_SHIFT                           (0x00000005U)
#define CSL_WR_MISC_EN_SEC_PEND_RESETVAL                        (0x00000000U)
#define CSL_WR_MISC_EN_SEC_PEND_MAX                             (0x00000001U)

#define CSL_WR_MISC_EN_DED_PEND_MASK                            (0x00000040U)
#define CSL_WR_MISC_EN_DED_PEND_SHIFT                           (0x00000006U)
#define CSL_WR_MISC_EN_DED_PEND_RESETVAL                        (0x00000000U)
#define CSL_WR_MISC_EN_DED_PEND_MAX                             (0x00000001U)

#define CSL_WR_MISC_EN_RESETVAL                                 (0x00000000U)

/* RX_THRESH_STAT */

#define CSL_WR_RX_THRESH_STAT_RX_THRESH_STAT_SHIFT              (0x00000000U)
#define CSL_WR_RX_THRESH_STAT_RX_THRESH_STAT_MASK               (0x000000FFU)
#define CSL_WR_RX_THRESH_STAT_RX_THRESH_STAT_RESETVAL           (0x00000000U)
#define CSL_WR_RX_THRESH_STAT_RX_THRESH_STAT_MAX                (0x000000FFU)

#define CSL_WR_RX_THRESH_STAT_RESETVAL                          (0x00000000U)

/* RX_STAT */

#define CSL_WR_RX_STAT_RX_STAT_SHIFT                            (0x00000000U)
#define CSL_WR_RX_STAT_RX_STAT_MASK                             (0x000000FFU)
#define CSL_WR_RX_STAT_RX_STAT_RESETVAL                         (0x00000000U)
#define CSL_WR_RX_STAT_RX_STAT_MAX                              (0x000000FFU)

#define CSL_WR_RX_STAT_RESETVAL                                 (0x00000000U)

/* TX_STAT */

#define CSL_WR_TX_STAT_TX_STAT_SHIFT                            (0x00000000U)
#define CSL_WR_TX_STAT_TX_STAT_MASK                             (0x000000FFU)
#define CSL_WR_TX_STAT_TX_STAT_RESETVAL                         (0x00000000U)
#define CSL_WR_TX_STAT_TX_STAT_MAX                              (0x000000FFU)

#define CSL_WR_TX_STAT_RESETVAL                                 (0x00000000U)

/* MISC_STAT */

#define CSL_WR_MISC_STAT_MDIO_USERINT_MASK                      (0x00000001U)
#define CSL_WR_MISC_STAT_MDIO_USERINT_SHIFT                     (0x00000000U)
#define CSL_WR_MISC_STAT_MDIO_USERINT_RESETVAL                  (0x00000000U)
#define CSL_WR_MISC_STAT_MDIO_USERINT_MAX                       (0x00000001U)

#define CSL_WR_MISC_STAT_MDIO_LINKINT_MASK                      (0x00000002U)
#define CSL_WR_MISC_STAT_MDIO_LINKINT_SHIFT                     (0x00000001U)
#define CSL_WR_MISC_STAT_MDIO_LINKINT_RESETVAL                  (0x00000000U)
#define CSL_WR_MISC_STAT_MDIO_LINKINT_MAX                       (0x00000001U)

#define CSL_WR_MISC_STAT_HOST_PEND_MASK                         (0x00000004U)
#define CSL_WR_MISC_STAT_HOST_PEND_SHIFT                        (0x00000002U)
#define CSL_WR_MISC_STAT_HOST_PEND_RESETVAL                     (0x00000000U)
#define CSL_WR_MISC_STAT_HOST_PEND_MAX                          (0x00000001U)

#define CSL_WR_MISC_STAT_STAT_PEND_MASK                         (0x00000008U)
#define CSL_WR_MISC_STAT_STAT_PEND_SHIFT                        (0x00000003U)
#define CSL_WR_MISC_STAT_STAT_PEND_RESETVAL                     (0x00000000U)
#define CSL_WR_MISC_STAT_STAT_PEND_MAX                          (0x00000001U)

#define CSL_WR_MISC_STAT_EVNT_PEND_MASK                         (0x00000010U)
#define CSL_WR_MISC_STAT_EVNT_PEND_SHIFT                        (0x00000004U)
#define CSL_WR_MISC_STAT_EVNT_PEND_RESETVAL                     (0x00000000U)
#define CSL_WR_MISC_STAT_EVNT_PEND_MAX                          (0x00000001U)

#define CSL_WR_MISC_STAT_SEC_PEND_MASK                          (0x00000020U)
#define CSL_WR_MISC_STAT_SEC_PEND_SHIFT                         (0x00000005U)
#define CSL_WR_MISC_STAT_SEC_PEND_RESETVAL                      (0x00000000U)
#define CSL_WR_MISC_STAT_SEC_PEND_MAX                           (0x00000001U)

#define CSL_WR_MISC_STAT_DED_PEND_MASK                          (0x00000040U)
#define CSL_WR_MISC_STAT_DED_PEND_SHIFT                         (0x00000006U)
#define CSL_WR_MISC_STAT_DED_PEND_RESETVAL                      (0x00000000U)
#define CSL_WR_MISC_STAT_DED_PEND_MAX                           (0x00000001U)

#define CSL_WR_MISC_STAT_RESETVAL                               (0x00000000U)

/* RX_IMAX */

#define CSL_WR_RX_IMAX_RX_IMAX_SHIFT                            (0x00000000U)
#define CSL_WR_RX_IMAX_RX_IMAX_MASK                             (0x0000003FU)
#define CSL_WR_RX_IMAX_RX_IMAX_RESETVAL                         (0x00000000U)
#define CSL_WR_RX_IMAX_RX_IMAX_MAX                              (0x0000003FU)

#define CSL_WR_RX_IMAX_RESETVAL                                 (0x00000000U)

/* TX_IMAX */

#define CSL_WR_TX_IMAX_TX_IMAX_SHIFT                            (0x00000000U)
#define CSL_WR_TX_IMAX_TX_IMAX_MASK                             (0x0000003FU)
#define CSL_WR_TX_IMAX_TX_IMAX_RESETVAL                         (0x00000000U)
#define CSL_WR_TX_IMAX_TX_IMAX_MAX                              (0x0000003FU)

#define CSL_WR_TX_IMAX_RESETVAL                                 (0x00000000U)

/* IDVER_REG */

#define CSL_XGE_CPSW_SS_S_IDVER_REG_MINOR_VER_MASK                   (0x000000FFU)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_MINOR_VER_SHIFT                  (0x00000000U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_MINOR_VER_RESETVAL               (0x00000003U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_MINOR_VER_MAX                    (0x000000FFU)

#define CSL_XGE_CPSW_SS_S_IDVER_REG_MAJOR_VER_MASK                   (0x00000700U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_MAJOR_VER_SHIFT                  (0x00000008U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_MAJOR_VER_RESETVAL               (0x00000001U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_MAJOR_VER_MAX                    (0x00000007U)

#define CSL_XGE_CPSW_SS_S_IDVER_REG_RTL_VER_MASK                     (0x0000F800U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_RTL_VER_SHIFT                    (0x0000000BU)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_RTL_VER_RESETVAL                 (0x00000003U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_RTL_VER_MAX                      (0x0000001FU)

#define CSL_XGE_CPSW_SS_S_IDVER_REG_IDENT_MASK                       (0xFFFF0000U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_IDENT_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_IDENT_RESETVAL                   (0x00006BA0U)
#define CSL_XGE_CPSW_SS_S_IDVER_REG_IDENT_MAX                        (0x0000FFFFU)

#define CSL_XGE_CPSW_SS_S_IDVER_REG_RESETVAL                         (0x6BA01903U)

/* SYNCE_COUNT_REG */

#define CSL_XGE_CPSW_SS_S_SYNCE_COUNT_REG_SYNCE_CNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_SS_S_SYNCE_COUNT_REG_SYNCE_CNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SYNCE_COUNT_REG_SYNCE_CNT_RESETVAL                   (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SYNCE_COUNT_REG_SYNCE_CNT_MAX                        (0xFFFFFFFFU)

#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG_RESETVAL                               (0x00000000U)

/* SYNCE_MUX_REG */

#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG_SYNCE_SEL_MASK                         (0x0000003FU)
#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG_SYNCE_SEL_SHIFT                        (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG_SYNCE_SEL_RESETVAL                     (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG_SYNCE_SEL_MAX                          (0x0000003FU)

#define CSL_XGE_CPSW_SS_S_SYNCE_MUX_REG_RESETVAL                               (0x00000000U)

/* CONTROL_REG */

#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_EN_MASK                              (0x00000001U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_EN_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_EN_RESETVAL                          (0x00000000U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_EN_MAX                               (0x00000001U)

#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_PHY_ONLY_MASK                        (0x00000002U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_PHY_ONLY_SHIFT                       (0x00000001U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_PHY_ONLY_RESETVAL                    (0x00000000U)
#define CSL_XGE_CPSW_SS_S_CONTROL_REG_EEE_PHY_ONLY_MAX                         (0x00000001U)

#define CSL_XGE_CPSW_SS_S_CONTROL_REG_RESETVAL                                 (0x00000000U)

/* SS_INT_CONTROL_REG */

#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_PRESCALE_MASK                    (0x00000FFFU)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_PRESCALE_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_PRESCALE_RESETVAL                (0x00000000U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_PRESCALE_MAX                     (0x00000FFFU)

#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_BYPASS_MASK                      (0x003F0000U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_BYPASS_SHIFT                     (0x00000010U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_BYPASS_RESETVAL                  (0x00000000U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_BYPASS_MAX                       (0x0000003FU)

#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_TEST_MASK                        (0x80000000U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_TEST_SHIFT                       (0x0000001FU)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_TEST_RESETVAL                    (0x00000000U)
#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_INT_TEST_MAX                         (0x00000001U)

#define CSL_XGE_CPSW_SS_S_INT_CONTROL_REG_RESETVAL                             (0x00000000U)

/* SUBSSYSTEM_STATUS_REG */

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_MASK            (0x00000001U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_SHIFT           (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_RESETVAL        (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_MAX             (0x00000001U)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_STATUS_REG_RESETVAL                        (0x00000000U)

/* SUBSYSTEM_CONFIG_REG */

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_PORTS_MASK                  (0x000000FFU)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_PORTS_SHIFT                 (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_PORTS_RESETVAL              (0x00000003U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_PORTS_MAX                   (0x000000FFU)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_GENF_MASK                   (0x00001F00U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_GENF_SHIFT                  (0x00000008U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_GENF_RESETVAL               (0x00000002U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_GENF_MAX                    (0x0000001FU)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RMII_MASK                       (0x00010000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RMII_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RMII_RESETVAL                   (0x00000001U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RMII_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RGMII_MASK                      (0x00020000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RGMII_SHIFT                     (0x00000011U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RGMII_RESETVAL                  (0x00000001U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RGMII_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_SGMII_MASK                      (0x00040000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_SGMII_SHIFT                     (0x00000012U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_SGMII_RESETVAL                  (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_SGMII_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_QSGMII_MASK                     (0x00080000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_QSGMII_SHIFT                    (0x00000013U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_QSGMII_RESETVAL                 (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_QSGMII_MAX                      (0x00000001U)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_XGMII_MASK                      (0x0FF00000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_XGMII_SHIFT                     (0x00000014U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_XGMII_RESETVAL                  (0x00000000U)
#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_XGMII_MAX                       (0x000000FFU)

#define CSL_XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RESETVAL                        (0x00030203U)

/* RGMII1_STATUS_REG */

#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK_MASK                          (0x00000001U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK_SHIFT                         (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK_RESETVAL                      (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED_MASK                         (0x00000006U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED_SHIFT                        (0x00000001U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED_RESETVAL                     (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED_MAX                          (0x00000003U)

#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX_MASK                    (0x00000008U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX_SHIFT                   (0x00000003U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX_RESETVAL                (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_SS_S_RGMII1_STATUS_REG_RESETVAL                           (0x00000000U)

/* RGMII2_STATUS_REG */

#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_LINK_MASK                          (0x00000001U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_LINK_SHIFT                         (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_LINK_RESETVAL                      (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_LINK_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_SPEED_MASK                         (0x00000006U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_SPEED_SHIFT                        (0x00000001U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_SPEED_RESETVAL                     (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_SPEED_MAX                          (0x00000003U)

#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_FULLDUPLEX_MASK                    (0x00000008U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_FULLDUPLEX_SHIFT                   (0x00000003U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_FULLDUPLEX_RESETVAL                (0x00000000U)
#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_FULLDUPLEX_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_SS_S_RGMII2_STATUS_REG_RESETVAL                           (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
