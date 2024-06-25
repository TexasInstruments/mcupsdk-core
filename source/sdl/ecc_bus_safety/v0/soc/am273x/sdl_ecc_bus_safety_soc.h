/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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

/**
@addtogroup SDL_ECC_BUS_SAFETY_MACROS
@{
*/

#ifndef SDL_ECC_BUS_SAFETY_SOC_H_
#define SDL_ECC_BUS_SAFETY_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am273x/sdlr_soc_baseaddress.h>
#include <sdl/include/am273x/sdlr_mss_ctrl.h>
#include <sdl/include/am273x/sdlr_dss_ctrl.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
/* MSS CTRL BASE Address */
#define SDL_ECC_BUS_SAFETY_MSS_BUS_CFG           (uint32_t)SDL_MSS_CTRL_U_BASE
/* DSS CTRL base Address */
#define SDL_ECC_BUS_SAFETY_DSS_BUS_CFG           (uint32_t)SDL_DSS_CTRL_U_BASE
#define DWORD                                    (0x20U)
/* Adress of different memory section */
/* DSS_CMC */
#define  SDL_DSS_CMC_COMP_U_END                  (SDL_DSS_CMC_COMP_U_BASE + 0X3FFCU-DWORD)
/* DSS MCRC */
#define  SDL_DSS_MCRC_U_END                      (SDL_DSS_MCRC_U_BASE + 0x144U-DWORD)
/* DSS CBUFF FIFO */
#define  SDL_DSS_CBUFF_FIFO_U_END                (SDL_DSS_CBUFF_FIFO_U_BASE + 0X3FFCU-DWORD)
/* DSS MDO FIFO */
#define SDL_DSS_MDO_FIFO_U_END                   (SDL_DSS_MDO_FIFO_U_BASE + SDL_DSS_MDO_FIFO_U_SIZE-DWORD)
/* DSS BANK A */
#define SDL_DSS_L3_BANKA_ADDRESS                 SDL_DSS_L3_U_BASE
#define SDL_DSS_L3_BANK_SIZE                     (0x100000U)
/* DSS BANK B */
#define SDL_DSS_L3_BANKB_ADDRESS                 SDL_DSS_L3_BANKA_ADDRESS+SDL_DSS_L3_BANK_SIZE
/* DSS BANK C */
#define SDL_DSS_L3_BANKC_ADDRESS                 SDL_DSS_L3_BANKB_ADDRESS+SDL_DSS_L3_BANK_SIZE
/* DSS BANK D */
#define SDL_DSS_L3_BANKD_ADDRESS                 SDL_DSS_L3_BANKC_ADDRESS+SDL_DSS_L3_BANK_SIZE
/* DSS L3 */
#define SDL_DSS_L3_END_ADDRESS                   (SDL_DSS_L3_U_BASE+SDL_DSS_L3_U_SIZE)
/* DSS BANK A END Address*/
#define SDL_DSS_L3_BANKA_ADDRESS_END             (SDL_DSS_L3_BANKB_ADDRESS-DWORD)
/* DSS BANK B END Address*/
#define SDL_DSS_L3_BANKB_ADDRESS_END             (SDL_DSS_L3_BANKC_ADDRESS-DWORD)
/* DSS BANK C END Address*/
#define SDL_DSS_L3_BANKC_ADDRESS_END             (SDL_DSS_L3_BANKD_ADDRESS-DWORD)
/* DSS BANK D END Address*/
#define SDL_DSS_L3_BANKD_ADDRESS_END             (SDL_DSS_L3_END_ADDRESS-DWORD)
/* DSS HWA DMA 0 */
#define SDL_DSS_HWA_DMA0_U_BASE_END              (SDL_DSS_HWA_DMA0_U_BASE+SDL_DSS_HWA_DMA0_U_SIZE-DWORD)
/* DSS HWA DMA 1 */
#define SDL_DSS_HWA_DMA1_U_BASE_END              (SDL_DSS_HWA_DMA1_U_BASE+SDL_DSS_HWA_DMA1_U_SIZE-DWORD)
/* DSS MAILBOX*/
#define SDL_DSS_MAILBOX_U_BASE_END               (SDL_DSS_MAILBOX_U_BASE+SDL_DSS_MAILBOX_U_SIZE-DWORD)
/*  DSS L2*/
#define SDL_DSS_L2_U_BASE_END                    (SDL_DSS_L2_U_BASE+SDL_DSS_L2_U_SIZE-DWORD)
/* MSS DMM A DATA */
#define SDL_MSS_DMM_A_DATA_U_BASE_END            (SDL_MSS_DMM_A_DATA_U_BASE+0x90U-DWORD)
/* MSS GPADC DATA RAM */
#define SDL_MSS_GPADC_DATA_RAM_U_BASE_END        (SDL_MSS_GPADC_DATA_RAM_U_BASE+SDL_MSS_GPADC_DATA_RAM_U_SIZE-DWORD)
/* MSS L2 */
#define SDL_MSS_L2_U_BASE_END                    (SDL_MSS_L2_U_BASE+SDL_MSS_L2_U_SIZE-DWORD)
/* MSS L2 A */
#define SDL_MSS_L2_A_BASE_START                  (0x10200000U)
#define SDL_MSS_L2_A_BASE_END                    (SDL_MSS_L2_A_BASE_START+0X7FFFCU-DWORD)
/* MSS L2 B */
#define SDL_MSS_L2_B_BASE_START                  (0x102E0000U)
#define SDL_MSS_L2_B_BASE_END                    (SDL_MSS_L2_B_BASE_START+0X6FFFCU-DWORD)
/* MSS DMM */
#define SDL_MSS_DMM_A_DATA_U_BASE                (0xCD000000U)
#define SDL_MSS_DMM_B_DATA_U_BASE                (0xCD010000U)
/* MSS CR5A AHB */
#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE        (SDL_MSS_CTRL_R5A_AHB_BASE )
#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_END         (SDL_MSS_CTRL_R5A_AHB_BASE + SDL_MSS_CTRL_R5A_AHB_SIZE-DWORD)
/* MSS CR5B AHB */
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE        (SDL_MSS_CTRL_R5B_AHB_BASE )
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_END         (SDL_MSS_CTRL_R5B_AHB_BASE + SDL_MSS_CTRL_R5B_AHB_SIZE-DWORD)
/* MSS MBOX */
#define SDL_MSS_MBOX_U_END                       (SDL_MSS_MBOX_U_BASE+ 0x00001FFCU-DWORD)
/* MSS QSPI */
#define SDL_MSS_QSPI_U_END                       (0xC8000070U-DWORD)
/* MSS MCRC */
#define SDL_MSS_MCRC_U_SIZE                      (0x00000144U)
#define SDL_MSS_MCRC_U_END                       (SDL_MSS_MCRC_U_BASE + SDL_MSS_MCRC_U_SIZE-DWORD)
/* MSS SWBUF */
#define SDL_MSS_SWBUF_U_BASE                     (SDL_MSS_RETRAM_U_BASE)
#define SDL_MSS_SWBUF_U_SIZE                     (0x000007FCU)
#define SDL_MSS_SWBUF_U_END                      (SDL_MSS_SWBUF_U_BASE + SDL_MSS_SWBUF_U_SIZE-DWORD)
/* MSS TO MDO */
#define SDL_MSS_TO_MDO_U_BASE                    (0xCA000000U)
#define SDL_MSS_TO_MDO_U_END                     (0xCA00FFFCU-DWORD)

/* Macro defines Ecc Bus Safety Nodes in the DSS Subsystem */
#define SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA        0U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKA        1U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKB        2U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKC        3U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKD        4U
#define SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA        5U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD      6U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD      7U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD      8U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD      9U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD      10U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD      11U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD      12U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD      13U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD      14U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD      15U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR      16U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR      17U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR      18U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR      19U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR      20U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR      21U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR      22U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR      23U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR      24U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR      25U
#define SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO      26U
#define SDL_ECC_BUS_SAFETY_DSS_MCRC            27U
#define SDL_ECC_BUS_SAFETY_DSS_PCR             28U
#define SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0        29U
#define SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1        30U
#define SDL_ECC_BUS_SAFETY_DSS_MBOX            31U
#define SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO        32U

/* Macro defines Ecc Bus Safety Nodes in the MSS Subsystem */
#define SDL_ECC_BUS_SAFETY_MSS_MBOX            0U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD      1U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD      2U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD      3U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR      4U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR      5U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR      6U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB        7U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB        8U
#define SDL_ECC_BUS_SAFETY_MSS_SCRP            9U
#define SDL_ECC_BUS_SAFETY_MSS_DMM             10U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR     11U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR     12U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD     13U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD     14U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S      15U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S      16U
#define SDL_ECC_BUS_SAFETY_MSS_QSPI            17U
#define SDL_ECC_BUS_SAFETY_MSS_CPSW            18U
#define SDL_ECC_BUS_SAFETY_MSS_MCRC            19U
#define SDL_ECC_BUS_SAFETY_MSS_PCR             20U
#define SDL_ECC_BUS_SAFETY_MSS_PCR2            21U
#define SDL_ECC_BUS_SAFETY_MSS_L2_A            22U
#define SDL_ECC_BUS_SAFETY_MSS_L2_B            23U
#define SDL_ECC_BUS_SAFETY_MSS_SWBUF           24U
#define SDL_ECC_BUS_SAFETY_MSS_GPADC           25U
#define SDL_ECC_BUS_SAFETY_MSS_DMM_SLV         26U
#define SDL_ECC_BUS_SAFETY_MSS_TO_MDO          27U
#define SDL_ECC_BUS_SAFETY_DAP_R232            28U

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
