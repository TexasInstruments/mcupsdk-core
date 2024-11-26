/*
 *   Copyright (c) 2022-24 Texas Instruments Incorporated
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
 *  @addtogroup SDL_MSS_CR5_API API's for MSS CR5
    @{
 */

#ifndef SDL_MSS_CR5_SOC_H_
#define SDL_MSS_CR5_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/am263px/sdlr_soc_baseaddress.h>
#include <sdl/include/am263px/sdlr_mss_ctrl.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_ECC_BUS_SAFETY_MSS_BUS_CFG         (uint32_t)SDL_MSS_CTRL_U_BASE
#define DWORD                                  (0x20U)
#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE      (0x000000A0U)
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE      (0x000000A4U)
#define SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE      (0x000000B0U)
#define SDL_MSS_CTRL_R5SS1_CORE1_AHB_BASE      (0x000000B4U)
#define SDL_MSS_CTRL_R5SS0_CORE0_AHB_END       (SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE + SDL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS1_CORE0_AHB_END       (SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE + SDL_MSS_CTRL_R5SS1_CORE0_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS0_CORE1_AHB_END       (SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE + SDL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE)
#define SDL_MSS_CTRL_R5SS1_CORE1_AHB_END       (SDL_MSS_CTRL_R5SS1_CORE1_AHB_BASE + SDL_MSS_CTRL_R5SS1_CORE1_AHB_SIZE)

#define SDL_R5SS0_CORE0_TCMA_U_SIZE            (0x000000020)
#define SDL_R5SS0_CORE0_TCMB_U_SIZE            (0x000000020)
#define SDL_MSS_CR5A_TCM_U_BASE                (SDL_R5SS0_CORE0_TCMA_U_BASE )
#define SDL_MSS_CR5B_TCM_U_BASE                (SDL_R5SS0_CORE0_TCMB_U_BASE )
#define SDL_MSS_CR5A_TCM_U_END                 (SDL_R5SS0_CORE0_TCMA_U_BASE + SDL_R5SS0_CORE0_TCMA_U_SIZE)
#define SDL_MSS_CR5B_TCM_U_END                 (SDL_R5SS0_CORE0_TCMB_U_BASE + SDL_R5SS0_CORE0_TCMB_U_SIZE)
#define SDL_MBOX_SRAM_U_BASE_END               (SDL_MBOX_SRAM_U_BASE+100U)
#define SDL_MMC0_U_BASE_END                    (SDL_MMC0_U_BASE+0X1FFCU-DWORD)
#define SDL_CORE_VBUSP_START                   (0x50800000U)
#define SDL_CORE_VBUSP_START_END               (SDL_CORE_VBUSP_START+0X1FFCU)
#define SDL_PERI_VBUSP_START                   (0x50200000)
#define SDL_PERI_VBUSP_START_END               (SDL_PERI_VBUSP_START+0X7FFFFCU)
#define SDL_MPU_L2OCRAM_BANK0                  (0x40020000U)
#define SDL_MPU_L2OCRAM_BANK0_END              (0x40020FFFU-DWORD)
#define SDL_MPU_L2OCRAM_BANK1                  (0x40040000U)
#define SDL_MPU_L2OCRAM_BANK1_END              (0x40040FFFU-DWORD)
#define SDL_MPU_L2OCRAM_BANK2                  (0x40060000U)
#define SDL_MPU_L2OCRAM_BANK2_END              (0x40060FFFU-DWORD)
#define SDL_MPU_L2OCRAM_BANK3                  (0x40080000U)
#define SDL_MPU_L2OCRAM_BANK3_END              (0x40080FFFU-DWORD)
#define SDL_MSS_MCRC_U_BASE                    (SDL_MCRC0_U_BASE)
#define SDL_MSS_MCRC_U_SIZE                    (0x000001E4U)
#define SDL_MSS_MCRC_U_END                     (SDL_MSS_MCRC_U_BASE + SDL_MSS_MCRC_U_SIZE)
#define SDL_MSS_STM_STIM_U_BASE                (SDL_STM_STIM_U_BASE)
#define SDL_MSS_STM_STIM_U_SIZE                (0x00FFFFFFU)
#define SDL_MSS_STM_STIM_U_END                 (SDL_MSS_STM_STIM_U_BASE + SDL_MSS_STM_STIM_U_SIZE)

#define SDL_MSS_CR5A_AXI_RD_START                   (0x35000000U)
#define SDL_MSS_CR5A_AXI_RD_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5A_AXI_WR_START                   (0x35000000U)
#define SDL_MSS_CR5A_AXI_WR_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5A_AXI_S_START                    (0x0U)
#define SDL_MSS_CR5A_AXI_S_END                      (0x0001FFFFU-8U)

#define SDL_MSS_CR5B_AXI_RD_START                   (0x35000000U)
#define SDL_MSS_CR5B_AXI_RD_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5B_AXI_WR_START                   (0x35000000U)
#define SDL_MSS_CR5B_AXI_WR_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5B_AXI_S_START                    (0x0U)
#define SDL_MSS_CR5B_AXI_S_END                      (0x0001FFFFU-8U)

#define SDL_MSS_CR5C_AXI_RD_START                   (0x35000000U)
#define SDL_MSS_CR5C_AXI_RD_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5C_AXI_WR_START                   (0x35000000U)
#define SDL_MSS_CR5C_AXI_WR_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5C_AXI_S_START                    (0x0U)
#define SDL_MSS_CR5C_AXI_S_END                      (0x0001FFFFU-8U)

#define SDL_MSS_CR5D_AXI_RD_START                   (0x35000000U)
#define SDL_MSS_CR5D_AXI_RD_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5D_AXI_WR_START                   (0x35000000U)
#define SDL_MSS_CR5D_AXI_WR_END                     (0x350003FFU-8U)
#define SDL_MSS_CR5D_AXI_S_START                    (0x0U)
#define SDL_MSS_CR5D_AXI_S_END                      (0x0001FFFFU-8U)

#define SDL_MSS_CTRL_TPCC_A0_WR_BASE                (0x52A40000U)
#define SDL_MSS_CTRL_TPCC_A0_WR_END                 (0x52A40400U-8U)

#define SDL_MSS_CTRL_TPCC_A1_WR_BASE                (0x52A60000U)
#define SDL_MSS_CTRL_TPCC_A1_WR_END                 (0x52A60400U-8U)

#define SDL_MSS_CTRL_TPCC_A0_RD_BASE                (0x52A40000U)
#define SDL_MSS_CTRL_TPCC_A0_RD_END                 (0x52A40400U-8U)

#define SDL_MSS_CTRL_TPCC_A1_RD_BASE                (0x52A60000U)
#define SDL_MSS_CTRL_TPCC_A1_RD_END                 (0x52A60400U-8U)

#define SDL_MSS_VBUSP_BASE                          (0x35000000U)
#define SDL_MSS_VBUSP_BASE_END                      (0x350003FFU-8U)

#define SDL_MSS_VBUSP_PERI_BASE                     (0x35000000U)
#define SDL_MSS_VBUSP_PERI_BASE_END                 (0x350003FFU-8U)

#define SDL_MSS_CPSW_BASE                           (0x52800000U)
#define SDL_MSS_CPSW_BASE_END                       (0x52800400U-8U)

#define SDL_MCRC_U_BASE                             (0x35000000U)
#define SDL_MCRC_U_BASE_END                         (0x350003FFU-8U)

#define SDL_STIM_U_BASE                             (0x53500000U)
#define SDL_STIM_U_BASE_END                         (0x535001FFU-8U)

#define SDL_SCRP0_U_BASE                            (0x48000000U)
#define SDL_SCRP0_U_BASE_END                        (0x4803FFFFU-8U)

#define SDL_SCRP1_U_BASE                            (0x48000000U)
#define SDL_SCRP1_U_BASE_END                        (0x4803FFFFU-8U)

#define SDL_ICSSM_PDSP0_U_BASE                      (0x48000000U)
#define SDL_ICSSM_PDSP0_U_BASE_END                  (0x4803FFFFU-8U)

#define SDL_ICSSM_PDSP1_U_BASE                      (0x48000000U)
#define SDL_ICSSM_PDSP1_U_BASE_END                  (0x4803FFFFU-8U)

#define SDL_ICSSM_S_BASE                            (0x48000000U)
#define SDL_ICSSM_S_BASE_END                        (0x4803FFFFU-8U)

#define SDL_DAP_U_BASE                              (0x48000000U)
#define SDL_DAP_U_BASE_END                          (0x4803FFFFU-8U)

/* Macro defines Ecc Bus Safety Nodes in the MSS Subsystem */
#define SDL_ECC_BUS_SAFETY_MSS_MBOX            0U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD      1U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD      2U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD     3U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD     4U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD     5U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD     6U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S      7U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S      8U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S      9U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S      10U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR      11U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR      12U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR      13U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB        14U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB        15U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB        16U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB        17U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR     18U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR     19U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR     20U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR     21U
#define SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP      22U
#define SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP      23U
#define SDL_ECC_BUS_SAFETY_MSS_QSPI            24U
#define SDL_ECC_BUS_SAFETY_MSS_CPSW            25U
#define SDL_ECC_BUS_SAFETY_MSS_MCRC            26U
#define SDL_ECC_BUS_SAFETY_MSS_L2_A            27U
#define SDL_ECC_BUS_SAFETY_MSS_L2_B            28U
#define SDL_ECC_BUS_SAFETY_MSS_L2_C            29U
#define SDL_ECC_BUS_SAFETY_MSS_L2_D            30U
#define SDL_ECC_BUS_SAFETY_MSS_SCRP            31U
#define SDL_ECC_BUS_SAFETY_MSS_DAP             32U
#define SDL_ECC_BUS_SAFETY_MSS_MMC             33U
#define SDL_ECC_BUS_SAFETY_MSS_SCRP0           35U
#define SDL_ECC_BUS_SAFETY_MSS_SCRP1           36U
#define SDL_ECC_BUS_SAFETY_ICSSM_PDSP0         37U
#define SDL_ECC_BUS_SAFETY_ICSSM_PDSP1         38U
#define SDL_ECC_BUS_SAFETY_ICSSM_S             39U
#define SDL_ECC_BUS_SAFETY_DAP                 40U
#define SDL_ECC_BUS_SAFETY_MSS_STM_STIM        41U

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
