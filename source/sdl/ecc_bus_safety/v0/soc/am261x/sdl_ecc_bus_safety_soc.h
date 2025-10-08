/*
 *   Copyright (c) 2022-25 Texas Instruments Incorporated
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

#include <sdl/include/am261x/sdlr_soc_baseaddress.h>
#include <sdl/include/am261x/sdlr_mss_ctrl.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_ECC_BUS_SAFETY_MSS_BUS_CFG              (uint32_t)SDL_MSS_CTRL_U_BASE
#define DWORD                                       (0x20U)
#define SDL_MSS_CTRL_MSS_CR5A0_AHB_BASE             (SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE)
#define SDL_MSS_CTRL_MSS_CR5B0_AHB_BASE             (SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE)
#define SDL_MSS_CTRL_MSS_CR5A0_AHB_END              (SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE + SDL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE)
#define SDL_MSS_CTRL_MSS_CR5B0_AHB_END              (SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE + SDL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE)

#define SDL_R5SS0_CORE0_TCMA_U_SIZE                 (0x000000020U)
#define SDL_R5SS0_CORE0_TCMB_U_SIZE                 (0x000000020U)
#define SDL_MSS_CR5A_TCM_U_BASE                     (SDL_R5SS0_CORE0_TCMA_U_BASE)
#define SDL_MSS_CR5B_TCM_U_BASE                     (SDL_R5SS0_CORE0_TCMB_U_BASE)
#define SDL_MSS_CR5A_TCM_U_END                      (SDL_R5SS0_CORE0_TCMA_U_BASE + SDL_R5SS0_CORE0_TCMA_U_SIZE)
#define SDL_MSS_CR5B_TCM_U_END                      (SDL_R5SS0_CORE0_TCMB_U_BASE + SDL_R5SS0_CORE0_TCMB_U_SIZE)
#define SDL_MBOX_SRAM_U_BASE_END                    (SDL_MBOX_SRAM_U_BASE+0x3FFFU)
#define SDL_MMC0_U_BASE_END                         (SDL_MMC0_U_BASE+0X1FFCU-DWORD)
#define SDL_CORE_VBUSP_START                        (0x50800000U)
#define SDL_CORE_VBUSP_START_END                    (SDL_CORE_VBUSP_START+0X1FFCU)
#define SDL_PERI_VBUSP_START                        (0x50200000U)
#define SDL_PERI_VBUSP_START_END                    (SDL_PERI_VBUSP_START+0X7FFFFCU)
#define SDL_MPU_L2OCRAM_BANK0                       (0x40020000U)
#define SDL_MPU_L2OCRAM_BANK0_END                   (0x40020FFFU-DWORD)
#define SDL_MPU_L2OCRAM_BANK1                       (0x40040000U)
#define SDL_MPU_L2OCRAM_BANK1_END                   (0x40040FFFU-DWORD)
#define SDL_MPU_L2OCRAM_BANK2                       (0x40060000U)
#define SDL_MPU_L2OCRAM_BANK2_END                   (0x40060FFFU-DWORD)
#define SDL_MPU_L2OCRAM_BANK3                       (0x40080000U)
#define SDL_MPU_L2OCRAM_BANK3_END                   (0x40080FFFU-DWORD)
#define SDL_MSS_QSPI_U_BASE                         (SDL_QSPI0_U_BASE)
#define SDL_MSS_QSPI_U_SIZE                         (0x000001D8U)
#define SDL_MSS_QSPI_U_END                          (SDL_MSS_QSPI_U_BASE + SDL_MSS_QSPI_U_SIZE)
#define SDL_MSS_MCRC_U_BASE                         (SDL_MCRC0_U_BASE)
#define SDL_MSS_MCRC_U_SIZE                         (0x000001E4U)
#define SDL_MSS_MCRC_U_END                          (SDL_MSS_MCRC_U_BASE + SDL_MSS_MCRC_U_SIZE)
#define SDL_STIM_U_BASE                             (0x53500000U)
#define SDL_STIM_U_END                              (0x535001FFU-8U)

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

#define SDL_QSPI_U_BASE                             (0x48200000U)
#define SDL_QSPI_U_BASE_END                         (0x482001FFU-8U)

#define SDL_MCRC_U_BASE                             (0x35000000U)
#define SDL_MCRC_U_BASE_END                         (0x350003FFU-8U)

#define SDL_SCRP0_U_BASE                            (0x48000000U)
#define SDL_SCRP0_U_BASE_END                        (0x4803FFFFU-8U)

#define SDL_SCRP1_U_BASE                            (0x48000000U)
#define SDL_SCRP1_U_BASE_END                        (0x4803FFFFU-8U)

#define SDL_ICSSM0_PDSP0_U_BASE                     (SDL_ICSSM0_INTERNAL_U_BASE)
#define SDL_ICSSM0_PDSP0_U_SIZE                     (0x000000FFU)
#define SDL_ICSSM0_PDSP0_U_BASE_END                 (0x48038000U-8U)

#define SDL_ICSSM0_PDSP1_U_BASE                     (SDL_ICSSM0_INTERNAL_U_BASE)
#define SDL_ICSSM0_PDSP1_U_SIZE                     (0x000000FFU)
#define SDL_ICSSM0_PDSP1_U_BASE_END                 (0x48038000U-8U)

#define SDL_ICSSM0_S_BASE                           (SDL_ICSSM0_INTERNAL_U_BASE)
#define SDL_ICSSM0_S_SIZE                           (0x000000FFU)
#define SDL_ICSSM0_S_BASE_END                       (0x48038000U-8U)

#define SDL_ICSSM1_PDSP0_U_BASE                     (SDL_ICSS_M_ICSSM_1_PR1_PDSP0_IRAM_U_BASE)
#define SDL_ICSSM1_PDSP0_U_SIZE                     (0x000000FFU)
#define SDL_ICSSM1_PDSP0_U_BASE_END                 (SDL_ICSS_M_ICSSM_1_PR1_PDSP0_IRAM_U_BASE+SDL_ICSSM1_PDSP0_U_SIZE)

#define SDL_ICSSM1_PDSP1_U_BASE                     (SDL_ICSS_M_ICSSM_1_PR1_PDSP1_IRAM_U_BASE)
#define SDL_ICSSM1_PDSP1_U_SIZE                     (0x000000FFU)
#define SDL_ICSSM1_PDSP1_U_BASE_END                 (SDL_ICSS_M_ICSSM_1_PR1_PDSP1_IRAM_U_BASE+SDL_ICSSM1_PDSP1_U_SIZE)

#define SDL_ICSSM1_S_BASE                           (SDL_ICSS_M_ICSSM_1_PR1_CFG_SLV_U_BASE)
#define SDL_ICSSM1_S_SIZE                           (0x000000FFU)
#define SDL_ICSSM1_S_BASE_END                       (SDL_ICSS_M_ICSSM_1_PR1_CFG_SLV_U_BASE+SDL_ICSSM1_S_SIZE)

#define SDL_DAP_U_BASE                             (0x48000000U)
#define SDL_DAP_U_BASE_END                         (0x4803FFFFU-8U)

#define SDL_GPMC0_CFG_U_BASE_END                   (SDL_GPMC0_CFG_U_BASE+0X3FCU-DWORD)

#define SDL_OSPI0_U_BASE                           SDL_FLASH_CONFIG_REG6_U_BASE
#define SDL_OSPI0_U_BASE_END                       (SDL_FLASH_CONFIG_REG6_U_BASE+0x00001FFU)

#define SDL_USB_RD_U_BASE                          (SDL_USB_RAM0_U_BASE)
#define SDL_USB_RD_U_BASE_END                      (SDL_USB_RAM0_U_BASE + 0x00007FFCU)

#define SDL_USB_WR_U_BASE                          (SDL_USB_RAM0_U_BASE)
#define SDL_USB_WR_U_BASE_END                      (SDL_USB_RAM0_U_BASE + 0x00007FFCU)

/* Macro defines Ecc Bus Safety Nodes in the MSS Subsystem */

#define SDL_ECC_BUS_SAFETY_MSS_MBOX                0U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD          1U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD          2U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD         3U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD         4U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S          5U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S          6U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR          7U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR          8U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB            9U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB            10U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR         11U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR         12U
#define SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP          13U
#define SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP          14U
#define SDL_ECC_BUS_SAFETY_MSS_OSPI                15U
#define SDL_ECC_BUS_SAFETY_MSS_CPSW                16U
#define SDL_ECC_BUS_SAFETY_MSS_MCRC                17U
#define SDL_ECC_BUS_SAFETY_MSS_L2_A                18U
#define SDL_ECC_BUS_SAFETY_MSS_L2_B                19U
#define SDL_ECC_BUS_SAFETY_MSS_L2_C                20U
#define SDL_ECC_BUS_SAFETY_DAP                     21U
#define SDL_ECC_BUS_SAFETY_MSS_MMC                 22U
#define SDL_ECC_BUS_SAFETY_MSS_SCRP0               23U
#define SDL_ECC_BUS_SAFETY_MSS_SCRP1               24U
#define SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0        25U
#define SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1        26U
#define SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S            27U
#define SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0        28U
#define SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1        29U
#define SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S            30U
#define SDL_ECC_BUS_SAFETY_MSS_USBSS_RD            31U
#define SDL_ECC_BUS_SAFETY_MSS_USBSS_WR            32U
#define SDL_ECC_BUS_SAFETY_MSS_GPMC                33U
#define SDL_ECC_BUS_SAFETY_MSS_STM_STIM            34U

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
