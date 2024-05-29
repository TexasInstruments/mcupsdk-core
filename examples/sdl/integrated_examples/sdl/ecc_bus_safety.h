/*  Copyright (c) 2022-2024 Texas Instruments Incorporated
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
 *  \file     ecc_bus_safety.h
 *
 *  \brief    This file contains ECC BUS SAFETY  Example test code declarations
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <dpl_interface.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc_bus_safety.h>
#include <sdl/esm/sdlr_esm.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr.h>


#if !defined(ECC_BUS_SAFETY_H)
#define ECC_BUS_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_ECC_BUS_SAFETY_TIMEOUT    (0x10000U)
#define SDL_APP_NOT_RUN               (-(int32_t) (2))
#define SDL_APP_FAILED                (-(int32_t) (1))
#define SDL_APP_PASS                  ((int32_t) (0))

/* Value for A count*/
#define EDMA_TEST_A_COUNT             (4U)
/* Value for B count */
#define EDMA_TEST_B_COUNT             (1U)
/* Value for C count */
#define EDMA_TEST_C_COUNT             (1U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO        (0U)

#define SDL_INTR_GROUP_NUM            (1U)
#define SDL_INTR_GROUP2_NUM           (2U)
#define SDL_INTR_PRIORITY_LVL_LOW     (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH    (1U)
#define SDL_ENABLE_ERR_PIN            (1U)
/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define EDMA_TEST_BUFFER_SIZE         (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/

extern void test_edmaATransfer(uint32_t busSftyNode, uint32_t dmaCh, uint32_t tcc,uint32_t param, uint32_t queNum,\
                                                                       uint32_t edmaNum );
#if defined (SUBSYS_DSS)
/* helper fuction to get the edma parameters on DSS */
extern void SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(uint32_t busSftyNode , uint32_t *dmaCh, uint32_t *tcc,\
                                               uint32_t *param, uint32_t *queNum, uint32_t *edmaNum );
#endif
#if defined (SUBSYS_MSS)
/* helper fuction to get the edma parameters on MSS */
extern void SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(uint32_t busSftyNode , uint32_t *dmaCh, uint32_t *tcc,\
                                                      uint32_t *param, uint32_t *queNum, uint32_t *edmaNum );
#endif


/* Node MSS_AHB_CR5A */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test(void);
/* Node MSS_AHB_CR5B */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test(void);
/* Node MSS_AHB_CR5C */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test(void);
/* Node MSS_AHB_CR5D */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test(void);

/* Node MSS_TPTC_A0_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test_Polling(void);

/* Node MSS_TPTC_A1_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test_Polling(void);

/* Node MSS_TPTC_A0_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test(void);

extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test_Polling(void);

/* Node MSS_TPTC_A1_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test(void);

extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test_Polling(void);

/* Node MSS_AXI_CR5A_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5B_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5C_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5D_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5A_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5B_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5C_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5D_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5A_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void);

/* Node MSS_AXI_CR5B_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void);

/* Node MSS_AXI_CR5C_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test(void);

/* Node MSS_AXI_CR5D_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test(void);
/* Node MSS_MMC_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test(void);

/* Node MSS_GPMC */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test(void);

/* Node MSS_L2_A */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_Test(void);

/* Node MSS_L2_B */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_Test(void);

/* Node MSS_L2_C */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_SEC_Test(void);

/* Node MSS_L2_D */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_Test(void);

/* Node MSS_CPSW */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_Test(void);

/* Node MAIN_VBUSP */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_Test(void);

/* Node PERI_VBUSP */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_Test(void);

/* Node MSS_MSS_MCRC */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test(void);

/* Node MSS_MSS_QSPI */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test(void);

/* Node MSS_MSS_MBOX */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test(void);

/* Node MSS_MSS_STM_STIM */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_RED_Test(void);

/* Node MSS_MSS_SCRP0 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_RED_Test(void);

/* Node MSS_MSS_SCRP1 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_RED_Test(void);

/* Node MSS_ICSSM_PDSP0 */
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_RED_Test(void);

/* Node MSS_ICSSM_PDSP1 */
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_RED_Test(void);

/* Node MSS_ICSSM_S */
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_RED_Test(void);

/* Node MSS_DAP */
extern int32_t SDL_ECC_BUS_SAFETY_DAP_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DAP_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DAP_RED_Test(void);


#ifdef __cplusplus
}
#endif /*extern "C" */

#endif /* SDL_ECCBUS_SAFETY_H */
/* Nothing past this point */
