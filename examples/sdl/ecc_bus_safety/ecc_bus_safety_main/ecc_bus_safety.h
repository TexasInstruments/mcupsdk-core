/*  Copyright (c) 2022-25 Texas Instruments Incorporated
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
#if defined (SOC_AM273X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/include/am273x/sdlr_intr_esm_dss.h>
#include <sdl/include/am273x/sdlr_intr_esm_mss.h>
#elif defined (SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/include/awr294x/sdlr_intr_esm_dss.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#endif
#if defined (SOC_AM263X)
#include <sdl/esm/v0/sdl_esm.h>
#endif
#if defined (SOC_AM263PX) || defined (SOC_AM261X)
#include <sdl/esm/v2/sdl_esm.h>
#endif
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
#if defined (SOC_AWR294X) || defined (SOC_AM273X)
#if defined (SUBSYS_MSS)
extern SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsMSS[30U];
#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
extern SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsDSS[33U];
#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
/* Event BitMap for ECC ESM callback for DSS */
extern SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsDSS[33U];
#endif
#endif
/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern void ecc_bus_safety_edma_boardInit(void);
extern void ecc_bus_safety_edma_boardDeinit(void);
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

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/* Node DSS_L3_BANKA */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_Test(void);


/* Node DSS_L3_BANKB */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_Test(void);

/* Node DSS_L3_BANKC */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_Test(void);

/* Node DSS_L3_BANKD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_Test(void);

/* Node DSS_HWA_DMA0 */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_Test(void);

/* Node DSS_HWA_DMA1 */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_Test(void);

/* Node DSS_DSP_SDMA */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_Test(void);

/* Node DSS_DSP_MDMA */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_Test(void);

/* Node DSS_PCR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_RED_Test(void);

/* Node DSS_MBOX */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_Test(void);

/* Node DSS_MCRC */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_Test(void);

/* Node DSS_CBUFF_FIFO */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_Test(void);

/* Node DSS_TPTC_A0_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_Test(void);

/* Node DSS_TPTC_A1_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_Test(void);

/* Node DSS_TPTC_B0_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_Test(void);

/* Node DSS_TPTC_B1_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_Test(void);

/* Node DSS_TPTC_C0_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_Test(void);

/* Node DSS_TPTC_C1_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_Test(void);

/* Node DSS_TPTC_C2_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_Test(void);

/* Node DSS_TPTC_C3_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_Test(void);

/* Node DSS_TPTC_C4_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_Test(void);

/* Node DSS_TPTC_C5_WR */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_Test(void);

/* Node DSS_TPTC_A0_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_Test(void);

/* Node DSS_TPTC_A1_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_Test(void);

/* Node DSS_TPTC_B0_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_Test(void);

/* Node DSS_TPTC_B1_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_Test(void);

/* Node DSS_TPTC_C0_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_Test(void);

/* Node DSS_TPTC_C1_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_Test(void);

/* Node DSS_TPTC_C2_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_Test(void);

/* Node DSS_TPTC_C3_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_Test(void);

/* Node DSS_TPTC_C4_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_Test(void);

/* Node DSS_TPTC_C5_RD */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_Test(void);

#endif
#endif

#if defined SOC_AM273X
#if defined (SUBSYS_DSS)

/* Node DSS_MDO_FIFO */
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling(void);

#endif
#endif


#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/* Node MSS_MSS_MCRC */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test(void);

/* Node MSS_MSS_QSPI */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test(void);

/* Node MSS_MSS_SWBUF */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_RED_Test(void);

/* Node MSS_MSS_SCRP */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP_RED_Test(void);

/* Node MSS_TO_MDO */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_RED_Test(void);

/* Node DAP_R232 */
extern int32_t SDL_ECC_BUS_SAFETY_DAP_R232_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DAP_R232_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DAP_R232_RED_Test(void);
/* Node MSS_TPTC_A0_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test_Polling(void);

/* Node MSS_TPTC_A1_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test_Polling(void);

/* Node MSS_TPTC_B0_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test_Polling(void);

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

/* Node MSS_TPTC_B0_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test(void);

extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test_Polling(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test_Polling(void);

/* Node MSS_AHB_CR5A */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test(void);
/* Node MSS_AHB_CR5B */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test(void);

/* Node MSS_AXI_CR5A_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5B_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5A_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5B_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5A_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void);

/* Node MSS_AXI_CR5B_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void);

/* Node MSS_MSS_MBOX */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test(void);

/* Node MSS_L2_A */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test(void);

/* Node MSS_L2_B */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test(void);

/* Node MSS_DMM_SLV */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_Test(void);

/* Node MSS_DMM */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_RED_Test(void);

/* Node MSS_GPADC */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_RED_Test(void);

/* Node MSS_PCR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_RED_Test(void);

/* Node MSS_PCR2 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_Test(void);

/* Nodes MSS_CPSW */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test(void);
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/* Node MSS_AHB_CR5A */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test(void);
/* Node MSS_AHB_CR5B */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test(void);
#if !defined(SOC_AM261X)
/* Node MSS_AHB_CR5C */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test(void);
/* Node MSS_AHB_CR5D */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test(void);
#endif

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

extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void);

extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void);

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

#if !defined(SOC_AM261X)
/* Node MSS_AXI_CR5C_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test(void);

/* Node MSS_AXI_CR5D_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test(void);
#endif

/* Node MSS_AXI_CR5A_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5B_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_SEC_Test(void);

#if !defined(SOC_AM261X)
/* Node MSS_AXI_CR5C_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_SEC_Test(void);

/* Node MSS_AXI_CR5D_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_SEC_Test(void);
#endif

/* Node MSS_AXI_CR5A_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void);

/* Node MSS_AXI_CR5B_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void);

#if !defined(SOC_AM261X)
/* Node MSS_AXI_CR5C_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test(void);

/* Node MSS_AXI_CR5D_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test(void);
#endif

/* Node MSS_MMC_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test(void);

#if !defined(SOC_AM263PX) || defined (SOC_AM261X)
/* Node MSS_GPMC */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test(void);
#endif

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

#if !defined(SOC_AM261X)
/* Node MSS_L2_D */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_Test(void);
#endif
#if defined (SOC_AM263PX)
/* Node MSS_L2_E */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_SEC_Test(void);

/* Node MSS_L2_F */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_RED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_SEC_Test(void);
#endif

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

#if defined (SOC_AM263X)
/* Node MSS_MSS_QSPI */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test(void);
#elif defined (SOC_AM263PX) || defined (SOC_AM261X)
/* Node MSS_MSS_OSPI */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_RED_Test(void);
#endif
#if defined (SOC_AM261X)
/* Node MSS_MSS_USBSS_RD */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_RED_Test(void);
/* Node MSS_MSS_USBSS_WR */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_RED_Test(void);
#endif
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

#if defined (SOC_AM261X)
/* Node MSS_MSS_ICSSM0_PDSP0 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_RED_Test(void);

/* Node MSS_MSS_ICSSM0_PDSP1 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_RED_Test(void);

/* Node MSS_MSS_ICSSM0_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_RED_Test(void);

/* Node MSS_MSS_ICSSM1_PDSP0 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_RED_Test(void);

/* Node MSS_MSS_ICSSM1_PDSP1 */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_RED_Test(void);

/* Node MSS_MSS_ICSSM1_S */
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_RED_Test(void);
#else
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
#endif

/* Node MSS_DAP */
extern int32_t SDL_ECC_BUS_SAFETY_DAP_SEC_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DAP_DED_Test(void);
extern int32_t SDL_ECC_BUS_SAFETY_DAP_RED_Test(void);
#endif

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X) || defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#if defined (SUBSYS_MSS)
/* CPSW setup */
extern void setup_CPSW(void);
/* CPSW transfer */
extern void cpsw_transfer(void);
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
extern void cpsw_transfer_ded();
#endif

#ifdef __cplusplus
}
#endif /*extern "C" */

#endif /* SDL_ECCBUS_SAFETY_H */
/* Nothing past this point */
