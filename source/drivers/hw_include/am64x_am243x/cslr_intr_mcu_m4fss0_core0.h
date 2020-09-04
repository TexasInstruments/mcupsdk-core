/********************************************************************
*
* MCU_M4FSS0_CORE0 INTERRUPT MAP. header file
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
#ifndef CSLR_MCU_M4FSS0_CORE0_INTERRUPT_MAP_H_
#define CSLR_MCU_M4FSS0_CORE0_INTERRUPT_MAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: MCU_M4FSS0_CORE0
*/

#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_4                               (0U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_5                               (1U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_6                               (2U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_7                               (3U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER0_INTR_PEND_0                                          (4U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER1_INTR_PEND_0                                          (5U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER2_INTR_PEND_0                                          (6U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER3_INTR_PEND_0                                          (7U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MASTER_SAFETY_GASKET0_TIMED_OUT_0                           (8U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_RESETZ_SYNC_STRETCH_0     (9U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_PORZ_SYNC_STRETCH_0       (10U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_CFG_LVL_0                                      (11U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_HI_LVL_0                                       (12U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_LOW_LVL_0                                      (13U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_M4FSS0_RAT_0_EXP_INTR_0                                     (14U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_VTM0_THERM_LVL_GT_TH1_INTR_0                                    (15U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCRC64_0_INT_MCRC_0                                         (16U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_I2C0_POINTRPEND_0                                           (17U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_I2C1_POINTRPEND_0                                           (18U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_RTI0_INTR_WWD_0                                             (19U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_PSC0_PSC_ALLINT_0                                           (20U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMEOUT0_TRANS_ERR_LVL_0                                    (21U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCSPI0_INTR_SPI_0                                           (22U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCSPI1_INTR_SPI_0                                           (23U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_UART0_USART_IRQ_0                                           (24U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_UART1_USART_IRQ_0                                           (25U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_CORTEX_M3_0_SEC_OUT_0                                     (26U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_CORTEX_M3_0_SEC_OUT_1                                     (27U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_DCC0_INTR_DONE_LEVEL_0                                      (28U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DDPA0_DDPA_INTR_0                                               (29U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_VTM0_THERM_LVL_GT_TH2_INTR_0                                    (30U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_CBASS0_DEFAULT_ERR_INTR_0                                   (31U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_168                         (32U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_169                         (33U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_170                         (34U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_171                         (35U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_172                         (36U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_173                         (37U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_174                         (38U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_175                         (39U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_176                         (40U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_177                         (41U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_178                         (42U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_179                         (43U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_180                         (44U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_181                         (45U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_182                         (46U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_183                         (47U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_AES_0_HIB_PUBLIC_0                                        (48U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_AES_0_HIB_SECURE_0                                        (49U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_DBG_AUTH_0_DEBUG_AUTH_INTR_0                              (50U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_PR1_HOST_INTR_PEND_6                                 (51U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_PR1_HOST_INTR_PEND_7                                 (52U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_ISO_RESET_PROTCOL_ACK_0                              (53U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_ISO_RESET_PROTCOL_ACK_0                              (54U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_VTM0_THERM_LVL_LT_TH0_INTR_0                                    (55U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_6_MAILBOX_CLUSTER_PEND_3               (56U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_7_MAILBOX_CLUSTER_PEND_3               (57U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_CTRL_MMR0_ACCESS_ERR_0                                          (58U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PADCFG_CTRL0_ACCESS_ERR_0                                       (59U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_PADCFG_CTRL0_ACCESS_ERR_0                                   (60U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_CTRL_MMR0_ACCESS_ERR_0                                      (61U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_PR1_HOST_INTR_PEND_6                                 (62U)
#define CSLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_PR1_HOST_INTR_PEND_7                                 (63U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_MCU_M4FSS0_CORE0_INTERRUPT_MAP_H_ */

