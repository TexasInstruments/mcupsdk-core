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
#ifndef SDLR_MCU_M4FSS0_CORE0_INTERRUPT_MAP_H_
#define SDLR_MCU_M4FSS0_CORE0_INTERRUPT_MAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: MCU_M4FSS0_CORE0
*/

#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_4                               (0U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_5                               (1U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_6                               (2U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCU_GPIOMUX_INTROUTER0_OUTP_7                               (3U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER0_INTR_PEND_0                                          (4U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER1_INTR_PEND_0                                          (5U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER2_INTR_PEND_0                                          (6U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER3_INTR_PEND_0                                          (7U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MASTER_SAFETY_GASKET0_TIMED_OUT_0                           (8U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_RESETZ_SYNC_STRETCH_0     (9U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_PORZ_SYNC_STRETCH_0       (10U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_CFG_LVL_0                                      (11U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_HI_LVL_0                                       (12U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_ESM0_ESM_INT_LOW_LVL_0                                      (13U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_M4FSS0_RAT_0_EXP_INTR_0                                     (14U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_VTM0_THERM_LVL_GT_TH1_INTR_0                                    (15U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCRC64_0_INT_MCRC_0                                         (16U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_I2C0_POINTRPEND_0                                           (17U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_I2C1_POINTRPEND_0                                           (18U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_RTI0_INTR_WWD_0                                             (19U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_PSC0_PSC_ALLINT_0                                           (20U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMEOUT0_TRANS_ERR_LVL_0                                    (21U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCSPI0_INTR_SPI_0                                           (22U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_MCSPI1_INTR_SPI_0                                           (23U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_UART0_USART_IRQ_0                                           (24U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_UART1_USART_IRQ_0                                           (25U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_CORTEX_M3_0_SEC_OUT_0                                     (26U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_CORTEX_M3_0_SEC_OUT_1                                     (27U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_DCC0_INTR_DONE_LEVEL_0                                      (28U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DDPA0_DDPA_INTR_0                                               (29U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_VTM0_THERM_LVL_GT_TH2_INTR_0                                    (30U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_CBASS0_DEFAULT_ERR_INTR_0                                   (31U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_168                         (32U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_169                         (33U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_170                         (34U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_171                         (35U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_172                         (36U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_173                         (37U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_174                         (38U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_175                         (39U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_176                         (40U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_177                         (41U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_178                         (42U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_179                         (43U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_180                         (44U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_181                         (45U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_182                         (46U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_183                         (47U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_AES_0_HIB_PUBLIC_0                                        (48U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_AES_0_HIB_SECURE_0                                        (49U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_DMSC0_DBG_AUTH_0_DEBUG_AUTH_INTR_0                              (50U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_PR1_HOST_INTR_PEND_6                                 (51U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_PR1_HOST_INTR_PEND_7                                 (52U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG0_ISO_RESET_PROTCOL_ACK_0                              (53U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_ISO_RESET_PROTCOL_ACK_0                              (54U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_VTM0_THERM_LVL_LT_TH0_INTR_0                                    (55U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_6_MAILBOX_CLUSTER_PEND_3               (56U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MAILBOX0_MAILBOX_CLUSTER_7_MAILBOX_CLUSTER_PEND_3               (57U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_CTRL_MMR0_ACCESS_ERR_0                                          (58U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PADCFG_CTRL0_ACCESS_ERR_0                                       (59U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_PADCFG_CTRL0_ACCESS_ERR_0                                   (60U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_CTRL_MMR0_ACCESS_ERR_0                                      (61U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_PR1_HOST_INTR_PEND_6                                 (62U)
#define SDLR_MCU_M4FSS0_CORE0_NVIC_PRU_ICSSG1_PR1_HOST_INTR_PEND_7                                 (63U)

#ifdef __cplusplus
}
#endif
#endif /* SDLR_MCU_M4FSS0_CORE0_INTERRUPT_MAP_H_ */

