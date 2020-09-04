/********************************************************************
*
* PRU_ICSSG1 INTERRUPT MAP. header file
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
#ifndef CSLR_PRU_ICSSG1_INTERRUPT_MAP_H_
#define CSLR_PRU_ICSSG1_INTERRUPT_MAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: PRU_ICSSG1
*/

#define CSLR_PRU_ICSSG1_ISO_RESET_PROTCOL_REQ_GLUELOGIC_MAINRESET_REQUEST_GLUE_MAIN_RESETZ_SYNC_STRETCH_0 (0U)
#define CSLR_PRU_ICSSG1_PR1_EDC0_LATCH0_IN_TIMESYNC_EVENT_INTROUTER0_OUTL_12                       (0U)
#define CSLR_PRU_ICSSG1_PR1_EDC0_LATCH1_IN_TIMESYNC_EVENT_INTROUTER0_OUTL_13                       (0U)
#define CSLR_PRU_ICSSG1_PR1_EDC1_LATCH0_IN_TIMESYNC_EVENT_INTROUTER0_OUTL_14                       (0U)
#define CSLR_PRU_ICSSG1_PR1_EDC1_LATCH1_IN_TIMESYNC_EVENT_INTROUTER0_OUTL_15                       (0U)
#define CSLR_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_18                      (0U)
#define CSLR_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_24                      (0U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_160                       (0U)
#define CSLR_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_19                      (1U)
#define CSLR_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_25                      (1U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_161                       (1U)
#define CSLR_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_20                      (2U)
#define CSLR_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_26                      (2U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_162                       (2U)
#define CSLR_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_21                      (3U)
#define CSLR_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_27                      (3U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_163                       (3U)
#define CSLR_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_22                      (4U)
#define CSLR_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_28                      (4U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_164                       (4U)
#define CSLR_PRU_ICSSG1_PR1_IEP0_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_23                      (5U)
#define CSLR_PRU_ICSSG1_PR1_IEP1_CAP_INTR_REQ_MAIN_GPIOMUX_INTROUTER0_OUTP_29                      (5U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_165                       (5U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_166                       (6U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_167                       (7U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_ECAP0_ECAP_INT_0                                              (8U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_ECAP1_ECAP_INT_0                                              (9U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_ECAP2_ECAP_INT_0                                              (10U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM0_EPWM_ETINT_0                                            (12U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM0_EPWM_TRIPZINT_0                                         (13U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM1_EPWM_ETINT_0                                            (14U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM1_EPWM_TRIPZINT_0                                         (15U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM2_EPWM_ETINT_0                                            (16U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM2_EPWM_TRIPZINT_0                                         (17U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM3_EPWM_ETINT_0                                            (18U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM3_EPWM_TRIPZINT_0                                         (19U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM4_EPWM_ETINT_0                                            (20U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM4_EPWM_TRIPZINT_0                                         (21U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM5_EPWM_ETINT_0                                            (22U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM5_EPWM_TRIPZINT_0                                         (23U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM6_EPWM_ETINT_0                                            (24U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM6_EPWM_TRIPZINT_0                                         (25U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM7_EPWM_ETINT_0                                            (26U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM7_EPWM_TRIPZINT_0                                         (27U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM8_EPWM_ETINT_0                                            (28U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EPWM8_EPWM_TRIPZINT_0                                         (29U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EQEP0_EQEP_INT_0                                              (30U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EQEP1_EQEP_INT_0                                              (31U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_EQEP2_EQEP_INT_0                                              (32U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX0_FSI_RX_OINT1_0                                         (33U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX0_FSI_RX_OINT2_0                                         (34U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX1_FSI_RX_OINT1_0                                         (35U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX1_FSI_RX_OINT2_0                                         (36U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX2_FSI_RX_OINT1_0                                         (37U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX2_FSI_RX_OINT2_0                                         (38U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX3_FSI_RX_OINT1_0                                         (39U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX3_FSI_RX_OINT2_0                                         (40U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX4_FSI_RX_OINT1_0                                         (41U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX4_FSI_RX_OINT2_0                                         (42U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX5_FSI_RX_OINT1_0                                         (43U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSIRX5_FSI_RX_OINT2_0                                         (44U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSITX0_FSI_TX_OINT1_0                                         (45U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_46                               (46U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_47                               (47U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_48                               (48U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_49                               (49U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_50                               (50U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_51                               (51U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_52                               (52U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_53                               (53U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSITX0_FSI_TX_OINT2_0                                         (54U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSITX1_FSI_TX_OINT1_0                                         (55U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_FSITX1_FSI_TX_OINT2_0                                         (56U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCSPI0_INTR_SPI_0                                             (57U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCSPI1_INTR_SPI_0                                             (58U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCSPI3_INTR_SPI_0                                             (59U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCSPI4_INTR_SPI_0                                             (60U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCU_MCSPI0_INTR_SPI_0                                         (61U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCU_MCSPI1_INTR_SPI_0                                         (62U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART0_USART_IRQ_0                                             (63U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART1_USART_IRQ_0                                             (64U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART2_USART_IRQ_0                                             (65U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART3_USART_IRQ_0                                             (66U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART4_USART_IRQ_0                                             (67U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART5_USART_IRQ_0                                             (68U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_UART6_USART_IRQ_0                                             (69U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCU_UART0_USART_IRQ_0                                         (70U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCU_UART1_USART_IRQ_0                                         (71U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCSPI2_INTR_SPI_0                                             (72U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_GLUELOGIC_SOCA_INT_GLUE_SOCA_INT_0                            (73U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_GLUELOGIC_SOCB_INT_GLUE_SOCB_INT_0                            (74U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_I2C0_POINTRPEND_0                                             (75U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_I2C1_POINTRPEND_0                                             (76U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_I2C2_POINTRPEND_0                                             (77U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_I2C3_POINTRPEND_0                                             (78U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCU_I2C0_POINTRPEND_0                                         (79U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCU_I2C1_POINTRPEND_0                                         (80U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCAN0_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0                        (81U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCAN0_MCANSS_MCAN_LVL_INT_0                                   (82U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCAN0_MCANSS_MCAN_LVL_INT_1                                   (83U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCAN1_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0                        (84U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCAN1_MCANSS_MCAN_LVL_INT_0                                   (85U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_MCAN1_MCANSS_MCAN_LVL_INT_1                                   (86U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_5                               (87U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_6                               (88U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_7                               (89U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_GPMC0_GPMC_SINTERRUPT_0                                       (90U)
#define CSLR_PRU_ICSSG1_PR1_SLV_INTR_ADC0_GEN_LEVEL_0                                              (91U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_PRU_ICSSG1_INTERRUPT_MAP_H_ */

