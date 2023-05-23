/********************************************************************
* Copyright (C) 2024 Texas Instruments Incorporated.
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
#ifndef CSLR_MAIN2MCU_LVL_INTRTR0_INTERRUPT_MAP_H_
#define CSLR_MAIN2MCU_LVL_INTRTR0_INTERRUPT_MAP_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: MAIN2MCU_LVL_INTRTR0
*/

#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_GLUELOGIC_NMI_INVERTER_NMI_Z_INVERTED_0                       (1U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DSS0_DISPC_INTR_REQ_0_0                                       (2U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DSS0_DISPC_INTR_REQ_1_0                                       (3U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_SA2_UL0_SA_UL_TRNG_0                                          (4U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_SA2_UL0_SA_UL_PKA_0                                           (5U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CTRL_MMR0_ACCESS_ERR_0                                        (6U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_ELM0_ELM_POROCPSINTERRUPT_LVL_0                               (7U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_GPMC0_GPMC_SINTERRUPT_0                                       (8U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DDRSS0_DDRSS_V2H_OTHER_ERR_LVL_0                              (10U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CAL0_INT_CAL_L_0                                              (11U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CCDEBUGSS0_AQCMPINTR_LEVEL_0                                  (13U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DEBUGSS0_AQCMPINTR_LEVEL_0                                    (14U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DEBUGSS0_CTM_LEVEL_0                                          (15U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCASP0_XMIT_INTR_PEND_0                                       (16U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCASP0_REC_INTR_PEND_0                                        (17U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCASP1_XMIT_INTR_PEND_0                                       (18U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCASP1_REC_INTR_PEND_0                                        (19U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCASP2_XMIT_INTR_PEND_0                                       (20U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCASP2_REC_INTR_PEND_0                                        (21U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MMCSD1_EMMCSDSS_INTR_0                                        (28U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MMCSD0_EMMCSDSS_INTR_0                                        (29U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_0                               (32U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_1                               (33U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_2                               (34U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_3                               (35U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_4                               (36U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_5                               (37U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_6                               (38U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG0_PR1_HOST_INTR_PEND_7                               (39U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_0                               (40U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_1                               (41U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_2                               (42U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_3                               (43U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_4                               (44U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_5                               (45U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_6                               (46U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG1_PR1_HOST_INTR_PEND_7                               (47U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_0                               (48U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_1                               (49U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_2                               (50U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_3                               (51U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_4                               (52U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_5                               (53U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_6                               (54U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PRU_ICSSG2_PR1_HOST_INTR_PEND_7                               (55U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_GPU0_GPU_IRQ_0                                                (56U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_GPU0_EXP_INTR_0                                               (57U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_GPU0_INIT_ERR_0                                               (58U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_GPU0_TARGET_ERR_0                                             (59U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE0_PEND_0                                            (64U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE1_PEND_0                                            (65U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE2_PEND_0                                            (66U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE3_PEND_0                                            (67U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE4_PEND_0                                            (68U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE5_PEND_0                                            (69U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE6_PEND_0                                            (70U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE7_PEND_0                                            (71U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE8_PEND_0                                            (72U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE9_PEND_0                                            (73U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE10_PEND_0                                           (74U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE11_PEND_0                                           (75U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE12_PEND_0                                           (76U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE13_PEND_0                                           (77U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE14_PEND_0                                           (78U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE0_PCIE_CPTS_PEND_0                                        (79U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE0_PEND_0                                            (80U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE1_PEND_0                                            (81U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE2_PEND_0                                            (82U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE3_PEND_0                                            (83U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE4_PEND_0                                            (84U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE5_PEND_0                                            (85U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE6_PEND_0                                            (86U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE7_PEND_0                                            (87U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE8_PEND_0                                            (88U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE9_PEND_0                                            (89U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE10_PEND_0                                           (90U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE11_PEND_0                                           (91U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE12_PEND_0                                           (92U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE13_PEND_0                                           (93U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE14_PEND_0                                           (94U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_PCIE1_PCIE_CPTS_PEND_0                                        (95U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCSPI0_INTR_SPI_0                                             (96U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCSPI1_INTR_SPI_0                                             (97U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCSPI2_INTR_SPI_0                                             (98U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_MCSPI3_INTR_SPI_0                                             (99U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_I2C0_POINTRPEND_0                                             (100U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_I2C1_POINTRPEND_0                                             (101U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_I2C2_POINTRPEND_0                                             (102U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_I2C3_POINTRPEND_0                                             (103U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_UART0_USART_IRQ_0                                             (104U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_UART1_USART_IRQ_0                                             (105U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_UART2_USART_IRQ_0                                             (106U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER0_INTR_PEND_0                                            (108U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER1_INTR_PEND_0                                            (109U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER2_INTR_PEND_0                                            (110U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER3_INTR_PEND_0                                            (111U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER4_INTR_PEND_0                                            (112U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER5_INTR_PEND_0                                            (113U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER6_INTR_PEND_0                                            (114U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER7_INTR_PEND_0                                            (115U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER8_INTR_PEND_0                                            (116U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER9_INTR_PEND_0                                            (117U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER10_INTR_PEND_0                                           (118U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_TIMER11_INTR_PEND_0                                           (119U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC0_INTR_DONE_LEVEL_0                                        (120U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC1_INTR_DONE_LEVEL_0                                        (121U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC2_INTR_DONE_LEVEL_0                                        (122U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC3_INTR_DONE_LEVEL_0                                        (123U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC4_INTR_DONE_LEVEL_0                                        (124U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC5_INTR_DONE_LEVEL_0                                        (125U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC6_INTR_DONE_LEVEL_0                                        (126U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_DCC7_INTR_DONE_LEVEL_0                                        (127U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_OTG_LVL_0                                             (128U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_MISC_LVL_0                                            (129U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_BC_LVL_0                                              (130U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_PME_GEN_LVL_0                                         (131U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I00_LVL_0                                             (132U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I01_LVL_0                                             (133U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I02_LVL_0                                             (134U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I03_LVL_0                                             (135U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I04_LVL_0                                             (136U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I05_LVL_0                                             (137U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I06_LVL_0                                             (138U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I07_LVL_0                                             (139U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I08_LVL_0                                             (140U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I09_LVL_0                                             (141U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I10_LVL_0                                             (142U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I11_LVL_0                                             (143U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I12_LVL_0                                             (144U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I13_LVL_0                                             (145U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I14_LVL_0                                             (146U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS0_I15_LVL_0                                             (147U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_OTG_LVL_0                                             (148U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_MISC_LVL_0                                            (149U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_BC_LVL_0                                              (150U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_PME_GEN_LVL_0                                         (151U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I00_LVL_0                                             (152U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I01_LVL_0                                             (153U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I02_LVL_0                                             (154U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I03_LVL_0                                             (155U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I04_LVL_0                                             (156U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I05_LVL_0                                             (157U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I06_LVL_0                                             (158U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I07_LVL_0                                             (159U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I08_LVL_0                                             (160U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I09_LVL_0                                             (161U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I10_LVL_0                                             (162U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I11_LVL_0                                             (163U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I12_LVL_0                                             (164U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I13_LVL_0                                             (165U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I14_LVL_0                                             (166U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_USB3SS1_I15_LVL_0                                             (167U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CBASS0_LPSC_PER_COMMON_ERR_INTR_0                             (172U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CBASS_DEBUG0_LPSC_MAIN_DEBUG_ERR_INTR_0                       (173U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CBASS_FW0_LPSC_MAIN_INFRA_ERR_INTR_0                          (174U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_CBASS_INFRA0_LPSC_MAIN_INFRA_ERR_INTR_0                       (175U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_120                                   (184U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_121                                   (185U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_122                                   (186U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_123                                   (187U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_124                                   (188U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_125                                   (189U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_126                                   (190U)
#define CSLR_MAIN2MCU_LVL_INTRTR0_IN_NAVSS0_INTR_0_OUTL_INTR_127                                   (191U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_MAIN2MCU_LVL_INTRTR0_INTERRUPT_MAP_H_ */

