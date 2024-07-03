/*
* R5FSS0_CORE0 INTERRUPT MAP. header file
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
#ifndef SDLR_R5FSS0_CORE0_INTERRUPT_MAP_H_
#define SDLR_R5FSS0_CORE0_INTERRUPT_MAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: R5FSS0_CORE0
*/


#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0      0U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1      1U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2      2U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_3      3U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_4      4U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_5      5U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_6      6U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_7      7U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_RX_SOF_INTR_REQ_0     8U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_RX_SOF_INTR_REQ_1     9U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_TX_SOF_INTR_REQ_0     10U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_TX_SOF_INTR_REQ_1     11U
#define  SDL_R5FSS0_CORE0_INTR_CPSW0_FH_INTR                        12U
#define  SDL_R5FSS0_CORE0_INTR_CPSW0_TH_INTR                        13U
#define  SDL_R5FSS0_CORE0_INTR_CPSW0_TH_THRESH_INTR                 14U
#define  SDL_R5FSS0_CORE0_INTR_CPSW0_MISC_INTR                      15U
#define  SDL_R5FSS0_CORE0_INTR_LIN0_INTR_0                          16U
#define  SDL_R5FSS0_CORE0_INTR_LIN0_INTR_1                          17U
#define  SDL_R5FSS0_CORE0_INTR_LIN1_INTR_0                          18U
#define  SDL_R5FSS0_CORE0_INTR_LIN1_INTR_1                          19U
#define  SDL_R5FSS0_CORE0_INTR_LIN2_INTR_0                          20U
#define  SDL_R5FSS0_CORE0_INTR_LIN2_INTR_1                          21U
#define  SDL_R5FSS0_CORE0_INTR_LIN3_INTR_0                          22U
#define  SDL_R5FSS0_CORE0_INTR_LIN3_INTR_1                          23U
#define  SDL_R5FSS0_CORE0_INTR_LIN4_INTR_0                          24U
#define  SDL_R5FSS0_CORE0_INTR_LIN4_INTR_1                          25U
#define  SDL_R5FSS0_CORE0_INTR_MCAN0_EXT_TS_ROLLOVER_LVL_INT_0      26U
#define  SDL_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_0                 27U
#define  SDL_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_1                 28U
#define  SDL_R5FSS0_CORE0_INTR_MCAN1_EXT_TS_ROLLOVER_LVL_INT_0      29U
#define  SDL_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_0                 30U
#define  SDL_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_1                 31U
#define  SDL_R5FSS0_CORE0_INTR_MCAN2_EXT_TS_ROLLOVER_LVL_INT_0      32U
#define  SDL_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_0                 33U
#define  SDL_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_1                 34U
#define  SDL_R5FSS0_CORE0_INTR_MCAN3_EXT_TS_ROLLOVER_LVL_INT_0      35U
#define  SDL_R5FSS0_CORE0_INTR_MCAN3_MCAN_LVL_INT_0                 36U
#define  SDL_R5FSS0_CORE0_INTR_MCAN3_MCAN_LVL_INT_1                 37U
#define  SDL_R5FSS0_CORE0_INTR_UART0_IRQ                            38U
#define  SDL_R5FSS0_CORE0_INTR_UART1_IRQ                            39U
#define  SDL_R5FSS0_CORE0_INTR_UART2_IRQ                            40U
#define  SDL_R5FSS0_CORE0_INTR_UART3_IRQ                            41U
#define  SDL_R5FSS0_CORE0_INTR_UART4_IRQ                            42U
#define  SDL_R5FSS0_CORE0_INTR_UART5_IRQ                            43U
#define  SDL_R5FSS0_CORE0_INTR_I2C0_IRQ                             44U
#define  SDL_R5FSS0_CORE0_INTR_I2C1_IRQ                             45U
#define  SDL_R5FSS0_CORE0_INTR_I2C2_IRQ                             46U
#define  SDL_R5FSS0_CORE0_INTR_I2C3_IRQ                             47U
#define  SDL_R5FSS0_CORE0_INTR_DTHE_SHA_S_INT                       48U
#define  SDL_R5FSS0_CORE0_INTR_DTHE_SHA_P_INT                       49U
#define  SDL_R5FSS0_CORE0_INTR_DTHE_TRNG_INT                        50U
#define  SDL_R5FSS0_CORE0_INTR_DTHE_PKAE_INT                        51U
#define  SDL_R5FSS0_CORE0_INTR_DTHE_AES_S_INT                       52U
#define  SDL_R5FSS0_CORE0_INTR_DTHE_AES_P_INT                       53U
#define  SDL_R5FSS0_CORE0_INTR_QSPI0_INT                            54U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INTG                           55U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_0                          56U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_1                          57U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_2                          58U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_3                          59U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_4                          60U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_5                          61U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_6                          62U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INT_7                          63U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_ERRINT                         64U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_MPINT                          65U
#define  SDL_R5FSS0_CORE0_INTR_TPTC0_ERINT_0                        66U
#define  SDL_R5FSS0_CORE0_INTR_TPTC0_ERINT_1                        67U
#define  SDL_R5FSS0_CORE0_INTR_MCRC0_INT                            68U
#define  SDL_R5FSS0_CORE0_INTR_MPU_ADDR_ERRAGG                      69U
#define  SDL_R5FSS0_CORE0_INTR_MPU_PROT_ERRAGG                      70U
#define  SDL_R5FSS0_CORE0_INTR_PBIST_DONE                           71U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_INTAGGR                        72U
#define  SDL_R5FSS0_CORE0_INTR_TPCC0_ERRAGGR                        73U
#define  SDL_R5FSS0_CORE0_INTR_DCC0_DONE                            74U
#define  SDL_R5FSS0_CORE0_INTR_DCC1_DONE                            75U
#define  SDL_R5FSS0_CORE0_INTR_DCC2_DONE                            76U
#define  SDL_R5FSS0_CORE0_INTR_DCC3_DONE                            77U
#define  SDL_R5FSS0_CORE0_INTR_MCSPI0_INTR                          78U
#define  SDL_R5FSS0_CORE0_INTR_MCSPI1_INTR                          79U
#define  SDL_R5FSS0_CORE0_INTR_MCSPI2_INTR                          80U
#define  SDL_R5FSS0_CORE0_INTR_MCSPI3_INTR                          81U
#define  SDL_R5FSS0_CORE0_INTR_MCSPI4_INTR                          82U
#define  SDL_R5FSS0_CORE0_INTR_MMC0_INTR                            83U
#define  SDL_R5FSS0_CORE0_INTR_RTI0_INTR_0                          84U
#define  SDL_R5FSS0_CORE0_INTR_RTI0_INTR_1                          85U
#define  SDL_R5FSS0_CORE0_INTR_RTI0_INTR_2                          86U
#define  SDL_R5FSS0_CORE0_INTR_RTI0_INTR_3                          87U
#define  SDL_R5FSS0_CORE0_INTR_RESERVED88                           88U
#define  SDL_R5FSS0_CORE0_INTR_RTI0_OVERFLOW_INT0                   89U
#define  SDL_R5FSS0_CORE0_INTR_RTI0_OVERFLOW_INT1                   90U
#define  SDL_R5FSS0_CORE0_INTR_RTI1_INTR_0                          91U
#define  SDL_R5FSS0_CORE0_INTR_RTI1_INTR_1                          92U
#define  SDL_R5FSS0_CORE0_INTR_RTI1_INTR_2                          93U
#define  SDL_R5FSS0_CORE0_INTR_RTI1_INTR_3                          94U
#define  SDL_R5FSS0_CORE0_INTR_RESERVED95                           95U
#define  SDL_R5FSS0_CORE0_INTR_RTI1_OVERFLOW_INT0                   96U
#define  SDL_R5FSS0_CORE0_INTR_RTI1_OVERFLOW_INT1                   97U
#define  SDL_R5FSS0_CORE0_INTR_RTI2_INTR_0                          98U
#define  SDL_R5FSS0_CORE0_INTR_RTI2_INTR_1                          99U
#define  SDL_R5FSS0_CORE0_INTR_RTI2_INTR_2                          100U
#define  SDL_R5FSS0_CORE0_INTR_RTI2_INTR_3                          101U
#define  SDL_R5FSS0_CORE0_INTR_RESERVED102                          102U
#define  SDL_R5FSS0_CORE0_INTR_RTI2_OVERFLOW_INT0                   103U
#define  SDL_R5FSS0_CORE0_INTR_RTI2_OVERFLOW_INT1                   104U
#define  SDL_R5FSS0_CORE0_INTR_RTI3_INTR_0                          105U
#define  SDL_R5FSS0_CORE0_INTR_RTI3_INTR_1                          106U
#define  SDL_R5FSS0_CORE0_INTR_RTI3_INTR_2                          107U
#define  SDL_R5FSS0_CORE0_INTR_RTI3_INTR_3                          108U
#define  SDL_R5FSS0_CORE0_INTR_RESERVED109                          109U
#define  SDL_R5FSS0_CORE0_INTR_RTI3_OVERFLOW_INT0                   110U
#define  SDL_R5FSS0_CORE0_INTR_RTI3_OVERFLOW_INT1                   111U
#define  SDL_R5FSS0_CORE0_INTR_RESERVED112                          112U
#define  SDL_R5FSS0_CORE0_INTR_ESM0_ESM_INT_CFG                     113U
#define  SDL_R5FSS0_CORE0_INTR_ESM0_ESM_INT_HI                      114U
#define  SDL_R5FSS0_CORE0_INTR_ESM0_ESM_INT_LOW                     115U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_COMMRX_0                       116U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_COMMTX_0                       117U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_CPU0_CTI_INT                   118U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_CPU0_VALFIQ                    119U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_CPU0_VALIRQ                    120U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_CPU1_CTI_INT                   121U
#define  SDL_R5FSS0_CORE0_INTR_R5SS1_CPU0_PMU_INT                   122U
#define  SDL_R5FSS0_CORE0_INTR_R5SS1_CPU1_PMU_INT                   123U
#define  SDL_R5FSS0_CORE0_INTR_MMR_ACC_ERRAGG                       124U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_LIVELOCK_1                     125U
#define  SDL_R5FSS0_CORE0_INTR_R5SS1_LIVELOCK_0                     126U
#define  SDL_R5FSS0_CORE0_INTR_R5SS1_LIVELOCK_1                     127U
#define  SDL_R5FSS0_CORE0_INTR_RTI_WDT0_NMI                         128U
#define  SDL_R5FSS0_CORE0_INTR_SW_IRQ                               129U
#define  SDL_R5FSS0_CORE0_INTR_R5SS0_CORE0_FPU_EXP                  130U
#define  SDL_R5FSS0_CORE0_INTR_DEBUGSS_TXDATA_AVAIL                 131U
#define  SDL_R5FSS0_CORE0_INTR_DEBUGSS_R5SS1_STC_DONE               132U
#define  SDL_R5FSS0_CORE0_INTR_TSENSE_H                             133U
#define  SDL_R5FSS0_CORE0_INTR_TSENSE_L                             134U
#define  SDL_R5FSS0_CORE0_INTR_AHB_WRITE_ERR                        135U
#define  SDL_R5FSS0_CORE0_INTR_MBOX_READ_REQ                        136U
#define  SDL_R5FSS0_CORE0_INTR_MBOX_READ_ACK                        137U
#define  SDL_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_2              138U
#define  SDL_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_3              139U
#define  SDL_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_4              140U
#define  SDL_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_5              141U
#define  SDL_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_14                 142U
#define  SDL_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_15                 143U
#define  SDL_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_16                 144U
#define  SDL_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_17                 145U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0                 146U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1                 147U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2                 148U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_3                 149U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_4                 150U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_5                 151U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_6                 152U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_7                 153U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_8                 154U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_9                 155U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_10                156U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_11                157U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_12                158U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_13                159U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_14                160U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_15                161U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_16                162U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_17                163U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_18                164U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_19                165U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_20                166U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_21                167U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_22                168U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_23                169U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_24                170U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_25                171U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_26                172U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_27                173U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_28                174U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_29                175U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_30                176U
#define  SDL_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_31                177U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_0   178U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_1   179U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_2   180U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_3   181U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_4   182U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_5   183U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_6   184U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_7   185U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_8   186U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_9   187U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_10  188U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_11  189U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_12  190U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_13  191U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_14  192U
#define  SDL_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_15  193U
#define  SDL_R5FSS0_CORE0_CPSW0_CPTS_COMP                           194U
#define  SDL_R5FSS0_CORE0_GPMC_SINTR                                195U
#define  SDL_R5FSS0_CORE0_ELM_SINTR                                 196U


#ifdef __cplusplus
}
#endif
#endif /* SDL_R5FSS0_CORE0_INTERRUPT_MAP_H_ */
