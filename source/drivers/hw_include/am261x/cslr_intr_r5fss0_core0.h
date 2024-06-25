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
#ifndef CSLR_R5FSS0_CORE0_INTERRUPT_MAP_H_
#define CSLR_R5FSS0_CORE0_INTERRUPT_MAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*
* List of intr sources for receiver: R5FSS0_CORE0
*/


#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0      0
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_1      1
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_2      2
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_3      3
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_4      4
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_5      5
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_6      6
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_7      7
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_RX_SOF_INTR_REQ_0     8
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_RX_SOF_INTR_REQ_1     9
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_TX_SOF_INTR_REQ_0     10
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_TX_SOF_INTR_REQ_1     11
#define  CSLR_R5FSS0_CORE0_INTR_CPSW0_FH_INTR                        12
#define  CSLR_R5FSS0_CORE0_INTR_CPSW0_TH_INTR                        13
#define  CSLR_R5FSS0_CORE0_INTR_CPSW0_TH_THRESH_INTR                 14
#define  CSLR_R5FSS0_CORE0_INTR_CPSW0_MISC_INTR                      15
#define  CSLR_R5FSS0_CORE0_INTR_LIN0_INTR_0                          16
#define  CSLR_R5FSS0_CORE0_INTR_LIN0_INTR_1                          17
#define  CSLR_R5FSS0_CORE0_INTR_LIN1_INTR_0                          18
#define  CSLR_R5FSS0_CORE0_INTR_LIN1_INTR_1                          19
#define  CSLR_R5FSS0_CORE0_INTR_LIN2_INTR_0                          20
#define  CSLR_R5FSS0_CORE0_INTR_LIN2_INTR_1                          21
#define  CSLR_R5FSS0_CORE0_INTR_LIN3_INTR_0                          22
#define  CSLR_R5FSS0_CORE0_INTR_LIN3_INTR_1                          23
#define  CSLR_R5FSS0_CORE0_INTR_LIN4_INTR_0                          24
#define  CSLR_R5FSS0_CORE0_INTR_LIN4_INTR_1                          25
#define  CSLR_R5FSS0_CORE0_INTR_MCAN0_EXT_TS_ROLLOVER_LVL_INT_0      26
#define  CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_0                 27
#define  CSLR_R5FSS0_CORE0_INTR_MCAN0_MCAN_LVL_INT_1                 28
#define  CSLR_R5FSS0_CORE0_INTR_MCAN1_EXT_TS_ROLLOVER_LVL_INT_0      29
#define  CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_0                 30
#define  CSLR_R5FSS0_CORE0_INTR_MCAN1_MCAN_LVL_INT_1                 31
#define  CSLR_R5FSS0_CORE0_INTR_MCAN2_EXT_TS_ROLLOVER_LVL_INT_0      32
#define  CSLR_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_0                 33
#define  CSLR_R5FSS0_CORE0_INTR_MCAN2_MCAN_LVL_INT_1                 34
#define  CSLR_R5FSS0_CORE0_INTR_MCAN3_EXT_TS_ROLLOVER_LVL_INT_0      35
#define  CSLR_R5FSS0_CORE0_INTR_MCAN3_MCAN_LVL_INT_0                 36
#define  CSLR_R5FSS0_CORE0_INTR_MCAN3_MCAN_LVL_INT_1                 37
#define  CSLR_R5FSS0_CORE0_INTR_UART0_IRQ                            38
#define  CSLR_R5FSS0_CORE0_INTR_UART1_IRQ                            39
#define  CSLR_R5FSS0_CORE0_INTR_UART2_IRQ                            40
#define  CSLR_R5FSS0_CORE0_INTR_UART3_IRQ                            41
#define  CSLR_R5FSS0_CORE0_INTR_UART4_IRQ                            42
#define  CSLR_R5FSS0_CORE0_INTR_UART5_IRQ                            43
#define  CSLR_R5FSS0_CORE0_INTR_I2C0_IRQ                             44
#define  CSLR_R5FSS0_CORE0_INTR_I2C1_IRQ                             45
#define  CSLR_R5FSS0_CORE0_INTR_I2C2_IRQ                             46
#define  CSLR_R5FSS0_CORE0_INTR_I2C3_IRQ                             47
#define  CSLR_R5FSS0_CORE0_INTR_DTHE_SHA_S_INT                       48
#define  CSLR_R5FSS0_CORE0_INTR_DTHE_SHA_P_INT                       49
#define  CSLR_R5FSS0_CORE0_INTR_DTHE_TRNG_INT                        50
#define  CSLR_R5FSS0_CORE0_INTR_DTHE_PKAE_INT                        51
#define  CSLR_R5FSS0_CORE0_INTR_DTHE_AES_S_INT                       52
#define  CSLR_R5FSS0_CORE0_INTR_DTHE_AES_P_INT                       53
#define  CSLR_R5FSS0_CORE0_INTR_OSPI0_INT                            54
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INTG                           55
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_0                          56
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_1                          57
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_2                          58
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_3                          59
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_4                          60
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_5                          61
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_6                          62
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_7                          63
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_ERRINT                         64
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_MPINT                          65
#define  CSLR_R5FSS0_CORE0_INTR_TPTC0_ERINT_0                        66
#define  CSLR_R5FSS0_CORE0_INTR_TPTC0_ERINT_1                        67
#define  CSLR_R5FSS0_CORE0_INTR_MCRC0_INT                            68
#define  CSLR_R5FSS0_CORE0_INTR_MPU_ADDR_ERRAGG                      69
#define  CSLR_R5FSS0_CORE0_INTR_MPU_PROT_ERRAGG                      70
#define  CSLR_R5FSS0_CORE0_INTR_PBIST_DONE                           71
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_INTAGGR                        72
#define  CSLR_R5FSS0_CORE0_INTR_TPCC0_ERRAGGR                        73
#define  CSLR_R5FSS0_CORE0_INTR_DCC0_DONE                            74
#define  CSLR_R5FSS0_CORE0_INTR_DCC1_DONE                            75
#define  CSLR_R5FSS0_CORE0_INTR_DCC2_DONE                            76
#define  CSLR_R5FSS0_CORE0_INTR_DCC3_DONE                            77
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI0_INTR                          78
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI1_INTR                          79
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI2_INTR                          80
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI3_INTR                          81
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI4_INTR                          82
#define  CSLR_R5FSS0_CORE0_INTR_MMC0_INTR                            83
#define  CSLR_R5FSS0_CORE0_INTR_RTI0_INTR_0                          84
#define  CSLR_R5FSS0_CORE0_INTR_RTI0_INTR_1                          85
#define  CSLR_R5FSS0_CORE0_INTR_RTI0_INTR_2                          86
#define  CSLR_R5FSS0_CORE0_INTR_RTI0_INTR_3                          87
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED88                           88
#define  CSLR_R5FSS0_CORE0_INTR_RTI0_OVERFLOW_INT0                   89
#define  CSLR_R5FSS0_CORE0_INTR_RTI0_OVERFLOW_INT1                   90
#define  CSLR_R5FSS0_CORE0_INTR_RTI1_INTR_0                          91
#define  CSLR_R5FSS0_CORE0_INTR_RTI1_INTR_1                          92
#define  CSLR_R5FSS0_CORE0_INTR_RTI1_INTR_2                          93
#define  CSLR_R5FSS0_CORE0_INTR_RTI1_INTR_3                          94
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED95                           95
#define  CSLR_R5FSS0_CORE0_INTR_RTI1_OVERFLOW_INT0                   96
#define  CSLR_R5FSS0_CORE0_INTR_RTI1_OVERFLOW_INT1                   97
#define  CSLR_R5FSS0_CORE0_INTR_RTI2_INTR_0                          98
#define  CSLR_R5FSS0_CORE0_INTR_RTI2_INTR_1                          99
#define  CSLR_R5FSS0_CORE0_INTR_RTI2_INTR_2                          100
#define  CSLR_R5FSS0_CORE0_INTR_RTI2_INTR_3                          101
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED102                          102
#define  CSLR_R5FSS0_CORE0_INTR_RTI2_OVERFLOW_INT0                   103
#define  CSLR_R5FSS0_CORE0_INTR_RTI2_OVERFLOW_INT1                   104
#define  CSLR_R5FSS0_CORE0_INTR_RTI3_INTR_0                          105
#define  CSLR_R5FSS0_CORE0_INTR_RTI3_INTR_1                          106
#define  CSLR_R5FSS0_CORE0_INTR_RTI3_INTR_2                          107
#define  CSLR_R5FSS0_CORE0_INTR_RTI3_INTR_3                          108
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED109                          109
#define  CSLR_R5FSS0_CORE0_INTR_RTI3_OVERFLOW_INT0                   110
#define  CSLR_R5FSS0_CORE0_INTR_RTI3_OVERFLOW_INT1                   111
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED112                          112
#define  CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_CFG                     113
#define  CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_HI                      114
#define  CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_LOW                     115
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_COMMRX_0                       116
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_COMMTX_0                       117
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU0_CTI_INT                   118
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU0_VALFIQ                    119
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU0_VALIRQ                    120
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU1_CTI_INT                   121
#define  CSLR_R5FSS0_CORE0_INTR_R5SS1_CPU0_PMU_INT                   122
#define  CSLR_R5FSS0_CORE0_INTR_R5SS1_CPU1_PMU_INT                   123
#define  CSLR_R5FSS0_CORE0_INTR_MMR_ACC_ERRAGG                       124
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_LIVELOCK_1                     125
#define  CSLR_R5FSS0_CORE0_INTR_R5SS1_LIVELOCK_0                     126
#define  CSLR_R5FSS0_CORE0_INTR_R5SS1_LIVELOCK_1                     127
#define  CSLR_R5FSS0_CORE0_INTR_RTI_WDT0_NMI                         128
#define  CSLR_R5FSS0_CORE0_INTR_SW_IRQ                               129
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CORE0_FPU_EXP                  130
#define  CSLR_R5FSS0_CORE0_INTR_DEBUGSS_TXDATA_AVAIL                 131
#define  CSLR_R5FSS0_CORE0_INTR_DEBUGSS_R5SS1_STC_DONE               132
#define  CSLR_R5FSS0_CORE0_INTR_TSENSE_H                             133
#define  CSLR_R5FSS0_CORE0_INTR_TSENSE_L                             134
#define  CSLR_R5FSS0_CORE0_INTR_AHB_WRITE_ERR                        135
#define  CSLR_R5FSS0_CORE0_INTR_MBOX_READ_REQ                        136
#define  CSLR_R5FSS0_CORE0_INTR_MBOX_READ_ACK                        137
#define  CSLR_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_2              138
#define  CSLR_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_3              139
#define  CSLR_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_4              140
#define  CSLR_R5FSS0_CORE0_INTR_SOC_TIMESYNCXBAR1_OUT_5              141
#define  CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_14                 142
#define  CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_15                 143
#define  CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_16                 144
#define  CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_17                 145
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0                 146
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1                 147
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_2                 148
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_3                 149
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_4                 150
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_5                 151
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_6                 152
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_7                 153
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_8                 154
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_9                 155
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_10                156
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_11                157
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_12                158
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_13                159
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_14                160
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_15                161
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_16                162
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_17                163
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_18                164
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_19                165
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_20                166
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_21                167
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_22                168
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_23                169
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_24                170
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_25                171
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_26                172
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_27                173
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_28                174
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_29                175
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_30                176
#define  CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_31                177
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_0   178
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_1   179
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_2   180
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_3   181
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_4   182
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_5   183
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_6   184
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_7   185
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_8   186
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_9   187
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_10  188
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_11  189
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_12  190
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_13  191
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_14  192
#define  CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_IEP0_CMP_INTR_REQ_15  193
#define  CSLR_R5FSS0_CORE0_CPSW0_CPTS_COMP                           194
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED195                          195
#define  CSLR_R5FSS0_CORE0_INTR_RESERVED196                          196
#define  CSLR_R5FSS0_CORE0_INTR_MCAN4_EXT_TS_ROLLOVER_LVL_INT_0      197
#define  CSLR_R5FSS0_CORE0_INTR_MCAN4_MCAN_LVL_INT_0                 198
#define  CSLR_R5FSS0_CORE0_INTR_MCAN4_MCAN_LVL_INT_1                 199
#define  CSLR_R5FSS0_CORE0_INTR_MCAN5_EXT_TS_ROLLOVER_LVL_INT_0      200
#define  CSLR_R5FSS0_CORE0_INTR_MCAN5_MCAN_LVL_INT_0                 201
#define  CSLR_R5FSS0_CORE0_INTR_MCAN5_MCAN_LVL_INT_1                 202
#define  CSLR_R5FSS0_CORE0_INTR_MCAN6_EXT_TS_ROLLOVER_LVL_INT_0      203
#define  CSLR_R5FSS0_CORE0_INTR_MCAN6_MCAN_LVL_INT_0                 204
#define  CSLR_R5FSS0_CORE0_INTR_MCAN6_MCAN_LVL_INT_1                 205
#define  CSLR_R5FSS0_CORE0_INTR_MCAN7_EXT_TS_ROLLOVER_LVL_INT_0      206
#define  CSLR_R5FSS0_CORE0_INTR_MCAN7_MCAN_LVL_INT_0                 207
#define  CSLR_R5FSS0_CORE0_INTR_MCAN7_MCAN_LVL_INT_1                 208
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU0_TMU_LVF                   209
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU0_TMU_LUF                   210
#define  CSLR_R5FSS0_CORE0_INTR_HW_RESOLVER_INTR                     211
#define  CSLR_R5FSS0_CORE0_INTR_FSS_VBUSM_TIMEOUT                    212
#define  CSLR_R5FSS0_CORE0_INTR_OTFA_ERROR                           213
#define  CSLR_R5FSS0_CORE0_INTR_FOTA_STAT_INTR                  	 214
#define  CSLR_R5FSS0_CORE0_INTR_FOTA_STAT_ERR_INTR	                 215
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI5_INTR                          216
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI6_INTR                          217
#define  CSLR_R5FSS0_CORE0_INTR_MCSPI7_INTR                          218
#define  CSLR_R5FSS0_CORE0_INTR_RTI4_INTR_0                          219
#define  CSLR_R5FSS0_CORE0_INTR_RTI4_INTR_1                          220
#define  CSLR_R5FSS0_CORE0_INTR_RTI4_INTR_2                          221
#define  CSLR_R5FSS0_CORE0_INTR_RTI4_INTR_3                          222
#define  CSLR_R5FSS0_CORE0_INTR_RTI4_OVERFLOW_INT0                   223
#define  CSLR_R5FSS0_CORE0_INTR_RTI4_OVERFLOW_INT1                   224
#define  CSLR_R5FSS0_CORE0_INTR_RTI5_INTR_0                          225
#define  CSLR_R5FSS0_CORE0_INTR_RTI5_INTR_1                          226
#define  CSLR_R5FSS0_CORE0_INTR_RTI5_INTR_2                          227
#define  CSLR_R5FSS0_CORE0_INTR_RTI5_INTR_3                          228
#define  CSLR_R5FSS0_CORE0_INTR_RTI5_OVERFLOW_INT0                   229
#define  CSLR_R5FSS0_CORE0_INTR_RTI5_OVERFLOW_INT1                   230
#define  CSLR_R5FSS0_CORE0_INTR_RTI6_INTR_0                          231
#define  CSLR_R5FSS0_CORE0_INTR_RTI6_INTR_1                          232
#define  CSLR_R5FSS0_CORE0_INTR_RTI6_INTR_2                          233
#define  CSLR_R5FSS0_CORE0_INTR_RTI6_INTR_3                          234
#define  CSLR_R5FSS0_CORE0_INTR_RTI6_OVERFLOW_INT0                   235
#define  CSLR_R5FSS0_CORE0_INTR_RTI6_OVERFLOW_INT1                   236
#define  CSLR_R5FSS0_CORE0_INTR_RTI7_INTR_0                          237
#define  CSLR_R5FSS0_CORE0_INTR_RTI7_INTR_1                          238
#define  CSLR_R5FSS0_CORE0_INTR_RTI7_INTR_2                          239
#define  CSLR_R5FSS0_CORE0_INTR_RTI7_INTR_3                          240
#define  CSLR_R5FSS0_CORE0_INTR_RTI7_OVERFLOW_INT0                   241
#define  CSLR_R5FSS0_CORE0_INTR_RTI7_OVERFLOW_INT1                   242
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU0_RL2_ERR_INTR              243
#define  CSLR_R5FSS0_CORE0_INTR_R5SS0_CPU1_RL2_ERR_INTR              244
#define  CSLR_R5FSS0_CORE0_INTR_R5SS1_CPU0_RL2_ERR_INTR              245
#define  CSLR_R5FSS0_CORE0_INTR_R5SS1_CPU1_RL3_ERR_INTR              246

#ifdef __cplusplus
}
#endif
#endif /* CSLR_R5FSS0_CORE0_INTERRUPT_MAP_H_ */
