/*
 *  Copyright (C) 2017-2022 Texas Instruments Incorporated
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
 * \ingroup TISCI
 * \defgroup tisci_devices tisci_devices
 *
 * DMSC controls the power management, security and resource management
 * of the device.
 *
 *
 * @{
 */
/**
 *
 *  \brief  This file contains:
 *
 *          WARNING!!: Autogenerated file from SYSFW. DO NOT MODIFY!!
 * Data version: 220228_160153
 *
 */
#ifndef SOC_TISCI_DEVICES_H
#define SOC_TISCI_DEVICES_H

#ifdef __cplusplus
extern "C"
{
#endif


#define TISCI_DEV_CMP_EVENT_INTROUTER0 1
#define TISCI_DEV_DBGSUSPENDROUTER0 2
#define TISCI_DEV_MAIN_GPIOMUX_INTROUTER0 3
#define TISCI_DEV_WKUP_MCU_GPIOMUX_INTROUTER0 5
#define TISCI_DEV_TIMESYNC_EVENT_ROUTER0 6
#define TISCI_DEV_MCU_M4FSS0 7
#define TISCI_DEV_MCU_M4FSS0_CBASS_0 8
#define TISCI_DEV_MCU_M4FSS0_CORE0 9
#define TISCI_DEV_CPSW0 13
#define TISCI_DEV_CPT2_AGGR0 14
#define TISCI_DEV_STM0 15
#define TISCI_DEV_DCC0 16
#define TISCI_DEV_DCC1 17
#define TISCI_DEV_DCC2 18
#define TISCI_DEV_DCC3 19
#define TISCI_DEV_DCC4 20
#define TISCI_DEV_DCC5 21
#define TISCI_DEV_SMS0 22
#define TISCI_DEV_MCU_DCC0 23
#define TISCI_DEV_DEBUGSS_WRAP0 24
#define TISCI_DEV_DMASS0 25
#define TISCI_DEV_DMASS0_BCDMA_0 26
#define TISCI_DEV_DMASS0_CBASS_0 27
#define TISCI_DEV_DMASS0_INTAGGR_0 28
#define TISCI_DEV_DMASS0_IPCSS_0 29
#define TISCI_DEV_DMASS0_PKTDMA_0 30
#define TISCI_DEV_DMASS0_RINGACC_0 33
#define TISCI_DEV_MCU_TIMER0 35
#define TISCI_DEV_TIMER0 36
#define TISCI_DEV_TIMER1 37
#define TISCI_DEV_TIMER2 38
#define TISCI_DEV_TIMER3 39
#define TISCI_DEV_TIMER4 40
#define TISCI_DEV_TIMER5 41
#define TISCI_DEV_TIMER6 42
#define TISCI_DEV_TIMER7 43
#define TISCI_DEV_MCU_TIMER1 48
#define TISCI_DEV_MCU_TIMER2 49
#define TISCI_DEV_MCU_TIMER3 50
#define TISCI_DEV_ECAP0 51
#define TISCI_DEV_ECAP1 52
#define TISCI_DEV_ECAP2 53
#define TISCI_DEV_ELM0 54
#define TISCI_DEV_EMIF_DATA_ISO_VD 55
#define TISCI_DEV_MMCSD0 57
#define TISCI_DEV_MMCSD1 58
#define TISCI_DEV_EQEP0 59
#define TISCI_DEV_EQEP1 60
#define TISCI_DEV_WKUP_GTC0 61
#define TISCI_DEV_EQEP2 62
#define TISCI_DEV_ESM0 63
#define TISCI_DEV_WKUP_ESM0 64
#define TISCI_DEV_SA3_SS0 65
#define TISCI_DEV_SA3_SS0_DMSS_ECCAGGR_0 66
#define TISCI_DEV_SA3_SS0_INTAGGR_0 67
#define TISCI_DEV_SA3_SS0_PKTDMA_0 68
#define TISCI_DEV_SA3_SS0_RINGACC_0 69
#define TISCI_DEV_SA3_SS0_SA_UL_0 70
#define TISCI_DEV_FSS0 73
#define TISCI_DEV_FSS0_FSAS_0 74
#define TISCI_DEV_FSS0_OSPI_0 75
#define TISCI_DEV_GICSS0 76
#define TISCI_DEV_GPIO0 77
#define TISCI_DEV_GPIO1 78
#define TISCI_DEV_MCU_GPIO0 79
#define TISCI_DEV_GPMC0 80
#define TISCI_DEV_ICSSM0 81
#define TISCI_DEV_LED0 83
#define TISCI_DEV_DDPA0 85
#define TISCI_DEV_EPWM0 86
#define TISCI_DEV_EPWM1 87
#define TISCI_DEV_EPWM2 88
#define TISCI_DEV_WKUP_VTM0 95
#define TISCI_DEV_MAILBOX0 96
#define TISCI_DEV_MAIN2MCU_VD 97
#define TISCI_DEV_MCAN0 98
#define TISCI_DEV_MCU_MCRC64_0 100
#define TISCI_DEV_MCU2MAIN_VD 101
#define TISCI_DEV_I2C0 102
#define TISCI_DEV_I2C1 103
#define TISCI_DEV_I2C2 104
#define TISCI_DEV_I2C3 105
#define TISCI_DEV_MCU_I2C0 106
#define TISCI_DEV_WKUP_I2C0 107
#define TISCI_DEV_WKUP_TIMER0 110
#define TISCI_DEV_WKUP_TIMER1 111
#define TISCI_DEV_WKUP_UART0 114
#define TISCI_DEV_MCRC64_0 116
#define TISCI_DEV_WKUP_RTCSS0 117
#define TISCI_DEV_R5FSS0_SS0 118
#define TISCI_DEV_R5FSS0 119
#define TISCI_DEV_R5FSS0_CORE0 121
#define TISCI_DEV_RTI0 125
#define TISCI_DEV_RTI1 126
#define TISCI_DEV_RTI2 127
#define TISCI_DEV_RTI3 128
#define TISCI_DEV_RTI15 130
#define TISCI_DEV_MCU_RTI0 131
#define TISCI_DEV_WKUP_RTI0 132
#define TISCI_DEV_COMPUTE_CLUSTER0 134
#define TISCI_DEV_A53SS0_CORE_0 135
#define TISCI_DEV_A53SS0_CORE_1 136
#define TISCI_DEV_A53SS0_CORE_2 137
#define TISCI_DEV_A53SS0_CORE_3 138
#define TISCI_DEV_PSC0 139
#define TISCI_DEV_WKUP_PSC0 140
#define TISCI_DEV_MCSPI0 141
#define TISCI_DEV_MCSPI1 142
#define TISCI_DEV_MCSPI2 143
#define TISCI_DEV_UART0 146
#define TISCI_DEV_MCU_MCSPI0 147
#define TISCI_DEV_MCU_MCSPI1 148
#define TISCI_DEV_MCU_UART0 149
#define TISCI_DEV_SPINLOCK0 150
#define TISCI_DEV_UART1 152
#define TISCI_DEV_UART2 153
#define TISCI_DEV_UART3 154
#define TISCI_DEV_UART4 155
#define TISCI_DEV_UART5 156
#define TISCI_DEV_BOARD0 157
#define TISCI_DEV_UART6 158
#define TISCI_DEV_USB0 161
#define TISCI_DEV_USB1 162
#define TISCI_DEV_PBIST0 163
#define TISCI_DEV_PBIST1 164
#define TISCI_DEV_WKUP_PBIST0 165
#define TISCI_DEV_A53SS0 166
#define TISCI_DEV_COMPUTE_CLUSTER0_PBIST_0 167
#define TISCI_DEV_PSC0_FW_0 168
#define TISCI_DEV_PSC0_PSC_0 169
#define TISCI_DEV_DDR16SS0 170
#define TISCI_DEV_DEBUGSS0 171
#define TISCI_DEV_A53_RS_BW_LIMITER0 172
#define TISCI_DEV_A53_WS_BW_LIMITER1 173
#define TISCI_DEV_GPU_RS_BW_LIMITER2 174
#define TISCI_DEV_GPU_WS_BW_LIMITER3 175
#define TISCI_DEV_WKUP_DEEPSLEEP_SOURCES0 176
#define TISCI_DEV_EMIF_CFG_ISO_VD 177
#define TISCI_DEV_MAIN_USB0_ISO_VD 178
#define TISCI_DEV_MAIN_USB1_ISO_VD 179
#define TISCI_DEV_MCU_MCU_16FF0 180
#define TISCI_DEV_CPT2_AGGR1 181
#define TISCI_DEV_CSI_RX_IF0 182
#define TISCI_DEV_DCC6 183
#define TISCI_DEV_MMCSD2 184
#define TISCI_DEV_DPHY_RX0 185
#define TISCI_DEV_DSS0 186
#define TISCI_DEV_GPU0 187
#define TISCI_DEV_MCU_MCAN0 188
#define TISCI_DEV_MCU_MCAN1 189
#define TISCI_DEV_MCASP0 190
#define TISCI_DEV_MCASP1 191
#define TISCI_DEV_MCASP2 192
#define TISCI_DEV_CLK_32K_RC_SEL_DEV_VD 193


#ifdef __cplusplus
}
#endif

#endif /* SOC_TISCI_DEVICES_H */

/* @} */
