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
#ifndef CSLR_INTR_ESM2_H_
#define CSLR_INTR_ESM2_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* List of intr sources for receiver: esm2 */
/* This instance name corresponds to design instance name: esm_wkup_wkup_0 */
#define CSL_ESM2_INTR_WKUP_DMSC_TIMER0_INT          (0U)   /* WKUP_DMSC0_TIMER0 interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_TIMER1_INT          (1U)   /* WKUP_DMSC0_TIMER1 interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_TIMER2_INT          (2U)   /* WKUP_DMSC0_TIMER2 interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_TIMER3_INT          (3U)   /* WKUP_DMSC0_TIMER3 interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_WWD_INT_4           (4U)   /* WKUP_DMSC0 windowed watchdog timeout interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_RAT_EXP_INT         (5U)   /* WKUP_DMSC regional address translation exception interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_SEC_INT             (6U)   /* WKUP_DMSC0 SEC ECC error interrupt */
#define CSL_ESM2_INTR_WKUP_DMSC_DED_INT             (7U)   /* WKUP_DMSC0 DED ECC error interrupt */
#define CSL_ESM2_INTR_WKUP_VTM_OVRTMP_INT           (8U)   /* WKUP_VTM0 over temperature interrupt */
#define CSL_ESM2_INTR_WKUP_VTM_UNDTMP_INT           (9U)   /* WKUP_VTM0 under temperature interrupt */
#define CSL_ESM2_INTR_WKUP_VTM_MAXTMP_INT           (10U)  /* WKUP_VTM0 maximum temperature interrupt */
#define CSL_ESM2_INTR_WKUP_HFOSC_CLKLOSS_ERR_INT    (13U)  /* WKUP_HFOSC0 clock loss error interrupt */
#define CSL_ESM2_INTR_WKUP_CLK2_ECC_SEC_INT         (14U)  /* WKUP_SYSCLK/2 domain SEC ECC error interrupt */
#define CSL_ESM2_INTR_WKUP_CLK2_ECC_DED_INT         (15U)  /* WKUP_SYSCLK/2 domain DED ECC error interrupt */
#define CSL_ESM2_INTR_GLTCH_DET_CORE                (24U)  /* Sticky power glitch detect for CORE voltage domain */
#define CSL_ESM2_INTR_GLTCH_DET_MPU0                (25U)  /* Sticky power glitch detect for MPU0 voltage domain */
#define CSL_ESM2_INTR_GLTCH_DET_MPU1                (26U)  /* Sticky power glitch detect for MPU1 voltage domain */
#define CSL_ESM2_INTR_WKUP_LPSC_0_CLKSTOP_REQ       (32U)  /* WKUP_LPSC0 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_1_CLKSTOP_REQ       (33U)  /* WKUP_LPSC1 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_2_CLKSTOP_REQ       (34U)  /* WKUP_LPSC2 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_3_CLKSTOP_REQ       (35U)  /* WKUP_LPSC3 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_4_CLKSTOP_REQ       (36U)  /* WKUP_LPSC4 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_5_CLKSTOP_REQ       (37U)  /* WKUP_LPSC5 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_6_CLKSTOP_REQ       (38U)  /* WKUP_LPSC6 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_7_CLKSTOP_REQ       (39U)  /* WKUP_LPSC7 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_8_CLKSTOP_REQ       (40U)  /* WKUP_LPSC8 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_9_CLKSTOP_REQ       (41U)  /* WKUP_LPSC9 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_10_CLKSTOP_REQ      (42U)  /* WKUP_LPSC10 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_11_CLKSTOP_REQ      (43U)  /* WKUP_LPSC11 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_12_CLKSTOP_REQ      (44U)  /* WKUP_LPSC12 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_13_CLKSTOP_REQ      (45U)  /* WKUP_LPSC13 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_14_CLKSTOP_REQ      (46U)  /* WKUP_LPSC14 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_15_CLKSTOP_REQ      (47U)  /* WKUP_LPSC15 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_16_CLKSTOP_REQ      (48U)  /* WKUP_LPSC16 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_17_CLKSTOP_REQ      (49U)  /* WKUP_LPSC17 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_18_CLKSTOP_REQ      (50U)  /* WKUP_LPSC18 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_19_CLKSTOP_REQ      (51U)  /* WKUP_LPSC19 clockstop request */
#define CSL_ESM2_INTR_WKUP_LPSC_20_CLKSTOP_REQ      (52U)  /* WKUP_LPSC20 clockstop request */
#define CSL_ESM2_INTR_WKUP_PRG_WKUP_CORE_POK        (64U)  /* WKUP domain PRG - WKUP Core POK undervoltage signal */
#define CSL_ESM2_INTR_WKUP_PRG_PMIC_POK             (65U)  /* WKUP domain PRG - PMIC POK undervoltage signal*/
#define CSL_ESM2_INTR_WKUP_PRG_MCU_CORE_POK         (66U)  /* WKUP domain PRG - MCU Core POK undervoltage signal*/
#define CSL_ESM2_INTR_WKUP_PRG_WKUP_1P8V_POK        (67U)  /* WKUP domain PRG - WKUP 1.8V POK undervoltage signal */
#define CSL_ESM2_INTR_WKUP_PRG_WKUP_3P3V_POK        (68U)  /* WKUP domain PRG - WKUP 3.3V POK undervoltage signal */
#define CSL_ESM2_INTR_MAIN_PRG_MAIN_CORE_POK        (72U)  /* MAIN domain PRG - MAIN Core POK undervoltage signal */
#define CSL_ESM2_INTR_MAIN_PRG_MPU0_POK             (73U)  /* MAIN domain PRG - MPU0 POK undervoltage signal */
#define CSL_ESM2_INTR_MAIN_PRG_MPU1_POK             (74U)  /* MAIN domain PRG - MPU1 POK undervoltage signal */
#define CSL_ESM2_INTR_MAIN_PRG_DDRIO_POK            (75U)  /* MAIN domain PRG - DDRIO POK undervoltage signal */
#define CSL_ESM2_INTR_MAIN_PRG_MAIN_1P8V_POK        (76U)  /* MAIN domain PRG - MAIN 1.8V POK undervoltage signal */
#define CSL_ESM2_INTR_MAIN_PRG_MAIN_3P3V_POK        (77U)  /* MAIN domain PRG - MAIN 3.3V POK undervoltage signal */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT8             (88U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 8 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT9             (89U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 9 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT10            (90U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 10 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT11            (91U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 11 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT12            (92U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 12 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT13            (93U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 13 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT14            (94U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 14 */
#define CSL_ESM2_INTR_WKUP_GPIOMUX_INT15            (95U)  /* WKUP_GPIOMUX_INTRTR0 interrupt output 15 */

#ifdef __cplusplus
}
#endif
#endif /* CSLR_INTR_ESM2_H_*/
