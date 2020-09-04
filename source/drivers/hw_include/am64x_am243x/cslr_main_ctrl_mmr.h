/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_main_ctrl_mmr.h
*/
#ifndef CSLR_MAIN_CTRL_MMR_H_
#define CSLR_MAIN_CTRL_MMR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_MAIN_CTRL_MMR_CFG0_REGS_BASE                                  (0x00000000U)


/**************************************************************************
* Hardware Region  : MMRs in region 0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* PID register */
    volatile uint32_t MMR_CFG0;
    volatile uint32_t MMR_CFG1;
    volatile uint8_t  Resv_20[8];
    volatile uint32_t JTAGID;
    volatile uint32_t JTAG_USER_ID;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t DIE_ID0;
    volatile uint32_t DIE_ID1;
    volatile uint32_t DIE_ID2;
    volatile uint32_t DIE_ID3;
    volatile uint32_t MAIN_DEVSTAT;
    volatile uint32_t MAIN_BOOTCFG;
    volatile uint8_t  Resv_64[8];
    volatile uint32_t A53SS_REVID;
    volatile uint32_t BOOT_PROGRESS;
    volatile uint8_t  Resv_76[4];
    volatile uint32_t PLL_INIT;
    volatile uint8_t  Resv_96[16];
    volatile uint32_t DEVICE_FEATURE0;
    volatile uint8_t  Resv_104[4];
    volatile uint32_t DEVICE_FEATURE2;
    volatile uint8_t  Resv_120[12];
    volatile uint32_t DEVICE_FEATURE6;
    volatile uint8_t  Resv_512[388];
    volatile uint32_t MAC_ID0;
    volatile uint32_t MAC_ID1;
    volatile uint8_t  Resv_528[8];
    volatile uint32_t PCI_DEVICE_ID0;
    volatile uint32_t PCI_DEVICE_ID1;
    volatile uint8_t  Resv_544[8];
    volatile uint32_t USB_DEVICE_ID0;
    volatile uint32_t USB_DEVICE_ID1;
    volatile uint8_t  Resv_560[8];
    volatile uint32_t GP_SW0;
    volatile uint32_t GP_SW1;
    volatile uint32_t GP_SW2;
    volatile uint32_t GP_SW3;
    volatile uint8_t  Resv_624[48];
    volatile uint32_t CBA_ERR_STAT;
    volatile uint8_t  Resv_768[140];
    volatile uint32_t SPARE_FUSE0;
    volatile uint8_t  Resv_4104[3332];
    volatile uint32_t LOCK0_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK0_KICK1;               /*  - KICK1 component */
    volatile uint32_t INTR_RAW_STATUS;           /* Interrupt Raw Status/Set Register */
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;   /* Interrupt Enabled Status/Clear register */
    volatile uint32_t INTR_ENABLE;               /* Interrupt Enable register */
    volatile uint32_t INTR_ENABLE_CLEAR;         /* Interrupt Enable Clear register */
    volatile uint32_t EOI;                       /* EOI register */
    volatile uint32_t FAULT_ADDRESS;             /* Fault Address register */
    volatile uint32_t FAULT_TYPE_STATUS;         /* Fault Type Status register */
    volatile uint32_t FAULT_ATTR_STATUS;         /* Fault Attribute Status register */
    volatile uint32_t FAULT_CLEAR;               /* Fault Clear register */
    volatile uint8_t  Resv_4352[204];
    volatile uint32_t CLAIMREG_P0_R0_READONLY;   /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R1_READONLY;   /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R2_READONLY;   /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R3_READONLY;   /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R4_READONLY;   /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R5_READONLY;   /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R6_READONLY;   /* Claim bits for Partition 0 */
    volatile uint8_t  Resv_8192[3812];
    volatile uint32_t PID_PROXY;                 /* PID register */
    volatile uint32_t MMR_CFG0_PROXY;
    volatile uint32_t MMR_CFG1_PROXY;
    volatile uint8_t  Resv_8212[8];
    volatile uint32_t JTAGID_PROXY;
    volatile uint32_t JTAG_USER_ID_PROXY;
    volatile uint8_t  Resv_8224[4];
    volatile uint32_t DIE_ID0_PROXY;
    volatile uint32_t DIE_ID1_PROXY;
    volatile uint32_t DIE_ID2_PROXY;
    volatile uint32_t DIE_ID3_PROXY;
    volatile uint32_t MAIN_DEVSTAT_PROXY;
    volatile uint32_t MAIN_BOOTCFG_PROXY;
    volatile uint8_t  Resv_8256[8];
    volatile uint32_t A53SS_REVID_PROXY;
    volatile uint32_t BOOT_PROGRESS_PROXY;
    volatile uint8_t  Resv_8268[4];
    volatile uint32_t PLL_INIT_PROXY;
    volatile uint8_t  Resv_8288[16];
    volatile uint32_t DEVICE_FEATURE0_PROXY;
    volatile uint8_t  Resv_8296[4];
    volatile uint32_t DEVICE_FEATURE2_PROXY;
    volatile uint8_t  Resv_8312[12];
    volatile uint32_t DEVICE_FEATURE6_PROXY;
    volatile uint8_t  Resv_8704[388];
    volatile uint32_t MAC_ID0_PROXY;
    volatile uint32_t MAC_ID1_PROXY;
    volatile uint8_t  Resv_8720[8];
    volatile uint32_t PCI_DEVICE_ID0_PROXY;
    volatile uint32_t PCI_DEVICE_ID1_PROXY;
    volatile uint8_t  Resv_8736[8];
    volatile uint32_t USB_DEVICE_ID0_PROXY;
    volatile uint32_t USB_DEVICE_ID1_PROXY;
    volatile uint8_t  Resv_8752[8];
    volatile uint32_t GP_SW0_PROXY;
    volatile uint32_t GP_SW1_PROXY;
    volatile uint32_t GP_SW2_PROXY;
    volatile uint32_t GP_SW3_PROXY;
    volatile uint8_t  Resv_8816[48];
    volatile uint32_t CBA_ERR_STAT_PROXY;
    volatile uint8_t  Resv_8960[140];
    volatile uint32_t SPARE_FUSE0_PROXY;
    volatile uint8_t  Resv_12296[3332];
    volatile uint32_t LOCK0_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK0_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint32_t INTR_RAW_STATUS_PROXY;     /* Interrupt Raw Status/Set Register */
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR_PROXY;   /* Interrupt Enabled Status/Clear register */
    volatile uint32_t INTR_ENABLE_PROXY;         /* Interrupt Enable register */
    volatile uint32_t INTR_ENABLE_CLEAR_PROXY;   /* Interrupt Enable Clear register */
    volatile uint32_t EOI_PROXY;                 /* EOI register */
    volatile uint32_t FAULT_ADDRESS_PROXY;       /* Fault Address register */
    volatile uint32_t FAULT_TYPE_STATUS_PROXY;   /* Fault Type Status register */
    volatile uint32_t FAULT_ATTR_STATUS_PROXY;   /* Fault Attribute Status register */
    volatile uint32_t FAULT_CLEAR_PROXY;         /* Fault Clear register */
    volatile uint8_t  Resv_12544[204];
    volatile uint32_t CLAIMREG_P0_R0;            /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R1;            /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R2;            /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R3;            /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R4;            /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R5;            /* Claim bits for Partition 0 */
    volatile uint32_t CLAIMREG_P0_R6;            /* Claim bits for Partition 0 */
    volatile uint8_t  Resv_16392[3820];
    volatile uint32_t USB0_PHY_CTRL;
    volatile uint8_t  Resv_16452[56];
    volatile uint32_t ENET1_CTRL;
    volatile uint32_t ENET2_CTRL;
    volatile uint8_t  Resv_16496[36];
    volatile uint32_t PCIE0_CTRL;
    volatile uint8_t  Resv_16512[12];
    volatile uint32_t SERDES0_LN0_CTRL;
    volatile uint8_t  Resv_16576[60];
    volatile uint32_t ADC0_TRIM;
    volatile uint8_t  Resv_16608[28];
    volatile uint32_t SERDES0_CTRL;
    volatile uint8_t  Resv_16640[28];
    volatile uint32_t ICSSG0_CTRL0;
    volatile uint32_t ICSSG0_CTRL1;
    volatile uint8_t  Resv_16656[8];
    volatile uint32_t ICSSG1_CTRL0;
    volatile uint32_t ICSSG1_CTRL1;
    volatile uint8_t  Resv_16688[24];
    volatile uint32_t EPWM_TB_CLKEN;
    volatile uint32_t EPWM_TB_CLKEN_SET;
    volatile uint32_t EPWM_TB_CLKEN_CLR;
    volatile uint8_t  Resv_16704[4];
    volatile uint32_t EPWM0_CTRL;
    volatile uint32_t EPWM1_CTRL;
    volatile uint32_t EPWM2_CTRL;
    volatile uint32_t EPWM3_CTRL;
    volatile uint32_t EPWM4_CTRL;
    volatile uint32_t EPWM5_CTRL;
    volatile uint32_t EPWM6_CTRL;
    volatile uint32_t EPWM7_CTRL;
    volatile uint32_t EPWM8_CTRL;
    volatile uint8_t  Resv_16752[12];
    volatile uint32_t SOCA_SEL;
    volatile uint32_t SOCB_SEL;
    volatile uint8_t  Resv_16768[8];
    volatile uint32_t EQEP0_CTRL;
    volatile uint32_t EQEP1_CTRL;
    volatile uint32_t EQEP2_CTRL;
    volatile uint8_t  Resv_16800[20];
    volatile uint32_t EQEP_STAT;
    volatile uint8_t  Resv_16820[16];
    volatile uint32_t SDIO1_CTRL;
    volatile uint8_t  Resv_16900[76];
    volatile uint32_t TIMER1_CTRL;
    volatile uint8_t  Resv_16908[4];
    volatile uint32_t TIMER3_CTRL;
    volatile uint8_t  Resv_16916[4];
    volatile uint32_t TIMER5_CTRL;
    volatile uint8_t  Resv_16924[4];
    volatile uint32_t TIMER7_CTRL;
    volatile uint8_t  Resv_16932[4];
    volatile uint32_t TIMER9_CTRL;
    volatile uint8_t  Resv_16940[4];
    volatile uint32_t TIMER11_CTRL;
    volatile uint8_t  Resv_17120[176];
    volatile uint32_t I2C0_CTRL;
    volatile uint8_t  Resv_17408[284];
    volatile uint32_t SPARE_CTRL0;
    volatile uint32_t SPARE_CTRL1;
    volatile uint8_t  Resv_17472[56];
    volatile uint32_t SPARE_STAT0;
    volatile uint32_t SPARE_STAT1;
    volatile uint8_t  Resv_17536[56];
    volatile uint32_t SPARE_TRIM0;
    volatile uint8_t  Resv_18176[636];
    volatile uint32_t FSS_CTRL;
    volatile uint8_t  Resv_18192[12];
    volatile uint32_t ADC0_CTRL;
    volatile uint8_t  Resv_18256[60];
    volatile uint32_t DCC_STAT;
    volatile uint8_t  Resv_20488[2228];
    volatile uint32_t LOCK1_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK1_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_20736[240];
    volatile uint32_t CLAIMREG_P1_R0_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R1_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R2_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R3_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R4_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R5_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R6_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R7_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R8_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R9_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R10_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R11_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R12_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R13_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R14_READONLY;   /* Claim bits for Partition 1 */
    volatile uint8_t  Resv_24584[3788];
    volatile uint32_t USB0_PHY_CTRL_PROXY;
    volatile uint8_t  Resv_24644[56];
    volatile uint32_t ENET1_CTRL_PROXY;
    volatile uint32_t ENET2_CTRL_PROXY;
    volatile uint8_t  Resv_24688[36];
    volatile uint32_t PCIE0_CTRL_PROXY;
    volatile uint8_t  Resv_24704[12];
    volatile uint32_t SERDES0_LN0_CTRL_PROXY;
    volatile uint8_t  Resv_24768[60];
    volatile uint32_t ADC0_TRIM_PROXY;
    volatile uint8_t  Resv_24800[28];
    volatile uint32_t SERDES0_CTRL_PROXY;
    volatile uint8_t  Resv_24832[28];
    volatile uint32_t ICSSG0_CTRL0_PROXY;
    volatile uint32_t ICSSG0_CTRL1_PROXY;
    volatile uint8_t  Resv_24848[8];
    volatile uint32_t ICSSG1_CTRL0_PROXY;
    volatile uint32_t ICSSG1_CTRL1_PROXY;
    volatile uint8_t  Resv_24880[24];
    volatile uint32_t EPWM_TB_CLKEN_PROXY;
    volatile uint32_t EPWM_TB_CLKEN_SET_PROXY;
    volatile uint32_t EPWM_TB_CLKEN_CLR_PROXY;
    volatile uint8_t  Resv_24896[4];
    volatile uint32_t EPWM0_CTRL_PROXY;
    volatile uint32_t EPWM1_CTRL_PROXY;
    volatile uint32_t EPWM2_CTRL_PROXY;
    volatile uint32_t EPWM3_CTRL_PROXY;
    volatile uint32_t EPWM4_CTRL_PROXY;
    volatile uint32_t EPWM5_CTRL_PROXY;
    volatile uint32_t EPWM6_CTRL_PROXY;
    volatile uint32_t EPWM7_CTRL_PROXY;
    volatile uint32_t EPWM8_CTRL_PROXY;
    volatile uint8_t  Resv_24944[12];
    volatile uint32_t SOCA_SEL_PROXY;
    volatile uint32_t SOCB_SEL_PROXY;
    volatile uint8_t  Resv_24960[8];
    volatile uint32_t EQEP0_CTRL_PROXY;
    volatile uint32_t EQEP1_CTRL_PROXY;
    volatile uint32_t EQEP2_CTRL_PROXY;
    volatile uint8_t  Resv_24992[20];
    volatile uint32_t EQEP_STAT_PROXY;
    volatile uint8_t  Resv_25012[16];
    volatile uint32_t SDIO1_CTRL_PROXY;
    volatile uint8_t  Resv_25092[76];
    volatile uint32_t TIMER1_CTRL_PROXY;
    volatile uint8_t  Resv_25100[4];
    volatile uint32_t TIMER3_CTRL_PROXY;
    volatile uint8_t  Resv_25108[4];
    volatile uint32_t TIMER5_CTRL_PROXY;
    volatile uint8_t  Resv_25116[4];
    volatile uint32_t TIMER7_CTRL_PROXY;
    volatile uint8_t  Resv_25124[4];
    volatile uint32_t TIMER9_CTRL_PROXY;
    volatile uint8_t  Resv_25132[4];
    volatile uint32_t TIMER11_CTRL_PROXY;
    volatile uint8_t  Resv_25312[176];
    volatile uint32_t I2C0_CTRL_PROXY;
    volatile uint8_t  Resv_25600[284];
    volatile uint32_t SPARE_CTRL0_PROXY;
    volatile uint32_t SPARE_CTRL1_PROXY;
    volatile uint8_t  Resv_25664[56];
    volatile uint32_t SPARE_STAT0_PROXY;
    volatile uint32_t SPARE_STAT1_PROXY;
    volatile uint8_t  Resv_25728[56];
    volatile uint32_t SPARE_TRIM0_PROXY;
    volatile uint8_t  Resv_26368[636];
    volatile uint32_t FSS_CTRL_PROXY;
    volatile uint8_t  Resv_26384[12];
    volatile uint32_t ADC0_CTRL_PROXY;
    volatile uint8_t  Resv_26448[60];
    volatile uint32_t DCC_STAT_PROXY;
    volatile uint8_t  Resv_28680[2228];
    volatile uint32_t LOCK1_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK1_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_28928[240];
    volatile uint32_t CLAIMREG_P1_R0;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R1;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R2;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R3;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R4;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R5;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R6;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R7;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R8;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R9;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R10;           /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R11;           /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R12;           /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R13;           /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R14;           /* Claim bits for Partition 1 */
    volatile uint8_t  Resv_32768[3780];
    volatile uint32_t OBSCLK0_CTRL;
    volatile uint8_t  Resv_32784[12];
    volatile uint32_t CLKOUT_CTRL;
    volatile uint8_t  Resv_32816[28];
    volatile uint32_t GTC_CLKSEL;
    volatile uint8_t  Resv_32828[8];
    volatile uint32_t EFUSE_CLKSEL;
    volatile uint32_t ICSSG0_CLKSEL;
    volatile uint32_t ICSSG1_CLKSEL;
    volatile uint8_t  Resv_32864[24];
    volatile uint32_t MAIN_PLL0_CLKSEL;
    volatile uint32_t MAIN_PLL1_CLKSEL;
    volatile uint32_t MAIN_PLL2_CLKSEL;
    volatile uint8_t  Resv_32896[20];
    volatile uint32_t MAIN_PLL8_CLKSEL;
    volatile uint8_t  Resv_32912[12];
    volatile uint32_t MAIN_PLL12_CLKSEL;
    volatile uint8_t  Resv_32920[4];
    volatile uint32_t MAIN_PLL14_CLKSEL;
    volatile uint8_t  Resv_33056[132];
    volatile uint32_t PCIE0_CLKSEL;
    volatile uint8_t  Resv_33088[28];
    volatile uint32_t CPSW_CLKSEL;
    volatile uint8_t  Resv_33104[12];
    volatile uint32_t CPTS_CLKSEL;
    volatile uint8_t  Resv_33120[12];
    volatile uint32_t EMMC0_CLKSEL;
    volatile uint8_t  Resv_33128[4];
    volatile uint32_t EMMC1_CLKSEL;
    volatile uint8_t  Resv_33152[20];
    volatile uint32_t GPMC_CLKSEL;
    volatile uint8_t  Resv_33168[12];
    volatile uint32_t USB0_CLKSEL;
    volatile uint8_t  Resv_33200[28];
    volatile uint32_t TIMER0_CLKSEL;
    volatile uint32_t TIMER1_CLKSEL;
    volatile uint32_t TIMER2_CLKSEL;
    volatile uint32_t TIMER3_CLKSEL;
    volatile uint32_t TIMER4_CLKSEL;
    volatile uint32_t TIMER5_CLKSEL;
    volatile uint32_t TIMER6_CLKSEL;
    volatile uint32_t TIMER7_CLKSEL;
    volatile uint32_t TIMER8_CLKSEL;
    volatile uint32_t TIMER9_CLKSEL;
    volatile uint32_t TIMER10_CLKSEL;
    volatile uint32_t TIMER11_CLKSEL;
    volatile uint8_t  Resv_33280[32];
    volatile uint32_t SPI0_CLKSEL;
    volatile uint32_t SPI1_CLKSEL;
    volatile uint32_t SPI2_CLKSEL;
    volatile uint32_t SPI3_CLKSEL;
    volatile uint32_t SPI4_CLKSEL;
    volatile uint8_t  Resv_33344[44];
    volatile uint32_t USART0_CLK_CTRL;
    volatile uint32_t USART1_CLK_CTRL;
    volatile uint32_t USART2_CLK_CTRL;
    volatile uint32_t USART3_CLK_CTRL;
    volatile uint32_t USART4_CLK_CTRL;
    volatile uint32_t USART5_CLK_CTRL;
    volatile uint32_t USART6_CLK_CTRL;
    volatile uint8_t  Resv_33408[36];
    volatile uint32_t USART0_CLKSEL;
    volatile uint32_t USART1_CLKSEL;
    volatile uint32_t USART2_CLKSEL;
    volatile uint32_t USART3_CLKSEL;
    volatile uint32_t USART4_CLKSEL;
    volatile uint32_t USART5_CLKSEL;
    volatile uint32_t USART6_CLKSEL;
    volatile uint8_t  Resv_33664[228];
    volatile uint32_t WWD0_CLKSEL;
    volatile uint32_t WWD1_CLKSEL;
    volatile uint8_t  Resv_33696[24];
    volatile uint32_t WWD8_CLKSEL;
    volatile uint32_t WWD9_CLKSEL;
    volatile uint32_t WWD10_CLKSEL;
    volatile uint32_t WWD11_CLKSEL;
    volatile uint8_t  Resv_33792[80];
    volatile uint32_t SERDES0_CLKSEL;
    volatile uint8_t  Resv_33920[124];
    volatile uint32_t MCAN0_CLKSEL;
    volatile uint32_t MCAN1_CLKSEL;
    volatile uint8_t  Resv_34048[120];
    volatile uint32_t OSPI0_CLKSEL;
    volatile uint8_t  Resv_34064[12];
    volatile uint32_t ADC0_CLKSEL;
    volatile uint8_t  Resv_34144[76];
    volatile uint32_t DDR0_RFCLK_TEST0_CLK_CTRL;
    volatile uint32_t DDR0_RFCLK_TEST1_CLK_CTRL;
    volatile uint8_t  Resv_34160[8];
    volatile uint32_t DDR0_TESTOUT0_CLK_CTRL;
    volatile uint32_t DDR0_TESTOUT1_CLK_CTRL;
    volatile uint8_t  Resv_36872[2704];
    volatile uint32_t LOCK2_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK2_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_37120[240];
    volatile uint32_t CLAIMREG_P2_R0_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R1_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R2_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R3_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R4_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R5_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R6_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R7_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R8_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R9_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R10_READONLY;   /* Claim bits for Partition 2 */
    volatile uint8_t  Resv_40960[3796];
    volatile uint32_t OBSCLK0_CTRL_PROXY;
    volatile uint8_t  Resv_40976[12];
    volatile uint32_t CLKOUT_CTRL_PROXY;
    volatile uint8_t  Resv_41008[28];
    volatile uint32_t GTC_CLKSEL_PROXY;
    volatile uint8_t  Resv_41020[8];
    volatile uint32_t EFUSE_CLKSEL_PROXY;
    volatile uint32_t ICSSG0_CLKSEL_PROXY;
    volatile uint32_t ICSSG1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41056[24];
    volatile uint32_t MAIN_PLL0_CLKSEL_PROXY;
    volatile uint32_t MAIN_PLL1_CLKSEL_PROXY;
    volatile uint32_t MAIN_PLL2_CLKSEL_PROXY;
    volatile uint8_t  Resv_41088[20];
    volatile uint32_t MAIN_PLL8_CLKSEL_PROXY;
    volatile uint8_t  Resv_41104[12];
    volatile uint32_t MAIN_PLL12_CLKSEL_PROXY;
    volatile uint8_t  Resv_41112[4];
    volatile uint32_t MAIN_PLL14_CLKSEL_PROXY;
    volatile uint8_t  Resv_41248[132];
    volatile uint32_t PCIE0_CLKSEL_PROXY;
    volatile uint8_t  Resv_41280[28];
    volatile uint32_t CPSW_CLKSEL_PROXY;
    volatile uint8_t  Resv_41296[12];
    volatile uint32_t CPTS_CLKSEL_PROXY;
    volatile uint8_t  Resv_41312[12];
    volatile uint32_t EMMC0_CLKSEL_PROXY;
    volatile uint8_t  Resv_41320[4];
    volatile uint32_t EMMC1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41344[20];
    volatile uint32_t GPMC_CLKSEL_PROXY;
    volatile uint8_t  Resv_41360[12];
    volatile uint32_t USB0_CLKSEL_PROXY;
    volatile uint8_t  Resv_41392[28];
    volatile uint32_t TIMER0_CLKSEL_PROXY;
    volatile uint32_t TIMER1_CLKSEL_PROXY;
    volatile uint32_t TIMER2_CLKSEL_PROXY;
    volatile uint32_t TIMER3_CLKSEL_PROXY;
    volatile uint32_t TIMER4_CLKSEL_PROXY;
    volatile uint32_t TIMER5_CLKSEL_PROXY;
    volatile uint32_t TIMER6_CLKSEL_PROXY;
    volatile uint32_t TIMER7_CLKSEL_PROXY;
    volatile uint32_t TIMER8_CLKSEL_PROXY;
    volatile uint32_t TIMER9_CLKSEL_PROXY;
    volatile uint32_t TIMER10_CLKSEL_PROXY;
    volatile uint32_t TIMER11_CLKSEL_PROXY;
    volatile uint8_t  Resv_41472[32];
    volatile uint32_t SPI0_CLKSEL_PROXY;
    volatile uint32_t SPI1_CLKSEL_PROXY;
    volatile uint32_t SPI2_CLKSEL_PROXY;
    volatile uint32_t SPI3_CLKSEL_PROXY;
    volatile uint32_t SPI4_CLKSEL_PROXY;
    volatile uint8_t  Resv_41536[44];
    volatile uint32_t USART0_CLK_CTRL_PROXY;
    volatile uint32_t USART1_CLK_CTRL_PROXY;
    volatile uint32_t USART2_CLK_CTRL_PROXY;
    volatile uint32_t USART3_CLK_CTRL_PROXY;
    volatile uint32_t USART4_CLK_CTRL_PROXY;
    volatile uint32_t USART5_CLK_CTRL_PROXY;
    volatile uint32_t USART6_CLK_CTRL_PROXY;
    volatile uint8_t  Resv_41600[36];
    volatile uint32_t USART0_CLKSEL_PROXY;
    volatile uint32_t USART1_CLKSEL_PROXY;
    volatile uint32_t USART2_CLKSEL_PROXY;
    volatile uint32_t USART3_CLKSEL_PROXY;
    volatile uint32_t USART4_CLKSEL_PROXY;
    volatile uint32_t USART5_CLKSEL_PROXY;
    volatile uint32_t USART6_CLKSEL_PROXY;
    volatile uint8_t  Resv_41856[228];
    volatile uint32_t WWD0_CLKSEL_PROXY;
    volatile uint32_t WWD1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41888[24];
    volatile uint32_t WWD8_CLKSEL_PROXY;
    volatile uint32_t WWD9_CLKSEL_PROXY;
    volatile uint32_t WWD10_CLKSEL_PROXY;
    volatile uint32_t WWD11_CLKSEL_PROXY;
    volatile uint8_t  Resv_41984[80];
    volatile uint32_t SERDES0_CLKSEL_PROXY;
    volatile uint8_t  Resv_42112[124];
    volatile uint32_t MCAN0_CLKSEL_PROXY;
    volatile uint32_t MCAN1_CLKSEL_PROXY;
    volatile uint8_t  Resv_42240[120];
    volatile uint32_t OSPI0_CLKSEL_PROXY;
    volatile uint8_t  Resv_42256[12];
    volatile uint32_t ADC0_CLKSEL_PROXY;
    volatile uint8_t  Resv_42336[76];
    volatile uint32_t DDR0_RFCLK_TEST0_CLK_CTRL_PROXY;
    volatile uint32_t DDR0_RFCLK_TEST1_CLK_CTRL_PROXY;
    volatile uint8_t  Resv_42352[8];
    volatile uint32_t DDR0_TESTOUT0_CLK_CTRL_PROXY;
    volatile uint32_t DDR0_TESTOUT1_CLK_CTRL_PROXY;
    volatile uint8_t  Resv_45064[2704];
    volatile uint32_t LOCK2_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK2_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_45312[240];
    volatile uint32_t CLAIMREG_P2_R0;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R1;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R2;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R3;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R4;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R5;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R6;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R7;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R8;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R9;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R10;           /* Claim bits for Partition 2 */
    volatile uint8_t  Resv_49920[4564];
    volatile uint32_t FUSE_CRC_CTRL;
    volatile uint32_t CHAIN1_CRC_FUSE;
    volatile uint32_t CHAIN2_CRC_FUSE;
    volatile uint32_t CHAIN3_CRC_FUSE;
    volatile uint32_t CHAIN4_CRC_FUSE;
    volatile uint32_t CHAIN5_CRC_FUSE;
    volatile uint32_t CHAIN6_CRC_FUSE;
    volatile uint32_t CHAIN7_CRC_FUSE;
    volatile uint32_t FUSE_CRC_STAT;
    volatile uint32_t CHAIN1_CRC_CALC;
    volatile uint32_t CHAIN2_CRC_CALC;
    volatile uint32_t CHAIN3_CRC_CALC;
    volatile uint32_t CHAIN4_CRC_CALC;
    volatile uint32_t CHAIN5_CRC_CALC;
    volatile uint32_t CHAIN6_CRC_CALC;
    volatile uint32_t CHAIN7_CRC_CALC;
    volatile uint8_t  Resv_50176[192];
    volatile uint32_t PBIST_EN;
    volatile uint8_t  Resv_53256[3076];
    volatile uint32_t LOCK3_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK3_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_53504[240];
    volatile uint32_t CLAIMREG_P3_R0_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R1_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R2_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R3_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R4_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R5_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R6_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R7_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R8_READONLY;   /* Claim bits for Partition 3 */
    volatile uint8_t  Resv_58112[4572];
    volatile uint32_t FUSE_CRC_CTRL_PROXY;
    volatile uint32_t CHAIN1_CRC_FUSE_PROXY;
    volatile uint32_t CHAIN2_CRC_FUSE_PROXY;
    volatile uint32_t CHAIN3_CRC_FUSE_PROXY;
    volatile uint32_t CHAIN4_CRC_FUSE_PROXY;
    volatile uint32_t CHAIN5_CRC_FUSE_PROXY;
    volatile uint32_t CHAIN6_CRC_FUSE_PROXY;
    volatile uint32_t CHAIN7_CRC_FUSE_PROXY;
    volatile uint32_t FUSE_CRC_STAT_PROXY;
    volatile uint32_t CHAIN1_CRC_CALC_PROXY;
    volatile uint32_t CHAIN2_CRC_CALC_PROXY;
    volatile uint32_t CHAIN3_CRC_CALC_PROXY;
    volatile uint32_t CHAIN4_CRC_CALC_PROXY;
    volatile uint32_t CHAIN5_CRC_CALC_PROXY;
    volatile uint32_t CHAIN6_CRC_CALC_PROXY;
    volatile uint32_t CHAIN7_CRC_CALC_PROXY;
    volatile uint8_t  Resv_58368[192];
    volatile uint32_t PBIST_EN_PROXY;
    volatile uint8_t  Resv_61448[3076];
    volatile uint32_t LOCK3_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK3_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_61696[240];
    volatile uint32_t CLAIMREG_P3_R0;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R1;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R2;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R3;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R4;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R5;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R6;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R7;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R8;            /* Claim bits for Partition 3 */
    volatile uint8_t  Resv_65536[3804];
    volatile uint32_t DV_REG0;
    volatile uint32_t DV_REG1;
    volatile uint32_t DV_REG2;
    volatile uint32_t DV_REG3;
    volatile uint32_t DV_REG4;
    volatile uint32_t DV_REG5;
    volatile uint32_t DV_REG6;
    volatile uint32_t DV_REG7;
    volatile uint32_t DV_REG8;
    volatile uint32_t DV_REG9;
    volatile uint32_t DV_REG10;
    volatile uint32_t DV_REG11;
    volatile uint32_t DV_REG12;
    volatile uint32_t DV_REG13;
    volatile uint32_t DV_REG14;
    volatile uint32_t DV_REG15;
    volatile uint32_t DV_REG16;
    volatile uint32_t DV_REG17;
    volatile uint32_t DV_REG18;
    volatile uint32_t DV_REG19;
    volatile uint32_t DV_REG20;
    volatile uint32_t DV_REG21;
    volatile uint32_t DV_REG22;
    volatile uint32_t DV_REG23;
    volatile uint32_t DV_REG24;
    volatile uint32_t DV_REG25;
    volatile uint32_t DV_REG26;
    volatile uint32_t DV_REG27;
    volatile uint32_t DV_REG28;
    volatile uint32_t DV_REG29;
    volatile uint32_t DV_REG30;
    volatile uint32_t DV_REG31;
    volatile uint32_t DV_REG32;
    volatile uint32_t DV_REG33;
    volatile uint32_t DV_REG34;
    volatile uint32_t DV_REG35;
    volatile uint32_t DV_REG36;
    volatile uint32_t DV_REG37;
    volatile uint32_t DV_REG38;
    volatile uint32_t DV_REG39;
    volatile uint32_t DV_REG40;
    volatile uint32_t DV_REG41;
    volatile uint32_t DV_REG42;
    volatile uint32_t DV_REG43;
    volatile uint32_t DV_REG44;
    volatile uint32_t DV_REG45;
    volatile uint32_t DV_REG46;
    volatile uint32_t DV_REG47;
    volatile uint32_t DV_REG48;
    volatile uint32_t DV_REG49;
    volatile uint32_t DV_REG50;
    volatile uint32_t DV_REG51;
    volatile uint32_t DV_REG52;
    volatile uint32_t DV_REG53;
    volatile uint32_t DV_REG54;
    volatile uint32_t DV_REG55;
    volatile uint32_t DV_REG56;
    volatile uint32_t DV_REG57;
    volatile uint32_t DV_REG58;
    volatile uint32_t DV_REG59;
    volatile uint32_t DV_REG60;
    volatile uint32_t DV_REG61;
    volatile uint32_t DV_REG62;
    volatile uint32_t DV_REG63;
    volatile uint8_t  Resv_66048[256];
    volatile uint32_t DV_REG0_SET;
    volatile uint32_t DV_REG1_SET;
    volatile uint32_t DV_REG2_SET;
    volatile uint32_t DV_REG3_SET;
    volatile uint32_t DV_REG4_SET;
    volatile uint32_t DV_REG5_SET;
    volatile uint32_t DV_REG6_SET;
    volatile uint32_t DV_REG7_SET;
    volatile uint32_t DV_REG8_SET;
    volatile uint32_t DV_REG9_SET;
    volatile uint32_t DV_REG10_SET;
    volatile uint32_t DV_REG11_SET;
    volatile uint32_t DV_REG12_SET;
    volatile uint32_t DV_REG13_SET;
    volatile uint32_t DV_REG14_SET;
    volatile uint32_t DV_REG15_SET;
    volatile uint32_t DV_REG16_SET;
    volatile uint32_t DV_REG17_SET;
    volatile uint32_t DV_REG18_SET;
    volatile uint32_t DV_REG19_SET;
    volatile uint32_t DV_REG20_SET;
    volatile uint32_t DV_REG21_SET;
    volatile uint32_t DV_REG22_SET;
    volatile uint32_t DV_REG23_SET;
    volatile uint32_t DV_REG24_SET;
    volatile uint32_t DV_REG25_SET;
    volatile uint32_t DV_REG26_SET;
    volatile uint32_t DV_REG27_SET;
    volatile uint32_t DV_REG28_SET;
    volatile uint32_t DV_REG29_SET;
    volatile uint32_t DV_REG30_SET;
    volatile uint32_t DV_REG31_SET;
    volatile uint8_t  Resv_66304[128];
    volatile uint32_t DV_REG0_CLR;
    volatile uint32_t DV_REG1_CLR;
    volatile uint32_t DV_REG2_CLR;
    volatile uint32_t DV_REG3_CLR;
    volatile uint32_t DV_REG4_CLR;
    volatile uint32_t DV_REG5_CLR;
    volatile uint32_t DV_REG6_CLR;
    volatile uint32_t DV_REG7_CLR;
    volatile uint32_t DV_REG8_CLR;
    volatile uint32_t DV_REG9_CLR;
    volatile uint32_t DV_REG10_CLR;
    volatile uint32_t DV_REG11_CLR;
    volatile uint32_t DV_REG12_CLR;
    volatile uint32_t DV_REG13_CLR;
    volatile uint32_t DV_REG14_CLR;
    volatile uint32_t DV_REG15_CLR;
    volatile uint32_t DV_REG16_CLR;
    volatile uint32_t DV_REG17_CLR;
    volatile uint32_t DV_REG18_CLR;
    volatile uint32_t DV_REG19_CLR;
    volatile uint32_t DV_REG20_CLR;
    volatile uint32_t DV_REG21_CLR;
    volatile uint32_t DV_REG22_CLR;
    volatile uint32_t DV_REG23_CLR;
    volatile uint32_t DV_REG24_CLR;
    volatile uint32_t DV_REG25_CLR;
    volatile uint32_t DV_REG26_CLR;
    volatile uint32_t DV_REG27_CLR;
    volatile uint32_t DV_REG28_CLR;
    volatile uint32_t DV_REG29_CLR;
    volatile uint32_t DV_REG30_CLR;
    volatile uint32_t DV_REG31_CLR;
    volatile uint8_t  Resv_66560[128];
    volatile uint32_t LED_PID;
    volatile uint32_t LED_PIDCTRL;
    volatile uint8_t  Resv_66816[248];
    volatile uint32_t MAIN_PLL0_TEST_CLKSEL;
    volatile uint32_t MAIN_PLL1_TEST_CLKSEL;
    volatile uint32_t MAIN_PLL2_TEST_CLKSEL;
    volatile uint8_t  Resv_66848[20];
    volatile uint32_t MAIN_PLL8_TEST_CLKSEL;
    volatile uint8_t  Resv_66864[12];
    volatile uint32_t MAIN_PLL12_TEST_CLKSEL;
    volatile uint8_t  Resv_66872[4];
    volatile uint32_t MAIN_PLL14_TEST_CLKSEL;
    volatile uint8_t  Resv_67072[196];
    volatile uint32_t A53SS_DFT_CTL;
    volatile uint8_t  Resv_69640[2564];
    volatile uint32_t LOCK4_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK4_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_69888[240];
    volatile uint32_t CLAIMREG_P4_R0_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R1_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R2_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R3_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R4_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R5_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R6_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R7_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R8_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R9_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R10_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R11_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R12_READONLY;   /* Claim bits for Partition 4 */
    volatile uint8_t  Resv_73728[3788];
    volatile uint32_t DV_REG0_PROXY;
    volatile uint32_t DV_REG1_PROXY;
    volatile uint32_t DV_REG2_PROXY;
    volatile uint32_t DV_REG3_PROXY;
    volatile uint32_t DV_REG4_PROXY;
    volatile uint32_t DV_REG5_PROXY;
    volatile uint32_t DV_REG6_PROXY;
    volatile uint32_t DV_REG7_PROXY;
    volatile uint32_t DV_REG8_PROXY;
    volatile uint32_t DV_REG9_PROXY;
    volatile uint32_t DV_REG10_PROXY;
    volatile uint32_t DV_REG11_PROXY;
    volatile uint32_t DV_REG12_PROXY;
    volatile uint32_t DV_REG13_PROXY;
    volatile uint32_t DV_REG14_PROXY;
    volatile uint32_t DV_REG15_PROXY;
    volatile uint32_t DV_REG16_PROXY;
    volatile uint32_t DV_REG17_PROXY;
    volatile uint32_t DV_REG18_PROXY;
    volatile uint32_t DV_REG19_PROXY;
    volatile uint32_t DV_REG20_PROXY;
    volatile uint32_t DV_REG21_PROXY;
    volatile uint32_t DV_REG22_PROXY;
    volatile uint32_t DV_REG23_PROXY;
    volatile uint32_t DV_REG24_PROXY;
    volatile uint32_t DV_REG25_PROXY;
    volatile uint32_t DV_REG26_PROXY;
    volatile uint32_t DV_REG27_PROXY;
    volatile uint32_t DV_REG28_PROXY;
    volatile uint32_t DV_REG29_PROXY;
    volatile uint32_t DV_REG30_PROXY;
    volatile uint32_t DV_REG31_PROXY;
    volatile uint32_t DV_REG32_PROXY;
    volatile uint32_t DV_REG33_PROXY;
    volatile uint32_t DV_REG34_PROXY;
    volatile uint32_t DV_REG35_PROXY;
    volatile uint32_t DV_REG36_PROXY;
    volatile uint32_t DV_REG37_PROXY;
    volatile uint32_t DV_REG38_PROXY;
    volatile uint32_t DV_REG39_PROXY;
    volatile uint32_t DV_REG40_PROXY;
    volatile uint32_t DV_REG41_PROXY;
    volatile uint32_t DV_REG42_PROXY;
    volatile uint32_t DV_REG43_PROXY;
    volatile uint32_t DV_REG44_PROXY;
    volatile uint32_t DV_REG45_PROXY;
    volatile uint32_t DV_REG46_PROXY;
    volatile uint32_t DV_REG47_PROXY;
    volatile uint32_t DV_REG48_PROXY;
    volatile uint32_t DV_REG49_PROXY;
    volatile uint32_t DV_REG50_PROXY;
    volatile uint32_t DV_REG51_PROXY;
    volatile uint32_t DV_REG52_PROXY;
    volatile uint32_t DV_REG53_PROXY;
    volatile uint32_t DV_REG54_PROXY;
    volatile uint32_t DV_REG55_PROXY;
    volatile uint32_t DV_REG56_PROXY;
    volatile uint32_t DV_REG57_PROXY;
    volatile uint32_t DV_REG58_PROXY;
    volatile uint32_t DV_REG59_PROXY;
    volatile uint32_t DV_REG60_PROXY;
    volatile uint32_t DV_REG61_PROXY;
    volatile uint32_t DV_REG62_PROXY;
    volatile uint32_t DV_REG63_PROXY;
    volatile uint8_t  Resv_74240[256];
    volatile uint32_t DV_REG0_SET_PROXY;
    volatile uint32_t DV_REG1_SET_PROXY;
    volatile uint32_t DV_REG2_SET_PROXY;
    volatile uint32_t DV_REG3_SET_PROXY;
    volatile uint32_t DV_REG4_SET_PROXY;
    volatile uint32_t DV_REG5_SET_PROXY;
    volatile uint32_t DV_REG6_SET_PROXY;
    volatile uint32_t DV_REG7_SET_PROXY;
    volatile uint32_t DV_REG8_SET_PROXY;
    volatile uint32_t DV_REG9_SET_PROXY;
    volatile uint32_t DV_REG10_SET_PROXY;
    volatile uint32_t DV_REG11_SET_PROXY;
    volatile uint32_t DV_REG12_SET_PROXY;
    volatile uint32_t DV_REG13_SET_PROXY;
    volatile uint32_t DV_REG14_SET_PROXY;
    volatile uint32_t DV_REG15_SET_PROXY;
    volatile uint32_t DV_REG16_SET_PROXY;
    volatile uint32_t DV_REG17_SET_PROXY;
    volatile uint32_t DV_REG18_SET_PROXY;
    volatile uint32_t DV_REG19_SET_PROXY;
    volatile uint32_t DV_REG20_SET_PROXY;
    volatile uint32_t DV_REG21_SET_PROXY;
    volatile uint32_t DV_REG22_SET_PROXY;
    volatile uint32_t DV_REG23_SET_PROXY;
    volatile uint32_t DV_REG24_SET_PROXY;
    volatile uint32_t DV_REG25_SET_PROXY;
    volatile uint32_t DV_REG26_SET_PROXY;
    volatile uint32_t DV_REG27_SET_PROXY;
    volatile uint32_t DV_REG28_SET_PROXY;
    volatile uint32_t DV_REG29_SET_PROXY;
    volatile uint32_t DV_REG30_SET_PROXY;
    volatile uint32_t DV_REG31_SET_PROXY;
    volatile uint8_t  Resv_74496[128];
    volatile uint32_t DV_REG0_CLR_PROXY;
    volatile uint32_t DV_REG1_CLR_PROXY;
    volatile uint32_t DV_REG2_CLR_PROXY;
    volatile uint32_t DV_REG3_CLR_PROXY;
    volatile uint32_t DV_REG4_CLR_PROXY;
    volatile uint32_t DV_REG5_CLR_PROXY;
    volatile uint32_t DV_REG6_CLR_PROXY;
    volatile uint32_t DV_REG7_CLR_PROXY;
    volatile uint32_t DV_REG8_CLR_PROXY;
    volatile uint32_t DV_REG9_CLR_PROXY;
    volatile uint32_t DV_REG10_CLR_PROXY;
    volatile uint32_t DV_REG11_CLR_PROXY;
    volatile uint32_t DV_REG12_CLR_PROXY;
    volatile uint32_t DV_REG13_CLR_PROXY;
    volatile uint32_t DV_REG14_CLR_PROXY;
    volatile uint32_t DV_REG15_CLR_PROXY;
    volatile uint32_t DV_REG16_CLR_PROXY;
    volatile uint32_t DV_REG17_CLR_PROXY;
    volatile uint32_t DV_REG18_CLR_PROXY;
    volatile uint32_t DV_REG19_CLR_PROXY;
    volatile uint32_t DV_REG20_CLR_PROXY;
    volatile uint32_t DV_REG21_CLR_PROXY;
    volatile uint32_t DV_REG22_CLR_PROXY;
    volatile uint32_t DV_REG23_CLR_PROXY;
    volatile uint32_t DV_REG24_CLR_PROXY;
    volatile uint32_t DV_REG25_CLR_PROXY;
    volatile uint32_t DV_REG26_CLR_PROXY;
    volatile uint32_t DV_REG27_CLR_PROXY;
    volatile uint32_t DV_REG28_CLR_PROXY;
    volatile uint32_t DV_REG29_CLR_PROXY;
    volatile uint32_t DV_REG30_CLR_PROXY;
    volatile uint32_t DV_REG31_CLR_PROXY;
    volatile uint8_t  Resv_74752[128];
    volatile uint32_t LED_PID_PROXY;
    volatile uint32_t LED_PIDCTRL_PROXY;
    volatile uint8_t  Resv_75008[248];
    volatile uint32_t MAIN_PLL0_TEST_CLKSEL_PROXY;
    volatile uint32_t MAIN_PLL1_TEST_CLKSEL_PROXY;
    volatile uint32_t MAIN_PLL2_TEST_CLKSEL_PROXY;
    volatile uint8_t  Resv_75040[20];
    volatile uint32_t MAIN_PLL8_TEST_CLKSEL_PROXY;
    volatile uint8_t  Resv_75056[12];
    volatile uint32_t MAIN_PLL12_TEST_CLKSEL_PROXY;
    volatile uint8_t  Resv_75064[4];
    volatile uint32_t MAIN_PLL14_TEST_CLKSEL_PROXY;
    volatile uint8_t  Resv_75264[196];
    volatile uint32_t A53SS_DFT_CTL_PROXY;
    volatile uint8_t  Resv_77832[2564];
    volatile uint32_t LOCK4_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK4_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_78080[240];
    volatile uint32_t CLAIMREG_P4_R0;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R1;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R2;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R3;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R4;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R5;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R6;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R7;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R8;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R9;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R10;           /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R11;           /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R12;           /* Claim bits for Partition 4 */
    volatile uint8_t  Resv_81920[3788];
    volatile uint32_t CHNG_DDR4_FSP_REQ;
    volatile uint32_t CHNG_DDR4_FSP_ACK;
    volatile uint8_t  Resv_82048[120];
    volatile uint32_t DDR4_FSP_CLKCHNG_REQ;
    volatile uint8_t  Resv_82112[60];
    volatile uint32_t DDR4_FSP_CLKCHNG_ACK;
    volatile uint8_t  Resv_86024[3908];
    volatile uint32_t LOCK5_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK5_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_86272[240];
    volatile uint32_t CLAIMREG_P5_R0_READONLY;   /* Claim bits for Partition 5 */
    volatile uint32_t CLAIMREG_P5_R1_READONLY;   /* Claim bits for Partition 5 */
    volatile uint8_t  Resv_90112[3832];
    volatile uint32_t CHNG_DDR4_FSP_REQ_PROXY;
    volatile uint32_t CHNG_DDR4_FSP_ACK_PROXY;
    volatile uint8_t  Resv_90240[120];
    volatile uint32_t DDR4_FSP_CLKCHNG_REQ_PROXY;
    volatile uint8_t  Resv_90304[60];
    volatile uint32_t DDR4_FSP_CLKCHNG_ACK_PROXY;
    volatile uint8_t  Resv_94216[3908];
    volatile uint32_t LOCK5_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK5_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_94464[240];
    volatile uint32_t CLAIMREG_P5_R0;            /* Claim bits for Partition 5 */
    volatile uint32_t CLAIMREG_P5_R1;            /* Claim bits for Partition 5 */
    volatile uint8_t  Resv_98672[4200];
    volatile uint32_t RST_CTRL;
    volatile uint32_t RST_STAT;
    volatile uint32_t RST_SRC;
    volatile uint32_t RST_MAGIC_WORD;
    volatile uint8_t  Resv_102408[3720];
    volatile uint32_t LOCK6_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK6_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_102656[240];
    volatile uint32_t CLAIMREG_P6_R0_READONLY;   /* Claim bits for Partition 6 */
    volatile uint32_t CLAIMREG_P6_R1_READONLY;   /* Claim bits for Partition 6 */
    volatile uint32_t CLAIMREG_P6_R2_READONLY;   /* Claim bits for Partition 6 */
    volatile uint8_t  Resv_106864[4196];
    volatile uint32_t RST_CTRL_PROXY;
    volatile uint32_t RST_STAT_PROXY;
    volatile uint32_t RST_SRC_PROXY;
    volatile uint32_t RST_MAGIC_WORD_PROXY;
    volatile uint8_t  Resv_110600[3720];
    volatile uint32_t LOCK6_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK6_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_110848[240];
    volatile uint32_t CLAIMREG_P6_R0;            /* Claim bits for Partition 6 */
    volatile uint32_t CLAIMREG_P6_R1;            /* Claim bits for Partition 6 */
    volatile uint32_t CLAIMREG_P6_R2;            /* Claim bits for Partition 6 */
} CSL_main_ctrl_mmr_cfg0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MAIN_CTRL_MMR_CFG0_PID                                        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0                                   (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1                                   (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID                                     (0x00000014U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID                               (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0                                    (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1                                    (0x00000024U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2                                    (0x00000028U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3                                    (0x0000002CU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT                               (0x00000030U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG                               (0x00000034U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID                                (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS                              (0x00000044U)
#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT                                   (0x0000004CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0                            (0x00000060U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2                            (0x00000068U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6                            (0x00000078U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0                                    (0x00000200U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1                                    (0x00000204U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0                             (0x00000210U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1                             (0x00000214U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0                             (0x00000220U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1                             (0x00000224U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0                                     (0x00000230U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1                                     (0x00000234U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2                                     (0x00000238U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3                                     (0x0000023CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT                               (0x00000270U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0                                (0x00000300U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0                                (0x00001008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1                                (0x0000100CU)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS                            (0x00001010U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR                  (0x00001014U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE                                (0x00001018U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR                          (0x0000101CU)
#define CSL_MAIN_CTRL_MMR_CFG0_EOI                                        (0x00001020U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS                              (0x00001024U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS                          (0x00001028U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS                          (0x0000102CU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR                                (0x00001030U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY                    (0x00001100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY                    (0x00001104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY                    (0x00001108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY                    (0x0000110CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY                    (0x00001110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_READONLY                    (0x00001114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_READONLY                    (0x00001118U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY                                  (0x00002000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY                             (0x00002004U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY                             (0x00002008U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY                               (0x00002014U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_PROXY                         (0x00002018U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_PROXY                              (0x00002020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_PROXY                              (0x00002024U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_PROXY                              (0x00002028U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_PROXY                              (0x0000202CU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_PROXY                         (0x00002030U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_PROXY                         (0x00002034U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_PROXY                          (0x00002040U)
#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROXY                        (0x00002044U)
#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_PROXY                             (0x0000204CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY                      (0x00002060U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY                      (0x00002068U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY                      (0x00002078U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_PROXY                              (0x00002200U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_PROXY                              (0x00002204U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_PROXY                       (0x00002210U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_PROXY                       (0x00002214U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_PROXY                       (0x00002220U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_PROXY                       (0x00002224U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_PROXY                               (0x00002230U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_PROXY                               (0x00002234U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_PROXY                               (0x00002238U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_PROXY                               (0x0000223CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY                         (0x00002270U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_PROXY                          (0x00002300U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY                          (0x00003008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY                          (0x0000300CU)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY                      (0x00003010U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY            (0x00003014U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY                          (0x00003018U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY                    (0x0000301CU)
#define CSL_MAIN_CTRL_MMR_CFG0_EOI_PROXY                                  (0x00003020U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY                        (0x00003024U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY                    (0x00003028U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY                    (0x0000302CU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY                          (0x00003030U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0                             (0x00003100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1                             (0x00003104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2                             (0x00003108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3                             (0x0000310CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4                             (0x00003110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5                             (0x00003114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6                             (0x00003118U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL                              (0x00004008U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL                                 (0x00004044U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL                                 (0x00004048U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL                                 (0x00004070U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL                           (0x00004080U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM                                  (0x000040C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL                               (0x000040E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0                               (0x00004100U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1                               (0x00004104U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0                               (0x00004110U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1                               (0x00004114U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN                              (0x00004130U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET                          (0x00004134U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR                          (0x00004138U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL                                 (0x00004140U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL                                 (0x00004144U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL                                 (0x00004148U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL                                 (0x0000414CU)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL                                 (0x00004150U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL                                 (0x00004154U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL                                 (0x00004158U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL                                 (0x0000415CU)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL                                 (0x00004160U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL                                   (0x00004170U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL                                   (0x00004174U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL                                 (0x00004180U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL                                 (0x00004184U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL                                 (0x00004188U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT                                  (0x000041A0U)
#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL                                 (0x000041B4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL                                (0x00004204U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL                                (0x0000420CU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL                                (0x00004214U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL                                (0x0000421CU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL                                (0x00004224U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL                               (0x0000422CU)
#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL                                  (0x000042E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0                                (0x00004400U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1                                (0x00004404U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0                                (0x00004440U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1                                (0x00004444U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0                                (0x00004480U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL                                   (0x00004700U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL                                  (0x00004710U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT                                   (0x00004750U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0                                (0x00005008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1                                (0x0000500CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY                    (0x00005100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY                    (0x00005104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY                    (0x00005108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY                    (0x0000510CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY                    (0x00005110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY                    (0x00005114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_READONLY                    (0x00005118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_READONLY                    (0x0000511CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_READONLY                    (0x00005120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_READONLY                    (0x00005124U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_READONLY                   (0x00005128U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_READONLY                   (0x0000512CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_READONLY                   (0x00005130U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_READONLY                   (0x00005134U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_READONLY                   (0x00005138U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY                        (0x00006008U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY                           (0x00006044U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY                           (0x00006048U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY                           (0x00006070U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_PROXY                     (0x00006080U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY                            (0x000060C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY                         (0x000060E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY                         (0x00006100U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY                         (0x00006104U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY                         (0x00006110U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY                         (0x00006114U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY                        (0x00006130U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY                    (0x00006134U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY                    (0x00006138U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY                           (0x00006140U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_PROXY                           (0x00006144U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_PROXY                           (0x00006148U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY                           (0x0000614CU)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_PROXY                           (0x00006150U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_PROXY                           (0x00006154U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY                           (0x00006158U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_PROXY                           (0x0000615CU)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_PROXY                           (0x00006160U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_PROXY                             (0x00006170U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_PROXY                             (0x00006174U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_PROXY                           (0x00006180U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_PROXY                           (0x00006184U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_PROXY                           (0x00006188U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY                            (0x000061A0U)
#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_PROXY                           (0x000061B4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_PROXY                          (0x00006204U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_PROXY                          (0x0000620CU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_PROXY                          (0x00006214U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_PROXY                          (0x0000621CU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_PROXY                          (0x00006224U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_PROXY                         (0x0000622CU)
#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_PROXY                            (0x000062E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_PROXY                          (0x00006400U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_PROXY                          (0x00006404U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_PROXY                          (0x00006440U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_PROXY                          (0x00006444U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_PROXY                          (0x00006480U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY                             (0x00006700U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY                            (0x00006710U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY                             (0x00006750U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY                          (0x00007008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY                          (0x0000700CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0                             (0x00007100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1                             (0x00007104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2                             (0x00007108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3                             (0x0000710CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4                             (0x00007110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5                             (0x00007114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6                             (0x00007118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7                             (0x0000711CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8                             (0x00007120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9                             (0x00007124U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10                            (0x00007128U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11                            (0x0000712CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12                            (0x00007130U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13                            (0x00007134U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14                            (0x00007138U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL                               (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL                                (0x00008010U)
#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL                                 (0x00008030U)
#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL                               (0x0000803CU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL                              (0x00008040U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL                              (0x00008044U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL                           (0x00008060U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL                           (0x00008064U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL                           (0x00008068U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL                           (0x00008080U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL                          (0x00008090U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL                          (0x00008098U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL                               (0x00008120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL                                (0x00008140U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL                                (0x00008150U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL                               (0x00008160U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL                               (0x00008168U)
#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL                                (0x00008180U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL                                (0x00008190U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL                              (0x000081B0U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL                              (0x000081B4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL                              (0x000081B8U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL                              (0x000081BCU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL                              (0x000081C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL                              (0x000081C4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL                              (0x000081C8U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL                              (0x000081CCU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL                              (0x000081D0U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL                              (0x000081D4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL                             (0x000081D8U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL                             (0x000081DCU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL                                (0x00008200U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL                                (0x00008204U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL                                (0x00008208U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL                                (0x0000820CU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL                                (0x00008210U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL                            (0x00008240U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL                            (0x00008244U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL                            (0x00008248U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL                            (0x0000824CU)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL                            (0x00008250U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL                            (0x00008254U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL                            (0x00008258U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL                              (0x00008280U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL                              (0x00008284U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL                              (0x00008288U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL                              (0x0000828CU)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL                              (0x00008290U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL                              (0x00008294U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL                              (0x00008298U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL                                (0x00008380U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL                                (0x00008384U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL                                (0x000083A0U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL                                (0x000083A4U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL                               (0x000083A8U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL                               (0x000083ACU)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL                             (0x00008400U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL                               (0x00008480U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL                               (0x00008484U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL                               (0x00008500U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL                                (0x00008510U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL                  (0x00008560U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL                  (0x00008564U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL                     (0x00008570U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL                     (0x00008574U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0                                (0x00009008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1                                (0x0000900CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY                    (0x00009100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY                    (0x00009104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY                    (0x00009108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY                    (0x0000910CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_READONLY                    (0x00009110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_READONLY                    (0x00009114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_READONLY                    (0x00009118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_READONLY                    (0x0000911CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_READONLY                    (0x00009120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_READONLY                    (0x00009124U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_READONLY                   (0x00009128U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY                         (0x0000A000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY                          (0x0000A010U)
#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_PROXY                           (0x0000A030U)
#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_PROXY                         (0x0000A03CU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY                        (0x0000A040U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY                        (0x0000A044U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY                     (0x0000A060U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY                     (0x0000A064U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY                     (0x0000A068U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY                     (0x0000A080U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY                    (0x0000A090U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY                    (0x0000A098U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_PROXY                         (0x0000A120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_PROXY                          (0x0000A140U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_PROXY                          (0x0000A150U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_PROXY                         (0x0000A160U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY                         (0x0000A168U)
#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_PROXY                          (0x0000A180U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_PROXY                          (0x0000A190U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_PROXY                        (0x0000A1B0U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_PROXY                        (0x0000A1B4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_PROXY                        (0x0000A1B8U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_PROXY                        (0x0000A1BCU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_PROXY                        (0x0000A1C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_PROXY                        (0x0000A1C4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_PROXY                        (0x0000A1C8U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_PROXY                        (0x0000A1CCU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_PROXY                        (0x0000A1D0U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_PROXY                        (0x0000A1D4U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_PROXY                       (0x0000A1D8U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_PROXY                       (0x0000A1DCU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_PROXY                          (0x0000A200U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_PROXY                          (0x0000A204U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_PROXY                          (0x0000A208U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_PROXY                          (0x0000A20CU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_PROXY                          (0x0000A210U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY                      (0x0000A240U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY                      (0x0000A244U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY                      (0x0000A248U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY                      (0x0000A24CU)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY                      (0x0000A250U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY                      (0x0000A254U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY                      (0x0000A258U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_PROXY                        (0x0000A280U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_PROXY                        (0x0000A284U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_PROXY                        (0x0000A288U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_PROXY                        (0x0000A28CU)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_PROXY                        (0x0000A290U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_PROXY                        (0x0000A294U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_PROXY                        (0x0000A298U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY                          (0x0000A380U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY                          (0x0000A384U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY                          (0x0000A3A0U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY                          (0x0000A3A4U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY                         (0x0000A3A8U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY                         (0x0000A3ACU)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_PROXY                       (0x0000A400U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_PROXY                         (0x0000A480U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_PROXY                         (0x0000A484U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY                         (0x0000A500U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_PROXY                          (0x0000A510U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY            (0x0000A560U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY            (0x0000A564U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY               (0x0000A570U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY               (0x0000A574U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY                          (0x0000B008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY                          (0x0000B00CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0                             (0x0000B100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1                             (0x0000B104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2                             (0x0000B108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3                             (0x0000B10CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4                             (0x0000B110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5                             (0x0000B114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6                             (0x0000B118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7                             (0x0000B11CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8                             (0x0000B120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9                             (0x0000B124U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10                            (0x0000B128U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL                              (0x0000C300U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE                            (0x0000C304U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE                            (0x0000C308U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE                            (0x0000C30CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE                            (0x0000C310U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE                            (0x0000C314U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE                            (0x0000C318U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE                            (0x0000C31CU)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT                              (0x0000C320U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC                            (0x0000C324U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC                            (0x0000C328U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC                            (0x0000C32CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC                            (0x0000C330U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC                            (0x0000C334U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC                            (0x0000C338U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC                            (0x0000C33CU)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN                                   (0x0000C400U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0                                (0x0000D008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1                                (0x0000D00CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY                    (0x0000D100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY                    (0x0000D104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY                    (0x0000D108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY                    (0x0000D10CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY                    (0x0000D110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY                    (0x0000D114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_READONLY                    (0x0000D118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_READONLY                    (0x0000D11CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_READONLY                    (0x0000D120U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY                        (0x0000E300U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_PROXY                      (0x0000E304U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_PROXY                      (0x0000E308U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_PROXY                      (0x0000E30CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_PROXY                      (0x0000E310U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_PROXY                      (0x0000E314U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_PROXY                      (0x0000E318U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_PROXY                      (0x0000E31CU)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY                        (0x0000E320U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_PROXY                      (0x0000E324U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_PROXY                      (0x0000E328U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_PROXY                      (0x0000E32CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_PROXY                      (0x0000E330U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_PROXY                      (0x0000E334U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_PROXY                      (0x0000E338U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_PROXY                      (0x0000E33CU)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY                             (0x0000E400U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY                          (0x0000F008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY                          (0x0000F00CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0                             (0x0000F100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1                             (0x0000F104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2                             (0x0000F108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3                             (0x0000F10CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4                             (0x0000F110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5                             (0x0000F114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6                             (0x0000F118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7                             (0x0000F11CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8                             (0x0000F120U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0                                    (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1                                    (0x00010004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2                                    (0x00010008U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3                                    (0x0001000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4                                    (0x00010010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5                                    (0x00010014U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6                                    (0x00010018U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7                                    (0x0001001CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8                                    (0x00010020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9                                    (0x00010024U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10                                   (0x00010028U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11                                   (0x0001002CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12                                   (0x00010030U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13                                   (0x00010034U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14                                   (0x00010038U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15                                   (0x0001003CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16                                   (0x00010040U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17                                   (0x00010044U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18                                   (0x00010048U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19                                   (0x0001004CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20                                   (0x00010050U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21                                   (0x00010054U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22                                   (0x00010058U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23                                   (0x0001005CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24                                   (0x00010060U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25                                   (0x00010064U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26                                   (0x00010068U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27                                   (0x0001006CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28                                   (0x00010070U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29                                   (0x00010074U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30                                   (0x00010078U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31                                   (0x0001007CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32                                   (0x00010080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33                                   (0x00010084U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34                                   (0x00010088U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35                                   (0x0001008CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36                                   (0x00010090U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37                                   (0x00010094U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38                                   (0x00010098U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39                                   (0x0001009CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40                                   (0x000100A0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41                                   (0x000100A4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42                                   (0x000100A8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43                                   (0x000100ACU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44                                   (0x000100B0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45                                   (0x000100B4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46                                   (0x000100B8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47                                   (0x000100BCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48                                   (0x000100C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49                                   (0x000100C4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50                                   (0x000100C8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51                                   (0x000100CCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52                                   (0x000100D0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53                                   (0x000100D4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54                                   (0x000100D8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55                                   (0x000100DCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56                                   (0x000100E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57                                   (0x000100E4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58                                   (0x000100E8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59                                   (0x000100ECU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60                                   (0x000100F0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61                                   (0x000100F4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62                                   (0x000100F8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63                                   (0x000100FCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET                                (0x00010200U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET                                (0x00010204U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET                                (0x00010208U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET                                (0x0001020CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET                                (0x00010210U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET                                (0x00010214U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET                                (0x00010218U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET                                (0x0001021CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET                                (0x00010220U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET                                (0x00010224U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET                               (0x00010228U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET                               (0x0001022CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET                               (0x00010230U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET                               (0x00010234U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET                               (0x00010238U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET                               (0x0001023CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET                               (0x00010240U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET                               (0x00010244U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET                               (0x00010248U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET                               (0x0001024CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET                               (0x00010250U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET                               (0x00010254U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET                               (0x00010258U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET                               (0x0001025CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET                               (0x00010260U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET                               (0x00010264U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET                               (0x00010268U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET                               (0x0001026CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET                               (0x00010270U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET                               (0x00010274U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET                               (0x00010278U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET                               (0x0001027CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR                                (0x00010300U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR                                (0x00010304U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR                                (0x00010308U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR                                (0x0001030CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR                                (0x00010310U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR                                (0x00010314U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR                                (0x00010318U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR                                (0x0001031CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR                                (0x00010320U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR                                (0x00010324U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR                               (0x00010328U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR                               (0x0001032CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR                               (0x00010330U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR                               (0x00010334U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR                               (0x00010338U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR                               (0x0001033CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR                               (0x00010340U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR                               (0x00010344U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR                               (0x00010348U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR                               (0x0001034CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR                               (0x00010350U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR                               (0x00010354U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR                               (0x00010358U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR                               (0x0001035CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR                               (0x00010360U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR                               (0x00010364U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR                               (0x00010368U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR                               (0x0001036CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR                               (0x00010370U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR                               (0x00010374U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR                               (0x00010378U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR                               (0x0001037CU)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID                                    (0x00010400U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL                                (0x00010404U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL                      (0x00010500U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL                      (0x00010504U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL                      (0x00010508U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL                      (0x00010520U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL                     (0x00010530U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL                     (0x00010538U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL                              (0x00010600U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0                                (0x00011008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1                                (0x0001100CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY                    (0x00011100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY                    (0x00011104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_READONLY                    (0x00011108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_READONLY                    (0x0001110CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_READONLY                    (0x00011110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_READONLY                    (0x00011114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_READONLY                    (0x00011118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_READONLY                    (0x0001111CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_READONLY                    (0x00011120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_READONLY                    (0x00011124U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_READONLY                   (0x00011128U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_READONLY                   (0x0001112CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_READONLY                   (0x00011130U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_PROXY                              (0x00012000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_PROXY                              (0x00012004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_PROXY                              (0x00012008U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_PROXY                              (0x0001200CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_PROXY                              (0x00012010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_PROXY                              (0x00012014U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_PROXY                              (0x00012018U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_PROXY                              (0x0001201CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_PROXY                              (0x00012020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_PROXY                              (0x00012024U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_PROXY                             (0x00012028U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_PROXY                             (0x0001202CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_PROXY                             (0x00012030U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_PROXY                             (0x00012034U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_PROXY                             (0x00012038U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_PROXY                             (0x0001203CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_PROXY                             (0x00012040U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_PROXY                             (0x00012044U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_PROXY                             (0x00012048U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_PROXY                             (0x0001204CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_PROXY                             (0x00012050U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_PROXY                             (0x00012054U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_PROXY                             (0x00012058U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_PROXY                             (0x0001205CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_PROXY                             (0x00012060U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_PROXY                             (0x00012064U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_PROXY                             (0x00012068U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_PROXY                             (0x0001206CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_PROXY                             (0x00012070U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_PROXY                             (0x00012074U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_PROXY                             (0x00012078U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_PROXY                             (0x0001207CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_PROXY                             (0x00012080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_PROXY                             (0x00012084U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_PROXY                             (0x00012088U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_PROXY                             (0x0001208CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_PROXY                             (0x00012090U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_PROXY                             (0x00012094U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_PROXY                             (0x00012098U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_PROXY                             (0x0001209CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_PROXY                             (0x000120A0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_PROXY                             (0x000120A4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_PROXY                             (0x000120A8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_PROXY                             (0x000120ACU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_PROXY                             (0x000120B0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_PROXY                             (0x000120B4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_PROXY                             (0x000120B8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_PROXY                             (0x000120BCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_PROXY                             (0x000120C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_PROXY                             (0x000120C4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_PROXY                             (0x000120C8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_PROXY                             (0x000120CCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_PROXY                             (0x000120D0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_PROXY                             (0x000120D4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_PROXY                             (0x000120D8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_PROXY                             (0x000120DCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_PROXY                             (0x000120E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_PROXY                             (0x000120E4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_PROXY                             (0x000120E8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_PROXY                             (0x000120ECU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_PROXY                             (0x000120F0U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_PROXY                             (0x000120F4U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_PROXY                             (0x000120F8U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_PROXY                             (0x000120FCU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_PROXY                          (0x00012200U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_PROXY                          (0x00012204U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_PROXY                          (0x00012208U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_PROXY                          (0x0001220CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_PROXY                          (0x00012210U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_PROXY                          (0x00012214U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_PROXY                          (0x00012218U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_PROXY                          (0x0001221CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_PROXY                          (0x00012220U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_PROXY                          (0x00012224U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_PROXY                         (0x00012228U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_PROXY                         (0x0001222CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_PROXY                         (0x00012230U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_PROXY                         (0x00012234U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_PROXY                         (0x00012238U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_PROXY                         (0x0001223CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_PROXY                         (0x00012240U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_PROXY                         (0x00012244U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_PROXY                         (0x00012248U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_PROXY                         (0x0001224CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_PROXY                         (0x00012250U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_PROXY                         (0x00012254U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_PROXY                         (0x00012258U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_PROXY                         (0x0001225CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_PROXY                         (0x00012260U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_PROXY                         (0x00012264U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_PROXY                         (0x00012268U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_PROXY                         (0x0001226CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_PROXY                         (0x00012270U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_PROXY                         (0x00012274U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_PROXY                         (0x00012278U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_PROXY                         (0x0001227CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_PROXY                          (0x00012300U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_PROXY                          (0x00012304U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_PROXY                          (0x00012308U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_PROXY                          (0x0001230CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_PROXY                          (0x00012310U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_PROXY                          (0x00012314U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_PROXY                          (0x00012318U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_PROXY                          (0x0001231CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_PROXY                          (0x00012320U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_PROXY                          (0x00012324U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_PROXY                         (0x00012328U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_PROXY                         (0x0001232CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_PROXY                         (0x00012330U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_PROXY                         (0x00012334U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_PROXY                         (0x00012338U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_PROXY                         (0x0001233CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_PROXY                         (0x00012340U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_PROXY                         (0x00012344U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_PROXY                         (0x00012348U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_PROXY                         (0x0001234CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_PROXY                         (0x00012350U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_PROXY                         (0x00012354U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_PROXY                         (0x00012358U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_PROXY                         (0x0001235CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_PROXY                         (0x00012360U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_PROXY                         (0x00012364U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_PROXY                         (0x00012368U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_PROXY                         (0x0001236CU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_PROXY                         (0x00012370U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_PROXY                         (0x00012374U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_PROXY                         (0x00012378U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_PROXY                         (0x0001237CU)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PROXY                              (0x00012400U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PROXY                          (0x00012404U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_PROXY                (0x00012500U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_PROXY                (0x00012504U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_PROXY                (0x00012508U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_PROXY                (0x00012520U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_PROXY               (0x00012530U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_PROXY               (0x00012538U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_PROXY                        (0x00012600U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY                          (0x00013008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY                          (0x0001300CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0                             (0x00013100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1                             (0x00013104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2                             (0x00013108U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3                             (0x0001310CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4                             (0x00013110U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5                             (0x00013114U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6                             (0x00013118U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7                             (0x0001311CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8                             (0x00013120U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9                             (0x00013124U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10                            (0x00013128U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11                            (0x0001312CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12                            (0x00013130U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ                          (0x00014000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK                          (0x00014004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ                       (0x00014080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK                       (0x000140C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0                                (0x00015008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1                                (0x0001500CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_READONLY                    (0x00015100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_READONLY                    (0x00015104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY                    (0x00016000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY                    (0x00016004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY                 (0x00016080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_PROXY                 (0x000160C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_PROXY                          (0x00017008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_PROXY                          (0x0001700CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0                             (0x00017100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1                             (0x00017104U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL                                   (0x00018170U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT                                   (0x00018174U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC                                    (0x00018178U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD                             (0x0001817CU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0                                (0x00019008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1                                (0x0001900CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY                    (0x00019100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_READONLY                    (0x00019104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_READONLY                    (0x00019108U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY                             (0x0001A170U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_PROXY                             (0x0001A174U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY                              (0x0001A178U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_PROXY                       (0x0001A17CU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY                          (0x0001B008U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY                          (0x0001B00CU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0                             (0x0001B100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1                             (0x0001B104U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2                             (0x0001B108U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MINOR_MASK                         (0x0000003FU)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MINOR_SHIFT                        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MINOR_MAX                          (0x0000003FU)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_CUSTOM_MASK                        (0x000000C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_CUSTOM_SHIFT                       (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_CUSTOM_MAX                         (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MAJOR_MASK                         (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MAJOR_SHIFT                        (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MAJOR_MAX                          (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MISC_MASK                          (0x0000F800U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MISC_SHIFT                         (0x0000000BU)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MISC_MAX                           (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MSB16_MASK                         (0xFFFF0000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MSB16_SHIFT                        (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PID_MSB16_MAX                          (0x0000FFFFU)

/* MMR_CFG0 */

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_MASK                     (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_SHIFT                    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_MAX                      (0x0000FFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_MASK                      (0xFFFF0000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_SHIFT                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_MAX                       (0x0000FFFFU)

/* MMR_CFG1 */

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_MASK                   (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_MAX                    (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_MASK                     (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_SHIFT                    (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_MAX                      (0x00000001U)

/* JTAGID */

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_LSB_MASK                            (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_LSB_SHIFT                           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_LSB_MAX                             (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_MFG_MASK                            (0x00000FFEU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_MFG_SHIFT                           (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_MFG_MAX                             (0x000007FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PARTNO_MASK                         (0x0FFFF000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PARTNO_SHIFT                        (0x0000000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PARTNO_MAX                          (0x0000FFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_VARIANT_MASK                        (0xF0000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_VARIANT_SHIFT                       (0x0000001CU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_VARIANT_MAX                         (0x0000000FU)

/* JTAG_USER_ID */

#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_USERCODE_MASK                 (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_USERCODE_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_USERCODE_MAX                  (0xFFFFFFFFU)

/* DIE_ID0 */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_DIEID_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_DIEID_SHIFT                        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_DIEID_MAX                          (0xFFFFFFFFU)

/* DIE_ID1 */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_DIEID_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_DIEID_SHIFT                        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_DIEID_MAX                          (0xFFFFFFFFU)

/* DIE_ID2 */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_DIEID_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_DIEID_SHIFT                        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_DIEID_MAX                          (0xFFFFFFFFU)

/* DIE_ID3 */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_DIEID_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_DIEID_SHIFT                        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_DIEID_MAX                          (0xFFFFFFFFU)

/* MAIN_DEVSTAT */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_BOOTMODE_MASK                 (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_BOOTMODE_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_BOOTMODE_MAX                  (0x0000FFFFU)

/* MAIN_BOOTCFG */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_BOOTMODE_MASK                 (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_BOOTMODE_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_BOOTMODE_MAX                  (0x0000FFFFU)

/* A53SS_REVID */

#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_REVID_MASK                     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_REVID_SHIFT                    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_REVID_MAX                      (0xFFFFFFFFU)

/* BOOT_PROGRESS */

#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROGRESS_MASK                (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROGRESS_SHIFT               (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROGRESS_MAX                 (0xFFFFFFFFU)

/* PLL_INIT */

#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_FRACF_CALIBRATION_STATUS_MASK     (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_FRACF_CALIBRATION_STATUS_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_FRACF_CALIBRATION_STATUS_MAX      (0x0000000FU)

/* DEVICE_FEATURE0 */

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_MPU_CLUSTER0_CORE0_MASK    (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_MPU_CLUSTER0_CORE0_SHIFT   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_MPU_CLUSTER0_CORE0_MAX     (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_MPU_CLUSTER0_CORE1_MASK    (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_MPU_CLUSTER0_CORE1_SHIFT   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_MPU_CLUSTER0_CORE1_MAX     (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS0_CORE0_MASK          (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS0_CORE0_SHIFT         (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS0_CORE0_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS0_CORE1_MASK          (0x00020000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS0_CORE1_SHIFT         (0x00000011U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS0_CORE1_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS1_CORE0_MASK          (0x00040000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS1_CORE0_SHIFT         (0x00000012U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS1_CORE0_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS1_CORE1_MASK          (0x00080000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS1_CORE1_SHIFT         (0x00000013U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_R5FSS1_CORE1_MAX           (0x00000001U)

/* DEVICE_FEATURE2 */

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_MCAN_FD_MODE_MASK          (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_MCAN_FD_MODE_SHIFT         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_MCAN_FD_MODE_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_AES_AUTH_EN_MASK           (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_AES_AUTH_EN_SHIFT          (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_AES_AUTH_EN_MAX            (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_SHA_EN_MASK         (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_SHA_EN_SHIFT        (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_SHA_EN_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_ENCR_EN_MASK        (0x00000200U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_ENCR_EN_SHIFT       (0x00000009U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_ENCR_EN_MAX         (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_PKA_EN_MASK         (0x00000400U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_PKA_EN_SHIFT        (0x0000000AU)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_CRYPTO_PKA_EN_MAX          (0x00000001U)

/* DEVICE_FEATURE6 */

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SA2_UL_MASK                (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SA2_UL_SHIFT               (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SA2_UL_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SPARE0_MASK                (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SPARE0_SHIFT               (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SPARE0_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SPARE1_MASK                (0x00020000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SPARE1_SHIFT               (0x00000011U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_SPARE1_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_0_MASK       (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_0_SHIFT      (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_0_MAX        (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_1_MASK       (0x02000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_1_SHIFT      (0x00000019U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_1_MAX        (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_2_MASK       (0x04000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_2_SHIFT      (0x0000001AU)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_MAIN_RESERVED_2_MAX        (0x00000001U)

/* MAC_ID0 */

#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_MAX                       (0xFFFFFFFFU)

/* MAC_ID1 */

#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_MASK                      (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_MAX                       (0x0000FFFFU)

/* PCI_DEVICE_ID0 */

#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_ID0_MASK                    (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_ID0_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_ID0_MAX                     (0xFFFFFFFFU)

/* PCI_DEVICE_ID1 */

#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_ID1_MASK                    (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_ID1_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_ID1_MAX                     (0xFFFFFFFFU)

/* USB_DEVICE_ID0 */

#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_ID0_MASK                    (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_ID0_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_ID0_MAX                     (0xFFFFFFFFU)

/* USB_DEVICE_ID1 */

#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_ID1_MASK                    (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_ID1_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_ID1_MAX                     (0xFFFFFFFFU)

/* GP_SW0 */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_VAL_SHIFT                           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_VAL_MAX                             (0xFFFFFFFFU)

/* GP_SW1 */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_VAL_SHIFT                           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_VAL_MAX                             (0xFFFFFFFFU)

/* GP_SW2 */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_VAL_SHIFT                           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_VAL_MAX                             (0xFFFFFFFFU)

/* GP_SW3 */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_VAL_MASK                            (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_VAL_SHIFT                           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_VAL_MAX                             (0x0000000FU)

/* CBA_ERR_STAT */

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MAIN_CBA_ERR_MASK             (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MAIN_CBA_ERR_SHIFT            (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MAIN_CBA_ERR_MAX              (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MAIN_INFRA_CBA_ERR_MASK       (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MAIN_INFRA_CBA_ERR_SHIFT      (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MAIN_INFRA_CBA_ERR_MAX        (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MCU_CBA_ERR_MASK              (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MCU_CBA_ERR_SHIFT             (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_MCU_CBA_ERR_MAX               (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_DBG_CBA_ERR_MASK              (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_DBG_CBA_ERR_SHIFT             (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_DBG_CBA_ERR_MAX               (0x00000001U)

/* SPARE_FUSE0 */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_FUSE_VAL_MASK                  (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_FUSE_VAL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_FUSE_VAL_MAX                   (0xFFFFFFFFU)

/* LOCK0_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK0_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_MAX                (0xFFFFFFFFU)

/* INTR_RAW_STATUS */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_MASK              (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_MAX               (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_MASK              (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_SHIFT             (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_MAX               (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_MASK              (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_SHIFT             (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_MAX               (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_MASK             (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_SHIFT            (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_MAX              (0x00000001U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX (0x00000001U)

/* INTR_ENABLE */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_MASK               (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_MASK               (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_SHIFT              (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_MASK               (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_SHIFT              (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_MASK              (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_SHIFT             (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_MAX               (0x00000001U)

/* INTR_ENABLE_CLEAR */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK     (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK     (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT    (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK     (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT    (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK    (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT   (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX     (0x00000001U)

/* EOI */

#define CSL_MAIN_CTRL_MMR_CFG0_EOI_EOI_VECTOR_MASK                        (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_EOI_EOI_VECTOR_SHIFT                       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EOI_EOI_VECTOR_MAX                         (0x000000FFU)

/* FAULT_ADDRESS */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_MASK              (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_MAX               (0xFFFFFFFFU)

/* FAULT_TYPE_STATUS */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_MASK          (0x0000003FU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_MAX           (0x0000003FU)

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_MASK            (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_SHIFT           (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_MAX             (0x00000001U)

/* FAULT_ATTR_STATUS */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK        (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX         (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK       (0x000FFF00U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT      (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX        (0x00000FFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_MASK           (0xFFF00000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_SHIFT          (0x00000014U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_MAX            (0x00000FFFU)

/* FAULT_CLEAR */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_MAX                  (0x00000001U)

/* CLAIMREG_P0_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P0_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P0_R2_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P0_R3_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P0_R4_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P0_R5_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_READONLY_CLAIMREG_P0_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_READONLY_CLAIMREG_P0_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_READONLY_CLAIMREG_P0_R5_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P0_R6_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_READONLY_CLAIMREG_P0_R6_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_READONLY_CLAIMREG_P0_R6_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_READONLY_CLAIMREG_P0_R6_READONLY_MAX (0xFFFFFFFFU)

/* PID_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_MASK             (0x0000003FU)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_SHIFT            (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_MAX              (0x0000003FU)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_MASK            (0x000000C0U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_SHIFT           (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_MAX             (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_MASK             (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_SHIFT            (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_MAX              (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_MASK              (0x0000F800U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_SHIFT             (0x0000000BU)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_MAX               (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_MASK             (0xFFFF0000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_SHIFT            (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_MAX              (0x0000FFFFU)

/* MMR_CFG0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_MASK (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_MAX (0x0000FFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_MASK (0xFFFF0000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_MAX  (0x0000FFFFU)

/* MMR_CFG1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_MASK (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_MAX (0x00000001U)

/* JTAGID_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_LSB_PROXY_MASK         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_LSB_PROXY_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_LSB_PROXY_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_MFG_PROXY_MASK         (0x00000FFEU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_MFG_PROXY_SHIFT        (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_MFG_PROXY_MAX          (0x000007FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_PARTNO_PROXY_MASK      (0x0FFFF000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_PARTNO_PROXY_SHIFT     (0x0000000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_PARTNO_PROXY_MAX       (0x0000FFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_VARIANT_PROXY_MASK     (0xF0000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_VARIANT_PROXY_SHIFT    (0x0000001CU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAGID_PROXY_JTAGID_VARIANT_PROXY_MAX      (0x0000000FU)

/* JTAG_USER_ID_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_PROXY_JTAG_USER_ID_USERCODE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_PROXY_JTAG_USER_ID_USERCODE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_JTAG_USER_ID_PROXY_JTAG_USER_ID_USERCODE_PROXY_MAX (0xFFFFFFFFU)

/* DIE_ID0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_PROXY_DIE_ID0_DIEID_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_PROXY_DIE_ID0_DIEID_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID0_PROXY_DIE_ID0_DIEID_PROXY_MAX      (0xFFFFFFFFU)

/* DIE_ID1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_PROXY_DIE_ID1_DIEID_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_PROXY_DIE_ID1_DIEID_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID1_PROXY_DIE_ID1_DIEID_PROXY_MAX      (0xFFFFFFFFU)

/* DIE_ID2_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_PROXY_DIE_ID2_DIEID_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_PROXY_DIE_ID2_DIEID_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID2_PROXY_DIE_ID2_DIEID_PROXY_MAX      (0xFFFFFFFFU)

/* DIE_ID3_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_PROXY_DIE_ID3_DIEID_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_PROXY_DIE_ID3_DIEID_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DIE_ID3_PROXY_DIE_ID3_DIEID_PROXY_MAX      (0xFFFFFFFFU)

/* MAIN_DEVSTAT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_PROXY_MAIN_DEVSTAT_BOOTMODE_PROXY_MASK (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_PROXY_MAIN_DEVSTAT_BOOTMODE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_DEVSTAT_PROXY_MAIN_DEVSTAT_BOOTMODE_PROXY_MAX (0x0000FFFFU)

/* MAIN_BOOTCFG_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_PROXY_MAIN_BOOTCFG_BOOTMODE_PROXY_MASK (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_PROXY_MAIN_BOOTCFG_BOOTMODE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_BOOTCFG_PROXY_MAIN_BOOTCFG_BOOTMODE_PROXY_MAX (0x0000FFFFU)

/* A53SS_REVID_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_PROXY_A53SS_REVID_REVID_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_PROXY_A53SS_REVID_REVID_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_REVID_PROXY_A53SS_REVID_REVID_PROXY_MAX (0xFFFFFFFFU)

/* BOOT_PROGRESS_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROXY_BOOT_PROGRESS_PROGRESS_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROXY_BOOT_PROGRESS_PROGRESS_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_BOOT_PROGRESS_PROXY_BOOT_PROGRESS_PROGRESS_PROXY_MAX (0xFFFFFFFFU)

/* PLL_INIT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_PROXY_PLL_INIT_FRACF_CALIBRATION_STATUS_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_PROXY_PLL_INIT_FRACF_CALIBRATION_STATUS_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PLL_INIT_PROXY_PLL_INIT_FRACF_CALIBRATION_STATUS_PROXY_MAX (0x0000000FU)

/* DEVICE_FEATURE0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_MPU_CLUSTER0_CORE0_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_MPU_CLUSTER0_CORE0_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_MPU_CLUSTER0_CORE0_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_MPU_CLUSTER0_CORE1_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_MPU_CLUSTER0_CORE1_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_MPU_CLUSTER0_CORE1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS0_CORE0_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS0_CORE0_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS0_CORE0_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS0_CORE1_PROXY_MASK (0x00020000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS0_CORE1_PROXY_SHIFT (0x00000011U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS0_CORE1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS1_CORE0_PROXY_MASK (0x00040000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS1_CORE0_PROXY_SHIFT (0x00000012U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS1_CORE0_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS1_CORE1_PROXY_MASK (0x00080000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS1_CORE1_PROXY_SHIFT (0x00000013U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE0_PROXY_DEVICE_FEATURE0_R5FSS1_CORE1_PROXY_MAX (0x00000001U)

/* DEVICE_FEATURE2_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_MCAN_FD_MODE_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_MCAN_FD_MODE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_MCAN_FD_MODE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_AES_AUTH_EN_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_AES_AUTH_EN_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_AES_AUTH_EN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_SHA_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_SHA_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_SHA_EN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_ENCR_EN_PROXY_MASK (0x00000200U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_ENCR_EN_PROXY_SHIFT (0x00000009U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_ENCR_EN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_PKA_EN_PROXY_MASK (0x00000400U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_PKA_EN_PROXY_SHIFT (0x0000000AU)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE2_PROXY_DEVICE_FEATURE2_CRYPTO_PKA_EN_PROXY_MAX (0x00000001U)

/* DEVICE_FEATURE6_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SA2_UL_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SA2_UL_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SA2_UL_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SPARE0_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SPARE0_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SPARE0_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SPARE1_PROXY_MASK (0x00020000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SPARE1_PROXY_SHIFT (0x00000011U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_SPARE1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_0_PROXY_MASK (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_0_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_0_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_1_PROXY_MASK (0x02000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_1_PROXY_SHIFT (0x00000019U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_2_PROXY_MASK (0x04000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_2_PROXY_SHIFT (0x0000001AU)
#define CSL_MAIN_CTRL_MMR_CFG0_DEVICE_FEATURE6_PROXY_DEVICE_FEATURE6_MAIN_RESERVED_2_PROXY_MAX (0x00000001U)

/* MAC_ID0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_MASK  (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_MAX   (0xFFFFFFFFU)

/* MAC_ID1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_MASK  (0x0000FFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_MAX   (0x0000FFFFU)

/* PCI_DEVICE_ID0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_PROXY_PCI_DEVICE_ID0_ID0_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_PROXY_PCI_DEVICE_ID0_ID0_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID0_PROXY_PCI_DEVICE_ID0_ID0_PROXY_MAX (0xFFFFFFFFU)

/* PCI_DEVICE_ID1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_PROXY_PCI_DEVICE_ID1_ID1_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_PROXY_PCI_DEVICE_ID1_ID1_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCI_DEVICE_ID1_PROXY_PCI_DEVICE_ID1_ID1_PROXY_MAX (0xFFFFFFFFU)

/* USB_DEVICE_ID0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_PROXY_USB_DEVICE_ID0_ID0_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_PROXY_USB_DEVICE_ID0_ID0_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID0_PROXY_USB_DEVICE_ID0_ID0_PROXY_MAX (0xFFFFFFFFU)

/* USB_DEVICE_ID1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_PROXY_USB_DEVICE_ID1_ID1_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_PROXY_USB_DEVICE_ID1_ID1_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB_DEVICE_ID1_PROXY_USB_DEVICE_ID1_ID1_PROXY_MAX (0xFFFFFFFFU)

/* GP_SW0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_PROXY_GP_SW0_VAL_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_PROXY_GP_SW0_VAL_PROXY_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW0_PROXY_GP_SW0_VAL_PROXY_MAX          (0xFFFFFFFFU)

/* GP_SW1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_PROXY_GP_SW1_VAL_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_PROXY_GP_SW1_VAL_PROXY_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW1_PROXY_GP_SW1_VAL_PROXY_MAX          (0xFFFFFFFFU)

/* GP_SW2_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_PROXY_GP_SW2_VAL_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_PROXY_GP_SW2_VAL_PROXY_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW2_PROXY_GP_SW2_VAL_PROXY_MAX          (0xFFFFFFFFU)

/* GP_SW3_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_PROXY_GP_SW3_VAL_PROXY_MASK         (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_PROXY_GP_SW3_VAL_PROXY_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GP_SW3_PROXY_GP_SW3_VAL_PROXY_MAX          (0x0000000FU)

/* CBA_ERR_STAT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MAIN_CBA_ERR_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MAIN_CBA_ERR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MAIN_CBA_ERR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MAIN_INFRA_CBA_ERR_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MAIN_INFRA_CBA_ERR_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MAIN_INFRA_CBA_ERR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MCU_CBA_ERR_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MCU_CBA_ERR_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_MCU_CBA_ERR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_DBG_CBA_ERR_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_DBG_CBA_ERR_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_CBA_ERR_STAT_PROXY_CBA_ERR_STAT_DBG_CBA_ERR_PROXY_MAX (0x00000001U)

/* SPARE_FUSE0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_PROXY_SPARE_FUSE0_FUSE_VAL_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_PROXY_SPARE_FUSE0_FUSE_VAL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_FUSE0_PROXY_SPARE_FUSE0_FUSE_VAL_PROXY_MAX (0xFFFFFFFFU)

/* LOCK0_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK0_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* INTR_RAW_STATUS_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_MASK  (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_MAX   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_MASK  (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_MAX   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_MASK  (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_MAX   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_MAX  (0x00000001U)

/* INTR_ENABLED_STATUS_CLEAR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_MAX (0x00000001U)

/* INTR_ENABLE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_MASK   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_MAX    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_MASK   (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_SHIFT  (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_MAX    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_MASK   (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_SHIFT  (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_MAX    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_MASK  (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_MAX   (0x00000001U)

/* INTR_ENABLE_CLEAR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_MAX (0x00000001U)

/* EOI_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_MASK            (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_SHIFT           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_MAX             (0x000000FFU)

/* FAULT_ADDRESS_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_MASK  (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_MAX   (0xFFFFFFFFU)

/* FAULT_TYPE_STATUS_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_MASK (0x0000003FU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_MAX (0x0000003FU)

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_MASK (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_SHIFT (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_MAX (0x00000001U)

/* FAULT_ATTR_STATUS_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_MASK (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_MASK (0x000FFF00U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_MAX (0x00000FFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_MASK (0xFFF00000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_SHIFT (0x00000014U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_MAX (0x00000FFFU)

/* FAULT_CLEAR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_MASK     (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_MAX      (0x00000001U)

/* CLAIMREG_P0_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P0_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P0_R2 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P0_R3 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P0_R4 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P0_R5 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_CLAIMREG_P0_R5_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_CLAIMREG_P0_R5_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R5_CLAIMREG_P0_R5_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P0_R6 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_CLAIMREG_P0_R6_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_CLAIMREG_P0_R6_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P0_R6_CLAIMREG_P0_R6_MAX          (0xFFFFFFFFU)

/* USB0_PHY_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_REF_SEL_MASK             (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_REF_SEL_SHIFT            (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_REF_SEL_MAX              (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_CLKOUT_SEL_MASK          (0x00000300U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_CLKOUT_SEL_SHIFT         (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_CLKOUT_SEL_MAX           (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_CLKOUT_ON_MASK           (0x00000800U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_CLKOUT_ON_SHIFT          (0x0000000BU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_CLKOUT_ON_MAX            (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_STANDALONE_MASK          (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_STANDALONE_SHIFT         (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PLL_STANDALONE_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_LOOPBACK_MODE_MASK           (0x00030000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_LOOPBACK_MODE_SHIFT          (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_LOOPBACK_MODE_MAX            (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_CORE_VOLTAGE_MASK            (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_CORE_VOLTAGE_SHIFT           (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_CORE_VOLTAGE_MAX             (0x00000001U)

/* ENET1_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PORT_MODE_SEL_MASK              (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PORT_MODE_SEL_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PORT_MODE_SEL_MAX               (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_RGMII_ID_MODE_MASK              (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_RGMII_ID_MODE_SHIFT             (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_RGMII_ID_MODE_MAX               (0x00000001U)

/* ENET2_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PORT_MODE_SEL_MASK              (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PORT_MODE_SEL_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PORT_MODE_SEL_MAX               (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_RGMII_ID_MODE_MASK              (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_RGMII_ID_MODE_SHIFT             (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_RGMII_ID_MODE_MAX               (0x00000001U)

/* PCIE0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_GENERATION_SEL_MASK             (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_GENERATION_SEL_SHIFT            (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_GENERATION_SEL_MAX              (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_MODE_SEL_MASK                   (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_MODE_SEL_SHIFT                  (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_MODE_SEL_MAX                    (0x00000001U)

/* SERDES0_LN0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_LANE_FUNC_SEL_MASK        (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_LANE_FUNC_SEL_SHIFT       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_LANE_FUNC_SEL_MAX         (0x00000003U)

/* ADC0_TRIM */

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_ENABLE_CAL_MASK                  (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_ENABLE_CAL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_ENABLE_CAL_MAX                   (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_ENABLE_CALB_MASK                 (0x000003E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_ENABLE_CALB_SHIFT                (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_ENABLE_CALB_MAX                  (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM1_MASK                       (0x00003C00U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM1_SHIFT                      (0x0000000AU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM1_MAX                        (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM2_MASK                       (0x0003C000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM2_SHIFT                      (0x0000000EU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM2_MAX                        (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM3_MASK                       (0x001C0000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM3_SHIFT                      (0x00000012U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM3_MAX                        (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM4_MASK                       (0x00E00000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM4_SHIFT                      (0x00000015U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM4_MAX                        (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM5_MASK                       (0x07000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM5_SHIFT                      (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_TRIM5_MAX                        (0x00000007U)

/* SERDES0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_RET_EN_MASK                   (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_RET_EN_SHIFT                  (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_RET_EN_MAX                    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_REF_SEL_MASK                  (0x00001000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_REF_SEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_REF_SEL_MAX                   (0x00000001U)

/* ICSSG0_CTRL0 */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_GPM_BIDI_MASK                 (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_GPM_BIDI_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_GPM_BIDI_MAX                  (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_RGMII0_ID_MODE_MASK           (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_RGMII0_ID_MODE_SHIFT          (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_RGMII0_ID_MODE_MAX            (0x00000001U)

/* ICSSG0_CTRL1 */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_GPM_BIDI_MASK                 (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_GPM_BIDI_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_GPM_BIDI_MAX                  (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_RGMII1_ID_MODE_MASK           (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_RGMII1_ID_MODE_SHIFT          (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_RGMII1_ID_MODE_MAX            (0x00000001U)

/* ICSSG1_CTRL0 */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_GPM_BIDI_MASK                 (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_GPM_BIDI_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_GPM_BIDI_MAX                  (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_RGMII0_ID_MODE_MASK           (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_RGMII0_ID_MODE_SHIFT          (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_RGMII0_ID_MODE_MAX            (0x00000001U)

/* ICSSG1_CTRL1 */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_GPM_BIDI_MASK                 (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_GPM_BIDI_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_GPM_BIDI_MAX                  (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_RGMII1_ID_MODE_MASK           (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_RGMII1_ID_MODE_SHIFT          (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_RGMII1_ID_MODE_MAX            (0x00000001U)

/* EPWM_TB_CLKEN */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM0_TB_CLKEN_MASK          (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM0_TB_CLKEN_SHIFT         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM0_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM1_TB_CLKEN_MASK          (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM1_TB_CLKEN_SHIFT         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM1_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM2_TB_CLKEN_MASK          (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM2_TB_CLKEN_SHIFT         (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM2_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM3_TB_CLKEN_MASK          (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM3_TB_CLKEN_SHIFT         (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM3_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM4_TB_CLKEN_MASK          (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM4_TB_CLKEN_SHIFT         (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM4_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM5_TB_CLKEN_MASK          (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM5_TB_CLKEN_SHIFT         (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM5_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM6_TB_CLKEN_MASK          (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM6_TB_CLKEN_SHIFT         (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM6_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM7_TB_CLKEN_MASK          (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM7_TB_CLKEN_SHIFT         (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM7_TB_CLKEN_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM8_TB_CLKEN_MASK          (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM8_TB_CLKEN_SHIFT         (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_EPWM8_TB_CLKEN_MAX           (0x00000001U)

/* EPWM_TB_CLKEN_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM0_TB_CLKEN_MASK      (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM0_TB_CLKEN_SHIFT     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM0_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM1_TB_CLKEN_MASK      (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM1_TB_CLKEN_SHIFT     (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM1_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM2_TB_CLKEN_MASK      (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM2_TB_CLKEN_SHIFT     (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM2_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM3_TB_CLKEN_MASK      (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM3_TB_CLKEN_SHIFT     (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM3_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM4_TB_CLKEN_MASK      (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM4_TB_CLKEN_SHIFT     (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM4_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM5_TB_CLKEN_MASK      (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM5_TB_CLKEN_SHIFT     (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM5_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM6_TB_CLKEN_MASK      (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM6_TB_CLKEN_SHIFT     (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM6_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM7_TB_CLKEN_MASK      (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM7_TB_CLKEN_SHIFT     (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM7_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM8_TB_CLKEN_MASK      (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM8_TB_CLKEN_SHIFT     (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_EPWM8_TB_CLKEN_MAX       (0x00000001U)

/* EPWM_TB_CLKEN_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM0_TB_CLKEN_MASK      (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM0_TB_CLKEN_SHIFT     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM0_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM1_TB_CLKEN_MASK      (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM1_TB_CLKEN_SHIFT     (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM1_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM2_TB_CLKEN_MASK      (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM2_TB_CLKEN_SHIFT     (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM2_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM3_TB_CLKEN_MASK      (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM3_TB_CLKEN_SHIFT     (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM3_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM4_TB_CLKEN_MASK      (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM4_TB_CLKEN_SHIFT     (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM4_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM5_TB_CLKEN_MASK      (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM5_TB_CLKEN_SHIFT     (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM5_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM6_TB_CLKEN_MASK      (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM6_TB_CLKEN_SHIFT     (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM6_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM7_TB_CLKEN_MASK      (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM7_TB_CLKEN_SHIFT     (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM7_TB_CLKEN_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM8_TB_CLKEN_MASK      (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM8_TB_CLKEN_SHIFT     (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_EPWM8_TB_CLKEN_MAX       (0x00000001U)

/* EPWM0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_EALLOW_MAX                      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_SYNCIN_SEL_MASK                 (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_SYNCIN_SEL_SHIFT                (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_SYNCIN_SEL_MAX                  (0x00000007U)

/* EPWM1_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_EALLOW_MAX                      (0x00000001U)

/* EPWM2_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_EALLOW_MAX                      (0x00000001U)

/* EPWM3_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_EALLOW_MAX                      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_MASK                 (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_SHIFT                (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_MAX                  (0x00000007U)

/* EPWM4_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_EALLOW_MAX                      (0x00000001U)

/* EPWM5_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_EALLOW_MAX                      (0x00000001U)

/* EPWM6_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_EALLOW_MAX                      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_SYNCIN_SEL_MASK                 (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_SYNCIN_SEL_SHIFT                (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_SYNCIN_SEL_MAX                  (0x00000007U)

/* EPWM7_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_EALLOW_MAX                      (0x00000001U)

/* EPWM8_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_EALLOW_MASK                     (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_EALLOW_SHIFT                    (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_EALLOW_MAX                      (0x00000001U)

/* SOCA_SEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_SOCA_SEL_MASK                     (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_SOCA_SEL_SHIFT                    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_SOCA_SEL_MAX                      (0x00000003U)

/* SOCB_SEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_SOCB_SEL_MASK                     (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_SOCB_SEL_SHIFT                    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_SOCB_SEL_MAX                      (0x00000003U)

/* EQEP0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_SOCA_SEL_MASK                   (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_SOCA_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_SOCA_SEL_MAX                    (0x0000001FU)

/* EQEP1_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_SOCA_SEL_MASK                   (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_SOCA_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_SOCA_SEL_MAX                    (0x0000001FU)

/* EQEP2_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_SOCA_SEL_MASK                   (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_SOCA_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_SOCA_SEL_MAX                    (0x0000001FU)

/* EQEP_STAT */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR0_MASK                  (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR0_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR0_MAX                   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR1_MASK                  (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR1_SHIFT                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR1_MAX                   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR2_MASK                  (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR2_SHIFT                 (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PHASE_ERR2_MAX                   (0x00000001U)

/* SDIO1_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_DRV_STR_MASK                    (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_DRV_STR_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_DRV_STR_MAX                     (0x0000001FU)

/* TIMER1_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_CASCADE_EN_MASK                (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_CASCADE_EN_SHIFT               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_CASCADE_EN_MAX                 (0x00000001U)

/* TIMER3_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_CASCADE_EN_MASK                (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_CASCADE_EN_SHIFT               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_CASCADE_EN_MAX                 (0x00000001U)

/* TIMER5_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_CASCADE_EN_MASK                (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_CASCADE_EN_SHIFT               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_CASCADE_EN_MAX                 (0x00000001U)

/* TIMER7_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_CASCADE_EN_MASK                (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_CASCADE_EN_SHIFT               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_CASCADE_EN_MAX                 (0x00000001U)

/* TIMER9_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_CASCADE_EN_MASK                (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_CASCADE_EN_SHIFT               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_CASCADE_EN_MAX                 (0x00000001U)

/* TIMER11_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_CASCADE_EN_MASK               (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_CASCADE_EN_SHIFT              (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_CASCADE_EN_MAX                (0x00000001U)

/* I2C0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_HS_MCS_EN_MASK                   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_HS_MCS_EN_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_HS_MCS_EN_MAX                    (0x00000001U)

/* SPARE_CTRL0 */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_SPARE_OUT_MASK                 (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_SPARE_OUT_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_SPARE_OUT_MAX                  (0xFFFFFFFFU)

/* SPARE_CTRL1 */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_SPARE_OUT_MASK                 (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_SPARE_OUT_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_SPARE_OUT_MAX                  (0xFFFFFFFFU)

/* SPARE_STAT0 */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_SPARE_IN_MASK                  (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_SPARE_IN_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_SPARE_IN_MAX                   (0xFFFFFFFFU)

/* SPARE_STAT1 */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_SPARE_IN_MASK                  (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_SPARE_IN_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_SPARE_IN_MAX                   (0xFFFFFFFFU)

/* SPARE_TRIM0 */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_TRIM_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_TRIM_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_TRIM_MAX                       (0xFFFFFFFFU)

/* FSS_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_S0_BOOT_SEG_MASK                  (0x0000003FU)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_S0_BOOT_SEG_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_S0_BOOT_SEG_MAX                   (0x0000003FU)

#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_S0_BOOT_SIZE_MASK                 (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_S0_BOOT_SIZE_SHIFT                (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_S0_BOOT_SIZE_MAX                  (0x00000001U)

/* ADC0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_TRIG_SEL_MASK                    (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_TRIG_SEL_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_TRIG_SEL_MAX                     (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_GPI_MODE_EN_MASK                 (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_GPI_MODE_EN_SHIFT                (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_GPI_MODE_EN_MAX                  (0x00000001U)

/* DCC_STAT */

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC0_INTR_DONE_MASK               (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC0_INTR_DONE_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC0_INTR_DONE_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC1_INTR_DONE_MASK               (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC1_INTR_DONE_SHIFT              (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC1_INTR_DONE_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC2_INTR_DONE_MASK               (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC2_INTR_DONE_SHIFT              (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC2_INTR_DONE_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC3_INTR_DONE_MASK               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC3_INTR_DONE_SHIFT              (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC3_INTR_DONE_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC4_INTR_DONE_MASK               (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC4_INTR_DONE_SHIFT              (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC4_INTR_DONE_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC5_INTR_DONE_MASK               (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC5_INTR_DONE_SHIFT              (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_DCC5_INTR_DONE_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_MCU_DCC0_INTR_DONE_MASK           (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_MCU_DCC0_INTR_DONE_SHIFT          (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_MCU_DCC0_INTR_DONE_MAX            (0x00000001U)

/* LOCK1_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK1_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_MAX                (0xFFFFFFFFU)

/* CLAIMREG_P1_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R2_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R3_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R4_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R5_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R6_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_READONLY_CLAIMREG_P1_R6_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_READONLY_CLAIMREG_P1_R6_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_READONLY_CLAIMREG_P1_R6_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R7_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_READONLY_CLAIMREG_P1_R7_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_READONLY_CLAIMREG_P1_R7_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_READONLY_CLAIMREG_P1_R7_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R8_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_READONLY_CLAIMREG_P1_R8_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_READONLY_CLAIMREG_P1_R8_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_READONLY_CLAIMREG_P1_R8_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R9_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_READONLY_CLAIMREG_P1_R9_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_READONLY_CLAIMREG_P1_R9_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_READONLY_CLAIMREG_P1_R9_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R10_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_READONLY_CLAIMREG_P1_R10_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_READONLY_CLAIMREG_P1_R10_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_READONLY_CLAIMREG_P1_R10_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R11_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_READONLY_CLAIMREG_P1_R11_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_READONLY_CLAIMREG_P1_R11_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_READONLY_CLAIMREG_P1_R11_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R12_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_READONLY_CLAIMREG_P1_R12_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_READONLY_CLAIMREG_P1_R12_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_READONLY_CLAIMREG_P1_R12_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R13_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_READONLY_CLAIMREG_P1_R13_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_READONLY_CLAIMREG_P1_R13_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_READONLY_CLAIMREG_P1_R13_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P1_R14_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_READONLY_CLAIMREG_P1_R14_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_READONLY_CLAIMREG_P1_R14_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_READONLY_CLAIMREG_P1_R14_READONLY_MAX (0xFFFFFFFFU)

/* USB0_PHY_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_REF_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_REF_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_REF_SEL_PROXY_MAX (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_CLKOUT_SEL_PROXY_MASK (0x00000300U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_CLKOUT_SEL_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_CLKOUT_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_CLKOUT_ON_PROXY_MASK (0x00000800U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_CLKOUT_ON_PROXY_SHIFT (0x0000000BU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_CLKOUT_ON_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_STANDALONE_PROXY_MASK (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_STANDALONE_PROXY_SHIFT (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_PLL_STANDALONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_LOOPBACK_MODE_PROXY_MASK (0x00030000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_LOOPBACK_MODE_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_LOOPBACK_MODE_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_CORE_VOLTAGE_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_CORE_VOLTAGE_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_PHY_CTRL_PROXY_USB0_PHY_CTRL_CORE_VOLTAGE_PROXY_MAX (0x00000001U)

/* ENET1_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY_ENET1_CTRL_PORT_MODE_SEL_PROXY_MASK (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY_ENET1_CTRL_PORT_MODE_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY_ENET1_CTRL_PORT_MODE_SEL_PROXY_MAX (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY_ENET1_CTRL_RGMII_ID_MODE_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY_ENET1_CTRL_RGMII_ID_MODE_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET1_CTRL_PROXY_ENET1_CTRL_RGMII_ID_MODE_PROXY_MAX (0x00000001U)

/* ENET2_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY_ENET2_CTRL_PORT_MODE_SEL_PROXY_MASK (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY_ENET2_CTRL_PORT_MODE_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY_ENET2_CTRL_PORT_MODE_SEL_PROXY_MAX (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY_ENET2_CTRL_RGMII_ID_MODE_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY_ENET2_CTRL_RGMII_ID_MODE_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_ENET2_CTRL_PROXY_ENET2_CTRL_RGMII_ID_MODE_PROXY_MAX (0x00000001U)

/* PCIE0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY_PCIE0_CTRL_GENERATION_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY_PCIE0_CTRL_GENERATION_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY_PCIE0_CTRL_GENERATION_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY_PCIE0_CTRL_MODE_SEL_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY_PCIE0_CTRL_MODE_SEL_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_PROXY_PCIE0_CTRL_MODE_SEL_PROXY_MAX (0x00000001U)

/* SERDES0_LN0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_PROXY_SERDES0_LN0_CTRL_LANE_FUNC_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_PROXY_SERDES0_LN0_CTRL_LANE_FUNC_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL_PROXY_SERDES0_LN0_CTRL_LANE_FUNC_SEL_PROXY_MAX (0x00000003U)

/* ADC0_TRIM_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_ENABLE_CAL_PROXY_MASK (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_ENABLE_CAL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_ENABLE_CAL_PROXY_MAX (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_ENABLE_CALB_PROXY_MASK (0x000003E0U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_ENABLE_CALB_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_ENABLE_CALB_PROXY_MAX (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM1_PROXY_MASK (0x00003C00U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM1_PROXY_SHIFT (0x0000000AU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM1_PROXY_MAX  (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM2_PROXY_MASK (0x0003C000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM2_PROXY_SHIFT (0x0000000EU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM2_PROXY_MAX  (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM3_PROXY_MASK (0x001C0000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM3_PROXY_SHIFT (0x00000012U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM3_PROXY_MAX  (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM4_PROXY_MASK (0x00E00000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM4_PROXY_SHIFT (0x00000015U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM4_PROXY_MAX  (0x00000007U)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM5_PROXY_MASK (0x07000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM5_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_TRIM_PROXY_ADC0_TRIM_TRIM5_PROXY_MAX  (0x00000007U)

/* SERDES0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY_SERDES0_CTRL_RET_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY_SERDES0_CTRL_RET_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY_SERDES0_CTRL_RET_EN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY_SERDES0_CTRL_REF_SEL_PROXY_MASK (0x00001000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY_SERDES0_CTRL_REF_SEL_PROXY_SHIFT (0x0000000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL_PROXY_SERDES0_CTRL_REF_SEL_PROXY_MAX (0x00000001U)

/* ICSSG0_CTRL0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY_ICSSG0_CTRL0_GPM_BIDI_PROXY_MASK (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY_ICSSG0_CTRL0_GPM_BIDI_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY_ICSSG0_CTRL0_GPM_BIDI_PROXY_MAX (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY_ICSSG0_CTRL0_RGMII0_ID_MODE_PROXY_MASK (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY_ICSSG0_CTRL0_RGMII0_ID_MODE_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL0_PROXY_ICSSG0_CTRL0_RGMII0_ID_MODE_PROXY_MAX (0x00000001U)

/* ICSSG0_CTRL1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY_ICSSG0_CTRL1_GPM_BIDI_PROXY_MASK (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY_ICSSG0_CTRL1_GPM_BIDI_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY_ICSSG0_CTRL1_GPM_BIDI_PROXY_MAX (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY_ICSSG0_CTRL1_RGMII1_ID_MODE_PROXY_MASK (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY_ICSSG0_CTRL1_RGMII1_ID_MODE_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CTRL1_PROXY_ICSSG0_CTRL1_RGMII1_ID_MODE_PROXY_MAX (0x00000001U)

/* ICSSG1_CTRL0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY_ICSSG1_CTRL0_GPM_BIDI_PROXY_MASK (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY_ICSSG1_CTRL0_GPM_BIDI_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY_ICSSG1_CTRL0_GPM_BIDI_PROXY_MAX (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY_ICSSG1_CTRL0_RGMII0_ID_MODE_PROXY_MASK (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY_ICSSG1_CTRL0_RGMII0_ID_MODE_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL0_PROXY_ICSSG1_CTRL0_RGMII0_ID_MODE_PROXY_MAX (0x00000001U)

/* ICSSG1_CTRL1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY_ICSSG1_CTRL1_GPM_BIDI_PROXY_MASK (0x000FFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY_ICSSG1_CTRL1_GPM_BIDI_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY_ICSSG1_CTRL1_GPM_BIDI_PROXY_MAX (0x000FFFFFU)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY_ICSSG1_CTRL1_RGMII1_ID_MODE_PROXY_MASK (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY_ICSSG1_CTRL1_RGMII1_ID_MODE_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CTRL1_PROXY_ICSSG1_CTRL1_RGMII1_ID_MODE_PROXY_MAX (0x00000001U)

/* EPWM_TB_CLKEN_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM0_TB_CLKEN_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM0_TB_CLKEN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM0_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM1_TB_CLKEN_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM1_TB_CLKEN_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM1_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM2_TB_CLKEN_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM2_TB_CLKEN_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM2_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM3_TB_CLKEN_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM3_TB_CLKEN_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM3_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM4_TB_CLKEN_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM4_TB_CLKEN_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM4_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM5_TB_CLKEN_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM5_TB_CLKEN_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM5_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM6_TB_CLKEN_PROXY_MASK (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM6_TB_CLKEN_PROXY_SHIFT (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM6_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM7_TB_CLKEN_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM7_TB_CLKEN_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM7_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM8_TB_CLKEN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM8_TB_CLKEN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_PROXY_EPWM_TB_CLKEN_EPWM8_TB_CLKEN_PROXY_MAX (0x00000001U)

/* EPWM_TB_CLKEN_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM0_TB_CLKEN_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM0_TB_CLKEN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM0_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM1_TB_CLKEN_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM1_TB_CLKEN_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM1_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM2_TB_CLKEN_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM2_TB_CLKEN_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM2_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM3_TB_CLKEN_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM3_TB_CLKEN_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM3_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM4_TB_CLKEN_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM4_TB_CLKEN_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM4_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM5_TB_CLKEN_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM5_TB_CLKEN_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM5_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM6_TB_CLKEN_PROXY_MASK (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM6_TB_CLKEN_PROXY_SHIFT (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM6_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM7_TB_CLKEN_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM7_TB_CLKEN_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM7_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM8_TB_CLKEN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM8_TB_CLKEN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_SET_PROXY_EPWM_TB_CLKEN_SET_EPWM8_TB_CLKEN_PROXY_MAX (0x00000001U)

/* EPWM_TB_CLKEN_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM0_TB_CLKEN_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM0_TB_CLKEN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM0_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM1_TB_CLKEN_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM1_TB_CLKEN_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM1_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM2_TB_CLKEN_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM2_TB_CLKEN_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM2_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM3_TB_CLKEN_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM3_TB_CLKEN_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM3_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM4_TB_CLKEN_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM4_TB_CLKEN_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM4_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM5_TB_CLKEN_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM5_TB_CLKEN_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM5_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM6_TB_CLKEN_PROXY_MASK (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM6_TB_CLKEN_PROXY_SHIFT (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM6_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM7_TB_CLKEN_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM7_TB_CLKEN_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM7_TB_CLKEN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM8_TB_CLKEN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM8_TB_CLKEN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM_TB_CLKEN_CLR_PROXY_EPWM_TB_CLKEN_CLR_EPWM8_TB_CLKEN_PROXY_MAX (0x00000001U)

/* EPWM0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY_EPWM0_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY_EPWM0_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY_EPWM0_CTRL_EALLOW_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY_EPWM0_CTRL_SYNCIN_SEL_PROXY_MASK (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY_EPWM0_CTRL_SYNCIN_SEL_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_PROXY_EPWM0_CTRL_SYNCIN_SEL_PROXY_MAX (0x00000007U)

/* EPWM1_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_PROXY_EPWM1_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_PROXY_EPWM1_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM1_CTRL_PROXY_EPWM1_CTRL_EALLOW_PROXY_MAX (0x00000001U)

/* EPWM2_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_PROXY_EPWM2_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_PROXY_EPWM2_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM2_CTRL_PROXY_EPWM2_CTRL_EALLOW_PROXY_MAX (0x00000001U)

/* EPWM3_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY_EPWM3_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY_EPWM3_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY_EPWM3_CTRL_EALLOW_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY_EPWM3_CTRL_SYNCIN_SEL_PROXY_MASK (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY_EPWM3_CTRL_SYNCIN_SEL_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_PROXY_EPWM3_CTRL_SYNCIN_SEL_PROXY_MAX (0x00000007U)

/* EPWM4_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_PROXY_EPWM4_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_PROXY_EPWM4_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM4_CTRL_PROXY_EPWM4_CTRL_EALLOW_PROXY_MAX (0x00000001U)

/* EPWM5_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_PROXY_EPWM5_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_PROXY_EPWM5_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM5_CTRL_PROXY_EPWM5_CTRL_EALLOW_PROXY_MAX (0x00000001U)

/* EPWM6_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY_EPWM6_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY_EPWM6_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY_EPWM6_CTRL_EALLOW_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY_EPWM6_CTRL_SYNCIN_SEL_PROXY_MASK (0x00000700U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY_EPWM6_CTRL_SYNCIN_SEL_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_PROXY_EPWM6_CTRL_SYNCIN_SEL_PROXY_MAX (0x00000007U)

/* EPWM7_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_PROXY_EPWM7_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_PROXY_EPWM7_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM7_CTRL_PROXY_EPWM7_CTRL_EALLOW_PROXY_MAX (0x00000001U)

/* EPWM8_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_PROXY_EPWM8_CTRL_EALLOW_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_PROXY_EPWM8_CTRL_EALLOW_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EPWM8_CTRL_PROXY_EPWM8_CTRL_EALLOW_PROXY_MAX (0x00000001U)

/* SOCA_SEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_PROXY_SOCA_SEL_SOCA_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_PROXY_SOCA_SEL_SOCA_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCA_SEL_PROXY_SOCA_SEL_SOCA_SEL_PROXY_MAX (0x00000003U)

/* SOCB_SEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_PROXY_SOCB_SEL_SOCB_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_PROXY_SOCB_SEL_SOCB_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SOCB_SEL_PROXY_SOCB_SEL_SOCB_SEL_PROXY_MAX (0x00000003U)

/* EQEP0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_PROXY_EQEP0_CTRL_SOCA_SEL_PROXY_MASK (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_PROXY_EQEP0_CTRL_SOCA_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP0_CTRL_PROXY_EQEP0_CTRL_SOCA_SEL_PROXY_MAX (0x0000001FU)

/* EQEP1_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_PROXY_EQEP1_CTRL_SOCA_SEL_PROXY_MASK (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_PROXY_EQEP1_CTRL_SOCA_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP1_CTRL_PROXY_EQEP1_CTRL_SOCA_SEL_PROXY_MAX (0x0000001FU)

/* EQEP2_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_PROXY_EQEP2_CTRL_SOCA_SEL_PROXY_MASK (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_PROXY_EQEP2_CTRL_SOCA_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP2_CTRL_PROXY_EQEP2_CTRL_SOCA_SEL_PROXY_MAX (0x0000001FU)

/* EQEP_STAT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR0_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR0_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR0_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR1_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR1_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR2_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR2_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_EQEP_STAT_PROXY_EQEP_STAT_PHASE_ERR2_PROXY_MAX (0x00000001U)

/* SDIO1_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_PROXY_SDIO1_CTRL_DRV_STR_PROXY_MASK (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_PROXY_SDIO1_CTRL_DRV_STR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SDIO1_CTRL_PROXY_SDIO1_CTRL_DRV_STR_PROXY_MAX (0x0000001FU)

/* TIMER1_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_PROXY_TIMER1_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_PROXY_TIMER1_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CTRL_PROXY_TIMER1_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

/* TIMER3_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_PROXY_TIMER3_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_PROXY_TIMER3_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CTRL_PROXY_TIMER3_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

/* TIMER5_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_PROXY_TIMER5_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_PROXY_TIMER5_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CTRL_PROXY_TIMER5_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

/* TIMER7_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_PROXY_TIMER7_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_PROXY_TIMER7_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CTRL_PROXY_TIMER7_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

/* TIMER9_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_PROXY_TIMER9_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_PROXY_TIMER9_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CTRL_PROXY_TIMER9_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

/* TIMER11_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_PROXY_TIMER11_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_PROXY_TIMER11_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CTRL_PROXY_TIMER11_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

/* I2C0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_PROXY_I2C0_CTRL_HS_MCS_EN_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_PROXY_I2C0_CTRL_HS_MCS_EN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_I2C0_CTRL_PROXY_I2C0_CTRL_HS_MCS_EN_PROXY_MAX (0x00000001U)

/* SPARE_CTRL0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_PROXY_SPARE_CTRL0_SPARE_OUT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_PROXY_SPARE_CTRL0_SPARE_OUT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL0_PROXY_SPARE_CTRL0_SPARE_OUT_PROXY_MAX (0xFFFFFFFFU)

/* SPARE_CTRL1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_PROXY_SPARE_CTRL1_SPARE_OUT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_PROXY_SPARE_CTRL1_SPARE_OUT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_CTRL1_PROXY_SPARE_CTRL1_SPARE_OUT_PROXY_MAX (0xFFFFFFFFU)

/* SPARE_STAT0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_PROXY_SPARE_STAT0_SPARE_IN_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_PROXY_SPARE_STAT0_SPARE_IN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT0_PROXY_SPARE_STAT0_SPARE_IN_PROXY_MAX (0xFFFFFFFFU)

/* SPARE_STAT1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_PROXY_SPARE_STAT1_SPARE_IN_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_PROXY_SPARE_STAT1_SPARE_IN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_STAT1_PROXY_SPARE_STAT1_SPARE_IN_PROXY_MAX (0xFFFFFFFFU)

/* SPARE_TRIM0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_PROXY_SPARE_TRIM0_TRIM_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_PROXY_SPARE_TRIM0_TRIM_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPARE_TRIM0_PROXY_SPARE_TRIM0_TRIM_PROXY_MAX (0xFFFFFFFFU)

/* FSS_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY_FSS_CTRL_S0_BOOT_SEG_PROXY_MASK (0x0000003FU)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY_FSS_CTRL_S0_BOOT_SEG_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY_FSS_CTRL_S0_BOOT_SEG_PROXY_MAX (0x0000003FU)

#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY_FSS_CTRL_S0_BOOT_SIZE_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY_FSS_CTRL_S0_BOOT_SIZE_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FSS_CTRL_PROXY_FSS_CTRL_S0_BOOT_SIZE_PROXY_MAX (0x00000001U)

/* ADC0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY_ADC0_CTRL_TRIG_SEL_PROXY_MASK (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY_ADC0_CTRL_TRIG_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY_ADC0_CTRL_TRIG_SEL_PROXY_MAX (0x0000001FU)

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY_ADC0_CTRL_GPI_MODE_EN_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY_ADC0_CTRL_GPI_MODE_EN_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CTRL_PROXY_ADC0_CTRL_GPI_MODE_EN_PROXY_MAX (0x00000001U)

/* DCC_STAT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC0_INTR_DONE_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC0_INTR_DONE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC0_INTR_DONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC1_INTR_DONE_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC1_INTR_DONE_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC1_INTR_DONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC2_INTR_DONE_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC2_INTR_DONE_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC2_INTR_DONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC3_INTR_DONE_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC3_INTR_DONE_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC3_INTR_DONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC4_INTR_DONE_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC4_INTR_DONE_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC4_INTR_DONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC5_INTR_DONE_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC5_INTR_DONE_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_DCC5_INTR_DONE_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_MCU_DCC0_INTR_DONE_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_MCU_DCC0_INTR_DONE_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_DCC_STAT_PROXY_DCC_STAT_MCU_DCC0_INTR_DONE_PROXY_MAX (0x00000001U)

/* LOCK1_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK1_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* CLAIMREG_P1_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R2 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R3 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R4 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R5 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R6 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_CLAIMREG_P1_R6_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_CLAIMREG_P1_R6_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R6_CLAIMREG_P1_R6_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R7 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_CLAIMREG_P1_R7_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_CLAIMREG_P1_R7_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R7_CLAIMREG_P1_R7_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R8 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_CLAIMREG_P1_R8_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_CLAIMREG_P1_R8_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R8_CLAIMREG_P1_R8_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R9 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_CLAIMREG_P1_R9_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_CLAIMREG_P1_R9_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R9_CLAIMREG_P1_R9_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P1_R10 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_CLAIMREG_P1_R10_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_CLAIMREG_P1_R10_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R10_CLAIMREG_P1_R10_MAX        (0xFFFFFFFFU)

/* CLAIMREG_P1_R11 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_CLAIMREG_P1_R11_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_CLAIMREG_P1_R11_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R11_CLAIMREG_P1_R11_MAX        (0xFFFFFFFFU)

/* CLAIMREG_P1_R12 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_CLAIMREG_P1_R12_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_CLAIMREG_P1_R12_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R12_CLAIMREG_P1_R12_MAX        (0xFFFFFFFFU)

/* CLAIMREG_P1_R13 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_CLAIMREG_P1_R13_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_CLAIMREG_P1_R13_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R13_CLAIMREG_P1_R13_MAX        (0xFFFFFFFFU)

/* CLAIMREG_P1_R14 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_CLAIMREG_P1_R14_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_CLAIMREG_P1_R14_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P1_R14_CLAIMREG_P1_R14_MAX        (0xFFFFFFFFU)

/* OBSCLK0_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_SEL_MASK                  (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_SEL_MAX                   (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_DIV_MASK                  (0x0000FF00U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_DIV_SHIFT                 (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_DIV_MAX                   (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_DIV_LD_MASK               (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_DIV_LD_SHIFT              (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_CLK_DIV_LD_MAX                (0x00000001U)

/* CLKOUT_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_SEL_MAX                    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_EN_MASK                    (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_EN_SHIFT                   (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_CLK_EN_MAX                     (0x00000001U)

/* GTC_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_CLK_SEL_MASK                    (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_CLK_SEL_MAX                     (0x00000007U)

/* EFUSE_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_CLK_SEL_MASK                  (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_CLK_SEL_MAX                   (0x00000001U)

/* ICSSG0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_CORE_CLKSEL_MASK             (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_CORE_CLKSEL_SHIFT            (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_CORE_CLKSEL_MAX              (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_IEP_CLKSEL_MASK              (0x00070000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_IEP_CLKSEL_SHIFT             (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_IEP_CLKSEL_MAX               (0x00000007U)

/* ICSSG1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_CORE_CLKSEL_MASK             (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_CORE_CLKSEL_SHIFT            (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_CORE_CLKSEL_MAX              (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_IEP_CLKSEL_MASK              (0x00070000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_IEP_CLKSEL_SHIFT             (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_IEP_CLKSEL_MAX               (0x00000007U)

/* MAIN_PLL0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_BYP_WARM_RST_MASK         (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_BYP_WARM_RST_SHIFT        (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_BYP_WARM_RST_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_BYPASS_SW_OVRD_MASK       (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_BYPASS_SW_OVRD_SHIFT      (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_BYPASS_SW_OVRD_MAX        (0x00000001U)

/* MAIN_PLL1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_BYP_WARM_RST_MASK         (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_BYP_WARM_RST_SHIFT        (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_BYP_WARM_RST_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_BYPASS_SW_OVRD_MASK       (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_BYPASS_SW_OVRD_SHIFT      (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_BYPASS_SW_OVRD_MAX        (0x00000001U)

/* MAIN_PLL2_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_BYP_WARM_RST_MASK         (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_BYP_WARM_RST_SHIFT        (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_BYP_WARM_RST_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_BYPASS_SW_OVRD_MASK       (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_BYPASS_SW_OVRD_SHIFT      (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_BYPASS_SW_OVRD_MAX        (0x00000001U)

/* MAIN_PLL8_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_BYP_WARM_RST_MASK         (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_BYP_WARM_RST_SHIFT        (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_BYP_WARM_RST_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_BYPASS_SW_OVRD_MASK       (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_BYPASS_SW_OVRD_SHIFT      (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_BYPASS_SW_OVRD_MAX        (0x00000001U)

/* MAIN_PLL12_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_BYP_WARM_RST_MASK        (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_BYP_WARM_RST_SHIFT       (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_BYP_WARM_RST_MAX         (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_BYPASS_SW_OVRD_MASK      (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_BYPASS_SW_OVRD_SHIFT     (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_BYPASS_SW_OVRD_MAX       (0x00000001U)

/* MAIN_PLL14_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_BYP_WARM_RST_MASK        (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_BYP_WARM_RST_SHIFT       (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_BYP_WARM_RST_MAX         (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_BYPASS_SW_OVRD_MASK      (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_BYPASS_SW_OVRD_SHIFT     (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_BYPASS_SW_OVRD_MAX       (0x00000001U)

/* PCIE0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_CPTS_CLKSEL_MASK              (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_CPTS_CLKSEL_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_CPTS_CLKSEL_MAX               (0x00000007U)

/* CPSW_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_CPTS_CLKSEL_MASK               (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_CPTS_CLKSEL_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_CPTS_CLKSEL_MAX                (0x0000000FU)

/* CPTS_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_CPTS_CLKSEL_MASK               (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_CPTS_CLKSEL_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_CPTS_CLKSEL_MAX                (0x00000007U)

/* EMMC0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_EMMCSD0_REFCLK_SEL_MASK       (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_EMMCSD0_REFCLK_SEL_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_EMMCSD0_REFCLK_SEL_MAX        (0x00000001U)

/* EMMC1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_EMMCSD1_REFCLK_SEL_MASK       (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_EMMCSD1_REFCLK_SEL_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_EMMCSD1_REFCLK_SEL_MAX        (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_EMMCSD1_IO_CLKLB_SEL_MASK     (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_EMMCSD1_IO_CLKLB_SEL_SHIFT    (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_EMMCSD1_IO_CLKLB_SEL_MAX      (0x00000001U)

/* GPMC_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_CLK_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_CLK_SEL_MAX                    (0x00000001U)

/* USB0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_REFCLK_SEL_MASK                (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_REFCLK_SEL_SHIFT               (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_REFCLK_SEL_MAX                 (0x00000001U)

/* TIMER0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER2_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER3_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER4_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER5_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER6_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER7_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER8_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER9_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_CLK_SEL_MASK                 (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_CLK_SEL_MAX                  (0x0000000FU)

/* TIMER10_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_CLK_SEL_MASK                (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_CLK_SEL_SHIFT               (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_CLK_SEL_MAX                 (0x0000000FU)

/* TIMER11_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_CLK_SEL_MASK                (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_CLK_SEL_SHIFT               (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_CLK_SEL_MAX                 (0x0000000FU)

/* SPI0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_MSTR_LB_CLKSEL_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_MSTR_LB_CLKSEL_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_MSTR_LB_CLKSEL_MAX             (0x00000001U)

/* SPI1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_MSTR_LB_CLKSEL_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_MSTR_LB_CLKSEL_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_MSTR_LB_CLKSEL_MAX             (0x00000001U)

/* SPI2_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_MSTR_LB_CLKSEL_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_MSTR_LB_CLKSEL_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_MSTR_LB_CLKSEL_MAX             (0x00000001U)

/* SPI3_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_MSTR_LB_CLKSEL_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_MSTR_LB_CLKSEL_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_MSTR_LB_CLKSEL_MAX             (0x00000001U)

/* SPI4_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_MSTR_LB_CLKSEL_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_MSTR_LB_CLKSEL_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_MSTR_LB_CLKSEL_MAX             (0x00000001U)

/* USART0_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART1_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART2_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART3_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART4_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART5_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART6_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_CLK_DIV_MASK               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_CLK_DIV_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_CLK_DIV_MAX                (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_CLK_DIV_LD_MASK            (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_CLK_DIV_LD_SHIFT           (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_CLK_DIV_LD_MAX             (0x00000001U)

/* USART0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* USART1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* USART2_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* USART3_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* USART4_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* USART5_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* USART6_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

/* WWD0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_CLK_SEL_MASK                   (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_CLK_SEL_MAX                    (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_WRTLOCK_MASK                   (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_WRTLOCK_SHIFT                  (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_WRTLOCK_MAX                    (0x00000001U)

/* WWD1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_CLK_SEL_MASK                   (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_CLK_SEL_MAX                    (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_WRTLOCK_MASK                   (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_WRTLOCK_SHIFT                  (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_WRTLOCK_MAX                    (0x00000001U)

/* WWD8_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_CLK_SEL_MASK                   (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_CLK_SEL_MAX                    (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_WRTLOCK_MASK                   (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_WRTLOCK_SHIFT                  (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_WRTLOCK_MAX                    (0x00000001U)

/* WWD9_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_CLK_SEL_MASK                   (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_CLK_SEL_MAX                    (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_WRTLOCK_MASK                   (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_WRTLOCK_SHIFT                  (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_WRTLOCK_MAX                    (0x00000001U)

/* WWD10_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_CLK_SEL_MASK                  (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_CLK_SEL_MAX                   (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_WRTLOCK_MASK                  (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_WRTLOCK_SHIFT                 (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_WRTLOCK_MAX                   (0x00000001U)

/* WWD11_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_CLK_SEL_MASK                  (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_CLK_SEL_MAX                   (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_WRTLOCK_MASK                  (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_WRTLOCK_SHIFT                 (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_WRTLOCK_MAX                   (0x00000001U)

/* SERDES0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_CORE_REFCLK_SEL_MASK        (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_CORE_REFCLK_SEL_SHIFT       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_CORE_REFCLK_SEL_MAX         (0x00000003U)

/* MCAN0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_CLK_SEL_MASK                  (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_CLK_SEL_MAX                   (0x00000003U)

/* MCAN1_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_CLK_SEL_MASK                  (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_CLK_SEL_MAX                   (0x00000003U)

/* OSPI0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_CLK_SEL_MASK                  (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_CLK_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_LOOPCLK_SEL_MASK              (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_LOOPCLK_SEL_SHIFT             (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_LOOPCLK_SEL_MAX               (0x00000001U)

/* ADC0_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_CLK_SEL_MASK                   (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_CLK_SEL_MAX                    (0x00000003U)

/* DDR0_RFCLK_TEST0_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_DIV_RATIO_MASK   (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_DIV_RATIO_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_DIV_RATIO_MAX    (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_EN_MASK          (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_EN_SHIFT         (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_EN_MAX           (0x00000001U)

/* DDR0_RFCLK_TEST1_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_DIV_RATIO_MASK   (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_DIV_RATIO_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_DIV_RATIO_MAX    (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_EN_MASK          (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_EN_SHIFT         (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_EN_MAX           (0x00000001U)

/* DDR0_TESTOUT0_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_DIV_RATIO_MASK      (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_DIV_RATIO_SHIFT     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_DIV_RATIO_MAX       (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_EN_MASK             (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_EN_SHIFT            (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_EN_MAX              (0x00000001U)

/* DDR0_TESTOUT1_CLK_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_DIV_RATIO_MASK      (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_DIV_RATIO_SHIFT     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_DIV_RATIO_MAX       (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_EN_MASK             (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_EN_SHIFT            (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_EN_MAX              (0x00000001U)

/* LOCK2_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK2_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_MAX                (0xFFFFFFFFU)

/* CLAIMREG_P2_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R2_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R3_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R4_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_READONLY_CLAIMREG_P2_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_READONLY_CLAIMREG_P2_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_READONLY_CLAIMREG_P2_R4_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R5_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_READONLY_CLAIMREG_P2_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_READONLY_CLAIMREG_P2_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_READONLY_CLAIMREG_P2_R5_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R6_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_READONLY_CLAIMREG_P2_R6_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_READONLY_CLAIMREG_P2_R6_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_READONLY_CLAIMREG_P2_R6_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R7_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_READONLY_CLAIMREG_P2_R7_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_READONLY_CLAIMREG_P2_R7_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_READONLY_CLAIMREG_P2_R7_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R8_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_READONLY_CLAIMREG_P2_R8_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_READONLY_CLAIMREG_P2_R8_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_READONLY_CLAIMREG_P2_R8_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R9_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_READONLY_CLAIMREG_P2_R9_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_READONLY_CLAIMREG_P2_R9_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_READONLY_CLAIMREG_P2_R9_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P2_R10_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_READONLY_CLAIMREG_P2_R10_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_READONLY_CLAIMREG_P2_R10_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_READONLY_CLAIMREG_P2_R10_READONLY_MAX (0xFFFFFFFFU)

/* OBSCLK0_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_SEL_PROXY_MAX (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_DIV_PROXY_MASK (0x0000FF00U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_DIV_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_DIV_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_OBSCLK0_CTRL_PROXY_OBSCLK0_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* CLKOUT_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY_CLKOUT_CTRL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY_CLKOUT_CTRL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY_CLKOUT_CTRL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY_CLKOUT_CTRL_CLK_EN_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY_CLKOUT_CTRL_CLK_EN_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLKOUT_CTRL_PROXY_CLKOUT_CTRL_CLK_EN_PROXY_MAX (0x00000001U)

/* GTC_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_PROXY_GTC_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_PROXY_GTC_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GTC_CLKSEL_PROXY_GTC_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

/* EFUSE_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_PROXY_EFUSE_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_PROXY_EFUSE_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EFUSE_CLKSEL_PROXY_EFUSE_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* ICSSG0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY_ICSSG0_CLKSEL_CORE_CLKSEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY_ICSSG0_CLKSEL_CORE_CLKSEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY_ICSSG0_CLKSEL_CORE_CLKSEL_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY_ICSSG0_CLKSEL_IEP_CLKSEL_PROXY_MASK (0x00070000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY_ICSSG0_CLKSEL_IEP_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG0_CLKSEL_PROXY_ICSSG0_CLKSEL_IEP_CLKSEL_PROXY_MAX (0x00000007U)

/* ICSSG1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY_ICSSG1_CLKSEL_CORE_CLKSEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY_ICSSG1_CLKSEL_CORE_CLKSEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY_ICSSG1_CLKSEL_CORE_CLKSEL_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY_ICSSG1_CLKSEL_IEP_CLKSEL_PROXY_MASK (0x00070000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY_ICSSG1_CLKSEL_IEP_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_ICSSG1_CLKSEL_PROXY_ICSSG1_CLKSEL_IEP_CLKSEL_PROXY_MAX (0x00000007U)

/* MAIN_PLL0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY_MAIN_PLL0_CLKSEL_BYP_WARM_RST_PROXY_MASK (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY_MAIN_PLL0_CLKSEL_BYP_WARM_RST_PROXY_SHIFT (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY_MAIN_PLL0_CLKSEL_BYP_WARM_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY_MAIN_PLL0_CLKSEL_BYPASS_SW_OVRD_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY_MAIN_PLL0_CLKSEL_BYPASS_SW_OVRD_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_CLKSEL_PROXY_MAIN_PLL0_CLKSEL_BYPASS_SW_OVRD_PROXY_MAX (0x00000001U)

/* MAIN_PLL1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY_MAIN_PLL1_CLKSEL_BYP_WARM_RST_PROXY_MASK (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY_MAIN_PLL1_CLKSEL_BYP_WARM_RST_PROXY_SHIFT (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY_MAIN_PLL1_CLKSEL_BYP_WARM_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY_MAIN_PLL1_CLKSEL_BYPASS_SW_OVRD_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY_MAIN_PLL1_CLKSEL_BYPASS_SW_OVRD_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_CLKSEL_PROXY_MAIN_PLL1_CLKSEL_BYPASS_SW_OVRD_PROXY_MAX (0x00000001U)

/* MAIN_PLL2_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY_MAIN_PLL2_CLKSEL_BYP_WARM_RST_PROXY_MASK (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY_MAIN_PLL2_CLKSEL_BYP_WARM_RST_PROXY_SHIFT (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY_MAIN_PLL2_CLKSEL_BYP_WARM_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY_MAIN_PLL2_CLKSEL_BYPASS_SW_OVRD_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY_MAIN_PLL2_CLKSEL_BYPASS_SW_OVRD_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_CLKSEL_PROXY_MAIN_PLL2_CLKSEL_BYPASS_SW_OVRD_PROXY_MAX (0x00000001U)

/* MAIN_PLL8_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY_MAIN_PLL8_CLKSEL_BYP_WARM_RST_PROXY_MASK (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY_MAIN_PLL8_CLKSEL_BYP_WARM_RST_PROXY_SHIFT (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY_MAIN_PLL8_CLKSEL_BYP_WARM_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY_MAIN_PLL8_CLKSEL_BYPASS_SW_OVRD_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY_MAIN_PLL8_CLKSEL_BYPASS_SW_OVRD_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_CLKSEL_PROXY_MAIN_PLL8_CLKSEL_BYPASS_SW_OVRD_PROXY_MAX (0x00000001U)

/* MAIN_PLL12_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY_MAIN_PLL12_CLKSEL_BYP_WARM_RST_PROXY_MASK (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY_MAIN_PLL12_CLKSEL_BYP_WARM_RST_PROXY_SHIFT (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY_MAIN_PLL12_CLKSEL_BYP_WARM_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY_MAIN_PLL12_CLKSEL_BYPASS_SW_OVRD_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY_MAIN_PLL12_CLKSEL_BYPASS_SW_OVRD_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_CLKSEL_PROXY_MAIN_PLL12_CLKSEL_BYPASS_SW_OVRD_PROXY_MAX (0x00000001U)

/* MAIN_PLL14_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY_MAIN_PLL14_CLKSEL_BYP_WARM_RST_PROXY_MASK (0x00800000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY_MAIN_PLL14_CLKSEL_BYP_WARM_RST_PROXY_SHIFT (0x00000017U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY_MAIN_PLL14_CLKSEL_BYP_WARM_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY_MAIN_PLL14_CLKSEL_BYPASS_SW_OVRD_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY_MAIN_PLL14_CLKSEL_BYPASS_SW_OVRD_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_CLKSEL_PROXY_MAIN_PLL14_CLKSEL_BYPASS_SW_OVRD_PROXY_MAX (0x00000001U)

/* PCIE0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_PROXY_PCIE0_CLKSEL_CPTS_CLKSEL_PROXY_MASK (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_PROXY_PCIE0_CLKSEL_CPTS_CLKSEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CLKSEL_PROXY_PCIE0_CLKSEL_CPTS_CLKSEL_PROXY_MAX (0x00000007U)

/* CPSW_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_PROXY_CPSW_CLKSEL_CPTS_CLKSEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_PROXY_CPSW_CLKSEL_CPTS_CLKSEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPSW_CLKSEL_PROXY_CPSW_CLKSEL_CPTS_CLKSEL_PROXY_MAX (0x0000000FU)

/* CPTS_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_PROXY_CPTS_CLKSEL_CPTS_CLKSEL_PROXY_MASK (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_PROXY_CPTS_CLKSEL_CPTS_CLKSEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CPTS_CLKSEL_PROXY_CPTS_CLKSEL_CPTS_CLKSEL_PROXY_MAX (0x00000007U)

/* EMMC0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_PROXY_EMMC0_CLKSEL_EMMCSD0_REFCLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_PROXY_EMMC0_CLKSEL_EMMCSD0_REFCLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC0_CLKSEL_PROXY_EMMC0_CLKSEL_EMMCSD0_REFCLK_SEL_PROXY_MAX (0x00000001U)

/* EMMC1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY_EMMC1_CLKSEL_EMMCSD1_REFCLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY_EMMC1_CLKSEL_EMMCSD1_REFCLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY_EMMC1_CLKSEL_EMMCSD1_REFCLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY_EMMC1_CLKSEL_EMMCSD1_IO_CLKLB_SEL_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY_EMMC1_CLKSEL_EMMCSD1_IO_CLKLB_SEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_EMMC1_CLKSEL_PROXY_EMMC1_CLKSEL_EMMCSD1_IO_CLKLB_SEL_PROXY_MAX (0x00000001U)

/* GPMC_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_PROXY_GPMC_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_PROXY_GPMC_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_GPMC_CLKSEL_PROXY_GPMC_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USB0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_PROXY_USB0_CLKSEL_REFCLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_PROXY_USB0_CLKSEL_REFCLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USB0_CLKSEL_PROXY_USB0_CLKSEL_REFCLK_SEL_PROXY_MAX (0x00000001U)

/* TIMER0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_PROXY_TIMER0_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_PROXY_TIMER0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER0_CLKSEL_PROXY_TIMER0_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_PROXY_TIMER1_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_PROXY_TIMER1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER1_CLKSEL_PROXY_TIMER1_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER2_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_PROXY_TIMER2_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_PROXY_TIMER2_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER2_CLKSEL_PROXY_TIMER2_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER3_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_PROXY_TIMER3_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_PROXY_TIMER3_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER3_CLKSEL_PROXY_TIMER3_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER4_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_PROXY_TIMER4_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_PROXY_TIMER4_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER4_CLKSEL_PROXY_TIMER4_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER5_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_PROXY_TIMER5_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_PROXY_TIMER5_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER5_CLKSEL_PROXY_TIMER5_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER6_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_PROXY_TIMER6_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_PROXY_TIMER6_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER6_CLKSEL_PROXY_TIMER6_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER7_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_PROXY_TIMER7_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_PROXY_TIMER7_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER7_CLKSEL_PROXY_TIMER7_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER8_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_PROXY_TIMER8_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_PROXY_TIMER8_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER8_CLKSEL_PROXY_TIMER8_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER9_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_PROXY_TIMER9_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_PROXY_TIMER9_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER9_CLKSEL_PROXY_TIMER9_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER10_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_PROXY_TIMER10_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_PROXY_TIMER10_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER10_CLKSEL_PROXY_TIMER10_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* TIMER11_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_PROXY_TIMER11_CLKSEL_CLK_SEL_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_PROXY_TIMER11_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_TIMER11_CLKSEL_PROXY_TIMER11_CLKSEL_CLK_SEL_PROXY_MAX (0x0000000FU)

/* SPI0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_PROXY_SPI0_CLKSEL_MSTR_LB_CLKSEL_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_PROXY_SPI0_CLKSEL_MSTR_LB_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI0_CLKSEL_PROXY_SPI0_CLKSEL_MSTR_LB_CLKSEL_PROXY_MAX (0x00000001U)

/* SPI1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_PROXY_SPI1_CLKSEL_MSTR_LB_CLKSEL_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_PROXY_SPI1_CLKSEL_MSTR_LB_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI1_CLKSEL_PROXY_SPI1_CLKSEL_MSTR_LB_CLKSEL_PROXY_MAX (0x00000001U)

/* SPI2_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_PROXY_SPI2_CLKSEL_MSTR_LB_CLKSEL_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_PROXY_SPI2_CLKSEL_MSTR_LB_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI2_CLKSEL_PROXY_SPI2_CLKSEL_MSTR_LB_CLKSEL_PROXY_MAX (0x00000001U)

/* SPI3_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_PROXY_SPI3_CLKSEL_MSTR_LB_CLKSEL_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_PROXY_SPI3_CLKSEL_MSTR_LB_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI3_CLKSEL_PROXY_SPI3_CLKSEL_MSTR_LB_CLKSEL_PROXY_MAX (0x00000001U)

/* SPI4_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_PROXY_SPI4_CLKSEL_MSTR_LB_CLKSEL_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_PROXY_SPI4_CLKSEL_MSTR_LB_CLKSEL_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_SPI4_CLKSEL_PROXY_SPI4_CLKSEL_MSTR_LB_CLKSEL_PROXY_MAX (0x00000001U)

/* USART0_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY_USART0_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY_USART0_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY_USART0_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY_USART0_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY_USART0_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLK_CTRL_PROXY_USART0_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART1_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY_USART1_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY_USART1_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY_USART1_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY_USART1_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY_USART1_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLK_CTRL_PROXY_USART1_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART2_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY_USART2_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY_USART2_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY_USART2_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY_USART2_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY_USART2_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLK_CTRL_PROXY_USART2_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART3_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY_USART3_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY_USART3_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY_USART3_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY_USART3_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY_USART3_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLK_CTRL_PROXY_USART3_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART4_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY_USART4_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY_USART4_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY_USART4_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY_USART4_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY_USART4_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLK_CTRL_PROXY_USART4_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART5_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY_USART5_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY_USART5_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY_USART5_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY_USART5_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY_USART5_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLK_CTRL_PROXY_USART5_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART6_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY_USART6_CLK_CTRL_CLK_DIV_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY_USART6_CLK_CTRL_CLK_DIV_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY_USART6_CLK_CTRL_CLK_DIV_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY_USART6_CLK_CTRL_CLK_DIV_LD_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY_USART6_CLK_CTRL_CLK_DIV_LD_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLK_CTRL_PROXY_USART6_CLK_CTRL_CLK_DIV_LD_PROXY_MAX (0x00000001U)

/* USART0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_PROXY_USART0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_PROXY_USART0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART0_CLKSEL_PROXY_USART0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USART1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_PROXY_USART1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_PROXY_USART1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART1_CLKSEL_PROXY_USART1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USART2_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_PROXY_USART2_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_PROXY_USART2_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART2_CLKSEL_PROXY_USART2_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USART3_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_PROXY_USART3_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_PROXY_USART3_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART3_CLKSEL_PROXY_USART3_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USART4_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_PROXY_USART4_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_PROXY_USART4_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART4_CLKSEL_PROXY_USART4_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USART5_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_PROXY_USART5_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_PROXY_USART5_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART5_CLKSEL_PROXY_USART5_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* USART6_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_PROXY_USART6_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_PROXY_USART6_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_USART6_CLKSEL_PROXY_USART6_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* WWD0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY_WWD0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY_WWD0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY_WWD0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY_WWD0_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY_WWD0_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD0_CLKSEL_PROXY_WWD0_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

/* WWD1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY_WWD1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY_WWD1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY_WWD1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY_WWD1_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY_WWD1_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD1_CLKSEL_PROXY_WWD1_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

/* WWD8_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY_WWD8_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY_WWD8_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY_WWD8_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY_WWD8_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY_WWD8_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD8_CLKSEL_PROXY_WWD8_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

/* WWD9_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY_WWD9_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY_WWD9_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY_WWD9_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY_WWD9_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY_WWD9_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD9_CLKSEL_PROXY_WWD9_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

/* WWD10_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY_WWD10_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY_WWD10_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY_WWD10_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY_WWD10_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY_WWD10_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD10_CLKSEL_PROXY_WWD10_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

/* WWD11_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY_WWD11_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY_WWD11_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY_WWD11_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY_WWD11_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY_WWD11_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_WWD11_CLKSEL_PROXY_WWD11_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

/* SERDES0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_PROXY_SERDES0_CLKSEL_CORE_REFCLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_PROXY_SERDES0_CLKSEL_CORE_REFCLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL_PROXY_SERDES0_CLKSEL_CORE_REFCLK_SEL_PROXY_MAX (0x00000003U)

/* MCAN0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_PROXY_MCAN0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_PROXY_MCAN0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN0_CLKSEL_PROXY_MCAN0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

/* MCAN1_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_PROXY_MCAN1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_PROXY_MCAN1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MCAN1_CLKSEL_PROXY_MCAN1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

/* OSPI0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY_OSPI0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY_OSPI0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY_OSPI0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_OSPI0_CLKSEL_PROXY_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_MAX (0x00000001U)

/* ADC0_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_PROXY_ADC0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_PROXY_ADC0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_ADC0_CLKSEL_PROXY_ADC0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

/* DDR0_RFCLK_TEST0_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY_DDR0_RFCLK_TEST0_CLK_CTRL_DIV_RATIO_PROXY_MASK (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY_DDR0_RFCLK_TEST0_CLK_CTRL_DIV_RATIO_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY_DDR0_RFCLK_TEST0_CLK_CTRL_DIV_RATIO_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY_DDR0_RFCLK_TEST0_CLK_CTRL_EN_PROXY_MASK (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY_DDR0_RFCLK_TEST0_CLK_CTRL_EN_PROXY_SHIFT (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST0_CLK_CTRL_PROXY_DDR0_RFCLK_TEST0_CLK_CTRL_EN_PROXY_MAX (0x00000001U)

/* DDR0_RFCLK_TEST1_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY_DDR0_RFCLK_TEST1_CLK_CTRL_DIV_RATIO_PROXY_MASK (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY_DDR0_RFCLK_TEST1_CLK_CTRL_DIV_RATIO_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY_DDR0_RFCLK_TEST1_CLK_CTRL_DIV_RATIO_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY_DDR0_RFCLK_TEST1_CLK_CTRL_EN_PROXY_MASK (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY_DDR0_RFCLK_TEST1_CLK_CTRL_EN_PROXY_SHIFT (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_RFCLK_TEST1_CLK_CTRL_PROXY_DDR0_RFCLK_TEST1_CLK_CTRL_EN_PROXY_MAX (0x00000001U)

/* DDR0_TESTOUT0_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY_DDR0_TESTOUT0_CLK_CTRL_DIV_RATIO_PROXY_MASK (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY_DDR0_TESTOUT0_CLK_CTRL_DIV_RATIO_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY_DDR0_TESTOUT0_CLK_CTRL_DIV_RATIO_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY_DDR0_TESTOUT0_CLK_CTRL_EN_PROXY_MASK (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY_DDR0_TESTOUT0_CLK_CTRL_EN_PROXY_SHIFT (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT0_CLK_CTRL_PROXY_DDR0_TESTOUT0_CLK_CTRL_EN_PROXY_MAX (0x00000001U)

/* DDR0_TESTOUT1_CLK_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY_DDR0_TESTOUT1_CLK_CTRL_DIV_RATIO_PROXY_MASK (0x000000FFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY_DDR0_TESTOUT1_CLK_CTRL_DIV_RATIO_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY_DDR0_TESTOUT1_CLK_CTRL_DIV_RATIO_PROXY_MAX (0x000000FFU)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY_DDR0_TESTOUT1_CLK_CTRL_EN_PROXY_MASK (0x00008000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY_DDR0_TESTOUT1_CLK_CTRL_EN_PROXY_SHIFT (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR0_TESTOUT1_CLK_CTRL_PROXY_DDR0_TESTOUT1_CLK_CTRL_EN_PROXY_MAX (0x00000001U)

/* LOCK2_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK2_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* CLAIMREG_P2_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R2 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R3 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R4 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_CLAIMREG_P2_R4_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_CLAIMREG_P2_R4_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R4_CLAIMREG_P2_R4_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R5 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_CLAIMREG_P2_R5_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_CLAIMREG_P2_R5_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R5_CLAIMREG_P2_R5_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R6 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_CLAIMREG_P2_R6_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_CLAIMREG_P2_R6_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R6_CLAIMREG_P2_R6_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R7 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_CLAIMREG_P2_R7_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_CLAIMREG_P2_R7_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R7_CLAIMREG_P2_R7_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R8 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_CLAIMREG_P2_R8_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_CLAIMREG_P2_R8_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R8_CLAIMREG_P2_R8_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R9 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_CLAIMREG_P2_R9_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_CLAIMREG_P2_R9_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R9_CLAIMREG_P2_R9_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P2_R10 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_CLAIMREG_P2_R10_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_CLAIMREG_P2_R10_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P2_R10_CLAIMREG_P2_R10_MAX        (0xFFFFFFFFU)

/* FUSE_CRC_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_1_MASK                (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_1_SHIFT               (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_1_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_2_MASK                (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_2_SHIFT               (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_2_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_3_MASK                (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_3_SHIFT               (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_3_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_4_MASK                (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_4_SHIFT               (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_4_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_5_MASK                (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_5_SHIFT               (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_5_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_6_MASK                (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_6_SHIFT               (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_6_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_7_MASK                (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_7_SHIFT               (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_CRC_EN_7_MAX                 (0x00000001U)

/* CHAIN1_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN2_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN3_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN4_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN5_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN6_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN7_CRC_FUSE */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_CRC_MAX                    (0xFFFFFFFFU)

/* FUSE_CRC_STAT */

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_1_MASK               (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_1_SHIFT              (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_1_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_2_MASK               (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_2_SHIFT              (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_2_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_3_MASK               (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_3_SHIFT              (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_3_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_4_MASK               (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_4_SHIFT              (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_4_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_5_MASK               (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_5_SHIFT              (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_5_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_6_MASK               (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_6_SHIFT              (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_6_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_7_MASK               (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_7_SHIFT              (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_CRC_ERR_7_MAX                (0x00000001U)

/* CHAIN1_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN2_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN3_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN4_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN5_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN6_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* CHAIN7_CRC_CALC */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_CRC_MASK                   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_CRC_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_CRC_MAX                    (0xFFFFFFFFU)

/* PBIST_EN */

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_EMMC0_MASK                        (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_EMMC0_SHIFT                       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_EMMC0_MAX                         (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_EMMC1_MASK                        (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_EMMC1_SHIFT                       (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_EMMC1_MAX                         (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_USB0_MASK                         (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_USB0_SHIFT                        (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_USB0_MAX                          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PCIE0_MASK                        (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PCIE0_SHIFT                       (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PCIE0_MAX                         (0x00000001U)

/* LOCK3_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK3_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_MAX                (0xFFFFFFFFU)

/* CLAIMREG_P3_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R2_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R3_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R4_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R5_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R6_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_READONLY_CLAIMREG_P3_R6_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_READONLY_CLAIMREG_P3_R6_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_READONLY_CLAIMREG_P3_R6_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R7_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_READONLY_CLAIMREG_P3_R7_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_READONLY_CLAIMREG_P3_R7_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_READONLY_CLAIMREG_P3_R7_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P3_R8_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_READONLY_CLAIMREG_P3_R8_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_READONLY_CLAIMREG_P3_R8_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_READONLY_CLAIMREG_P3_R8_READONLY_MAX (0xFFFFFFFFU)

/* FUSE_CRC_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_1_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_1_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_2_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_2_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_2_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_3_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_3_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_3_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_4_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_4_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_4_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_5_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_5_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_5_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_6_PROXY_MASK (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_6_PROXY_SHIFT (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_6_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_7_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_7_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_CTRL_PROXY_FUSE_CRC_CTRL_CRC_EN_7_PROXY_MAX (0x00000001U)

/* CHAIN1_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_PROXY_CHAIN1_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_PROXY_CHAIN1_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_FUSE_PROXY_CHAIN1_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN2_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_PROXY_CHAIN2_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_PROXY_CHAIN2_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_FUSE_PROXY_CHAIN2_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN3_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_PROXY_CHAIN3_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_PROXY_CHAIN3_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_FUSE_PROXY_CHAIN3_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN4_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_PROXY_CHAIN4_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_PROXY_CHAIN4_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_FUSE_PROXY_CHAIN4_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN5_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_PROXY_CHAIN5_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_PROXY_CHAIN5_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_FUSE_PROXY_CHAIN5_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN6_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_PROXY_CHAIN6_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_PROXY_CHAIN6_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_FUSE_PROXY_CHAIN6_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN7_CRC_FUSE_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_PROXY_CHAIN7_CRC_FUSE_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_PROXY_CHAIN7_CRC_FUSE_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_FUSE_PROXY_CHAIN7_CRC_FUSE_CRC_PROXY_MAX (0xFFFFFFFFU)

/* FUSE_CRC_STAT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_1_PROXY_MASK (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_1_PROXY_SHIFT (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_1_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_2_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_2_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_2_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_3_PROXY_MASK (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_3_PROXY_SHIFT (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_3_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_4_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_4_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_4_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_5_PROXY_MASK (0x00000020U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_5_PROXY_SHIFT (0x00000005U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_5_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_6_PROXY_MASK (0x00000040U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_6_PROXY_SHIFT (0x00000006U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_6_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_7_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_7_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_FUSE_CRC_STAT_PROXY_FUSE_CRC_STAT_CRC_ERR_7_PROXY_MAX (0x00000001U)

/* CHAIN1_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_PROXY_CHAIN1_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_PROXY_CHAIN1_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN1_CRC_CALC_PROXY_CHAIN1_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN2_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_PROXY_CHAIN2_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_PROXY_CHAIN2_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN2_CRC_CALC_PROXY_CHAIN2_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN3_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_PROXY_CHAIN3_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_PROXY_CHAIN3_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN3_CRC_CALC_PROXY_CHAIN3_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN4_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_PROXY_CHAIN4_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_PROXY_CHAIN4_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN4_CRC_CALC_PROXY_CHAIN4_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN5_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_PROXY_CHAIN5_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_PROXY_CHAIN5_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN5_CRC_CALC_PROXY_CHAIN5_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN6_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_PROXY_CHAIN6_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_PROXY_CHAIN6_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN6_CRC_CALC_PROXY_CHAIN6_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* CHAIN7_CRC_CALC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_PROXY_CHAIN7_CRC_CALC_CRC_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_PROXY_CHAIN7_CRC_CALC_CRC_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHAIN7_CRC_CALC_PROXY_CHAIN7_CRC_CALC_CRC_PROXY_MAX (0xFFFFFFFFU)

/* PBIST_EN_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_EMMC0_PROXY_MASK   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_EMMC0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_EMMC0_PROXY_MAX    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_EMMC1_PROXY_MASK   (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_EMMC1_PROXY_SHIFT  (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_EMMC1_PROXY_MAX    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_USB0_PROXY_MASK    (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_USB0_PROXY_SHIFT   (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_USB0_PROXY_MAX     (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_PCIE0_PROXY_MASK   (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_PCIE0_PROXY_SHIFT  (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_PBIST_EN_PROXY_PBIST_EN_PCIE0_PROXY_MAX    (0x00000001U)

/* LOCK3_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK3_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* CLAIMREG_P3_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R2 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R3 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R4 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R5 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R6 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_CLAIMREG_P3_R6_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_CLAIMREG_P3_R6_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R6_CLAIMREG_P3_R6_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R7 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_CLAIMREG_P3_R7_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_CLAIMREG_P3_R7_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R7_CLAIMREG_P3_R7_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P3_R8 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_CLAIMREG_P3_R8_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_CLAIMREG_P3_R8_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P3_R8_CLAIMREG_P3_R8_MAX          (0xFFFFFFFFU)

/* DV_REG0 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG1 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG2 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG3 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG4 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG5 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG6 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG7 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG8 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG9 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_BIT_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_BIT_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_BIT_MAX                            (0xFFFFFFFFU)

/* DV_REG10 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG11 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG12 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG13 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG14 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG15 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG16 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG17 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG18 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG19 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG20 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG21 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG22 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG23 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG24 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG25 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG26 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG27 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG28 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG29 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG30 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG31 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG32 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG33 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG34 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG35 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG36 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG37 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG38 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG39 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG40 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG41 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG42 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG43 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG44 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG45 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG46 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG47 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG48 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG49 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG50 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG51 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG52 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG53 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG54 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG55 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG56 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG57 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG58 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG59 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG60 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG61 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG62 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG63 */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_BIT_MASK                          (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_BIT_SHIFT                         (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_BIT_MAX                           (0xFFFFFFFFU)

/* DV_REG0_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG1_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG2_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG3_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG4_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG5_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG6_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG7_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG8_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG9_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG10_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG11_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG12_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG13_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG14_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG15_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG16_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG17_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG18_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG19_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG20_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG21_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG22_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG23_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG24_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG25_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG26_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG27_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG28_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG29_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG30_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG31_SET */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG0_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG1_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG2_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG3_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG4_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG5_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG6_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG7_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG8_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG9_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_BIT_MASK                       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_BIT_SHIFT                      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_BIT_MAX                        (0xFFFFFFFFU)

/* DV_REG10_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG11_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG12_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG13_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG14_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG15_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG16_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG17_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG18_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG19_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG20_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG21_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG22_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG23_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG24_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG25_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG26_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG27_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG28_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG29_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG30_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* DV_REG31_CLR */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_BIT_MASK                      (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_BIT_SHIFT                     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_BIT_MAX                       (0xFFFFFFFFU)

/* LED_PID */

#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PID_MASK                           (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PID_SHIFT                          (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PID_MAX                            (0xFFFFFFFFU)

/* LED_PIDCTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PID_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PID_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PID_SEL_MAX                    (0x00000001U)

/* MAIN_PLL0_TEST_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_CLK_SEL_MASK         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_CLK_SEL_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_CLK_SEL_MAX          (0x00000001U)

/* MAIN_PLL1_TEST_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_CLK_SEL_MASK         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_CLK_SEL_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_CLK_SEL_MAX          (0x00000001U)

/* MAIN_PLL2_TEST_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_CLK_SEL_MASK         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_CLK_SEL_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_CLK_SEL_MAX          (0x00000001U)

/* MAIN_PLL8_TEST_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_CLK_SEL_MASK         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_CLK_SEL_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_CLK_SEL_MAX          (0x00000001U)

/* MAIN_PLL12_TEST_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_CLK_SEL_MASK        (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_CLK_SEL_SHIFT       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_CLK_SEL_MAX         (0x00000001U)

/* MAIN_PLL14_TEST_CLKSEL */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_CLK_SEL_MASK        (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_CLK_SEL_SHIFT       (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_CLK_SEL_MAX         (0x00000001U)

/* A53SS_DFT_CTL */

#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_A53_BISOR_OVERRIDE_MASK      (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_A53_BISOR_OVERRIDE_SHIFT     (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_A53_BISOR_OVERRIDE_MAX       (0x00000001U)

/* LOCK4_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK4_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_MAX                (0xFFFFFFFFU)

/* CLAIMREG_P4_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R2_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_READONLY_CLAIMREG_P4_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_READONLY_CLAIMREG_P4_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_READONLY_CLAIMREG_P4_R2_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R3_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_READONLY_CLAIMREG_P4_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_READONLY_CLAIMREG_P4_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_READONLY_CLAIMREG_P4_R3_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R4_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_READONLY_CLAIMREG_P4_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_READONLY_CLAIMREG_P4_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_READONLY_CLAIMREG_P4_R4_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R5_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_READONLY_CLAIMREG_P4_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_READONLY_CLAIMREG_P4_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_READONLY_CLAIMREG_P4_R5_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R6_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_READONLY_CLAIMREG_P4_R6_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_READONLY_CLAIMREG_P4_R6_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_READONLY_CLAIMREG_P4_R6_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R7_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_READONLY_CLAIMREG_P4_R7_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_READONLY_CLAIMREG_P4_R7_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_READONLY_CLAIMREG_P4_R7_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R8_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_READONLY_CLAIMREG_P4_R8_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_READONLY_CLAIMREG_P4_R8_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_READONLY_CLAIMREG_P4_R8_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R9_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_READONLY_CLAIMREG_P4_R9_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_READONLY_CLAIMREG_P4_R9_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_READONLY_CLAIMREG_P4_R9_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R10_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_READONLY_CLAIMREG_P4_R10_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_READONLY_CLAIMREG_P4_R10_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_READONLY_CLAIMREG_P4_R10_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R11_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_READONLY_CLAIMREG_P4_R11_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_READONLY_CLAIMREG_P4_R11_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_READONLY_CLAIMREG_P4_R11_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P4_R12_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_READONLY_CLAIMREG_P4_R12_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_READONLY_CLAIMREG_P4_R12_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_READONLY_CLAIMREG_P4_R12_READONLY_MAX (0xFFFFFFFFU)

/* DV_REG0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_PROXY_DV_REG0_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_PROXY_DV_REG0_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_PROXY_DV_REG0_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_PROXY_DV_REG1_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_PROXY_DV_REG1_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_PROXY_DV_REG1_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG2_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_PROXY_DV_REG2_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_PROXY_DV_REG2_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_PROXY_DV_REG2_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG3_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_PROXY_DV_REG3_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_PROXY_DV_REG3_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_PROXY_DV_REG3_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG4_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_PROXY_DV_REG4_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_PROXY_DV_REG4_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_PROXY_DV_REG4_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG5_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_PROXY_DV_REG5_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_PROXY_DV_REG5_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_PROXY_DV_REG5_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG6_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_PROXY_DV_REG6_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_PROXY_DV_REG6_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_PROXY_DV_REG6_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG7_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_PROXY_DV_REG7_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_PROXY_DV_REG7_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_PROXY_DV_REG7_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG8_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_PROXY_DV_REG8_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_PROXY_DV_REG8_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_PROXY_DV_REG8_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG9_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_PROXY_DV_REG9_BIT_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_PROXY_DV_REG9_BIT_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_PROXY_DV_REG9_BIT_PROXY_MAX        (0xFFFFFFFFU)

/* DV_REG10_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_PROXY_DV_REG10_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_PROXY_DV_REG10_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_PROXY_DV_REG10_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG11_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_PROXY_DV_REG11_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_PROXY_DV_REG11_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_PROXY_DV_REG11_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG12_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_PROXY_DV_REG12_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_PROXY_DV_REG12_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_PROXY_DV_REG12_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG13_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_PROXY_DV_REG13_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_PROXY_DV_REG13_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_PROXY_DV_REG13_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG14_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_PROXY_DV_REG14_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_PROXY_DV_REG14_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_PROXY_DV_REG14_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG15_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_PROXY_DV_REG15_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_PROXY_DV_REG15_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_PROXY_DV_REG15_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG16_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_PROXY_DV_REG16_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_PROXY_DV_REG16_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_PROXY_DV_REG16_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG17_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_PROXY_DV_REG17_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_PROXY_DV_REG17_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_PROXY_DV_REG17_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG18_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_PROXY_DV_REG18_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_PROXY_DV_REG18_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_PROXY_DV_REG18_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG19_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_PROXY_DV_REG19_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_PROXY_DV_REG19_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_PROXY_DV_REG19_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG20_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_PROXY_DV_REG20_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_PROXY_DV_REG20_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_PROXY_DV_REG20_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG21_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_PROXY_DV_REG21_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_PROXY_DV_REG21_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_PROXY_DV_REG21_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG22_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_PROXY_DV_REG22_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_PROXY_DV_REG22_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_PROXY_DV_REG22_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG23_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_PROXY_DV_REG23_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_PROXY_DV_REG23_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_PROXY_DV_REG23_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG24_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_PROXY_DV_REG24_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_PROXY_DV_REG24_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_PROXY_DV_REG24_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG25_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_PROXY_DV_REG25_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_PROXY_DV_REG25_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_PROXY_DV_REG25_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG26_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_PROXY_DV_REG26_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_PROXY_DV_REG26_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_PROXY_DV_REG26_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG27_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_PROXY_DV_REG27_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_PROXY_DV_REG27_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_PROXY_DV_REG27_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG28_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_PROXY_DV_REG28_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_PROXY_DV_REG28_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_PROXY_DV_REG28_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG29_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_PROXY_DV_REG29_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_PROXY_DV_REG29_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_PROXY_DV_REG29_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG30_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_PROXY_DV_REG30_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_PROXY_DV_REG30_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_PROXY_DV_REG30_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG31_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_PROXY_DV_REG31_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_PROXY_DV_REG31_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_PROXY_DV_REG31_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG32_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_PROXY_DV_REG32_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_PROXY_DV_REG32_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG32_PROXY_DV_REG32_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG33_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_PROXY_DV_REG33_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_PROXY_DV_REG33_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG33_PROXY_DV_REG33_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG34_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_PROXY_DV_REG34_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_PROXY_DV_REG34_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG34_PROXY_DV_REG34_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG35_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_PROXY_DV_REG35_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_PROXY_DV_REG35_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG35_PROXY_DV_REG35_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG36_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_PROXY_DV_REG36_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_PROXY_DV_REG36_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG36_PROXY_DV_REG36_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG37_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_PROXY_DV_REG37_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_PROXY_DV_REG37_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG37_PROXY_DV_REG37_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG38_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_PROXY_DV_REG38_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_PROXY_DV_REG38_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG38_PROXY_DV_REG38_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG39_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_PROXY_DV_REG39_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_PROXY_DV_REG39_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG39_PROXY_DV_REG39_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG40_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_PROXY_DV_REG40_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_PROXY_DV_REG40_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG40_PROXY_DV_REG40_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG41_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_PROXY_DV_REG41_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_PROXY_DV_REG41_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG41_PROXY_DV_REG41_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG42_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_PROXY_DV_REG42_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_PROXY_DV_REG42_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG42_PROXY_DV_REG42_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG43_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_PROXY_DV_REG43_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_PROXY_DV_REG43_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG43_PROXY_DV_REG43_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG44_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_PROXY_DV_REG44_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_PROXY_DV_REG44_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG44_PROXY_DV_REG44_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG45_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_PROXY_DV_REG45_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_PROXY_DV_REG45_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG45_PROXY_DV_REG45_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG46_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_PROXY_DV_REG46_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_PROXY_DV_REG46_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG46_PROXY_DV_REG46_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG47_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_PROXY_DV_REG47_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_PROXY_DV_REG47_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG47_PROXY_DV_REG47_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG48_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_PROXY_DV_REG48_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_PROXY_DV_REG48_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG48_PROXY_DV_REG48_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG49_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_PROXY_DV_REG49_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_PROXY_DV_REG49_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG49_PROXY_DV_REG49_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG50_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_PROXY_DV_REG50_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_PROXY_DV_REG50_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG50_PROXY_DV_REG50_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG51_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_PROXY_DV_REG51_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_PROXY_DV_REG51_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG51_PROXY_DV_REG51_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG52_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_PROXY_DV_REG52_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_PROXY_DV_REG52_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG52_PROXY_DV_REG52_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG53_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_PROXY_DV_REG53_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_PROXY_DV_REG53_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG53_PROXY_DV_REG53_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG54_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_PROXY_DV_REG54_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_PROXY_DV_REG54_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG54_PROXY_DV_REG54_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG55_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_PROXY_DV_REG55_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_PROXY_DV_REG55_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG55_PROXY_DV_REG55_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG56_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_PROXY_DV_REG56_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_PROXY_DV_REG56_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG56_PROXY_DV_REG56_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG57_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_PROXY_DV_REG57_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_PROXY_DV_REG57_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG57_PROXY_DV_REG57_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG58_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_PROXY_DV_REG58_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_PROXY_DV_REG58_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG58_PROXY_DV_REG58_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG59_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_PROXY_DV_REG59_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_PROXY_DV_REG59_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG59_PROXY_DV_REG59_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG60_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_PROXY_DV_REG60_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_PROXY_DV_REG60_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG60_PROXY_DV_REG60_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG61_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_PROXY_DV_REG61_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_PROXY_DV_REG61_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG61_PROXY_DV_REG61_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG62_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_PROXY_DV_REG62_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_PROXY_DV_REG62_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG62_PROXY_DV_REG62_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG63_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_PROXY_DV_REG63_BIT_PROXY_MASK     (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_PROXY_DV_REG63_BIT_PROXY_SHIFT    (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG63_PROXY_DV_REG63_BIT_PROXY_MAX      (0xFFFFFFFFU)

/* DV_REG0_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_PROXY_DV_REG0_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_PROXY_DV_REG0_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_SET_PROXY_DV_REG0_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG1_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_PROXY_DV_REG1_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_PROXY_DV_REG1_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_SET_PROXY_DV_REG1_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG2_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_PROXY_DV_REG2_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_PROXY_DV_REG2_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_SET_PROXY_DV_REG2_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG3_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_PROXY_DV_REG3_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_PROXY_DV_REG3_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_SET_PROXY_DV_REG3_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG4_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_PROXY_DV_REG4_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_PROXY_DV_REG4_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_SET_PROXY_DV_REG4_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG5_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_PROXY_DV_REG5_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_PROXY_DV_REG5_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_SET_PROXY_DV_REG5_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG6_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_PROXY_DV_REG6_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_PROXY_DV_REG6_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_SET_PROXY_DV_REG6_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG7_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_PROXY_DV_REG7_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_PROXY_DV_REG7_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_SET_PROXY_DV_REG7_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG8_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_PROXY_DV_REG8_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_PROXY_DV_REG8_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_SET_PROXY_DV_REG8_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG9_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_PROXY_DV_REG9_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_PROXY_DV_REG9_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_SET_PROXY_DV_REG9_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG10_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_PROXY_DV_REG10_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_PROXY_DV_REG10_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_SET_PROXY_DV_REG10_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG11_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_PROXY_DV_REG11_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_PROXY_DV_REG11_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_SET_PROXY_DV_REG11_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG12_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_PROXY_DV_REG12_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_PROXY_DV_REG12_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_SET_PROXY_DV_REG12_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG13_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_PROXY_DV_REG13_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_PROXY_DV_REG13_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_SET_PROXY_DV_REG13_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG14_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_PROXY_DV_REG14_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_PROXY_DV_REG14_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_SET_PROXY_DV_REG14_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG15_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_PROXY_DV_REG15_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_PROXY_DV_REG15_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_SET_PROXY_DV_REG15_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG16_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_PROXY_DV_REG16_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_PROXY_DV_REG16_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_SET_PROXY_DV_REG16_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG17_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_PROXY_DV_REG17_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_PROXY_DV_REG17_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_SET_PROXY_DV_REG17_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG18_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_PROXY_DV_REG18_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_PROXY_DV_REG18_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_SET_PROXY_DV_REG18_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG19_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_PROXY_DV_REG19_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_PROXY_DV_REG19_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_SET_PROXY_DV_REG19_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG20_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_PROXY_DV_REG20_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_PROXY_DV_REG20_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_SET_PROXY_DV_REG20_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG21_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_PROXY_DV_REG21_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_PROXY_DV_REG21_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_SET_PROXY_DV_REG21_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG22_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_PROXY_DV_REG22_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_PROXY_DV_REG22_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_SET_PROXY_DV_REG22_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG23_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_PROXY_DV_REG23_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_PROXY_DV_REG23_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_SET_PROXY_DV_REG23_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG24_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_PROXY_DV_REG24_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_PROXY_DV_REG24_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_SET_PROXY_DV_REG24_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG25_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_PROXY_DV_REG25_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_PROXY_DV_REG25_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_SET_PROXY_DV_REG25_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG26_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_PROXY_DV_REG26_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_PROXY_DV_REG26_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_SET_PROXY_DV_REG26_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG27_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_PROXY_DV_REG27_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_PROXY_DV_REG27_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_SET_PROXY_DV_REG27_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG28_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_PROXY_DV_REG28_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_PROXY_DV_REG28_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_SET_PROXY_DV_REG28_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG29_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_PROXY_DV_REG29_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_PROXY_DV_REG29_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_SET_PROXY_DV_REG29_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG30_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_PROXY_DV_REG30_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_PROXY_DV_REG30_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_SET_PROXY_DV_REG30_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG31_SET_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_PROXY_DV_REG31_SET_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_PROXY_DV_REG31_SET_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_SET_PROXY_DV_REG31_SET_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG0_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_PROXY_DV_REG0_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_PROXY_DV_REG0_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG0_CLR_PROXY_DV_REG0_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG1_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_PROXY_DV_REG1_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_PROXY_DV_REG1_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG1_CLR_PROXY_DV_REG1_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG2_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_PROXY_DV_REG2_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_PROXY_DV_REG2_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG2_CLR_PROXY_DV_REG2_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG3_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_PROXY_DV_REG3_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_PROXY_DV_REG3_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG3_CLR_PROXY_DV_REG3_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG4_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_PROXY_DV_REG4_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_PROXY_DV_REG4_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG4_CLR_PROXY_DV_REG4_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG5_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_PROXY_DV_REG5_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_PROXY_DV_REG5_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG5_CLR_PROXY_DV_REG5_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG6_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_PROXY_DV_REG6_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_PROXY_DV_REG6_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG6_CLR_PROXY_DV_REG6_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG7_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_PROXY_DV_REG7_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_PROXY_DV_REG7_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG7_CLR_PROXY_DV_REG7_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG8_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_PROXY_DV_REG8_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_PROXY_DV_REG8_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG8_CLR_PROXY_DV_REG8_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG9_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_PROXY_DV_REG9_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_PROXY_DV_REG9_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG9_CLR_PROXY_DV_REG9_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG10_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_PROXY_DV_REG10_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_PROXY_DV_REG10_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG10_CLR_PROXY_DV_REG10_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG11_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_PROXY_DV_REG11_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_PROXY_DV_REG11_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG11_CLR_PROXY_DV_REG11_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG12_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_PROXY_DV_REG12_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_PROXY_DV_REG12_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG12_CLR_PROXY_DV_REG12_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG13_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_PROXY_DV_REG13_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_PROXY_DV_REG13_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG13_CLR_PROXY_DV_REG13_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG14_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_PROXY_DV_REG14_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_PROXY_DV_REG14_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG14_CLR_PROXY_DV_REG14_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG15_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_PROXY_DV_REG15_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_PROXY_DV_REG15_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG15_CLR_PROXY_DV_REG15_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG16_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_PROXY_DV_REG16_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_PROXY_DV_REG16_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG16_CLR_PROXY_DV_REG16_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG17_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_PROXY_DV_REG17_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_PROXY_DV_REG17_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG17_CLR_PROXY_DV_REG17_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG18_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_PROXY_DV_REG18_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_PROXY_DV_REG18_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG18_CLR_PROXY_DV_REG18_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG19_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_PROXY_DV_REG19_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_PROXY_DV_REG19_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG19_CLR_PROXY_DV_REG19_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG20_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_PROXY_DV_REG20_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_PROXY_DV_REG20_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG20_CLR_PROXY_DV_REG20_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG21_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_PROXY_DV_REG21_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_PROXY_DV_REG21_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG21_CLR_PROXY_DV_REG21_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG22_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_PROXY_DV_REG22_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_PROXY_DV_REG22_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG22_CLR_PROXY_DV_REG22_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG23_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_PROXY_DV_REG23_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_PROXY_DV_REG23_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG23_CLR_PROXY_DV_REG23_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG24_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_PROXY_DV_REG24_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_PROXY_DV_REG24_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG24_CLR_PROXY_DV_REG24_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG25_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_PROXY_DV_REG25_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_PROXY_DV_REG25_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG25_CLR_PROXY_DV_REG25_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG26_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_PROXY_DV_REG26_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_PROXY_DV_REG26_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG26_CLR_PROXY_DV_REG26_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG27_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_PROXY_DV_REG27_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_PROXY_DV_REG27_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG27_CLR_PROXY_DV_REG27_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG28_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_PROXY_DV_REG28_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_PROXY_DV_REG28_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG28_CLR_PROXY_DV_REG28_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG29_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_PROXY_DV_REG29_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_PROXY_DV_REG29_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG29_CLR_PROXY_DV_REG29_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG30_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_PROXY_DV_REG30_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_PROXY_DV_REG30_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG30_CLR_PROXY_DV_REG30_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* DV_REG31_CLR_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_PROXY_DV_REG31_CLR_BIT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_PROXY_DV_REG31_CLR_BIT_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DV_REG31_CLR_PROXY_DV_REG31_CLR_BIT_PROXY_MAX (0xFFFFFFFFU)

/* LED_PID_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PROXY_LED_PID_PID_PROXY_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PROXY_LED_PID_PID_PROXY_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PID_PROXY_LED_PID_PID_PROXY_MAX        (0xFFFFFFFFU)

/* LED_PIDCTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PROXY_LED_PIDCTRL_PID_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PROXY_LED_PIDCTRL_PID_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LED_PIDCTRL_PROXY_LED_PIDCTRL_PID_SEL_PROXY_MAX (0x00000001U)

/* MAIN_PLL0_TEST_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_PROXY_MAIN_PLL0_TEST_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_PROXY_MAIN_PLL0_TEST_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL0_TEST_CLKSEL_PROXY_MAIN_PLL0_TEST_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* MAIN_PLL1_TEST_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_PROXY_MAIN_PLL1_TEST_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_PROXY_MAIN_PLL1_TEST_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL1_TEST_CLKSEL_PROXY_MAIN_PLL1_TEST_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* MAIN_PLL2_TEST_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_PROXY_MAIN_PLL2_TEST_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_PROXY_MAIN_PLL2_TEST_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL2_TEST_CLKSEL_PROXY_MAIN_PLL2_TEST_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* MAIN_PLL8_TEST_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_PROXY_MAIN_PLL8_TEST_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_PROXY_MAIN_PLL8_TEST_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL8_TEST_CLKSEL_PROXY_MAIN_PLL8_TEST_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* MAIN_PLL12_TEST_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_PROXY_MAIN_PLL12_TEST_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_PROXY_MAIN_PLL12_TEST_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL12_TEST_CLKSEL_PROXY_MAIN_PLL12_TEST_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* MAIN_PLL14_TEST_CLKSEL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_PROXY_MAIN_PLL14_TEST_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_PROXY_MAIN_PLL14_TEST_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_MAIN_PLL14_TEST_CLKSEL_PROXY_MAIN_PLL14_TEST_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

/* A53SS_DFT_CTL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_PROXY_A53SS_DFT_CTL_A53_BISOR_OVERRIDE_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_PROXY_A53SS_DFT_CTL_A53_BISOR_OVERRIDE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_A53SS_DFT_CTL_PROXY_A53SS_DFT_CTL_A53_BISOR_OVERRIDE_PROXY_MAX (0x00000001U)

/* LOCK4_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK4_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* CLAIMREG_P4_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R2 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_CLAIMREG_P4_R2_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_CLAIMREG_P4_R2_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R2_CLAIMREG_P4_R2_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R3 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_CLAIMREG_P4_R3_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_CLAIMREG_P4_R3_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R3_CLAIMREG_P4_R3_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R4 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_CLAIMREG_P4_R4_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_CLAIMREG_P4_R4_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R4_CLAIMREG_P4_R4_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R5 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_CLAIMREG_P4_R5_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_CLAIMREG_P4_R5_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R5_CLAIMREG_P4_R5_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R6 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_CLAIMREG_P4_R6_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_CLAIMREG_P4_R6_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R6_CLAIMREG_P4_R6_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R7 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_CLAIMREG_P4_R7_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_CLAIMREG_P4_R7_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R7_CLAIMREG_P4_R7_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R8 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_CLAIMREG_P4_R8_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_CLAIMREG_P4_R8_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R8_CLAIMREG_P4_R8_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R9 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_CLAIMREG_P4_R9_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_CLAIMREG_P4_R9_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R9_CLAIMREG_P4_R9_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P4_R10 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_CLAIMREG_P4_R10_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_CLAIMREG_P4_R10_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R10_CLAIMREG_P4_R10_MAX        (0xFFFFFFFFU)

/* CLAIMREG_P4_R11 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_CLAIMREG_P4_R11_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_CLAIMREG_P4_R11_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R11_CLAIMREG_P4_R11_MAX        (0xFFFFFFFFU)

/* CLAIMREG_P4_R12 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_CLAIMREG_P4_R12_MASK       (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_CLAIMREG_P4_R12_SHIFT      (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P4_R12_CLAIMREG_P4_R12_MAX        (0xFFFFFFFFU)

/* CHNG_DDR4_FSP_REQ */

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_REQ_TYPE_MASK            (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_REQ_TYPE_SHIFT           (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_REQ_TYPE_MAX             (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_REQ_MASK                 (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_REQ_SHIFT                (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_REQ_MAX                  (0x00000001U)

/* CHNG_DDR4_FSP_ACK */

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_ERROR_MASK               (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_ERROR_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_ERROR_MAX                (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_ACK_MASK                 (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_ACK_SHIFT                (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_ACK_MAX                  (0x00000001U)

/* DDR4_FSP_CLKCHNG_REQ */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_REQ_TYPE_MASK         (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_REQ_TYPE_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_REQ_TYPE_MAX          (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_REQ_MASK              (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_REQ_SHIFT             (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_REQ_MAX               (0x00000001U)

/* DDR4_FSP_CLKCHNG_ACK */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_ACK_MASK              (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_ACK_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_ACK_MAX               (0x00000001U)

/* LOCK5_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_LOCK5_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_LOCK5_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_LOCK5_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK5_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_LOCK5_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_LOCK5_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_LOCK5_KICK1_MAX                (0xFFFFFFFFU)

/* CLAIMREG_P5_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_READONLY_CLAIMREG_P5_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_READONLY_CLAIMREG_P5_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_READONLY_CLAIMREG_P5_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P5_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_READONLY_CLAIMREG_P5_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_READONLY_CLAIMREG_P5_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_READONLY_CLAIMREG_P5_R1_READONLY_MAX (0xFFFFFFFFU)

/* CHNG_DDR4_FSP_REQ_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY_CHNG_DDR4_FSP_REQ_REQ_TYPE_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY_CHNG_DDR4_FSP_REQ_REQ_TYPE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY_CHNG_DDR4_FSP_REQ_REQ_TYPE_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY_CHNG_DDR4_FSP_REQ_REQ_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY_CHNG_DDR4_FSP_REQ_REQ_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_REQ_PROXY_CHNG_DDR4_FSP_REQ_REQ_PROXY_MAX (0x00000001U)

/* CHNG_DDR4_FSP_ACK_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY_CHNG_DDR4_FSP_ACK_ERROR_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY_CHNG_DDR4_FSP_ACK_ERROR_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY_CHNG_DDR4_FSP_ACK_ERROR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY_CHNG_DDR4_FSP_ACK_ACK_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY_CHNG_DDR4_FSP_ACK_ACK_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_CHNG_DDR4_FSP_ACK_PROXY_CHNG_DDR4_FSP_ACK_ACK_PROXY_MAX (0x00000001U)

/* DDR4_FSP_CLKCHNG_REQ_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY_DDR4_FSP_CLKCHNG_REQ_REQ_TYPE_PROXY_MASK (0x00000003U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY_DDR4_FSP_CLKCHNG_REQ_REQ_TYPE_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY_DDR4_FSP_CLKCHNG_REQ_REQ_TYPE_PROXY_MAX (0x00000003U)

#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY_DDR4_FSP_CLKCHNG_REQ_REQ_PROXY_MASK (0x00000080U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY_DDR4_FSP_CLKCHNG_REQ_REQ_PROXY_SHIFT (0x00000007U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_REQ_PROXY_DDR4_FSP_CLKCHNG_REQ_REQ_PROXY_MAX (0x00000001U)

/* DDR4_FSP_CLKCHNG_ACK_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_PROXY_DDR4_FSP_CLKCHNG_ACK_ACK_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_PROXY_DDR4_FSP_CLKCHNG_ACK_ACK_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_DDR4_FSP_CLKCHNG_ACK_PROXY_DDR4_FSP_CLKCHNG_ACK_ACK_PROXY_MAX (0x00000001U)

/* LOCK5_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_PROXY_LOCK5_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_PROXY_LOCK5_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0_PROXY_LOCK5_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK5_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_PROXY_LOCK5_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_PROXY_LOCK5_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1_PROXY_LOCK5_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* CLAIMREG_P5_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_CLAIMREG_P5_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_CLAIMREG_P5_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R0_CLAIMREG_P5_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P5_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_CLAIMREG_P5_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_CLAIMREG_P5_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P5_R1_CLAIMREG_P5_R1_MAX          (0xFFFFFFFFU)

/* RST_CTRL */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_SW_MAIN_WARMRST_MASK              (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_SW_MAIN_WARMRST_SHIFT             (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_SW_MAIN_WARMRST_MAX               (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_SW_MAIN_POR_MASK                  (0x000000F0U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_SW_MAIN_POR_SHIFT                 (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_SW_MAIN_POR_MAX                   (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_DMSC_COLD_RESET_EN_Z_MASK         (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_DMSC_COLD_RESET_EN_Z_SHIFT        (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_DMSC_COLD_RESET_EN_Z_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_MAIN_ESM_ERROR_RST_EN_Z_MASK      (0x00020000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_MAIN_ESM_ERROR_RST_EN_Z_SHIFT     (0x00000011U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_MAIN_ESM_ERROR_RST_EN_Z_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_MAIN_RESET_ISO_DONE_Z_MASK        (0x00040000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_MAIN_RESET_ISO_DONE_Z_SHIFT       (0x00000012U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_MAIN_RESET_ISO_DONE_Z_MAX         (0x00000001U)

/* RST_STAT */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_MCU_RESET_ISO_DONE_Z_MASK         (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_MCU_RESET_ISO_DONE_Z_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_MCU_RESET_ISO_DONE_Z_MAX          (0x00000001U)

/* RST_SRC */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MCU_RESET_PIN_MASK                 (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MCU_RESET_PIN_SHIFT                (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MCU_RESET_PIN_MAX                  (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MAIN_RESET_REQ_MASK                (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MAIN_RESET_REQ_SHIFT               (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MAIN_RESET_REQ_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_THERMAL_RST_MASK                   (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_THERMAL_RST_SHIFT                  (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_THERMAL_RST_MAX                    (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_DEBUG_RST_MASK                     (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_DEBUG_RST_SHIFT                    (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_DEBUG_RST_MAX                      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_COLD_OUT_RST_MASK                  (0x00001000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_COLD_OUT_RST_SHIFT                 (0x0000000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_COLD_OUT_RST_MAX                   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_WARM_OUT_RST_MASK                  (0x00002000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_WARM_OUT_RST_SHIFT                 (0x0000000DU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_WARM_OUT_RST_MAX                   (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MCU_WARMRST_MASK                (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MCU_WARMRST_SHIFT               (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MCU_WARMRST_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_MASK      (0x00100000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_SHIFT     (0x00000014U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_MAX       (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_MASK     (0x00200000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_SHIFT    (0x00000015U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_MAX      (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MCU_MASK          (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MCU_SHIFT         (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MCU_MAX           (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MAIN_MASK         (0x02000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MAIN_SHIFT        (0x00000019U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MAIN_MAX          (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MAIN_ESM_ERROR_MASK                (0x40000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MAIN_ESM_ERROR_SHIFT               (0x0000001EU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MAIN_ESM_ERROR_MAX                 (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MCU_ESM_ERROR_MASK                 (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MCU_ESM_ERROR_SHIFT                (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_MCU_ESM_ERROR_MAX                  (0x00000001U)

/* RST_MAGIC_WORD */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_MCU_MAGIC_WORD_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_MCU_MAGIC_WORD_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_MCU_MAGIC_WORD_MAX          (0xFFFFFFFFU)

/* LOCK6_KICK0 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_MAX                (0xFFFFFFFFU)

/* LOCK6_KICK1 */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_MASK               (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_SHIFT              (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_MAX                (0xFFFFFFFFU)

/* CLAIMREG_P6_R0_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P6_R1_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_READONLY_CLAIMREG_P6_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_READONLY_CLAIMREG_P6_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_READONLY_CLAIMREG_P6_R1_READONLY_MAX (0xFFFFFFFFU)

/* CLAIMREG_P6_R2_READONLY */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_READONLY_CLAIMREG_P6_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_READONLY_CLAIMREG_P6_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_READONLY_CLAIMREG_P6_R2_READONLY_MAX (0xFFFFFFFFU)

/* RST_CTRL_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_SW_MAIN_WARMRST_PROXY_MASK (0x0000000FU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_SW_MAIN_WARMRST_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_SW_MAIN_WARMRST_PROXY_MAX (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_SW_MAIN_POR_PROXY_MASK (0x000000F0U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_SW_MAIN_POR_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_SW_MAIN_POR_PROXY_MAX (0x0000000FU)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_DMSC_COLD_RESET_EN_Z_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_DMSC_COLD_RESET_EN_Z_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_DMSC_COLD_RESET_EN_Z_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_MAIN_ESM_ERROR_RST_EN_Z_PROXY_MASK (0x00020000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_MAIN_ESM_ERROR_RST_EN_Z_PROXY_SHIFT (0x00000011U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_MAIN_ESM_ERROR_RST_EN_Z_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_MAIN_RESET_ISO_DONE_Z_PROXY_MASK (0x00040000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_MAIN_RESET_ISO_DONE_Z_PROXY_SHIFT (0x00000012U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_CTRL_PROXY_RST_CTRL_MAIN_RESET_ISO_DONE_Z_PROXY_MAX (0x00000001U)

/* RST_STAT_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_PROXY_RST_STAT_MCU_RESET_ISO_DONE_Z_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_PROXY_RST_STAT_MCU_RESET_ISO_DONE_Z_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_STAT_PROXY_RST_STAT_MCU_RESET_ISO_DONE_Z_PROXY_MAX (0x00000001U)

/* RST_SRC_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MCU_RESET_PIN_PROXY_MASK (0x00000001U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MCU_RESET_PIN_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MCU_RESET_PIN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MAIN_RESET_REQ_PROXY_MASK (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MAIN_RESET_REQ_PROXY_SHIFT (0x00000002U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MAIN_RESET_REQ_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_THERMAL_RST_PROXY_MASK (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_THERMAL_RST_PROXY_SHIFT (0x00000004U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_THERMAL_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_DEBUG_RST_PROXY_MASK (0x00000100U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_DEBUG_RST_PROXY_SHIFT (0x00000008U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_DEBUG_RST_PROXY_MAX  (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_COLD_OUT_RST_PROXY_MASK (0x00001000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_COLD_OUT_RST_PROXY_SHIFT (0x0000000CU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_COLD_OUT_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_WARM_OUT_RST_PROXY_MASK (0x00002000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_WARM_OUT_RST_PROXY_SHIFT (0x0000000DU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_WARM_OUT_RST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MCU_WARMRST_PROXY_MASK (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MCU_WARMRST_PROXY_SHIFT (0x00000010U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MCU_WARMRST_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_PROXY_MASK (0x00100000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_PROXY_SHIFT (0x00000014U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_PROXY_MASK (0x00200000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_PROXY_SHIFT (0x00000015U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_POR_FROM_MCU_PROXY_MASK (0x01000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_POR_FROM_MCU_PROXY_SHIFT (0x00000018U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_POR_FROM_MCU_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_POR_FROM_MAIN_PROXY_MASK (0x02000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_POR_FROM_MAIN_PROXY_SHIFT (0x00000019U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SW_MAIN_POR_FROM_MAIN_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MAIN_ESM_ERROR_PROXY_MASK (0x40000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MAIN_ESM_ERROR_PROXY_SHIFT (0x0000001EU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_MAIN_ESM_ERROR_PROXY_MAX (0x00000001U)

#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SAFETY_ERROR_PROXY_MASK (0x80000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SAFETY_ERROR_PROXY_SHIFT (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_PROXY_RST_SRC_SAFETY_ERROR_PROXY_MAX (0x00000001U)

/* RST_MAGIC_WORD_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_PROXY_RST_MAGIC_WORD_MCU_MAGIC_WORD_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_PROXY_RST_MAGIC_WORD_MCU_MAGIC_WORD_PROXY_SHIFT (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_RST_MAGIC_WORD_PROXY_RST_MAGIC_WORD_MCU_MAGIC_WORD_PROXY_MAX (0xFFFFFFFFU)

/* LOCK6_KICK0_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_MAX    (0xFFFFFFFFU)

/* LOCK6_KICK1_PROXY */

#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_MASK   (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_SHIFT  (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_MAX    (0xFFFFFFFFU)

/* CLAIMREG_P6_R0 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P6_R1 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_CLAIMREG_P6_R1_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_CLAIMREG_P6_R1_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R1_CLAIMREG_P6_R1_MAX          (0xFFFFFFFFU)

/* CLAIMREG_P6_R2 */

#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_CLAIMREG_P6_R2_MASK         (0xFFFFFFFFU)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_CLAIMREG_P6_R2_SHIFT        (0x00000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CLAIMREG_P6_R2_CLAIMREG_P6_R2_MAX          (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
