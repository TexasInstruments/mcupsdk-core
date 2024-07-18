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
 *  Name        : cslr_top_ctrl.h
*/
#ifndef CSLR_TOP_CTRL_H_
#define CSLR_TOP_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
	volatile uint32_t	PID;
	volatile uint8_t	Resv_16[12];
	volatile uint32_t	EFUSE_DIEID0;
	volatile uint32_t	EFUSE_DIEID1;
	volatile uint32_t	EFUSE_DIEID2;
	volatile uint32_t	EFUSE_DIEID3;
	volatile uint32_t	EFUSE_UID0;
	volatile uint32_t	EFUSE_UID1;
	volatile uint32_t	EFUSE_UID2;
	volatile uint32_t	EFUSE_UID3;
	volatile uint32_t	EFUSE_DEVICE_TYPE;
	volatile uint32_t	EFUSE_FROM0_CHECKSUM;
	volatile uint32_t	EFUSE_JTAG_USERCODE_ID;
	volatile uint8_t	Resv_1024[964];
	volatile uint32_t	EFUSE0_ROW_61;
	volatile uint32_t	EFUSE0_ROW_62;
	volatile uint32_t	EFUSE0_ROW_63;
	volatile uint32_t	EFUSE1_ROW_5;
	volatile uint32_t	EFUSE1_ROW_6;
	volatile uint32_t	EFUSE1_ROW_7;
	volatile uint32_t	EFUSE1_ROW_8;
	volatile uint32_t	EFUSE1_ROW_9;
	volatile uint32_t	EFUSE1_ROW_10;
	volatile uint32_t	EFUSE1_ROW_11;
	volatile uint32_t	EFUSE1_ROW_12;
	volatile uint32_t	EFUSE1_ROW_13;
	volatile uint32_t	EFUSE1_ROW_14;
	volatile uint32_t	EFUSE1_ROW_15;
	volatile uint32_t	EFUSE1_ROW_16;
	volatile uint32_t	EFUSE1_ROW_17;
	volatile uint32_t	EFUSE1_ROW_18;
	volatile uint32_t	EFUSE1_ROW_19;
	volatile uint32_t	EFUSE1_ROW_20;
	volatile uint32_t	EFUSE1_ROW_21;
	volatile uint32_t	EFUSE1_ROW_22;
	volatile uint32_t	EFUSE1_ROW_23;
	volatile uint32_t	EFUSE1_ROW_24;
	volatile uint32_t	EFUSE1_ROW_25;
	volatile uint32_t	EFUSE1_ROW_26;
	volatile uint32_t	EFUSE1_ROW_27;
	volatile uint32_t	EFUSE1_ROW_28;
	volatile uint32_t	EFUSE1_ROW_29;
	volatile uint32_t	EFUSE1_ROW_30;
	volatile uint32_t	EFUSE1_ROW_31;
	volatile uint32_t	EFUSE1_ROW_32;
	volatile uint32_t	EFUSE1_ROW_33;
	volatile uint32_t	EFUSE1_ROW_34;
	volatile uint32_t	EFUSE1_ROW_35;
	volatile uint32_t	EFUSE1_ROW_36;
	volatile uint32_t	EFUSE1_ROW_37;
	volatile uint32_t	EFUSE1_ROW_38;
	volatile uint32_t	EFUSE1_ROW_39;
	volatile uint32_t	EFUSE1_ROW_40;
	volatile uint32_t	EFUSE1_ROW_41;
	volatile uint32_t	EFUSE1_ROW_42;
	volatile uint32_t	EFUSE1_ROW_43;
	volatile uint32_t	EFUSE1_ROW_44;
	volatile uint32_t	EFUSE1_ROW_45;
	volatile uint32_t	EFUSE1_ROW_46;
	volatile uint32_t	EFUSE1_ROW_47;
	volatile uint32_t	EFUSE1_ROW_48;
	volatile uint32_t	EFUSE1_ROW_49;
	volatile uint32_t	EFUSE1_ROW_50;
	volatile uint32_t	EFUSE1_ROW_51;
	volatile uint32_t	EFUSE1_ROW_52;
	volatile uint32_t	EFUSE1_ROW_53;
	volatile uint32_t	EFUSE1_ROW_54;
	volatile uint32_t	EFUSE1_ROW_55;
	volatile uint32_t	EFUSE1_ROW_56;
	volatile uint8_t	Resv_1280[36];
	volatile uint32_t	MAC_ID0;
	volatile uint32_t	MAC_ID1;
	volatile uint8_t	Resv_1296[8];
	volatile uint32_t	TRIM_TEMP_M40C;
	volatile uint32_t	TRIM_TEMPSENSE_M40C0;
	volatile uint32_t	TRIM_TEMPSENSE_M40C1;
	volatile uint32_t	TRIM_TEMP_150C;
	volatile uint32_t	TRIM_TEMPSENSE_150C0;
	volatile uint32_t	TRIM_TEMPSENSE_150C1;
	volatile uint32_t	TRIM_TEMP_30C;
	volatile uint32_t	TRIM_TEMPSENSE_30C0;
	volatile uint32_t	TRIM_TEMPSENSE_30C1;
	volatile uint32_t	N_FACTOR_TEMPSENSE;
	volatile uint32_t	TSHUT_HOT;
	volatile uint32_t	TSHUT_COLD;
	volatile uint32_t	JTAG_ID;
	volatile uint8_t	Resv_2048[700];
	volatile uint32_t	EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN;
	volatile uint32_t	EFUSE_OVERRIDE_MARGINCTRL;
	volatile uint32_t	EFUSE_OVERRIDE_ADC0_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_ADC1_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_ADC2_TRIM;
	volatile uint8_t	Resv_2096[28];
	volatile uint32_t	EFUSE_OVERRIDE_ADC_CFG_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_ADC_CFG0;
	volatile uint32_t	EFUSE_OVERRIDE_ADC_CFG1;
	volatile uint32_t	EFUSE_OVERRIDE_ADC_CFG2;
	volatile uint8_t	Resv_2176[64];
	volatile uint32_t	EFUSE_OVERRIDE_CSSA0_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA0_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA1_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA1_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA2_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA2_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA3_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA3_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA4_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA4_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA5_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA5_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA6_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA6_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA7_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA7_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA8_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSSA8_TRIM;
	volatile uint8_t	Resv_2352[104];
	volatile uint32_t	EFUSE_OVERRIDE_CSS_CFG_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_CSS_CFG0;
	volatile uint8_t	Resv_2364[4];
	volatile uint32_t	EFUSE_OVERRIDE_DAC_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_DAC_CFG;
	volatile uint32_t	EFUSE_OVERRIDE_REFBUF0_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_REFBUF0_CFG;
	volatile uint8_t	Resv_2400[20];
	volatile uint32_t	EFUSE_OVERRIDE_PMU_CFG;
	volatile uint32_t	EFUSE_OVERRIDE_PMU_SPARE_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_LDO_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_BG_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0;
	volatile uint32_t	EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1;
	volatile uint32_t	EFUSE_SAFETYMON_SPARE;
	volatile uint32_t	EFUSE_OVERRIDE_TSENSE_TRIM_CTRL;
	volatile uint32_t	EFUSE_OVERRIDE_TSENSE_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_PLL_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_RCOSC_TRIM;
	volatile uint32_t	EFUSE_OVERRIDE_RCOSC_CFG;
	volatile uint32_t	EFUSE_OVERRIDE_HOPP_MUX;
	volatile uint8_t	Resv_2564[108];
	volatile uint32_t	EFUSE_SPARE_0;
	volatile uint32_t	EFUSE_SPARE_1;
	volatile uint32_t	EFUSE_SPARE_2;
	volatile uint8_t	Resv_2816[240];
	volatile uint32_t	ADC_REFBUF0_CTRL;
	volatile uint8_t	Resv_2832[12];
	volatile uint32_t	ADC_REF_COMP_CTRL_CTRL;
	volatile uint32_t	ADC_REF_GOOD_STATUS_STATUS;
	volatile uint32_t	ADC_REF_RNG_CTRL;
	volatile uint32_t	ADC0_OSD_CHEN;
	volatile uint32_t	ADC1_OSD_CHEN;
	volatile uint32_t	ADC2_OSD_CHEN;
	volatile uint8_t	Resv_2880[24];
	volatile uint32_t	ADC0_OSD_CTRL;
	volatile uint32_t	ADC1_OSD_CTRL;
	volatile uint32_t	ADC2_OSD_CTRL;
	volatile uint8_t	Resv_2912[20];
	volatile uint32_t	ADC_LOOPBACK_CTRL;
	volatile uint8_t	Resv_3072[156];
	volatile uint32_t	VMON_UV;
	volatile uint32_t	VMON_OV;
	volatile uint32_t	VMON_CONTROLLER;
	volatile uint32_t	VMON_CTRL;
	volatile uint32_t	VMON_STAT;
	volatile uint32_t	MASK_VMON_ERROR_ESM_H;
	volatile uint32_t	MASK_VMON_ERROR_ESM_L;
	volatile uint32_t	VMON_FILTER_CTRL;
	volatile uint32_t	PMU_COARSE_STAT;
	volatile uint32_t	PMU_CTRL_EXTREF;
	volatile uint8_t	Resv_3328[216];
	volatile uint32_t	TSENSE_CFG;
	volatile uint32_t	TSENSE_STATUS;
	volatile uint32_t	TSENSE_STATUS_RAW;
	volatile uint8_t	Resv_3344[4];
	volatile uint32_t	TSENSE0_TSHUT;
	volatile uint32_t	TSENSE0_ALERT;
	volatile uint32_t	TSENSE0_CNTL;
	volatile uint32_t	TSENSE0_RESULT;
	volatile uint32_t	TSENSE0_DATA0;
	volatile uint32_t	TSENSE0_DATA1;
	volatile uint32_t	TSENSE0_DATA2;
	volatile uint32_t	TSENSE0_DATA3;
	volatile uint32_t	TSENSE0_ACCU;
	volatile uint8_t	Resv_3392[12];
	volatile uint32_t	TSENSE1_TSHUT;
	volatile uint32_t	TSENSE1_ALERT;
	volatile uint32_t	TSENSE1_CNTL;
	volatile uint32_t	TSENSE1_RESULT;
	volatile uint32_t	TSENSE1_DATA0;
	volatile uint32_t	TSENSE1_DATA1;
	volatile uint32_t	TSENSE1_DATA2;
	volatile uint32_t	TSENSE1_DATA3;
	volatile uint32_t	TSENSE1_ACCU;
	volatile uint8_t	Resv_3452[24];
	volatile uint32_t	TSENSE2_RESULT;
	volatile uint8_t	Resv_3500[44];
	volatile uint32_t	TSENSE3_RESULT;
	volatile uint8_t	Resv_3520[16];
	volatile uint32_t	ICSSM1_GPIO_OUT_CTRL;
	volatile uint32_t	MASK_ANA_ISO;
	volatile uint32_t	CMPSSA_LOOPBACK_CTRL;
	volatile uint8_t	Resv_3584[52];
	volatile uint32_t	DFT_ATB_GLOBALEN_ADC_CSS;
	volatile uint32_t	DFT_ATB0_MASTEREN_ADC_CSS_DAC;
	volatile uint32_t	DFT_ATB1_MASTEREN_ADC_CSS_DAC;
	volatile uint32_t	DFT_PMU_REFSYS_SAFETY;
	volatile uint32_t	DFT_ANA_DTB_ENABLES;
	volatile uint32_t	DFT_ADC_CHSEL_OV_CTRL_VALUE;
	volatile uint32_t	DFT_DAC_CTRL;
	volatile uint32_t	DFT_CSS0_CTRL;
	volatile uint32_t	DFT_CSS1_CTRL;
	volatile uint32_t	DFT_CSS2_CTRL;
	volatile uint8_t	Resv_3632[8];
	volatile uint32_t	DFT_RAMP_DACL;
	volatile uint32_t	DFT_REFBUF_CTRL;
	volatile uint32_t	DFT_ODP_ATB_LOOPBACK_CTRL;
	volatile uint32_t	DFT_SOC_DTB_MUX_SEL;
	volatile uint32_t	DFT_TEMPSENSE_CTRL;
	volatile uint32_t	DFT_CTRL_1;
	volatile uint32_t	DFT_CTRL_2;
	volatile uint32_t	DFT_CTRL_3;
	volatile uint32_t	DFT_CTRL_4;
	volatile uint32_t	DFT_CTRL_5;
	volatile uint8_t	Resv_3680[8];
	volatile uint32_t	DFT_ADC0_CTRL;
	volatile uint32_t	DFT_ADC1_CTRL;
	volatile uint32_t	DFT_ADC2_CTRL;
	volatile uint32_t	DFT_SDADC_CLKBUF_CTRL;
	volatile uint32_t	DFT_SDADC_RESOLVEREF_SEL;
	volatile uint32_t	DFT_SDADC;
	volatile uint8_t	Resv_3844[140];
	volatile uint32_t	PROBE_BUS_SEL0;
	volatile uint32_t	PROBE_BUS_SEL1;
	volatile uint8_t	Resv_3904[52];
	volatile uint32_t	HW_SPARE_RW0;
	volatile uint32_t	HW_SPARE_RW1;
	volatile uint32_t	HW_SPARE_RW2;
	volatile uint32_t	HW_SPARE_RW3;
	volatile uint32_t	HW_SPARE_PORZ_RW0;
	volatile uint32_t	HW_SPARE_PORZ_RW1;
	volatile uint8_t	Resv_3968[40];
	volatile uint32_t	HW_SPARE_RO0;
	volatile uint32_t	HW_SPARE_RO1;
	volatile uint32_t	HW_SPARE_RO2;
	volatile uint32_t	HW_SPARE_RO3;
	volatile uint8_t	Resv_4032[48];
	volatile uint32_t	HW_SPARE_WPH;
	volatile uint32_t	HW_SPARE_REC;
	volatile uint32_t	HW_SPARE_REC0;
	volatile uint32_t	HW_SPARE_REC1;
	volatile uint32_t	EFUSE_VPP_EN;
	volatile uint8_t	Resv_4104[52];
	volatile uint32_t	LOCK0_KICK0;
	volatile uint32_t	LOCK0_KICK1;
	volatile uint32_t	INTR_RAW_STATUS;
	volatile uint32_t	INTR_ENABLED_STATUS_CLEAR;
	volatile uint32_t	INTR_ENABLE;
	volatile uint32_t	INTR_ENABLE_CLEAR;
	volatile uint32_t	EOI;
	volatile uint32_t	FAULT_ADDRESS;
	volatile uint32_t	FAULT_TYPE_STATUS;
	volatile uint32_t	FAULT_ATTR_STATUS;
	volatile uint32_t	FAULT_CLEAR;
} CSL_top_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/*--------TOP_CTRL_--------*/
#define CSL_TOP_CTRL_PID                                                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID0                                               (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_DIEID1                                               (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_DIEID2                                               (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_DIEID3                                               (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE_UID0                                                 (0x00000020U)
#define CSL_TOP_CTRL_EFUSE_UID1                                                 (0x00000024U)
#define CSL_TOP_CTRL_EFUSE_UID2                                                 (0x00000028U)
#define CSL_TOP_CTRL_EFUSE_UID3                                                 (0x0000002CU)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE                                          (0x00000030U)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM                                       (0x00000034U)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID                                     (0x00000038U)
#define CSL_TOP_CTRL_EFUSE0_ROW_61                                              (0x00000400U)
#define CSL_TOP_CTRL_EFUSE0_ROW_62                                              (0x00000404U)
#define CSL_TOP_CTRL_EFUSE0_ROW_63                                              (0x00000408U)
#define CSL_TOP_CTRL_EFUSE1_ROW_5                                               (0x0000040CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_6                                               (0x00000410U)
#define CSL_TOP_CTRL_EFUSE1_ROW_7                                               (0x00000414U)
#define CSL_TOP_CTRL_EFUSE1_ROW_8                                               (0x00000418U)
#define CSL_TOP_CTRL_EFUSE1_ROW_9                                               (0x0000041CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_10                                              (0x00000420U)
#define CSL_TOP_CTRL_EFUSE1_ROW_11                                              (0x00000424U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12                                              (0x00000428U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13                                              (0x0000042CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_14                                              (0x00000430U)
#define CSL_TOP_CTRL_EFUSE1_ROW_15                                              (0x00000434U)
#define CSL_TOP_CTRL_EFUSE1_ROW_16                                              (0x00000438U)
#define CSL_TOP_CTRL_EFUSE1_ROW_17                                              (0x0000043CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_18                                              (0x00000440U)
#define CSL_TOP_CTRL_EFUSE1_ROW_19                                              (0x00000444U)
#define CSL_TOP_CTRL_EFUSE1_ROW_20                                              (0x00000448U)
#define CSL_TOP_CTRL_EFUSE1_ROW_21                                              (0x0000044CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_22                                              (0x00000450U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23                                              (0x00000454U)
#define CSL_TOP_CTRL_EFUSE1_ROW_24                                              (0x00000458U)
#define CSL_TOP_CTRL_EFUSE1_ROW_25                                              (0x0000045CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_26                                              (0x00000460U)
#define CSL_TOP_CTRL_EFUSE1_ROW_27                                              (0x00000464U)
#define CSL_TOP_CTRL_EFUSE1_ROW_28                                              (0x00000468U)
#define CSL_TOP_CTRL_EFUSE1_ROW_29                                              (0x0000046CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_30                                              (0x00000470U)
#define CSL_TOP_CTRL_EFUSE1_ROW_31                                              (0x00000474U)
#define CSL_TOP_CTRL_EFUSE1_ROW_32                                              (0x00000478U)
#define CSL_TOP_CTRL_EFUSE1_ROW_33                                              (0x0000047CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_34                                              (0x00000480U)
#define CSL_TOP_CTRL_EFUSE1_ROW_35                                              (0x00000484U)
#define CSL_TOP_CTRL_EFUSE1_ROW_36                                              (0x00000488U)
#define CSL_TOP_CTRL_EFUSE1_ROW_37                                              (0x0000048CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_38                                              (0x00000490U)
#define CSL_TOP_CTRL_EFUSE1_ROW_39                                              (0x00000494U)
#define CSL_TOP_CTRL_EFUSE1_ROW_40                                              (0x00000498U)
#define CSL_TOP_CTRL_EFUSE1_ROW_41                                              (0x0000049CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_42                                              (0x000004A0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_43                                              (0x000004A4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_44                                              (0x000004A8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_45                                              (0x000004ACU)
#define CSL_TOP_CTRL_EFUSE1_ROW_46                                              (0x000004B0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_47                                              (0x000004B4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_48                                              (0x000004B8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_49                                              (0x000004BCU)
#define CSL_TOP_CTRL_EFUSE1_ROW_50                                              (0x000004C0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_51                                              (0x000004C4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_52                                              (0x000004C8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_53                                              (0x000004CCU)
#define CSL_TOP_CTRL_EFUSE1_ROW_54                                              (0x000004D0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_55                                              (0x000004D4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56                                              (0x000004D8U)
#define CSL_TOP_CTRL_MAC_ID0                                                    (0x00000500U)
#define CSL_TOP_CTRL_MAC_ID1                                                    (0x00000504U)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C                                             (0x00000510U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0                                       (0x00000514U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1                                       (0x00000518U)
#define CSL_TOP_CTRL_TRIM_TEMP_150C                                             (0x0000051CU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0                                       (0x00000520U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1                                       (0x00000524U)
#define CSL_TOP_CTRL_TRIM_TEMP_30C                                              (0x00000528U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0                                        (0x0000052CU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1                                        (0x00000530U)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE                                         (0x00000534U)
#define CSL_TOP_CTRL_TSHUT_HOT                                                  (0x00000538U)
#define CSL_TOP_CTRL_TSHUT_COLD                                                 (0x0000053CU)
#define CSL_TOP_CTRL_JTAG_ID                                                    (0x00000540U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN                  (0x00000800U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL                                  (0x00000804U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM                                   (0x00000808U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM                                   (0x0000080CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM                                   (0x00000810U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL                                (0x00000830U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0                                    (0x00000834U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1                                    (0x00000838U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2                                    (0x0000083CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL                             (0x00000880U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM                                  (0x00000884U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL                             (0x00000888U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM                                  (0x0000088CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL                             (0x00000890U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM                                  (0x00000894U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL                             (0x00000898U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM                                  (0x0000089CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL                             (0x000008A0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM                                  (0x000008A4U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL                             (0x000008A8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM                                  (0x000008ACU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL                             (0x000008B0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM                                  (0x000008B4U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL                             (0x000008B8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM                                  (0x000008BCU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL                             (0x000008C0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM                                  (0x000008C4U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL                                (0x00000930U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0                                    (0x00000934U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM                                    (0x0000093CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG                                     (0x00000940U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM                                (0x00000944U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG                                 (0x00000948U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG                                     (0x00000960U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM                              (0x00000964U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM                                    (0x00000968U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM                                     (0x0000096CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL                    (0x00000970U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0                        (0x00000974U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1                        (0x00000978U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE                                      (0x0000097CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL                            (0x00000980U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM                                 (0x00000984U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM                                    (0x00000988U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM                                  (0x0000098CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG                                   (0x00000990U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX                                    (0x00000994U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0                                              (0x00000A04U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1                                              (0x00000A08U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2                                              (0x00000A0CU)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL                                    		(0x00000B00U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL                                   		(0x00000B10U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS                                 		(0x00000B14U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL                                        	(0x00000B18U)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN                                              (0x00000B1CU)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN                                              (0x00000B20U)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN                                              (0x00000B24U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL                                              (0x00000B40U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL                                              (0x00000B44U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL                                              (0x00000B48U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL                                          (0x00000B60U)
#define CSL_TOP_CTRL_VMON_UV                                                    (0x00000C00U)
#define CSL_TOP_CTRL_VMON_OV                                                    (0x00000C04U)
#define CSL_TOP_CTRL_VMON_CONTROLLER                                            (0x00000C08U)
#define CSL_TOP_CTRL_VMON_CTRL                                                  (0x00000C0CU)
#define CSL_TOP_CTRL_VMON_STAT                                                  (0x00000C10U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H                                      (0x00000C14U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L                                      (0x00000C18U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL                                           (0x00000C1CU)
#define CSL_TOP_CTRL_PMU_COARSE_STAT                                            (0x00000C20U)
#define CSL_TOP_CTRL_PMU_CTRL_EXTREF                                            (0x00000C24U)
#define CSL_TOP_CTRL_TSENSE_CFG                                                 (0x00000D00U)
#define CSL_TOP_CTRL_TSENSE_STATUS                                              (0x00000D04U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW                                          (0x00000D08U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT                                              (0x00000D10U)
#define CSL_TOP_CTRL_TSENSE0_ALERT                                              (0x00000D14U)
#define CSL_TOP_CTRL_TSENSE0_CNTL                                               (0x00000D18U)
#define CSL_TOP_CTRL_TSENSE0_RESULT                                             (0x00000D1CU)
#define CSL_TOP_CTRL_TSENSE0_DATA0                                              (0x00000D20U)
#define CSL_TOP_CTRL_TSENSE0_DATA1                                              (0x00000D24U)
#define CSL_TOP_CTRL_TSENSE0_DATA2                                              (0x00000D28U)
#define CSL_TOP_CTRL_TSENSE0_DATA3                                              (0x00000D2CU)
#define CSL_TOP_CTRL_TSENSE0_ACCU                                               (0x00000D30U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT                                              (0x00000D40U)
#define CSL_TOP_CTRL_TSENSE1_ALERT                                              (0x00000D44U)
#define CSL_TOP_CTRL_TSENSE1_CNTL                                               (0x00000D48U)
#define CSL_TOP_CTRL_TSENSE1_RESULT                                             (0x00000D4CU)
#define CSL_TOP_CTRL_TSENSE1_DATA0                                              (0x00000D50U)
#define CSL_TOP_CTRL_TSENSE1_DATA1                                              (0x00000D54U)
#define CSL_TOP_CTRL_TSENSE1_DATA2                                              (0x00000D58U)
#define CSL_TOP_CTRL_TSENSE1_DATA3                                              (0x00000D5CU)
#define CSL_TOP_CTRL_TSENSE1_ACCU                                               (0x00000D60U)
#define CSL_TOP_CTRL_TSENSE2_RESULT                                             (0x00000D7CU)
#define CSL_TOP_CTRL_TSENSE3_RESULT                                             (0x00000DACU)
#define CSL_TOP_CTRL_ICSSM1_GPIO_OUT_CTRL                                       (0x00000DC0U)
#define CSL_TOP_CTRL_MASK_ANA_ISO                                               (0x00000DC4U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL                                       (0x00000DC8U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS                                   (0x00000E00U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC                              (0x00000E04U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC                              (0x00000E08U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY                                      (0x00000E0CU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES                                        (0x00000E10U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE                                (0x00000E14U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL                                               (0x00000E18U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL                                              (0x00000E1CU)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL                                              (0x00000E20U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL                                              (0x00000E24U)
#define CSL_TOP_CTRL_DFT_RAMP_DACL                                              (0x00000E30U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL                                            (0x00000E34U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL                                  (0x00000E38U)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL                                        (0x00000E3CU)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL                                         (0x00000E40U)
#define CSL_TOP_CTRL_DFT_CTRL_1                                                 (0x00000E44U)
#define CSL_TOP_CTRL_DFT_CTRL_2                                                 (0x00000E48U)
#define CSL_TOP_CTRL_DFT_CTRL_3                                                 (0x00000E4CU)
#define CSL_TOP_CTRL_DFT_CTRL_4                                                 (0x00000E50U)
#define CSL_TOP_CTRL_DFT_CTRL_5                                                 (0x00000E54U)
#define CSL_TOP_CTRL_DFT_ADC0_CTRL                                              (0x00000E60U)
#define CSL_TOP_CTRL_DFT_ADC1_CTRL                                              (0x00000E64U)
#define CSL_TOP_CTRL_DFT_ADC2_CTRL                                              (0x00000E68U)
#define CSL_TOP_CTRL_DFT_SDADC_CLKBUF_CTRL                                      (0x00000E6CU)
#define CSL_TOP_CTRL_DFT_SDADC_RESOLVEREF_SEL                                   (0x00000E70U)
#define CSL_TOP_CTRL_DFT_SDADC                                                  (0x00000E74U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0                                             (0x00000F04U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1                                             (0x00000F08U)
#define CSL_TOP_CTRL_HW_SPARE_RW0                                               (0x00000F40U)
#define CSL_TOP_CTRL_HW_SPARE_RW1                                               (0x00000F44U)
#define CSL_TOP_CTRL_HW_SPARE_RW2                                               (0x00000F48U)
#define CSL_TOP_CTRL_HW_SPARE_RW3                                               (0x00000F4CU)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW0                                          (0x00000F50U)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW1                                          (0x00000F54U)
#define CSL_TOP_CTRL_HW_SPARE_RO0                                               (0x00000F80U)
#define CSL_TOP_CTRL_HW_SPARE_RO1                                               (0x00000F84U)
#define CSL_TOP_CTRL_HW_SPARE_RO2                                               (0x00000F88U)
#define CSL_TOP_CTRL_HW_SPARE_RO3                                               (0x00000F8CU)
#define CSL_TOP_CTRL_HW_SPARE_WPH                                               (0x00000FC0U)
#define CSL_TOP_CTRL_HW_SPARE_REC                                               (0x00000FC4U)
#define CSL_TOP_CTRL_HW_SPARE_REC0                                              (0x00000FC8U)
#define CSL_TOP_CTRL_HW_SPARE_REC1                                              (0x00000FCCU)
#define CSL_TOP_CTRL_EFUSE_VPP_EN                                               (0x00000FD0U)
#define CSL_TOP_CTRL_LOCK0_KICK0                                                (0x00001008U)
#define CSL_TOP_CTRL_LOCK0_KICK1                                                (0x0000100CU)
#define CSL_TOP_CTRL_INTR_RAW_STATUS                                            (0x00001010U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR                                  (0x00001014U)
#define CSL_TOP_CTRL_INTR_ENABLE                                                (0x00001018U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR                                          (0x0000101CU)
#define CSL_TOP_CTRL_EOI                                                        (0x00001020U)
#define CSL_TOP_CTRL_FAULT_ADDRESS                                              (0x00001024U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS                                          (0x00001028U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS                                          (0x0000102CU)
#define CSL_TOP_CTRL_FAULT_CLEAR                                                (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* PID */
#define CSL_TOP_CTRL_PID_PID_CUSTOM_MASK                                        (0x000000C0U)
#define CSL_TOP_CTRL_PID_PID_CUSTOM_SHIFT                                       (0x00000006U)
#define CSL_TOP_CTRL_PID_PID_CUSTOM_RESETVAL                                    (0x00000000U)
#define CSL_TOP_CTRL_PID_PID_CUSTOM_MAX                                         (0x00000003U)


#define CSL_TOP_CTRL_PID_PID_MAJOR_MASK                                         (0x00000700U)
#define CSL_TOP_CTRL_PID_PID_MAJOR_SHIFT                                        (0x00000008U)
#define CSL_TOP_CTRL_PID_PID_MAJOR_RESETVAL                                     (0x00000002U)
#define CSL_TOP_CTRL_PID_PID_MAJOR_MAX                                          (0x00000007U)


#define CSL_TOP_CTRL_PID_PID_MINOR_MASK                                         (0x0000003FU)
#define CSL_TOP_CTRL_PID_PID_MINOR_SHIFT                                        (0x00000000U)
#define CSL_TOP_CTRL_PID_PID_MINOR_RESETVAL                                     (0x00000015U)
#define CSL_TOP_CTRL_PID_PID_MINOR_MAX                                          (0x0000003FU)


#define CSL_TOP_CTRL_PID_PID_MISC_MASK                                          (0x0000F800U)
#define CSL_TOP_CTRL_PID_PID_MISC_SHIFT                                         (0x0000000BU)
#define CSL_TOP_CTRL_PID_PID_MISC_RESETVAL                                      (0x00000000U)
#define CSL_TOP_CTRL_PID_PID_MISC_MAX                                           (0x0000001FU)


#define CSL_TOP_CTRL_PID_PID_MSB16_MASK                                         (0xFFFF0000U)
#define CSL_TOP_CTRL_PID_PID_MSB16_SHIFT                                        (0x00000010U)
#define CSL_TOP_CTRL_PID_PID_MSB16_RESETVAL                                     (0x00006180U)
#define CSL_TOP_CTRL_PID_PID_MSB16_MAX                                          (0x0000FFFFU)



/* EFUSE_DIEID0 */
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_SHIFT                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_RESETVAL                     (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_MAX                          (0xFFFFFFFFU)



/* EFUSE_DIEID1 */
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_SHIFT                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_RESETVAL                     (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_MAX                          (0xFFFFFFFFU)



/* EFUSE_DIEID2 */
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_SHIFT                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_RESETVAL                     (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_MAX                          (0xFFFFFFFFU)



/* EFUSE_DIEID3 */
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_SHIFT                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_RESETVAL                     (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_MAX                          (0xFFFFFFFFU)



/* EFUSE_UID0 */
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_MAX                              (0xFFFFFFFFU)



/* EFUSE_UID1 */
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_MAX                              (0xFFFFFFFFU)



/* EFUSE_UID2 */
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_MAX                              (0xFFFFFFFFU)



/* EFUSE_UID3 */
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_MASK                             (0x00FFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_MAX                              (0x00FFFFFFU)



/* EFUSE_DEVICE_TYPE */
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_MASK               (0x0000FFFFU)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_MAX                (0x0000FFFFU)



/* EFUSE_FROM0_CHECKSUM */
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_MASK         (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_SHIFT        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_MAX          (0xFFFFFFFFU)



/* EFUSE_JTAG_USERCODE_ID */
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_MASK     (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_SHIFT    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_MAX      (0xFFFFFFFFU)



/* EFUSE0_ROW_61 */
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_MAX              (0x03FFFFFFU)



/* EFUSE0_ROW_62 */
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_MAX              (0x03FFFFFFU)



/* EFUSE0_ROW_63 */
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_5 */
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_MASK                (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_MAX                 (0x03FFFFFFU)



/* EFUSE1_ROW_6 */
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_MASK                (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_MAX                 (0x03FFFFFFU)



/* EFUSE1_ROW_7 */
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_MASK                (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_MAX                 (0x03FFFFFFU)



/* EFUSE1_ROW_8 */
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_MASK                (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_MAX                 (0x03FFFFFFU)



/* EFUSE1_ROW_9 */
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_MASK                (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_MAX                 (0x03FFFFFFU)



/* EFUSE1_ROW_10 */
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_11 */
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_12 */
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_MASK                 (0x01800000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_SHIFT                (0x00000017U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_MAX                  (0x00000003U)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_FEATURE_DISABLE_MASK           (0x02000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_FEATURE_DISABLE_SHIFT          (0x00000019U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_FEATURE_DISABLE_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_FEATURE_DISABLE_MAX            (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS0_HW_DIS_MASK          (0x00007F80U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS0_HW_DIS_SHIFT         (0x00000007U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS0_HW_DIS_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS0_HW_DIS_MAX           (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS1_HW_DIS_MASK          (0x007F8000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS1_HW_DIS_SHIFT         (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS1_HW_DIS_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_PRU_ICSS1_HW_DIS_MAX           (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_SIZE_MASK                   (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_SIZE_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_SIZE_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_SIZE_MAX                    (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_MASK   (0x00000020U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_SHIFT  (0x00000005U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_MAX    (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_MASK     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_SHIFT    (0x00000004U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_MAX      (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_MASK                 (0x00000040U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_SHIFT                (0x00000006U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_MAX                  (0x00000001U)



/* EFUSE1_ROW_13 */
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_MASK               (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_SHIFT              (0x00000002U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_MAX                (0x00000007U)


#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HRPWM_DISABLE_MASK             (0x00000080U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HRPWM_DISABLE_SHIFT            (0x00000007U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HRPWM_DISABLE_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HRPWM_DISABLE_MAX              (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_MASK       (0x00000040U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_SHIFT      (0x00000006U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_MAX        (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_MASK              (0x00000002U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_SHIFT             (0x00000001U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_MAX               (0x00000001U)



/* EFUSE1_ROW_14 */
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_15 */
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_16 */
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_17 */
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_18 */
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_19 */
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_20 */
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_21 */
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_22 */
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_23 */
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_MASK           (0x0000001FU)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_SHIFT          (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_MAX            (0x0000001FU)


#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_MASK     (0x00008000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_SHIFT    (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_MAX      (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_MASK            (0x000003E0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_SHIFT           (0x00000005U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_MAX             (0x0000001FU)


#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_VALID_MASK      (0x00010000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_VALID_SHIFT     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_VALID_RESETVAL  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_ETH_ADPLL_TRIM_VALID_MAX       (0x00000001U)


#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_MASK            (0x00007C00U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_SHIFT           (0x0000000AU)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_MAX             (0x0000001FU)


#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_MASK      (0x00020000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_SHIFT     (0x00000011U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_RESETVAL  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_MAX       (0x00000001U)



/* EFUSE1_ROW_24 */
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_25 */
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_26 */
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_27 */
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_28 */
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_29 */
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_30 */
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_31 */
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_32 */
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_33 */
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_34 */
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_35 */
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_36 */
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_37 */
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_38 */
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_39 */
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_40 */
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_41 */
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_42 */
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_43 */
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_44 */
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_45 */
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_46 */
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_47 */
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_48 */
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_49 */
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_50 */
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_51 */
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_52 */
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_53 */
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_54 */
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_MASK             (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_MAX              (0x03FFFFFFU)



/* EFUSE1_ROW_55 */
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_BOOTROM_CFG_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_BOOTROM_CFG_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_BOOTROM_CFG_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_BOOTROM_CFG_MAX                (0x03FFFFFFU)



/* EFUSE1_ROW_56 */
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_USB2_DISABLE_MASK              (0x00000001U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_USB2_DISABLE_SHIFT             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_USB2_DISABLE_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_USB2_DISABLE_MAX               (0x00000001U)



/* MAC_ID0 */
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_MAX                               (0xFFFFFFFFU)



/* MAC_ID1 */
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_MASK                              (0x0000FFFFU)
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_MAX                               (0x0000FFFFU)



/* TRIM_TEMP_M40C */
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_MASK                    (0x000007FFU)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_SHIFT                   (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_MAX                     (0x000007FFU)



/* TRIM_TEMPSENSE_M40C0 */
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_MASK        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_MAX         (0xFFFFFFFFU)



/* TRIM_TEMPSENSE_M40C1 */
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_MASK        (0x000001FFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_MAX         (0x000001FFU)



/* TRIM_TEMP_150C */
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_MASK                    (0x000007FFU)
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_SHIFT                   (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_MAX                     (0x000007FFU)



/* TRIM_TEMPSENSE_150C0 */
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_MASK        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_MAX         (0xFFFFFFFFU)



/* TRIM_TEMPSENSE_150C1 */
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_MASK        (0x000001FFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_MAX         (0x000001FFU)



/* TRIM_TEMP_30C */
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_MASK                      (0x000007FFU)
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_MAX                       (0x000007FFU)



/* TRIM_TEMPSENSE_30C0 */
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_MASK          (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_SHIFT         (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_MAX           (0xFFFFFFFFU)



/* TRIM_TEMPSENSE_30C1 */
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_MASK          (0x000001FFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_SHIFT         (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_MAX           (0x000001FFU)



/* N_FACTOR_TEMPSENSE */
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_MASK             (0x00003FFFU)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_MAX              (0x00003FFFU)



/* TSHUT_HOT */
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_MASK                               (0x000000FFU)
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_SHIFT                              (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_RESETVAL                           (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_MAX                                (0x000000FFU)



/* TSHUT_COLD */
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_MASK                             (0x000000FFU)
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_MAX                              (0x000000FFU)



/* JTAG_ID */
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_MASK                                    (0xFFFFFFFFU)
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_SHIFT                                   (0x00000000U)
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_RESETVAL                                (0x00000000U)
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_MAX                                     (0xFFFFFFFFU)



/* EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_MASK (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_MAX (0x00000001U)



/* EFUSE_OVERRIDE_MARGINCTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_MASK (0x30000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_SHIFT (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_MAX (0x00000003U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_OVERRIDE_MASK (0x07000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_OVERRIDE_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BRG_MARGIN_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_MASK (0x00300000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_MAX (0x00000003U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_OVERRIDE_MASK (0x00070000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_OVERRIDE_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_BYG_MARGIN_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_MASK (0x00000030U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_MAX (0x00000003U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GLG_MARGIN_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_MASK (0x0000F000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_OVERRIDE_MASK (0x00000700U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_OVERRIDE_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MARGINCTRL_EFUSE_OVERRIDE_MARGINCTRL_GWG_MARGIN_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_ADC0_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_MASK (0x00FFFFF8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_MAX (0x001FFFFFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_ADC1_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_MASK (0x00FFFFF8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_MAX (0x001FFFFFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_ADC2_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_MASK (0x00FFFFF8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_MAX (0x001FFFFFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_ADC_CFG_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_MASK (0x00000038U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_MAX (0x00000007U)



/* EFUSE_OVERRIDE_ADC_CFG0 */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_RESETVAL (0x514554C9U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_MAX (0xFFFFFFFFU)



/* EFUSE_OVERRIDE_ADC_CFG1 */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_RESETVAL (0x21B32908U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_MAX (0xFFFFFFFFU)



/* EFUSE_OVERRIDE_ADC_CFG2 */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_85_64_MASK (0x003FFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_85_64_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_85_64_RESETVAL (0x0020000AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_85_64_MAX (0x003FFFFFU)



/* EFUSE_OVERRIDE_CSSA0_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA0_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_EFUSE_OVERRIDE_CSSA0_TRIM_CSSA0_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_EFUSE_OVERRIDE_CSSA0_TRIM_CSSA0_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_EFUSE_OVERRIDE_CSSA0_TRIM_CSSA0_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA0_TRIM_EFUSE_OVERRIDE_CSSA0_TRIM_CSSA0_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA1_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA1_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_EFUSE_OVERRIDE_CSSA1_TRIM_CSSA1_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_EFUSE_OVERRIDE_CSSA1_TRIM_CSSA1_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_EFUSE_OVERRIDE_CSSA1_TRIM_CSSA1_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA1_TRIM_EFUSE_OVERRIDE_CSSA1_TRIM_CSSA1_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA2_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA2_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_EFUSE_OVERRIDE_CSSA2_TRIM_CSSA2_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_EFUSE_OVERRIDE_CSSA2_TRIM_CSSA2_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_EFUSE_OVERRIDE_CSSA2_TRIM_CSSA2_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA2_TRIM_EFUSE_OVERRIDE_CSSA2_TRIM_CSSA2_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA3_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA3_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_EFUSE_OVERRIDE_CSSA3_TRIM_CSSA3_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_EFUSE_OVERRIDE_CSSA3_TRIM_CSSA3_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_EFUSE_OVERRIDE_CSSA3_TRIM_CSSA3_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA3_TRIM_EFUSE_OVERRIDE_CSSA3_TRIM_CSSA3_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA4_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA4_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_EFUSE_OVERRIDE_CSSA4_TRIM_CSSA4_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_EFUSE_OVERRIDE_CSSA4_TRIM_CSSA4_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_EFUSE_OVERRIDE_CSSA4_TRIM_CSSA4_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA4_TRIM_EFUSE_OVERRIDE_CSSA4_TRIM_CSSA4_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA5_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA5_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_EFUSE_OVERRIDE_CSSA5_TRIM_CSSA5_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_EFUSE_OVERRIDE_CSSA5_TRIM_CSSA5_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_EFUSE_OVERRIDE_CSSA5_TRIM_CSSA5_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA5_TRIM_EFUSE_OVERRIDE_CSSA5_TRIM_CSSA5_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA6_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA6_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_EFUSE_OVERRIDE_CSSA6_TRIM_CSSA6_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_EFUSE_OVERRIDE_CSSA6_TRIM_CSSA6_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_EFUSE_OVERRIDE_CSSA6_TRIM_CSSA6_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA6_TRIM_EFUSE_OVERRIDE_CSSA6_TRIM_CSSA6_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA7_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA7_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_EFUSE_OVERRIDE_CSSA7_TRIM_CSSA7_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_EFUSE_OVERRIDE_CSSA7_TRIM_CSSA7_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_EFUSE_OVERRIDE_CSSA7_TRIM_CSSA7_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA7_TRIM_EFUSE_OVERRIDE_CSSA7_TRIM_CSSA7_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSSA8_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSSA8_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_EFUSE_OVERRIDE_CSSA8_TRIM_CSSA8_TRIM_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_EFUSE_OVERRIDE_CSSA8_TRIM_CSSA8_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_EFUSE_OVERRIDE_CSSA8_TRIM_CSSA8_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSSA8_TRIM_EFUSE_OVERRIDE_CSSA8_TRIM_CSSA8_TRIM_MAX (0x000000FFU)



/* EFUSE_OVERRIDE_CSS_CFG_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_CFG_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_CSS_CFG0 */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_16_0_MASK (0x0001FFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_16_0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_16_0_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_16_0_MAX (0x0001FFFFU)



/* EFUSE_OVERRIDE_DAC_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_MASK (0x1FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_MAX (0x00001FFFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_DAC_CFG */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_MASK (0x01000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_MASK (0x00030000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_MAX (0x00000003U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_REFBUF0_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_REF_TRIM_MASK (0x0000FF00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_REF_TRIM_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_REF_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_REF_TRIM_MAX (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_OV_TRIM_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_OV_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_OV_TRIM_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_OV_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_UV_TRIM_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_UV_TRIM_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_UV_TRIM_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0A_UV_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_OV_TRIM_MASK (0x0F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_OV_TRIM_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_OV_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_OV_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_UV_TRIM_MASK (0xF0000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_UV_TRIM_SHIFT (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_UV_TRIM_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_EFUSE_OVERRIDE_REFBUF0_TRIM_ROK0B_UV_TRIM_MAX (0x0000000FU)



/* EFUSE_OVERRIDE_REFBUF0_CFG */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_MASK (0x00007F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_RESETVAL (0x0000002AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_MAX (0x0000007FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_MASK (0x0F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_MAX (0x0000000FU)



/* EFUSE_OVERRIDE_PMU_CFG */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_MASK (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_MAX  (0x0000001FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_MASK (0x7F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_MAX (0x0000007FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_PMU_SPARE_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_MASK (0x03FF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_MAX (0x000003FFU)



/* EFUSE_OVERRIDE_LDO_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_MASK (0x3F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_MAX (0x0000003FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_BG_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_MASK (0x3F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_RESETVAL (0x0000001AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_MAX (0x0000003FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_MASK (0x0000FF00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_RESETVAL (0x00000028U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_MAX (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0 */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_MASK (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_MASK (0x000000F0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_MASK (0x00000F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_MASK (0x0000F000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_MASK (0x003F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_RESETVAL (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_MAX (0x0000003FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_MASK (0x0FC00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_SHIFT (0x00000016U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_RESETVAL (0x0000000BU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_MAX (0x0000003FU)



/* EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1 */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_HIGH_TRIM_MASK (0x0000003FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_HIGH_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_HIGH_TRIM_RESETVAL (0x00000015U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_HIGH_TRIM_MAX (0x0000003FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_LOW_TRIM_MASK (0x00000FC0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_LOW_TRIM_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_LOW_TRIM_RESETVAL (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C3_HOPP_LOW_TRIM_MAX (0x0000003FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_HIGH_TRIM_MASK (0x0000F000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_HIGH_TRIM_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_HIGH_TRIM_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_HIGH_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_LOW_TRIM_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_LOW_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_LOW_TRIM_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C5_LOW_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_HOPP_LOW_TRIM_MASK (0x0F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_HOPP_LOW_TRIM_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_HOPP_LOW_TRIM_RESETVAL (0x00000005U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_HOPP_LOW_TRIM_MAX (0x0000000FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_RESETVAL (0x0000000AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_MAX (0x0000000FU)



/* EFUSE_SAFETYMON_SPARE */
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_MASK  (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_MAX   (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_MASK       (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_SHIFT      (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_MAX        (0x0000000FU)



/* EFUSE_OVERRIDE_TSENSE_TRIM_CTRL */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_TSENSE_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPC_MASK (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPC_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPC_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPC_MAX (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPV_MASK (0x0000FF00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPV_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPV_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRBGAPV_MAX (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRTEMPS_MASK (0x00FF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRTEMPS_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRTEMPS_RESETVAL (0x00000005U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_DTRTEMPS_MAX (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_TSHUT_TRIM_MASK (0x3F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_TSHUT_TRIM_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_TSHUT_TRIM_RESETVAL (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_TSHUT_TRIM_MAX (0x0000003FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_VPTAT_RTRIM_MASK (0xC0000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_VPTAT_RTRIM_SHIFT (0x0000001EU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_VPTAT_RTRIM_RESETVAL (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_EFUSE_OVERRIDE_TSENSE_TRIM_VPTAT_RTRIM_MAX (0x00000003U)



/* EFUSE_OVERRIDE_PLL_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_MASK (0x000000F8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_MAX (0x0000001FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_MASK (0x00070000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_VAL_MASK (0x00F80000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_VAL_SHIFT (0x00000013U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_VAL_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_ETH_NWELLTRIM_OVERRIDE_VAL_MAX (0x0000001FU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_MASK (0x00000700U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_MASK (0x0000F800U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_MAX (0x0000001FU)



/* EFUSE_OVERRIDE_RCOSC_TRIM */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_MASK (0x00FF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_MAX (0x000000FFU)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_RCOSC_CFG */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_FREQ_SEL_MASK (0x00030000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_FREQ_SEL_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_FREQ_SEL_RESETVAL (0x00000002U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_FREQ_SEL_MAX (0x00000003U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_CFG_EFUSE_OVERRIDE_RCOSC_CFG_OVERRIDE_MAX (0x00000007U)



/* EFUSE_OVERRIDE_HOPP_MUX */
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_OVERRIDE_MASK (0x0000000EU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_OVERRIDE_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_OVERRIDE_MAX (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_SELECT_VALUE_MASK (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_SELECT_VALUE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_SELECT_VALUE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HOPP_MUX_EFUSE_OVERRIDE_HOPP_MUX_SELECT_VALUE_MAX (0x00000001U)



/* EFUSE_SPARE_0 */
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_OVERRIDE_MASK                  (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_OVERRIDE_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_OVERRIDE_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_OVERRIDE_MAX                   (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_VAL_MASK                       (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_VAL_SHIFT                      (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_VAL_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_0_EFUSE_SPARE_0_VAL_MAX                        (0x0000001FU)



/* EFUSE_SPARE_1 */
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_MASK                  (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_MAX                   (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_MASK                       (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_SHIFT                      (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_MAX                        (0x0000001FU)



/* EFUSE_SPARE_2 */
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_MASK                  (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_MAX                   (0x00000007U)


#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_MASK                       (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_SHIFT                      (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_MAX                        (0x0000001FU)



/* ADC_REFBUF0_CTRL */
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_MASK       (0x00000007U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_MAX        (0x00000007U)



/* ADC_REF_COMP_CTRL_CTRL */
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC01_REFOK_EN_MASK (0x00000007U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC01_REFOK_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC01_REFOK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC01_REFOK_EN_MAX (0x00000007U)


#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC2_REFOK_EN_MASK (0x00000070U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC2_REFOK_EN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC2_REFOK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC2_REFOK_EN_MAX (0x00000007U)



/* ADC_REF_GOOD_STATUS_STATUS */
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_OV_GOOD_MASK (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_OV_GOOD_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_OV_GOOD_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_OV_GOOD_MAX (0x00000001U)


#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_UV_GOOD_MASK (0x00000002U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_UV_GOOD_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_UV_GOOD_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC01_REF_UV_GOOD_MAX (0x00000001U)


#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_OV_GOOD_MASK (0x00000004U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_OV_GOOD_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_OV_GOOD_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_OV_GOOD_MAX (0x00000001U)


#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_UV_GOOD_MASK (0x00000008U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_UV_GOOD_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_UV_GOOD_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC2_REF_UV_GOOD_MAX (0x00000001U)



/* ADC_REF_RNG_CTRL */
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_MODE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_MODE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_MODE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_MODE_MAX                  (0x00000007U)


#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_MASK          (0x00000700U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_SHIFT         (0x00000008U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_MAX           (0x00000007U)



/* ADC0_OSD_CHEN */
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_MASK                 (0x0000007FU)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_MAX                  (0x0000007FU)



/* ADC1_OSD_CHEN */
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_MASK                 (0x0000007FU)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_MAX                  (0x0000007FU)



/* ADC2_OSD_CHEN */
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_MASK                 (0x0000007FU)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_MAX                  (0x0000007FU)



/* ADC0_OSD_CTRL */
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_MASK                  (0x00000007U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_MAX                   (0x00000007U)



/* ADC1_OSD_CTRL */
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_MASK                  (0x00000007U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_MAX                   (0x00000007U)



/* ADC2_OSD_CTRL */
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_MASK                  (0x00000007U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_MAX                   (0x00000007U)



/* ADC_LOOPBACK_CTRL */
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_MASK   (0x00000001U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_SHIFT  (0x00000000U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_MAX    (0x00000001U)



/* VMON_UV */
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP1_UV_EN_MASK                            (0x00000007U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP1_UV_EN_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP1_UV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP1_UV_EN_MAX                             (0x00000007U)


#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP2_UV_EN_MASK                            (0x00000070U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP2_UV_EN_SHIFT                           (0x00000004U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP2_UV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP2_UV_EN_MAX                             (0x00000007U)


#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP3_UV_EN_MASK                            (0x00000700U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP3_UV_EN_SHIFT                           (0x00000008U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP3_UV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP3_UV_EN_MAX                             (0x00000007U)


#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP5_UV_EN_MASK                            (0x00007000U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP5_UV_EN_SHIFT                           (0x0000000CU)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP5_UV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_UV_VMON_UV_CMP5_UV_EN_MAX                             (0x00000007U)



/* VMON_OV */
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP1_OV_EN_MASK                            (0x00000007U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP1_OV_EN_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP1_OV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP1_OV_EN_MAX                             (0x00000007U)


#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP2_OV_EN_MASK                            (0x00000070U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP2_OV_EN_SHIFT                           (0x00000004U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP2_OV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP2_OV_EN_MAX                             (0x00000007U)


#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP3_OV_EN_MASK                            (0x00000700U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP3_OV_EN_SHIFT                           (0x00000008U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP3_OV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP3_OV_EN_MAX                             (0x00000007U)


#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP5_OV_EN_MASK                            (0x00007000U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP5_OV_EN_SHIFT                           (0x0000000CU)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP5_OV_EN_RESETVAL                        (0x00000007U)
#define CSL_TOP_CTRL_VMON_OV_VMON_OV_CMP5_OV_EN_MAX                             (0x00000007U)



/* VMON_CONTROLLER */
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_EN_MASK                    (0x00000007U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_EN_SHIFT                   (0x00000000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_EN_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_EN_MAX                     (0x00000007U)


#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_ENAZ_MASK                  (0x000E0000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_ENAZ_SHIFT                 (0x00000011U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_ENAZ_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_ENAZ_MAX                   (0x00000007U)


#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_FILTER_WINDOW_MASK         (0x07800000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_FILTER_WINDOW_SHIFT        (0x00000017U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_FILTER_WINDOW_RESETVAL     (0x00000005U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_FILTER_WINDOW_MAX          (0x0000000FU)


#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_INIT_TIME_MASK         (0x000001F0U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_INIT_TIME_SHIFT        (0x00000004U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_INIT_TIME_RESETVAL     (0x0000000FU)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_INIT_TIME_MAX          (0x0000001FU)


#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SAMPLES_MASK           (0x0001E000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SAMPLES_SHIFT          (0x0000000DU)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SAMPLES_RESETVAL       (0x00000005U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SAMPLES_MAX            (0x0000000FU)


#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SET_TIME_MASK          (0x00001E00U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SET_TIME_SHIFT         (0x00000009U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SET_TIME_RESETVAL      (0x00000005U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_PSM_SET_TIME_MAX           (0x0000000FU)


#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_SAFETY_SEL_MASK            (0x00700000U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_SAFETY_SEL_SHIFT           (0x00000014U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_SAFETY_SEL_RESETVAL        (0x00000007U)
#define CSL_TOP_CTRL_VMON_CONTROLLER_VMON_CONTROLLER_SAFETY_SEL_MAX             (0x00000007U)



/* VMON_CTRL */
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_AUTOZERO_COMP_REFRESH_DISABLE_MASK     (0x70000000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_AUTOZERO_COMP_REFRESH_DISABLE_SHIFT    (0x0000001CU)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_AUTOZERO_COMP_REFRESH_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_AUTOZERO_COMP_REFRESH_DISABLE_MAX      (0x00000007U)


#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_MASK                           (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_SHIFT                          (0x00000000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_RESETVAL                       (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_MAX                            (0x00000007U)


#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_MASK                           (0x00700000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_SHIFT                          (0x00000014U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_RESETVAL                       (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_MAX                            (0x00000007U)


#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_MASK                           (0x07000000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_SHIFT                          (0x00000018U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_RESETVAL                       (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_MAX                            (0x00000007U)



/* VMON_STAT */
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_MASK                        (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_MASK                        (0x00000002U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_SHIFT                       (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_MASK                        (0x00000004U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_SHIFT                       (0x00000002U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_MASK                        (0x00000008U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_SHIFT                       (0x00000003U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_MASK                        (0x00000010U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_SHIFT                       (0x00000004U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_MASK                        (0x00000020U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_SHIFT                       (0x00000005U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_MASK                        (0x00000040U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_SHIFT                       (0x00000006U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_MASK                        (0x00000080U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_SHIFT                       (0x00000007U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_MASK                        (0x00000100U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_SHIFT                       (0x00000008U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_MASK                        (0x00000200U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_SHIFT                       (0x00000009U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_MAX                         (0x00000001U)


#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_MASK                        (0x00000400U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_SHIFT                       (0x0000000AU)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_MAX                         (0x00000001U)



/* MASK_VMON_ERROR_ESM_H */
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_OV_MASK_MASK (0x00000800U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_OV_MASK_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_OV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_UV_MASK_MASK (0x00001000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_UV_MASK_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC01_REF_UV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_OV_MASK_MASK (0x00002000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_OV_MASK_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_OV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_UV_MASK_MASK (0x00004000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_UV_MASK_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC2_REF_UV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP0_UV_ERR_MASK_MASK (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP0_UV_ERR_MASK_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP0_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP0_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_OV_ERR_MASK_MASK (0x00000002U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_OV_ERR_MASK_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_UV_ERR_NASK_MASK (0x00000004U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_UV_ERR_NASK_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_UV_ERR_NASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP1_UV_ERR_NASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_OV_ERR_MASK_MASK (0x00000008U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_OV_ERR_MASK_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_UV_ERR_MASK_MASK (0x00000010U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_UV_ERR_MASK_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP2_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_OV_ERR_MASK_MASK (0x00000020U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_OV_ERR_MASK_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_UV_ERR_MASK_MASK (0x00000040U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_UV_ERR_MASK_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP3_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_OV_ERR_MASK_MASK (0x00000080U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_OV_ERR_MASK_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_UV_ERR_MASK_MASK (0x00000100U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_UV_ERR_MASK_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP5_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP7_UV_ERR_MASK_MASK (0x00000200U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP7_UV_ERR_MASK_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP7_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP7_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP8_UV_ERR_MASK_MASK (0x00000400U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP8_UV_ERR_MASK_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP8_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_CMP8_UV_ERR_MASK_MAX (0x00000001U)



/* MASK_VMON_ERROR_ESM_L */
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_OV_MASK_MASK (0x00000800U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_OV_MASK_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_OV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_UV_MASK_MASK (0x00001000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_UV_MASK_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC01_REF_UV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_OV_MASK_MASK (0x00002000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_OV_MASK_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_OV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_UV_MASK_MASK (0x00004000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_UV_MASK_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC2_REF_UV_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP0_UV_ERR_MASK_MASK (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP0_UV_ERR_MASK_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP0_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP0_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_OV_ERR_MASK_MASK (0x00000002U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_OV_ERR_MASK_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_UV_ERR_NASK_MASK (0x00000004U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_UV_ERR_NASK_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_UV_ERR_NASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP1_UV_ERR_NASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_OV_ERR_MASK_MASK (0x00000008U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_OV_ERR_MASK_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_UV_ERR_MASK_MASK (0x00000010U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_UV_ERR_MASK_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP2_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_OV_ERR_MASK_MASK (0x00000020U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_OV_ERR_MASK_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_UV_ERR_MASK_MASK (0x00000040U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_UV_ERR_MASK_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP3_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_OV_ERR_MASK_MASK (0x00000080U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_OV_ERR_MASK_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_OV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_OV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_UV_ERR_MASK_MASK (0x00000100U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_UV_ERR_MASK_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP5_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP7_UV_ERR_MASK_MASK (0x00000200U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP7_UV_ERR_MASK_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP7_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP7_UV_ERR_MASK_MAX (0x00000001U)


#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP8_UV_ERR_MASK_MASK (0x00000400U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP8_UV_ERR_MASK_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP8_UV_ERR_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_CMP8_UV_ERR_MASK_MAX (0x00000001U)



/* VMON_FILTER_CTRL */
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_MASK        (0x00000003U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_MAX         (0x00000003U)



/* PMU_COARSE_STAT */
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_MASK                (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_RESETVAL            (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_MAX                 (0x00000001U)


#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_MASK               (0x00000002U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_SHIFT              (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_RESETVAL           (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_MAX                (0x00000001U)


#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_MASK             (0x00000004U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_SHIFT            (0x00000002U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_RESETVAL         (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_MAX              (0x00000001U)


#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_MASK            (0x00000008U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_SHIFT           (0x00000003U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_RESETVAL        (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_MAX             (0x00000001U)



/* PMU_CTRL_EXTREF */
#define CSL_TOP_CTRL_PMU_CTRL_EXTREF_PMU_CTRL_EXTREF_SELECT_MASK                (0x00000003U)
#define CSL_TOP_CTRL_PMU_CTRL_EXTREF_PMU_CTRL_EXTREF_SELECT_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_PMU_CTRL_EXTREF_PMU_CTRL_EXTREF_SELECT_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_PMU_CTRL_EXTREF_PMU_CTRL_EXTREF_SELECT_MAX                 (0x00000003U)



/* TSENSE_CFG */
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_MASK                          (0x00100000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_SHIFT                         (0x00000014U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_RESETVAL                      (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_MAX                           (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_MASK                          (0x01000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_SHIFT                         (0x00000018U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_RESETVAL                      (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_MAX                           (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_MASK                           (0x00003F00U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_SHIFT                          (0x00000008U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_RESETVAL                       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_MAX                            (0x0000003FU)


#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_MASK                          (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_SHIFT                         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_RESETVAL                      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_MAX                           (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_MASK                      (0x000000F0U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_SHIFT                     (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_MAX                       (0x0000000FU)


#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_MASK                     (0x00010000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_SHIFT                    (0x00000010U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_RESETVAL                 (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_MAX                      (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_MASK                         (0x10000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_SHIFT                        (0x0000001CU)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_RESETVAL                     (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_MAX                          (0x00000001U)



/* TSENSE_STATUS */
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_MASK                   (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_SHIFT                  (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_MAX                    (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_MASK                    (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_SHIFT                   (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_MAX                     (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_MASK             (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_MAX              (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_MASK                   (0x00000020U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_SHIFT                  (0x00000005U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_MAX                    (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_MASK                    (0x00000040U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_SHIFT                   (0x00000006U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_MAX                     (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_MASK             (0x00000010U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_SHIFT            (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_MAX              (0x00000001U)



/* TSENSE_STATUS_RAW */
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_MASK           (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_SHIFT          (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_MAX            (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_MASK            (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_SHIFT           (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_MAX             (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_MASK     (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_SHIFT    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_MAX      (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_MASK           (0x00000020U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_SHIFT          (0x00000005U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_MAX            (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_MASK            (0x00000040U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_SHIFT           (0x00000006U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_MAX             (0x00000001U)


#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_MASK     (0x00000010U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_SHIFT    (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_MAX      (0x00000001U)



/* TSENSE0_TSHUT */
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_MASK            (0xE0000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_SHIFT           (0x0000001DU)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_MAX             (0x00000007U)


#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_MASK          (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_SHIFT         (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_MAX           (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_MASK        (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_MAX         (0x000000FFU)



/* TSENSE0_ALERT */
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_MASK         (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_SHIFT        (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_MAX          (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_MASK          (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_SHIFT         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_MAX           (0x000000FFU)



/* TSENSE0_CNTL */
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_MASK                  (0x00000100U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_SHIFT                 (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_MAX                   (0x00000001U)


#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_MASK                  (0x00000001U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_MAX                   (0x00000001U)


#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_MASK                 (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_SHIFT                (0x00000004U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_MAX                  (0x00000001U)


#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_MASK                   (0x00010000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_SHIFT                  (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_MAX                    (0x00000001U)


#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_MASK                    (0x00100000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_SHIFT                   (0x00000014U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_RESETVAL                (0x00000001U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_MAX                     (0x00000001U)


#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_MASK             (0x01000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_SHIFT            (0x00000018U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_MAX              (0x00000001U)



/* TSENSE0_RESULT */
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_MASK                   (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_RESETVAL               (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_MAX                    (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_MASK                    (0x00010000U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_SHIFT                   (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_MAX                     (0x00000001U)



/* TSENSE0_DATA0 */
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_RESETVAL                  (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE0_DATA1 */
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE0_DATA2 */
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE0_DATA3 */
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE0_ACCU */
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_MASK                       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_MAX                        (0xFFFFFFFFU)



/* TSENSE1_TSHUT */
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_MASK            (0xE0000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_SHIFT           (0x0000001DU)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_MAX             (0x00000007U)


#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_MASK          (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_SHIFT         (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_MAX           (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_MASK        (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_MAX         (0x000000FFU)



/* TSENSE1_ALERT */
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_MASK         (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_SHIFT        (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_MAX          (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_MASK          (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_SHIFT         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_MAX           (0x000000FFU)



/* TSENSE1_CNTL */
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_MASK                  (0x00000100U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_SHIFT                 (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_MAX                   (0x00000001U)


#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_MASK                  (0x00000001U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_MAX                   (0x00000001U)


#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_MASK                 (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_SHIFT                (0x00000004U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_MAX                  (0x00000001U)


#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_MASK                   (0x00010000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_SHIFT                  (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_MAX                    (0x00000001U)


#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_MASK                    (0x00100000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_SHIFT                   (0x00000014U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_RESETVAL                (0x00000001U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_MAX                     (0x00000001U)


#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_MASK             (0x01000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_SHIFT            (0x00000018U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_MAX              (0x00000001U)



/* TSENSE1_RESULT */
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_MASK                   (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_RESETVAL               (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_MAX                    (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_MASK                    (0x00010000U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_SHIFT                   (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_MAX                     (0x00000001U)



/* TSENSE1_DATA0 */
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_RESETVAL                  (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE1_DATA1 */
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE1_DATA2 */
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE1_DATA3 */
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_MASK                      (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_MAX                       (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_MASK                       (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_MAX                        (0x00FFFFFFU)



/* TSENSE1_ACCU */
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_MASK                       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_MAX                        (0xFFFFFFFFU)



/* TSENSE2_RESULT */
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_MASK                   (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_MAX                    (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_MASK                    (0x00010000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_SHIFT                   (0x00000010U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_MAX                     (0x00000001U)



/* TSENSE3_RESULT */
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_MASK                   (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_MAX                    (0x000000FFU)


#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_MASK                    (0x00010000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_SHIFT                   (0x00000010U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_MAX                     (0x00000001U)



/* ICSSM1_GPIO_OUT_CTRL */
#define CSL_TOP_CTRL_ICSSM1_GPIO_OUT_CTRL_PRU_ICSS1_GPIO_OUT_CTRL_ICSSM1_SEL_MASK (0x00000007U)
#define CSL_TOP_CTRL_ICSSM1_GPIO_OUT_CTRL_PRU_ICSS1_GPIO_OUT_CTRL_ICSSM1_SEL_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_ICSSM1_GPIO_OUT_CTRL_PRU_ICSS1_GPIO_OUT_CTRL_ICSSM1_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ICSSM1_GPIO_OUT_CTRL_PRU_ICSS1_GPIO_OUT_CTRL_ICSSM1_SEL_MAX (0x00000007U)



/* MASK_ANA_ISO */
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_MASK                        (0x00000007U)
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_MAX                         (0x00000007U)



/* CMPSSA_LOOPBACK_CTRL */
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_MASK (0x00010000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_MASK (0x00020000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_MASK (0x00040000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_MASK (0x00080000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_SHIFT (0x00000013U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_MASK (0x00100000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_MASK (0x00200000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_SHIFT (0x00000015U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_MASK (0x00400000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_SHIFT (0x00000016U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_MASK (0x00800000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_SHIFT (0x00000017U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_MASK (0x01000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_MASK (0x00000002U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_MASK (0x00000004U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_MASK (0x00000008U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_MASK (0x00000010U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_MASK (0x00000020U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_MASK (0x00000040U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_MASK (0x00000080U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_MASK (0x00000100U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_MAX (0x00000001U)



/* DFT_ATB_GLOBALEN_ADC_CSS */
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC0_CSS_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC0_CSS_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC0_CSS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC0_CSS_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC1_CSS_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC1_CSS_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC1_CSS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC1_CSS_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC2_CSS_MASK (0x00000004U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC2_CSS_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC2_CSS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC2_CSS_MAX (0x00000001U)



/* DFT_ATB0_MASTEREN_ADC_CSS_DAC */
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC0_ATB0_MASTEREN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC0_ATB0_MASTEREN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC0_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC1_ATB0_MASTEREN_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC1_ATB0_MASTEREN_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC1_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC2_ATB0_MASTEREN_MASK (0x00000004U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC2_ATB0_MASTEREN_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC2_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC2_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB0_MASTEREN_MASK (0x00000080U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB0_MASTEREN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB0_MASTEREN_MASK (0x00000100U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB0_MASTEREN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB0_MASTEREN_MASK (0x00000200U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB0_MASTEREN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB0_MASTEREN_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB0_MASTEREN_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB0_MASTEREN_MASK (0x00000800U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB0_MASTEREN_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB0_MASTEREN_MASK (0x00001000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB0_MASTEREN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB0_MASTEREN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB0_MASTEREN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB0_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_MASK (0x02000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_MAX (0x00000001U)



/* DFT_ATB1_MASTEREN_ADC_CSS_DAC */
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB1_MASTEREN_MASK (0x00000080U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB1_MASTEREN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A2_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB1_MASTEREN_MASK (0x00000100U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB1_MASTEREN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A3_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB1_MASTEREN_MASK (0x00000200U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB1_MASTEREN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A4_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB1_MASTEREN_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB1_MASTEREN_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A5_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB1_MASTEREN_MASK (0x00000800U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB1_MASTEREN_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A6_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB1_MASTEREN_MASK (0x00001000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB1_MASTEREN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A7_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB1_MASTEREN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB1_MASTEREN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A8_ATB1_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_MASK (0x02000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_MAX (0x00000001U)



/* DFT_PMU_REFSYS_SAFETY */
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_MAX  (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_MASK (0x00000010U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_MASK (0x00000180U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_MAX (0x00000003U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_MASK (0x00000008U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_MAX  (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_MASK (0x00000007U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_MAX (0x00000007U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_MASK (0x00000E00U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_MAX (0x00000007U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_MASK    (0x00004000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_SHIFT   (0x0000000EU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_MAX     (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_MASK   (0x00001000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_SHIFT  (0x0000000CU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_MAX    (0x00000001U)


#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_MASK (0x00008000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_MAX (0x00000001U)



/* DFT_ANA_DTB_ENABLES */
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC0_DTB_MASTEREN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC0_DTB_MASTEREN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC0_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC1_DTB_MASTEREN_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC1_DTB_MASTEREN_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC1_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC2_DTB_MASTEREN_MASK (0x00000004U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC2_DTB_MASTEREN_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC2_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC2_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A2_DTB_MASTEREN_MASK (0x00000080U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A2_DTB_MASTEREN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A2_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A2_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A3_DTB_MASTEREN_MASK (0x00000100U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A3_DTB_MASTEREN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A3_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A3_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A4_DTB_MASTEREN_MASK (0x00000200U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A4_DTB_MASTEREN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A4_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A4_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A5_DTB_MASTEREN_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A5_DTB_MASTEREN_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A5_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A5_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A6_DTB_MASTEREN_MASK (0x00000800U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A6_DTB_MASTEREN_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A6_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A6_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A7_DTB_MASTEREN_MASK (0x00001000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A7_DTB_MASTEREN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A7_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A7_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A8_DTB_MASTEREN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A8_DTB_MASTEREN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A8_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A8_DTB_MASTEREN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_MASK (0x02000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_MAX (0x00000001U)



/* DFT_ADC_CHSEL_OV_CTRL_VALUE */
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_MASK (0x000003FFU)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_MAX (0x000003FFU)


#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_MAX (0x00000001U)



/* DFT_DAC_CTRL */
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_MASK            (0x00000100U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_SHIFT           (0x00000008U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_MAX             (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_MASK             (0x00000080U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_SHIFT            (0x00000007U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_MAX              (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_MASK              (0x00000040U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_SHIFT             (0x00000006U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_MAX               (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_MASK               (0x00000002U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_SHIFT              (0x00000001U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_MAX                (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_MASK            (0x00000020U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_SHIFT           (0x00000005U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_MAX             (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_MASK           (0x00000010U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_SHIFT          (0x00000004U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_MAX            (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_MASK               (0x00000001U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_MAX                (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_MASK               (0x00000004U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_SHIFT              (0x00000002U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_MAX                (0x00000001U)


#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_MASK                     (0x00000008U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_SHIFT                    (0x00000003U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_MAX                      (0x00000001U)



/* DFT_CSS0_CTRL */
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_ATB_SELECT_MASK           (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_ATB_SELECT_SHIFT          (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_ATB_SELECT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_ATB_SELECT_MAX            (0x00000003U)


#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_EN_MASK    (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_EN_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_EN_MAX     (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_VAL_MASK   (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_VAL_SHIFT  (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_COMPOUT_BYPASS_VAL_MAX    (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_MUX_CONFIG_MASK       (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_MUX_CONFIG_SHIFT      (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_MUX_CONFIG_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_MUX_CONFIG_MAX        (0x00000003U)


#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_SELECT_MASK           (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_SELECT_SHIFT          (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_SELECT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_DTB_SELECT_MAX            (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)


#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS0_CTRL_DFT_CSS0_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)



/* DFT_CSS1_CTRL */
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_ATB_SELECT_MASK           (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_ATB_SELECT_SHIFT          (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_ATB_SELECT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_ATB_SELECT_MAX            (0x00000003U)


#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_EN_MASK    (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_EN_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_EN_MAX     (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_VAL_MASK   (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_VAL_SHIFT  (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_COMPOUT_BYPASS_VAL_MAX    (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_MUX_CONFIG_MASK       (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_MUX_CONFIG_SHIFT      (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_MUX_CONFIG_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_MUX_CONFIG_MAX        (0x00000003U)


#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_SELECT_MASK           (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_SELECT_SHIFT          (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_SELECT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_DTB_SELECT_MAX            (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)


#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS1_CTRL_DFT_CSS1_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)



/* DFT_CSS2_CTRL */
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_ATB_SELECT_MASK           (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_ATB_SELECT_SHIFT          (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_ATB_SELECT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_ATB_SELECT_MAX            (0x00000003U)


#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_EN_MASK    (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_EN_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_EN_MAX     (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_VAL_MASK   (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_VAL_SHIFT  (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_COMPOUT_BYPASS_VAL_MAX    (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_MUX_CONFIG_MASK       (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_MUX_CONFIG_SHIFT      (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_MUX_CONFIG_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_MUX_CONFIG_MAX        (0x00000003U)


#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_SELECT_MASK           (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_SELECT_SHIFT          (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_SELECT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_DTB_SELECT_MAX            (0x00000001U)


#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)


#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS2_CTRL_DFT_CSS2_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)



/* DFT_RAMP_DACL */
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_MASK                 (0x000FFFFFU)
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_MAX                  (0x000FFFFFU)



/* DFT_REFBUF_CTRL */
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_MASK         (0x00000004U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_SHIFT        (0x00000002U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_MAX          (0x00000001U)


#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_MASK    (0x00000002U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_SHIFT   (0x00000001U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_MAX     (0x00000001U)


#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_MASK   (0x00000001U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_SHIFT  (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_MAX    (0x00000001U)


#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_MASK   (0x00000060U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_SHIFT  (0x00000005U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_MAX    (0x00000003U)



/* DFT_ODP_ATB_LOOPBACK_CTRL */
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATEST1_TO_ADCCAL_SHORT_SWITCH_MASK (0x00000008U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATEST1_TO_ADCCAL_SHORT_SWITCH_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATEST1_TO_ADCCAL_SHORT_SWITCH_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATEST1_TO_ADCCAL_SHORT_SWITCH_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_BG_REF_CURRENT_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_BG_REF_CURRENT_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_BG_REF_CURRENT_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_BG_REF_CURRENT_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_EN_ATEST0_ON_ATEST1_MASK (0x00000004U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_EN_ATEST0_ON_ATEST1_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_EN_ATEST0_ON_ATEST1_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_EN_ATEST0_ON_ATEST1_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_MAX (0x00000001U)


#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_MASK (0xFFFFFFF0U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_MAX (0x0FFFFFFFU)



/* DFT_SOC_DTB_MUX_SEL */
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_MASK   (0x000000FFU)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_SHIFT  (0x00000000U)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_MAX    (0x000000FFU)



/* DFT_TEMPSENSE_CTRL */
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_MASK     (0x00000001U)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_SHIFT    (0x00000000U)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_MAX      (0x00000001U)



/* DFT_CTRL_1 */
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_MAX                         (0xFFFFFFFFU)



/* DFT_CTRL_2 */
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_MAX                         (0xFFFFFFFFU)



/* DFT_CTRL_3 */
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_MAX                         (0xFFFFFFFFU)



/* DFT_CTRL_4 */
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_MAX                         (0xFFFFFFFFU)



/* DFT_CTRL_5 */

/* DFT_ADC0_CTRL */
#define CSL_TOP_CTRL_DFT_ADC0_CTRL_DFT_ADC0_CTRL_DFT_SEL_MASK                   (0x000FFFFFU)
#define CSL_TOP_CTRL_DFT_ADC0_CTRL_DFT_ADC0_CTRL_DFT_SEL_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC0_CTRL_DFT_ADC0_CTRL_DFT_SEL_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC0_CTRL_DFT_ADC0_CTRL_DFT_SEL_MAX                    (0x000FFFFFU)



/* DFT_ADC1_CTRL */
#define CSL_TOP_CTRL_DFT_ADC1_CTRL_DFT_ADC1_CTRL_DFT_SEL_MASK                   (0x000FFFFFU)
#define CSL_TOP_CTRL_DFT_ADC1_CTRL_DFT_ADC1_CTRL_DFT_SEL_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC1_CTRL_DFT_ADC1_CTRL_DFT_SEL_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC1_CTRL_DFT_ADC1_CTRL_DFT_SEL_MAX                    (0x000FFFFFU)



/* DFT_ADC2_CTRL */
#define CSL_TOP_CTRL_DFT_ADC2_CTRL_DFT_ADC2_CTRL_DFT_SEL_MASK                   (0x000FFFFFU)
#define CSL_TOP_CTRL_DFT_ADC2_CTRL_DFT_ADC2_CTRL_DFT_SEL_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC2_CTRL_DFT_ADC2_CTRL_DFT_SEL_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC2_CTRL_DFT_ADC2_CTRL_DFT_SEL_MAX                    (0x000FFFFFU)



/* DFT_SDADC_CLKBUF_CTRL */

/* DFT_SDADC_RESOLVEREF_SEL */

/* DFT_SDADC */

/* PROBE_BUS_SEL0 */
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_MASK                     (0xFFFFFFFFU)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_MAX                      (0xFFFFFFFFU)



/* PROBE_BUS_SEL1 */
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_MASK                     (0xFFFFFFFFU)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_MAX                      (0xFFFFFFFFU)



/* HW_SPARE_RW0 */
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_RW1 */
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_RW2 */
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_RW3 */
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_PORZ_RW0 */
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_HW_SPARE_PORZ_RW0_MAX  (0xFFFFFFFFU)



/* HW_SPARE_PORZ_RW1 */
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_HW_SPARE_PORZ_RW1_MAX  (0xFFFFFFFFU)



/* HW_SPARE_RO0 */
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_RO1 */
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_RO2 */
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_RO3 */
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_WPH */
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MASK                (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MAX                 (0xFFFFFFFFU)



/* HW_SPARE_REC */
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK               (0x00000001U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK               (0x00000002U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT              (0x00000001U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK              (0x00000400U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT             (0x0000000AU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK              (0x00000800U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT             (0x0000000BU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK              (0x00001000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT             (0x0000000CU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK              (0x00002000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT             (0x0000000DU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK              (0x00004000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT             (0x0000000EU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK              (0x00008000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT             (0x0000000FU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK              (0x00010000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT             (0x00000010U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK              (0x00020000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT             (0x00000011U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK              (0x00040000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT             (0x00000012U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK              (0x00080000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT             (0x00000013U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK               (0x00000004U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT              (0x00000002U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK              (0x00100000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT             (0x00000014U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK              (0x00200000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT             (0x00000015U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK              (0x00400000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT             (0x00000016U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK              (0x00800000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT             (0x00000017U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK              (0x01000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT             (0x00000018U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK              (0x02000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT             (0x00000019U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK              (0x04000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT             (0x0000001AU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK              (0x08000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT             (0x0000001BU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK              (0x10000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT             (0x0000001CU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK              (0x20000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT             (0x0000001DU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK               (0x00000008U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT              (0x00000003U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK              (0x40000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT             (0x0000001EU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK              (0x80000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT             (0x0000001FU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX               (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK               (0x00000010U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT              (0x00000004U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK               (0x00000020U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT              (0x00000005U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK               (0x00000040U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT              (0x00000006U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK               (0x00000080U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT              (0x00000007U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK               (0x00000100U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT              (0x00000008U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX                (0x00000001U)


#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK               (0x00000200U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT              (0x00000009U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX                (0x00000001U)



/* HW_SPARE_REC0 */

/* HW_SPARE_REC1 */

/* EFUSE_VPP_EN */
#define CSL_TOP_CTRL_EFUSE_VPP_EN_EFUSE_VPP_EN_VPP_EN_MASK                      (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_VPP_EN_EFUSE_VPP_EN_VPP_EN_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_VPP_EN_EFUSE_VPP_EN_VPP_EN_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_VPP_EN_EFUSE_VPP_EN_VPP_EN_MAX                       (0x00000001U)



/* LOCK0_KICK0 */
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                              (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                           (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                                (0xFFFFFFFFU)



/* LOCK0_KICK1 */
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                              (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                           (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                                (0xFFFFFFFFU)



/* INTR_RAW_STATUS */
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                              (0x00000002U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                             (0x00000001U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                               (0x00000001U)


#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                              (0x00000004U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                             (0x00000002U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                               (0x00000001U)


#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                              (0x00000001U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                               (0x00000001U)


#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                             (0x00000008U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                            (0x00000003U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                              (0x00000001U)



/* INTR_ENABLED_STATUS_CLEAR */
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK            (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT           (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX             (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK            (0x00000004U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT           (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX             (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK            (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX             (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK           (0x00000008U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT          (0x00000003U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX            (0x00000001U)



/* INTR_ENABLE */
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                               (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                              (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                                (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                               (0x00000004U)
#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                              (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                                (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                               (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                              (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                           (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                                (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                              (0x00000008U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                             (0x00000003U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                               (0x00000001U)



/* INTR_ENABLE_CLEAR */
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                     (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                    (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                      (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                     (0x00000004U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                    (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                      (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                     (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                      (0x00000001U)


#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                    (0x00000008U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                   (0x00000003U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                     (0x00000001U)



/* EOI */
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_MASK                                        (0x000000FFU)
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_SHIFT                                       (0x00000000U)
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_RESETVAL                                    (0x00000000U)
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_MAX                                         (0x000000FFU)



/* FAULT_ADDRESS */
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                               (0xFFFFFFFFU)



/* FAULT_TYPE_STATUS */
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                            (0x00000040U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                           (0x00000006U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                             (0x00000001U)


#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                          (0x0000003FU)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                         (0x00000000U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                      (0x00000000U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                           (0x0000003FU)



/* FAULT_ATTR_STATUS */
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                        (0x000000FFU)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                         (0x000000FFU)


#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                       (0x000FFF00U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                        (0x00000FFFU)


#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                           (0xFFF00000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                          (0x00000014U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                       (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                            (0x00000FFFU)



/* FAULT_CLEAR */
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                                 (0x00000001U)
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                                (0x00000000U)
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                             (0x00000000U)
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                                  (0x00000001U)



#ifdef __cplusplus
}
#endif
#endif