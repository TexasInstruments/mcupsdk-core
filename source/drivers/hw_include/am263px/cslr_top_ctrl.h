/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
    volatile uint32_t PID;
    volatile uint8_t  Resv_16[12];
    volatile uint32_t EFUSE_DIEID0;
    volatile uint32_t EFUSE_DIEID1;
    volatile uint32_t EFUSE_DIEID2;
    volatile uint32_t EFUSE_DIEID3;
    volatile uint32_t EFUSE_UID0;
    volatile uint32_t EFUSE_UID1;
    volatile uint32_t EFUSE_UID2;
    volatile uint32_t EFUSE_UID3;
    volatile uint32_t EFUSE_DEVICE_TYPE;
    volatile uint32_t EFUSE_FROM0_CHECKSUM;
    volatile uint32_t EFUSE_JTAG_USERCODE_ID;
    volatile uint8_t  Resv_1024[964];
    volatile uint32_t EFUSE0_ROW_61;
    volatile uint32_t EFUSE0_ROW_62;
    volatile uint32_t EFUSE0_ROW_63;
    volatile uint32_t EFUSE1_ROW_5;
    volatile uint32_t EFUSE1_ROW_6;
    volatile uint32_t EFUSE1_ROW_7;
    volatile uint32_t EFUSE1_ROW_8;
    volatile uint32_t EFUSE1_ROW_9;
    volatile uint32_t EFUSE1_ROW_10;
    volatile uint32_t EFUSE1_ROW_11;
    volatile uint32_t EFUSE1_ROW_12;
    volatile uint32_t EFUSE1_ROW_13;
    volatile uint32_t EFUSE1_ROW_14;
    volatile uint32_t EFUSE1_ROW_15;
    volatile uint32_t EFUSE1_ROW_16;
    volatile uint32_t EFUSE1_ROW_17;
    volatile uint32_t EFUSE1_ROW_18;
    volatile uint32_t EFUSE1_ROW_19;
    volatile uint32_t EFUSE1_ROW_20;
    volatile uint32_t EFUSE1_ROW_21;
    volatile uint32_t EFUSE1_ROW_22;
    volatile uint32_t EFUSE1_ROW_23;
    volatile uint32_t EFUSE1_ROW_24;
    volatile uint32_t EFUSE1_ROW_25;
    volatile uint32_t EFUSE1_ROW_26;
    volatile uint32_t EFUSE1_ROW_27;
    volatile uint32_t EFUSE1_ROW_28;
    volatile uint32_t EFUSE1_ROW_29;
    volatile uint32_t EFUSE1_ROW_30;
    volatile uint32_t EFUSE1_ROW_31;
    volatile uint32_t EFUSE1_ROW_32;
    volatile uint32_t EFUSE1_ROW_33;
    volatile uint32_t EFUSE1_ROW_34;
    volatile uint32_t EFUSE1_ROW_35;
    volatile uint32_t EFUSE1_ROW_36;
    volatile uint32_t EFUSE1_ROW_37;
    volatile uint32_t EFUSE1_ROW_38;
    volatile uint32_t EFUSE1_ROW_39;
    volatile uint32_t EFUSE1_ROW_40;
    volatile uint32_t EFUSE1_ROW_41;
    volatile uint32_t EFUSE1_ROW_42;
    volatile uint32_t EFUSE1_ROW_43;
    volatile uint32_t EFUSE1_ROW_44;
    volatile uint32_t EFUSE1_ROW_45;
    volatile uint32_t EFUSE1_ROW_46;
    volatile uint32_t EFUSE1_ROW_47;
    volatile uint32_t EFUSE1_ROW_48;
    volatile uint32_t EFUSE1_ROW_49;
    volatile uint32_t EFUSE1_ROW_50;
    volatile uint32_t EFUSE1_ROW_51;
    volatile uint32_t EFUSE1_ROW_52;
    volatile uint32_t EFUSE1_ROW_53;
    volatile uint32_t EFUSE1_ROW_54;
    volatile uint32_t EFUSE1_ROW_55;
    volatile uint32_t EFUSE1_ROW_56;
    volatile uint32_t EFUSE1_ROW_57;
    volatile uint32_t EFUSE1_ROW_58;
    volatile uint32_t EFUSE1_ROW_59;
    volatile uint32_t EFUSE1_ROW_60;
    volatile uint32_t EFUSE1_ROW_61;
    volatile uint32_t EFUSE1_ROW_62;
    volatile uint32_t EFUSE1_ROW_63;
    volatile uint32_t EFUSE2_ROW_5;
    volatile uint32_t EFUSE2_ROW_6;
    volatile uint32_t MAC_ID0;
    volatile uint32_t MAC_ID1;
    volatile uint8_t  Resv_1296[8];
    volatile uint32_t TRIM_TEMP_M40C;
    volatile uint32_t TRIM_TEMPSENSE_M40C0;
    volatile uint32_t TRIM_TEMPSENSE_M40C1;
    volatile uint32_t TRIM_TEMP_150C;
    volatile uint32_t TRIM_TEMPSENSE_150C0;
    volatile uint32_t TRIM_TEMPSENSE_150C1;
    volatile uint32_t TRIM_TEMP_30C;
    volatile uint32_t TRIM_TEMPSENSE_30C0;
    volatile uint32_t TRIM_TEMPSENSE_30C1;
    volatile uint32_t N_FACTOR_TEMPSENSE;
    volatile uint32_t TSHUT_HOT;
    volatile uint32_t TSHUT_COLD;
    volatile uint32_t JTAG_ID;
    volatile uint8_t  Resv_2048[700];
    volatile uint32_t EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN;
    volatile uint32_t EFUSE_OVERRIDE_MEM_MARGINCTRL;
    volatile uint32_t EFUSE_OVERRIDE_ADC0_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADC1_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADC2_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADC3_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADC4_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADC_CFG_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_ADC_CFG0;
    volatile uint32_t EFUSE_OVERRIDE_ADC_CFG1;
    volatile uint32_t EFUSE_OVERRIDE_ADC_CFG2;
    volatile uint32_t EFUSE_OVERRIDE_CSS01_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_CSS01_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_CSS23_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_CSS23_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_CSS45_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_CSS45_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_CSS67_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_CSS67_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_CSS89_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_CSS89_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_CSS_CFG_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_CSS_CFG0;
    volatile uint32_t EFUSE_OVERRIDE_CSS_CFG1;
    volatile uint32_t EFUSE_OVERRIDE_DAC_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_DAC_CFG;
    volatile uint32_t EFUSE_OVERRIDE_REFBUF0_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_REFBUF0_CFG;
    volatile uint32_t EFUSE_OVERRIDE_REFBUF1_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_REFBUF1_CFG;
    volatile uint32_t EFUSE_OVERRIDE_PMU_CFG;
    volatile uint32_t EFUSE_OVERRIDE_PMU_SPARE_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_LDO_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_BG_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0;
    volatile uint32_t EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1;
    volatile uint32_t EFUSE_OVERRIDE_RCOSC_TRIM;
    volatile uint32_t EFUSE_SAFETYMON_SPARE;
    volatile uint32_t EFUSE_SPARE_1;
    volatile uint32_t EFUSE_SPARE_2;
    volatile uint32_t EFUSE_SPARE_3;
    volatile uint32_t EFUSE_SPARE_4;
    volatile uint32_t EFUSE_SPARE_5;
    volatile uint32_t EFUSE_SPARE_6;
    volatile uint32_t EFUSE_SPARE_7;
    volatile uint32_t EFUSE_OVERRIDE_TSENSE_TRIM_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_TSENSE_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_PLL_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_REFBUF2_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_REFBUF2_CFG;
    volatile uint32_t EFUSE_OVERRIDE_ADCR0_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADCR1_TRIM;
    volatile uint32_t EFUSE_OVERRIDE_ADCR_CFG_CTRL;
    volatile uint32_t EFUSE_OVERRIDE_ADCR_CFG0;
    volatile uint32_t EFUSE_OVERRIDE_ADCR_CFG1;
    volatile uint32_t EFUSE_OVERRIDE_ADCR_CFG2;
    volatile uint8_t  Resv_3072[796];
    volatile uint32_t ADC_REFBUF0_CTRL;
    volatile uint32_t ADC_REFBUF1_CTRL;
    volatile uint32_t ADC_REF_COMP_CTRL;
    volatile uint32_t ADC_REF_GOOD_STATUS;
    volatile uint32_t VMON_CTRL;
    volatile uint32_t VMON_STAT;
    volatile uint32_t PMU_COARSE_STAT;
    volatile uint8_t  Resv_3104[4];
    volatile uint32_t MASK_VMON_ERROR_ESM_H;
    volatile uint32_t MASK_VMON_ERROR_ESM_L;
    volatile uint8_t  Resv_3120[8];
    volatile uint32_t MASK_ANA_ISO;
    volatile uint32_t VMON_FILTER_CTRL;
    volatile uint8_t  Resv_3136[8];
    volatile uint32_t ADC_RNG_CTRL;
    volatile uint32_t ADC0_OSD_CHEN;
    volatile uint32_t ADC1_OSD_CHEN;
    volatile uint32_t ADC2_OSD_CHEN;
    volatile uint32_t ADC3_OSD_CHEN;
    volatile uint32_t ADC4_OSD_CHEN;
    volatile uint32_t ADC0_OSD_CTRL;
    volatile uint32_t ADC1_OSD_CTRL;
    volatile uint32_t ADC2_OSD_CTRL;
    volatile uint32_t ADC3_OSD_CTRL;
    volatile uint32_t ADC4_OSD_CTRL;
    volatile uint8_t  Resv_3200[20];
    volatile uint32_t ADC_LOOPBACK_CTRL;
    volatile uint32_t CMPSSA_LOOPBACK_CTRL;
    volatile uint32_t CMPSSB_LOOPBACK_CTRL;
    volatile uint32_t ADCR01_OSD_CHEN;
    volatile uint32_t ADCR01_OSD_CTRL;
    volatile uint32_t ADC_REFBUF2_CTRL;
    volatile uint32_t TB_CTRL_ADC5_ADC6_RESERVED;
    volatile uint8_t  Resv_3328[100];
    volatile uint32_t TSENSE_CFG;
    volatile uint32_t TSENSE_STATUS;
    volatile uint32_t TSENSE_STATUS_RAW;
    volatile uint8_t  Resv_3344[4];
    volatile uint32_t TSENSE0_TSHUT;
    volatile uint32_t TSENSE0_ALERT;
    volatile uint32_t TSENSE0_CNTL;
    volatile uint32_t TSENSE0_RESULT;
    volatile uint32_t TSENSE0_DATA0;
    volatile uint32_t TSENSE0_DATA1;
    volatile uint32_t TSENSE0_DATA2;
    volatile uint32_t TSENSE0_DATA3;
    volatile uint32_t TSENSE0_ACCU;
    volatile uint8_t  Resv_3392[12];
    volatile uint32_t TSENSE1_TSHUT;
    volatile uint32_t TSENSE1_ALERT;
    volatile uint32_t TSENSE1_CNTL;
    volatile uint32_t TSENSE1_RESULT;
    volatile uint32_t TSENSE1_DATA0;
    volatile uint32_t TSENSE1_DATA1;
    volatile uint32_t TSENSE1_DATA2;
    volatile uint32_t TSENSE1_DATA3;
    volatile uint32_t TSENSE1_ACCU;
    volatile uint8_t  Resv_3452[24];
    volatile uint32_t TSENSE2_RESULT;
    volatile uint8_t  Resv_3500[44];
    volatile uint32_t TSENSE3_RESULT;
    volatile uint8_t  Resv_3584[80];
    volatile uint32_t DFT_ATB_GLOBALEN_ADC_CSS;
    volatile uint32_t DFT_ATB0_MASTEREN_ADC_CSS_DAC;
    volatile uint32_t DFT_ATB1_MASTEREN_ADC_CSS_DAC;
    volatile uint32_t DFT_PMU_REFSYS_SAFETY;
    volatile uint32_t DFT_ANA_DTB_ENABLES;
    volatile uint32_t DFT_ADC_CHSEL_OV_CTRL_VALUE;
    volatile uint32_t DFT_DAC_CTRL;
    volatile uint32_t DFT_CSS01_CTRL;
    volatile uint32_t DFT_CSS23_CTRL;
    volatile uint32_t DFT_CSS45_CTRL;
    volatile uint32_t DFT_CSS67_CTRL;
    volatile uint32_t DFT_CSS89_CTRL;
    volatile uint32_t DFT_RAMP_DACL;
    volatile uint32_t DFT_REFBUF_CTRL;
    volatile uint32_t DFT_ODP_ATB_LOOPBACK_CTRL;
    volatile uint32_t DFT_SOC_DTB_MUX_SEL;
    volatile uint32_t DFT_TEMPSENSE_CTRL;
    volatile uint32_t DFT_CTRL_1;
    volatile uint32_t DFT_CTRL_2;
    volatile uint32_t DFT_CTRL_3;
    volatile uint32_t DFT_CTRL_4;
    volatile uint32_t DFT_CTRL_5;
    volatile uint8_t  Resv_3844[172];
    volatile uint32_t PROBE_BUS_SEL0;
    volatile uint32_t PROBE_BUS_SEL1;
    volatile uint8_t  Resv_4048[196];
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint32_t HW_SPARE_WPH;
    volatile uint32_t HW_SPARE_REC;
    volatile uint32_t HW_SPARE_REC0;
    volatile uint32_t HW_SPARE_REC1;
    volatile uint8_t  Resv_4104[8];
    volatile uint32_t LOCK0_KICK0;
    volatile uint32_t LOCK0_KICK1;
    volatile uint32_t INTR_RAW_STATUS;
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;
    volatile uint32_t INTR_ENABLE;
    volatile uint32_t INTR_ENABLE_CLEAR;
    volatile uint32_t EOI;
    volatile uint32_t FAULT_ADDRESS;
    volatile uint32_t FAULT_TYPE_STATUS;
    volatile uint32_t FAULT_ATTR_STATUS;
    volatile uint32_t FAULT_CLEAR;
} CSL_top_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TOP_CTRL_PID                                                       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID0                                              (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_DIEID1                                              (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_DIEID2                                              (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_DIEID3                                              (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE_UID0                                                (0x00000020U)
#define CSL_TOP_CTRL_EFUSE_UID1                                                (0x00000024U)
#define CSL_TOP_CTRL_EFUSE_UID2                                                (0x00000028U)
#define CSL_TOP_CTRL_EFUSE_UID3                                                (0x0000002CU)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE                                         (0x00000030U)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM                                      (0x00000034U)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID                                    (0x00000038U)
#define CSL_TOP_CTRL_EFUSE0_ROW_61                                             (0x00000400U)
#define CSL_TOP_CTRL_EFUSE0_ROW_62                                             (0x00000404U)
#define CSL_TOP_CTRL_EFUSE0_ROW_63                                             (0x00000408U)
#define CSL_TOP_CTRL_EFUSE1_ROW_5                                              (0x0000040CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_6                                              (0x00000410U)
#define CSL_TOP_CTRL_EFUSE1_ROW_7                                              (0x00000414U)
#define CSL_TOP_CTRL_EFUSE1_ROW_8                                              (0x00000418U)
#define CSL_TOP_CTRL_EFUSE1_ROW_9                                              (0x0000041CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_10                                             (0x00000420U)
#define CSL_TOP_CTRL_EFUSE1_ROW_11                                             (0x00000424U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12                                             (0x00000428U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13                                             (0x0000042CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_14                                             (0x00000430U)
#define CSL_TOP_CTRL_EFUSE1_ROW_15                                             (0x00000434U)
#define CSL_TOP_CTRL_EFUSE1_ROW_16                                             (0x00000438U)
#define CSL_TOP_CTRL_EFUSE1_ROW_17                                             (0x0000043CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_18                                             (0x00000440U)
#define CSL_TOP_CTRL_EFUSE1_ROW_19                                             (0x00000444U)
#define CSL_TOP_CTRL_EFUSE1_ROW_20                                             (0x00000448U)
#define CSL_TOP_CTRL_EFUSE1_ROW_21                                             (0x0000044CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_22                                             (0x00000450U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23                                             (0x00000454U)
#define CSL_TOP_CTRL_EFUSE1_ROW_24                                             (0x00000458U)
#define CSL_TOP_CTRL_EFUSE1_ROW_25                                             (0x0000045CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_26                                             (0x00000460U)
#define CSL_TOP_CTRL_EFUSE1_ROW_27                                             (0x00000464U)
#define CSL_TOP_CTRL_EFUSE1_ROW_28                                             (0x00000468U)
#define CSL_TOP_CTRL_EFUSE1_ROW_29                                             (0x0000046CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_30                                             (0x00000470U)
#define CSL_TOP_CTRL_EFUSE1_ROW_31                                             (0x00000474U)
#define CSL_TOP_CTRL_EFUSE1_ROW_32                                             (0x00000478U)
#define CSL_TOP_CTRL_EFUSE1_ROW_33                                             (0x0000047CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_34                                             (0x00000480U)
#define CSL_TOP_CTRL_EFUSE1_ROW_35                                             (0x00000484U)
#define CSL_TOP_CTRL_EFUSE1_ROW_36                                             (0x00000488U)
#define CSL_TOP_CTRL_EFUSE1_ROW_37                                             (0x0000048CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_38                                             (0x00000490U)
#define CSL_TOP_CTRL_EFUSE1_ROW_39                                             (0x00000494U)
#define CSL_TOP_CTRL_EFUSE1_ROW_40                                             (0x00000498U)
#define CSL_TOP_CTRL_EFUSE1_ROW_41                                             (0x0000049CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_42                                             (0x000004A0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_43                                             (0x000004A4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_44                                             (0x000004A8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_45                                             (0x000004ACU)
#define CSL_TOP_CTRL_EFUSE1_ROW_46                                             (0x000004B0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_47                                             (0x000004B4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_48                                             (0x000004B8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_49                                             (0x000004BCU)
#define CSL_TOP_CTRL_EFUSE1_ROW_50                                             (0x000004C0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_51                                             (0x000004C4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_52                                             (0x000004C8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_53                                             (0x000004CCU)
#define CSL_TOP_CTRL_EFUSE1_ROW_54                                             (0x000004D0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_55                                             (0x000004D4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56                                             (0x000004D8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_57                                             (0x000004DCU)
#define CSL_TOP_CTRL_EFUSE1_ROW_58                                             (0x000004E0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_59                                             (0x000004E4U)
#define CSL_TOP_CTRL_EFUSE1_ROW_60                                             (0x000004E8U)
#define CSL_TOP_CTRL_EFUSE1_ROW_61                                             (0x000004ECU)
#define CSL_TOP_CTRL_EFUSE1_ROW_62                                             (0x000004F0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_63                                             (0x000004F4U)
#define CSL_TOP_CTRL_EFUSE2_ROW_5                                              (0x000004F8U)
#define CSL_TOP_CTRL_EFUSE2_ROW_6                                              (0x000004FCU)
#define CSL_TOP_CTRL_MAC_ID0                                                   (0x00000500U)
#define CSL_TOP_CTRL_MAC_ID1                                                   (0x00000504U)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C                                            (0x00000510U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0                                      (0x00000514U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1                                      (0x00000518U)
#define CSL_TOP_CTRL_TRIM_TEMP_150C                                            (0x0000051CU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0                                      (0x00000520U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1                                      (0x00000524U)
#define CSL_TOP_CTRL_TRIM_TEMP_30C                                             (0x00000528U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0                                       (0x0000052CU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1                                       (0x00000530U)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE                                        (0x00000534U)
#define CSL_TOP_CTRL_TSHUT_HOT                                                 (0x00000538U)
#define CSL_TOP_CTRL_TSHUT_COLD                                                (0x0000053CU)
#define CSL_TOP_CTRL_JTAG_ID                                                   (0x00000540U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN                 (0x00000800U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL                             (0x00000804U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM                                  (0x00000808U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM                                  (0x0000080CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM                                  (0x00000810U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM                                  (0x00000814U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM                                  (0x00000818U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL                               (0x0000081CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0                                   (0x00000820U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1                                   (0x00000824U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2                                   (0x00000828U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL                            (0x0000082CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM                                 (0x00000830U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL                            (0x00000834U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM                                 (0x00000838U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL                            (0x0000083CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM                                 (0x00000840U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL                            (0x00000844U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM                                 (0x00000848U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL                            (0x0000084CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM                                 (0x00000850U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL                               (0x00000854U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0                                   (0x00000858U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG1                                   (0x0000085CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM                                   (0x00000860U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG                                    (0x00000864U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM                               (0x00000868U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG                                (0x0000086CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM                               (0x00000870U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG                                (0x00000874U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG                                    (0x00000878U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM                             (0x0000087CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM                                   (0x00000880U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM                                    (0x00000884U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL                   (0x00000888U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0                       (0x0000088CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1                       (0x00000890U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM                                 (0x00000894U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE                                     (0x00000898U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1                                             (0x0000089CU)
#define CSL_TOP_CTRL_EFUSE_SPARE_2                                             (0x000008A0U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3                                             (0x000008A4U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4                                             (0x000008A8U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5                                             (0x000008ACU)
#define CSL_TOP_CTRL_EFUSE_SPARE_6                                             (0x000008B0U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7                                             (0x000008B4U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL                           (0x000008B8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM                                (0x000008BCU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM                                   (0x000008C0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM                               (0x000008C4U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG                                (0x000008C8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM                                 (0x000008CCU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM                                 (0x000008D0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL                              (0x000008D4U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG0                                  (0x000008D8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG1                                  (0x000008DCU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG2                                  (0x000008E0U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL                                          (0x00000C00U)
#define CSL_TOP_CTRL_ADC_REFBUF1_CTRL                                          (0x00000C04U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL                                         (0x00000C08U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS                                       (0x00000C0CU)
#define CSL_TOP_CTRL_VMON_CTRL                                                 (0x00000C10U)
#define CSL_TOP_CTRL_VMON_STAT                                                 (0x00000C14U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT                                           (0x00000C18U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H                                     (0x00000C20U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L                                     (0x00000C24U)
#define CSL_TOP_CTRL_MASK_ANA_ISO                                              (0x00000C30U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL                                          (0x00000C34U)
#define CSL_TOP_CTRL_ADC_RNG_CTRL                                              (0x00000C40U)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN                                             (0x00000C44U)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN                                             (0x00000C48U)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN                                             (0x00000C4CU)
#define CSL_TOP_CTRL_ADC3_OSD_CHEN                                             (0x00000C50U)
#define CSL_TOP_CTRL_ADC4_OSD_CHEN                                             (0x00000C54U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL                                             (0x00000C58U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL                                             (0x00000C5CU)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL                                             (0x00000C60U)
#define CSL_TOP_CTRL_ADC3_OSD_CTRL                                             (0x00000C64U)
#define CSL_TOP_CTRL_ADC4_OSD_CTRL                                             (0x00000C68U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL                                         (0x00000C80U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL                                      (0x00000C84U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL                                      (0x00000C88U)
#define CSL_TOP_CTRL_ADCR01_OSD_CHEN                                           (0x00000C8CU)
#define CSL_TOP_CTRL_ADCR01_OSD_CTRL                                           (0x00000C90U)
#define CSL_TOP_CTRL_ADC_REFBUF2_CTRL                                          (0x00000C94U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED                                (0x00000C98U)
#define CSL_TOP_CTRL_TSENSE_CFG                                                (0x00000D00U)
#define CSL_TOP_CTRL_TSENSE_STATUS                                             (0x00000D04U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW                                         (0x00000D08U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT                                             (0x00000D10U)
#define CSL_TOP_CTRL_TSENSE0_ALERT                                             (0x00000D14U)
#define CSL_TOP_CTRL_TSENSE0_CNTL                                              (0x00000D18U)
#define CSL_TOP_CTRL_TSENSE0_RESULT                                            (0x00000D1CU)
#define CSL_TOP_CTRL_TSENSE0_DATA0                                             (0x00000D20U)
#define CSL_TOP_CTRL_TSENSE0_DATA1                                             (0x00000D24U)
#define CSL_TOP_CTRL_TSENSE0_DATA2                                             (0x00000D28U)
#define CSL_TOP_CTRL_TSENSE0_DATA3                                             (0x00000D2CU)
#define CSL_TOP_CTRL_TSENSE0_ACCU                                              (0x00000D30U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT                                             (0x00000D40U)
#define CSL_TOP_CTRL_TSENSE1_ALERT                                             (0x00000D44U)
#define CSL_TOP_CTRL_TSENSE1_CNTL                                              (0x00000D48U)
#define CSL_TOP_CTRL_TSENSE1_RESULT                                            (0x00000D4CU)
#define CSL_TOP_CTRL_TSENSE1_DATA0                                             (0x00000D50U)
#define CSL_TOP_CTRL_TSENSE1_DATA1                                             (0x00000D54U)
#define CSL_TOP_CTRL_TSENSE1_DATA2                                             (0x00000D58U)
#define CSL_TOP_CTRL_TSENSE1_DATA3                                             (0x00000D5CU)
#define CSL_TOP_CTRL_TSENSE1_ACCU                                              (0x00000D60U)
#define CSL_TOP_CTRL_TSENSE2_RESULT                                            (0x00000D7CU)
#define CSL_TOP_CTRL_TSENSE3_RESULT                                            (0x00000DACU)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS                                  (0x00000E00U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC                             (0x00000E04U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC                             (0x00000E08U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY                                     (0x00000E0CU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES                                       (0x00000E10U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE                               (0x00000E14U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL                                              (0x00000E18U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL                                            (0x00000E1CU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL                                            (0x00000E20U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL                                            (0x00000E24U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL                                            (0x00000E28U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL                                            (0x00000E2CU)
#define CSL_TOP_CTRL_DFT_RAMP_DACL                                             (0x00000E30U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL                                           (0x00000E34U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL                                 (0x00000E38U)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL                                       (0x00000E3CU)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL                                        (0x00000E40U)
#define CSL_TOP_CTRL_DFT_CTRL_1                                                (0x00000E44U)
#define CSL_TOP_CTRL_DFT_CTRL_2                                                (0x00000E48U)
#define CSL_TOP_CTRL_DFT_CTRL_3                                                (0x00000E4CU)
#define CSL_TOP_CTRL_DFT_CTRL_4                                                (0x00000E50U)
#define CSL_TOP_CTRL_DFT_CTRL_5                                                (0x00000E54U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0                                            (0x00000F04U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1                                            (0x00000F08U)
#define CSL_TOP_CTRL_HW_SPARE_RW0                                              (0x00000FD0U)
#define CSL_TOP_CTRL_HW_SPARE_RW1                                              (0x00000FD4U)
#define CSL_TOP_CTRL_HW_SPARE_RW2                                              (0x00000FD8U)
#define CSL_TOP_CTRL_HW_SPARE_RW3                                              (0x00000FDCU)
#define CSL_TOP_CTRL_HW_SPARE_RO0                                              (0x00000FE0U)
#define CSL_TOP_CTRL_HW_SPARE_RO1                                              (0x00000FE4U)
#define CSL_TOP_CTRL_HW_SPARE_RO2                                              (0x00000FE8U)
#define CSL_TOP_CTRL_HW_SPARE_RO3                                              (0x00000FECU)
#define CSL_TOP_CTRL_HW_SPARE_WPH                                              (0x00000FF0U)
#define CSL_TOP_CTRL_HW_SPARE_REC                                              (0x00000FF4U)
#define CSL_TOP_CTRL_HW_SPARE_REC0                                             (0x00000FF8U)
#define CSL_TOP_CTRL_HW_SPARE_REC1                                             (0x00000FFCU)
#define CSL_TOP_CTRL_LOCK0_KICK0                                               (0x00001008U)
#define CSL_TOP_CTRL_LOCK0_KICK1                                               (0x0000100CU)
#define CSL_TOP_CTRL_INTR_RAW_STATUS                                           (0x00001010U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR                                 (0x00001014U)
#define CSL_TOP_CTRL_INTR_ENABLE                                               (0x00001018U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR                                         (0x0000101CU)
#define CSL_TOP_CTRL_EOI                                                       (0x00001020U)
#define CSL_TOP_CTRL_FAULT_ADDRESS                                             (0x00001024U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS                                         (0x00001028U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS                                         (0x0000102CU)
#define CSL_TOP_CTRL_FAULT_CLEAR                                               (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_TOP_CTRL_PID_PID_MINOR_MASK                                        (0x0000003FU)
#define CSL_TOP_CTRL_PID_PID_MINOR_SHIFT                                       (0x00000000U)
#define CSL_TOP_CTRL_PID_PID_MINOR_RESETVAL                                    (0x00000014U)
#define CSL_TOP_CTRL_PID_PID_MINOR_MAX                                         (0x0000003FU)

#define CSL_TOP_CTRL_PID_PID_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_TOP_CTRL_PID_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_TOP_CTRL_PID_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_TOP_CTRL_PID_PID_CUSTOM_MAX                                        (0x00000003U)

#define CSL_TOP_CTRL_PID_PID_MAJOR_MASK                                        (0x00000700U)
#define CSL_TOP_CTRL_PID_PID_MAJOR_SHIFT                                       (0x00000008U)
#define CSL_TOP_CTRL_PID_PID_MAJOR_RESETVAL                                    (0x00000002U)
#define CSL_TOP_CTRL_PID_PID_MAJOR_MAX                                         (0x00000007U)

#define CSL_TOP_CTRL_PID_PID_MISC_MASK                                         (0x0000F800U)
#define CSL_TOP_CTRL_PID_PID_MISC_SHIFT                                        (0x0000000BU)
#define CSL_TOP_CTRL_PID_PID_MISC_RESETVAL                                     (0x00000000U)
#define CSL_TOP_CTRL_PID_PID_MISC_MAX                                          (0x0000001FU)

#define CSL_TOP_CTRL_PID_PID_MSB16_MASK                                        (0xFFFF0000U)
#define CSL_TOP_CTRL_PID_PID_MSB16_SHIFT                                       (0x00000010U)
#define CSL_TOP_CTRL_PID_PID_MSB16_RESETVAL                                    (0x00006180U)
#define CSL_TOP_CTRL_PID_PID_MSB16_MAX                                         (0x0000FFFFU)

#define CSL_TOP_CTRL_PID_RESETVAL                                              (0x61800214U)

/* EFUSE_DIEID0 */

#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID0_EFUSE_DIEID0_VAL_MAX                         (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_DIEID0_RESETVAL                                     (0x00000000U)

/* EFUSE_DIEID1 */

#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID1_EFUSE_DIEID1_VAL_MAX                         (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_DIEID1_RESETVAL                                     (0x00000000U)

/* EFUSE_DIEID2 */

#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID2_EFUSE_DIEID2_VAL_MAX                         (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_DIEID2_RESETVAL                                     (0x00000000U)

/* EFUSE_DIEID3 */

#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_SHIFT                       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_RESETVAL                    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DIEID3_EFUSE_DIEID3_VAL_MAX                         (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_DIEID3_RESETVAL                                     (0x00000000U)

/* EFUSE_UID0 */

#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID0_EFUSE_UID0_VAL_MAX                             (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_UID0_RESETVAL                                       (0x00000000U)

/* EFUSE_UID1 */

#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID1_EFUSE_UID1_VAL_MAX                             (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_UID1_RESETVAL                                       (0x00000000U)

/* EFUSE_UID2 */

#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_MASK                            (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID2_EFUSE_UID2_VAL_MAX                             (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_UID2_RESETVAL                                       (0x00000000U)

/* EFUSE_UID3 */

#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_MASK                            (0x00FFFFFFU)
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_UID3_EFUSE_UID3_VAL_MAX                             (0x00FFFFFFU)

#define CSL_TOP_CTRL_EFUSE_UID3_RESETVAL                                       (0x00000000U)

/* EFUSE_DEVICE_TYPE */

#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_MASK              (0x0000FFFFU)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_SHIFT             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_VAL_MAX               (0x0000FFFFU)

#define CSL_TOP_CTRL_EFUSE_DEVICE_TYPE_RESETVAL                                (0x00000000U)

/* EFUSE_FROM0_CHECKSUM */

#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_MASK        (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_SHIFT       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_EFUSE_FROM0_CHECKSUM_VAL_MAX         (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_FROM0_CHECKSUM_RESETVAL                             (0x00000000U)

/* EFUSE_JTAG_USERCODE_ID */

#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_MASK    (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_EFUSE_JTAG_USERCODE_ID_VAL_MAX     (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_JTAG_USERCODE_ID_RESETVAL                           (0x00000000U)

/* EFUSE0_ROW_61 */

#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_61_EFUSE0_ROW_61_EFUSE0_ROW_61_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE0_ROW_61_RESETVAL                                    (0x00000000U)

/* EFUSE0_ROW_62 */

#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_62_EFUSE0_ROW_62_EFUSE0_ROW_62_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE0_ROW_62_RESETVAL                                    (0x00000000U)

/* EFUSE0_ROW_63 */

#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE0_ROW_63_EFUSE0_ROW_63_EFUSE0_ROW_63_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE0_ROW_63_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_5 */

#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_5_EFUSE1_ROW_5_EFUSE1_ROW_5_MAX                (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_5_RESETVAL                                     (0x00000000U)

/* EFUSE1_ROW_6 */

#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_6_EFUSE1_ROW_6_EFUSE1_ROW_6_MAX                (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_6_RESETVAL                                     (0x00000000U)

/* EFUSE1_ROW_7 */

#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_7_EFUSE1_ROW_7_EFUSE1_ROW_7_MAX                (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_7_RESETVAL                                     (0x00000000U)

/* EFUSE1_ROW_8 */

#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_8_EFUSE1_ROW_8_EFUSE1_ROW_8_MAX                (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_8_RESETVAL                                     (0x00000000U)

/* EFUSE1_ROW_9 */

#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_9_EFUSE1_ROW_9_EFUSE1_ROW_9_MAX                (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_9_RESETVAL                                     (0x00000000U)

/* EFUSE1_ROW_10 */

#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_10_EFUSE1_ROW_10_EFUSE1_ROW_10_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_10_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_11 */

#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_11_EFUSE1_ROW_11_EFUSE1_ROW_11_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_11_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_12 */

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_MEM_SIZE_MASK              (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_MEM_SIZE_SHIFT             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_MEM_SIZE_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_L2_MEM_SIZE_MAX               (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_MASK    (0x00000010U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_SHIFT   (0x00000004U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_FORCE_DUAL_CORE_MAX     (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_MASK  (0x00000020U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS0_DUAL_CORE_DISABLE_MAX   (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_FORCE_DUAL_CORE_MASK    (0x00000040U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_FORCE_DUAL_CORE_SHIFT   (0x00000006U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_FORCE_DUAL_CORE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_FORCE_DUAL_CORE_MAX     (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DUAL_CORE_DISABLE_MASK  (0x00000080U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DUAL_CORE_DISABLE_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DUAL_CORE_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DUAL_CORE_DISABLE_MAX   (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DISABLE_MASK            (0x00000100U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DISABLE_SHIFT           (0x00000008U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DISABLE_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS1_DISABLE_MAX             (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_MASK                (0x00000200U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_SHIFT               (0x00000009U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_MAX                 (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_TWOX_CTRL_PERIP_DISABLE_MASK  (0x00000400U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_TWOX_CTRL_PERIP_DISABLE_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_TWOX_CTRL_PERIP_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_TWOX_CTRL_PERIP_DISABLE_MAX   (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_DIS_MASK                (0x00000800U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_DIS_SHIFT               (0x0000000BU)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_DIS_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_DIS_MAX                 (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_HW_DIS_MASK             (0x000FF000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_HW_DIS_SHIFT            (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_HW_DIS_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_ICSSM_HW_DIS_MAX              (0x000000FFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_MASK                (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_SHIFT               (0x00000014U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_CANFD_DIS_MAX                 (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_EPWM_FEATURE_DISABLE_MASK     (0x01000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_EPWM_FEATURE_DISABLE_SHIFT    (0x00000018U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_EPWM_FEATURE_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_EPWM_FEATURE_DISABLE_MAX      (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_12_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_13 */

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_HSM_HALT_ON_ROM_ECC_ERR_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_MASK             (0x00000002U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_SHIFT            (0x00000001U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_ROM_PBIST_EN_MAX              (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_MASK              (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_SHIFT             (0x00000002U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_AES_DISABLE_MAX               (0x00000007U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_CANFD_DIS_MASK                (0x000003C0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_CANFD_DIS_SHIFT               (0x00000006U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_CANFD_DIS_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_CANFD_DIS_MAX                 (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_IP_DISABLE_MASK      (0x00000400U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_IP_DISABLE_SHIFT     (0x0000000AU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_IP_DISABLE_RESETVAL  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_IP_DISABLE_MAX       (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_ADC_DISABLE_MASK     (0x00000800U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_ADC_DISABLE_SHIFT    (0x0000000BU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_ADC_DISABLE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_RESOLVER_ADC_DISABLE_MAX      (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_MASK      (0x00001000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_SHIFT     (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_RESETVAL  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS0_CORE1_DISABLE_MAX       (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS1_CORE1_DISABLE_MASK      (0x00002000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS1_CORE1_DISABLE_SHIFT     (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS1_CORE1_DISABLE_RESETVAL  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_R5SS1_CORE1_DISABLE_MAX       (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_FLASH_SIP_PACKAGE_MASK        (0x00004000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_FLASH_SIP_PACKAGE_SHIFT       (0x0000000EU)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_FLASH_SIP_PACKAGE_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_13_EFUSE1_ROW_13_FLASH_SIP_PACKAGE_MAX         (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_13_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_14 */

#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_14_EFUSE1_ROW_14_EFUSE1_ROW_14_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_14_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_15 */

#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_15_EFUSE1_ROW_15_EFUSE1_ROW_15_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_15_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_16 */

#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_16_EFUSE1_ROW_16_EFUSE1_ROW_16_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_16_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_17 */

#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_17_EFUSE1_ROW_17_EFUSE1_ROW_17_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_17_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_18 */

#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_18_EFUSE1_ROW_18_EFUSE1_ROW_18_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_18_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_19 */

#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_19_EFUSE1_ROW_19_EFUSE1_ROW_19_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_19_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_20 */

#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_20_EFUSE1_ROW_20_EFUSE1_ROW_20_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_20_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_21 */

#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_21_EFUSE1_ROW_21_EFUSE1_ROW_21_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_21_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_22 */

#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_22_EFUSE1_ROW_22_EFUSE1_ROW_22_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_22_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_23 */

#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_MASK          (0x0000001FU)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_SHIFT         (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_MAX           (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_MASK           (0x000003E0U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_SHIFT          (0x00000005U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_MAX            (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_MASK    (0x00000400U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_SHIFT   (0x0000000AU)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_CORE_ADPLL_TRIM_VALID_MAX     (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_MASK     (0x00000800U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_SHIFT    (0x0000000BU)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_23_EFUSE1_ROW_23_PER_ADPLL_TRIM_VALID_MAX      (0x00000001U)

#define CSL_TOP_CTRL_EFUSE1_ROW_23_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_24 */

#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_24_EFUSE1_ROW_24_EFUSE1_ROW_24_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_24_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_25 */

#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_25_EFUSE1_ROW_25_EFUSE1_ROW_25_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_25_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_26 */

#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_26_EFUSE1_ROW_26_EFUSE1_ROW_26_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_26_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_27 */

#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_27_EFUSE1_ROW_27_EFUSE1_ROW_27_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_27_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_28 */

#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_28_EFUSE1_ROW_28_EFUSE1_ROW_28_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_28_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_29 */

#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_29_EFUSE1_ROW_29_EFUSE1_ROW_29_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_29_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_30 */

#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_30_EFUSE1_ROW_30_EFUSE1_ROW_30_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_30_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_31 */

#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_31_EFUSE1_ROW_31_EFUSE1_ROW_31_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_31_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_32 */

#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_32_EFUSE1_ROW_32_EFUSE1_ROW_32_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_32_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_33 */

#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_33_EFUSE1_ROW_33_EFUSE1_ROW_33_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_33_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_34 */

#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_34_EFUSE1_ROW_34_EFUSE1_ROW_34_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_34_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_35 */

#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_35_EFUSE1_ROW_35_EFUSE1_ROW_35_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_35_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_36 */

#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_36_EFUSE1_ROW_36_EFUSE1_ROW_36_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_36_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_37 */

#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_37_EFUSE1_ROW_37_EFUSE1_ROW_37_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_37_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_38 */

#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_38_EFUSE1_ROW_38_EFUSE1_ROW_38_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_38_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_39 */

#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_39_EFUSE1_ROW_39_EFUSE1_ROW_39_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_39_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_40 */

#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_40_EFUSE1_ROW_40_EFUSE1_ROW_40_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_40_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_41 */

#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_41_EFUSE1_ROW_41_EFUSE1_ROW_41_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_41_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_42 */

#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_42_EFUSE1_ROW_42_EFUSE1_ROW_42_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_42_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_43 */

#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_43_EFUSE1_ROW_43_EFUSE1_ROW_43_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_43_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_44 */

#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_44_EFUSE1_ROW_44_EFUSE1_ROW_44_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_44_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_45 */

#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_45_EFUSE1_ROW_45_EFUSE1_ROW_45_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_45_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_46 */

#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_46_EFUSE1_ROW_46_EFUSE1_ROW_46_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_46_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_47 */

#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_47_EFUSE1_ROW_47_EFUSE1_ROW_47_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_47_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_48 */

#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_48_EFUSE1_ROW_48_EFUSE1_ROW_48_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_48_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_49 */

#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_49_EFUSE1_ROW_49_EFUSE1_ROW_49_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_49_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_50 */

#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_50_EFUSE1_ROW_50_EFUSE1_ROW_50_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_50_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_51 */

#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_51_EFUSE1_ROW_51_EFUSE1_ROW_51_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_51_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_52 */

#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_52_EFUSE1_ROW_52_EFUSE1_ROW_52_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_52_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_53 */

#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_53_EFUSE1_ROW_53_EFUSE1_ROW_53_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_53_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_54 */

#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_54_EFUSE1_ROW_54_EFUSE1_ROW_54_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_54_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_55 */

#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_EFUSE1_ROW_55_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_EFUSE1_ROW_55_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_EFUSE1_ROW_55_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_55_EFUSE1_ROW_55_EFUSE1_ROW_55_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_55_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_56 */

#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_EFUSE1_ROW_56_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_EFUSE1_ROW_56_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_EFUSE1_ROW_56_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_56_EFUSE1_ROW_56_EFUSE1_ROW_56_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_56_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_57 */

#define CSL_TOP_CTRL_EFUSE1_ROW_57_EFUSE1_ROW_57_EFUSE1_ROW_57_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_57_EFUSE1_ROW_57_EFUSE1_ROW_57_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_57_EFUSE1_ROW_57_EFUSE1_ROW_57_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_57_EFUSE1_ROW_57_EFUSE1_ROW_57_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_57_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_58 */

#define CSL_TOP_CTRL_EFUSE1_ROW_58_EFUSE1_ROW_58_EFUSE1_ROW_58_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_58_EFUSE1_ROW_58_EFUSE1_ROW_58_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_58_EFUSE1_ROW_58_EFUSE1_ROW_58_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_58_EFUSE1_ROW_58_EFUSE1_ROW_58_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_58_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_59 */

#define CSL_TOP_CTRL_EFUSE1_ROW_59_EFUSE1_ROW_59_EFUSE1_ROW_59_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_59_EFUSE1_ROW_59_EFUSE1_ROW_59_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_59_EFUSE1_ROW_59_EFUSE1_ROW_59_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_59_EFUSE1_ROW_59_EFUSE1_ROW_59_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_59_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_60 */

#define CSL_TOP_CTRL_EFUSE1_ROW_60_EFUSE1_ROW_60_EFUSE1_ROW_60_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_60_EFUSE1_ROW_60_EFUSE1_ROW_60_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_60_EFUSE1_ROW_60_EFUSE1_ROW_60_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_60_EFUSE1_ROW_60_EFUSE1_ROW_60_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_60_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_61 */

#define CSL_TOP_CTRL_EFUSE1_ROW_61_EFUSE1_ROW_61_EFUSE1_ROW_61_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_61_EFUSE1_ROW_61_EFUSE1_ROW_61_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_61_EFUSE1_ROW_61_EFUSE1_ROW_61_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_61_EFUSE1_ROW_61_EFUSE1_ROW_61_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_61_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_62 */

#define CSL_TOP_CTRL_EFUSE1_ROW_62_EFUSE1_ROW_62_EFUSE1_ROW_62_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_62_EFUSE1_ROW_62_EFUSE1_ROW_62_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_62_EFUSE1_ROW_62_EFUSE1_ROW_62_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_62_EFUSE1_ROW_62_EFUSE1_ROW_62_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_62_RESETVAL                                    (0x00000000U)

/* EFUSE1_ROW_63 */

#define CSL_TOP_CTRL_EFUSE1_ROW_63_EFUSE1_ROW_63_EFUSE1_ROW_63_MASK            (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE1_ROW_63_EFUSE1_ROW_63_EFUSE1_ROW_63_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_63_EFUSE1_ROW_63_EFUSE1_ROW_63_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_EFUSE1_ROW_63_EFUSE1_ROW_63_EFUSE1_ROW_63_MAX             (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE1_ROW_63_RESETVAL                                    (0x00000000U)

/* EFUSE2_ROW_5 */

#define CSL_TOP_CTRL_EFUSE2_ROW_5_EFUSE2_ROW_5_EFUSE2_ROW_5_MASK               (0x03FFFFFFU)
#define CSL_TOP_CTRL_EFUSE2_ROW_5_EFUSE2_ROW_5_EFUSE2_ROW_5_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_EFUSE2_ROW_5_EFUSE2_ROW_5_EFUSE2_ROW_5_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_EFUSE2_ROW_5_EFUSE2_ROW_5_EFUSE2_ROW_5_MAX                (0x03FFFFFFU)

#define CSL_TOP_CTRL_EFUSE2_ROW_5_RESETVAL                                     (0x00000000U)

/* EFUSE2_ROW_6 */

#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_BOOTROM_CFG_MASK                (0x000000FFU)
#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_BOOTROM_CFG_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_BOOTROM_CFG_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_BOOTROM_CFG_MAX                 (0x000000FFU)

#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_SPARE_MASK                      (0x03FFFF00U)
#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_SPARE_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_SPARE_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE2_ROW_6_EFUSE2_ROW_6_SPARE_MAX                       (0x0003FFFFU)

#define CSL_TOP_CTRL_EFUSE2_ROW_6_RESETVAL                                     (0x00000000U)

/* MAC_ID0 */

#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID0_MAC_ID0_MACID_LO_MAX                              (0xFFFFFFFFU)

#define CSL_TOP_CTRL_MAC_ID0_RESETVAL                                          (0x00000000U)

/* MAC_ID1 */

#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_MASK                             (0x0000FFFFU)
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_MAC_ID1_MAC_ID1_MACID_HI_MAX                              (0x0000FFFFU)

#define CSL_TOP_CTRL_MAC_ID1_RESETVAL                                          (0x00000000U)

/* TRIM_TEMP_M40C */

#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_MASK                   (0x000007FFU)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_M40C_TRIM_TEMP_M40C_TEMP_MAX                    (0x000007FFU)

#define CSL_TOP_CTRL_TRIM_TEMP_M40C_RESETVAL                                   (0x00000000U)

/* TRIM_TEMPSENSE_M40C0 */

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_MASK       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_TRIM_TEMPSENSE_M40C0_TRIM_MAX        (0xFFFFFFFFU)

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C0_RESETVAL                             (0x00000000U)

/* TRIM_TEMPSENSE_M40C1 */

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_MASK       (0x000001FFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_TRIM_TEMPSENSE_M40C1_TRIM_MAX        (0x000001FFU)

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_M40C1_RESETVAL                             (0x00000000U)

/* TRIM_TEMP_150C */

#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_MASK                   (0x000007FFU)
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_SHIFT                  (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_150C_TRIM_TEMP_150C_TEMP_MAX                    (0x000007FFU)

#define CSL_TOP_CTRL_TRIM_TEMP_150C_RESETVAL                                   (0x00000000U)

/* TRIM_TEMPSENSE_150C0 */

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_MASK       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_TRIM_TEMPSENSE_150C0_TRIM_MAX        (0xFFFFFFFFU)

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C0_RESETVAL                             (0x00000000U)

/* TRIM_TEMPSENSE_150C1 */

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_MASK       (0x000001FFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_TRIM_TEMPSENSE_150C1_TRIM_MAX        (0x000001FFU)

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_150C1_RESETVAL                             (0x00000000U)

/* TRIM_TEMP_30C */

#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_MASK                     (0x000007FFU)
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMP_30C_TRIM_TEMP_30C_TEMP_MAX                      (0x000007FFU)

#define CSL_TOP_CTRL_TRIM_TEMP_30C_RESETVAL                                    (0x00000000U)

/* TRIM_TEMPSENSE_30C0 */

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_MASK         (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_SHIFT        (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_TRIM_TEMPSENSE_30C0_TRIM_MAX          (0xFFFFFFFFU)

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C0_RESETVAL                              (0x00000000U)

/* TRIM_TEMPSENSE_30C1 */

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_MASK         (0x000001FFU)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_SHIFT        (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_TRIM_TEMPSENSE_30C1_TRIM_MAX          (0x000001FFU)

#define CSL_TOP_CTRL_TRIM_TEMPSENSE_30C1_RESETVAL                              (0x00000000U)

/* N_FACTOR_TEMPSENSE */

#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_MASK            (0x00003FFFU)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_N_FACTOR_TEMPSENSE_VAL_MAX             (0x00003FFFU)

#define CSL_TOP_CTRL_N_FACTOR_TEMPSENSE_RESETVAL                               (0x00000000U)

/* TSHUT_HOT */

#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_MASK                              (0x000000FFU)
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_HOT_TSHUT_HOT_VAL_MAX                               (0x000000FFU)

#define CSL_TOP_CTRL_TSHUT_HOT_RESETVAL                                        (0x00000000U)

/* TSHUT_COLD */

#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_MASK                            (0x000000FFU)
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_SHIFT                           (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_TSHUT_COLD_TSHUT_COLD_VAL_MAX                             (0x000000FFU)

#define CSL_TOP_CTRL_TSHUT_COLD_RESETVAL                                       (0x00000000U)

/* JTAG_ID */

#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_MASK                                   (0xFFFFFFFFU)
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_SHIFT                                  (0x00000000U)
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_RESETVAL                               (0x00000000U)
#define CSL_TOP_CTRL_JTAG_ID_JTAG_ID_ID_MAX                                    (0xFFFFFFFFU)

#define CSL_TOP_CTRL_JTAG_ID_RESETVAL                                          (0x00000000U)

/* EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_MASK (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_OVERRIDE_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_HSM_HALT_ON_ROM_ECC_ERR_EN_RESETVAL        (0x00000000U)

/* EFUSE_OVERRIDE_MEM_MARGINCTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_MASK (0x00000030U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_MAX (0x00000003U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_MASK (0x00000700U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_MASK (0x0000F000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_MASK (0x00070000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_MASK (0x00300000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_MAX (0x00000003U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_MASK (0x07000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_MASK (0x30000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_SHIFT (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_MAX (0x00000003U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_RESETVAL                    (0x00000000U)

/* EFUSE_OVERRIDE_ADC0_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_MASK (0x7FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_EFUSE_OVERRIDE_ADC0_TRIM_ADC0_TRIM_MAX (0x00007FFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC0_TRIM_RESETVAL                         (0x00000000U)

/* EFUSE_OVERRIDE_ADC1_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_MASK (0x7FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_EFUSE_OVERRIDE_ADC1_TRIM_ADC1_TRIM_MAX (0x00007FFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC1_TRIM_RESETVAL                         (0x00000000U)

/* EFUSE_OVERRIDE_ADC2_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_MASK (0x7FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_EFUSE_OVERRIDE_ADC2_TRIM_ADC2_TRIM_MAX (0x00007FFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC2_TRIM_RESETVAL                         (0x00000000U)

/* EFUSE_OVERRIDE_ADC3_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_ADC3_TRIM_MASK (0x7FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_ADC3_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_ADC3_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_EFUSE_OVERRIDE_ADC3_TRIM_ADC3_TRIM_MAX (0x00007FFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC3_TRIM_RESETVAL                         (0x00000000U)

/* EFUSE_OVERRIDE_ADC4_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_ADC4_TRIM_MASK (0x7FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_ADC4_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_ADC4_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_EFUSE_OVERRIDE_ADC4_TRIM_ADC4_TRIM_MAX (0x00007FFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC4_TRIM_RESETVAL                         (0x00000000U)

/* EFUSE_OVERRIDE_ADC_CFG_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_MASK (0x00000038U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_OVERRIDE_RANGE_CTRL_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG_CTRL_RESETVAL                      (0x00000000U)

/* EFUSE_OVERRIDE_ADC_CFG0 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_RESETVAL (0x514554C9U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_EFUSE_OVERRIDE_ADC_CFG0_ADC_CFG_31_0_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG0_RESETVAL                          (0x514554C9U)

/* EFUSE_OVERRIDE_ADC_CFG1 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_RESETVAL (0xA21B2908U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_EFUSE_OVERRIDE_ADC_CFG1_ADC_CFG_63_32_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG1_RESETVAL                          (0xA21B2908U)

/* EFUSE_OVERRIDE_ADC_CFG2 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_87_64_MASK (0x00FFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_87_64_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_87_64_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_EFUSE_OVERRIDE_ADC_CFG2_ADC_CFG_87_64_MAX (0x00FFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADC_CFG2_RESETVAL                          (0x00000000U)

/* EFUSE_OVERRIDE_CSS01_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_CTRL_RESETVAL                   (0x00000000U)

/* EFUSE_OVERRIDE_CSS01_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_EFUSE_OVERRIDE_CSS01_TRIM_CSS01_TRIM_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_EFUSE_OVERRIDE_CSS01_TRIM_CSS01_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_EFUSE_OVERRIDE_CSS01_TRIM_CSS01_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_EFUSE_OVERRIDE_CSS01_TRIM_CSS01_TRIM_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS01_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_CSS23_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_CTRL_RESETVAL                   (0x00000000U)

/* EFUSE_OVERRIDE_CSS23_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_EFUSE_OVERRIDE_CSS23_TRIM_CSS23_TRIM_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_EFUSE_OVERRIDE_CSS23_TRIM_CSS23_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_EFUSE_OVERRIDE_CSS23_TRIM_CSS23_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_EFUSE_OVERRIDE_CSS23_TRIM_CSS23_TRIM_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS23_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_CSS45_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_CTRL_RESETVAL                   (0x00000000U)

/* EFUSE_OVERRIDE_CSS45_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_EFUSE_OVERRIDE_CSS45_TRIM_CSS45_TRIM_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_EFUSE_OVERRIDE_CSS45_TRIM_CSS45_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_EFUSE_OVERRIDE_CSS45_TRIM_CSS45_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_EFUSE_OVERRIDE_CSS45_TRIM_CSS45_TRIM_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS45_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_CSS67_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_CTRL_RESETVAL                   (0x00000000U)

/* EFUSE_OVERRIDE_CSS67_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_EFUSE_OVERRIDE_CSS67_TRIM_CSS67_TRIM_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_EFUSE_OVERRIDE_CSS67_TRIM_CSS67_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_EFUSE_OVERRIDE_CSS67_TRIM_CSS67_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_EFUSE_OVERRIDE_CSS67_TRIM_CSS67_TRIM_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS67_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_CSS89_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_CTRL_RESETVAL                   (0x00000000U)

/* EFUSE_OVERRIDE_CSS89_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_EFUSE_OVERRIDE_CSS89_TRIM_CSS89_TRIM_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_EFUSE_OVERRIDE_CSS89_TRIM_CSS89_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_EFUSE_OVERRIDE_CSS89_TRIM_CSS89_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_EFUSE_OVERRIDE_CSS89_TRIM_CSS89_TRIM_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS89_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_CSS_CFG_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG_CTRL_RESETVAL                      (0x00000000U)

/* EFUSE_OVERRIDE_CSS_CFG0 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_31_0_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_31_0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_31_0_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_EFUSE_OVERRIDE_CSS_CFG0_CSS_CFG_31_0_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG0_RESETVAL                          (0x00000000U)

/* EFUSE_OVERRIDE_CSS_CFG1 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG1_EFUSE_OVERRIDE_CSS_CFG1_CSS_CFG_0_63_32_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG1_EFUSE_OVERRIDE_CSS_CFG1_CSS_CFG_0_63_32_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG1_EFUSE_OVERRIDE_CSS_CFG1_CSS_CFG_0_63_32_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG1_EFUSE_OVERRIDE_CSS_CFG1_CSS_CFG_0_63_32_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_CSS_CFG1_RESETVAL                          (0x00000000U)

/* EFUSE_OVERRIDE_DAC_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_MASK (0x1FFF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_EFUSE_OVERRIDE_DAC_TRIM_DAC_TRIM_MAX (0x00001FFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_TRIM_RESETVAL                          (0x00000000U)

/* EFUSE_OVERRIDE_DAC_CFG */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_MASK (0x00030000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_IBIAS_CFG_MAX (0x00000003U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_MASK (0x01000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_EFUSE_OVERRIDE_DAC_CFG_ASYNC_MODE_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_DAC_CFG_RESETVAL                           (0x00000000U)

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

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_TRIM_RESETVAL                      (0x1FD10000U)

/* EFUSE_OVERRIDE_REFBUF0_CFG */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_MASK (0x00007F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_RESETVAL (0x0000002AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_REFBUF0_CFG_MAX (0x0000007FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ADCREF_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0A_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_MASK (0x0F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_EFUSE_OVERRIDE_REFBUF0_CFG_ROK0B_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF0_CFG_RESETVAL                       (0x0DDD2A00U)

/* EFUSE_OVERRIDE_REFBUF1_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_REF_TRIM_MASK (0x0000FF00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_REF_TRIM_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_REF_TRIM_RESETVAL (0x0000001FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_REF_TRIM_MAX (0x000000FFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_OV_TRIM_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_OV_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_OV_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_OV_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_UV_TRIM_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_UV_TRIM_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_UV_TRIM_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_EFUSE_OVERRIDE_REFBUF1_TRIM_ROK1_UV_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_TRIM_RESETVAL                      (0x001F1F00U)

/* EFUSE_OVERRIDE_REFBUF1_CFG */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_REFBUF1_CFG_MASK (0x00007F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_REFBUF1_CFG_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_REFBUF1_CFG_RESETVAL (0x0000002AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_REFBUF1_CFG_MAX (0x0000007FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ADCREF_VSEL_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ADCREF_VSEL_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ADCREF_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ADCREF_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ROK1_VSEL_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ROK1_VSEL_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ROK1_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_EFUSE_OVERRIDE_REFBUF1_CFG_ROK1_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF1_CFG_RESETVAL                       (0x00DD2A00U)

/* EFUSE_OVERRIDE_PMU_CFG */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_MASK (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_BG_DFTC_MAX (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_MASK (0x7F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_EFUSE_OVERRIDE_PMU_CFG_LDO_DFTC_MAX (0x0000007FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_CFG_RESETVAL                           (0x09000000U)

/* EFUSE_OVERRIDE_PMU_SPARE_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_MASK (0x03FF0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_EFUSE_OVERRIDE_PMU_SPARE_TRIM_TRIM_MAX (0x000003FFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PMU_SPARE_TRIM_RESETVAL                    (0x00000000U)

/* EFUSE_OVERRIDE_LDO_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_PROG_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_MASK (0x3F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_EFUSE_OVERRIDE_LDO_TRIM_LDO_TRIM_OFFSET_MAX (0x0000003FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_LDO_TRIM_RESETVAL                          (0x00000000U)

/* EFUSE_OVERRIDE_BG_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_MASK (0x0000FF00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_RESETVAL (0x00000028U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMI_MAX (0x000000FFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_RESETVAL (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMMAG_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_MASK (0x3F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_RESETVAL (0x0000001AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_EFUSE_OVERRIDE_BG_TRIM_BG_TRIMC_MAX (0x0000003FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_BG_TRIM_RESETVAL                           (0x1A082800U)

/* EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM_CTRL_RESETVAL          (0x00000000U)

/* EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_MASK (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_HIGH_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_MASK (0x000000F0U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C1_LOW_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_MASK (0x00000F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_HIGH_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_MASK (0x0000F000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C2_LOW_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_HIGH_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C3_LOW_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_HIGH_TRIM_MASK (0x0F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_HIGH_TRIM_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_HIGH_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_HIGH_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_LOW_TRIM_MASK (0xF0000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_LOW_TRIM_SHIFT (0x0000001CU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_LOW_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_C5_LOW_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM0_RESETVAL              (0x0F0F0F0FU)

/* EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_MASK (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_C8_LOW_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_SFTYMON_THRHLD_TRIM1_RESETVAL              (0x00000000U)

/* EFUSE_OVERRIDE_RCOSC_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_MASK (0x007F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_EFUSE_OVERRIDE_RCOSC_TRIM_FREQ_TRIM_MAX (0x0000007FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_RCOSC_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_SAFETYMON_SPARE */

#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_OVERRIDE_MAX  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_MASK      (0x00FF0000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_SHIFT     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_RESETVAL  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_EFUSE_SAFETYMON_SPARE_VAL_MAX       (0x000000FFU)

#define CSL_TOP_CTRL_EFUSE_SAFETYMON_SPARE_RESETVAL                            (0x00000000U)

/* EFUSE_SPARE_1 */

#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_1_EFUSE_SPARE_1_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_1_RESETVAL                                    (0x00000000U)

/* EFUSE_SPARE_2 */

#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_2_EFUSE_SPARE_2_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_2_RESETVAL                                    (0x00000000U)

/* EFUSE_SPARE_3 */

#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_3_EFUSE_SPARE_3_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_3_RESETVAL                                    (0x00000000U)

/* EFUSE_SPARE_4 */

#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_4_EFUSE_SPARE_4_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_4_RESETVAL                                    (0x00000000U)

/* EFUSE_SPARE_5 */

#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_5_EFUSE_SPARE_5_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_5_RESETVAL                                    (0x00000000U)

/* EFUSE_SPARE_6 */

#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_6_EFUSE_SPARE_6_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_6_RESETVAL                                    (0x00000000U)

/* EFUSE_SPARE_7 */

#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_OVERRIDE_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_OVERRIDE_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_OVERRIDE_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_VAL_MASK                      (0x001F0000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_VAL_SHIFT                     (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_VAL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_SPARE_7_EFUSE_SPARE_7_VAL_MAX                       (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_SPARE_7_RESETVAL                                    (0x00000000U)

/* EFUSE_OVERRIDE_TSENSE_TRIM_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_CTRL_RESETVAL                  (0x00000000U)

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

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_TSENSE_TRIM_RESETVAL                       (0xD0050000U)

/* EFUSE_OVERRIDE_PLL_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_MASK (0x00001F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_CORE_NWELLTRIM_OVERRIDE_VAL_MAX (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_MASK (0x00070000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_MASK (0x1F000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_RESETVAL (0x00000009U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_EFUSE_OVERRIDE_PLL_TRIM_PER_NWELLTRIM_OVERRIDE_VAL_MAX (0x0000001FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_PLL_TRIM_RESETVAL                          (0x09000900U)

/* EFUSE_OVERRIDE_REFBUF2_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_REF_TRIM_MASK (0x0000FF00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_REF_TRIM_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_REF_TRIM_RESETVAL (0x0000001FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_REF_TRIM_MAX (0x000000FFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_OV_TRIM_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_OV_TRIM_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_OV_TRIM_RESETVAL (0x0000000FU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_OV_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_UV_TRIM_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_UV_TRIM_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_UV_TRIM_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_EFUSE_OVERRIDE_REFBUF2_TRIM_ROK2_UV_TRIM_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_TRIM_RESETVAL                      (0x001F1F00U)

/* EFUSE_OVERRIDE_REFBUF2_CFG */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_REFBUF2_CFG_MASK (0x00007F00U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_REFBUF2_CFG_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_REFBUF2_CFG_RESETVAL (0x0000002AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_REFBUF2_CFG_MAX (0x0000007FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ADCREF_VSEL_MASK (0x000F0000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ADCREF_VSEL_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ADCREF_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ADCREF_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ROK2_VSEL_MASK (0x00F00000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ROK2_VSEL_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ROK2_VSEL_RESETVAL (0x0000000DU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_ROK2_VSEL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_SIG_5_U_BG_REF_TRIM_DONE_MASK (0x01000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_SIG_5_U_BG_REF_TRIM_DONE_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_SIG_5_U_BG_REF_TRIM_DONE_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_EFUSE_OVERRIDE_REFBUF2_CFG_SIG_5_U_BG_REF_TRIM_DONE_MAX (0x00000001U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_REFBUF2_CFG_RESETVAL                       (0x01DD2A00U)

/* EFUSE_OVERRIDE_ADCR0_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_ADCR0_TRIM_MASK (0x00FFFFF8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_ADCR0_TRIM_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_ADCR0_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_EFUSE_OVERRIDE_ADCR0_TRIM_ADCR0_TRIM_MAX (0x001FFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR0_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_ADCR1_TRIM */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_ADCR1_TRIM_MASK (0x00FFFFF8U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_ADCR1_TRIM_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_ADCR1_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_EFUSE_OVERRIDE_ADCR1_TRIM_ADCR1_TRIM_MAX (0x001FFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR1_TRIM_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_ADCR_CFG_CTRL */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_MASK (0x00000007U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_RANGE_CTRL_MASK (0x00000038U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_RANGE_CTRL_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_RANGE_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_OVERRIDE_RANGE_CTRL_MAX (0x00000007U)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG_CTRL_RESETVAL                     (0x00000000U)

/* EFUSE_OVERRIDE_ADCR_CFG0 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG0_EFUSE_OVERRIDE_ADCR_CFG0_ADCR_CFG_31_0_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG0_EFUSE_OVERRIDE_ADCR_CFG0_ADCR_CFG_31_0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG0_EFUSE_OVERRIDE_ADCR_CFG0_ADCR_CFG_31_0_RESETVAL (0x514554C9U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG0_EFUSE_OVERRIDE_ADCR_CFG0_ADCR_CFG_31_0_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG0_RESETVAL                         (0x514554C9U)

/* EFUSE_OVERRIDE_ADCR_CFG1 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG1_EFUSE_OVERRIDE_ADCR_CFG1_ADCR_CFG_63_32_MASK (0xFFFFFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG1_EFUSE_OVERRIDE_ADCR_CFG1_ADCR_CFG_63_32_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG1_EFUSE_OVERRIDE_ADCR_CFG1_ADCR_CFG_63_32_RESETVAL (0x219F2908U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG1_EFUSE_OVERRIDE_ADCR_CFG1_ADCR_CFG_63_32_MAX (0xFFFFFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG1_RESETVAL                         (0x219F2908U)

/* EFUSE_OVERRIDE_ADCR_CFG2 */

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG2_EFUSE_OVERRIDE_ADCR_CFG2_ADCR_CFG_86_64_MASK (0x007FFFFFU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG2_EFUSE_OVERRIDE_ADCR_CFG2_ADCR_CFG_86_64_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG2_EFUSE_OVERRIDE_ADCR_CFG2_ADCR_CFG_86_64_RESETVAL (0x0000000AU)
#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG2_EFUSE_OVERRIDE_ADCR_CFG2_ADCR_CFG_86_64_MAX (0x007FFFFFU)

#define CSL_TOP_CTRL_EFUSE_OVERRIDE_ADCR_CFG2_RESETVAL                         (0x0000000AU)

/* ADC_REFBUF0_CTRL */

#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_TOP_CTRL_ADC_REFBUF0_CTRL_RESETVAL                                 (0x00000000U)

/* ADC_REFBUF1_CTRL */

#define CSL_TOP_CTRL_CTRL_ADC_REFBUF1_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_TOP_CTRL_CTRL_ADC_REFBUF1_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_CTRL_ADC_REFBUF1_CTRL_ENABLE_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_CTRL_ADC_REFBUF1_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_TOP_CTRL_ADC_REFBUF1_CTRL_RESETVAL                                 (0x00000000U)

/* ADC_REF_COMP_CTRL */

#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC0_REFOK_EN_MASK    (0x00000007U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC0_REFOK_EN_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC0_REFOK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC0_REFOK_EN_MAX     (0x00000007U)

#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC12_REFOK_EN_MASK   (0x00000070U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC12_REFOK_EN_SHIFT  (0x00000004U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC12_REFOK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC12_REFOK_EN_MAX    (0x00000007U)

#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC34_REFOK_EN_MASK   (0x00000700U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC34_REFOK_EN_SHIFT  (0x00000008U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC34_REFOK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC34_REFOK_EN_MAX    (0x00000007U)

#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADCR01_REFOK_EN_MASK  (0x00007000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADCR01_REFOK_EN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADCR01_REFOK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADCR01_REFOK_EN_MAX   (0x00000007U)

#define CSL_TOP_CTRL_ADC_REF_COMP_CTRL_RESETVAL                                (0x00000000U)

/* ADC_REF_GOOD_STATUS */

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_OV_GOOD_MASK           (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_OV_GOOD_SHIFT          (0x00000000U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_OV_GOOD_RESETVAL       (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_OV_GOOD_MAX            (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_UV_GOOD_MASK           (0x00000002U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_UV_GOOD_SHIFT          (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_UV_GOOD_RESETVAL       (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_UV_GOOD_MAX            (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_OV_GOOD_MASK            (0x00000004U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_OV_GOOD_SHIFT           (0x00000002U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_OV_GOOD_RESETVAL        (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_OV_GOOD_MAX             (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_UV_GOOD_MASK            (0x00000008U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_UV_GOOD_SHIFT           (0x00000003U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_UV_GOOD_RESETVAL        (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_UV_GOOD_MAX             (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_OV_GOOD_MASK           (0x00000010U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_OV_GOOD_SHIFT          (0x00000004U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_OV_GOOD_RESETVAL       (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_OV_GOOD_MAX            (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_UV_GOOD_MASK           (0x00000020U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_UV_GOOD_SHIFT          (0x00000005U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_UV_GOOD_RESETVAL       (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_UV_GOOD_MAX            (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_OV_GOOD_MASK          (0x00000040U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_OV_GOOD_SHIFT         (0x00000006U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_OV_GOOD_RESETVAL      (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_OV_GOOD_MAX           (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_UV_GOOD_MASK          (0x00000080U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_UV_GOOD_SHIFT         (0x00000007U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_UV_GOOD_RESETVAL      (0x00000001U)
#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_UV_GOOD_MAX           (0x00000001U)

#define CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_RESETVAL                              (0x000000FFU)

/* VMON_CTRL */

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_MASK                          (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_SHIFT                         (0x00000000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP0_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP1_EN_MASK                          (0x00000070U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP1_EN_SHIFT                         (0x00000004U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP1_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP1_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP2_EN_MASK                          (0x00000700U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP2_EN_SHIFT                         (0x00000008U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP2_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP2_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP3_EN_MASK                          (0x00007000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP3_EN_SHIFT                         (0x0000000CU)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP3_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP3_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP5_EN_MASK                          (0x00070000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP5_EN_SHIFT                         (0x00000010U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP5_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP5_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_MASK                          (0x00700000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_SHIFT                         (0x00000014U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP7_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_MASK                          (0x07000000U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_SHIFT                         (0x00000018U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_RESETVAL                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_CTRL_VMON_CTRL_CMP8_EN_MAX                           (0x00000007U)

#define CSL_TOP_CTRL_VMON_CTRL_RESETVAL                                        (0x07777777U)

/* VMON_STAT */

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_MASK                       (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP0_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_MASK                       (0x00000002U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_SHIFT                      (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_OV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_MASK                       (0x00000004U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_SHIFT                      (0x00000002U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP1_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_MASK                       (0x00000008U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_SHIFT                      (0x00000003U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_OV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_MASK                       (0x00000010U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_SHIFT                      (0x00000004U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP2_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_MASK                       (0x00000020U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_SHIFT                      (0x00000005U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_OV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_MASK                       (0x00000040U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_SHIFT                      (0x00000006U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP3_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_MASK                       (0x00000080U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_SHIFT                      (0x00000007U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_OV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_MASK                       (0x00000100U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_SHIFT                      (0x00000008U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP5_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_MASK                       (0x00000200U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_SHIFT                      (0x00000009U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP7_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_MASK                       (0x00000400U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_SHIFT                      (0x0000000AU)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_RESETVAL                   (0x00000001U)
#define CSL_TOP_CTRL_VMON_STAT_VMON_STAT_CMP8_UV_OK_MAX                        (0x00000001U)

#define CSL_TOP_CTRL_VMON_STAT_RESETVAL                                        (0x000007FFU)

/* PMU_COARSE_STAT */

#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_MASK               (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_RESETVAL           (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_BG_RDY_MAX                (0x00000001U)

#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_MASK              (0x00000002U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_SHIFT             (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_RESETVAL          (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_LDO_RDY_MAX               (0x00000001U)

#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_MASK            (0x00000004U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_SHIFT           (0x00000002U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_RESETVAL        (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VCORE_RDY_MAX             (0x00000001U)

#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_MASK           (0x00000008U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_SHIFT          (0x00000003U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_RESETVAL       (0x00000001U)
#define CSL_TOP_CTRL_PMU_COARSE_STAT_PMU_COARSE_STAT_VSUP18_RDY_MAX            (0x00000001U)

#define CSL_TOP_CTRL_PMU_COARSE_STAT_RESETVAL                                  (0x0000000FU)

/* MASK_VMON_ERROR_ESM_H */

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

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_OV_MASK_MASK (0x00000800U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_OV_MASK_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_UV_MASK_MASK (0x00001000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_UV_MASK_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC0_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_OV_MASK_MASK (0x00002000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_OV_MASK_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_UV_MASK_MASK (0x00004000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_UV_MASK_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC12_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_OV_MASK_MASK (0x00008000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_OV_MASK_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_UV_MASK_MASK (0x00010000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_UV_MASK_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADC34_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_OV_MASK_MASK (0x00020000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_OV_MASK_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_UV_MASK_MASK (0x00040000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_UV_MASK_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_MASK_VMON_ERROR_ESM_H_ADCR01_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_H_RESETVAL                            (0x0007FFFFU)

/* MASK_VMON_ERROR_ESM_L */

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

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_OV_MASK_MASK (0x00000800U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_OV_MASK_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_UV_MASK_MASK (0x00001000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_UV_MASK_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC0_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_OV_MASK_MASK (0x00002000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_OV_MASK_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_UV_MASK_MASK (0x00004000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_UV_MASK_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC12_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_OV_MASK_MASK (0x00008000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_OV_MASK_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_UV_MASK_MASK (0x00010000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_UV_MASK_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADC34_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_OV_MASK_MASK (0x00020000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_OV_MASK_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_OV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_OV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_UV_MASK_MASK (0x00040000U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_UV_MASK_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_UV_MASK_RESETVAL (0x00000001U)
#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_MASK_VMON_ERROR_ESM_L_ADCR01_REF_UV_MASK_MAX (0x00000001U)

#define CSL_TOP_CTRL_MASK_VMON_ERROR_ESM_L_RESETVAL                            (0x0007FFFFU)

/* MASK_ANA_ISO */

#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_MASK                       (0x00000007U)
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_MAX                        (0x00000007U)

#define CSL_TOP_CTRL_MASK_ANA_ISO_RESETVAL                                     (0x00000000U)

/* VMON_FILTER_CTRL */

#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_MASK       (0x00000003U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_VMON_FILTER_CTRL_VMON_FILTER_CTRL_SELECT_VALUE_MAX        (0x00000003U)

#define CSL_TOP_CTRL_VMON_FILTER_CTRL_RESETVAL                                 (0x00000000U)

/* ADC_RNG_CTRL */

#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_MODE_MASK                       (0x0000007FU)
#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_MODE_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_MODE_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_MODE_MAX                        (0x0000007FU)

#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_MASK                (0x00000300U)
#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_SHIFT               (0x00000008U)
#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_ADC_RNG_CTRL_ADC_RNG_CTRL_SCALED_MODE_MAX                 (0x00000003U)

#define CSL_TOP_CTRL_ADC_RNG_CTRL_RESETVAL                                     (0x00000000U)

/* ADC0_OSD_CHEN */

#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_MASK                (0x0000003FU)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_MAX                 (0x0000003FU)

#define CSL_TOP_CTRL_ADC0_OSD_CHEN_RESETVAL                                    (0x00000000U)

/* ADC1_OSD_CHEN */

#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_MASK                (0x0000003FU)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CHEN_ADC1_OSD_CHEN_CH_OSD_EN_MAX                 (0x0000003FU)

#define CSL_TOP_CTRL_ADC1_OSD_CHEN_RESETVAL                                    (0x00000000U)

/* ADC2_OSD_CHEN */

#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_MASK                (0x0000003FU)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CHEN_ADC2_OSD_CHEN_CH_OSD_EN_MAX                 (0x0000003FU)

#define CSL_TOP_CTRL_ADC2_OSD_CHEN_RESETVAL                                    (0x00000000U)

/* ADC3_OSD_CHEN */

#define CSL_TOP_CTRL_ADC3_OSD_CHEN_ADC3_OSD_CHEN_CH_OSD_EN_MASK                (0x0000003FU)
#define CSL_TOP_CTRL_ADC3_OSD_CHEN_ADC3_OSD_CHEN_CH_OSD_EN_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_ADC3_OSD_CHEN_ADC3_OSD_CHEN_CH_OSD_EN_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_ADC3_OSD_CHEN_ADC3_OSD_CHEN_CH_OSD_EN_MAX                 (0x0000003FU)

#define CSL_TOP_CTRL_ADC3_OSD_CHEN_RESETVAL                                    (0x00000000U)

/* ADC4_OSD_CHEN */

#define CSL_TOP_CTRL_ADC4_OSD_CHEN_ADC4_OSD_CHEN_CH_OSD_EN_MASK                (0x0000003FU)
#define CSL_TOP_CTRL_ADC4_OSD_CHEN_ADC4_OSD_CHEN_CH_OSD_EN_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_ADC4_OSD_CHEN_ADC4_OSD_CHEN_CH_OSD_EN_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_ADC4_OSD_CHEN_ADC4_OSD_CHEN_CH_OSD_EN_MAX                 (0x0000003FU)

#define CSL_TOP_CTRL_ADC4_OSD_CHEN_RESETVAL                                    (0x00000000U)

/* ADC0_OSD_CTRL */

#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_ADC0_OSD_CTRL_RESETVAL                                    (0x00000000U)

/* ADC1_OSD_CTRL */

#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC1_OSD_CTRL_ADC1_OSD_CTRL_FUNCTION_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_ADC1_OSD_CTRL_RESETVAL                                    (0x00000000U)

/* ADC2_OSD_CTRL */

#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC2_OSD_CTRL_ADC2_OSD_CTRL_FUNCTION_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_ADC2_OSD_CTRL_RESETVAL                                    (0x00000000U)

/* ADC3_OSD_CTRL */

#define CSL_TOP_CTRL_ADC3_OSD_CTRL_ADC3_OSD_CTRL_FUNCTION_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_ADC3_OSD_CTRL_ADC3_OSD_CTRL_FUNCTION_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC3_OSD_CTRL_ADC3_OSD_CTRL_FUNCTION_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC3_OSD_CTRL_ADC3_OSD_CTRL_FUNCTION_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_ADC3_OSD_CTRL_RESETVAL                                    (0x00000000U)

/* ADC4_OSD_CTRL */

#define CSL_TOP_CTRL_ADC4_OSD_CTRL_ADC4_OSD_CTRL_FUNCTION_MASK                 (0x00000007U)
#define CSL_TOP_CTRL_ADC4_OSD_CTRL_ADC4_OSD_CTRL_FUNCTION_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_ADC4_OSD_CTRL_ADC4_OSD_CTRL_FUNCTION_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_ADC4_OSD_CTRL_ADC4_OSD_CTRL_FUNCTION_MAX                  (0x00000007U)

#define CSL_TOP_CTRL_ADC4_OSD_CTRL_RESETVAL                                    (0x00000000U)

/* ADC_LOOPBACK_CTRL */

#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_MASK  (0x00000001U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_MAX   (0x00000001U)

#define CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_RESETVAL                                (0x00000000U)

/* CMPSSA_LOOPBACK_CTRL */

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

#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_MASK (0x00000200U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_MAX (0x00000001U)

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

#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_MASK (0x02000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_RESETVAL                             (0x00000000U)

/* CMPSSB_LOOPBACK_CTRL */

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_MASK (0x00000002U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL1_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_MASK (0x00000004U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL2_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_MASK (0x00000008U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL3_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_MASK (0x00000010U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL4_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_MASK (0x00000020U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL5_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_MASK (0x00000040U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL6_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_MASK (0x00000080U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL7_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_MASK (0x00000100U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL8_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_MASK (0x00000200U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL9_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_MASK (0x00010000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_MASK (0x00020000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH1_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_MASK (0x00040000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH2_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_MASK (0x00080000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_SHIFT (0x00000013U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH3_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_MASK (0x00100000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH4_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_MASK (0x00200000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_SHIFT (0x00000015U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH5_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_MASK (0x00400000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_SHIFT (0x00000016U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH6_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_MASK (0x00800000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_SHIFT (0x00000017U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH7_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_MASK (0x01000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH8_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_MASK (0x02000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH9_LOOPBACK_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_RESETVAL                             (0x00000000U)

/* ADCR01_OSD_CHEN */

#define CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_MASK            (0x000000FFU)
#define CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_MAX             (0x000000FFU)

#define CSL_TOP_CTRL_ADCR01_OSD_CHEN_RESETVAL                                  (0x00000000U)

/* ADCR01_OSD_CTRL */

#define CSL_TOP_CTRL_ADCR01_OSD_CTRL_ADCR01_OSD_CTRL_FUNCTION_MASK             (0x00000007U)
#define CSL_TOP_CTRL_ADCR01_OSD_CTRL_ADCR01_OSD_CTRL_FUNCTION_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_ADCR01_OSD_CTRL_ADCR01_OSD_CTRL_FUNCTION_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_ADCR01_OSD_CTRL_ADCR01_OSD_CTRL_FUNCTION_MAX              (0x00000007U)

#define CSL_TOP_CTRL_ADCR01_OSD_CTRL_RESETVAL                                  (0x00000000U)

/* ADC_REFBUF2_CTRL */

#define CSL_TOP_CTRL_ADC_REFBUF2_CTRL_ADC_REFBUF2_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_TOP_CTRL_ADC_REFBUF2_CTRL_ADC_REFBUF2_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_TOP_CTRL_ADC_REFBUF2_CTRL_ADC_REFBUF2_CTRL_ENABLE_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_ADC_REFBUF2_CTRL_ADC_REFBUF2_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_TOP_CTRL_ADC_REFBUF2_CTRL_RESETVAL                                 (0x00000000U)

/* TB_CTRL_ADC5_ADC6_RESERVED */

#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT0_MASK (0x00000001U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT0_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT0_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT0_MAX (0x00000001U)

#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT1_MASK (0x00000002U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT1_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT1_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_TB_CTRL_ADC5_ADC6_RESERVED_BIT1_MAX (0x00000001U)

#define CSL_TOP_CTRL_TB_CTRL_ADC5_ADC6_RESERVED_RESETVAL                       (0x00000000U)

/* TSENSE_CFG */

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_MASK                         (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_SHIFT                        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_RESETVAL                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE_MAX                          (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_MASK                     (0x000000F0U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_SHIFT                    (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL_MAX                      (0x0000000FU)

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_MASK                          (0x00003F00U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_SHIFT                         (0x00000008U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_RESETVAL                      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY_MAX                           (0x0000003FU)

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_MASK                    (0x00010000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_SHIFT                   (0x00000010U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_RESETVAL                (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SNSR_MX_HIZ_MAX                     (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_MASK                         (0x00100000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_SHIFT                        (0x00000014U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_RESETVAL                     (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF_MAX                          (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_MASK                         (0x01000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_SHIFT                        (0x00000018U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_RESETVAL                     (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF_MAX                          (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_MASK                        (0x10000000U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_SHIFT                       (0x0000001CU)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_RESETVAL                    (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF_MAX                         (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_CFG_RESETVAL                                       (0x11110000U)

/* TSENSE_STATUS */

#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_MASK            (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_SHIFT           (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD_MAX             (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_MASK                  (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_SHIFT                 (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD_MAX                   (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_MASK                   (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_SHIFT                  (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_MASK            (0x00000010U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_SHIFT           (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD_MAX             (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_MASK                  (0x00000020U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_SHIFT                 (0x00000005U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD_MAX                   (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_MASK                   (0x00000040U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_SHIFT                  (0x00000006U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RESETVAL                                    (0x00000000U)

/* TSENSE_STATUS_RAW */

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_MASK    (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_LOW_THRHLD_MAX     (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_MASK          (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_SHIFT         (0x00000001U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_COLD_MAX           (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_MASK           (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_SHIFT          (0x00000002U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S0_HOT_MAX            (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_MASK    (0x00000010U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_SHIFT   (0x00000004U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_LOW_THRHLD_MAX     (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_MASK          (0x00000020U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_SHIFT         (0x00000005U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_COLD_MAX           (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_MASK           (0x00000040U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_SHIFT          (0x00000006U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_TSENSE_STATUS_RAW_S1_HOT_MAX            (0x00000001U)

#define CSL_TOP_CTRL_TSENSE_STATUS_RAW_RESETVAL                                (0x00000000U)

/* TSENSE0_TSHUT */

#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_MASK       (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD_MAX        (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_MASK         (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_SHIFT        (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT_MAX          (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_MASK           (0xE0000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_SHIFT          (0x0000001DU)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE_MAX            (0x00000007U)

#define CSL_TOP_CTRL_TSENSE0_TSHUT_RESETVAL                                    (0x00000000U)

/* TSENSE0_ALERT */

#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_MASK         (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_SHIFT        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT_MAX          (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_MASK        (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_SHIFT       (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD_MAX         (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_ALERT_RESETVAL                                    (0x00000000U)

/* TSENSE0_CNTL */

#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_MASK                 (0x00000001U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_CLEAR_MAX                  (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_MASK                (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_SHIFT               (0x00000004U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_FIFO_FREEZE_MAX                 (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_MASK                 (0x00000100U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_SHIFT                (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_ACCU_CLEAR_MAX                  (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_MASK                  (0x00010000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_SHIFT                 (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD_MAX                   (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_MASK                   (0x00100000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_SHIFT                  (0x00000014U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_RESETVAL               (0x00000001U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_MASK            (0x01000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_SHIFT           (0x00000018U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD_MAX             (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_CNTL_RESETVAL                                     (0x00100000U)

/* TSENSE0_RESULT */

#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_MASK                  (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_RESETVAL              (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP_MAX                   (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_MASK                   (0x00010000U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_SHIFT                  (0x00000010U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_ECOZ_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE0_RESULT_RESETVAL                                   (0x000000FFU)

/* TSENSE0_DATA0 */

#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_RESETVAL                 (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA0_TSENSE0_DATA0_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE0_DATA0_RESETVAL                                    (0x000000FFU)

/* TSENSE0_DATA1 */

#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA1_TSENSE0_DATA1_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE0_DATA1_RESETVAL                                    (0x00000000U)

/* TSENSE0_DATA2 */

#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA2_TSENSE0_DATA2_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE0_DATA2_RESETVAL                                    (0x00000000U)

/* TSENSE0_DATA3 */

#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_DATA3_TSENSE0_DATA3_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE0_DATA3_RESETVAL                                    (0x00000000U)

/* TSENSE0_ACCU */

#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_MASK                      (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE0_ACCU_TSENSE0_ACCU_CUMUL_MAX                       (0xFFFFFFFFU)

#define CSL_TOP_CTRL_TSENSE0_ACCU_RESETVAL                                     (0x00000000U)

/* TSENSE1_TSHUT */

#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_MASK       (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_SHIFT      (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_RESETVAL   (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD_MAX        (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_MASK         (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_SHIFT        (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT_MAX          (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_MASK           (0xE0000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_SHIFT          (0x0000001DU)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE_MAX            (0x00000007U)

#define CSL_TOP_CTRL_TSENSE1_TSHUT_RESETVAL                                    (0x00000000U)

/* TSENSE1_ALERT */

#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_MASK         (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_SHIFT        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_RESETVAL     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT_MAX          (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_MASK        (0x00FF0000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_SHIFT       (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD_MAX         (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_ALERT_RESETVAL                                    (0x00000000U)

/* TSENSE1_CNTL */

#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_MASK                 (0x00000001U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_SHIFT                (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_CLEAR_MAX                  (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_MASK                (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_SHIFT               (0x00000004U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_FIFO_FREEZE_MAX                 (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_MASK                 (0x00000100U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_SHIFT                (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_RESETVAL             (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_ACCU_CLEAR_MAX                  (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_MASK                  (0x00010000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_SHIFT                 (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD_MAX                   (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_MASK                   (0x00100000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_SHIFT                  (0x00000014U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_RESETVAL               (0x00000001U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_MASK            (0x01000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_SHIFT           (0x00000018U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD_MAX             (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_CNTL_RESETVAL                                     (0x00100000U)

/* TSENSE1_RESULT */

#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_MASK                  (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_RESETVAL              (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP_MAX                   (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_MASK                   (0x00010000U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_SHIFT                  (0x00000010U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_ECOZ_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE1_RESULT_RESETVAL                                   (0x000000FFU)

/* TSENSE1_DATA0 */

#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_RESETVAL                 (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA0_TSENSE1_DATA0_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE1_DATA0_RESETVAL                                    (0x000000FFU)

/* TSENSE1_DATA1 */

#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA1_TSENSE1_DATA1_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE1_DATA1_RESETVAL                                    (0x00000000U)

/* TSENSE1_DATA2 */

#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA2_TSENSE1_DATA2_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE1_DATA2_RESETVAL                                    (0x00000000U)

/* TSENSE1_DATA3 */

#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_MASK                     (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_SHIFT                    (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_RESETVAL                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_DATA_MAX                      (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_MASK                      (0xFFFFFF00U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_DATA3_TSENSE1_DATA3_TAG_MAX                       (0x00FFFFFFU)

#define CSL_TOP_CTRL_TSENSE1_DATA3_RESETVAL                                    (0x00000000U)

/* TSENSE1_ACCU */

#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_MASK                      (0xFFFFFFFFU)
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_SHIFT                     (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_TSENSE1_ACCU_TSENSE1_ACCU_CUMUL_MAX                       (0xFFFFFFFFU)

#define CSL_TOP_CTRL_TSENSE1_ACCU_RESETVAL                                     (0x00000000U)

/* TSENSE2_RESULT */

#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_MASK                  (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP_MAX                   (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_MASK                   (0x00010000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_SHIFT                  (0x00000010U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_ECOZ_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE2_RESULT_RESETVAL                                   (0x00000000U)

/* TSENSE3_RESULT */

#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_MASK                  (0x000000FFU)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_SHIFT                 (0x00000000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_RESETVAL              (0x00000000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP_MAX                   (0x000000FFU)

#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_MASK                   (0x00010000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_SHIFT                  (0x00000010U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_ECOZ_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_TSENSE3_RESULT_RESETVAL                                   (0x00000000U)

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

#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC3_CSS_MASK (0x00000008U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC3_CSS_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC3_CSS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC3_CSS_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC4_CSS_MASK (0x00000010U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC4_CSS_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC4_CSS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADC4_CSS_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADCR01_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADCR01_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADCR01_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_DFT_ATB_GLOBALEN_ADC_CSS_ATB_GLOBALEN_ADCR01_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB_GLOBALEN_ADC_CSS_RESETVAL                         (0x00000000U)

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

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC3_ATB0_MASTEREN_MASK (0x00000008U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC3_ATB0_MASTEREN_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC3_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC3_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC4_ATB0_MASTEREN_MASK (0x00000010U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC4_ATB0_MASTEREN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC4_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADC4_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB0_MASTEREN_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB0_MASTEREN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB0_MASTEREN_MASK (0x00000080U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB0_MASTEREN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB0_MASTEREN_MASK (0x00000100U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB0_MASTEREN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB0_MASTEREN_MASK (0x00000200U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB0_MASTEREN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB0_MASTEREN_MASK (0x00000800U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB0_MASTEREN_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB0_MASTEREN_MASK (0x00001000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB0_MASTEREN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB0_MASTEREN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB0_MASTEREN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB0_MASTEREN_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB0_MASTEREN_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB0_MASTEREN_MASK (0x00008000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB0_MASTEREN_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB0_MASTEREN_MASK (0x00010000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB0_MASTEREN_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB0_MASTEREN_MASK (0x00020000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB0_MASTEREN_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB0_MASTEREN_MASK (0x00040000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB0_MASTEREN_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB0_MASTEREN_MASK (0x00080000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB0_MASTEREN_SHIFT (0x00000013U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB0_MASTEREN_MASK (0x00100000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB0_MASTEREN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB0_MASTEREN_MASK (0x00200000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB0_MASTEREN_SHIFT (0x00000015U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB0_MASTEREN_MASK (0x00400000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB0_MASTEREN_SHIFT (0x00000016U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB0_MASTEREN_MASK (0x00800000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB0_MASTEREN_SHIFT (0x00000017U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB0_MASTEREN_MASK (0x01000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB0_MASTEREN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_MASK (0x02000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DAC_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR0_ATB0_MASTEREN_MASK (0x04000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR0_ATB0_MASTEREN_SHIFT (0x0000001AU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR0_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR0_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR1_ATB0_MASTEREN_MASK (0x08000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR1_ATB0_MASTEREN_SHIFT (0x0000001BU)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR1_ATB0_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_DFT_ATB0_MASTEREN_ADC_CSS_DAC_ADCR1_ATB0_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB0_MASTEREN_ADC_CSS_DAC_RESETVAL                    (0x00000000U)

/* DFT_ATB1_MASTEREN_ADC_CSS_DAC */

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB1_MASTEREN_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB1_MASTEREN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB1_MASTEREN_MASK (0x00000080U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB1_MASTEREN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB1_MASTEREN_MASK (0x00000100U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB1_MASTEREN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB1_MASTEREN_MASK (0x00000200U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB1_MASTEREN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0A1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB1_MASTEREN_MASK (0x00000800U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB1_MASTEREN_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1A1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB1_MASTEREN_MASK (0x00001000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB1_MASTEREN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2A1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB1_MASTEREN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB1_MASTEREN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3A1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB1_MASTEREN_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB1_MASTEREN_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4A1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB1_MASTEREN_MASK (0x00008000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB1_MASTEREN_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB1_MASTEREN_MASK (0x00010000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB1_MASTEREN_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB1_MASTEREN_MASK (0x00020000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB1_MASTEREN_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB1_MASTEREN_MASK (0x00040000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB1_MASTEREN_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB1_MASTEREN_MASK (0x00080000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB1_MASTEREN_SHIFT (0x00000013U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B0_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB1_MASTEREN_MASK (0x00100000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB1_MASTEREN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS0B1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB1_MASTEREN_MASK (0x00200000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB1_MASTEREN_SHIFT (0x00000015U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS1B1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB1_MASTEREN_MASK (0x00400000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB1_MASTEREN_SHIFT (0x00000016U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS2B1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB1_MASTEREN_MASK (0x00800000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB1_MASTEREN_SHIFT (0x00000017U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS3B1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB1_MASTEREN_MASK (0x01000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB1_MASTEREN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_CSS4B1_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_MASK (0x02000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DFT_ATB1_MASTEREN_ADC_CSS_DAC_DAC_ATB1_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ATB1_MASTEREN_ADC_CSS_DAC_RESETVAL                    (0x00000000U)

/* DFT_PMU_REFSYS_SAFETY */

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_MASK (0x00000007U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_PMU_REFSYS_MAX (0x00000007U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_MASK (0x00000008U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SYSBOOT_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_MASK (0x00000010U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_LDO_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_BG_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_TS_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_MASK (0x00000180U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_EN_SAFETYCOMP_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_MASK (0x00000E00U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_ATB_MUX_SEL_SAFETYCOMP_MAX (0x00000007U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_MASK  (0x00001000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_LDO_DIS_MAX   (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_FLIPEN_CTRL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_MASK   (0x00004000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_SHIFT  (0x0000000EU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_BG_DIS_MAX    (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_MASK (0x00008000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_DFT_PMU_REFSYS_SAFETY_RCOSC_STOP_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_PMU_REFSYS_SAFETY_RESETVAL                            (0x00000000U)

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

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC3_DTB_MASTEREN_MASK (0x00000008U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC3_DTB_MASTEREN_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC3_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC3_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC4_DTB_MASTEREN_MASK (0x00000010U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC4_DTB_MASTEREN_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC4_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADC4_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_MASK (0x00000020U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A0_DTB_MASTEREN_MASK (0x00000040U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A0_DTB_MASTEREN_SHIFT (0x00000006U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A0_DTB_MASTEREN_MASK (0x00000080U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A0_DTB_MASTEREN_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A0_DTB_MASTEREN_MASK (0x00000100U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A0_DTB_MASTEREN_SHIFT (0x00000008U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A0_DTB_MASTEREN_MASK (0x00000200U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A0_DTB_MASTEREN_SHIFT (0x00000009U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0A1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A1_DTB_MASTEREN_MASK (0x00000800U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A1_DTB_MASTEREN_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1A1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A1_DTB_MASTEREN_MASK (0x00001000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A1_DTB_MASTEREN_SHIFT (0x0000000CU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2A1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A1_DTB_MASTEREN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A1_DTB_MASTEREN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3A1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A1_DTB_MASTEREN_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A1_DTB_MASTEREN_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4A1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B0_DTB_MASTEREN_MASK (0x00008000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B0_DTB_MASTEREN_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B0_DTB_MASTEREN_MASK (0x00010000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B0_DTB_MASTEREN_SHIFT (0x00000010U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B0_DTB_MASTEREN_MASK (0x00020000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B0_DTB_MASTEREN_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B0_DTB_MASTEREN_MASK (0x00040000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B0_DTB_MASTEREN_SHIFT (0x00000012U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B0_DTB_MASTEREN_MASK (0x00080000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B0_DTB_MASTEREN_SHIFT (0x00000013U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B1_DTB_MASTEREN_MASK (0x00100000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B1_DTB_MASTEREN_SHIFT (0x00000014U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS0B1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B1_DTB_MASTEREN_MASK (0x00200000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B1_DTB_MASTEREN_SHIFT (0x00000015U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS1B1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B1_DTB_MASTEREN_MASK (0x00400000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B1_DTB_MASTEREN_SHIFT (0x00000016U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS2B1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B1_DTB_MASTEREN_MASK (0x00800000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B1_DTB_MASTEREN_SHIFT (0x00000017U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS3B1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B1_DTB_MASTEREN_MASK (0x01000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B1_DTB_MASTEREN_SHIFT (0x00000018U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_CSS4B1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_MASK (0x02000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_SHIFT (0x00000019U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_DAC_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR0_DTB_MASTEREN_MASK (0x04000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR0_DTB_MASTEREN_SHIFT (0x0000001AU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR0_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR0_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR1_DTB_MASTEREN_MASK (0x08000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR1_DTB_MASTEREN_SHIFT (0x0000001BU)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR1_DTB_MASTEREN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_DFT_ANA_DTB_ENABLES_ADCR1_DTB_MASTEREN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ANA_DTB_ENABLES_RESETVAL                              (0x00000000U)

/* DFT_ADC_CHSEL_OV_CTRL_VALUE */

#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_MASK (0x000003FFU)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_MAX (0x000003FFU)

#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_MASK (0x00000400U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_SHIFT (0x0000000AU)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_DFT_ADC_CHSEL_OV_CTRL_VALUE_ADC_CHSEL_OV_CTRL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ADC_CHSEL_OV_CTRL_VALUE_RESETVAL                      (0x00000000U)

/* DFT_DAC_CTRL */

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_MASK              (0x00000001U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_SHIFT             (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VTOP_ATB_MAX               (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_MASK              (0x00000002U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_SHIFT             (0x00000001U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VBOT_ATB_MAX               (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_MASK              (0x00000004U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_SHIFT             (0x00000002U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VFEEDBACK_ATB_MAX               (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_MASK                    (0x00000008U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_SHIFT                   (0x00000003U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_VSS_ATB_MAX                     (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_MASK          (0x00000010U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_SHIFT         (0x00000004U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_LOW_ATB_MAX           (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_MASK           (0x00000020U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_SHIFT          (0x00000005U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_RDAC_VREF_HI_ATB_MAX            (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_MASK             (0x00000040U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_SHIFT            (0x00000006U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_IBIAS_20UA_ATB_MAX              (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_MASK            (0x00000080U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_SHIFT           (0x00000007U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_RESETVAL        (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_LOW_EN_MAX             (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_MASK           (0x00000100U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_SHIFT          (0x00000008U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_DFT_DAC_CTRL_DFT_DAC_CTRL_DFT_LOAD_HIGH_EN_MAX            (0x00000001U)

#define CSL_TOP_CTRL_DFT_DAC_CTRL_RESETVAL                                     (0x00000000U)

/* DFT_CSS01_CTRL */

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_VAL_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_VAL_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_ATB_SELECT_MASK        (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_ATB_SELECT_SHIFT       (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_SELECT_MASK        (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_SELECT_SHIFT       (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_MUX_CONFIG_MASK    (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_MUX_CONFIG_SHIFT   (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS0_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_EN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_EN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_VAL_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_VAL_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_MASK (0x00018000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_MASK (0x001E0000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_ATB_SELECT_MASK        (0x00600000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_ATB_SELECT_SHIFT       (0x00000015U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_SELECT_MASK        (0x00800000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_SELECT_SHIFT       (0x00000017U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_MUX_CONFIG_MASK    (0x03000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_MUX_CONFIG_SHIFT   (0x00000018U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS01_CTRL_DFT_CSS01_CTRL_CSS1_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS01_CTRL_RESETVAL                                   (0x00000000U)

/* DFT_CSS23_CTRL */

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_VAL_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_VAL_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_ATB_SELECT_MASK        (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_ATB_SELECT_SHIFT       (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_SELECT_MASK        (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_SELECT_SHIFT       (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_MUX_CONFIG_MASK    (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_MUX_CONFIG_SHIFT   (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS2_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_EN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_EN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_VAL_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_VAL_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_SUP_MUX_CONTROL_MASK (0x00018000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_ANA_MUX_CONTROL_MASK (0x001E0000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_ATB_SELECT_MASK        (0x00600000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_ATB_SELECT_SHIFT       (0x00000015U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_SELECT_MASK        (0x00800000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_SELECT_SHIFT       (0x00000017U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_MUX_CONFIG_MASK    (0x03000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_MUX_CONFIG_SHIFT   (0x00000018U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS23_CTRL_DFT_CSS23_CTRL_CSS3_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS23_CTRL_RESETVAL                                   (0x00000000U)

/* DFT_CSS45_CTRL */

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_VAL_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_VAL_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_ATB_SELECT_MASK        (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_ATB_SELECT_SHIFT       (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_SELECT_MASK        (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_SELECT_SHIFT       (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_MUX_CONFIG_MASK    (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_MUX_CONFIG_SHIFT   (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS4_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_EN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_EN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_VAL_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_VAL_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_SUP_MUX_CONTROL_MASK (0x00018000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_ANA_MUX_CONTROL_MASK (0x001E0000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_ATB_SELECT_MASK        (0x00600000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_ATB_SELECT_SHIFT       (0x00000015U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_SELECT_MASK        (0x00800000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_SELECT_SHIFT       (0x00000017U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_MUX_CONFIG_MASK    (0x03000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_MUX_CONFIG_SHIFT   (0x00000018U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS45_CTRL_DFT_CSS45_CTRL_CSS5_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS45_CTRL_RESETVAL                                   (0x00000000U)

/* DFT_CSS67_CTRL */

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_VAL_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_VAL_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_ATB_SELECT_MASK        (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_ATB_SELECT_SHIFT       (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_SELECT_MASK        (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_SELECT_SHIFT       (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_MUX_CONFIG_MASK    (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_MUX_CONFIG_SHIFT   (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS6_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_EN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_EN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_VAL_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_VAL_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_SUP_MUX_CONTROL_MASK (0x00018000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_ANA_MUX_CONTROL_MASK (0x001E0000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_ATB_SELECT_MASK        (0x00600000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_ATB_SELECT_SHIFT       (0x00000015U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_SELECT_MASK        (0x00800000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_SELECT_SHIFT       (0x00000017U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_MUX_CONFIG_MASK    (0x03000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_MUX_CONFIG_SHIFT   (0x00000018U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS67_CTRL_DFT_CSS67_CTRL_CSS7_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS67_CTRL_RESETVAL                                   (0x00000000U)

/* DFT_CSS89_CTRL */

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_VAL_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_VAL_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_SUP_MUX_CONTROL_MASK (0x0000000CU)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_ANA_MUX_CONTROL_MASK (0x000000F0U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000004U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_ATB_SELECT_MASK        (0x00000300U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_ATB_SELECT_SHIFT       (0x00000008U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_SELECT_MASK        (0x00000400U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_SELECT_SHIFT       (0x0000000AU)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_MUX_CONFIG_MASK    (0x00001800U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_MUX_CONFIG_SHIFT   (0x0000000BU)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS8_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_EN_MASK (0x00002000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_EN_SHIFT (0x0000000DU)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_EN_MAX  (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_VAL_MASK (0x00004000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_VAL_SHIFT (0x0000000EU)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_COMPOUT_BYPASS_VAL_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_SUP_MUX_CONTROL_MASK (0x00018000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_SUP_MUX_CONTROL_SHIFT (0x0000000FU)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_SUP_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_SUP_MUX_CONTROL_MAX (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_ANA_MUX_CONTROL_MASK (0x001E0000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_ANA_MUX_CONTROL_SHIFT (0x00000011U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_ANA_MUX_CONTROL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_TESTANA_L_ANA_MUX_CONTROL_MAX (0x0000000FU)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_ATB_SELECT_MASK        (0x00600000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_ATB_SELECT_SHIFT       (0x00000015U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_ATB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_ATB_SELECT_MAX         (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_SELECT_MASK        (0x00800000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_SELECT_SHIFT       (0x00000017U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_SELECT_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_SELECT_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_MUX_CONFIG_MASK    (0x03000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_MUX_CONFIG_SHIFT   (0x00000018U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_MUX_CONFIG_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_CSS89_CTRL_DFT_CSS89_CTRL_CSS9_DTB_MUX_CONFIG_MAX     (0x00000003U)

#define CSL_TOP_CTRL_DFT_CSS89_CTRL_RESETVAL                                   (0x00000000U)

/* DFT_RAMP_DACL */

#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_MASK                (0x000FFFFFU)
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_SHIFT               (0x00000000U)
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_RESETVAL            (0x00000000U)
#define CSL_TOP_CTRL_DFT_RAMP_DACL_DFT_RAMP_DACL_RAMP_DACL_MAX                 (0x000FFFFFU)

#define CSL_TOP_CTRL_DFT_RAMP_DACL_RESETVAL                                    (0x00000000U)

/* DFT_REFBUF_CTRL */

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_MASK  (0x00000001U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0B_MAX   (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_MASK   (0x00000002U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_SHIFT  (0x00000001U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_ROK0_MAX    (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_MASK        (0x00000004U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_SHIFT       (0x00000002U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATBEN_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_ROK1_MASK   (0x00000008U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_ROK1_SHIFT  (0x00000003U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_ROK1_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_ROK1_MAX    (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_MASK        (0x00000010U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_SHIFT       (0x00000004U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATBEN_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_MASK  (0x00000060U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_SHIFT (0x00000005U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF0_ATB_MUX_SEL_MAX   (0x00000003U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATB_MUX_SEL_MASK  (0x00000180U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATB_MUX_SEL_SHIFT (0x00000007U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATB_MUX_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF1_ATB_MUX_SEL_MAX   (0x00000003U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_MASK        (0x00000200U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_SHIFT       (0x00000009U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_RESETVAL    (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_MAX         (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_ROK2_MASK   (0x00000400U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_ROK2_SHIFT  (0x0000000AU)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_ROK2_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATBEN_ROK2_MAX    (0x00000001U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATB_MUX_SEL_MASK  (0x00001800U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATB_MUX_SEL_SHIFT (0x0000000BU)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATB_MUX_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_DFT_REFBUF_CTRL_REFBUF2_ATB_MUX_SEL_MAX   (0x00000003U)

#define CSL_TOP_CTRL_DFT_REFBUF_CTRL_RESETVAL                                  (0x00000000U)

/* DFT_ODP_ATB_LOOPBACK_CTRL */

#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_MASK (0x00000001U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ODP_EN_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB0_ADCCAL0_LB_MASK (0x00000002U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB0_ADCCAL0_LB_SHIFT (0x00000001U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB0_ADCCAL0_LB_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB0_ADCCAL0_LB_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB1_ADCCAL1_LB_MASK (0x00000004U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB1_ADCCAL1_LB_SHIFT (0x00000002U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB1_ADCCAL1_LB_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_ATB1_ADCCAL1_LB_MAX (0x00000001U)

#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_MASK (0xFFFFFFF8U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_SHIFT (0x00000003U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESERVED_MAX (0x1FFFFFFFU)

#define CSL_TOP_CTRL_DFT_ODP_ATB_LOOPBACK_CTRL_RESETVAL                        (0x00000000U)

/* DFT_SOC_DTB_MUX_SEL */

#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_MASK  (0x000000FFU)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_SHIFT (0x00000000U)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_DFT_SOC_DTB_MUX_SEL_DTB_MUX_SEL_MAX   (0x000000FFU)

#define CSL_TOP_CTRL_DFT_SOC_DTB_MUX_SEL_RESETVAL                              (0x00000000U)

/* DFT_TEMPSENSE_CTRL */

#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_MASK    (0x00000001U)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_SHIFT   (0x00000000U)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_DFT_TEMPSENSE_CTRL_SENSOR5_SEL_MAX     (0x00000001U)

#define CSL_TOP_CTRL_DFT_TEMPSENSE_CTRL_RESETVAL                               (0x00000000U)

/* DFT_CTRL_1 */

#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_MASK                       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_1_DFT_CTRL_1_RESERVED_MAX                        (0xFFFFFFFFU)

#define CSL_TOP_CTRL_DFT_CTRL_1_RESETVAL                                       (0x00000000U)

/* DFT_CTRL_2 */

#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_MASK                       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_2_DFT_CTRL_2_RESERVED_MAX                        (0xFFFFFFFFU)

#define CSL_TOP_CTRL_DFT_CTRL_2_RESETVAL                                       (0x00000000U)

/* DFT_CTRL_3 */

#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_MASK                       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_3_DFT_CTRL_3_RESERVED_MAX                        (0xFFFFFFFFU)

#define CSL_TOP_CTRL_DFT_CTRL_3_RESETVAL                                       (0x00000000U)

/* DFT_CTRL_4 */

#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_MASK                       (0xFFFFFFFFU)
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_DFT_CTRL_4_DFT_CTRL_4_RESERVED_MAX                        (0xFFFFFFFFU)

#define CSL_TOP_CTRL_DFT_CTRL_4_RESETVAL                                       (0x00000000U)

/* DFT_CTRL_5 */


/* PROBE_BUS_SEL0 */

#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_SHIFT                   (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL0_PROBE_BUS_SEL0_SEL_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_CTRL_PROBE_BUS_SEL0_RESETVAL                                   (0x00000000U)

/* PROBE_BUS_SEL1 */

#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_SHIFT                   (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_PROBE_BUS_SEL1_PROBE_BUS_SEL1_SEL_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_CTRL_PROBE_BUS_SEL1_RESETVAL                                   (0x00000000U)

/* HW_SPARE_RW0 */

#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RW0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RW1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RW2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RW3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RO0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RO1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RO2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_RO3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_WPH */

#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MASK               (0xFFFFFFFFU)
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT              (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL           (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MAX                (0xFFFFFFFFU)

#define CSL_TOP_CTRL_HW_SPARE_WPH_RESETVAL                                     (0x00000000U)

/* HW_SPARE_REC */

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK              (0x00000001U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT             (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK              (0x00000002U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT             (0x00000001U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK              (0x00000004U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT             (0x00000002U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK              (0x00000008U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT             (0x00000003U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK              (0x00000010U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT             (0x00000004U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK              (0x00000020U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT             (0x00000005U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK              (0x00000040U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT             (0x00000006U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK              (0x00000080U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT             (0x00000007U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK              (0x00000100U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT             (0x00000008U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK              (0x00000200U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT             (0x00000009U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL          (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX               (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK             (0x00000400U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT            (0x0000000AU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK             (0x00000800U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT            (0x0000000BU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK             (0x00001000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT            (0x0000000CU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK             (0x00002000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT            (0x0000000DU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK             (0x00004000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT            (0x0000000EU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK             (0x00008000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT            (0x0000000FU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK             (0x00010000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT            (0x00000010U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK             (0x00020000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT            (0x00000011U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK             (0x00040000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT            (0x00000012U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK             (0x00080000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT            (0x00000013U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK             (0x00100000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT            (0x00000014U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK             (0x00200000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT            (0x00000015U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK             (0x00400000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT            (0x00000016U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK             (0x00800000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT            (0x00000017U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK             (0x01000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT            (0x00000018U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK             (0x02000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT            (0x00000019U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK             (0x04000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT            (0x0000001AU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK             (0x08000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT            (0x0000001BU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK             (0x10000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT            (0x0000001CU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK             (0x20000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT            (0x0000001DU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK             (0x40000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT            (0x0000001EU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK             (0x80000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT            (0x0000001FU)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL         (0x00000000U)
#define CSL_TOP_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX              (0x00000001U)

#define CSL_TOP_CTRL_HW_SPARE_REC_RESETVAL                                     (0x00000000U)

/* HW_SPARE_REC0 */


/* HW_SPARE_REC1 */


/* LOCK0_KICK0 */

#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                               (0xFFFFFFFFU)

#define CSL_TOP_CTRL_LOCK0_KICK0_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                               (0xFFFFFFFFU)

#define CSL_TOP_CTRL_LOCK0_KICK1_RESETVAL                                      (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                             (0x00000001U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                              (0x00000001U)

#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                             (0x00000002U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                            (0x00000001U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                              (0x00000001U)

#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                             (0x00000004U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                            (0x00000002U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                              (0x00000001U)

#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                            (0x00000008U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                           (0x00000003U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                        (0x00000000U)
#define CSL_TOP_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                             (0x00000001U)

#define CSL_TOP_CTRL_INTR_RAW_STATUS_RESETVAL                                  (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK           (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT          (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX            (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK           (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT          (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX            (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK           (0x00000004U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT          (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL       (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX            (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK          (0x00000008U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT         (0x00000003U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL      (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX           (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                        (0x00000000U)

/* INTR_ENABLE */

#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                              (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                             (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                               (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                              (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                             (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                               (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                              (0x00000004U)
#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                             (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                               (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                             (0x00000008U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                            (0x00000003U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                              (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_RESETVAL                                      (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                    (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                   (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                    (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                   (0x00000001U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                    (0x00000004U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                   (0x00000002U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                   (0x00000008U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                  (0x00000003U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                    (0x00000001U)

#define CSL_TOP_CTRL_INTR_ENABLE_CLEAR_RESETVAL                                (0x00000000U)

/* EOI */

#define CSL_TOP_CTRL_EOI_EOI_VECTOR_MASK                                       (0x000000FFU)
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_SHIFT                                      (0x00000000U)
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_RESETVAL                                   (0x00000000U)
#define CSL_TOP_CTRL_EOI_EOI_VECTOR_MAX                                        (0x000000FFU)

#define CSL_TOP_CTRL_EOI_RESETVAL                                              (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                            (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                         (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                              (0xFFFFFFFFU)

#define CSL_TOP_CTRL_FAULT_ADDRESS_RESETVAL                                    (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                         (0x0000003FU)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                        (0x00000000U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                     (0x00000000U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                          (0x0000003FU)

#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                           (0x00000040U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                          (0x00000006U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                       (0x00000000U)
#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                            (0x00000001U)

#define CSL_TOP_CTRL_FAULT_TYPE_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                       (0x000000FFU)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                      (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                   (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                        (0x000000FFU)

#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                      (0x000FFF00U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                     (0x00000008U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                  (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                       (0x00000FFFU)

#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                          (0xFFF00000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                         (0x00000014U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                      (0x00000000U)
#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                           (0x00000FFFU)

#define CSL_TOP_CTRL_FAULT_ATTR_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_CLEAR */

#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                                (0x00000001U)
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                               (0x00000000U)
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                            (0x00000000U)
#define CSL_TOP_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                                 (0x00000001U)

#define CSL_TOP_CTRL_FAULT_CLEAR_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
