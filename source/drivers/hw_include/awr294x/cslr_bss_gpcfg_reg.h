/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_bss_gpcfg_reg.h
*/
#ifndef CSLR_BSS_GPCFG_REG_H_
#define CSLR_BSS_GPCFG_REG_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RCM1;
    volatile uint32_t RCM2;
    volatile uint32_t RCM3;
    volatile uint32_t RCM4;
    volatile uint32_t RCM5;
    volatile uint32_t SPARE1;
    volatile uint32_t MSSBASEADDR;
    volatile uint32_t INTREG1;
    volatile uint32_t INTREG2;
    volatile uint32_t INTREG3;
    volatile uint32_t INTREG4;
    volatile uint32_t INTREG5;
    volatile uint32_t INTREG6;
    volatile uint32_t SPARE3;
    volatile uint8_t  Resv_64[8];
    volatile uint32_t ORBITGPIO;
    volatile uint32_t ORBITTESTID;
    volatile uint32_t ORBITSCRATCH1;
    volatile uint32_t ORBITSCRATCH2;
    volatile uint32_t ORBITSCRATCH3;
    volatile uint32_t ORBITSCRATCH4;
    volatile uint32_t SPARE4;
    volatile uint32_t MPUREG1;
    volatile uint32_t MPUREG2;
    volatile uint32_t MPUREG3;
    volatile uint32_t MPUREG4;
    volatile uint32_t MPUREG5;
    volatile uint32_t MPUREG6;
    volatile uint32_t MPUREG7;
    volatile uint32_t MPUREG8;
    volatile uint32_t MPUREG9;
    volatile uint32_t MPUREG10;
    volatile uint32_t STCCTRL;
    volatile uint32_t SPARE7;
    volatile uint32_t SPARE8;
    volatile uint32_t EFUSEOVR;
    volatile uint32_t EFUSEWRSPARE1;
    volatile uint32_t EFUSEWRSPARE2;
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
    volatile uint32_t EFUSE2_ROW_7;
    volatile uint32_t EFUSE2_ROW_8;
    volatile uint32_t RDSPARE1;
    volatile uint8_t  Resv_260[4];
    volatile uint32_t RDSPARE2;
    volatile uint32_t FRCREG1;
    volatile uint32_t FRCREG2;
    volatile uint32_t FRCREG3;
    volatile uint32_t FRCREG4;
    volatile uint32_t FRCREG5;
    volatile uint32_t FRCREG6;
    volatile uint32_t FRCREG7;
    volatile uint32_t FRCREG8;
    volatile uint32_t FRCREG9;
    volatile uint32_t FRCREG10;
    volatile uint32_t FRCREG11;
    volatile uint32_t FRCREG12;
    volatile uint32_t FRCREG13;
    volatile uint32_t FRCREG14;
    volatile uint32_t FRCREG15;
    volatile uint32_t FRCREG16;
    volatile uint32_t FRCREG17;
    volatile uint32_t FRCREG18;
    volatile uint32_t FRCREG19;
    volatile uint32_t FRCREG20;
    volatile uint32_t FRCREG21;
    volatile uint32_t FRCREG22;
    volatile uint32_t FRCREG23;
    volatile uint32_t SPARE9;
    volatile uint32_t SPARE10;
    volatile uint32_t RX_STATUS_REG1_MASK;
    volatile uint32_t TX_STAT_CLK_STAT_MASK;
    volatile uint32_t LODIST_WU_STAT_MASK;
    volatile uint32_t CCMREG6;
    volatile uint32_t CCMREG0;
    volatile uint32_t CCMREG1;
    volatile uint32_t CCMREG2;
    volatile uint32_t CCMREG3;
    volatile uint32_t CCMREG4;
    volatile uint32_t CCMREG5;
    volatile uint32_t MSS_ESM_SPARE;
    volatile uint32_t ESM_GP2_MASK1;
    volatile uint32_t ESM_GP2_MASK2;
    volatile uint32_t ESM_GP2_MASK3;
    volatile uint32_t MSS_DSS_INTR_MASK;
    volatile uint32_t UDMA_INT1;
    volatile uint32_t UDMA_INT2;
    volatile uint32_t UDMA_INT3;
    volatile uint32_t TCMA_PARITY;
    volatile uint32_t TCMB0_PARITY;
    volatile uint32_t TCMB1_PARITY;
    volatile uint32_t TCMECCCHK;
    volatile uint32_t TXPALOOPBCKCLK;
    volatile uint32_t RXIFALOOPBCKCLK;
    volatile uint32_t DBGACKCTL;
    volatile uint32_t AXI2VBUSPSEL;
} CSL_bss_gpcfg_regRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_GPCFG_REG_RCM1                                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2                                                 (0x00000004U)
#define CSL_BSS_GPCFG_REG_RCM3                                                 (0x00000008U)
#define CSL_BSS_GPCFG_REG_RCM4                                                 (0x0000000CU)
#define CSL_BSS_GPCFG_REG_RCM5                                                 (0x00000010U)
#define CSL_BSS_GPCFG_REG_SPARE1                                               (0x00000014U)
#define CSL_BSS_GPCFG_REG_MSSBASEADDR                                          (0x00000018U)
#define CSL_BSS_GPCFG_REG_INTREG1                                              (0x0000001CU)
#define CSL_BSS_GPCFG_REG_INTREG2                                              (0x00000020U)
#define CSL_BSS_GPCFG_REG_INTREG3                                              (0x00000024U)
#define CSL_BSS_GPCFG_REG_INTREG4                                              (0x00000028U)
#define CSL_BSS_GPCFG_REG_INTREG5                                              (0x0000002CU)
#define CSL_BSS_GPCFG_REG_INTREG6                                              (0x00000030U)
#define CSL_BSS_GPCFG_REG_SPARE3                                               (0x00000034U)
#define CSL_BSS_GPCFG_REG_ORBITGPIO                                            (0x00000040U)
#define CSL_BSS_GPCFG_REG_ORBITTESTID                                          (0x00000044U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH1                                        (0x00000048U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH2                                        (0x0000004CU)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH3                                        (0x00000050U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH4                                        (0x00000054U)
#define CSL_BSS_GPCFG_REG_SPARE4                                               (0x00000058U)
#define CSL_BSS_GPCFG_REG_MPUREG1                                              (0x0000005CU)
#define CSL_BSS_GPCFG_REG_MPUREG2                                              (0x00000060U)
#define CSL_BSS_GPCFG_REG_MPUREG3                                              (0x00000064U)
#define CSL_BSS_GPCFG_REG_MPUREG4                                              (0x00000068U)
#define CSL_BSS_GPCFG_REG_MPUREG5                                              (0x0000006CU)
#define CSL_BSS_GPCFG_REG_MPUREG6                                              (0x00000070U)
#define CSL_BSS_GPCFG_REG_MPUREG7                                              (0x00000074U)
#define CSL_BSS_GPCFG_REG_MPUREG8                                              (0x00000078U)
#define CSL_BSS_GPCFG_REG_MPUREG9                                              (0x0000007CU)
#define CSL_BSS_GPCFG_REG_MPUREG10                                             (0x00000080U)
#define CSL_BSS_GPCFG_REG_STCCTRL                                              (0x00000084U)
#define CSL_BSS_GPCFG_REG_SPARE7                                               (0x00000088U)
#define CSL_BSS_GPCFG_REG_SPARE8                                               (0x0000008CU)
#define CSL_BSS_GPCFG_REG_EFUSEOVR                                             (0x00000090U)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE1                                        (0x00000094U)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE2                                        (0x00000098U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_44                                        (0x0000009CU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_45                                        (0x000000A0U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_46                                        (0x000000A4U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_47                                        (0x000000A8U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_48                                        (0x000000ACU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_49                                        (0x000000B0U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_50                                        (0x000000B4U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_51                                        (0x000000B8U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_52                                        (0x000000BCU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_53                                        (0x000000C0U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_54                                        (0x000000C4U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_55                                        (0x000000C8U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_56                                        (0x000000CCU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_57                                        (0x000000D0U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_58                                        (0x000000D4U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_59                                        (0x000000D8U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_60                                        (0x000000DCU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_61                                        (0x000000E0U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_62                                        (0x000000E4U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_63                                        (0x000000E8U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_5                                         (0x000000ECU)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_6                                         (0x000000F0U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_7                                         (0x000000F4U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_8                                         (0x000000F8U)
#define CSL_BSS_GPCFG_REG_RDSPARE1                                             (0x000000FCU)
#define CSL_BSS_GPCFG_REG_RDSPARE2                                             (0x00000104U)
#define CSL_BSS_GPCFG_REG_FRCREG1                                              (0x00000108U)
#define CSL_BSS_GPCFG_REG_FRCREG2                                              (0x0000010CU)
#define CSL_BSS_GPCFG_REG_FRCREG3                                              (0x00000110U)
#define CSL_BSS_GPCFG_REG_FRCREG4                                              (0x00000114U)
#define CSL_BSS_GPCFG_REG_FRCREG5                                              (0x00000118U)
#define CSL_BSS_GPCFG_REG_FRCREG6                                              (0x0000011CU)
#define CSL_BSS_GPCFG_REG_FRCREG7                                              (0x00000120U)
#define CSL_BSS_GPCFG_REG_FRCREG8                                              (0x00000124U)
#define CSL_BSS_GPCFG_REG_FRCREG9                                              (0x00000128U)
#define CSL_BSS_GPCFG_REG_FRCREG10                                             (0x0000012CU)
#define CSL_BSS_GPCFG_REG_FRCREG11                                             (0x00000130U)
#define CSL_BSS_GPCFG_REG_FRCREG12                                             (0x00000134U)
#define CSL_BSS_GPCFG_REG_FRCREG13                                             (0x00000138U)
#define CSL_BSS_GPCFG_REG_FRCREG14                                             (0x0000013CU)
#define CSL_BSS_GPCFG_REG_FRCREG15                                             (0x00000140U)
#define CSL_BSS_GPCFG_REG_FRCREG16                                             (0x00000144U)
#define CSL_BSS_GPCFG_REG_FRCREG17                                             (0x00000148U)
#define CSL_BSS_GPCFG_REG_FRCREG18                                             (0x0000014CU)
#define CSL_BSS_GPCFG_REG_FRCREG19                                             (0x00000150U)
#define CSL_BSS_GPCFG_REG_FRCREG20                                             (0x00000154U)
#define CSL_BSS_GPCFG_REG_FRCREG21                                             (0x00000158U)
#define CSL_BSS_GPCFG_REG_FRCREG22                                             (0x0000015CU)
#define CSL_BSS_GPCFG_REG_FRCREG23                                             (0x00000160U)
#define CSL_BSS_GPCFG_REG_SPARE9                                               (0x00000164U)
#define CSL_BSS_GPCFG_REG_SPARE10                                              (0x00000168U)
#define CSL_BSS_GPCFG_REG_RX_STATUS_REG1_MASK                                  (0x0000016CU)
#define CSL_BSS_GPCFG_REG_TX_STAT_CLK_STAT_MASK                                (0x00000170U)
#define CSL_BSS_GPCFG_REG_LODIST_WU_STAT_MASK                                  (0x00000174U)
#define CSL_BSS_GPCFG_REG_CCMREG6                                              (0x00000178U)
#define CSL_BSS_GPCFG_REG_CCMREG0                                              (0x0000017CU)
#define CSL_BSS_GPCFG_REG_CCMREG1                                              (0x00000180U)
#define CSL_BSS_GPCFG_REG_CCMREG2                                              (0x00000184U)
#define CSL_BSS_GPCFG_REG_CCMREG3                                              (0x00000188U)
#define CSL_BSS_GPCFG_REG_CCMREG4                                              (0x0000018CU)
#define CSL_BSS_GPCFG_REG_CCMREG5                                              (0x00000190U)
#define CSL_BSS_GPCFG_REG_MSS_ESM_SPARE                                        (0x00000194U)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK1                                        (0x00000198U)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK2                                        (0x0000019CU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK3                                        (0x000001A0U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK                                    (0x000001A4U)
#define CSL_BSS_GPCFG_REG_UDMA_INT1                                            (0x000001A8U)
#define CSL_BSS_GPCFG_REG_UDMA_INT2                                            (0x000001ACU)
#define CSL_BSS_GPCFG_REG_UDMA_INT3                                            (0x000001B0U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY                                          (0x000001B4U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY                                         (0x000001B8U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY                                         (0x000001BCU)
#define CSL_BSS_GPCFG_REG_TCMECCCHK                                            (0x000001C0U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK                                       (0x000001C4U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK                                      (0x000001C8U)
#define CSL_BSS_GPCFG_REG_DBGACKCTL                                            (0x000001CCU)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL                                         (0x000001D0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RCM1 */

#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSE_MASK                                   (0x000001FFU)
#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSE_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSE_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSE_MAX                                    (0x000001FFU)

#define CSL_BSS_GPCFG_REG_RCM1_NU1_MASK                                        (0x0000FE00U)
#define CSL_BSS_GPCFG_REG_RCM1_NU1_SHIFT                                       (0x00000009U)
#define CSL_BSS_GPCFG_REG_RCM1_NU1_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_NU1_MAX                                         (0x0000007FU)

#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSECLR_MASK                                (0x00070000U)
#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSECLR_SHIFT                               (0x00000010U)
#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSECLR_RESETVAL                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_RSTCAUSECLR_MAX                                 (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM1_NU2_MASK                                        (0x00F80000U)
#define CSL_BSS_GPCFG_REG_RCM1_NU2_SHIFT                                       (0x00000013U)
#define CSL_BSS_GPCFG_REG_RCM1_NU2_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_NU2_MAX                                         (0x0000001FU)

#define CSL_BSS_GPCFG_REG_RCM1_KEYCR4WDRSTNEN_MASK                             (0x07000000U)
#define CSL_BSS_GPCFG_REG_RCM1_KEYCR4WDRSTNEN_SHIFT                            (0x00000018U)
#define CSL_BSS_GPCFG_REG_RCM1_KEYCR4WDRSTNEN_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_KEYCR4WDRSTNEN_MAX                              (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM1_NU3_MASK                                        (0xF8000000U)
#define CSL_BSS_GPCFG_REG_RCM1_NU3_SHIFT                                       (0x0000001BU)
#define CSL_BSS_GPCFG_REG_RCM1_NU3_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM1_NU3_MAX                                         (0x0000001FU)

#define CSL_BSS_GPCFG_REG_RCM1_RESETVAL                                        (0x00000000U)

/* RCM2 */

#define CSL_BSS_GPCFG_REG_RCM2_SCCRESET_MASK                                   (0x00000007U)
#define CSL_BSS_GPCFG_REG_RCM2_SCCRESET_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_SCCRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_SCCRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU1_MASK                                        (0x00000008U)
#define CSL_BSS_GPCFG_REG_RCM2_NU1_SHIFT                                       (0x00000003U)
#define CSL_BSS_GPCFG_REG_RCM2_NU1_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU1_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_CR4RESET_MASK                                   (0x00000070U)
#define CSL_BSS_GPCFG_REG_RCM2_CR4RESET_SHIFT                                  (0x00000004U)
#define CSL_BSS_GPCFG_REG_RCM2_CR4RESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_CR4RESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU2_MASK                                        (0x00000080U)
#define CSL_BSS_GPCFG_REG_RCM2_NU2_SHIFT                                       (0x00000007U)
#define CSL_BSS_GPCFG_REG_RCM2_NU2_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU2_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_CRCRESET_MASK                                   (0x00000700U)
#define CSL_BSS_GPCFG_REG_RCM2_CRCRESET_SHIFT                                  (0x00000008U)
#define CSL_BSS_GPCFG_REG_RCM2_CRCRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_CRCRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU3_MASK                                        (0x00000800U)
#define CSL_BSS_GPCFG_REG_RCM2_NU3_SHIFT                                       (0x0000000BU)
#define CSL_BSS_GPCFG_REG_RCM2_NU3_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU3_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_DMARESET_MASK                                   (0x00007000U)
#define CSL_BSS_GPCFG_REG_RCM2_DMARESET_SHIFT                                  (0x0000000CU)
#define CSL_BSS_GPCFG_REG_RCM2_DMARESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_DMARESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU4_MASK                                        (0x00008000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU4_SHIFT                                       (0x0000000FU)
#define CSL_BSS_GPCFG_REG_RCM2_NU4_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU4_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_DCCRESET_MASK                                   (0x00070000U)
#define CSL_BSS_GPCFG_REG_RCM2_DCCRESET_SHIFT                                  (0x00000010U)
#define CSL_BSS_GPCFG_REG_RCM2_DCCRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_DCCRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU5_MASK                                        (0x00080000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU5_SHIFT                                       (0x00000013U)
#define CSL_BSS_GPCFG_REG_RCM2_NU5_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU5_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_VIMRESET_MASK                                   (0x00700000U)
#define CSL_BSS_GPCFG_REG_RCM2_VIMRESET_SHIFT                                  (0x00000014U)
#define CSL_BSS_GPCFG_REG_RCM2_VIMRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_VIMRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU6_MASK                                        (0x00800000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU6_SHIFT                                       (0x00000017U)
#define CSL_BSS_GPCFG_REG_RCM2_NU6_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU6_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_ESMRESET_MASK                                   (0x07000000U)
#define CSL_BSS_GPCFG_REG_RCM2_ESMRESET_SHIFT                                  (0x00000018U)
#define CSL_BSS_GPCFG_REG_RCM2_ESMRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_ESMRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU7_MASK                                        (0x08000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU7_SHIFT                                       (0x0000001BU)
#define CSL_BSS_GPCFG_REG_RCM2_NU7_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU7_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_RTIRESET_MASK                                   (0x70000000U)
#define CSL_BSS_GPCFG_REG_RCM2_RTIRESET_SHIFT                                  (0x0000001CU)
#define CSL_BSS_GPCFG_REG_RCM2_RTIRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_RTIRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM2_NU8_MASK                                        (0x80000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU8_SHIFT                                       (0x0000001FU)
#define CSL_BSS_GPCFG_REG_RCM2_NU8_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM2_NU8_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM2_RESETVAL                                        (0x00000000U)

/* RCM3 */

#define CSL_BSS_GPCFG_REG_RCM3_FRCRESET_MASK                                   (0x00000007U)
#define CSL_BSS_GPCFG_REG_RCM3_FRCRESET_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_FRCRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_FRCRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU1_MASK                                        (0x00000008U)
#define CSL_BSS_GPCFG_REG_RCM3_NU1_SHIFT                                       (0x00000003U)
#define CSL_BSS_GPCFG_REG_RCM3_NU1_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU1_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_SCRRESET_MASK                                   (0x00000070U)
#define CSL_BSS_GPCFG_REG_RCM3_SCRRESET_SHIFT                                  (0x00000004U)
#define CSL_BSS_GPCFG_REG_RCM3_SCRRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_SCRRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU2_MASK                                        (0x00000080U)
#define CSL_BSS_GPCFG_REG_RCM3_NU2_SHIFT                                       (0x00000007U)
#define CSL_BSS_GPCFG_REG_RCM3_NU2_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU2_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_SCIRESET_MASK                                   (0x00000700U)
#define CSL_BSS_GPCFG_REG_RCM3_SCIRESET_SHIFT                                  (0x00000008U)
#define CSL_BSS_GPCFG_REG_RCM3_SCIRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_SCIRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU3_MASK                                        (0x00000800U)
#define CSL_BSS_GPCFG_REG_RCM3_NU3_SHIFT                                       (0x0000000BU)
#define CSL_BSS_GPCFG_REG_RCM3_NU3_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU3_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_GPADCRESET_MASK                                 (0x00007000U)
#define CSL_BSS_GPCFG_REG_RCM3_GPADCRESET_SHIFT                                (0x0000000CU)
#define CSL_BSS_GPCFG_REG_RCM3_GPADCRESET_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_GPADCRESET_MAX                                  (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU4_MASK                                        (0x00008000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU4_SHIFT                                       (0x0000000FU)
#define CSL_BSS_GPCFG_REG_RCM3_NU4_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU4_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_FFTRESET_MASK                                   (0x00070000U)
#define CSL_BSS_GPCFG_REG_RCM3_FFTRESET_SHIFT                                  (0x00000010U)
#define CSL_BSS_GPCFG_REG_RCM3_FFTRESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_FFTRESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU5_MASK                                        (0x00080000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU5_SHIFT                                       (0x00000013U)
#define CSL_BSS_GPCFG_REG_RCM3_NU5_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU5_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_DFERESET_MASK                                   (0x00700000U)
#define CSL_BSS_GPCFG_REG_RCM3_DFERESET_SHIFT                                  (0x00000014U)
#define CSL_BSS_GPCFG_REG_RCM3_DFERESET_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_DFERESET_MAX                                    (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU6_MASK                                        (0x00800000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU6_SHIFT                                       (0x00000017U)
#define CSL_BSS_GPCFG_REG_RCM3_NU6_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU6_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_RAMPGENRESET_MASK                               (0x07000000U)
#define CSL_BSS_GPCFG_REG_RCM3_RAMPGENRESET_SHIFT                              (0x00000018U)
#define CSL_BSS_GPCFG_REG_RCM3_RAMPGENRESET_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_RAMPGENRESET_MAX                                (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU7_MASK                                        (0x08000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU7_SHIFT                                       (0x0000001BU)
#define CSL_BSS_GPCFG_REG_RCM3_NU7_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU7_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_SEQEXTNRESET_MASK                               (0x70000000U)
#define CSL_BSS_GPCFG_REG_RCM3_SEQEXTNRESET_SHIFT                              (0x0000001CU)
#define CSL_BSS_GPCFG_REG_RCM3_SEQEXTNRESET_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_SEQEXTNRESET_MAX                                (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM3_NU8_MASK                                        (0x80000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU8_SHIFT                                       (0x0000001FU)
#define CSL_BSS_GPCFG_REG_RCM3_NU8_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM3_NU8_MAX                                         (0x00000001U)

#define CSL_BSS_GPCFG_REG_RCM3_RESETVAL                                        (0x00000000U)

/* RCM4 */

#define CSL_BSS_GPCFG_REG_RCM4_SPARE_MASK                                      (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_RCM4_SPARE_SHIFT                                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM4_SPARE_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM4_SPARE_MAX                                       (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_RCM4_RESETVAL                                        (0x00000000U)

/* RCM5 */

#define CSL_BSS_GPCFG_REG_RCM5_CR4_CORE2_GATE_MASK                             (0x00000007U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_CORE2_GATE_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_CORE2_GATE_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_CORE2_GATE_MAX                              (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM5_NU1_MASK                                        (0x000000F8U)
#define CSL_BSS_GPCFG_REG_RCM5_NU1_SHIFT                                       (0x00000003U)
#define CSL_BSS_GPCFG_REG_RCM5_NU1_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM5_NU1_MAX                                         (0x0000001FU)

#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTTOASSERTDLY_MASK                         (0x0000FF00U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTTOASSERTDLY_SHIFT                        (0x00000008U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTTOASSERTDLY_RESETVAL                     (0x0000000FU)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTTOASSERTDLY_MAX                          (0x000000FFU)

#define CSL_BSS_GPCFG_REG_RCM5_RSTASSERTDLY_MASK                               (0x00FF0000U)
#define CSL_BSS_GPCFG_REG_RCM5_RSTASSERTDLY_SHIFT                              (0x00000010U)
#define CSL_BSS_GPCFG_REG_RCM5_RSTASSERTDLY_RESETVAL                           (0x0000000FU)
#define CSL_BSS_GPCFG_REG_RCM5_RSTASSERTDLY_MAX                                (0x000000FFU)

#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTWFICHECKEN_MASK                          (0x07000000U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTWFICHECKEN_SHIFT                         (0x00000018U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTWFICHECKEN_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM5_CR4_RSTWFICHECKEN_MAX                           (0x00000007U)

#define CSL_BSS_GPCFG_REG_RCM5_NU4_MASK                                        (0xF8000000U)
#define CSL_BSS_GPCFG_REG_RCM5_NU4_SHIFT                                       (0x0000001BU)
#define CSL_BSS_GPCFG_REG_RCM5_NU4_RESETVAL                                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_RCM5_NU4_MAX                                         (0x0000001FU)

#define CSL_BSS_GPCFG_REG_RCM5_RESETVAL                                        (0x000F0F00U)

/* SPARE1 */

#define CSL_BSS_GPCFG_REG_SPARE1_SPARE1_MASK                                   (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE1_SPARE1_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE1_SPARE1_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE1_SPARE1_MAX                                    (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE1_RESETVAL                                      (0x00000000U)

/* MSSBASEADDR */

#define CSL_BSS_GPCFG_REG_MSSBASEADDR_MSSBASEADDR_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MSSBASEADDR_MSSBASEADDR_SHIFT                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSSBASEADDR_MSSBASEADDR_RESETVAL                     (0x0C0F0500U)
#define CSL_BSS_GPCFG_REG_MSSBASEADDR_MSSBASEADDR_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MSSBASEADDR_RESETVAL                                 (0x0C0F0500U)

/* INTREG1 */

#define CSL_BSS_GPCFG_REG_INTREG1_INTBYPSEL_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_INTREG1_INTBYPSEL_SHIFT                              (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG1_INTBYPSEL_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG1_INTBYPSEL_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_INTREG1_RESETVAL                                     (0x00000000U)

/* INTREG2 */

#define CSL_BSS_GPCFG_REG_INTREG2_INTBYPVAL0TO31_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_INTREG2_INTBYPVAL0TO31_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG2_INTBYPVAL0TO31_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG2_INTBYPVAL0TO31_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_INTREG2_RESETVAL                                     (0x00000000U)

/* INTREG3 */

#define CSL_BSS_GPCFG_REG_INTREG3_INTBYPVAL32TO63_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_INTREG3_INTBYPVAL32TO63_SHIFT                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG3_INTBYPVAL32TO63_RESETVAL                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG3_INTBYPVAL32TO63_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_INTREG3_RESETVAL                                     (0x00000000U)

/* INTREG4 */

#define CSL_BSS_GPCFG_REG_INTREG4_INTBYPVAL63TO95_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_INTREG4_INTBYPVAL63TO95_SHIFT                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG4_INTBYPVAL63TO95_RESETVAL                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG4_INTBYPVAL63TO95_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_INTREG4_RESETVAL                                     (0x00000000U)

/* INTREG5 */

#define CSL_BSS_GPCFG_REG_INTREG5_SWINTR96TO127_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_INTREG5_SWINTR96TO127_SHIFT                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG5_SWINTR96TO127_RESETVAL                       (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG5_SWINTR96TO127_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_INTREG5_RESETVAL                                     (0x00000000U)

/* INTREG6 */

#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT1_MASK                              (0x00000001U)
#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT1_SHIFT                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT1_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT1_MAX                               (0x00000001U)

#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT2_MASK                              (0x00000002U)
#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT2_SHIFT                             (0x00000001U)
#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT2_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG6_MSS_SWINT2_MAX                               (0x00000001U)

#define CSL_BSS_GPCFG_REG_INTREG6_NU_MASK                                      (0xFFFFFFFCU)
#define CSL_BSS_GPCFG_REG_INTREG6_NU_SHIFT                                     (0x00000002U)
#define CSL_BSS_GPCFG_REG_INTREG6_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_INTREG6_NU_MAX                                       (0x3FFFFFFFU)

#define CSL_BSS_GPCFG_REG_INTREG6_RESETVAL                                     (0x00000000U)

/* SPARE3 */

#define CSL_BSS_GPCFG_REG_SPARE3_SPARE3_MASK                                   (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE3_SPARE3_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE3_SPARE3_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE3_SPARE3_MAX                                    (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE3_RESETVAL                                      (0x00000000U)

/* ORBITGPIO */

#define CSL_BSS_GPCFG_REG_ORBITGPIO_ORBITGPIO_MASK                             (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ORBITGPIO_ORBITGPIO_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITGPIO_ORBITGPIO_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITGPIO_ORBITGPIO_MAX                              (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ORBITGPIO_RESETVAL                                   (0x00000000U)

/* ORBITTESTID */

#define CSL_BSS_GPCFG_REG_ORBITTESTID_ORBITTESTID_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ORBITTESTID_ORBITTESTID_SHIFT                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITTESTID_ORBITTESTID_RESETVAL                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITTESTID_ORBITTESTID_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ORBITTESTID_RESETVAL                                 (0x00000000U)

/* ORBITSCRATCH1 */

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH1_ORBITSCRATCH1_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH1_ORBITSCRATCH1_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH1_ORBITSCRATCH1_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH1_ORBITSCRATCH1_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH1_RESETVAL                               (0x00000000U)

/* ORBITSCRATCH2 */

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH2_ORBITSCRATCH2_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH2_ORBITSCRATCH2_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH2_ORBITSCRATCH2_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH2_ORBITSCRATCH2_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH2_RESETVAL                               (0x00000000U)

/* ORBITSCRATCH3 */

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH3_ORBITSCRATCH3_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH3_ORBITSCRATCH3_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH3_ORBITSCRATCH3_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH3_ORBITSCRATCH3_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH3_RESETVAL                               (0x00000000U)

/* ORBITSCRATCH4 */

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH4_ORBITSCRATCH4_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH4_ORBITSCRATCH4_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH4_ORBITSCRATCH4_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_ORBITSCRATCH4_ORBITSCRATCH4_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ORBITSCRATCH4_RESETVAL                               (0x00000000U)

/* SPARE4 */

#define CSL_BSS_GPCFG_REG_SPARE4_SPARE4_MASK                                   (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE4_SPARE4_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE4_SPARE4_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE4_SPARE4_MAX                                    (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE4_RESETVAL                                      (0x00000000U)

/* MPUREG1 */

#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_EN_MASK                              (0x00000001U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_EN_SHIFT                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_EN_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_EN_MAX                               (0x00000001U)

#define CSL_BSS_GPCFG_REG_MPUREG1_NU1_MASK                                     (0x000000FEU)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU1_SHIFT                                    (0x00000001U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU1_MAX                                      (0x0000007FU)

#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_VLD_REGION_MASK                      (0x00000F00U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_VLD_REGION_SHIFT                     (0x00000008U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_VLD_REGION_RESETVAL                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_VLD_REGION_MAX                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_MPUREG1_NU2_MASK                                     (0x0000F000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU2_SHIFT                                    (0x0000000CU)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU2_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_ERR_CLR_MASK                         (0x00010000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_ERR_CLR_SHIFT                        (0x00000010U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_ERR_CLR_RESETVAL                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_DMA_MPU_ERR_CLR_MAX                          (0x00000001U)

#define CSL_BSS_GPCFG_REG_MPUREG1_NU3_MASK                                     (0xFFFE0000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU3_SHIFT                                    (0x00000011U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU3_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG1_NU3_MAX                                      (0x00007FFFU)

#define CSL_BSS_GPCFG_REG_MPUREG1_RESETVAL                                     (0x00000000U)

/* MPUREG2 */

#define CSL_BSS_GPCFG_REG_MPUREG2_DMACFGSTADDR00_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG2_DMACFGSTADDR00_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG2_DMACFGSTADDR00_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG2_DMACFGSTADDR00_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG2_RESETVAL                                     (0x00000000U)

/* MPUREG3 */

#define CSL_BSS_GPCFG_REG_MPUREG3_DMACFGENADDR00_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG3_DMACFGENADDR00_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG3_DMACFGENADDR00_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG3_DMACFGENADDR00_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG3_RESETVAL                                     (0x00000000U)

/* MPUREG4 */

#define CSL_BSS_GPCFG_REG_MPUREG4_DMACFGSTADDR01_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG4_DMACFGSTADDR01_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG4_DMACFGSTADDR01_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG4_DMACFGSTADDR01_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG4_RESETVAL                                     (0x00000000U)

/* MPUREG5 */

#define CSL_BSS_GPCFG_REG_MPUREG5_DMACFGENADDR01_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG5_DMACFGENADDR01_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG5_DMACFGENADDR01_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG5_DMACFGENADDR01_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG5_RESETVAL                                     (0x00000000U)

/* MPUREG6 */

#define CSL_BSS_GPCFG_REG_MPUREG6_DMACFGSTADDR02_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG6_DMACFGSTADDR02_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG6_DMACFGSTADDR02_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG6_DMACFGSTADDR02_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG6_RESETVAL                                     (0x00000000U)

/* MPUREG7 */

#define CSL_BSS_GPCFG_REG_MPUREG7_DMACFGENADDR02_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG7_DMACFGENADDR02_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG7_DMACFGENADDR02_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG7_DMACFGENADDR02_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG7_RESETVAL                                     (0x00000000U)

/* MPUREG8 */

#define CSL_BSS_GPCFG_REG_MPUREG8_DMACFGSTADDR03_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG8_DMACFGSTADDR03_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG8_DMACFGSTADDR03_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG8_DMACFGSTADDR03_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG8_RESETVAL                                     (0x00000000U)

/* MPUREG9 */

#define CSL_BSS_GPCFG_REG_MPUREG9_DMACFGENADDR03_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG9_DMACFGENADDR03_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG9_DMACFGENADDR03_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG9_DMACFGENADDR03_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG9_RESETVAL                                     (0x00000000U)

/* MPUREG10 */

#define CSL_BSS_GPCFG_REG_MPUREG10_DMA_MPU_ERR_ADDR_MASK                       (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MPUREG10_DMA_MPU_ERR_ADDR_SHIFT                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG10_DMA_MPU_ERR_ADDR_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_MPUREG10_DMA_MPU_ERR_ADDR_MAX                        (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MPUREG10_RESETVAL                                    (0x00000000U)

/* STCCTRL */

#define CSL_BSS_GPCFG_REG_STCCTRL_STCCTRL_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_STCCTRL_STCCTRL_SHIFT                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_STCCTRL_STCCTRL_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_STCCTRL_STCCTRL_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_STCCTRL_RESETVAL                                     (0x00000000U)

/* SPARE7 */

#define CSL_BSS_GPCFG_REG_SPARE7_SPARE7_MASK                                   (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE7_SPARE7_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE7_SPARE7_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE7_SPARE7_MAX                                    (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE7_RESETVAL                                      (0x00000000U)

/* SPARE8 */

#define CSL_BSS_GPCFG_REG_SPARE8_SPARE8_MASK                                   (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE8_SPARE8_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE8_SPARE8_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE8_SPARE8_MAX                                    (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE8_RESETVAL                                      (0x00000000U)

/* EFUSEOVR */

#define CSL_BSS_GPCFG_REG_EFUSEOVR_EFUSEOVRSEL_MASK                            (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSEOVR_EFUSEOVRSEL_SHIFT                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSEOVR_EFUSEOVRSEL_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSEOVR_EFUSEOVRSEL_MAX                             (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSEOVR_RESETVAL                                    (0x00000000U)

/* EFUSEWRSPARE1 */

#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE1_EFUSEWRSPARE1_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE1_EFUSEWRSPARE1_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE1_EFUSEWRSPARE1_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE1_EFUSEWRSPARE1_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE1_RESETVAL                               (0x00000000U)

/* EFUSEWRSPARE2 */

#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE2_EFUSEWRSPARE2_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE2_EFUSEWRSPARE2_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE2_EFUSEWRSPARE2_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE2_EFUSEWRSPARE2_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSEWRSPARE2_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_44 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_44_EFUSE1_ROW_44_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_44_EFUSE1_ROW_44_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_44_EFUSE1_ROW_44_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_44_EFUSE1_ROW_44_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_44_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_45 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_45_EFUSE1_ROW_45_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_45_EFUSE1_ROW_45_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_45_EFUSE1_ROW_45_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_45_EFUSE1_ROW_45_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_45_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_46 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_46_EFUSE1_ROW_46_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_46_EFUSE1_ROW_46_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_46_EFUSE1_ROW_46_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_46_EFUSE1_ROW_46_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_46_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_47 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_47_EFUSE1_ROW_47_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_47_EFUSE1_ROW_47_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_47_EFUSE1_ROW_47_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_47_EFUSE1_ROW_47_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_47_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_48 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_48_EFUSE1_ROW_48_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_48_EFUSE1_ROW_48_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_48_EFUSE1_ROW_48_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_48_EFUSE1_ROW_48_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_48_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_49 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_49_EFUSE1_ROW_49_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_49_EFUSE1_ROW_49_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_49_EFUSE1_ROW_49_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_49_EFUSE1_ROW_49_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_49_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_50 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_50_EFUSE1_ROW_50_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_50_EFUSE1_ROW_50_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_50_EFUSE1_ROW_50_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_50_EFUSE1_ROW_50_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_50_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_51 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_51_EFUSE1_ROW_51_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_51_EFUSE1_ROW_51_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_51_EFUSE1_ROW_51_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_51_EFUSE1_ROW_51_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_51_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_52 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_52_EFUSE1_ROW_52_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_52_EFUSE1_ROW_52_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_52_EFUSE1_ROW_52_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_52_EFUSE1_ROW_52_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_52_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_53 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_53_EFUSE1_ROW_53_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_53_EFUSE1_ROW_53_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_53_EFUSE1_ROW_53_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_53_EFUSE1_ROW_53_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_53_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_54 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_54_EFUSE1_ROW_54_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_54_EFUSE1_ROW_54_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_54_EFUSE1_ROW_54_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_54_EFUSE1_ROW_54_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_54_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_55 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_55_EFUSE1_ROW_55_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_55_EFUSE1_ROW_55_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_55_EFUSE1_ROW_55_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_55_EFUSE1_ROW_55_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_55_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_56 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_56_EFUSE1_ROW_56_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_56_EFUSE1_ROW_56_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_56_EFUSE1_ROW_56_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_56_EFUSE1_ROW_56_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_56_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_57 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_57_EFUSE1_ROW_57_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_57_EFUSE1_ROW_57_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_57_EFUSE1_ROW_57_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_57_EFUSE1_ROW_57_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_57_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_58 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_58_EFUSE1_ROW_58_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_58_EFUSE1_ROW_58_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_58_EFUSE1_ROW_58_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_58_EFUSE1_ROW_58_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_58_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_59 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_59_EFUSE1_ROW_59_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_59_EFUSE1_ROW_59_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_59_EFUSE1_ROW_59_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_59_EFUSE1_ROW_59_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_59_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_60 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_60_EFUSE1_ROW_60_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_60_EFUSE1_ROW_60_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_60_EFUSE1_ROW_60_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_60_EFUSE1_ROW_60_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_60_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_61 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_61_EFUSE1_ROW_61_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_61_EFUSE1_ROW_61_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_61_EFUSE1_ROW_61_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_61_EFUSE1_ROW_61_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_61_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_62 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_62_EFUSE1_ROW_62_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_62_EFUSE1_ROW_62_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_62_EFUSE1_ROW_62_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_62_EFUSE1_ROW_62_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_62_RESETVAL                               (0x00000000U)

/* EFUSE1_ROW_63 */

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_63_EFUSE1_ROW_63_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_63_EFUSE1_ROW_63_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_63_EFUSE1_ROW_63_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_63_EFUSE1_ROW_63_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE1_ROW_63_RESETVAL                               (0x00000000U)

/* EFUSE2_ROW_5 */

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_5_EFUSE2_ROW_5_MASK                       (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_5_EFUSE2_ROW_5_SHIFT                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_5_EFUSE2_ROW_5_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_5_EFUSE2_ROW_5_MAX                        (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_5_RESETVAL                                (0x00000000U)

/* EFUSE2_ROW_6 */

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_6_EFUSE2_ROW_6_MASK                       (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_6_EFUSE2_ROW_6_SHIFT                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_6_EFUSE2_ROW_6_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_6_EFUSE2_ROW_6_MAX                        (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_6_RESETVAL                                (0x00000000U)

/* EFUSE2_ROW_7 */

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_7_EFUSE2_ROW_7_MASK                       (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_7_EFUSE2_ROW_7_SHIFT                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_7_EFUSE2_ROW_7_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_7_EFUSE2_ROW_7_MAX                        (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_7_RESETVAL                                (0x00000000U)

/* EFUSE2_ROW_8 */

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_8_EFUSE2_ROW_8_MASK                       (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_8_EFUSE2_ROW_8_SHIFT                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_8_EFUSE2_ROW_8_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_8_EFUSE2_ROW_8_MAX                        (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_EFUSE2_ROW_8_RESETVAL                                (0x00000000U)

/* RDSPARE1 */

#define CSL_BSS_GPCFG_REG_RDSPARE1_RDSPARE1_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_RDSPARE1_RDSPARE1_SHIFT                              (0x00000000U)
#define CSL_BSS_GPCFG_REG_RDSPARE1_RDSPARE1_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_RDSPARE1_RDSPARE1_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_RDSPARE1_RESETVAL                                    (0x00000000U)

/* RDSPARE2 */

#define CSL_BSS_GPCFG_REG_RDSPARE2_RDSPARE2_MASK                               (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_RDSPARE2_RDSPARE2_SHIFT                              (0x00000000U)
#define CSL_BSS_GPCFG_REG_RDSPARE2_RDSPARE2_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_RDSPARE2_RDSPARE2_MAX                                (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_RDSPARE2_RESETVAL                                    (0x00000000U)

/* FRCREG1 */

#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL1_MASK                               (0x00000001U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL1_SHIFT                              (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL1_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL1_MAX                                (0x00000001U)

#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL2_MASK                               (0x00000002U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL2_SHIFT                              (0x00000001U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL2_RESETVAL                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCNTRL2_MAX                                (0x00000001U)

#define CSL_BSS_GPCFG_REG_FRCREG1_NU1_MASK                                     (0x000000FCU)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU1_SHIFT                                    (0x00000002U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU1_MAX                                      (0x0000003FU)

#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCLKDISPULSE_MASK                          (0x00000100U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCLKDISPULSE_SHIFT                         (0x00000008U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCLKDISPULSE_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_ADCCLKDISPULSE_MAX                           (0x00000001U)

#define CSL_BSS_GPCFG_REG_FRCREG1_NU2_MASK                                     (0x0000FE00U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU2_SHIFT                                    (0x00000009U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU2_MAX                                      (0x0000007FU)

#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN1CTRMODE_MASK                          (0x00030000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN1CTRMODE_SHIFT                         (0x00000010U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN1CTRMODE_RESETVAL                      (0x00000003U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN1CTRMODE_MAX                           (0x00000003U)

#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN2CTRMODE_MASK                          (0x000C0000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN2CTRMODE_SHIFT                         (0x00000012U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN2CTRMODE_RESETVAL                      (0x00000003U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN2CTRMODE_MAX                           (0x00000003U)

#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN3CTRMODE_MASK                          (0x00300000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN3CTRMODE_SHIFT                         (0x00000014U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN3CTRMODE_RESETVAL                      (0x00000003U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN3CTRMODE_MAX                           (0x00000003U)

#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN4CTRMODE_MASK                          (0x00C00000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN4CTRMODE_SHIFT                         (0x00000016U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN4CTRMODE_RESETVAL                      (0x00000003U)
#define CSL_BSS_GPCFG_REG_FRCREG1_EVTGEN4CTRMODE_MAX                           (0x00000003U)

#define CSL_BSS_GPCFG_REG_FRCREG1_NU4_MASK                                     (0xFF000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU4_SHIFT                                    (0x00000018U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU4_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG1_NU4_MAX                                      (0x000000FFU)

#define CSL_BSS_GPCFG_REG_FRCREG1_RESETVAL                                     (0x00FF0000U)

/* FRCREG2 */

#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG1_MASK                                 (0x00000001U)
#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG1_SHIFT                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG1_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG1_MAX                                  (0x00000001U)

#define CSL_BSS_GPCFG_REG_FRCREG2_NU1_MASK                                     (0x000000FEU)
#define CSL_BSS_GPCFG_REG_FRCREG2_NU1_SHIFT                                    (0x00000001U)
#define CSL_BSS_GPCFG_REG_FRCREG2_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG2_NU1_MAX                                      (0x0000007FU)

#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG2_MASK                                 (0x00000100U)
#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG2_SHIFT                                (0x00000008U)
#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG2_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG2_SWTRIG2_MAX                                  (0x00000001U)

#define CSL_BSS_GPCFG_REG_FRCREG2_NU2_MASK                                     (0xFFFFFE00U)
#define CSL_BSS_GPCFG_REG_FRCREG2_NU2_SHIFT                                    (0x00000009U)
#define CSL_BSS_GPCFG_REG_FRCREG2_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG2_NU2_MAX                                      (0x007FFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG2_RESETVAL                                     (0x00000000U)

/* FRCREG3 */

#define CSL_BSS_GPCFG_REG_FRCREG3_TS1LSBCOUNT_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG3_TS1LSBCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG3_TS1LSBCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG3_TS1LSBCOUNT_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG3_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG3_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG3_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG3_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG3_RESETVAL                                     (0x00000000U)

/* FRCREG4 */

#define CSL_BSS_GPCFG_REG_FRCREG4_TS1MSBCOUNT_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG4_TS1MSBCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG4_TS1MSBCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG4_TS1MSBCOUNT_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG4_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG4_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG4_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG4_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG4_RESETVAL                                     (0x00000000U)

/* FRCREG5 */

#define CSL_BSS_GPCFG_REG_FRCREG5_TS2LSBCOUNT_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG5_TS2LSBCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG5_TS2LSBCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG5_TS2LSBCOUNT_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG5_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG5_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG5_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG5_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG5_RESETVAL                                     (0x00000000U)

/* FRCREG6 */

#define CSL_BSS_GPCFG_REG_FRCREG6_TS2MSBCOUNT_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG6_TS2MSBCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG6_TS2MSBCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG6_TS2MSBCOUNT_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG6_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG6_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG6_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG6_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG6_RESETVAL                                     (0x00000000U)

/* FRCREG7 */

#define CSL_BSS_GPCFG_REG_FRCREG7_TS1MIDCOUNT_MASK                             (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG7_TS1MIDCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_TS1MIDCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_TS1MIDCOUNT_MAX                              (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG7_NU1_MASK                                     (0x0000F000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_NU1_SHIFT                                    (0x0000000CU)
#define CSL_BSS_GPCFG_REG_FRCREG7_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_NU1_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG7_TS2MIDCOUNT_MASK                             (0x0FFF0000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_TS2MIDCOUNT_SHIFT                            (0x00000010U)
#define CSL_BSS_GPCFG_REG_FRCREG7_TS2MIDCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_TS2MIDCOUNT_MAX                              (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG7_NU2_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_NU2_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG7_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG7_NU2_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG7_RESETVAL                                     (0x00000000U)

/* FRCREG8 */

#define CSL_BSS_GPCFG_REG_FRCREG8_TS3LSBCOUNT_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG8_TS3LSBCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG8_TS3LSBCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG8_TS3LSBCOUNT_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG8_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG8_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG8_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG8_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG8_RESETVAL                                     (0x00000000U)

/* FRCREG9 */

#define CSL_BSS_GPCFG_REG_FRCREG9_TS3MSBCOUNT_MASK                             (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG9_TS3MSBCOUNT_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG9_TS3MSBCOUNT_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG9_TS3MSBCOUNT_MAX                              (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG9_NU_MASK                                      (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG9_NU_SHIFT                                     (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG9_NU_RESETVAL                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG9_NU_MAX                                       (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG9_RESETVAL                                     (0x00000000U)

/* FRCREG10 */

#define CSL_BSS_GPCFG_REG_FRCREG10_TS4LSBCOUNT_MASK                            (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG10_TS4LSBCOUNT_SHIFT                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG10_TS4LSBCOUNT_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG10_TS4LSBCOUNT_MAX                             (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG10_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG10_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG10_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG10_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG10_RESETVAL                                    (0x00000000U)

/* FRCREG11 */

#define CSL_BSS_GPCFG_REG_FRCREG11_TS4MSBCOUNT_MASK                            (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG11_TS4MSBCOUNT_SHIFT                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG11_TS4MSBCOUNT_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG11_TS4MSBCOUNT_MAX                             (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG11_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG11_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG11_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG11_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG11_RESETVAL                                    (0x00000000U)

/* FRCREG12 */

#define CSL_BSS_GPCFG_REG_FRCREG12_TS3MIDCOUNT_MASK                            (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG12_TS3MIDCOUNT_SHIFT                           (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_TS3MIDCOUNT_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_TS3MIDCOUNT_MAX                             (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG12_NU1_MASK                                    (0x0000F000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_NU1_SHIFT                                   (0x0000000CU)
#define CSL_BSS_GPCFG_REG_FRCREG12_NU1_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_NU1_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG12_TS4MIDCOUNT_MASK                            (0x0FFF0000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_TS4MIDCOUNT_SHIFT                           (0x00000010U)
#define CSL_BSS_GPCFG_REG_FRCREG12_TS4MIDCOUNT_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_TS4MIDCOUNT_MAX                             (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG12_NU2_MASK                                    (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_NU2_SHIFT                                   (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG12_NU2_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG12_NU2_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG12_RESETVAL                                    (0x00000000U)

/* FRCREG13 */

#define CSL_BSS_GPCFG_REG_FRCREG13_TS1TRIGSRCSEL_MASK                          (0x0000000FU)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS1TRIGSRCSEL_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS1TRIGSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS1TRIGSRCSEL_MAX                           (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_NU1_MASK                                    (0x000000F0U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU1_SHIFT                                   (0x00000004U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU1_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU1_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_TS2TRIGSRCSEL_MASK                          (0x00000F00U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS2TRIGSRCSEL_SHIFT                         (0x00000008U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS2TRIGSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS2TRIGSRCSEL_MAX                           (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_NU2_MASK                                    (0x0000F000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU2_SHIFT                                   (0x0000000CU)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU2_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU2_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_TS3TRIGSRCSEL_MASK                          (0x000F0000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS3TRIGSRCSEL_SHIFT                         (0x00000010U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS3TRIGSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS3TRIGSRCSEL_MAX                           (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_NU3_MASK                                    (0x00F00000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU3_SHIFT                                   (0x00000014U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU3_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU3_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_TS4TRIGSRCSEL_MASK                          (0x0F000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS4TRIGSRCSEL_SHIFT                         (0x00000018U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS4TRIGSRCSEL_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_TS4TRIGSRCSEL_MAX                           (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_NU4_MASK                                    (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU4_SHIFT                                   (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU4_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG13_NU4_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG13_RESETVAL                                    (0x00000000U)

/* FRCREG14 */

#define CSL_BSS_GPCFG_REG_FRCREG14_EVTGEN1LSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG14_EVTGEN1LSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG14_EVTGEN1LSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG14_EVTGEN1LSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG14_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG14_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG14_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG14_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG14_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG15 */

#define CSL_BSS_GPCFG_REG_FRCREG15_EVTGEN1MSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG15_EVTGEN1MSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG15_EVTGEN1MSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG15_EVTGEN1MSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG15_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG15_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG15_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG15_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG15_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG16 */

#define CSL_BSS_GPCFG_REG_FRCREG16_EVTGEN2LSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG16_EVTGEN2LSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG16_EVTGEN2LSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG16_EVTGEN2LSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG16_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG16_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG16_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG16_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG16_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG17 */

#define CSL_BSS_GPCFG_REG_FRCREG17_EVTGEN2MSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG17_EVTGEN2MSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG17_EVTGEN2MSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG17_EVTGEN2MSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG17_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG17_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG17_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG17_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG17_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG18 */

#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN1MIDPER_MASK                          (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN1MIDPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN1MIDPER_RESETVAL                      (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN1MIDPER_MAX                           (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG18_NU1_MASK                                    (0x0000F000U)
#define CSL_BSS_GPCFG_REG_FRCREG18_NU1_SHIFT                                   (0x0000000CU)
#define CSL_BSS_GPCFG_REG_FRCREG18_NU1_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG18_NU1_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN2MIDPER_MASK                          (0x0FFF0000U)
#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN2MIDPER_SHIFT                         (0x00000010U)
#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN2MIDPER_RESETVAL                      (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG18_EVTGEN2MIDPER_MAX                           (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG18_NU2_MASK                                    (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG18_NU2_SHIFT                                   (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG18_NU2_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG18_NU2_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG18_RESETVAL                                    (0x0FFF0FFFU)

/* FRCREG19 */

#define CSL_BSS_GPCFG_REG_FRCREG19_EVTGEN3LSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG19_EVTGEN3LSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG19_EVTGEN3LSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG19_EVTGEN3LSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG19_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG19_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG19_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG19_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG19_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG20 */

#define CSL_BSS_GPCFG_REG_FRCREG20_EVTGEN3MSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG20_EVTGEN3MSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG20_EVTGEN3MSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG20_EVTGEN3MSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG20_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG20_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG20_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG20_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG20_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG21 */

#define CSL_BSS_GPCFG_REG_FRCREG21_EVTGEN4LSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG21_EVTGEN4LSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG21_EVTGEN4LSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG21_EVTGEN4LSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG21_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG21_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG21_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG21_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG21_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG22 */

#define CSL_BSS_GPCFG_REG_FRCREG22_EVTGEN4MSBPER_MASK                          (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG22_EVTGEN4MSBPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG22_EVTGEN4MSBPER_RESETVAL                      (0x0FFFFFFFU)
#define CSL_BSS_GPCFG_REG_FRCREG22_EVTGEN4MSBPER_MAX                           (0x0FFFFFFFU)

#define CSL_BSS_GPCFG_REG_FRCREG22_NU_MASK                                     (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG22_NU_SHIFT                                    (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG22_NU_RESETVAL                                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG22_NU_MAX                                      (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG22_RESETVAL                                    (0x0FFFFFFFU)

/* FRCREG23 */

#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN3MIDPER_MASK                          (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN3MIDPER_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN3MIDPER_RESETVAL                      (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN3MIDPER_MAX                           (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG23_NU1_MASK                                    (0x0000F000U)
#define CSL_BSS_GPCFG_REG_FRCREG23_NU1_SHIFT                                   (0x0000000CU)
#define CSL_BSS_GPCFG_REG_FRCREG23_NU1_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG23_NU1_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN4MIDPER_MASK                          (0x0FFF0000U)
#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN4MIDPER_SHIFT                         (0x00000010U)
#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN4MIDPER_RESETVAL                      (0x00000FFFU)
#define CSL_BSS_GPCFG_REG_FRCREG23_EVTGEN4MIDPER_MAX                           (0x00000FFFU)

#define CSL_BSS_GPCFG_REG_FRCREG23_NU2_MASK                                    (0xF0000000U)
#define CSL_BSS_GPCFG_REG_FRCREG23_NU2_SHIFT                                   (0x0000001CU)
#define CSL_BSS_GPCFG_REG_FRCREG23_NU2_RESETVAL                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_FRCREG23_NU2_MAX                                     (0x0000000FU)

#define CSL_BSS_GPCFG_REG_FRCREG23_RESETVAL                                    (0x0FFF0FFFU)

/* SPARE9 */

#define CSL_BSS_GPCFG_REG_SPARE9_SPARE9_MASK                                   (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE9_SPARE9_SHIFT                                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE9_SPARE9_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE9_SPARE9_MAX                                    (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE9_RESETVAL                                      (0x00000000U)

/* SPARE10 */

#define CSL_BSS_GPCFG_REG_SPARE10_SPARE10_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_SPARE10_SPARE10_SHIFT                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE10_SPARE10_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_SPARE10_SPARE10_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_SPARE10_RESETVAL                                     (0x00000000U)

/* RX_STATUS_REG1_MASK */

#define CSL_BSS_GPCFG_REG_RX_STATUS_REG1_MASK_RX_STATUS_REG1_MASK_MASK         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_RX_STATUS_REG1_MASK_RX_STATUS_REG1_MASK_SHIFT        (0x00000000U)
#define CSL_BSS_GPCFG_REG_RX_STATUS_REG1_MASK_RX_STATUS_REG1_MASK_RESETVAL     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_RX_STATUS_REG1_MASK_RX_STATUS_REG1_MASK_MAX          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_RX_STATUS_REG1_MASK_RESETVAL                         (0xFFFFFFFFU)

/* TX_STAT_CLK_STAT_MASK */

#define CSL_BSS_GPCFG_REG_TX_STAT_CLK_STAT_MASK_TX_STAT_CLK_STAT_MASK_MASK     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_TX_STAT_CLK_STAT_MASK_TX_STAT_CLK_STAT_MASK_SHIFT    (0x00000000U)
#define CSL_BSS_GPCFG_REG_TX_STAT_CLK_STAT_MASK_TX_STAT_CLK_STAT_MASK_RESETVAL (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_TX_STAT_CLK_STAT_MASK_TX_STAT_CLK_STAT_MASK_MAX      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_TX_STAT_CLK_STAT_MASK_RESETVAL                       (0xFFFFFFFFU)

/* LODIST_WU_STAT_MASK */

#define CSL_BSS_GPCFG_REG_LODIST_WU_STAT_MASK_LODIST_WU_STAT_MASK_MASK         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_LODIST_WU_STAT_MASK_LODIST_WU_STAT_MASK_SHIFT        (0x00000000U)
#define CSL_BSS_GPCFG_REG_LODIST_WU_STAT_MASK_LODIST_WU_STAT_MASK_RESETVAL     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_LODIST_WU_STAT_MASK_LODIST_WU_STAT_MASK_MAX          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_LODIST_WU_STAT_MASK_RESETVAL                         (0xFFFFFFFFU)

/* CCMREG6 */

#define CSL_BSS_GPCFG_REG_CCMREG6_CCMMODST_MASK                                (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG6_CCMMODST_SHIFT                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG6_CCMMODST_RESETVAL                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG6_CCMMODST_MAX                                 (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG6_RESETVAL                                     (0x00000000U)

/* CCMREG0 */

#define CSL_BSS_GPCFG_REG_CCMREG0_CCMREG0_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG0_CCMREG0_SHIFT                                (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG0_CCMREG0_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG0_CCMREG0_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG0_RESETVAL                                     (0x00000000U)

/* CCMREG1 */

#define CSL_BSS_GPCFG_REG_CCMREG1_CCMCNT0EXPIRYVAL_MASK                        (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG1_CCMCNT0EXPIRYVAL_SHIFT                       (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG1_CCMCNT0EXPIRYVAL_RESETVAL                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG1_CCMCNT0EXPIRYVAL_MAX                         (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG1_RESETVAL                                     (0x00000000U)

/* CCMREG2 */

#define CSL_BSS_GPCFG_REG_CCMREG2_CCMCNT1EXPECTVAL_MASK                        (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG2_CCMCNT1EXPECTVAL_SHIFT                       (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG2_CCMCNT1EXPECTVAL_RESETVAL                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG2_CCMCNT1EXPECTVAL_MAX                         (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG2_RESETVAL                                     (0x00000000U)

/* CCMREG3 */

#define CSL_BSS_GPCFG_REG_CCMREG3_CCMMARGINCOUNT_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG3_CCMMARGINCOUNT_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG3_CCMMARGINCOUNT_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG3_CCMMARGINCOUNT_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG3_RESETVAL                                     (0x00000000U)

/* CCMREG4 */

#define CSL_BSS_GPCFG_REG_CCMREG4_CCMCNT1VAL_MASK                              (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG4_CCMCNT1VAL_SHIFT                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG4_CCMCNT1VAL_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG4_CCMCNT1VAL_MAX                               (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG4_RESETVAL                                     (0x00000000U)

/* CCMREG5 */

#define CSL_BSS_GPCFG_REG_CCMREG5_CCMTIMEOUTCNT_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_CCMREG5_CCMTIMEOUTCNT_SHIFT                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG5_CCMTIMEOUTCNT_RESETVAL                       (0x00000000U)
#define CSL_BSS_GPCFG_REG_CCMREG5_CCMTIMEOUTCNT_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_CCMREG5_RESETVAL                                     (0x00000000U)

/* MSS_ESM_SPARE */

#define CSL_BSS_GPCFG_REG_MSS_ESM_SPARE_MSS_ESM2_SPARE_MASK                    (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_MSS_ESM_SPARE_MSS_ESM2_SPARE_SHIFT                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_ESM_SPARE_MSS_ESM2_SPARE_RESETVAL                (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_ESM_SPARE_MSS_ESM2_SPARE_MAX                     (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_MSS_ESM_SPARE_RESETVAL                               (0x00000000U)

/* ESM_GP2_MASK1 */

#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK1_ESM_GP2_MASK1_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK1_ESM_GP2_MASK1_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK1_ESM_GP2_MASK1_RESETVAL                 (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK1_ESM_GP2_MASK1_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK1_RESETVAL                               (0xFFFFFFFFU)

/* ESM_GP2_MASK2 */

#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK2_ESM_GP2_MASK2_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK2_ESM_GP2_MASK2_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK2_ESM_GP2_MASK2_RESETVAL                 (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK2_ESM_GP2_MASK2_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK2_RESETVAL                               (0xFFFFFFFFU)

/* ESM_GP2_MASK3 */

#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK3_ESM_GP2_MASK3_MASK                     (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK3_ESM_GP2_MASK3_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK3_ESM_GP2_MASK3_RESETVAL                 (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK3_ESM_GP2_MASK3_MAX                      (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_ESM_GP2_MASK3_RESETVAL                               (0xFFFFFFFFU)

/* MSS_DSS_INTR_MASK */

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_DSS_FS_INTR_MASK_MASK              (0x00000001U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_DSS_FS_INTR_MASK_SHIFT             (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_DSS_FS_INTR_MASK_RESETVAL          (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_DSS_FS_INTR_MASK_MAX               (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_ST_INTR_MASK_MASK           (0x00000002U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_ST_INTR_MASK_SHIFT          (0x00000001U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_ST_INTR_MASK_RESETVAL       (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_ST_INTR_MASK_MAX            (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_END_INTR_MASK_MASK          (0x00000004U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_END_INTR_MASK_SHIFT         (0x00000002U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_END_INTR_MASK_RESETVAL      (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_FR_END_INTR_MASK_MAX           (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_ST_MASK_MASK             (0x00000008U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_ST_MASK_SHIFT            (0x00000003U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_ST_MASK_RESETVAL         (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_ST_MASK_MAX              (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_END_MASK_MASK            (0x00000010U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_END_MASK_SHIFT           (0x00000004U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_END_MASK_RESETVAL        (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_CHIRP_END_MASK_MAX             (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_ADC_VLD_FALL_MASK_MASK         (0x00000020U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_ADC_VLD_FALL_MASK_SHIFT        (0x00000005U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_ADC_VLD_FALL_MASK_RESETVAL     (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_MSS_ADC_VLD_FALL_MASK_MAX          (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU1_MASK                           (0x000000C0U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU1_SHIFT                          (0x00000006U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU1_RESETVAL                       (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU1_MAX                            (0x00000003U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_HI_MASK_MASK               (0x00000100U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_HI_MASK_SHIFT              (0x00000008U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_HI_MASK_RESETVAL           (0x00000001U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_HI_MASK_MAX                (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_LO_MASK_MASK               (0x00000200U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_LO_MASK_SHIFT              (0x00000009U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_LO_MASK_RESETVAL           (0x00000001U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_BSS_ESM_LO_MASK_MAX                (0x00000001U)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU2_MASK                           (0xFFFFFC00U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU2_SHIFT                          (0x0000000AU)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU2_RESETVAL                       (0x00000000U)
#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_NU2_MAX                            (0x003FFFFFU)

#define CSL_BSS_GPCFG_REG_MSS_DSS_INTR_MASK_RESETVAL                           (0x00000300U)

/* UDMA_INT1 */

#define CSL_BSS_GPCFG_REG_UDMA_INT1_UDMA_INT_STAT_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_UDMA_INT1_UDMA_INT_STAT_SHIFT                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_UDMA_INT1_UDMA_INT_STAT_RESETVAL                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_UDMA_INT1_UDMA_INT_STAT_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_UDMA_INT1_RESETVAL                                   (0x00000000U)

/* UDMA_INT2 */

#define CSL_BSS_GPCFG_REG_UDMA_INT2_UDMA_INT_CLR_MASK                          (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_UDMA_INT2_UDMA_INT_CLR_SHIFT                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_UDMA_INT2_UDMA_INT_CLR_RESETVAL                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_UDMA_INT2_UDMA_INT_CLR_MAX                           (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_UDMA_INT2_RESETVAL                                   (0x00000000U)

/* UDMA_INT3 */

#define CSL_BSS_GPCFG_REG_UDMA_INT3_UDMA_INT_MASK_MASK                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_UDMA_INT3_UDMA_INT_MASK_SHIFT                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_UDMA_INT3_UDMA_INT_MASK_RESETVAL                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_UDMA_INT3_UDMA_INT_MASK_MAX                          (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_UDMA_INT3_RESETVAL                                   (0x00000000U)

/* TCMA_PARITY */

#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_CLR_MASK                    (0x00000001U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_CLR_SHIFT                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_CLR_RESETVAL                (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_CLR_MAX                     (0x00000001U)

#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ATCMFORCE_ERR_MASK                       (0x0000000EU)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ATCMFORCE_ERR_SHIFT                      (0x00000001U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ATCMFORCE_ERR_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ATCMFORCE_ERR_MAX                        (0x00000007U)

#define CSL_BSS_GPCFG_REG_TCMA_PARITY_NU_MASK                                  (0x000000F0U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_NU_SHIFT                                 (0x00000004U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_NU_RESETVAL                              (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_NU_MAX                                   (0x0000000FU)

#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_MASK                        (0xFFFFFF00U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_SHIFT                       (0x00000008U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_RESETVAL                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMA_PARITY_ERR_ATCMADDR_MAX                         (0x00FFFFFFU)

#define CSL_BSS_GPCFG_REG_TCMA_PARITY_RESETVAL                                 (0x00000000U)

/* TCMB0_PARITY */

#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_CLR_MASK                  (0x00000001U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_CLR_SHIFT                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_CLR_RESETVAL              (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_CLR_MAX                   (0x00000001U)

#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_B0TCMFORCE_ERR_MASK                     (0x0000000EU)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_B0TCMFORCE_ERR_SHIFT                    (0x00000001U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_B0TCMFORCE_ERR_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_B0TCMFORCE_ERR_MAX                      (0x00000007U)

#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_NU_MASK                                 (0x000000F0U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_NU_SHIFT                                (0x00000004U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_NU_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_NU_MAX                                  (0x0000000FU)

#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_MASK                      (0xFFFFFF00U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_SHIFT                     (0x00000008U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_RESETVAL                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_ERR_B0TCMADDR_MAX                       (0x00FFFFFFU)

#define CSL_BSS_GPCFG_REG_TCMB0_PARITY_RESETVAL                                (0x00000000U)

/* TCMB1_PARITY */

#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_CLR_MASK                  (0x00000001U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_CLR_SHIFT                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_CLR_RESETVAL              (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_CLR_MAX                   (0x00000001U)

#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_B1TCMFORCE_ERR_MASK                     (0x0000000EU)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_B1TCMFORCE_ERR_SHIFT                    (0x00000001U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_B1TCMFORCE_ERR_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_B1TCMFORCE_ERR_MAX                      (0x00000007U)

#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_NU_MASK                                 (0x000000F0U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_NU_SHIFT                                (0x00000004U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_NU_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_NU_MAX                                  (0x0000000FU)

#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_MASK                      (0xFFFFFF00U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_SHIFT                     (0x00000008U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_RESETVAL                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_ERR_B1TCMADDR_MAX                       (0x00FFFFFFU)

#define CSL_BSS_GPCFG_REG_TCMB1_PARITY_RESETVAL                                (0x00000000U)

/* TCMECCCHK */

#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMAEZDIS_MASK                             (0x00000007U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMAEZDIS_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMAEZDIS_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMAEZDIS_MAX                              (0x00000007U)

#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU1_MASK                                   (0x000000F8U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU1_SHIFT                                  (0x00000003U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU1_MAX                                    (0x0000001FU)

#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB0EZDIS_MASK                            (0x00000700U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB0EZDIS_SHIFT                           (0x00000008U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB0EZDIS_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB0EZDIS_MAX                             (0x00000007U)

#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU2_MASK                                   (0x0000F800U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU2_SHIFT                                  (0x0000000BU)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU2_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU2_MAX                                    (0x0000001FU)

#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB1EZDIS_MASK                            (0x00070000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB1EZDIS_SHIFT                           (0x00000010U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB1EZDIS_RESETVAL                        (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_TCMB1EZDIS_MAX                             (0x00000007U)

#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU3_MASK                                   (0xFFF80000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU3_SHIFT                                  (0x00000013U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU3_RESETVAL                               (0x00000000U)
#define CSL_BSS_GPCFG_REG_TCMECCCHK_NU3_MAX                                    (0x00001FFFU)

#define CSL_BSS_GPCFG_REG_TCMECCCHK_RESETVAL                                   (0x00000000U)

/* TXPALOOPBCKCLK */

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVEN_MASK                     (0x00000001U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVEN_SHIFT                    (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVEN_RESETVAL                 (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVEN_MAX                      (0x00000001U)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKFREERUNEN_MASK                 (0x00000002U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKFREERUNEN_SHIFT                (0x00000001U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKFREERUNEN_RESETVAL             (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKFREERUNEN_MAX                  (0x00000001U)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVCLKSEL_MASK                 (0x00000004U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVCLKSEL_SHIFT                (0x00000002U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVCLKSEL_RESETVAL             (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVCLKSEL_MAX                  (0x00000001U)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU1_MASK                              (0x000000F8U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU1_SHIFT                             (0x00000003U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU1_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU1_MAX                               (0x0000001FU)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL1_MASK                   (0x00001F00U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL1_SHIFT                  (0x00000008U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL1_RESETVAL               (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL1_MAX                    (0x0000001FU)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU2_MASK                              (0x0000E000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU2_SHIFT                             (0x0000000DU)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU2_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU2_MAX                               (0x00000007U)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL2_MASK                   (0x00FF0000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL2_SHIFT                  (0x00000010U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL2_RESETVAL               (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_TXPACLKDIVVAL2_MAX                    (0x000000FFU)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU3_MASK                              (0xFF000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU3_SHIFT                             (0x00000018U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU3_RESETVAL                          (0x00000000U)
#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_NU3_MAX                               (0x000000FFU)

#define CSL_BSS_GPCFG_REG_TXPALOOPBCKCLK_RESETVAL                              (0x00000000U)

/* RXIFALOOPBCKCLK */

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVEN_MASK                      (0x00000001U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVEN_SHIFT                     (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVEN_RESETVAL                  (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVEN_MAX                       (0x00000001U)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFAFREERUNEN_MASK                  (0x00000002U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFAFREERUNEN_SHIFT                 (0x00000001U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFAFREERUNEN_RESETVAL              (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFAFREERUNEN_MAX                   (0x00000001U)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVCLKSEL_MASK                  (0x00000004U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVCLKSEL_SHIFT                 (0x00000002U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVCLKSEL_RESETVAL              (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFADIVCLKSEL_MAX                   (0x00000001U)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU1_MASK                             (0x000000F8U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU1_SHIFT                            (0x00000003U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU1_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU1_MAX                              (0x0000001FU)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL1_MASK                 (0x00001F00U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL1_SHIFT                (0x00000008U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL1_RESETVAL             (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL1_MAX                  (0x0000001FU)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU2_MASK                             (0x0000E000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU2_SHIFT                            (0x0000000DU)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU2_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU2_MAX                              (0x00000007U)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL2_MASK                 (0x00FF0000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL2_SHIFT                (0x00000010U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL2_RESETVAL             (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RXIFACLKDIVVAL2_MAX                  (0x000000FFU)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU3_MASK                             (0xFF000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU3_SHIFT                            (0x00000018U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU3_RESETVAL                         (0x00000000U)
#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_NU3_MAX                              (0x000000FFU)

#define CSL_BSS_GPCFG_REG_RXIFALOOPBCKCLK_RESETVAL                             (0x00000000U)

/* DBGACKCTL */

#define CSL_BSS_GPCFG_REG_DBGACKCTL_DBGACKCTL_MASK                             (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_DBGACKCTL_DBGACKCTL_SHIFT                            (0x00000000U)
#define CSL_BSS_GPCFG_REG_DBGACKCTL_DBGACKCTL_RESETVAL                         (0xFFFFFFFFU)
#define CSL_BSS_GPCFG_REG_DBGACKCTL_DBGACKCTL_MAX                              (0xFFFFFFFFU)

#define CSL_BSS_GPCFG_REG_DBGACKCTL_RESETVAL                                   (0xFFFFFFFFU)

/* AXI2VBUSPSEL */

#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_AXI2VBUSPSEL_MASK                       (0x00000007U)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_AXI2VBUSPSEL_SHIFT                      (0x00000000U)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_AXI2VBUSPSEL_RESETVAL                   (0x00000000U)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_AXI2VBUSPSEL_MAX                        (0x00000007U)

#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_NU_MASK                                 (0xFFFFFFF8U)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_NU_SHIFT                                (0x00000003U)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_NU_RESETVAL                             (0x00000000U)
#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_NU_MAX                                  (0x1FFFFFFFU)

#define CSL_BSS_GPCFG_REG_AXI2VBUSPSEL_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
