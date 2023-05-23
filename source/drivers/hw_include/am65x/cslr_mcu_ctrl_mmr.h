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
*  Name        : cslr_mcu_ctrl_mmr.h
*/
#ifndef CSLR_MCU_CTRL_MMR_H_
#define CSLR_MCU_CTRL_MMR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_MCU_CTRL_MMR_CFG0_REGS_BASE                                        (0x00000000U)


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
    volatile uint8_t  Resv_256[244];
    volatile uint32_t IPC_SET0;
    volatile uint32_t IPC_SET1;
    volatile uint8_t  Resv_288[24];
    volatile uint32_t IPC_SET8;
    volatile uint8_t  Resv_384[92];
    volatile uint32_t IPC_CLR0;
    volatile uint32_t IPC_CLR1;
    volatile uint8_t  Resv_416[24];
    volatile uint32_t IPC_CLR8;
    volatile uint8_t  Resv_512[92];
    volatile uint32_t MAC_ID0;
    volatile uint32_t MAC_ID1;
    volatile uint8_t  Resv_4104[3584];
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
    volatile uint8_t  Resv_8192[3820];
    volatile uint32_t PID_PROXY;                 /* PID register */
    volatile uint32_t MMR_CFG0_PROXY;
    volatile uint32_t MMR_CFG1_PROXY;
    volatile uint8_t  Resv_8448[244];
    volatile uint32_t IPC_SET0_PROXY;
    volatile uint32_t IPC_SET1_PROXY;
    volatile uint8_t  Resv_8480[24];
    volatile uint32_t IPC_SET8_PROXY;
    volatile uint8_t  Resv_8576[92];
    volatile uint32_t IPC_CLR0_PROXY;
    volatile uint32_t IPC_CLR1_PROXY;
    volatile uint8_t  Resv_8608[24];
    volatile uint32_t IPC_CLR8_PROXY;
    volatile uint8_t  Resv_8704[92];
    volatile uint32_t MAC_ID0_PROXY;
    volatile uint32_t MAC_ID1_PROXY;
    volatile uint8_t  Resv_12296[3584];
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
    volatile uint8_t  Resv_16432[3868];
    volatile uint32_t MSMC_CFG;
    volatile uint8_t  Resv_16448[12];
    volatile uint32_t MCU_ENET_CTRL;
    volatile uint8_t  Resv_16480[28];
    volatile uint32_t MCU_SPI1_CTRL;
    volatile uint8_t  Resv_16544[60];
    volatile uint32_t MCU_FSS_CTRL;
    volatile uint8_t  Resv_16560[12];
    volatile uint32_t MCU_ADC0_CTRL;
    volatile uint32_t MCU_ADC1_CTRL;
    volatile uint8_t  Resv_16896[328];
    volatile uint32_t MCU_TIMER0_CTRL;
    volatile uint32_t MCU_TIMER1_CTRL;
    volatile uint32_t MCU_TIMER2_CTRL;
    volatile uint32_t MCU_TIMER3_CTRL;
    volatile uint8_t  Resv_17024[112];
    volatile uint32_t MCU_TIMERIO0_CTRL;
    volatile uint32_t MCU_TIMERIO1_CTRL;
    volatile uint8_t  Resv_20488[3456];
    volatile uint32_t LOCK1_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK1_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_20736[240];
    volatile uint32_t CLAIMREG_P1_R0_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R1_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R2_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R3_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R4_READONLY;   /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R5_READONLY;   /* Claim bits for Partition 1 */
    volatile uint8_t  Resv_24624[3864];
    volatile uint32_t MSMC_CFG_PROXY;
    volatile uint8_t  Resv_24640[12];
    volatile uint32_t MCU_ENET_CTRL_PROXY;
    volatile uint8_t  Resv_24672[28];
    volatile uint32_t MCU_SPI1_CTRL_PROXY;
    volatile uint8_t  Resv_24736[60];
    volatile uint32_t MCU_FSS_CTRL_PROXY;
    volatile uint8_t  Resv_24752[12];
    volatile uint32_t MCU_ADC0_CTRL_PROXY;
    volatile uint32_t MCU_ADC1_CTRL_PROXY;
    volatile uint8_t  Resv_25088[328];
    volatile uint32_t MCU_TIMER0_CTRL_PROXY;
    volatile uint32_t MCU_TIMER1_CTRL_PROXY;
    volatile uint32_t MCU_TIMER2_CTRL_PROXY;
    volatile uint32_t MCU_TIMER3_CTRL_PROXY;
    volatile uint8_t  Resv_25216[112];
    volatile uint32_t MCU_TIMERIO0_CTRL_PROXY;
    volatile uint32_t MCU_TIMERIO1_CTRL_PROXY;
    volatile uint8_t  Resv_28680[3456];
    volatile uint32_t LOCK1_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK1_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_28928[240];
    volatile uint32_t CLAIMREG_P1_R0;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R1;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R2;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R3;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R4;            /* Claim bits for Partition 1 */
    volatile uint32_t CLAIMREG_P1_R5;            /* Claim bits for Partition 1 */
    volatile uint8_t  Resv_32784[3832];
    volatile uint32_t MCU_CLKOUT0_CTRL;
    volatile uint8_t  Resv_32792[4];
    volatile uint32_t MCU_EFUSE_CLKSEL;
    volatile uint8_t  Resv_32800[4];
    volatile uint32_t MCU_MCAN0_CLKSEL;
    volatile uint32_t MCU_MCAN1_CLKSEL;
    volatile uint8_t  Resv_32816[8];
    volatile uint32_t MCU_OSPI0_CLKSEL;
    volatile uint32_t MCU_OSPI1_CLKSEL;
    volatile uint8_t  Resv_32832[8];
    volatile uint32_t MCU_ADC0_CLKSEL;
    volatile uint32_t MCU_ADC1_CLKSEL;
    volatile uint8_t  Resv_32848[8];
    volatile uint32_t MCU_ENET_CLKSEL;
    volatile uint8_t  Resv_32896[44];
    volatile uint32_t MCU_R5_CORE0_CLKSEL;
    volatile uint32_t MCU_R5_CORE1_CLKSEL;
    volatile uint8_t  Resv_33024[120];
    volatile uint32_t MCU_TIMER0_CLKSEL;
    volatile uint32_t MCU_TIMER1_CLKSEL;
    volatile uint32_t MCU_TIMER2_CLKSEL;
    volatile uint32_t MCU_TIMER3_CLKSEL;
    volatile uint8_t  Resv_33152[112];
    volatile uint32_t MCU_RTI0_CLKSEL;
    volatile uint32_t MCU_RTI1_CLKSEL;
    volatile uint8_t  Resv_33216[56];
    volatile uint32_t MCU_USART_CLKSEL;
    volatile uint8_t  Resv_36872[3652];
    volatile uint32_t LOCK2_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK2_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_37120[240];
    volatile uint32_t CLAIMREG_P2_R0_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R1_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R2_READONLY;   /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R3_READONLY;   /* Claim bits for Partition 2 */
    volatile uint8_t  Resv_40976[3840];
    volatile uint32_t MCU_CLKOUT0_CTRL_PROXY;
    volatile uint8_t  Resv_40984[4];
    volatile uint32_t MCU_EFUSE_CLKSEL_PROXY;
    volatile uint8_t  Resv_40992[4];
    volatile uint32_t MCU_MCAN0_CLKSEL_PROXY;
    volatile uint32_t MCU_MCAN1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41008[8];
    volatile uint32_t MCU_OSPI0_CLKSEL_PROXY;
    volatile uint32_t MCU_OSPI1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41024[8];
    volatile uint32_t MCU_ADC0_CLKSEL_PROXY;
    volatile uint32_t MCU_ADC1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41040[8];
    volatile uint32_t MCU_ENET_CLKSEL_PROXY;
    volatile uint8_t  Resv_41088[44];
    volatile uint32_t MCU_R5_CORE0_CLKSEL_PROXY;
    volatile uint32_t MCU_R5_CORE1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41216[120];
    volatile uint32_t MCU_TIMER0_CLKSEL_PROXY;
    volatile uint32_t MCU_TIMER1_CLKSEL_PROXY;
    volatile uint32_t MCU_TIMER2_CLKSEL_PROXY;
    volatile uint32_t MCU_TIMER3_CLKSEL_PROXY;
    volatile uint8_t  Resv_41344[112];
    volatile uint32_t MCU_RTI0_CLKSEL_PROXY;
    volatile uint32_t MCU_RTI1_CLKSEL_PROXY;
    volatile uint8_t  Resv_41408[56];
    volatile uint32_t MCU_USART_CLKSEL_PROXY;
    volatile uint8_t  Resv_45064[3652];
    volatile uint32_t LOCK2_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK2_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_45312[240];
    volatile uint32_t CLAIMREG_P2_R0;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R1;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R2;            /* Claim bits for Partition 2 */
    volatile uint32_t CLAIMREG_P2_R3;            /* Claim bits for Partition 2 */
    volatile uint8_t  Resv_49152[3824];
    volatile uint32_t MCU_LBIST_CTRL;
    volatile uint32_t MCU_LBIST_PATCOUNT;
    volatile uint32_t MCU_LBIST_SEED0;
    volatile uint32_t MCU_LBIST_SEED1;
    volatile uint32_t MCU_LBIST_SPARE0;
    volatile uint32_t MCU_LBIST_SPARE1;
    volatile uint32_t MCU_LBIST_STAT;
    volatile uint32_t MCU_LBIST_MISR;
    volatile uint8_t  Resv_49792[608];
    volatile uint32_t MCU_LBIST_SIG;
    volatile uint8_t  Resv_53256[3460];
    volatile uint32_t LOCK3_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK3_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_53504[240];
    volatile uint32_t CLAIMREG_P3_R0_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R1_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R2_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R3_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R4_READONLY;   /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R5_READONLY;   /* Claim bits for Partition 3 */
    volatile uint8_t  Resv_57344[3816];
    volatile uint32_t MCU_LBIST_CTRL_PROXY;
    volatile uint32_t MCU_LBIST_PATCOUNT_PROXY;
    volatile uint32_t MCU_LBIST_SEED0_PROXY;
    volatile uint32_t MCU_LBIST_SEED1_PROXY;
    volatile uint32_t MCU_LBIST_SPARE0_PROXY;
    volatile uint32_t MCU_LBIST_SPARE1_PROXY;
    volatile uint32_t MCU_LBIST_STAT_PROXY;
    volatile uint32_t MCU_LBIST_MISR_PROXY;
    volatile uint8_t  Resv_57984[608];
    volatile uint32_t MCU_LBIST_SIG_PROXY;
    volatile uint8_t  Resv_61448[3460];
    volatile uint32_t LOCK3_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK3_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_61696[240];
    volatile uint32_t CLAIMREG_P3_R0;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R1;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R2;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R3;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R4;            /* Claim bits for Partition 3 */
    volatile uint32_t CLAIMREG_P3_R5;            /* Claim bits for Partition 3 */
    volatile uint8_t  Resv_65536[3816];
    volatile uint32_t CORE_PASS_DONE_SET0;
    volatile uint8_t  Resv_65552[12];
    volatile uint32_t CORE_PASS_DONE_CLR0;
    volatile uint8_t  Resv_65568[12];
    volatile uint32_t ACTIVE_CORES;
    volatile uint8_t  Resv_65584[12];
    volatile uint32_t CORE_DEBUG0;
    volatile uint32_t CORE_DEBUG1;
    volatile uint32_t CORE_DEBUG2;
    volatile uint32_t CORE_DEBUG3;
    volatile uint8_t  Resv_65616[16];
    volatile uint32_t DBG_FEATURE0;
    volatile uint32_t DBG_FEATURE1;
    volatile uint32_t DBG_FEATURE2;
    volatile uint32_t DBG_FEATURE3;
    volatile uint8_t  Resv_65664[32];
    volatile uint32_t TEST_DEBUG_PORRST0;
    volatile uint8_t  Resv_65696[28];
    volatile uint32_t TEST_DEBUG_ALLRST0;
    volatile uint8_t  Resv_69640[3940];
    volatile uint32_t LOCK4_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK4_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_69888[240];
    volatile uint32_t CLAIMREG_P4_R0_READONLY;   /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R1_READONLY;   /* Claim bits for Partition 4 */
    volatile uint8_t  Resv_73728[3832];
    volatile uint32_t CORE_PASS_DONE_SET0_PROXY;
    volatile uint8_t  Resv_73744[12];
    volatile uint32_t CORE_PASS_DONE_CLR0_PROXY;
    volatile uint8_t  Resv_73760[12];
    volatile uint32_t ACTIVE_CORES_PROXY;
    volatile uint8_t  Resv_73776[12];
    volatile uint32_t CORE_DEBUG0_PROXY;
    volatile uint32_t CORE_DEBUG1_PROXY;
    volatile uint32_t CORE_DEBUG2_PROXY;
    volatile uint32_t CORE_DEBUG3_PROXY;
    volatile uint8_t  Resv_73808[16];
    volatile uint32_t DBG_FEATURE0_PROXY;
    volatile uint32_t DBG_FEATURE1_PROXY;
    volatile uint32_t DBG_FEATURE2_PROXY;
    volatile uint32_t DBG_FEATURE3_PROXY;
    volatile uint8_t  Resv_73856[32];
    volatile uint32_t TEST_DEBUG_PORRST0_PROXY;
    volatile uint8_t  Resv_73888[28];
    volatile uint32_t TEST_DEBUG_ALLRST0_PROXY;
    volatile uint8_t  Resv_77832[3940];
    volatile uint32_t LOCK4_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK4_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_78080[240];
    volatile uint32_t CLAIMREG_P4_R0;            /* Claim bits for Partition 4 */
    volatile uint32_t CLAIMREG_P4_R1;            /* Claim bits for Partition 4 */
    volatile uint8_t  Resv_98352[20264];
    volatile uint32_t MCU_SRAM_LDO_CTRL;
    volatile uint8_t  Resv_102408[4052];
    volatile uint32_t LOCK6_KICK0;               /*  - KICK0 component */
    volatile uint32_t LOCK6_KICK1;               /*  - KICK1 component */
    volatile uint8_t  Resv_102656[240];
    volatile uint32_t CLAIMREG_P6_R0_READONLY;   /* Claim bits for Partition 6 */
    volatile uint8_t  Resv_106544[3884];
    volatile uint32_t MCU_SRAM_LDO_CTRL_PROXY;
    volatile uint8_t  Resv_110600[4052];
    volatile uint32_t LOCK6_KICK0_PROXY;         /*  - KICK0 component */
    volatile uint32_t LOCK6_KICK1_PROXY;         /*  - KICK1 component */
    volatile uint8_t  Resv_110848[240];
    volatile uint32_t CLAIMREG_P6_R0;            /* Claim bits for Partition 6 */
} CSL_mcu_ctrl_mmr_cfg0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MCU_CTRL_MMR_CFG0_PID                                              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0                                         (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1                                         (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0                                         (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1                                         (0x00000104U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8                                         (0x00000120U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0                                         (0x00000180U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1                                         (0x00000184U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8                                         (0x000001A0U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0                                          (0x00000200U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1                                          (0x00000204U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0                                      (0x00001008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1                                      (0x0000100CU)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS                                  (0x00001010U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR                        (0x00001014U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE                                      (0x00001018U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR                                (0x0000101CU)
#define CSL_MCU_CTRL_MMR_CFG0_EOI                                              (0x00001020U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS                                    (0x00001024U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS                                (0x00001028U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS                                (0x0000102CU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR                                      (0x00001030U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY                          (0x00001100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY                          (0x00001104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY                          (0x00001108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY                          (0x0000110CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY                          (0x00001110U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY                                        (0x00002000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY                                   (0x00002004U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY                                   (0x00002008U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY                                   (0x00002100U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY                                   (0x00002104U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY                                   (0x00002120U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY                                   (0x00002180U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY                                   (0x00002184U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY                                   (0x000021A0U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_PROXY                                    (0x00002200U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_PROXY                                    (0x00002204U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY                                (0x00003008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY                                (0x0000300CU)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY                            (0x00003010U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY                  (0x00003014U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY                                (0x00003018U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY                          (0x0000301CU)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_PROXY                                        (0x00003020U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY                              (0x00003024U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY                          (0x00003028U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY                          (0x0000302CU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY                                (0x00003030U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0                                   (0x00003100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1                                   (0x00003104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2                                   (0x00003108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3                                   (0x0000310CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4                                   (0x00003110U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG                                         (0x00004030U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL                                    (0x00004040U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL                                    (0x00004060U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL                                     (0x000040A0U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL                                    (0x000040B0U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL                                    (0x000040B4U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL                                  (0x00004200U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL                                  (0x00004204U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL                                  (0x00004208U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL                                  (0x0000420CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL                                (0x00004280U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL                                (0x00004284U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0                                      (0x00005008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1                                      (0x0000500CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY                          (0x00005100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY                          (0x00005104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY                          (0x00005108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY                          (0x0000510CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY                          (0x00005110U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY                          (0x00005114U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_PROXY                                   (0x00006030U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY                              (0x00006040U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_PROXY                              (0x00006060U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY                               (0x000060A0U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_PROXY                              (0x000060B0U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_PROXY                              (0x000060B4U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_PROXY                            (0x00006200U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY                            (0x00006204U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_PROXY                            (0x00006208U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY                            (0x0000620CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_PROXY                          (0x00006280U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_PROXY                          (0x00006284U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY                                (0x00007008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY                                (0x0000700CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0                                   (0x00007100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1                                   (0x00007104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2                                   (0x00007108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3                                   (0x0000710CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4                                   (0x00007110U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5                                   (0x00007114U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL                                 (0x00008010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL                                 (0x00008018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL                                 (0x00008020U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL                                 (0x00008024U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL                                 (0x00008030U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL                                 (0x00008034U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL                                  (0x00008040U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL                                  (0x00008044U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL                                  (0x00008050U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL                              (0x00008080U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL                              (0x00008084U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL                                (0x00008100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL                                (0x00008104U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL                                (0x00008108U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL                                (0x0000810CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL                                  (0x00008180U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL                                  (0x00008184U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL                                 (0x000081C0U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0                                      (0x00009008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1                                      (0x0000900CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY                          (0x00009100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY                          (0x00009104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY                          (0x00009108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY                          (0x0000910CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY                           (0x0000A010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_PROXY                           (0x0000A018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_PROXY                           (0x0000A020U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_PROXY                           (0x0000A024U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY                           (0x0000A030U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY                           (0x0000A034U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_PROXY                            (0x0000A040U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_PROXY                            (0x0000A044U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_PROXY                            (0x0000A050U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_PROXY                        (0x0000A080U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_PROXY                        (0x0000A084U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_PROXY                          (0x0000A100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_PROXY                          (0x0000A104U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_PROXY                          (0x0000A108U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_PROXY                          (0x0000A10CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY                            (0x0000A180U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY                            (0x0000A184U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_PROXY                           (0x0000A1C0U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY                                (0x0000B008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY                                (0x0000B00CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0                                   (0x0000B100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1                                   (0x0000B104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2                                   (0x0000B108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3                                   (0x0000B10CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL                                   (0x0000C000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT                               (0x0000C004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0                                  (0x0000C008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1                                  (0x0000C00CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0                                 (0x0000C010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1                                 (0x0000C014U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT                                   (0x0000C018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR                                   (0x0000C01CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG                                    (0x0000C280U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0                                      (0x0000D008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1                                      (0x0000D00CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY                          (0x0000D100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY                          (0x0000D104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY                          (0x0000D108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY                          (0x0000D10CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY                          (0x0000D110U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY                          (0x0000D114U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY                             (0x0000E000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY                         (0x0000E004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PROXY                            (0x0000E008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PROXY                            (0x0000E00CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY                           (0x0000E010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_PROXY                           (0x0000E014U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY                             (0x0000E018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_PROXY                             (0x0000E01CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_PROXY                              (0x0000E280U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY                                (0x0000F008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY                                (0x0000F00CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0                                   (0x0000F100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1                                   (0x0000F104U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2                                   (0x0000F108U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3                                   (0x0000F10CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4                                   (0x0000F110U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5                                   (0x0000F114U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0                              (0x00010000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0                              (0x00010010U)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES                                     (0x00010020U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0                                      (0x00010030U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1                                      (0x00010034U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2                                      (0x00010038U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3                                      (0x0001003CU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0                                     (0x00010050U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1                                     (0x00010054U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2                                     (0x00010058U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3                                     (0x0001005CU)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0                               (0x00010080U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0                               (0x000100A0U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0                                      (0x00011008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1                                      (0x0001100CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY                          (0x00011100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY                          (0x00011104U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_PROXY                        (0x00012000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_PROXY                        (0x00012010U)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_PROXY                               (0x00012020U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_PROXY                                (0x00012030U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_PROXY                                (0x00012034U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_PROXY                                (0x00012038U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_PROXY                                (0x0001203CU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_PROXY                               (0x00012050U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_PROXY                               (0x00012054U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_PROXY                               (0x00012058U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_PROXY                               (0x0001205CU)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_PROXY                         (0x00012080U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_PROXY                         (0x000120A0U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY                                (0x00013008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY                                (0x0001300CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0                                   (0x00013100U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1                                   (0x00013104U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL                                (0x00018030U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0                                      (0x00019008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1                                      (0x0001900CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY                          (0x00019100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY                          (0x0001A030U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY                                (0x0001B008U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY                                (0x0001B00CU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0                                   (0x0001B100U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MINOR_MASK                               (0x0000003FU)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MINOR_SHIFT                              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MINOR_RESETVAL                           (0x00000012U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MINOR_MAX                                (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_CUSTOM_MASK                              (0x000000C0U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_CUSTOM_SHIFT                             (0x00000006U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_CUSTOM_RESETVAL                          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_CUSTOM_MAX                               (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MAJOR_MASK                               (0x00000700U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MAJOR_SHIFT                              (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MAJOR_RESETVAL                           (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MAJOR_MAX                                (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MISC_MASK                                (0x0000F800U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MISC_SHIFT                               (0x0000000BU)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MISC_RESETVAL                            (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MISC_MAX                                 (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MSB16_MASK                               (0xFFFF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MSB16_SHIFT                              (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MSB16_RESETVAL                           (0x00006180U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PID_MSB16_MAX                                (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_PID_RESETVAL                                     (0x61800212U)

/* MMR_CFG0 */

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_MASK                           (0x0000FFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_SHIFT                          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_RESETVAL                       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_SPEC_REV_MAX                            (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_MASK                            (0xFFFF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_SHIFT                           (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_RESETVAL                        (0x00000018U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_CFG_REV_MAX                             (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_RESETVAL                                (0x00180000U)

/* MMR_CFG1 */

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_MASK                         (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_SHIFT                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_RESETVAL                     (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PARTITIONS_MAX                          (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_MASK                           (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_SHIFT                          (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_RESETVAL                       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_EN_MAX                            (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_RESETVAL                                (0x8000001FU)

/* IPC_SET0 */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SET_MASK                            (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SET_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SET_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SET_MAX                             (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SRC_SET_MASK                        (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SRC_SET_SHIFT                       (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SRC_SET_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_IPC_SRC_SET_MAX                         (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_RESETVAL                                (0x00000000U)

/* IPC_SET1 */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SET_MASK                            (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SET_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SET_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SET_MAX                             (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SRC_SET_MASK                        (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SRC_SET_SHIFT                       (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SRC_SET_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_IPC_SRC_SET_MAX                         (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_RESETVAL                                (0x00000000U)

/* IPC_SET8 */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SET_MASK                            (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SET_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SET_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SET_MAX                             (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SRC_SET_MASK                        (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SRC_SET_SHIFT                       (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SRC_SET_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_IPC_SRC_SET_MAX                         (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_RESETVAL                                (0x00000000U)

/* IPC_CLR0 */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_CLR_MASK                            (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_CLR_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_CLR_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_CLR_MAX                             (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_SRC_CLR_MASK                        (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_SRC_CLR_SHIFT                       (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_SRC_CLR_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_IPC_SRC_CLR_MAX                         (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_RESETVAL                                (0x00000000U)

/* IPC_CLR1 */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_CLR_MASK                            (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_CLR_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_CLR_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_CLR_MAX                             (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_SRC_CLR_MASK                        (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_SRC_CLR_SHIFT                       (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_SRC_CLR_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_IPC_SRC_CLR_MAX                         (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_RESETVAL                                (0x00000000U)

/* IPC_CLR8 */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_CLR_MASK                            (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_CLR_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_CLR_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_CLR_MAX                             (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_SRC_CLR_MASK                        (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_SRC_CLR_SHIFT                       (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_SRC_CLR_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_IPC_SRC_CLR_MAX                         (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_RESETVAL                                (0x00000000U)

/* MAC_ID0 */

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_MASK                            (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_MACID_LO_MAX                             (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_RESETVAL                                 (0x00000000U)

/* MAC_ID1 */

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_MASK                            (0x0000FFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_SHIFT                           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_RESETVAL                        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_MACID_HI_MAX                             (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_RESETVAL                                 (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_LOCK0_KICK0_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_RESETVAL                             (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_LOCK0_KICK1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_RESETVAL                             (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROT_ERR_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_MASK                    (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_SHIFT                   (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_ADDR_ERR_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_MASK                    (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_SHIFT                   (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_KICK_ERR_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_MASK                   (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_SHIFT                  (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ERR_MAX                    (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_RESETVAL                         (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK  (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK  (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX  (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_RESETVAL               (0x00000000U)

/* INTR_ENABLE */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_MASK                     (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROT_ERR_EN_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_MASK                     (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_SHIFT                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_ADDR_ERR_EN_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_MASK                     (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_SHIFT                    (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_KICK_ERR_EN_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_MASK                    (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_SHIFT                   (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ERR_EN_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_RESETVAL                             (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK           (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX            (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK           (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT          (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX            (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK           (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT          (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX            (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK          (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT         (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX           (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_RESETVAL                       (0x00000000U)

/* EOI */

#define CSL_MCU_CTRL_MMR_CFG0_EOI_EOI_VECTOR_MASK                              (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_EOI_VECTOR_SHIFT                             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_EOI_VECTOR_RESETVAL                          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_EOI_VECTOR_MAX                               (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_EOI_RESETVAL                                     (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_MASK                    (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_FAULT_ADDR_MAX                     (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_RESETVAL                           (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                (0x0000003FU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL            (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                 (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_MASK                  (0x00000040U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                 (0x00000006U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_FAULT_NS_MAX                   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_RESETVAL                       (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK              (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX               (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK             (0x000FFF00U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT            (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL         (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX              (0x00000FFFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_MASK                 (0xFFF00000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                (0x00000014U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_FAULT_XID_MAX                  (0x00000FFFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_RESETVAL                       (0x00000000U)

/* FAULT_CLEAR */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_MASK                       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_SHIFT                      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_RESETVAL                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_FAULT_CLR_MAX                        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_RESETVAL                             (0x00000000U)

/* CLAIMREG_P0_R0_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_CLAIMREG_P0_R0_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P0_R1_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_CLAIMREG_P0_R1_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P0_R2_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_CLAIMREG_P0_R2_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P0_R3_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_CLAIMREG_P0_R3_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P0_R4_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_CLAIMREG_P0_R4_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_READONLY_RESETVAL                 (0x00000000U)

/* PID_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_MASK                   (0x0000003FU)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_RESETVAL               (0x00000012U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MINOR_PROXY_MAX                    (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_MASK                  (0x000000C0U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_SHIFT                 (0x00000006U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_CUSTOM_PROXY_MAX                   (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_MASK                   (0x00000700U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_SHIFT                  (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_RESETVAL               (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MAJOR_PROXY_MAX                    (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_MASK                    (0x0000F800U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_SHIFT                   (0x0000000BU)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MISC_PROXY_MAX                     (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_MASK                   (0xFFFF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_SHIFT                  (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_RESETVAL               (0x00006180U)
#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_PID_MSB16_PROXY_MAX                    (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_PID_PROXY_RESETVAL                               (0x61800212U)

/* MMR_CFG0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_MASK      (0x0000FFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_SHIFT     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_RESETVAL  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_SPEC_REV_PROXY_MAX       (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_MASK       (0xFFFF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_SHIFT      (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_RESETVAL   (0x00000018U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_MMR_CFG0_CFG_REV_PROXY_MAX        (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG0_PROXY_RESETVAL                          (0x00180000U)

/* MMR_CFG1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_MASK    (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_SHIFT   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_RESETVAL (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PARTITIONS_PROXY_MAX     (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_MASK      (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_SHIFT     (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_RESETVAL  (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_MMR_CFG1_PROXY_EN_PROXY_MAX       (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MMR_CFG1_PROXY_RESETVAL                          (0x8000001FU)

/* IPC_SET0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SET_PROXY_MASK       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SET_PROXY_SHIFT      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SET_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SET_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SRC_SET_PROXY_MASK   (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SRC_SET_PROXY_SHIFT  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SRC_SET_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_IPC_SET0_IPC_SRC_SET_PROXY_MAX    (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET0_PROXY_RESETVAL                          (0x00000000U)

/* IPC_SET1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SET_PROXY_MASK       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SET_PROXY_SHIFT      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SET_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SET_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SRC_SET_PROXY_MASK   (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SRC_SET_PROXY_SHIFT  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SRC_SET_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_IPC_SET1_IPC_SRC_SET_PROXY_MAX    (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET1_PROXY_RESETVAL                          (0x00000000U)

/* IPC_SET8_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SET_PROXY_MASK       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SET_PROXY_SHIFT      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SET_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SET_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SRC_SET_PROXY_MASK   (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SRC_SET_PROXY_SHIFT  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SRC_SET_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_IPC_SET8_IPC_SRC_SET_PROXY_MAX    (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_SET8_PROXY_RESETVAL                          (0x00000000U)

/* IPC_CLR0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_CLR_PROXY_MASK       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_CLR_PROXY_SHIFT      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_CLR_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_CLR_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_SRC_CLR_PROXY_MASK   (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_SRC_CLR_PROXY_SHIFT  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_SRC_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_IPC_CLR0_IPC_SRC_CLR_PROXY_MAX    (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR0_PROXY_RESETVAL                          (0x00000000U)

/* IPC_CLR1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_CLR_PROXY_MASK       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_CLR_PROXY_SHIFT      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_CLR_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_CLR_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_SRC_CLR_PROXY_MASK   (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_SRC_CLR_PROXY_SHIFT  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_SRC_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_IPC_CLR1_IPC_SRC_CLR_PROXY_MAX    (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR1_PROXY_RESETVAL                          (0x00000000U)

/* IPC_CLR8_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_CLR_PROXY_MASK       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_CLR_PROXY_SHIFT      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_CLR_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_CLR_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_SRC_CLR_PROXY_MASK   (0xFFFFFFF0U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_SRC_CLR_PROXY_SHIFT  (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_SRC_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_IPC_CLR8_IPC_SRC_CLR_PROXY_MAX    (0x0FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_IPC_CLR8_PROXY_RESETVAL                          (0x00000000U)

/* MAC_ID0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_MASK        (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_SHIFT       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_PROXY_MAC_ID0_MACID_LO_PROXY_MAX         (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID0_PROXY_RESETVAL                           (0x00000000U)

/* MAC_ID1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_MASK        (0x0000FFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_SHIFT       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_PROXY_MAC_ID1_MACID_HI_PROXY_MAX         (0x0000FFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MAC_ID1_PROXY_RESETVAL                           (0x00000000U)

/* LOCK0_KICK0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_LOCK0_KICK0_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0_PROXY_RESETVAL                       (0x00000000U)

/* LOCK0_KICK1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_LOCK0_KICK1_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1_PROXY_RESETVAL                       (0x00000000U)

/* INTR_RAW_STATUS_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_MASK        (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_SHIFT       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROT_ERR_PROXY_MAX         (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_MASK        (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_SHIFT       (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_ADDR_ERR_PROXY_MAX         (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_MASK        (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_SHIFT       (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_KICK_ERR_PROXY_MAX         (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_MASK       (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_SHIFT      (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_RESETVAL   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_PROXY_ERR_PROXY_MAX        (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_RAW_STATUS_PROXY_RESETVAL                   (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROT_ERR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_MASK (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_SHIFT (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_ADDR_ERR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_MASK (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_SHIFT (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_KICK_ERR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_MASK (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_SHIFT (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_ENABLED_PROXY_ERR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLED_STATUS_CLEAR_PROXY_RESETVAL         (0x00000000U)

/* INTR_ENABLE_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_MASK         (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROT_ERR_EN_PROXY_MAX          (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_MASK         (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_SHIFT        (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_ADDR_ERR_EN_PROXY_MAX          (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_MASK         (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_SHIFT        (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_KICK_ERR_EN_PROXY_MAX          (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_MASK        (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_SHIFT       (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_PROXY_ERR_EN_PROXY_MAX         (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_PROXY_RESETVAL                       (0x00000000U)

/* INTR_ENABLE_CLEAR_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROT_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_MASK (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_SHIFT (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_ADDR_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_MASK (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_SHIFT (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_KICK_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_MASK (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_SHIFT (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_PROXY_ERR_EN_CLR_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_INTR_ENABLE_CLEAR_PROXY_RESETVAL                 (0x00000000U)

/* EOI_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_MASK                  (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_SHIFT                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_EOI_PROXY_EOI_VECTOR_PROXY_MAX                   (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_EOI_PROXY_RESETVAL                               (0x00000000U)

/* FAULT_ADDRESS_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_MASK        (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_SHIFT       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_RESETVAL    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_FAULT_ADDR_PROXY_MAX         (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ADDRESS_PROXY_RESETVAL                     (0x00000000U)

/* FAULT_TYPE_STATUS_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_MASK    (0x0000003FU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_SHIFT   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_TYPE_PROXY_MAX     (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_MASK      (0x00000040U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_SHIFT     (0x00000006U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_RESETVAL  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_FAULT_NS_PROXY_MAX       (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_TYPE_STATUS_PROXY_RESETVAL                 (0x00000000U)

/* FAULT_ATTR_STATUS_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_MASK  (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_PRIVID_PROXY_MAX   (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_MASK (0x000FFF00U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_ROUTEID_PROXY_MAX  (0x00000FFFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_MASK     (0xFFF00000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_SHIFT    (0x00000014U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_FAULT_XID_PROXY_MAX      (0x00000FFFU)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_ATTR_STATUS_PROXY_RESETVAL                 (0x00000000U)

/* FAULT_CLEAR_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_MASK           (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_SHIFT          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_RESETVAL       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_FAULT_CLR_PROXY_MAX            (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_FAULT_CLEAR_PROXY_RESETVAL                       (0x00000000U)

/* CLAIMREG_P0_R0 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_CLAIMREG_P0_R0_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R0_RESETVAL                          (0x00000000U)

/* CLAIMREG_P0_R1 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_CLAIMREG_P0_R1_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R1_RESETVAL                          (0x00000000U)

/* CLAIMREG_P0_R2 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_CLAIMREG_P0_R2_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R2_RESETVAL                          (0x00000000U)

/* CLAIMREG_P0_R3 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_CLAIMREG_P0_R3_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R3_RESETVAL                          (0x00000000U)

/* CLAIMREG_P0_R4 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_CLAIMREG_P0_R4_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P0_R4_RESETVAL                          (0x00000000U)

/* MSMC_CFG */

#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_DDR_32B_ADDR_EN_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_DDR_32B_ADDR_EN_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_DDR_32B_ADDR_EN_RESETVAL                (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_DDR_32B_ADDR_EN_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_RESETVAL                                (0x00000001U)

/* MCU_ENET_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_MODE_SEL_MASK                      (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_MODE_SEL_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_MODE_SEL_RESETVAL                  (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_MODE_SEL_MAX                       (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_RGMII_ID_MODE_MASK                 (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_RGMII_ID_MODE_SHIFT                (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_RGMII_ID_MODE_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_RGMII_ID_MODE_MAX                  (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_RESETVAL                           (0x00000001U)

/* MCU_SPI1_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_SPI1_LINKDIS_MASK                  (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_SPI1_LINKDIS_SHIFT                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_SPI1_LINKDIS_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_SPI1_LINKDIS_MAX                   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_RESETVAL                           (0x00000000U)

/* MCU_FSS_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SEG_MASK                    (0x0000003FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SEG_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SEG_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SEG_MAX                     (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SIZE_MASK                   (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SIZE_SHIFT                  (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SIZE_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S0_BOOT_SIZE_MAX                    (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SEG_MASK                    (0x003F0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SEG_SHIFT                   (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SEG_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SEG_MAX                     (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SIZE_MASK                   (0x01000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SIZE_SHIFT                  (0x00000018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SIZE_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_S1_BOOT_SIZE_MAX                    (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_RESETVAL                            (0x00000000U)

/* MCU_ADC0_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_TRIG_SEL_MASK                      (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_TRIG_SEL_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_TRIG_SEL_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_TRIG_SEL_MAX                       (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_RESETVAL                           (0x00000000U)

/* MCU_ADC1_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_TRIG_SEL_MASK                      (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_TRIG_SEL_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_TRIG_SEL_RESETVAL                  (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_TRIG_SEL_MAX                       (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_RESETVAL                           (0x00000001U)

/* MCU_TIMER0_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_CAP_SEL_MASK                     (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_CAP_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_CAP_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_CAP_SEL_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_RESETVAL                         (0x00000000U)

/* MCU_TIMER1_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CAP_SEL_MASK                     (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CAP_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CAP_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CAP_SEL_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CASCADE_EN_MASK                  (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CASCADE_EN_SHIFT                 (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CASCADE_EN_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_CASCADE_EN_MAX                   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_RESETVAL                         (0x00000000U)

/* MCU_TIMER2_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_CAP_SEL_MASK                     (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_CAP_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_CAP_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_CAP_SEL_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_RESETVAL                         (0x00000000U)

/* MCU_TIMER3_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CAP_SEL_MASK                     (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CAP_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CAP_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CAP_SEL_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CASCADE_EN_MASK                  (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CASCADE_EN_SHIFT                 (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CASCADE_EN_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_CASCADE_EN_MAX                   (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_RESETVAL                         (0x00000000U)

/* MCU_TIMERIO0_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_OUT_SEL_MASK                   (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_OUT_SEL_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_OUT_SEL_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_OUT_SEL_MAX                    (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_RESETVAL                       (0x00000000U)

/* MCU_TIMERIO1_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_OUT_SEL_MASK                   (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_OUT_SEL_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_OUT_SEL_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_OUT_SEL_MAX                    (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_RESETVAL                       (0x00000000U)

/* LOCK1_KICK0 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_LOCK1_KICK0_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_RESETVAL                             (0x00000000U)

/* LOCK1_KICK1 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_LOCK1_KICK1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_RESETVAL                             (0x00000000U)

/* CLAIMREG_P1_R0_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_CLAIMREG_P1_R0_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P1_R1_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_CLAIMREG_P1_R1_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P1_R2_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_CLAIMREG_P1_R2_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P1_R3_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_CLAIMREG_P1_R3_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P1_R4_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_CLAIMREG_P1_R4_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P1_R5_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_CLAIMREG_P1_R5_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_READONLY_RESETVAL                 (0x00000000U)

/* MSMC_CFG_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_PROXY_MSMC_CFG_DDR_32B_ADDR_EN_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_PROXY_MSMC_CFG_DDR_32B_ADDR_EN_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_PROXY_MSMC_CFG_DDR_32B_ADDR_EN_PROXY_RESETVAL (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_PROXY_MSMC_CFG_DDR_32B_ADDR_EN_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MSMC_CFG_PROXY_RESETVAL                          (0x00000001U)

/* MCU_ENET_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_MODE_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_MODE_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_MODE_SEL_PROXY_RESETVAL (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_MODE_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_RGMII_ID_MODE_PROXY_MASK (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_RGMII_ID_MODE_PROXY_SHIFT (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_RGMII_ID_MODE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_MCU_ENET_CTRL_RGMII_ID_MODE_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CTRL_PROXY_RESETVAL                     (0x00000001U)

/* MCU_SPI1_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_PROXY_MCU_SPI1_CTRL_SPI1_LINKDIS_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_PROXY_MCU_SPI1_CTRL_SPI1_LINKDIS_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_PROXY_MCU_SPI1_CTRL_SPI1_LINKDIS_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_PROXY_MCU_SPI1_CTRL_SPI1_LINKDIS_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SPI1_CTRL_PROXY_RESETVAL                     (0x00000000U)

/* MCU_FSS_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SEG_PROXY_MASK (0x0000003FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SEG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SEG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SEG_PROXY_MAX (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SIZE_PROXY_MASK (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SIZE_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SIZE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S0_BOOT_SIZE_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SEG_PROXY_MASK (0x003F0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SEG_PROXY_SHIFT (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SEG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SEG_PROXY_MAX (0x0000003FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SIZE_PROXY_MASK (0x01000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SIZE_PROXY_SHIFT (0x00000018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SIZE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_MCU_FSS_CTRL_S1_BOOT_SIZE_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_FSS_CTRL_PROXY_RESETVAL                      (0x00000000U)

/* MCU_ADC0_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_PROXY_MCU_ADC0_CTRL_TRIG_SEL_PROXY_MASK (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_PROXY_MCU_ADC0_CTRL_TRIG_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_PROXY_MCU_ADC0_CTRL_TRIG_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_PROXY_MCU_ADC0_CTRL_TRIG_SEL_PROXY_MAX (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CTRL_PROXY_RESETVAL                     (0x00000000U)

/* MCU_ADC1_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_PROXY_MCU_ADC1_CTRL_TRIG_SEL_PROXY_MASK (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_PROXY_MCU_ADC1_CTRL_TRIG_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_PROXY_MCU_ADC1_CTRL_TRIG_SEL_PROXY_RESETVAL (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_PROXY_MCU_ADC1_CTRL_TRIG_SEL_PROXY_MAX (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CTRL_PROXY_RESETVAL                     (0x00000001U)

/* MCU_TIMER0_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_PROXY_MCU_TIMER0_CTRL_CAP_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_PROXY_MCU_TIMER0_CTRL_CAP_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_PROXY_MCU_TIMER0_CTRL_CAP_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_PROXY_MCU_TIMER0_CTRL_CAP_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CTRL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_TIMER1_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CAP_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CAP_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CAP_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CAP_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CASCADE_EN_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_MCU_TIMER1_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CTRL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_TIMER2_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_PROXY_MCU_TIMER2_CTRL_CAP_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_PROXY_MCU_TIMER2_CTRL_CAP_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_PROXY_MCU_TIMER2_CTRL_CAP_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_PROXY_MCU_TIMER2_CTRL_CAP_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CTRL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_TIMER3_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CAP_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CAP_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CAP_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CAP_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CASCADE_EN_PROXY_MASK (0x00000100U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CASCADE_EN_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CASCADE_EN_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_MCU_TIMER3_CTRL_CASCADE_EN_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CTRL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_TIMERIO0_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_PROXY_MCU_TIMERIO0_CTRL_OUT_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_PROXY_MCU_TIMERIO0_CTRL_OUT_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_PROXY_MCU_TIMERIO0_CTRL_OUT_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_PROXY_MCU_TIMERIO0_CTRL_OUT_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO0_CTRL_PROXY_RESETVAL                 (0x00000000U)

/* MCU_TIMERIO1_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_PROXY_MCU_TIMERIO1_CTRL_OUT_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_PROXY_MCU_TIMERIO1_CTRL_OUT_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_PROXY_MCU_TIMERIO1_CTRL_OUT_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_PROXY_MCU_TIMERIO1_CTRL_OUT_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMERIO1_CTRL_PROXY_RESETVAL                 (0x00000000U)

/* LOCK1_KICK0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_LOCK1_KICK0_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0_PROXY_RESETVAL                       (0x00000000U)

/* LOCK1_KICK1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_LOCK1_KICK1_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1_PROXY_RESETVAL                       (0x00000000U)

/* CLAIMREG_P1_R0 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_CLAIMREG_P1_R0_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R0_RESETVAL                          (0x00000000U)

/* CLAIMREG_P1_R1 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_CLAIMREG_P1_R1_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R1_RESETVAL                          (0x00000000U)

/* CLAIMREG_P1_R2 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_CLAIMREG_P1_R2_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R2_RESETVAL                          (0x00000000U)

/* CLAIMREG_P1_R3 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_CLAIMREG_P1_R3_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R3_RESETVAL                          (0x00000000U)

/* CLAIMREG_P1_R4 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_CLAIMREG_P1_R4_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R4_RESETVAL                          (0x00000000U)

/* CLAIMREG_P1_R5 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_CLAIMREG_P1_R5_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P1_R5_RESETVAL                          (0x00000000U)

/* MCU_CLKOUT0_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_SEL_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_SEL_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_EN_MASK                     (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_EN_SHIFT                    (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_CLK_EN_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_RESETVAL                        (0x00000000U)

/* MCU_EFUSE_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_CLK_SEL_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_CLK_SEL_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_RESETVAL                        (0x00000000U)

/* MCU_MCAN0_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_CLK_SEL_MASK                    (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_CLK_SEL_MAX                     (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_RESETVAL                        (0x00000000U)

/* MCU_MCAN1_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_CLK_SEL_MASK                    (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_CLK_SEL_MAX                     (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_RESETVAL                        (0x00000000U)

/* MCU_OSPI0_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_CLK_SEL_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_CLK_SEL_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_MASK                (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_SHIFT               (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_RESETVAL            (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_MAX                 (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_RESETVAL                        (0x00000000U)

/* MCU_OSPI1_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_CLK_SEL_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_CLK_SEL_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_MASK                (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_SHIFT               (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_RESETVAL            (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_MAX                 (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_RESETVAL                        (0x00000000U)

/* MCU_ADC0_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_CLK_SEL_MASK                     (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_CLK_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_CLK_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_CLK_SEL_MAX                      (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_RESETVAL                         (0x00000000U)

/* MCU_ADC1_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_CLK_SEL_MASK                     (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_CLK_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_CLK_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_CLK_SEL_MAX                      (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_RESETVAL                         (0x00000000U)

/* MCU_ENET_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_RMII_CLK_SEL_MASK                (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_RMII_CLK_SEL_SHIFT               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_RMII_CLK_SEL_RESETVAL            (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_RMII_CLK_SEL_MAX                 (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_RESETVAL                         (0x00000000U)

/* MCU_R5_CORE0_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_CLK_SEL_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_RESETVAL                     (0x00000000U)

/* MCU_R5_CORE1_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_CLK_SEL_MASK                 (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_CLK_SEL_SHIFT                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_CLK_SEL_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_CLK_SEL_MAX                  (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_RESETVAL                     (0x00000000U)

/* MCU_TIMER0_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_CLK_SEL_MASK                   (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_CLK_SEL_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_CLK_SEL_MAX                    (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_RESETVAL                       (0x00000000U)

/* MCU_TIMER1_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_CLK_SEL_MASK                   (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_CLK_SEL_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_CLK_SEL_MAX                    (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_RESETVAL                       (0x00000000U)

/* MCU_TIMER2_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_CLK_SEL_MASK                   (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_CLK_SEL_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_CLK_SEL_MAX                    (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_RESETVAL                       (0x00000000U)

/* MCU_TIMER3_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_CLK_SEL_MASK                   (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_CLK_SEL_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_CLK_SEL_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_CLK_SEL_MAX                    (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_RESETVAL                       (0x00000000U)

/* MCU_RTI0_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_MASK                     (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_MAX                      (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_WRTLOCK_MASK                     (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_WRTLOCK_SHIFT                    (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_WRTLOCK_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_WRTLOCK_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_RESETVAL                         (0x00000000U)

/* MCU_RTI1_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_MASK                     (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_MAX                      (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_WRTLOCK_MASK                     (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_WRTLOCK_SHIFT                    (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_WRTLOCK_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_WRTLOCK_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_RESETVAL                         (0x00000000U)

/* MCU_USART_CLKSEL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_CLK_SEL_MASK                    (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_CLK_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_CLK_SEL_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_CLK_SEL_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_RESETVAL                        (0x00000000U)

/* LOCK2_KICK0 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_LOCK2_KICK0_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_RESETVAL                             (0x00000000U)

/* LOCK2_KICK1 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_LOCK2_KICK1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_RESETVAL                             (0x00000000U)

/* CLAIMREG_P2_R0_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_CLAIMREG_P2_R0_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P2_R1_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_CLAIMREG_P2_R1_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P2_R2_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_CLAIMREG_P2_R2_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P2_R3_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_CLAIMREG_P2_R3_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_READONLY_RESETVAL                 (0x00000000U)

/* MCU_CLKOUT0_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_EN_PROXY_MASK (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_EN_PROXY_SHIFT (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_EN_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_MCU_CLKOUT0_CTRL_CLK_EN_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_CLKOUT0_CTRL_PROXY_RESETVAL                  (0x00000000U)

/* MCU_EFUSE_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_PROXY_MCU_EFUSE_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_PROXY_MCU_EFUSE_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_PROXY_MCU_EFUSE_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_PROXY_MCU_EFUSE_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_EFUSE_CLKSEL_PROXY_RESETVAL                  (0x00000000U)

/* MCU_MCAN0_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_PROXY_MCU_MCAN0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_PROXY_MCU_MCAN0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_PROXY_MCU_MCAN0_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_PROXY_MCU_MCAN0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN0_CLKSEL_PROXY_RESETVAL                  (0x00000000U)

/* MCU_MCAN1_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_PROXY_MCU_MCAN1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_PROXY_MCU_MCAN1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_PROXY_MCU_MCAN1_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_PROXY_MCU_MCAN1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_MCAN1_CLKSEL_PROXY_RESETVAL                  (0x00000000U)

/* MCU_OSPI0_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_MASK (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_SHIFT (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_MCU_OSPI0_CLKSEL_LOOPCLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL_PROXY_RESETVAL                  (0x00000000U)

/* MCU_OSPI1_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_PROXY_MASK (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_PROXY_SHIFT (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_MCU_OSPI1_CLKSEL_LOOPCLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI1_CLKSEL_PROXY_RESETVAL                  (0x00000000U)

/* MCU_ADC0_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_PROXY_MCU_ADC0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_PROXY_MCU_ADC0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_PROXY_MCU_ADC0_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_PROXY_MCU_ADC0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC0_CLKSEL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_ADC1_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_PROXY_MCU_ADC1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000003U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_PROXY_MCU_ADC1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_PROXY_MCU_ADC1_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_PROXY_MCU_ADC1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ADC1_CLKSEL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_ENET_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_PROXY_MCU_ENET_CLKSEL_RMII_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_PROXY_MCU_ENET_CLKSEL_RMII_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_PROXY_MCU_ENET_CLKSEL_RMII_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_PROXY_MCU_ENET_CLKSEL_RMII_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_ENET_CLKSEL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_R5_CORE0_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_PROXY_MCU_R5_CORE0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_PROXY_MCU_R5_CORE0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_PROXY_MCU_R5_CORE0_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_PROXY_MCU_R5_CORE0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE0_CLKSEL_PROXY_RESETVAL               (0x00000000U)

/* MCU_R5_CORE1_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_PROXY_MCU_R5_CORE1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_PROXY_MCU_R5_CORE1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_PROXY_MCU_R5_CORE1_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_PROXY_MCU_R5_CORE1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_R5_CORE1_CLKSEL_PROXY_RESETVAL               (0x00000000U)

/* MCU_TIMER0_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_PROXY_MCU_TIMER0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_PROXY_MCU_TIMER0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_PROXY_MCU_TIMER0_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_PROXY_MCU_TIMER0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER0_CLKSEL_PROXY_RESETVAL                 (0x00000000U)

/* MCU_TIMER1_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_PROXY_MCU_TIMER1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_PROXY_MCU_TIMER1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_PROXY_MCU_TIMER1_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_PROXY_MCU_TIMER1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER1_CLKSEL_PROXY_RESETVAL                 (0x00000000U)

/* MCU_TIMER2_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_PROXY_MCU_TIMER2_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_PROXY_MCU_TIMER2_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_PROXY_MCU_TIMER2_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_PROXY_MCU_TIMER2_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER2_CLKSEL_PROXY_RESETVAL                 (0x00000000U)

/* MCU_TIMER3_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_PROXY_MCU_TIMER3_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_PROXY_MCU_TIMER3_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_PROXY_MCU_TIMER3_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_PROXY_MCU_TIMER3_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_TIMER3_CLKSEL_PROXY_RESETVAL                 (0x00000000U)

/* MCU_RTI0_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_WRTLOCK_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_MCU_RTI0_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_RTI1_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_CLK_SEL_PROXY_MASK (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_CLK_SEL_PROXY_MAX (0x00000007U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_WRTLOCK_PROXY_MASK (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_WRTLOCK_PROXY_SHIFT (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_WRTLOCK_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_MCU_RTI1_CLKSEL_WRTLOCK_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_PROXY_RESETVAL                   (0x00000000U)

/* MCU_USART_CLKSEL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_PROXY_MCU_USART_CLKSEL_CLK_SEL_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_PROXY_MCU_USART_CLKSEL_CLK_SEL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_PROXY_MCU_USART_CLKSEL_CLK_SEL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_PROXY_MCU_USART_CLKSEL_CLK_SEL_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_USART_CLKSEL_PROXY_RESETVAL                  (0x00000000U)

/* LOCK2_KICK0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_LOCK2_KICK0_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0_PROXY_RESETVAL                       (0x00000000U)

/* LOCK2_KICK1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_LOCK2_KICK1_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1_PROXY_RESETVAL                       (0x00000000U)

/* CLAIMREG_P2_R0 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_CLAIMREG_P2_R0_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R0_RESETVAL                          (0x00000000U)

/* CLAIMREG_P2_R1 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_CLAIMREG_P2_R1_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R1_RESETVAL                          (0x00000000U)

/* CLAIMREG_P2_R2 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_CLAIMREG_P2_R2_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R2_RESETVAL                          (0x00000000U)

/* CLAIMREG_P2_R3 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_CLAIMREG_P2_R3_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P2_R3_RESETVAL                          (0x00000000U)

/* MCU_LBIST_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DIVIDE_RATIO_MASK                 (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DIVIDE_RATIO_SHIFT                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DIVIDE_RATIO_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DIVIDE_RATIO_MAX                  (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_LOAD_DIV_MASK                     (0x00000080U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_LOAD_DIV_SHIFT                    (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_LOAD_DIV_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_LOAD_DIV_MAX                      (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DC_DEF_MASK                       (0x00000300U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DC_DEF_SHIFT                      (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DC_DEF_RESETVAL                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_DC_DEF_MAX                        (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_RUNBIST_MODE_MASK                 (0x0000F000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_RUNBIST_MODE_SHIFT                (0x0000000CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_RUNBIST_MODE_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_RUNBIST_MODE_MAX                  (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RUN_MASK                     (0x0F000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RUN_SHIFT                    (0x00000018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RUN_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RUN_MAX                      (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RESET_MASK                   (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RESET_SHIFT                  (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RESET_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_BIST_RESET_MAX                    (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_RESETVAL                          (0x00000000U)

/* MCU_LBIST_PATCOUNT */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_MASK              (0x0000000FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_SHIFT             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_RESETVAL          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_MAX               (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_RESET_PC_DEF_MASK             (0x000000F0U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_RESET_PC_DEF_SHIFT            (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_RESET_PC_DEF_RESETVAL         (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_RESET_PC_DEF_MAX              (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SET_PC_DEF_MASK               (0x00000F00U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SET_PC_DEF_SHIFT              (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SET_PC_DEF_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_SET_PC_DEF_MAX                (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_MASK            (0x3FFF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_SHIFT           (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_RESETVAL        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_MAX             (0x00003FFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_RESETVAL                      (0x00000000U)

/* MCU_LBIST_SEED0 */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PRPG_DEF_MASK                    (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PRPG_DEF_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PRPG_DEF_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PRPG_DEF_MAX                     (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_RESETVAL                         (0x00000000U)

/* MCU_LBIST_SEED1 */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PRPG_DEF_MASK                    (0x001FFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PRPG_DEF_SHIFT                   (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PRPG_DEF_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PRPG_DEF_MAX                     (0x001FFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_RESETVAL                         (0x00000000U)

/* MCU_LBIST_SPARE0 */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_MASK          (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_SHIFT         (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_RESETVAL      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_MAX           (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_MASK          (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_SHIFT         (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_RESETVAL      (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_MAX           (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_SPARE0_MASK                     (0xFFFFFFFCU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_SPARE0_SHIFT                    (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_SPARE0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_SPARE0_MAX                      (0x3FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_RESETVAL                        (0x00000000U)

/* MCU_LBIST_SPARE1 */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_SPARE1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_SPARE1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_SPARE1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_SPARE1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_RESETVAL                        (0x00000000U)

/* MCU_LBIST_STAT */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_MISR_MUX_CTL_MASK                 (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_MISR_MUX_CTL_SHIFT                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_MISR_MUX_CTL_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_MISR_MUX_CTL_MAX                  (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_OUT_MUX_CTL_MASK                  (0x00000300U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_OUT_MUX_CTL_SHIFT                 (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_OUT_MUX_CTL_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_OUT_MUX_CTL_MAX                   (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_RUNNING_MASK                 (0x00008000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_RUNNING_SHIFT                (0x0000000FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_RUNNING_RESETVAL             (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_RUNNING_MAX                  (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_DONE_MASK                    (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_DONE_SHIFT                   (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_DONE_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_BIST_DONE_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_RESETVAL                          (0x00000000U)

/* MCU_LBIST_MISR */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_MISR_RESULT_MASK                  (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_MISR_RESULT_SHIFT                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_MISR_RESULT_RESETVAL              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_MISR_RESULT_MAX                   (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_RESETVAL                          (0x00000000U)

/* MCU_LBIST_SIG */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_MISR_SIG_MASK                      (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_MISR_SIG_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_MISR_SIG_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_MISR_SIG_MAX                       (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_RESETVAL                           (0x00000000U)

/* LOCK3_KICK0 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_LOCK3_KICK0_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_RESETVAL                             (0x00000000U)

/* LOCK3_KICK1 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_LOCK3_KICK1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_RESETVAL                             (0x00000000U)

/* CLAIMREG_P3_R0_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_CLAIMREG_P3_R0_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P3_R1_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_CLAIMREG_P3_R1_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P3_R2_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_CLAIMREG_P3_R2_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P3_R3_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_CLAIMREG_P3_R3_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P3_R4_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_CLAIMREG_P3_R4_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P3_R5_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_CLAIMREG_P3_R5_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_READONLY_RESETVAL                 (0x00000000U)

/* MCU_LBIST_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DIVIDE_RATIO_PROXY_MASK (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DIVIDE_RATIO_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DIVIDE_RATIO_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DIVIDE_RATIO_PROXY_MAX (0x0000001FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_LOAD_DIV_PROXY_MASK (0x00000080U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_LOAD_DIV_PROXY_SHIFT (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_LOAD_DIV_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_LOAD_DIV_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DC_DEF_PROXY_MASK (0x00000300U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DC_DEF_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DC_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_DC_DEF_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_RUNBIST_MODE_PROXY_MASK (0x0000F000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_RUNBIST_MODE_PROXY_SHIFT (0x0000000CU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_RUNBIST_MODE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_RUNBIST_MODE_PROXY_MAX (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RUN_PROXY_MASK (0x0F000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RUN_PROXY_SHIFT (0x00000018U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RUN_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RUN_PROXY_MAX (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RESET_PROXY_MASK (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RESET_PROXY_SHIFT (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RESET_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_MCU_LBIST_CTRL_BIST_RESET_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_CTRL_PROXY_RESETVAL                    (0x00000000U)

/* MCU_LBIST_PATCOUNT_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_PROXY_MASK (0x0000000FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SCAN_PC_DEF_PROXY_MAX (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_RESET_PC_DEF_PROXY_MASK (0x000000F0U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_RESET_PC_DEF_PROXY_SHIFT (0x00000004U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_RESET_PC_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_RESET_PC_DEF_PROXY_MAX (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SET_PC_DEF_PROXY_MASK (0x00000F00U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SET_PC_DEF_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SET_PC_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_SET_PC_DEF_PROXY_MAX (0x0000000FU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_PROXY_MASK (0x3FFF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_PROXY_SHIFT (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_MCU_LBIST_PATCOUNT_STATIC_PC_DEF_PROXY_MAX (0x00003FFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_PATCOUNT_PROXY_RESETVAL                (0x00000000U)

/* MCU_LBIST_SEED0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PROXY_MCU_LBIST_SEED0_PRPG_DEF_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PROXY_MCU_LBIST_SEED0_PRPG_DEF_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PROXY_MCU_LBIST_SEED0_PRPG_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PROXY_MCU_LBIST_SEED0_PRPG_DEF_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED0_PROXY_RESETVAL                   (0x00000000U)

/* MCU_LBIST_SEED1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PROXY_MCU_LBIST_SEED1_PRPG_DEF_PROXY_MASK (0x001FFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PROXY_MCU_LBIST_SEED1_PRPG_DEF_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PROXY_MCU_LBIST_SEED1_PRPG_DEF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PROXY_MCU_LBIST_SEED1_PRPG_DEF_PROXY_MAX (0x001FFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SEED1_PROXY_RESETVAL                   (0x00000000U)

/* MCU_LBIST_SPARE0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_LBIST_SELFTEST_EN_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_PROXY_MASK (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_PROXY_SHIFT (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_PBIST_SELFTEST_EN_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_SPARE0_PROXY_MASK (0xFFFFFFFCU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_SPARE0_PROXY_SHIFT (0x00000002U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_SPARE0_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_MCU_LBIST_SPARE0_SPARE0_PROXY_MAX (0x3FFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE0_PROXY_RESETVAL                  (0x00000000U)

/* MCU_LBIST_SPARE1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_PROXY_MCU_LBIST_SPARE1_SPARE1_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_PROXY_MCU_LBIST_SPARE1_SPARE1_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_PROXY_MCU_LBIST_SPARE1_SPARE1_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_PROXY_MCU_LBIST_SPARE1_SPARE1_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SPARE1_PROXY_RESETVAL                  (0x00000000U)

/* MCU_LBIST_STAT_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_MISR_MUX_CTL_PROXY_MASK (0x000000FFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_MISR_MUX_CTL_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_MISR_MUX_CTL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_MISR_MUX_CTL_PROXY_MAX (0x000000FFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_OUT_MUX_CTL_PROXY_MASK (0x00000300U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_OUT_MUX_CTL_PROXY_SHIFT (0x00000008U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_OUT_MUX_CTL_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_OUT_MUX_CTL_PROXY_MAX (0x00000003U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_RUNNING_PROXY_MASK (0x00008000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_RUNNING_PROXY_SHIFT (0x0000000FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_RUNNING_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_RUNNING_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_DONE_PROXY_MASK (0x80000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_DONE_PROXY_SHIFT (0x0000001FU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_DONE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_MCU_LBIST_STAT_BIST_DONE_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_STAT_PROXY_RESETVAL                    (0x00000000U)

/* MCU_LBIST_MISR_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_PROXY_MCU_LBIST_MISR_MISR_RESULT_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_PROXY_MCU_LBIST_MISR_MISR_RESULT_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_PROXY_MCU_LBIST_MISR_MISR_RESULT_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_PROXY_MCU_LBIST_MISR_MISR_RESULT_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_MISR_PROXY_RESETVAL                    (0x00000000U)

/* MCU_LBIST_SIG_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_PROXY_MCU_LBIST_SIG_MISR_SIG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_PROXY_MCU_LBIST_SIG_MISR_SIG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_PROXY_MCU_LBIST_SIG_MISR_SIG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_PROXY_MCU_LBIST_SIG_MISR_SIG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_LBIST_SIG_PROXY_RESETVAL                     (0x00000000U)

/* LOCK3_KICK0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_LOCK3_KICK0_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0_PROXY_RESETVAL                       (0x00000000U)

/* LOCK3_KICK1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_LOCK3_KICK1_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1_PROXY_RESETVAL                       (0x00000000U)

/* CLAIMREG_P3_R0 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_CLAIMREG_P3_R0_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R0_RESETVAL                          (0x00000000U)

/* CLAIMREG_P3_R1 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_CLAIMREG_P3_R1_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R1_RESETVAL                          (0x00000000U)

/* CLAIMREG_P3_R2 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_CLAIMREG_P3_R2_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R2_RESETVAL                          (0x00000000U)

/* CLAIMREG_P3_R3 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_CLAIMREG_P3_R3_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R3_RESETVAL                          (0x00000000U)

/* CLAIMREG_P3_R4 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_CLAIMREG_P3_R4_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R4_RESETVAL                          (0x00000000U)

/* CLAIMREG_P3_R5 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_CLAIMREG_P3_R5_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P3_R5_RESETVAL                          (0x00000000U)

/* CORE_PASS_DONE_SET0 */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_CORE_PASSDONE_MASK           (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_CORE_PASSDONE_SHIFT          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_CORE_PASSDONE_RESETVAL       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_CORE_PASSDONE_MAX            (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_RESETVAL                     (0x00000000U)

/* CORE_PASS_DONE_CLR0 */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_CORE_PASSDONE_MASK           (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_CORE_PASSDONE_SHIFT          (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_CORE_PASSDONE_RESETVAL       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_CORE_PASSDONE_MAX            (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_RESETVAL                     (0x00000000U)

/* ACTIVE_CORES */

#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_ACTIVE_CORES_MASK                   (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_ACTIVE_CORES_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_ACTIVE_CORES_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_ACTIVE_CORES_MAX                    (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_RESETVAL                            (0x00000000U)

/* CORE_DEBUG0 */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_CORE_DEBUG_MASK                      (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_CORE_DEBUG_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_CORE_DEBUG_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_CORE_DEBUG_MAX                       (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_RESETVAL                             (0x00000000U)

/* CORE_DEBUG1 */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_CORE_DEBUG_MASK                      (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_CORE_DEBUG_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_CORE_DEBUG_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_CORE_DEBUG_MAX                       (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_RESETVAL                             (0x00000000U)

/* CORE_DEBUG2 */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_CORE_DEBUG_MASK                      (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_CORE_DEBUG_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_CORE_DEBUG_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_CORE_DEBUG_MAX                       (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_RESETVAL                             (0x00000000U)

/* CORE_DEBUG3 */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_CORE_DEBUG_MASK                      (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_CORE_DEBUG_SHIFT                     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_CORE_DEBUG_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_CORE_DEBUG_MAX                       (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_RESETVAL                             (0x00000000U)

/* DBG_FEATURE0 */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_FEATURE_MASK                        (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_FEATURE_SHIFT                       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_FEATURE_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_FEATURE_MAX                         (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_RESETVAL                            (0x00000000U)

/* DBG_FEATURE1 */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_FEATURE_MASK                        (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_FEATURE_SHIFT                       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_FEATURE_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_FEATURE_MAX                         (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_RESETVAL                            (0x00000000U)

/* DBG_FEATURE2 */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_FEATURE_MASK                        (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_FEATURE_SHIFT                       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_FEATURE_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_FEATURE_MAX                         (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_RESETVAL                            (0x00000000U)

/* DBG_FEATURE3 */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_FEATURE_MASK                        (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_FEATURE_SHIFT                       (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_FEATURE_RESETVAL                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_FEATURE_MAX                         (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_RESETVAL                            (0x00000000U)

/* TEST_DEBUG_PORRST0 */

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_TEST_DEBUG_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_TEST_DEBUG_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_TEST_DEBUG_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_TEST_DEBUG_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_RESETVAL                      (0x00000000U)

/* TEST_DEBUG_ALLRST0 */

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_TEST_DEBUG_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_TEST_DEBUG_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_TEST_DEBUG_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_TEST_DEBUG_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_RESETVAL                      (0x00000000U)

/* LOCK4_KICK0 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_LOCK4_KICK0_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_RESETVAL                             (0x00000000U)

/* LOCK4_KICK1 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_LOCK4_KICK1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_RESETVAL                             (0x00000000U)

/* CLAIMREG_P4_R0_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_CLAIMREG_P4_R0_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_READONLY_RESETVAL                 (0x00000000U)

/* CLAIMREG_P4_R1_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_CLAIMREG_P4_R1_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_READONLY_RESETVAL                 (0x00000000U)

/* CORE_PASS_DONE_SET0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_PROXY_CORE_PASS_DONE_SET0_CORE_PASSDONE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_PROXY_CORE_PASS_DONE_SET0_CORE_PASSDONE_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_PROXY_CORE_PASS_DONE_SET0_CORE_PASSDONE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_PROXY_CORE_PASS_DONE_SET0_CORE_PASSDONE_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_SET0_PROXY_RESETVAL               (0x00000000U)

/* CORE_PASS_DONE_CLR0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_PROXY_CORE_PASS_DONE_CLR0_CORE_PASSDONE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_PROXY_CORE_PASS_DONE_CLR0_CORE_PASSDONE_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_PROXY_CORE_PASS_DONE_CLR0_CORE_PASSDONE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_PROXY_CORE_PASS_DONE_CLR0_CORE_PASSDONE_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_PASS_DONE_CLR0_PROXY_RESETVAL               (0x00000000U)

/* ACTIVE_CORES_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_PROXY_ACTIVE_CORES_ACTIVE_CORES_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_PROXY_ACTIVE_CORES_ACTIVE_CORES_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_PROXY_ACTIVE_CORES_ACTIVE_CORES_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_PROXY_ACTIVE_CORES_ACTIVE_CORES_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_ACTIVE_CORES_PROXY_RESETVAL                      (0x00000000U)

/* CORE_DEBUG0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_PROXY_CORE_DEBUG0_CORE_DEBUG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_PROXY_CORE_DEBUG0_CORE_DEBUG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_PROXY_CORE_DEBUG0_CORE_DEBUG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_PROXY_CORE_DEBUG0_CORE_DEBUG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG0_PROXY_RESETVAL                       (0x00000000U)

/* CORE_DEBUG1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_PROXY_CORE_DEBUG1_CORE_DEBUG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_PROXY_CORE_DEBUG1_CORE_DEBUG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_PROXY_CORE_DEBUG1_CORE_DEBUG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_PROXY_CORE_DEBUG1_CORE_DEBUG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG1_PROXY_RESETVAL                       (0x00000000U)

/* CORE_DEBUG2_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_PROXY_CORE_DEBUG2_CORE_DEBUG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_PROXY_CORE_DEBUG2_CORE_DEBUG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_PROXY_CORE_DEBUG2_CORE_DEBUG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_PROXY_CORE_DEBUG2_CORE_DEBUG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG2_PROXY_RESETVAL                       (0x00000000U)

/* CORE_DEBUG3_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_PROXY_CORE_DEBUG3_CORE_DEBUG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_PROXY_CORE_DEBUG3_CORE_DEBUG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_PROXY_CORE_DEBUG3_CORE_DEBUG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_PROXY_CORE_DEBUG3_CORE_DEBUG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CORE_DEBUG3_PROXY_RESETVAL                       (0x00000000U)

/* DBG_FEATURE0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_PROXY_DBG_FEATURE0_FEATURE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_PROXY_DBG_FEATURE0_FEATURE_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_PROXY_DBG_FEATURE0_FEATURE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_PROXY_DBG_FEATURE0_FEATURE_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE0_PROXY_RESETVAL                      (0x00000000U)

/* DBG_FEATURE1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_PROXY_DBG_FEATURE1_FEATURE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_PROXY_DBG_FEATURE1_FEATURE_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_PROXY_DBG_FEATURE1_FEATURE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_PROXY_DBG_FEATURE1_FEATURE_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE1_PROXY_RESETVAL                      (0x00000000U)

/* DBG_FEATURE2_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_PROXY_DBG_FEATURE2_FEATURE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_PROXY_DBG_FEATURE2_FEATURE_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_PROXY_DBG_FEATURE2_FEATURE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_PROXY_DBG_FEATURE2_FEATURE_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE2_PROXY_RESETVAL                      (0x00000000U)

/* DBG_FEATURE3_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_PROXY_DBG_FEATURE3_FEATURE_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_PROXY_DBG_FEATURE3_FEATURE_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_PROXY_DBG_FEATURE3_FEATURE_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_PROXY_DBG_FEATURE3_FEATURE_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_DBG_FEATURE3_PROXY_RESETVAL                      (0x00000000U)

/* TEST_DEBUG_PORRST0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_PROXY_TEST_DEBUG_PORRST0_TEST_DEBUG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_PROXY_TEST_DEBUG_PORRST0_TEST_DEBUG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_PROXY_TEST_DEBUG_PORRST0_TEST_DEBUG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_PROXY_TEST_DEBUG_PORRST0_TEST_DEBUG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_PORRST0_PROXY_RESETVAL                (0x00000000U)

/* TEST_DEBUG_ALLRST0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_PROXY_TEST_DEBUG_ALLRST0_TEST_DEBUG_PROXY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_PROXY_TEST_DEBUG_ALLRST0_TEST_DEBUG_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_PROXY_TEST_DEBUG_ALLRST0_TEST_DEBUG_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_PROXY_TEST_DEBUG_ALLRST0_TEST_DEBUG_PROXY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_TEST_DEBUG_ALLRST0_PROXY_RESETVAL                (0x00000000U)

/* LOCK4_KICK0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_LOCK4_KICK0_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0_PROXY_RESETVAL                       (0x00000000U)

/* LOCK4_KICK1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_LOCK4_KICK1_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1_PROXY_RESETVAL                       (0x00000000U)

/* CLAIMREG_P4_R0 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_CLAIMREG_P4_R0_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R0_RESETVAL                          (0x00000000U)

/* CLAIMREG_P4_R1 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_CLAIMREG_P4_R1_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P4_R1_RESETVAL                          (0x00000000U)

/* MCU_SRAM_LDO_CTRL */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ENFUNC1_MASK                   (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ENFUNC1_SHIFT                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ENFUNC1_RESETVAL               (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ENFUNC1_MAX                    (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ABBOFF_MASK                    (0x00000020U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ABBOFF_SHIFT                   (0x00000005U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ABBOFF_RESETVAL                (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_ABBOFF_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_SRAMALLRET_MASK                (0x00000040U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_SRAMALLRET_SHIFT               (0x00000006U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_SRAMALLRET_RESETVAL            (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_SRAMALLRET_MAX                 (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_AIPOFF_MASK                    (0x00000080U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_AIPOFF_SHIFT                   (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_AIPOFF_RESETVAL                (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_AIPOFF_MAX                     (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_VSET_MASK                      (0x03FF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_VSET_SHIFT                     (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_VSET_RESETVAL                  (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_VSET_MAX                       (0x000003FFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_RESETVAL                       (0x00000020U)

/* LOCK6_KICK0 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_LOCK6_KICK0_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_RESETVAL                             (0x00000000U)

/* LOCK6_KICK1 */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_MASK                     (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_SHIFT                    (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_RESETVAL                 (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_LOCK6_KICK1_MAX                      (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_RESETVAL                             (0x00000000U)

/* CLAIMREG_P6_R0_READONLY */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_MASK (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_CLAIMREG_P6_R0_READONLY_MAX (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_READONLY_RESETVAL                 (0x00000000U)

/* MCU_SRAM_LDO_CTRL_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ENFUNC1_PROXY_MASK (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ENFUNC1_PROXY_SHIFT (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ENFUNC1_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ENFUNC1_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ABBOFF_PROXY_MASK (0x00000020U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ABBOFF_PROXY_SHIFT (0x00000005U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ABBOFF_PROXY_RESETVAL (0x00000001U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_ABBOFF_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_SRAMALLRET_PROXY_MASK (0x00000040U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_SRAMALLRET_PROXY_SHIFT (0x00000006U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_SRAMALLRET_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_SRAMALLRET_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_AIPOFF_PROXY_MASK (0x00000080U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_AIPOFF_PROXY_SHIFT (0x00000007U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_AIPOFF_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_AIPOFF_PROXY_MAX (0x00000001U)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_VSET_PROXY_MASK (0x03FF0000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_VSET_PROXY_SHIFT (0x00000010U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_VSET_PROXY_RESETVAL (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_MCU_SRAM_LDO_CTRL_VSET_PROXY_MAX (0x000003FFU)

#define CSL_MCU_CTRL_MMR_CFG0_MCU_SRAM_LDO_CTRL_PROXY_RESETVAL                 (0x00000020U)

/* LOCK6_KICK0_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_LOCK6_KICK0_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK0_PROXY_RESETVAL                       (0x00000000U)

/* LOCK6_KICK1_PROXY */

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_MASK         (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_SHIFT        (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_RESETVAL     (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_LOCK6_KICK1_PROXY_MAX          (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_LOCK6_KICK1_PROXY_RESETVAL                       (0x00000000U)

/* CLAIMREG_P6_R0 */

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_MASK               (0xFFFFFFFFU)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_SHIFT              (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_RESETVAL           (0x00000000U)
#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_CLAIMREG_P6_R0_MAX                (0xFFFFFFFFU)

#define CSL_MCU_CTRL_MMR_CFG0_CLAIMREG_P6_R0_RESETVAL                          (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
