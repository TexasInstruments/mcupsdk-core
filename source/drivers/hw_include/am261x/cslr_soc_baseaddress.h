/********************************************************************
 * *
 * * SOC memory map header file
 * *
 * * Copyright (C) 2023 Texas Instruments Incorporated.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *    Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * *
 * *    Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the
 * *    distribution.
 * *
 * *    Neither the name of Texas Instruments Incorporated nor the names of
 * *    its contributors may be used to endorse or promote products derived
 * *    from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * *
 * */
#ifndef CSLR_SOC_BASEADDRESS_H
#define CSLR_SOC_BASEADDRESS_H

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus

extern "C"
{
#endif

/* Global addresses in unified address space */
 
#define CSL_R5SS0_CORE0_TCMA_ROM_U_BASE								(0x00000000ul)
#define CSL_HSM_M4_SEC_ROM_U_BASE	    							(0x00010000ul)
#define CSL_R5SS0_CORE0_TCMA_RAM_U_BASE								(0x00020000ul)
#define CSL_R5SS0_CORE0_TMU_U_BASE	    							(0x00060000ul)
#define CSL_R5SS0_CORE0_TCMB_RAM_U_BASE								(0x00080000ul)
#define CSL_HSM_ROM_U_BASE	            							(0x20000000ul)
#define CSL_HSM_SEC_ROM_U_BASE	        							(0x20010000ul)
#define CSL_HSM_RAM_U_BASE	            							(0x20020000ul)
#define CSL_MCRC0_U_BASE	            							(0x35000000ul)
#define CSL_STM_STIM_U_BASE	            							(0x39000000ul)
#define CSL_HSM_SOC_CTRL_U_BASE	        							(0x40000000ul)
#define CSL_MPU_L2OCRAM_BANK0_U_BASE								(0x40020000ul)
#define CSL_MPU_L2OCRAM_BANK1_U_BASE								(0x40040000ul)
#define CSL_MPU_L2OCRAM_BANK2_U_BASE								(0x40060000ul)
#define CSL_MPU_R5SS0_CORE0_AXIS_U_BASE								(0x400A0000ul)
#define CSL_MPU_R5SS0_CORE1_AXIS_U_BASE								(0x400C0000ul)
#define CSL_MPU_HSM_DTHE_U_BASE	        							(0x40120000ul)
#define CSL_MPU_MBOX_SRAM_U_BASE	    							(0x40140000ul)
#define CSL_MPU_OSPI0_U_BASE	        							(0x40160000ul)
#define CSL_MPU_SCRM2SCRP0_U_BASE	    							(0x40180000ul)
#define CSL_MPU_SCRM2SCRP1_U_BASE	    							(0x401A0000ul)
#define CSL_MPU_R5SS0_CORE0_AHB_U_BASE								(0x401C0000ul)
#define CSL_MPU_R5SS0_CORE1_AHB_U_BASE								(0x401E0000ul)
#define CSL_MPU_HSM_U_BASE	            							(0x40240000ul)
#define CSL_MPU_FLASH_U_BASE	        							(0x40260000ul)
#define CSL_MPU_R5SS0_U_BASE	        							(0x40280000ul)
#define CSL_HSM_SOC_PCR_U_BASE	        							(0x40F78000ul)
#define CSL_HSM_STC_U_BASE	            							(0x40F78C00ul)
#define CSL_HSM_PBIST_U_BASE	        							(0x40F79000ul)
#define CSL_HSM_ECC_AGGR_U_BASE	        							(0x40F79400ul)
#define CSL_HSM_MBOX_SRAM_U_BASE	    							(0x44000000ul)
#define CSL_HSM_SEC_MGR_U_BASE	        							(0x46000000ul)
#define CSL_HSM_SEC_RAM_U_BASE	        						    (0x46050000ul)
#define CSL_HSM_CTRL_U_BASE	            							(0x47000000ul)
#define CSL_HSM_TPCC0_U_BASE	        							(0x47020000ul)
#define CSL_HSM_TPTC00_U_BASE	        							(0x47040000ul)
#define CSL_HSM_TPTC01_U_BASE	        							(0x47060000ul)
#define CSL_HSM_PCR_U_BASE	            							(0x47F78000ul)
#define CSL_HSM_RTI0_U_BASE	            							(0x47F78C00ul)
#define CSL_HSM_WDT0_U_BASE	            							(0x47F78D00ul)
#define CSL_HSM_DCC0_U_BASE	            							(0x47F79000ul)
#define CSL_HSM_ESM_U_BASE	            							(0x47F79400ul)
#define CSL_HSM_DMT0_U_BASE	            							(0x47F79800ul)
#define CSL_HSM_DMT1_U_BASE	            							(0x47F79900ul)
#define CSL_ICSSM0_INTERNAL_U_BASE	            					(0x48000000ul)
#define CSL_ICSS_M_ICSSM_0_DRAM1_SLV_RAM_U_BASE	            		(0x48002000ul)
#define CSL_ICSS_M_ICSSM_0_RAT_SLICE0_CFG_U_BASE	            	(0x48008000ul)
#define CSL_ICSS_M_ICSSM_0_RAT_SLICE1_CFG_U_BASE	            	(0x48009000ul)
#define CSL_ICSS_M_ICSSM_0_RAM_SLV_RAM_U_BASE	            		(0x48010000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_ICSS_INTC_INTC_SLV_U_BASE	        (0x48020000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PDSP0_IRAM_U_BASE	            	(0x48022000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PDSP0_IRAM_DEBUG_U_BASE	            (0x48022400ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PDSP1_IRAM_U_BASE	            	(0x48024000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PDSP1_IRAM_DEBUG_U_BASE	            (0x48024400ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PROT_SLV_U_BASE	            		(0x48024C00ul)
#define CSL_ICSS_M_ICSSM_0_PR1_CFG_SLV_U_BASE	            		(0x48026000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_ICSS_UART_UART_SLV_U_BASE	        (0x48028000ul)
#define CSL_ICSS_M_ICSSM_0_IEP0_U_BASE	            				(0x4802E000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_ICSS_ECAP0_ECAP_SLV_U_BASE	        (0x48030000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_MII_RT_PR1_MII_RT_CFG_U_BASE	        (0x48032000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_MDIO_V1P7_MDIO_U_BASE	            (0x48032400ul)
#define CSL_ICSS_M_ICSSM_0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_U_BAS	            (0x48033000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PDSP0_IRAM_RAM_U_BASE	            (0x48034000ul)
#define CSL_ICSS_M_ICSSM_0_PR1_PDSP1_IRAM_RAM_U_BASE	            (0x48038000ul)
#define CSL_ICSSM0_ECC_U_BASE	            						(0x48100000ul)
#define CSL_MMC0_U_BASE	            								(0x48300000ul)
#define CSL_GPMC0_CFG	            								(0x48400000ul)
#define CSL_ICSSM1_INTERNAL	        								(0x48600000ul)
#define CSL_ICSS_M_ICSSM_1_DRAM1_SLV_RAM_U_BASE	            		(0x48602000ul)
#define CSL_ICSS_M_ICSSM_1_RAT_SLICE0_CFG_U_BASE	            	(0x48608000ul)
#define CSL_ICSS_M_ICSSM_1_RAT_SLICE1_CFG_U_BASE	            	(0x48609000ul)
#define CSL_ICSS_M_ICSSM_1_RAM_SLV_RAM_U_BASE	            		(0x48610000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_ICSS_INTC_INTC_SLV_U_BASE	        (0x48620000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PDSP0_IRAM_U_BASE	            	(0x48622000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PDSP0_IRAM_DEBUG_U_BASE	            (0x48622400ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PDSP1_IRAM_U_BASE	            	(0x48624000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PDSP1_IRAM_DEBUG_U_BASE	            (0x48624400ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PROT_SLV_U_BASE	            		(0x48624C00ul)
#define CSL_ICSS_M_ICSSM_1_PR1_CFG_SLV_U_BASE	            		(0x48626000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_ICSS_UART_UART_SLV_U_BASE	        (0x48628000ul)
#define CSL_ICSS_M_ICSSM_1_IEP0_U_BASE	            				(0x4862E000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_ICSS_ECAP0_ECAP_SLV_U_BASE	        (0x48630000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_MII_RT_PR1_MII_RT_CFG_U_BASE	        (0x48632000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_MDIO_V1P7_MDIO_U_BASE	            (0x48632400ul)
#define CSL_ICSS_M_ICSSM_1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_U_BAS	(0x48633000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PDSP0_IRAM_RAM_U_BASE	            (0x48634000ul)
#define CSL_ICSS_M_ICSSM_1_PR1_PDSP1_IRAM_RAM_U_BASE	            (0x48638000ul)
#define CSL_ICSSM1_ECC	            								(0x48700000ul)
#define CSL_CONTROLSS_G0_EPWM0_U_BASE	            				(0x50000000ul)
#define CSL_CONTROLSS_G0_EPWM1_U_BASE	            				(0x50001000ul)
#define CSL_CONTROLSS_G0_EPWM2_U_BASE	            				(0x50002000ul)
#define CSL_CONTROLSS_G0_EPWM3_U_BASE	            				(0x50003000ul)
#define CSL_CONTROLSS_G0_EPWM4_U_BASE	            				(0x50004000ul)
#define CSL_CONTROLSS_G0_EPWM5_U_BASE	            				(0x50005000ul)
#define CSL_CONTROLSS_G0_EPWM6_U_BASE	            				(0x50006000ul)
#define CSL_CONTROLSS_G0_EPWM7_U_BASE	            				(0x50007000ul)
#define CSL_CONTROLSS_G0_EPWM8_U_BASE	            				(0x50008000ul)
#define CSL_CONTROLSS_G0_EPWM9_U_BASE	            				(0x50009000ul)
#define CSL_CONTROLSS_G0_EPWM_WLINK_U_BASE	        				(0x50020000ul)
#define CSL_CONTROLSS_G1_EPWM0_U_BASE	            				(0x50040000ul)
#define CSL_CONTROLSS_G1_EPWM1_U_BASE	            				(0x50041000ul)
#define CSL_CONTROLSS_G1_EPWM2_U_BASE	            				(0x50042000ul)
#define CSL_CONTROLSS_G1_EPWM3_U_BASE	            				(0x50043000ul)
#define CSL_CONTROLSS_G1_EPWM4_U_BASE	            				(0x50044000ul)
#define CSL_CONTROLSS_G1_EPWM5_U_BASE	            				(0x50045000ul)
#define CSL_CONTROLSS_G1_EPWM6_U_BASE	            				(0x50046000ul)
#define CSL_CONTROLSS_G1_EPWM7_U_BASE	            				(0x50047000ul)
#define CSL_CONTROLSS_G1_EPWM8_U_BASE	            				(0x50048000ul)
#define CSL_CONTROLSS_G1_EPWM9_U_BASE	            				(0x50049000ul)
#define CSL_CONTROLSS_G1_EPWM_WLINK_U_BASE	        				(0x50060000ul)
#define CSL_CONTROLSS_G2_EPWM0_U_BASE	            				(0x50080000ul)
#define CSL_CONTROLSS_G2_EPWM1_U_BASE	            				(0x50081000ul)
#define CSL_CONTROLSS_G2_EPWM2_U_BASE	            				(0x50082000ul)
#define CSL_CONTROLSS_G2_EPWM3_U_BASE	            				(0x50083000ul)
#define CSL_CONTROLSS_G2_EPWM4_U_BASE	            				(0x50084000ul)
#define CSL_CONTROLSS_G2_EPWM5_U_BASE	            				(0x50085000ul)
#define CSL_CONTROLSS_G2_EPWM6_U_BASE	            				(0x50086000ul)
#define CSL_CONTROLSS_G2_EPWM7_U_BASE	            				(0x50087000ul)
#define CSL_CONTROLSS_G2_EPWM8_U_BASE	            				(0x50088000ul)
#define CSL_CONTROLSS_G2_EPWM9_U_BASE	            				(0x50089000ul)
#define CSL_CONTROLSS_G2_EPWM_WLINK_U_BASE	        				(0x500A0000ul)
#define CSL_CONTROLSS_G3_EPWM0_U_BASE	            				(0x500C0000ul)
#define CSL_CONTROLSS_G3_EPWM1_U_BASE	            				(0x500C1000ul)
#define CSL_CONTROLSS_G3_EPWM2_U_BASE	            				(0x500C2000ul)
#define CSL_CONTROLSS_G3_EPWM3_U_BASE	            				(0x500C3000ul)
#define CSL_CONTROLSS_G3_EPWM4_U_BASE	            				(0x500C4000ul)
#define CSL_CONTROLSS_G3_EPWM5_U_BASE	            				(0x500C5000ul)
#define CSL_CONTROLSS_G3_EPWM6_U_BASE	            				(0x500C6000ul)
#define CSL_CONTROLSS_G3_EPWM7_U_BASE	            				(0x500C7000ul)
#define CSL_CONTROLSS_G3_EPWM8_U_BASE	            				(0x500C8000ul)
#define CSL_CONTROLSS_G3_EPWM9_U_BASE	            				(0x500C9000ul)
#define CSL_CONTROLSS_G3_EPWM_WLINK_U_BASE	        				(0x500E0000ul)
#define CSL_CONTROLSS_ADC0_RESULT_U_BASE	        				(0x50100000ul)
#define CSL_CONTROLSS_ADC1_RESULT_U_BASE	        				(0x50101000ul)
#define CSL_CONTROLSS_ADC2_RESULT_U_BASE	        				(0x50102000ul)
#define CSL_CONTROLSS_CMPSSA0_U_BASE	            				(0x50200000ul)
#define CSL_CONTROLSS_CMPSSA1_U_BASE	            				(0x50201000ul)
#define CSL_CONTROLSS_CMPSSA2_U_BASE	            				(0x50202000ul)
#define CSL_CONTROLSS_CMPSSA3_U_BASE	            				(0x50203000ul)
#define CSL_CONTROLSS_CMPSSA4_U_BASE	            				(0x50204000ul)
#define CSL_CONTROLSS_CMPSSA5_U_BASE	            				(0x50205000ul)
#define CSL_CONTROLSS_CMPSSA6_U_BASE	            				(0x50206000ul)
#define CSL_CONTROLSS_CMPSSA7_U_BASE	            				(0x50207000ul)
#define CSL_CONTROLSS_CMPSSA8_U_BASE	            				(0x50208000ul)
#define CSL_CONTROLSS_ECAP0_U_BASE	                				(0x50240000ul)
#define CSL_CONTROLSS_ECAP1_U_BASE	                				(0x50241000ul)
#define CSL_CONTROLSS_ECAP2_U_BASE	                				(0x50242000ul)
#define CSL_CONTROLSS_ECAP3_U_BASE	                				(0x50243000ul)
#define CSL_CONTROLSS_ECAP4_U_BASE	                				(0x50244000ul)
#define CSL_CONTROLSS_ECAP5_U_BASE	                				(0x50245000ul)
#define CSL_CONTROLSS_ECAP6_U_BASE	                				(0x50246000ul)
#define CSL_CONTROLSS_ECAP7_U_BASE	                				(0x50247000ul)
#define CSL_CONTROLSS_DAC0_U_BASE	                				(0x50260000ul)
#define CSL_CONTROLSS_SDFM0_U_BASE	                				(0x50268000ul)
#define CSL_CONTROLSS_SDFM1_U_BASE	                				(0x50269000ul)
#define CSL_CONTROLSS_EQEP0_U_BASE	                				(0x50270000ul)
#define CSL_CONTROLSS_EQEP1_U_BASE	                				(0x50271000ul)
#define CSL_CONTROLSS_FSI_TX0_U_BASE	            				(0x50280000ul)
#define CSL_CONTROLSS_FSI_RX0_U_BASE	            				(0x50290000ul)
#define CSL_CONTROLSS_ADC0_U_BASE	                				(0x502C0000ul)
#define CSL_CONTROLSS_ADC1_U_BASE	                				(0x502C1000ul)
#define CSL_CONTROLSS_ADC2_U_BASE	                				(0x502C2000ul)
#define CSL_CONTROLSS_ADCSAFE0_U_BASE	            				(0x502CB400ul)
#define CSL_CONTROLSS_ADCSAFE1_U_BASE	            				(0x502CB800ul)
#define CSL_CONTROLSS_ADCSAFE2_U_BASE	            				(0x502CBC00ul)
#define CSL_CONTROLSS_ADCSAFE3_U_BASE	            				(0x502CC000ul)
#define CSL_CONTROLSS_ADCSAFE4_U_BASE	            				(0x502CC400ul)
#define CSL_CONTROLSS_ADCSAFE5_U_BASE	            				(0x502CC800ul)
#define CSL_CONTROLSS_ADCSAFE_EVENT_AGG_U_BASE	    				(0x502CEC00ul)
#define CSL_CONTROLSS_INPUTXBAR_U_BASE	            				(0x502D0000ul)
#define CSL_CONTROLSS_PWMXBAR_U_BASE	            				(0x502D1000ul)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_U_BASE	        				(0x502D2000ul)
#define CSL_CONTROLSS_MDLXBAR_U_BASE	            				(0x502D3000ul)
#define CSL_CONTROLSS_ICLXBAR_U_BASE	            				(0x502D4000ul)
#define CSL_CONTROLSS_INTXBAR_U_BASE	            				(0x502D5000ul)
#define CSL_CONTROLSS_DMAXBAR_U_BASE	            				(0x502D6000ul)
#define CSL_CONTROLSS_OUTPUTXBAR_U_BASE	            				(0x502D8000ul)
#define CSL_CONTROLSS_OTTOCAL0_U_BASE	            				(0x502E0000ul)
#define CSL_CONTROLSS_OTTOCAL1_U_BASE	            				(0x502E1000ul)
#define CSL_CONTROLSS_CTRL_U_BASE	            					(0x502F0000ul)
#define CSL_MSS_CTRL_U_BASE	            							(0x50D00000ul)
#define CSL_TOP_CTRL_U_BASE	            							(0x50D80000ul)
#define CSL_SPINLOCK0_BASE	            							(0x50E00000ul)
#define CSL_VIM_U_BASE	            								(0x50F00000ul)
#define CSL_GPIO0_U_BASE	            							(0x52000000ul)
#define CSL_GPIO1_U_BASE	            							(0x52001000ul)
#define CSL_WDT0_U_BASE	            								(0x52100000ul)
#define CSL_WDT1_U_BASE	            								(0x52101000ul)
#define CSL_RTI0_U_BASE	            								(0x52180000ul)
#define CSL_RTI1_U_BASE	            								(0x52181000ul)
#define CSL_RTI2_U_BASE	            								(0x52182000ul)
#define CSL_RTI3_U_BASE	            								(0x52183000ul)
#define CSL_MCSPI0_U_BASE	            							(0x52200000ul)
#define CSL_MCSPI1_U_BASE	            							(0x52201000ul)
#define CSL_MCSPI2_U_BASE	            							(0x52202000ul)
#define CSL_MCSPI3_U_BASE	            							(0x52203000ul)
#define CSL_UART0_U_BASE	            							(0x52300000ul)
#define CSL_UART1_U_BASE	            							(0x52301000ul)
#define CSL_UART2_U_BASE	            							(0x52302000ul)
#define CSL_UART3_U_BASE	            							(0x52303000ul)
#define CSL_UART4_U_BASE	            							(0x52304000ul)
#define CSL_UART5_U_BASE	            							(0x52305000ul)
#define CSL_LIN0_U_BASE	            								(0x52400000ul)
#define CSL_LIN1_U_BASE	            								(0x52401000ul)
#define CSL_LIN_LIN_2_U_BASE	            						(0x52402000ul)
#define CSL_I2C0_U_BASE	            								(0x52500000ul)
#define CSL_I2C1_U_BASE	            								(0x52501000ul)
#define CSL_I2C2_U_BASE	            								(0x52502000ul)
#define CSL_MCAN0_MSG_RAM_U_BASE	            					(0x52600000ul)
#define CSL_MCAN0_CFG_U_BASE	            						(0x52608000ul)
#define CSL_MCAN1_MSG_RAM_U_BASE	            					(0x52610000ul)
#define CSL_MCAN1_CFG_U_BASE	            						(0x52618000ul)
#define CSL_MCAN0_ECC_U_BASE	            						(0x52700000ul)
#define CSL_MCAN1_ECC_U_BASE	            						(0x52701000ul)
#define CSL_ELM_ELM_0_ELM_U_BASE	            					(0x527F0000ul)
#define CSL_CPSW0_U_BASE	            							(0x52800000ul)
#define CSL_TPCC0_U_BASE	            							(0x52A00000ul)
#define CSL_TPTC00_U_BASE	            							(0x52A40000ul)
#define CSL_TPTC01_U_BASE	            							(0x52A60000ul)
#define CSL_DCC0_U_BASE	            								(0x52B00000ul)
#define CSL_DCC1_U_BASE	            								(0x52B01000ul)
#define CSL_DCC2_U_BASE	            								(0x52B02000ul)
#define CSL_DCC3_U_BASE	            								(0x52B03000ul)
#define CSL_TOP_ESM_U_BASE	        								(0x52D00000ul)
#define CSL_SOC_TIMESYNC_XBAR0_U_BASE	            				(0x52E00000ul)
#define CSL_EDMA_TRIG_XBAR_U_BASE	            					(0x52E01000ul)
#define CSL_GPIO_INTR_XBAR_U_BASE	            					(0x52E02000ul)
#define CSL_ICSSM_INTR_XBAR_U_BASE	            					(0x52E03000ul)
#define CSL_SOC_TIMESYNC_XBAR1_U_BASE	            				(0x52E04000ul)
#define CSL_ECC_AGG_R5SS0_CORE0_U_BASE	            				(0x53000000ul)
#define CSL_ECC_AGG_R5SS0_CORE1_U_BASE	            				(0x53003000ul)
#define CSL_ECC_AGG_TOP_U_BASE	            						(0x53010000ul)
#define CSL_TMU_ROM_R5SS0_CORE0_U_BASE	            				(0x53020000ul)
#define CSL_TMU_ROM_R5SS0_CORE1_U_BASE	            				(0x53024000ul)
#define CSL_IOMUX_U_BASE	            							(0x53100000ul)
#define CSL_TOP_RCM_U_BASE	            							(0x53200000ul)
#define CSL_MSS_RCM_U_BASE	            							(0x53208000ul)
#define CSL_R5SS0_CCMR_U_BASE	            						(0x53210000ul)
#define CSL_RL2_REGS_R5SS0_CORE0_U_BASE	            				(0x53212000ul)
#define CSL_RL2_REGS_R5SS0_CORE1_U_BASE	            				(0x53213000ul)
#define CSL_PBIST_PBIST_0_U_BASE	            					(0x53300200ul)
#define CSL_FSS_VBUSM_TO_CFG_U_BASE	            					(0x53400000ul)
#define CSL_R5SS0_STC_U_BASE	            						(0x53500000ul)
#define CSL_TOP_EFUSE_FARM_U_BASE	            					(0x53600000ul)
#define CSL_FLASH_CONFIG_REG0_U_BASE	            				(0x53800000ul)
#define CSL_FLASH_CONFIG_REG1_U_BASE	            				(0x53801000ul)
#define CSL_FLASH_CONFIG_REG2_U_BASE	            				(0x53802000ul)
#define CSL_FLASH_CONFIG_REG6_U_BASE	            				(0x53806000ul)
#define CSL_FLASH_CONFIG_REG7_U_BASE	            				(0x53807000ul)
#define CSL_FLASH_CONFIG_REG8_U_BASE	            				(0x53808000ul)
#define CSL_FLASH_CONFIG_REG11_U_BASE	            				(0x5380B000ul)
#define CSL_FLASH_CONFIG_REG12_U_BASE	            				(0x5380C000ul)
#define CSL_FLASH_CONFIG_REG13_U_BASE	            				(0x5380D000ul)
#define CSL_FLASH_CONFIG_REG14_U_BASE	            				(0x5380E000ul)
#define CSL_FLASH_CONFIG_REG15_U_BASE	            				(0x5380F000ul)
#define CSL_USB_OTGSS_C2_U_BASE	            						(0x53900000ul)
#define CSL_USB_TRBB_U_BASE	            							(0x53904000ul)
#define CSL_USB_RAM0_U_BASE	            							(0x53908000ul)
#define CSL_USB_DWC_3_U_BASE	            						(0x53910000ul)
#define CSL_USB_OCP2SCP_REG_U_BASE	            					(0x53940000ul)
#define CSL_USB2PHY_U_BASE	            							(0x53944000ul)
#define CSL_FSS_UL_128_FSS_OF_UL_OSPI0_OSPI_CFG_VBUSP_VBP_U_BASE	            (0x53A00000ul)
#define CSL_FSS_UL_128_FSS_OF_UL_OSPI0_OSPI_CFG_VBUSP_OSPI_WRAP_ECC_AGG_VBP_U_BASE	            (0x53A01000ul)
#define CSL_FSS_UL_128_FSS_OF_UL_OSPI0_OSPI_CFG_VBUSP_VBP2APB_WRAP_OSPI_CFG_VBP_OSPI_FLASH_APB_U_BASE			(0x53A02000ul)
#define CSL_FLASH_DATA_REG0_U_BASE	            					(0x60000000ul)
#define CSL_L2OCRAM_U_BASE	            							(0x70000000ul)
#define CSL_MSRAM_MSRAM_BANK_1_U_BASE	            				(0x70080000ul)
#define CSL_MSRAM_MSRAM_BANK_2_U_BASE	            				(0x70100000ul)
#define CSL_MBOX_SRAM_U_BASE	            						(0x72000000ul)
#define CSL_R5SS0_CORE0_ICACHE_U_BASE	            				(0x74000000ul)
#define CSL_R5SS0_CORE0_DCACHE_U_BASE	            				(0x74800000ul)
#define CSL_R5SS0_CORE1_ICACHE_U_BASE	            				(0x75000000ul)
#define CSL_R5SS0_CORE1_DCACHE_U_BASE	            				(0x75800000ul)
#define CSL_R5SS0_CORE0_TCMA_U_BASE	            					(0x78000000ul)
#define CSL_R5SS0_CORE0_TMU_EXT_U_BASE	            				(0x78060000ul)
#define CSL_R5SS0_CORE0_TCMB_U_BASE	            					(0x78100000ul)
#define CSL_R5SS0_CORE1_TCMA_U_BASE	            					(0x78200000ul)
#define CSL_R5SS0_CORE1_TMU_EXT_U_BASE	            				(0x78260000ul)
#define CSL_R5SS0_CORE1_TCMB_U_BASE	            					(0x78300000ul)
#define CSL_FLASH_DATA_REG1_U_BASE	            					(0x80000000ul)
#define CSL_FLASH_DATA_REG3_U_BASE	            					(0x88000000ul)
#define CSL_GPMC0_MEM_U_BASE	            						(0x90000000ul)
#define CSL_FSS_UL_128_FSS_OF_UL_DAT_REG0_U_BASE					(0xA0000000ul)
#define CSL_HSM_DTHE_U_BASE	            							(0xCE000000ul)
#define CSL_HSM_SHA_U_BASE	            							(0xCE004000ul)
#define CSL_HSM_AES_U_BASE	            							(0xCE006000ul)
#define CSL_HSM_TRNG_U_BASE	            							(0xCE00A000ul)
#define CSL_HSM_PKE_U_BASE	            							(0xCE010000ul)
#define CSL_HSM_PKA_U_BASE	            							(0xCE012000ul)
#define CSL_HSM_CM4_ICFG_U_BASE	            						(0xE0000000ul)
#define CSL_MPU_R5SS1_U_BASE	            						(0x402A0000ul)
#define CSL_MPU_L2OCRAM_BANK4_U_BASE	            				(0x402C0000ul)
#define CSL_MPU_L2OCRAM_BANK5_U_BASE	            				(0x402E0000ul)
#define CSL_CONTROLSS_CMPSSA9_U_BASE	            				(0x50209000ul)
#define CSL_DEBUGSS_U_BASE	            							(0x50800000ul)
#define CSL_GPIO2_U_BASE	            							(0x52002000ul)
#define CSL_GPIO3_U_BASE	            							(0x52003000ul)
#define CSL_TOP_PBIST_U_BASE	            						(0x53300000ul)
#define CSL_HSM_PKE_RAM_U_BASE	            						(0xCE014000ul)

// FIXME : ADD EXT FLASH ADDRESSES
#define CSL_EXT_FLASH0_U_BASE                  (0x60000000ul)
#define CSL_EXT_FLASH1_U_BASE                  (0x62000000ul)
#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_BASEADDRESS_H_ */
