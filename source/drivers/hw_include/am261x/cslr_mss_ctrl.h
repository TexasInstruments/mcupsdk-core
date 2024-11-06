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
 *  Name        : cslr_mss_ctrl.h
 *  VPVERSION   : 3.0.356 - 2023.07.16.20.08.37
 *  VPREV       : 2.23.0
*/
#ifndef CSLR_MSS_CTRL_H_
#define CSLR_MSS_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_MSS_CTRL_REGS_BASE                                            (0x00000000U)


/**************************************************************************
* Hardware Region  : MMRs in region 0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* PID register */
    volatile uint8_t  Resv_32[28];
    volatile uint32_t R5SS0_CONTROL;
    volatile uint8_t  Resv_40[4];
    volatile uint32_t R5SS0_FORCE_WFI;
    volatile uint8_t  Resv_64[20];
    volatile uint32_t R5SS0_CORE0_HALT;
    volatile uint32_t R5SS0_CORE1_HALT;
    volatile uint8_t  Resv_100[28];
    volatile uint32_t R5SS0_STATUS_REG;
    volatile uint8_t  Resv_128[24];
    volatile uint32_t R5SS0_CORE0_STAT;
    volatile uint32_t R5SS0_CORE1_STAT;
    volatile uint8_t  Resv_160[24];
    volatile uint32_t R5SS0_INIT_TCM;
    volatile uint8_t  Resv_192[28];
    volatile uint32_t R5SS0_AHB_EN;
    volatile uint8_t  Resv_200[4];
    volatile uint32_t R5SS0_TCM_EXT_ERR_EN;
    volatile uint8_t  Resv_208[4];
    volatile uint32_t R5SS0_TCM_ERR_EN;
    volatile uint8_t  Resv_216[4];
    volatile uint32_t R5SS0_TCM_ECC_WRENZ_EN;
    volatile uint8_t  Resv_272[52];
    volatile uint32_t R5SS0_CORE0_AHB_BASE;
    volatile uint8_t  Resv_280[4];
    volatile uint32_t R5SS0_CORE0_AHB_SIZE;
    volatile uint8_t  Resv_288[4];
    volatile uint32_t R5SS0_CORE1_AHB_BASE;
    volatile uint8_t  Resv_296[4];
    volatile uint32_t R5SS0_CORE1_AHB_SIZE;
    volatile uint8_t  Resv_336[36];
    volatile uint32_t R5SS0_TEINIT;
    volatile uint8_t  Resv_512[172];
    volatile uint32_t BOOT_INFO_REG0;
    volatile uint32_t BOOT_INFO_REG1;
    volatile uint32_t BOOT_INFO_REG2;
    volatile uint32_t BOOT_INFO_REG3;
    volatile uint32_t BOOT_INFO_REG4;
    volatile uint32_t BOOT_INFO_REG5;
    volatile uint32_t BOOT_INFO_REG6;
    volatile uint32_t BOOT_INFO_REG7;
    volatile uint8_t  Resv_768[224];
    volatile uint32_t R5SS0_ATCM_MEM_INIT;
    volatile uint32_t R5SS0_ATCM_MEM_INIT_DONE;
    volatile uint32_t R5SS0_ATCM_MEM_INIT_STATUS;
    volatile uint32_t R5SS0_BTCM_MEM_INIT;
    volatile uint32_t R5SS0_BTCM_MEM_INIT_DONE;
    volatile uint32_t R5SS0_BTCM_MEM_INIT_STATUS;
    volatile uint8_t  Resv_816[24];
    volatile uint32_t L2IOCRAM_MEM_INIT;
    volatile uint32_t L2OCRAM_MEM_INIT_DONE;
    volatile uint32_t L2OCRAM_MEM_INIT_STATUS;
    volatile uint32_t MAILBOXRAM_MEM_INIT;
    volatile uint32_t MAILBOXRAM_MEM_INIT_DONE;
    volatile uint32_t MAILBOXRAM_MEM_INIT_STATUS;
    volatile uint32_t TPCC_MEM_INIT;
    volatile uint32_t TPCC_MEM_INIT_DONE;
    volatile uint32_t TPCC_MEMINIT_STATUS;
    volatile uint8_t  Resv_1024[172];
    volatile uint32_t TOP_PBIST_KEY_RST;
    volatile uint32_t TOP_PBIST_REG0;
    volatile uint32_t TOP_PBIST_REG1;
    volatile uint32_t TOP_PBIST_REG2;
    volatile uint8_t  Resv_1280[240];
    volatile uint32_t R5SS0_CTI_TRIG_SEL;
    volatile uint8_t  Resv_1288[4];
    volatile uint32_t DBGSS_CTI_TRIG_SEL;
    volatile uint8_t  Resv_1312[20];
    volatile uint32_t MCAN0_HALTEN;
    volatile uint32_t MCAN1_HALTEN;
    volatile uint8_t  Resv_1376[56];
    volatile uint32_t LIN0_HALTEN;
    volatile uint32_t LIN1_HALTEN;
    volatile uint32_t LIN2_HALTEN;
    volatile uint8_t  Resv_1440[52];
    volatile uint32_t I2C0_HALTEN;
    volatile uint32_t I2C1_HALTEN;
    volatile uint32_t I2C2_HALTEN;
    volatile uint8_t  Resv_1504[52];
    volatile uint32_t RTI0_HALTEN;
    volatile uint32_t RTI1_HALTEN;
    volatile uint32_t RTI2_HALTEN;
    volatile uint32_t RTI3_HALTEN;
    volatile uint8_t  Resv_1568[48];
    volatile uint32_t CPSW_HALTEN;
    volatile uint32_t MCRC0_HALTEN;
    volatile uint8_t  Resv_2048[472];
    volatile uint32_t ICSSM0_PRU0_GPI_SEL;
    volatile uint32_t ICSSM0_PRU1_GPI_SEL;
    volatile uint32_t ICSSM1_PRU0_GPI_SEL;
    volatile uint32_t ICSSM1_PRU1_GPI_SEL;
    volatile uint32_t ICSSM0_PRU0_GPIO_OUT_CTRL;
    volatile uint32_t ICSSM0_PRU1_GPIO_OUT_CTRL;
    volatile uint32_t ICSSM1_PRU0_GPIO_OUT_CTRL;
    volatile uint32_t ICSSM1_PRU1_GPIO_OUT_CTRL;
    volatile uint32_t ICSSM0_RX_ERR_COUNTER;
    volatile uint32_t ICSSM0_RX_ERR_COUNTER_CLR;
    volatile uint32_t ICSSM1_RX_ERR_COUNTER;
    volatile uint32_t ICSSM1_RX_ERR_COUNTER_CLR;
    volatile uint32_t TPTC_BOUNDARY_CFG;
    volatile uint32_t TPTC_XID_REORDER_CFG;
    volatile uint32_t TPTC_DBS_CONFIG;
    volatile uint32_t OSPI_CONFIG;
    volatile uint32_t OSPI_BOOT_CONFIG_MASK;
    volatile uint32_t OSPI_BOOT_CONFIG_SEG;
    volatile uint32_t TPCC0_INTAGG_MASK;
    volatile uint32_t TPCC0_INTAGG_STATUS;
    volatile uint32_t TPCC0_INTAGG_STATUS_RAW;
    volatile uint32_t ICSSM_IDLE_CONTROL;
    volatile uint32_t GPMC_CONTROL;
    volatile uint32_t INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL;
    volatile uint32_t CPSW_CONTROL;
    volatile uint32_t ICSSM1_INPUT_INTR_SEL;
    volatile uint32_t ICSSM_GPO_SEL;
    volatile uint32_t DEBUGSS_CSETB_FLUSH;
    volatile uint32_t DEBUGSS_STM_NSGUAREN;
    volatile uint32_t CTRL_USB_CTRL;
    volatile uint32_t CTRL_USB_STS;
    volatile uint8_t  Resv_2176[4];
    volatile uint32_t USB_SLAVE_CONTROL;
    volatile uint32_t USB_MASTER_STANDBY;
    volatile uint8_t  Resv_2188[4];
    volatile uint32_t USB_UTMI_DRVVBUS_CONTROL;
    volatile uint8_t  Resv_2196[4];
    volatile uint32_t CONTROL_USBOTGHS_CONTROL;
    volatile uint32_t R5SS0_ROM_ECLIPSE;
    volatile uint32_t FSS_OE_NEXT_EN;
    volatile uint32_t OSPI1_CONFIG_CONTROL;
    volatile uint32_t CTRLMMR_ICSSM0_CTRL0;
    volatile uint32_t CTRLMMR_ICSSM0_CTRL1;
    volatile uint32_t CTRLMMR_ICSSM1_CTRL0;
    volatile uint32_t CTRLMMR_ICSSM1_CTRL1;
    volatile uint8_t  Resv_3904[1676];
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint8_t  Resv_3968[48];
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint8_t  Resv_4032[48];
    volatile uint32_t HW_SPARE_REC;
    volatile uint8_t  Resv_4104[68];
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
    volatile uint8_t  Resv_16384[12236];
    volatile uint32_t R5SS0_CORE0_MBOX_WRITE_DONE;
    volatile uint32_t R5SS0_CORE0_MBOX_READ_REQ;
    volatile uint32_t R5SS0_CORE0_MBOX_READ_DONE_ACK;
    volatile uint32_t R5SS0_CORE0_MBOX_READ_DONE;
    volatile uint32_t R5SS0_CORE0_SW_INT;
    volatile uint8_t  Resv_16416[12];
    volatile uint32_t MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK;
    volatile uint32_t MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS;
    volatile uint32_t MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW;
    volatile uint8_t  Resv_16432[4];
    volatile uint32_t MPU_PROT_ERRAGG_R5SS0_CPU0_MASK;
    volatile uint32_t MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS;
    volatile uint32_t MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW;
    volatile uint8_t  Resv_32768[16324];
    volatile uint32_t R5SS0_CORE1_MBOX_WRITE_DONE;
    volatile uint32_t R5SS0_CORE1_MBOX_READ_REQ;
    volatile uint32_t R5SS0_CORE1_MBOX_READ_DONE_ACK;
    volatile uint32_t R5SS0_CORE1_MBOX_READ_DONE;
    volatile uint32_t R5SS0_CORE1_SW_INT;
    volatile uint8_t  Resv_32800[12];
    volatile uint32_t MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK;
    volatile uint32_t MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS;
    volatile uint32_t MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW;
    volatile uint8_t  Resv_32816[4];
    volatile uint32_t MPU_PROT_ERRAGG_R5SS0_CPU1_MASK;
    volatile uint32_t MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS;
    volatile uint32_t MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW;
    volatile uint8_t  Resv_81920[49092];
    volatile uint32_t ICSSM0_PRU0_MBOX_WRITE_DONE;
    volatile uint32_t ICSSM0_PRU0_MBOX_READ_REQ;
    volatile uint32_t ICSSM0_PRU0_MBOX_READ_DONE_ACK;
    volatile uint32_t ICSSM0_PRU0_MBOX_READ_DONE;
    volatile uint32_t ICSSM0_PRU1_MBOX_WRITE_DONE;
    volatile uint32_t ICSSM0_PRU1_MBOX_READ_REQ;
    volatile uint32_t ICSSM0_PRU1_MBOX_READ_DONE_ACK;
    volatile uint32_t ICSSM0_PRU1_MBOX_READ_DONE;
    volatile uint32_t ICSSM1_PRU0_MBOX_WRITE_DONE;
    volatile uint32_t ICSSM1_PRU0_MBOX_READ_REQ;
    volatile uint32_t ICSSM1_PRU0_MBOX_READ_DONE_ACK;
    volatile uint32_t ICSSM1_PRU0_MBOX_READ_DONE;
    volatile uint32_t ICSSM1_PRU1_MBOX_WRITE_DONE;
    volatile uint32_t ICSSM1_PRU1_MBOX_READ_REQ;
    volatile uint32_t ICSSM1_PRU1_MBOX_READ_DONE_ACK;
    volatile uint32_t ICSSM1_PRU1_MBOX_READ_DONE;
    volatile uint8_t  Resv_98304[16320];
    volatile uint32_t TPCC0_ERRAGG_MASK;
    volatile uint32_t TPCC0_ERRAGG_STATUS;
    volatile uint32_t TPCC0_ERRAGG_STATUS_RAW;
    volatile uint32_t MMR_ACCESS_ERRAGG_MASK0;
    volatile uint32_t MMR_ACCESS_ERRAGG_STATUS0;
    volatile uint32_t MMR_ACCESS_ERRAGG_STATUS_RAW0;
    volatile uint32_t R5SS0_CPU0_ECC_CORR_ERRAGG_MASK;
    volatile uint32_t R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS;
    volatile uint32_t R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW;
    volatile uint32_t R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK;
    volatile uint32_t R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS;
    volatile uint32_t R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW;
    volatile uint32_t R5SS0_CPU1_ECC_CORR_ERRAGG_MASK;
    volatile uint32_t R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS;
    volatile uint32_t R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW;
    volatile uint32_t R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK;
    volatile uint32_t R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS;
    volatile uint32_t R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW;
    volatile uint8_t  Resv_98424[48];
    volatile uint32_t R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK;
    volatile uint32_t R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS;
    volatile uint32_t R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW;
    volatile uint32_t R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK;
    volatile uint32_t R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS;
    volatile uint32_t R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW;
    volatile uint8_t  Resv_98560[112];
    volatile uint32_t MSS_VBUSM_SAFETY_H_ERRAGG_MASK0;
    volatile uint32_t MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0;
    volatile uint32_t MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0;
    volatile uint32_t MSS_VBUSM_SAFETY_H_ERRAGG_MASK1;
    volatile uint32_t MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1;
    volatile uint32_t MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1;
    volatile uint32_t MSS_VBUSM_SAFETY_L_ERRAGG_MASK0;
    volatile uint32_t MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0;
    volatile uint32_t MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0;
    volatile uint32_t MSS_VBUSM_SAFETY_L_ERRAGG_MASK1;
    volatile uint32_t MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1;
    volatile uint32_t MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1;
    volatile uint8_t  Resv_98688[80];
    volatile uint32_t MSS_VBUSP_SAFETY_H_ERRAGG_MASK;
    volatile uint32_t MSS_VBUSP_SAFETY_H_ERRAGG_STATUS;
    volatile uint32_t MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW;
    volatile uint8_t  Resv_98816[116];
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t TPTC00_RD_BUS_SAFETY_CTRL;
    volatile uint32_t TPTC00_RD_BUS_SAFETY_FI;
    volatile uint32_t TPTC00_RD_BUS_SAFETY_ERR;
    volatile uint32_t TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t TPTC00_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t TPTC01_RD_BUS_SAFETY_CTRL;
    volatile uint32_t TPTC01_RD_BUS_SAFETY_FI;
    volatile uint32_t TPTC01_RD_BUS_SAFETY_ERR;
    volatile uint32_t TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t TPTC01_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_CTRL;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_FI;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_ERR;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_CTRL;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_FI;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_ERR;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_FI;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DAP_BUS_SAFETY_CTRL;
    volatile uint32_t DAP_BUS_SAFETY_FI;
    volatile uint32_t DAP_BUS_SAFETY_ERR;
    volatile uint32_t DAP_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DAP_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DAP_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DAP_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DAP_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_CTRL;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_FI;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_CTRL;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_FI;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_CTRL;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_FI;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_CTRL;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_FI;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_ERR;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t STM_STIM_BUS_SAFETY_CTRL;
    volatile uint32_t STM_STIM_BUS_SAFETY_FI;
    volatile uint32_t STM_STIM_BUS_SAFETY_ERR;
    volatile uint32_t STM_STIM_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t STM_STIM_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t STM_STIM_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t STM_STIM_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_99360[48];
    volatile uint32_t HSM_TPTC0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC0_RD_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC0_RD_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint8_t  Resv_99392[8];
    volatile uint32_t HSM_TPTC1_RD_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC1_RD_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC1_RD_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint8_t  Resv_99424[8];
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_99456[4];
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t OSPI0_BUS_SAFETY_CTRL;
    volatile uint32_t OSPI0_BUS_SAFETY_FI;
    volatile uint32_t OSPI0_BUS_SAFETY_ERR;
    volatile uint32_t OSPI0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t OSPI0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t OSPI0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t OSPI0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_99520[4];
    volatile uint32_t HSM_DTHE_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_FI;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MMC0_BUS_SAFETY_CTRL;
    volatile uint32_t MMC0_BUS_SAFETY_FI;
    volatile uint32_t MMC0_BUS_SAFETY_ERR;
    volatile uint32_t MMC0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MMC0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MMC0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MMC0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MMC0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_CTRL;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_FI;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_ERR;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_CTRL;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_FI;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_ERR;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MCRC0_BUS_SAFETY_CTRL;
    volatile uint32_t MCRC0_BUS_SAFETY_FI;
    volatile uint32_t MCRC0_BUS_SAFETY_ERR;
    volatile uint32_t MCRC0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MCRC0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MCRC0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MCRC0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_CTRL;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_FI;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_ERR;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t HSM_M_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_M_BUS_SAFETY_FI;
    volatile uint32_t HSM_M_BUS_SAFETY_ERR;
    volatile uint32_t HSM_M_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_M_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_M_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_M_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t HSM_S_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_S_BUS_SAFETY_FI;
    volatile uint32_t HSM_S_BUS_SAFETY_ERR;
    volatile uint32_t HSM_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_CTRL;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_FI;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_ERR;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_CTRL;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_FI;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_ERR;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_CTRL;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_FI;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_ERR;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_CTRL;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_FI;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_ERR;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_CTRL;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_FI;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_ERR;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_CTRL;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_FI;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_ERR;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t USBSS_RD_BUS_SAFETY_CTRL;
    volatile uint32_t USBSS_RD_BUS_SAFETY_FI;
    volatile uint32_t USBSS_RD_BUS_SAFETY_ERR;
    volatile uint32_t USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t USBSS_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t USBSS_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t USBSS_WR_BUS_SAFETY_CTRL;
    volatile uint32_t USBSS_WR_BUS_SAFETY_FI;
    volatile uint32_t USBSS_WR_BUS_SAFETY_ERR;
    volatile uint32_t USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t USBSS_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t GPMC0_BUS_SAFETY_CTRL;
    volatile uint32_t GPMC0_BUS_SAFETY_FI;
    volatile uint32_t GPMC0_BUS_SAFETY_ERR;
    volatile uint32_t GPMC0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t GPMC0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t GPMC0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t GPMC0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_100096[12];
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ;
    volatile uint32_t L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP;
    volatile uint8_t  Resv_100240[24];
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_CTRL;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_FI;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_ERR;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_CTRL;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_FI;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_ERR;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L;
    volatile uint32_t PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H;
    volatile uint32_t BUS_SAFETY_CTRL;
    volatile uint32_t MSS_BUS_SAFETY_SEC_ERR_STAT0;
    volatile uint32_t MSS_BUS_SAFETY_SEC_ERR_STAT1;
    volatile uint8_t  Resv_100352[24];
    volatile uint32_t R5SS0_TCM_ADDRPARITY_CLR;
    volatile uint32_t R5SS0_CORE0_ADDRPARITY_ERR_ATCM;
    volatile uint32_t R5SS0_CORE1_ADDRPARITY_ERR_ATCM;
    volatile uint32_t R5SS0_CORE0_ERR_ADDRPARITY_B0TCM;
    volatile uint32_t R5SS0_CORE1_ERR_ADDRPARITY_B0TCM;
    volatile uint32_t R5SS0_CORE0_ERR_ADDRPARITY_B1TCM;
    volatile uint32_t R5SS0_CORE1_ERR_ADDRPARITY_B1TCM;
    volatile uint8_t  Resv_100404[24];
    volatile uint32_t R5SS0_TCM_ADDRPARITY_ERRFORCE;
    volatile uint8_t  Resv_100416[8];
    volatile uint32_t TPCC0_PARITY_CTRL;
    volatile uint32_t TPCC0_PARITY_STATUS;
    volatile uint32_t TMU_R5SS0_CORE0_ROM_PARITY_CTRL;
    volatile uint32_t TMU_R5SS0_CORE0_ROM_PARITY_STATUS;
    volatile uint32_t TMU_R5SS0_CORE1_ROM_PARITY_CTRL;
    volatile uint32_t TMU_R5SS0_CORE1_ROM_PARITY_STATUS;
    volatile uint8_t  Resv_100456[16];
    volatile uint32_t OSPI1_BUS_SAFETY_CTRL;
    volatile uint32_t OSPI1_BUS_SAFETY_FI;
    volatile uint32_t OSPI1_BUS_SAFETY_ERR;
    volatile uint32_t OSPI1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t OSPI1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t OSPI1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t OSPI1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_100608[120];
    volatile uint32_t NERROR_MASK;
} CSL_mss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_CTRL_PID                                                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CONTROL                                        (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_FORCE_WFI                                      (0x00000028U)
#define CSL_MSS_CTRL_R5SS0_CORE0_HALT                                     (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CORE1_HALT                                     (0x00000044U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG                                     (0x00000064U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT                                     (0x00000080U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT                                     (0x00000084U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM                                       (0x000000A0U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN                                         (0x000000C0U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN                                 (0x000000C8U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN                                     (0x000000D0U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN                               (0x000000D8U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BASE                                 (0x00000110U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE                                 (0x00000118U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BASE                                 (0x00000120U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE                                 (0x00000128U)
#define CSL_MSS_CTRL_R5SS0_TEINIT                                         (0x00000150U)
#define CSL_MSS_CTRL_BOOT_INFO_REG0                                       (0x00000200U)
#define CSL_MSS_CTRL_BOOT_INFO_REG1                                       (0x00000204U)
#define CSL_MSS_CTRL_BOOT_INFO_REG2                                       (0x00000208U)
#define CSL_MSS_CTRL_BOOT_INFO_REG3                                       (0x0000020CU)
#define CSL_MSS_CTRL_BOOT_INFO_REG4                                       (0x00000210U)
#define CSL_MSS_CTRL_BOOT_INFO_REG5                                       (0x00000214U)
#define CSL_MSS_CTRL_BOOT_INFO_REG6                                       (0x00000218U)
#define CSL_MSS_CTRL_BOOT_INFO_REG7                                       (0x0000021CU)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT                                  (0x00000300U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE                             (0x00000304U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS                           (0x00000308U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT                                  (0x0000030CU)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE                             (0x00000310U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS                           (0x00000314U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT                                    (0x00000330U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE                                (0x00000334U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS                              (0x00000338U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT                                  (0x0000033CU)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE                             (0x00000340U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_STATUS                           (0x00000344U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT                                        (0x00000348U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_DONE                                   (0x0000034CU)
#define CSL_MSS_CTRL_TPCC_MEMINIT_STATUS                                  (0x00000350U)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST                                    (0x00000400U)
#define CSL_MSS_CTRL_TOP_PBIST_REG0                                       (0x00000404U)
#define CSL_MSS_CTRL_TOP_PBIST_REG1                                       (0x00000408U)
#define CSL_MSS_CTRL_TOP_PBIST_REG2                                       (0x0000040CU)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL                                   (0x00000500U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL                                   (0x00000508U)
#define CSL_MSS_CTRL_MCAN0_HALTEN                                         (0x00000520U)
#define CSL_MSS_CTRL_MCAN1_HALTEN                                         (0x00000524U)
#define CSL_MSS_CTRL_LIN0_HALTEN                                          (0x00000560U)
#define CSL_MSS_CTRL_LIN1_HALTEN                                          (0x00000564U)
#define CSL_MSS_CTRL_LIN2_HALTEN                                          (0x00000568U)
#define CSL_MSS_CTRL_I2C0_HALTEN                                          (0x000005A0U)
#define CSL_MSS_CTRL_I2C1_HALTEN                                          (0x000005A4U)
#define CSL_MSS_CTRL_I2C2_HALTEN                                          (0x000005A8U)
#define CSL_MSS_CTRL_RTI0_HALTEN                                          (0x000005E0U)
#define CSL_MSS_CTRL_RTI1_HALTEN                                          (0x000005E4U)
#define CSL_MSS_CTRL_RTI2_HALTEN                                          (0x000005E8U)
#define CSL_MSS_CTRL_RTI3_HALTEN                                          (0x000005ECU)
#define CSL_MSS_CTRL_CPSW_HALTEN                                          (0x00000620U)
#define CSL_MSS_CTRL_MCRC0_HALTEN                                         (0x00000624U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPI_SEL                                  (0x00000800U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPI_SEL                                  (0x00000804U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPI_SEL                                  (0x00000808U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPI_SEL                                  (0x0000080CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL                            (0x00000810U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPIO_OUT_CTRL                            (0x00000814U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL                            (0x00000818U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPIO_OUT_CTRL                            (0x0000081CU)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER                                (0x00000820U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR                            (0x00000824U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER                                (0x00000828U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR                            (0x0000082CU)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG                                    (0x00000830U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG                                 (0x00000834U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG                                      (0x00000838U)
#define CSL_MSS_CTRL_OSPI_CONFIG                                          (0x0000083CU)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_MASK                                (0x00000840U)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_SEG                                 (0x00000844U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK                                    (0x00000848U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS                                  (0x0000084CU)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW                              (0x00000850U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL                                   (0x00000854U)
#define CSL_MSS_CTRL_GPMC_CONTROL                                         (0x00000858U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL                (0x0000085CU)
#define CSL_MSS_CTRL_CPSW_CONTROL                                         (0x00000860U)
#define CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL                                (0x00000864U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL                                        (0x00000868U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH                                  (0x0000086CU)
#define CSL_MSS_CTRL_DEBUGSS_STM_NSGUAREN                                 (0x00000870U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL                                        (0x00000874U)
#define CSL_MSS_CTRL_CTRL_USB_STS                                         (0x00000878U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL                                    (0x00000880U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY                                   (0x00000884U)
#define CSL_MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL                             (0x0000088CU)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL                             (0x00000894U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE                                    (0x00000898U)
#define CSL_MSS_CTRL_FSS_OE_NEXT_EN                                       (0x0000089CU)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL                                 (0x000008A0U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL0                                 (0x000008A4U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL1                                 (0x000008A8U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL0                                 (0x000008ACU)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL1                                 (0x000008B0U)
#define CSL_MSS_CTRL_HW_SPARE_RW0                                         (0x00000F40U)
#define CSL_MSS_CTRL_HW_SPARE_RW1                                         (0x00000F44U)
#define CSL_MSS_CTRL_HW_SPARE_RW2                                         (0x00000F48U)
#define CSL_MSS_CTRL_HW_SPARE_RW3                                         (0x00000F4CU)
#define CSL_MSS_CTRL_HW_SPARE_RO0                                         (0x00000F80U)
#define CSL_MSS_CTRL_HW_SPARE_RO1                                         (0x00000F84U)
#define CSL_MSS_CTRL_HW_SPARE_RO2                                         (0x00000F88U)
#define CSL_MSS_CTRL_HW_SPARE_RO3                                         (0x00000F8CU)
#define CSL_MSS_CTRL_HW_SPARE_REC                                         (0x00000FC0U)
#define CSL_MSS_CTRL_LOCK0_KICK0                                          (0x00001008U)
#define CSL_MSS_CTRL_LOCK0_KICK1                                          (0x0000100CU)
#define CSL_MSS_CTRL_INTR_RAW_STATUS                                      (0x00001010U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR                            (0x00001014U)
#define CSL_MSS_CTRL_INTR_ENABLE                                          (0x00001018U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR                                    (0x0000101CU)
#define CSL_MSS_CTRL_EOI                                                  (0x00001020U)
#define CSL_MSS_CTRL_FAULT_ADDRESS                                        (0x00001024U)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS                                    (0x00001028U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS                                    (0x0000102CU)
#define CSL_MSS_CTRL_FAULT_CLEAR                                          (0x00001030U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE                          (0x00004000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ                            (0x00004004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_ACK                       (0x00004008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE                           (0x0000400CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_SW_INT                                   (0x00004010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK                      (0x00004020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS                    (0x00004024U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW                (0x00004028U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK                      (0x00004030U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS                    (0x00004034U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW                (0x00004038U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE                          (0x00008000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ                            (0x00008004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_ACK                       (0x00008008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE                           (0x0000800CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_SW_INT                                   (0x00008010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK                      (0x00008020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS                    (0x00008024U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW                (0x00008028U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK                      (0x00008030U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS                    (0x00008034U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW                (0x00008038U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE                          (0x00014000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ                            (0x00014004U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_ACK                       (0x00014008U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE                           (0x0001400CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE                          (0x00014010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ                            (0x00014014U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_ACK                       (0x00014018U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE                           (0x0001401CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE                          (0x00014020U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ                            (0x00014024U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_ACK                       (0x00014028U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE                           (0x0001402CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE                          (0x00014030U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ                            (0x00014034U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_ACK                       (0x00014038U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE                           (0x0001403CU)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK                                    (0x00018000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS                                  (0x00018004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW                              (0x00018008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0                              (0x0001800CU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0                            (0x00018010U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0                        (0x00018014U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK                      (0x00018018U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS                    (0x0001801CU)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW                (0x00018020U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK                    (0x00018024U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS                  (0x00018028U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW              (0x0001802CU)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK                      (0x00018030U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS                    (0x00018034U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW                (0x00018038U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK                    (0x0001803CU)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS                  (0x00018040U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW              (0x00018044U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK                (0x00018078U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS              (0x0001807CU)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW          (0x00018080U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK                (0x00018084U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS              (0x00018088U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW          (0x0001808CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0                      (0x00018100U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0                    (0x00018104U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0                (0x00018108U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1                      (0x0001810CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1                    (0x00018110U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1                (0x00018114U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0                      (0x00018118U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0                    (0x0001811CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0                (0x00018120U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1                      (0x00018124U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1                    (0x00018128U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1                (0x0001812CU)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK                       (0x00018180U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS                     (0x00018184U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW                 (0x00018188U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL                   (0x00018200U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI                     (0x00018204U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR                    (0x00018208U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0         (0x0001820CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD           (0x00018210U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ          (0x00018214U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL                   (0x00018218U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI                     (0x0001821CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR                    (0x00018220U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0         (0x00018224U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD           (0x00018228U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ          (0x0001822CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL                   (0x00018230U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI                     (0x00018234U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR                    (0x00018238U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0         (0x0001823CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD           (0x00018240U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE         (0x00018244U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP     (0x00018248U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL                   (0x0001824CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI                     (0x00018250U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR                    (0x00018254U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0         (0x00018258U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD           (0x0001825CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE         (0x00018260U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP     (0x00018264U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL                    (0x00018268U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI                      (0x0001826CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR                     (0x00018270U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0          (0x00018274U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD            (0x00018278U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE          (0x0001827CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ           (0x00018280U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP      (0x00018284U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL                    (0x00018288U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI                      (0x0001828CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR                     (0x00018290U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0          (0x00018294U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD            (0x00018298U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE          (0x0001829CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ           (0x000182A0U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP      (0x000182A4U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL                            (0x000182A8U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI                              (0x000182ACU)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR                             (0x000182B0U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x000182B4U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000182B8U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000182BCU)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL                            (0x000182C0U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI                              (0x000182C4U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR                             (0x000182C8U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x000182CCU)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000182D0U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000182D4U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL                            (0x000182D8U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI                              (0x000182DCU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR                             (0x000182E0U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000182E4U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000182E8U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000182ECU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000182F0U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL                            (0x000182F4U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI                              (0x000182F8U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR                             (0x000182FCU)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00018300U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00018304U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00018308U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x0001830CU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL                             (0x00018310U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI                               (0x00018314U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR                              (0x00018318U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0                   (0x0001831CU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD                     (0x00018320U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE                   (0x00018324U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ                    (0x00018328U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP               (0x0001832CU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL                                  (0x00018330U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI                                    (0x00018334U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR                                   (0x00018338U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0                        (0x0001833CU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_CMD                          (0x00018340U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITE                        (0x00018344U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_READ                         (0x00018348U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x0001834CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL                        (0x00018350U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI                          (0x00018354U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR                         (0x00018358U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0              (0x0001835CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD                (0x00018360U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE              (0x00018364U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ               (0x00018368U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP          (0x0001836CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL                        (0x00018370U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI                          (0x00018374U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR                         (0x00018378U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0              (0x0001837CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD                (0x00018380U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE              (0x00018384U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ               (0x00018388U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP          (0x0001838CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL                        (0x00018390U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI                          (0x00018394U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR                         (0x00018398U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0              (0x0001839CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD                (0x000183A0U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE              (0x000183A4U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ               (0x000183A8U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP          (0x000183ACU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL                            (0x000183B0U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI                              (0x000183B4U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR                             (0x000183B8U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0                  (0x000183BCU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD                    (0x000183C0U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE                  (0x000183C4U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ                   (0x000183C8U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000183CCU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL                             (0x000183D0U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI                               (0x000183D4U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR                              (0x000183D8U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0                   (0x000183DCU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_CMD                     (0x000183E0U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITE                   (0x000183E4U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_READ                    (0x000183E8U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP               (0x000183ECU)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL                         (0x00018420U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI                           (0x00018424U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR                          (0x00018428U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0               (0x0001842CU)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD                 (0x00018430U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ                (0x00018434U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL                         (0x00018440U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI                           (0x00018444U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR                          (0x00018448U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0               (0x0001844CU)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD                 (0x00018450U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ                (0x00018454U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL                         (0x00018460U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI                           (0x00018464U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR                          (0x00018468U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0               (0x0001846CU)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD                 (0x00018470U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE               (0x00018474U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP           (0x00018478U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL                         (0x00018480U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI                           (0x00018484U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR                          (0x00018488U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0               (0x0001848CU)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD                 (0x00018490U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE               (0x00018494U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP           (0x00018498U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL                                (0x0001849CU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI                                  (0x000184A0U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR                                 (0x000184A4U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0                      (0x000184A8U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_CMD                        (0x000184ACU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITE                      (0x000184B0U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_READ                       (0x000184B4U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x000184B8U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL                             (0x000184C0U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI                               (0x000184C4U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR                              (0x000184C8U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0                   (0x000184CCU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD                     (0x000184D0U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE                   (0x000184D4U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ                    (0x000184D8U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP               (0x000184DCU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL                                 (0x000184E0U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI                                   (0x000184E4U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR                                  (0x000184E8U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0                       (0x000184ECU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_CMD                         (0x000184F0U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITE                       (0x000184F4U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_READ                        (0x000184F8U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x000184FCU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL                          (0x00018500U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI                            (0x00018504U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR                           (0x00018508U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0                (0x0001850CU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD                  (0x00018510U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE                (0x00018514U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ                 (0x00018518U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP            (0x0001851CU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL                          (0x00018520U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI                            (0x00018524U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR                           (0x00018528U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0                (0x0001852CU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD                  (0x00018530U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE                (0x00018534U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ                 (0x00018538U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP            (0x0001853CU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL                                (0x00018540U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI                                  (0x00018544U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR                                 (0x00018548U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0                      (0x0001854CU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_CMD                        (0x00018550U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITE                      (0x00018554U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_READ                       (0x00018558U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x0001855CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL                      (0x00018560U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI                        (0x00018564U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR                       (0x00018568U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0            (0x0001856CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD              (0x00018570U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE            (0x00018574U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ             (0x00018578U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP        (0x0001857CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL                      (0x00018580U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI                        (0x00018584U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR                       (0x00018588U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0            (0x0001858CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD              (0x00018590U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE            (0x00018594U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ             (0x00018598U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP        (0x0001859CU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL                                (0x000185A0U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI                                  (0x000185A4U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR                                 (0x000185A8U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0                      (0x000185ACU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD                        (0x000185B0U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE                      (0x000185B4U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ                       (0x000185B8U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x000185BCU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL                                (0x000185C0U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI                                  (0x000185C4U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR                                 (0x000185C8U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0                      (0x000185CCU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD                        (0x000185D0U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE                      (0x000185D4U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ                       (0x000185D8U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x000185DCU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL                             (0x000185E0U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI                               (0x000185E4U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR                              (0x000185E8U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0                   (0x000185ECU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD                     (0x000185F0U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE                   (0x000185F4U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_READ                    (0x000185F8U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP               (0x000185FCU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL                         (0x00018600U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI                           (0x00018604U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR                          (0x00018608U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0               (0x0001860CU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD                 (0x00018610U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE               (0x00018614U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ                (0x00018618U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP           (0x0001861CU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL                         (0x00018620U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI                           (0x00018624U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR                          (0x00018628U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0               (0x0001862CU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD                 (0x00018630U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE               (0x00018634U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ                (0x00018638U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP           (0x0001863CU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL                         (0x00018640U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI                           (0x00018644U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR                          (0x00018648U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0               (0x0001864CU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD                 (0x00018650U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE               (0x00018654U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ                (0x00018658U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP           (0x0001865CU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL                         (0x00018660U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI                           (0x00018664U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR                          (0x00018668U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0               (0x0001866CU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD                 (0x00018670U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE               (0x00018674U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ                (0x00018678U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP           (0x0001867CU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL                             (0x00018680U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI                               (0x00018684U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR                              (0x00018688U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0                   (0x0001868CU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD                     (0x00018690U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE                   (0x00018694U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_READ                    (0x00018698U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP               (0x0001869CU)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL                             (0x000186A0U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI                               (0x000186A4U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR                              (0x000186A8U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0                   (0x000186ACU)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_CMD                     (0x000186B0U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_READ                    (0x000186B4U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL                             (0x000186B8U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI                               (0x000186BCU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR                              (0x000186C0U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0                   (0x000186C4U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_CMD                     (0x000186C8U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE                   (0x000186CCU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP               (0x000186D0U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL                                (0x000186D4U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI                                  (0x000186D8U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR                                 (0x000186DCU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0                      (0x000186E0U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_CMD                        (0x000186E4U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITE                      (0x000186E8U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_READ                       (0x000186ECU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x000186F0U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD      (0x00018700U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ     (0x00018704U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD      (0x00018708U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ     (0x0001870CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD      (0x00018710U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE    (0x00018714U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP (0x00018718U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD      (0x0001871CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE    (0x00018720U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP (0x00018724U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD       (0x00018728U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE     (0x0001872CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ      (0x00018730U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP (0x00018734U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD       (0x00018738U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE     (0x0001873CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ      (0x00018740U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP (0x00018744U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD           (0x00018748U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE         (0x0001874CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ          (0x00018750U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP     (0x00018754U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD           (0x00018758U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE         (0x0001875CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ          (0x00018760U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP     (0x00018764U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD           (0x00018768U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE         (0x0001876CU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ          (0x00018770U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP     (0x00018774U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL                           (0x00018790U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI                             (0x00018794U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR                            (0x00018798U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0                 (0x0001879CU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD                   (0x000187A0U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE                 (0x000187A4U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ                  (0x000187A8U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000187ACU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS                  (0x000187B0U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL                           (0x000187B4U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI                             (0x000187B8U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR                            (0x000187BCU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0                 (0x000187C0U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD                   (0x000187C4U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE                 (0x000187C8U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ                  (0x000187CCU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000187D0U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L                (0x000187D4U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H                (0x000187D8U)
#define CSL_MSS_CTRL_BUS_SAFETY_CTRL                                      (0x000187DCU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0                         (0x000187E0U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1                         (0x000187E4U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR                             (0x00018800U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ADDRPARITY_ERR_ATCM                      (0x00018804U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ADDRPARITY_ERR_ATCM                      (0x00018808U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B0TCM                     (0x0001880CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B0TCM                     (0x00018810U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B1TCM                     (0x00018814U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B1TCM                     (0x00018818U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE                        (0x00018834U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL                                    (0x00018840U)
#define CSL_MSS_CTRL_TPCC0_PARITY_STATUS                                  (0x00018844U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL                      (0x00018848U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_STATUS                    (0x0001884CU)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL                      (0x00018850U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_STATUS                    (0x00018854U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL                                (0x00018868U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI                                  (0x0001886CU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR                                 (0x00018870U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0                      (0x00018874U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_CMD                        (0x00018878U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITE                      (0x0001887CU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_READ                       (0x00018880U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x00018884U)
#define CSL_MSS_CTRL_NERROR_MASK                                          (0x00018900U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MSS_CTRL_PID_PID_MINOR_MASK                                   (0x0000003FU)
#define CSL_MSS_CTRL_PID_PID_MINOR_SHIFT                                  (0x00000000U)
#define CSL_MSS_CTRL_PID_PID_MINOR_RESETVAL                               (0x00000015U)
#define CSL_MSS_CTRL_PID_PID_MINOR_MAX                                    (0x0000003FU)

#define CSL_MSS_CTRL_PID_PID_CUSTOM_MASK                                  (0x000000C0U)
#define CSL_MSS_CTRL_PID_PID_CUSTOM_SHIFT                                 (0x00000006U)
#define CSL_MSS_CTRL_PID_PID_CUSTOM_RESETVAL                              (0x00000000U)
#define CSL_MSS_CTRL_PID_PID_CUSTOM_MAX                                   (0x00000003U)

#define CSL_MSS_CTRL_PID_PID_MAJOR_MASK                                   (0x00000700U)
#define CSL_MSS_CTRL_PID_PID_MAJOR_SHIFT                                  (0x00000008U)
#define CSL_MSS_CTRL_PID_PID_MAJOR_RESETVAL                               (0x00000002U)
#define CSL_MSS_CTRL_PID_PID_MAJOR_MAX                                    (0x00000007U)

#define CSL_MSS_CTRL_PID_PID_MISC_MASK                                    (0x0000F800U)
#define CSL_MSS_CTRL_PID_PID_MISC_SHIFT                                   (0x0000000BU)
#define CSL_MSS_CTRL_PID_PID_MISC_RESETVAL                                (0x00000000U)
#define CSL_MSS_CTRL_PID_PID_MISC_MAX                                     (0x0000001FU)

#define CSL_MSS_CTRL_PID_PID_MSB16_MASK                                   (0xFFFF0000U)
#define CSL_MSS_CTRL_PID_PID_MSB16_SHIFT                                  (0x00000010U)
#define CSL_MSS_CTRL_PID_PID_MSB16_RESETVAL                               (0x00006180U)
#define CSL_MSS_CTRL_PID_PID_MSB16_MAX                                    (0x0000FFFFU)

#define CSL_MSS_CTRL_PID_RESETVAL                                         (0x61800215U)

/* R5SS0_CONTROL */

#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_MASK                         (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_RESETVAL                     (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_MAX                          (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SWITCH_WAIT_MASK             (0x00000700U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SWITCH_WAIT_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SWITCH_WAIT_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SWITCH_WAIT_MAX              (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CONTROL_RESET_FSM_TRIGGER_MASK                 (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_RESET_FSM_TRIGGER_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_RESET_FSM_TRIGGER_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_RESET_FSM_TRIGGER_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CONTROL_ROM_WAIT_STATE_MASK                    (0x07000000U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_ROM_WAIT_STATE_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_ROM_WAIT_STATE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CONTROL_ROM_WAIT_STATE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CONTROL_RESETVAL                               (0x00000707U)

/* R5SS0_FORCE_WFI */

#define CSL_MSS_CTRL_R5SS0_FORCE_WFI_CR5_WFI_OVERIDE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_FORCE_WFI_CR5_WFI_OVERIDE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_FORCE_WFI_CR5_WFI_OVERIDE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_FORCE_WFI_CR5_WFI_OVERIDE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_FORCE_WFI_RESETVAL                             (0x00000000U)

/* R5SS0_CORE0_HALT */

#define CSL_MSS_CTRL_R5SS0_CORE0_HALT_HALT_MASK                           (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_HALT_HALT_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_HALT_HALT_RESETVAL                       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_HALT_HALT_MAX                            (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE0_HALT_RESETVAL                            (0x00000007U)

/* R5SS0_CORE1_HALT */

#define CSL_MSS_CTRL_R5SS0_CORE1_HALT_HALT_MASK                           (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_HALT_HALT_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_HALT_HALT_RESETVAL                       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_HALT_HALT_MAX                            (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE1_HALT_RESETVAL                            (0x00000007U)

/* R5SS0_STATUS_REG */

#define CSL_MSS_CTRL_R5SS0_STATUS_REG_MEMSWAP_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG_MEMSWAP_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG_MEMSWAP_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG_MEMSWAP_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_STATUS_REG_LOCK_STEP_MASK                      (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG_LOCK_STEP_SHIFT                     (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG_LOCK_STEP_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_STATUS_REG_LOCK_STEP_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_STATUS_REG_RESETVAL                            (0x00000000U)

/* R5SS0_CORE0_STAT */

#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFI_STAT_MASK                       (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFI_STAT_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFI_STAT_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFI_STAT_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFE_STAT_MASK                       (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFE_STAT_SHIFT                      (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFE_STAT_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_WFE_STAT_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_STAT_RESETVAL                            (0x00000000U)

/* R5SS0_CORE1_STAT */

#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFI_STAT_MASK                       (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFI_STAT_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFI_STAT_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFI_STAT_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFE_STAT_MASK                       (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFE_STAT_SHIFT                      (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFE_STAT_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_WFE_STAT_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_STAT_RESETVAL                            (0x00000000U)

/* R5SS0_INIT_TCM */

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU0_MASK                        (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU0_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU0_RESETVAL                    (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU0_MAX                         (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU0_MASK                        (0x00000070U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU0_SHIFT                       (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU0_RESETVAL                    (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU0_MAX                         (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU0_MASK                    (0x00000700U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU0_SHIFT                   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU0_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU0_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU1_MASK                        (0x00007000U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU1_SHIFT                       (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU1_RESETVAL                    (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMA_CPU1_MAX                         (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU1_MASK                        (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU1_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU1_RESETVAL                    (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_TCMB_CPU1_MAX                         (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU1_MASK                    (0x00700000U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU1_SHIFT                   (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU1_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_INIT_TCM_LOCKZRAM_CPU1_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_INIT_TCM_RESETVAL                              (0x00777777U)

/* R5SS0_AHB_EN */

#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU0_AHB_INIT_MASK                      (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU0_AHB_INIT_SHIFT                     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU0_AHB_INIT_RESETVAL                  (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU0_AHB_INIT_MAX                       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU1_AHB_INIT_MASK                      (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU1_AHB_INIT_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU1_AHB_INIT_RESETVAL                  (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_AHB_EN_CPU1_AHB_INIT_MAX                       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_AHB_EN_RESETVAL                                (0x00070007U)

/* R5SS0_TCM_EXT_ERR_EN */

#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU0_TCM_MASK                   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU0_TCM_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU0_TCM_RESETVAL               (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU0_TCM_MAX                    (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU1_TCM_MASK                   (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU1_TCM_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU1_TCM_RESETVAL               (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_CPU1_TCM_MAX                    (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_EXT_ERR_EN_RESETVAL                        (0x00070007U)

/* R5SS0_TCM_ERR_EN */

#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU0_TCM_MASK                       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU0_TCM_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU0_TCM_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU0_TCM_MAX                        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU1_TCM_MASK                       (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU1_TCM_SHIFT                      (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU1_TCM_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_CPU1_TCM_MAX                        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ERR_EN_RESETVAL                            (0x00000000U)

/* R5SS0_TCM_ECC_WRENZ_EN */

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_MASK       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_RESETVAL   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_MAX        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_MASK      (0x00000070U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_SHIFT     (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_RESETVAL  (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_MASK      (0x00000700U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_SHIFT     (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_RESETVAL  (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_MASK       (0x00007000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_SHIFT      (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_RESETVAL   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_MAX        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_MASK      (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_SHIFT     (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_RESETVAL  (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_MASK      (0x00700000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_SHIFT     (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_RESETVAL  (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ECC_WRENZ_EN_RESETVAL                      (0x00777777U)

/* R5SS0_CORE0_AHB_BASE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BASE_AHB_BASE_MASK                   (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BASE_AHB_BASE_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BASE_AHB_BASE_RESETVAL               (0x00050000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BASE_AHB_BASE_MAX                    (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BASE_RESETVAL                        (0x00050000U)

/* R5SS0_CORE0_AHB_SIZE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE_AHB_SIZE_MASK                   (0x0000001FU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE_AHB_SIZE_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE_AHB_SIZE_RESETVAL               (0x00000013U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE_AHB_SIZE_MAX                    (0x0000001FU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_SIZE_RESETVAL                        (0x00000013U)

/* R5SS0_CORE1_AHB_BASE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BASE_AHB_BASE_MASK                   (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BASE_AHB_BASE_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BASE_AHB_BASE_RESETVAL               (0x00050000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BASE_AHB_BASE_MAX                    (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BASE_RESETVAL                        (0x00050000U)

/* R5SS0_CORE1_AHB_SIZE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE_AHB_SIZE_MASK                   (0x0000001FU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE_AHB_SIZE_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE_AHB_SIZE_RESETVAL               (0x00000013U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE_AHB_SIZE_MAX                    (0x0000001FU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_SIZE_RESETVAL                        (0x00000013U)

/* R5SS0_TEINIT */

#define CSL_MSS_CTRL_R5SS0_TEINIT_TEINIT_MASK                             (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_TEINIT_TEINIT_SHIFT                            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TEINIT_TEINIT_RESETVAL                         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TEINIT_TEINIT_MAX                              (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_TEINIT_RESETVAL                                (0x00000000U)

/* BOOT_INFO_REG0 */

#define CSL_MSS_CTRL_BOOT_INFO_REG0_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG0_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG0_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG0_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG0_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG1 */

#define CSL_MSS_CTRL_BOOT_INFO_REG1_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG1_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG1_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG1_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG1_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG2 */

#define CSL_MSS_CTRL_BOOT_INFO_REG2_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG2_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG2_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG2_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG2_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG3 */

#define CSL_MSS_CTRL_BOOT_INFO_REG3_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG3_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG3_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG3_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG3_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG4 */

#define CSL_MSS_CTRL_BOOT_INFO_REG4_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG4_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG4_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG4_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG4_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG5 */

#define CSL_MSS_CTRL_BOOT_INFO_REG5_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG5_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG5_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG5_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG5_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG6 */

#define CSL_MSS_CTRL_BOOT_INFO_REG6_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG6_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG6_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG6_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG6_RESETVAL                              (0x00000000U)

/* BOOT_INFO_REG7 */

#define CSL_MSS_CTRL_BOOT_INFO_REG7_CONFIG_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_BOOT_INFO_REG7_CONFIG_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG7_CONFIG_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_BOOT_INFO_REG7_CONFIG_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_BOOT_INFO_REG7_RESETVAL                              (0x00000000U)

/* R5SS0_ATCM_MEM_INIT */

#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_MEM_INIT_MASK                    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_MEM_INIT_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_MEM_INIT_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_MEM_INIT_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_RESETVAL                         (0x00000000U)

/* R5SS0_ATCM_MEM_INIT_DONE */

#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_MASK          (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_RESETVAL                    (0x00000000U)

/* R5SS0_ATCM_MEM_INIT_STATUS */

#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_MEM_STATUS_MASK           (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_MEM_STATUS_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_MEM_STATUS_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_MEM_STATUS_MAX            (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_RESETVAL                  (0x00000000U)

/* R5SS0_BTCM_MEM_INIT */

#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_MEM_INIT_MASK                    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_MEM_INIT_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_MEM_INIT_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_MEM_INIT_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_RESETVAL                         (0x00000000U)

/* R5SS0_BTCM_MEM_INIT_DONE */

#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_MASK          (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_RESETVAL                    (0x00000000U)

/* R5SS0_BTCM_MEM_INIT_STATUS */

#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_MEM_STATUS_MASK           (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_MEM_STATUS_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_MEM_STATUS_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_MEM_STATUS_MAX            (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_RESETVAL                  (0x00000000U)

/* L2IOCRAM_MEM_INIT */

#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION0_MASK                    (0x00000001U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION0_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION0_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION0_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION1_MASK                    (0x00000002U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION1_SHIFT                   (0x00000001U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION1_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION1_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION2_MASK                    (0x00000004U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION2_SHIFT                   (0x00000002U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION2_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION2_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_L2IOCRAM_MEM_INIT_RESETVAL                           (0x00000000U)

/* L2OCRAM_MEM_INIT_DONE */

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION1_MASK                (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION1_SHIFT               (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION2_MASK                (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION2_SHIFT               (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_DONE_RESETVAL                       (0x00000000U)

/* L2OCRAM_MEM_INIT_STATUS */

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION1_MASK              (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION1_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION2_MASK              (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION2_SHIFT             (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_PARTITION2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_MEM_INIT_STATUS_RESETVAL                     (0x00000000U)

/* MAILBOXRAM_MEM_INIT */

#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_MEM0_INIT_MASK                   (0x00000001U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_MEM0_INIT_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_MEM0_INIT_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_MEM0_INIT_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_RESETVAL                         (0x00000000U)

/* MAILBOXRAM_MEM_INIT_DONE */

#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE_MEM0_DONE_MASK              (0x00000001U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE_MEM0_DONE_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE_MEM0_DONE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE_MEM0_DONE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE_RESETVAL                    (0x00000000U)

/* MAILBOXRAM_MEM_INIT_STATUS */

#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_STATUS_MEM0_STATUS_MASK          (0x00000001U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_STATUS_MEM0_STATUS_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_STATUS_MEM0_STATUS_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_STATUS_MEM0_STATUS_MAX           (0x00000001U)

#define CSL_MSS_CTRL_MAILBOXRAM_MEM_INIT_STATUS_RESETVAL                  (0x00000000U)

/* TPCC_MEM_INIT */

#define CSL_MSS_CTRL_TPCC_MEM_INIT_TPCC_A_MEMINIT_START_MASK              (0x00000001U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_TPCC_A_MEMINIT_START_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_TPCC_A_MEMINIT_START_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_TPCC_A_MEMINIT_START_MAX               (0x00000001U)

#define CSL_MSS_CTRL_TPCC_MEM_INIT_RESETVAL                               (0x00000000U)

/* TPCC_MEM_INIT_DONE */

#define CSL_MSS_CTRL_TPCC_MEM_INIT_DONE_TPCC_A_MEMINIT_DONE_MASK          (0x00000001U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_DONE_TPCC_A_MEMINIT_DONE_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_DONE_TPCC_A_MEMINIT_DONE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPCC_MEM_INIT_DONE_TPCC_A_MEMINIT_DONE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_TPCC_MEM_INIT_DONE_RESETVAL                          (0x00000000U)

/* TPCC_MEMINIT_STATUS */

#define CSL_MSS_CTRL_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MASK       (0x00000001U)
#define CSL_MSS_CTRL_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MAX        (0x00000001U)

#define CSL_MSS_CTRL_TPCC_MEMINIT_STATUS_RESETVAL                         (0x00000000U)

/* TOP_PBIST_KEY_RST */

#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_KEY_MASK                  (0x0000000FU)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_KEY_SHIFT                 (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_KEY_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_KEY_MAX                   (0x0000000FU)

#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_RST_MASK                  (0x000000F0U)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_RST_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_RST_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_PBIST_ST_RST_MAX                   (0x0000000FU)

#define CSL_MSS_CTRL_TOP_PBIST_KEY_RST_RESETVAL                           (0x00000000U)

/* TOP_PBIST_REG0 */

#define CSL_MSS_CTRL_TOP_PBIST_REG0_PBIST_REG_MASK                        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TOP_PBIST_REG0_PBIST_REG_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_REG0_PBIST_REG_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_REG0_PBIST_REG_MAX                         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TOP_PBIST_REG0_RESETVAL                              (0x00000000U)

/* TOP_PBIST_REG1 */

#define CSL_MSS_CTRL_TOP_PBIST_REG1_PBIST_REG_MASK                        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TOP_PBIST_REG1_PBIST_REG_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_REG1_PBIST_REG_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_REG1_PBIST_REG_MAX                         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TOP_PBIST_REG1_RESETVAL                              (0x00000000U)

/* TOP_PBIST_REG2 */

#define CSL_MSS_CTRL_TOP_PBIST_REG2_PBIST_REG_MASK                        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TOP_PBIST_REG2_PBIST_REG_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_REG2_PBIST_REG_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_TOP_PBIST_REG2_PBIST_REG_MAX                         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TOP_PBIST_REG2_RESETVAL                              (0x00000000U)

/* R5SS0_CTI_TRIG_SEL */

#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG0_MASK                        (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG0_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG0_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG0_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG1_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG1_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG1_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_TRIG1_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CTI_TRIG_SEL_RESETVAL                          (0x00000000U)

/* DBGSS_CTI_TRIG_SEL */

#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG0_MASK                        (0x000000FFU)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG0_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG0_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG0_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG1_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG1_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG1_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG1_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG2_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG2_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG2_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG2_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG3_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG3_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG3_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_TRIG3_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_DBGSS_CTI_TRIG_SEL_RESETVAL                          (0x00000000U)

/* MCAN0_HALTEN */

#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5A0_HALTEN_MASK                       (0x00000001U)
#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5A0_HALTEN_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5A0_HALTEN_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5A0_HALTEN_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5B0_HALTEN_MASK                       (0x00000002U)
#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5B0_HALTEN_SHIFT                      (0x00000001U)
#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5B0_HALTEN_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MCAN0_HALTEN_CR5B0_HALTEN_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_MCAN0_HALTEN_RESETVAL                                (0x00000000U)

/* MCAN1_HALTEN */

#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5A0_HALTEN_MASK                       (0x00000001U)
#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5A0_HALTEN_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5A0_HALTEN_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5A0_HALTEN_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5B0_HALTEN_MASK                       (0x00000002U)
#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5B0_HALTEN_SHIFT                      (0x00000001U)
#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5B0_HALTEN_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MCAN1_HALTEN_CR5B0_HALTEN_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_MCAN1_HALTEN_RESETVAL                                (0x00000000U)

/* LIN0_HALTEN */

#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A1_HALTEN_MASK                        (0x00000004U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A1_HALTEN_SHIFT                       (0x00000002U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A1_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5A1_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B1_HALTEN_MASK                        (0x00000008U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B1_HALTEN_SHIFT                       (0x00000003U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B1_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN0_HALTEN_CR5B1_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN0_HALTEN_RESETVAL                                 (0x00000000U)

/* LIN1_HALTEN */

#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A1_HALTEN_MASK                        (0x00000004U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A1_HALTEN_SHIFT                       (0x00000002U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A1_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5A1_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B1_HALTEN_MASK                        (0x00000008U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B1_HALTEN_SHIFT                       (0x00000003U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B1_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN1_HALTEN_CR5B1_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN1_HALTEN_RESETVAL                                 (0x00000000U)

/* LIN2_HALTEN */

#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A1_HALTEN_MASK                        (0x00000004U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A1_HALTEN_SHIFT                       (0x00000002U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A1_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5A1_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B1_HALTEN_MASK                        (0x00000008U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B1_HALTEN_SHIFT                       (0x00000003U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B1_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_LIN2_HALTEN_CR5B1_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_LIN2_HALTEN_RESETVAL                                 (0x00000000U)

/* I2C0_HALTEN */

#define CSL_MSS_CTRL_I2C0_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_I2C0_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_I2C0_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_I2C0_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_I2C0_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_I2C0_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_I2C0_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_I2C0_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_I2C0_HALTEN_RESETVAL                                 (0x00000000U)

/* I2C1_HALTEN */

#define CSL_MSS_CTRL_I2C1_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_I2C1_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_I2C1_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_I2C1_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_I2C1_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_I2C1_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_I2C1_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_I2C1_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_I2C1_HALTEN_RESETVAL                                 (0x00000000U)

/* I2C2_HALTEN */

#define CSL_MSS_CTRL_I2C2_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_I2C2_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_I2C2_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_I2C2_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_I2C2_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_I2C2_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_I2C2_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_I2C2_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_I2C2_HALTEN_RESETVAL                                 (0x00000000U)

/* RTI0_HALTEN */

#define CSL_MSS_CTRL_RTI0_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_RTI0_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_RTI0_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI0_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI0_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_RTI0_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_RTI0_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI0_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI0_HALTEN_RESETVAL                                 (0x00000000U)

/* RTI1_HALTEN */

#define CSL_MSS_CTRL_RTI1_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_RTI1_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_RTI1_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI1_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI1_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_RTI1_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_RTI1_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI1_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI1_HALTEN_RESETVAL                                 (0x00000000U)

/* RTI2_HALTEN */

#define CSL_MSS_CTRL_RTI2_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_RTI2_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_RTI2_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI2_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI2_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_RTI2_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_RTI2_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI2_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI2_HALTEN_RESETVAL                                 (0x00000000U)

/* RTI3_HALTEN */

#define CSL_MSS_CTRL_RTI3_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_RTI3_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_RTI3_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI3_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI3_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_RTI3_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_RTI3_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_RTI3_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_RTI3_HALTEN_RESETVAL                                 (0x00000000U)

/* CPSW_HALTEN */

#define CSL_MSS_CTRL_CPSW_HALTEN_CR5A0_HALTEN_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_CPSW_HALTEN_CR5A0_HALTEN_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_CPSW_HALTEN_CR5A0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_CPSW_HALTEN_CR5A0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_CPSW_HALTEN_CR5B0_HALTEN_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_CPSW_HALTEN_CR5B0_HALTEN_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_CPSW_HALTEN_CR5B0_HALTEN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_CPSW_HALTEN_CR5B0_HALTEN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_CPSW_HALTEN_RESETVAL                                 (0x00000000U)

/* MCRC0_HALTEN */

#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5A0_HALTEN_MASK                       (0x00000001U)
#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5A0_HALTEN_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5A0_HALTEN_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5A0_HALTEN_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5B0_HALTEN_MASK                       (0x00000002U)
#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5B0_HALTEN_SHIFT                      (0x00000001U)
#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5B0_HALTEN_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_HALTEN_CR5B0_HALTEN_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_HALTEN_RESETVAL                                (0x00000000U)

/* ICSSM0_PRU0_GPI_SEL */

#define CSL_MSS_CTRL_ICSSM0_PRU0_GPI_SEL_SEL_MASK                         (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPI_SEL_SEL_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPI_SEL_SEL_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPI_SEL_SEL_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PRU0_GPI_SEL_RESETVAL                         (0x00000000U)

/* ICSSM0_PRU1_GPI_SEL */

#define CSL_MSS_CTRL_ICSSM0_PRU1_GPI_SEL_SEL_MASK                         (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPI_SEL_SEL_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPI_SEL_SEL_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPI_SEL_SEL_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PRU1_GPI_SEL_RESETVAL                         (0x00000000U)

/* ICSSM1_PRU0_GPI_SEL */

#define CSL_MSS_CTRL_ICSSM1_PRU0_GPI_SEL_SEL_MASK                         (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPI_SEL_SEL_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPI_SEL_SEL_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPI_SEL_SEL_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PRU0_GPI_SEL_RESETVAL                         (0x00000000U)

/* ICSSM1_PRU1_GPI_SEL */

#define CSL_MSS_CTRL_ICSSM1_PRU1_GPI_SEL_SEL_MASK                         (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPI_SEL_SEL_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPI_SEL_SEL_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPI_SEL_SEL_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PRU1_GPI_SEL_RESETVAL                         (0x00000000U)

/* ICSSM0_PRU0_GPIO_OUT_CTRL */

#define CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL_OUTDISABLE_MASK            (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL_OUTDISABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL_OUTDISABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL_OUTDISABLE_MAX             (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL_RESETVAL                   (0x00000000U)

/* ICSSM0_PRU1_GPIO_OUT_CTRL */

#define CSL_MSS_CTRL_ICSSM0_PRU1_GPIO_OUT_CTRL_OUTDISABLE_MASK            (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPIO_OUT_CTRL_OUTDISABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPIO_OUT_CTRL_OUTDISABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_GPIO_OUT_CTRL_OUTDISABLE_MAX             (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PRU1_GPIO_OUT_CTRL_RESETVAL                   (0x00000000U)

/* ICSSM1_PRU0_GPIO_OUT_CTRL */

#define CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL_OUTDISABLE_MASK            (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL_OUTDISABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL_OUTDISABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL_OUTDISABLE_MAX             (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL_RESETVAL                   (0x00000000U)

/* ICSSM1_PRU1_GPIO_OUT_CTRL */

#define CSL_MSS_CTRL_ICSSM1_PRU1_GPIO_OUT_CTRL_OUTDISABLE_MASK            (0x3FFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPIO_OUT_CTRL_OUTDISABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPIO_OUT_CTRL_OUTDISABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_GPIO_OUT_CTRL_OUTDISABLE_MAX             (0x3FFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PRU1_GPIO_OUT_CTRL_RESETVAL                   (0x00000000U)

/* ICSSM0_RX_ERR_COUNTER */

#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII0_RXERR_CNT_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII0_RXERR_CNT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII0_RXERR_CNT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII0_RXERR_CNT_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII1_RXERR_CNT_MASK            (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII1_RXERR_CNT_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII1_RXERR_CNT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_MII1_RXERR_CNT_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_RESETVAL                       (0x00000000U)

/* ICSSM0_RX_ERR_COUNTER_CLR */

#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_MASK    (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_MASK    (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_SHIFT   (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_RX_ERR_COUNTER_CLR_RESETVAL                   (0x00000000U)

/* ICSSM1_RX_ERR_COUNTER */

#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII0_RXERR_CNT_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII0_RXERR_CNT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII0_RXERR_CNT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII0_RXERR_CNT_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII1_RXERR_CNT_MASK            (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII1_RXERR_CNT_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII1_RXERR_CNT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_MII1_RXERR_CNT_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_RESETVAL                       (0x00000000U)

/* ICSSM1_RX_ERR_COUNTER_CLR */

#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_MASK    (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII0_RXERR_WRT1CLR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_MASK    (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_SHIFT   (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_MII1_RXERR_WRT1CLR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_RX_ERR_COUNTER_CLR_RESETVAL                   (0x00000000U)

/* TPTC_BOUNDARY_CFG */

#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MASK                  (0x0000003FU)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_SHIFT                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_RESETVAL              (0x00000013U)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MAX                   (0x0000003FU)

#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_MASK                  (0x00003F00U)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_SHIFT                 (0x00000008U)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_RESETVAL              (0x00000013U)
#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_MAX                   (0x0000003FU)

#define CSL_MSS_CTRL_TPTC_BOUNDARY_CFG_RESETVAL                           (0x00001313U)

/* TPTC_XID_REORDER_CFG */

#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MASK            (0x00000001U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MAX             (0x00000001U)

#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_MASK            (0x00000100U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_MAX             (0x00000001U)

#define CSL_MSS_CTRL_TPTC_XID_REORDER_CFG_RESETVAL                        (0x00000000U)

/* TPTC_DBS_CONFIG */

#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A0_MASK                         (0x00000003U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A0_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A0_RESETVAL                     (0x00000001U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A0_MAX                          (0x00000003U)

#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A1_MASK                         (0x00000030U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A1_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A1_RESETVAL                     (0x00000001U)
#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_A1_MAX                          (0x00000003U)

#define CSL_MSS_CTRL_TPTC_DBS_CONFIG_RESETVAL                             (0x00000011U)

/* OSPI_CONFIG */

#define CSL_MSS_CTRL_OSPI_CONFIG_EXT_CLK_MASK                             (0x00000007U)
#define CSL_MSS_CTRL_OSPI_CONFIG_EXT_CLK_SHIFT                            (0x00000000U)
#define CSL_MSS_CTRL_OSPI_CONFIG_EXT_CLK_RESETVAL                         (0x00000000U)
#define CSL_MSS_CTRL_OSPI_CONFIG_EXT_CLK_MAX                              (0x00000007U)

#define CSL_MSS_CTRL_OSPI_CONFIG_ICLK_SEL_MASK                            (0x00000070U)
#define CSL_MSS_CTRL_OSPI_CONFIG_ICLK_SEL_SHIFT                           (0x00000004U)
#define CSL_MSS_CTRL_OSPI_CONFIG_ICLK_SEL_RESETVAL                        (0x00000000U)
#define CSL_MSS_CTRL_OSPI_CONFIG_ICLK_SEL_MAX                             (0x00000007U)

#define CSL_MSS_CTRL_OSPI_CONFIG_RTXIP_PENDING_MASK                       (0x00007000U)
#define CSL_MSS_CTRL_OSPI_CONFIG_RTXIP_PENDING_SHIFT                      (0x0000000CU)
#define CSL_MSS_CTRL_OSPI_CONFIG_RTXIP_PENDING_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_OSPI_CONFIG_RTXIP_PENDING_MAX                        (0x00000007U)

#define CSL_MSS_CTRL_OSPI_CONFIG_RESETVAL                                 (0x00000000U)

/* OSPI_BOOT_CONFIG_MASK */

#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_MASK_BOOT_SIZE_MASK                 (0x000FFFFFU)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_MASK_BOOT_SIZE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_MASK_BOOT_SIZE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_MASK_BOOT_SIZE_MAX                  (0x000FFFFFU)

#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_MASK_RESETVAL                       (0x00000000U)

/* OSPI_BOOT_CONFIG_SEG */

#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_SEG_BOOT_SEG_MASK                   (0x000FFFFFU)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_SEG_BOOT_SEG_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_SEG_BOOT_SEG_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_SEG_BOOT_SEG_MAX                    (0x000FFFFFU)

#define CSL_MSS_CTRL_OSPI_BOOT_CONFIG_SEG_RESETVAL                        (0x00000000U)

/* TPCC0_INTAGG_MASK */

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INTG_MASK                   (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INTG_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INTG_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INTG_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT0_MASK                   (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT0_SHIFT                  (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT0_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT0_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT1_MASK                   (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT1_SHIFT                  (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT1_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT1_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT2_MASK                   (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT2_SHIFT                  (0x00000003U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT2_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT2_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT3_MASK                   (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT3_SHIFT                  (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT3_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT3_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT4_MASK                   (0x00000020U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT4_SHIFT                  (0x00000005U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT4_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT4_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT5_MASK                   (0x00000040U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT5_SHIFT                  (0x00000006U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT5_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT5_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT6_MASK                   (0x00000080U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT6_SHIFT                  (0x00000007U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT6_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT6_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT7_MASK                   (0x00000100U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT7_SHIFT                  (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT7_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPCC_A_INT7_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A0_MASK                       (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A0_SHIFT                      (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A0_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A0_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A1_MASK                       (0x00020000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A1_SHIFT                      (0x00000011U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A1_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_TPTC_A1_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* TPCC0_INTAGG_STATUS */

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INTG_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INTG_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INTG_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INTG_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT0_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT0_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT0_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT0_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT1_MASK                 (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT1_SHIFT                (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT1_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT1_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT2_MASK                 (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT2_SHIFT                (0x00000003U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT2_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT2_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT3_MASK                 (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT3_SHIFT                (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT3_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT3_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT4_MASK                 (0x00000020U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT4_SHIFT                (0x00000005U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT4_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT4_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT5_MASK                 (0x00000040U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT5_SHIFT                (0x00000006U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT5_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT5_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT6_MASK                 (0x00000080U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT6_SHIFT                (0x00000007U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT6_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT6_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT7_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT7_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT7_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPCC_A_INT7_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A0_MASK                     (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A0_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A0_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A0_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A1_MASK                     (0x00020000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A1_SHIFT                    (0x00000011U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A1_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_TPTC_A1_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* TPCC0_INTAGG_STATUS_RAW */

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INTG_MASK             (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INTG_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INTG_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INTG_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT0_MASK             (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT0_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT0_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT0_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT1_MASK             (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT1_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT1_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT1_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT2_MASK             (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT2_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT2_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT2_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT3_MASK             (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT3_SHIFT            (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT3_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT3_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT4_MASK             (0x00000020U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT4_SHIFT            (0x00000005U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT4_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT4_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT5_MASK             (0x00000040U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT5_SHIFT            (0x00000006U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT5_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT5_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT6_MASK             (0x00000080U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT6_SHIFT            (0x00000007U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT6_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT6_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT7_MASK             (0x00000100U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT7_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT7_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPCC_A_INT7_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A0_MASK                 (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A0_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A0_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A0_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A1_MASK                 (0x00020000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A1_SHIFT                (0x00000011U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A1_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_TPTC_A1_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* ICSSM_IDLE_CONTROL */

#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM0_NOGATE_MASK                (0x00000001U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM0_NOGATE_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM0_NOGATE_RESETVAL            (0x00000001U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM0_NOGATE_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM1_NOGATE_MASK                (0x00000002U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM1_NOGATE_SHIFT               (0x00000001U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM1_NOGATE_RESETVAL            (0x00000001U)
#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_ICSSM1_NOGATE_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM_IDLE_CONTROL_RESETVAL                          (0x00000003U)

/* GPMC_CONTROL */

#define CSL_MSS_CTRL_GPMC_CONTROL_CLKOUT_SEL_MASK                         (0x00000001U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLKOUT_SEL_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLKOUT_SEL_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLKOUT_SEL_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_SEL_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_SEL_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_SEL_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_SEL_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_OE_N_MASK                           (0x00000100U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_OE_N_SHIFT                          (0x00000008U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_OE_N_RESETVAL                       (0x00000001U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_OE_N_MAX                            (0x00000001U)

#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_OE_N_MASK                        (0x00001000U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_OE_N_SHIFT                       (0x0000000CU)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_OE_N_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_GPMC_CONTROL_CLK_LB_OE_N_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_GPMC_CONTROL_RESETVAL                                (0x00000100U)

/* INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL */

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G0_DYNAMIC_CLK_GATE_EN_MASK (0x00000007U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G0_DYNAMIC_CLK_GATE_EN_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G0_DYNAMIC_CLK_GATE_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G0_DYNAMIC_CLK_GATE_EN_MAX (0x00000007U)

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G1_DYNAMIC_CLK_GATE_EN_MASK (0x00000070U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G1_DYNAMIC_CLK_GATE_EN_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G1_DYNAMIC_CLK_GATE_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_INFRA_G1_DYNAMIC_CLK_GATE_EN_MAX (0x00000007U)

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_PERI_DYNAMIC_CLK_GATE_EN_MASK (0x00000700U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_PERI_DYNAMIC_CLK_GATE_EN_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_PERI_DYNAMIC_CLK_GATE_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_PERI_DYNAMIC_CLK_GATE_EN_MAX (0x00000007U)

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG0_DYNAMIC_CLK_GATE_EN_MASK (0x00007000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG0_DYNAMIC_CLK_GATE_EN_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG0_DYNAMIC_CLK_GATE_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG0_DYNAMIC_CLK_GATE_EN_MAX (0x00000007U)

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG1_DYNAMIC_CLK_GATE_EN_MASK (0x00070000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG1_DYNAMIC_CLK_GATE_EN_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG1_DYNAMIC_CLK_GATE_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_CONFIG1_DYNAMIC_CLK_GATE_EN_MAX (0x00000007U)

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_PERIPH_DYNAMIC_CLK_GATE_EN_MASK (0x00700000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_PERIPH_DYNAMIC_CLK_GATE_EN_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_PERIPH_DYNAMIC_CLK_GATE_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_MISC_PERIPH_DYNAMIC_CLK_GATE_EN_MAX (0x00000007U)

#define CSL_MSS_CTRL_INTERCONNECT_CLK_GATE_DYNAMIC_CONTROL_RESETVAL       (0x00000000U)

/* CPSW_CONTROL */

#define CSL_MSS_CTRL_CPSW_CONTROL_PORT1_MODE_SEL_MASK                     (0x00000007U)
#define CSL_MSS_CTRL_CPSW_CONTROL_PORT1_MODE_SEL_SHIFT                    (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_PORT1_MODE_SEL_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_PORT1_MODE_SEL_MAX                      (0x00000007U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_OE_N_MASK                 (0x00000010U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_OE_N_SHIFT                (0x00000004U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_OE_N_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_OE_N_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_SEL_MASK                  (0x00000040U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_SEL_SHIFT                 (0x00000006U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_SEL_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII1_REF_CLK_SEL_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII1_ID_MODE_MASK                     (0x00000100U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII1_ID_MODE_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII1_ID_MODE_RESETVAL                 (0x00000001U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII1_ID_MODE_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_CPSW_CONTROL_PORT2_MODE_SEL_MASK                     (0x00070000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_PORT2_MODE_SEL_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_CPSW_CONTROL_PORT2_MODE_SEL_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_PORT2_MODE_SEL_MAX                      (0x00000007U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_OE_N_MASK                 (0x00100000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_OE_N_SHIFT                (0x00000014U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_OE_N_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_OE_N_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_SEL_MASK                  (0x00400000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_SEL_SHIFT                 (0x00000016U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_SEL_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RMII2_REF_CLK_SEL_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII2_ID_MODE_MASK                     (0x01000000U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII2_ID_MODE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII2_ID_MODE_RESETVAL                 (0x00000001U)
#define CSL_MSS_CTRL_CPSW_CONTROL_RGMII2_ID_MODE_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_CPSW_CONTROL_RESETVAL                                (0x01000100U)

/* ICSSM1_INPUT_INTR_SEL */

#define CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL_SEL_MASK                       (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL_SEL_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL_SEL_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL_SEL_MAX                        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL_RESETVAL                       (0x00000000U)

/* ICSSM_GPO_SEL */

#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU0_SEL_MASK                          (0x00000001U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU0_SEL_SHIFT                         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU0_SEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU0_SEL_MAX                           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU1_SEL_MASK                          (0x00000002U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU1_SEL_SHIFT                         (0x00000001U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU1_SEL_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM_GPO_SEL_PRU1_SEL_MAX                           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM_GPO_SEL_RESETVAL                               (0x00000000U)

/* DEBUGSS_CSETB_FLUSH */

#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_MASK               (0x00000001U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_MAX                (0x00000001U)

#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_MASK            (0x00000100U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_MAX             (0x00000001U)

#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_MASK          (0x00000200U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_SHIFT         (0x00000009U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FULL_MASK                  (0x00000400U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FULL_SHIFT                 (0x0000000AU)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FULL_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_CSETB_FULL_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_RESETVAL                         (0x00000000U)

/* DEBUGSS_STM_NSGUAREN */

#define CSL_MSS_CTRL_DEBUGSS_STM_NSGUAREN_ENABLE_MASK                     (0x00000400U)
#define CSL_MSS_CTRL_DEBUGSS_STM_NSGUAREN_ENABLE_SHIFT                    (0x0000000AU)
#define CSL_MSS_CTRL_DEBUGSS_STM_NSGUAREN_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_DEBUGSS_STM_NSGUAREN_ENABLE_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_DEBUGSS_STM_NSGUAREN_RESETVAL                        (0x00000000U)

/* CTRL_USB_CTRL */

#define CSL_MSS_CTRL_CTRL_USB_CTRL_CM_PWRDN_MASK                          (0x00000001U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL_CM_PWRDN_SHIFT                         (0x00000000U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL_CM_PWRDN_RESETVAL                      (0x00000001U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL_CM_PWRDN_MAX                           (0x00000001U)

#define CSL_MSS_CTRL_CTRL_USB_CTRL_USB_WUEN_MASK                          (0x00000004U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL_USB_WUEN_SHIFT                         (0x00000002U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL_USB_WUEN_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_CTRL_USB_CTRL_USB_WUEN_MAX                           (0x00000001U)

#define CSL_MSS_CTRL_CTRL_USB_CTRL_RESETVAL                               (0x00000001U)

/* CTRL_USB_STS */

#define CSL_MSS_CTRL_CTRL_USB_STS_WUEVT_MASK                              (0x00000001U)
#define CSL_MSS_CTRL_CTRL_USB_STS_WUEVT_SHIFT                             (0x00000000U)
#define CSL_MSS_CTRL_CTRL_USB_STS_WUEVT_RESETVAL                          (0x00000000U)
#define CSL_MSS_CTRL_CTRL_USB_STS_WUEVT_MAX                               (0x00000001U)

#define CSL_MSS_CTRL_CTRL_USB_STS_RESETVAL                                (0x00000000U)

/* USB_SLAVE_CONTROL */

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEREQ_MASK          (0x00000001U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEREQ_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEREQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEREQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEACK_MASK          (0x00000006U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEACK_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEACK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_SIDLEACK_MAX           (0x00000003U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0P_SWAKEUP_MASK           (0x00000008U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0P_SWAKEUP_SHIFT          (0x00000003U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0P_SWAKEUP_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0P_SWAKEUP_MAX            (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_FCLKEN_MASK            (0x00000010U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_FCLKEN_SHIFT           (0x00000004U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_FCLKEN_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV0C_FCLKEN_MAX             (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEREQ_MASK          (0x00000020U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEREQ_SHIFT         (0x00000005U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEREQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEREQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEACK_MASK          (0x000000C0U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEACK_SHIFT         (0x00000006U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEACK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_SLV1C_SIDLEACK_MAX           (0x00000003U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_CTRL_MISC_PWR_ISO_MASK       (0x00000100U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_CTRL_MISC_PWR_ISO_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_CTRL_MISC_PWR_ISO_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_CTRL_MISC_PWR_ISO_MAX        (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKIN_MASK      (0x00000200U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKIN_SHIFT     (0x00000009U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKIN_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKIN_MAX       (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKOUT_MASK     (0x00000400U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKOUT_SHIFT    (0x0000000AU)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKOUT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUCLKOUT_MAX      (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUOUT_MASK        (0x00000800U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUOUT_SHIFT       (0x0000000BU)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUOUT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_USBSS_PHY_WAKEUP_WUOUT_MAX         (0x00000001U)

#define CSL_MSS_CTRL_USB_SLAVE_CONTROL_RESETVAL                           (0x00000000U)

/* USB_MASTER_STANDBY */

#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MSTANDBY_MASK          (0x00000001U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MSTANDBY_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MSTANDBY_RESETVAL      (0x00000001U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MSTANDBY_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MWAIT_MASK             (0x00000002U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MWAIT_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MWAIT_RESETVAL         (0x00000001U)
#define CSL_MSS_CTRL_USB_MASTER_STANDBY_USBSS_MSTC_MWAIT_MAX              (0x00000001U)

#define CSL_MSS_CTRL_USB_MASTER_STANDBY_RESETVAL                          (0x00000003U)

/* USB_UTMI_DRVVBUS_CONTROL */

#define CSL_MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL_USBSS_DRVVBUS_CONTROL_MASK  (0x00000001U)
#define CSL_MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL_USBSS_DRVVBUS_CONTROL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL_USBSS_DRVVBUS_CONTROL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL_USBSS_DRVVBUS_CONTROL_MAX   (0x00000001U)

#define CSL_MSS_CTRL_USB_UTMI_DRVVBUS_CONTROL_RESETVAL                    (0x00000000U)

/* CONTROL_USBOTGHS_CONTROL */

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_AVALID_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_AVALID_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_AVALID_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_AVALID_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_BVALID_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_BVALID_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_BVALID_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_BVALID_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_VBUSVALID_MASK              (0x00000004U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_VBUSVALID_SHIFT             (0x00000002U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_VBUSVALID_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_VBUSVALID_MAX               (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_SESSEND_MASK                (0x00000008U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_SESSEND_SHIFT               (0x00000003U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_SESSEND_RESETVAL            (0x00000001U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_SESSEND_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDDIG_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDDIG_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDDIG_RESETVAL              (0x00000001U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDDIG_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDPULLUP_MASK               (0x00000020U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDPULLUP_SHIFT              (0x00000005U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDPULLUP_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_IDPULLUP_MAX                (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DRVVBUS_MASK                (0x00000040U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DRVVBUS_SHIFT               (0x00000006U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DRVVBUS_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DRVVBUS_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_CHRGVBUS_MASK               (0x00000080U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_CHRGVBUS_SHIFT              (0x00000007U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_CHRGVBUS_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_CHRGVBUS_MAX                (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DISCHRGVBUS_MASK            (0x00000100U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DISCHRGVBUS_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DISCHRGVBUS_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_DISCHRGVBUS_MAX             (0x00000001U)

#define CSL_MSS_CTRL_CONTROL_USBOTGHS_CONTROL_RESETVAL                    (0x00000018U)

/* R5SS0_ROM_ECLIPSE */

#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_MASK                       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_MAX                        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_WAIT_MASK                  (0x00000700U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_WAIT_SHIFT                 (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_WAIT_RESETVAL              (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_MEMSWAP_WAIT_MAX                   (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_ROM_ECLIPSE_RESETVAL                           (0x00000700U)

/* FSS_OE_NEXT_EN */

#define CSL_MSS_CTRL_FSS_OE_NEXT_EN_ENABLE_MASK                           (0x00000007U)
#define CSL_MSS_CTRL_FSS_OE_NEXT_EN_ENABLE_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_FSS_OE_NEXT_EN_ENABLE_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_FSS_OE_NEXT_EN_ENABLE_MAX                            (0x00000007U)

#define CSL_MSS_CTRL_FSS_OE_NEXT_EN_RESETVAL                              (0x00000000U)

/* OSPI1_CONFIG_CONTROL */

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EXT_CLK_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EXT_CLK_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EXT_CLK_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EXT_CLK_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ICLK_SEL_MASK                   (0x00000070U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ICLK_SEL_SHIFT                  (0x00000004U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ICLK_SEL_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ICLK_SEL_MAX                    (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EARLY_OE_N_MASK                 (0x00000700U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EARLY_OE_N_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EARLY_OE_N_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_EARLY_OE_N_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ADDRESS_TRANSLATE_EN_MASK       (0x00007000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ADDRESS_TRANSLATE_EN_SHIFT      (0x0000000CU)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ADDRESS_TRANSLATE_EN_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_ADDRESS_TRANSLATE_EN_MAX        (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_32_BIT_MODE_MASK           (0x00070000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_32_BIT_MODE_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_32_BIT_MODE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_32_BIT_MODE_MAX            (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_DDR_MODE_MASK              (0x00700000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_DDR_MODE_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_DDR_MODE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_OSPI_DDR_MODE_MAX               (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_DATA_PORT_TIMEOUT_EN_MASK       (0x07000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_DATA_PORT_TIMEOUT_EN_SHIFT      (0x00000018U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_DATA_PORT_TIMEOUT_EN_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_DATA_PORT_TIMEOUT_EN_MAX        (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_CONFIG_PORT_TIMEOUT_EN_MASK     (0x70000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_CONFIG_PORT_TIMEOUT_EN_SHIFT    (0x0000001CU)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_CONFIG_PORT_TIMEOUT_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_CONFIG_PORT_TIMEOUT_EN_MAX      (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_CONFIG_CONTROL_RESETVAL                        (0x00000000U)

/* CTRLMMR_ICSSM0_CTRL0 */

#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL0_GPM_BIDI_MASK                   (0x001FFFFFU)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL0_GPM_BIDI_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL0_GPM_BIDI_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL0_GPM_BIDI_MAX                    (0x001FFFFFU)

#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL0_RESETVAL                        (0x00000000U)

/* CTRLMMR_ICSSM0_CTRL1 */

#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL1_GPM_BIDI_MASK                   (0x001FFFFFU)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL1_GPM_BIDI_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL1_GPM_BIDI_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL1_GPM_BIDI_MAX                    (0x001FFFFFU)

#define CSL_MSS_CTRL_CTRLMMR_ICSSM0_CTRL1_RESETVAL                        (0x00000000U)

/* CTRLMMR_ICSSM1_CTRL0 */

#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL0_GPM_BIDI_MASK                   (0x001FFFFFU)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL0_GPM_BIDI_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL0_GPM_BIDI_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL0_GPM_BIDI_MAX                    (0x001FFFFFU)

#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL0_RESETVAL                        (0x00000000U)

/* CTRLMMR_ICSSM1_CTRL1 */

#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL1_GPM_BIDI_MASK                   (0x001FFFFFU)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL1_GPM_BIDI_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL1_GPM_BIDI_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL1_GPM_BIDI_MAX                    (0x001FFFFFU)

#define CSL_MSS_CTRL_CTRLMMR_ICSSM1_CTRL1_RESETVAL                        (0x00000000U)

/* HW_SPARE_RW0 */

#define CSL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RW0_RESETVAL                                (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RW1_RESETVAL                                (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RW2_RESETVAL                                (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RW3_RESETVAL                                (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RO0_RESETVAL                                (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RO1_RESETVAL                                (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RO2_RESETVAL                                (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT                      (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HW_SPARE_RO3_RESETVAL                                (0x00000000U)

/* HW_SPARE_REC */

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC0_MASK                      (0x00000001U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC0_SHIFT                     (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC0_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC1_MASK                      (0x00000002U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC1_SHIFT                     (0x00000001U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC1_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC2_MASK                      (0x00000004U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC2_SHIFT                     (0x00000002U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC2_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC3_MASK                      (0x00000008U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC3_SHIFT                     (0x00000003U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC3_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC4_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC4_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC4_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC5_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC5_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC5_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC6_MASK                      (0x00000040U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC6_SHIFT                     (0x00000006U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC6_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC7_MASK                      (0x00000080U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC7_SHIFT                     (0x00000007U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC7_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC8_MASK                      (0x00000100U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC8_SHIFT                     (0x00000008U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC8_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC9_MASK                      (0x00000200U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC9_SHIFT                     (0x00000009U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC9_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC10_MASK                     (0x00000400U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC10_SHIFT                    (0x0000000AU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC10_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC11_MASK                     (0x00000800U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC11_SHIFT                    (0x0000000BU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC11_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC12_MASK                     (0x00001000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC12_SHIFT                    (0x0000000CU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC12_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC13_MASK                     (0x00002000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC13_SHIFT                    (0x0000000DU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC13_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC14_MASK                     (0x00004000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC14_SHIFT                    (0x0000000EU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC14_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC15_MASK                     (0x00008000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC15_SHIFT                    (0x0000000FU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC15_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC16_MASK                     (0x00010000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC16_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC16_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC17_MASK                     (0x00020000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC17_SHIFT                    (0x00000011U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC17_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC18_MASK                     (0x00040000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC18_SHIFT                    (0x00000012U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC18_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC19_MASK                     (0x00080000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC19_SHIFT                    (0x00000013U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC19_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC20_MASK                     (0x00100000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC20_SHIFT                    (0x00000014U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC20_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC21_MASK                     (0x00200000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC21_SHIFT                    (0x00000015U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC21_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC22_MASK                     (0x00400000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC22_SHIFT                    (0x00000016U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC22_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC23_MASK                     (0x00800000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC23_SHIFT                    (0x00000017U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC23_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC24_MASK                     (0x01000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC24_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC24_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC25_MASK                     (0x02000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC25_SHIFT                    (0x00000019U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC25_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC26_MASK                     (0x04000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC26_SHIFT                    (0x0000001AU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC26_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC27_MASK                     (0x08000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC27_SHIFT                    (0x0000001BU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC27_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC28_MASK                     (0x10000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC28_SHIFT                    (0x0000001CU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC28_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC29_MASK                     (0x20000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC29_SHIFT                    (0x0000001DU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC29_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC30_MASK                     (0x40000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC30_SHIFT                    (0x0000001EU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC30_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC31_MASK                     (0x80000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC31_SHIFT                    (0x0000001FU)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC31_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_HW_SPARE_REC_RESETVAL                                (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_LOCK0_KICK0_RESETVAL                                 (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_LOCK0_KICK1_RESETVAL                                 (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                        (0x00000002U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                       (0x00000001U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                        (0x00000004U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                       (0x00000002U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                       (0x00000008U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                      (0x00000003U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                        (0x00000001U)

#define CSL_MSS_CTRL_INTR_RAW_STATUS_RESETVAL                             (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK      (0x00000001U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX       (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK      (0x00000002U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT     (0x00000001U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX       (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK      (0x00000004U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX       (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK     (0x00000008U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT    (0x00000003U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                   (0x00000000U)

/* INTR_ENABLE */

#define CSL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                         (0x00000001U)
#define CSL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                        (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                         (0x00000002U)
#define CSL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                        (0x00000001U)
#define CSL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                         (0x00000004U)
#define CSL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                        (0x00000002U)
#define CSL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                        (0x00000008U)
#define CSL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                       (0x00000003U)
#define CSL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_RESETVAL                                 (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK               (0x00000001U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK               (0x00000002U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT              (0x00000001U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK               (0x00000004U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT              (0x00000002U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK              (0x00000008U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT             (0x00000003U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_INTR_ENABLE_CLEAR_RESETVAL                           (0x00000000U)

/* EOI */

#define CSL_MSS_CTRL_EOI_EOI_VECTOR_MASK                                  (0x000000FFU)
#define CSL_MSS_CTRL_EOI_EOI_VECTOR_SHIFT                                 (0x00000000U)
#define CSL_MSS_CTRL_EOI_EOI_VECTOR_RESETVAL                              (0x00000000U)
#define CSL_MSS_CTRL_EOI_EOI_VECTOR_MAX                                   (0x000000FFU)

#define CSL_MSS_CTRL_EOI_RESETVAL                                         (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_FAULT_ADDRESS_RESETVAL                               (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                    (0x0000003FU)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                     (0x0000003FU)

#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                      (0x00000040U)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                     (0x00000006U)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_FAULT_TYPE_STATUS_RESETVAL                           (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                  (0x000000FFU)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                 (0x00000000U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                 (0x000FFF00U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                  (0x00000FFFU)

#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                     (0xFFF00000U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                    (0x00000014U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                      (0x00000FFFU)

#define CSL_MSS_CTRL_FAULT_ATTR_STATUS_RESETVAL                           (0x00000000U)

/* FAULT_CLEAR */

#define CSL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                           (0x00000001U)
#define CSL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                          (0x00000000U)
#define CSL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                            (0x00000001U)

#define CSL_MSS_CTRL_FAULT_CLEAR_RESETVAL                                 (0x00000000U)

/* R5SS0_CORE0_MBOX_WRITE_DONE */

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_1_MASK              (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_1_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_2_MASK              (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_2_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_3_MASK              (0x00001000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_3_SHIFT             (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_3_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_3_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_4_MASK              (0x00010000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_4_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_4_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_4_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_5_MASK              (0x00100000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_5_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_5_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_5_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_6_MASK              (0x01000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_6_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_6_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_6_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_7_MASK              (0x10000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_7_SHIFT             (0x0000001CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_7_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_7_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_8_MASK              (0x40000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_8_SHIFT             (0x0000001EU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_8_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_PROC_8_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_WRITE_DONE_RESETVAL                 (0x00000000U)

/* R5SS0_CORE0_MBOX_READ_REQ */

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_1_MASK                (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_1_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_2_MASK                (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_2_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_3_MASK                (0x00001000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_3_SHIFT               (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_3_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_3_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_4_MASK                (0x00010000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_4_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_4_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_4_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_5_MASK                (0x00100000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_5_SHIFT               (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_5_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_5_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_6_MASK                (0x01000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_6_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_6_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_6_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_7_MASK                (0x10000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_7_SHIFT               (0x0000001CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_7_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_7_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_8_MASK                (0x40000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_8_SHIFT               (0x0000001EU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_8_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_PROC_8_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_REQ_RESETVAL                   (0x00000000U)

/* R5SS0_CORE0_MBOX_READ_DONE_ACK */

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_ACK_PROC_MASK             (0x000001FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_ACK_PROC_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_ACK_PROC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_ACK_PROC_MAX              (0x000001FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_ACK_RESETVAL              (0x00000000U)

/* R5SS0_CORE0_MBOX_READ_DONE */

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_0_MASK               (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_0_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_1_MASK               (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_1_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_1_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_2_MASK               (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_2_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_2_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_2_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_3_MASK               (0x00001000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_3_SHIFT              (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_3_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_3_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_4_MASK               (0x00010000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_4_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_4_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_4_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_5_MASK               (0x00100000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_5_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_5_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_5_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_6_MASK               (0x01000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_6_SHIFT              (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_6_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_6_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_7_MASK               (0x10000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_7_SHIFT              (0x0000001CU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_7_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_7_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_8_MASK               (0x40000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_8_SHIFT              (0x0000001EU)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_8_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_PROC_8_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_MBOX_READ_DONE_RESETVAL                  (0x00000000U)

/* R5SS0_CORE0_SW_INT */

#define CSL_MSS_CTRL_R5SS0_CORE0_SW_INT_PULSE_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_SW_INT_PULSE_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_SW_INT_PULSE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_SW_INT_PULSE_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_SW_INT_RESETVAL                          (0x00000000U)

/* MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK */

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_ADDR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_ADDR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_ADDR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_ADDR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_ADDR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_ADDR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_ADDR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_ADDR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_ADDR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_ADDR_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_ADDR_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_ADDR_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_ADDR_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_ADDR_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_ADDR_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_ADDR_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_ADDR_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_ADDR_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_ADDR_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_ADDR_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_ADDR_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_ADDR_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_ADDR_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_ADDR_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_ADDR_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_ADDR_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_ADDR_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_ADDR_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_ADDR_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_ADDR_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_ADDR_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_ADDR_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_ADDR_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_MASK_RESETVAL             (0x00000000U)

/* MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS */

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_ADDR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_ADDR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_ADDR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_ADDR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_ADDR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_ADDR_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_ADDR_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_ADDR_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_ADDR_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_ADDR_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_ADDR_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_ADDR_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_ADDR_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_ADDR_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_ADDR_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_ADDR_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_ADDR_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_ADDR_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_ADDR_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_ADDR_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_ADDR_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_ADDR_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_ADDR_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_ADDR_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_ADDR_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RESETVAL           (0x00000000U)

/* MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW */

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_ADDR_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_ADDR_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_ADDR_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_ADDR_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_ADDR_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_ADDR_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_ADDR_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_ADDR_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_ADDR_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_ADDR_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU0_STATUS_RAW_RESETVAL       (0x00000000U)

/* MPU_PROT_ERRAGG_R5SS0_CPU0_MASK */

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_PROT_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_PROT_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_PROT_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_B_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_PROT_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_PROT_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_L2_BANK_C_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_PROT_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_PROT_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_PROT_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_PROT_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_PROT_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_PROT_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_DTHE_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_PROT_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_PROT_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_MBOX_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_PROT_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_PROT_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_PROT_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_PROT_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_PROT_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_PROT_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_SCRM2SCRP1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_PROT_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_PROT_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5A0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_CR5B0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_PROT_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_PROT_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_HSM_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_PROT_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_PROT_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OPTI_FLASH_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_PROT_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_PROT_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_R5SS0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_PROT_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_PROT_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_PROT_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_PROT_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_MPU_OSPI1_CFG_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_MASK_RESETVAL             (0x00000000U)

/* MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS */

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_PROT_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_PROT_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_PROT_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_B_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_PROT_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_PROT_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_L2_BANK_C_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_PROT_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_PROT_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_PROT_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_PROT_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_PROT_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_PROT_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_DTHE_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_PROT_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_PROT_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_MBOX_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_PROT_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_PROT_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_PROT_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_PROT_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_PROT_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_PROT_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_SCRM2SCRP1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_PROT_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_PROT_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5A0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_PROT_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_PROT_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_CR5B0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_PROT_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_PROT_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_HSM_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_PROT_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_PROT_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OPTI_FLASH_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_PROT_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_PROT_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_R5SS0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_PROT_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_PROT_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_PROT_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_PROT_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_MPU_OSPI1_CFG_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RESETVAL           (0x00000000U)

/* MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW */

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_PROT_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_PROT_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_DTHE_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_PROT_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_PROT_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_MBOX_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_PROT_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_PROT_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_PROT_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_PROT_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_HSM_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_PROT_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_PROT_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_R5SS0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_PROT_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_PROT_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU0_STATUS_RAW_RESETVAL       (0x00000000U)

/* R5SS0_CORE1_MBOX_WRITE_DONE */

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_1_MASK              (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_1_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_2_MASK              (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_2_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_3_MASK              (0x00001000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_3_SHIFT             (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_3_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_3_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_4_MASK              (0x00010000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_4_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_4_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_4_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_5_MASK              (0x00100000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_5_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_5_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_5_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_6_MASK              (0x01000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_6_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_6_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_6_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_7_MASK              (0x10000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_7_SHIFT             (0x0000001CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_7_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_7_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_8_MASK              (0x40000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_8_SHIFT             (0x0000001EU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_8_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_PROC_8_MAX               (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_WRITE_DONE_RESETVAL                 (0x00000000U)

/* R5SS0_CORE1_MBOX_READ_REQ */

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_1_MASK                (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_1_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_2_MASK                (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_2_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_3_MASK                (0x00001000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_3_SHIFT               (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_3_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_3_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_4_MASK                (0x00010000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_4_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_4_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_4_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_5_MASK                (0x00100000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_5_SHIFT               (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_5_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_5_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_6_MASK                (0x01000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_6_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_6_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_6_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_7_MASK                (0x10000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_7_SHIFT               (0x0000001CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_7_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_7_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_8_MASK                (0x40000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_8_SHIFT               (0x0000001EU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_8_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_PROC_8_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_REQ_RESETVAL                   (0x00000000U)

/* R5SS0_CORE1_MBOX_READ_DONE_ACK */

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_ACK_PROC_MASK             (0x000001FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_ACK_PROC_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_ACK_PROC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_ACK_PROC_MAX              (0x000001FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_ACK_RESETVAL              (0x00000000U)

/* R5SS0_CORE1_MBOX_READ_DONE */

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_0_MASK               (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_0_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_1_MASK               (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_1_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_1_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_2_MASK               (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_2_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_2_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_2_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_3_MASK               (0x00001000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_3_SHIFT              (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_3_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_3_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_4_MASK               (0x00010000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_4_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_4_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_4_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_5_MASK               (0x00100000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_5_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_5_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_5_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_6_MASK               (0x01000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_6_SHIFT              (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_6_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_6_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_7_MASK               (0x10000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_7_SHIFT              (0x0000001CU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_7_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_7_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_8_MASK               (0x40000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_8_SHIFT              (0x0000001EU)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_8_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_PROC_8_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_MBOX_READ_DONE_RESETVAL                  (0x00000000U)

/* R5SS0_CORE1_SW_INT */

#define CSL_MSS_CTRL_R5SS0_CORE1_SW_INT_PULSE_MASK                        (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_SW_INT_PULSE_SHIFT                       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_SW_INT_PULSE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_SW_INT_PULSE_MAX                         (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_SW_INT_RESETVAL                          (0x00000000U)

/* MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK */

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_ADDR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_ADDR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_ADDR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_ADDR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_ADDR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_ADDR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_ADDR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_ADDR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_ADDR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_ADDR_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_ADDR_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_ADDR_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_ADDR_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_ADDR_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_ADDR_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_ADDR_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_ADDR_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_ADDR_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_ADDR_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_ADDR_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_ADDR_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_ADDR_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_ADDR_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_ADDR_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_ADDR_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_ADDR_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_ADDR_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_ADDR_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_ADDR_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_ADDR_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_ADDR_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_ADDR_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_ADDR_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_MASK_RESETVAL             (0x00000000U)

/* MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS */

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_ADDR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_ADDR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_ADDR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_ADDR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_ADDR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_ADDR_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_ADDR_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_ADDR_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_ADDR_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_ADDR_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_ADDR_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_ADDR_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_ADDR_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_ADDR_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_ADDR_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_ADDR_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_ADDR_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_ADDR_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_ADDR_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_ADDR_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_ADDR_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_ADDR_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_ADDR_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_ADDR_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_ADDR_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RESETVAL           (0x00000000U)

/* MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW */

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_ADDR_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_ADDR_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_ADDR_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_ADDR_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_ADDR_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_ADDR_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_ADDR_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_ADDR_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_ADDR_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_ADDR_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_ADDR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_ADDR_ERRAGG_R5SS0_CPU1_STATUS_RAW_RESETVAL       (0x00000000U)

/* MPU_PROT_ERRAGG_R5SS0_CPU1_MASK */

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_PROT_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_PROT_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_PROT_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_B_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_PROT_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_PROT_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_L2_BANK_C_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_PROT_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_PROT_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_PROT_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_PROT_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_PROT_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_PROT_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_DTHE_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_PROT_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_PROT_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_MBOX_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_PROT_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_PROT_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_PROT_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_PROT_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_PROT_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_PROT_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_SCRM2SCRP1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_PROT_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_PROT_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5A0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_PROT_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_PROT_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_CR5B0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_PROT_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_PROT_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_HSM_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_PROT_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_PROT_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OPTI_FLASH_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_PROT_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_PROT_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_R5SS0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_PROT_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_PROT_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_PROT_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_PROT_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_MPU_OSPI1_CFG_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_MASK_RESETVAL             (0x00000000U)

/* MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS */

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_PROT_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_PROT_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_PROT_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_B_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_PROT_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_PROT_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_L2_BANK_C_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_PROT_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_PROT_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_PROT_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_PROT_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_PROT_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_PROT_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_DTHE_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_PROT_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_PROT_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_MBOX_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_PROT_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_PROT_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_PROT_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_PROT_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_PROT_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_PROT_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_SCRM2SCRP1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_PROT_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_PROT_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5A0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_PROT_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_PROT_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_CR5B0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_PROT_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_PROT_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_HSM_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_PROT_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_PROT_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OPTI_FLASH_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_PROT_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_PROT_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_R5SS0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_PROT_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_PROT_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_PROT_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_PROT_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_MPU_OSPI1_CFG_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RESETVAL           (0x00000000U)

/* MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW */

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_B_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_L2_BANK_C_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AXIS_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_PROT_ERR_MASK (0x00000100U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_PROT_ERR_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_DTHE_A_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_PROT_ERR_MASK (0x00000200U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_PROT_ERR_SHIFT (0x00000009U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_MBOX_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_PROT_ERR_MASK (0x00000400U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_PROT_ERR_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_MASK (0x00000800U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_MASK (0x00001000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_SCRM2SCRP1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_MASK (0x00002000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5A0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_MASK (0x00004000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_CR5B0_AHB_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_PROT_ERR_MASK (0x00020000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_PROT_ERR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_HSM_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_MASK (0x00040000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OPTI_FLASH_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_PROT_ERR_MASK (0x00080000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_PROT_ERR_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_R5SS0_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_PROT_ERR_MASK (0x00800000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_PROT_ERR_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_MASK (0x01000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_MPU_OSPI1_CFG_PROT_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_MPU_PROT_ERRAGG_R5SS0_CPU1_STATUS_RAW_RESETVAL       (0x00000000U)

/* ICSSM0_PRU0_MBOX_WRITE_DONE */

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_1_MASK              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_1_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_2_MASK              (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_2_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_3_MASK              (0x00001000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_3_SHIFT             (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_3_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_3_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_4_MASK              (0x00010000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_4_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_4_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_4_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_5_MASK              (0x00100000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_5_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_5_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_5_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_6_MASK              (0x01000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_6_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_6_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_6_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_7_MASK              (0x10000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_7_SHIFT             (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_7_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_7_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_8_MASK              (0x40000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_8_SHIFT             (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_8_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_PROC_8_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_WRITE_DONE_RESETVAL                 (0x00000000U)

/* ICSSM0_PRU0_MBOX_READ_REQ */

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_1_MASK                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_1_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_2_MASK                (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_2_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_3_MASK                (0x00001000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_3_SHIFT               (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_3_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_3_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_4_MASK                (0x00010000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_4_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_4_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_4_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_5_MASK                (0x00100000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_5_SHIFT               (0x00000014U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_5_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_5_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_6_MASK                (0x01000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_6_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_6_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_6_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_7_MASK                (0x10000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_7_SHIFT               (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_7_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_7_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_8_MASK                (0x40000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_8_SHIFT               (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_8_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_PROC_8_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_REQ_RESETVAL                   (0x00000000U)

/* ICSSM0_PRU0_MBOX_READ_DONE_ACK */

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_ACK_PROC_MASK             (0x000001FFU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_ACK_PROC_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_ACK_PROC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_ACK_PROC_MAX              (0x000001FFU)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_ACK_RESETVAL              (0x00000000U)

/* ICSSM0_PRU0_MBOX_READ_DONE */

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_0_MASK               (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_0_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_1_MASK               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_1_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_1_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_2_MASK               (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_2_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_2_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_2_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_3_MASK               (0x00001000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_3_SHIFT              (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_3_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_3_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_4_MASK               (0x00010000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_4_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_4_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_4_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_5_MASK               (0x00100000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_5_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_5_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_5_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_6_MASK               (0x01000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_6_SHIFT              (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_6_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_6_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_7_MASK               (0x10000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_7_SHIFT              (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_7_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_7_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_8_MASK               (0x40000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_8_SHIFT              (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_8_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_PROC_8_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU0_MBOX_READ_DONE_RESETVAL                  (0x00000000U)

/* ICSSM0_PRU1_MBOX_WRITE_DONE */

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_1_MASK              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_1_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_2_MASK              (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_2_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_3_MASK              (0x00001000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_3_SHIFT             (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_3_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_3_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_4_MASK              (0x00010000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_4_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_4_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_4_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_5_MASK              (0x00100000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_5_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_5_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_5_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_6_MASK              (0x01000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_6_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_6_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_6_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_7_MASK              (0x10000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_7_SHIFT             (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_7_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_7_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_8_MASK              (0x40000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_8_SHIFT             (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_8_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_PROC_8_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_WRITE_DONE_RESETVAL                 (0x00000000U)

/* ICSSM0_PRU1_MBOX_READ_REQ */

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_1_MASK                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_1_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_2_MASK                (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_2_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_3_MASK                (0x00001000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_3_SHIFT               (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_3_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_3_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_4_MASK                (0x00010000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_4_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_4_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_4_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_5_MASK                (0x00100000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_5_SHIFT               (0x00000014U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_5_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_5_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_6_MASK                (0x01000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_6_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_6_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_6_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_7_MASK                (0x10000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_7_SHIFT               (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_7_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_7_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_8_MASK                (0x40000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_8_SHIFT               (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_8_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_PROC_8_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_REQ_RESETVAL                   (0x00000000U)

/* ICSSM0_PRU1_MBOX_READ_DONE_ACK */

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_ACK_PROC_MASK             (0x000001FFU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_ACK_PROC_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_ACK_PROC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_ACK_PROC_MAX              (0x000001FFU)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_ACK_RESETVAL              (0x00000000U)

/* ICSSM0_PRU1_MBOX_READ_DONE */

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_0_MASK               (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_0_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_1_MASK               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_1_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_1_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_2_MASK               (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_2_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_2_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_2_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_3_MASK               (0x00001000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_3_SHIFT              (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_3_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_3_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_4_MASK               (0x00010000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_4_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_4_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_4_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_5_MASK               (0x00100000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_5_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_5_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_5_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_6_MASK               (0x01000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_6_SHIFT              (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_6_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_6_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_7_MASK               (0x10000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_7_SHIFT              (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_7_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_7_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_8_MASK               (0x40000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_8_SHIFT              (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_8_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_PROC_8_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PRU1_MBOX_READ_DONE_RESETVAL                  (0x00000000U)

/* ICSSM1_PRU0_MBOX_WRITE_DONE */

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_1_MASK              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_1_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_2_MASK              (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_2_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_3_MASK              (0x00001000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_3_SHIFT             (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_3_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_3_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_4_MASK              (0x00010000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_4_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_4_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_4_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_5_MASK              (0x00100000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_5_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_5_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_5_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_6_MASK              (0x01000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_6_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_6_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_6_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_7_MASK              (0x10000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_7_SHIFT             (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_7_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_7_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_8_MASK              (0x40000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_8_SHIFT             (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_8_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_PROC_8_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_WRITE_DONE_RESETVAL                 (0x00000000U)

/* ICSSM1_PRU0_MBOX_READ_REQ */

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_1_MASK                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_1_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_2_MASK                (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_2_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_3_MASK                (0x00001000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_3_SHIFT               (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_3_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_3_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_4_MASK                (0x00010000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_4_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_4_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_4_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_5_MASK                (0x00100000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_5_SHIFT               (0x00000014U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_5_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_5_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_6_MASK                (0x01000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_6_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_6_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_6_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_7_MASK                (0x10000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_7_SHIFT               (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_7_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_7_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_8_MASK                (0x40000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_8_SHIFT               (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_8_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_PROC_8_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_REQ_RESETVAL                   (0x00000000U)

/* ICSSM1_PRU0_MBOX_READ_DONE_ACK */

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_ACK_PROC_MASK             (0x000001FFU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_ACK_PROC_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_ACK_PROC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_ACK_PROC_MAX              (0x000001FFU)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_ACK_RESETVAL              (0x00000000U)

/* ICSSM1_PRU0_MBOX_READ_DONE */

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_0_MASK               (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_0_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_1_MASK               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_1_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_1_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_2_MASK               (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_2_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_2_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_2_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_3_MASK               (0x00001000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_3_SHIFT              (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_3_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_3_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_4_MASK               (0x00010000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_4_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_4_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_4_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_5_MASK               (0x00100000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_5_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_5_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_5_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_6_MASK               (0x01000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_6_SHIFT              (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_6_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_6_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_7_MASK               (0x10000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_7_SHIFT              (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_7_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_7_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_8_MASK               (0x40000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_8_SHIFT              (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_8_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_PROC_8_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU0_MBOX_READ_DONE_RESETVAL                  (0x00000000U)

/* ICSSM1_PRU1_MBOX_WRITE_DONE */

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_0_MASK              (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_0_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_1_MASK              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_1_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_2_MASK              (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_2_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_2_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_2_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_3_MASK              (0x00001000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_3_SHIFT             (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_3_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_3_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_4_MASK              (0x00010000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_4_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_4_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_4_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_5_MASK              (0x00100000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_5_SHIFT             (0x00000014U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_5_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_5_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_6_MASK              (0x01000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_6_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_6_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_6_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_7_MASK              (0x10000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_7_SHIFT             (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_7_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_7_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_8_MASK              (0x40000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_8_SHIFT             (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_8_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_PROC_8_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_WRITE_DONE_RESETVAL                 (0x00000000U)

/* ICSSM1_PRU1_MBOX_READ_REQ */

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_0_MASK                (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_0_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_1_MASK                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_1_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_1_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_2_MASK                (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_2_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_2_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_2_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_3_MASK                (0x00001000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_3_SHIFT               (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_3_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_3_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_4_MASK                (0x00010000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_4_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_4_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_4_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_5_MASK                (0x00100000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_5_SHIFT               (0x00000014U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_5_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_5_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_6_MASK                (0x01000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_6_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_6_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_6_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_7_MASK                (0x10000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_7_SHIFT               (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_7_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_7_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_8_MASK                (0x40000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_8_SHIFT               (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_8_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_PROC_8_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_REQ_RESETVAL                   (0x00000000U)

/* ICSSM1_PRU1_MBOX_READ_DONE_ACK */

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_ACK_PROC_MASK             (0x000001FFU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_ACK_PROC_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_ACK_PROC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_ACK_PROC_MAX              (0x000001FFU)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_ACK_RESETVAL              (0x00000000U)

/* ICSSM1_PRU1_MBOX_READ_DONE */

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_0_MASK               (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_0_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_1_MASK               (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_1_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_1_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_2_MASK               (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_2_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_2_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_2_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_3_MASK               (0x00001000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_3_SHIFT              (0x0000000CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_3_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_3_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_4_MASK               (0x00010000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_4_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_4_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_4_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_5_MASK               (0x00100000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_5_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_5_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_5_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_6_MASK               (0x01000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_6_SHIFT              (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_6_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_6_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_7_MASK               (0x10000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_7_SHIFT              (0x0000001CU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_7_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_7_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_8_MASK               (0x40000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_8_SHIFT              (0x0000001EU)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_8_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_PROC_8_MAX                (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PRU1_MBOX_READ_DONE_RESETVAL                  (0x00000000U)

/* TPCC0_ERRAGG_MASK */

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_ERRINT_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_ERRINT_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_ERRINT_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_ERRINT_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_MPINT_MASK                  (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_MPINT_SHIFT                 (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_MPINT_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_MPINT_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_ERR_MASK                   (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_ERR_SHIFT                  (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_ERR_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_ERR_MASK                   (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_ERR_SHIFT                  (0x00000003U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_ERR_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_PAR_ERR_MASK                (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_PAR_ERR_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_PAR_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_PAR_ERR_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MASK     (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_SHIFT    (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MASK    (0x00020000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT   (0x00000011U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MASK    (0x00040000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT   (0x00000012U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MASK      (0x01000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_SHIFT     (0x00000018U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MAX       (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MASK     (0x02000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_SHIFT    (0x00000019U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MASK     (0x04000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_SHIFT    (0x0000001AU)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* TPCC0_ERRAGG_STATUS */

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_ERRINT_MASK               (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_ERRINT_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_ERRINT_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_ERRINT_MAX                (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_MPINT_MASK                (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_MPINT_SHIFT               (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_MPINT_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_MPINT_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_ERR_MASK                 (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_ERR_SHIFT                (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_ERR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_ERR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_ERR_MASK                 (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_ERR_SHIFT                (0x00000003U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_ERR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_ERR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_PAR_ERR_MASK              (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_PAR_ERR_SHIFT             (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_PAR_ERR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_PAR_ERR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MASK   (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_SHIFT  (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MAX    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MASK  (0x00020000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MAX   (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MASK  (0x00040000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MAX   (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MASK    (0x01000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_SHIFT   (0x00000018U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MASK   (0x02000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_SHIFT  (0x00000019U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MAX    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MASK   (0x04000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_SHIFT  (0x0000001AU)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MAX    (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* TPCC0_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MASK           (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MAX            (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MASK            (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_MPINT_SHIFT           (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_MPINT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MAX             (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MASK             (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_ERR_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_ERR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MASK             (0x00000008U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_ERR_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_ERR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_MASK          (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_SHIFT         (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* MMR_ACCESS_ERRAGG_MASK0 */

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_RD_MASK             (0x00000001U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_RD_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_RD_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_RD_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_WR_MASK             (0x00000002U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_WR_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_WR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_CTRL_WR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_RD_MASK              (0x00000004U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_RD_SHIFT             (0x00000002U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_RD_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_RD_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_WR_MASK              (0x00000008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_WR_SHIFT             (0x00000003U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_WR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_MSS_RCM_WR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_RD_MASK             (0x00000010U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_RD_SHIFT            (0x00000004U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_RD_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_RD_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_WR_MASK             (0x00000020U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_WR_SHIFT            (0x00000005U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_WR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_CTRL_WR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_RD_MASK              (0x00000040U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_RD_SHIFT             (0x00000006U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_RD_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_RD_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_WR_MASK              (0x00000080U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_WR_SHIFT             (0x00000007U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_WR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_TOP_RCM_WR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_RD_MASK         (0x00000100U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_RD_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_RD_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_RD_MAX          (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_WR_MASK         (0x00000200U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_WR_SHIFT        (0x00000009U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_WR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_SOC_CTRL_WR_MAX          (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_RD_MASK             (0x00000400U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_RD_SHIFT            (0x0000000AU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_RD_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_RD_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_WR_MASK             (0x00000800U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_WR_SHIFT            (0x0000000BU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_WR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_HSM_CTRL_WR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_MASK0_RESETVAL                     (0x00000000U)

/* MMR_ACCESS_ERRAGG_STATUS0 */

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_RD_MASK           (0x00000001U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_RD_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_RD_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_RD_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_WR_MASK           (0x00000002U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_WR_SHIFT          (0x00000001U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_WR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_CTRL_WR_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_RD_MASK            (0x00000004U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_RD_SHIFT           (0x00000002U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_RD_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_RD_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_WR_MASK            (0x00000008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_WR_SHIFT           (0x00000003U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_WR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_MSS_RCM_WR_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_RD_MASK           (0x00000010U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_RD_SHIFT          (0x00000004U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_RD_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_RD_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_WR_MASK           (0x00000020U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_WR_SHIFT          (0x00000005U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_WR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_CTRL_WR_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_RD_MASK            (0x00000040U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_RD_SHIFT           (0x00000006U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_RD_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_RD_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_WR_MASK            (0x00000080U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_WR_SHIFT           (0x00000007U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_WR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_TOP_RCM_WR_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_MASK       (0x00000100U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_MASK       (0x00000200U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_SHIFT      (0x00000009U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_RD_MASK           (0x00000400U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_RD_SHIFT          (0x0000000AU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_RD_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_RD_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_WR_MASK           (0x00000800U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_WR_SHIFT          (0x0000000BU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_WR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_HSM_CTRL_WR_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS0_RESETVAL                   (0x00000000U)

/* MMR_ACCESS_ERRAGG_STATUS_RAW0 */

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_MASK       (0x00000001U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_MASK       (0x00000002U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_SHIFT      (0x00000001U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_RD_MASK        (0x00000004U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_RD_SHIFT       (0x00000002U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_RD_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_RD_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_WR_MASK        (0x00000008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_WR_SHIFT       (0x00000003U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_WR_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_MSS_RCM_WR_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_MASK       (0x00000010U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_SHIFT      (0x00000004U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_MASK       (0x00000020U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_SHIFT      (0x00000005U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_RD_MASK        (0x00000040U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_RD_SHIFT       (0x00000006U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_RD_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_RD_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_WR_MASK        (0x00000080U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_WR_SHIFT       (0x00000007U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_WR_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_TOP_RCM_WR_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_MASK   (0x00000100U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_SHIFT  (0x00000008U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_MAX    (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_MASK   (0x00000200U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_SHIFT  (0x00000009U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_MAX    (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_MASK       (0x00000400U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_SHIFT      (0x0000000AU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_MASK       (0x00000800U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_SHIFT      (0x0000000BU)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MMR_ACCESS_ERRAGG_STATUS_RAW0_RESETVAL               (0x00000000U)

/* R5SS0_CPU0_ECC_CORR_ERRAGG_MASK */

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_CORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_CORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_CORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_CORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_CORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_CORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_CORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_CORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_CORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ITAG_CORR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ITAG_CORR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ITAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_ITAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_IDATA_CORR_ERR_MASK (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_IDATA_CORR_ERR_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_IDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_R5SS0_CPU0_IDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK_RESETVAL             (0x00000000U)

/* R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS */

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_CORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_CORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_CORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_CORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_CORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_CORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_CORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_CORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_CORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ITAG_CORR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ITAG_CORR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ITAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_ITAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_IDATA_CORR_ERR_MASK (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_IDATA_CORR_ERR_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_IDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU0_IDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RESETVAL           (0x00000000U)

/* R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_CORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_CORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_CORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_CORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_CORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_CORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_CORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_CORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_CORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ITAG_CORR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ITAG_CORR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ITAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ITAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_IDATA_CORR_ERR_MASK (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_IDATA_CORR_ERR_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_IDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_IDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW_RESETVAL       (0x00000000U)

/* R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK */

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_UNCORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_UNCORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_ATCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B1TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_UNCORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_UNCORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_B0TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_UNCORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_UNCORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DTAG_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_UNCORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_UNCORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU0_DDATA_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK_RESETVAL           (0x00000000U)

/* R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS */

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_UNCORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_UNCORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_ATCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B1TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_UNCORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_UNCORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_B0TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_UNCORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_UNCORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DTAG_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_UNCORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_UNCORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU0_DDATA_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RESETVAL         (0x00000000U)

/* R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_UNCORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_UNCORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_UNCORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_UNCORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_UNCORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_UNCORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DTAG_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_UNCORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_UNCORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU0_DDATA_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW_RESETVAL     (0x00000000U)

/* R5SS0_CPU1_ECC_CORR_ERRAGG_MASK */

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_CORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_CORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_CORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_CORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_CORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_CORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_CORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_CORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_CORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ITAG_CORR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ITAG_CORR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ITAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_ITAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_IDATA_CORR_ERR_MASK (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_IDATA_CORR_ERR_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_IDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_R5SS0_CPU1_IDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_MASK_RESETVAL             (0x00000000U)

/* R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS */

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_CORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_CORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_CORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_CORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_CORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_CORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_CORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_CORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_CORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ITAG_CORR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ITAG_CORR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ITAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_ITAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_IDATA_CORR_ERR_MASK (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_IDATA_CORR_ERR_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_IDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_R5SS0_CPU1_IDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RESETVAL           (0x00000000U)

/* R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_CORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_CORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_CORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_CORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_CORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_CORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_CORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_CORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_CORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ITAG_CORR_ERR_MASK (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ITAG_CORR_ERR_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ITAG_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ITAG_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_IDATA_CORR_ERR_MASK (0x00000040U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_IDATA_CORR_ERR_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_IDATA_CORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_IDATA_CORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_CORR_ERRAGG_STATUS_RAW_RESETVAL       (0x00000000U)

/* R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK */

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_UNCORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_UNCORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_ATCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B1TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_UNCORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_UNCORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_B0TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_UNCORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_UNCORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DTAG_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_UNCORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_UNCORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_R5SS0_CPU1_DDATA_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_MASK_RESETVAL           (0x00000000U)

/* R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS */

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_UNCORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_UNCORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_ATCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B1TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_UNCORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_UNCORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_B0TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_UNCORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_UNCORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DTAG_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_UNCORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_UNCORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_R5SS0_CPU1_DDATA_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RESETVAL         (0x00000000U)

/* R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_UNCORR_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_UNCORR_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_UNCORR_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_UNCORR_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_UNCORR_ERR_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_UNCORR_ERR_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DTAG_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_UNCORR_ERR_MASK (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_UNCORR_ERR_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_UNCORR_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_R5SS0_CPU1_DDATA_UNCORR_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_ECC_UNCORR_ERRAGG_STATUS_RAW_RESETVAL     (0x00000000U)

/* R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK */

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_ATCM0_PARITY_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_ATCM0_PARITY_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_ATCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_ATCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B0TCM0_PARITY_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B0TCM0_PARITY_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B0TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B0TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B1TCM0_PARITY_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B1TCM0_PARITY_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B1TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU0_B1TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_MASK_RESETVAL       (0x00000000U)

/* R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS */

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_ATCM0_PARITY_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_ATCM0_PARITY_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_ATCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_ATCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B0TCM0_PARITY_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B0TCM0_PARITY_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B0TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B0TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B1TCM0_PARITY_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B1TCM0_PARITY_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B1TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU0_B1TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RESETVAL     (0x00000000U)

/* R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM0_PARITY_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM0_PARITY_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_ATCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM0_PARITY_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM0_PARITY_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B0TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM0_PARITY_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM0_PARITY_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU0_B1TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU0_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_RESETVAL (0x00000000U)

/* R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK */

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_ATCM1_PARITY_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_ATCM1_PARITY_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_ATCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_ATCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM0_PARITY_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM0_PARITY_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM0_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM0_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM1_PARITY_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM1_PARITY_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_R5SS0_CPU1_B1TCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_MASK_RESETVAL       (0x00000000U)

/* R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS */

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_ATCM1_PARITY_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_ATCM1_PARITY_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_ATCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_ATCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B0TCM1_PARITY_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B0TCM1_PARITY_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B0TCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B0TCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B1TCM1_PARITY_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B1TCM1_PARITY_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B1TCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_R5SS0_CPU1_B1TCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RESETVAL     (0x00000000U)

/* R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM1_PARITY_ERR_MASK (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM1_PARITY_ERR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_ATCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM1_PARITY_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM1_PARITY_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B0TCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM1_PARITY_ERR_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM1_PARITY_ERR_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM1_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_R5SS0_CPU1_B1TCM1_PARITY_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CPU1_TCM_ADDRPARITY_ERRAGG_STATUS_RAW_RESETVAL (0x00000000U)

/* MSS_VBUSM_SAFETY_H_ERRAGG_MASK0 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRH_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRH_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRH_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRH_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DAP_VBUSM_ERRH_MASK  (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DAP_VBUSM_ERRH_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DAP_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DAP_VBUSM_ERRH_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_VBUSM_ERRH_MASK  (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_VBUSM_ERRH_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_VBUSM_ERRH_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CPSW_VBUSM_ERRH_MASK (0x00004000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CPSW_VBUSM_ERRH_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CPSW_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_CPSW_VBUSM_ERRH_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM0_VBUSM_ERRH_MASK (0x00008000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM0_VBUSM_ERRH_SHIFT (0x0000000FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM1_VBUSM_ERRH_MASK (0x00010000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM1_VBUSM_ERRH_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM2_VBUSM_ERRH_MASK (0x00020000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM2_VBUSM_ERRH_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM2_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_L2RAM2_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRH_MASK (0x00080000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRH_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRH_MASK (0x00100000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRH_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRH_MASK (0x00200000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRH_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRH_MASK (0x00400000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRH_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRH_MASK (0x00800000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRH_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRH_MASK (0x01000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRH_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRH_MASK (0x02000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRH_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRH_MASK (0x04000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRH_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRH_MASK (0x08000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRH_SHIFT (0x0000001BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRH_MASK (0x10000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRH_SHIFT (0x0000001CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_OSPI_VBUSM_ERRH_MASK (0x20000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_OSPI_VBUSM_ERRH_SHIFT (0x0000001DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_OSPI_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_OSPI_VBUSM_ERRH_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_MCRC_VBUSM_ERRH_MASK (0x40000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_MCRC_VBUSM_ERRH_SHIFT (0x0000001EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_MCRC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_MCRC_VBUSM_ERRH_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DTHE_VBUSM_ERRH_MASK (0x80000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DTHE_VBUSM_ERRH_SHIFT (0x0000001FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DTHE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_DTHE_VBUSM_ERRH_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK0_RESETVAL             (0x00000000U)

/* MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRH_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRH_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRH_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRH_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DAP_VBUSM_ERRH_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DAP_VBUSM_ERRH_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DAP_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DAP_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_VBUSM_ERRH_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_VBUSM_ERRH_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CPSW_VBUSM_ERRH_MASK (0x00004000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CPSW_VBUSM_ERRH_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CPSW_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_CPSW_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRH_MASK (0x00008000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRH_SHIFT (0x0000000FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRH_MASK (0x00010000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRH_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRH_MASK (0x00020000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRH_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRH_MASK (0x00080000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRH_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRH_MASK (0x00100000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRH_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRH_MASK (0x00200000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRH_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRH_MASK (0x00400000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRH_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRH_MASK (0x00800000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRH_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRH_MASK (0x01000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRH_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRH_MASK (0x02000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRH_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRH_MASK (0x04000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRH_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRH_MASK (0x08000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRH_SHIFT (0x0000001BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRH_MASK (0x10000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRH_SHIFT (0x0000001CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_OSPI_VBUSM_ERRH_MASK (0x20000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_OSPI_VBUSM_ERRH_SHIFT (0x0000001DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_OSPI_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_OSPI_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_MCRC_VBUSM_ERRH_MASK (0x40000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_MCRC_VBUSM_ERRH_SHIFT (0x0000001EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_MCRC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_MCRC_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DTHE_VBUSM_ERRH_MASK (0x80000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DTHE_VBUSM_ERRH_SHIFT (0x0000001FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DTHE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_DTHE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS0_RESETVAL           (0x00000000U)

/* MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRH_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRH_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRH_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRH_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRH_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRH_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRH_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRH_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRH_MASK (0x00004000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRH_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRH_MASK (0x00008000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRH_SHIFT (0x0000000FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRH_MASK (0x00010000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRH_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRH_MASK (0x00020000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRH_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRH_MASK (0x00080000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRH_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRH_MASK (0x00100000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRH_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRH_MASK (0x00200000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRH_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRH_MASK (0x00400000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRH_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRH_MASK (0x00800000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRH_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRH_MASK (0x01000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRH_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRH_MASK (0x02000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRH_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRH_MASK (0x04000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRH_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRH_MASK (0x08000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRH_SHIFT (0x0000001BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRH_MASK (0x10000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRH_SHIFT (0x0000001CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRH_MASK (0x20000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRH_SHIFT (0x0000001DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRH_MASK (0x40000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRH_SHIFT (0x0000001EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRH_MASK (0x80000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRH_SHIFT (0x0000001FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW0_RESETVAL       (0x00000000U)

/* MSS_VBUSM_SAFETY_H_ERRAGG_MASK1 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_HSM_S_VBUSM_ERRH_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_HSM_S_VBUSM_ERRH_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_HSM_S_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_HSM_S_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRH_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRH_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_STM_STIM_VBUSM_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_STM_STIM_VBUSM_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_STM_STIM_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_STM_STIM_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MMC_VBUSM_ERRH_MASK  (0x00000040U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MMC_VBUSM_ERRH_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MMC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_MMC_VBUSM_ERRH_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_GPMC_VBUSM_ERRH_MASK (0x00000080U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_GPMC_VBUSM_ERRH_SHIFT (0x00000007U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_GPMC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_GPMC_VBUSM_ERRH_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRH_MASK (0x00000400U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRH_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRH_MASK (0x00000800U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRH_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRH_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRH_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_OSPI1_VBUSM_ERRH_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_OSPI1_VBUSM_ERRH_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_OSPI1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_OSPI1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_MASK1_RESETVAL             (0x00000000U)

/* MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_HSM_S_VBUSM_ERRH_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_HSM_S_VBUSM_ERRH_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_HSM_S_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_HSM_S_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRH_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRH_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MMC_VBUSM_ERRH_MASK (0x00000040U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MMC_VBUSM_ERRH_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MMC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_MMC_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_GPMC_VBUSM_ERRH_MASK (0x00000080U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_GPMC_VBUSM_ERRH_SHIFT (0x00000007U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_GPMC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_GPMC_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRH_MASK (0x00000400U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRH_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRH_MASK (0x00000800U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRH_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRH_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRH_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_OSPI1_VBUSM_ERRH_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_OSPI1_VBUSM_ERRH_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_OSPI1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_OSPI1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS1_RESETVAL           (0x00000000U)

/* MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRH_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRH_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRH_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRH_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRH_MASK (0x00000040U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRH_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRH_MASK (0x00000080U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRH_SHIFT (0x00000007U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRH_MASK (0x00000400U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRH_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRH_MASK (0x00000800U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRH_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRH_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRH_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRH_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRH_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_H_ERRAGG_STATUS_RAW1_RESETVAL       (0x00000000U)

/* MSS_VBUSM_SAFETY_L_ERRAGG_MASK0 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRL_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRL_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRL_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRL_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRL_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRL_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRL_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRL_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRL_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5A0_SLV_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRL_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRL_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CR5B0_SLV_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DAP_VBUSM_ERRL_MASK  (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DAP_VBUSM_ERRL_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DAP_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DAP_VBUSM_ERRL_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_VBUSM_ERRL_MASK  (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_VBUSM_ERRL_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_VBUSM_ERRL_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CPSW_VBUSM_ERRL_MASK (0x00004000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CPSW_VBUSM_ERRL_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CPSW_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_CPSW_VBUSM_ERRL_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM0_VBUSM_ERRL_MASK (0x00008000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM0_VBUSM_ERRL_SHIFT (0x0000000FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM1_VBUSM_ERRL_MASK (0x00010000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM1_VBUSM_ERRL_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM2_VBUSM_ERRL_MASK (0x00020000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM2_VBUSM_ERRL_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM2_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_L2RAM2_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRL_MASK (0x00080000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRL_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRL_MASK (0x00100000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRL_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRL_MASK (0x00200000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRL_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRL_MASK (0x00400000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRL_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_TPTC1_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRL_MASK (0x00800000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRL_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRL_MASK (0x01000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRL_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRL_MASK (0x02000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRL_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRL_MASK (0x04000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRL_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_HSM_TPTC1_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRL_MASK (0x08000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRL_SHIFT (0x0000001BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRL_MASK (0x10000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRL_SHIFT (0x0000001CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_ICSSM0_PDSP1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_OSPI_VBUSM_ERRL_MASK (0x20000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_OSPI_VBUSM_ERRL_SHIFT (0x0000001DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_OSPI_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_OSPI_VBUSM_ERRL_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_MCRC_VBUSM_ERRL_MASK (0x40000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_MCRC_VBUSM_ERRL_SHIFT (0x0000001EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_MCRC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_MCRC_VBUSM_ERRL_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DTHE_VBUSM_ERRL_MASK (0x80000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DTHE_VBUSM_ERRL_SHIFT (0x0000001FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DTHE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_DTHE_VBUSM_ERRL_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK0_RESETVAL             (0x00000000U)

/* MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRL_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRL_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRL_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRL_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRL_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRL_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRL_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRL_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRL_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5A0_SLV_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRL_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRL_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CR5B0_SLV_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DAP_VBUSM_ERRL_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DAP_VBUSM_ERRL_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DAP_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DAP_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_VBUSM_ERRL_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_VBUSM_ERRL_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CPSW_VBUSM_ERRL_MASK (0x00004000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CPSW_VBUSM_ERRL_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CPSW_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_CPSW_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRL_MASK (0x00008000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRL_SHIFT (0x0000000FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRL_MASK (0x00010000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRL_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRL_MASK (0x00020000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRL_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_L2RAM2_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRL_MASK (0x00080000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRL_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRL_MASK (0x00100000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRL_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRL_MASK (0x00200000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRL_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRL_MASK (0x00400000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRL_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_TPTC1_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRL_MASK (0x00800000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRL_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRL_MASK (0x01000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRL_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRL_MASK (0x02000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRL_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRL_MASK (0x04000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRL_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_HSM_TPTC1_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRL_MASK (0x08000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRL_SHIFT (0x0000001BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRL_MASK (0x10000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRL_SHIFT (0x0000001CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_ICSSM0_PDSP1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_OSPI_VBUSM_ERRL_MASK (0x20000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_OSPI_VBUSM_ERRL_SHIFT (0x0000001DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_OSPI_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_OSPI_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_MCRC_VBUSM_ERRL_MASK (0x40000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_MCRC_VBUSM_ERRL_SHIFT (0x0000001EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_MCRC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_MCRC_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DTHE_VBUSM_ERRL_MASK (0x80000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DTHE_VBUSM_ERRL_SHIFT (0x0000001FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DTHE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_DTHE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS0_RESETVAL           (0x00000000U)

/* MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRL_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRL_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRL_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRL_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRL_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRL_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRL_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRL_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRL_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5A0_SLV_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRL_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRL_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CR5B0_SLV_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRL_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRL_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DAP_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRL_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRL_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRL_MASK (0x00004000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRL_SHIFT (0x0000000EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_CPSW_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRL_MASK (0x00008000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRL_SHIFT (0x0000000FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRL_MASK (0x00010000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRL_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRL_MASK (0x00020000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRL_SHIFT (0x00000011U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_L2RAM2_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRL_MASK (0x00080000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRL_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRL_MASK (0x00100000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRL_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRL_MASK (0x00200000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRL_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRL_MASK (0x00400000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRL_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_TPTC1_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRL_MASK (0x00800000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRL_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRL_MASK (0x01000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRL_SHIFT (0x00000018U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC0_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRL_MASK (0x02000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRL_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_RD_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRL_MASK (0x04000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRL_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_HSM_TPTC1_WR_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRL_MASK (0x08000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRL_SHIFT (0x0000001BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRL_MASK (0x10000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRL_SHIFT (0x0000001CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_ICSSM0_PDSP1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRL_MASK (0x20000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRL_SHIFT (0x0000001DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_OSPI_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRL_MASK (0x40000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRL_SHIFT (0x0000001EU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_MCRC_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRL_MASK (0x80000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRL_SHIFT (0x0000001FU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_DTHE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW0_RESETVAL       (0x00000000U)

/* MSS_VBUSM_SAFETY_L_ERRAGG_MASK1 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRL_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRL_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRL_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_SCRM2SCRP_1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_HSM_S_VBUSM_ERRL_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_HSM_S_VBUSM_ERRL_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_HSM_S_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_HSM_S_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRL_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRL_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM0SLAVE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRL_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRL_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MSS_MBOX_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_STM_STIM_VBUSM_ERRL_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_STM_STIM_VBUSM_ERRL_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_STM_STIM_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_STM_STIM_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MMC_VBUSM_ERRL_MASK  (0x00000040U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MMC_VBUSM_ERRL_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MMC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_MMC_VBUSM_ERRL_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_GPMC_VBUSM_ERRL_MASK (0x00000080U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_GPMC_VBUSM_ERRL_SHIFT (0x00000007U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_GPMC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_GPMC_VBUSM_ERRL_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRL_MASK (0x00000400U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRL_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRL_MASK (0x00000800U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRL_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1_PDSP1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRL_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRL_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_ICSSM1SLAVE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_OSPI1_VBUSM_ERRL_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_OSPI1_VBUSM_ERRL_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_OSPI1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_OSPI1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_MASK1_RESETVAL             (0x00000000U)

/* MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRL_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRL_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRL_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_SCRM2SCRP_1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_HSM_S_VBUSM_ERRL_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_HSM_S_VBUSM_ERRL_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_HSM_S_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_HSM_S_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRL_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRL_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM0SLAVE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRL_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRL_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MSS_MBOX_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRL_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRL_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_STM_STIM_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MMC_VBUSM_ERRL_MASK (0x00000040U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MMC_VBUSM_ERRL_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MMC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_MMC_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_GPMC_VBUSM_ERRL_MASK (0x00000080U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_GPMC_VBUSM_ERRL_SHIFT (0x00000007U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_GPMC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_GPMC_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRL_MASK (0x00000400U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRL_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRL_MASK (0x00000800U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRL_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1_PDSP1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRL_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRL_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_ICSSM1SLAVE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_OSPI1_VBUSM_ERRL_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_OSPI1_VBUSM_ERRL_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_OSPI1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_OSPI1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS1_RESETVAL           (0x00000000U)

/* MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1 */

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRL_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRL_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRL_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRL_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_SCRM2SCRP_1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRL_MASK (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRL_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_HSM_S_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRL_MASK (0x00000008U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRL_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM0SLAVE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRL_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRL_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MSS_MBOX_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRL_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRL_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_STM_STIM_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRL_MASK (0x00000040U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRL_SHIFT (0x00000006U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_MMC_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRL_MASK (0x00000080U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRL_SHIFT (0x00000007U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_GPMC_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRL_MASK (0x00000400U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRL_SHIFT (0x0000000AU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP0_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRL_MASK (0x00000800U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRL_SHIFT (0x0000000BU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1_PDSP1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRL_MASK (0x00001000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRL_SHIFT (0x0000000CU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_ICSSM1SLAVE_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRL_MASK (0x00002000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRL_SHIFT (0x0000000DU)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_OSPI1_VBUSM_ERRL_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSM_SAFETY_L_ERRAGG_STATUS_RAW1_RESETVAL       (0x00000000U)

/* MSS_VBUSP_SAFETY_H_ERRAGG_MASK */

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5A0_AHB_VBUSP_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5A0_AHB_VBUSP_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5A0_AHB_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5A0_AHB_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5B0_AHB_VBUSP_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5B0_AHB_VBUSP_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5B0_AHB_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_CR5B0_AHB_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_MAIN_VBUSP_VBUSP_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_MAIN_VBUSP_VBUSP_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_MAIN_VBUSP_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_MAIN_VBUSP_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_PERI_VBUSP_VBUSP_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_PERI_VBUSP_VBUSP_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_PERI_VBUSP_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_PERI_VBUSP_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_MASK_RESETVAL              (0x00000000U)

/* MSS_VBUSP_SAFETY_H_ERRAGG_STATUS */

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5A0_AHB_VBUSP_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5A0_AHB_VBUSP_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5A0_AHB_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5A0_AHB_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5B0_AHB_VBUSP_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5B0_AHB_VBUSP_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5B0_AHB_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_CR5B0_AHB_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_MAIN_VBUSP_VBUSP_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_MAIN_VBUSP_VBUSP_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_MAIN_VBUSP_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_MAIN_VBUSP_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_PERI_VBUSP_VBUSP_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_PERI_VBUSP_VBUSP_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_PERI_VBUSP_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_PERI_VBUSP_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RESETVAL            (0x00000000U)

/* MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW */

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5A0_AHB_VBUSP_ERRH_MASK (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5A0_AHB_VBUSP_ERRH_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5A0_AHB_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5A0_AHB_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5B0_AHB_VBUSP_ERRH_MASK (0x00000002U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5B0_AHB_VBUSP_ERRH_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5B0_AHB_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_CR5B0_AHB_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_MAIN_VBUSP_VBUSP_ERRH_MASK (0x00000010U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_MAIN_VBUSP_VBUSP_ERRH_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_MAIN_VBUSP_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_MAIN_VBUSP_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_PERI_VBUSP_VBUSP_ERRH_MASK (0x00000020U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_PERI_VBUSP_VBUSP_ERRH_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_PERI_VBUSP_VBUSP_ERRH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_PERI_VBUSP_VBUSP_ERRH_MAX (0x00000001U)

#define CSL_MSS_CTRL_MSS_VBUSP_SAFETY_H_ERRAGG_STATUS_RAW_RESETVAL        (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK    (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_TYPE_MASK         (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_TYPE_SHIFT        (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_TYPE_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_CTRL_RESETVAL          (0x00000007U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK    (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT   (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SEC_MASK            (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SEC_SHIFT           (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SEC_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SEC_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DED_MASK            (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DED_SHIFT           (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DED_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DED_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DATA_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DATA_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DATA_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_DATA_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_MAIN_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_MAIN_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_MAIN_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_MAIN_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SAFE_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SAFE_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SAFE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_SAFE_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_FI_RESETVAL            (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK    (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_SEC_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_SEC_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_SEC_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_SEC_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_DED_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_DED_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_DED_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_DED_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_RESETVAL           (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL  (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK    (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_TYPE_MASK         (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_TYPE_SHIFT        (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_TYPE_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_CTRL_RESETVAL          (0x00000007U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK    (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT   (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SEC_MASK            (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SEC_SHIFT           (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SEC_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SEC_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DED_MASK            (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DED_SHIFT           (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DED_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DED_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DATA_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DATA_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DATA_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_DATA_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_MAIN_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_MAIN_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_MAIN_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_MAIN_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SAFE_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SAFE_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SAFE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_SAFE_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_FI_RESETVAL            (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK    (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_SEC_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_SEC_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_SEC_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_SEC_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_DED_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_DED_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_DED_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_DED_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_RESETVAL           (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL  (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK    (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_TYPE_MASK         (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_TYPE_SHIFT        (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_TYPE_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_CTRL_RESETVAL          (0x00000007U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK    (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT   (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SEC_MASK            (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SEC_SHIFT           (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SEC_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SEC_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DED_MASK            (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DED_SHIFT           (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DED_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DED_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DATA_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DATA_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DATA_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_DATA_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_MAIN_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_MAIN_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_MAIN_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_MAIN_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SAFE_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SAFE_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SAFE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_SAFE_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_FI_RESETVAL            (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK    (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_SEC_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_SEC_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_SEC_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_SEC_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_DED_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_DED_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_DED_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_DED_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_RESETVAL           (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL  (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK    (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_TYPE_MASK         (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_TYPE_SHIFT        (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_TYPE_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_CTRL_RESETVAL          (0x00000007U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK    (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT   (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX     (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SEC_MASK            (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SEC_SHIFT           (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SEC_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SEC_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DED_MASK            (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DED_SHIFT           (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DED_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DED_MAX             (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DATA_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DATA_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DATA_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_DATA_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_MAIN_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_MAIN_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_MAIN_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_MAIN_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SAFE_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SAFE_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SAFE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_SAFE_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_FI_RESETVAL            (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK    (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_SEC_MASK           (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_SEC_SHIFT          (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_SEC_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_SEC_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_DED_MASK           (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_DED_SHIFT          (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_DED_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_DED_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_RESETVAL           (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX  (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL  (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ENABLE_MASK        (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ENABLE_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL    (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ENABLE_MAX         (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK     (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT    (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_TYPE_MASK          (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_TYPE_SHIFT         (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_TYPE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_TYPE_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_CTRL_RESETVAL           (0x00000007U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK     (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX      (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK     (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX      (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX  (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX  (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SEC_MASK             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SEC_SHIFT            (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SEC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SEC_MAX              (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DED_MASK             (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DED_SHIFT            (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DED_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DED_MAX              (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DATA_MASK            (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DATA_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DATA_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_DATA_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_MAIN_MASK            (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_MAIN_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_MAIN_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_MAIN_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SAFE_MASK            (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SAFE_SHIFT           (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SAFE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_SAFE_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_FI_RESETVAL             (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MASK     (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT    (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MAX      (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_SEC_MASK            (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_SEC_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_SEC_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_SEC_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_DED_MASK            (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_DED_SHIFT           (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_DED_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_DED_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_RESETVAL            (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK  (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX   (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK  (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX   (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK  (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX   (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL   (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL  (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ENABLE_MASK        (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ENABLE_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL    (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ENABLE_MAX         (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK     (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT    (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_TYPE_MASK          (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_TYPE_SHIFT         (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_TYPE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_TYPE_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_CTRL_RESETVAL           (0x00000007U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK     (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX      (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK     (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT    (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX      (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX  (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX  (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SEC_MASK             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SEC_SHIFT            (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SEC_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SEC_MAX              (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DED_MASK             (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DED_SHIFT            (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DED_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DED_MAX              (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DATA_MASK            (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DATA_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DATA_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_DATA_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_MAIN_MASK            (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_MAIN_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_MAIN_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_MAIN_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SAFE_MASK            (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SAFE_SHIFT           (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SAFE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_SAFE_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_FI_RESETVAL             (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MASK     (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT    (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MAX      (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_SEC_MASK            (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_SEC_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_SEC_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_SEC_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_DED_MASK            (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_DED_SHIFT           (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_DED_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_DED_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_RESETVAL            (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK  (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX   (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK  (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX   (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK  (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX   (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL   (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL  (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* TPTC00_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ENABLE_MASK                (0x00000007U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL            (0x00000007U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ENABLE_MAX                 (0x00000007U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK             (0x00000100U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_TYPE_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_TYPE_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_TYPE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* TPTC00_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK             (0x00000001U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK             (0x00000002U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK         (0x00000004U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT        (0x00000002U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK         (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT        (0x00000003U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SEC_MASK                     (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SEC_SHIFT                    (0x00000004U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SEC_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DED_MASK                     (0x00000020U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DED_SHIFT                    (0x00000005U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DED_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DATA_MASK                    (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DATA_SHIFT                   (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DATA_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_DATA_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_MAIN_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_MAIN_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_MAIN_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_MAIN_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SAFE_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SAFE_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SAFE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_SAFE_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* TPTC00_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_ERR_MASK               (0x000000FFU)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_ERR_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK             (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_SEC_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_SEC_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_SEC_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_DED_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_DED_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_DED_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK          (0x000000FFU)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* TPTC00_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC00_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* TPTC01_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ENABLE_MASK                (0x00000007U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL            (0x00000007U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ENABLE_MAX                 (0x00000007U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK             (0x00000100U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_TYPE_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_TYPE_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_TYPE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* TPTC01_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK             (0x00000001U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK             (0x00000002U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK         (0x00000004U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT        (0x00000002U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK         (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT        (0x00000003U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SEC_MASK                     (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SEC_SHIFT                    (0x00000004U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SEC_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DED_MASK                     (0x00000020U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DED_SHIFT                    (0x00000005U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DED_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DATA_MASK                    (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DATA_SHIFT                   (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DATA_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_DATA_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_MAIN_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_MAIN_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_MAIN_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_MAIN_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SAFE_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SAFE_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SAFE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_SAFE_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* TPTC01_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_ERR_MASK               (0x000000FFU)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_ERR_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK             (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_SEC_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_SEC_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_SEC_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_DED_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_DED_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_DED_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK          (0x000000FFU)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* TPTC01_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC01_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* TPTC00_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ENABLE_MASK                (0x00000007U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL            (0x00000007U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ENABLE_MAX                 (0x00000007U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK             (0x00000100U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_TYPE_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_TYPE_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_TYPE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* TPTC00_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK             (0x00000001U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK             (0x00000002U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK         (0x00000004U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT        (0x00000002U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK         (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT        (0x00000003U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SEC_MASK                     (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SEC_SHIFT                    (0x00000004U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SEC_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DED_MASK                     (0x00000020U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DED_SHIFT                    (0x00000005U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DED_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DATA_MASK                    (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DATA_SHIFT                   (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DATA_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_DATA_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_MAIN_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_MAIN_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_MAIN_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_MAIN_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SAFE_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SAFE_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SAFE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_SAFE_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* TPTC00_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_ERR_MASK               (0x000000FFU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_ERR_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK             (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_SEC_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_SEC_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_SEC_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_DED_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_DED_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_DED_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK          (0x000000FFU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC00_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* TPTC01_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ENABLE_MASK                (0x00000007U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL            (0x00000007U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ENABLE_MAX                 (0x00000007U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK             (0x00000100U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_TYPE_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_TYPE_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_TYPE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* TPTC01_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK             (0x00000001U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK             (0x00000002U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX              (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK         (0x00000004U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT        (0x00000002U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK         (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT        (0x00000003U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SEC_MASK                     (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SEC_SHIFT                    (0x00000004U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SEC_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DED_MASK                     (0x00000020U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DED_SHIFT                    (0x00000005U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DED_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DATA_MASK                    (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DATA_SHIFT                   (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DATA_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_DATA_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_MAIN_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_MAIN_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_MAIN_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_MAIN_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SAFE_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SAFE_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SAFE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_SAFE_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* TPTC01_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_ERR_MASK               (0x000000FFU)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_ERR_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK             (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_SEC_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_SEC_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_SEC_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_DED_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_DED_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_DED_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK          (0x000000FFU)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_TPTC01_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* MSS_CPSW_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* DAP_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ENABLE_MASK                      (0x00000007U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ENABLE_SHIFT                     (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ENABLE_RESETVAL                  (0x00000007U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ENABLE_MAX                       (0x00000007U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                   (0x00000100U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                  (0x00000008U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_TYPE_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_TYPE_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_TYPE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_TYPE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* DAP_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                   (0x00000001U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                   (0x00000002U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                  (0x00000001U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK               (0x00000004U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT              (0x00000002U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX                (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK               (0x00000008U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT              (0x00000003U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX                (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SEC_MASK                           (0x00000010U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SEC_SHIFT                          (0x00000004U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SEC_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SEC_MAX                            (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DED_MASK                           (0x00000020U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DED_SHIFT                          (0x00000005U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DED_RESETVAL                       (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DED_MAX                            (0x00000001U)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DATA_MASK                          (0x0000FF00U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DATA_SHIFT                         (0x00000008U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DATA_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_DATA_MAX                           (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_MAIN_MASK                          (0x00FF0000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_MAIN_SHIFT                         (0x00000010U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_MAIN_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_MAIN_MAX                           (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SAFE_MASK                          (0xFF000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SAFE_SHIFT                         (0x00000018U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SAFE_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_SAFE_MAX                           (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* DAP_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_ERR_MASK                     (0x000000FFU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_ERR_SHIFT                    (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_ERR_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_ERR_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_CHECK_MASK                   (0x0000FF00U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                  (0x00000008U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_COMP_CHECK_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_SEC_MASK                          (0x00FF0000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_SEC_SHIFT                         (0x00000010U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_SEC_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_SEC_MAX                           (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_DED_MASK                          (0xFF000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_DED_SHIFT                         (0x00000018U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_DED_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_DED_MAX                           (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* DAP_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK                (0x0000FF00U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* DAP_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK                (0xFFFFFFFFU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX                 (0xFFFFFFFFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* DAP_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* DAP_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_READ_STAT_MASK               (0xFFFFFFFFU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_READ_STAT_MAX                (0xFFFFFFFFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* DAP_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_DAP_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ENABLE_MASK            (0x00000007U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ENABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ENABLE_RESETVAL        (0x00000007U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ENABLE_MAX             (0x00000007U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK         (0x00000100U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_TYPE_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_TYPE_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_TYPE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_TYPE_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_CTRL_RESETVAL               (0x00000007U)

/* L2OCRAM_BANK0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK         (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK         (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT        (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK     (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT    (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX      (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK     (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT    (0x00000003U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX      (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SEC_MASK                 (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SEC_SHIFT                (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SEC_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DED_MASK                 (0x00000020U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DED_SHIFT                (0x00000005U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DED_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DATA_MASK                (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DATA_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DATA_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_DATA_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_MAIN_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_MAIN_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_MAIN_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_MAIN_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SAFE_MASK                (0xFF000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SAFE_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SAFE_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_SAFE_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_FI_RESETVAL                 (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_ERR_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_ERR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_ERR_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_CHECK_MASK         (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_COMP_CHECK_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_SEC_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_SEC_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_SEC_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_SEC_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_DED_MASK                (0xFF000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_DED_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_DED_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_DED_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_RESETVAL                (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK      (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT     (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL     (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL       (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL     (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_READ_RESETVAL      (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ENABLE_MASK            (0x00000007U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ENABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ENABLE_RESETVAL        (0x00000007U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ENABLE_MAX             (0x00000007U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK         (0x00000100U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_TYPE_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_TYPE_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_TYPE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_TYPE_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_CTRL_RESETVAL               (0x00000007U)

/* L2OCRAM_BANK1_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK         (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK         (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT        (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK     (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT    (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX      (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK     (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT    (0x00000003U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX      (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SEC_MASK                 (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SEC_SHIFT                (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SEC_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DED_MASK                 (0x00000020U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DED_SHIFT                (0x00000005U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DED_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DATA_MASK                (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DATA_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DATA_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_DATA_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_MAIN_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_MAIN_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_MAIN_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_MAIN_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SAFE_MASK                (0xFF000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SAFE_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SAFE_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_SAFE_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_FI_RESETVAL                 (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_ERR_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_ERR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_ERR_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_CHECK_MASK         (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_COMP_CHECK_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_SEC_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_SEC_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_SEC_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_SEC_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_DED_MASK                (0xFF000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_DED_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_DED_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_DED_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_RESETVAL                (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK      (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT     (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL     (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL       (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL     (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_READ_RESETVAL      (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ENABLE_MASK            (0x00000007U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ENABLE_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ENABLE_RESETVAL        (0x00000007U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ENABLE_MAX             (0x00000007U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ERR_CLEAR_MASK         (0x00000100U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_ERR_CLEAR_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_TYPE_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_TYPE_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_TYPE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_TYPE_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_CTRL_RESETVAL               (0x00000007U)

/* L2OCRAM_BANK2_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_MASK         (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_MASK         (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT        (0x00000001U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_MAX          (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK     (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT    (0x00000002U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX      (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK     (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT    (0x00000003U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX      (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SEC_MASK                 (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SEC_SHIFT                (0x00000004U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SEC_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DED_MASK                 (0x00000020U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DED_SHIFT                (0x00000005U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DED_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DATA_MASK                (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DATA_SHIFT               (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DATA_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_DATA_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_MAIN_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_MAIN_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_MAIN_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_MAIN_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SAFE_MASK                (0xFF000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SAFE_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SAFE_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_SAFE_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_FI_RESETVAL                 (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_ERR_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_ERR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_ERR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_ERR_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_CHECK_MASK         (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_CHECK_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_COMP_CHECK_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_SEC_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_SEC_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_SEC_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_SEC_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_DED_MASK                (0xFF000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_DED_SHIFT               (0x00000018U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_DED_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_DED_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_RESETVAL                (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK      (0x000000FFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK      (0x0000FF00U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT     (0x00000008U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX       (0x000000FFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL     (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_CMD_RESETVAL       (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL     (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_READ_RESETVAL      (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ENABLE_MASK                (0x00000007U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ENABLE_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ENABLE_RESETVAL            (0x00000007U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ENABLE_MAX                 (0x00000007U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ERR_CLEAR_MASK             (0x00000100U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_ERR_CLEAR_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_TYPE_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_TYPE_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_TYPE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_TYPE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MBOX_SRAM_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_MASK             (0x00000001U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_MASK             (0x00000002U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT            (0x00000001U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK         (0x00000004U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT        (0x00000002U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK         (0x00000008U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT        (0x00000003U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX          (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SEC_MASK                     (0x00000010U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SEC_SHIFT                    (0x00000004U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SEC_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DED_MASK                     (0x00000020U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DED_SHIFT                    (0x00000005U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DED_MAX                      (0x00000001U)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DATA_MASK                    (0x0000FF00U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DATA_SHIFT                   (0x00000008U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DATA_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_DATA_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_MAIN_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_MAIN_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_MAIN_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_MAIN_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SAFE_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SAFE_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SAFE_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_SAFE_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_ERR_MASK               (0x000000FFU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_ERR_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_ERR_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_ERR_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_CHECK_MASK             (0x0000FF00U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_CHECK_SHIFT            (0x00000008U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_COMP_CHECK_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_SEC_MASK                    (0x00FF0000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_SEC_SHIFT                   (0x00000010U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_SEC_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_DED_MASK                    (0xFF000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_DED_SHIFT                   (0x00000018U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_DED_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK          (0x000000FFU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* STM_STIM_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* STM_STIM_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* STM_STIM_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* STM_STIM_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* STM_STIM_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* STM_STIM_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* STM_STIM_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_READ_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_READ_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_STM_STIM_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* HSM_TPTC0_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* HSM_TPTC0_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* HSM_TPTC0_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL       (0x00000000U)

/* HSM_TPTC1_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* HSM_TPTC1_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* HSM_TPTC1_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC1_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL       (0x00000000U)

/* HSM_TPTC0_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* HSM_TPTC0_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* HSM_TPTC0_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL      (0x00000000U)

/* HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL  (0x00000000U)

/* HSM_TPTC1_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* HSM_TPTC1_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* HSM_TPTC1_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL      (0x00000000U)

/* HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_TPTC1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL  (0x00000000U)

/* OSPI0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ENABLE_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ENABLE_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ENABLE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_TYPE_MASK                      (0x00FF0000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_TYPE_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_TYPE_MAX                       (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* OSPI0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK             (0x00000004U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK             (0x00000008U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SEC_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SEC_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SEC_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DED_MASK                         (0x00000020U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DED_SHIFT                        (0x00000005U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DED_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DATA_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DATA_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DATA_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_DATA_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_MAIN_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_MAIN_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_MAIN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_MAIN_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SAFE_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SAFE_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SAFE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_SAFE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* OSPI0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_ERR_MASK                   (0x000000FFU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_ERR_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_ERR_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_CHECK_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_COMP_CHECK_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_SEC_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_SEC_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_SEC_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_SEC_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_DED_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_DED_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_DED_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_DED_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* OSPI0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* OSPI0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* OSPI0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK            (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX             (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* OSPI0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* HSM_DTHE_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* MMC0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ENABLE_MASK                     (0x00000007U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ENABLE_SHIFT                    (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ENABLE_RESETVAL                 (0x00000007U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ENABLE_MAX                      (0x00000007U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                  (0x00000100U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                 (0x00000008U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_TYPE_MASK                       (0x00FF0000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_TYPE_SHIFT                      (0x00000010U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_TYPE_RESETVAL                   (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_TYPE_MAX                        (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_CTRL_RESETVAL                        (0x00000007U)

/* MMC0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                  (0x00000001U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                 (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                  (0x00000002U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                 (0x00000001U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK              (0x00000004U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT             (0x00000002U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK              (0x00000008U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT             (0x00000003U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SEC_MASK                          (0x00000010U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SEC_SHIFT                         (0x00000004U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SEC_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SEC_MAX                           (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DED_MASK                          (0x00000020U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DED_SHIFT                         (0x00000005U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DED_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DED_MAX                           (0x00000001U)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DATA_MASK                         (0x0000FF00U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DATA_SHIFT                        (0x00000008U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DATA_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_DATA_MAX                          (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_MAIN_MASK                         (0x00FF0000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_MAIN_SHIFT                        (0x00000010U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_MAIN_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_MAIN_MAX                          (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SAFE_MASK                         (0xFF000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SAFE_SHIFT                        (0x00000018U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SAFE_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_SAFE_MAX                          (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_FI_RESETVAL                          (0x00000000U)

/* MMC0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_ERR_MASK                    (0x000000FFU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_ERR_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_ERR_MAX                     (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_CHECK_MASK                  (0x0000FF00U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                 (0x00000008U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_COMP_CHECK_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_SEC_MASK                         (0x00FF0000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_SEC_SHIFT                        (0x00000010U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_SEC_MAX                          (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_DED_MASK                         (0xFF000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_DED_SHIFT                        (0x00000018U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_DED_MAX                          (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_RESETVAL                         (0x00000000U)

/* MMC0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK               (0x000000FFU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK               (0x0000FF00U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL              (0x00000000U)

/* MMC0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK               (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX                (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                (0x00000000U)

/* MMC0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL              (0x00000000U)

/* MMC0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_READ_RESETVAL               (0x00000000U)

/* MMC0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MMC0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL          (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ENABLE_MASK              (0x00000007U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ENABLE_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ENABLE_RESETVAL          (0x00000007U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ENABLE_MAX               (0x00000007U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK           (0x00000100U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX            (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_TYPE_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_TYPE_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_TYPE_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_TYPE_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_CTRL_RESETVAL                 (0x00000007U)

/* SCRM2SCRP_0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK           (0x00000001U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX            (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK           (0x00000002U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT          (0x00000001U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX            (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK       (0x00000004U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT      (0x00000002U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX        (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK       (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT      (0x00000003U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX        (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SEC_MASK                   (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SEC_SHIFT                  (0x00000004U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SEC_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SEC_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DED_MASK                   (0x00000020U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DED_SHIFT                  (0x00000005U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DED_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DED_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DATA_MASK                  (0x0000FF00U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DATA_SHIFT                 (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DATA_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_DATA_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_MAIN_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_MAIN_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_MAIN_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_MAIN_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SAFE_MASK                  (0xFF000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SAFE_SHIFT                 (0x00000018U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SAFE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_SAFE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_FI_RESETVAL                   (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_ERR_MASK             (0x000000FFU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_ERR_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_ERR_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_CHECK_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_COMP_CHECK_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_SEC_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_SEC_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_SEC_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_DED_MASK                  (0xFF000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_DED_SHIFT                 (0x00000018U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_DED_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_RESETVAL                  (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK        (0x000000FFU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX         (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK        (0x0000FF00U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT       (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX         (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL       (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL         (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL       (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_READ_RESETVAL        (0x00000000U)

/* SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK  (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX   (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL   (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ENABLE_MASK              (0x00000007U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ENABLE_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ENABLE_RESETVAL          (0x00000007U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ENABLE_MAX               (0x00000007U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK           (0x00000100U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX            (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_TYPE_MASK                (0x00FF0000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_TYPE_SHIFT               (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_TYPE_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_TYPE_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_CTRL_RESETVAL                 (0x00000007U)

/* SCRM2SCRP_1_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK           (0x00000001U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX            (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK           (0x00000002U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT          (0x00000001U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX            (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK       (0x00000004U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT      (0x00000002U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX        (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK       (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT      (0x00000003U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX        (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SEC_MASK                   (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SEC_SHIFT                  (0x00000004U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SEC_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SEC_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DED_MASK                   (0x00000020U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DED_SHIFT                  (0x00000005U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DED_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DED_MAX                    (0x00000001U)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DATA_MASK                  (0x0000FF00U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DATA_SHIFT                 (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DATA_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_DATA_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_MAIN_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_MAIN_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_MAIN_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_MAIN_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SAFE_MASK                  (0xFF000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SAFE_SHIFT                 (0x00000018U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SAFE_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_SAFE_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_FI_RESETVAL                   (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_ERR_MASK             (0x000000FFU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_ERR_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_ERR_MAX              (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_CHECK_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_COMP_CHECK_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_SEC_MASK                  (0x00FF0000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_SEC_SHIFT                 (0x00000010U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_SEC_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_DED_MASK                  (0xFF000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_DED_SHIFT                 (0x00000018U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_DED_MAX                   (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_RESETVAL                  (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK        (0x000000FFU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX         (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK        (0x0000FF00U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT       (0x00000008U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX         (0x000000FFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL       (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL         (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL       (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_READ_RESETVAL        (0x00000000U)

/* SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK  (0xFFFFFFFFU)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX   (0xFFFFFFFFU)

#define CSL_MSS_CTRL_SCRM2SCRP_1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL   (0x00000000U)

/* MCRC0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ENABLE_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ENABLE_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ENABLE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_TYPE_MASK                      (0x00FF0000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_TYPE_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_TYPE_MAX                       (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* MCRC0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK             (0x00000004U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK             (0x00000008U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SEC_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SEC_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SEC_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DED_MASK                         (0x00000020U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DED_SHIFT                        (0x00000005U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DED_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DATA_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DATA_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DATA_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_DATA_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_MAIN_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_MAIN_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_MAIN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_MAIN_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SAFE_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SAFE_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SAFE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_SAFE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* MCRC0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_ERR_MASK                   (0x000000FFU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_ERR_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_ERR_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_CHECK_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_COMP_CHECK_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_SEC_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_SEC_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_SEC_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_SEC_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_DED_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_DED_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_DED_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_DED_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* MCRC0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* MCRC0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* MCRC0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK            (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX             (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* MCRC0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MCRC0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ENABLE_MASK          (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ENABLE_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ENABLE_RESETVAL      (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ENABLE_MAX           (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MASK       (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MAX        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_TYPE_MASK            (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_TYPE_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_TYPE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_TYPE_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_CTRL_RESETVAL             (0x00000007U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MASK       (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MAX        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MASK       (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT      (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MAX        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK   (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT  (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX    (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT  (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX    (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SEC_MASK               (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SEC_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SEC_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SEC_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DED_MASK               (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DED_SHIFT              (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DED_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DED_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DATA_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DATA_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DATA_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_DATA_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_MAIN_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_MAIN_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_MAIN_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SAFE_MASK              (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SAFE_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_SAFE_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_FI_RESETVAL               (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_ERR_MASK         (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_ERR_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_ERR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_ERR_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_CHECK_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_CHECK_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_COMP_CHECK_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_SEC_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_SEC_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_SEC_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_SEC_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_DED_MASK              (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_DED_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_DED_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_DED_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_RESETVAL              (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK    (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK    (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL   (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_CMD_RESETVAL     (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK  (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX   (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL   (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MASK   (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MAX    (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_READ_RESETVAL    (0x00000000U)

/* R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ENABLE_MASK          (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ENABLE_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ENABLE_RESETVAL      (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ENABLE_MAX           (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MASK       (0x00000100U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MAX        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_TYPE_MASK            (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_TYPE_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_TYPE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_TYPE_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_CTRL_RESETVAL             (0x00000007U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MASK       (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MAX        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MASK       (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT      (0x00000001U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MAX        (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK   (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT  (0x00000002U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX    (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT  (0x00000003U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX    (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SEC_MASK               (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SEC_SHIFT              (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SEC_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SEC_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DED_MASK               (0x00000020U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DED_SHIFT              (0x00000005U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DED_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DED_MAX                (0x00000001U)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DATA_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DATA_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DATA_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_DATA_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_MAIN_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_MAIN_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_MAIN_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SAFE_MASK              (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SAFE_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_SAFE_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_FI_RESETVAL               (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_ERR_MASK         (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_ERR_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_ERR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_ERR_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_CHECK_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_CHECK_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_COMP_CHECK_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_SEC_MASK              (0x00FF0000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_SEC_SHIFT             (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_SEC_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_SEC_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_DED_MASK              (0xFF000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_DED_SHIFT             (0x00000018U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_DED_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_DED_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_RESETVAL              (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK    (0x000000FFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK    (0x0000FF00U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT   (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX     (0x000000FFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL   (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK    (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT   (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX     (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_CMD_RESETVAL     (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK  (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX   (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL   (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MASK   (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MAX    (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_READ_RESETVAL    (0x00000000U)

/* R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL (0x00000000U)

/* HSM_M_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_MASK                      (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_MAX                       (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* HSM_M_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK             (0x00000004U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK             (0x00000008U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SEC_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SEC_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SEC_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DED_MASK                         (0x00000020U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DED_SHIFT                        (0x00000005U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DED_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DATA_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DATA_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DATA_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_DATA_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_MAIN_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_MAIN_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_MAIN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_MAIN_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SAFE_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SAFE_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SAFE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_SAFE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_ERR_MASK                   (0x000000FFU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_ERR_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_ERR_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_SEC_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_SEC_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_SEC_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_SEC_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_DED_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_DED_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_DED_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_DED_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK            (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX             (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* HSM_S_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_MASK                      (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_MAX                       (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* HSM_S_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK             (0x00000004U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK             (0x00000008U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SEC_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SEC_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SEC_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DED_MASK                         (0x00000020U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DED_SHIFT                        (0x00000005U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DED_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DATA_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DATA_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DATA_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_DATA_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_MAIN_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_MAIN_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_MAIN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_MAIN_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SAFE_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SAFE_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SAFE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_SAFE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_ERR_MASK                   (0x000000FFU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_ERR_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_SEC_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_SEC_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_SEC_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_SEC_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_DED_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_DED_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_DED_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_DED_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK            (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX             (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* ICSSM0_S_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* ICSSM0_PDSP0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL      (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_READ_RESETVAL       (0x00000000U)

/* ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL  (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* ICSSM0_PDSP1_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL      (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_READ_RESETVAL       (0x00000000U)

/* ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM0_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL  (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* ICSSM1_PDSP0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL      (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_READ_RESETVAL       (0x00000000U)

/* ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL  (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ENABLE_MASK             (0x00000007U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ENABLE_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ENABLE_RESETVAL         (0x00000007U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ENABLE_MAX              (0x00000007U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK          (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_TYPE_MASK               (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_TYPE_SHIFT              (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_TYPE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_TYPE_MAX                (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_CTRL_RESETVAL                (0x00000007U)

/* ICSSM1_PDSP1_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK          (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK          (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT         (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK      (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SEC_MASK                  (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SEC_SHIFT                 (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SEC_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SEC_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DED_MASK                  (0x00000020U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DED_SHIFT                 (0x00000005U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DED_RESETVAL              (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DED_MAX                   (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DATA_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DATA_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DATA_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_DATA_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_MAIN_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_MAIN_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_MAIN_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SAFE_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SAFE_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_SAFE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_FI_RESETVAL                  (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_ERR_MASK            (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_ERR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_ERR_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_MASK          (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT         (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_COMP_CHECK_MAX           (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_SEC_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_SEC_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_SEC_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_SEC_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_DED_MASK                 (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_DED_SHIFT                (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_DED_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_DED_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_RESETVAL                 (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK       (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK       (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX        (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL      (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL        (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL      (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK      (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX       (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_READ_RESETVAL       (0x00000000U)

/* ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_PDSP1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL  (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* ICSSM1_S_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_ICSSM1_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* USBSS_RD_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* USBSS_RD_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* USBSS_RD_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* USBSS_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* USBSS_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK          (0xFFFFFFFFU)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX           (0xFFFFFFFFU)

#define CSL_MSS_CTRL_USBSS_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* USBSS_WR_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ENABLE_MASK                 (0x00000007U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL             (0x00000007U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ENABLE_MAX                  (0x00000007U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK              (0x00000100U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX               (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_TYPE_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_TYPE_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_TYPE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_CTRL_RESETVAL                    (0x00000007U)

/* USBSS_WR_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK              (0x00000002U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT             (0x00000001U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX               (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK          (0x00000004U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT         (0x00000002U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK          (0x00000008U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT         (0x00000003U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX           (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SEC_MASK                      (0x00000010U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SEC_SHIFT                     (0x00000004U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SEC_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SEC_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DED_MASK                      (0x00000020U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DED_SHIFT                     (0x00000005U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DED_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DED_MAX                       (0x00000001U)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DATA_MASK                     (0x0000FF00U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DATA_SHIFT                    (0x00000008U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DATA_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_DATA_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_MAIN_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_MAIN_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_MAIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_MAIN_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SAFE_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SAFE_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SAFE_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_SAFE_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* USBSS_WR_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_ERR_MASK                (0x000000FFU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT               (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_ERR_MAX                 (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_SEC_MASK                     (0x00FF0000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_SEC_SHIFT                    (0x00000010U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_SEC_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_DED_MASK                     (0xFF000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_DED_SHIFT                    (0x00000018U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_DED_MAX                      (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK           (0x000000FFU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK           (0x0000FF00U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT          (0x00000008U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX            (0x000000FFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* USBSS_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT    (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_CTRL_USBSS_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* GPMC0_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ENABLE_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ENABLE_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ENABLE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_TYPE_MASK                      (0x00FF0000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_TYPE_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_TYPE_MAX                       (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* GPMC0_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK             (0x00000004U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK             (0x00000008U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SEC_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SEC_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SEC_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DED_MASK                         (0x00000020U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DED_SHIFT                        (0x00000005U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DED_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DATA_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DATA_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DATA_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_DATA_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_MAIN_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_MAIN_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_MAIN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_MAIN_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SAFE_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SAFE_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SAFE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_SAFE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* GPMC0_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_ERR_MASK                   (0x000000FFU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_ERR_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_ERR_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_CHECK_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_COMP_CHECK_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_SEC_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_SEC_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_SEC_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_SEC_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_DED_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_DED_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_DED_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_DED_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* GPMC0_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* GPMC0_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* GPMC0_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK            (0xFFFFFFFFU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX             (0xFFFFFFFFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* GPMC0_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_GPMC0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_RD_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_WR_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_AXI_S_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL  (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK0_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL  (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK1_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD_STAT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_CMD_RESETVAL  (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITE_RESETVAL (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_READ_RESETVAL (0x00000000U)

/* L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP */

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_MSS_CTRL_L2OCRAM_BANK2_BUS_SAFETY_ERR_STAT_BRDG_WRITERESP_RESETVAL (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ENABLE_MASK               (0x00000007U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ENABLE_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ENABLE_RESETVAL           (0x00000007U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ENABLE_MAX                (0x00000007U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_MASK            (0x00000100U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_TYPE_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_TYPE_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_TYPE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_TYPE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_CTRL_RESETVAL                  (0x00000007U)

/* MAIN_VBUSP_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_MASK            (0x00000001U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_MASK            (0x00000002U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT           (0x00000001U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK        (0x00000004U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT       (0x00000002U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK        (0x00000008U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT       (0x00000003U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SEC_MASK                    (0x00000010U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SEC_SHIFT                   (0x00000004U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SEC_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DED_MASK                    (0x00000020U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DED_SHIFT                   (0x00000005U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DED_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DATA_MASK                   (0x0000FF00U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DATA_SHIFT                  (0x00000008U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DATA_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_DATA_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_MAIN_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_MAIN_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_MAIN_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_MAIN_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SAFE_MASK                   (0xFF000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SAFE_SHIFT                  (0x00000018U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SAFE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_SAFE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_FI_RESETVAL                    (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_ERR_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_ERR_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_ERR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_ERR_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_MASK            (0x0000FF00U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_SEC_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_SEC_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_SEC_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_SEC_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_DED_MASK                   (0xFF000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_DED_SHIFT                  (0x00000018U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_DED_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_DED_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_RESETVAL                   (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK         (0x000000FFU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK         (0x0000FF00U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL        (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_CMD_RESETVAL          (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL        (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_READ_RESETVAL         (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK   (0xFFFFFFFFU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT  (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX    (0xFFFFFFFFU)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL    (0x00000000U)

/* MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS */

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_0_MASK   (0x00000001U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_0_SHIFT  (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_0_MAX    (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_1_MASK   (0x00000002U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_1_SHIFT  (0x00000001U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_R5FSS_0_1_MAX    (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP0_MASK  (0x00000010U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP0_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP0_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP1_MASK  (0x00000020U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP1_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SCRM2SCRP1_MAX   (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G0_MASK     (0x00000040U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G0_SHIFT    (0x00000006U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G0_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G1_MASK     (0x00000080U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G1_SHIFT    (0x00000007U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_EPWM_G1_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_0_MASK       (0x00000400U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_0_SHIFT      (0x0000000AU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_0_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_1_MASK       (0x00000800U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_1_SHIFT      (0x0000000BU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_1_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_4_MASK       (0x00004000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_4_SHIFT      (0x0000000EU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_4_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_4_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_5_MASK       (0x00008000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_5_SHIFT      (0x0000000FU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_5_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_ADC_5_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_PERIPH_MASK (0x00010000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_PERIPH_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_PERIPH_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_PERIPH_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_FSI_0_MASK       (0x00020000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_FSI_0_SHIFT      (0x00000011U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_FSI_0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_FSI_0_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG0_MASK (0x00080000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG0_SHIFT (0x00000013U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG0_MAX (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG1_MASK (0x00100000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG1_SHIFT (0x00000014U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MISC_CONFIG1_MAX (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_0_MASK (0x00200000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_0_SHIFT (0x00000015U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_0_MAX (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_1_MASK (0x00400000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_1_SHIFT (0x00000016U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_R5SS_0_1_MAX (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP0_MASK (0x02000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP0_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP0_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP1_MASK (0x04000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP1_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_PERI_VBUSP1_MAX  (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SPINLOCK_MASK    (0x08000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SPINLOCK_SHIFT   (0x0000001BU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SPINLOCK_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_SPINLOCK_MAX     (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_DEBUGSS_MASK     (0x10000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_DEBUGSS_SHIFT    (0x0000001CU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_DEBUGSS_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_DEBUGSS_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MSS_CTRL_MASK    (0x20000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MSS_CTRL_SHIFT   (0x0000001DU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MSS_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_MSS_CTRL_MAX     (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_TOP_CTRL_MASK    (0x40000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_TOP_CTRL_SHIFT   (0x0000001EU)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_TOP_CTRL_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_TOP_CTRL_MAX     (0x00000001U)

#define CSL_MSS_CTRL_MAIN_VBUSP_BUS_SAFETY_EP_ERR_STATUS_RESETVAL         (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ENABLE_MASK               (0x00000007U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ENABLE_SHIFT              (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ENABLE_RESETVAL           (0x00000007U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ENABLE_MAX                (0x00000007U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_MASK            (0x00000100U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_ERR_CLEAR_MAX             (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_TYPE_MASK                 (0x00FF0000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_TYPE_SHIFT                (0x00000010U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_TYPE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_TYPE_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_CTRL_RESETVAL                  (0x00000007U)

/* PERI_VBUSP_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_MASK            (0x00000001U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_MAX             (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_MASK            (0x00000002U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT           (0x00000001U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_MAX             (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK        (0x00000004U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT       (0x00000002U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX         (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK        (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT       (0x00000003U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX         (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SEC_MASK                    (0x00000010U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SEC_SHIFT                   (0x00000004U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SEC_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SEC_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DED_MASK                    (0x00000020U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DED_SHIFT                   (0x00000005U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DED_RESETVAL                (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DED_MAX                     (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DATA_MASK                   (0x0000FF00U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DATA_SHIFT                  (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DATA_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_DATA_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_MAIN_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_MAIN_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_MAIN_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_MAIN_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SAFE_MASK                   (0xFF000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SAFE_SHIFT                  (0x00000018U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SAFE_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_SAFE_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_FI_RESETVAL                    (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_ERR_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_ERR_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_ERR_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_ERR_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_MASK            (0x0000FF00U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_COMP_CHECK_MAX             (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_SEC_MASK                   (0x00FF0000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_SEC_SHIFT                  (0x00000010U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_SEC_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_SEC_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_DED_MASK                   (0xFF000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_DED_SHIFT                  (0x00000018U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_DED_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_DED_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_RESETVAL                   (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK         (0x000000FFU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK         (0x0000FF00U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT        (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX          (0x000000FFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL        (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT        (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_CMD_RESETVAL          (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT      (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL        (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_READ_RESETVAL         (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK   (0xFFFFFFFFU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX    (0xFFFFFFFFU)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL    (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_0_MASK (0x00000001U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_0_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_0_MAX  (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_1_MASK (0x00000002U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_1_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_R5FSS_0_1_MAX  (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP0_MASK (0x00000010U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP0_SHIFT (0x00000004U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP0_MAX (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP1_MASK (0x00000020U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP1_SHIFT (0x00000005U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_PERI_VBUSP1_MAX (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO0_MASK     (0x00000040U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO0_SHIFT    (0x00000006U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO0_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO1_MASK     (0x00000080U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO1_SHIFT    (0x00000007U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_GPIO1_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT0_MASK      (0x00000400U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT0_SHIFT     (0x0000000AU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT0_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT1_MASK      (0x00000800U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT1_SHIFT     (0x0000000BU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_WDT1_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO0_MASK     (0x00004000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO0_SHIFT    (0x0000000EU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO0_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO1_MASK     (0x00008000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO1_SHIFT    (0x0000000FU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO1_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO2_MASK     (0x00010000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO2_SHIFT    (0x00000010U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO2_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO2_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO3_MASK     (0x00020000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO3_SHIFT    (0x00000011U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO3_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_SPIO3_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART0_MASK     (0x00400000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART0_SHIFT    (0x00000016U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART0_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART1_MASK     (0x00800000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART1_SHIFT    (0x00000017U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART1_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART2_MASK     (0x01000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART2_SHIFT    (0x00000018U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART2_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART2_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART3_MASK     (0x02000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART3_SHIFT    (0x00000019U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART3_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART3_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART4_MASK     (0x04000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART4_SHIFT    (0x0000001AU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART4_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART4_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART5_MASK     (0x08000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART5_SHIFT    (0x0000001BU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART5_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_UART5_MAX      (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN0_MASK      (0x10000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN0_SHIFT     (0x0000001CU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN0_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN1_MASK      (0x20000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN1_SHIFT     (0x0000001DU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN1_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN2_MASK      (0x40000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN2_SHIFT     (0x0000001EU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN2_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_LIN2_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_L_RESETVAL       (0x00000000U)

/* PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H */

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C0_MASK      (0x00000002U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C0_SHIFT     (0x00000001U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C0_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C1_MASK      (0x00000004U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C1_SHIFT     (0x00000002U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C1_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C2_MASK      (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C2_SHIFT     (0x00000003U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C2_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_I2C2_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI0_MASK      (0x00000020U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI0_SHIFT     (0x00000005U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI0_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI0_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI1_MASK      (0x00000040U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI1_SHIFT     (0x00000006U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI1_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI1_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI2_MASK      (0x00000080U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI2_SHIFT     (0x00000007U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI2_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI2_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI3_MASK      (0x00000100U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI3_SHIFT     (0x00000008U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI3_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RTI3_MAX       (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD0_MASK    (0x00002000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD0_SHIFT   (0x0000000DU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD0_MAX     (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD1_MASK    (0x00004000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD1_SHIFT   (0x0000000EU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_CANFD1_MAX     (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA0_MASK    (0x00200000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA0_SHIFT   (0x00000015U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA0_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA0_MAX     (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA1_MASK    (0x00400000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA1_SHIFT   (0x00000016U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA1_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_INFRA1_MAX     (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_R5SS0_PERI_MASK (0x00800000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_R5SS0_PERI_SHIFT (0x00000017U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_R5SS0_PERI_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_R5SS0_PERI_MAX (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPP_MASK (0x02000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPP_SHIFT (0x00000019U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPP_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPP_MAX (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPS_MASK (0x04000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPS_SHIFT (0x0000001AU)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPS_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_USBSS_OCPS_MAX (0x00000001U)

#define CSL_MSS_CTRL_PERI_VBUSP_BUS_SAFETY_EP_ERR_STATUS_H_RESETVAL       (0x00000000U)

/* BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_BUS_SAFETY_CTRL_ENABLE_MASK                          (0x00000007U)
#define CSL_MSS_CTRL_BUS_SAFETY_CTRL_ENABLE_SHIFT                         (0x00000000U)
#define CSL_MSS_CTRL_BUS_SAFETY_CTRL_ENABLE_RESETVAL                      (0x00000000U)
#define CSL_MSS_CTRL_BUS_SAFETY_CTRL_ENABLE_MAX                           (0x00000007U)

#define CSL_MSS_CTRL_BUS_SAFETY_CTRL_RESETVAL                             (0x00000000U)

/* MSS_BUS_SAFETY_SEC_ERR_STAT0 */

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_MASK            (0x00000001U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_MASK            (0x00000002U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_SHIFT           (0x00000001U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_MASK            (0x00000004U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_SHIFT           (0x00000002U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_MASK            (0x00000008U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_SHIFT           (0x00000003U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_MAX             (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_MASK           (0x00000010U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_SHIFT          (0x00000004U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_MASK           (0x00000020U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_SHIFT          (0x00000005U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_MASK                (0x00000040U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_SHIFT               (0x00000006U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_MASK                (0x00000080U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_SHIFT               (0x00000007U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_MASK               (0x00000100U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_SHIFT              (0x00000008U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_MAX                (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_MASK     (0x00000200U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_SHIFT    (0x00000009U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_MASK     (0x00000400U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_SHIFT    (0x0000000AU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_MASK     (0x00000800U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_SHIFT    (0x0000000BU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_MASK     (0x00001000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_SHIFT    (0x0000000CU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_MASK     (0x00002000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_SHIFT    (0x0000000DU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_MASK     (0x00004000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_SHIFT    (0x0000000EU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_MASK     (0x00008000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_SHIFT    (0x0000000FU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_MASK     (0x00010000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_SHIFT    (0x00000010U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_MAX      (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_OSPI_MASK               (0x00020000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_OSPI_SHIFT              (0x00000011U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_OSPI_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_OSPI_MAX                (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_MASK               (0x00040000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_SHIFT              (0x00000012U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_MAX                (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_MASK              (0x00080000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_SHIFT             (0x00000013U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_MASK               (0x00100000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_SHIFT              (0x00000014U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_MAX                (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_0_MASK        (0x00200000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_0_SHIFT       (0x00000015U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_0_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_0_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_1_MASK        (0x00400000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_1_SHIFT       (0x00000016U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_1_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_SCRM2SCRP_1_MAX         (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_MASK             (0x00800000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_SHIFT            (0x00000017U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_MASK             (0x01000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_SHIFT            (0x00000018U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM2_MASK             (0x02000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM2_SHIFT            (0x00000019U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM2_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM2_MAX              (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_MASK           (0x08000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_SHIFT          (0x0000001BU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP0_MASK       (0x10000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP0_SHIFT      (0x0000001CU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP0_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP1_MASK       (0x20000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP1_SHIFT      (0x0000001DU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_PDSP1_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_SLAVE_MASK       (0x40000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_SLAVE_SHIFT      (0x0000001EU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_SLAVE_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_ICSSM0_SLAVE_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_RESETVAL                (0x00000000U)

/* MSS_BUS_SAFETY_SEC_ERR_STAT1 */

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_STM_STIM_MASK           (0x00000008U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_STM_STIM_SHIFT          (0x00000003U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_STM_STIM_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_STM_STIM_MAX            (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MMC_MASK                (0x00000010U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MMC_SHIFT               (0x00000004U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MMC_RESETVAL            (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MMC_MAX                 (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_GPMC_MASK               (0x00000020U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_GPMC_SHIFT              (0x00000005U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_GPMC_RESETVAL           (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_GPMC_MAX                (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP0_MASK       (0x00000100U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP0_SHIFT      (0x00000008U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP0_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP0_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP1_MASK       (0x00000200U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP1_SHIFT      (0x00000009U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP1_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_PDSP1_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_SLAVE_MASK       (0x00000400U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_SLAVE_SHIFT      (0x0000000AU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_SLAVE_RESETVAL   (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_ICSSM1_SLAVE_MAX        (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_OSPI1_MASK              (0x00000800U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_OSPI1_SHIFT             (0x0000000BU)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_OSPI1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_OSPI1_MAX               (0x00000001U)

#define CSL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_RESETVAL                (0x00000000U)

/* R5SS0_TCM_ADDRPARITY_CLR */

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM0_ERRADDR_CLR_MASK      (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM0_ERRADDR_CLR_SHIFT     (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM0_ERRADDR_CLR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM0_ERRADDR_CLR_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM1_ERRADDR_CLR_MASK      (0x00000070U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM1_ERRADDR_CLR_SHIFT     (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM1_ERRADDR_CLR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_ATCM1_ERRADDR_CLR_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0TCM0_ERRADDR_CLR_MASK     (0x00000700U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0TCM0_ERRADDR_CLR_SHIFT    (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0TCM0_ERRADDR_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0TCM0_ERRADDR_CLR_MAX      (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0CM1_ERRADDR_CLR_MASK      (0x00007000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0CM1_ERRADDR_CLR_SHIFT     (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0CM1_ERRADDR_CLR_RESETVAL  (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B0CM1_ERRADDR_CLR_MAX       (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM0_ERRADDR_CLR_MASK     (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM0_ERRADDR_CLR_SHIFT    (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM0_ERRADDR_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM0_ERRADDR_CLR_MAX      (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM1_ERRADDR_CLR_MASK     (0x00700000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM1_ERRADDR_CLR_SHIFT    (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM1_ERRADDR_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_B1TCM1_ERRADDR_CLR_MAX      (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_CLR_RESETVAL                    (0x00000000U)

/* R5SS0_CORE0_ADDRPARITY_ERR_ATCM */

#define CSL_MSS_CTRL_R5SS0_CORE0_ADDRPARITY_ERR_ATCM_ADDR_MASK            (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_ADDRPARITY_ERR_ATCM_ADDR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ADDRPARITY_ERR_ATCM_ADDR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ADDRPARITY_ERR_ATCM_ADDR_MAX             (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_ADDRPARITY_ERR_ATCM_RESETVAL             (0x00000000U)

/* R5SS0_CORE1_ADDRPARITY_ERR_ATCM */

#define CSL_MSS_CTRL_R5SS0_CORE1_ADDRPARITY_ERR_ATCM_ADDR_MASK            (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_ADDRPARITY_ERR_ATCM_ADDR_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ADDRPARITY_ERR_ATCM_ADDR_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ADDRPARITY_ERR_ATCM_ADDR_MAX             (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_ADDRPARITY_ERR_ATCM_RESETVAL             (0x00000000U)

/* R5SS0_CORE0_ERR_ADDRPARITY_B0TCM */

#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B0TCM_ADDR_MASK           (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B0TCM_ADDR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B0TCM_ADDR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B0TCM_ADDR_MAX            (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B0TCM_RESETVAL            (0x00000000U)

/* R5SS0_CORE1_ERR_ADDRPARITY_B0TCM */

#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B0TCM_ADDR_MASK           (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B0TCM_ADDR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B0TCM_ADDR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B0TCM_ADDR_MAX            (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B0TCM_RESETVAL            (0x00000000U)

/* R5SS0_CORE0_ERR_ADDRPARITY_B1TCM */

#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B1TCM_ADDR_MASK           (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B1TCM_ADDR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B1TCM_ADDR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B1TCM_ADDR_MAX            (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE0_ERR_ADDRPARITY_B1TCM_RESETVAL            (0x00000000U)

/* R5SS0_CORE1_ERR_ADDRPARITY_B1TCM */

#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B1TCM_ADDR_MASK           (0x000FFFFFU)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B1TCM_ADDR_SHIFT          (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B1TCM_ADDR_RESETVAL       (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B1TCM_ADDR_MAX            (0x000FFFFFU)

#define CSL_MSS_CTRL_R5SS0_CORE1_ERR_ADDRPARITY_B1TCM_RESETVAL            (0x00000000U)

/* R5SS0_TCM_ADDRPARITY_ERRFORCE */

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM0_MASK             (0x00000007U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM0_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM0_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM0_MAX              (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM1_MASK             (0x00000070U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM1_SHIFT            (0x00000004U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM1_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_ATCM1_MAX              (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM0_MASK            (0x00000700U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM0_SHIFT           (0x00000008U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM0_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM0_MAX             (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM1_MASK            (0x00007000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM1_SHIFT           (0x0000000CU)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM1_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B0TCM1_MAX             (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM0_MASK            (0x00070000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM0_SHIFT           (0x00000010U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM0_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM0_MAX             (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM1_MASK            (0x00700000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM1_SHIFT           (0x00000014U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM1_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_B1TCM1_MAX             (0x00000007U)

#define CSL_MSS_CTRL_R5SS0_TCM_ADDRPARITY_ERRFORCE_RESETVAL               (0x00000000U)

/* TPCC0_PARITY_CTRL */

#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_EN_MASK              (0x00000001U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_EN_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_EN_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_EN_MAX               (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_TESTEN_MASK          (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_TESTEN_SHIFT         (0x00000004U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_TESTEN_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_TESTEN_MAX           (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_MASK         (0x00010000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_SHIFT        (0x00000010U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_RESETVAL     (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_MAX          (0x00000001U)

#define CSL_MSS_CTRL_TPCC0_PARITY_CTRL_RESETVAL                           (0x00000000U)

/* TPCC0_PARITY_STATUS */

#define CSL_MSS_CTRL_TPCC0_PARITY_STATUS_TPCC_A_PARITY_ADDR_MASK          (0x000001FFU)
#define CSL_MSS_CTRL_TPCC0_PARITY_STATUS_TPCC_A_PARITY_ADDR_SHIFT         (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_STATUS_TPCC_A_PARITY_ADDR_RESETVAL      (0x00000000U)
#define CSL_MSS_CTRL_TPCC0_PARITY_STATUS_TPCC_A_PARITY_ADDR_MAX           (0x000001FFU)

#define CSL_MSS_CTRL_TPCC0_PARITY_STATUS_RESETVAL                         (0x00000000U)

/* TMU_R5SS0_CORE0_ROM_PARITY_CTRL */

#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_EN_MASK (0x00000001U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_EN_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_EN_MAX (0x00000001U)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_FORCE_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_FORCE_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_FORCE_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_FORCE_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_ERR_CLR_MASK (0x00010000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_ERR_CLR_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_TMU0_ROM_PARITY_ERR_CLR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_CTRL_RESETVAL             (0x00000000U)

/* TMU_R5SS0_CORE0_ROM_PARITY_STATUS */

#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_STATUS_TMU0_ROM_PARITY_ERR_ADDR_MASK (0x000007FFU)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_STATUS_TMU0_ROM_PARITY_ERR_ADDR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_STATUS_TMU0_ROM_PARITY_ERR_ADDR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_STATUS_TMU0_ROM_PARITY_ERR_ADDR_MAX (0x000007FFU)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE0_ROM_PARITY_STATUS_RESETVAL           (0x00000000U)

/* TMU_R5SS0_CORE1_ROM_PARITY_CTRL */

#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_EN_MASK (0x00000001U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_EN_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_EN_MAX (0x00000001U)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_FORCE_ERR_MASK (0x00000002U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_FORCE_ERR_SHIFT (0x00000001U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_FORCE_ERR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_FORCE_ERR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_ERR_CLR_MASK (0x00010000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_ERR_CLR_SHIFT (0x00000010U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_TMU1_ROM_PARITY_ERR_CLR_MAX (0x00000001U)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_CTRL_RESETVAL             (0x00000000U)

/* TMU_R5SS0_CORE1_ROM_PARITY_STATUS */

#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_STATUS_TMU1_ROM_PARITY_ERR_ADDR_MASK (0x000007FFU)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_STATUS_TMU1_ROM_PARITY_ERR_ADDR_SHIFT (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_STATUS_TMU1_ROM_PARITY_ERR_ADDR_RESETVAL (0x00000000U)
#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_STATUS_TMU1_ROM_PARITY_ERR_ADDR_MAX (0x000007FFU)

#define CSL_MSS_CTRL_TMU_R5SS0_CORE1_ROM_PARITY_STATUS_RESETVAL           (0x00000000U)

/* OSPI1_BUS_SAFETY_CTRL */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ENABLE_MASK                    (0x00000007U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ENABLE_SHIFT                   (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ENABLE_RESETVAL                (0x00000007U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ENABLE_MAX                     (0x00000007U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK                 (0x00000100U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_TYPE_MASK                      (0x00FF0000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_TYPE_SHIFT                     (0x00000010U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_TYPE_MAX                       (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* OSPI1_BUS_SAFETY_FI */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK                 (0x00000001U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT                (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK                 (0x00000002U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT                (0x00000001U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX                  (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK             (0x00000004U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT            (0x00000002U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK             (0x00000008U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT            (0x00000003U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX              (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SEC_MASK                         (0x00000010U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SEC_SHIFT                        (0x00000004U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SEC_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SEC_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DED_MASK                         (0x00000020U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DED_SHIFT                        (0x00000005U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DED_RESETVAL                     (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DED_MAX                          (0x00000001U)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DATA_MASK                        (0x0000FF00U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DATA_SHIFT                       (0x00000008U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DATA_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_DATA_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_MAIN_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_MAIN_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_MAIN_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_MAIN_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SAFE_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SAFE_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SAFE_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_SAFE_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* OSPI1_BUS_SAFETY_ERR */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_ERR_MASK                   (0x000000FFU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_ERR_SHIFT                  (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL               (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_ERR_MAX                    (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_CHECK_MASK                 (0x0000FF00U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT                (0x00000008U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_COMP_CHECK_MAX                  (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_SEC_MASK                        (0x00FF0000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_SEC_SHIFT                       (0x00000010U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_SEC_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_SEC_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_DED_MASK                        (0xFF000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_DED_SHIFT                       (0x00000018U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_DED_RESETVAL                    (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_DED_MAX                         (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* OSPI1_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK              (0x000000FFU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK              (0x0000FF00U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT             (0x00000008U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX               (0x000000FFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* OSPI1_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK              (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT             (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL          (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX               (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* OSPI1_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK            (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT           (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL        (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX             (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* OSPI1_BUS_SAFETY_ERR_STAT_READ */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT            (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL         (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK        (0xFFFFFFFFU)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT       (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL    (0x00000000U)
#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX         (0xFFFFFFFFU)

#define CSL_MSS_CTRL_OSPI1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* NERROR_MASK */

#define CSL_MSS_CTRL_NERROR_MASK_MASK_MASK                                (0x00000007U)
#define CSL_MSS_CTRL_NERROR_MASK_MASK_SHIFT                               (0x00000000U)
#define CSL_MSS_CTRL_NERROR_MASK_MASK_RESETVAL                            (0x00000000U)
#define CSL_MSS_CTRL_NERROR_MASK_MASK_MAX                                 (0x00000007U)

#define CSL_MSS_CTRL_NERROR_MASK_RESETVAL                                 (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
