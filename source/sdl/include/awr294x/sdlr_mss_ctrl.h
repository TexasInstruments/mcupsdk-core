/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : sdlr_mss_ctrl.h
*/
#ifndef SDLR_MSS_CTRL_H_
#define SDLR_MSS_CTRL_H_
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>

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
    volatile uint32_t PID;
    volatile uint32_t MSS_SW_INT;
    volatile uint32_t MSS_CAPEVNT_SEL;
    volatile uint32_t MSS_DMA_REQ_SEL;
    volatile uint32_t MSS_DMA1_REQ_SEL;
    volatile uint32_t MSS_IRQ_REQ_SEL;
    volatile uint32_t MSS_SPI_TRIG_SRC;
    volatile uint32_t MSS_ATCM_MEM_INIT;
    volatile uint32_t MSS_ATCM_MEM_INIT_DONE;
    volatile uint32_t MSS_ATCM_MEM_INIT_STATUS;
    volatile uint32_t MSS_BTCM_MEM_INIT;
    volatile uint32_t MSS_BTCM_MEM_INIT_DONE;
    volatile uint32_t MSS_BTCM_MEM_INIT_STATUS;
    volatile uint32_t MSS_L2_MEM_INIT;
    volatile uint32_t MSS_L2_MEM_INIT_DONE;
    volatile uint32_t MSS_L2_MEM_INIT_STATUS;
    volatile uint32_t MSS_MAILBOX_MEM_INIT;
    volatile uint32_t MSS_MAILBOX_MEM_INIT_DONE;
    volatile uint32_t MSS_MAILBOX_MEM_INIT_STATUS;
    volatile uint32_t MSS_RETRAM_MEM_INIT;
    volatile uint32_t MSS_RETRAM_MEM_INIT_DONE;
    volatile uint32_t MSS_RETRAM_MEM_INIT_STATUS;
    volatile uint32_t MSS_SPIA_MEM_INIT;
    volatile uint32_t MSS_SPIA_MEM_INIT_DONE;
    volatile uint32_t MSS_SPIA_MEM_INIT_STATUS;
    volatile uint32_t MSS_SPIB_MEM_INIT;
    volatile uint32_t MSS_SPIB_MEM_INIT_DONE;
    volatile uint32_t MSS_SPIB_MEM_INIT_STATUS;
    volatile uint32_t MSS_TPCC_MEMINIT_START;
    volatile uint32_t MSS_TPCC_MEMINIT_DONE;
    volatile uint32_t MSS_TPCC_MEMINIT_STATUS;
    volatile uint32_t MSS_GPADC_MEM_INIT;
    volatile uint32_t MSS_GPADC_MEM_INIT_DONE;
    volatile uint32_t MSS_GPADC_MEM_INIT_STATUS;
    volatile uint32_t MSS_SPIA_CFG;
    volatile uint32_t MSS_SPIB_CFG;
    volatile uint32_t MSS_EPWM_CFG;
    volatile uint32_t MSS_GIO_CFG;
    volatile uint32_t MSS_MCAN_FE_SELECT;
    volatile uint32_t HW_SPARE_REG1;
    volatile uint32_t MSS_MCANA_INT_CLR;
    volatile uint32_t MSS_MCANA_INT_MASK;
    volatile uint32_t MSS_MCANA_INT_STAT;
    volatile uint32_t HW_SPARE_REG2;
    volatile uint32_t CCC_ERR_STATUS;
    volatile uint32_t CCCA_CFG0;
    volatile uint32_t CCCA_CFG1;
    volatile uint32_t CCCA_CFG2;
    volatile uint32_t CCCA_CFG3;
    volatile uint32_t CCCA_CNTVAL;
    volatile uint32_t CCCB_CFG0;
    volatile uint32_t CCCB_CFG1;
    volatile uint32_t CCCB_CFG2;
    volatile uint32_t CCCB_CFG3;
    volatile uint32_t CCCB_CNTVAL;
    volatile uint32_t CCC_DCC_COMMON;
    volatile uint32_t R5_GLOBAL_CONFIG;
    volatile uint32_t R5_AHB_EN;
    volatile uint32_t R5A_AHB_BASE;
    volatile uint32_t R5A_AHB_SIZE;
    volatile uint32_t R5B_AHB_BASE;
    volatile uint32_t R5B_AHB_SIZE;
    volatile uint32_t R5_TCM_EXT_ERR_EN;
    volatile uint32_t R5_TCM_ERR_EN;
    volatile uint32_t R5_INIT_TCM;
    volatile uint32_t R5_TCM_ECC_WRENZ_EN;
    volatile uint32_t ESM_GATING0;
    volatile uint32_t ESM_GATING1;
    volatile uint32_t ESM_GATING2;
    volatile uint32_t ESM_GATING3;
    volatile uint32_t ESM_GATING4;
    volatile uint32_t ESM_GATING5;
    volatile uint32_t ESM_GATING6;
    volatile uint32_t ESM_GATING7;
    volatile uint32_t ERR_PARITY_ATCM0;
    volatile uint32_t ERR_PARITY_ATCM1;
    volatile uint32_t ERR_PARITY_B0TCM0;
    volatile uint32_t ERR_PARITY_B0TCM1;
    volatile uint32_t ERR_PARITY_B1TCM0;
    volatile uint32_t ERR_PARITY_B1TCM1;
    volatile uint32_t TCM_PARITY_CTRL;
    volatile uint32_t TCM_PARITY_ERRFRC;
    volatile uint32_t HW_SPARE_REG3;
    volatile uint32_t SPIA_IO_CFG;
    volatile uint32_t SPIB_IO_CFG;
    volatile uint32_t SPI_HOST_IRQ;
    volatile uint32_t TPTC_DBS_CONFIG;
    volatile uint32_t TPCC_PARITY_CTRL;
    volatile uint32_t TPCC_PARITY_STATUS;
    volatile uint32_t MSS_DBG_ACK_CTL0;
    volatile uint32_t MSS_DBG_ACK_CTL1;
    volatile uint32_t CPSW_CONTROL;
    volatile uint32_t MSS_TPCC_A_ERRAGG_MASK;
    volatile uint32_t MSS_TPCC_A_ERRAGG_STATUS;
    volatile uint32_t MSS_TPCC_A_ERRAGG_STATUS_RAW;
    volatile uint32_t MSS_TPCC_A_INTAGG_MASK;
    volatile uint32_t MSS_TPCC_A_INTAGG_STATUS;
    volatile uint32_t MSS_TPCC_A_INTAGG_STATUS_RAW;
    volatile uint32_t MSS_TPCC_B_ERRAGG_MASK;
    volatile uint32_t MSS_TPCC_B_ERRAGG_STATUS;
    volatile uint32_t MSS_TPCC_B_ERRAGG_STATUS_RAW;
    volatile uint32_t MSS_TPCC_B_INTAGG_MASK;
    volatile uint32_t MSS_TPCC_B_INTAGG_STATUS;
    volatile uint32_t MSS_TPCC_B_INTAGG_STATUS_RAW;
    volatile uint32_t MSS_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5A_AXI_RD_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5A_AXI_RD_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5B_AXI_RD_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5B_AXI_RD_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_TPTC_A0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TPTC_A0_RD_BUS_SAFETY_FI;
    volatile uint32_t MSS_TPTC_A0_RD_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_TPTC_A1_RD_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TPTC_A1_RD_BUS_SAFETY_FI;
    volatile uint32_t MSS_TPTC_A1_RD_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_TPTC_B0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TPTC_B0_RD_BUS_SAFETY_FI;
    volatile uint32_t MSS_TPTC_B0_RD_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_FI;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_FI;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_FI;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t HSM_TPTC_A0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC_A0_RD_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC_A0_RD_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t HSM_TPTC_A1_RD_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC_A1_RD_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC_A1_RD_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_FI;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_ERR;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_FI;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_ERR;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_CTRL;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_FI;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_FI;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_FI;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_ERR;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_PCR_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_PCR_BUS_SAFETY_FI;
    volatile uint32_t MSS_PCR_BUS_SAFETY_ERR;
    volatile uint32_t MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_PCR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_PCR_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_FI;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_ERR;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP;
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
    volatile uint32_t DAP_R232_BUS_SAFETY_CTRL;
    volatile uint32_t DAP_R232_BUS_SAFETY_FI;
    volatile uint32_t DAP_R232_BUS_SAFETY_ERR;
    volatile uint32_t DAP_R232_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DAP_R232_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DAP_R232_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DAP_R232_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_FI;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_ERR;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_FI;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_ERR;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_FI;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_ERR;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_FI;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_ERR;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_FI;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_ERR;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_BUS_SAFETY_SEC_ERR_STAT0;
    volatile uint32_t MSS_BUS_SAFETY_SEC_ERR_STAT1;
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint32_t HW_REG4;
    volatile uint32_t HW_REG5;
    volatile uint32_t HW_REG6;
    volatile uint32_t HW_REG7;
    volatile uint32_t MSS_DMM_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_DMM_BUS_SAFETY_FI;
    volatile uint32_t MSS_DMM_BUS_SAFETY_ERR;
    volatile uint32_t MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_DMM_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_DMM_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_FI;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_ERR;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_FI;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_ERR;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_FI;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_ERR;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_CTRL;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_FI;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_ERR;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DMM_CTRL_REG;
    volatile uint32_t MSS_CR5A_MBOX_WRITE_DONE;
    volatile uint32_t MSS_CR5A_MBOX_READ_REQ;
    volatile uint32_t MSS_CR5A_MBOX_READ_DONE;
    volatile uint32_t MSS_CR5B_MBOX_WRITE_DONE;
    volatile uint32_t MSS_CR5B_MBOX_READ_REQ;
    volatile uint32_t MSS_CR5B_MBOX_READ_DONE;
    volatile uint32_t MSS_PBIST_KEY_RST;
    volatile uint32_t MSS_PBIST_REG0;
    volatile uint32_t MSS_PBIST_REG1;
    volatile uint32_t MSS_PBIST_REG2;
    volatile uint32_t MSS_QSPI_CONFIG;
    volatile uint32_t MSS_STC_CONTROL;
    volatile uint32_t MSS_CTI_TRIG_SEL;
    volatile uint32_t MSS_DBGSS_CTI_TRIG_SEL;
    volatile uint32_t MSS_BOOT_INFO_REG0;
    volatile uint32_t MSS_BOOT_INFO_REG1;
    volatile uint32_t MSS_BOOT_INFO_REG2;
    volatile uint32_t MSS_BOOT_INFO_REG3;
    volatile uint32_t MSS_BOOT_INFO_REG4;
    volatile uint32_t MSS_BOOT_INFO_REG5;
    volatile uint32_t MSS_BOOT_INFO_REG6;
    volatile uint32_t MSS_BOOT_INFO_REG7;
    volatile uint32_t MSS_TPTC_ECCAGGR_CLK_CNTRL;
    volatile uint32_t MSS_PERIPH_ERRAGG_MASK0;
    volatile uint32_t MSS_PERIPH_ERRAGG_STATUS0;
    volatile uint32_t MSS_PERIPH_ERRAGG_STATUS_RAW0;
    volatile uint32_t MSS_PERIPH_ERRAGG_MASK1;
    volatile uint32_t MSS_PERIPH_ERRAGG_STATUS1;
    volatile uint32_t MSS_PERIPH_ERRAGG_STATUS_RAW1;
    volatile uint32_t MSS_DMM_EVENT0_REG;
    volatile uint32_t MSS_DMM_EVENT1_REG;
    volatile uint32_t MSS_DMM_EVENT2_REG;
    volatile uint32_t MSS_DMM_EVENT3_REG;
    volatile uint32_t MSS_DMM_EVENT4_REG;
    volatile uint32_t MSS_DMM_EVENT5_REG;
    volatile uint32_t MSS_DMM_EVENT6_REG;
    volatile uint32_t MSS_DMM_EVENT7_REG;
    volatile uint32_t MSS_DMM_EVENT8_REG;
    volatile uint32_t MSS_DMM_EVENT9_REG;
    volatile uint32_t MSS_DMM_EVENT10_REG;
    volatile uint32_t MSS_DMM_EVENT11_REG;
    volatile uint32_t MSS_DMM_EVENT12_REG;
    volatile uint32_t MSS_DMM_EVENT13_REG;
    volatile uint32_t MSS_DMM_EVENT14_REG;
    volatile uint32_t MSS_DMM_EVENT15_REG;
    volatile uint32_t MSS_TPTC_BOUNDARY_CFG;
    volatile uint32_t MSS_TPTC_XID_REORDER_CFG;
    volatile uint32_t GPADC_CTRL;
    volatile uint32_t HW_SYNC_FE_CTRL;
    volatile uint32_t DEBUGSS_CSETB_FLUSH;
    volatile uint32_t ANALOG_WU_STATUS_REG_POLARITY_INV;
    volatile uint32_t ANALOG_CLK_STATUS_REG_POLARITY_INV;
    volatile uint32_t ANALOG_WU_STATUS_REG_GRP1_MASK;
    volatile uint32_t ANALOG_CLK_STATUS_REG_GRP1_MASK;
    volatile uint32_t ANALOG_WU_STATUS_REG_GRP2_MASK;
    volatile uint32_t ANALOG_CLK_STATUS_REG_GRP2_MASK;
    volatile uint32_t NERROR_MASK;
    volatile uint8_t  Resv_1824[64];
    volatile uint32_t MSS_DMM_ACCESS_MODE;
    volatile uint8_t  Resv_2048[220];
    volatile uint32_t R5_CONTROL;
    volatile uint32_t R5_ROM_ECLIPSE;
    volatile uint32_t R5_COREA_HALT;
    volatile uint32_t R5_COREB_HALT;
    volatile uint32_t R5_STATUS_REG;
    volatile uint8_t  Resv_4048[1980];
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint32_t MSS_CR5A_B_MBOX_READ_DONE_ACK;
    volatile uint32_t HW_SPARE_REC;
    volatile uint8_t  Resv_4104[16];
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
} SDL_mss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_MSS_CTRL_PID                                                       (0x00000000U)
#define SDL_MSS_CTRL_MSS_SW_INT                                                (0x00000004U)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL                                           (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMA_REQ_SEL                                           (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMA1_REQ_SEL                                          (0x00000010U)
#define SDL_MSS_CTRL_MSS_IRQ_REQ_SEL                                           (0x00000014U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC                                          (0x00000018U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT                                         (0x0000001CU)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_DONE                                    (0x00000020U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS                                  (0x00000024U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT                                         (0x00000028U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_DONE                                    (0x0000002CU)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS                                  (0x00000030U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT                                           (0x00000034U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE                                      (0x00000038U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS                                    (0x0000003CU)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT                                      (0x00000040U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE                                 (0x00000044U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_STATUS                               (0x00000048U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT                                       (0x0000004CU)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_DONE                                  (0x00000050U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_STATUS                                (0x00000054U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT                                         (0x00000058U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_DONE                                    (0x0000005CU)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_STATUS                                  (0x00000060U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT                                         (0x00000064U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_DONE                                    (0x00000068U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_STATUS                                  (0x0000006CU)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START                                    (0x00000070U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE                                     (0x00000074U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS                                   (0x00000078U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT                                        (0x0000007CU)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_DONE                                   (0x00000080U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_STATUS                                 (0x00000084U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG                                              (0x00000088U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG                                              (0x0000008CU)
#define SDL_MSS_CTRL_MSS_EPWM_CFG                                              (0x00000090U)
#define SDL_MSS_CTRL_MSS_GIO_CFG                                               (0x00000094U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT                                        (0x00000098U)
#define SDL_MSS_CTRL_HW_SPARE_REG1                                             (0x0000009CU)
#define SDL_MSS_CTRL_MSS_MCANA_INT_CLR                                         (0x000000A0U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_MASK                                        (0x000000A4U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_STAT                                        (0x000000A8U)
#define SDL_MSS_CTRL_HW_SPARE_REG2                                             (0x000000ACU)
#define SDL_MSS_CTRL_CCC_ERR_STATUS                                            (0x000000B0U)
#define SDL_MSS_CTRL_CCCA_CFG0                                                 (0x000000B4U)
#define SDL_MSS_CTRL_CCCA_CFG1                                                 (0x000000B8U)
#define SDL_MSS_CTRL_CCCA_CFG2                                                 (0x000000BCU)
#define SDL_MSS_CTRL_CCCA_CFG3                                                 (0x000000C0U)
#define SDL_MSS_CTRL_CCCA_CNTVAL                                               (0x000000C4U)
#define SDL_MSS_CTRL_CCCB_CFG0                                                 (0x000000C8U)
#define SDL_MSS_CTRL_CCCB_CFG1                                                 (0x000000CCU)
#define SDL_MSS_CTRL_CCCB_CFG2                                                 (0x000000D0U)
#define SDL_MSS_CTRL_CCCB_CFG3                                                 (0x000000D4U)
#define SDL_MSS_CTRL_CCCB_CNTVAL                                               (0x000000D8U)
#define SDL_MSS_CTRL_CCC_DCC_COMMON                                            (0x000000DCU)
#define SDL_MSS_CTRL_R5_GLOBAL_CONFIG                                          (0x000000E0U)
#define SDL_MSS_CTRL_R5_AHB_EN                                                 (0x000000E4U)
#define SDL_MSS_CTRL_R5A_AHB_BASE                                              (0x000000E8U)
#define SDL_MSS_CTRL_R5A_AHB_SIZE                                              (0x000000ECU)
#define SDL_MSS_CTRL_R5B_AHB_BASE                                              (0x000000F0U)
#define SDL_MSS_CTRL_R5B_AHB_SIZE                                              (0x000000F4U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN                                         (0x000000F8U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN                                             (0x000000FCU)
#define SDL_MSS_CTRL_R5_INIT_TCM                                               (0x00000100U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN                                       (0x00000104U)
#define SDL_MSS_CTRL_ESM_GATING0                                               (0x00000108U)
#define SDL_MSS_CTRL_ESM_GATING1                                               (0x0000010CU)
#define SDL_MSS_CTRL_ESM_GATING2                                               (0x00000110U)
#define SDL_MSS_CTRL_ESM_GATING3                                               (0x00000114U)
#define SDL_MSS_CTRL_ESM_GATING4                                               (0x00000118U)
#define SDL_MSS_CTRL_ESM_GATING5                                               (0x0000011CU)
#define SDL_MSS_CTRL_ESM_GATING6                                               (0x00000120U)
#define SDL_MSS_CTRL_ESM_GATING7                                               (0x00000124U)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM0                                          (0x00000128U)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM1                                          (0x0000012CU)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM0                                         (0x00000130U)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM1                                         (0x00000134U)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM0                                         (0x00000138U)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM1                                         (0x0000013CU)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL                                           (0x00000140U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC                                         (0x00000144U)
#define SDL_MSS_CTRL_HW_SPARE_REG3                                             (0x00000148U)
#define SDL_MSS_CTRL_SPIA_IO_CFG                                               (0x0000014CU)
#define SDL_MSS_CTRL_SPIB_IO_CFG                                               (0x00000150U)
#define SDL_MSS_CTRL_SPI_HOST_IRQ                                              (0x00000154U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG                                           (0x00000158U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL                                          (0x0000015CU)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS                                        (0x00000160U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0                                          (0x00000164U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1                                          (0x00000168U)
#define SDL_MSS_CTRL_CPSW_CONTROL                                              (0x0000016CU)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK                                    (0x00000170U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS                                  (0x00000174U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW                              (0x00000178U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK                                    (0x0000017CU)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS                                  (0x00000180U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW                              (0x00000184U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK                                    (0x00000188U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS                                  (0x0000018CU)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW                              (0x00000190U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK                                    (0x00000194U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS                                  (0x00000198U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW                              (0x0000019CU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL                                       (0x000001A0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL                           (0x000001A4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI                             (0x000001A8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR                            (0x000001ACU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001B0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x000001B4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x000001B8U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL                           (0x000001BCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI                             (0x000001C0U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR                            (0x000001C4U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001C8U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x000001CCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x000001D0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL                           (0x000001D4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI                             (0x000001D8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR                            (0x000001DCU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001E0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x000001E4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x000001E8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000001ECU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL                           (0x000001F0U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI                             (0x000001F4U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR                            (0x000001F8U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001FCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x00000200U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x00000204U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x00000208U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL                            (0x0000020CU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI                              (0x00000210U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR                             (0x00000214U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000218U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x0000021CU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000220U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00000224U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000228U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL                            (0x0000022CU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI                              (0x00000230U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR                             (0x00000234U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000238U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x0000023CU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000240U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00000244U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000248U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL                            (0x0000024CU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI                              (0x00000250U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR                             (0x00000254U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000258U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x0000025CU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000260U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL                            (0x00000264U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI                              (0x00000268U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR                             (0x0000026CU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000270U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000274U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000278U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL                            (0x0000027CU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI                              (0x00000280U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR                             (0x00000284U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000288U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x0000028CU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000290U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL                            (0x00000294U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI                              (0x00000298U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR                             (0x0000029CU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002A0U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000002A4U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000002A8U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000002ACU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL                            (0x000002B0U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI                              (0x000002B4U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR                             (0x000002B8U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002BCU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000002C0U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000002C4U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000002C8U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL                            (0x000002CCU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI                              (0x000002D0U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR                             (0x000002D4U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002D8U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000002DCU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000002E0U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000002E4U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL                            (0x000002E8U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI                              (0x000002ECU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR                             (0x000002F0U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002F4U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000002F8U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000002FCU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL                            (0x00000300U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI                              (0x00000304U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR                             (0x00000308U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x0000030CU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000310U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000314U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL                            (0x00000318U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI                              (0x0000031CU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR                             (0x00000320U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000324U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000328U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x0000032CU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000330U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL                            (0x00000334U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI                              (0x00000338U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR                             (0x0000033CU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000340U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000344U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000348U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x0000034CU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL                                  (0x00000350U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI                                    (0x00000354U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR                                   (0x00000358U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0                        (0x0000035CU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD                          (0x00000360U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000364U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ                         (0x00000368U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x0000036CU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL                                  (0x00000370U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI                                    (0x00000374U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR                                   (0x00000378U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0                        (0x0000037CU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD                          (0x00000380U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000384U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ                         (0x00000388U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x0000038CU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL                                  (0x00000390U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI                                    (0x00000394U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR                                   (0x00000398U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0                        (0x0000039CU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD                          (0x000003A0U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE                        (0x000003A4U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ                         (0x000003A8U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x000003ACU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL                                  (0x000003B0U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI                                    (0x000003B4U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR                                   (0x000003B8U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0                        (0x000003BCU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD                          (0x000003C0U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE                        (0x000003C4U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ                         (0x000003C8U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x000003CCU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL                                   (0x000003D0U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI                                     (0x000003D4U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR                                    (0x000003D8U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0                         (0x000003DCU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD                           (0x000003E0U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE                         (0x000003E4U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ                          (0x000003E8U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP                     (0x000003ECU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL                                  (0x000003F0U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI                                    (0x000003F4U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR                                   (0x000003F8U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0                        (0x000003FCU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD                          (0x00000400U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000404U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ                         (0x00000408U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x0000040CU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL                                     (0x00000410U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI                                       (0x00000414U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR                                      (0x00000418U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0                           (0x0000041CU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD                             (0x00000420U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE                           (0x00000424U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ                            (0x00000428U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP                       (0x0000042CU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL                                     (0x00000430U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI                                       (0x00000434U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR                                      (0x00000438U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0                           (0x0000043CU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD                             (0x00000440U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE                           (0x00000444U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ                            (0x00000448U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP                       (0x0000044CU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL                                  (0x00000450U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI                                    (0x00000454U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR                                   (0x00000458U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0                        (0x0000045CU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD                          (0x00000460U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000464U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ                         (0x00000468U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x0000046CU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL                                  (0x00000470U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI                                    (0x00000474U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR                                   (0x00000478U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0                        (0x0000047CU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD                          (0x00000480U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000484U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ                         (0x00000488U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x0000048CU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL                                  (0x00000490U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI                                    (0x00000494U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR                                   (0x00000498U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0                        (0x0000049CU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD                          (0x000004A0U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE                        (0x000004A4U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ                         (0x000004A8U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x000004ACU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL                                  (0x000004B0U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI                                    (0x000004B4U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR                                   (0x000004B8U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0                        (0x000004BCU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD                          (0x000004C0U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE                        (0x000004C4U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ                         (0x000004C8U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x000004CCU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL                                 (0x000004D0U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI                                   (0x000004D4U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR                                  (0x000004D8U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0                       (0x000004DCU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD                         (0x000004E0U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE                       (0x000004E4U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ                        (0x000004E8U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x000004ECU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL                                 (0x000004F0U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI                                   (0x000004F4U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR                                  (0x000004F8U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0                       (0x000004FCU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD                         (0x00000500U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE                       (0x00000504U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ                        (0x00000508U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x0000050CU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0                              (0x00000510U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1                              (0x00000514U)
#define SDL_MSS_CTRL_HW_REG0                                                   (0x00000518U)
#define SDL_MSS_CTRL_HW_REG1                                                   (0x0000051CU)
#define SDL_MSS_CTRL_HW_REG2                                                   (0x00000520U)
#define SDL_MSS_CTRL_HW_REG3                                                   (0x00000524U)
#define SDL_MSS_CTRL_HW_REG4                                                   (0x00000528U)
#define SDL_MSS_CTRL_HW_REG5                                                   (0x0000052CU)
#define SDL_MSS_CTRL_HW_REG6                                                   (0x00000530U)
#define SDL_MSS_CTRL_HW_REG7                                                   (0x00000534U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL                                   (0x00000538U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI                                     (0x0000053CU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR                                    (0x00000540U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0                         (0x00000544U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD                           (0x00000548U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE                         (0x0000054CU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ                          (0x00000550U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP                     (0x00000554U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL                               (0x00000558U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI                                 (0x0000055CU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR                                (0x00000560U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0                     (0x00000564U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD                       (0x00000568U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE                     (0x0000056CU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ                      (0x00000570U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP                 (0x00000574U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL                                (0x00000578U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI                                  (0x0000057CU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR                                 (0x00000580U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0                      (0x00000584U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD                        (0x00000588U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE                      (0x0000058CU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ                       (0x00000590U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP                  (0x00000594U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL                                  (0x00000598U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI                                    (0x0000059CU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR                                   (0x000005A0U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0                        (0x000005A4U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD                          (0x000005A8U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE                        (0x000005ACU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ                         (0x000005B0U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x000005B4U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL                              (0x000005B8U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI                                (0x000005BCU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR                               (0x000005C0U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x000005C4U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x000005C8U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x000005CCU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x000005D0U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000005D4U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL                              (0x000005D8U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI                                (0x000005DCU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR                               (0x000005E0U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x000005E4U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x000005E8U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x000005ECU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x000005F0U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000005F4U)
#define SDL_MSS_CTRL_DMM_CTRL_REG                                              (0x000005F8U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE                                  (0x000005FCU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ                                    (0x00000600U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE                                   (0x00000604U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE                                  (0x00000608U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ                                    (0x0000060CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE                                   (0x00000610U)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST                                         (0x00000614U)
#define SDL_MSS_CTRL_MSS_PBIST_REG0                                            (0x00000618U)
#define SDL_MSS_CTRL_MSS_PBIST_REG1                                            (0x0000061CU)
#define SDL_MSS_CTRL_MSS_PBIST_REG2                                            (0x00000620U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG                                           (0x00000624U)
#define SDL_MSS_CTRL_MSS_STC_CONTROL                                           (0x00000628U)
#define SDL_MSS_CTRL_MSS_CTI_TRIG_SEL                                          (0x0000062CU)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL                                    (0x00000630U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG0                                        (0x00000634U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG1                                        (0x00000638U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG2                                        (0x0000063CU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG3                                        (0x00000640U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG4                                        (0x00000644U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG5                                        (0x00000648U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG6                                        (0x0000064CU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG7                                        (0x00000650U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL                                (0x00000654U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0                                   (0x00000658U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0                                 (0x0000065CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0                             (0x00000660U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1                                   (0x00000664U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1                                 (0x00000668U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1                             (0x0000066CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG                                        (0x00000670U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG                                        (0x00000674U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG                                        (0x00000678U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG                                        (0x0000067CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG                                        (0x00000680U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG                                        (0x00000684U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG                                        (0x00000688U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG                                        (0x0000068CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG                                        (0x00000690U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG                                        (0x00000694U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG                                       (0x00000698U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG                                       (0x0000069CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG                                       (0x000006A0U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG                                       (0x000006A4U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG                                       (0x000006A8U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG                                       (0x000006ACU)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG                                     (0x000006B0U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG                                  (0x000006B4U)
#define SDL_MSS_CTRL_GPADC_CTRL                                                (0x000006B8U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL                                           (0x000006BCU)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH                                       (0x000006C0U)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_POLARITY_INV                         (0x000006C4U)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_POLARITY_INV                        (0x000006C8U)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP1_MASK                            (0x000006CCU)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK                           (0x000006D0U)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK                            (0x000006D4U)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP2_MASK                           (0x000006D8U)
#define SDL_MSS_CTRL_NERROR_MASK                                               (0x000006DCU)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE                                       (0x00000720U)
#define SDL_MSS_CTRL_R5_CONTROL                                                (0x00000800U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE                                            (0x00000804U)
#define SDL_MSS_CTRL_R5_COREA_HALT                                             (0x00000808U)
#define SDL_MSS_CTRL_R5_COREB_HALT                                             (0x0000080CU)
#define SDL_MSS_CTRL_R5_STATUS_REG                                             (0x00000810U)
#define SDL_MSS_CTRL_HW_SPARE_RW0                                              (0x00000FD0U)
#define SDL_MSS_CTRL_HW_SPARE_RW1                                              (0x00000FD4U)
#define SDL_MSS_CTRL_HW_SPARE_RW2                                              (0x00000FD8U)
#define SDL_MSS_CTRL_HW_SPARE_RW3                                              (0x00000FDCU)
#define SDL_MSS_CTRL_HW_SPARE_RO0                                              (0x00000FE0U)
#define SDL_MSS_CTRL_HW_SPARE_RO1                                              (0x00000FE4U)
#define SDL_MSS_CTRL_HW_SPARE_RO2                                              (0x00000FE8U)
#define SDL_MSS_CTRL_HW_SPARE_RO3                                              (0x00000FECU)
#define SDL_MSS_CTRL_MSS_CR5A_B_MBOX_READ_DONE_ACK                             (0x00000FF0U)
#define SDL_MSS_CTRL_HW_SPARE_REC                                              (0x00000FF4U)
#define SDL_MSS_CTRL_LOCK0_KICK0                                               (0x00001008U)
#define SDL_MSS_CTRL_LOCK0_KICK1                                               (0x0000100CU)
#define SDL_MSS_CTRL_INTR_RAW_STATUS                                           (0x00001010U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR                                 (0x00001014U)
#define SDL_MSS_CTRL_INTR_ENABLE                                               (0x00001018U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR                                         (0x0000101CU)
#define SDL_MSS_CTRL_EOI                                                       (0x00001020U)
#define SDL_MSS_CTRL_FAULT_ADDRESS                                             (0x00001024U)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS                                         (0x00001028U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS                                         (0x0000102CU)
#define SDL_MSS_CTRL_FAULT_CLEAR                                               (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define SDL_MSS_CTRL_PID_PID_MINOR_MASK                                        (0x0000003FU)
#define SDL_MSS_CTRL_PID_PID_MINOR_SHIFT                                       (0x00000000U)
#define SDL_MSS_CTRL_PID_PID_MINOR_RESETVAL                                    (0x00000014U)
#define SDL_MSS_CTRL_PID_PID_MINOR_MAX                                         (0x0000003FU)

#define SDL_MSS_CTRL_PID_PID_CUSTOM_MASK                                       (0x000000C0U)
#define SDL_MSS_CTRL_PID_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define SDL_MSS_CTRL_PID_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define SDL_MSS_CTRL_PID_PID_CUSTOM_MAX                                        (0x00000003U)

#define SDL_MSS_CTRL_PID_PID_MAJOR_MASK                                        (0x00000700U)
#define SDL_MSS_CTRL_PID_PID_MAJOR_SHIFT                                       (0x00000008U)
#define SDL_MSS_CTRL_PID_PID_MAJOR_RESETVAL                                    (0x00000002U)
#define SDL_MSS_CTRL_PID_PID_MAJOR_MAX                                         (0x00000007U)

#define SDL_MSS_CTRL_PID_PID_MISC_MASK                                         (0x0000F800U)
#define SDL_MSS_CTRL_PID_PID_MISC_SHIFT                                        (0x0000000BU)
#define SDL_MSS_CTRL_PID_PID_MISC_RESETVAL                                     (0x00000000U)
#define SDL_MSS_CTRL_PID_PID_MISC_MAX                                          (0x0000001FU)

#define SDL_MSS_CTRL_PID_PID_MSB16_MASK                                        (0xFFFF0000U)
#define SDL_MSS_CTRL_PID_PID_MSB16_SHIFT                                       (0x00000010U)
#define SDL_MSS_CTRL_PID_PID_MSB16_RESETVAL                                    (0x00006180U)
#define SDL_MSS_CTRL_PID_PID_MSB16_MAX                                         (0x0000FFFFU)

#define SDL_MSS_CTRL_PID_RESETVAL                                              (0x61800214U)

/* MSS_SW_INT */

#define SDL_MSS_CTRL_MSS_SW_INT_MSS_SW_INT_PULSE_MASK                          (0x0000001FU)
#define SDL_MSS_CTRL_MSS_SW_INT_MSS_SW_INT_PULSE_SHIFT                         (0x00000000U)
#define SDL_MSS_CTRL_MSS_SW_INT_MSS_SW_INT_PULSE_RESETVAL                      (0x00000000U)
#define SDL_MSS_CTRL_MSS_SW_INT_MSS_SW_INT_PULSE_MAX                           (0x0000001FU)

#define SDL_MSS_CTRL_MSS_SW_INT_RESETVAL                                       (0x00000000U)

/* MSS_CAPEVNT_SEL */

#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC0_MASK                 (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC0_SHIFT                (0x00000000U)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC0_RESETVAL             (0x00000000U)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC0_MAX                  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC1_MASK                 (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC1_SHIFT                (0x00000008U)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC1_RESETVAL             (0x00000000U)
#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_MSS_CAPEVNT_SEL_SRC1_MAX                  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CAPEVNT_SEL_RESETVAL                                  (0x00000000U)

/* MSS_DMA_REQ_SEL */

#define SDL_MSS_CTRL_MSS_DMA_REQ_SEL_MSS_DMA_REQ_SEL_SELECT_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMA_REQ_SEL_MSS_DMA_REQ_SEL_SELECT_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMA_REQ_SEL_MSS_DMA_REQ_SEL_SELECT_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMA_REQ_SEL_MSS_DMA_REQ_SEL_SELECT_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMA_REQ_SEL_RESETVAL                                  (0x00000000U)

/* MSS_DMA1_REQ_SEL */

#define SDL_MSS_CTRL_MSS_DMA1_REQ_SEL_MSS_DMA1_REQ_SEL_SELECT_MASK             (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMA1_REQ_SEL_MSS_DMA1_REQ_SEL_SELECT_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMA1_REQ_SEL_MSS_DMA1_REQ_SEL_SELECT_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMA1_REQ_SEL_MSS_DMA1_REQ_SEL_SELECT_MAX              (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMA1_REQ_SEL_RESETVAL                                 (0x00000000U)

/* MSS_IRQ_REQ_SEL */

#define SDL_MSS_CTRL_MSS_IRQ_REQ_SEL_MSS_IRQ_REQ_SEL_SELECT_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_IRQ_REQ_SEL_MSS_IRQ_REQ_SEL_SELECT_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_MSS_IRQ_REQ_SEL_MSS_IRQ_REQ_SEL_SELECT_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_IRQ_REQ_SEL_MSS_IRQ_REQ_SEL_SELECT_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_IRQ_REQ_SEL_RESETVAL                                  (0x00000000U)

/* MSS_SPI_TRIG_SRC */

#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIA_MASK          (0x00000003U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIA_SHIFT         (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIA_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIA_MAX           (0x00000003U)

#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIB_MASK          (0x07FF0000U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIB_SHIFT         (0x00000010U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIB_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_MSS_SPI_TRIG_SRC_TRIG_SPIB_MAX           (0x000007FFU)

#define SDL_MSS_CTRL_MSS_SPI_TRIG_SRC_RESETVAL                                 (0x00000000U)

/* MSS_ATCM_MEM_INIT */

#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT_MASK         (0x00000001U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT_MAX          (0x00000001U)

#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_RESETVAL                                (0x00000000U)

/* MSS_ATCM_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_RESETVAL                           (0x00000000U)

/* MSS_ATCM_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_RESETVAL                         (0x00000000U)

/* MSS_BTCM_MEM_INIT */

#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_MSS_BTCM_MEM_INIT_MEM_INIT_MASK         (0x00000001U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_MSS_BTCM_MEM_INIT_MEM_INIT_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_MSS_BTCM_MEM_INIT_MEM_INIT_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_MSS_BTCM_MEM_INIT_MEM_INIT_MAX          (0x00000001U)

#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_RESETVAL                                (0x00000000U)

/* MSS_BTCM_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_RESETVAL                           (0x00000000U)

/* MSS_BTCM_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_RESETVAL                         (0x00000000U)

/* MSS_L2_MEM_INIT */

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION0_MASK           (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION0_SHIFT          (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION0_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION0_MAX            (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION1_MASK           (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION1_SHIFT          (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION1_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_MSS_L2_MEM_INIT_PARTITION1_MAX            (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_RESETVAL                                  (0x00000000U)

/* MSS_L2_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION0_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION1_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION1_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_MSS_L2_MEM_INIT_DONE_PARTITION1_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_DONE_RESETVAL                             (0x00000000U)

/* MSS_L2_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION1_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION1_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_MSS_L2_MEM_INIT_STATUS_PARTITION1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_MEM_INIT_STATUS_RESETVAL                           (0x00000000U)

/* MSS_MAILBOX_MEM_INIT */

#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_MSS_MAILBOX_MEM_INIT_MEM0_INIT_MASK  (0x00000001U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_MSS_MAILBOX_MEM_INIT_MEM0_INIT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_MSS_MAILBOX_MEM_INIT_MEM0_INIT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_MSS_MAILBOX_MEM_INIT_MEM0_INIT_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_RESETVAL                             (0x00000000U)

/* MSS_MAILBOX_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_MSS_MAILBOX_MEM_INIT_DONE_MEM0_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_MSS_MAILBOX_MEM_INIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_MSS_MAILBOX_MEM_INIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_MSS_MAILBOX_MEM_INIT_DONE_MEM0_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_RESETVAL                        (0x00000000U)

/* MSS_MAILBOX_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_STATUS_MSS_MAILBOX_MEM_INIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_STATUS_MSS_MAILBOX_MEM_INIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_STATUS_MSS_MAILBOX_MEM_INIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_STATUS_MSS_MAILBOX_MEM_INIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MAILBOX_MEM_INIT_STATUS_RESETVAL                      (0x00000000U)

/* MSS_RETRAM_MEM_INIT */

#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_MSS_RETRAM_MEM_INIT_MEM0_INIT_MASK    (0x00000001U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_MSS_RETRAM_MEM_INIT_MEM0_INIT_SHIFT   (0x00000000U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_MSS_RETRAM_MEM_INIT_MEM0_INIT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_MSS_RETRAM_MEM_INIT_MEM0_INIT_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_RESETVAL                              (0x00000000U)

/* MSS_RETRAM_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_DONE_MSS_RETRAM_MEM_INIT_DONE_MEM0_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_DONE_MSS_RETRAM_MEM_INIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_DONE_MSS_RETRAM_MEM_INIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_DONE_MSS_RETRAM_MEM_INIT_DONE_MEM0_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_DONE_RESETVAL                         (0x00000000U)

/* MSS_RETRAM_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_STATUS_MSS_RETRAM_MEM_INIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_STATUS_MSS_RETRAM_MEM_INIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_STATUS_MSS_RETRAM_MEM_INIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_STATUS_MSS_RETRAM_MEM_INIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_RETRAM_MEM_INIT_STATUS_RESETVAL                       (0x00000000U)

/* MSS_SPIA_MEM_INIT */

#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_MSS_SPIA_MEM_INIT_MEM0_INIT_MASK        (0x00000001U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_MSS_SPIA_MEM_INIT_MEM0_INIT_SHIFT       (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_MSS_SPIA_MEM_INIT_MEM0_INIT_RESETVAL    (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_MSS_SPIA_MEM_INIT_MEM0_INIT_MAX         (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_RESETVAL                                (0x00000000U)

/* MSS_SPIA_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_MSS_SPIA_MEM_INIT_DONE_MEM0_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_DONE_RESETVAL                           (0x00000000U)

/* MSS_SPIA_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_STATUS_MSS_SPIA_MEM_INIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_STATUS_MSS_SPIA_MEM_INIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_STATUS_MSS_SPIA_MEM_INIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_STATUS_MSS_SPIA_MEM_INIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIA_MEM_INIT_STATUS_RESETVAL                         (0x00000000U)

/* MSS_SPIB_MEM_INIT */

#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_MSS_SPIB_MEM_INIT_MEM0_INIT_MASK        (0x00000001U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_MSS_SPIB_MEM_INIT_MEM0_INIT_SHIFT       (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_MSS_SPIB_MEM_INIT_MEM0_INIT_RESETVAL    (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_MSS_SPIB_MEM_INIT_MEM0_INIT_MAX         (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_RESETVAL                                (0x00000000U)

/* MSS_SPIB_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_MSS_SPIB_MEM_INIT_DONE_MEM0_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_DONE_RESETVAL                           (0x00000000U)

/* MSS_SPIB_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_STATUS_MSS_SPIB_MEM_INIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_STATUS_MSS_SPIB_MEM_INIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_STATUS_MSS_SPIB_MEM_INIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_STATUS_MSS_SPIB_MEM_INIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIB_MEM_INIT_STATUS_RESETVAL                         (0x00000000U)

/* MSS_TPCC_MEMINIT_START */

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_MSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_START_RESETVAL                           (0x00000000U)

/* MSS_TPCC_MEMINIT_DONE */

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_MSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_DONE_RESETVAL                            (0x00000000U)

/* MSS_TPCC_MEMINIT_STATUS */

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_MSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_MEMINIT_STATUS_RESETVAL                          (0x00000000U)

/* MSS_GPADC_MEM_INIT */

#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_MSS_GPADC_MEM_INIT_MEM0_INIT_MASK      (0x00000001U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_MSS_GPADC_MEM_INIT_MEM0_INIT_SHIFT     (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_MSS_GPADC_MEM_INIT_MEM0_INIT_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_MSS_GPADC_MEM_INIT_MEM0_INIT_MAX       (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_RESETVAL                               (0x00000000U)

/* MSS_GPADC_MEM_INIT_DONE */

#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_DONE_MSS_GPADC_MEM_INIT_DONE_MEM0_DONE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_DONE_MSS_GPADC_MEM_INIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_DONE_MSS_GPADC_MEM_INIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_DONE_MSS_GPADC_MEM_INIT_DONE_MEM0_DONE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_DONE_RESETVAL                          (0x00000000U)

/* MSS_GPADC_MEM_INIT_STATUS */

#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_STATUS_MSS_GPADC_MEM_INIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_STATUS_MSS_GPADC_MEM_INIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_STATUS_MSS_GPADC_MEM_INIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_STATUS_MSS_GPADC_MEM_INIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_MEM_INIT_STATUS_RESETVAL                        (0x00000000U)

/* MSS_SPIA_CFG */

#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIASYNC2SEN_MASK               (0x00000007U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIASYNC2SEN_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIASYNC2SEN_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIASYNC2SEN_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_MASK         (0x00000100U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_SHIFT        (0x00000008U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_CS_TRIGSRC_EN_MAX          (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_TRIG_GATE_EN_MASK          (0x00010000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_TRIG_GATE_EN_SHIFT         (0x00000010U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_TRIG_GATE_EN_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_TRIG_GATE_EN_MAX           (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_MASK     (0x01000000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_SHIFT    (0x00000018U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIA_CFG_MSS_SPIA_CFG_SPIA_INT_TRIG_POLARITY_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIA_CFG_RESETVAL                                     (0x00000000U)

/* MSS_SPIB_CFG */

#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIBSYNC2SEN_MASK               (0x00000007U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIBSYNC2SEN_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIBSYNC2SEN_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIBSYNC2SEN_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_MASK         (0x00000100U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_SHIFT        (0x00000008U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_CS_TRIGSRC_EN_MAX          (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_TRIG_GATE_EN_MASK          (0x00010000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_TRIG_GATE_EN_SHIFT         (0x00000010U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_TRIG_GATE_EN_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_TRIG_GATE_EN_MAX           (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_MASK     (0x01000000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_SHIFT    (0x00000018U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SPIB_CFG_MSS_SPIB_CFG_SPIB_INT_TRIG_POLARITY_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_SPIB_CFG_RESETVAL                                     (0x00000000U)

/* MSS_EPWM_CFG */

#define SDL_MSS_CTRL_MSS_EPWM_CFG_MSS_EPWM_CFG_EPWM_CONFIG_MASK                (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_EPWM_CFG_MSS_EPWM_CFG_EPWM_CONFIG_SHIFT               (0x00000000U)
#define SDL_MSS_CTRL_MSS_EPWM_CFG_MSS_EPWM_CFG_EPWM_CONFIG_RESETVAL            (0x0F000000U)
#define SDL_MSS_CTRL_MSS_EPWM_CFG_MSS_EPWM_CFG_EPWM_CONFIG_MAX                 (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_EPWM_CFG_RESETVAL                                     (0x0F000000U)

/* MSS_GIO_CFG */

#define SDL_MSS_CTRL_MSS_GIO_CFG_MSS_GIO_CFG_GIO_CONFIG_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_GIO_CFG_MSS_GIO_CFG_GIO_CONFIG_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_MSS_GIO_CFG_MSS_GIO_CFG_GIO_CONFIG_RESETVAL               (0x00000000U)
#define SDL_MSS_CTRL_MSS_GIO_CFG_MSS_GIO_CFG_GIO_CONFIG_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_GIO_CFG_RESETVAL                                      (0x00000000U)

/* MSS_MCAN_FE_SELECT */

#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANA_FE_SELECT_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANA_FE_SELECT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANA_FE_SELECT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANA_FE_SELECT_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANB_FE_SELECT_MASK (0x00070000U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANB_FE_SELECT_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANB_FE_SELECT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_MSS_MCAN_FE_SELECT_MCANB_FE_SELECT_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_MCAN_FE_SELECT_RESETVAL                               (0x00000000U)

/* HW_SPARE_REG1 */

#define SDL_MSS_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_MASK                       (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_SHIFT                      (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_RESETVAL                   (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REG1_HW_SPARE_REG1_NU_MAX                        (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_REG1_RESETVAL                                    (0x00000000U)

/* MSS_MCANA_INT_CLR */

#define SDL_MSS_CTRL_MSS_MCANA_INT_CLR_MSS_MCANA_INT_CLR_MCAN_INT_CLR_MASK     (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCANA_INT_CLR_MSS_MCANA_INT_CLR_MCAN_INT_CLR_SHIFT    (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_CLR_MSS_MCANA_INT_CLR_MCAN_INT_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_CLR_MSS_MCANA_INT_CLR_MCAN_INT_CLR_MAX      (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCANA_INT_CLR_RESETVAL                                (0x00000000U)

/* MSS_MCANA_INT_MASK */

#define SDL_MSS_CTRL_MSS_MCANA_INT_MASK_MSS_MCANA_INT_MASK_MCAN_INT_MASK_MASK  (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCANA_INT_MASK_MSS_MCANA_INT_MASK_MCAN_INT_MASK_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_MASK_MSS_MCANA_INT_MASK_MCAN_INT_MASK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_MASK_MSS_MCANA_INT_MASK_MCAN_INT_MASK_MAX   (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCANA_INT_MASK_RESETVAL                               (0x00000000U)

/* MSS_MCANA_INT_STAT */

#define SDL_MSS_CTRL_MSS_MCANA_INT_STAT_MSS_MCANA_INT_STAT_MCAN_INT_STATUS_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCANA_INT_STAT_MSS_MCANA_INT_STAT_MCAN_INT_STATUS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_STAT_MSS_MCANA_INT_STAT_MCAN_INT_STATUS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCANA_INT_STAT_MSS_MCANA_INT_STAT_MCAN_INT_STATUS_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCANA_INT_STAT_RESETVAL                               (0x00000000U)

/* HW_SPARE_REG2 */

#define SDL_MSS_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_MASK                       (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_SHIFT                      (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_RESETVAL                   (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REG2_HW_SPARE_REG2_NU_MAX                        (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_REG2_RESETVAL                                    (0x00000000U)

/* CCC_ERR_STATUS */

#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCA_ERROT_STATUS_MASK      (0x000000FFU)
#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCA_ERROT_STATUS_SHIFT     (0x00000000U)
#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCA_ERROT_STATUS_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCA_ERROT_STATUS_MAX       (0x000000FFU)

#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCB_ERROT_STATUS_MASK      (0x00FF0000U)
#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCB_ERROT_STATUS_SHIFT     (0x00000010U)
#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCB_ERROT_STATUS_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_CCC_ERR_STATUS_CCC_ERR_STATUS_CCCB_ERROT_STATUS_MAX       (0x000000FFU)

#define SDL_MSS_CTRL_CCC_ERR_STATUS_RESETVAL                                   (0x00000000U)

/* CCCA_CFG0 */

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK0_SEL_MASK                    (0x00000007U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK0_SEL_SHIFT                   (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK0_SEL_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK0_SEL_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK1_SEL_MASK                    (0x00000038U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK1_SEL_SHIFT                   (0x00000003U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK1_SEL_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_CLK1_SEL_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_DISABLE_CLOCKS_MASK              (0x00000040U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_DISABLE_CLOCKS_SHIFT             (0x00000006U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_DISABLE_CLOCKS_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_DISABLE_CLOCKS_MAX               (0x00000001U)

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_ENABLE_MODULE_MASK               (0x00000080U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_ENABLE_MODULE_SHIFT              (0x00000007U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_ENABLE_MODULE_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_ENABLE_MODULE_MAX                (0x00000001U)

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_SINGLE_SHOT_MODE_MASK            (0x00000100U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_SINGLE_SHOT_MODE_SHIFT           (0x00000008U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_SINGLE_SHOT_MODE_RESETVAL        (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_SINGLE_SHOT_MODE_MAX             (0x00000001U)

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_RESERVED_MASK                         (0x0000FE00U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_RESERVED_SHIFT                        (0x00000009U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_RESERVED_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_RESERVED_MAX                          (0x0000007FU)

#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_MARGIN_COUNT_MASK                (0xFFFF0000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_MARGIN_COUNT_SHIFT               (0x00000010U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_MARGIN_COUNT_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG0_CCCA_CFG0_CCCA_MARGIN_COUNT_MAX                 (0x0000FFFFU)

#define SDL_MSS_CTRL_CCCA_CFG0_RESETVAL                                        (0x00000000U)

/* CCCA_CFG1 */

#define SDL_MSS_CTRL_CCCA_CFG1_CCCA_CFG1_CCCA_CFG_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCA_CFG1_CCCA_CFG1_CCCA_CFG_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG1_CCCA_CFG1_CCCA_CFG_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG1_CCCA_CFG1_CCCA_CFG_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCA_CFG1_RESETVAL                                        (0x00000000U)

/* CCCA_CFG2 */

#define SDL_MSS_CTRL_CCCA_CFG2_CCCA_CFG2_CCCA_CFG_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCA_CFG2_CCCA_CFG2_CCCA_CFG_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG2_CCCA_CFG2_CCCA_CFG_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG2_CCCA_CFG2_CCCA_CFG_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCA_CFG2_RESETVAL                                        (0x00000000U)

/* CCCA_CFG3 */

#define SDL_MSS_CTRL_CCCA_CFG3_CCCA_CFG3_CCCA_CFG_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCA_CFG3_CCCA_CFG3_CCCA_CFG_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG3_CCCA_CFG3_CCCA_CFG_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CFG3_CCCA_CFG3_CCCA_CFG_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCA_CFG3_RESETVAL                                        (0x00000000U)

/* CCCA_CNTVAL */

#define SDL_MSS_CTRL_CCCA_CNTVAL_CCCA_CNTVAL_CCCA_CFG_MASK                     (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCA_CNTVAL_CCCA_CNTVAL_CCCA_CFG_SHIFT                    (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CNTVAL_CCCA_CNTVAL_CCCA_CFG_RESETVAL                 (0x00000000U)
#define SDL_MSS_CTRL_CCCA_CNTVAL_CCCA_CNTVAL_CCCA_CFG_MAX                      (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCA_CNTVAL_RESETVAL                                      (0x00000000U)

/* CCCB_CFG0 */

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK0_SEL_MASK                    (0x00000007U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK0_SEL_SHIFT                   (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK0_SEL_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK0_SEL_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK1_SEL_MASK                    (0x00000038U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK1_SEL_SHIFT                   (0x00000003U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK1_SEL_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_CLK1_SEL_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_DISABLE_CLOCKS_MASK              (0x00000040U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_DISABLE_CLOCKS_SHIFT             (0x00000006U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_DISABLE_CLOCKS_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_DISABLE_CLOCKS_MAX               (0x00000001U)

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_ENABLE_MODULE_MASK               (0x00000080U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_ENABLE_MODULE_SHIFT              (0x00000007U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_ENABLE_MODULE_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_ENABLE_MODULE_MAX                (0x00000001U)

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_SINGLE_SHOT_MODE_MASK            (0x00000100U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_SINGLE_SHOT_MODE_SHIFT           (0x00000008U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_SINGLE_SHOT_MODE_RESETVAL        (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_SINGLE_SHOT_MODE_MAX             (0x00000001U)

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_RESERVED_MASK                         (0x0000FE00U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_RESERVED_SHIFT                        (0x00000009U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_RESERVED_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_RESERVED_MAX                          (0x0000007FU)

#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_MARGIN_COUNT_MASK                (0xFFFF0000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_MARGIN_COUNT_SHIFT               (0x00000010U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_MARGIN_COUNT_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG0_CCCB_CFG0_CCCB_MARGIN_COUNT_MAX                 (0x0000FFFFU)

#define SDL_MSS_CTRL_CCCB_CFG0_RESETVAL                                        (0x00000000U)

/* CCCB_CFG1 */

#define SDL_MSS_CTRL_CCCB_CFG1_CCCB_CFG1_CCCB_CFG_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCB_CFG1_CCCB_CFG1_CCCB_CFG_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG1_CCCB_CFG1_CCCB_CFG_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG1_CCCB_CFG1_CCCB_CFG_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCB_CFG1_RESETVAL                                        (0x00000000U)

/* CCCB_CFG2 */

#define SDL_MSS_CTRL_CCCB_CFG2_CCCB_CFG2_CCCB_CFG_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCB_CFG2_CCCB_CFG2_CCCB_CFG_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG2_CCCB_CFG2_CCCB_CFG_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG2_CCCB_CFG2_CCCB_CFG_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCB_CFG2_RESETVAL                                        (0x00000000U)

/* CCCB_CFG3 */

#define SDL_MSS_CTRL_CCCB_CFG3_CCCB_CFG3_CCCB_CFG_MASK                         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCB_CFG3_CCCB_CFG3_CCCB_CFG_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG3_CCCB_CFG3_CCCB_CFG_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CFG3_CCCB_CFG3_CCCB_CFG_MAX                          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCB_CFG3_RESETVAL                                        (0x00000000U)

/* CCCB_CNTVAL */

#define SDL_MSS_CTRL_CCCB_CNTVAL_CCCB_CNTVAL_CCCB_CFG_MASK                     (0xFFFFFFFFU)
#define SDL_MSS_CTRL_CCCB_CNTVAL_CCCB_CNTVAL_CCCB_CFG_SHIFT                    (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CNTVAL_CCCB_CNTVAL_CCCB_CFG_RESETVAL                 (0x00000000U)
#define SDL_MSS_CTRL_CCCB_CNTVAL_CCCB_CNTVAL_CCCB_CFG_MAX                      (0xFFFFFFFFU)

#define SDL_MSS_CTRL_CCCB_CNTVAL_RESETVAL                                      (0x00000000U)

/* CCC_DCC_COMMON */

#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_RSTN_MASK   (0x00000100U)
#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_RSTN_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_RSTN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_RSTN_MAX    (0x00000001U)

#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_NMI_MASK    (0x00001000U)
#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_NMI_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_NMI_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_CCC_DCC_COMMON_CCC_DCC_COMMON_ENABLE_CCCB_ERR_NMI_MAX     (0x00000001U)

#define SDL_MSS_CTRL_CCC_DCC_COMMON_RESETVAL                                   (0x00000000U)

/* R5_GLOBAL_CONFIG */

#define SDL_MSS_CTRL_R5_GLOBAL_CONFIG_R5_GLOBAL_CONFIG_TEINIT_MASK             (0x00000001U)
#define SDL_MSS_CTRL_R5_GLOBAL_CONFIG_R5_GLOBAL_CONFIG_TEINIT_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_R5_GLOBAL_CONFIG_R5_GLOBAL_CONFIG_TEINIT_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_R5_GLOBAL_CONFIG_R5_GLOBAL_CONFIG_TEINIT_MAX              (0x00000001U)

#define SDL_MSS_CTRL_R5_GLOBAL_CONFIG_RESETVAL                                 (0x00000000U)

/* R5_AHB_EN */

#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU0_AHB_INIT_MASK                    (0x00000007U)
#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU0_AHB_INIT_SHIFT                   (0x00000000U)
#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU0_AHB_INIT_RESETVAL                (0x00000007U)
#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU0_AHB_INIT_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU1_AHB_INIT_MASK                    (0x00070000U)
#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU1_AHB_INIT_SHIFT                   (0x00000010U)
#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU1_AHB_INIT_RESETVAL                (0x00000007U)
#define SDL_MSS_CTRL_R5_AHB_EN_R5_AHB_EN_CPU1_AHB_INIT_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_R5_AHB_EN_RESETVAL                                        (0x00070007U)

/* R5A_AHB_BASE */

#define SDL_MSS_CTRL_R5A_AHB_BASE_R5A_AHB_BASE_AHB_BASE_MASK                   (0x000FFFFFU)
#define SDL_MSS_CTRL_R5A_AHB_BASE_R5A_AHB_BASE_AHB_BASE_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_R5A_AHB_BASE_R5A_AHB_BASE_AHB_BASE_RESETVAL               (0x00000000U)
#define SDL_MSS_CTRL_R5A_AHB_BASE_R5A_AHB_BASE_AHB_BASE_MAX                    (0x000FFFFFU)

#define SDL_MSS_CTRL_R5A_AHB_BASE_RESETVAL                                     (0x00000000U)

/* R5A_AHB_SIZE */

#define SDL_MSS_CTRL_R5A_AHB_SIZE_R5A_AHB_SIZE_AHB_SIZE_MASK                   (0x0000001FU)
#define SDL_MSS_CTRL_R5A_AHB_SIZE_R5A_AHB_SIZE_AHB_SIZE_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_R5A_AHB_SIZE_R5A_AHB_SIZE_AHB_SIZE_RESETVAL               (0x00000012U)
#define SDL_MSS_CTRL_R5A_AHB_SIZE_R5A_AHB_SIZE_AHB_SIZE_MAX                    (0x0000001FU)

#define SDL_MSS_CTRL_R5A_AHB_SIZE_RESETVAL                                     (0x00000012U)

/* R5B_AHB_BASE */

#define SDL_MSS_CTRL_R5B_AHB_BASE_R5B_AHB_BASE_AHB_BASE_MASK                   (0x000FFFFFU)
#define SDL_MSS_CTRL_R5B_AHB_BASE_R5B_AHB_BASE_AHB_BASE_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_R5B_AHB_BASE_R5B_AHB_BASE_AHB_BASE_RESETVAL               (0x00000000U)
#define SDL_MSS_CTRL_R5B_AHB_BASE_R5B_AHB_BASE_AHB_BASE_MAX                    (0x000FFFFFU)

#define SDL_MSS_CTRL_R5B_AHB_BASE_RESETVAL                                     (0x00000000U)

/* R5B_AHB_SIZE */

#define SDL_MSS_CTRL_R5B_AHB_SIZE_R5B_AHB_SIZE_AHB_SIZE_MASK                   (0x0000001FU)
#define SDL_MSS_CTRL_R5B_AHB_SIZE_R5B_AHB_SIZE_AHB_SIZE_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_R5B_AHB_SIZE_R5B_AHB_SIZE_AHB_SIZE_RESETVAL               (0x00000012U)
#define SDL_MSS_CTRL_R5B_AHB_SIZE_R5B_AHB_SIZE_AHB_SIZE_MAX                    (0x0000001FU)

#define SDL_MSS_CTRL_R5B_AHB_SIZE_RESETVAL                                     (0x00000012U)

/* R5_TCM_EXT_ERR_EN */

#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU0_TCM_MASK         (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU0_TCM_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU0_TCM_RESETVAL     (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU0_TCM_MAX          (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU1_TCM_MASK         (0x00070000U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU1_TCM_SHIFT        (0x00000010U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU1_TCM_RESETVAL     (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_R5_TCM_EXT_ERR_EN_CPU1_TCM_MAX          (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_EXT_ERR_EN_RESETVAL                                (0x00070007U)

/* R5_TCM_ERR_EN */

#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU0_TCM_MASK                 (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU0_TCM_SHIFT                (0x00000000U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU0_TCM_RESETVAL             (0x00000000U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU0_TCM_MAX                  (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU1_TCM_MASK                 (0x00070000U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU1_TCM_SHIFT                (0x00000010U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU1_TCM_RESETVAL             (0x00000000U)
#define SDL_MSS_CTRL_R5_TCM_ERR_EN_R5_TCM_ERR_EN_CPU1_TCM_MAX                  (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ERR_EN_RESETVAL                                    (0x00000000U)

/* R5_INIT_TCM */

#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU0_MASK                    (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU0_SHIFT                   (0x00000000U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU0_RESETVAL                (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU0_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU0_MASK                    (0x00000070U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU0_SHIFT                   (0x00000004U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU0_RESETVAL                (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU0_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU0_MASK                (0x00000700U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU0_SHIFT               (0x00000008U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU0_RESETVAL            (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU0_MAX                 (0x00000007U)

#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU1_MASK                    (0x00007000U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU1_SHIFT                   (0x0000000CU)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU1_RESETVAL                (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMA_CPU1_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU1_MASK                    (0x00070000U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU1_SHIFT                   (0x00000010U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU1_RESETVAL                (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_TCMB_CPU1_MAX                     (0x00000007U)

#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU1_MASK                (0x00700000U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU1_SHIFT               (0x00000014U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU1_RESETVAL            (0x00000007U)
#define SDL_MSS_CTRL_R5_INIT_TCM_R5_INIT_TCM_LOCKZRAM_CPU1_MAX                 (0x00000007U)

#define SDL_MSS_CTRL_R5_INIT_TCM_RESETVAL                                      (0x00777777U)

/* R5_TCM_ECC_WRENZ_EN */

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_MASK (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMA_WRENZ_EN_MAX (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_MASK (0x00000070U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB0_WRENZ_EN_MAX (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_MASK (0x00000700U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU0_TCMB1_WRENZ_EN_MAX (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_MASK (0x00007000U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMA_WRENZ_EN_MAX (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_MASK (0x00070000U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB0_WRENZ_EN_MAX (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_MASK (0x00700000U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_R5_TCM_ECC_WRENZ_EN_CPU1_TCMB1_WRENZ_EN_MAX (0x00000007U)

#define SDL_MSS_CTRL_R5_TCM_ECC_WRENZ_EN_RESETVAL                              (0x00777777U)

/* ESM_GATING0 */

#define SDL_MSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING0_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING1 */

#define SDL_MSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING1_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING2 */

#define SDL_MSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING2_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING3 */

#define SDL_MSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING3_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING4 */

#define SDL_MSS_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING4_ESM_GATING4_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING4_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING5 */

#define SDL_MSS_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING5_ESM_GATING5_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING5_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING6 */

#define SDL_MSS_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING6_ESM_GATING6_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING6_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING7 */

#define SDL_MSS_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ESM_GATING7_ESM_GATING7_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ESM_GATING7_RESETVAL                                      (0xFFFFFFFFU)

/* ERR_PARITY_ATCM0 */

#define SDL_MSS_CTRL_ERR_PARITY_ATCM0_ERR_PARITY_ATCM0_ADDR_MASK               (0x000FFFFFU)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM0_ERR_PARITY_ATCM0_ADDR_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM0_ERR_PARITY_ATCM0_ADDR_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM0_ERR_PARITY_ATCM0_ADDR_MAX                (0x000FFFFFU)

#define SDL_MSS_CTRL_ERR_PARITY_ATCM0_RESETVAL                                 (0x00000000U)

/* ERR_PARITY_ATCM1 */

#define SDL_MSS_CTRL_ERR_PARITY_ATCM1_ERR_PARITY_ATCM1_ADDR_MASK               (0x000FFFFFU)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM1_ERR_PARITY_ATCM1_ADDR_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM1_ERR_PARITY_ATCM1_ADDR_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_ATCM1_ERR_PARITY_ATCM1_ADDR_MAX                (0x000FFFFFU)

#define SDL_MSS_CTRL_ERR_PARITY_ATCM1_RESETVAL                                 (0x00000000U)

/* ERR_PARITY_B0TCM0 */

#define SDL_MSS_CTRL_ERR_PARITY_B0TCM0_ERR_PARITY_B0TCM0_ADDR_MASK             (0x000FFFFFU)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM0_ERR_PARITY_B0TCM0_ADDR_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM0_ERR_PARITY_B0TCM0_ADDR_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM0_ERR_PARITY_B0TCM0_ADDR_MAX              (0x000FFFFFU)

#define SDL_MSS_CTRL_ERR_PARITY_B0TCM0_RESETVAL                                (0x00000000U)

/* ERR_PARITY_B0TCM1 */

#define SDL_MSS_CTRL_ERR_PARITY_B0TCM1_ERR_PARITY_B0TCM1_ADDR_MASK             (0x000FFFFFU)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM1_ERR_PARITY_B0TCM1_ADDR_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM1_ERR_PARITY_B0TCM1_ADDR_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B0TCM1_ERR_PARITY_B0TCM1_ADDR_MAX              (0x000FFFFFU)

#define SDL_MSS_CTRL_ERR_PARITY_B0TCM1_RESETVAL                                (0x00000000U)

/* ERR_PARITY_B1TCM0 */

#define SDL_MSS_CTRL_ERR_PARITY_B1TCM0_ERR_PARITY_B1TCM0_ADDR_MASK             (0x000FFFFFU)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM0_ERR_PARITY_B1TCM0_ADDR_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM0_ERR_PARITY_B1TCM0_ADDR_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM0_ERR_PARITY_B1TCM0_ADDR_MAX              (0x000FFFFFU)

#define SDL_MSS_CTRL_ERR_PARITY_B1TCM0_RESETVAL                                (0x00000000U)

/* ERR_PARITY_B1TCM1 */

#define SDL_MSS_CTRL_ERR_PARITY_B1TCM1_ERR_PARITY_B1TCM1_ADDR_MASK             (0x000FFFFFU)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM1_ERR_PARITY_B1TCM1_ADDR_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM1_ERR_PARITY_B1TCM1_ADDR_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_ERR_PARITY_B1TCM1_ERR_PARITY_B1TCM1_ADDR_MAX              (0x000FFFFFU)

#define SDL_MSS_CTRL_ERR_PARITY_B1TCM1_RESETVAL                                (0x00000000U)

/* TCM_PARITY_CTRL */

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM0_ERRADDR_CLR_MASK    (0x00000007U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM0_ERRADDR_CLR_SHIFT   (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM0_ERRADDR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM0_ERRADDR_CLR_MAX     (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM1_ERRADDR_CLR_MASK    (0x00000070U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM1_ERRADDR_CLR_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM1_ERRADDR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_ATCM1_ERRADDR_CLR_MAX     (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0TCM0_ERRADDR_CLR_MASK   (0x00000700U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0TCM0_ERRADDR_CLR_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0TCM0_ERRADDR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0TCM0_ERRADDR_CLR_MAX    (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0CM1_ERRADDR_CLR_MASK    (0x00007000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0CM1_ERRADDR_CLR_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0CM1_ERRADDR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B0CM1_ERRADDR_CLR_MAX     (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM0_ERRADDR_CLR_MASK   (0x00070000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM0_ERRADDR_CLR_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM0_ERRADDR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM0_ERRADDR_CLR_MAX    (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM1_ERRADDR_CLR_MASK   (0x00700000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM1_ERRADDR_CLR_SHIFT  (0x00000014U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM1_ERRADDR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_CTRL_TCM_PARITY_CTRL_B1TCM1_ERRADDR_CLR_MAX    (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_CTRL_RESETVAL                                  (0x00000000U)

/* TCM_PARITY_ERRFRC */

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM0_MASK            (0x00000007U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM0_SHIFT           (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM0_RESETVAL        (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM0_MAX             (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM1_MASK            (0x00000070U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM1_SHIFT           (0x00000004U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM1_RESETVAL        (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_ATCM1_MAX             (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM0_MASK           (0x00000700U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM0_SHIFT          (0x00000008U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM0_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM0_MAX            (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM1_MASK           (0x00007000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM1_SHIFT          (0x0000000CU)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM1_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B0TCM1_MAX            (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM0_MASK           (0x00070000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM0_SHIFT          (0x00000010U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM0_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM0_MAX            (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM1_MASK           (0x00700000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM1_SHIFT          (0x00000014U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM1_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_TCM_PARITY_ERRFRC_B1TCM1_MAX            (0x00000007U)

#define SDL_MSS_CTRL_TCM_PARITY_ERRFRC_RESETVAL                                (0x00000000U)

/* HW_SPARE_REG3 */

#define SDL_MSS_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_MASK                       (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_SHIFT                      (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_RESETVAL                   (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REG3_HW_SPARE_REG3_NU_MAX                        (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_REG3_RESETVAL                                    (0x00000000U)

/* SPIA_IO_CFG */

#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_MASK                     (0x00000007U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_SHIFT                    (0x00000000U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_RESETVAL                 (0x00000000U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_DEACT_MAX                      (0x00000007U)

#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_MASK                       (0x00000700U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_SHIFT                      (0x00000008U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_RESETVAL                   (0x00000000U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_CS_POL_MAX                        (0x00000007U)

#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_MASK               (0x00070000U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_SHIFT              (0x00000010U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_SPIA_IO_CFG_SPIA_IO_CFG_MISO_OEN_BY_CS_MAX                (0x00000007U)

#define SDL_MSS_CTRL_SPIA_IO_CFG_RESETVAL                                      (0x00000000U)

/* SPIB_IO_CFG */

#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_MASK                     (0x00000007U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_SHIFT                    (0x00000000U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_RESETVAL                 (0x00000000U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_DEACT_MAX                      (0x00000007U)

#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_MASK                       (0x00000700U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_SHIFT                      (0x00000008U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_RESETVAL                   (0x00000000U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_CS_POL_MAX                        (0x00000007U)

#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_MASK               (0x00070000U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_SHIFT              (0x00000010U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_SPIB_IO_CFG_SPIB_IO_CFG_MISO_OEN_BY_CS_MAX                (0x00000007U)

#define SDL_MSS_CTRL_SPIB_IO_CFG_RESETVAL                                      (0x00000000U)

/* SPI_HOST_IRQ */

#define SDL_MSS_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_MASK                   (0x00000007U)
#define SDL_MSS_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_SHIFT                  (0x00000000U)
#define SDL_MSS_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_RESETVAL               (0x00000000U)
#define SDL_MSS_CTRL_SPI_HOST_IRQ_SPI_HOST_IRQ_HOST_IRQ_MAX                    (0x00000007U)

#define SDL_MSS_CTRL_SPI_HOST_IRQ_RESETVAL                                     (0x00000000U)

/* TPTC_DBS_CONFIG */

#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_MASK              (0x00000003U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_RESETVAL          (0x00000001U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_MAX               (0x00000003U)

#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_MASK              (0x00000030U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_SHIFT             (0x00000004U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_RESETVAL          (0x00000001U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_MAX               (0x00000003U)

#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_MASK              (0x00000300U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_SHIFT             (0x00000008U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_RESETVAL          (0x00000001U)
#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_MAX               (0x00000003U)

#define SDL_MSS_CTRL_TPTC_DBS_CONFIG_RESETVAL                                  (0x00000111U)

/* TPCC_PARITY_CTRL */

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_MASK   (0x00000001U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_EN_MAX    (0x00000001U)

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_MASK (0x00000010U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_TESTEN_MAX (0x00000001U)

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_MASK   (0x00000100U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_EN_MAX    (0x00000001U)

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_MASK (0x00001000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_TESTEN_MAX (0x00000001U)

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_MASK (0x00010000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_A_PARITY_ERR_CLR_MAX (0x00000001U)

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_MASK (0x00100000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_TPCC_PARITY_CTRL_TPCC_B_PARITY_ERR_CLR_MAX (0x00000001U)

#define SDL_MSS_CTRL_TPCC_PARITY_CTRL_RESETVAL                                 (0x00000000U)

/* TPCC_PARITY_STATUS */

#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_A_PARITY_ADDR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_TPCC_PARITY_STATUS_TPCC_B_PARITY_ADDR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_TPCC_PARITY_STATUS_RESETVAL                               (0x00000000U)

/* MSS_DBG_ACK_CTL0 */

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCA_MASK               (0x00000007U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCA_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCA_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCA_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCB_MASK               (0x00000070U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCB_SHIFT              (0x00000004U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCB_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CCCB_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCA_MASK               (0x00000700U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCA_SHIFT              (0x00000008U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCA_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCA_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCB_MASK               (0x00007000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCB_SHIFT              (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCB_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCB_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCC_MASK               (0x00070000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCC_SHIFT              (0x00000010U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCC_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCC_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCD_MASK               (0x00700000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCD_SHIFT              (0x00000014U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCD_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_DCCD_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CPSW_MASK               (0x07000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CPSW_SHIFT              (0x00000018U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CPSW_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_MSS_DBG_ACK_CTL0_CPSW_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL0_RESETVAL                                 (0x00000000U)

/* MSS_DBG_ACK_CTL1 */

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_DCAN_MASK               (0x00000007U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_DCAN_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_DCAN_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_DCAN_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_RTI_MASK                (0x00000070U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_RTI_SHIFT               (0x00000004U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_RTI_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_RTI_MAX                 (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_WDT_MASK                (0x00000700U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_WDT_SHIFT               (0x00000008U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_WDT_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_WDT_MAX                 (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_MCRC_MASK               (0x00007000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_MCRC_SHIFT              (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_MCRC_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_MCRC_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_I2C_MASK                (0x00070000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_I2C_SHIFT               (0x00000010U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_I2C_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_I2C_MAX                 (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIA_MASK               (0x00700000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIA_SHIFT              (0x00000014U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIA_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIA_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIB_MASK               (0x07000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIB_SHIFT              (0x00000018U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIB_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_MSS_DBG_ACK_CTL1_SCIB_MAX                (0x00000007U)

#define SDL_MSS_CTRL_MSS_DBG_ACK_CTL1_RESETVAL                                 (0x00000000U)

/* CPSW_CONTROL */

#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_MASK             (0x00000007U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_PORT1_MODE_SEL_MAX              (0x00000007U)

#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_MASK          (0x00000100U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_SHIFT         (0x00000008U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RMII_REF_CLK_OE_N_MAX           (0x00000001U)

#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_MASK             (0x00010000U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_SHIFT            (0x00000010U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_CPSW_CONTROL_CPSW_CONTROL_RGMII1_ID_MODE_MAX              (0x00000001U)

#define SDL_MSS_CTRL_CPSW_CONTROL_RESETVAL                                     (0x00000000U)

/* MSS_TPCC_A_ERRAGG_MASK */

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_PAR_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_MSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* MSS_TPCC_A_ERRAGG_STATUS */

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PAR_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_MSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* MSS_TPCC_A_ERRAGG_STATUS_RAW */

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PAR_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_MSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* MSS_TPCC_A_INTAGG_MASK */

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A0_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A0_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A1_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A1_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_MSS_TPCC_A_INTAGG_MASK_TPTC_A1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* MSS_TPCC_A_INTAGG_STATUS */

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A0_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A1_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A1_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_MSS_TPCC_A_INTAGG_STATUS_TPTC_A1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* MSS_TPCC_A_INTAGG_STATUS_RAW */

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_MSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* MSS_TPCC_B_ERRAGG_MASK */

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_PAR_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_MSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* MSS_TPCC_B_ERRAGG_STATUS */

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PAR_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_MSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* MSS_TPCC_B_ERRAGG_STATUS_RAW */

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PAR_ERR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_MSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* MSS_TPCC_B_INTAGG_MASK */

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPTC_B0_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPTC_B0_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPTC_B0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_MSS_TPCC_B_INTAGG_MASK_TPTC_B0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* MSS_TPCC_B_INTAGG_STATUS */

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPTC_B0_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPTC_B0_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPTC_B0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_MSS_TPCC_B_INTAGG_STATUS_TPTC_B0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* MSS_TPCC_B_INTAGG_STATUS_RAW */

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_MSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPCC_B_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* MSS_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_RESETVAL                              (0x00000000U)

/* MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL_RESETVAL                  (0x00000007U)

/* MSS_CR5A_AXI_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI_RESETVAL                    (0x00000000U)

/* MSS_CR5A_AXI_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_RESETVAL                   (0x00000000U)

/* MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL        (0x00000000U)

/* MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL          (0x00000000U)

/* MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL         (0x00000000U)

/* MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL_RESETVAL                  (0x00000007U)

/* MSS_CR5B_AXI_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI_RESETVAL                    (0x00000000U)

/* MSS_CR5B_AXI_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_RESETVAL                   (0x00000000U)

/* MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL        (0x00000000U)

/* MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL          (0x00000000U)

/* MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL         (0x00000000U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL_RESETVAL                  (0x00000007U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI_RESETVAL                    (0x00000000U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_RESETVAL                   (0x00000000U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL        (0x00000000U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL          (0x00000000U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL        (0x00000000U)

/* MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL    (0x00000000U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL_RESETVAL                  (0x00000007U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI_RESETVAL                    (0x00000000U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_RESETVAL                   (0x00000000U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL        (0x00000000U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL          (0x00000000U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL        (0x00000000U)

/* MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL    (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_MSS_CR5A_AXI_S_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_MSS_CR5B_AXI_S_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* MSS_TPTC_A0_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_TPTC_A0_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_MSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_TPTC_A0_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* MSS_TPTC_A1_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_TPTC_A1_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_MSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_TPTC_A1_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* MSS_TPTC_B0_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_TPTC_B0_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_MSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_TPTC_B0_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_MSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_MSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_MSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* HSM_TPTC_A0_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* HSM_TPTC_A0_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_HSM_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* HSM_TPTC_A0_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* HSM_TPTC_A1_RD_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* HSM_TPTC_A1_RD_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_HSM_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* HSM_TPTC_A1_RD_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_HSM_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_HSM_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_QSPI_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_MSS_QSPI_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_MSS_QSPI_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* HSM_DTHE_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_HSM_DTHE_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_HSM_DTHE_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_DTHE_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_CPSW_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_MSS_CPSW_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_MSS_CPSW_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_MCRC_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_MSS_MCRC_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_MSS_MCRC_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_PCR_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_MSS_PCR_BUS_SAFETY_CTRL_TYPE_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL_RESETVAL                          (0x00000007U)

/* MSS_PCR_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SEC_MASK      (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SEC_SHIFT     (0x00000004U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SEC_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SEC_MAX       (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DED_MASK      (0x00000020U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DED_SHIFT     (0x00000005U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DED_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DED_MAX       (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DATA_MASK     (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DATA_SHIFT    (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_DATA_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_MAIN_MASK     (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_MAIN_SHIFT    (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_MAIN_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SAFE_MASK     (0xFF000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SAFE_SHIFT    (0x00000018U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_MSS_PCR_BUS_SAFETY_FI_SAFE_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI_RESETVAL                            (0x00000000U)

/* MSS_PCR_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_SEC_MASK    (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_SEC_SHIFT   (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_SEC_MAX     (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_DED_MASK    (0xFF000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_DED_SHIFT   (0x00000018U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_MSS_PCR_BUS_SAFETY_ERR_DED_MAX     (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_RESETVAL                           (0x00000000U)

/* MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL                (0x00000000U)

/* MSS_PCR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                  (0x00000000U)

/* MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL                (0x00000000U)

/* MSS_PCR_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ_RESETVAL                 (0x00000000U)

/* MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL            (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_PCR2_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_MSS_PCR2_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_MSS_PCR2_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* HSM_M_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_MASK   (0x00000007U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ENABLE_MAX    (0x00000007U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_MASK     (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_SHIFT    (0x00000010U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_HSM_M_BUS_SAFETY_CTRL_TYPE_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_CTRL_RESETVAL                            (0x00000007U)

/* HSM_M_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_MASK  (0x00000001U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_MAX   (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_MASK  (0x00000002U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_MAX   (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SEC_MASK          (0x00000010U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SEC_SHIFT         (0x00000004U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SEC_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SEC_MAX           (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DED_MASK          (0x00000020U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DED_SHIFT         (0x00000005U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DED_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DED_MAX           (0x00000001U)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DATA_MASK         (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DATA_SHIFT        (0x00000008U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DATA_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_DATA_MAX          (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_MAIN_MASK         (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_MAIN_SHIFT        (0x00000010U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_MAIN_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_MAIN_MAX          (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SAFE_MASK         (0xFF000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SAFE_SHIFT        (0x00000018U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SAFE_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_HSM_M_BUS_SAFETY_FI_SAFE_MAX          (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_FI_RESETVAL                              (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_ERR_MASK   (0x000000FFU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_ERR_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_ERR_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_COMP_CHECK_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_SEC_MASK        (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_SEC_SHIFT       (0x00000010U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_SEC_RESETVAL    (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_SEC_MAX         (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_DED_MASK        (0xFF000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_DED_SHIFT       (0x00000018U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_DED_RESETVAL    (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_HSM_M_BUS_SAFETY_ERR_DED_MAX         (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_RESETVAL                             (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL                  (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_HSM_M_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                    (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL                  (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_HSM_M_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_READ_RESETVAL                   (0x00000000U)

/* HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_M_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL              (0x00000000U)

/* HSM_S_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_MASK   (0x00000007U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ENABLE_MAX    (0x00000007U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_MASK     (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_SHIFT    (0x00000010U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_HSM_S_BUS_SAFETY_CTRL_TYPE_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_CTRL_RESETVAL                            (0x00000007U)

/* HSM_S_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK  (0x00000001U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX   (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK  (0x00000002U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX   (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SEC_MASK          (0x00000010U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SEC_SHIFT         (0x00000004U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SEC_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SEC_MAX           (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DED_MASK          (0x00000020U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DED_SHIFT         (0x00000005U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DED_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DED_MAX           (0x00000001U)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DATA_MASK         (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DATA_SHIFT        (0x00000008U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DATA_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_DATA_MAX          (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_MAIN_MASK         (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_MAIN_SHIFT        (0x00000010U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_MAIN_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_MAIN_MAX          (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SAFE_MASK         (0xFF000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SAFE_SHIFT        (0x00000018U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SAFE_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_HSM_S_BUS_SAFETY_FI_SAFE_MAX          (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_FI_RESETVAL                              (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_ERR_MASK   (0x000000FFU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_ERR_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_COMP_CHECK_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_SEC_MASK        (0x00FF0000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_SEC_SHIFT       (0x00000010U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_SEC_RESETVAL    (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_SEC_MAX         (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_DED_MASK        (0xFF000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_DED_SHIFT       (0x00000018U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_DED_RESETVAL    (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_HSM_S_BUS_SAFETY_ERR_DED_MAX         (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_RESETVAL                             (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL                  (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_HSM_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                    (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL                  (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_HSM_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL                   (0x00000000U)

/* HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HSM_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL              (0x00000000U)

/* DAP_R232_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_DAP_R232_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* DAP_R232_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_DAP_R232_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* DAP_R232_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_DAP_R232_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* DAP_R232_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* DAP_R232_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* DAP_R232_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* DAP_R232_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ_DAP_R232_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ_DAP_R232_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ_DAP_R232_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ_DAP_R232_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_L2_A_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_MSS_L2_A_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_MSS_L2_A_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_L2_B_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_MSS_L2_B_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_MSS_L2_B_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_MBOX_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_MSS_MBOX_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL_RESETVAL                        (0x00000007U)

/* MSS_SWBUF_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SEC_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SEC_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DED_MASK  (0x00000020U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DED_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_DATA_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_MAIN_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_MSS_SWBUF_BUS_SAFETY_FI_SAFE_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI_RESETVAL                          (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_MSS_SWBUF_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_RESETVAL                         (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL              (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL              (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ_RESETVAL               (0x00000000U)

/* MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL          (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL_RESETVAL                        (0x00000007U)

/* MSS_GPADC_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SEC_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SEC_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DED_MASK  (0x00000020U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DED_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_DATA_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_MAIN_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_MSS_GPADC_BUS_SAFETY_FI_SAFE_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI_RESETVAL                          (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_MSS_GPADC_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_RESETVAL                         (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL              (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL              (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ_RESETVAL               (0x00000000U)

/* MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL          (0x00000000U)

/* MSS_BUS_SAFETY_SEC_ERR_STAT0 */

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5A_SLV_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CR5B_SLV_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_RS232_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_RS232_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_RS232_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DAP_RS232_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_CPSW_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_RD_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_RD_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A0_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_A1_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_WR_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_WR_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_TPTC_B1_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A0_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_TPTC_A1_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_QSPI_MASK (0x00080000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_QSPI_SHIFT (0x00000013U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_QSPI_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_QSPI_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MCRC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR_MASK (0x00200000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR_SHIFT (0x00000015U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR2_MASK (0x00400000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR2_SHIFT (0x00000016U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_PER_PCR2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_MASK (0x00800000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_SHIFT (0x00000017U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_HSM_S_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_DTHE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_L2RAM1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_MASK (0x08000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_SHIFT (0x0000001BU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_SWBUF_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_SWBUF_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_SWBUF_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_SWBUF_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_GPADC_MASK (0x20000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_GPADC_SHIFT (0x0000001DU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_GPADC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_GPADC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMM_MASK (0x40000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMM_SHIFT (0x0000001EU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMM_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMM_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMMSLV_MASK (0x80000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMMSLV_SHIFT (0x0000001FU)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMMSLV_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_DMMSLV_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_RESETVAL                     (0x00000000U)

/* MSS_BUS_SAFETY_SEC_ERR_STAT1 */

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS2RSS_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS2RSS_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS2RSS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS2RSS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2MSS_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2MSS_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2MSS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2MSS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_TO_MDO_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_TO_MDO_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_TO_MDO_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_BUS_SAFETY_SEC_ERR_STAT1_MSS_TO_MDO_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT1_RESETVAL                     (0x00000000U)

/* HW_REG0 */

#define SDL_MSS_CTRL_HW_REG0_HW_REG0_HWREG0_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG0_HW_REG0_HWREG0_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG0_HW_REG0_HWREG0_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG0_HW_REG0_HWREG0_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG0_RESETVAL                                          (0x00000000U)

/* HW_REG1 */

#define SDL_MSS_CTRL_HW_REG1_HW_REG1_HWREG1_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG1_HW_REG1_HWREG1_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG1_HW_REG1_HWREG1_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG1_HW_REG1_HWREG1_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG1_RESETVAL                                          (0x00000000U)

/* HW_REG2 */

#define SDL_MSS_CTRL_HW_REG2_HW_REG2_HWREG2_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG2_HW_REG2_HWREG2_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG2_HW_REG2_HWREG2_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG2_HW_REG2_HWREG2_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG2_RESETVAL                                          (0x00000000U)

/* HW_REG3 */

#define SDL_MSS_CTRL_HW_REG3_HW_REG3_HWREG3_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG3_HW_REG3_HWREG3_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG3_HW_REG3_HWREG3_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG3_HW_REG3_HWREG3_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG3_RESETVAL                                          (0x00000000U)

/* HW_REG4 */

#define SDL_MSS_CTRL_HW_REG4_HW_REG4_HWREG4_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG4_HW_REG4_HWREG4_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG4_HW_REG4_HWREG4_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG4_HW_REG4_HWREG4_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG4_RESETVAL                                          (0x00000000U)

/* HW_REG5 */

#define SDL_MSS_CTRL_HW_REG5_HW_REG5_HWREG5_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG5_HW_REG5_HWREG5_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG5_HW_REG5_HWREG5_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG5_HW_REG5_HWREG5_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG5_RESETVAL                                          (0x00000000U)

/* HW_REG6 */

#define SDL_MSS_CTRL_HW_REG6_HW_REG6_HWREG6_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG6_HW_REG6_HWREG6_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG6_HW_REG6_HWREG6_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG6_HW_REG6_HWREG6_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG6_RESETVAL                                          (0x00000000U)

/* HW_REG7 */

#define SDL_MSS_CTRL_HW_REG7_HW_REG7_HWREG7_MASK                               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_REG7_HW_REG7_HWREG7_SHIFT                              (0x00000000U)
#define SDL_MSS_CTRL_HW_REG7_HW_REG7_HWREG7_RESETVAL                           (0x00000000U)
#define SDL_MSS_CTRL_HW_REG7_HW_REG7_HWREG7_MAX                                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_REG7_RESETVAL                                          (0x00000000U)

/* MSS_DMM_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_MSS_DMM_BUS_SAFETY_CTRL_TYPE_MAX  (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL_RESETVAL                          (0x00000007U)

/* MSS_DMM_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SEC_MASK      (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SEC_SHIFT     (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SEC_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SEC_MAX       (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DED_MASK      (0x00000020U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DED_SHIFT     (0x00000005U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DED_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DED_MAX       (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DATA_MASK     (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DATA_SHIFT    (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_DATA_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_MAIN_MASK     (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_MAIN_SHIFT    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_MAIN_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SAFE_MASK     (0xFF000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SAFE_SHIFT    (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_MSS_DMM_BUS_SAFETY_FI_SAFE_MAX      (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI_RESETVAL                            (0x00000000U)

/* MSS_DMM_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_SEC_MASK    (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_SEC_SHIFT   (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_SEC_MAX     (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_DED_MASK    (0xFF000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_DED_SHIFT   (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_MSS_DMM_BUS_SAFETY_ERR_DED_MAX     (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_RESETVAL                           (0x00000000U)

/* MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL                (0x00000000U)

/* MSS_DMM_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                  (0x00000000U)

/* MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL                (0x00000000U)

/* MSS_DMM_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ_RESETVAL                 (0x00000000U)

/* MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL            (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL_RESETVAL                      (0x00000007U)

/* MSS_DMM_SLV_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_MSS_DMM_SLV_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI_RESETVAL                        (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_MSS_DMM_SLV_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_RESETVAL                       (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL            (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD_RESETVAL              (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL            (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ_RESETVAL             (0x00000000U)

/* MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL        (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL_RESETVAL                       (0x00000007U)

/* MSS_TO_MDO_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_MSS_TO_MDO_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI_RESETVAL                         (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_MSS_TO_MDO_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_RESETVAL                        (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL             (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD_RESETVAL               (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL             (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ_RESETVAL              (0x00000000U)

/* MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL         (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* MSS_SCRP_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_MSS_SCRP_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_MSS_SCRP_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL_RESETVAL                     (0x00000007U)

/* MSS_CR5A_AHB_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_MSS_CR5A_AHB_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_MSS_CR5A_AHB_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_CTRL */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL_RESETVAL                     (0x00000007U)

/* MSS_CR5B_AHB_BUS_SAFETY_FI */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_MSS_CR5B_AHB_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_ERR */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_MSS_CR5B_AHB_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DMM_CTRL_REG */

#define SDL_MSS_CTRL_DMM_CTRL_REG_DMM_CTRL_REG_DMM_PAD_SELECT_MASK             (0x00000001U)
#define SDL_MSS_CTRL_DMM_CTRL_REG_DMM_CTRL_REG_DMM_PAD_SELECT_SHIFT            (0x00000000U)
#define SDL_MSS_CTRL_DMM_CTRL_REG_DMM_CTRL_REG_DMM_PAD_SELECT_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_DMM_CTRL_REG_DMM_CTRL_REG_DMM_PAD_SELECT_MAX              (0x00000001U)

#define SDL_MSS_CTRL_DMM_CTRL_REG_RESETVAL                                     (0x00000000U)

/* MSS_CR5A_MBOX_WRITE_DONE */

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_1_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_1_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_2_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_3_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_3_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_4_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_4_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_5_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_5_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_6_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_6_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_7_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_7_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_MSS_CR5A_MBOX_WRITE_DONE_PROC_7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_WRITE_DONE_RESETVAL                         (0x00000000U)

/* MSS_CR5A_MBOX_READ_REQ */

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_0_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_1_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_1_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_1_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_2_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_2_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_3_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_3_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_3_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_4_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_4_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_4_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_5_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_5_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_5_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_6_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_6_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_6_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_7_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_7_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_MSS_CR5A_MBOX_READ_REQ_PROC_7_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_REQ_RESETVAL                           (0x00000000U)

/* MSS_CR5A_MBOX_READ_DONE */

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_1_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_1_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_2_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_3_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_3_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_4_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_4_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_5_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_5_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_6_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_6_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_7_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_7_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_MSS_CR5A_MBOX_READ_DONE_PROC_7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5A_MBOX_READ_DONE_RESETVAL                          (0x00000000U)

/* MSS_CR5B_MBOX_WRITE_DONE */

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_1_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_1_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_2_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_3_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_3_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_4_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_4_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_5_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_5_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_6_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_6_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_7_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_7_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_MSS_CR5B_MBOX_WRITE_DONE_PROC_7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_WRITE_DONE_RESETVAL                         (0x00000000U)

/* MSS_CR5B_MBOX_READ_REQ */

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_0_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_1_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_1_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_1_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_2_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_2_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_3_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_3_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_3_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_4_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_4_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_4_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_5_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_5_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_5_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_6_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_6_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_6_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_7_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_7_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_MSS_CR5B_MBOX_READ_REQ_PROC_7_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_REQ_RESETVAL                           (0x00000000U)

/* MSS_CR5B_MBOX_READ_DONE */

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_1_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_1_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_2_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_2_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_3_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_3_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_3_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_4_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_4_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_4_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_5_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_5_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_5_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_6_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_6_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_6_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_7_MASK (0x10000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_7_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_MSS_CR5B_MBOX_READ_DONE_PROC_7_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_CR5B_MBOX_READ_DONE_RESETVAL                          (0x00000000U)

/* MSS_PBIST_KEY_RST */

#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_KEY_MASK     (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_KEY_SHIFT    (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_KEY_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_KEY_MAX      (0x0000000FU)

#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_RST_MASK     (0x000000F0U)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_RST_SHIFT    (0x00000004U)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_RST_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_RST_MAX      (0x0000000FU)

#define SDL_MSS_CTRL_MSS_PBIST_KEY_RST_RESETVAL                                (0x00000000U)

/* MSS_PBIST_REG0 */

#define SDL_MSS_CTRL_MSS_PBIST_REG0_MSS_PBIST_REG0_PBIST_REG_MASK              (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PBIST_REG0_MSS_PBIST_REG0_PBIST_REG_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_REG0_MSS_PBIST_REG0_PBIST_REG_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_REG0_MSS_PBIST_REG0_PBIST_REG_MAX               (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PBIST_REG0_RESETVAL                                   (0x00000000U)

/* MSS_PBIST_REG1 */

#define SDL_MSS_CTRL_MSS_PBIST_REG1_MSS_PBIST_REG1_PBIST_REG_MASK              (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PBIST_REG1_MSS_PBIST_REG1_PBIST_REG_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_REG1_MSS_PBIST_REG1_PBIST_REG_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_REG1_MSS_PBIST_REG1_PBIST_REG_MAX               (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PBIST_REG1_RESETVAL                                   (0x00000000U)

/* MSS_PBIST_REG2 */

#define SDL_MSS_CTRL_MSS_PBIST_REG2_MSS_PBIST_REG2_PBIST_REG_MASK              (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_PBIST_REG2_MSS_PBIST_REG2_PBIST_REG_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_REG2_MSS_PBIST_REG2_PBIST_REG_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_MSS_PBIST_REG2_MSS_PBIST_REG2_PBIST_REG_MAX               (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_PBIST_REG2_RESETVAL                                   (0x00000000U)

/* MSS_QSPI_CONFIG */

#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_EXT_CLK_MASK              (0x00000007U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_EXT_CLK_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_EXT_CLK_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_EXT_CLK_MAX               (0x00000007U)

#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_CLK_LOOPBACK_MASK         (0x00000700U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_CLK_LOOPBACK_SHIFT        (0x00000008U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_CLK_LOOPBACK_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_MSS_QSPI_CONFIG_CLK_LOOPBACK_MAX          (0x00000007U)

#define SDL_MSS_CTRL_MSS_QSPI_CONFIG_RESETVAL                                  (0x00000000U)

/* MSS_STC_CONTROL */

#define SDL_MSS_CTRL_MSS_STC_CONTROL_MSS_STC_CONTROL_CR5_WFI_OVERIDE_MASK      (0x00000007U)
#define SDL_MSS_CTRL_MSS_STC_CONTROL_MSS_STC_CONTROL_CR5_WFI_OVERIDE_SHIFT     (0x00000000U)
#define SDL_MSS_CTRL_MSS_STC_CONTROL_MSS_STC_CONTROL_CR5_WFI_OVERIDE_RESETVAL  (0x00000000U)
#define SDL_MSS_CTRL_MSS_STC_CONTROL_MSS_STC_CONTROL_CR5_WFI_OVERIDE_MAX       (0x00000007U)

#define SDL_MSS_CTRL_MSS_STC_CONTROL_RESETVAL                                  (0x00000000U)

/* MSS_CTI_TRIG_SEL */

#define SDL_MSS_CTRL_MSS_CTI_TRIG_SEL_MSS_CTI_TRIG_SEL_TRIG8_SEL_MASK          (0x000000FFU)
#define SDL_MSS_CTRL_MSS_CTI_TRIG_SEL_MSS_CTI_TRIG_SEL_TRIG8_SEL_SHIFT         (0x00000000U)
#define SDL_MSS_CTRL_MSS_CTI_TRIG_SEL_MSS_CTI_TRIG_SEL_TRIG8_SEL_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_MSS_CTI_TRIG_SEL_MSS_CTI_TRIG_SEL_TRIG8_SEL_MAX           (0x000000FFU)

#define SDL_MSS_CTRL_MSS_CTI_TRIG_SEL_RESETVAL                                 (0x00000000U)

/* MSS_DBGSS_CTI_TRIG_SEL */

#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG1_MASK  (0x000000FFU)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG1_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG1_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG2_MASK  (0x0000FF00U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG2_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG2_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG3_MASK  (0x00FF0000U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG3_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_MSS_DBGSS_CTI_TRIG_SEL_TRIG3_MAX   (0x000000FFU)

#define SDL_MSS_CTRL_MSS_DBGSS_CTI_TRIG_SEL_RESETVAL                           (0x00000000U)

/* MSS_BOOT_INFO_REG0 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG0_MSS_BOOT_INFO_REG0_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG0_MSS_BOOT_INFO_REG0_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG0_MSS_BOOT_INFO_REG0_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG0_MSS_BOOT_INFO_REG0_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG0_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG1 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG1_MSS_BOOT_INFO_REG1_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG1_MSS_BOOT_INFO_REG1_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG1_MSS_BOOT_INFO_REG1_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG1_MSS_BOOT_INFO_REG1_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG1_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG2 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG2_MSS_BOOT_INFO_REG2_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG2_MSS_BOOT_INFO_REG2_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG2_MSS_BOOT_INFO_REG2_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG2_MSS_BOOT_INFO_REG2_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG2_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG3 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG3_MSS_BOOT_INFO_REG3_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG3_MSS_BOOT_INFO_REG3_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG3_MSS_BOOT_INFO_REG3_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG3_MSS_BOOT_INFO_REG3_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG3_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG4 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG4_MSS_BOOT_INFO_REG4_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG4_MSS_BOOT_INFO_REG4_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG4_MSS_BOOT_INFO_REG4_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG4_MSS_BOOT_INFO_REG4_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG4_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG5 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG5_MSS_BOOT_INFO_REG5_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG5_MSS_BOOT_INFO_REG5_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG5_MSS_BOOT_INFO_REG5_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG5_MSS_BOOT_INFO_REG5_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG5_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG6 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG6_MSS_BOOT_INFO_REG6_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG6_MSS_BOOT_INFO_REG6_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG6_MSS_BOOT_INFO_REG6_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG6_MSS_BOOT_INFO_REG6_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG6_RESETVAL                               (0x00000000U)

/* MSS_BOOT_INFO_REG7 */

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG7_MSS_BOOT_INFO_REG7_CONFIG_MASK         (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG7_MSS_BOOT_INFO_REG7_CONFIG_SHIFT        (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG7_MSS_BOOT_INFO_REG7_CONFIG_RESETVAL     (0x00000000U)
#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG7_MSS_BOOT_INFO_REG7_CONFIG_MAX          (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_BOOT_INFO_REG7_RESETVAL                               (0x00000000U)

/* MSS_TPTC_ECCAGGR_CLK_CNTRL */

#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_RESETVAL (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_RESETVAL (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_A1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_RESETVAL (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_TPTC_B0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_ECCAGGR_CLK_CNTRL_RESETVAL                       (0x00000007U)

/* MSS_PERIPH_ERRAGG_MASK0 */

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_RD_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_RD_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_WR_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_WR_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_RD_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_RD_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_WR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_WR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_MSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_RD_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_RD_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_WR_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_WR_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_RD_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_RD_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_WR_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_WR_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_RD_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_RD_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_WR_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_WR_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_AURORA_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_RD_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_RD_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_WR_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_WR_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_SOC_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_RD_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_RD_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_WR_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_WR_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HSM_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_MASK (0x00080000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_SHIFT (0x00000013U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_RD_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_RD_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_WR_MASK (0x00200000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_WR_SHIFT (0x00000015U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_HWA_CFG_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_MASK (0x00400000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_SHIFT (0x00000016U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_MASK (0x00800000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_SHIFT (0x00000017U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_RD_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_RD_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_WR_MASK (0x08000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_WR_SHIFT (0x0000001BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_MSS_PERIPH_ERRAGG_MASK0_TOP_MDO_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK0_RESETVAL                          (0x00000000U)

/* MSS_PERIPH_ERRAGG_STATUS0 */

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_RD_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_RD_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_WR_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_WR_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_RD_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_RD_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_WR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_WR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_MSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_RD_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_RD_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_WR_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_WR_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_RD_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_RD_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_WR_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_WR_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_RD_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_RD_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_WR_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_WR_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_AURORA_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_SOC_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_RD_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_RD_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_WR_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_WR_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HSM_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_MASK (0x00080000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_SHIFT (0x00000013U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_RD_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_RD_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_WR_MASK (0x00200000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_WR_SHIFT (0x00000015U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_HWA_CFG_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_MASK (0x00400000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_SHIFT (0x00000016U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_MASK (0x00800000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_SHIFT (0x00000017U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_RD_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_RD_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_WR_MASK (0x08000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_WR_SHIFT (0x0000001BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_MSS_PERIPH_ERRAGG_STATUS0_TOP_MDO_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS0_RESETVAL                        (0x00000000U)

/* MSS_PERIPH_ERRAGG_STATUS_RAW0 */

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_RD_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_RD_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_WR_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_WR_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_RD_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_RD_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_WR_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_WR_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_RD_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_RD_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_WR_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_WR_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_AURORA_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_SOC_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HSM_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_MASK (0x00020000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_SHIFT (0x00000011U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_MASK (0x00040000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_SHIFT (0x00000012U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_MASK (0x00080000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_SHIFT (0x00000013U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_RD_MASK (0x00100000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_RD_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_WR_MASK (0x00200000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_WR_SHIFT (0x00000015U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_HWA_CFG_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_MASK (0x00400000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_SHIFT (0x00000016U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_MASK (0x00800000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_SHIFT (0x00000017U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_MASK (0x02000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_SHIFT (0x00000019U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_RD_MASK (0x04000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_RD_SHIFT (0x0000001AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_RD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_RD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_WR_MASK (0x08000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_WR_SHIFT (0x0000001BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_WR_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_MSS_PERIPH_ERRAGG_STATUS_RAW0_TOP_MDO_WR_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW0_RESETVAL                    (0x00000000U)

/* MSS_PERIPH_ERRAGG_MASK1 */

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKA_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKA_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKB_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKB_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKB_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_L2_BANKB_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_DTHE_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_DTHE_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_DTHE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_DTHE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_MBOX_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_MBOX_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_PCRA_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_PCRA_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_PCRA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_PCRA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_QSPI_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_QSPI_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_QSPI_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_QSPI_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5A_AXIS_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5A_AXIS_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5A_AXIS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5A_AXIS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5B_AXIS_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5B_AXIS_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5B_AXIS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_MSS_CR5B_AXIS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKA_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKB_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKB_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKB_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKB_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKC_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKC_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKD_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKD_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_L3_BANKD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA0_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA0_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA1_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA1_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_DMA1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_PROC_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_PROC_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_PROC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_HWA_PROC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_MBOX_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_MBOX_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_DSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_MSS_PERIPH_ERRAGG_MASK1_MPU_RD_HSM_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_MASK1_RESETVAL                          (0x00000000U)

/* MSS_PERIPH_ERRAGG_STATUS1 */

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKA_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKA_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKB_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKB_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKB_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_L2_BANKB_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_DTHE_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_DTHE_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_DTHE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_DTHE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_MBOX_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_MBOX_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_PCRA_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_PCRA_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_PCRA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_PCRA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_QSPI_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_QSPI_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_QSPI_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_QSPI_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5A_AXIS_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5A_AXIS_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5A_AXIS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5A_AXIS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5B_AXIS_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5B_AXIS_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5B_AXIS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_MSS_CR5B_AXIS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKA_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKB_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKB_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKB_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKB_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKC_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKC_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKD_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKD_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_L3_BANKD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA0_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA0_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA1_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA1_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_DMA1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_PROC_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_PROC_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_PROC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_HWA_PROC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_MBOX_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_MBOX_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_DSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_MSS_PERIPH_ERRAGG_STATUS1_MPU_RD_HSM_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS1_RESETVAL                        (0x00000000U)

/* MSS_PERIPH_ERRAGG_STATUS_RAW1 */

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKA_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKA_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKB_MASK (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKB_SHIFT (0x00000001U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKB_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_L2_BANKB_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_DTHE_MASK (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_DTHE_SHIFT (0x00000002U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_DTHE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_DTHE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_MBOX_MASK (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_MBOX_SHIFT (0x00000003U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_PCRA_MASK (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_PCRA_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_PCRA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_PCRA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_QSPI_MASK (0x00000020U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_QSPI_SHIFT (0x00000005U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_QSPI_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_QSPI_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5A_AXIS_MASK (0x00000040U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5A_AXIS_SHIFT (0x00000006U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5A_AXIS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5A_AXIS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5B_AXIS_MASK (0x00000080U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5B_AXIS_SHIFT (0x00000007U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5B_AXIS_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_MSS_CR5B_AXIS_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKA_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKA_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKA_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKA_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKB_MASK (0x00000200U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKB_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKB_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKB_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKC_MASK (0x00000400U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKC_SHIFT (0x0000000AU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKD_MASK (0x00000800U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKD_SHIFT (0x0000000BU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKD_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_L3_BANKD_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA0_MASK (0x00001000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA0_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA0_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA1_MASK (0x00002000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA1_SHIFT (0x0000000DU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_DMA1_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_PROC_MASK (0x00004000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_PROC_SHIFT (0x0000000EU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_PROC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_HWA_PROC_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_MBOX_MASK (0x00008000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_MBOX_SHIFT (0x0000000FU)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_MBOX_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_DSS_MBOX_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_MSS_PERIPH_ERRAGG_STATUS_RAW1_MPU_RD_HSM_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_PERIPH_ERRAGG_STATUS_RAW1_RESETVAL                    (0x00000000U)

/* MSS_DMM_EVENT0_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG0_MASK    (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG0_SHIFT   (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG0_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL0_MASK     (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL0_SHIFT    (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL0_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL0_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG1_MASK    (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG1_SHIFT   (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG1_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL1_MASK     (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL1_SHIFT    (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL1_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL1_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG2_MASK    (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG2_SHIFT   (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG2_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL2_MASK     (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL2_SHIFT    (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL2_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL2_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG3_MASK    (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG3_SHIFT   (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_TRIG3_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL3_MASK     (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL3_SHIFT    (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL3_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_MSS_DMM_EVENT0_REG_EVENT_SEL3_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT0_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT1_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG4_MASK    (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG4_SHIFT   (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG4_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL4_MASK     (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL4_SHIFT    (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL4_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL4_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG5_MASK    (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG5_SHIFT   (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG5_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL5_MASK     (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL5_SHIFT    (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL5_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL5_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG6_MASK    (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG6_SHIFT   (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG6_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL6_MASK     (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL6_SHIFT    (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL6_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL6_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG7_MASK    (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG7_SHIFT   (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_TRIG7_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL7_MASK     (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL7_SHIFT    (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL7_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_MSS_DMM_EVENT1_REG_EVENT_SEL7_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT1_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT2_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG8_MASK    (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG8_SHIFT   (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG8_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG8_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL8_MASK     (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL8_SHIFT    (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL8_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL8_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG9_MASK    (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG9_SHIFT   (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG9_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG9_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL9_MASK     (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL9_SHIFT    (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL9_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL9_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG10_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG10_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG10_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG10_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL10_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL10_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL10_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL10_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG11_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG11_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG11_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_TRIG11_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL11_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL11_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL11_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_MSS_DMM_EVENT2_REG_EVENT_SEL11_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT2_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT3_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG12_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG12_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG12_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG12_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL12_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL12_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL12_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL12_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG13_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG13_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG13_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG13_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL13_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL13_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL13_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL13_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG14_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG14_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG14_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG14_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL14_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL14_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL14_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL14_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG15_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG15_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG15_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_TRIG15_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL15_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL15_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL15_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_MSS_DMM_EVENT3_REG_EVENT_SEL15_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT3_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT4_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG16_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG16_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG16_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG16_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL16_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL16_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL16_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL16_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG17_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG17_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG17_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG17_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL17_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL17_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL17_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL17_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG18_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG18_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG18_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG18_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL18_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL18_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL18_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL18_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG19_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG19_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG19_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_TRIG19_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL19_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL19_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL19_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_MSS_DMM_EVENT4_REG_EVENT_SEL19_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT4_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT5_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG20_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG20_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG20_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG20_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL20_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL20_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL20_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL20_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG21_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG21_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG21_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG21_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL21_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL21_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL21_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL21_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG22_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG22_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG22_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG22_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL22_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL22_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL22_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL22_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG23_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG23_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG23_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_TRIG23_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL23_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL23_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL23_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_MSS_DMM_EVENT5_REG_EVENT_SEL23_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT5_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT6_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG24_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG24_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG24_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG24_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL24_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL24_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL24_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL24_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG25_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG25_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG25_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG25_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL25_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL25_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL25_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL25_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG26_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG26_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG26_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG26_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL26_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL26_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL26_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL26_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG27_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG27_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG27_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_TRIG27_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL27_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL27_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL27_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_MSS_DMM_EVENT6_REG_EVENT_SEL27_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT6_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT7_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG28_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG28_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG28_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG28_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL28_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL28_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL28_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL28_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG29_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG29_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG29_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG29_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL29_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL29_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL29_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL29_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG30_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG30_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG30_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG30_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL30_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL30_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL30_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL30_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG31_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG31_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG31_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_TRIG31_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL31_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL31_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL31_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_MSS_DMM_EVENT7_REG_EVENT_SEL31_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT7_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT8_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG32_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG32_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG32_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG32_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL32_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL32_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL32_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL32_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG33_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG33_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG33_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG33_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL33_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL33_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL33_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL33_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG34_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG34_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG34_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG34_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL34_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL34_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL34_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL34_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG35_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG35_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG35_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_TRIG35_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL35_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL35_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL35_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_MSS_DMM_EVENT8_REG_EVENT_SEL35_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT8_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT9_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG36_MASK   (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG36_SHIFT  (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG36_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG36_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL36_MASK    (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL36_SHIFT   (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL36_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL36_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG37_MASK   (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG37_SHIFT  (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG37_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG37_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL37_MASK    (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL37_SHIFT   (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL37_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL37_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG38_MASK   (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG38_SHIFT  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG38_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG38_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL38_MASK    (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL38_SHIFT   (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL38_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL38_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG39_MASK   (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG39_SHIFT  (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG39_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_TRIG39_MAX    (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL39_MASK    (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL39_SHIFT   (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL39_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_MSS_DMM_EVENT9_REG_EVENT_SEL39_MAX     (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT9_REG_RESETVAL                               (0x00000000U)

/* MSS_DMM_EVENT10_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG40_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG40_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG40_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG40_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL40_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL40_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL40_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL40_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG41_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG41_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG41_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG41_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL41_MASK  (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL41_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL41_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL41_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG42_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG42_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG42_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG42_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL42_MASK  (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL42_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL42_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL42_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG43_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG43_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG43_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_TRIG43_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL43_MASK  (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL43_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL43_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_MSS_DMM_EVENT10_REG_EVENT_SEL43_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT10_REG_RESETVAL                              (0x00000000U)

/* MSS_DMM_EVENT11_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG44_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG44_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG44_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG44_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL44_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL44_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL44_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL44_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG45_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG45_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG45_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG45_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL45_MASK  (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL45_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL45_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL45_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG46_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG46_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG46_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG46_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL46_MASK  (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL46_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL46_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL46_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG47_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG47_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG47_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_TRIG47_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL47_MASK  (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL47_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL47_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_MSS_DMM_EVENT11_REG_EVENT_SEL47_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT11_REG_RESETVAL                              (0x00000000U)

/* MSS_DMM_EVENT12_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG48_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG48_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG48_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG48_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL48_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL48_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL48_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL48_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG49_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG49_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG49_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG49_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL49_MASK  (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL49_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL49_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL49_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG50_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG50_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG50_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG50_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL50_MASK  (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL50_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL50_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL50_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG51_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG51_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG51_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_TRIG51_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL51_MASK  (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL51_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL51_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_MSS_DMM_EVENT12_REG_EVENT_SEL51_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT12_REG_RESETVAL                              (0x00000000U)

/* MSS_DMM_EVENT13_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG52_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG52_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG52_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG52_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL52_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL52_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL52_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL52_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG53_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG53_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG53_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG53_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL53_MASK  (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL53_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL53_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL53_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG54_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG54_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG54_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG54_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL54_MASK  (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL54_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL54_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL54_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG55_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG55_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG55_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_TRIG55_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL55_MASK  (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL55_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL55_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_MSS_DMM_EVENT13_REG_EVENT_SEL55_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT13_REG_RESETVAL                              (0x00000000U)

/* MSS_DMM_EVENT14_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG56_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG56_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG56_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG56_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL56_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL56_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL56_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL56_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG57_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG57_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG57_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG57_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL57_MASK  (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL57_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL57_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL57_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG58_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG58_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG58_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG58_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL58_MASK  (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL58_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL58_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL58_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG59_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG59_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG59_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_TRIG59_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL59_MASK  (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL59_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL59_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_MSS_DMM_EVENT14_REG_EVENT_SEL59_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT14_REG_RESETVAL                              (0x00000000U)

/* MSS_DMM_EVENT15_REG */

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG60_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG60_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG60_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG60_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL60_MASK  (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL60_SHIFT (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL60_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL60_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG61_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG61_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG61_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG61_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL61_MASK  (0x00001000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL61_SHIFT (0x0000000CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL61_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL61_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG62_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG62_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG62_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG62_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL62_MASK  (0x00100000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL62_SHIFT (0x00000014U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL62_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL62_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG63_MASK (0x01000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG63_SHIFT (0x00000018U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG63_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_TRIG63_MAX  (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL63_MASK  (0x10000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL63_SHIFT (0x0000001CU)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL63_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_MSS_DMM_EVENT15_REG_EVENT_SEL63_MAX   (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_EVENT15_REG_RESETVAL                              (0x00000000U)

/* MSS_TPTC_BOUNDARY_CFG */

#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MASK (0x0000003FU)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_RESETVAL (0x00000012U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MAX (0x0000003FU)

#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_MASK (0x00003F00U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_RESETVAL (0x00000012U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_A1_SIZE_MAX (0x0000003FU)

#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_MASK (0x003F0000U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_RESETVAL (0x00000012U)
#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_MSS_TPTC_BOUNDARY_CFG_TPTC_B0_SIZE_MAX (0x0000003FU)

#define SDL_MSS_CTRL_MSS_TPTC_BOUNDARY_CFG_RESETVAL                            (0x00121212U)

/* MSS_TPTC_XID_REORDER_CFG */

#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MASK (0x00000001U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_MASK (0x00000100U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_A1_DISABLE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_MASK (0x00010000U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_SHIFT (0x00000010U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_MSS_TPTC_XID_REORDER_CFG_TPTC_B0_DISABLE_MAX (0x00000001U)

#define SDL_MSS_CTRL_MSS_TPTC_XID_REORDER_CFG_RESETVAL                         (0x00000000U)

/* GPADC_CTRL */

#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_SW_TRIG_MASK                  (0x00000001U)
#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_SW_TRIG_SHIFT                 (0x00000000U)
#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_SW_TRIG_RESETVAL              (0x00000000U)
#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_SW_TRIG_MAX                   (0x00000001U)

#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_TRIGIN_SEL_MASK               (0x00001F00U)
#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_TRIGIN_SEL_SHIFT              (0x00000008U)
#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_TRIGIN_SEL_RESETVAL           (0x0000000FU)
#define SDL_MSS_CTRL_GPADC_CTRL_GPADC_CTRL_GPADC_TRIGIN_SEL_MAX                (0x0000001FU)

#define SDL_MSS_CTRL_GPADC_CTRL_RESETVAL                                       (0x00000F00U)

/* HW_SYNC_FE_CTRL */

#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_MASK              (0x00000001U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE1_SEL_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_MASK              (0x00000100U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_SHIFT             (0x00000008U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_HW_SYNC_FE_CTRL_FE2_SEL_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SYNC_FE_CTRL_RESETVAL                                  (0x00000000U)

/* DEBUGSS_CSETB_FLUSH */

#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_MASK (0x00000001U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHIN_MAX (0x00000001U)

#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_MASK (0x00000100U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_SHIFT (0x00000008U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FLUSHINACK_MAX (0x00000001U)

#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_MASK (0x00000200U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_SHIFT (0x00000009U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_ACQ_COMPLETE_MAX (0x00000001U)

#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_MASK   (0x00000400U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_SHIFT  (0x0000000AU)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_DEBUGSS_CSETB_FLUSH_CSETB_FULL_MAX    (0x00000001U)

#define SDL_MSS_CTRL_DEBUGSS_CSETB_FLUSH_RESETVAL                              (0x00000000U)

/* ANALOG_WU_STATUS_REG_POLARITY_INV */

#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_POLARITY_INV_ANALOG_WU_STATUS_REG_POLARITY_INV_INV_CTRL_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_POLARITY_INV_ANALOG_WU_STATUS_REG_POLARITY_INV_INV_CTRL_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_POLARITY_INV_ANALOG_WU_STATUS_REG_POLARITY_INV_INV_CTRL_RESETVAL (0x00003D5CU)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_POLARITY_INV_ANALOG_WU_STATUS_REG_POLARITY_INV_INV_CTRL_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_POLARITY_INV_RESETVAL                (0x00003D5CU)

/* ANALOG_CLK_STATUS_REG_POLARITY_INV */

#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_POLARITY_INV_ANALOG_CLK_STATUS_REG_POLARITY_INV_INV_CTRL_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_POLARITY_INV_ANALOG_CLK_STATUS_REG_POLARITY_INV_INV_CTRL_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_POLARITY_INV_ANALOG_CLK_STATUS_REG_POLARITY_INV_INV_CTRL_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_POLARITY_INV_ANALOG_CLK_STATUS_REG_POLARITY_INV_INV_CTRL_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_POLARITY_INV_RESETVAL               (0x00000000U)

/* ANALOG_WU_STATUS_REG_GRP1_MASK */

#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP1_MASK_ANALOG_WU_STATUS_REG_GRP1_MASK_MASK_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP1_MASK_ANALOG_WU_STATUS_REG_GRP1_MASK_MASK_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP1_MASK_ANALOG_WU_STATUS_REG_GRP1_MASK_MASK_RESETVAL (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP1_MASK_ANALOG_WU_STATUS_REG_GRP1_MASK_MASK_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP1_MASK_RESETVAL                   (0xFFFFFFFFU)

/* ANALOG_CLK_STATUS_REG_GRP1_MASK */

#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK_ANALOG_CLK_STATUS_REG_GRP1_MASK_MASK_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK_ANALOG_CLK_STATUS_REG_GRP1_MASK_MASK_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK_ANALOG_CLK_STATUS_REG_GRP1_MASK_MASK_RESETVAL (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK_ANALOG_CLK_STATUS_REG_GRP1_MASK_MASK_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK_RESETVAL                  (0xFFFFFFFFU)

/* ANALOG_WU_STATUS_REG_GRP2_MASK */

#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK_ANALOG_WU_STATUS_REG_GRP2_MASK_MASK_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK_ANALOG_WU_STATUS_REG_GRP2_MASK_MASK_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK_ANALOG_WU_STATUS_REG_GRP2_MASK_MASK_RESETVAL (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK_ANALOG_WU_STATUS_REG_GRP2_MASK_MASK_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK_RESETVAL                   (0xFFFFFFFFU)

/* ANALOG_CLK_STATUS_REG_GRP2_MASK */

#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP2_MASK_ANALOG_CLK_STATUS_REG_GRP2_MASK_MASK_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP2_MASK_ANALOG_CLK_STATUS_REG_GRP2_MASK_MASK_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP2_MASK_ANALOG_CLK_STATUS_REG_GRP2_MASK_MASK_RESETVAL (0xFFFFFFFFU)
#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP2_MASK_ANALOG_CLK_STATUS_REG_GRP2_MASK_MASK_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP2_MASK_RESETVAL                  (0xFFFFFFFFU)

/* NERROR_MASK */

#define SDL_MSS_CTRL_NERROR_MASK_NERROR_MASK_MASK_MASK                         (0x00000007U)
#define SDL_MSS_CTRL_NERROR_MASK_NERROR_MASK_MASK_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_NERROR_MASK_NERROR_MASK_MASK_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_NERROR_MASK_NERROR_MASK_MASK_MAX                          (0x00000007U)

#define SDL_MSS_CTRL_NERROR_MASK_RESETVAL                                      (0x00000000U)

/* MSS_DMM_ACCESS_MODE */

#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMA_SEL_MASK     (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMA_SEL_SHIFT    (0x00000000U)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMA_SEL_RESETVAL (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMA_SEL_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMB_SEL_MASK     (0x00000010U)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMB_SEL_SHIFT    (0x00000004U)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMB_SEL_RESETVAL (0x00000001U)
#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_MSS_DMM_ACCESS_MODE_DMMB_SEL_MAX      (0x00000001U)

#define SDL_MSS_CTRL_MSS_DMM_ACCESS_MODE_RESETVAL                              (0x00000011U)

/* R5_CONTROL */

#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_MASK                      (0x00000007U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SHIFT                     (0x00000000U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_RESETVAL                  (0x00000007U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_MAX                       (0x00000007U)

#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT_MASK          (0x00000700U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT_SHIFT         (0x00000008U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT_MAX           (0x00000007U)

#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_RESET_FSM_TRIGGER_MASK              (0x00070000U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_RESET_FSM_TRIGGER_SHIFT             (0x00000010U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_RESET_FSM_TRIGGER_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_RESET_FSM_TRIGGER_MAX               (0x00000007U)

#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_ROM_WAIT_STATE_MASK                 (0x07000000U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_ROM_WAIT_STATE_SHIFT                (0x00000018U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_ROM_WAIT_STATE_RESETVAL             (0x00000000U)
#define SDL_MSS_CTRL_R5_CONTROL_R5_CONTROL_ROM_WAIT_STATE_MAX                  (0x00000007U)

#define SDL_MSS_CTRL_R5_CONTROL_RESETVAL                                       (0x00000007U)

/* R5_ROM_ECLIPSE */

#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_MASK                (0x00000007U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_SHIFT               (0x00000000U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_MAX                 (0x00000007U)

#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_WAIT_MASK           (0x00000700U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_WAIT_SHIFT          (0x00000008U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_WAIT_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_R5_ROM_ECLIPSE_MEMSWAP_WAIT_MAX            (0x00000007U)

#define SDL_MSS_CTRL_R5_ROM_ECLIPSE_RESETVAL                                   (0x00000000U)

/* R5_COREA_HALT */

#define SDL_MSS_CTRL_R5_COREA_HALT_R5_COREA_HALT_HALT_MASK                     (0x00000007U)
#define SDL_MSS_CTRL_R5_COREA_HALT_R5_COREA_HALT_HALT_SHIFT                    (0x00000000U)
#define SDL_MSS_CTRL_R5_COREA_HALT_R5_COREA_HALT_HALT_RESETVAL                 (0x00000007U)
#define SDL_MSS_CTRL_R5_COREA_HALT_R5_COREA_HALT_HALT_MAX                      (0x00000007U)

#define SDL_MSS_CTRL_R5_COREA_HALT_RESETVAL                                    (0x00000007U)

/* R5_COREB_HALT */

#define SDL_MSS_CTRL_R5_COREB_HALT_R5_COREB_HALT_HALT_MASK                     (0x00000007U)
#define SDL_MSS_CTRL_R5_COREB_HALT_R5_COREB_HALT_HALT_SHIFT                    (0x00000000U)
#define SDL_MSS_CTRL_R5_COREB_HALT_R5_COREB_HALT_HALT_RESETVAL                 (0x00000007U)
#define SDL_MSS_CTRL_R5_COREB_HALT_R5_COREB_HALT_HALT_MAX                      (0x00000007U)

#define SDL_MSS_CTRL_R5_COREB_HALT_RESETVAL                                    (0x00000007U)

/* R5_STATUS_REG */

#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_MEMSWAP_MASK                  (0x00000001U)
#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_MEMSWAP_SHIFT                 (0x00000000U)
#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_MEMSWAP_RESETVAL              (0x00000000U)
#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_MEMSWAP_MAX                   (0x00000001U)

#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_LOCK_STEP_MASK                (0x00000100U)
#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_LOCK_STEP_SHIFT               (0x00000008U)
#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_LOCK_STEP_RESETVAL            (0x00000000U)
#define SDL_MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_LOCK_STEP_MAX                 (0x00000001U)

#define SDL_MSS_CTRL_R5_STATUS_REG_RESETVAL                                    (0x00000000U)

/* HW_SPARE_RW0 */

#define SDL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RW0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW1 */

#define SDL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RW1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW2 */

#define SDL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RW2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW3 */

#define SDL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RW3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO0 */

#define SDL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RO0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO1 */

#define SDL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RO1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO2 */

#define SDL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RO2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO3 */

#define SDL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK               (0xFFFFFFFFU)
#define SDL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT              (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL           (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                (0xFFFFFFFFU)

#define SDL_MSS_CTRL_HW_SPARE_RO3_RESETVAL                                     (0x00000000U)

/* MSS_CR5A_B_MBOX_READ_DONE_ACK */

#define SDL_MSS_CTRL_MSS_CR5A_B_MBOX_READ_DONE_ACK_MSS_CR5A_B_MBOX_READ_DONE_ACK_PROC_MASK (0xFFFFFFFFU)
#define SDL_MSS_CTRL_MSS_CR5A_B_MBOX_READ_DONE_ACK_MSS_CR5A_B_MBOX_READ_DONE_ACK_PROC_SHIFT (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_B_MBOX_READ_DONE_ACK_MSS_CR5A_B_MBOX_READ_DONE_ACK_PROC_RESETVAL (0x00000000U)
#define SDL_MSS_CTRL_MSS_CR5A_B_MBOX_READ_DONE_ACK_MSS_CR5A_B_MBOX_READ_DONE_ACK_PROC_MAX (0xFFFFFFFFU)

#define SDL_MSS_CTRL_MSS_CR5A_B_MBOX_READ_DONE_ACK_RESETVAL                    (0x00000000U)

/* HW_SPARE_REC */

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK              (0x00000001U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT             (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK              (0x00000002U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT             (0x00000001U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK              (0x00000004U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT             (0x00000002U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK              (0x00000008U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT             (0x00000003U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK              (0x00000010U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT             (0x00000004U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK              (0x00000020U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT             (0x00000005U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK              (0x00000040U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT             (0x00000006U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK              (0x00000080U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT             (0x00000007U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK              (0x00000100U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT             (0x00000008U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK              (0x00000200U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT             (0x00000009U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL          (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX               (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK             (0x00000400U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT            (0x0000000AU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK             (0x00000800U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT            (0x0000000BU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK             (0x00001000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT            (0x0000000CU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK             (0x00002000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT            (0x0000000DU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK             (0x00004000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT            (0x0000000EU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK             (0x00008000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT            (0x0000000FU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK             (0x00010000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT            (0x00000010U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK             (0x00020000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT            (0x00000011U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK             (0x00040000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT            (0x00000012U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK             (0x00080000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT            (0x00000013U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK             (0x00100000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT            (0x00000014U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK             (0x00200000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT            (0x00000015U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK             (0x00400000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT            (0x00000016U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK             (0x00800000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT            (0x00000017U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK             (0x01000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT            (0x00000018U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK             (0x02000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT            (0x00000019U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK             (0x04000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT            (0x0000001AU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK             (0x08000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT            (0x0000001BU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK             (0x10000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT            (0x0000001CU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK             (0x20000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT            (0x0000001DU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK             (0x40000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT            (0x0000001EU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK             (0x80000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT            (0x0000001FU)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL         (0x00000000U)
#define SDL_MSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX              (0x00000001U)

#define SDL_MSS_CTRL_HW_SPARE_REC_RESETVAL                                     (0x00000000U)

/* LOCK0_KICK0 */

#define SDL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                              (0xFFFFFFFFU)
#define SDL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                             (0x00000000U)
#define SDL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                          (0x00000000U)
#define SDL_MSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                               (0xFFFFFFFFU)

#define SDL_MSS_CTRL_LOCK0_KICK0_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK1 */

#define SDL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                              (0xFFFFFFFFU)
#define SDL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                             (0x00000000U)
#define SDL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                          (0x00000000U)
#define SDL_MSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                               (0xFFFFFFFFU)

#define SDL_MSS_CTRL_LOCK0_KICK1_RESETVAL                                      (0x00000000U)

/* INTR_RAW_STATUS */

#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                             (0x00000001U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                            (0x00000000U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                         (0x00000000U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                              (0x00000001U)

#define SDL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                             (0x00000002U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                            (0x00000001U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                         (0x00000000U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                              (0x00000001U)

#define SDL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                             (0x00000004U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                            (0x00000002U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                         (0x00000000U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                              (0x00000001U)

#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                            (0x00000008U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                           (0x00000003U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                        (0x00000000U)
#define SDL_MSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                             (0x00000001U)

#define SDL_MSS_CTRL_INTR_RAW_STATUS_RESETVAL                                  (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK           (0x00000001U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT          (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX            (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK           (0x00000002U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT          (0x00000001U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX            (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK           (0x00000004U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT          (0x00000002U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL       (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX            (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK          (0x00000008U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT         (0x00000003U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL      (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX           (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                        (0x00000000U)

/* INTR_ENABLE */

#define SDL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                              (0x00000001U)
#define SDL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                             (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                               (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                              (0x00000002U)
#define SDL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                             (0x00000001U)
#define SDL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                               (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                              (0x00000004U)
#define SDL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                             (0x00000002U)
#define SDL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                               (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                             (0x00000008U)
#define SDL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                            (0x00000003U)
#define SDL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                         (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                              (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_RESETVAL                                      (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                    (0x00000001U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                   (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                    (0x00000002U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                   (0x00000001U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                    (0x00000004U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                   (0x00000002U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                   (0x00000008U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                  (0x00000003U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                    (0x00000001U)

#define SDL_MSS_CTRL_INTR_ENABLE_CLEAR_RESETVAL                                (0x00000000U)

/* EOI */

#define SDL_MSS_CTRL_EOI_EOI_VECTOR_MASK                                       (0x000000FFU)
#define SDL_MSS_CTRL_EOI_EOI_VECTOR_SHIFT                                      (0x00000000U)
#define SDL_MSS_CTRL_EOI_EOI_VECTOR_RESETVAL                                   (0x00000000U)
#define SDL_MSS_CTRL_EOI_EOI_VECTOR_MAX                                        (0x000000FFU)

#define SDL_MSS_CTRL_EOI_RESETVAL                                              (0x00000000U)

/* FAULT_ADDRESS */

#define SDL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                             (0xFFFFFFFFU)
#define SDL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                            (0x00000000U)
#define SDL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                         (0x00000000U)
#define SDL_MSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                              (0xFFFFFFFFU)

#define SDL_MSS_CTRL_FAULT_ADDRESS_RESETVAL                                    (0x00000000U)

/* FAULT_TYPE_STATUS */

#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                         (0x0000003FU)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                        (0x00000000U)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                     (0x00000000U)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                          (0x0000003FU)

#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                           (0x00000040U)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                          (0x00000006U)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                       (0x00000000U)
#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                            (0x00000001U)

#define SDL_MSS_CTRL_FAULT_TYPE_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_ATTR_STATUS */

#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                       (0x000000FFU)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                      (0x00000000U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                   (0x00000000U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                        (0x000000FFU)

#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                      (0x000FFF00U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                     (0x00000008U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                  (0x00000000U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                       (0x00000FFFU)

#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                          (0xFFF00000U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                         (0x00000014U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                      (0x00000000U)
#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                           (0x00000FFFU)

#define SDL_MSS_CTRL_FAULT_ATTR_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_CLEAR */

#define SDL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                                (0x00000001U)
#define SDL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                               (0x00000000U)
#define SDL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                            (0x00000000U)
#define SDL_MSS_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                                 (0x00000001U)

#define SDL_MSS_CTRL_FAULT_CLEAR_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
