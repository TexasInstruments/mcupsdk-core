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
 *  Name        : sdlr_dss_ctrl.h
*/
#ifndef SDLR_DSS_CTRL_H_
#define SDLR_DSS_CTRL_H_

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
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t HW_REG2;
    volatile uint32_t HW_REG3;
    volatile uint32_t DSS_SW_INT;
    volatile uint32_t DSS_TPCC_A_ERRAGG_MASK;
    volatile uint32_t DSS_TPCC_A_ERRAGG_STATUS;
    volatile uint32_t DSS_TPCC_A_ERRAGG_STATUS_RAW;
    volatile uint32_t DSS_TPCC_A_INTAGG_MASK;
    volatile uint32_t DSS_TPCC_A_INTAGG_STATUS;
    volatile uint32_t DSS_TPCC_A_INTAGG_STATUS_RAW;
    volatile uint32_t DSS_TPCC_B_ERRAGG_MASK;
    volatile uint32_t DSS_TPCC_B_ERRAGG_STATUS;
    volatile uint32_t DSS_TPCC_B_ERRAGG_STATUS_RAW;
    volatile uint32_t DSS_TPCC_B_INTAGG_MASK;
    volatile uint32_t DSS_TPCC_B_INTAGG_STATUS;
    volatile uint32_t DSS_TPCC_B_INTAGG_STATUS_RAW;
    volatile uint32_t DSS_TPCC_C_ERRAGG_MASK;
    volatile uint32_t DSS_TPCC_C_ERRAGG_STATUS;
    volatile uint32_t DSS_TPCC_C_ERRAGG_STATUS_RAW;
    volatile uint32_t DSS_TPCC_C_INTAGG_MASK;
    volatile uint32_t DSS_TPCC_C_INTAGG_STATUS;
    volatile uint32_t DSS_TPCC_C_INTAGG_STATUS_RAW;
    volatile uint32_t DSS_TPCC_MEMINIT_START;
    volatile uint32_t DSS_TPCC_MEMINIT_STATUS;
    volatile uint32_t DSS_TPCC_MEMINIT_DONE;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_CTRL;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3;
    volatile uint32_t DSS_DSP_L2RAM_MEMINIT_START;
    volatile uint32_t DSS_DSP_L2RAM_MEMINIT_STATUS;
    volatile uint32_t DSS_DSP_L2RAM_MEMINIT_DONE;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_MEMINIT_START;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS;
    volatile uint32_t DSS_DSP_L2RAM_PARITY_MEMINIT_DONE;
    volatile uint32_t DSS_L3RAM_MEMINIT_START;
    volatile uint32_t DSS_L3RAM_MEMINIT_STATUS;
    volatile uint32_t DSS_L3RAM_MEMINIT_DONE;
    volatile uint8_t  Resv_176[12];
    volatile uint32_t DSS_MAILBOX_MEMINIT_START;
    volatile uint32_t DSS_MAILBOX_MEMINIT_STATUS;
    volatile uint32_t DSS_MAILBOX_MEMINIT_DONE;
    volatile uint32_t DSS_TPCC_A_PARITY_CTRL;
    volatile uint32_t DSS_TPCC_B_PARITY_CTRL;
    volatile uint32_t DSS_TPCC_C_PARITY_CTRL;
    volatile uint32_t DSS_TPCC_A_PARITY_STATUS;
    volatile uint32_t DSS_TPCC_B_PARITY_STATUS;
    volatile uint32_t DSS_TPCC_C_PARITY_STATUS;
    volatile uint32_t TPTC_DBS_CONFIG;
    volatile uint32_t DSS_DSP_BOOTCFG;
    volatile uint32_t DSS_DSP_NMI_GATE;
    volatile uint32_t DSS_PBIST_KEY_RESET;
    volatile uint32_t DSS_PBIST_REG0;
    volatile uint32_t DSS_PBIST_REG1;
    volatile uint32_t DSS_TPTC_BOUNDARY_CFG0;
    volatile uint32_t DSS_TPTC_BOUNDARY_CFG1;
    volatile uint32_t DSS_TPTC_BOUNDARY_CFG2;
    volatile uint32_t DSS_TPTC_XID_REORDER_CFG0;
    volatile uint32_t DSS_TPTC_XID_REORDER_CFG1;
    volatile uint32_t DSS_TPTC_XID_REORDER_CFG2;
    volatile uint8_t  Resv_264[4];
    volatile uint32_t ESM_GATING0;
    volatile uint32_t ESM_GATING1;
    volatile uint32_t ESM_GATING2;
    volatile uint32_t ESM_GATING3;
    volatile uint8_t  Resv_1376[1096];
    volatile uint32_t DSS_PERIPH_ERRAGG_MASK0;
    volatile uint32_t DSS_PERIPH_ERRAGG_STATUS0;
    volatile uint32_t DSS_PERIPH_ERRAGG_STATUS_RAW0;
    volatile uint32_t DSS_DSP_MBOX_WRITE_DONE;
    volatile uint32_t DSS_DSP_MBOX_READ_REQ;
    volatile uint32_t DSS_DSP_MBOX_READ_DONE;
    volatile uint32_t DSS_WDT_EVENT_CAPTURE_SEL;
    volatile uint32_t DSS_RTIA_EVENT_CAPTURE_SEL;
    volatile uint32_t DSS_RTIB_EVENT_CAPTURE_SEL;
    volatile uint32_t DBG_ACK_CPU_CTRL;
    volatile uint32_t DBG_ACK_CTL0;
    volatile uint32_t DBG_ACK_CTL1;
    volatile uint32_t DSS_DSP_INT_SEL;
    volatile uint32_t DSS_CBUFF_TRIGGER_SEL;
    volatile uint8_t  Resv_2048[616];
    volatile uint32_t DSS_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_BUS_SAFETY_SEC_ERR_STAT0;
    volatile uint32_t DSS_BUS_SAFETY_SEC_ERR_STAT1;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_FI;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_FI;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_FI;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_FI;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_FI;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_FI;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_ERR;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_A0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_A0_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_A0_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_A1_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_A1_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_A1_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_B0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_B0_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_B0_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_B1_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_B1_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_B1_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_C0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C0_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C0_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_C1_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C1_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C1_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_C2_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C2_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C2_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_C3_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C3_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C3_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_C4_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C4_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C4_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_C5_RD_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C5_RD_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C5_RD_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_FI;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_FI;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_ERR;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_FI;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_FI;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_FI;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_FI;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_FI;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_FI;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_FI;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_ERR;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_PCR_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_PCR_BUS_SAFETY_FI;
    volatile uint32_t DSS_PCR_BUS_SAFETY_ERR;
    volatile uint32_t DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_PCR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_PCR_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_FI;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_ERR;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_FI;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_ERR;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_FI;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_FI;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_ERR;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_CTRL;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_FI;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_ERR;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_4048[808];
    volatile uint32_t HW_SPARE_RW0;
    volatile uint32_t HW_SPARE_RW1;
    volatile uint32_t HW_SPARE_RW2;
    volatile uint32_t HW_SPARE_RW3;
    volatile uint32_t HW_SPARE_RO0;
    volatile uint32_t HW_SPARE_RO1;
    volatile uint32_t HW_SPARE_RO2;
    volatile uint32_t HW_SPARE_RO3;
    volatile uint32_t DSS_DSP_MBOX_READ_DONE_ACK;
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
} SDL_dss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_DSS_CTRL_PID                                                       (0x00000000U)
#define SDL_DSS_CTRL_HW_REG0                                                   (0x00000004U)
#define SDL_DSS_CTRL_HW_REG1                                                   (0x00000008U)
#define SDL_DSS_CTRL_HW_REG2                                                   (0x0000000CU)
#define SDL_DSS_CTRL_HW_REG3                                                   (0x00000010U)
#define SDL_DSS_CTRL_DSS_SW_INT                                                (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK                                    (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS                                  (0x0000001CU)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW                              (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK                                    (0x00000024U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS                                  (0x00000028U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW                              (0x0000002CU)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK                                    (0x00000030U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS                                  (0x00000034U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW                              (0x00000038U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK                                    (0x0000003CU)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS                                  (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW                              (0x00000044U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK                                    (0x00000048U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS                                  (0x0000004CU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW                              (0x00000050U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK                                    (0x00000054U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS                                  (0x00000058U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW                              (0x0000005CU)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START                                    (0x00000060U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS                                   (0x00000064U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE                                     (0x00000068U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL                                 (0x0000006CU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0                       (0x00000070U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1                       (0x00000074U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2                       (0x00000078U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3                       (0x0000007CU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START                               (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS                              (0x00000084U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE                                (0x00000088U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START                        (0x0000008CU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS                       (0x00000090U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE                         (0x00000094U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START                                   (0x00000098U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS                                  (0x0000009CU)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE                                    (0x000000A0U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START                                 (0x000000B0U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_STATUS                                (0x000000B4U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE                                  (0x000000B8U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL                                    (0x000000BCU)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL                                    (0x000000C0U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL                                    (0x000000C4U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_STATUS                                  (0x000000C8U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_STATUS                                  (0x000000CCU)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_STATUS                                  (0x000000D0U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG                                           (0x000000D4U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG                                           (0x000000D8U)
#define SDL_DSS_CTRL_DSS_DSP_NMI_GATE                                          (0x000000DCU)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET                                       (0x000000E0U)
#define SDL_DSS_CTRL_DSS_PBIST_REG0                                            (0x000000E4U)
#define SDL_DSS_CTRL_DSS_PBIST_REG1                                            (0x000000E8U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0                                    (0x000000ECU)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1                                    (0x000000F0U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2                                    (0x000000F4U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0                                 (0x000000F8U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1                                 (0x000000FCU)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2                                 (0x00000100U)
#define SDL_DSS_CTRL_ESM_GATING0                                               (0x00000108U)
#define SDL_DSS_CTRL_ESM_GATING1                                               (0x0000010CU)
#define SDL_DSS_CTRL_ESM_GATING2                                               (0x00000110U)
#define SDL_DSS_CTRL_ESM_GATING3                                               (0x00000114U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0                                   (0x00000560U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0                                 (0x00000564U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0                             (0x00000568U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE                                   (0x0000056CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ                                     (0x00000570U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE                                    (0x00000574U)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL                                 (0x00000578U)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL                                (0x0000057CU)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL                                (0x00000580U)
#define SDL_DSS_CTRL_DBG_ACK_CPU_CTRL                                          (0x00000584U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0                                              (0x00000588U)
#define SDL_DSS_CTRL_DBG_ACK_CTL1                                              (0x0000058CU)
#define SDL_DSS_CTRL_DSS_DSP_INT_SEL                                           (0x00000590U)
#define SDL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL                                     (0x00000594U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL                                       (0x00000800U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0                              (0x00000804U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1                              (0x00000808U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL                              (0x0000080CU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI                                (0x00000810U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR                               (0x00000814U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000818U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1                    (0x0000081CU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD                      (0x00000820U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000824U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ                     (0x00000828U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP                (0x0000082CU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL                              (0x00000830U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI                                (0x00000834U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR                               (0x00000838U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0                    (0x0000083CU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1                    (0x00000840U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD                      (0x00000844U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000848U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ                     (0x0000084CU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000850U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL                              (0x00000854U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI                                (0x00000858U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR                               (0x0000085CU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000860U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1                    (0x00000864U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD                      (0x00000868U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE                    (0x0000086CU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ                     (0x00000870U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000874U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL                              (0x00000878U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI                                (0x0000087CU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR                               (0x00000880U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000884U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1                    (0x00000888U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD                      (0x0000088CU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000890U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ                     (0x00000894U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000898U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL                              (0x0000089CU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI                                (0x000008A0U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR                               (0x000008A4U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0                    (0x000008A8U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1                    (0x000008ACU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD                      (0x000008B0U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE                    (0x000008B4U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ                     (0x000008B8U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000008BCU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL                              (0x000008C0U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI                                (0x000008C4U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR                               (0x000008C8U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0                    (0x000008CCU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD                      (0x000008D0U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE                    (0x000008D4U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ                     (0x000008D8U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000008DCU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL                            (0x000008E0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI                              (0x000008E4U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR                             (0x000008E8U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x000008ECU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000008F0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000008F4U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL                            (0x000008F8U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI                              (0x000008FCU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR                             (0x00000900U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000904U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000908U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000090CU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL                            (0x00000910U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI                              (0x00000914U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR                             (0x00000918U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x0000091CU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000920U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000924U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL                            (0x00000928U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI                              (0x0000092CU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR                             (0x00000930U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000934U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000938U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000093CU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL                            (0x00000940U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI                              (0x00000944U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR                             (0x00000948U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x0000094CU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000950U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000954U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL                            (0x00000958U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI                              (0x0000095CU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR                             (0x00000960U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000964U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000968U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000096CU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL                            (0x00000970U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI                              (0x00000974U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR                             (0x00000978U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x0000097CU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000980U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000984U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL                            (0x00000988U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI                              (0x0000098CU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR                             (0x00000990U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000994U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000998U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000099CU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL                            (0x000009A0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI                              (0x000009A4U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR                             (0x000009A8U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x000009ACU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000009B0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000009B4U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL                            (0x000009B8U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI                              (0x000009BCU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR                             (0x000009C0U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0                  (0x000009C4U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000009C8U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000009CCU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL                            (0x000009D0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI                              (0x000009D4U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR                             (0x000009D8U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000009DCU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000009E0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000009E4U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000009E8U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL                            (0x000009ECU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI                              (0x000009F0U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR                             (0x000009F4U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000009F8U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000009FCU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A04U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL                            (0x00000A08U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI                              (0x00000A0CU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR                             (0x00000A10U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000A14U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A18U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A1CU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A20U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL                            (0x00000A24U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI                              (0x00000A28U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR                             (0x00000A2CU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000A30U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A34U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A38U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A3CU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL                            (0x00000A40U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI                              (0x00000A44U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR                             (0x00000A48U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000A4CU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A50U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A54U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A58U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL                            (0x00000A5CU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI                              (0x00000A60U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR                             (0x00000A64U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000A68U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A6CU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A70U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A74U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL                            (0x00000A78U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI                              (0x00000A7CU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR                             (0x00000A80U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000A84U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A88U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A8CU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A90U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL                            (0x00000A94U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI                              (0x00000A98U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR                             (0x00000A9CU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000AA0U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000AA4U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000AA8U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000AACU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL                            (0x00000AB0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI                              (0x00000AB4U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR                             (0x00000AB8U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000ABCU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000AC0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000AC4U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000AC8U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL                            (0x00000ACCU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI                              (0x00000AD0U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR                             (0x00000AD4U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000AD8U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000ADCU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000AE0U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000AE4U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL                              (0x00000AE8U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI                                (0x00000AECU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR                               (0x00000AF0U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000AF4U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD                      (0x00000AF8U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000AFCU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ                     (0x00000B00U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000B04U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL                            (0x00000B08U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI                              (0x00000B0CU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR                             (0x00000B10U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000B14U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD                    (0x00000B18U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000B1CU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ                   (0x00000B20U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000B24U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL                            (0x00000B28U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI                              (0x00000B2CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR                             (0x00000B30U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000B34U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD                    (0x00000B38U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000B3CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ                   (0x00000B40U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000B44U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL                            (0x00000B48U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI                              (0x00000B4CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR                             (0x00000B50U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000B54U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD                    (0x00000B58U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000B5CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ                   (0x00000B60U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000B64U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL                            (0x00000B68U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI                              (0x00000B6CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR                             (0x00000B70U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000B74U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD                    (0x00000B78U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000B7CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ                   (0x00000B80U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000B84U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL                            (0x00000B88U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI                              (0x00000B8CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR                             (0x00000B90U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000B94U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD                    (0x00000B98U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000B9CU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ                   (0x00000BA0U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000BA4U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL                              (0x00000BA8U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI                                (0x00000BACU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR                               (0x00000BB0U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000BB4U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD                      (0x00000BB8U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000BBCU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ                     (0x00000BC0U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000BC4U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL                                  (0x00000BC8U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI                                    (0x00000BCCU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR                                   (0x00000BD0U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0                        (0x00000BD4U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD                          (0x00000BD8U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000BDCU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ                         (0x00000BE0U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x00000BE4U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL                                   (0x00000BE8U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI                                     (0x00000BECU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR                                    (0x00000BF0U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0                         (0x00000BF4U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD                           (0x00000BF8U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE                         (0x00000BFCU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ                          (0x00000C00U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP                     (0x00000C04U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL                              (0x00000C08U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI                                (0x00000C0CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR                               (0x00000C10U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000C14U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD                      (0x00000C18U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000C1CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ                     (0x00000C20U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000C24U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL                              (0x00000C28U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI                                (0x00000C2CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR                               (0x00000C30U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000C34U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD                      (0x00000C38U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000C3CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ                     (0x00000C40U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000C44U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL                                 (0x00000C48U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI                                   (0x00000C4CU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR                                  (0x00000C50U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0                       (0x00000C54U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD                         (0x00000C58U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE                       (0x00000C5CU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ                        (0x00000C60U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x00000C64U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL                                 (0x00000C68U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI                                   (0x00000C6CU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR                                  (0x00000C70U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0                       (0x00000C74U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD                         (0x00000C78U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE                       (0x00000C7CU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ                        (0x00000C80U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x00000C84U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL                                  (0x00000C88U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI                                    (0x00000C8CU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR                                   (0x00000C90U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0                        (0x00000C94U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD                          (0x00000C98U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000C9CU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ                         (0x00000CA0U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x00000CA4U)
#define SDL_DSS_CTRL_HW_SPARE_RW0                                              (0x00000FD0U)
#define SDL_DSS_CTRL_HW_SPARE_RW1                                              (0x00000FD4U)
#define SDL_DSS_CTRL_HW_SPARE_RW2                                              (0x00000FD8U)
#define SDL_DSS_CTRL_HW_SPARE_RW3                                              (0x00000FDCU)
#define SDL_DSS_CTRL_HW_SPARE_RO0                                              (0x00000FE0U)
#define SDL_DSS_CTRL_HW_SPARE_RO1                                              (0x00000FE4U)
#define SDL_DSS_CTRL_HW_SPARE_RO2                                              (0x00000FE8U)
#define SDL_DSS_CTRL_HW_SPARE_RO3                                              (0x00000FECU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_ACK                                (0x00000FF0U)
#define SDL_DSS_CTRL_HW_SPARE_REC                                              (0x00000FF4U)
#define SDL_DSS_CTRL_LOCK0_KICK0                                               (0x00001008U)
#define SDL_DSS_CTRL_LOCK0_KICK1                                               (0x0000100CU)
#define SDL_DSS_CTRL_INTR_RAW_STATUS                                           (0x00001010U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR                                 (0x00001014U)
#define SDL_DSS_CTRL_INTR_ENABLE                                               (0x00001018U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR                                         (0x0000101CU)
#define SDL_DSS_CTRL_EOI                                                       (0x00001020U)
#define SDL_DSS_CTRL_FAULT_ADDRESS                                             (0x00001024U)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS                                         (0x00001028U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS                                         (0x0000102CU)
#define SDL_DSS_CTRL_FAULT_CLEAR                                               (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define SDL_DSS_CTRL_PID_PID_MINOR_MASK                                        (0x0000003FU)
#define SDL_DSS_CTRL_PID_PID_MINOR_SHIFT                                       (0x00000000U)
#define SDL_DSS_CTRL_PID_PID_MINOR_RESETVAL                                    (0x00000014U)
#define SDL_DSS_CTRL_PID_PID_MINOR_MAX                                         (0x0000003FU)

#define SDL_DSS_CTRL_PID_PID_CUSTOM_MASK                                       (0x000000C0U)
#define SDL_DSS_CTRL_PID_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define SDL_DSS_CTRL_PID_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define SDL_DSS_CTRL_PID_PID_CUSTOM_MAX                                        (0x00000003U)

#define SDL_DSS_CTRL_PID_PID_MAJOR_MASK                                        (0x00000700U)
#define SDL_DSS_CTRL_PID_PID_MAJOR_SHIFT                                       (0x00000008U)
#define SDL_DSS_CTRL_PID_PID_MAJOR_RESETVAL                                    (0x00000002U)
#define SDL_DSS_CTRL_PID_PID_MAJOR_MAX                                         (0x00000007U)

#define SDL_DSS_CTRL_PID_PID_MISC_MASK                                         (0x0000F800U)
#define SDL_DSS_CTRL_PID_PID_MISC_SHIFT                                        (0x0000000BU)
#define SDL_DSS_CTRL_PID_PID_MISC_RESETVAL                                     (0x00000000U)
#define SDL_DSS_CTRL_PID_PID_MISC_MAX                                          (0x0000001FU)

#define SDL_DSS_CTRL_PID_PID_MSB16_MASK                                        (0xFFFF0000U)
#define SDL_DSS_CTRL_PID_PID_MSB16_SHIFT                                       (0x00000010U)
#define SDL_DSS_CTRL_PID_PID_MSB16_RESETVAL                                    (0x00006180U)
#define SDL_DSS_CTRL_PID_PID_MSB16_MAX                                         (0x0000FFFFU)

#define SDL_DSS_CTRL_PID_RESETVAL                                              (0x61800214U)

/* HW_REG0 */

#define SDL_DSS_CTRL_HW_REG0_HW_REG0_HWREG0_MASK                               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_REG0_HW_REG0_HWREG0_SHIFT                              (0x00000000U)
#define SDL_DSS_CTRL_HW_REG0_HW_REG0_HWREG0_RESETVAL                           (0x00000000U)
#define SDL_DSS_CTRL_HW_REG0_HW_REG0_HWREG0_MAX                                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_REG0_RESETVAL                                          (0x00000000U)

/* HW_REG1 */

#define SDL_DSS_CTRL_HW_REG1_HW_REG1_HWREG1_MASK                               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_REG1_HW_REG1_HWREG1_SHIFT                              (0x00000000U)
#define SDL_DSS_CTRL_HW_REG1_HW_REG1_HWREG1_RESETVAL                           (0x00000000U)
#define SDL_DSS_CTRL_HW_REG1_HW_REG1_HWREG1_MAX                                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_REG1_RESETVAL                                          (0x00000000U)

/* HW_REG2 */

#define SDL_DSS_CTRL_HW_REG2_HW_REG2_HWREG2_MASK                               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_REG2_HW_REG2_HWREG2_SHIFT                              (0x00000000U)
#define SDL_DSS_CTRL_HW_REG2_HW_REG2_HWREG2_RESETVAL                           (0x00000000U)
#define SDL_DSS_CTRL_HW_REG2_HW_REG2_HWREG2_MAX                                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_REG2_RESETVAL                                          (0x00000000U)

/* HW_REG3 */

#define SDL_DSS_CTRL_HW_REG3_HW_REG3_HWREG3_MASK                               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_REG3_HW_REG3_HWREG3_SHIFT                              (0x00000000U)
#define SDL_DSS_CTRL_HW_REG3_HW_REG3_HWREG3_RESETVAL                           (0x00000000U)
#define SDL_DSS_CTRL_HW_REG3_HW_REG3_HWREG3_MAX                                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_REG3_RESETVAL                                          (0x00000000U)

/* DSS_SW_INT */

#define SDL_DSS_CTRL_DSS_SW_INT_DSS_SW_INT_DSS_SWINT_MASK                      (0x0000000FU)
#define SDL_DSS_CTRL_DSS_SW_INT_DSS_SW_INT_DSS_SWINT_SHIFT                     (0x00000000U)
#define SDL_DSS_CTRL_DSS_SW_INT_DSS_SW_INT_DSS_SWINT_RESETVAL                  (0x00000000U)
#define SDL_DSS_CTRL_DSS_SW_INT_DSS_SW_INT_DSS_SWINT_MAX                       (0x0000000FU)

#define SDL_DSS_CTRL_DSS_SW_INT_RESETVAL                                       (0x00000000U)

/* DSS_TPCC_A_ERRAGG_MASK */

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_DSS_TPCC_A_ERRAGG_MASK_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* DSS_TPCC_A_ERRAGG_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_DSS_TPCC_A_ERRAGG_STATUS_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_A_ERRAGG_STATUS_RAW */

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_DSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* DSS_TPCC_A_INTAGG_MASK */

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_DSS_TPCC_A_INTAGG_MASK_TPTC_A1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* DSS_TPCC_A_INTAGG_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_DSS_TPCC_A_INTAGG_STATUS_TPTC_A1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_A_INTAGG_STATUS_RAW */

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_DSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* DSS_TPCC_B_ERRAGG_MASK */

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_DSS_TPCC_B_ERRAGG_MASK_TPTC_B1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* DSS_TPCC_B_ERRAGG_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_DSS_TPCC_B_ERRAGG_STATUS_TPTC_B1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_B_ERRAGG_STATUS_RAW */

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPCC_B_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_DSS_TPCC_B_ERRAGG_STATUS_RAW_TPTC_B1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* DSS_TPCC_B_INTAGG_MASK */

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPCC_B_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_DSS_TPCC_B_INTAGG_MASK_TPTC_B1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* DSS_TPCC_B_INTAGG_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPCC_B_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_DSS_TPCC_B_INTAGG_STATUS_TPTC_B1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_B_INTAGG_STATUS_RAW */

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPCC_B_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_DSS_TPCC_B_INTAGG_STATUS_RAW_TPTC_B1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* DSS_TPCC_C_ERRAGG_MASK */

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_ERR_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_ERR_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_ERR_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_ERR_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_ERR_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_ERR_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_ERR_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_ERR_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_WRITE_ACCESS_ERROR_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_WRITE_ACCESS_ERROR_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_WRITE_ACCESS_ERROR_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_WRITE_ACCESS_ERROR_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_WRITE_ACCESS_ERROR_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_WRITE_ACCESS_ERROR_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_WRITE_ACCESS_ERROR_MASK (0x00400000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_WRITE_ACCESS_ERROR_SHIFT (0x00000016U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPCC_C_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_READ_ACCESS_ERROR_MASK (0x08000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_READ_ACCESS_ERROR_SHIFT (0x0000001BU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C2_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_READ_ACCESS_ERROR_MASK (0x10000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_READ_ACCESS_ERROR_SHIFT (0x0000001CU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C3_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_READ_ACCESS_ERROR_MASK (0x20000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_READ_ACCESS_ERROR_SHIFT (0x0000001DU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C4_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_READ_ACCESS_ERROR_MASK (0x40000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_READ_ACCESS_ERROR_SHIFT (0x0000001EU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_DSS_TPCC_C_ERRAGG_MASK_TPTC_C5_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* DSS_TPCC_C_ERRAGG_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_ERR_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_ERR_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_ERR_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_ERR_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_ERR_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_ERR_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_ERR_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_ERR_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_WRITE_ACCESS_ERROR_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_WRITE_ACCESS_ERROR_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_WRITE_ACCESS_ERROR_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_WRITE_ACCESS_ERROR_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_WRITE_ACCESS_ERROR_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_WRITE_ACCESS_ERROR_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_WRITE_ACCESS_ERROR_MASK (0x00400000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_WRITE_ACCESS_ERROR_SHIFT (0x00000016U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPCC_C_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_READ_ACCESS_ERROR_MASK (0x08000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_READ_ACCESS_ERROR_SHIFT (0x0000001BU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C2_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_READ_ACCESS_ERROR_MASK (0x10000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_READ_ACCESS_ERROR_SHIFT (0x0000001CU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C3_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_READ_ACCESS_ERROR_MASK (0x20000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_READ_ACCESS_ERROR_SHIFT (0x0000001DU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C4_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_READ_ACCESS_ERROR_MASK (0x40000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_READ_ACCESS_ERROR_SHIFT (0x0000001EU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_DSS_TPCC_C_ERRAGG_STATUS_TPTC_C5_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_C_ERRAGG_STATUS_RAW */

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_ERRINT_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_ERRINT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_ERRINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_ERRINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_MPINT_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_MPINT_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_MPINT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_MPINT_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_ERR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_ERR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_ERR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_ERR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_ERR_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_ERR_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_ERR_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_ERR_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_ERR_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_ERR_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_ERR_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_ERR_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_PARITY_ERR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_PARITY_ERR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_PARITY_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_PARITY_ERR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_WRITE_ACCESS_ERROR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_WRITE_ACCESS_ERROR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_WRITE_ACCESS_ERROR_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_WRITE_ACCESS_ERROR_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_WRITE_ACCESS_ERROR_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_WRITE_ACCESS_ERROR_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_WRITE_ACCESS_ERROR_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_WRITE_ACCESS_ERROR_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_WRITE_ACCESS_ERROR_MASK (0x00400000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_WRITE_ACCESS_ERROR_SHIFT (0x00000016U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_READ_ACCESS_ERROR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPCC_C_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_READ_ACCESS_ERROR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_READ_ACCESS_ERROR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C1_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_READ_ACCESS_ERROR_MASK (0x08000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_READ_ACCESS_ERROR_SHIFT (0x0000001BU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C2_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_READ_ACCESS_ERROR_MASK (0x10000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_READ_ACCESS_ERROR_SHIFT (0x0000001CU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C3_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_READ_ACCESS_ERROR_MASK (0x20000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_READ_ACCESS_ERROR_SHIFT (0x0000001DU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C4_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_READ_ACCESS_ERROR_MASK (0x40000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_READ_ACCESS_ERROR_SHIFT (0x0000001EU)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_DSS_TPCC_C_ERRAGG_STATUS_RAW_TPTC_C5_READ_ACCESS_ERROR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* DSS_TPCC_C_INTAGG_MASK */

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPCC_C_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C2_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C2_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C3_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C3_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C4_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C4_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C5_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C5_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_DSS_TPCC_C_INTAGG_MASK_TPTC_C5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* DSS_TPCC_C_INTAGG_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPCC_C_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C2_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C2_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C3_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C3_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C4_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C4_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C5_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C5_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_DSS_TPCC_C_INTAGG_STATUS_TPTC_C5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_C_INTAGG_STATUS_RAW */

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INTG_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INTG_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INTG_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INTG_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT2_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT2_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT3_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT3_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT4_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT4_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT5_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT5_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT6_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT6_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT7_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT7_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPCC_C_INT7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C2_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C2_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C3_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C3_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C4_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C4_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C5_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C5_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_DSS_TPCC_C_INTAGG_STATUS_RAW_TPTC_C5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* DSS_TPCC_MEMINIT_START */

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_C_MEMINIT_START_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_C_MEMINIT_START_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_C_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_DSS_TPCC_MEMINIT_START_TPCC_C_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_START_RESETVAL                           (0x00000000U)

/* DSS_TPCC_MEMINIT_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_B_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_C_MEMINIT_STATUS_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_C_MEMINIT_STATUS_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_C_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_DSS_TPCC_MEMINIT_STATUS_TPCC_C_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_STATUS_RESETVAL                          (0x00000000U)

/* DSS_TPCC_MEMINIT_DONE */

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_C_MEMINIT_DONE_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_C_MEMINIT_DONE_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_C_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_DSS_TPCC_MEMINIT_DONE_TPCC_C_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_MEMINIT_DONE_RESETVAL                            (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_CTRL */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ENABLE_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ENABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ENABLE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ERR_CLEAR_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_ERR_CLEAR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_CTRL_RESETVAL                        (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0 */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR0_MASK (0x00000FFFU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR0_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR1_MASK (0x0FFF0000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR1_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_ADDR1_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB0_RESETVAL              (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1 */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR0_MASK (0x00000FFFU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR0_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR1_MASK (0x0FFF0000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR1_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_ADDR1_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1_RESETVAL              (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2 */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR0_MASK (0x00000FFFU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR0_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR1_MASK (0x0FFF0000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR1_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_ADDR1_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB2_RESETVAL              (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3 */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR0_MASK (0x00000FFFU)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR0_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR1_MASK (0x0FFF0000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR1_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_ADDR1_MAX (0x00000FFFU)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB3_RESETVAL              (0x00000000U)

/* DSS_DSP_L2RAM_MEMINIT_START */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB00_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB00_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB00_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB00_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB01_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB01_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB01_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB01_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB10_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB10_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB10_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB10_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB11_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB11_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB11_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB11_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB20_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB20_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB20_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB20_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB21_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB21_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB21_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB21_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB30_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB30_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB30_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB30_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB31_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB31_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB31_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB31_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_RESETVAL                      (0x00000000U)

/* DSS_DSP_L2RAM_MEMINIT_STATUS */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB00_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB00_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB00_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB00_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB01_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB01_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB01_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB01_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB10_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB10_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB10_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB10_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB11_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB11_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB11_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB11_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB20_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB20_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB20_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB20_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB21_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB21_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB21_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB21_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB30_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB30_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB30_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB30_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB31_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB31_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB31_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB31_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_RESETVAL                     (0x00000000U)

/* DSS_DSP_L2RAM_MEMINIT_DONE */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_RESETVAL                       (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_MEMINIT_START */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB00_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB00_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB00_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB00_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB01_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB01_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB01_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB01_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB10_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB10_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB10_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB10_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB11_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB11_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB11_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB11_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB20_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB20_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB20_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB20_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB21_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB21_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB21_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB21_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB30_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB30_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB30_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB30_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB31_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB31_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB31_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_DSS_DSP_L2RAM_PARITY_MEMINIT_START_VB31_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_START_RESETVAL               (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB00_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB00_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB00_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB00_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB01_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB01_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB01_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB01_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB10_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB10_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB10_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB10_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB11_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB11_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB11_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB11_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB20_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB20_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB20_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB20_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB21_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB21_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB21_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB21_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB30_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB30_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB30_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB30_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB31_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB31_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB31_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_VB31_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_STATUS_RESETVAL              (0x00000000U)

/* DSS_DSP_L2RAM_PARITY_MEMINIT_DONE */

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB00_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB00_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB00_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB00_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB01_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB01_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB01_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB01_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB10_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB10_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB10_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB10_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB11_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB11_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB11_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB11_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB20_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB20_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB20_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB20_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB21_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB21_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB21_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB21_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB30_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB30_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB30_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB30_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB31_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB31_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB31_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_VB31_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_L2RAM_PARITY_MEMINIT_DONE_RESETVAL                (0x00000000U)

/* DSS_L3RAM_MEMINIT_START */

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM0_MEMINIT_START_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM0_MEMINIT_START_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM0_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM0_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM1_MEMINIT_START_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM1_MEMINIT_START_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM1_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM1_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM2_MEMINIT_START_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM2_MEMINIT_START_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM2_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM2_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM3_MEMINIT_START_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM3_MEMINIT_START_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM3_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM3_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_RESETVAL                          (0x00000000U)

/* DSS_L3RAM_MEMINIT_STATUS */

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM0_MEMINIT_STATUS_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM0_MEMINIT_STATUS_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM0_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM0_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM1_MEMINIT_STATUS_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM1_MEMINIT_STATUS_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM1_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM1_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM2_MEMINIT_STATUS_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM2_MEMINIT_STATUS_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM2_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM2_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM3_MEMINIT_STATUS_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM3_MEMINIT_STATUS_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM3_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM3_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_RESETVAL                         (0x00000000U)

/* DSS_L3RAM_MEMINIT_DONE */

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_RESETVAL                           (0x00000000U)

/* DSS_MAILBOX_MEMINIT_START */

#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START_DSS_MAILBOX_MEMINIT_START_MEMINIT_START_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START_DSS_MAILBOX_MEMINIT_START_MEMINIT_START_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START_DSS_MAILBOX_MEMINIT_START_MEMINIT_START_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START_DSS_MAILBOX_MEMINIT_START_MEMINIT_START_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_START_RESETVAL                        (0x00000000U)

/* DSS_MAILBOX_MEMINIT_STATUS */

#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_STATUS_DSS_MAILBOX_MEMINIT_STATUS_MEMINIT_STATUS_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_STATUS_DSS_MAILBOX_MEMINIT_STATUS_MEMINIT_STATUS_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_STATUS_DSS_MAILBOX_MEMINIT_STATUS_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_STATUS_DSS_MAILBOX_MEMINIT_STATUS_MEMINIT_STATUS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_STATUS_RESETVAL                       (0x00000000U)

/* DSS_MAILBOX_MEMINIT_DONE */

#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_DSS_MAILBOX_MEMINIT_DONE_MEMINIT_DONE_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_DSS_MAILBOX_MEMINIT_DONE_MEMINIT_DONE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_DSS_MAILBOX_MEMINIT_DONE_MEMINIT_DONE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_DSS_MAILBOX_MEMINIT_DONE_MEMINIT_DONE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_RESETVAL                         (0x00000000U)

/* DSS_TPCC_A_PARITY_CTRL */

#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_EN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_EN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_EN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_EN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_DSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_CTRL_RESETVAL                           (0x00000000U)

/* DSS_TPCC_B_PARITY_CTRL */

#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_EN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_EN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_EN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_EN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_TESTEN_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_TESTEN_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_TESTEN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_TESTEN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_ERR_CLR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_ERR_CLR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_DSS_TPCC_B_PARITY_CTRL_PARITY_ERR_CLR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_CTRL_RESETVAL                           (0x00000000U)

/* DSS_TPCC_C_PARITY_CTRL */

#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_EN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_EN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_EN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_EN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_TESTEN_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_TESTEN_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_TESTEN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_TESTEN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_ERR_CLR_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_ERR_CLR_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_DSS_TPCC_C_PARITY_CTRL_PARITY_ERR_CLR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_CTRL_RESETVAL                           (0x00000000U)

/* DSS_TPCC_A_PARITY_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_STATUS_DSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_STATUS_DSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_STATUS_DSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_STATUS_DSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPCC_A_PARITY_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_B_PARITY_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_STATUS_DSS_TPCC_B_PARITY_STATUS_PARITY_ADDR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_STATUS_DSS_TPCC_B_PARITY_STATUS_PARITY_ADDR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_STATUS_DSS_TPCC_B_PARITY_STATUS_PARITY_ADDR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_STATUS_DSS_TPCC_B_PARITY_STATUS_PARITY_ADDR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPCC_B_PARITY_STATUS_RESETVAL                         (0x00000000U)

/* DSS_TPCC_C_PARITY_STATUS */

#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_STATUS_DSS_TPCC_C_PARITY_STATUS_PARITY_ADDR_MASK (0x000001FFU)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_STATUS_DSS_TPCC_C_PARITY_STATUS_PARITY_ADDR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_STATUS_DSS_TPCC_C_PARITY_STATUS_PARITY_ADDR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_STATUS_DSS_TPCC_C_PARITY_STATUS_PARITY_ADDR_MAX (0x000001FFU)

#define SDL_DSS_CTRL_DSS_TPCC_C_PARITY_STATUS_RESETVAL                         (0x00000000U)

/* TPTC_DBS_CONFIG */

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_MASK              (0x00000003U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_SHIFT             (0x00000000U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A0_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_MASK              (0x0000000CU)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_SHIFT             (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_A1_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_MASK              (0x00000030U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_SHIFT             (0x00000004U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B0_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_MASK              (0x000000C0U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_SHIFT             (0x00000006U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_B1_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C0_MASK              (0x00000300U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C0_SHIFT             (0x00000008U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C0_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C0_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C1_MASK              (0x00000C00U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C1_SHIFT             (0x0000000AU)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C1_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C1_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C2_MASK              (0x00003000U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C2_SHIFT             (0x0000000CU)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C2_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C2_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C3_MASK              (0x0000C000U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C3_SHIFT             (0x0000000EU)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C3_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C3_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C4_MASK              (0x00030000U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C4_SHIFT             (0x00000010U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C4_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C4_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C5_MASK              (0x000C0000U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C5_SHIFT             (0x00000012U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C5_RESETVAL          (0x00000002U)
#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_TPTC_DBS_CONFIG_TPTC_C5_MAX               (0x00000003U)

#define SDL_DSS_CTRL_TPTC_DBS_CONFIG_RESETVAL                                  (0x000AAAAAU)

/* DSS_DSP_BOOTCFG */

#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_ISTP_RST_VAL_MASK         (0x003FFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_ISTP_RST_VAL_SHIFT        (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_ISTP_RST_VAL_RESETVAL     (0x00002000U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_ISTP_RST_VAL_MAX          (0x003FFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1D_CACHE_MODE_MASK       (0x01000000U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1D_CACHE_MODE_SHIFT      (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1D_CACHE_MODE_RESETVAL   (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1D_CACHE_MODE_MAX        (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1P_CACHE_MODE_MASK       (0x02000000U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1P_CACHE_MODE_SHIFT      (0x00000019U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1P_CACHE_MODE_RESETVAL   (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_DSS_DSP_BOOTCFG_L1P_CACHE_MODE_MAX        (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_BOOTCFG_RESETVAL                                  (0x00002000U)

/* DSS_DSP_NMI_GATE */

#define SDL_DSS_CTRL_DSS_DSP_NMI_GATE_DSS_DSP_NMI_GATE_GATE_MASK               (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_NMI_GATE_DSS_DSP_NMI_GATE_GATE_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_NMI_GATE_DSS_DSP_NMI_GATE_GATE_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_NMI_GATE_DSS_DSP_NMI_GATE_GATE_MAX                (0x00000007U)

#define SDL_DSS_CTRL_DSS_DSP_NMI_GATE_RESETVAL                                 (0x00000000U)

/* DSS_PBIST_KEY_RESET */

#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_KEY_MASK (0x0000000FU)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_KEY_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_KEY_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_KEY_MAX (0x0000000FU)

#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_RESET_MASK (0x000000F0U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_RESET_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_RESET_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_PBIST_ST_RESET_MAX (0x0000000FU)

#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_L2_PBIST_ST_KEY_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_L2_PBIST_ST_KEY_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_L2_PBIST_ST_KEY_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_DSS_PBIST_KEY_RESET_DSS_L2_PBIST_ST_KEY_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PBIST_KEY_RESET_RESETVAL                              (0x00000000U)

/* DSS_PBIST_REG0 */

#define SDL_DSS_CTRL_DSS_PBIST_REG0_DSS_PBIST_REG0_DSS_PBIST_REG0_MASK         (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_PBIST_REG0_DSS_PBIST_REG0_DSS_PBIST_REG0_SHIFT        (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_REG0_DSS_PBIST_REG0_DSS_PBIST_REG0_RESETVAL     (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_REG0_DSS_PBIST_REG0_DSS_PBIST_REG0_MAX          (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_PBIST_REG0_RESETVAL                                   (0x00000000U)

/* DSS_PBIST_REG1 */

#define SDL_DSS_CTRL_DSS_PBIST_REG1_DSS_PBIST_REG1_DSS_PBIST_REG1_MASK         (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_PBIST_REG1_DSS_PBIST_REG1_DSS_PBIST_REG1_SHIFT        (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_REG1_DSS_PBIST_REG1_DSS_PBIST_REG1_RESETVAL     (0x00000000U)
#define SDL_DSS_CTRL_DSS_PBIST_REG1_DSS_PBIST_REG1_DSS_PBIST_REG1_MAX          (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_PBIST_REG1_RESETVAL                                   (0x00000000U)

/* DSS_TPTC_BOUNDARY_CFG0 */

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A0_SIZE_MASK (0x0000003FU)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A0_SIZE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A0_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A0_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A1_SIZE_MASK (0x00003F00U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A1_SIZE_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A1_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_A1_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B0_SIZE_MASK (0x003F0000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B0_SIZE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B0_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B0_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B1_SIZE_MASK (0x3F000000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B1_SIZE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B1_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_DSS_TPTC_BOUNDARY_CFG0_TPTC_B1_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG0_RESETVAL                           (0x12121212U)

/* DSS_TPTC_BOUNDARY_CFG1 */

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C0_SIZE_MASK (0x0000003FU)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C0_SIZE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C0_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C0_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C1_SIZE_MASK (0x00003F00U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C1_SIZE_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C1_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C1_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C2_SIZE_MASK (0x003F0000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C2_SIZE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C2_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C2_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C3_SIZE_MASK (0x3F000000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C3_SIZE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C3_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_DSS_TPTC_BOUNDARY_CFG1_TPTC_C3_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG1_RESETVAL                           (0x12121212U)

/* DSS_TPTC_BOUNDARY_CFG2 */

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C4_SIZE_MASK (0x0000003FU)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C4_SIZE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C4_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C4_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C5_SIZE_MASK (0x00003F00U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C5_SIZE_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C5_SIZE_RESETVAL (0x00000012U)
#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_DSS_TPTC_BOUNDARY_CFG2_TPTC_C5_SIZE_MAX (0x0000003FU)

#define SDL_DSS_CTRL_DSS_TPTC_BOUNDARY_CFG2_RESETVAL                           (0x00001212U)

/* DSS_TPTC_XID_REORDER_CFG0 */

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A0_DISABLE_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A0_DISABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A0_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A0_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A1_DISABLE_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A1_DISABLE_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A1_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_A1_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B0_DISABLE_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B0_DISABLE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B0_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B0_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B1_DISABLE_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B1_DISABLE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B1_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_DSS_TPTC_XID_REORDER_CFG0_TPTC_B1_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG0_RESETVAL                        (0x00000000U)

/* DSS_TPTC_XID_REORDER_CFG1 */

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C0_DISABLE_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C0_DISABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C0_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C0_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C1_DISABLE_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C1_DISABLE_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C1_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C1_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C2_DISABLE_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C2_DISABLE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C2_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C2_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C3_DISABLE_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C3_DISABLE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C3_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_DSS_TPTC_XID_REORDER_CFG1_TPTC_C3_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG1_RESETVAL                        (0x00000000U)

/* DSS_TPTC_XID_REORDER_CFG2 */

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C4_DISABLE_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C4_DISABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C4_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C4_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C5_DISABLE_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C5_DISABLE_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C5_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_DSS_TPTC_XID_REORDER_CFG2_TPTC_C5_DISABLE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_XID_REORDER_CFG2_RESETVAL                        (0x00000000U)

/* ESM_GATING0 */

#define SDL_DSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_DSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING0_ESM_GATING0_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_DSS_CTRL_ESM_GATING0_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING1 */

#define SDL_DSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_DSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING1_ESM_GATING1_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_DSS_CTRL_ESM_GATING1_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING2 */

#define SDL_DSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_DSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING2_ESM_GATING2_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_DSS_CTRL_ESM_GATING2_RESETVAL                                      (0xFFFFFFFFU)

/* ESM_GATING3 */

#define SDL_DSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_MASK                   (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_SHIFT                  (0x00000000U)
#define SDL_DSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_RESETVAL               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_ESM_GATING3_ESM_GATING3_ESM_GATING_MAX                    (0xFFFFFFFFU)

#define SDL_DSS_CTRL_ESM_GATING3_RESETVAL                                      (0xFFFFFFFFU)

/* DSS_PERIPH_ERRAGG_MASK0 */

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_RCM_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_CM4_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_RD_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_RD_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_WR_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_WR_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_DSS_HWA_CFG_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_MASK (0x00000200U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_SHIFT (0x00000009U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_RCM_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_MASK (0x00000400U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_SHIFT (0x0000000AU)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_MASK (0x00000800U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_SHIFT (0x0000000BU)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_DSS_PERIPH_ERRAGG_MASK0_RSS_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_MASK0_RESETVAL                          (0x00000000U)

/* DSS_PERIPH_ERRAGG_STATUS0 */

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_RCM_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_CM4_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_RD_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_RD_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_WR_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_WR_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_DSS_HWA_CFG_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_MASK (0x00000200U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_SHIFT (0x00000009U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_RCM_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_MASK (0x00000400U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_SHIFT (0x0000000AU)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_MASK (0x00000800U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_SHIFT (0x0000000BU)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_DSS_PERIPH_ERRAGG_STATUS0_RSS_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS0_RESETVAL                        (0x00000000U)

/* DSS_PERIPH_ERRAGG_STATUS_RAW0 */

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_RCM_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_CM4_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_RD_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_RD_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_WR_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_WR_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_HWA_CFG_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_MASK (0x00000200U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_SHIFT (0x00000009U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_RCM_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_MASK (0x00000400U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_SHIFT (0x0000000AU)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_MASK (0x00000800U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_SHIFT (0x0000000BU)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_DSS_PERIPH_ERRAGG_STATUS_RAW0_RSS_CTRL_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PERIPH_ERRAGG_STATUS_RAW0_RESETVAL                    (0x00000000U)

/* DSS_DSP_MBOX_WRITE_DONE */

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_0_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_1_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_1_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_2_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_2_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_3_MASK (0x00001000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_3_SHIFT (0x0000000CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_4_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_4_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_4_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_5_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_5_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_5_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_6_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_6_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_6_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_7_MASK (0x10000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_7_SHIFT (0x0000001CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_DSS_DSP_MBOX_WRITE_DONE_PROC_7_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_WRITE_DONE_RESETVAL                          (0x00000000U)

/* DSS_DSP_MBOX_READ_REQ */

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_0_MASK   (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_0_SHIFT  (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_0_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_1_MASK   (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_1_SHIFT  (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_1_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_2_MASK   (0x00000100U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_2_SHIFT  (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_2_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_3_MASK   (0x00001000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_3_SHIFT  (0x0000000CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_3_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_4_MASK   (0x00010000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_4_SHIFT  (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_4_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_5_MASK   (0x00100000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_5_SHIFT  (0x00000014U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_5_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_6_MASK   (0x01000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_6_SHIFT  (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_6_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_7_MASK   (0x10000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_7_SHIFT  (0x0000001CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_DSS_DSP_MBOX_READ_REQ_PROC_7_MAX    (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_REQ_RESETVAL                            (0x00000000U)

/* DSS_DSP_MBOX_READ_DONE */

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_0_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_0_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_1_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_1_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_1_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_2_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_2_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_2_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_3_MASK (0x00001000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_3_SHIFT (0x0000000CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_3_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_4_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_4_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_4_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_5_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_5_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_5_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_6_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_6_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_6_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_7_MASK (0x10000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_7_SHIFT (0x0000001CU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_DSS_DSP_MBOX_READ_DONE_PROC_7_MAX  (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_RESETVAL                           (0x00000000U)

/* DSS_WDT_EVENT_CAPTURE_SEL */

#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP0_MASK (0x0000007FU)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP0_MAX (0x0000007FU)

#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP1_MASK (0x00007F00U)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_DSS_WDT_EVENT_CAPTURE_SEL_CAP1_MAX (0x0000007FU)

#define SDL_DSS_CTRL_DSS_WDT_EVENT_CAPTURE_SEL_RESETVAL                        (0x00000000U)

/* DSS_RTIA_EVENT_CAPTURE_SEL */

#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP0_MASK (0x0000007FU)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP0_MAX (0x0000007FU)

#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP1_MASK (0x00007F00U)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_DSS_RTIA_EVENT_CAPTURE_SEL_CAP1_MAX (0x0000007FU)

#define SDL_DSS_CTRL_DSS_RTIA_EVENT_CAPTURE_SEL_RESETVAL                       (0x00000000U)

/* DSS_RTIB_EVENT_CAPTURE_SEL */

#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP0_MASK (0x0000007FU)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP0_MAX (0x0000007FU)

#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP1_MASK (0x00007F00U)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_DSS_RTIB_EVENT_CAPTURE_SEL_CAP1_MAX (0x0000007FU)

#define SDL_DSS_CTRL_DSS_RTIB_EVENT_CAPTURE_SEL_RESETVAL                       (0x00000000U)

/* DBG_ACK_CPU_CTRL */

#define SDL_DSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_MASK                (0x00000001U)
#define SDL_DSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_SHIFT               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_RESETVAL            (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_MAX                 (0x00000001U)

#define SDL_DSS_CTRL_DBG_ACK_CPU_CTRL_RESETVAL                                 (0x00000000U)

/* DBG_ACK_CTL0 */

#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCA_MASK                   (0x00000007U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCA_SHIFT                  (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCA_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCA_MAX                    (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCB_MASK                   (0x00000070U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCB_SHIFT                  (0x00000004U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCB_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_DCCB_MAX                    (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIA_MASK                   (0x00000700U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIA_SHIFT                  (0x00000008U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIA_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIA_MAX                    (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIB_MASK                   (0x00007000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIB_SHIFT                  (0x0000000CU)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIB_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_RTIB_MAX                    (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_SCIA_MASK                   (0x00070000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_SCIA_SHIFT                  (0x00000010U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_SCIA_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_SCIA_MAX                    (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_WDT_MASK                    (0x00700000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_WDT_SHIFT                   (0x00000014U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_WDT_RESETVAL                (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL0_DBG_ACK_CTL0_DSS_WDT_MAX                     (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL0_RESETVAL                                     (0x00000000U)

/* DBG_ACK_CTL1 */

#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_MCRC_MASK                   (0x07000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_MCRC_SHIFT                  (0x00000018U)
#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_MCRC_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_MCRC_MAX                    (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_HWA_MASK                    (0x70000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_HWA_SHIFT                   (0x0000001CU)
#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_HWA_RESETVAL                (0x00000000U)
#define SDL_DSS_CTRL_DBG_ACK_CTL1_DBG_ACK_CTL1_DSS_HWA_MAX                     (0x00000007U)

#define SDL_DSS_CTRL_DBG_ACK_CTL1_RESETVAL                                     (0x00000000U)

/* DSS_DSP_INT_SEL */

#define SDL_DSS_CTRL_DSS_DSP_INT_SEL_DSS_DSP_INT_SEL_RSS_CSI2_ICSSM_MASK      (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_INT_SEL_DSS_DSP_INT_SEL_RSS_CSI2_ICSSM_SHIFT     (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_INT_SEL_DSS_DSP_INT_SEL_RSS_CSI2_ICSSM_RESETVAL  (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_INT_SEL_DSS_DSP_INT_SEL_RSS_CSI2_ICSSM_MAX       (0x00000007U)

#define SDL_DSS_CTRL_DSS_DSP_INT_SEL_RESETVAL                                  (0x00000000U)

/* DSS_CBUFF_TRIGGER_SEL */

#define SDL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL_DSS_CBUFF_TRIGGER_SEL_SEL_MASK      (0x0000007FU)
#define SDL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL_DSS_CBUFF_TRIGGER_SEL_SEL_SHIFT     (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL_DSS_CBUFF_TRIGGER_SEL_SEL_RESETVAL  (0x00000028U)
#define SDL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL_DSS_CBUFF_TRIGGER_SEL_SEL_MAX       (0x0000007FU)

#define SDL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL_RESETVAL                            (0x00000028U)

/* DSS_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_CLK_DISABLE_MASK  (0x00000070U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_CLK_DISABLE_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_CLK_DISABLE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_CLK_DISABLE_MAX   (0x00000007U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_RESETVAL                              (0x00000000U)

/* DSS_BUS_SAFETY_SEC_ERR_STAT0 */

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_MDMA_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_MDMA_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_MDMA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_MDMA_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKA_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKA_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKA_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKB_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKB_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKB_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKB_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKC_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKC_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKD_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKD_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_SDMA_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_SDMA_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_SDMA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_DSP_SDMA_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_RD_MASK (0x00000040U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_RD_SHIFT (0x00000006U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_RD_MASK (0x00000080U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_RD_SHIFT (0x00000007U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_RD_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_RD_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_RD_MASK (0x00000200U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_RD_SHIFT (0x00000009U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_RD_MASK (0x00000400U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_RD_SHIFT (0x0000000AU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_RD_MASK (0x00000800U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_RD_SHIFT (0x0000000BU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_RD_MASK (0x00001000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_RD_SHIFT (0x0000000CU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_RD_MASK (0x00002000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_RD_SHIFT (0x0000000DU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_RD_MASK (0x00004000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_RD_SHIFT (0x0000000EU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_RD_MASK (0x00008000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_RD_SHIFT (0x0000000FU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_RD_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_RD_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_WR_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_WR_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A0_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_WR_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_WR_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_A1_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_WR_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_WR_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B0_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_WR_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_WR_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_B1_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_WR_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_WR_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C0_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_WR_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_WR_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C1_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_WR_MASK (0x00400000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_WR_SHIFT (0x00000016U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C2_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_WR_MASK (0x00800000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_WR_SHIFT (0x00000017U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C3_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_WR_MASK (0x01000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_WR_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C4_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_WR_MASK (0x02000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_WR_SHIFT (0x00000019U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_WR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_TPTC_C5_WR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_PCR_MASK (0x04000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_PCR_SHIFT (0x0000001AU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_PCR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_PCR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_CBUFF_FIFO_MASK (0x08000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_CBUFF_FIFO_SHIFT (0x0000001BU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_CBUFF_FIFO_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_CBUFF_FIFO_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_MDO_FIFO_MASK (0x10000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_MDO_FIFO_SHIFT (0x0000001CU)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_MDO_FIFO_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_MDO_FIFO_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_RESETVAL                     (0x00000000U)

/* DSS_BUS_SAFETY_SEC_ERR_STAT1 */

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MCRC_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MCRC_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MCRC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MCRC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA0_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA0_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA1_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA1_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_M_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_M_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_M_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_M_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_S_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_S_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_S_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CM4_S_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MBOX_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MBOX_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MBOX_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MBOX_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP0_MASK (0x00010000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP0_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP0_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP1_MASK (0x00020000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP1_SHIFT (0x00000011U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP1_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP2_MASK (0x00040000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP2_SHIFT (0x00000012U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP2_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP3_MASK (0x00080000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP3_SHIFT (0x00000013U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_UCOMP3_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_COMP_MASK (0x00100000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_COMP_SHIFT (0x00000014U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_COMP_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_CMC_COMP_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS2RSS_MASK (0x00200000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS2RSS_SHIFT (0x00000015U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS2RSS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS2RSS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2DSS_MASK (0x00400000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2DSS_SHIFT (0x00000016U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2DSS_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_RSS2DSS_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_RESETVAL                     (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL_RESETVAL                     (0x000F0007U)

/* DSS_DSP_MDMA_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_DSS_DSP_MDMA_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_DSS_DSP_MDMA_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1 */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D4_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D4_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D4_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D5_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D5_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D5_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D6_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D6_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D6_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D7_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D7_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_D7_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1_RESETVAL           (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_RESETVAL                     (0x001F0007U)

/* DSS_L3_BANKA_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1 */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D4_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D4_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D4_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D5_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D5_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D5_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D6_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D6_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D6_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D7_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D7_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_D7_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_DATA1_RESETVAL           (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_RESETVAL                     (0x001F0007U)

/* DSS_L3_BANKB_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1 */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D4_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D4_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D4_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D5_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D5_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D5_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D6_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D6_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D6_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D7_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D7_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_D7_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_DATA1_RESETVAL           (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_RESETVAL                     (0x001F0007U)

/* DSS_L3_BANKC_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1 */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D4_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D4_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D4_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D5_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D5_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D5_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D6_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D6_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D6_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D7_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D7_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_D7_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_DATA1_RESETVAL           (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_RESETVAL                     (0x001F0007U)

/* DSS_L3_BANKD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1 */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D4_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D4_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D4_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D4_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D5_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D5_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D5_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D5_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D6_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D6_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D6_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D6_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D7_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D7_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D7_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_D7_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_DATA1_RESETVAL           (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL_RESETVAL                     (0x000F0007U)

/* DSS_DSP_SDMA_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_DSS_DSP_SDMA_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_DSS_DSP_SDMA_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_TPTC_A0_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_A0_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_DSS_TPTC_A0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_A0_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_A1_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_A1_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_DSS_TPTC_A1_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_A1_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_B0_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_B0_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_DSS_TPTC_B0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_B0_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_B1_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_B1_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_DSS_TPTC_B1_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_B1_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_C0_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_C0_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_DSS_TPTC_C0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C0_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_C1_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_C1_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_DSS_TPTC_C1_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C1_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_C2_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_C2_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_DSS_TPTC_C2_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C2_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_C3_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_C3_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_DSS_TPTC_C3_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C3_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_C4_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_C4_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_DSS_TPTC_C4_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C4_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_C5_RD_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL_RESETVAL                   (0x00090007U)

/* DSS_TPTC_C5_RD_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_DSS_TPTC_C5_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C5_RD_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_DSS_TPTC_A0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_DSS_TPTC_A1_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_DSS_TPTC_B0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_DSS_TPTC_B1_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_DSS_TPTC_C0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_DSS_TPTC_C1_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_DSS_TPTC_C2_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_DSS_TPTC_C3_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_DSS_TPTC_C4_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_DSS_TPTC_C5_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL_RESETVAL                     (0x00070007U)

/* DSS_MDO_FIFO_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_DSS_MDO_FIFO_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_DSS_MDO_FIFO_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL_RESETVAL                   (0x00070007U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_DSS_CBUFF_FIFO_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_CTRL_RESETVAL                   (0x000F0007U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_DSS_CMC_UCOMP0_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_CTRL_RESETVAL                   (0x000F0007U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_DSS_CMC_UCOMP1_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_CTRL_RESETVAL                   (0x000F0007U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_DSS_CMC_UCOMP2_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP2_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_CTRL_RESETVAL                   (0x000F0007U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_DSS_CMC_UCOMP3_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_UCOMP3_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_CTRL_RESETVAL                     (0x000F0007U)

/* DSS_CMC_COMP_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_DSS_CMC_COMP_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_DSS_CMC_COMP_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CMC_COMP_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL_RESETVAL                         (0x000F0007U)

/* DSS_MCRC_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_DSS_MCRC_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_DSS_MCRC_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* DSS_PCR_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_DSS_PCR_BUS_SAFETY_CTRL_TYPE_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL_RESETVAL                          (0x000F0007U)

/* DSS_PCR_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SEC_MASK      (0x00000010U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SEC_SHIFT     (0x00000004U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SEC_RESETVAL  (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SEC_MAX       (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DED_MASK      (0x00000020U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DED_SHIFT     (0x00000005U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DED_RESETVAL  (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DED_MAX       (0x00000001U)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DATA_MASK     (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DATA_SHIFT    (0x00000008U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_DATA_MAX      (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_MAIN_MASK     (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_MAIN_SHIFT    (0x00000010U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_MAIN_MAX      (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SAFE_MASK     (0xFF000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SAFE_SHIFT    (0x00000018U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_DSS_PCR_BUS_SAFETY_FI_SAFE_MAX      (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI_RESETVAL                            (0x00000000U)

/* DSS_PCR_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_SEC_MASK    (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_SEC_SHIFT   (0x00000010U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_SEC_MAX     (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_DED_MASK    (0xFF000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_DED_SHIFT   (0x00000018U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_DSS_PCR_BUS_SAFETY_ERR_DED_MAX     (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_RESETVAL                           (0x00000000U)

/* DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL                (0x00000000U)

/* DSS_PCR_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                  (0x00000000U)

/* DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL                (0x00000000U)

/* DSS_PCR_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ_RESETVAL                 (0x00000000U)

/* DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL            (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_RESETVAL                     (0x001F0007U)

/* DSS_HWA_DMA0_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_RESETVAL                     (0x001F0007U)

/* DSS_HWA_DMA1_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_RESETVAL                       (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_RESETVAL                      (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL           (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_RESETVAL             (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL           (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_RESETVAL            (0x00000000U)

/* DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL       (0x00000007U)

/* DSS_CM4_M_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000001U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_CTRL_RESETVAL                        (0x00000107U)

/* DSS_CM4_M_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SEC_MASK  (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SEC_MAX   (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DED_MASK  (0x00000020U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DED_MAX   (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_DATA_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_MAIN_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_DSS_CM4_M_BUS_SAFETY_FI_SAFE_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_FI_RESETVAL                          (0x00000000U)

/* DSS_CM4_M_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_DSS_CM4_M_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_RESETVAL                         (0x00000000U)

/* DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL              (0x00000000U)

/* DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                (0x00000000U)

/* DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL              (0x00000000U)

/* DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_READ_RESETVAL               (0x00000000U)

/* DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_M_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL          (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000000FU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL_RESETVAL                        (0x000F0007U)

/* DSS_CM4_S_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SEC_MASK  (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SEC_MAX   (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DED_MASK  (0x00000020U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DED_MAX   (0x00000001U)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_DATA_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_MAIN_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_DSS_CM4_S_BUS_SAFETY_FI_SAFE_MAX  (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI_RESETVAL                          (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_DSS_CM4_S_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_RESETVAL                         (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL              (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL              (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ_RESETVAL               (0x00000000U)

/* DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL          (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_CTRL */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x0000001FU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_RESETVAL                         (0x001F0007U)

/* DSS_MBOX_BUS_SAFETY_FI */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_ERR */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0 */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D2_MASK (0x00FF0000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D2_SHIFT (0x00000010U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D2_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D2_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D3_MASK (0xFF000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D3_SHIFT (0x00000018U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D3_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D3_MAX (0x000000FFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_ERR_STAT_READ */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* HW_SPARE_RW0 */

#define SDL_DSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RW0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW1 */

#define SDL_DSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RW1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW2 */

#define SDL_DSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RW2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW3 */

#define SDL_DSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RW3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO0 */

#define SDL_DSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RO0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO1 */

#define SDL_DSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RO1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO2 */

#define SDL_DSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RO2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO3 */

#define SDL_DSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK               (0xFFFFFFFFU)
#define SDL_DSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT              (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL           (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                (0xFFFFFFFFU)

#define SDL_DSS_CTRL_HW_SPARE_RO3_RESETVAL                                     (0x00000000U)

/* DSS_DSP_MBOX_READ_DONE_ACK */

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_ACK_DSS_DSP_MBOX_READ_DONE_ACK_PROC_MASK (0xFFFFFFFFU)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_ACK_DSS_DSP_MBOX_READ_DONE_ACK_PROC_SHIFT (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_ACK_DSS_DSP_MBOX_READ_DONE_ACK_PROC_RESETVAL (0x00000000U)
#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_ACK_DSS_DSP_MBOX_READ_DONE_ACK_PROC_MAX (0xFFFFFFFFU)

#define SDL_DSS_CTRL_DSS_DSP_MBOX_READ_DONE_ACK_RESETVAL                       (0x00000000U)

/* HW_SPARE_REC */

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK              (0x00000001U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT             (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK              (0x00000002U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT             (0x00000001U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK              (0x00000004U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT             (0x00000002U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK              (0x00000008U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT             (0x00000003U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK              (0x00000010U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT             (0x00000004U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK              (0x00000020U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT             (0x00000005U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK              (0x00000040U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT             (0x00000006U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK              (0x00000080U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT             (0x00000007U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK              (0x00000100U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT             (0x00000008U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK              (0x00000200U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT             (0x00000009U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL          (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX               (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK             (0x00000400U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT            (0x0000000AU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK             (0x00000800U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT            (0x0000000BU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK             (0x00001000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT            (0x0000000CU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK             (0x00002000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT            (0x0000000DU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK             (0x00004000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT            (0x0000000EU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK             (0x00008000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT            (0x0000000FU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK             (0x00010000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT            (0x00000010U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK             (0x00020000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT            (0x00000011U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK             (0x00040000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT            (0x00000012U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK             (0x00080000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT            (0x00000013U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK             (0x00100000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT            (0x00000014U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK             (0x00200000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT            (0x00000015U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK             (0x00400000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT            (0x00000016U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK             (0x00800000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT            (0x00000017U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK             (0x01000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT            (0x00000018U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK             (0x02000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT            (0x00000019U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK             (0x04000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT            (0x0000001AU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK             (0x08000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT            (0x0000001BU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK             (0x10000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT            (0x0000001CU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK             (0x20000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT            (0x0000001DU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK             (0x40000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT            (0x0000001EU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK             (0x80000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT            (0x0000001FU)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL         (0x00000000U)
#define SDL_DSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX              (0x00000001U)

#define SDL_DSS_CTRL_HW_SPARE_REC_RESETVAL                                     (0x00000000U)

/* LOCK0_KICK0 */

#define SDL_DSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                              (0xFFFFFFFFU)
#define SDL_DSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                             (0x00000000U)
#define SDL_DSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                          (0x00000000U)
#define SDL_DSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                               (0xFFFFFFFFU)

#define SDL_DSS_CTRL_LOCK0_KICK0_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK1 */

#define SDL_DSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                              (0xFFFFFFFFU)
#define SDL_DSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                             (0x00000000U)
#define SDL_DSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                          (0x00000000U)
#define SDL_DSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                               (0xFFFFFFFFU)

#define SDL_DSS_CTRL_LOCK0_KICK1_RESETVAL                                      (0x00000000U)

/* INTR_RAW_STATUS */

#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                             (0x00000001U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                            (0x00000000U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                         (0x00000000U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                              (0x00000001U)

#define SDL_DSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                             (0x00000002U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                            (0x00000001U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                         (0x00000000U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                              (0x00000001U)

#define SDL_DSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                             (0x00000004U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                            (0x00000002U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                         (0x00000000U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                              (0x00000001U)

#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                            (0x00000008U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                           (0x00000003U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                        (0x00000000U)
#define SDL_DSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                             (0x00000001U)

#define SDL_DSS_CTRL_INTR_RAW_STATUS_RESETVAL                                  (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK           (0x00000001U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT          (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL       (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX            (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK           (0x00000002U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT          (0x00000001U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL       (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX            (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK           (0x00000004U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT          (0x00000002U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL       (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX            (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK          (0x00000008U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT         (0x00000003U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL      (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX           (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                        (0x00000000U)

/* INTR_ENABLE */

#define SDL_DSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                              (0x00000001U)
#define SDL_DSS_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                             (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                               (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                              (0x00000002U)
#define SDL_DSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                             (0x00000001U)
#define SDL_DSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                               (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                              (0x00000004U)
#define SDL_DSS_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                             (0x00000002U)
#define SDL_DSS_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                          (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                               (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                             (0x00000008U)
#define SDL_DSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                            (0x00000003U)
#define SDL_DSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                         (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                              (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_RESETVAL                                      (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                    (0x00000001U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                   (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                    (0x00000002U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                   (0x00000001U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                    (0x00000004U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                   (0x00000002U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                     (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                   (0x00000008U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                  (0x00000003U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                    (0x00000001U)

#define SDL_DSS_CTRL_INTR_ENABLE_CLEAR_RESETVAL                                (0x00000000U)

/* EOI */

#define SDL_DSS_CTRL_EOI_EOI_VECTOR_MASK                                       (0x000000FFU)
#define SDL_DSS_CTRL_EOI_EOI_VECTOR_SHIFT                                      (0x00000000U)
#define SDL_DSS_CTRL_EOI_EOI_VECTOR_RESETVAL                                   (0x00000000U)
#define SDL_DSS_CTRL_EOI_EOI_VECTOR_MAX                                        (0x000000FFU)

#define SDL_DSS_CTRL_EOI_RESETVAL                                              (0x00000000U)

/* FAULT_ADDRESS */

#define SDL_DSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                             (0xFFFFFFFFU)
#define SDL_DSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                            (0x00000000U)
#define SDL_DSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                         (0x00000000U)
#define SDL_DSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                              (0xFFFFFFFFU)

#define SDL_DSS_CTRL_FAULT_ADDRESS_RESETVAL                                    (0x00000000U)

/* FAULT_TYPE_STATUS */

#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                         (0x0000003FU)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                        (0x00000000U)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                     (0x00000000U)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                          (0x0000003FU)

#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                           (0x00000040U)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                          (0x00000006U)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                       (0x00000000U)
#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                            (0x00000001U)

#define SDL_DSS_CTRL_FAULT_TYPE_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_ATTR_STATUS */

#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                       (0x000000FFU)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                      (0x00000000U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                   (0x00000000U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                        (0x000000FFU)

#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                      (0x000FFF00U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                     (0x00000008U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                  (0x00000000U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                       (0x00000FFFU)

#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                          (0xFFF00000U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                         (0x00000014U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                      (0x00000000U)
#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                           (0x00000FFFU)

#define SDL_DSS_CTRL_FAULT_ATTR_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_CLEAR */

#define SDL_DSS_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                                (0x00000001U)
#define SDL_DSS_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                               (0x00000000U)
#define SDL_DSS_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                            (0x00000000U)
#define SDL_DSS_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                                 (0x00000001U)

#define SDL_DSS_CTRL_FAULT_CLEAR_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
