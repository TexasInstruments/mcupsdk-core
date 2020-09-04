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
 *  Name        : cslr_rss_ctrl.h
*/
#ifndef CSLR_RSS_CTRL_H_
#define CSLR_RSS_CTRL_H_

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
    volatile uint32_t PID;
    volatile uint8_t  Resv_8[4];
    volatile uint32_t RSS_TPCC_A_ERRAGG_MASK;
    volatile uint32_t RSS_TPCC_A_ERRAGG_STATUS;
    volatile uint32_t RSS_TPCC_A_ERRAGG_STATUS_RAW;
    volatile uint32_t RSS_TPCC_A_INTAGG_MASK;
    volatile uint32_t RSS_TPCC_A_INTAGG_STATUS;
    volatile uint32_t RSS_TPCC_A_INTAGG_STATUS_RAW;
    volatile uint32_t RSS_TPCC_MEMINIT_START;
    volatile uint32_t RSS_TPCC_MEMINIT_DONE;
    volatile uint32_t RSS_TPCC_MEMINIT_STATUS;
    volatile uint32_t TPTC_DBS_CFG;
    volatile uint32_t RSS_TPCC_A_PARITY_CTRL;
    volatile uint32_t RSS_TPCC_A_PARITY_STATUS;
    volatile uint32_t RSS_CSI2A_CFG;
    volatile uint32_t RSS_CSI2A_CTX_LINE_PING_PONG[8];
    volatile uint32_t RSS_CSI2A_PARITY_CTRL;
    volatile uint32_t RSS_CSI2A_PARITY_STATUS;
    volatile uint32_t RSS_CSI2A_LANE0_CFG;
    volatile uint32_t RSS_CSI2A_LANE1_CFG;
    volatile uint32_t RSS_CSI2A_LANE2_CFG;
    volatile uint32_t RSS_CSI2A_LANE3_CFG;
    volatile uint32_t RSS_CSI2A_LANE4_CFG;
    volatile uint32_t RSS_CSI2A_FIFO_MEMINIT;
    volatile uint32_t RSS_CSI2A_FIFO_MEMINIT_DONE;
    volatile uint32_t RSS_CSI2A_FIFO_MEMINIT_STATUS;
    volatile uint32_t RSS_CSI2A_CTX_MEMINIT;
    volatile uint32_t RSS_CSI2A_CTX_MEMINIT_DONE;
    volatile uint32_t RSS_CSI2A_CTX_MEMINIT_STATUS;
    volatile uint32_t RSS_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_BUS_SAFETY_SEC_ERR_STAT0;
    volatile uint32_t RSS_TPTCA0_RD_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_TPTCA0_RD_BUS_SAFETY_FI;
    volatile uint32_t RSS_TPTCA0_RD_BUS_SAFETY_ERR;
    volatile uint32_t RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_FI;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_ERR;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_FI;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_ERR;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_PCR_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_PCR_BUS_SAFETY_FI;
    volatile uint32_t RSS_PCR_BUS_SAFETY_ERR;
    volatile uint32_t RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_PCR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_PCR_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_ADCBUF_RD_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_ADCBUF_RD_BUS_SAFETY_FI;
    volatile uint32_t RSS_ADCBUF_RD_BUS_SAFETY_ERR;
    volatile uint32_t RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_FI;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_ERR;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_CQ_MEM_RD_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_CQ_MEM_RD_BUS_SAFETY_FI;
    volatile uint32_t RSS_CQ_MEM_RD_BUS_SAFETY_ERR;
    volatile uint32_t RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_FI;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_ERR;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint8_t  Resv_500[128];
    volatile uint32_t RSS_MBOX_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_FI;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_ERR;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_FI;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_ERR;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_FI;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_ERR;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_CTRL;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_FI;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_ERR;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ;
    volatile uint32_t RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP;
    volatile uint32_t RSS_TPTC_BOUNDARY_CFG;
    volatile uint32_t RSS_TPTC_XID_REORDER_CFG;
    volatile uint32_t DBG_ACK_CPU_CTRL;
    volatile uint32_t RSS_ADCBUF_PING_MEMINIT;
    volatile uint32_t RSS_ADCBUF_PING_MEMINIT_DONE;
    volatile uint32_t RSS_ADCBUF_PING_MEMINIT_STATUS;
    volatile uint32_t RSS_ADCBUF_PONG_MEMINIT;
    volatile uint32_t RSS_ADCBUF_PONG_MEMINIT_DONE;
    volatile uint32_t RSS_ADCBUF_PONG_MEMINIT_STATUS;
    volatile uint8_t  Resv_712[48];
    volatile uint32_t SOC_TO_BSS_SW_INT;
    volatile uint32_t RSS_DBG_ACK_CTL0;
    volatile uint32_t DMMSWINT1;
    volatile uint32_t RSS_SHARED_MEM_MEMINIT;
    volatile uint32_t RSS_SHARED_MEM_MEMINIT_DONE;
    volatile uint32_t RSS_SHARED_MEM_MEMINIT_STATUS;
    volatile uint32_t RSS_CSI_ACCESS_MODE;
    volatile uint8_t  Resv_1024[284];
    volatile uint32_t BSS_CONTROL;
    volatile uint32_t BSS_TCM_MEMINIT;
    volatile uint32_t BSS_TCM_MEMINIT_DONE;
    volatile uint32_t BSS_TCM_MEMINIT_STATUS;
    volatile uint32_t BSS_VIM_MEMINIT;
    volatile uint32_t BSS_VIM_MEMINIT_DONE;
    volatile uint32_t BSS_VIM_MEMINIT_STATUS;
    volatile uint32_t BSS_DFE_MEMINIT;
    volatile uint32_t BSS_DFE_MEMINIT_DONE;
    volatile uint32_t BSS_DFE_MEMINIT_STATUS;
    volatile uint32_t BSS_RAMPGEN_MEMINIT;
    volatile uint32_t BSS_RAMPGEN_MEMINIT_DONE;
    volatile uint32_t BSS_RAMPGEN_MEMINIT_STATUS;
    volatile uint32_t BSS_DSS_L3_STICKY;
    volatile uint32_t BSS_DSS_L3_ACCESS;
    volatile uint8_t  Resv_2048[964];
    volatile uint32_t TESTPATTERNRX1ICFG;
    volatile uint32_t TESTPATTERNRX2ICFG;
    volatile uint32_t TESTPATTERNRX3ICFG;
    volatile uint32_t TESTPATTERNRX4ICFG;
    volatile uint32_t TESTPATTERNRX1QCFG;
    volatile uint32_t TESTPATTERNRX2QCFG;
    volatile uint32_t TESTPATTERNRX3QCFG;
    volatile uint32_t TESTPATTERNRX4QCFG;
    volatile uint32_t TESTPATTERNVLDCFG;
    volatile uint32_t ADCBUFCFG1;
    volatile uint32_t ADCBUFCFG1_EXTD;
    volatile uint32_t ADCBUFCFG2;
    volatile uint32_t ADCBUFCFG3;
    volatile uint32_t ADCBUFCFG4;
    volatile uint32_t ADCBUFINTGENDITHERDLY;
    volatile uint32_t CBUFF_FRAME_START_SEL;
    volatile uint8_t  Resv_3072[960];
    volatile uint32_t CQCFG1;
    volatile uint32_t CQCFG2;
    volatile uint32_t CPREG0;
    volatile uint32_t CPREG1;
    volatile uint32_t CPREG2;
    volatile uint32_t CPREG3;
    volatile uint32_t CPREG4;
    volatile uint32_t CPREG5;
    volatile uint32_t CPREG6;
    volatile uint32_t CPREG7;
    volatile uint32_t CPREG8;
    volatile uint32_t CPREG9;
    volatile uint32_t CPREG10;
    volatile uint32_t CPREG11;
    volatile uint32_t CPREG12;
    volatile uint32_t CPREG13;
    volatile uint32_t CPREG14;
    volatile uint32_t CPREG15;
    volatile uint32_t CH0CPREG0;
    volatile uint32_t CH0CPREG1;
    volatile uint32_t CH0CPREG2;
    volatile uint32_t CH0CPREG3;
    volatile uint32_t CH0CPREG4;
    volatile uint32_t CH0CPREG5;
    volatile uint32_t CH0CPREG6;
    volatile uint32_t CH0CPREG7;
    volatile uint32_t CH0CPREG8;
    volatile uint32_t CH0CPREG9;
    volatile uint32_t CH0CPREG10;
    volatile uint32_t CH0CPREG11;
    volatile uint32_t CH0CPREG12;
    volatile uint32_t CH0CPREG13;
    volatile uint32_t CH0CPREG14;
    volatile uint32_t CH0CPREG15;
    volatile uint32_t CH1CPREG0;
    volatile uint32_t CH1CPREG1;
    volatile uint32_t CH1CPREG2;
    volatile uint32_t CH1CPREG3;
    volatile uint32_t CH1CPREG4;
    volatile uint32_t CH1CPREG5;
    volatile uint32_t CH1CPREG6;
    volatile uint32_t CH1CPREG7;
    volatile uint32_t CH1CPREG8;
    volatile uint32_t CH1CPREG9;
    volatile uint32_t CH1CPREG10;
    volatile uint32_t CH1CPREG11;
    volatile uint32_t CH1CPREG12;
    volatile uint32_t CH1CPREG13;
    volatile uint32_t CH1CPREG14;
    volatile uint32_t CH1CPREG15;
    volatile uint32_t CH2CPREG0;
    volatile uint32_t CH2CPREG1;
    volatile uint32_t CH2CPREG2;
    volatile uint32_t CH2CPREG3;
    volatile uint32_t CH2CPREG4;
    volatile uint32_t CH2CPREG5;
    volatile uint32_t CH2CPREG6;
    volatile uint32_t CH2CPREG7;
    volatile uint32_t CH2CPREG8;
    volatile uint32_t CH2CPREG9;
    volatile uint32_t CH2CPREG10;
    volatile uint32_t CH2CPREG11;
    volatile uint32_t CH2CPREG12;
    volatile uint32_t CH2CPREG13;
    volatile uint32_t CH2CPREG14;
    volatile uint32_t CH2CPREG15;
    volatile uint32_t CH3CPREG0;
    volatile uint32_t CH3CPREG1;
    volatile uint32_t CH3CPREG2;
    volatile uint32_t CH3CPREG3;
    volatile uint32_t CH3CPREG4;
    volatile uint32_t CH3CPREG5;
    volatile uint32_t CH3CPREG6;
    volatile uint32_t CH3CPREG7;
    volatile uint32_t CH3CPREG8;
    volatile uint32_t CH3CPREG9;
    volatile uint32_t CH3CPREG10;
    volatile uint32_t CH3CPREG11;
    volatile uint32_t CH3CPREG12;
    volatile uint32_t CH3CPREG13;
    volatile uint32_t CH3CPREG14;
    volatile uint32_t CH3CPREG15;
    volatile uint32_t CH4CPREG0;
    volatile uint32_t CH4CPREG1;
    volatile uint32_t CH4CPREG2;
    volatile uint32_t CH4CPREG3;
    volatile uint32_t CH4CPREG4;
    volatile uint32_t CH4CPREG5;
    volatile uint32_t CH4CPREG6;
    volatile uint32_t CH4CPREG7;
    volatile uint32_t CH4CPREG8;
    volatile uint32_t CH4CPREG9;
    volatile uint32_t CH4CPREG10;
    volatile uint32_t CH4CPREG11;
    volatile uint32_t CH4CPREG12;
    volatile uint32_t CH4CPREG13;
    volatile uint32_t CH4CPREG14;
    volatile uint32_t CH4CPREG15;
    volatile uint32_t CH5CPREG0;
    volatile uint32_t CH5CPREG1;
    volatile uint32_t CH5CPREG2;
    volatile uint32_t CH5CPREG3;
    volatile uint32_t CH5CPREG4;
    volatile uint32_t CH5CPREG5;
    volatile uint32_t CH5CPREG6;
    volatile uint32_t CH5CPREG7;
    volatile uint32_t CH5CPREG8;
    volatile uint32_t CH5CPREG9;
    volatile uint32_t CH5CPREG10;
    volatile uint32_t CH5CPREG11;
    volatile uint32_t CH5CPREG12;
    volatile uint32_t CH5CPREG13;
    volatile uint32_t CH5CPREG14;
    volatile uint32_t CH5CPREG15;
    volatile uint32_t CH6CPREG0;
    volatile uint32_t CH6CPREG1;
    volatile uint32_t CH6CPREG2;
    volatile uint32_t CH6CPREG3;
    volatile uint32_t CH6CPREG4;
    volatile uint32_t CH6CPREG5;
    volatile uint32_t CH6CPREG6;
    volatile uint32_t CH6CPREG7;
    volatile uint32_t CH6CPREG8;
    volatile uint32_t CH6CPREG9;
    volatile uint32_t CH6CPREG10;
    volatile uint32_t CH6CPREG11;
    volatile uint32_t CH6CPREG12;
    volatile uint32_t CH6CPREG13;
    volatile uint32_t CH6CPREG14;
    volatile uint32_t CH6CPREG15;
    volatile uint32_t CH7CPREG0;
    volatile uint32_t CH7CPREG1;
    volatile uint32_t CH7CPREG2;
    volatile uint32_t CH7CPREG3;
    volatile uint32_t CH7CPREG4;
    volatile uint32_t CH7CPREG5;
    volatile uint32_t CH7CPREG6;
    volatile uint32_t CH7CPREG7;
    volatile uint32_t CH7CPREG8;
    volatile uint32_t CH7CPREG9;
    volatile uint32_t CH7CPREG10;
    volatile uint32_t CH7CPREG11;
    volatile uint32_t CH7CPREG12;
    volatile uint32_t CH7CPREG13;
    volatile uint32_t CH7CPREG14;
    volatile uint32_t CH7CPREG15;
    volatile uint32_t CH01_HIL_CP_OVERRIDE;
    volatile uint32_t CH23_HIL_CP_OVERRIDE;
    volatile uint32_t CH45_HIL_CP_OVERRIDE;
    volatile uint32_t CH67_HIL_CP_OVERRIDE;
    volatile uint32_t CH_HIL_CP_OVERRIDE;
    volatile uint8_t  Resv_4048[372];
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
} CSL_rss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RSS_CTRL_PID                                                       (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK                                    (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS                                  (0x0000000CU)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW                              (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK                                    (0x00000014U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS                                  (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW                              (0x0000001CU)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_START                                    (0x00000020U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_DONE                                     (0x00000024U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_STATUS                                   (0x00000028U)
#define CSL_RSS_CTRL_TPTC_DBS_CFG                                              (0x0000002CU)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL                                    (0x00000030U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_STATUS                                  (0x00000034U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG                                             (0x00000038U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG                             (0x0000003CU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG                             (0x00000040U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG                             (0x00000044U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG                             (0x00000048U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG                             (0x0000004CU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG                             (0x00000050U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG                             (0x00000054U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG                             (0x00000058U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL                                     (0x0000005CU)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS                                   (0x00000060U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG                                       (0x00000064U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG                                       (0x00000068U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG                                       (0x0000006CU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG                                       (0x00000070U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG                                       (0x00000074U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT                                    (0x00000078U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_DONE                               (0x0000007CU)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_STATUS                             (0x00000080U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT                                     (0x00000084U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_DONE                                (0x00000088U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_STATUS                              (0x0000008CU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL                                       (0x00000090U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0                              (0x00000094U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL                             (0x00000098U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI                               (0x0000009CU)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR                              (0x000000A0U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0                   (0x000000A4U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD                     (0x000000A8U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ                    (0x000000ACU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL                             (0x000000B0U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI                               (0x000000B4U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR                              (0x000000B8U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0                   (0x000000BCU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD                     (0x000000C0U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE                   (0x000000C4U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP               (0x000000C8U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL                            (0x000000CCU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI                              (0x000000D0U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR                             (0x000000D4U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0                  (0x000000D8U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD                    (0x000000DCU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE                  (0x000000E0U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ                   (0x000000E4U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000000E8U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL                                   (0x000000ECU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI                                     (0x000000F0U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR                                    (0x000000F4U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0                         (0x000000F8U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD                           (0x000000FCU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE                         (0x00000100U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_READ                          (0x00000104U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP                     (0x00000108U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL                             (0x0000010CU)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI                               (0x00000110U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR                              (0x00000114U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0                   (0x00000118U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD                     (0x0000011CU)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ                    (0x00000120U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL                             (0x00000124U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI                               (0x00000128U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR                              (0x0000012CU)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0                   (0x00000130U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD                     (0x00000134U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE                   (0x00000138U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP               (0x0000013CU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL                             (0x00000140U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI                               (0x00000144U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR                              (0x00000148U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0                   (0x0000014CU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD                     (0x00000150U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ                    (0x00000154U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL                             (0x00000158U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI                               (0x0000015CU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR                              (0x00000160U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0                   (0x00000164U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD                     (0x00000168U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE                   (0x0000016CU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP               (0x00000170U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL                                  (0x000001F4U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI                                    (0x000001F8U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR                                   (0x000001FCU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0                        (0x00000200U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD                          (0x00000204U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000208U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ                         (0x0000020CU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x00000210U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL                            (0x00000214U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI                              (0x00000218U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR                             (0x0000021CU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000220U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD                    (0x00000224U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000228U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ                   (0x0000022CU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000230U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL                               (0x00000234U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI                                 (0x00000238U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR                                (0x0000023CU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0                     (0x00000240U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD                       (0x00000244U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE                     (0x00000248U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ                      (0x0000024CU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP                 (0x00000250U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL                               (0x00000254U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI                                 (0x00000258U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR                                (0x0000025CU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0                     (0x00000260U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD                       (0x00000264U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE                     (0x00000268U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ                      (0x0000026CU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP                 (0x00000270U)
#define CSL_RSS_CTRL_RSS_TPTC_BOUNDARY_CFG                                     (0x00000274U)
#define CSL_RSS_CTRL_RSS_TPTC_XID_REORDER_CFG                                  (0x00000278U)
#define CSL_RSS_CTRL_DBG_ACK_CPU_CTRL                                          (0x0000027CU)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT                                   (0x00000280U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE                              (0x00000284U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_STATUS                            (0x00000288U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT                                   (0x0000028CU)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE                              (0x00000290U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_STATUS                            (0x00000294U)
#define CSL_RSS_CTRL_SOC_TO_BSS_SW_INT                                         (0x000002C8U)
#define CSL_RSS_CTRL_RSS_DBG_ACK_CTL0                                          (0x000002CCU)
#define CSL_RSS_CTRL_DMMSWINT1                                                 (0x000002D0U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT                                    (0x000002D4U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_DONE                               (0x000002D8U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_STATUS                             (0x000002DCU)
#define CSL_RSS_CTRL_RSS_CSI_ACCESS_MODE                                       (0x000002E0U)
#define CSL_RSS_CTRL_BSS_CONTROL                                               (0x00000400U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT                                           (0x00000404U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_DONE                                      (0x00000408U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_STATUS                                    (0x0000040CU)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT                                           (0x00000410U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_DONE                                      (0x00000414U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_STATUS                                    (0x00000418U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT                                           (0x0000041CU)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_DONE                                      (0x00000420U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_STATUS                                    (0x00000424U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT                                       (0x00000428U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_DONE                                  (0x0000042CU)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_STATUS                                (0x00000430U)
#define CSL_RSS_CTRL_BSS_DSS_L3_STICKY                                         (0x00000434U)
#define CSL_RSS_CTRL_BSS_DSS_L3_ACCESS                                         (0x00000438U)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG                                        (0x00000800U)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG                                        (0x00000804U)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG                                        (0x00000808U)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG                                        (0x0000080CU)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG                                        (0x00000810U)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG                                        (0x00000814U)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG                                        (0x00000818U)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG                                        (0x0000081CU)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG                                         (0x00000820U)
#define CSL_RSS_CTRL_ADCBUFCFG1                                                (0x00000824U)
#define CSL_RSS_CTRL_ADCBUFCFG1_EXTD                                           (0x00000828U)
#define CSL_RSS_CTRL_ADCBUFCFG2                                                (0x0000082CU)
#define CSL_RSS_CTRL_ADCBUFCFG3                                                (0x00000830U)
#define CSL_RSS_CTRL_ADCBUFCFG4                                                (0x00000834U)
#define CSL_RSS_CTRL_ADCBUFINTGENDITHERDLY                                     (0x00000838U)
#define CSL_RSS_CTRL_CBUFF_FRAME_START_SEL                                     (0x0000083CU)
#define CSL_RSS_CTRL_CQCFG1                                                    (0x00000C00U)
#define CSL_RSS_CTRL_CQCFG2                                                    (0x00000C04U)
#define CSL_RSS_CTRL_CPREG0                                                    (0x00000C08U)
#define CSL_RSS_CTRL_CPREG1                                                    (0x00000C0CU)
#define CSL_RSS_CTRL_CPREG2                                                    (0x00000C10U)
#define CSL_RSS_CTRL_CPREG3                                                    (0x00000C14U)
#define CSL_RSS_CTRL_CPREG4                                                    (0x00000C18U)
#define CSL_RSS_CTRL_CPREG5                                                    (0x00000C1CU)
#define CSL_RSS_CTRL_CPREG6                                                    (0x00000C20U)
#define CSL_RSS_CTRL_CPREG7                                                    (0x00000C24U)
#define CSL_RSS_CTRL_CPREG8                                                    (0x00000C28U)
#define CSL_RSS_CTRL_CPREG9                                                    (0x00000C2CU)
#define CSL_RSS_CTRL_CPREG10                                                   (0x00000C30U)
#define CSL_RSS_CTRL_CPREG11                                                   (0x00000C34U)
#define CSL_RSS_CTRL_CPREG12                                                   (0x00000C38U)
#define CSL_RSS_CTRL_CPREG13                                                   (0x00000C3CU)
#define CSL_RSS_CTRL_CPREG14                                                   (0x00000C40U)
#define CSL_RSS_CTRL_CPREG15                                                   (0x00000C44U)
#define CSL_RSS_CTRL_CH0CPREG0                                                 (0x00000C48U)
#define CSL_RSS_CTRL_CH0CPREG1                                                 (0x00000C4CU)
#define CSL_RSS_CTRL_CH0CPREG2                                                 (0x00000C50U)
#define CSL_RSS_CTRL_CH0CPREG3                                                 (0x00000C54U)
#define CSL_RSS_CTRL_CH0CPREG4                                                 (0x00000C58U)
#define CSL_RSS_CTRL_CH0CPREG5                                                 (0x00000C5CU)
#define CSL_RSS_CTRL_CH0CPREG6                                                 (0x00000C60U)
#define CSL_RSS_CTRL_CH0CPREG7                                                 (0x00000C64U)
#define CSL_RSS_CTRL_CH0CPREG8                                                 (0x00000C68U)
#define CSL_RSS_CTRL_CH0CPREG9                                                 (0x00000C6CU)
#define CSL_RSS_CTRL_CH0CPREG10                                                (0x00000C70U)
#define CSL_RSS_CTRL_CH0CPREG11                                                (0x00000C74U)
#define CSL_RSS_CTRL_CH0CPREG12                                                (0x00000C78U)
#define CSL_RSS_CTRL_CH0CPREG13                                                (0x00000C7CU)
#define CSL_RSS_CTRL_CH0CPREG14                                                (0x00000C80U)
#define CSL_RSS_CTRL_CH0CPREG15                                                (0x00000C84U)
#define CSL_RSS_CTRL_CH1CPREG0                                                 (0x00000C88U)
#define CSL_RSS_CTRL_CH1CPREG1                                                 (0x00000C8CU)
#define CSL_RSS_CTRL_CH1CPREG2                                                 (0x00000C90U)
#define CSL_RSS_CTRL_CH1CPREG3                                                 (0x00000C94U)
#define CSL_RSS_CTRL_CH1CPREG4                                                 (0x00000C98U)
#define CSL_RSS_CTRL_CH1CPREG5                                                 (0x00000C9CU)
#define CSL_RSS_CTRL_CH1CPREG6                                                 (0x00000CA0U)
#define CSL_RSS_CTRL_CH1CPREG7                                                 (0x00000CA4U)
#define CSL_RSS_CTRL_CH1CPREG8                                                 (0x00000CA8U)
#define CSL_RSS_CTRL_CH1CPREG9                                                 (0x00000CACU)
#define CSL_RSS_CTRL_CH1CPREG10                                                (0x00000CB0U)
#define CSL_RSS_CTRL_CH1CPREG11                                                (0x00000CB4U)
#define CSL_RSS_CTRL_CH1CPREG12                                                (0x00000CB8U)
#define CSL_RSS_CTRL_CH1CPREG13                                                (0x00000CBCU)
#define CSL_RSS_CTRL_CH1CPREG14                                                (0x00000CC0U)
#define CSL_RSS_CTRL_CH1CPREG15                                                (0x00000CC4U)
#define CSL_RSS_CTRL_CH2CPREG0                                                 (0x00000CC8U)
#define CSL_RSS_CTRL_CH2CPREG1                                                 (0x00000CCCU)
#define CSL_RSS_CTRL_CH2CPREG2                                                 (0x00000CD0U)
#define CSL_RSS_CTRL_CH2CPREG3                                                 (0x00000CD4U)
#define CSL_RSS_CTRL_CH2CPREG4                                                 (0x00000CD8U)
#define CSL_RSS_CTRL_CH2CPREG5                                                 (0x00000CDCU)
#define CSL_RSS_CTRL_CH2CPREG6                                                 (0x00000CE0U)
#define CSL_RSS_CTRL_CH2CPREG7                                                 (0x00000CE4U)
#define CSL_RSS_CTRL_CH2CPREG8                                                 (0x00000CE8U)
#define CSL_RSS_CTRL_CH2CPREG9                                                 (0x00000CECU)
#define CSL_RSS_CTRL_CH2CPREG10                                                (0x00000CF0U)
#define CSL_RSS_CTRL_CH2CPREG11                                                (0x00000CF4U)
#define CSL_RSS_CTRL_CH2CPREG12                                                (0x00000CF8U)
#define CSL_RSS_CTRL_CH2CPREG13                                                (0x00000CFCU)
#define CSL_RSS_CTRL_CH2CPREG14                                                (0x00000D00U)
#define CSL_RSS_CTRL_CH2CPREG15                                                (0x00000D04U)
#define CSL_RSS_CTRL_CH3CPREG0                                                 (0x00000D08U)
#define CSL_RSS_CTRL_CH3CPREG1                                                 (0x00000D0CU)
#define CSL_RSS_CTRL_CH3CPREG2                                                 (0x00000D10U)
#define CSL_RSS_CTRL_CH3CPREG3                                                 (0x00000D14U)
#define CSL_RSS_CTRL_CH3CPREG4                                                 (0x00000D18U)
#define CSL_RSS_CTRL_CH3CPREG5                                                 (0x00000D1CU)
#define CSL_RSS_CTRL_CH3CPREG6                                                 (0x00000D20U)
#define CSL_RSS_CTRL_CH3CPREG7                                                 (0x00000D24U)
#define CSL_RSS_CTRL_CH3CPREG8                                                 (0x00000D28U)
#define CSL_RSS_CTRL_CH3CPREG9                                                 (0x00000D2CU)
#define CSL_RSS_CTRL_CH3CPREG10                                                (0x00000D30U)
#define CSL_RSS_CTRL_CH3CPREG11                                                (0x00000D34U)
#define CSL_RSS_CTRL_CH3CPREG12                                                (0x00000D38U)
#define CSL_RSS_CTRL_CH3CPREG13                                                (0x00000D3CU)
#define CSL_RSS_CTRL_CH3CPREG14                                                (0x00000D40U)
#define CSL_RSS_CTRL_CH3CPREG15                                                (0x00000D44U)
#define CSL_RSS_CTRL_CH4CPREG0                                                 (0x00000D48U)
#define CSL_RSS_CTRL_CH4CPREG1                                                 (0x00000D4CU)
#define CSL_RSS_CTRL_CH4CPREG2                                                 (0x00000D50U)
#define CSL_RSS_CTRL_CH4CPREG3                                                 (0x00000D54U)
#define CSL_RSS_CTRL_CH4CPREG4                                                 (0x00000D58U)
#define CSL_RSS_CTRL_CH4CPREG5                                                 (0x00000D5CU)
#define CSL_RSS_CTRL_CH4CPREG6                                                 (0x00000D60U)
#define CSL_RSS_CTRL_CH4CPREG7                                                 (0x00000D64U)
#define CSL_RSS_CTRL_CH4CPREG8                                                 (0x00000D68U)
#define CSL_RSS_CTRL_CH4CPREG9                                                 (0x00000D6CU)
#define CSL_RSS_CTRL_CH4CPREG10                                                (0x00000D70U)
#define CSL_RSS_CTRL_CH4CPREG11                                                (0x00000D74U)
#define CSL_RSS_CTRL_CH4CPREG12                                                (0x00000D78U)
#define CSL_RSS_CTRL_CH4CPREG13                                                (0x00000D7CU)
#define CSL_RSS_CTRL_CH4CPREG14                                                (0x00000D80U)
#define CSL_RSS_CTRL_CH4CPREG15                                                (0x00000D84U)
#define CSL_RSS_CTRL_CH5CPREG0                                                 (0x00000D88U)
#define CSL_RSS_CTRL_CH5CPREG1                                                 (0x00000D8CU)
#define CSL_RSS_CTRL_CH5CPREG2                                                 (0x00000D90U)
#define CSL_RSS_CTRL_CH5CPREG3                                                 (0x00000D94U)
#define CSL_RSS_CTRL_CH5CPREG4                                                 (0x00000D98U)
#define CSL_RSS_CTRL_CH5CPREG5                                                 (0x00000D9CU)
#define CSL_RSS_CTRL_CH5CPREG6                                                 (0x00000DA0U)
#define CSL_RSS_CTRL_CH5CPREG7                                                 (0x00000DA4U)
#define CSL_RSS_CTRL_CH5CPREG8                                                 (0x00000DA8U)
#define CSL_RSS_CTRL_CH5CPREG9                                                 (0x00000DACU)
#define CSL_RSS_CTRL_CH5CPREG10                                                (0x00000DB0U)
#define CSL_RSS_CTRL_CH5CPREG11                                                (0x00000DB4U)
#define CSL_RSS_CTRL_CH5CPREG12                                                (0x00000DB8U)
#define CSL_RSS_CTRL_CH5CPREG13                                                (0x00000DBCU)
#define CSL_RSS_CTRL_CH5CPREG14                                                (0x00000DC0U)
#define CSL_RSS_CTRL_CH5CPREG15                                                (0x00000DC4U)
#define CSL_RSS_CTRL_CH6CPREG0                                                 (0x00000DC8U)
#define CSL_RSS_CTRL_CH6CPREG1                                                 (0x00000DCCU)
#define CSL_RSS_CTRL_CH6CPREG2                                                 (0x00000DD0U)
#define CSL_RSS_CTRL_CH6CPREG3                                                 (0x00000DD4U)
#define CSL_RSS_CTRL_CH6CPREG4                                                 (0x00000DD8U)
#define CSL_RSS_CTRL_CH6CPREG5                                                 (0x00000DDCU)
#define CSL_RSS_CTRL_CH6CPREG6                                                 (0x00000DE0U)
#define CSL_RSS_CTRL_CH6CPREG7                                                 (0x00000DE4U)
#define CSL_RSS_CTRL_CH6CPREG8                                                 (0x00000DE8U)
#define CSL_RSS_CTRL_CH6CPREG9                                                 (0x00000DECU)
#define CSL_RSS_CTRL_CH6CPREG10                                                (0x00000DF0U)
#define CSL_RSS_CTRL_CH6CPREG11                                                (0x00000DF4U)
#define CSL_RSS_CTRL_CH6CPREG12                                                (0x00000DF8U)
#define CSL_RSS_CTRL_CH6CPREG13                                                (0x00000DFCU)
#define CSL_RSS_CTRL_CH6CPREG14                                                (0x00000E00U)
#define CSL_RSS_CTRL_CH6CPREG15                                                (0x00000E04U)
#define CSL_RSS_CTRL_CH7CPREG0                                                 (0x00000E08U)
#define CSL_RSS_CTRL_CH7CPREG1                                                 (0x00000E0CU)
#define CSL_RSS_CTRL_CH7CPREG2                                                 (0x00000E10U)
#define CSL_RSS_CTRL_CH7CPREG3                                                 (0x00000E14U)
#define CSL_RSS_CTRL_CH7CPREG4                                                 (0x00000E18U)
#define CSL_RSS_CTRL_CH7CPREG5                                                 (0x00000E1CU)
#define CSL_RSS_CTRL_CH7CPREG6                                                 (0x00000E20U)
#define CSL_RSS_CTRL_CH7CPREG7                                                 (0x00000E24U)
#define CSL_RSS_CTRL_CH7CPREG8                                                 (0x00000E28U)
#define CSL_RSS_CTRL_CH7CPREG9                                                 (0x00000E2CU)
#define CSL_RSS_CTRL_CH7CPREG10                                                (0x00000E30U)
#define CSL_RSS_CTRL_CH7CPREG11                                                (0x00000E34U)
#define CSL_RSS_CTRL_CH7CPREG12                                                (0x00000E38U)
#define CSL_RSS_CTRL_CH7CPREG13                                                (0x00000E3CU)
#define CSL_RSS_CTRL_CH7CPREG14                                                (0x00000E40U)
#define CSL_RSS_CTRL_CH7CPREG15                                                (0x00000E44U)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE                                      (0x00000E48U)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE                                      (0x00000E4CU)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE                                      (0x00000E50U)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE                                      (0x00000E54U)
#define CSL_RSS_CTRL_CH_HIL_CP_OVERRIDE                                        (0x00000E58U)
#define CSL_RSS_CTRL_HW_SPARE_RW0                                              (0x00000FD0U)
#define CSL_RSS_CTRL_HW_SPARE_RW1                                              (0x00000FD4U)
#define CSL_RSS_CTRL_HW_SPARE_RW2                                              (0x00000FD8U)
#define CSL_RSS_CTRL_HW_SPARE_RW3                                              (0x00000FDCU)
#define CSL_RSS_CTRL_HW_SPARE_RO0                                              (0x00000FE0U)
#define CSL_RSS_CTRL_HW_SPARE_RO1                                              (0x00000FE4U)
#define CSL_RSS_CTRL_HW_SPARE_RO2                                              (0x00000FE8U)
#define CSL_RSS_CTRL_HW_SPARE_RO3                                              (0x00000FECU)
#define CSL_RSS_CTRL_HW_SPARE_WPH                                              (0x00000FF0U)
#define CSL_RSS_CTRL_HW_SPARE_REC                                              (0x00000FF4U)
#define CSL_RSS_CTRL_LOCK0_KICK0                                               (0x00001008U)
#define CSL_RSS_CTRL_LOCK0_KICK1                                               (0x0000100CU)
#define CSL_RSS_CTRL_INTR_RAW_STATUS                                           (0x00001010U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR                                 (0x00001014U)
#define CSL_RSS_CTRL_INTR_ENABLE                                               (0x00001018U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR                                         (0x0000101CU)
#define CSL_RSS_CTRL_EOI                                                       (0x00001020U)
#define CSL_RSS_CTRL_FAULT_ADDRESS                                             (0x00001024U)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS                                         (0x00001028U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS                                         (0x0000102CU)
#define CSL_RSS_CTRL_FAULT_CLEAR                                               (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_RSS_CTRL_PID_PID_MINOR_MASK                                        (0x0000003FU)
#define CSL_RSS_CTRL_PID_PID_MINOR_SHIFT                                       (0x00000000U)
#define CSL_RSS_CTRL_PID_PID_MINOR_RESETVAL                                    (0x00000014U)
#define CSL_RSS_CTRL_PID_PID_MINOR_MAX                                         (0x0000003FU)

#define CSL_RSS_CTRL_PID_PID_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_RSS_CTRL_PID_PID_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_RSS_CTRL_PID_PID_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_RSS_CTRL_PID_PID_CUSTOM_MAX                                        (0x00000003U)

#define CSL_RSS_CTRL_PID_PID_MAJOR_MASK                                        (0x00000700U)
#define CSL_RSS_CTRL_PID_PID_MAJOR_SHIFT                                       (0x00000008U)
#define CSL_RSS_CTRL_PID_PID_MAJOR_RESETVAL                                    (0x00000002U)
#define CSL_RSS_CTRL_PID_PID_MAJOR_MAX                                         (0x00000007U)

#define CSL_RSS_CTRL_PID_PID_MISC_MASK                                         (0x0000F800U)
#define CSL_RSS_CTRL_PID_PID_MISC_SHIFT                                        (0x0000000BU)
#define CSL_RSS_CTRL_PID_PID_MISC_RESETVAL                                     (0x00000000U)
#define CSL_RSS_CTRL_PID_PID_MISC_MAX                                          (0x0000001FU)

#define CSL_RSS_CTRL_PID_PID_MSB16_MASK                                        (0xFFFF0000U)
#define CSL_RSS_CTRL_PID_PID_MSB16_SHIFT                                       (0x00000010U)
#define CSL_RSS_CTRL_PID_PID_MSB16_RESETVAL                                    (0x00006180U)
#define CSL_RSS_CTRL_PID_PID_MSB16_MAX                                         (0x0000FFFFU)

#define CSL_RSS_CTRL_PID_RESETVAL                                              (0x61800214U)

/* RSS_TPCC_A_ERRAGG_MASK */

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_ERRINT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_MPINT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_ERR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_PARITY_ERR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RSS_TPCC_A_ERRAGG_MASK_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_MASK_RESETVAL                           (0x00000000U)

/* RSS_TPCC_A_ERRAGG_STATUS */

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_ERRINT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_MPINT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_ERR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_PARITY_ERR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RSS_TPCC_A_ERRAGG_STATUS_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RESETVAL                         (0x00000000U)

/* RSS_TPCC_A_ERRAGG_STATUS_RAW */

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_ERRINT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_MPINT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_ERR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_PARITY_ERR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MASK (0x00020000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_SHIFT (0x00000011U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_WRITE_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MASK (0x01000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPCC_A_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MASK (0x02000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_SHIFT (0x00000019U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RSS_TPCC_A_ERRAGG_STATUS_RAW_TPTC_A0_READ_ACCESS_ERROR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_ERRAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* RSS_TPCC_A_INTAGG_MASK */

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INTG_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT0_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT1_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT2_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT3_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT4_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MASK (0x00000040U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_SHIFT (0x00000006U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT5_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MASK (0x00000080U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT6_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPCC_A_INT7_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPTC_A0_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPTC_A0_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPTC_A0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RSS_TPCC_A_INTAGG_MASK_TPTC_A0_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK_RESETVAL                           (0x00000000U)

/* RSS_TPCC_A_INTAGG_STATUS */

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INTG_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT0_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT1_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT2_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT3_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT4_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MASK (0x00000040U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_SHIFT (0x00000006U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT5_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MASK (0x00000080U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT6_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPCC_A_INT7_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPTC_A0_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPTC_A0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RSS_TPCC_A_INTAGG_STATUS_TPTC_A0_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RESETVAL                         (0x00000000U)

/* RSS_TPCC_A_INTAGG_STATUS_RAW */

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INTG_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT0_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT1_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT2_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT3_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT4_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MASK (0x00000040U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_SHIFT (0x00000006U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT5_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MASK (0x00000080U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT6_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPCC_A_INT7_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RSS_TPCC_A_INTAGG_STATUS_RAW_TPTC_A0_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS_RAW_RESETVAL                     (0x00000000U)

/* RSS_TPCC_MEMINIT_START */

#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_START_RSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_START_RSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_START_RSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_START_RSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_START_RESETVAL                           (0x00000000U)

/* RSS_TPCC_MEMINIT_DONE */

#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_DONE_RSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_DONE_RSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_DONE_RSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_DONE_RSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_DONE_RESETVAL                            (0x00000000U)

/* RSS_TPCC_MEMINIT_STATUS */

#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_STATUS_RSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_STATUS_RSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_STATUS_RSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_STATUS_RSS_TPCC_MEMINIT_STATUS_TPCC_A_MEMINIT_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_MEMINIT_STATUS_RESETVAL                          (0x00000000U)

/* TPTC_DBS_CFG */

#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A0_MASK                    (0x00000003U)
#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A0_SHIFT                   (0x00000000U)
#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A0_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A0_MAX                     (0x00000003U)

#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A1_MASK                    (0x0000000CU)
#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A1_SHIFT                   (0x00000002U)
#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A1_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_TPTC_DBS_CFG_TPTC_DBS_CFG_TPTC_A1_MAX                     (0x00000003U)

#define CSL_RSS_CTRL_TPTC_DBS_CFG_RESETVAL                                     (0x00000000U)

/* RSS_TPCC_A_PARITY_CTRL */

#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_EN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_EN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_EN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_TESTEN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RSS_TPCC_A_PARITY_CTRL_PARITY_ERR_CLR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_CTRL_RESETVAL                           (0x00000000U)

/* RSS_TPCC_A_PARITY_STATUS */

#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_STATUS_RSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_STATUS_RSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_STATUS_RSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_STATUS_RSS_TPCC_A_PARITY_STATUS_PARITY_ADDR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPCC_A_PARITY_STATUS_RESETVAL                         (0x00000000U)

/* RSS_CSI2A_CFG */

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_LANE_ENABLE_MASK              (0x0000001FU)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_LANE_ENABLE_SHIFT             (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_LANE_ENABLE_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_LANE_ENABLE_MAX               (0x0000001FU)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_MWAIT_MASK                    (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_MWAIT_SHIFT                   (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_MWAIT_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_MWAIT_MAX                     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SIGN_EXT_EN_MASK              (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SIGN_EXT_EN_SHIFT             (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SIGN_EXT_EN_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SIGN_EXT_EN_MAX               (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR0_SEL_MASK            (0x000E0000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR0_SEL_SHIFT           (0x00000011U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR0_SEL_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR0_SEL_MAX             (0x00000007U)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR1_SEL_MASK            (0x00700000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR1_SEL_SHIFT           (0x00000014U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR1_SEL_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_EOF_INTR1_SEL_MAX             (0x00000007U)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR0_SEL_MASK            (0x07000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR0_SEL_SHIFT           (0x00000018U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR0_SEL_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR0_SEL_MAX             (0x00000007U)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR1_SEL_MASK            (0x70000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR1_SEL_SHIFT           (0x0000001CU)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR1_SEL_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RSS_CSI2A_CFG_SOF_INTR1_SEL_MAX             (0x00000007U)

#define CSL_RSS_CTRL_RSS_CSI2A_CFG_RESETVAL                                    (0x00000000U)

/* RSS_CSI2A_CTX0_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RSS_CSI2A_CTX0_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX1_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RSS_CSI2A_CTX1_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX2_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RSS_CSI2A_CTX2_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX3_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RSS_CSI2A_CTX3_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX4_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RSS_CSI2A_CTX4_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX5_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RSS_CSI2A_CTX5_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX6_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RSS_CSI2A_CTX6_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX7_LINE_PING_PONG */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_NUM_LINES_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_NUM_LINES_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_NUM_LINES_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_NUM_LINES_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_ENABLE_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_ENABLE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RSS_CSI2A_CTX7_LINE_PING_PONG_ENABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_PARITY_CTRL */

#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_CTX_PARITY_EN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_CTX_PARITY_EN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_CTX_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_CTX_PARITY_EN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_FIFO_PARITY_EN_MASK (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_FIFO_PARITY_EN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_FIFO_PARITY_EN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RSS_CSI2A_PARITY_CTRL_FIFO_PARITY_EN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_CTRL_RESETVAL                            (0x00000000U)

/* RSS_CSI2A_PARITY_STATUS */

#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_CTX_PARITY_ADDR_MASK (0x0000000FU)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_CTX_PARITY_ADDR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_CTX_PARITY_ADDR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_CTX_PARITY_ADDR_MAX (0x0000000FU)

#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_FIFO_PARITY_ADDR_MASK (0x007F0000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_FIFO_PARITY_ADDR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_FIFO_PARITY_ADDR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RSS_CSI2A_PARITY_STATUS_FIFO_PARITY_ADDR_MAX (0x0000007FU)

#define CSL_RSS_CTRL_RSS_CSI2A_PARITY_STATUS_RESETVAL                          (0x00000000U)

/* RSS_CSI2A_LANE0_CFG */

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPU_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPU_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPD_MASK    (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPD_SHIFT   (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IN_MASK       (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IN_SHIFT      (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IE_MASK       (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IE_SHIFT      (0x00000003U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOIN_MASK    (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOIN_SHIFT   (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOIN_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKIN_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKIN_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKIN_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUIN_MASK     (0x00000040U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUIN_SHIFT    (0x00000006U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUIN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKIN_MASK  (0x00000080U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKIN_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKIN_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEN_MASK     (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEN_SHIFT    (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEVNT_MASK   (0x00000200U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEVNT_SHIFT  (0x00000009U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOOUT_MASK   (0x00000400U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOOUT_SHIFT  (0x0000000AU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOOUT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKOUT_MASK (0x00000800U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKOUT_SHIFT (0x0000000BU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_ISOCLKOUT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUOUT_MASK    (0x00001000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUOUT_SHIFT   (0x0000000CU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUOUT_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKOUT_MASK (0x00002000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKOUT_SHIFT (0x0000000DU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DX0_WUCLKOUT_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPU_MASK    (0x00004000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPU_SHIFT   (0x0000000EU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPD_MASK    (0x00008000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPD_SHIFT   (0x0000000FU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IN_MASK       (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IN_SHIFT      (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IE_MASK       (0x00020000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IE_SHIFT      (0x00000011U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEN_MASK     (0x00040000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEN_SHIFT    (0x00000012U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEVNT_MASK   (0x00080000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEVNT_SHIFT  (0x00000013U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RSS_CSI2A_LANE0_CFG_DY0_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG_RESETVAL                              (0x00004001U)

/* RSS_CSI2A_LANE1_CFG */

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPU_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPU_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPD_MASK    (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPD_SHIFT   (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IN_MASK       (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IN_SHIFT      (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IE_MASK       (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IE_SHIFT      (0x00000003U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOIN_MASK    (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOIN_SHIFT   (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOIN_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKIN_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKIN_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKIN_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUIN_MASK     (0x00000040U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUIN_SHIFT    (0x00000006U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUIN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKIN_MASK  (0x00000080U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKIN_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKIN_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEN_MASK     (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEN_SHIFT    (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEVNT_MASK   (0x00000200U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEVNT_SHIFT  (0x00000009U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOOUT_MASK   (0x00000400U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOOUT_SHIFT  (0x0000000AU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOOUT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKOUT_MASK (0x00000800U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKOUT_SHIFT (0x0000000BU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_ISOCLKOUT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUOUT_MASK    (0x00001000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUOUT_SHIFT   (0x0000000CU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUOUT_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKOUT_MASK (0x00002000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKOUT_SHIFT (0x0000000DU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DX1_WUCLKOUT_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPU_MASK    (0x00004000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPU_SHIFT   (0x0000000EU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPD_MASK    (0x00008000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPD_SHIFT   (0x0000000FU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IN_MASK       (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IN_SHIFT      (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IE_MASK       (0x00020000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IE_SHIFT      (0x00000011U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEN_MASK     (0x00040000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEN_SHIFT    (0x00000012U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEVNT_MASK   (0x00080000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEVNT_SHIFT  (0x00000013U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RSS_CSI2A_LANE1_CFG_DY1_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG_RESETVAL                              (0x00004001U)

/* RSS_CSI2A_LANE2_CFG */

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPU_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPU_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPD_MASK    (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPD_SHIFT   (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IN_MASK       (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IN_SHIFT      (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IE_MASK       (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IE_SHIFT      (0x00000003U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOIN_MASK    (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOIN_SHIFT   (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOIN_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOCLKIN_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOCLKIN_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_ISOCLKIN_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUIN_MASK     (0x00000040U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUIN_SHIFT    (0x00000006U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUIN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUCLKIN_MASK  (0x00000080U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUCLKIN_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUCLKIN_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEN_MASK     (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEN_SHIFT    (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEVNT_MASK   (0x00000200U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEVNT_SHIFT  (0x00000009U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DX2_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPU_MASK    (0x00004000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPU_SHIFT   (0x0000000EU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPD_MASK    (0x00008000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPD_SHIFT   (0x0000000FU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IN_MASK       (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IN_SHIFT      (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IE_MASK       (0x00020000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IE_SHIFT      (0x00000011U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEN_MASK     (0x00040000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEN_SHIFT    (0x00000012U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEVNT_MASK   (0x00080000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEVNT_SHIFT  (0x00000013U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RSS_CSI2A_LANE2_CFG_DY2_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG_RESETVAL                              (0x00004001U)

/* RSS_CSI2A_LANE3_CFG */

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPU_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPU_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPD_MASK    (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPD_SHIFT   (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IN_MASK       (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IN_SHIFT      (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IE_MASK       (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IE_SHIFT      (0x00000003U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOIN_MASK    (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOIN_SHIFT   (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOIN_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKIN_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKIN_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKIN_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUIN_MASK     (0x00000040U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUIN_SHIFT    (0x00000006U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUIN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKIN_MASK  (0x00000080U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKIN_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKIN_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEN_MASK     (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEN_SHIFT    (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEVNT_MASK   (0x00000200U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEVNT_SHIFT  (0x00000009U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOOUT_MASK   (0x00000400U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOOUT_SHIFT  (0x0000000AU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOOUT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKOUT_MASK (0x00000800U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKOUT_SHIFT (0x0000000BU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_ISOCLKOUT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUOUT_MASK    (0x00001000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUOUT_SHIFT   (0x0000000CU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUOUT_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKOUT_MASK (0x00002000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKOUT_SHIFT (0x0000000DU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DX3_WUCLKOUT_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPU_MASK    (0x00004000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPU_SHIFT   (0x0000000EU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPD_MASK    (0x00008000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPD_SHIFT   (0x0000000FU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IN_MASK       (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IN_SHIFT      (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IE_MASK       (0x00020000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IE_SHIFT      (0x00000011U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEN_MASK     (0x00040000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEN_SHIFT    (0x00000012U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEVNT_MASK   (0x00080000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEVNT_SHIFT  (0x00000013U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RSS_CSI2A_LANE3_CFG_DY3_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG_RESETVAL                              (0x00004001U)

/* RSS_CSI2A_LANE4_CFG */

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPU_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPU_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPD_MASK    (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPD_SHIFT   (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IN_MASK       (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IN_SHIFT      (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IE_MASK       (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IE_SHIFT      (0x00000003U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOIN_MASK    (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOIN_SHIFT   (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOIN_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKIN_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKIN_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKIN_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUIN_MASK     (0x00000040U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUIN_SHIFT    (0x00000006U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUIN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKIN_MASK  (0x00000080U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKIN_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKIN_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEN_MASK     (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEN_SHIFT    (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEVNT_MASK   (0x00000200U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEVNT_SHIFT  (0x00000009U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOOUT_MASK   (0x00000400U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOOUT_SHIFT  (0x0000000AU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOOUT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKOUT_MASK (0x00000800U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKOUT_SHIFT (0x0000000BU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_ISOCLKOUT_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUOUT_MASK    (0x00001000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUOUT_SHIFT   (0x0000000CU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUOUT_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKOUT_MASK (0x00002000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKOUT_SHIFT (0x0000000DU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKOUT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DX4_WUCLKOUT_MAX  (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPU_MASK    (0x00004000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPU_SHIFT   (0x0000000EU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPU_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPU_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPD_MASK    (0x00008000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPD_SHIFT   (0x0000000FU)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_ENBPD_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IN_MASK       (0x00010000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IN_SHIFT      (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IN_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IN_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IE_MASK       (0x00020000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IE_SHIFT      (0x00000011U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_IE_MAX        (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEN_MASK     (0x00040000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEN_SHIFT    (0x00000012U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEN_MAX      (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEVNT_MASK   (0x00080000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEVNT_SHIFT  (0x00000013U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEVNT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RSS_CSI2A_LANE4_CFG_DY4_WUEVNT_MAX    (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG_RESETVAL                              (0x00004001U)

/* RSS_CSI2A_FIFO_MEMINIT */

#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_RSS_CSI2A_FIFO_MEMINIT_START_MASK  (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_RSS_CSI2A_FIFO_MEMINIT_START_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_RSS_CSI2A_FIFO_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_RSS_CSI2A_FIFO_MEMINIT_START_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_RESETVAL                           (0x00000000U)

/* RSS_CSI2A_FIFO_MEMINIT_DONE */

#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_DONE_RSS_CSI2A_FIFO_MEMINIT_DONE_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_DONE_RSS_CSI2A_FIFO_MEMINIT_DONE_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_DONE_RSS_CSI2A_FIFO_MEMINIT_DONE_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_DONE_RSS_CSI2A_FIFO_MEMINIT_DONE_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_DONE_RESETVAL                      (0x00000000U)

/* RSS_CSI2A_FIFO_MEMINIT_STATUS */

#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_STATUS_RSS_CSI2A_FIFO_MEMINIT_STATUS_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_STATUS_RSS_CSI2A_FIFO_MEMINIT_STATUS_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_STATUS_RSS_CSI2A_FIFO_MEMINIT_STATUS_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_STATUS_RSS_CSI2A_FIFO_MEMINIT_STATUS_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_FIFO_MEMINIT_STATUS_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_CTX_MEMINIT */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_RSS_CSI2A_CTX_MEMINIT_START_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_RSS_CSI2A_CTX_MEMINIT_START_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_RSS_CSI2A_CTX_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_RSS_CSI2A_CTX_MEMINIT_START_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_RESETVAL                            (0x00000000U)

/* RSS_CSI2A_CTX_MEMINIT_DONE */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_DONE_RSS_CSI2A_CTX_MEMINIT_DONE_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_DONE_RSS_CSI2A_CTX_MEMINIT_DONE_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_DONE_RSS_CSI2A_CTX_MEMINIT_DONE_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_DONE_RSS_CSI2A_CTX_MEMINIT_DONE_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_DONE_RESETVAL                       (0x00000000U)

/* RSS_CSI2A_CTX_MEMINIT_STATUS */

#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_STATUS_RSS_CSI2A_CTX_MEMINIT_STATUS_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_STATUS_RSS_CSI2A_CTX_MEMINIT_STATUS_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_STATUS_RSS_CSI2A_CTX_MEMINIT_STATUS_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_STATUS_RSS_CSI2A_CTX_MEMINIT_STATUS_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_CTX_MEMINIT_STATUS_RESETVAL                     (0x00000000U)

/* RSS_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_CLK_DISABLE_MASK  (0x00000070U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_CLK_DISABLE_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_CLK_DISABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_CLK_DISABLE_MAX   (0x00000007U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RESETVAL                              (0x00000000U)

/* RSS_BUS_SAFETY_SEC_ERR_STAT0 */

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_CSI2A_MDMA_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_CSI2A_MDMA_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_CSI2A_MDMA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_CSI2A_MDMA_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_MST_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_MST_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_MST_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_MST_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_RD_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_RD_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_RD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_RD_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_WR_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_WR_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_WR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_TPTC_A0_WR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_PCR_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_PCR_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_PCR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_PCR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WR_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WR_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WRD_MASK (0x00000040U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WRD_SHIFT (0x00000006U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WRD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WRD_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_RD_MASK (0x00000080U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_RD_SHIFT (0x00000007U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_RD_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_RD_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_WR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_WR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_WR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_CQ_MEM_WR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_STATIC_MEM_MASK (0x00000200U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_STATIC_MEM_SHIFT (0x00000009U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_STATIC_MEM_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_STATIC_MEM_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_MBOX_MASK (0x00000400U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_MBOX_SHIFT (0x0000000AU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_MBOX_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_MBOX_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_SLV_MASK (0x00000800U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_SLV_SHIFT (0x0000000BU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_SLV_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_BSS_SLV_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2DSS_MASK (0x00001000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2DSS_SHIFT (0x0000000CU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2DSS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2DSS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_DSS2RSS_MASK (0x00002000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_DSS2RSS_SHIFT (0x0000000DU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_DSS2RSS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_DSS2RSS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_MSS2RSS_MASK (0x00004000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_MSS2RSS_SHIFT (0x0000000EU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_MSS2RSS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_MSS2RSS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2MSS_MASK (0x00008000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2MSS_SHIFT (0x0000000FU)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2MSS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS2MSS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RESETVAL                     (0x00000000U)

/* RSS_TPTCA0_RD_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_CTRL_RESETVAL                    (0x00090007U)

/* RSS_TPTCA0_RD_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RSS_TPTCA0_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* RSS_TPTCA0_RD_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RSS_TPTCA0_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* RSS_TPTCA0_WR_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_CTRL_RESETVAL                    (0x00070007U)

/* RSS_TPTCA0_WR_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RSS_TPTCA0_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* RSS_TPTCA0_WR_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RSS_TPTCA0_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_TPTCA0_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RSS_CSI2A_MDMA_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CSI2A_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* RSS_PCR_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RSS_PCR_BUS_SAFETY_CTRL_TYPE_MAX  (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_CTRL_RESETVAL                          (0x00000007U)

/* RSS_PCR_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SEC_MASK      (0x00000010U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SEC_SHIFT     (0x00000004U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SEC_RESETVAL  (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SEC_MAX       (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DED_MASK      (0x00000020U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DED_SHIFT     (0x00000005U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DED_RESETVAL  (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DED_MAX       (0x00000001U)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DATA_MASK     (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DATA_SHIFT    (0x00000008U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_DATA_MAX      (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_MAIN_MASK     (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_MAIN_SHIFT    (0x00000010U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_MAIN_MAX      (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SAFE_MASK     (0xFF000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SAFE_SHIFT    (0x00000018U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RSS_PCR_BUS_SAFETY_FI_SAFE_MAX      (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_FI_RESETVAL                            (0x00000000U)

/* RSS_PCR_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_SEC_MASK    (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_SEC_SHIFT   (0x00000010U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_SEC_MAX     (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_DED_MASK    (0xFF000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_DED_SHIFT   (0x00000018U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RSS_PCR_BUS_SAFETY_ERR_DED_MAX     (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_RESETVAL                           (0x00000000U)

/* RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL                (0x00000000U)

/* RSS_PCR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                  (0x00000000U)

/* RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL                (0x00000000U)

/* RSS_PCR_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_READ_RESETVAL                 (0x00000000U)

/* RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL            (0x00000000U)

/* RSS_ADCBUF_RD_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RESETVAL                    (0x00090007U)

/* RSS_ADCBUF_RD_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* RSS_ADCBUF_RD_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RSS_ADCBUF_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* RSS_ADCBUF_WR_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RESETVAL                    (0x00070007U)

/* RSS_ADCBUF_WR_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* RSS_ADCBUF_WR_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* RSS_CQ_MEM_RD_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000009U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_CTRL_RESETVAL                    (0x00090007U)

/* RSS_CQ_MEM_RD_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RSS_CQ_MEM_RD_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* RSS_CQ_MEM_RD_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_RD_BUS_SAFETY_ERR_STAT_READ_RESETVAL           (0x00000000U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_CTRL_RESETVAL                    (0x00070007U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RSS_CQ_MEM_WR_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_FI_RESETVAL                      (0x00000000U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_RESETVAL                     (0x00000000U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL          (0x00000000U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_CMD_RESETVAL            (0x00000000U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL          (0x00000000U)

/* RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_CQ_MEM_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL      (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RESETVAL                         (0x00000007U)

/* RSS_MBOX_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RESETVAL                           (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RESETVAL                          (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL               (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RESETVAL                 (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL               (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RESETVAL                (0x00000000U)

/* RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL           (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_CTRL_RESETVAL                   (0x00000007U)

/* RSS_STATIC_MEM_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RSS_STATIC_MEM_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_FI_RESETVAL                     (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RSS_STATIC_MEM_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_RESETVAL                    (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL         (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_CMD_RESETVAL           (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL         (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_READ_RESETVAL          (0x00000000U)

/* RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_STATIC_MEM_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL     (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_CTRL_RESETVAL                      (0x00000007U)

/* RSS_BSS_MST_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RSS_BSS_MST_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_FI_RESETVAL                        (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RSS_BSS_MST_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_RESETVAL                       (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL            (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_CMD_RESETVAL              (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL            (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_READ_RESETVAL             (0x00000000U)

/* RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_MST_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL        (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_CTRL */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_TYPE_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_TYPE_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_TYPE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_TYPE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_CTRL_RESETVAL                      (0x00000007U)

/* RSS_BSS_SLV_BUS_SAFETY_FI */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MASK (0x00000004U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_SHIFT (0x00000002U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_MAIN_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MASK (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_SHIFT (0x00000003U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_GLOBAL_SAFE_REQ_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SEC_MASK (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SEC_SHIFT (0x00000004U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SEC_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DED_MASK (0x00000020U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DED_SHIFT (0x00000005U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DED_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DATA_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DATA_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_DATA_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_MAIN_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_MAIN_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_MAIN_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SAFE_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SAFE_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RSS_BSS_SLV_BUS_SAFETY_FI_SAFE_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_FI_RESETVAL                        (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_ERR */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_ERR_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_ERR_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_ERR_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_ERR_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_CHECK_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_CHECK_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_CHECK_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_COMP_CHECK_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_SEC_MASK (0x00FF0000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_SEC_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_DED_MASK (0xFF000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RSS_BSS_SLV_BUS_SAFETY_ERR_DED_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_RESETVAL                       (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0 */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_MASK (0x000000FFU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D0_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_MASK (0x0000FF00U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_SHIFT (0x00000008U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_D1_MAX (0x000000FFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_DATA0_RESETVAL            (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_CMD_RESETVAL              (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITE_RESETVAL            (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_READ_RESETVAL             (0x00000000U)

/* RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP */

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_RSS_BSS_SLV_BUS_SAFETY_ERR_STAT_WRITERESP_RESETVAL        (0x00000000U)

/* RSS_TPTC_BOUNDARY_CFG */

#define CSL_RSS_CTRL_RSS_TPTC_BOUNDARY_CFG_RSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MASK (0x0000003FU)
#define CSL_RSS_CTRL_RSS_TPTC_BOUNDARY_CFG_RSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTC_BOUNDARY_CFG_RSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_RESETVAL (0x00000012U)
#define CSL_RSS_CTRL_RSS_TPTC_BOUNDARY_CFG_RSS_TPTC_BOUNDARY_CFG_TPTC_A0_SIZE_MAX (0x0000003FU)

#define CSL_RSS_CTRL_RSS_TPTC_BOUNDARY_CFG_RESETVAL                            (0x00000012U)

/* RSS_TPTC_XID_REORDER_CFG */

#define CSL_RSS_CTRL_RSS_TPTC_XID_REORDER_CFG_RSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_TPTC_XID_REORDER_CFG_RSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTC_XID_REORDER_CFG_RSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_TPTC_XID_REORDER_CFG_RSS_TPTC_XID_REORDER_CFG_TPTC_A0_DISABLE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_TPTC_XID_REORDER_CFG_RESETVAL                         (0x00000000U)

/* DBG_ACK_CPU_CTRL */

#define CSL_RSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_MASK                (0x00000007U)
#define CSL_RSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_SHIFT               (0x00000000U)
#define CSL_RSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_RESETVAL            (0x00000000U)
#define CSL_RSS_CTRL_DBG_ACK_CPU_CTRL_DBG_ACK_CPU_CTRL_SEL_MAX                 (0x00000007U)

#define CSL_RSS_CTRL_DBG_ACK_CPU_CTRL_RESETVAL                                 (0x00000000U)

/* RSS_ADCBUF_PING_MEMINIT */

#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_RSS_ADCBUF_PING_MEMINIT_START_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_RSS_ADCBUF_PING_MEMINIT_START_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_RSS_ADCBUF_PING_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_RSS_ADCBUF_PING_MEMINIT_START_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_RESETVAL                          (0x00000000U)

/* RSS_ADCBUF_PING_MEMINIT_DONE */

#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE_RSS_ADCBUF_PING_MEMINIT_DONE_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE_RSS_ADCBUF_PING_MEMINIT_DONE_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE_RSS_ADCBUF_PING_MEMINIT_DONE_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE_RSS_ADCBUF_PING_MEMINIT_DONE_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_DONE_RESETVAL                     (0x00000000U)

/* RSS_ADCBUF_PING_MEMINIT_STATUS */

#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_STATUS_RSS_ADCBUF_PING_MEMINIT_STATUS_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_STATUS_RSS_ADCBUF_PING_MEMINIT_STATUS_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_STATUS_RSS_ADCBUF_PING_MEMINIT_STATUS_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_STATUS_RSS_ADCBUF_PING_MEMINIT_STATUS_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_PING_MEMINIT_STATUS_RESETVAL                   (0x00000000U)

/* RSS_ADCBUF_PONG_MEMINIT */

#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_RSS_ADCBUF_PONG_MEMINIT_START_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_RSS_ADCBUF_PONG_MEMINIT_START_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_RSS_ADCBUF_PONG_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_RSS_ADCBUF_PONG_MEMINIT_START_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_RESETVAL                          (0x00000000U)

/* RSS_ADCBUF_PONG_MEMINIT_DONE */

#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE_RSS_ADCBUF_PONG_MEMINIT_DONE_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE_RSS_ADCBUF_PONG_MEMINIT_DONE_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE_RSS_ADCBUF_PONG_MEMINIT_DONE_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE_RSS_ADCBUF_PONG_MEMINIT_DONE_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_DONE_RESETVAL                     (0x00000000U)

/* RSS_ADCBUF_PONG_MEMINIT_STATUS */

#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_STATUS_RSS_ADCBUF_PONG_MEMINIT_STATUS_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_STATUS_RSS_ADCBUF_PONG_MEMINIT_STATUS_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_STATUS_RSS_ADCBUF_PONG_MEMINIT_STATUS_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_STATUS_RSS_ADCBUF_PONG_MEMINIT_STATUS_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_ADCBUF_PONG_MEMINIT_STATUS_RESETVAL                   (0x00000000U)

/* SOC_TO_BSS_SW_INT */

#define CSL_RSS_CTRL_SOC_TO_BSS_SW_INT_SOC_TO_BSS_SW_INT_TRIG_MASK             (0x000000FFU)
#define CSL_RSS_CTRL_SOC_TO_BSS_SW_INT_SOC_TO_BSS_SW_INT_TRIG_SHIFT            (0x00000000U)
#define CSL_RSS_CTRL_SOC_TO_BSS_SW_INT_SOC_TO_BSS_SW_INT_TRIG_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_SOC_TO_BSS_SW_INT_SOC_TO_BSS_SW_INT_TRIG_MAX              (0x000000FFU)

#define CSL_RSS_CTRL_SOC_TO_BSS_SW_INT_RESETVAL                                (0x00000000U)

/* RSS_DBG_ACK_CTL0 */

#define CSL_RSS_CTRL_RSS_DBG_ACK_CTL0_RSS_DBG_ACK_CTL0_FRC_MASK                (0x00000007U)
#define CSL_RSS_CTRL_RSS_DBG_ACK_CTL0_RSS_DBG_ACK_CTL0_FRC_SHIFT               (0x00000000U)
#define CSL_RSS_CTRL_RSS_DBG_ACK_CTL0_RSS_DBG_ACK_CTL0_FRC_RESETVAL            (0x00000000U)
#define CSL_RSS_CTRL_RSS_DBG_ACK_CTL0_RSS_DBG_ACK_CTL0_FRC_MAX                 (0x00000007U)

#define CSL_RSS_CTRL_RSS_DBG_ACK_CTL0_RESETVAL                                 (0x00000000U)

/* DMMSWINT1 */

#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFPINPONSEL_MASK               (0x00010000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFPINPONSEL_SHIFT              (0x00000010U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFPINPONSEL_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFPINPONSEL_MAX                (0x00000001U)

#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_MASK                    (0x00020000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_SHIFT                   (0x00000011U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_MAX                     (0x00000001U)

#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCPWREN_MASK                        (0x00040000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCPWREN_SHIFT                       (0x00000012U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCPWREN_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCPWREN_MAX                         (0x00000001U)

#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQPINPONSEL_MASK                   (0x00200000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQPINPONSEL_SHIFT                  (0x00000015U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQPINPONSEL_RESETVAL               (0x00000000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQPINPONSEL_MAX                    (0x00000001U)

#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQWREN_MASK                        (0x00400000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQWREN_SHIFT                       (0x00000016U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQWREN_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMCQWREN_MAX                         (0x00000001U)

#define CSL_RSS_CTRL_DMMSWINT1_RESETVAL                                        (0x00000000U)

/* RSS_SHARED_MEM_MEMINIT */

#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_RSS_SHARED_MEM_MEMINIT_START_MASK  (0x00000001U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_RSS_SHARED_MEM_MEMINIT_START_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_RSS_SHARED_MEM_MEMINIT_START_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_RSS_SHARED_MEM_MEMINIT_START_MAX   (0x00000001U)

#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_RESETVAL                           (0x00000000U)

/* RSS_SHARED_MEM_MEMINIT_DONE */

#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_DONE_RSS_SHARED_MEM_MEMINIT_DONE_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_DONE_RSS_SHARED_MEM_MEMINIT_DONE_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_DONE_RSS_SHARED_MEM_MEMINIT_DONE_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_DONE_RSS_SHARED_MEM_MEMINIT_DONE_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_DONE_RESETVAL                      (0x00000000U)

/* RSS_SHARED_MEM_MEMINIT_STATUS */

#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_STATUS_RSS_SHARED_MEM_MEMINIT_STATUS_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_STATUS_RSS_SHARED_MEM_MEMINIT_STATUS_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_STATUS_RSS_SHARED_MEM_MEMINIT_STATUS_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_STATUS_RSS_SHARED_MEM_MEMINIT_STATUS_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_RSS_SHARED_MEM_MEMINIT_STATUS_RESETVAL                    (0x00000000U)

/* RSS_CSI_ACCESS_MODE */

#define CSL_RSS_CTRL_RSS_CSI_ACCESS_MODE_RSS_CSI_ACCESS_MODE_CSI2A_SEL_MASK    (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI_ACCESS_MODE_RSS_CSI_ACCESS_MODE_CSI2A_SEL_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_RSS_CSI_ACCESS_MODE_RSS_CSI_ACCESS_MODE_CSI2A_SEL_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_RSS_CSI_ACCESS_MODE_RSS_CSI_ACCESS_MODE_CSI2A_SEL_MAX     (0x00000001U)

#define CSL_RSS_CTRL_RSS_CSI_ACCESS_MODE_RESETVAL                              (0x00000001U)

/* BSS_CONTROL */

#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_BOOTMODE_MASK                     (0x00000FFFU)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_BOOTMODE_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_BOOTMODE_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_BOOTMODE_MAX                      (0x00000FFFU)

#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_WFI_OVERRIDE_MASK                 (0x00070000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_WFI_OVERRIDE_SHIFT                (0x00000010U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_WFI_OVERRIDE_RESETVAL             (0x00000000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_WFI_OVERRIDE_MAX                  (0x00000007U)

#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_HALT_MASK                         (0x07000000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_HALT_SHIFT                        (0x00000018U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_HALT_RESETVAL                     (0x00000007U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_HALT_MAX                          (0x00000007U)

#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_DSS_L3_ACCESS_MASK                (0x70000000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_DSS_L3_ACCESS_SHIFT               (0x0000001CU)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_DSS_L3_ACCESS_RESETVAL            (0x00000000U)
#define CSL_RSS_CTRL_BSS_CONTROL_BSS_CONTROL_DSS_L3_ACCESS_MAX                 (0x00000007U)

#define CSL_RSS_CTRL_BSS_CONTROL_RESETVAL                                      (0x07000000U)

/* BSS_TCM_MEMINIT */

#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_BSS_TCM_MEMINIT_MEM0_INIT_MASK            (0x00000001U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_BSS_TCM_MEMINIT_MEM0_INIT_SHIFT           (0x00000000U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_BSS_TCM_MEMINIT_MEM0_INIT_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_BSS_TCM_MEMINIT_MEM0_INIT_MAX             (0x00000001U)

#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_RESETVAL                                  (0x00000000U)

/* BSS_TCM_MEMINIT_DONE */

#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_DONE_BSS_TCM_MEMINIT_DONE_MEM0_DONE_MASK  (0x00000001U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_DONE_BSS_TCM_MEMINIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_DONE_BSS_TCM_MEMINIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_DONE_BSS_TCM_MEMINIT_DONE_MEM0_DONE_MAX   (0x00000001U)

#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_DONE_RESETVAL                             (0x00000000U)

/* BSS_TCM_MEMINIT_STATUS */

#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_STATUS_BSS_TCM_MEMINIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_STATUS_BSS_TCM_MEMINIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_STATUS_BSS_TCM_MEMINIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_STATUS_BSS_TCM_MEMINIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_BSS_TCM_MEMINIT_STATUS_RESETVAL                           (0x00000000U)

/* BSS_VIM_MEMINIT */

#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_BSS_VIM_MEMINIT_MEM0_INIT_MASK            (0x00000001U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_BSS_VIM_MEMINIT_MEM0_INIT_SHIFT           (0x00000000U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_BSS_VIM_MEMINIT_MEM0_INIT_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_BSS_VIM_MEMINIT_MEM0_INIT_MAX             (0x00000001U)

#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_RESETVAL                                  (0x00000000U)

/* BSS_VIM_MEMINIT_DONE */

#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_DONE_BSS_VIM_MEMINIT_DONE_MEM0_DONE_MASK  (0x00000001U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_DONE_BSS_VIM_MEMINIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_DONE_BSS_VIM_MEMINIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_DONE_BSS_VIM_MEMINIT_DONE_MEM0_DONE_MAX   (0x00000001U)

#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_DONE_RESETVAL                             (0x00000000U)

/* BSS_VIM_MEMINIT_STATUS */

#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_STATUS_BSS_VIM_MEMINIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_STATUS_BSS_VIM_MEMINIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_STATUS_BSS_VIM_MEMINIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_STATUS_BSS_VIM_MEMINIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_BSS_VIM_MEMINIT_STATUS_RESETVAL                           (0x00000000U)

/* BSS_DFE_MEMINIT */

#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_BSS_DFE_MEMINIT_MEM0_INIT_MASK            (0x00000001U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_BSS_DFE_MEMINIT_MEM0_INIT_SHIFT           (0x00000000U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_BSS_DFE_MEMINIT_MEM0_INIT_RESETVAL        (0x00000000U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_BSS_DFE_MEMINIT_MEM0_INIT_MAX             (0x00000001U)

#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_RESETVAL                                  (0x00000000U)

/* BSS_DFE_MEMINIT_DONE */

#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_DONE_BSS_DFE_MEMINIT_DONE_MEM0_DONE_MASK  (0x00000001U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_DONE_BSS_DFE_MEMINIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_DONE_BSS_DFE_MEMINIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_DONE_BSS_DFE_MEMINIT_DONE_MEM0_DONE_MAX   (0x00000001U)

#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_DONE_RESETVAL                             (0x00000000U)

/* BSS_DFE_MEMINIT_STATUS */

#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_STATUS_BSS_DFE_MEMINIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_STATUS_BSS_DFE_MEMINIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_STATUS_BSS_DFE_MEMINIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_STATUS_BSS_DFE_MEMINIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_BSS_DFE_MEMINIT_STATUS_RESETVAL                           (0x00000000U)

/* BSS_RAMPGEN_MEMINIT */

#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_BSS_RAMPGEN_MEMINIT_MEM0_INIT_MASK    (0x00000001U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_BSS_RAMPGEN_MEMINIT_MEM0_INIT_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_BSS_RAMPGEN_MEMINIT_MEM0_INIT_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_BSS_RAMPGEN_MEMINIT_MEM0_INIT_MAX     (0x00000001U)

#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_RESETVAL                              (0x00000000U)

/* BSS_RAMPGEN_MEMINIT_DONE */

#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_DONE_BSS_RAMPGEN_MEMINIT_DONE_MEM0_DONE_MASK (0x00000001U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_DONE_BSS_RAMPGEN_MEMINIT_DONE_MEM0_DONE_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_DONE_BSS_RAMPGEN_MEMINIT_DONE_MEM0_DONE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_DONE_BSS_RAMPGEN_MEMINIT_DONE_MEM0_DONE_MAX (0x00000001U)

#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_DONE_RESETVAL                         (0x00000000U)

/* BSS_RAMPGEN_MEMINIT_STATUS */

#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_STATUS_BSS_RAMPGEN_MEMINIT_STATUS_MEM0_STATUS_MASK (0x00000001U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_STATUS_BSS_RAMPGEN_MEMINIT_STATUS_MEM0_STATUS_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_STATUS_BSS_RAMPGEN_MEMINIT_STATUS_MEM0_STATUS_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_STATUS_BSS_RAMPGEN_MEMINIT_STATUS_MEM0_STATUS_MAX (0x00000001U)

#define CSL_RSS_CTRL_BSS_RAMPGEN_MEMINIT_STATUS_RESETVAL                       (0x00000000U)

/* BSS_DSS_L3_STICKY */

#define CSL_RSS_CTRL_BSS_DSS_L3_STICKY_BSS_DSS_L3_STICKY_STICKY_ENABLE_MASK    (0x00000007U)
#define CSL_RSS_CTRL_BSS_DSS_L3_STICKY_BSS_DSS_L3_STICKY_STICKY_ENABLE_SHIFT   (0x00000000U)
#define CSL_RSS_CTRL_BSS_DSS_L3_STICKY_BSS_DSS_L3_STICKY_STICKY_ENABLE_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_BSS_DSS_L3_STICKY_BSS_DSS_L3_STICKY_STICKY_ENABLE_MAX     (0x00000007U)

#define CSL_RSS_CTRL_BSS_DSS_L3_STICKY_RESETVAL                                (0x00000000U)

/* BSS_DSS_L3_ACCESS */

#define CSL_RSS_CTRL_BSS_DSS_L3_ACCESS_BSS_DSS_L3_ACCESS_STATUS_MASK           (0x00000001U)
#define CSL_RSS_CTRL_BSS_DSS_L3_ACCESS_BSS_DSS_L3_ACCESS_STATUS_SHIFT          (0x00000000U)
#define CSL_RSS_CTRL_BSS_DSS_L3_ACCESS_BSS_DSS_L3_ACCESS_STATUS_RESETVAL       (0x00000000U)
#define CSL_RSS_CTRL_BSS_DSS_L3_ACCESS_BSS_DSS_L3_ACCESS_STATUS_MAX            (0x00000001U)

#define CSL_RSS_CTRL_BSS_DSS_L3_ACCESS_RESETVAL                                (0x00000000U)

/* TESTPATTERNRX1ICFG */

#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_TESTPATTERNRX1ICFG_TSTPATRX1IINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX1ICFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX2ICFG */

#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_TESTPATTERNRX2ICFG_TSTPATRX2IINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX2ICFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX3ICFG */

#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_TESTPATTERNRX3ICFG_TSTPATRX3IINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX3ICFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX4ICFG */

#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_TESTPATTERNRX4ICFG_TSTPATRX4IINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX4ICFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX1QCFG */

#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_TESTPATTERNRX1QCFG_TSTPATRX1QINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX1QCFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX2QCFG */

#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_TESTPATTERNRX2QCFG_TSTPATRX2QINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX2QCFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX3QCFG */

#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_TESTPATTERNRX3QCFG_TSTPATRX3QINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX3QCFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNRX4QCFG */

#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QOFFSET_MASK (0x0000FFFFU)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QOFFSET_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QOFFSET_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QOFFSET_MAX (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QINCR_MASK (0xFFFF0000U)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QINCR_SHIFT (0x00000010U)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QINCR_RESETVAL (0x00000001U)
#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_TESTPATTERNRX4QCFG_TSTPATRX4QINCR_MAX  (0x0000FFFFU)

#define CSL_RSS_CTRL_TESTPATTERNRX4QCFG_RESETVAL                               (0x00010000U)

/* TESTPATTERNVLDCFG */

#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_MASK     (0x000000FFU)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_SHIFT    (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_RESETVAL (0x00000008U)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATVLDCNT_MAX      (0x000000FFU)

#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_MASK      (0x00000700U)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_SHIFT     (0x00000008U)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_RESETVAL  (0x00000000U)
#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_TESTPATTERNVLDCFG_TSTPATGENEN_MAX       (0x00000007U)

#define CSL_RSS_CTRL_TESTPATTERNVLDCFG_RESETVAL                                (0x00000008U)

/* ADCBUFCFG1 */

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRSOURCE_MASK                 (0x00000001U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRSOURCE_SHIFT                (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRSOURCE_RESETVAL             (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRSOURCE_MAX                  (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSELINV_MASK               (0x00000002U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSELINV_SHIFT              (0x00000001U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSELINV_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSELINV_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_MASK             (0x00000004U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_SHIFT            (0x00000002U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_RESETVAL         (0x00000001U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFREALONLYMODE_MAX              (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRLMODECHSEL_MASK              (0x00000008U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRLMODECHSEL_SHIFT             (0x00000003U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRLMODECHSEL_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRLMODECHSEL_MAX               (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRL2CHINTRL_MASK               (0x00000010U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRL2CHINTRL_SHIFT              (0x00000004U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRL2CHINTRL_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFRL2CHINTRL_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP_MASK                   (0x00000020U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP_SHIFT                  (0x00000005U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP_RESETVAL               (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFIQSWAP_MAX                    (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_MASK                          (0x00000040U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_SHIFT                         (0x00000006U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_MASK                          (0x00000080U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_SHIFT                         (0x00000007U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_MASK                          (0x00000100U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_SHIFT                         (0x00000008U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_MASK                          (0x00000200U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_SHIFT                         (0x00000009U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX3EN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRCNT_MASK               (0x00000400U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRCNT_SHIFT              (0x0000000AU)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRCNT_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRCNT_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRVAL_MASK               (0x00000800U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRVAL_SHIFT              (0x0000000BU)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRVAL_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOOVRVAL_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE_MASK                (0x00001000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE_SHIFT               (0x0000000CU)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE_RESETVAL            (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFWRITEMODE_MAX                 (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN_MASK               (0x00002000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN_SHIFT              (0x0000000DU)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTMODEEN_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTRTPL_MASK               (0x00004000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTRTPL_SHIFT              (0x0000000EU)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTRTPL_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTRTPL_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTOPPL_MASK               (0x00008000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTOPPL_SHIFT              (0x0000000FU)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTOPPL_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFCONTSTOPPL_MAX                (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSEL_MASK                  (0x00010000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSEL_SHIFT                 (0x00000010U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSEL_RESETVAL              (0x00000001U)
#define CSL_RSS_CTRL_ADCBUFCFG1_ADCBUFCFG1_ADCBUFPIPOSEL_MAX                   (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG1_RESETVAL                                       (0x00010004U)

/* ADCBUFCFG1_EXTD */

#define CSL_RSS_CTRL_ADCBUFCFG1_EXTD_ADCBUFCFG1_EXTD_ADCBUFINTGENDLY_MASK      (0xFFFFFFFFU)
#define CSL_RSS_CTRL_ADCBUFCFG1_EXTD_ADCBUFCFG1_EXTD_ADCBUFINTGENDLY_SHIFT     (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_EXTD_ADCBUFCFG1_EXTD_ADCBUFINTGENDLY_RESETVAL  (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG1_EXTD_ADCBUFCFG1_EXTD_ADCBUFINTGENDLY_MAX       (0xFFFFFFFFU)

#define CSL_RSS_CTRL_ADCBUFCFG1_EXTD_RESETVAL                                  (0x00000000U)

/* ADCBUFCFG2 */

#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_MASK                   (0x000007FFU)
#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_SHIFT                  (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_RESETVAL               (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0_MAX                    (0x000007FFU)

#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_MASK                   (0x07FF0000U)
#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_SHIFT                  (0x00000010U)
#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_RESETVAL               (0x00000100U)
#define CSL_RSS_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1_MAX                    (0x000007FFU)

#define CSL_RSS_CTRL_ADCBUFCFG2_RESETVAL                                       (0x01000000U)

/* ADCBUFCFG3 */

#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_MASK                   (0x000007FFU)
#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_SHIFT                  (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_RESETVAL               (0x00000200U)
#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2_MAX                    (0x000007FFU)

#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_MASK                   (0x07FF0000U)
#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_SHIFT                  (0x00000010U)
#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_RESETVAL               (0x00000300U)
#define CSL_RSS_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX3_MAX                    (0x000007FFU)

#define CSL_RSS_CTRL_ADCBUFCFG3_RESETVAL                                       (0x03000200U)

/* ADCBUFCFG4 */

#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_MASK                  (0x0000FFFFU)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_SHIFT                 (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_RESETVAL              (0x00000400U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFSAMPCNT_MAX                   (0x0000FFFFU)

#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING_MASK              (0x001F0000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING_SHIFT             (0x00000010U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPING_MAX               (0x0000001FU)

#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG_MASK              (0x03E00000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG_SHIFT             (0x00000015U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFNUMCHRPPONG_MAX               (0x0000001FU)

#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFPNGSELTGLDIS_MASK             (0x40000000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFPNGSELTGLDIS_SHIFT            (0x0000001EU)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFPNGSELTGLDIS_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFCFG4_ADCBUFCFG4_ADCBUFPNGSELTGLDIS_MAX              (0x00000001U)

#define CSL_RSS_CTRL_ADCBUFCFG4_RESETVAL                                       (0x00000400U)

/* ADCBUFINTGENDITHERDLY */

#define CSL_RSS_CTRL_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_MASK (0xFFFFFFFFU)
#define CSL_RSS_CTRL_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_SHIFT (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_ADCBUFINTGENDITHERDLY_MAX (0xFFFFFFFFU)

#define CSL_RSS_CTRL_ADCBUFINTGENDITHERDLY_RESETVAL                            (0x00000000U)

/* CBUFF_FRAME_START_SEL */

#define CSL_RSS_CTRL_CBUFF_FRAME_START_SEL_CBUFF_FRAME_START_SEL_SEL_MASK      (0x00000001U)
#define CSL_RSS_CTRL_CBUFF_FRAME_START_SEL_CBUFF_FRAME_START_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_RSS_CTRL_CBUFF_FRAME_START_SEL_CBUFF_FRAME_START_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_RSS_CTRL_CBUFF_FRAME_START_SEL_CBUFF_FRAME_START_SEL_SEL_MAX       (0x00000001U)

#define CSL_RSS_CTRL_CBUFF_FRAME_START_SEL_RESETVAL                            (0x00000000U)

/* CQCFG1 */

#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQDATAWIDTH_MASK                            (0x00000003U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQDATAWIDTH_SHIFT                           (0x00000000U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQDATAWIDTH_RESETVAL                        (0x00000000U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQDATAWIDTH_MAX                             (0x00000003U)

#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ96BITPACKEN_MASK                          (0x00000008U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ96BITPACKEN_SHIFT                         (0x00000003U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ96BITPACKEN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ96BITPACKEN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_MASK                            (0x00001FF0U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_SHIFT                           (0x00000004U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_RESETVAL                        (0x00000000U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ0BASEADDR_MAX                             (0x000001FFU)

#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_MASK                            (0x003FE000U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_SHIFT                           (0x0000000DU)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_RESETVAL                        (0x00000080U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ1BASEADDR_MAX                             (0x000001FFU)

#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_MASK                            (0x7FC00000U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_SHIFT                           (0x00000016U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_RESETVAL                        (0x00000100U)
#define CSL_RSS_CTRL_CQCFG1_CQCFG1_CQ2BASEADDR_MAX                             (0x000001FFU)

#define CSL_RSS_CTRL_CQCFG1_RESETVAL                                           (0x40100000U)

/* CQCFG2 */

#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ0TESTMODEEN_MASK                          (0x00000001U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ0TESTMODEEN_SHIFT                         (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ0TESTMODEEN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ0TESTMODEEN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ1TESTMODEEN_MASK                          (0x00000010U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ1TESTMODEEN_SHIFT                         (0x00000004U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ1TESTMODEEN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ1TESTMODEEN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ2TESTMODEEN_MASK                          (0x00000100U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ2TESTMODEEN_SHIFT                         (0x00000008U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ2TESTMODEEN_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ2TESTMODEEN_MAX                           (0x00000001U)

#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELCNT_MASK                           (0x00001000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELCNT_SHIFT                          (0x0000000CU)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELCNT_RESETVAL                       (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELCNT_MAX                            (0x00000001U)

#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELVAL_MASK                           (0x00010000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELVAL_SHIFT                          (0x00000010U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELVAL_RESETVAL                       (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQPIPOSELVAL_MAX                            (0x00000001U)

#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ_CLK_GATE_MASK                            (0x00020000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ_CLK_GATE_SHIFT                           (0x00000011U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ_CLK_GATE_RESETVAL                        (0x00000000U)
#define CSL_RSS_CTRL_CQCFG2_CQCFG2_CQ_CLK_GATE_MAX                             (0x00000001U)

#define CSL_RSS_CTRL_CQCFG2_RESETVAL                                           (0x00000000U)

/* CPREG0 */

#define CSL_RSS_CTRL_CPREG0_CPREG0_CPREG0_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG0_CPREG0_CPREG0_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG0_CPREG0_CPREG0_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG0_CPREG0_CPREG0_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG0_RESETVAL                                           (0x00000000U)

/* CPREG1 */

#define CSL_RSS_CTRL_CPREG1_CPREG1_CPREG1_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG1_CPREG1_CPREG1_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG1_CPREG1_CPREG1_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG1_CPREG1_CPREG1_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG1_RESETVAL                                           (0x00000000U)

/* CPREG2 */

#define CSL_RSS_CTRL_CPREG2_CPREG2_CPREG2_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG2_CPREG2_CPREG2_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG2_CPREG2_CPREG2_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG2_CPREG2_CPREG2_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG2_RESETVAL                                           (0x00000000U)

/* CPREG3 */

#define CSL_RSS_CTRL_CPREG3_CPREG3_CPREG3_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG3_CPREG3_CPREG3_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG3_CPREG3_CPREG3_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG3_CPREG3_CPREG3_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG3_RESETVAL                                           (0x00000000U)

/* CPREG4 */

#define CSL_RSS_CTRL_CPREG4_CPREG4_CPREG4_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG4_CPREG4_CPREG4_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG4_CPREG4_CPREG4_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG4_CPREG4_CPREG4_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG4_RESETVAL                                           (0x00000000U)

/* CPREG5 */

#define CSL_RSS_CTRL_CPREG5_CPREG5_CPREG5_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG5_CPREG5_CPREG5_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG5_CPREG5_CPREG5_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG5_CPREG5_CPREG5_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG5_RESETVAL                                           (0x00000000U)

/* CPREG6 */

#define CSL_RSS_CTRL_CPREG6_CPREG6_CPREG6_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG6_CPREG6_CPREG6_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG6_CPREG6_CPREG6_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG6_CPREG6_CPREG6_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG6_RESETVAL                                           (0x00000000U)

/* CPREG7 */

#define CSL_RSS_CTRL_CPREG7_CPREG7_CPREG7_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG7_CPREG7_CPREG7_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG7_CPREG7_CPREG7_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG7_CPREG7_CPREG7_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG7_RESETVAL                                           (0x00000000U)

/* CPREG8 */

#define CSL_RSS_CTRL_CPREG8_CPREG8_CPREG8_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG8_CPREG8_CPREG8_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG8_CPREG8_CPREG8_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG8_CPREG8_CPREG8_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG8_RESETVAL                                           (0x00000000U)

/* CPREG9 */

#define CSL_RSS_CTRL_CPREG9_CPREG9_CPREG9_MASK                                 (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG9_CPREG9_CPREG9_SHIFT                                (0x00000000U)
#define CSL_RSS_CTRL_CPREG9_CPREG9_CPREG9_RESETVAL                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG9_CPREG9_CPREG9_MAX                                  (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG9_RESETVAL                                           (0x00000000U)

/* CPREG10 */

#define CSL_RSS_CTRL_CPREG10_CPREG10_CPREG10_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG10_CPREG10_CPREG10_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG10_CPREG10_CPREG10_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_CPREG10_CPREG10_CPREG10_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG10_RESETVAL                                          (0x00000000U)

/* CPREG11 */

#define CSL_RSS_CTRL_CPREG11_CPREG11_CPREG11_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG11_CPREG11_CPREG11_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG11_CPREG11_CPREG11_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_CPREG11_CPREG11_CPREG11_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG11_RESETVAL                                          (0x00000000U)

/* CPREG12 */

#define CSL_RSS_CTRL_CPREG12_CPREG12_CPREG12_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG12_CPREG12_CPREG12_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG12_CPREG12_CPREG12_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_CPREG12_CPREG12_CPREG12_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG12_RESETVAL                                          (0x00000000U)

/* CPREG13 */

#define CSL_RSS_CTRL_CPREG13_CPREG13_CPREG13_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG13_CPREG13_CPREG13_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG13_CPREG13_CPREG13_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_CPREG13_CPREG13_CPREG13_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG13_RESETVAL                                          (0x00000000U)

/* CPREG14 */

#define CSL_RSS_CTRL_CPREG14_CPREG14_CPREG14_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG14_CPREG14_CPREG14_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG14_CPREG14_CPREG14_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_CPREG14_CPREG14_CPREG14_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG14_RESETVAL                                          (0x00000000U)

/* CPREG15 */

#define CSL_RSS_CTRL_CPREG15_CPREG15_CPREG15_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CPREG15_CPREG15_CPREG15_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_CPREG15_CPREG15_CPREG15_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_CPREG15_CPREG15_CPREG15_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CPREG15_RESETVAL                                          (0x00000000U)

/* CH0CPREG0 */

#define CSL_RSS_CTRL_CH0CPREG0_CH0CPREG0_CH0CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG0_CH0CPREG0_CH0CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG0_CH0CPREG0_CH0CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG0_CH0CPREG0_CH0CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG0_RESETVAL                                        (0x00000000U)

/* CH0CPREG1 */

#define CSL_RSS_CTRL_CH0CPREG1_CH0CPREG1_CH0CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG1_CH0CPREG1_CH0CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG1_CH0CPREG1_CH0CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG1_CH0CPREG1_CH0CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG1_RESETVAL                                        (0x00000000U)

/* CH0CPREG2 */

#define CSL_RSS_CTRL_CH0CPREG2_CH0CPREG2_CH0CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG2_CH0CPREG2_CH0CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG2_CH0CPREG2_CH0CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG2_CH0CPREG2_CH0CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG2_RESETVAL                                        (0x00000000U)

/* CH0CPREG3 */

#define CSL_RSS_CTRL_CH0CPREG3_CH0CPREG3_CH0CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG3_CH0CPREG3_CH0CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG3_CH0CPREG3_CH0CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG3_CH0CPREG3_CH0CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG3_RESETVAL                                        (0x00000000U)

/* CH0CPREG4 */

#define CSL_RSS_CTRL_CH0CPREG4_CH0CPREG4_CH0CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG4_CH0CPREG4_CH0CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG4_CH0CPREG4_CH0CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG4_CH0CPREG4_CH0CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG4_RESETVAL                                        (0x00000000U)

/* CH0CPREG5 */

#define CSL_RSS_CTRL_CH0CPREG5_CH0CPREG5_CH0CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG5_CH0CPREG5_CH0CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG5_CH0CPREG5_CH0CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG5_CH0CPREG5_CH0CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG5_RESETVAL                                        (0x00000000U)

/* CH0CPREG6 */

#define CSL_RSS_CTRL_CH0CPREG6_CH0CPREG6_CH0CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG6_CH0CPREG6_CH0CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG6_CH0CPREG6_CH0CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG6_CH0CPREG6_CH0CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG6_RESETVAL                                        (0x00000000U)

/* CH0CPREG7 */

#define CSL_RSS_CTRL_CH0CPREG7_CH0CPREG7_CH0CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG7_CH0CPREG7_CH0CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG7_CH0CPREG7_CH0CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG7_CH0CPREG7_CH0CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG7_RESETVAL                                        (0x00000000U)

/* CH0CPREG8 */

#define CSL_RSS_CTRL_CH0CPREG8_CH0CPREG8_CH0CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG8_CH0CPREG8_CH0CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG8_CH0CPREG8_CH0CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG8_CH0CPREG8_CH0CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG8_RESETVAL                                        (0x00000000U)

/* CH0CPREG9 */

#define CSL_RSS_CTRL_CH0CPREG9_CH0CPREG9_CH0CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG9_CH0CPREG9_CH0CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG9_CH0CPREG9_CH0CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG9_CH0CPREG9_CH0CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG9_RESETVAL                                        (0x00000000U)

/* CH0CPREG10 */

#define CSL_RSS_CTRL_CH0CPREG10_CH0CPREG10_CH0CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG10_CH0CPREG10_CH0CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG10_CH0CPREG10_CH0CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG10_CH0CPREG10_CH0CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG10_RESETVAL                                       (0x00000000U)

/* CH0CPREG11 */

#define CSL_RSS_CTRL_CH0CPREG11_CH0CPREG11_CH0CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG11_CH0CPREG11_CH0CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG11_CH0CPREG11_CH0CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG11_CH0CPREG11_CH0CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG11_RESETVAL                                       (0x00000000U)

/* CH0CPREG12 */

#define CSL_RSS_CTRL_CH0CPREG12_CH0CPREG12_CH0CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG12_CH0CPREG12_CH0CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG12_CH0CPREG12_CH0CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG12_CH0CPREG12_CH0CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG12_RESETVAL                                       (0x00000000U)

/* CH0CPREG13 */

#define CSL_RSS_CTRL_CH0CPREG13_CH0CPREG13_CH0CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG13_CH0CPREG13_CH0CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG13_CH0CPREG13_CH0CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG13_CH0CPREG13_CH0CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG13_RESETVAL                                       (0x00000000U)

/* CH0CPREG14 */

#define CSL_RSS_CTRL_CH0CPREG14_CH0CPREG14_CH0CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG14_CH0CPREG14_CH0CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG14_CH0CPREG14_CH0CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG14_CH0CPREG14_CH0CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG14_RESETVAL                                       (0x00000000U)

/* CH0CPREG15 */

#define CSL_RSS_CTRL_CH0CPREG15_CH0CPREG15_CH0CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH0CPREG15_CH0CPREG15_CH0CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG15_CH0CPREG15_CH0CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH0CPREG15_CH0CPREG15_CH0CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH0CPREG15_RESETVAL                                       (0x00000000U)

/* CH1CPREG0 */

#define CSL_RSS_CTRL_CH1CPREG0_CH1CPREG0_CH1CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG0_CH1CPREG0_CH1CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG0_CH1CPREG0_CH1CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG0_CH1CPREG0_CH1CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG0_RESETVAL                                        (0x00000000U)

/* CH1CPREG1 */

#define CSL_RSS_CTRL_CH1CPREG1_CH1CPREG1_CH1CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG1_CH1CPREG1_CH1CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG1_CH1CPREG1_CH1CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG1_CH1CPREG1_CH1CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG1_RESETVAL                                        (0x00000000U)

/* CH1CPREG2 */

#define CSL_RSS_CTRL_CH1CPREG2_CH1CPREG2_CH1CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG2_CH1CPREG2_CH1CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG2_CH1CPREG2_CH1CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG2_CH1CPREG2_CH1CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG2_RESETVAL                                        (0x00000000U)

/* CH1CPREG3 */

#define CSL_RSS_CTRL_CH1CPREG3_CH1CPREG3_CH1CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG3_CH1CPREG3_CH1CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG3_CH1CPREG3_CH1CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG3_CH1CPREG3_CH1CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG3_RESETVAL                                        (0x00000000U)

/* CH1CPREG4 */

#define CSL_RSS_CTRL_CH1CPREG4_CH1CPREG4_CH1CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG4_CH1CPREG4_CH1CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG4_CH1CPREG4_CH1CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG4_CH1CPREG4_CH1CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG4_RESETVAL                                        (0x00000000U)

/* CH1CPREG5 */

#define CSL_RSS_CTRL_CH1CPREG5_CH1CPREG5_CH1CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG5_CH1CPREG5_CH1CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG5_CH1CPREG5_CH1CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG5_CH1CPREG5_CH1CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG5_RESETVAL                                        (0x00000000U)

/* CH1CPREG6 */

#define CSL_RSS_CTRL_CH1CPREG6_CH1CPREG6_CH1CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG6_CH1CPREG6_CH1CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG6_CH1CPREG6_CH1CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG6_CH1CPREG6_CH1CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG6_RESETVAL                                        (0x00000000U)

/* CH1CPREG7 */

#define CSL_RSS_CTRL_CH1CPREG7_CH1CPREG7_CH1CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG7_CH1CPREG7_CH1CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG7_CH1CPREG7_CH1CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG7_CH1CPREG7_CH1CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG7_RESETVAL                                        (0x00000000U)

/* CH1CPREG8 */

#define CSL_RSS_CTRL_CH1CPREG8_CH1CPREG8_CH1CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG8_CH1CPREG8_CH1CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG8_CH1CPREG8_CH1CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG8_CH1CPREG8_CH1CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG8_RESETVAL                                        (0x00000000U)

/* CH1CPREG9 */

#define CSL_RSS_CTRL_CH1CPREG9_CH1CPREG9_CH1CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG9_CH1CPREG9_CH1CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG9_CH1CPREG9_CH1CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG9_CH1CPREG9_CH1CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG9_RESETVAL                                        (0x00000000U)

/* CH1CPREG10 */

#define CSL_RSS_CTRL_CH1CPREG10_CH1CPREG10_CH1CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG10_CH1CPREG10_CH1CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG10_CH1CPREG10_CH1CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG10_CH1CPREG10_CH1CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG10_RESETVAL                                       (0x00000000U)

/* CH1CPREG11 */

#define CSL_RSS_CTRL_CH1CPREG11_CH1CPREG11_CH1CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG11_CH1CPREG11_CH1CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG11_CH1CPREG11_CH1CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG11_CH1CPREG11_CH1CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG11_RESETVAL                                       (0x00000000U)

/* CH1CPREG12 */

#define CSL_RSS_CTRL_CH1CPREG12_CH1CPREG12_CH1CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG12_CH1CPREG12_CH1CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG12_CH1CPREG12_CH1CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG12_CH1CPREG12_CH1CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG12_RESETVAL                                       (0x00000000U)

/* CH1CPREG13 */

#define CSL_RSS_CTRL_CH1CPREG13_CH1CPREG13_CH1CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG13_CH1CPREG13_CH1CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG13_CH1CPREG13_CH1CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG13_CH1CPREG13_CH1CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG13_RESETVAL                                       (0x00000000U)

/* CH1CPREG14 */

#define CSL_RSS_CTRL_CH1CPREG14_CH1CPREG14_CH1CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG14_CH1CPREG14_CH1CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG14_CH1CPREG14_CH1CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG14_CH1CPREG14_CH1CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG14_RESETVAL                                       (0x00000000U)

/* CH1CPREG15 */

#define CSL_RSS_CTRL_CH1CPREG15_CH1CPREG15_CH1CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH1CPREG15_CH1CPREG15_CH1CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG15_CH1CPREG15_CH1CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH1CPREG15_CH1CPREG15_CH1CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH1CPREG15_RESETVAL                                       (0x00000000U)

/* CH2CPREG0 */

#define CSL_RSS_CTRL_CH2CPREG0_CH2CPREG0_CH2CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG0_CH2CPREG0_CH2CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG0_CH2CPREG0_CH2CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG0_CH2CPREG0_CH2CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG0_RESETVAL                                        (0x00000000U)

/* CH2CPREG1 */

#define CSL_RSS_CTRL_CH2CPREG1_CH2CPREG1_CH2CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG1_CH2CPREG1_CH2CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG1_CH2CPREG1_CH2CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG1_CH2CPREG1_CH2CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG1_RESETVAL                                        (0x00000000U)

/* CH2CPREG2 */

#define CSL_RSS_CTRL_CH2CPREG2_CH2CPREG2_CH2CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG2_CH2CPREG2_CH2CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG2_CH2CPREG2_CH2CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG2_CH2CPREG2_CH2CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG2_RESETVAL                                        (0x00000000U)

/* CH2CPREG3 */

#define CSL_RSS_CTRL_CH2CPREG3_CH2CPREG3_CH2CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG3_CH2CPREG3_CH2CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG3_CH2CPREG3_CH2CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG3_CH2CPREG3_CH2CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG3_RESETVAL                                        (0x00000000U)

/* CH2CPREG4 */

#define CSL_RSS_CTRL_CH2CPREG4_CH2CPREG4_CH2CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG4_CH2CPREG4_CH2CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG4_CH2CPREG4_CH2CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG4_CH2CPREG4_CH2CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG4_RESETVAL                                        (0x00000000U)

/* CH2CPREG5 */

#define CSL_RSS_CTRL_CH2CPREG5_CH2CPREG5_CH2CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG5_CH2CPREG5_CH2CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG5_CH2CPREG5_CH2CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG5_CH2CPREG5_CH2CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG5_RESETVAL                                        (0x00000000U)

/* CH2CPREG6 */

#define CSL_RSS_CTRL_CH2CPREG6_CH2CPREG6_CH2CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG6_CH2CPREG6_CH2CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG6_CH2CPREG6_CH2CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG6_CH2CPREG6_CH2CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG6_RESETVAL                                        (0x00000000U)

/* CH2CPREG7 */

#define CSL_RSS_CTRL_CH2CPREG7_CH2CPREG7_CH2CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG7_CH2CPREG7_CH2CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG7_CH2CPREG7_CH2CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG7_CH2CPREG7_CH2CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG7_RESETVAL                                        (0x00000000U)

/* CH2CPREG8 */

#define CSL_RSS_CTRL_CH2CPREG8_CH2CPREG8_CH2CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG8_CH2CPREG8_CH2CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG8_CH2CPREG8_CH2CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG8_CH2CPREG8_CH2CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG8_RESETVAL                                        (0x00000000U)

/* CH2CPREG9 */

#define CSL_RSS_CTRL_CH2CPREG9_CH2CPREG9_CH2CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG9_CH2CPREG9_CH2CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG9_CH2CPREG9_CH2CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG9_CH2CPREG9_CH2CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG9_RESETVAL                                        (0x00000000U)

/* CH2CPREG10 */

#define CSL_RSS_CTRL_CH2CPREG10_CH2CPREG10_CH2CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG10_CH2CPREG10_CH2CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG10_CH2CPREG10_CH2CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG10_CH2CPREG10_CH2CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG10_RESETVAL                                       (0x00000000U)

/* CH2CPREG11 */

#define CSL_RSS_CTRL_CH2CPREG11_CH2CPREG11_CH2CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG11_CH2CPREG11_CH2CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG11_CH2CPREG11_CH2CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG11_CH2CPREG11_CH2CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG11_RESETVAL                                       (0x00000000U)

/* CH2CPREG12 */

#define CSL_RSS_CTRL_CH2CPREG12_CH2CPREG12_CH2CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG12_CH2CPREG12_CH2CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG12_CH2CPREG12_CH2CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG12_CH2CPREG12_CH2CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG12_RESETVAL                                       (0x00000000U)

/* CH2CPREG13 */

#define CSL_RSS_CTRL_CH2CPREG13_CH2CPREG13_CH2CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG13_CH2CPREG13_CH2CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG13_CH2CPREG13_CH2CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG13_CH2CPREG13_CH2CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG13_RESETVAL                                       (0x00000000U)

/* CH2CPREG14 */

#define CSL_RSS_CTRL_CH2CPREG14_CH2CPREG14_CH2CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG14_CH2CPREG14_CH2CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG14_CH2CPREG14_CH2CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG14_CH2CPREG14_CH2CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG14_RESETVAL                                       (0x00000000U)

/* CH2CPREG15 */

#define CSL_RSS_CTRL_CH2CPREG15_CH2CPREG15_CH2CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH2CPREG15_CH2CPREG15_CH2CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG15_CH2CPREG15_CH2CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH2CPREG15_CH2CPREG15_CH2CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH2CPREG15_RESETVAL                                       (0x00000000U)

/* CH3CPREG0 */

#define CSL_RSS_CTRL_CH3CPREG0_CH3CPREG0_CH3CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG0_CH3CPREG0_CH3CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG0_CH3CPREG0_CH3CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG0_CH3CPREG0_CH3CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG0_RESETVAL                                        (0x00000000U)

/* CH3CPREG1 */

#define CSL_RSS_CTRL_CH3CPREG1_CH3CPREG1_CH3CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG1_CH3CPREG1_CH3CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG1_CH3CPREG1_CH3CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG1_CH3CPREG1_CH3CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG1_RESETVAL                                        (0x00000000U)

/* CH3CPREG2 */

#define CSL_RSS_CTRL_CH3CPREG2_CH3CPREG2_CH3CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG2_CH3CPREG2_CH3CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG2_CH3CPREG2_CH3CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG2_CH3CPREG2_CH3CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG2_RESETVAL                                        (0x00000000U)

/* CH3CPREG3 */

#define CSL_RSS_CTRL_CH3CPREG3_CH3CPREG3_CH3CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG3_CH3CPREG3_CH3CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG3_CH3CPREG3_CH3CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG3_CH3CPREG3_CH3CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG3_RESETVAL                                        (0x00000000U)

/* CH3CPREG4 */

#define CSL_RSS_CTRL_CH3CPREG4_CH3CPREG4_CH3CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG4_CH3CPREG4_CH3CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG4_CH3CPREG4_CH3CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG4_CH3CPREG4_CH3CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG4_RESETVAL                                        (0x00000000U)

/* CH3CPREG5 */

#define CSL_RSS_CTRL_CH3CPREG5_CH3CPREG5_CH3CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG5_CH3CPREG5_CH3CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG5_CH3CPREG5_CH3CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG5_CH3CPREG5_CH3CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG5_RESETVAL                                        (0x00000000U)

/* CH3CPREG6 */

#define CSL_RSS_CTRL_CH3CPREG6_CH3CPREG6_CH3CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG6_CH3CPREG6_CH3CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG6_CH3CPREG6_CH3CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG6_CH3CPREG6_CH3CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG6_RESETVAL                                        (0x00000000U)

/* CH3CPREG7 */

#define CSL_RSS_CTRL_CH3CPREG7_CH3CPREG7_CH3CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG7_CH3CPREG7_CH3CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG7_CH3CPREG7_CH3CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG7_CH3CPREG7_CH3CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG7_RESETVAL                                        (0x00000000U)

/* CH3CPREG8 */

#define CSL_RSS_CTRL_CH3CPREG8_CH3CPREG8_CH3CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG8_CH3CPREG8_CH3CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG8_CH3CPREG8_CH3CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG8_CH3CPREG8_CH3CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG8_RESETVAL                                        (0x00000000U)

/* CH3CPREG9 */

#define CSL_RSS_CTRL_CH3CPREG9_CH3CPREG9_CH3CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG9_CH3CPREG9_CH3CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG9_CH3CPREG9_CH3CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG9_CH3CPREG9_CH3CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG9_RESETVAL                                        (0x00000000U)

/* CH3CPREG10 */

#define CSL_RSS_CTRL_CH3CPREG10_CH3CPREG10_CH3CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG10_CH3CPREG10_CH3CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG10_CH3CPREG10_CH3CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG10_CH3CPREG10_CH3CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG10_RESETVAL                                       (0x00000000U)

/* CH3CPREG11 */

#define CSL_RSS_CTRL_CH3CPREG11_CH3CPREG11_CH3CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG11_CH3CPREG11_CH3CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG11_CH3CPREG11_CH3CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG11_CH3CPREG11_CH3CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG11_RESETVAL                                       (0x00000000U)

/* CH3CPREG12 */

#define CSL_RSS_CTRL_CH3CPREG12_CH3CPREG12_CH3CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG12_CH3CPREG12_CH3CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG12_CH3CPREG12_CH3CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG12_CH3CPREG12_CH3CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG12_RESETVAL                                       (0x00000000U)

/* CH3CPREG13 */

#define CSL_RSS_CTRL_CH3CPREG13_CH3CPREG13_CH3CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG13_CH3CPREG13_CH3CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG13_CH3CPREG13_CH3CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG13_CH3CPREG13_CH3CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG13_RESETVAL                                       (0x00000000U)

/* CH3CPREG14 */

#define CSL_RSS_CTRL_CH3CPREG14_CH3CPREG14_CH3CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG14_CH3CPREG14_CH3CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG14_CH3CPREG14_CH3CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG14_CH3CPREG14_CH3CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG14_RESETVAL                                       (0x00000000U)

/* CH3CPREG15 */

#define CSL_RSS_CTRL_CH3CPREG15_CH3CPREG15_CH3CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH3CPREG15_CH3CPREG15_CH3CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG15_CH3CPREG15_CH3CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH3CPREG15_CH3CPREG15_CH3CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH3CPREG15_RESETVAL                                       (0x00000000U)

/* CH4CPREG0 */

#define CSL_RSS_CTRL_CH4CPREG0_CH4CPREG0_CH4CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG0_CH4CPREG0_CH4CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG0_CH4CPREG0_CH4CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG0_CH4CPREG0_CH4CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG0_RESETVAL                                        (0x00000000U)

/* CH4CPREG1 */

#define CSL_RSS_CTRL_CH4CPREG1_CH4CPREG1_CH4CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG1_CH4CPREG1_CH4CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG1_CH4CPREG1_CH4CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG1_CH4CPREG1_CH4CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG1_RESETVAL                                        (0x00000000U)

/* CH4CPREG2 */

#define CSL_RSS_CTRL_CH4CPREG2_CH4CPREG2_CH4CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG2_CH4CPREG2_CH4CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG2_CH4CPREG2_CH4CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG2_CH4CPREG2_CH4CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG2_RESETVAL                                        (0x00000000U)

/* CH4CPREG3 */

#define CSL_RSS_CTRL_CH4CPREG3_CH4CPREG3_CH4CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG3_CH4CPREG3_CH4CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG3_CH4CPREG3_CH4CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG3_CH4CPREG3_CH4CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG3_RESETVAL                                        (0x00000000U)

/* CH4CPREG4 */

#define CSL_RSS_CTRL_CH4CPREG4_CH4CPREG4_CH4CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG4_CH4CPREG4_CH4CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG4_CH4CPREG4_CH4CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG4_CH4CPREG4_CH4CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG4_RESETVAL                                        (0x00000000U)

/* CH4CPREG5 */

#define CSL_RSS_CTRL_CH4CPREG5_CH4CPREG5_CH4CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG5_CH4CPREG5_CH4CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG5_CH4CPREG5_CH4CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG5_CH4CPREG5_CH4CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG5_RESETVAL                                        (0x00000000U)

/* CH4CPREG6 */

#define CSL_RSS_CTRL_CH4CPREG6_CH4CPREG6_CH4CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG6_CH4CPREG6_CH4CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG6_CH4CPREG6_CH4CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG6_CH4CPREG6_CH4CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG6_RESETVAL                                        (0x00000000U)

/* CH4CPREG7 */

#define CSL_RSS_CTRL_CH4CPREG7_CH4CPREG7_CH4CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG7_CH4CPREG7_CH4CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG7_CH4CPREG7_CH4CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG7_CH4CPREG7_CH4CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG7_RESETVAL                                        (0x00000000U)

/* CH4CPREG8 */

#define CSL_RSS_CTRL_CH4CPREG8_CH4CPREG8_CH4CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG8_CH4CPREG8_CH4CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG8_CH4CPREG8_CH4CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG8_CH4CPREG8_CH4CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG8_RESETVAL                                        (0x00000000U)

/* CH4CPREG9 */

#define CSL_RSS_CTRL_CH4CPREG9_CH4CPREG9_CH4CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG9_CH4CPREG9_CH4CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG9_CH4CPREG9_CH4CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG9_CH4CPREG9_CH4CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG9_RESETVAL                                        (0x00000000U)

/* CH4CPREG10 */

#define CSL_RSS_CTRL_CH4CPREG10_CH4CPREG10_CH4CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG10_CH4CPREG10_CH4CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG10_CH4CPREG10_CH4CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG10_CH4CPREG10_CH4CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG10_RESETVAL                                       (0x00000000U)

/* CH4CPREG11 */

#define CSL_RSS_CTRL_CH4CPREG11_CH4CPREG11_CH4CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG11_CH4CPREG11_CH4CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG11_CH4CPREG11_CH4CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG11_CH4CPREG11_CH4CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG11_RESETVAL                                       (0x00000000U)

/* CH4CPREG12 */

#define CSL_RSS_CTRL_CH4CPREG12_CH4CPREG12_CH4CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG12_CH4CPREG12_CH4CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG12_CH4CPREG12_CH4CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG12_CH4CPREG12_CH4CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG12_RESETVAL                                       (0x00000000U)

/* CH4CPREG13 */

#define CSL_RSS_CTRL_CH4CPREG13_CH4CPREG13_CH4CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG13_CH4CPREG13_CH4CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG13_CH4CPREG13_CH4CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG13_CH4CPREG13_CH4CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG13_RESETVAL                                       (0x00000000U)

/* CH4CPREG14 */

#define CSL_RSS_CTRL_CH4CPREG14_CH4CPREG14_CH4CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG14_CH4CPREG14_CH4CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG14_CH4CPREG14_CH4CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG14_CH4CPREG14_CH4CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG14_RESETVAL                                       (0x00000000U)

/* CH4CPREG15 */

#define CSL_RSS_CTRL_CH4CPREG15_CH4CPREG15_CH4CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH4CPREG15_CH4CPREG15_CH4CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG15_CH4CPREG15_CH4CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH4CPREG15_CH4CPREG15_CH4CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH4CPREG15_RESETVAL                                       (0x00000000U)

/* CH5CPREG0 */

#define CSL_RSS_CTRL_CH5CPREG0_CH5CPREG0_CH5CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG0_CH5CPREG0_CH5CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG0_CH5CPREG0_CH5CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG0_CH5CPREG0_CH5CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG0_RESETVAL                                        (0x00000000U)

/* CH5CPREG1 */

#define CSL_RSS_CTRL_CH5CPREG1_CH5CPREG1_CH5CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG1_CH5CPREG1_CH5CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG1_CH5CPREG1_CH5CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG1_CH5CPREG1_CH5CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG1_RESETVAL                                        (0x00000000U)

/* CH5CPREG2 */

#define CSL_RSS_CTRL_CH5CPREG2_CH5CPREG2_CH5CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG2_CH5CPREG2_CH5CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG2_CH5CPREG2_CH5CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG2_CH5CPREG2_CH5CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG2_RESETVAL                                        (0x00000000U)

/* CH5CPREG3 */

#define CSL_RSS_CTRL_CH5CPREG3_CH5CPREG3_CH5CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG3_CH5CPREG3_CH5CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG3_CH5CPREG3_CH5CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG3_CH5CPREG3_CH5CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG3_RESETVAL                                        (0x00000000U)

/* CH5CPREG4 */

#define CSL_RSS_CTRL_CH5CPREG4_CH5CPREG4_CH5CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG4_CH5CPREG4_CH5CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG4_CH5CPREG4_CH5CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG4_CH5CPREG4_CH5CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG4_RESETVAL                                        (0x00000000U)

/* CH5CPREG5 */

#define CSL_RSS_CTRL_CH5CPREG5_CH5CPREG5_CH5CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG5_CH5CPREG5_CH5CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG5_CH5CPREG5_CH5CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG5_CH5CPREG5_CH5CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG5_RESETVAL                                        (0x00000000U)

/* CH5CPREG6 */

#define CSL_RSS_CTRL_CH5CPREG6_CH5CPREG6_CH5CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG6_CH5CPREG6_CH5CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG6_CH5CPREG6_CH5CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG6_CH5CPREG6_CH5CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG6_RESETVAL                                        (0x00000000U)

/* CH5CPREG7 */

#define CSL_RSS_CTRL_CH5CPREG7_CH5CPREG7_CH5CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG7_CH5CPREG7_CH5CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG7_CH5CPREG7_CH5CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG7_CH5CPREG7_CH5CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG7_RESETVAL                                        (0x00000000U)

/* CH5CPREG8 */

#define CSL_RSS_CTRL_CH5CPREG8_CH5CPREG8_CH5CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG8_CH5CPREG8_CH5CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG8_CH5CPREG8_CH5CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG8_CH5CPREG8_CH5CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG8_RESETVAL                                        (0x00000000U)

/* CH5CPREG9 */

#define CSL_RSS_CTRL_CH5CPREG9_CH5CPREG9_CH5CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG9_CH5CPREG9_CH5CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG9_CH5CPREG9_CH5CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG9_CH5CPREG9_CH5CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG9_RESETVAL                                        (0x00000000U)

/* CH5CPREG10 */

#define CSL_RSS_CTRL_CH5CPREG10_CH5CPREG10_CH5CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG10_CH5CPREG10_CH5CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG10_CH5CPREG10_CH5CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG10_CH5CPREG10_CH5CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG10_RESETVAL                                       (0x00000000U)

/* CH5CPREG11 */

#define CSL_RSS_CTRL_CH5CPREG11_CH5CPREG11_CH5CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG11_CH5CPREG11_CH5CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG11_CH5CPREG11_CH5CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG11_CH5CPREG11_CH5CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG11_RESETVAL                                       (0x00000000U)

/* CH5CPREG12 */

#define CSL_RSS_CTRL_CH5CPREG12_CH5CPREG12_CH5CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG12_CH5CPREG12_CH5CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG12_CH5CPREG12_CH5CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG12_CH5CPREG12_CH5CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG12_RESETVAL                                       (0x00000000U)

/* CH5CPREG13 */

#define CSL_RSS_CTRL_CH5CPREG13_CH5CPREG13_CH5CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG13_CH5CPREG13_CH5CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG13_CH5CPREG13_CH5CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG13_CH5CPREG13_CH5CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG13_RESETVAL                                       (0x00000000U)

/* CH5CPREG14 */

#define CSL_RSS_CTRL_CH5CPREG14_CH5CPREG14_CH5CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG14_CH5CPREG14_CH5CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG14_CH5CPREG14_CH5CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG14_CH5CPREG14_CH5CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG14_RESETVAL                                       (0x00000000U)

/* CH5CPREG15 */

#define CSL_RSS_CTRL_CH5CPREG15_CH5CPREG15_CH5CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH5CPREG15_CH5CPREG15_CH5CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG15_CH5CPREG15_CH5CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH5CPREG15_CH5CPREG15_CH5CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH5CPREG15_RESETVAL                                       (0x00000000U)

/* CH6CPREG0 */

#define CSL_RSS_CTRL_CH6CPREG0_CH6CPREG0_CH6CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG0_CH6CPREG0_CH6CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG0_CH6CPREG0_CH6CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG0_CH6CPREG0_CH6CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG0_RESETVAL                                        (0x00000000U)

/* CH6CPREG1 */

#define CSL_RSS_CTRL_CH6CPREG1_CH6CPREG1_CH6CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG1_CH6CPREG1_CH6CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG1_CH6CPREG1_CH6CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG1_CH6CPREG1_CH6CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG1_RESETVAL                                        (0x00000000U)

/* CH6CPREG2 */

#define CSL_RSS_CTRL_CH6CPREG2_CH6CPREG2_CH6CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG2_CH6CPREG2_CH6CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG2_CH6CPREG2_CH6CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG2_CH6CPREG2_CH6CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG2_RESETVAL                                        (0x00000000U)

/* CH6CPREG3 */

#define CSL_RSS_CTRL_CH6CPREG3_CH6CPREG3_CH6CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG3_CH6CPREG3_CH6CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG3_CH6CPREG3_CH6CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG3_CH6CPREG3_CH6CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG3_RESETVAL                                        (0x00000000U)

/* CH6CPREG4 */

#define CSL_RSS_CTRL_CH6CPREG4_CH6CPREG4_CH6CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG4_CH6CPREG4_CH6CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG4_CH6CPREG4_CH6CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG4_CH6CPREG4_CH6CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG4_RESETVAL                                        (0x00000000U)

/* CH6CPREG5 */

#define CSL_RSS_CTRL_CH6CPREG5_CH6CPREG5_CH6CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG5_CH6CPREG5_CH6CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG5_CH6CPREG5_CH6CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG5_CH6CPREG5_CH6CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG5_RESETVAL                                        (0x00000000U)

/* CH6CPREG6 */

#define CSL_RSS_CTRL_CH6CPREG6_CH6CPREG6_CH6CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG6_CH6CPREG6_CH6CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG6_CH6CPREG6_CH6CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG6_CH6CPREG6_CH6CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG6_RESETVAL                                        (0x00000000U)

/* CH6CPREG7 */

#define CSL_RSS_CTRL_CH6CPREG7_CH6CPREG7_CH6CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG7_CH6CPREG7_CH6CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG7_CH6CPREG7_CH6CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG7_CH6CPREG7_CH6CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG7_RESETVAL                                        (0x00000000U)

/* CH6CPREG8 */

#define CSL_RSS_CTRL_CH6CPREG8_CH6CPREG8_CH6CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG8_CH6CPREG8_CH6CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG8_CH6CPREG8_CH6CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG8_CH6CPREG8_CH6CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG8_RESETVAL                                        (0x00000000U)

/* CH6CPREG9 */

#define CSL_RSS_CTRL_CH6CPREG9_CH6CPREG9_CH6CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG9_CH6CPREG9_CH6CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG9_CH6CPREG9_CH6CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG9_CH6CPREG9_CH6CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG9_RESETVAL                                        (0x00000000U)

/* CH6CPREG10 */

#define CSL_RSS_CTRL_CH6CPREG10_CH6CPREG10_CH6CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG10_CH6CPREG10_CH6CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG10_CH6CPREG10_CH6CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG10_CH6CPREG10_CH6CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG10_RESETVAL                                       (0x00000000U)

/* CH6CPREG11 */

#define CSL_RSS_CTRL_CH6CPREG11_CH6CPREG11_CH6CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG11_CH6CPREG11_CH6CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG11_CH6CPREG11_CH6CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG11_CH6CPREG11_CH6CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG11_RESETVAL                                       (0x00000000U)

/* CH6CPREG12 */

#define CSL_RSS_CTRL_CH6CPREG12_CH6CPREG12_CH6CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG12_CH6CPREG12_CH6CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG12_CH6CPREG12_CH6CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG12_CH6CPREG12_CH6CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG12_RESETVAL                                       (0x00000000U)

/* CH6CPREG13 */

#define CSL_RSS_CTRL_CH6CPREG13_CH6CPREG13_CH6CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG13_CH6CPREG13_CH6CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG13_CH6CPREG13_CH6CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG13_CH6CPREG13_CH6CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG13_RESETVAL                                       (0x00000000U)

/* CH6CPREG14 */

#define CSL_RSS_CTRL_CH6CPREG14_CH6CPREG14_CH6CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG14_CH6CPREG14_CH6CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG14_CH6CPREG14_CH6CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG14_CH6CPREG14_CH6CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG14_RESETVAL                                       (0x00000000U)

/* CH6CPREG15 */

#define CSL_RSS_CTRL_CH6CPREG15_CH6CPREG15_CH6CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH6CPREG15_CH6CPREG15_CH6CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG15_CH6CPREG15_CH6CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH6CPREG15_CH6CPREG15_CH6CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH6CPREG15_RESETVAL                                       (0x00000000U)

/* CH7CPREG0 */

#define CSL_RSS_CTRL_CH7CPREG0_CH7CPREG0_CH7CPREG0_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG0_CH7CPREG0_CH7CPREG0_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG0_CH7CPREG0_CH7CPREG0_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG0_CH7CPREG0_CH7CPREG0_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG0_RESETVAL                                        (0x00000000U)

/* CH7CPREG1 */

#define CSL_RSS_CTRL_CH7CPREG1_CH7CPREG1_CH7CPREG1_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG1_CH7CPREG1_CH7CPREG1_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG1_CH7CPREG1_CH7CPREG1_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG1_CH7CPREG1_CH7CPREG1_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG1_RESETVAL                                        (0x00000000U)

/* CH7CPREG2 */

#define CSL_RSS_CTRL_CH7CPREG2_CH7CPREG2_CH7CPREG2_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG2_CH7CPREG2_CH7CPREG2_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG2_CH7CPREG2_CH7CPREG2_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG2_CH7CPREG2_CH7CPREG2_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG2_RESETVAL                                        (0x00000000U)

/* CH7CPREG3 */

#define CSL_RSS_CTRL_CH7CPREG3_CH7CPREG3_CH7CPREG3_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG3_CH7CPREG3_CH7CPREG3_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG3_CH7CPREG3_CH7CPREG3_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG3_CH7CPREG3_CH7CPREG3_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG3_RESETVAL                                        (0x00000000U)

/* CH7CPREG4 */

#define CSL_RSS_CTRL_CH7CPREG4_CH7CPREG4_CH7CPREG4_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG4_CH7CPREG4_CH7CPREG4_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG4_CH7CPREG4_CH7CPREG4_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG4_CH7CPREG4_CH7CPREG4_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG4_RESETVAL                                        (0x00000000U)

/* CH7CPREG5 */

#define CSL_RSS_CTRL_CH7CPREG5_CH7CPREG5_CH7CPREG5_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG5_CH7CPREG5_CH7CPREG5_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG5_CH7CPREG5_CH7CPREG5_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG5_CH7CPREG5_CH7CPREG5_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG5_RESETVAL                                        (0x00000000U)

/* CH7CPREG6 */

#define CSL_RSS_CTRL_CH7CPREG6_CH7CPREG6_CH7CPREG6_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG6_CH7CPREG6_CH7CPREG6_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG6_CH7CPREG6_CH7CPREG6_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG6_CH7CPREG6_CH7CPREG6_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG6_RESETVAL                                        (0x00000000U)

/* CH7CPREG7 */

#define CSL_RSS_CTRL_CH7CPREG7_CH7CPREG7_CH7CPREG7_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG7_CH7CPREG7_CH7CPREG7_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG7_CH7CPREG7_CH7CPREG7_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG7_CH7CPREG7_CH7CPREG7_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG7_RESETVAL                                        (0x00000000U)

/* CH7CPREG8 */

#define CSL_RSS_CTRL_CH7CPREG8_CH7CPREG8_CH7CPREG8_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG8_CH7CPREG8_CH7CPREG8_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG8_CH7CPREG8_CH7CPREG8_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG8_CH7CPREG8_CH7CPREG8_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG8_RESETVAL                                        (0x00000000U)

/* CH7CPREG9 */

#define CSL_RSS_CTRL_CH7CPREG9_CH7CPREG9_CH7CPREG9_MASK                        (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG9_CH7CPREG9_CH7CPREG9_SHIFT                       (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG9_CH7CPREG9_CH7CPREG9_RESETVAL                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG9_CH7CPREG9_CH7CPREG9_MAX                         (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG9_RESETVAL                                        (0x00000000U)

/* CH7CPREG10 */

#define CSL_RSS_CTRL_CH7CPREG10_CH7CPREG10_CH7CPREG10_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG10_CH7CPREG10_CH7CPREG10_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG10_CH7CPREG10_CH7CPREG10_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG10_CH7CPREG10_CH7CPREG10_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG10_RESETVAL                                       (0x00000000U)

/* CH7CPREG11 */

#define CSL_RSS_CTRL_CH7CPREG11_CH7CPREG11_CH7CPREG11_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG11_CH7CPREG11_CH7CPREG11_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG11_CH7CPREG11_CH7CPREG11_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG11_CH7CPREG11_CH7CPREG11_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG11_RESETVAL                                       (0x00000000U)

/* CH7CPREG12 */

#define CSL_RSS_CTRL_CH7CPREG12_CH7CPREG12_CH7CPREG12_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG12_CH7CPREG12_CH7CPREG12_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG12_CH7CPREG12_CH7CPREG12_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG12_CH7CPREG12_CH7CPREG12_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG12_RESETVAL                                       (0x00000000U)

/* CH7CPREG13 */

#define CSL_RSS_CTRL_CH7CPREG13_CH7CPREG13_CH7CPREG13_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG13_CH7CPREG13_CH7CPREG13_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG13_CH7CPREG13_CH7CPREG13_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG13_CH7CPREG13_CH7CPREG13_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG13_RESETVAL                                       (0x00000000U)

/* CH7CPREG14 */

#define CSL_RSS_CTRL_CH7CPREG14_CH7CPREG14_CH7CPREG14_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG14_CH7CPREG14_CH7CPREG14_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG14_CH7CPREG14_CH7CPREG14_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG14_CH7CPREG14_CH7CPREG14_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG14_RESETVAL                                       (0x00000000U)

/* CH7CPREG15 */

#define CSL_RSS_CTRL_CH7CPREG15_CH7CPREG15_CH7CPREG15_MASK                     (0xFFFFFFFFU)
#define CSL_RSS_CTRL_CH7CPREG15_CH7CPREG15_CH7CPREG15_SHIFT                    (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG15_CH7CPREG15_CH7CPREG15_RESETVAL                 (0x00000000U)
#define CSL_RSS_CTRL_CH7CPREG15_CH7CPREG15_CH7CPREG15_MAX                      (0xFFFFFFFFU)

#define CSL_RSS_CTRL_CH7CPREG15_RESETVAL                                       (0x00000000U)

/* CH01_HIL_CP_OVERRIDE */

#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP0_MASK     (0x0000FFFFU)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP0_SHIFT    (0x00000000U)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP0_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP0_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP1_MASK     (0xFFFF0000U)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP1_SHIFT    (0x00000010U)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP1_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_CH01_HIL_CP_OVERRIDE_CHIRP1_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH01_HIL_CP_OVERRIDE_RESETVAL                             (0x00000000U)

/* CH23_HIL_CP_OVERRIDE */

#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP2_MASK     (0x0000FFFFU)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP2_SHIFT    (0x00000000U)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP2_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP2_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP3_MASK     (0xFFFF0000U)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP3_SHIFT    (0x00000010U)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP3_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_CH23_HIL_CP_OVERRIDE_CHIRP3_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH23_HIL_CP_OVERRIDE_RESETVAL                             (0x00000000U)

/* CH45_HIL_CP_OVERRIDE */

#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP4_MASK     (0x0000FFFFU)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP4_SHIFT    (0x00000000U)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP4_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP4_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP5_MASK     (0xFFFF0000U)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP5_SHIFT    (0x00000010U)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP5_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_CH45_HIL_CP_OVERRIDE_CHIRP5_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH45_HIL_CP_OVERRIDE_RESETVAL                             (0x00000000U)

/* CH67_HIL_CP_OVERRIDE */

#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP6_MASK     (0x0000FFFFU)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP6_SHIFT    (0x00000000U)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP6_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP6_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP7_MASK     (0xFFFF0000U)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP7_SHIFT    (0x00000010U)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP7_RESETVAL (0x00000000U)
#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_CH67_HIL_CP_OVERRIDE_CHIRP7_MAX      (0x0000FFFFU)

#define CSL_RSS_CTRL_CH67_HIL_CP_OVERRIDE_RESETVAL                             (0x00000000U)

/* CH_HIL_CP_OVERRIDE */

#define CSL_RSS_CTRL_CH_HIL_CP_OVERRIDE_CH_HIL_CP_OVERRIDE_CHIRP_MASK          (0x0000FFFFU)
#define CSL_RSS_CTRL_CH_HIL_CP_OVERRIDE_CH_HIL_CP_OVERRIDE_CHIRP_SHIFT         (0x00000000U)
#define CSL_RSS_CTRL_CH_HIL_CP_OVERRIDE_CH_HIL_CP_OVERRIDE_CHIRP_RESETVAL      (0x00000000U)
#define CSL_RSS_CTRL_CH_HIL_CP_OVERRIDE_CH_HIL_CP_OVERRIDE_CHIRP_MAX           (0x0000FFFFU)

#define CSL_RSS_CTRL_CH_HIL_CP_OVERRIDE_RESETVAL                               (0x00000000U)

/* HW_SPARE_RW0 */

#define CSL_RSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW0_HW_SPARE_RW0_HW_SPARE_RW0_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RW0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW1 */

#define CSL_RSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW1_HW_SPARE_RW1_HW_SPARE_RW1_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RW1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW2 */

#define CSL_RSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW2_HW_SPARE_RW2_HW_SPARE_RW2_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RW2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RW3 */

#define CSL_RSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RW3_HW_SPARE_RW3_HW_SPARE_RW3_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RW3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO0 */

#define CSL_RSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO0_HW_SPARE_RO0_HW_SPARE_RO0_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RO0_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO1 */

#define CSL_RSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO1_HW_SPARE_RO1_HW_SPARE_RO1_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RO1_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO2 */

#define CSL_RSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO2_HW_SPARE_RO2_HW_SPARE_RO2_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RO2_RESETVAL                                     (0x00000000U)

/* HW_SPARE_RO3 */

#define CSL_RSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_RO3_HW_SPARE_RO3_HW_SPARE_RO3_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_RO3_RESETVAL                                     (0x00000000U)

/* HW_SPARE_WPH */

#define CSL_RSS_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MASK               (0xFFFFFFFFU)
#define CSL_RSS_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_SHIFT              (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_RESETVAL           (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_WPH_HW_SPARE_WPH_HW_SPARE_WPH_MAX                (0xFFFFFFFFU)

#define CSL_RSS_CTRL_HW_SPARE_WPH_RESETVAL                                     (0x00000000U)

/* HW_SPARE_REC */

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MASK              (0x00000001U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_SHIFT             (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC0_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MASK              (0x00000002U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_SHIFT             (0x00000001U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC1_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MASK              (0x00000004U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_SHIFT             (0x00000002U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC2_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MASK              (0x00000008U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_SHIFT             (0x00000003U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC3_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MASK              (0x00000010U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_SHIFT             (0x00000004U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC4_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MASK              (0x00000020U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_SHIFT             (0x00000005U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC5_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MASK              (0x00000040U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_SHIFT             (0x00000006U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC6_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MASK              (0x00000080U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_SHIFT             (0x00000007U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC7_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MASK              (0x00000100U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_SHIFT             (0x00000008U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC8_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MASK              (0x00000200U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_SHIFT             (0x00000009U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_RESETVAL          (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC9_MAX               (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MASK             (0x00000400U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_SHIFT            (0x0000000AU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC10_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MASK             (0x00000800U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_SHIFT            (0x0000000BU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC11_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MASK             (0x00001000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_SHIFT            (0x0000000CU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC12_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MASK             (0x00002000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_SHIFT            (0x0000000DU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC13_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MASK             (0x00004000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_SHIFT            (0x0000000EU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC14_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MASK             (0x00008000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_SHIFT            (0x0000000FU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC15_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MASK             (0x00010000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_SHIFT            (0x00000010U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC16_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MASK             (0x00020000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_SHIFT            (0x00000011U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC17_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MASK             (0x00040000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_SHIFT            (0x00000012U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC18_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MASK             (0x00080000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_SHIFT            (0x00000013U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC19_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MASK             (0x00100000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_SHIFT            (0x00000014U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC20_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MASK             (0x00200000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_SHIFT            (0x00000015U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC21_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MASK             (0x00400000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_SHIFT            (0x00000016U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC22_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MASK             (0x00800000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_SHIFT            (0x00000017U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC23_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MASK             (0x01000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_SHIFT            (0x00000018U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC24_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MASK             (0x02000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_SHIFT            (0x00000019U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC25_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MASK             (0x04000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_SHIFT            (0x0000001AU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC26_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MASK             (0x08000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_SHIFT            (0x0000001BU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC27_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MASK             (0x10000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_SHIFT            (0x0000001CU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC28_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MASK             (0x20000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_SHIFT            (0x0000001DU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC29_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MASK             (0x40000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_SHIFT            (0x0000001EU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC30_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MASK             (0x80000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_SHIFT            (0x0000001FU)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_RESETVAL         (0x00000000U)
#define CSL_RSS_CTRL_HW_SPARE_REC_HW_SPARE_REC_HW_SPARE_REC31_MAX              (0x00000001U)

#define CSL_RSS_CTRL_HW_SPARE_REC_RESETVAL                                     (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_RSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_LOCK0_KICK0_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_RSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                              (0xFFFFFFFFU)
#define CSL_RSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                               (0xFFFFFFFFU)

#define CSL_RSS_CTRL_LOCK0_KICK1_RESETVAL                                      (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                             (0x00000001U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                            (0x00000000U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                         (0x00000000U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                              (0x00000001U)

#define CSL_RSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                             (0x00000002U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                            (0x00000001U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                         (0x00000000U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                              (0x00000001U)

#define CSL_RSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                             (0x00000004U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                            (0x00000002U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                         (0x00000000U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                              (0x00000001U)

#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                            (0x00000008U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                           (0x00000003U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                        (0x00000000U)
#define CSL_RSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                             (0x00000001U)

#define CSL_RSS_CTRL_INTR_RAW_STATUS_RESETVAL                                  (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK           (0x00000001U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT          (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL       (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX            (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK           (0x00000002U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT          (0x00000001U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL       (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX            (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK           (0x00000004U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT          (0x00000002U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL       (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX            (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK          (0x00000008U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT         (0x00000003U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL      (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX           (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                        (0x00000000U)

/* INTR_ENABLE */

#define CSL_RSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                              (0x00000001U)
#define CSL_RSS_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                             (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                               (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                              (0x00000002U)
#define CSL_RSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                             (0x00000001U)
#define CSL_RSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                               (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                              (0x00000004U)
#define CSL_RSS_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                             (0x00000002U)
#define CSL_RSS_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                          (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                               (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                             (0x00000008U)
#define CSL_RSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                            (0x00000003U)
#define CSL_RSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                         (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                              (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_RESETVAL                                      (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                    (0x00000001U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                   (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                    (0x00000002U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                   (0x00000001U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                    (0x00000004U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                   (0x00000002U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL                (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                     (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                   (0x00000008U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                  (0x00000003U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                    (0x00000001U)

#define CSL_RSS_CTRL_INTR_ENABLE_CLEAR_RESETVAL                                (0x00000000U)

/* EOI */

#define CSL_RSS_CTRL_EOI_EOI_VECTOR_MASK                                       (0x000000FFU)
#define CSL_RSS_CTRL_EOI_EOI_VECTOR_SHIFT                                      (0x00000000U)
#define CSL_RSS_CTRL_EOI_EOI_VECTOR_RESETVAL                                   (0x00000000U)
#define CSL_RSS_CTRL_EOI_EOI_VECTOR_MAX                                        (0x000000FFU)

#define CSL_RSS_CTRL_EOI_RESETVAL                                              (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_RSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                             (0xFFFFFFFFU)
#define CSL_RSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                            (0x00000000U)
#define CSL_RSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                         (0x00000000U)
#define CSL_RSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                              (0xFFFFFFFFU)

#define CSL_RSS_CTRL_FAULT_ADDRESS_RESETVAL                                    (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                         (0x0000003FU)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                        (0x00000000U)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                     (0x00000000U)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                          (0x0000003FU)

#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                           (0x00000040U)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                          (0x00000006U)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                       (0x00000000U)
#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                            (0x00000001U)

#define CSL_RSS_CTRL_FAULT_TYPE_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                       (0x000000FFU)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                      (0x00000000U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                   (0x00000000U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                        (0x000000FFU)

#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                      (0x000FFF00U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                     (0x00000008U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                  (0x00000000U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                       (0x00000FFFU)

#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                          (0xFFF00000U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                         (0x00000014U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                      (0x00000000U)
#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                           (0x00000FFFU)

#define CSL_RSS_CTRL_FAULT_ATTR_STATUS_RESETVAL                                (0x00000000U)

/* FAULT_CLEAR */

#define CSL_RSS_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                                (0x00000001U)
#define CSL_RSS_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                               (0x00000000U)
#define CSL_RSS_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                            (0x00000000U)
#define CSL_RSS_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                                 (0x00000001U)

#define CSL_RSS_CTRL_FAULT_CLEAR_RESETVAL                                      (0x00000000U)

/*! \brief define for efficient programming of RSS_CTRL CSI2 lane config registers */
#define CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE  \
                                   (CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG - \
                                    CSL_RSS_CTRL_RSS_CSI2A_LANE0_CFG)

/*! \brief check assumption of constancy of \ref CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE */
#if ( (CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE  != (CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG -    \
                                         CSL_RSS_CTRL_RSS_CSI2A_LANE1_CFG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE  != (CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG -    \
                                         CSL_RSS_CTRL_RSS_CSI2A_LANE2_CFG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE  != (CSL_RSS_CTRL_RSS_CSI2A_LANE4_CFG -    \
                                         CSL_RSS_CTRL_RSS_CSI2A_LANE3_CFG)) \
     )
#error CSL_RSS_CTRL_CSI2_INTER_LANE_CFG_ADDR_DISTANCE assumption violated
#endif

/*! \brief define for efficient programming of RSS_CTRL CSI2 line ping-pong context
       registers across contexts. */
#define CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE  \
           (CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG - \
            CSL_RSS_CTRL_RSS_CSI2A_CTX0_LINE_PING_PONG)

/*! \brief check assumption of constancy of
       \ref CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE */
#if ( (CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE != \
        (CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG -    \
         CSL_RSS_CTRL_RSS_CSI2A_CTX1_LINE_PING_PONG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE != \
        (CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG -    \
         CSL_RSS_CTRL_RSS_CSI2A_CTX2_LINE_PING_PONG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE != \
        (CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG -    \
         CSL_RSS_CTRL_RSS_CSI2A_CTX3_LINE_PING_PONG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE != \
        (CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG -    \
         CSL_RSS_CTRL_RSS_CSI2A_CTX4_LINE_PING_PONG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE != \
        (CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG -    \
         CSL_RSS_CTRL_RSS_CSI2A_CTX5_LINE_PING_PONG)) || \
      (CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE != \
        (CSL_RSS_CTRL_RSS_CSI2A_CTX7_LINE_PING_PONG -    \
         CSL_RSS_CTRL_RSS_CSI2A_CTX6_LINE_PING_PONG))    \
    )
#error CSL_RSS_CTRL_CSI2_INTER_CONTEXT_LINE_PING_PONG_DISTANCE assumption violated
#endif

#ifdef __cplusplus
}
#endif
#endif
