/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 */

/**
 *  \file test_mcanTestCasesConfig.h
 *
 *  \brief This file defines the test cases for mcan UT.
 */

#ifndef TEST_MCAN_TEST_CASES_CONFIG_H_
#define TEST_MCAN_TEST_CASES_CONFIG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "test_mcan.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define APP_MCAN_TX_BUFF_START_ADDR              (0x0U)
#define APP_MCAN_TX_BUFF_SIZE                    (16U)
#define APP_MCAN_TX_FIFO_SIZE                    (16U)
#define APP_MCAN_RX_BUFF_START_ADDR              (0x900U)
#define APP_MCAN_FIFO_0_START_ADDR               (0x1B00U)
#define APP_MCAN_FIFO_0_NUM                      (64U)
#define APP_MCAN_FIFO_1_START_ADDR               (0x2D00U)
#define APP_MCAN_FIFO_1_NUM                      (64U)
#define APP_MCAN_TX_EVENT_START_ADDR             (0x3F00U)
#define APP_MCAN_TX_EVENT_SIZE                   (32U)
#define APP_MCAN_STD_ID_FILT_START_ADDR          (0x4000U)
#define APP_MCAN_STD_ID_FILTER_NUM               (128U)
#define APP_MCAN_EXT_ID_FILT_START_ADDR          (0x4200U)
#define APP_MCAN_EXT_ID_FILTER_NUM               (64U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/**
 *  \brief Different Bit Timings Parameters Configurations.
 */
MCAN_BitTimingParams canFDBitTimings[] =
{
    /* 1000kbps and 5000 kbps*/
    /* This is default baud*/
    {
        0x07U, /* Nominal Baud Rate Pre-scaler */
        0x05U, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        0x00U  /* Data (Re)Synchronization Jump Width */
    },
    /* 1000kbps and 2500 kbps*/
    {
        0x07U, /* Nominal Baud Rate Pre-scaler */
        0x05U, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x03U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        0x00U  /* Data (Re)Synchronization Jump Width */
    },
    /* 250kbps and 5000 kbps*/
    {
        0x0EU, /* Nominal Baud Rate Pre-scaler */
        0x05U, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        0x00U  /* Data (Re)Synchronization Jump Width */
    },
    /* 125kbps and 5000 kbps*/
    {
        0x1FU, /* Nominal Baud Rate Pre-scaler */
        0x05U, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        0x00U  /* Data (Re)Synchronization Jump Width */
    },
    {
        0x1FU, /* Nominal Baud Rate Pre-scaler */
        0x05U, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        MCAN_NBTP_NSJW_MAX + 1, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        0x00U  /* Data (Re)Synchronization Jump Width */
    },
    {
        0x1FU, /* Nominal Baud Rate Pre-scaler */
        0x05U, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX + 1  /* Data (Re)Synchronization Jump Width */
    },
    {
        0x1FU, /* Nominal Baud Rate Pre-scaler */
        MCAN_NBTP_NTSEG1_MAX + 1, /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX  /* Data (Re)Synchronization Jump Width */
    },
    {
        MCAN_NBTP_NBRP_MAX + 1, /* Nominal Baud Rate Pre-scaler */
        MCAN_NBTP_NTSEG1_MAX, /* Nominal Time segment before sample point */
        MCAN_NBTP_NTSEG2_MAX, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX  /* Data (Re)Synchronization Jump Width */
    },
    {
        MCAN_NBTP_NBRP_MAX, /* Nominal Baud Rate Pre-scaler */
        MCAN_NBTP_NTSEG1_MAX, /* Nominal Time segment before sample point */
        MCAN_NBTP_NTSEG2_MAX, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        MCAN_DBTP_DBRP_MAX + 1, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX  /* Data (Re)Synchronization Jump Width */
    },
    {
        MCAN_NBTP_NBRP_MAX, /* Nominal Baud Rate Pre-scaler */
        MCAN_NBTP_NTSEG1_MAX, /* Nominal Time segment before sample point */
        MCAN_NBTP_NTSEG2_MAX, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        MCAN_DBTP_DBRP_MAX, /* Data Baud Rate Pre-scaler */
        MCAN_DBTP_DTSEG1_MAX + 1, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX  /* Data (Re)Synchronization Jump Width */
    },
    {
        MCAN_NBTP_NBRP_MAX, /* Nominal Baud Rate Pre-scaler */
        MCAN_NBTP_NTSEG1_MAX, /* Nominal Time segment before sample point */
        MCAN_NBTP_NTSEG2_MAX, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        MCAN_DBTP_DBRP_MAX, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        MCAN_DBTP_DTSEG2_MAX + 1, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX  /* Data (Re)Synchronization Jump Width */
    },
    {
        0x1FU, /* Nominal Baud Rate Pre-scaler */
        MCAN_NBTP_NTSEG1_MIN - 1U,  /* Nominal Time segment before sample point */
        0x02U, /* Nominal Time segment after sample point */
        0x00U, /* Nominal (Re)Synchronization Jump Width */
        0x01U, /* Data Baud Rate Pre-scaler */
        0x03U, /* Data Time segment before sample point */
        0x02U, /* Data Time segment after sample point */
        MCAN_DBTP_DSJW_MAX  /* Data (Re)Synchronization Jump Width */
    },
};

/**
 *  \brief MCAN Initialization Parameters Configurations.
 */
MCAN_InitParams canFDInitParams[] =
{
    /* Config 0 */
    {
        0x01U, /* FD Operation Enable */
        0x01U, /* Bit Rate Switch Enable */
        0x00U, /* Transmit Pause */
        0x00U, /* FEdge Filtering during Bus Integration */
        0x00U, /* Protocol Exception Handling Disable */
        0x01U, /* Disable Automatic Retransmission */
        0x01U, /* Wakeup Request Enable */
        0x01U, /* Auto-Wakeup Enable */
        0x00U, /* Emulation/Debug Suspend Enable */
        0x00U, /* Emulation/Debug Suspend Fast Ack Enable */
        0x00U, /* Clock Stop Fast Ack Enable */
        0xFFU, /* Start value of the Message RAM Watchdog Counter */
        {
            0x0AU, /* Transmitter Delay Compensation Offset */
            0x06U,  /* Transmitter Delay Compensation Filter Window Length */
        },
        0x01U, /* Transmitter Delay Compensation Enable */
    },
    /* Config 1 */
    {
        0x01U, /* FD Operation Enable */
        0x01U, /* Bit Rate Switch Enable */
        0x00U, /* Transmit Pause */
        0x00U, /* FEdge Filtering during Bus Integration */
        0x00U, /* Protocol Exception Handling Disable */
        0x01U, /* Disable Automatic Retransmission */
        0x01U, /* Wakeup Request Enable */
        0x01U, /* Auto-Wakeup Enable */
        0x00U, /* Emulation/Debug Suspend Enable */
        0x00U, /* Emulation/Debug Suspend Fast Ack Enable */
        0x00U, /* Clock Stop Fast Ack Enable */
        MCAN_RWD_WDC_MAX, /* Start value of the Message RAM Watchdog Counter */
        {
            MCAN_TDCR_TDCF_MAX + 1, /* Transmitter Delay Compensation Offset */
            MCAN_TDCR_TDCO_MAX,  /* Transmitter Delay Compensation Filter Window Length */
        },
        0x01U, /* Transmitter Delay Compensation Enable */
    },
    {
        0x01U, /* FD Operation Enable */
        0x01U, /* Bit Rate Switch Enable */
        0x00U, /* Transmit Pause */
        0x00U, /* FEdge Filtering during Bus Integration */
        0x00U, /* Protocol Exception Handling Disable */
        0x01U, /* Disable Automatic Retransmission */
        0x01U, /* Wakeup Request Enable */
        0x01U, /* Auto-Wakeup Enable */
        0x00U, /* Emulation/Debug Suspend Enable */
        0x00U, /* Emulation/Debug Suspend Fast Ack Enable */
        0x00U, /* Clock Stop Fast Ack Enable */
        MCAN_RWD_WDC_MAX, /* Start value of the Message RAM Watchdog Counter */
        {
            MCAN_TDCR_TDCF_MAX, /* Transmitter Delay Compensation Offset */
            MCAN_TDCR_TDCO_MAX + 1,  /* Transmitter Delay Compensation Filter Window Length */
        },
        0x01U, /* Transmitter Delay Compensation Enable */
    },
    {
        0x01U, /* FD Operation Enable */
        0x01U, /* Bit Rate Switch Enable */
        0x00U, /* Transmit Pause */
        0x00U, /* FEdge Filtering during Bus Integration */
        0x00U, /* Protocol Exception Handling Disable */
        0x01U, /* Disable Automatic Retransmission */
        0x01U, /* Wakeup Request Enable */
        0x01U, /* Auto-Wakeup Enable */
        0x00U, /* Emulation/Debug Suspend Enable */
        0x00U, /* Emulation/Debug Suspend Fast Ack Enable */
        0x00U, /* Clock Stop Fast Ack Enable */
        MCAN_RWD_WDC_MAX + 1, /* Start value of the Message RAM Watchdog Counter */
        {
            MCAN_TDCR_TDCF_MAX, /* Transmitter Delay Compensation Offset */
            MCAN_TDCR_TDCO_MAX,  /* Transmitter Delay Compensation Filter Window Length */
        },
        0x01U, /* Transmitter Delay Compensation Enable */
    },
};

/**
 *  \brief MCAN Configuration Parameters Configurations.
 */
MCAN_ConfigParams canFDConfigParams[] =
{
    /* Config 0 */
    {
        0x0U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        0xFU, /* Timestamp Counter Prescaler */
        0x0U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        0xFFFFU, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 1 */
    {
        0x0U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        0xFU, /* Timestamp Counter Prescaler */
        0x1U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        0xFFFFU, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 2 */
    {
        0x0U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        0xFU, /* Timestamp Counter Prescaler */
        0x0U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        0xFFFFU, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 3 */
    {
        0x0U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        0xFU, /* Timestamp Counter Prescaler */
        0x2U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        0xFFFFU, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 4 */
    {
        0x1U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        0xFU, /* Timestamp Counter Prescaler */
        0x0U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        0xFFFFU, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 5 */
    {
        0x1U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        MCAN_TSCC_TCP_MAX + 1U, /* Timestamp Counter Prescaler */
        0x0U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        MCAN_TOCC_TOP_MAX + 1U, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 6 */
    {
        0x1U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        MCAN_TSCC_TCP_MAX, /* Timestamp Counter Prescaler */
        0x0U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        MCAN_TOCC_TOP_MAX + 1U, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
    /* Config 7 */
    {
        0x1U, /* Bus Monitoring Mode */
        0x0U, /* Restricted Operation Mode */
        0xFU, /* Timestamp Counter Prescaler */
        0x1U, /* Timestamp source selection */
        MCAN_TIMEOUT_SELECT_CONT, /* Time-out counter source select */
        0xFFFFU, /* Start value of the Timeout Counter (down-counter) */
        0x0U, /* Time-out Counter Enable */
        {
            0x1U, /* Reject Remote Frames Extended */
            0x1U, /* Reject Remote Frames Standard */
            0x1U, /* Accept Non-matching Frames Extended */
            0x1U, /* Accept Non-matching Frames Standard */
        },
    },
};

/**
 *  \brief MCAN MSG RAM Configuration Parameters Configurations.
 */
MCAN_MsgRAMConfigParams canFDRAMConfigParams[] =
{
    /* Config 0 */
    [0U] = 
    {
        .lss = APP_MCAN_STD_ID_FILTER_NUM,      /* List Size: Standard ID */
        .lse = APP_MCAN_EXT_ID_FILTER_NUM,      /* List Size: Extended ID*/
        .txBufCnt = APP_MCAN_TX_BUFF_SIZE,           /* Number of Dedicated Transmit Buffers */
        .txFIFOCnt = APP_MCAN_TX_FIFO_SIZE,            /* Transmit FIFO/Queue Size */
        .txBufMode = 0U,                              /* Tx FIFO/Queue Mode */
        .txEventFIFOCnt = APP_MCAN_TX_BUFF_SIZE,          /* Tx Buffer Element Size */
        .rxFIFO0Cnt = APP_MCAN_FIFO_0_NUM,           /* Event FIFO Size */
        .rxFIFO0OpMode = 0U,                              /* Rx FIFO0 Operation Mode */
        .rxFIFO1Cnt = APP_MCAN_FIFO_1_NUM,             /* Rx FIFO1 Size */
        .rxFIFO1OpMode = 0U,                              /* Rx FIFO1 Operation Mode */
        .rxBufElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx Buffer Element Size */
        .rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO0 Element Size */
        .rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO1 Element Size */
        .txEventFIFOWaterMark = 3U,                         /* Tx Event FIFO Watermark */
        .rxFIFO0WaterMark = 3U,                         /* RX FIFO0 Watermark */
        .rxFIFO1WaterMark = 3U,                         /* RX FIFO1 FIFO Watermark */
    },
    [1U] = 
    /* Config 1 */
    {
        .lss = APP_MCAN_STD_ID_FILTER_NUM,      /* List Size: Standard ID */
        .lse = APP_MCAN_EXT_ID_FILTER_NUM,      /* List Size: Extended ID*/
        .txBufCnt = APP_MCAN_TX_BUFF_SIZE,           /* Number of Dedicated Transmit Buffers */
        .txFIFOCnt = APP_MCAN_TX_FIFO_SIZE,            /* Transmit FIFO/Queue Size */
        .txBufMode = 1U,                              /* Tx FIFO/Queue Mode */
        .txEventFIFOCnt = APP_MCAN_TX_BUFF_SIZE,          /* Tx Buffer Element Size */
        .rxFIFO0Cnt = APP_MCAN_FIFO_0_NUM,           /* Event FIFO Size */
        .rxFIFO0OpMode = 0U,                              /* Rx FIFO0 Operation Mode */
        .rxFIFO1Cnt = APP_MCAN_FIFO_1_NUM,             /* Rx FIFO1 Size */
        .rxFIFO1OpMode = 0U,                              /* Rx FIFO1 Operation Mode */
        .rxBufElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx Buffer Element Size */
        .rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO0 Element Size */
        .rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO1 Element Size */
        .txEventFIFOWaterMark = 3U,                         /* Tx Event FIFO Watermark */
        .rxFIFO0WaterMark = 3U,                         /* RX FIFO0 Watermark */
        .rxFIFO1WaterMark = 3U,                         /* RX FIFO1 FIFO Watermark */
    },
    [2U] = 
    /* Config 2 */
    {
        .lss = APP_MCAN_STD_ID_FILTER_NUM,      /* List Size: Standard ID */
        .lse = APP_MCAN_EXT_ID_FILTER_NUM,      /* List Size: Extended ID*/
        .txBufCnt = APP_MCAN_TX_BUFF_SIZE,           /* Number of Dedicated Transmit Buffers */
        .txFIFOCnt = APP_MCAN_TX_FIFO_SIZE,            /* Transmit FIFO/Queue Size */
        .txBufMode = 1U,                              /* Tx FIFO/Queue Mode */
        .txEventFIFOCnt = APP_MCAN_TX_BUFF_SIZE,          /* Tx Buffer Element Size */
        .rxFIFO0Cnt = 5U,           /* Event FIFO Size */
        .rxFIFO0OpMode = 1U,                              /* Rx FIFO0 Operation Mode */
        .rxFIFO1Cnt = 5U,             /* Rx FIFO1 Size */
        .rxFIFO1OpMode = 0U,                              /* Rx FIFO1 Operation Mode */
        .rxBufElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx Buffer Element Size */
        .rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO0 Element Size */
        .rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO1 Element Size */
        .txEventFIFOWaterMark = 3U,                         /* Tx Event FIFO Watermark */
        .rxFIFO0WaterMark = 3U,                         /* RX FIFO0 Watermark */
        .rxFIFO1WaterMark = 3U,                         /* RX FIFO1 FIFO Watermark */
    },
    [3U] = 
    /* Config 3 */
    {
        .lss = APP_MCAN_STD_ID_FILTER_NUM,      /* List Size: Standard ID */
        .lse = APP_MCAN_EXT_ID_FILTER_NUM,      /* List Size: Extended ID*/
        .txBufCnt = 64,           /* Number of Dedicated Transmit Buffers */
        .txFIFOCnt = 64,            /* Transmit FIFO/Queue Size */
        .txBufMode = 1U,                              /* Tx FIFO/Queue Mode */
        .txEventFIFOCnt = APP_MCAN_TX_BUFF_SIZE,          /* Tx Buffer Element Size */
        .rxFIFO0Cnt = 5U,           /* Event FIFO Size */
        .rxFIFO0OpMode = 1U,                              /* Rx FIFO0 Operation Mode */
        .rxFIFO1Cnt = 5U,             /* Rx FIFO1 Size */
        .rxFIFO1OpMode = 0U,                              /* Rx FIFO1 Operation Mode */
        .rxBufElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx Buffer Element Size */
        .rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO0 Element Size */
        .rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO1 Element Size */
        .txEventFIFOWaterMark = 3U,                         /* Tx Event FIFO Watermark */
        .rxFIFO0WaterMark = 3U,                         /* RX FIFO0 Watermark */
        .rxFIFO1WaterMark = 3U,                         /* RX FIFO1 FIFO Watermark */
    },
    [4U] =
    /* Config 4 */
    {
        .lss = 0U,      /* List Size: Standard ID */
        .lse = 0U,      /* List Size: Extended ID*/
        .txBufCnt = 0U,           /* Number of Dedicated Transmit Buffers */
        .txFIFOCnt = 0U,            /* Transmit FIFO/Queue Size */
        .txBufMode = 1U,                              /* Tx FIFO/Queue Mode */
        .txEventFIFOCnt = APP_MCAN_TX_BUFF_SIZE,          /* Tx Buffer Element Size */
        .rxFIFO0Cnt = 0U,           /* Event FIFO Size */
        .rxFIFO0OpMode = 1U,                              /* Rx FIFO0 Operation Mode */
        .rxFIFO1Cnt = 0U,             /* Rx FIFO1 Size */
        .rxFIFO1OpMode = 0U,                              /* Rx FIFO1 Operation Mode */
        .rxBufElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx Buffer Element Size */
        .rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO0 Element Size */
        .rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO1 Element Size */
        .txEventFIFOWaterMark = 3U,                         /* Tx Event FIFO Watermark */
        .rxFIFO0WaterMark = 3U,                         /* RX FIFO0 Watermark */
        .rxFIFO1WaterMark = 3U,                         /* RX FIFO1 FIFO Watermark */
    },
    [5U] =
    /* Config 4 */
    {
        .lss = 128U,      /* List Size: Standard ID */
        .lse = 128U,      /* List Size: Extended ID*/
        .txBufCnt = 64U,           /* Number of Dedicated Transmit Buffers */
        .txFIFOCnt = 32U,            /* Transmit FIFO/Queue Size */
        .txBufMode = 1U,                              /* Tx FIFO/Queue Mode */
        .txEventFIFOCnt = 32U,          /* Tx Buffer Element Size */
        .rxFIFO0Cnt = 64U,           /* Event FIFO Size */
        .rxFIFO0OpMode = 1U,                              /* Rx FIFO0 Operation Mode */
        .rxFIFO1Cnt = 64U,             /* Rx FIFO1 Size */
        .rxFIFO1OpMode = 0U,                              /* Rx FIFO1 Operation Mode */
        .rxBufElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx Buffer Element Size */
        .rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO0 Element Size */
        .rxFIFO1ElemSize = MCAN_ELEM_SIZE_64BYTES,          /* Rx FIFO1 Element Size */
        .txEventFIFOWaterMark = 3U,                         /* Tx Event FIFO Watermark */
        .rxFIFO0WaterMark = 3U,                         /* RX FIFO0 Watermark */
        .rxFIFO1WaterMark = 3U,                         /* RX FIFO1 FIFO Watermark */
    },
};

/**
 *  \brief MCAN ECC Configuration Parameters Configurations.
 */
MCAN_ECCConfigParams canFDECCConfigParams[] =
{
    /* Config 0 */
    {
        0x01U, /* Enable/disable ECC */
        0x01U, /* Enable/disable ECC Check */
        0x01U, /* Enable/disable Read Modify Write operation */
    },
};

/**
 *  \brief MCAN ECC Error Force Configuration Parameters Configurations.
 */
MCAN_ECCErrForceParams canFDECCErrForceConfigParams[] =
{
    /* Config 0 */
    {
        MCAN_ECC_ERR_TYPE_SEC, /* Error type to be forced */
        0x0AU, /* Row address where error needs to be applied. */
        0x04U, /* Column/Data bit that needs to be flipped */
        0x01U, /* Data bit that needs to be flipped when force_ded is set */
        0x01U, /* Force Error once */
        0x00U, /* Force error on the next RAM read */
    },
    /* Config 1 */
    {
        MCAN_ECC_ERR_TYPE_DED, /* Error type to be forced */
        0x40U, /* Row address where error needs to be applied. */
        0x04U, /* Column/Data bit that needs to be flipped */
        0x01U, /* Data bit that needs to be flipped when force_ded is set */
        0x01U, /* Force Error once */
        0x00U, /* Force error on the next RAM read */
    },
};

/**
 *  \brief MCAN Tx Message .
 */
st_mcanTxMSGParams_t canTxMSG[] =
{
    /* Message 0 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 1 */
    {
        {
            ((uint32_t)0xC4U), /* Identifier */
            0U,      /* Remote Transmission Request */
            1U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0x6U,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 2 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 3 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0x8U,    /* Data Length Code */
            0U,      /* Bit Rate Switching */
            0U,      /* FD Format */
            0U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
            },
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 4 */
    {
        {
            ((uint32_t)0x9U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0x8U,    /* Data Length Code */
            0U,      /* Bit Rate Switching */
            0U,      /* FD Format */
            0U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
            },
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 5 */
    {
        {
            ((uint32_t)0xAU << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            0U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            0U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 6 */
    {
        {
            ((uint32_t)0xBU << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            0U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            0U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 7 */
    {
        {
            ((uint32_t)0xEU << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            0U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            0U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 8 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 9 */
    {
        {
            ((uint32_t)0xFFU << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 10 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00, 0x00,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 11 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00, 0x00,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 12 */
    {
        {
            ((uint32_t)0x8U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00, 0x00,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        1U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 13 */
    {
        {
            ((uint32_t)0x5U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00, 0x00,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        2U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 14 */
    {
        {
            ((uint32_t)0x3U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00, 0x00,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 15 */
    {
        {
            ((uint32_t)0x3U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00, 0x00,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 16 */
    {
        {
            ((uint32_t)0x4U), /* Identifier */
            0U,      /* Remote Transmission Request */
            1U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0x8U,    /* Data Length Code */
            0U,      /* Bit Rate Switching */
            0U,      /* FD Format */
            0U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
            },
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 17 */
    {
        {
            ((uint32_t)0x3U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0x7U,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0x00, 0x00, 0x00,
            },
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 18 */
    {
        {
            ((uint32_t)0x4U << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 19 */
    {
        {
            ((uint32_t)0x4U), /* Identifier */
            0U,      /* Remote Transmission Request */
            1U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 20 */
    {
        {
            ((uint32_t)0x7FFU << 18U), /* Identifier */
            0U,      /* Remote Transmission Request */
            0U,      /* Extended Identifier */
            0U,      /* Error State Indicator */
            0xFU,    /* Data Length Code */
            1U,      /* Bit Rate Switching */
            1U,      /* FD Format */
            1U,      /* Event FIFO Control */
            0xAAU,   /* Message Marker */
            { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD, 0xBA, 0xBE,
                0x12, 0x34, 0x56, 0x78,
                0x9A, 0xBC, 0xDE, 0xF0,
                0x44, 0xf0, 0x0D, 0x44,
                0x11, 0x11, 0x11, 0x11,
                0x01, 0x32, 0x29, 0x50,
                0x44, 0x44, 0x44, 0x44,
            },
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
};

/**
 *  \brief CAN Standard ID Filter Configurations.
 */
MCAN_StdMsgIDFilterElement canSTDIDFilter[] =
{
    /* Filter 0 */
    {
        (0x2U << 6U), /* Standard Filter ID 2 */
        0x04U, /* Standard Filter ID 1 */
        0x07U, /* Standard Filter Element Configuration */
        0x00U, /* Standard Filter Type */
    },
    /* Filter 1 */
    {
        0x00U, /* Standard Filter ID 2 */
        0xFFU, /* Standard Filter ID 1 */
        0x07U, /* Standard Filter Element Configuration */
        0x00U, /* Standard Filter Type */
    },
    /* Filter 2 */
    {
        0x0AU, /* Standard Filter ID 2 */
        0x04U, /* Standard Filter ID 1 */
        0x01U, /* Standard Filter Element Configuration */
        0x00U, /* Standard Filter Type */
    },
    /* Filter 3 */
    {
        0x0AU, /* Standard Filter ID 2 */
        0x0FU, /* Standard Filter ID 1 */
        0x01U, /* Standard Filter Element Configuration */
        0x02U, /* Standard Filter Type */
    },
    /* Filter 4 */
    {
        0x0AU, /* Standard Filter ID 2 */
        0x0FU, /* Standard Filter ID 1 */
        0x01U, /* Standard Filter Element Configuration */
        0x02U, /* Standard Filter Type */
    },
    /* Filter 5 */
    {
        0x0AU, /* Standard Filter ID 2 */
        0x0FU, /* Standard Filter ID 1 */
        0x05U, /* Standard Filter Element Configuration */
        0x02U, /* Standard Filter Type */
    },
    /* Filter 6 */
    {
        (0x2U << 6U), /* Standard Filter ID 2 */
        0x04U, /* Standard Filter ID 1 */
        0x07U, /* Standard Filter Element Configuration */
        0x01U, /* Standard Filter Type */
    },
    /* Filter 7 */
    {
        (0x2U << 6U), /* Standard Filter ID 2 */
        0x04U, /* Standard Filter ID 1 */
        0x02U, /* Standard Filter Element Configuration */
        0x00U, /* Standard Filter Type */
    },
    /* Filter 8 */
    {
        0x0U, /* Standard Filter ID 2 */
        0xFFFFFFFFU, /* Standard Filter ID 1 */
        0x07U, /* Standard Filter Element Configuration */
        0x00U, /* Standard Filter Type */
    },
};

/**
 *  \brief CAN Extended ID Filter Configurations.
 */
MCAN_ExtMsgIDFilterElement canEXTIDFilter[] =
{
    /* Filter 0 */
    {
        0xC4U, /* Extended Filter ID 1 */
        0x07U, /* Extended Filter Element Configuration */
        0x00U, /* Extended Filter ID 2 */
        0x00U, /* Extended Filter Type */
    },
    /* Filter 1 */
    {
        0xD4U, /* Extended Filter ID 1 */
        0x07U, /* Extended Filter Element Configuration */
        0x00U, /* Extended Filter ID 2 */
        0x00U, /* Extended Filter Type */
    },
    /* Filter 2 */
    {
        0x04U, /* Extended Filter ID 1 */
        0x07U, /* Extended Filter Element Configuration */
        (0x2U << 6U), /* Extended Filter ID 2 */
        0x00U, /* Extended Filter Type */
    },
    /* Filter 3 */
    {
        0x04U, /* Extended Filter ID 1 */
        0x02U, /* Extended Filter Element Configuration */
        (0x2U << 6U), /* Extended Filter ID 2 */
        0x00U, /* Extended Filter Type */
    },
};

#endif /* #ifndef ST_MCAN_TEST_CASES_CONFIG_H_ */
