/*
 *  Copyright (C) 2024-25 Texas Instruments Incorporated
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
 *  \file test_mcan_canfd.h
 *
 *  \brief This file contains all the structures, macros, enums
 *  used by the mcan UT applications.
 */

#ifndef TEST_MCAN_H_
#define TEST_MCAN_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <unity.h>
#include <drivers/mcan.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/soc.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*  MCAN functional cloclk  */
#define APP_MCAN_FUNCTIONAL_CLK             (80000U)
/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         100U
/** \brief Data size per transfer */
#define MCAN_APP_TEST_DATA_SIZE             64U
/** \brief Number of messages sent */
#define MCAN_APP_TEST_PERFORMANCE_MESSAGE_COUNT     100U

/* Macro's for Msg RAM configuration */
#define APP_MCAN_STD_ID_FILTER_NUM               (128U)
#define APP_MCAN_EXT_ID_FILTER_NUM               (64U)
#define APP_MCAN_TX_BUFF_SIZE                    (16U)
#define APP_MCAN_TX_FIFO_SIZE                    (16U)
#define APP_MCAN_FIFO_0_NUM                      (64U)
#define APP_MCAN_FIFO_1_NUM                      (64U)
#define APP_MCAN_TX_BUFF_MAX_SIZE                (32U)
#define APP_MCAN_TX_FIFO_MAX_SIZE                (32U)
#define APP_MCAN_RX_BUFF_MAX_NUM                 (64U)
#define App_MCAN_FIFO_WATERMARK_LEVEL            (3U)

/* Theoretical maximum throughput numbers */
#define MCAN_THEOROTICAL_MAX_STD_1_5_MBPS       (7430U)
#define MCAN_THEOROTICAL_MAX_EXT_1_5_MBPS       (6510U)
#define MCAN_CLASSIC_CAN_THEOROTICAL_MAX_STD_1_MBPS       (9260U)
#define MCAN_CLASSIC_CAN_THEOROTICAL_MAX_EXT_1_MBPS       (7810U)

/* Standard Filter Element Configuration */
#define APP_CANFD_DISABLE_FILTER                        (0U)
#define APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES   (1U) 
#define APP_CANFD_STORE_IN_RXFIFO_1_IF_FILTER_MATCHES   (2U) 
#define APP_CANFD_SET_PRIORITY_AND_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES    (5U)
#define APP_CANFD_SET_PRIORITY_AND_STORE_IN_RXFIFO_1_IF_FILTER_MATCHES    (6U)
#define APP_CANFD_STORE_INTO_RX_BUFFER                  (7U) 
    
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct App_CANFD_TxMsgParams_t {
    /* Message ID type. 11 bit or 29 bits */
    CANFD_MCANXidType       msgIdType;

    /** Data Length used by application for transmission and reception.
     */
    uint32_t       dataLength;

    /* Data bytes. */
    uint8_t        data[MCAN_MAX_PAYLOAD_BYTES];

    /**
     *  Part of message ram to accessed by this message object. Refer enum #MCAN_MemType.
     */
    MCAN_MemType   txMemType;

    /**<  Buffer number where tx message is to be stored */
    uint32_t       txBuffNum;

    /**< standard or extended filter element. Refer efec in MCAN_ExtMsgIDFilterElement or sfec in MCAN_StdMsgIDFilterElement structure  */
    uint32_t       filterElement;
    
    /**
     *   Part of message ram to accessed by this message object.
     *   Refer 'efec' varaible in #MCAN_ExtMsgIDFilterElement structure 
     *   or 'eft' varrible in #MCAN_StdMsgIDFilterElement
     */
    uint32_t   rxfilterType;

    /**
     *  Part of message ram to accessed by this message object. Refer enum #MCAN_MemType.
     */
    MCAN_MemType   rxMemType;
    
    /**
     * \brief   FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).
     */
    uint32_t       rxFifoNum;
}App_CANFD_TxMsgParams;

typedef struct CANFD_TestParams_s {
     /**< mcan canfd config. */
    CANFD_Config         canfdConfig;

     /**< mcan canfd open params. */
    CANFD_OpenParams     openParams;

     /**< tx message object. */
    CANFD_MessageObject  txMsgObject;

     /**< rx message object. */
    CANFD_MessageObject  rxMsgObject;
 
    /* Tx msg param */
    App_CANFD_TxMsgParams  *txMsgParams;
    /* Test case ID */
    uint32_t testCaseId;
    /*  Test result. */
    int32_t        testResult;
    /*  CANFD instance. */
    uint32_t       canfdInstance;
    /*  This enumeration describes a list of all the reasons for which the driver 
        will invoke application callback functions. */
    CANFD_Reason reason;
    /**
     * \brief   FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).
     */
    uint32_t       rxFifoNum;
} CANFD_TestParams;

/* ========================================================================== */
/*                            Global varibales                                */
/* ========================================================================== */

App_CANFD_TxMsgParams canTxMsg[] = 
{
    /* Message 0 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier (txMemType). Where the txMsg will be stored (BUFFER/FIFO) */
        0U, /* txBuffNum. Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement. */
        0U, /* filter type(rxfilterType): Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* (rxMemType). Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 1 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x6U,    /* Data Length */
        { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD,
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement. */
        1U, /* Rx filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 2 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement.*/
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 3 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        8U,    /* Data Length */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        1U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 4 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        8U,    /* Data Length */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 5 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier (txMemType). Where the txMsg will be stored (BUFFER/FIFO) */
        0U, /* txBuffNum. Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. standard or extended filter element. 
                    Refer efec in MCAN_ExtMsgIDFilterElement or sfec in MCAN_StdMsgIDFilterElement structure */
        0U, /* filter type(rxfilterType): Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* (rxMemType). Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 6 */
    {  
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */    
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 7 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_SET_PRIORITY_AND_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        2U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 8 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement. */
        1U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 9 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 10 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 11 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 12 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        1U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 13 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        2U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 14 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 15 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_1_IF_FILTER_MATCHES, /* filterElement. */
        3U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 16 */
    { 
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x8U,    /* Data Length */  
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        1U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },

    /* Message 17 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x7U,    /* Data Length */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0x00, 0x00, 0x00,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 18 */
    {
        0U,      /* Message ID type. 11 bit or 29 bits  */
        0x8U,    /* Data Length */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 19 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_1_IF_FILTER_MATCHES, /* filterElement. */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 20 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_INTO_RX_BUFFER, /* filterElement.  */
        0U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_BUF, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 21 */
    {
        0U,      /* Message ID type. 11 bit or 29 bits  */ /* classic CAN*/
        0x8U,    /* Data Length */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_0_IF_FILTER_MATCHES, /* filterElement. */
        2U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_0, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
    /* Message 22 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        64U,    /* Data Length */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        APP_CANFD_STORE_IN_RXFIFO_1_IF_FILTER_MATCHES, /* filterElement. */
        3U, /* filter type: Refer sft in MCAN_StdMsgIDFilterElement structure or eft in MCAN_ExtMsgIDFilterElement structure. */
        MCAN_MEM_TYPE_FIFO, /* Buffer/FIFO number where received message is to be stored */
        MCAN_RX_FIFO_NUM_1, /* FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).  */
    },
};

/**
 *  \brief CAN Extended ID Filter Configurations.
 */
MCAN_ExtMsgIDFilterElement canExtIdFilter[] =
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
    /* Filter 4 */
    {
        0x29E, /* Extended Filter ID 1 */
        MCAN_EXT_FILT_ELEM_BUFFER, /* Extended Filter Element Configuration */
        0x29E, /* Extended Filter ID 2 */
        MCAN_EXT_FILT_TYPE_RANGE, /* Extended Filter Type */
    },
};

/**
 *  \brief CAN Standard ID Filter Configurations.
 */
MCAN_StdMsgIDFilterElement canStdIDFilter[] =
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

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif
