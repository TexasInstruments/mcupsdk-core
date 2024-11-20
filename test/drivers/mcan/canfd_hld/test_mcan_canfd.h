/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
/** \brief Number of messages sent */
#define MCAN_APP_TEST_MESSAGE_COUNT         100U
/** \brief Data size per transfer */
#define MCAN_APP_TEST_DATA_SIZE             64U

/* Macro's for Msg RAM configuration */
#define APP_MCAN_STD_ID_FILTER_NUM               (128U)
#define APP_MCAN_EXT_ID_FILTER_NUM               (64U)
#define APP_MCAN_TX_BUFF_SIZE                    (16U)
#define APP_MCAN_TX_FIFO_SIZE                    (16U)
#define APP_MCAN_FIFO_0_NUM                      (64U)
#define APP_MCAN_FIFO_1_NUM                      (64U)

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

    /**< Buffer/FIFO number where received message is to be stored */
    uint32_t       rxBuffNum;
    
    /**
     *   Part of message ram to accessed by this message object.
     *   Refer 'efec' varaible in #MCAN_ExtMsgIDFilterElement structure 
     *   or 'sfec' varrible in #MCAN_StdMsgIDFilterElement
     */
    uint32_t   rxfilterType;

    /**
     *  Part of message ram to accessed by this message object. Refer enum #MCAN_MemType.
     */
    MCAN_MemType   rxMemType;
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

    /**< tx message number. */
    uint32_t txMsgNum;

    /**< standard ID message filter number. */
    uint32_t stdIdFiltNum;
    
    /**< extended ID message filter number. */
    uint32_t extIdFiltNum;
    
    /* Tx msg param */
    App_CANFD_TxMsgParams  *txMsgParams;
} CANFD_TestParams;

/* ========================================================================== */
/*                            Global varibales                                */
/* ========================================================================== */

App_CANFD_TxMsgParams canTxMsg[] = 
{
    /* Message 0 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 1 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x6U,    /* Data Length Code */
        { /* Data */
                0x12, 0x34, 0xAB, 0xCD,
                0xDE, 0xAD,
        },
        MCAN_MEM_TYPE_BUF, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 2 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 3 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x8U,    /* Data Length Code */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 4 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x8U,    /* Data Length Code */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 5 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 6 */
    {  
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */    
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 7 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_0, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 8 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 9 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 10 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 11 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 12 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 13 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 14 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 15 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 16 */
    { 
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x8U,    /* Data Length Code */  
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0xDE, 0xAD, 0xBA, 0xBE,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        0U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
    },

    /* Message 17 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0x7U,    /* Data Length Code */
        { /* Data */
            0x12, 0x34, 0xAB, 0xCD,
            0x00, 0x00, 0x00,
        },
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier */
        3U, /* Buffer number where message is to be stored. */
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 18 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 19 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_FIFO, /* Storage Identifier- where received message shall be stored */
        MCAN_RX_FIFO_NUM_1, /* Buffer/FIFO number where received message is to be stored */
    },
    /* Message 20 */
    {
        1U,      /* Message ID type. 11 bit or 29 bits  */
        0xFU,    /* Data Length Code */
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
        MCAN_MEM_TYPE_BUF, /* Storage Identifier- where received message shall be stored */
        0U, /* Buffer/FIFO number where received message is to be stored */
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
};

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif
